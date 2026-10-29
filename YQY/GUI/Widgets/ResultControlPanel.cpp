#include "Widgets/ResultControlPanel.h"
#include <QVariant>
#include <QApplication>
#include <QCheckBox>
#include <QEvent>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QResizeEvent>
#include <QSlider>
#include <QVBoxLayout>
#include <QWidget>
#include "Widgets/ResultControlInputWidgets.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <QSignalBlocker>

ResultControlPanel::ResultControlPanel(QWidget* parent)
    : QWidget(parent)
    , m_playbackTimer(new QTimer(this))
{
    initializeUi();
    setObjectName(QStringLiteral("resultControlPanel"));
    m_cardLayout->setAlignment(Qt::AlignTop);
    m_cardLayout->setSizeConstraint(QLayout::SetMinimumSize);
    m_playButton->setEnabled(false);
    m_frameSlider->setEnabled(false);
    m_exportButton->setObjectName(QStringLiteral("exportNodeResultsButton"));
    m_resultForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_resultForm->setRowWrapPolicy(QFormLayout::WrapLongRows);
    m_resultForm->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_resultForm->setHorizontalSpacing(12);
    m_resultForm->setVerticalSpacing(8);
    constexpr int captionWidth = 64;
    for (auto* label :
         {m_fieldLabel, m_stepLabel, m_scaleLabel, m_frameCaptionLabel, m_timeCaptionLabel, m_deformationCaptionLabel})
    {
        label->setMinimumWidth(captionWidth);
        label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    }
    m_resultFrameStatusLayout->setColumnMinimumWidth(0, captionWidth);
    m_resultFrameStatusLayout->setColumnStretch(1, 1);
    for (auto* combo : {m_fieldCombo, m_stepCombo, m_speedCombo})
    {
        combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setMinimumContentsLength(combo == m_speedCombo ? 4 : 8);
        combo->setMinimumHeight(32);
        if (combo != m_speedCombo)
            connect(combo, &QComboBox::currentTextChanged, combo, &QWidget::setToolTip);
    }
    m_scaleSpin->setMinimumHeight(32);
    m_scaleSpin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_scaleSpin->setToolTip(QStringLiteral("0 表示自动确定变形比例；1 表示按实际位移显示。"));
    m_frameSlider->setMinimumHeight(26);
    m_frameSlider->setFocusPolicy(Qt::StrongFocus);
    m_frameSlider->setToolTip(QStringLiteral("拖动浏览结果帧；方向键逐帧调整，Home / End 跳到首帧 / 末帧。"));
    m_frameRow->setSpacing(8);
    m_frameRow->setStretch(0, 1);
    m_playButton->setMinimumHeight(32);
    m_speedLabel->setAlignment(Qt::AlignCenter);
    for (auto* button :
         {m_exportButton, m_exportElementButton, m_exportIterationButton, m_exportPlotButton, m_exportEnergyButton})
    {
        button->setMinimumWidth(0);
        button->setMinimumHeight(34);
        // 网格分栏由可用宽度决定，不让双列按钮的建议宽度撑大整个侧栏。
        button->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        button->setToolTip(button->text());
    }
    for (auto* label : {m_frameLabel, m_timeValueLabel, m_deformationValueLabel})
    {
        label->setWordWrap(true);
        label->setMinimumWidth(0);
        auto policy = label->sizePolicy();
        policy.setHorizontalPolicy(QSizePolicy::Ignored);
        label->setSizePolicy(policy);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    }
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    constexpr std::array playbackSpeeds{0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0};
    m_speedCombo->clear();
    for (const double speed : playbackSpeeds)
    {
        m_speedCombo->addItem(QStringLiteral("%1×").arg(speed, 0, 'g'), speed);
    }
    m_speedCombo->setCurrentIndex(2);

    m_playbackTimer->setTimerType(Qt::PreciseTimer);
    m_playbackTimer->setInterval(16);
    connect(m_playbackTimer, &QTimer::timeout, this,
            [this]()
            {
                if (m_frames.size() < 2)
                    return;

                const double speed = m_speedCombo->currentData().toDouble();
                const double firstTime = m_frames.front().time;
                const double lastTime = m_frames.back().time;
                const double duration = lastTime - firstTime;
                if (duration <= 0.0)
                    return;

                m_playbackTime += static_cast<double>(m_playbackTimer->interval()) * speed / 1000.0;
                m_playbackTime = firstTime + std::fmod(std::max(0.0, m_playbackTime - firstTime), duration);

                const auto upper = std::upper_bound(m_frames.cbegin(), m_frames.cend(), m_playbackTime,
                                                    [](double time, const Hdf5ResultFrameInfo& frame)
                                                    {
                                                        return time < frame.time;
                                                    });
                const int nextFrame = static_cast<int>(std::distance(m_frames.cbegin(), upper));
                const int displayedFrame = std::clamp(nextFrame - 1, 0, static_cast<int>(m_frames.size()) - 2);
                const double frameDuration = m_frames[displayedFrame + 1].time - m_frames[displayedFrame].time;
                const double interpolation =
                    frameDuration > 0.0 ? (m_playbackTime - m_frames[displayedFrame].time) / frameDuration : 0.0;
                m_playbackPosition = static_cast<double>(displayedFrame) + std::clamp(interpolation, 0.0, 1.0);

                {
                    const QSignalBlocker blocker(m_frameSlider);
                    m_frameSlider->setValue(displayedFrame);
                }
                if (m_frameChangedHandler)
                    m_frameChangedHandler(m_playbackPosition);
            });
    connect(m_playButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_frames.size() < 2)
                    return;

                const bool playing = m_playbackTimer->isActive();
                if (playing)
                    m_playbackTimer->stop();
                else
                {
                    const int frameIndex = std::clamp(m_frameSlider->value(), 0, static_cast<int>(m_frames.size()) - 1);
                    m_playbackPosition = static_cast<double>(frameIndex);
                    m_playbackTime = m_frames[static_cast<std::size_t>(frameIndex)].time;
                    m_playbackTimer->start();
                }
                m_playButton->setText(playing ? QStringLiteral("播放") : QStringLiteral("暂停"));
                if (m_playbackStateChangedHandler)
                    m_playbackStateChangedHandler(!playing);
            });
    connect(m_frameSlider, &QSlider::valueChanged, this,
            [this](int frameIndex)
            {
                m_playbackPosition = static_cast<double>(frameIndex);
                if (frameIndex >= 0 && frameIndex < static_cast<int>(m_frames.size()))
                    m_playbackTime = m_frames[static_cast<std::size_t>(frameIndex)].time;
                if (m_frameChangedHandler)
                    m_frameChangedHandler(m_playbackPosition);
            });
    const auto visualizationChanged = [this]()
    {
        if (m_visualizationChangedHandler)
            m_visualizationChangedHandler();
    };
    connect(m_fieldCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [visualizationChanged](int)
            {
                visualizationChanged();
            });
    connect(m_scaleSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [visualizationChanged](double)
            {
                visualizationChanged();
            });
    connect(m_originalCheck, &QCheckBox::toggled, this,
            [visualizationChanged](bool)
            {
                visualizationChanged();
            });
    connect(m_exportButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_exportHandler)
                    m_exportHandler();
            });
    connect(m_exportElementButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_elementExportHandler)
                    m_elementExportHandler();
            });
    connect(m_exportIterationButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_iterationExportHandler)
                    m_iterationExportHandler();
            });
    connect(m_exportPlotButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_plotExportHandler)
                    m_plotExportHandler();
            });
    connect(m_exportEnergyButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_energyExportHandler)
                    m_energyExportHandler();
            });
}
ResultControlPanel::~ResultControlPanel() = default;

bool ResultControlPanel::event(QEvent* event)
{
    const bool handled = QWidget::event(event);
    if (event->type() == QEvent::Show || event->type() == QEvent::StyleChange || event->type() == QEvent::FontChange)
    {
        updateExportLayout();
    }
    return handled;
}

void ResultControlPanel::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    updateExportLayout();
}

void ResultControlPanel::updateExportLayout()
{
    if (!m_resultExportGrid)
    {
        return;
    }
    const std::array buttons{m_exportButton, m_exportElementButton, m_exportIterationButton, m_exportPlotButton,
                             m_exportEnergyButton};
    int buttonWidth = 0;
    for (auto* button : buttons)
    {
        buttonWidth = qMax(buttonWidth, button->sizeHint().width());
    }
    const QMargins margins = m_cardLayout->contentsMargins();
    const QMargins border = m_cardLayout->parentWidget()->contentsMargins();
    const int availableWidth = contentsRect().width() - margins.left() - margins.right() -
                               border.left() - border.right();
    const int columns = availableWidth >= 2 * buttonWidth + m_resultExportGrid->horizontalSpacing() ? 2 : 1;
    if (columns == m_exportColumns)
    {
        return;
    }
    m_exportColumns = columns;
    for (auto* button : buttons)
    {
        m_resultExportGrid->removeWidget(button);
    }
    for (int index = 0; index < int(buttons.size()); ++index)
    {
        const int span = index == int(buttons.size()) - 1 && columns == 2 ? 2 : 1;
        m_resultExportGrid->addWidget(buttons[index], index / columns, index % columns, 1, span);
    }
    m_resultExportGrid->setColumnStretch(1, columns == 2 ? 1 : 0);
    updateGeometry();
}

QComboBox* ResultControlPanel::fieldCombo() const
{
    return m_fieldCombo;
}
QDoubleSpinBox* ResultControlPanel::scaleSpin() const
{
    return m_scaleSpin;
}
QCheckBox* ResultControlPanel::originalCheck() const
{
    return m_originalCheck;
}
QPushButton* ResultControlPanel::playButton() const
{
    return m_playButton;
}
QSlider* ResultControlPanel::frameSlider() const
{
    return m_frameSlider;
}
QComboBox* ResultControlPanel::speedCombo() const
{
    return m_speedCombo;
}
QLabel* ResultControlPanel::frameLabel() const
{
    return m_frameLabel;
}

QLabel* ResultControlPanel::timeValueLabel() const
{
    return m_timeValueLabel;
}

QLabel* ResultControlPanel::deformationValueLabel() const
{
    return m_deformationValueLabel;
}

QPushButton* ResultControlPanel::exportButton() const
{
    return m_exportButton;
}

QPushButton* ResultControlPanel::exportElementButton() const
{
    return m_exportElementButton;
}

QPushButton* ResultControlPanel::exportIterationButton() const
{
    return m_exportIterationButton;
}

QComboBox* ResultControlPanel::stepCombo() const
{
    return m_stepCombo;
}

QPushButton* ResultControlPanel::exportPlotButton() const
{
    return m_exportPlotButton;
}

QPushButton* ResultControlPanel::exportEnergyButton() const
{
    return m_exportEnergyButton;
}

std::vector<Hdf5ResultFrameInfo>& ResultControlPanel::frames()
{
    return m_frames;
}

const std::vector<Hdf5ResultFrameInfo>& ResultControlPanel::frames() const
{
    return m_frames;
}

QString& ResultControlPanel::resultFilePath()
{
    return m_resultFilePath;
}

const QString& ResultControlPanel::resultFilePath() const
{
    return m_resultFilePath;
}

bool& ResultControlPanel::partialResult()
{
    return m_partialResult;
}

double& ResultControlPanel::automaticScale()
{
    return m_automaticScale;
}

QTimer* ResultControlPanel::playbackTimer() const
{
    return m_playbackTimer;
}

void ResultControlPanel::setFrameChangedHandler(std::function<void(double)> handler)
{
    m_frameChangedHandler = std::move(handler);
}

void ResultControlPanel::setVisualizationChangedHandler(std::function<void()> handler)
{
    m_visualizationChangedHandler = std::move(handler);
}

void ResultControlPanel::setPlaybackStateChangedHandler(std::function<void(bool)> handler)
{
    m_playbackStateChangedHandler = std::move(handler);
}

void ResultControlPanel::setExportHandler(std::function<void()> handler)
{
    m_exportHandler = std::move(handler);
}

void ResultControlPanel::setElementExportHandler(std::function<void()> handler)
{
    m_elementExportHandler = std::move(handler);
}

void ResultControlPanel::setIterationExportHandler(std::function<void()> handler)
{
    m_iterationExportHandler = std::move(handler);
}

void ResultControlPanel::setPlotExportHandler(std::function<void()> handler)
{
    m_plotExportHandler = std::move(handler);
}

void ResultControlPanel::setEnergyExportHandler(std::function<void()> handler)
{
    m_energyExportHandler = std::move(handler);
}

void ResultControlPanel::stopPlayback()
{
    const bool wasPlaying = m_playbackTimer->isActive();
    m_playbackTimer->stop();
    m_playButton->setText(QStringLiteral("播放"));
    if (wasPlaying && m_playbackStateChangedHandler)
        m_playbackStateChangedHandler(false);
}

void ResultControlPanel::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("ResultControlPanelClass");
    }
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(0, 0, 0, 0);
    QFrame* analysisParameterCard = new QFrame(this);
    analysisParameterCard->setObjectName("analysisParameterCard");
    m_cardLayout = new QVBoxLayout(analysisParameterCard);
    m_cardLayout->setSpacing(8);
    m_cardLayout->setObjectName("cardLayout");
    m_cardLayout->setContentsMargins(12, 12, 12, 12);
    QLabel* resultDisplaySectionTitle = new QLabel(analysisParameterCard);
    resultDisplaySectionTitle->setObjectName("resultDisplaySectionTitle");
    QFont font;
    font.setBold(true);
    resultDisplaySectionTitle->setFont(font);

    m_cardLayout->addWidget(resultDisplaySectionTitle);

    m_resultForm = new QFormLayout();
    m_resultForm->setObjectName("resultForm");
    m_fieldLabel = new QLabel(analysisParameterCard);
    m_fieldLabel->setObjectName("fieldLabel");

    m_resultForm->setWidget(0, QFormLayout::LabelRole, m_fieldLabel);

    m_fieldCombo = new ResultSelectionComboBox(analysisParameterCard);
    m_fieldCombo->addItem(QString());
    m_fieldCombo->addItem(QString());
    m_fieldCombo->addItem(QString());
    m_fieldCombo->addItem(QString());
    m_fieldCombo->setObjectName("fieldCombo");

    m_resultForm->setWidget(0, QFormLayout::FieldRole, m_fieldCombo);

    m_stepLabel = new QLabel(analysisParameterCard);
    m_stepLabel->setObjectName("stepLabel");

    m_resultForm->setWidget(1, QFormLayout::LabelRole, m_stepLabel);

    m_stepCombo = new ResultSelectionComboBox(analysisParameterCard);
    m_stepCombo->setObjectName("stepCombo");
    m_stepCombo->setEnabled(false);

    m_resultForm->setWidget(1, QFormLayout::FieldRole, m_stepCombo);

    m_scaleLabel = new QLabel(analysisParameterCard);
    m_scaleLabel->setObjectName("scaleLabel");

    m_resultForm->setWidget(2, QFormLayout::LabelRole, m_scaleLabel);

    m_scaleSpin = new ResultScaleSpinBox(analysisParameterCard);
    m_scaleSpin->setObjectName("scaleSpin");
    m_scaleSpin->setMaximum(1000000.000000000000000);
    m_scaleSpin->setDecimals(4);

    m_resultForm->setWidget(2, QFormLayout::FieldRole, m_scaleSpin);

    m_originalCheck = new QCheckBox(analysisParameterCard);
    m_originalCheck->setObjectName("originalCheck");
    m_originalCheck->setChecked(true);

    m_resultForm->setWidget(3, QFormLayout::FieldRole, m_originalCheck);

    m_cardLayout->addLayout(m_resultForm);

    QLabel* resultPlaybackSectionTitle = new QLabel(analysisParameterCard);
    resultPlaybackSectionTitle->setObjectName("resultPlaybackSectionTitle");
    resultPlaybackSectionTitle->setFont(font);

    m_cardLayout->addWidget(resultPlaybackSectionTitle);

    m_frameRow = new QHBoxLayout();
    m_frameRow->setObjectName("frameRow");
    m_playButton = new QPushButton(analysisParameterCard);
    m_playButton->setObjectName("playButton");
    m_playButton->setEnabled(false);

    m_frameRow->addWidget(m_playButton);

    m_speedLabel = new QLabel(analysisParameterCard);
    m_speedLabel->setObjectName("speedLabel");

    m_frameRow->addWidget(m_speedLabel);

    m_speedCombo = new ResultSelectionComboBox(analysisParameterCard);
    m_speedCombo->addItem(QString());
    m_speedCombo->addItem(QString());
    m_speedCombo->addItem(QString());
    m_speedCombo->addItem(QString());
    m_speedCombo->addItem(QString());
    m_speedCombo->addItem(QString());
    m_speedCombo->setObjectName("speedCombo");
    m_speedCombo->setMinimumSize(QSize(70, 0));

    m_frameRow->addWidget(m_speedCombo);

    m_cardLayout->addLayout(m_frameRow);

    m_frameSlider = new QSlider(analysisParameterCard);
    m_frameSlider->setObjectName("frameSlider");
    m_frameSlider->setOrientation(Qt::Horizontal);
    m_frameSlider->setEnabled(false);

    m_cardLayout->addWidget(m_frameSlider);

    m_resultFrameStatusLayout = new QGridLayout();
    m_resultFrameStatusLayout->setObjectName("resultFrameStatusLayout");
    m_resultFrameStatusLayout->setHorizontalSpacing(12);
    m_resultFrameStatusLayout->setVerticalSpacing(4);
    m_frameCaptionLabel = new QLabel(analysisParameterCard);
    m_frameCaptionLabel->setObjectName("frameCaptionLabel");
    m_frameCaptionLabel->setMinimumSize(QSize(72, 22));
    m_frameCaptionLabel->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);

    m_resultFrameStatusLayout->addWidget(m_frameCaptionLabel, 0, 0, 1, 1);

    m_frameLabel = new QLabel(analysisParameterCard);
    m_frameLabel->setObjectName("frameLabel");
    m_frameLabel->setMinimumSize(QSize(120, 22));

    m_resultFrameStatusLayout->addWidget(m_frameLabel, 0, 1, 1, 1);

    m_timeCaptionLabel = new QLabel(analysisParameterCard);
    m_timeCaptionLabel->setObjectName("timeCaptionLabel");
    m_timeCaptionLabel->setMinimumSize(QSize(72, 22));
    m_timeCaptionLabel->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);

    m_resultFrameStatusLayout->addWidget(m_timeCaptionLabel, 1, 0, 1, 1);

    m_timeValueLabel = new QLabel(analysisParameterCard);
    m_timeValueLabel->setObjectName("timeValueLabel");
    m_timeValueLabel->setMinimumSize(QSize(120, 22));

    m_resultFrameStatusLayout->addWidget(m_timeValueLabel, 1, 1, 1, 1);

    m_deformationCaptionLabel = new QLabel(analysisParameterCard);
    m_deformationCaptionLabel->setObjectName("deformationCaptionLabel");
    m_deformationCaptionLabel->setMinimumSize(QSize(72, 22));
    m_deformationCaptionLabel->setAlignment(Qt::AlignLeading | Qt::AlignLeft | Qt::AlignTop);

    m_resultFrameStatusLayout->addWidget(m_deformationCaptionLabel, 2, 0, 1, 1);

    m_deformationValueLabel = new QLabel(analysisParameterCard);
    m_deformationValueLabel->setObjectName("deformationValueLabel");
    m_deformationValueLabel->setMinimumSize(QSize(120, 22));

    m_resultFrameStatusLayout->addWidget(m_deformationValueLabel, 2, 1, 1, 1);

    m_resultFrameStatusLayout->setColumnStretch(1, 1);

    m_cardLayout->addLayout(m_resultFrameStatusLayout);

    QLabel* resultExportSectionTitle = new QLabel(analysisParameterCard);
    resultExportSectionTitle->setObjectName("resultExportSectionTitle");
    resultExportSectionTitle->setFont(font);

    m_cardLayout->addWidget(resultExportSectionTitle);

    m_resultExportGrid = new QGridLayout();
    m_resultExportGrid->setObjectName("resultExportGrid");
    m_resultExportGrid->setHorizontalSpacing(8);
    m_resultExportGrid->setVerticalSpacing(8);
    m_exportButton = new QPushButton(analysisParameterCard);
    m_exportButton->setObjectName("exportButton");
    m_exportButton->setEnabled(false);

    m_resultExportGrid->addWidget(m_exportButton, 0, 0, 1, 1);

    m_exportElementButton = new QPushButton(analysisParameterCard);
    m_exportElementButton->setObjectName("exportElementButton");
    m_exportElementButton->setEnabled(false);

    m_resultExportGrid->addWidget(m_exportElementButton, 0, 1, 1, 1);

    m_exportIterationButton = new QPushButton(analysisParameterCard);
    m_exportIterationButton->setObjectName("exportIterationButton");
    m_exportIterationButton->setEnabled(false);

    m_resultExportGrid->addWidget(m_exportIterationButton, 1, 0, 1, 1);

    m_exportPlotButton = new QPushButton(analysisParameterCard);
    m_exportPlotButton->setObjectName("exportPlotButton");
    m_exportPlotButton->setEnabled(false);
    m_exportPlotButton->setMinimumSize(QSize(0, 34));

    m_resultExportGrid->addWidget(m_exportPlotButton, 1, 1, 1, 1);

    m_exportEnergyButton = new QPushButton(analysisParameterCard);
    m_exportEnergyButton->setObjectName("exportEnergyButton");
    m_exportEnergyButton->setEnabled(false);
    m_resultExportGrid->addWidget(m_exportEnergyButton, 2, 0, 1, 2);

    m_resultExportGrid->setColumnStretch(0, 1);
    m_resultExportGrid->setColumnStretch(1, 1);

    m_cardLayout->addLayout(m_resultExportGrid);

    rootLayout->addWidget(analysisParameterCard);

#if QT_CONFIG(shortcut)
    m_fieldLabel->setBuddy(m_fieldCombo);
    m_stepLabel->setBuddy(m_stepCombo);
    m_scaleLabel->setBuddy(m_scaleSpin);
    m_speedLabel->setBuddy(m_speedCombo);
#endif // QT_CONFIG(shortcut)
    QWidget::setTabOrder(m_fieldCombo, m_stepCombo);
    QWidget::setTabOrder(m_stepCombo, m_scaleSpin);
    QWidget::setTabOrder(m_scaleSpin, m_originalCheck);
    QWidget::setTabOrder(m_originalCheck, m_playButton);
    QWidget::setTabOrder(m_playButton, m_speedCombo);
    QWidget::setTabOrder(m_speedCombo, m_frameSlider);
    QWidget::setTabOrder(m_frameSlider, m_exportButton);
    QWidget::setTabOrder(m_exportButton, m_exportElementButton);
    QWidget::setTabOrder(m_exportElementButton, m_exportIterationButton);
    QWidget::setTabOrder(m_exportIterationButton, m_exportPlotButton);
    QWidget::setTabOrder(m_exportPlotButton, m_exportEnergyButton);

    resultDisplaySectionTitle->setText(QCoreApplication::translate("ResultControlPanelClass", "显示设置", nullptr));
    m_fieldLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "云图", nullptr));
    m_fieldCombo->setItemText(0, QCoreApplication::translate("ResultControlPanelClass", "位移合量 |U|", nullptr));
    m_fieldCombo->setItemText(1, QCoreApplication::translate("ResultControlPanelClass", "轴力 / 弹簧力", nullptr));
    m_fieldCombo->setItemText(2, QCoreApplication::translate("ResultControlPanelClass", "应力", nullptr));
    m_fieldCombo->setItemText(3, QCoreApplication::translate("ResultControlPanelClass", "应变", nullptr));

    m_stepLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "分析步", nullptr));
#if QT_CONFIG(tooltip)
    m_stepCombo->setToolTip(
        QCoreApplication::translate("ResultControlPanelClass", "切换当前 H5 结果中的分析步", nullptr));
#endif // QT_CONFIG(tooltip)
    m_scaleLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "变形比例", nullptr));
    m_scaleSpin->setSpecialValueText(QCoreApplication::translate("ResultControlPanelClass", "自动", nullptr));
    m_originalCheck->setText(QCoreApplication::translate("ResultControlPanelClass", "叠加原模型", nullptr));
    resultPlaybackSectionTitle->setText(QCoreApplication::translate("ResultControlPanelClass", "帧与播放", nullptr));
    m_playButton->setText(QCoreApplication::translate("ResultControlPanelClass", "播放", nullptr));
    m_speedLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "速度", nullptr));
    m_speedCombo->setItemText(0, QCoreApplication::translate("ResultControlPanelClass", "0.25×", nullptr));
    m_speedCombo->setItemText(1, QCoreApplication::translate("ResultControlPanelClass", "0.5×", nullptr));
    m_speedCombo->setItemText(2, QCoreApplication::translate("ResultControlPanelClass", "1×", nullptr));
    m_speedCombo->setItemText(3, QCoreApplication::translate("ResultControlPanelClass", "2×", nullptr));
    m_speedCombo->setItemText(4, QCoreApplication::translate("ResultControlPanelClass", "4×", nullptr));
    m_speedCombo->setItemText(5, QCoreApplication::translate("ResultControlPanelClass", "8×", nullptr));

#if QT_CONFIG(tooltip)
    m_speedCombo->setToolTip(QCoreApplication::translate("ResultControlPanelClass", "结果动画播放速度", nullptr));
#endif // QT_CONFIG(tooltip)
    m_frameCaptionLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "帧", nullptr));
    m_frameLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "尚未加载结果帧", nullptr));
    m_timeCaptionLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "时间", nullptr));
    m_timeValueLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "--", nullptr));
    m_deformationCaptionLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "变形比例", nullptr));
    m_deformationValueLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "--", nullptr));
    resultExportSectionTitle->setText(QCoreApplication::translate("ResultControlPanelClass", "导出与查看", nullptr));
    m_exportButton->setText(QCoreApplication::translate("ResultControlPanelClass", "导出节点数据", nullptr));
    m_exportElementButton->setText(QCoreApplication::translate("ResultControlPanelClass", "导出单元数据", nullptr));
    m_exportIterationButton->setText(QCoreApplication::translate("ResultControlPanelClass", "导出迭代步", nullptr));
    m_exportPlotButton->setText(QCoreApplication::translate("ResultControlPanelClass", "查看时程曲线", nullptr));
    m_exportEnergyButton->setText(QCoreApplication::translate("ResultControlPanelClass", "导出能量", nullptr));

    m_speedCombo->setCurrentIndex(2);
}
