#include "Widgets/ResultControlPanel.h"
#include "Export/Hdf5ModelIO.h"
#include "ui_ResultControlPanel.h"

#include <array>
#include <cmath>

#include <QSignalBlocker>

ResultControlPanel::ResultControlPanel(QWidget* parent)
    : QWidget(parent)
    , m_ui(new Ui::ResultControlPanelClass)
    , m_reader(std::make_unique<Hdf5ModelIO>())
    , m_playbackTimer(new QTimer(this))
{
    m_ui->setupUi(this);
    m_ui->exportButton->setObjectName(QStringLiteral("exportNodeResultsButton"));
    constexpr std::array playbackSpeeds{0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0};
    m_ui->speedCombo->clear();
    for (const double speed : playbackSpeeds)
    {
        m_ui->speedCombo->addItem(QStringLiteral("%1×").arg(speed, 0, 'g'), speed);
    }
    m_ui->speedCombo->setCurrentIndex(2);

    m_playbackTimer->setTimerType(Qt::PreciseTimer);
    m_playbackTimer->setInterval(16);
    connect(m_playbackTimer, &QTimer::timeout, this,
            [this]()
            {
                if (m_frames.size() < 2)
                    return;

                const double speed = m_ui->speedCombo->currentData().toDouble();
                constexpr double sourceFrameDurationMs = 80.0;
                m_playbackPosition += static_cast<double>(m_playbackTimer->interval()) / sourceFrameDurationMs * speed;
                m_playbackPosition = std::fmod(m_playbackPosition, static_cast<double>(m_frames.size()));

                const int displayedFrame = static_cast<int>(std::floor(m_playbackPosition));
                {
                    const QSignalBlocker blocker(m_ui->frameSlider);
                    m_ui->frameSlider->setValue(displayedFrame);
                }
                if (m_frameChangedHandler)
                    m_frameChangedHandler(m_playbackPosition);
            });
    connect(m_ui->playButton, &QPushButton::clicked, this,
            [this]()
            {
                const bool playing = m_playbackTimer->isActive();
                if (playing)
                    m_playbackTimer->stop();
                else
                {
                    m_playbackPosition = static_cast<double>(m_ui->frameSlider->value());
                    m_playbackTimer->start();
                }
                m_ui->playButton->setText(playing ? QStringLiteral("播放") : QStringLiteral("暂停"));
                if (m_playbackStateChangedHandler)
                    m_playbackStateChangedHandler(!playing);
            });
    connect(m_ui->frameSlider, &QSlider::valueChanged, this,
            [this](int frameIndex)
            {
                m_playbackPosition = static_cast<double>(frameIndex);
                if (m_frameChangedHandler)
                    m_frameChangedHandler(m_playbackPosition);
            });
    const auto visualizationChanged = [this]()
    {
        if (m_visualizationChangedHandler)
            m_visualizationChangedHandler();
    };
    connect(m_ui->fieldCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [visualizationChanged](int)
            {
                visualizationChanged();
            });
    connect(m_ui->scaleSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [visualizationChanged](double)
            {
                visualizationChanged();
            });
    connect(m_ui->originalCheck, &QCheckBox::toggled, this,
            [visualizationChanged](bool)
            {
                visualizationChanged();
            });
    connect(m_ui->exportButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_exportHandler)
                    m_exportHandler();
            });
    connect(m_ui->exportElementButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_elementExportHandler)
                    m_elementExportHandler();
            });
    connect(m_ui->exportIterationButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_iterationExportHandler)
                    m_iterationExportHandler();
            });
}
ResultControlPanel::~ResultControlPanel()
{
    delete m_ui;
}
QComboBox* ResultControlPanel::fieldCombo() const
{
    return m_ui->fieldCombo;
}
QDoubleSpinBox* ResultControlPanel::scaleSpin() const
{
    return m_ui->scaleSpin;
}
QCheckBox* ResultControlPanel::originalCheck() const
{
    return m_ui->originalCheck;
}
QPushButton* ResultControlPanel::playButton() const
{
    return m_ui->playButton;
}
QSlider* ResultControlPanel::frameSlider() const
{
    return m_ui->frameSlider;
}
QComboBox* ResultControlPanel::speedCombo() const
{
    return m_ui->speedCombo;
}
QLabel* ResultControlPanel::frameLabel() const
{
    return m_ui->frameLabel;
}

QLabel* ResultControlPanel::timeValueLabel() const
{
    return m_ui->timeValueLabel;
}

QLabel* ResultControlPanel::deformationValueLabel() const
{
    return m_ui->deformationValueLabel;
}

QPushButton* ResultControlPanel::exportButton() const
{
    return m_ui->exportButton;
}

QPushButton* ResultControlPanel::exportElementButton() const
{
    return m_ui->exportElementButton;
}

QPushButton* ResultControlPanel::exportIterationButton() const
{
    return m_ui->exportIterationButton;
}

Hdf5ModelIO* ResultControlPanel::reader() const
{
    return m_reader.get();
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

void ResultControlPanel::stopPlayback()
{
    const bool wasPlaying = m_playbackTimer->isActive();
    m_playbackTimer->stop();
    m_ui->playButton->setText(QStringLiteral("播放"));
    if (wasPlaying && m_playbackStateChangedHandler)
        m_playbackStateChangedHandler(false);
}
