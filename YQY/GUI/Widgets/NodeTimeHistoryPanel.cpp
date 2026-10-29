#include "Widgets/NodeTimeHistoryPanel.h"

#include "Export/Hdf5ResultReader.h"
#include "Export/ResultExporter.h"
#include "GUI/Dialogs/FileDialogService.h"
#include "Utility/ResultExportPaths.h"
#include "Widgets/SingleLineLabel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

#include <QCheckBox>
#include <QComboBox>
#include <QFileInfo>
#include <QFutureWatcher>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStringList>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>

static constexpr std::array<const char*, 6> ComponentSymbols = {"ux", "uy", "uz", "rx", "ry", "rz"};
static constexpr std::array<const char*, 6> ComponentTitles = {"X 向位移",  "Y 向位移",  "Z 向位移",
                                                        "绕 X 转角", "绕 Y 转角", "绕 Z 转角"};
static constexpr std::array<const char*, 6> ComponentUnits = {"m", "m", "m", "rad", "rad", "rad"};
static constexpr std::array<const char*, 6> ComponentColors = {"#35C6D8", "#5E96FF", "#8DDA78",
                                                        "#F5C451", "#E989B7", "#BAA7FF"};
static constexpr std::array<const char*, 4> EnergyTitles = {"动能", "应变能", "重力势能", "总机械能"};
static constexpr std::array<unsigned int, 4> EnergyMasks = {EnergyResult::Kinetic, EnergyResult::Strain,
                                                          EnergyResult::Gravity, EnergyResult::Mechanical};

static double componentValue(const Hdf5NodalDisplacementHistorySample& sample, int component)
{
    return sample.displacement[static_cast<std::size_t>(component)];
}

static QString compactNumber(double value)
{
    return QString::number(value, 'g', 5);
}

class NodeTimeHistoryChart final : public QWidget
{
public:
    explicit NodeTimeHistoryChart(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumSize(260, 360);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    void setData(int nodeId, const std::vector<Hdf5NodalDisplacementHistorySample>& samples)
    {
        m_nodeId = nodeId;
        m_energy = false;
        updateGridSize();
        prepareCurves(&samples, nullptr);
        update();
    }

    void setEnergyData(const std::vector<Hdf5EnergyHistorySample>& samples)
    {
        m_nodeId = -1;
        m_energy = true;
        updateGridSize();
        prepareCurves(nullptr, &samples);
        update();
    }

    void clearData(const QString& message = QStringLiteral("输入节点 ID 后点击“绘制”查看六自由度时程"))
    {
        m_nodeId = -1;
        m_hasData = false;
        for (auto& curve : m_curves)
            curve = CurveData{};
        m_emptyMessage = message;
        update();
    }

    void setComponents(const std::array<bool, 6>& components)
    {
        m_components = components;
        update();
    }

protected:
    void resizeEvent(QResizeEvent* event) override
    {
        QWidget::resizeEvent(event);
        updateGridSize();
    }

    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.fillRect(rect(), QColor(QStringLiteral("#101924")));
        if (!m_hasData)
        {
            painter.setPen(QColor(QStringLiteral("#9CB1C7")));
            painter.drawText(rect().adjusted(24, 16, -24, -16), Qt::AlignCenter | Qt::TextWordWrap, m_emptyMessage);
            return;
        }

        const QRectF content = rect().adjusted(8, 8, -8, -8);
        const double gap = 8.0;
        const int columns = m_columns;
        const int componentCount = m_energy ? 4 : 6;
        const int rows = (componentCount + columns - 1) / columns;
        const double panelWidth = (content.width() - gap * (columns - 1)) / columns;
        const double panelHeight = (content.height() - gap * (rows - 1)) / rows;
        for (int component = 0; component < componentCount; ++component)
        {
            const int row = component / columns;
            const int column = component % columns;
            const QRectF panel(content.left() + column * (panelWidth + gap), content.top() + row * (panelHeight + gap),
                               panelWidth, panelHeight);
            paintComponent(painter, panel, component);
        }
    }

private:
    void updateGridSize()
    {
        m_columns = width() >= 780 && !m_energy ? 3 : width() >= 520 ? 2 : 1;
        const int rows = ((m_energy ? 4 : 6) + m_columns - 1) / m_columns;
        const int minimumHeight = 180 * rows + 8 * (rows - 1) + 16;
        if (minimumHeight != this->minimumHeight())
        {
            setMinimumHeight(minimumHeight);
        }
    }

    struct CurveData
    {
        std::vector<QPointF> points;
        double minimum = 0.0;
        double maximum = 0.0;
        bool valid = false;
        std::size_t segmentStride = 1;
    };

    QPointF samplePoint(std::size_t index, int component,
                        const std::vector<Hdf5NodalDisplacementHistorySample>* nodes,
                        const std::vector<Hdf5EnergyHistorySample>* energies) const
    {
        if (nodes)
            return QPointF((*nodes)[index].time, componentValue((*nodes)[index], component));
        const auto& sample = (*energies)[index];
        if (!(sample.energy.availableMask & EnergyMasks[static_cast<std::size_t>(component)]))
            return QPointF(sample.info.time, std::numeric_limits<double>::quiet_NaN());
        const double values[] = {sample.energy.kinetic, sample.energy.strain, sample.energy.gravity,
                                 sample.energy.mechanical};
        return QPointF(sample.info.time, values[component]);
    }

    bool validSample(std::size_t index, int component,
                     const std::vector<Hdf5NodalDisplacementHistorySample>* nodes,
                     const std::vector<Hdf5EnergyHistorySample>* energies) const
    {
        const QPointF point = samplePoint(index, component, nodes, energies);
        return std::isfinite(point.x()) && std::isfinite(point.y());
    }

    void appendSegment(_OUT CurveData& curve, std::size_t begin, std::size_t end, std::size_t pointBudget,
                       int component, const std::vector<Hdf5NodalDisplacementHistorySample>* nodes,
                       const std::vector<Hdf5EnergyHistorySample>* energies)
    {
        if (!curve.points.empty())
            curve.points.emplace_back(0.0, std::numeric_limits<double>::quiet_NaN());
        const std::size_t bucketCount = std::max<std::size_t>(1, pointBudget / 4);
        const std::size_t stride = std::max<std::size_t>(1, (end - begin + bucketCount - 1) / bucketCount);
        for (std::size_t bucket = begin; bucket < end; bucket += stride)
        {
            const std::size_t bucketEnd = std::min(end, bucket + stride);
            std::size_t minimumIndex = bucket, maximumIndex = bucket;
            double minimum = samplePoint(bucket, component, nodes, energies).y();
            double maximum = minimum;
            for (std::size_t index = bucket + 1; index < bucketEnd; ++index)
            {
                const double value = samplePoint(index, component, nodes, energies).y();
                if (value < minimum)
                {
                    minimum = value;
                    minimumIndex = index;
                }
                if (value > maximum)
                {
                    maximum = value;
                    maximumIndex = index;
                }
            }
            std::array<std::size_t, 4> indices = {bucket, minimumIndex, maximumIndex, bucketEnd - 1};
            std::sort(indices.begin(), indices.end());
            std::size_t previous = end;
            for (const std::size_t index : indices)
            {
                if (index != previous)
                    curve.points.push_back(samplePoint(index, component, nodes, energies));
                previous = index;
            }
        }
    }

    void prepareCurves(const std::vector<Hdf5NodalDisplacementHistorySample>* nodes,
                       const std::vector<Hdf5EnergyHistorySample>* energies)
    {
        for (auto& curve : m_curves)
            curve = CurveData{};
        const std::size_t count = nodes ? nodes->size() : energies->size();
        m_hasData = count != 0;
        if (!m_hasData)
            return;
        m_firstTime = samplePoint(0, 0, nodes, energies).x();
        m_lastTime = samplePoint(count - 1, 0, nodes, energies).x();
        // 先识别有效连续段，再在段内保留首末点与极值；缺失值不会吞掉同桶的有效点。
        // 超过 1024 段时按段抽样并提示；包括分隔符在内，每条绘图曲线至多 8192 点。
        const int componentCount = energies ? 4 : 6;
        for (int component = 0; component < componentCount; ++component)
        {
            CurveData& curve = m_curves[static_cast<std::size_t>(component)];
            curve.points.reserve(std::min<std::size_t>(count, 8192));
            std::size_t segmentCount = 0;
            bool previousValid = false;
            for (std::size_t index = 0; index < count; ++index)
            {
                const QPointF point = samplePoint(index, component, nodes, energies);
                const bool valid = std::isfinite(point.x()) && std::isfinite(point.y());
                if (valid)
                {
                    if (!previousValid)
                        ++segmentCount;
                    if (!curve.valid)
                        curve.minimum = curve.maximum = point.y();
                    curve.valid = true;
                    curve.minimum = std::min(curve.minimum, point.y());
                    curve.maximum = std::max(curve.maximum, point.y());
                }
                previousValid = valid;
            }
            if (!segmentCount)
                continue;
            curve.segmentStride = (segmentCount + 1023) / 1024;
            const std::size_t displayedSegments = (segmentCount + curve.segmentStride - 1) / curve.segmentStride;
            const std::size_t pointBudget = (8192 - displayedSegments) / displayedSegments;
            std::size_t segmentIndex = 0;
            std::size_t index = 0;
            while (index < count)
            {
                while (index < count && !validSample(index, component, nodes, energies))
                    ++index;
                const std::size_t begin = index;
                while (index < count && validSample(index, component, nodes, energies))
                    ++index;
                if (begin == index)
                    break;
                if (segmentIndex % curve.segmentStride == 0)
                    appendSegment(curve, begin, index, pointBudget, component, nodes, energies);
                ++segmentIndex;
            }
        }
    }

    void paintComponent(QPainter& painter, const QRectF& panel, int component) const
    {
        painter.setPen(QPen(QColor(QStringLiteral("#2A3C52")), 1.0));
        painter.setBrush(QColor(QStringLiteral("#141F2C")));
        painter.drawRoundedRect(panel, 5.0, 5.0);
        const auto componentIndex = static_cast<std::size_t>(component);
        QString title =
            m_energy ? QStringLiteral("%1 [J]").arg(QString::fromUtf8(EnergyTitles[componentIndex]))
                     : QStringLiteral("%1  %2 [%3]")
                           .arg(QString::fromLatin1(ComponentSymbols[componentIndex]),
                                QString::fromUtf8(ComponentTitles[componentIndex]),
                                QString::fromLatin1(ComponentUnits[componentIndex]));
        const CurveData& curve = m_curves[componentIndex];
        if (curve.segmentStride > 1)
            title += QStringLiteral(" · 有效段抽样 1/%1").arg(curve.segmentStride);
        painter.setPen(m_components[static_cast<std::size_t>(component)] ? QColor(QStringLiteral("#E8F0FA"))
                                                                         : QColor(QStringLiteral("#6F839A")));
        painter.drawText(panel.adjusted(10, 6, -10, -4), Qt::AlignLeft | Qt::AlignTop, title);

        if (!m_components[static_cast<std::size_t>(component)])
        {
            painter.setPen(QColor(QStringLiteral("#6F839A")));
            painter.drawText(panel.adjusted(8, 22, -8, -8), Qt::AlignCenter, QStringLiteral("未选择"));
            return;
        }

        if (!curve.valid)
        {
            painter.setPen(QColor(QStringLiteral("#9CB1C7")));
            painter.drawText(panel.adjusted(8, 22, -8, -8), Qt::AlignCenter | Qt::TextWordWrap,
                             QStringLiteral("无有效记录或当前模型不支持此结果"));
            return;
        }

        const QRectF plot = panel.adjusted(45, 26, -12, -26);
        if (plot.width() <= 4.0 || plot.height() <= 4.0)
            return;
        const double firstTime = m_firstTime;
        const double lastTime = m_lastTime;
        const double duration = std::max(1.0e-12, lastTime - firstTime);
        double minimum = curve.minimum;
        double maximum = curve.maximum;
        if (std::abs(maximum - minimum) <= 1.0e-15)
        {
            const double margin = std::max(1.0, std::abs(maximum) * 0.05);
            minimum -= margin;
            maximum += margin;
        }
        else
        {
            const double margin = (maximum - minimum) * 0.06;
            minimum -= margin;
            maximum += margin;
        }
        const auto mapX = [&plot, firstTime, duration](double time)
        {
            return plot.left() + (time - firstTime) / duration * plot.width();
        };
        const auto mapY = [&plot, minimum, maximum](double value)
        {
            return plot.bottom() - (value - minimum) / (maximum - minimum) * plot.height();
        };

        painter.setPen(QPen(QColor(QStringLiteral("#27394E")), 1.0));
        for (int line = 0; line <= 4; ++line)
        {
            const double y = plot.top() + plot.height() * static_cast<double>(line) / 4.0;
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
        }
        if (minimum <= 0.0 && maximum >= 0.0)
        {
            painter.setPen(QPen(QColor(QStringLiteral("#61748A")), 1.0, Qt::DashLine));
            painter.drawLine(QPointF(plot.left(), mapY(0.0)), QPointF(plot.right(), mapY(0.0)));
        }
        QPainterPath path;
        bool connected = false;
        for (const QPointF& sample : curve.points)
        {
            if (!std::isfinite(sample.y()))
            {
                connected = false;
                continue;
            }
            const QPointF point(mapX(sample.x()), mapY(sample.y()));
            if (!connected)
                path.moveTo(point);
            else
                path.lineTo(point);
            connected = true;
        }
        painter.setPen(QPen(QColor(QString::fromLatin1(ComponentColors[static_cast<std::size_t>(component)])), 1.35,
                            Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);
        for (std::size_t index = 0; index < curve.points.size(); ++index)
        {
            const QPointF& point = curve.points[index];
            const bool firstInSegment = index == 0 || !std::isfinite(curve.points[index - 1].y());
            const bool lastInSegment = index + 1 == curve.points.size() ||
                                       !std::isfinite(curve.points[index + 1].y());
            if (firstInSegment && lastInSegment && std::isfinite(point.y()))
                painter.drawEllipse(QPointF(mapX(point.x()), mapY(point.y())), 2.0, 2.0);
        }

        painter.setPen(QColor(QStringLiteral("#9CB1C7")));
        painter.drawText(QPointF(panel.left() + 7, plot.top() + 9), compactNumber(maximum));
        painter.drawText(QPointF(panel.left() + 7, plot.bottom()), compactNumber(minimum));
        const QString firstText = QStringLiteral("%1 s").arg(compactNumber(firstTime));
        const QString lastText = QStringLiteral("%1 s").arg(compactNumber(lastTime));
        painter.drawText(QPointF(plot.left(), panel.bottom() - 8), firstText);
        painter.drawText(QPointF(plot.right() - painter.fontMetrics().horizontalAdvance(lastText), panel.bottom() - 8),
                         lastText);
    }

    int m_nodeId = -1;
    bool m_energy = false;
    bool m_hasData = false;
    double m_firstTime = 0.0;
    double m_lastTime = 0.0;
    QString m_emptyMessage = QStringLiteral("输入节点 ID 后点击“绘制”查看六自由度时程");
    std::array<bool, 6> m_components = {true, true, true, true, true, true};
    std::array<CurveData, 6> m_curves;
    int m_columns = 3;
};

NodeTimeHistoryPanel::NodeTimeHistoryPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    auto* scroll = new QScrollArea(this);
    m_scrollArea = scroll;
    scroll->viewport()->installEventFilter(this);
    scroll->setObjectName(QStringLiteral("historyScrollArea"));
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidgetResizable(true);
    auto* content = new QWidget(scroll);
    auto* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(12, 8, 12, 10);
    contentLayout->setSpacing(8);
    scroll->setWidget(content);
    root->addWidget(scroll);

    m_queryHost = new QWidget(content);
    m_queryHost->setObjectName(QStringLiteral("historyQueryBar"));
    m_queryHost->installEventFilter(this);
    m_queryLayout = new QVBoxLayout(m_queryHost);
    m_queryLayout->setContentsMargins(0, 0, 0, 0);
    m_queryLayout->setSpacing(8);
    m_returnButton = new QPushButton(QStringLiteral("返回模型视图"), this);
    m_plotType = new QComboBox(this);
    m_plotType->setObjectName(QStringLiteral("historyPlotType"));
    m_plotType->addItem(QStringLiteral("节点时程"));
    m_plotType->addItem(QStringLiteral("能量时程"));
    m_nodeQueryPanel = new QWidget(m_queryHost);
    m_nodeQueryPanel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
    auto* nodeLayout = new QHBoxLayout(m_nodeQueryPanel);
    nodeLayout->setContentsMargins(0, 0, 0, 0);
    nodeLayout->setSpacing(6);
    m_nodeLabel = new QLabel(QStringLiteral("节点 ID"), m_nodeQueryPanel);
    m_nodeId = new QSpinBox(m_nodeQueryPanel);
    m_nodeId->setObjectName(QStringLiteral("historyNodeId"));
    m_nodeId->setRange(1, std::numeric_limits<int>::max());
    m_nodeId->setValue(1);
    m_nodeId->setFixedWidth(124);
    m_nodeId->setKeyboardTracking(false);
    m_drawButton = new QPushButton(QStringLiteral("绘制"), this);
    m_drawButton->setObjectName(QStringLiteral("historyDrawButton"));
    nodeLayout->addWidget(m_nodeLabel);
    nodeLayout->addWidget(m_nodeId);
    contentLayout->addWidget(m_queryHost);
    for (int component = 0; component < 6; ++component)
    {
        const auto index = static_cast<std::size_t>(component);
        auto* check = new QCheckBox(QString::fromLatin1(ComponentSymbols[index]), m_queryHost);
        check->setToolTip(QStringLiteral("%1 [%2]")
                              .arg(QString::fromUtf8(ComponentTitles[index]),
                                   QString::fromLatin1(ComponentUnits[index])));
        check->setChecked(true);
        m_componentChecks[index] = check;
        connect(check, &QCheckBox::toggled, this,
                [this](bool)
                {
                    m_chart->setComponents(selectedComponents());
                    updateEnergyExportState();
                });
    }
    m_energyQueryPanel = new QWidget(m_queryHost);
    m_energyQueryPanel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Fixed);
    auto* energyLayout = new QHBoxLayout(m_energyQueryPanel);
    energyLayout->setContentsMargins(0, 0, 0, 0);
    energyLayout->setSpacing(6);
    energyLayout->addWidget(new QLabel(QStringLiteral("分析步"), m_energyQueryPanel));
    m_energyStep = new QComboBox(m_energyQueryPanel);
    m_energyStep->setObjectName(QStringLiteral("historyEnergyStep"));
    m_energyStep->addItem(QStringLiteral("自动选择"), 0);
    m_energyStep->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    m_energyStep->setMinimumContentsLength(8);
    m_energyStep->setFixedWidth(160);
    energyLayout->addWidget(m_energyStep);
    for (int component = 0; component < 4; ++component)
    {
        const auto index = static_cast<std::size_t>(component);
        auto* check = new QCheckBox(QString::fromUtf8(EnergyTitles[index]), m_queryHost);
        check->setChecked(true);
        m_energyChecks[index] = check;
        check->hide();
        connect(check, &QCheckBox::toggled, this,
                [this](bool)
                {
                    m_chart->setComponents(selectedComponents());
                    updateEnergyExportState();
                });
    }
    m_energyChecks[3]->setToolTip(QStringLiteral("动能＋应变能＋重力势能，单位 J"));
    m_energyQueryPanel->hide();

    m_status = new SingleLineLabel(this);
    m_status->setObjectName(QStringLiteral("nodeHistoryStatus"));
    setStatus(QStringLiteral("输入节点 ID 后点击绘制，或按回车。"));
    contentLayout->addWidget(m_status);

    m_chart = new NodeTimeHistoryChart(this);
    m_chart->setObjectName(QStringLiteral("nodeHistoryChart"));
    contentLayout->addWidget(m_chart, 1);
    connect(m_drawButton, &QPushButton::clicked, this, &NodeTimeHistoryPanel::requestHistory);
    connect(m_plotType, &QComboBox::currentIndexChanged, this,
            [this](int)
            {
                updatePlotType();
                if (m_plotType->currentIndex() == 1)
                {
                    const int stepId = m_energyStep->currentData().toInt();
                    if (stepId > 0 && m_energyStepChangedHandler)
                        m_energyStepChangedHandler(stepId);
                    requestHistory();
                }
            });
    connect(m_nodeId, &QSpinBox::valueChanged, this, &NodeTimeHistoryPanel::invalidateQuery);
    connect(m_energyStep, &QComboBox::currentIndexChanged, this,
            [this](int)
            {
                invalidateQuery();
                const int stepId = m_energyStep->currentData().toInt();
                if (stepId > 0 && m_energyStepChangedHandler)
                {
                    m_energyStepChangedHandler(stepId);
                }
                if (m_plotType->currentIndex() == 1)
                {
                    requestHistory();
                }
            });
    // 失去焦点不自动查询，避免点击“绘制”前 editingFinished 抢先发出重复请求。
    connect(m_nodeId->findChild<QLineEdit*>(), &QLineEdit::returnPressed, this,
            &NodeTimeHistoryPanel::requestHistory);
    connect(m_returnButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_returnToModelHandler)
                    m_returnToModelHandler();
            });
    updateQueryLayout();
}

void NodeTimeHistoryPanel::setResultFile(const QString& resultFile)
{
    const QString normalized = resultFile.trimmed().isEmpty() ? QString() : QFileInfo(resultFile).absoluteFilePath();
    if (m_resultFile.compare(normalized, Qt::CaseInsensitive) == 0)
        return;
    m_resultFile = normalized;
    resetCache();
    m_energyAvailabilityKnown = false;
    m_hasEnergyRecords = true;
    const QSignalBlocker blocker(m_energyStep);
    m_energyStep->clear();
    m_energyStep->addItem(QStringLiteral("自动选择"), 0);
    invalidateQuery();
}

void NodeTimeHistoryPanel::setResultContext(const QString& resultFile,
                                          const std::vector<Hdf5ResultFrameInfo>& frames, int currentStepId,
                                          bool hasEnergyRecords)
{
    m_resultFile = resultFile.trimmed().isEmpty() ? QString() : QFileInfo(resultFile).absoluteFilePath();
    resetCache();
    m_energyAvailabilityKnown = true;
    m_hasEnergyRecords = hasEnergyRecords;
    {
        const QSignalBlocker blocker(m_energyStep);
        m_energyStep->clear();
        for (const auto& frame : frames)
        {
            if (frame.stepId > 0 && m_energyStep->findData(frame.stepId) < 0)
            {
                m_energyStep->addItem(QStringLiteral("分析步 %1").arg(frame.stepId), frame.stepId);
            }
        }
        m_energyStep->setCurrentIndex(-1);
        m_energyStep->setCurrentIndex(m_energyStep->findData(resolvedAnalysisStep(currentStepId)));
    }
    invalidateQuery();
    if (m_plotType->currentIndex() == 1 && isVisible())
    {
        requestHistory();
    }
}

int NodeTimeHistoryPanel::resolvedAnalysisStep(int stepId) const
{
    if (stepId > 0 && m_energyStep->findData(stepId) >= 0)
    {
        return stepId;
    }
    const int selectedStep = m_energyStep->currentData().toInt();
    if (selectedStep > 0)
    {
        return selectedStep;
    }
    return m_energyStep->count() > 0 ? m_energyStep->itemData(m_energyStep->count() - 1).toInt() : 0;
}

void NodeTimeHistoryPanel::setAnalysisStep(int stepId)
{
    const int resolvedStep = resolvedAnalysisStep(stepId);
    if (m_energyStep->currentData().toInt() == resolvedStep)
    {
        return;
    }
    {
        const QSignalBlocker blocker(m_energyStep);
        m_energyStep->setCurrentIndex(m_energyStep->findData(resolvedStep));
    }
    if (m_plotType->currentIndex() == 1)
    {
        invalidateQuery();
        if (isVisible())
        {
            requestHistory();
        }
    }
    updateEnergyExportState();
}

void NodeTimeHistoryPanel::showEnergyHistory(int stepId)
{
    {
        const QSignalBlocker typeBlocker(m_plotType);
        const QSignalBlocker stepBlocker(m_energyStep);
        m_plotType->setCurrentIndex(1);
        m_energyStep->setCurrentIndex(m_energyStep->findData(resolvedAnalysisStep(stepId)));
    }
    updatePlotType();
    const int selectedStep = m_energyStep->currentData().toInt();
    if (selectedStep > 0 && m_energyStepChangedHandler)
        m_energyStepChangedHandler(selectedStep);
    requestHistory();
}

void NodeTimeHistoryPanel::showTimeHistory(int nodeId)
{
    if (m_plotType->currentIndex() == 1)
    {
        showEnergyHistory(m_energyStep->currentData().toInt());
        return;
    }
    showNodeHistory(nodeId);
}

void NodeTimeHistoryPanel::showNodeHistory(int nodeId)
{
    {
        const QSignalBlocker typeBlocker(m_plotType);
        const QSignalBlocker nodeBlocker(m_nodeId);
        m_plotType->setCurrentIndex(0);
        if (nodeId > 0)
        {
            m_nodeId->setValue(nodeId);
        }
    }
    updatePlotType();
    if (nodeId > 0)
    {
        requestHistory();
    }
}

void NodeTimeHistoryPanel::resetCache()
{
    m_cachedFile.clear();
    m_cachedNodeId = -1;
    m_cachedStepId = 0;
    m_availableEnergyMask = 0;
    m_cachedSamples.reset();
    m_cachedEnergySamples.reset();
}

void NodeTimeHistoryPanel::setNodeId(int nodeId)
{
    if (nodeId > 0)
        m_nodeId->setValue(nodeId);
}

void NodeTimeHistoryPanel::drawCurrentNode()
{
    showNodeHistory(m_nodeId->value());
}

void NodeTimeHistoryPanel::clearHistory()
{
    m_resultFile.clear();
    resetCache();
    m_energyAvailabilityKnown = true;
    m_hasEnergyRecords = false;
    const QSignalBlocker blocker(m_energyStep);
    m_energyStep->clear();
    m_energyStep->addItem(QStringLiteral("自动选择"), 0);
    invalidateQuery();
}

void NodeTimeHistoryPanel::setReturnToModelHandler(std::function<void()> handler)
{
    m_returnToModelHandler = std::move(handler);
}

void NodeTimeHistoryPanel::setEnergyStepChangedHandler(std::function<void(int)> handler)
{
    m_energyStepChangedHandler = std::move(handler);
}

void NodeTimeHistoryPanel::setEnergyExportStateHandler(std::function<void(bool, bool)> handler)
{
    m_energyExportStateHandler = std::move(handler);
    updateEnergyExportState();
}

void NodeTimeHistoryPanel::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    updateQueryLayout();
}

bool NodeTimeHistoryPanel::eventFilter(QObject* watched, QEvent* event)
{
    if ((watched == m_queryHost || (m_scrollArea && watched == m_scrollArea->viewport())) &&
        (event->type() == QEvent::Resize || event->type() == QEvent::FontChange || event->type() == QEvent::StyleChange))
    {
        updateQueryLayout();
    }
    return QWidget::eventFilter(watched, event);
}

void NodeTimeHistoryPanel::updateQueryLayout()
{
    if (!m_queryLayout || !m_energyStep || !m_chart)
    {
        return;
    }
    const bool energy = m_plotType->currentIndex() == 1;
    std::vector<QWidget*> widgets{m_returnButton, m_plotType,
                                  energy ? m_energyQueryPanel : m_nodeQueryPanel, m_drawButton};
    if (energy)
        widgets.insert(widgets.end(), m_energyChecks.begin(), m_energyChecks.end());
    else
        widgets.insert(widgets.end(), m_componentChecks.begin(), m_componentChecks.end());

    const QMargins margins = m_queryHost->parentWidget()->layout()->contentsMargins();
    const int availableWidth = m_scrollArea->viewport()->width() - margins.left() - margins.right();
    constexpr int spacing = 8;
    int row = 0;
    int usedWidth = 0;
    std::vector<int> rows;
    for (QWidget* widget : widgets)
    {
        const int widgetWidth = widget->sizeHint().expandedTo(widget->minimumSizeHint())
                                    .boundedTo(widget->maximumSize()).width();
        if (usedWidth > 0 && usedWidth + spacing + widgetWidth > availableWidth)
        {
            ++row;
            usedWidth = 0;
        }
        rows.push_back(row);
        usedWidth += (usedWidth > 0 ? spacing : 0) + widgetWidth;
    }
    if (rows == m_queryRows && energy == m_queryEnergy)
    {
        return;
    }
    m_queryRows = rows;
    m_queryEnergy = energy;
    while (auto* item = m_queryLayout->takeAt(0))
    {
        if (auto* layout = item->layout())
        {
            while (auto* child = layout->takeAt(0))
                delete child;
        }
        delete item;
    }
    QHBoxLayout* rowLayout = nullptr;
    int currentRow = -1;
    for (std::size_t index = 0; index < widgets.size(); ++index)
    {
        if (rows[index] != currentRow)
        {
            if (rowLayout)
                rowLayout->addStretch();
            rowLayout = new QHBoxLayout;
            rowLayout->setContentsMargins(0, 0, 0, 0);
            rowLayout->setSpacing(spacing);
            m_queryLayout->addLayout(rowLayout);
            currentRow = rows[index];
        }
        rowLayout->addWidget(widgets[index], 0, Qt::AlignVCenter);
    }
    if (rowLayout)
        rowLayout->addStretch();
}

std::array<bool, 6> NodeTimeHistoryPanel::selectedComponents() const
{
    std::array<bool, 6> result{};
    const bool energy = m_plotType->currentIndex() == 1;
    const int count = energy ? 4 : 6;
    for (int component = 0; component < count; ++component)
        result[static_cast<std::size_t>(component)] =
            energy ? m_energyChecks[static_cast<std::size_t>(component)]->isChecked()
                   : m_componentChecks[static_cast<std::size_t>(component)]->isChecked();
    return result;
}

void NodeTimeHistoryPanel::updatePlotType()
{
    const bool energy = m_plotType->currentIndex() == 1;
    m_nodeQueryPanel->setVisible(!energy);
    m_energyQueryPanel->setVisible(energy);
    for (auto* check : m_componentChecks)
        check->setVisible(!energy);
    for (auto* check : m_energyChecks)
        check->setVisible(energy);
    updateQueryLayout();
    invalidateQuery();
}

void NodeTimeHistoryPanel::updateEnergyAvailability(unsigned int mask, bool known)
{
    if (known)
    {
        m_availableEnergyMask = mask;
    }
    for (std::size_t component = 0; component < m_energyChecks.size(); ++component)
    {
        auto* check = m_energyChecks[component];
        const bool available = !known || (mask & EnergyMasks[component]) != 0;
        check->setEnabled(available);
        QString tip = component == 3 ? QStringLiteral("动能＋应变能＋重力势能，单位 J")
                                     : QStringLiteral("%1，单位 J").arg(QString::fromUtf8(EnergyTitles[component]));
        if (!available)
        {
            tip += QStringLiteral("；当前分析步无有效记录");
        }
        check->setToolTip(tip);
    }
}

QString NodeTimeHistoryPanel::energyStatus(int stepId, std::size_t sampleCount) const
{
    QStringList available;
    QStringList missing;
    for (std::size_t component = 0; component < m_energyChecks.size(); ++component)
    {
        auto& names = (m_availableEnergyMask & EnergyMasks[component]) ? available : missing;
        names.append(QString::fromUtf8(EnergyTitles[component]));
    }
    QString text = QStringLiteral("分析步 %1 · %2 帧能量 · 单位 J").arg(stepId).arg(sampleCount);
    if (!available.isEmpty())
    {
        text += QStringLiteral(" · 可用：%1").arg(available.join(QStringLiteral("、")));
    }
    if (!missing.isEmpty())
    {
        text += QStringLiteral(" · 无有效记录：%1").arg(missing.join(QStringLiteral("、")));
    }
    return text;
}

void NodeTimeHistoryPanel::invalidateQuery()
{
    ++m_requestSerial;
    m_queryPending = false;
    const bool energy = m_plotType->currentIndex() == 1;
    const bool unavailable = energy && m_energyAvailabilityKnown && !m_hasEnergyRecords;
    m_drawButton->setEnabled(!m_resultFile.isEmpty() && !unavailable);
    updateEnergyAvailability(0, unavailable);
    const QString message = m_resultFile.isEmpty() ? QStringLiteral("尚未加载 H5 结果文件。")
                             : unavailable ? QStringLiteral("此结果文件没有能量记录，可重新计算后查看能量时程。")
                             : energy ? QStringLiteral("选择分析步后点击绘制能量时程。")
                                      : QStringLiteral("输入节点 ID 后点击绘制，或按回车。");
    m_chart->clearData(message);
    setStatus(message);
    updateEnergyExportState();
}

void NodeTimeHistoryPanel::setStatus(const QString& text, bool error)
{
    m_status->setText(text);
    m_status->setStyleSheet(error ? QStringLiteral("color: #E67373;") : QString());
}

void NodeTimeHistoryPanel::requestHistory()
{
    const bool energy = m_plotType->currentIndex() == 1;
    if (m_resultFile.isEmpty())
    {
        const QString message = QStringLiteral("请先加载 H5 结果文件。");
        setStatus(message, true);
        m_chart->clearData(message);
        return;
    }
    if (energy && m_energyAvailabilityKnown && (!m_hasEnergyRecords || m_energyStep->count() == 0))
    {
        const QString message =
            !m_hasEnergyRecords ? QStringLiteral("此结果文件没有能量记录，可重新计算后查看能量时程。")
                                : QStringLiteral("当前结果没有可用的分析步。");
        m_chart->clearData(message);
        setStatus(message);
        updateEnergyAvailability(0, true);
        m_drawButton->setEnabled(false);
        return;
    }
    if (m_loader)
    {
        ++m_requestSerial;
        m_queryPending = true;
        setStatus(QStringLiteral("已接收查询，正在等待上一次读取结束…"));
        return;
    }
    const int nodeId = m_nodeId->value();
    const int stepId = m_energyStep->currentData().toInt();
    const bool sameFile = m_cachedFile.compare(m_resultFile, Qt::CaseInsensitive) == 0;
    if (energy && sameFile && m_cachedEnergySamples && m_cachedStepId == stepId)
    {
        updateEnergyAvailability(m_availableEnergyMask, true);
        m_chart->setEnergyData(*m_cachedEnergySamples);
        m_chart->setComponents(selectedComponents());
        setStatus(energyStatus(stepId, m_cachedEnergySamples->size()));
        updateEnergyExportState();
        return;
    }
    if (!energy && sameFile && m_cachedNodeId == nodeId && m_cachedSamples)
    {
        m_chart->setData(nodeId, *m_cachedSamples);
        m_chart->setComponents(selectedComponents());
        setStatus(
            QStringLiteral("节点 %1 · 已使用内存中的 %2 帧时程。").arg(nodeId).arg(m_cachedSamples->size()));
        return;
    }

    m_drawButton->setEnabled(false);
    // 切换查询时释放旧历史；后台任务、完成结果和面板缓存共享新数据，不复制整条时程。
    resetCache();
    updateEnergyAvailability(0, false);
    const QString loading = energy ? QStringLiteral("正在直接读取 H5 能量时程…")
                                   : QStringLiteral("正在直接读取节点 %1 的 H5 时程…").arg(nodeId);
    setStatus(loading);
    m_chart->clearData(loading);
    const int requestSerial = ++m_requestSerial;
    auto* watcher = new QFutureWatcher<LoadResult>(this);
    m_loader = watcher;
    updateEnergyExportState();
    connect(watcher, &QFutureWatcher<LoadResult>::finished, this,
            [this, watcher, requestSerial]()
            {
                const LoadResult result = watcher->result();
                watcher->deleteLater();
                if (m_loader == watcher)
                    m_loader = nullptr;
                const bool energyUnavailable = m_plotType->currentIndex() == 1 &&
                                               m_energyAvailabilityKnown && !m_hasEnergyRecords;
                m_drawButton->setEnabled(!m_resultFile.isEmpty() && !energyUnavailable);
                if (m_queryPending)
                {
                    m_queryPending = false;
                    requestHistory();
                    return;
                }
                if (requestSerial != m_requestSerial)
                {
                    updateEnergyExportState();
                    return;
                }
                applyHistory(result);
                updateEnergyExportState();
            });
    const QString resultFile = m_resultFile;
    watcher->setFuture(QtConcurrent::run(
        [resultFile, nodeId, energy, stepId]()
        {
            LoadResult result;
            result.nodeId = nodeId;
            result.energy = energy;
            result.stepId = stepId;
            Hdf5ResultReader reader;
            std::vector<Hdf5ResultFrameInfo> frames;
            const bool opened = energy ? reader.openEnergyHistory(resultFile, frames) : reader.open(resultFile, frames);
            if (!opened)
            {
                result.error = QStringLiteral("无法打开 H5 结果文件。");
                return result;
            }
            if (energy)
            {
                for (const auto& frame : frames)
                {
                    if (frame.stepId > 0 &&
                        std::find(result.energyStepIds.cbegin(), result.energyStepIds.cend(), frame.stepId) ==
                            result.energyStepIds.cend())
                    {
                        result.energyStepIds.push_back(frame.stepId);
                    }
                }
                if (result.energyStepIds.empty())
                {
                    result.emptyMessage = QStringLiteral("此结果文件没有可用的能量分析步。");
                    return result;
                }
                if (std::find(result.energyStepIds.cbegin(), result.energyStepIds.cend(), result.stepId) ==
                    result.energyStepIds.cend())
                {
                    result.stepId = result.energyStepIds.back();
                }
                std::vector<Hdf5ResultFrameInfo>().swap(frames);
                auto samples = std::make_shared<std::vector<Hdf5EnergyHistorySample>>();
                const bool read = reader.readEnergyHistory(result.stepId, *samples);
                reader.close();
                if (!read)
                    result.error = QStringLiteral("无法读取 H5 能量时程，请检查结果文件。");
                else if (samples->empty())
                    result.emptyMessage = QStringLiteral("当前分析步没有能量记录，可选择其他分析步或重新计算。");
                for (const auto& sample : *samples)
                {
                    result.availableEnergyMask |= sample.energy.availableMask;
                }
                result.energySamples = std::move(samples);
            }
            else
            {
                std::vector<Hdf5ResultFrameInfo>().swap(frames);
                auto samples = std::make_shared<std::vector<Hdf5NodalDisplacementHistorySample>>();
                const bool read = reader.readNodalDisplacementHistory(nodeId, *samples);
                reader.close();
                if (!read)
                    result.error = QStringLiteral("H5 节点时程布局不满足直接读取条件。");
                else if (samples->empty())
                    result.error = QStringLiteral("节点 ID %1 不存在于当前结果的节点时程记录中，请检查节点编号。")
                                       .arg(nodeId);
                result.samples = std::move(samples);
            }
            return result;
        }));
}

unsigned int NodeTimeHistoryPanel::selectedEnergyMask() const
{
    if (m_plotType->currentIndex() != 1)
        return EnergyResult::All;
    unsigned int mask = 0;
    for (std::size_t component = 0; component < m_energyChecks.size(); ++component)
        if (m_energyChecks[component]->isEnabled() && m_energyChecks[component]->isChecked())
            mask |= EnergyMasks[component];
    return mask;
}

bool NodeTimeHistoryPanel::canExportEnergy() const
{
    return !m_resultFile.isEmpty() && !m_energyExportInProgress && !m_loader && !m_queryPending &&
           (!m_energyAvailabilityKnown || m_hasEnergyRecords) && resolvedAnalysisStep(0) > 0 &&
           selectedEnergyMask() != 0;
}

void NodeTimeHistoryPanel::updateEnergyExportState()
{
    if (m_energyExportStateHandler)
        m_energyExportStateHandler(canExportEnergy(), m_energyExportInProgress);
}

void NodeTimeHistoryPanel::exportEnergyHistory()
{
    if (!canExportEnergy())
        return;
    // 保存对话框运行期间也可能切换结果，固定本次文件、分析步和勾选分量。
    const QString resultFile = m_resultFile;
    const int stepId = resolvedAnalysisStep(0);
    const unsigned int mask = selectedEnergyMask();
    const auto samples = m_cachedFile.compare(resultFile, Qt::CaseInsensitive) == 0 && m_cachedStepId == stepId
                             ? m_cachedEnergySamples : nullptr;
    const int requestSerial = m_requestSerial;
    const QString defaultFile =
        ResultExportPaths::filePath(resultFile, QStringLiteral("能量"), QStringLiteral("bdf"));
    if (defaultFile.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("导出失败"), QStringLiteral("无法创建结果导出目录。"));
        return;
    }
    m_energyExportInProgress = true;
    updateEnergyExportState();
    const auto cancelExport = [this]()
    {
        m_energyExportInProgress = false;
        updateEnergyExportState();
    };
    QString outputFile = FileDialogService::selectSaveFile(
        this, QStringLiteral("导出分析步 %1 的能量时程").arg(stepId), defaultFile,
        QStringLiteral("BDF 数据文件 (*.bdf)"), QStringLiteral("energyResultExport"), false);
    if (outputFile.isEmpty())
    {
        cancelExport();
        return;
    }
    if (QFileInfo(outputFile).suffix().isEmpty())
        outputFile += QStringLiteral(".bdf");
    if (QFileInfo::exists(outputFile) &&
        QMessageBox::question(this, QStringLiteral("替换已有文件"),
                              QStringLiteral("文件已经存在，是否替换？\n%1").arg(outputFile),
                              QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
    {
        cancelExport();
        return;
    }
    if (requestSerial != m_requestSerial || stepId != resolvedAnalysisStep(0) ||
        resultFile.compare(m_resultFile, Qt::CaseInsensitive) != 0)
    {
        cancelExport();
        return;
    }
    if (QFileInfo(outputFile).absoluteFilePath().compare(resultFile, Qt::CaseInsensitive) == 0)
    {
        cancelExport();
        QMessageBox::warning(this, QStringLiteral("导出失败"), QStringLiteral("导出文件不能覆盖原始 H5 结果文件。"));
        return;
    }
    auto* watcher = new QFutureWatcher<QString>(this);
    connect(watcher, &QFutureWatcher<QString>::finished, this,
            [this, watcher, outputFile, requestSerial, resultFile, stepId]()
            {
                const QString error = watcher->result();
                watcher->deleteLater();
                m_energyExportInProgress = false;
                updateEnergyExportState();
                if (requestSerial == m_requestSerial && stepId == resolvedAnalysisStep(0) &&
                    resultFile.compare(m_resultFile, Qt::CaseInsensitive) == 0)
                {
                    setStatus(error.isEmpty() ? QStringLiteral("能量已导出：%1").arg(outputFile) : error,
                              !error.isEmpty());
                    if (!error.isEmpty())
                        QMessageBox::warning(this, QStringLiteral("导出失败"), error);
                }
            });
    watcher->setFuture(QtConcurrent::run(
        [samples, resultFile, stepId, outputFile, mask]()
        {
            std::vector<Hdf5EnergyHistorySample> queried;
            const auto* values = samples.get();
            if (!values)
            {
                Hdf5ResultReader reader;
                std::vector<Hdf5ResultFrameInfo> frames;
                if (!reader.openEnergyHistory(resultFile, frames))
                    return QStringLiteral("无法读取当前分析步的能量，请重新加载结果文件后再导出。");
                std::vector<Hdf5ResultFrameInfo>().swap(frames);
                if (!reader.readEnergyHistory(stepId, queried))
                    return QStringLiteral("无法读取当前分析步的能量，请重新加载结果文件后再导出。");
                reader.close();
                values = &queried;
            }
            unsigned int availableMask = 0;
            for (const auto& sample : *values)
                availableMask |= sample.energy.availableMask;
            if (values->empty() || (availableMask & mask) == 0)
                return QStringLiteral("当前分析步没有所选分量的有效能量记录。");
            if (!ResultExporter().exportEnergyBdf(outputFile, *values, mask))
                return QStringLiteral("无法写入能量 BDF 文件，请检查保存路径。");
            return QString();
        }));
}

void NodeTimeHistoryPanel::applyHistory(const LoadResult& result)
{
    if (result.energy && !result.energyStepIds.empty())
    {
        const QSignalBlocker blocker(m_energyStep);
        m_energyStep->clear();
        for (const int stepId : result.energyStepIds)
            m_energyStep->addItem(QStringLiteral("分析步 %1").arg(stepId), stepId);
        m_energyStep->setCurrentIndex(m_energyStep->findData(result.stepId));
    }
    if (result.energy)
    {
        updateEnergyAvailability(result.availableEnergyMask, true);
    }
    if (!result.emptyMessage.isEmpty())
    {
        m_chart->clearData(result.emptyMessage);
        setStatus(result.emptyMessage);
        return;
    }
    if (!result.error.isEmpty())
    {
        m_chart->clearData(result.error);
        setStatus(result.error, true);
        auto* message = new QMessageBox(QMessageBox::Information, QStringLiteral("无法绘制时程"), result.error,
                                       QMessageBox::Ok, this);
        message->setAttribute(Qt::WA_DeleteOnClose);
        message->open();
        return;
    }
    m_cachedFile = m_resultFile;
    m_cachedNodeId = result.nodeId;
    m_cachedStepId = result.stepId;
    m_cachedSamples = result.samples;
    m_cachedEnergySamples = result.energySamples;
    if (result.energy)
    {
        m_chart->setEnergyData(*m_cachedEnergySamples);
        m_chart->setComponents(selectedComponents());
        setStatus(energyStatus(result.stepId, m_cachedEnergySamples->size()));
        updateEnergyExportState();
        return;
    }
    m_chart->setData(result.nodeId, *m_cachedSamples);
    m_chart->setComponents(selectedComponents());
    setStatus(
        QStringLiteral("节点 %1 · 已直接读取 %2 帧 · 未生成任何文件。").arg(result.nodeId).arg(m_cachedSamples->size()));
}
