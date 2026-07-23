#include "YQY.h"

#include "Controllers/ModelController.h"
#include "Controllers/SolveTaskController.h"
#include "Widgets/ModelViewport.h"
#include "Widgets/NavigationButton.h"
#include "Widgets/SettingsPanel.h"
#include "Widgets/ResultControlPanel.h"
#include "Widgets/AnalysisSummaryPanel.h"
#include "Widgets/PropertyModule.h"
#include "Widgets/ConductorModule.h"
#include "Dialogs/AnalysisManagerDialog.h"
#include "Dialogs/ComputeRegionManagerDialog.h"
#include "Dialogs/SolveTaskManagerDialog.h"
#include "Dialogs/PropertyItemEditorDialog.h"
#include "Dialogs/PropertyLibraryDialog.h"
#include "Dialogs/NodeResultExportDialog.h"
#include "Dialogs/ModelImportFileDialog.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Section/SectionRectangle.h"
#include "Conductor/PropertyLibrary.h"

#include <cmath>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace
{
QString solveTaskDisplayStatus(const SolveTaskController::TaskInfo& info)
{
    if (info.partialResult)
    {
        return info.status == SolveTaskController::Status::Cancelled ? QStringLiteral("已停止·部分结果")
                                                                     : QStringLiteral("失败·部分结果");
    }
    return SolveTaskController::statusText(info.status);
}

bool modelsMatchForResults(
    const std::shared_ptr<StructureData>& current, const std::shared_ptr<StructureData>& embedded)
{
    if (!current || !embedded || current->m_Nodes.size() != embedded->m_Nodes.size()
        || current->m_Elements.size() != embedded->m_Elements.size())
    {
        return false;
    }

    constexpr double coordinateTolerance = 1.0e-10;
    for (const auto& [nodeId, embeddedNode] : embedded->m_Nodes)
    {
        const auto found = current->m_Nodes.find(nodeId);
        if (found == current->m_Nodes.end() || !found->second || !embeddedNode
            || std::abs(found->second->m_X - embeddedNode->m_X) > coordinateTolerance
            || std::abs(found->second->m_Y - embeddedNode->m_Y) > coordinateTolerance
            || std::abs(found->second->m_Z - embeddedNode->m_Z) > coordinateTolerance)
        {
            return false;
        }
    }

    for (const auto& [elementId, embeddedElement] : embedded->m_Elements)
    {
        const auto found = current->m_Elements.find(elementId);
        if (found == current->m_Elements.end() || !found->second || !embeddedElement
            || found->second->m_pNode.size() != embeddedElement->m_pNode.size())
        {
            return false;
        }
        for (int nodeIndex = 0; nodeIndex < embeddedElement->m_pNode.size(); ++nodeIndex)
        {
            const auto currentNode = found->second->m_pNode[nodeIndex].lock();
            const auto embeddedNode = embeddedElement->m_pNode[nodeIndex].lock();
            if (!currentNode || !embeddedNode || currentNode->m_Id != embeddedNode->m_Id)
                return false;
        }
    }
    return true;
}

Hdf5ResultFrame interpolateResultFrames(
    const Hdf5ResultFrame& first, const Hdf5ResultFrame& second, double interpolation)
{
    interpolation = std::clamp(interpolation, 0.0, 1.0);
    const double firstWeight = 1.0 - interpolation;
    Hdf5ResultFrame result = first;
    result.info.time = first.info.time * firstWeight + second.info.time * interpolation;
    result.info.loadFactor = first.info.loadFactor * firstWeight + second.info.loadFactor * interpolation;

    if (result.nodes.size() == second.nodes.size())
    {
        for (std::size_t index = 0; index < result.nodes.size(); ++index)
        {
            Hdf5NodalResult& target = result.nodes[index];
            const Hdf5NodalResult& next = second.nodes[index];
            if (target.id != next.id)
                continue;
            for (int component = 0; component < 3; ++component)
            {
                target.displacement[component] = target.displacement[component] * firstWeight
                    + next.displacement[component] * interpolation;
                target.currentCoordinate[component] = target.currentCoordinate[component] * firstWeight
                    + next.currentCoordinate[component] * interpolation;
            }
        }
    }

    if (result.elements.size() == second.elements.size())
    {
        for (std::size_t index = 0; index < result.elements.size(); ++index)
        {
            Hdf5ElementResult& target = result.elements[index];
            const Hdf5ElementResult& next = second.elements[index];
            if (target.id != next.id)
                continue;
            target.axialForce = target.axialForce * firstWeight + next.axialForce * interpolation;
            target.currentStress = target.currentStress * firstWeight + next.currentStress * interpolation;
            target.strain = target.strain * firstWeight + next.strain * interpolation;
        }
    }
    return result;
}

const Hdf5ResultRange& resultRangeForField(
    const Hdf5ResultRanges& ranges, ModelViewport::ResultField field)
{
    switch (field)
    {
    case ModelViewport::ResultField::DisplacementMagnitude: return ranges.displacementMagnitude;
    case ModelViewport::ResultField::DisplacementX: return ranges.displacementX;
    case ModelViewport::ResultField::DisplacementY: return ranges.displacementY;
    case ModelViewport::ResultField::DisplacementZ: return ranges.displacementZ;
    case ModelViewport::ResultField::AxialForce: return ranges.axialForce;
    case ModelViewport::ResultField::Stress: return ranges.stress;
    case ModelViewport::ResultField::Strain: return ranges.strain;
    }
    return ranges.displacementMagnitude;
}

void animateProgressBar(QProgressBar* progressBar, int targetValue)
{
    if (!progressBar)
        return;

    targetValue = qBound(progressBar->minimum(), targetValue, progressBar->maximum());
    progressBar->setProperty("smoothTargetValue", targetValue);

    // A solve may only produce a handful of visible progress events.  Keep one
    // lightweight 60 FPS timer alive per bar and let the displayed value catch
    // up with the latest value instead of restarting an animation on every event.
    auto* timer = progressBar->findChild<QTimer*>(QStringLiteral("smoothProgressTimer"), Qt::FindDirectChildrenOnly);
    if (!timer)
    {
        timer = new QTimer(progressBar);
        timer->setObjectName(QStringLiteral("smoothProgressTimer"));
        timer->setInterval(16);
        QObject::connect(timer, &QTimer::timeout, progressBar,
                         [progressBar, timer]()
                         {
                             const int target = progressBar->property("smoothTargetValue").toInt();
                             const int current = progressBar->value();
                             const int distance = target - current;
                             if (distance == 0)
                             {
                                 timer->stop();
                                 return;
                             }

                             const int step = qMax(1, qRound(qAbs(distance) * 0.38));
                             const int next =
                                 distance > 0 ? qMin(target, current + step) : qMax(target, current - step);
                             progressBar->setValue(next);
                         });
    }

    // A restarted calculation must return to zero immediately. Forward progress
    // is interpolated so even a very small model remains visually continuous.
    if (targetValue < progressBar->value())
        progressBar->setValue(targetValue);
    else if (targetValue != progressBar->value() && !timer->isActive())
        timer->start();
}

QString importFileDirectory()
{
    const QStringList candidates = {
        QDir::current().absoluteFilePath(QStringLiteral("YQY/Import/ImportFile")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../../YQY/Import/ImportFile")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../../Import/ImportFile")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../Import/ImportFile"))};
    for (const QString& candidate : candidates)
    {
        const QDir directory(candidate);
        if (directory.exists())
            return directory.absolutePath();
    }
    return QDir::currentPath();
}

QString hdf5ResultDirectory()
{
    const QStringList candidates = {
        QDir::current().absoluteFilePath(QStringLiteral("YQY/Export/ExportH5")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../../YQY/Export/ExportH5")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../../Export/ExportH5")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../Export/ExportH5"))};
    for (const QString& candidate : candidates)
    {
        const QDir directory(candidate);
        if (directory.exists())
            return directory.absolutePath();
    }
    return importFileDirectory();
}

enum class ToolbarGlyph
{
    Import,
    Undo,
    Model,
    Conductor,
    Analysis,
    Results,
    Settings,
    Select,
    Rotate,
    Pan,
    Zoom,
    Nodes,
    Elements,
    NodeIds,
    ElementIds,
    Front,
    Back,
    Left,
    Right,
    Top,
    Bottom,
    Fit,
    Solve,
    Stop
};

enum class TreeGlyph
{
    Project,
    Model,
    Nodes,
    Elements,
    Material,
    Section,
    ElementTypes,
    Truss,
    Cable,
    Beam,
    Other,
    AnalysisSteps,
    AnalysisStep,
    Load,
    Constraint,
    SolveTask,
    StopTask,
    ResultFile,
    ResultFrames,
    Displacement,
    Stress,
    Strain
};

enum class NavigationGlyph
{
    Model,
    Properties,
    Conductor,
    Analysis,
    Solve,
    Results,
    Settings
};

enum class ActionGlyph
{
    Import,
    Fit,
    OpenFile,
    Run,
    Stop,
    Library,
    CreateConductor,
    Refresh,
    Apply,
    Export
};

constexpr auto ActionGlyphProperty = "semanticActionGlyph";

constexpr int TreeGlyphRole = Qt::UserRole + 20;

class InstantToolTipFilter final : public QObject
{
public:
    explicit InstantToolTipFilter(QAbstractButton* button) : QObject(button), m_button(button) {}

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (watched != m_button)
            return QObject::eventFilter(watched, event);

        if ((event->type() == QEvent::Enter || event->type() == QEvent::HoverEnter ||
             event->type() == QEvent::HoverMove) &&
            !m_button->toolTip().isEmpty())
        {
            const QPoint position = m_button->mapToGlobal(QPoint(m_button->width() / 2, m_button->height() + 4));
            QToolTip::showText(position, m_button->toolTip(), m_button, m_button->rect(), 3500);
        }
        else if (event->type() == QEvent::Leave || event->type() == QEvent::MouseButtonPress)
        {
            QToolTip::hideText();
        }
        else if (event->type() == QEvent::ToolTip)
        {
            return true;
        }
        return QObject::eventFilter(watched, event);
    }

private:
    QAbstractButton* m_button = nullptr;
};

constexpr auto PopupViewStyleProperty = "yqyPopupViewStyle";
constexpr auto CaptionColorProperty = "yqyCaptionColor";
constexpr auto CaptionTextColorProperty = "yqyCaptionTextColor";
constexpr auto WindowBorderColorProperty = "yqyWindowBorderColor";

#ifdef Q_OS_WIN
void applyWindowsTitleBarTheme(QWidget* widget)
{
    if (!widget || !widget->isWindow() || widget->windowType() == Qt::Popup ||
        widget->windowType() == Qt::ToolTip)
        return;

    using DwmSetWindowAttributeFunction = HRESULT(WINAPI*)(HWND, DWORD, LPCVOID, DWORD);
    static const DwmSetWindowAttributeFunction setWindowAttribute = []()
    {
        const HMODULE dwmApi = LoadLibraryW(L"dwmapi.dll");
        return dwmApi ? reinterpret_cast<DwmSetWindowAttributeFunction>(
                            GetProcAddress(dwmApi, "DwmSetWindowAttribute"))
                      : nullptr;
    }();
    if (!setWindowAttribute)
        return;

    const QColor caption = qApp->property(CaptionColorProperty).value<QColor>();
    const QColor captionText = qApp->property(CaptionTextColorProperty).value<QColor>();
    const QColor border = qApp->property(WindowBorderColorProperty).value<QColor>();
    if (!caption.isValid() || !captionText.isValid() || !border.isValid())
        return;

    const HWND windowHandle = reinterpret_cast<HWND>(widget->winId());
    const BOOL darkMode = caption.lightness() < 128;
    constexpr DWORD DwmwaUseImmersiveDarkModeBefore20H1 = 19;
    constexpr DWORD DwmwaUseImmersiveDarkMode = 20;
    constexpr DWORD DwmwaBorderColor = 34;
    constexpr DWORD DwmwaCaptionColor = 35;
    constexpr DWORD DwmwaTextColor = 36;
    const DWORD captionColor = RGB(caption.red(), caption.green(), caption.blue());
    const DWORD captionTextColor = RGB(captionText.red(), captionText.green(), captionText.blue());
    const DWORD borderColor = RGB(border.red(), border.green(), border.blue());

    if (FAILED(setWindowAttribute(windowHandle, DwmwaUseImmersiveDarkMode, &darkMode, sizeof(darkMode))))
        setWindowAttribute(windowHandle, DwmwaUseImmersiveDarkModeBefore20H1, &darkMode, sizeof(darkMode));
    setWindowAttribute(windowHandle, DwmwaBorderColor, &borderColor, sizeof(borderColor));
    setWindowAttribute(windowHandle, DwmwaCaptionColor, &captionColor, sizeof(captionColor));
    setWindowAttribute(windowHandle, DwmwaTextColor, &captionTextColor, sizeof(captionTextColor));
}
#else
void applyWindowsTitleBarTheme(QWidget*)
{
}
#endif

class PopupThemeFilter final : public QObject
{
public:
    explicit PopupThemeFilter(QObject* parent) : QObject(parent) {}

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() != QEvent::Show)
            return QObject::eventFilter(watched, event);

        auto* widget = qobject_cast<QWidget*>(watched);
        if (widget && widget->isWindow())
        {
            QTimer::singleShot(0, widget, [widget]()
            {
                applyWindowsTitleBarTheme(widget);
            });
        }

        auto* view = qobject_cast<QAbstractItemView*>(watched);
        if (!view || !view->window()->inherits("QComboBoxPrivateContainer"))
            return QObject::eventFilter(watched, event);

        const QString popupStyle = qApp->property(PopupViewStyleProperty).toString();
        if (!popupStyle.isEmpty() && view->styleSheet() != popupStyle)
            view->setStyleSheet(popupStyle);

        QPalette popupPalette = qApp->palette();
        view->setPalette(popupPalette);
        view->window()->setPalette(popupPalette);
        view->window()->setAutoFillBackground(true);
        return QObject::eventFilter(watched, event);
    }
};

QPolygonF viewFace(ToolbarGlyph glyph)
{
    switch (glyph)
    {
    case ToolbarGlyph::Front:
        return {QPointF(5, 8), QPointF(15, 8), QPointF(15, 18), QPointF(5, 18)};
    case ToolbarGlyph::Back:
        return {QPointF(9, 4), QPointF(19, 4), QPointF(19, 14), QPointF(9, 14)};
    case ToolbarGlyph::Left:
        return {QPointF(5, 8), QPointF(9, 4), QPointF(9, 14), QPointF(5, 18)};
    case ToolbarGlyph::Right:
        return {QPointF(15, 8), QPointF(19, 4), QPointF(19, 14), QPointF(15, 18)};
    case ToolbarGlyph::Top:
        return {QPointF(5, 8), QPointF(9, 4), QPointF(19, 4), QPointF(15, 8)};
    case ToolbarGlyph::Bottom:
        return {QPointF(5, 18), QPointF(9, 14), QPointF(19, 14), QPointF(15, 18)};
    default:
        return {};
    }
}

QPixmap renderToolbarGlyph(ToolbarGlyph glyph, const QColor& color, const QColor& accent)
{
    QPixmap pixmap(48, 48);
    pixmap.fill(Qt::transparent);
    pixmap.setDevicePixelRatio(2.0);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.scale(2.0, 2.0);
    QPen pen(color, 1.7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    QColor accentFill = accent;
    accentFill.setAlpha(90);

    auto drawArrowHead = [&painter](const QPointF& tip, const QPointF& first, const QPointF& second)
    {
        painter.drawLine(tip, first);
        painter.drawLine(tip, second);
    };

    switch (glyph)
    {
    case ToolbarGlyph::Import:
        painter.drawLine(QPointF(12, 3), QPointF(12, 14));
        drawArrowHead(QPointF(12, 14), QPointF(8.5, 10.5), QPointF(15.5, 10.5));
        painter.drawLine(QPointF(4, 17), QPointF(4, 20));
        painter.drawLine(QPointF(4, 20), QPointF(20, 20));
        painter.drawLine(QPointF(20, 20), QPointF(20, 17));
        break;
    case ToolbarGlyph::Undo:
    {
        QPainterPath path;
        path.moveTo(5, 11);
        path.cubicTo(8, 5, 18.5, 6, 19.5, 14);
        path.cubicTo(20, 17, 18, 19.5, 15.5, 20.5);
        painter.drawPath(path);
        drawArrowHead(QPointF(5, 11), QPointF(5.2, 6), QPointF(10, 10));
        break;
    }
    case ToolbarGlyph::Model:
    {
        painter.drawRoundedRect(QRectF(4, 7, 13, 13), 1.2, 1.2);
        painter.setPen(QPen(accent, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawRoundedRect(QRectF(8, 3, 13, 13), 1.2, 1.2);
        painter.drawLine(QPointF(4, 7), QPointF(8, 3));
        painter.drawLine(QPointF(17, 7), QPointF(21, 3));
        painter.drawLine(QPointF(17, 20), QPointF(21, 16));
        break;
    }
    case ToolbarGlyph::Conductor:
    {
        painter.drawLine(QPointF(4, 4), QPointF(4, 21));
        painter.drawLine(QPointF(20, 4), QPointF(20, 21));
        painter.drawLine(QPointF(1.5, 7), QPointF(6.5, 7));
        painter.drawLine(QPointF(17.5, 7), QPointF(22.5, 7));
        painter.setPen(QPen(accent, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        QPainterPath cable;
        cable.moveTo(4, 7);
        cable.cubicTo(8, 18, 16, 18, 20, 7);
        painter.drawPath(cable);
        break;
    }
    case ToolbarGlyph::Analysis:
        painter.drawLine(QPointF(4, 3), QPointF(4, 20));
        painter.drawLine(QPointF(4, 20), QPointF(21, 20));
        painter.setPen(QPen(accent, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawPolyline(QPolygonF{QPointF(6, 16), QPointF(10, 11), QPointF(14, 14), QPointF(20, 6)});
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(10, 11), 1.5, 1.5);
        painter.drawEllipse(QPointF(20, 6), 1.5, 1.5);
        break;
    case ToolbarGlyph::Results:
        painter.drawLine(QPointF(3, 21), QPointF(21, 21));
        painter.setPen(QPen(accent, 1.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.setBrush(accent);
        painter.drawRoundedRect(QRectF(5, 12, 3.5, 8), 1, 1);
        painter.drawRoundedRect(QRectF(10.3, 7, 3.5, 13), 1, 1);
        painter.drawRoundedRect(QRectF(15.7, 3, 3.5, 17), 1, 1);
        break;
    case ToolbarGlyph::Settings:
    {
        painter.setBrush(accentFill);
        painter.drawEllipse(QRectF(6, 6, 12, 12));
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(QRectF(9.2, 9.2, 5.6, 5.6));
        for (int angle = 0; angle < 360; angle += 45)
        {
            painter.save();
            painter.translate(12, 12);
            painter.rotate(angle);
            painter.drawLine(QPointF(0, -9.5), QPointF(0, -6));
            painter.restore();
        }
        break;
    }
    case ToolbarGlyph::Select:
    {
        QPainterPath cursor;
        cursor.moveTo(5, 3);
        cursor.lineTo(18.5, 13);
        cursor.lineTo(12.5, 14.2);
        cursor.lineTo(16.2, 20.2);
        cursor.lineTo(13.1, 22);
        cursor.lineTo(9.6, 15.7);
        cursor.lineTo(5, 19.5);
        cursor.closeSubpath();
        painter.setBrush(color);
        painter.drawPath(cursor);
        break;
    }
    case ToolbarGlyph::Rotate:
        painter.drawArc(QRectF(4, 4, 16, 16), 35 * 16, 285 * 16);
        drawArrowHead(QPointF(18.8, 5.4), QPointF(14.2, 5), QPointF(18, 9.4));
        break;
    case ToolbarGlyph::Pan:
        painter.drawLine(QPointF(12, 3), QPointF(12, 21));
        painter.drawLine(QPointF(3, 12), QPointF(21, 12));
        drawArrowHead(QPointF(12, 3), QPointF(9.3, 6), QPointF(14.7, 6));
        drawArrowHead(QPointF(12, 21), QPointF(9.3, 18), QPointF(14.7, 18));
        drawArrowHead(QPointF(3, 12), QPointF(6, 9.3), QPointF(6, 14.7));
        drawArrowHead(QPointF(21, 12), QPointF(18, 9.3), QPointF(18, 14.7));
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(12, 12), 1.7, 1.7);
        break;
    case ToolbarGlyph::Zoom:
        painter.drawEllipse(QRectF(3.5, 3.5, 12.5, 12.5));
        painter.drawLine(QPointF(14.2, 14.2), QPointF(21, 21));
        painter.drawLine(QPointF(7, 9.75), QPointF(12.5, 9.75));
        painter.drawLine(QPointF(9.75, 7), QPointF(9.75, 12.5));
        break;
    case ToolbarGlyph::Nodes:
        painter.drawLine(QPointF(5, 18), QPointF(12, 5));
        painter.drawLine(QPointF(12, 5), QPointF(20, 17));
        painter.drawLine(QPointF(5, 18), QPointF(20, 17));
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(5, 18), 2.2, 2.2);
        painter.drawEllipse(QPointF(12, 5), 2.2, 2.2);
        painter.drawEllipse(QPointF(20, 17), 2.2, 2.2);
        break;
    case ToolbarGlyph::Elements:
        painter.drawPolygon(QPolygonF{QPointF(3.5, 19.5), QPointF(12, 3.5), QPointF(20.5, 19.5)});
        painter.drawLine(QPointF(3.5, 19.5), QPointF(12, 12));
        painter.drawLine(QPointF(12, 3.5), QPointF(12, 12));
        painter.drawLine(QPointF(20.5, 19.5), QPointF(12, 12));
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(12, 12), 1.7, 1.7);
        break;
    case ToolbarGlyph::NodeIds:
        painter.drawLine(QPointF(4, 18), QPointF(10, 11));
        painter.drawLine(QPointF(10, 11), QPointF(17, 18));
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(10, 11), 2.4, 2.4);
        painter.setBrush(Qt::NoBrush);
        painter.drawRoundedRect(QRectF(12.5, 3, 9, 6), 1.5, 1.5);
        painter.drawLine(QPointF(14.5, 5), QPointF(19.5, 5));
        painter.drawLine(QPointF(14.5, 7), QPointF(18, 7));
        break;
    case ToolbarGlyph::ElementIds:
        painter.drawPolygon(QPolygonF{QPointF(3.5, 19.5), QPointF(11, 5), QPointF(18.5, 19.5)});
        painter.drawRoundedRect(QRectF(13, 3, 8.5, 6.5), 1.5, 1.5);
        painter.drawLine(QPointF(15, 5.2), QPointF(19.5, 5.2));
        painter.drawLine(QPointF(15, 7.3), QPointF(18.2, 7.3));
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(11, 12), 1.6, 1.6);
        break;
    case ToolbarGlyph::Front:
    case ToolbarGlyph::Back:
    case ToolbarGlyph::Left:
    case ToolbarGlyph::Right:
    case ToolbarGlyph::Top:
    case ToolbarGlyph::Bottom:
    {
        QColor faceColor = accent;
        faceColor.setAlpha(90);
        painter.setBrush(faceColor);
        painter.drawPolygon(viewFace(glyph));
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(QRectF(5, 8, 10, 10));
        painter.drawRect(QRectF(9, 4, 10, 10));
        painter.drawLine(QPointF(5, 8), QPointF(9, 4));
        painter.drawLine(QPointF(15, 8), QPointF(19, 4));
        painter.drawLine(QPointF(15, 18), QPointF(19, 14));
        painter.drawLine(QPointF(5, 18), QPointF(9, 14));
        painter.setPen(QPen(accent, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        switch (glyph)
        {
        case ToolbarGlyph::Front:
            painter.drawLine(QPointF(14, 10), QPointF(7, 17));
            drawArrowHead(QPointF(7, 17), QPointF(7.5, 12.8), QPointF(11.2, 16.5));
            break;
        case ToolbarGlyph::Back:
            painter.drawLine(QPointF(10, 14), QPointF(17, 7));
            drawArrowHead(QPointF(17, 7), QPointF(16.5, 11.2), QPointF(12.8, 7.5));
            break;
        case ToolbarGlyph::Left:
            painter.drawLine(QPointF(14, 12), QPointF(3, 12));
            drawArrowHead(QPointF(3, 12), QPointF(7, 8.8), QPointF(7, 15.2));
            break;
        case ToolbarGlyph::Right:
            painter.drawLine(QPointF(10, 12), QPointF(21, 12));
            drawArrowHead(QPointF(21, 12), QPointF(17, 8.8), QPointF(17, 15.2));
            break;
        case ToolbarGlyph::Top:
            painter.drawLine(QPointF(12, 14), QPointF(12, 2));
            drawArrowHead(QPointF(12, 2), QPointF(8.8, 6), QPointF(15.2, 6));
            break;
        case ToolbarGlyph::Bottom:
            painter.drawLine(QPointF(12, 10), QPointF(12, 22));
            drawArrowHead(QPointF(12, 22), QPointF(8.8, 18), QPointF(15.2, 18));
            break;
        default:
            break;
        }
        break;
    }
    case ToolbarGlyph::Fit:
        painter.drawLine(QPointF(4, 9), QPointF(4, 4));
        painter.drawLine(QPointF(4, 4), QPointF(9, 4));
        painter.drawLine(QPointF(15, 4), QPointF(20, 4));
        painter.drawLine(QPointF(20, 4), QPointF(20, 9));
        painter.drawLine(QPointF(20, 15), QPointF(20, 20));
        painter.drawLine(QPointF(20, 20), QPointF(15, 20));
        painter.drawLine(QPointF(9, 20), QPointF(4, 20));
        painter.drawLine(QPointF(4, 20), QPointF(4, 15));
        painter.setBrush(accent);
        painter.drawRoundedRect(QRectF(8, 8, 8, 8), 1.5, 1.5);
        break;
    case ToolbarGlyph::Solve:
        painter.setBrush(accent);
        painter.drawPolygon(QPolygonF{QPointF(7, 4), QPointF(20, 12), QPointF(7, 20)});
        break;
    case ToolbarGlyph::Stop:
        painter.setBrush(accent);
        painter.drawRoundedRect(QRectF(5, 5, 14, 14), 2, 2);
        break;
    }
    return pixmap;
}

QIcon toolbarIcon(ToolbarGlyph glyph, const QColor& color, const QColor& accent)
{
    QIcon icon;
    icon.addPixmap(renderToolbarGlyph(glyph, color, accent), QIcon::Normal, QIcon::Off);
    icon.addPixmap(renderToolbarGlyph(glyph, accent, accent), QIcon::Normal, QIcon::On);
    return icon;
}

QPixmap renderNavigationGlyph(NavigationGlyph glyph, const QColor& color, const QColor& accent)
{
    // Keep this as a plain high-resolution pixmap. QIcon performs the final
    // DPI scaling, avoiding the double-DPR clipping that affected the rail.
    QPixmap pixmap(64, 64);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.scale(2.0, 2.0);
    painter.setPen(QPen(color, 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.setBrush(Qt::NoBrush);

    switch (glyph)
    {
    case NavigationGlyph::Model:
        painter.drawPolygon(QPolygonF{QPointF(16, 3), QPointF(27, 9), QPointF(16, 15), QPointF(5, 9)});
        painter.drawPolyline(
            QPolygonF{QPointF(5, 9), QPointF(5, 22), QPointF(16, 29), QPointF(27, 22), QPointF(27, 9)});
        painter.drawLine(QPointF(16, 15), QPointF(16, 29));
        painter.setPen(QPen(accent, 2.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(QPointF(5, 9), QPointF(16, 15));
        painter.drawLine(QPointF(27, 9), QPointF(16, 15));
        break;
    case NavigationGlyph::Properties:
        painter.drawRoundedRect(QRectF(5, 4, 22, 24), 2.5, 2.5);
        for (int y : {10, 16, 22})
            painter.drawLine(QPointF(9, y), QPointF(23, y));
        painter.setPen(QPen(accent, 2.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawEllipse(QPointF(13, 10), 2.1, 2.1);
        painter.drawEllipse(QPointF(20, 16), 2.1, 2.1);
        painter.drawEllipse(QPointF(11, 22), 2.1, 2.1);
        break;
    case NavigationGlyph::Conductor:
    {
        painter.drawLine(QPointF(6, 5), QPointF(6, 27));
        painter.drawLine(QPointF(26, 5), QPointF(26, 27));
        painter.drawLine(QPointF(2, 9), QPointF(10, 9));
        painter.drawLine(QPointF(22, 9), QPointF(30, 9));
        painter.setPen(QPen(accent, 2.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        QPainterPath wire;
        wire.moveTo(6, 9);
        wire.cubicTo(10, 25, 22, 25, 26, 9);
        painter.drawPath(wire);
        break;
    }
    case NavigationGlyph::Analysis:
    {
        painter.drawLine(QPointF(4, 5), QPointF(4, 27));
        painter.drawLine(QPointF(4, 27), QPointF(28, 27));
        painter.setPen(QPen(accent, 2.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        QPainterPath curve;
        curve.moveTo(6, 22);
        curve.cubicTo(10, 22, 10, 10, 15, 14);
        curve.cubicTo(20, 18, 21, 7, 27, 7);
        painter.drawPath(curve);
        break;
    }
    case NavigationGlyph::Solve:
        painter.drawEllipse(QRectF(3.5, 3.5, 25, 25));
        painter.setPen(QPen(accent, 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.setBrush(accent);
        painter.drawPolygon(QPolygonF{QPointF(12, 9), QPointF(24, 16), QPointF(12, 23)});
        break;
    case NavigationGlyph::Results:
        painter.drawLine(QPointF(4, 28), QPointF(29, 28));
        painter.setPen(QPen(accent, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.setBrush(accent);
        painter.drawRoundedRect(QRectF(7, 18, 4, 9), 1, 1);
        painter.drawRoundedRect(QRectF(14, 11, 4, 16), 1, 1);
        painter.drawRoundedRect(QRectF(21, 5, 4, 22), 1, 1);
        break;
    case NavigationGlyph::Settings:
        painter.drawEllipse(QRectF(8, 8, 16, 16));
        painter.drawEllipse(QRectF(13, 13, 6, 6));
        for (int angle = 0; angle < 360; angle += 45)
        {
            painter.save();
            painter.translate(16, 16);
            painter.rotate(angle);
            painter.drawLine(QPointF(0, -14), QPointF(0, -9));
            painter.restore();
        }
        painter.setPen(QPen(accent, 2.2));
        painter.drawEllipse(QRectF(13, 13, 6, 6));
        break;
    }
    return pixmap;
}

QIcon navigationIcon(NavigationGlyph glyph, const QColor& color, const QColor& accent)
{
    QIcon icon;
    icon.addPixmap(renderNavigationGlyph(glyph, color, accent), QIcon::Normal, QIcon::Off);
    icon.addPixmap(renderNavigationGlyph(glyph, accent, accent), QIcon::Normal, QIcon::On);
    return icon;
}

QPixmap renderActionGlyph(ActionGlyph glyph, const QColor& color, const QColor& accent)
{
    QPixmap pixmap(64, 64);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.scale(2.0, 2.0);
    painter.setPen(QPen(color, 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.setBrush(Qt::NoBrush);

    auto arrowHead = [&painter](const QPointF& tip, const QPointF& a, const QPointF& b)
    {
        painter.drawLine(tip, a);
        painter.drawLine(tip, b);
    };

    switch (glyph)
    {
    case ActionGlyph::Import:
        painter.drawLine(QPointF(16, 4), QPointF(16, 21));
        arrowHead(QPointF(16, 21), QPointF(10, 15), QPointF(22, 15));
        painter.setPen(QPen(accent, 2.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawPolyline(QPolygonF{QPointF(5, 23), QPointF(5, 28), QPointF(27, 28), QPointF(27, 23)});
        break;
    case ActionGlyph::Fit:
        painter.drawPolyline(QPolygonF{QPointF(12, 5), QPointF(5, 5), QPointF(5, 12)});
        painter.drawPolyline(QPolygonF{QPointF(20, 5), QPointF(27, 5), QPointF(27, 12)});
        painter.drawPolyline(QPolygonF{QPointF(27, 20), QPointF(27, 27), QPointF(20, 27)});
        painter.drawPolyline(QPolygonF{QPointF(12, 27), QPointF(5, 27), QPointF(5, 20)});
        painter.setPen(QPen(accent, 2.0));
        painter.drawRoundedRect(QRectF(11, 11, 10, 10), 1.5, 1.5);
        break;
    case ActionGlyph::OpenFile:
        painter.drawRoundedRect(QRectF(3, 8, 26, 19), 2, 2);
        painter.drawPolyline(QPolygonF{QPointF(3, 11), QPointF(3, 5), QPointF(13, 5), QPointF(17, 9)});
        painter.setPen(QPen(accent, 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(QPointF(16, 13), QPointF(16, 23));
        arrowHead(QPointF(16, 23), QPointF(12, 19), QPointF(20, 19));
        break;
    case ActionGlyph::Run:
        painter.drawEllipse(QRectF(3, 3, 26, 26));
        painter.setBrush(accent);
        painter.setPen(QPen(accent, 2));
        painter.drawPolygon(QPolygonF{QPointF(12, 9), QPointF(24, 16), QPointF(12, 23)});
        break;
    case ActionGlyph::Stop:
        painter.drawEllipse(QRectF(3, 3, 26, 26));
        painter.setBrush(accent);
        painter.setPen(QPen(accent, 2));
        painter.drawRoundedRect(QRectF(10, 10, 12, 12), 1.5, 1.5);
        break;
    case ActionGlyph::Library:
        for (int y : {5, 14, 23})
        {
            painter.drawEllipse(QRectF(5, y, 22, 6));
            if (y < 23)
                painter.drawLine(QPointF(5, y + 3), QPointF(5, y + 12));
        }
        painter.drawLine(QPointF(27, 8), QPointF(27, 26));
        painter.setPen(QPen(accent, 2));
        painter.drawLine(QPointF(10, 17), QPointF(22, 17));
        break;
    case ActionGlyph::CreateConductor:
    {
        painter.drawLine(QPointF(4, 5), QPointF(4, 27));
        painter.drawLine(QPointF(24, 5), QPointF(24, 27));
        QPainterPath wire;
        wire.moveTo(4, 9);
        wire.cubicTo(8, 24, 20, 24, 24, 9);
        painter.drawPath(wire);
        painter.setPen(QPen(accent, 2.4, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(27, 18), QPointF(27, 28));
        painter.drawLine(QPointF(22, 23), QPointF(32, 23));
        break;
    }
    case ActionGlyph::Refresh:
        painter.drawArc(QRectF(5, 5, 22, 22), 35 * 16, 285 * 16);
        arrowHead(QPointF(25, 7), QPointF(19, 7), QPointF(24, 13));
        painter.setPen(QPen(accent, 2));
        painter.drawEllipse(QPointF(16, 16), 2, 2);
        break;
    case ActionGlyph::Apply:
        painter.drawRoundedRect(QRectF(3, 3, 26, 26), 3, 3);
        painter.setPen(QPen(accent, 3.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawPolyline(QPolygonF{QPointF(8, 16), QPointF(14, 22), QPointF(25, 10)});
        break;
    case ActionGlyph::Export:
        painter.drawRoundedRect(QRectF(4, 4, 24, 24), 3, 3);
        painter.drawLine(QPointF(16, 20), QPointF(16, 8));
        arrowHead(QPointF(16, 8), QPointF(11, 13), QPointF(21, 13));
        painter.setPen(QPen(accent, 2.4, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(9, 24), QPointF(23, 24));
        break;
    }
    return pixmap;
}

QIcon actionIcon(ActionGlyph glyph, const QColor& color, const QColor& accent)
{
    return QIcon(renderActionGlyph(glyph, color, accent));
}

void setActionGlyph(QAbstractButton* button, ActionGlyph glyph)
{
    if (button)
        button->setProperty(ActionGlyphProperty, static_cast<int>(glyph));
}

void applyActionIcons(QWidget* root, const QColor& color, const QColor& accent)
{
    if (!root)
        return;
    const auto buttons = root->findChildren<QAbstractButton*>();
    for (QAbstractButton* button : buttons)
    {
        const QVariant value = button->property(ActionGlyphProperty);
        if (value.isValid())
            button->setIcon(actionIcon(static_cast<ActionGlyph>(value.toInt()), color, accent));
    }
}

QIcon treeIcon(TreeGlyph glyph, const QColor& color, const QColor& accent)
{
    // Render at 64 px without a device-pixel-ratio flag. The tree requests a
    // 20 px icon and Qt downsamples this cleanly on every display scale.
    QPixmap pixmap(64, 64);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.scale(4.0, 4.0);
    painter.setPen(QPen(color, 1.25, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.setBrush(Qt::NoBrush);

    QColor accentFill = accent;
    accentFill.setAlpha(95);

    switch (glyph)
    {
    case TreeGlyph::Project:
        painter.drawRoundedRect(QRectF(3, 2, 10, 12), 1.2, 1.2);
        painter.drawLine(QPointF(9, 2), QPointF(13, 6));
        painter.drawLine(QPointF(9, 2), QPointF(9, 6));
        painter.drawLine(QPointF(9, 6), QPointF(13, 6));
        painter.setPen(QPen(accent, 1.25));
        painter.drawLine(QPointF(5, 9), QPointF(11, 9));
        painter.drawLine(QPointF(5, 11.5), QPointF(10, 11.5));
        break;
    case TreeGlyph::Model:
        painter.setBrush(accentFill);
        painter.drawPolygon(QPolygonF{QPointF(8, 1.5), QPointF(14, 5), QPointF(8, 8.5), QPointF(2, 5)});
        painter.setBrush(Qt::NoBrush);
        painter.drawLine(QPointF(2, 5), QPointF(2, 11));
        painter.drawLine(QPointF(14, 5), QPointF(14, 11));
        painter.drawLine(QPointF(8, 8.5), QPointF(8, 14.5));
        painter.drawLine(QPointF(2, 11), QPointF(8, 14.5));
        painter.drawLine(QPointF(14, 11), QPointF(8, 14.5));
        break;
    case TreeGlyph::Nodes:
        painter.drawLine(QPointF(3, 12.5), QPointF(8, 3));
        painter.drawLine(QPointF(8, 3), QPointF(13, 12.5));
        painter.drawLine(QPointF(3, 12.5), QPointF(13, 12.5));
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(3, 12.5), 1.6, 1.6);
        painter.drawEllipse(QPointF(8, 3), 1.6, 1.6);
        painter.drawEllipse(QPointF(13, 12.5), 1.6, 1.6);
        break;
    case TreeGlyph::Elements:
    case TreeGlyph::Truss:
        painter.setBrush(accentFill);
        painter.drawPolygon(QPolygonF{QPointF(2, 13), QPointF(8, 2), QPointF(14, 13)});
        painter.setBrush(Qt::NoBrush);
        painter.drawLine(QPointF(2, 13), QPointF(8, 8));
        painter.drawLine(QPointF(8, 2), QPointF(8, 8));
        painter.drawLine(QPointF(14, 13), QPointF(8, 8));
        break;
    case TreeGlyph::Material:
        painter.setBrush(accentFill);
        painter.drawEllipse(QRectF(3, 2, 10, 4));
        painter.setBrush(Qt::NoBrush);
        painter.drawLine(QPointF(3, 4), QPointF(3, 12));
        painter.drawLine(QPointF(13, 4), QPointF(13, 12));
        painter.drawArc(QRectF(3, 10, 10, 4), 180 * 16, 180 * 16);
        painter.drawArc(QRectF(3, 6, 10, 4), 180 * 16, 180 * 16);
        break;
    case TreeGlyph::Section:
    case TreeGlyph::Beam:
        painter.setBrush(accentFill);
        painter.drawRoundedRect(QRectF(2, 2, 12, 3), 0.8, 0.8);
        painter.drawRect(QRectF(6.5, 5, 3, 6));
        painter.drawRoundedRect(QRectF(2, 11, 12, 3), 0.8, 0.8);
        break;
    case TreeGlyph::ElementTypes:
        painter.drawPolygon(QPolygonF{QPointF(2, 5), QPointF(4, 2), QPointF(6, 5)});
        painter.drawArc(QRectF(2, 6, 4, 4), 0, 180 * 16);
        painter.drawLine(QPointF(2, 13), QPointF(6, 13));
        painter.setPen(QPen(accent, 1.25));
        painter.drawLine(QPointF(8, 3), QPointF(14, 3));
        painter.drawLine(QPointF(8, 8), QPointF(14, 8));
        painter.drawLine(QPointF(8, 13), QPointF(14, 13));
        break;
    case TreeGlyph::Cable:
        painter.drawLine(QPointF(2, 3), QPointF(2, 6));
        painter.drawLine(QPointF(14, 3), QPointF(14, 6));
        painter.drawArc(QRectF(2, 1, 12, 11), 180 * 16, 180 * 16);
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(2, 5.5), 1.2, 1.2);
        painter.drawEllipse(QPointF(14, 5.5), 1.2, 1.2);
        break;
    case TreeGlyph::Other:
        painter.setBrush(accentFill);
        painter.drawPolygon(QPolygonF{QPointF(8, 1.5), QPointF(13.5, 4.8), QPointF(13.5, 11.2), QPointF(8, 14.5),
                                      QPointF(2.5, 11.2), QPointF(2.5, 4.8)});
        break;
    case TreeGlyph::AnalysisSteps:
        painter.drawLine(QPointF(3, 8), QPointF(13, 8));
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(3, 8), 1.6, 1.6);
        painter.drawEllipse(QPointF(8, 8), 1.6, 1.6);
        painter.drawEllipse(QPointF(13, 8), 1.6, 1.6);
        painter.drawLine(QPointF(11, 5.5), QPointF(13.5, 8));
        painter.drawLine(QPointF(13.5, 8), QPointF(11, 10.5));
        break;
    case TreeGlyph::AnalysisStep:
        painter.drawPolyline(
            QPolygonF{QPointF(2, 12), QPointF(5, 12), QPointF(5, 8), QPointF(9, 8), QPointF(9, 4), QPointF(14, 4)});
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(5, 12), 1.2, 1.2);
        painter.drawEllipse(QPointF(9, 8), 1.2, 1.2);
        painter.drawEllipse(QPointF(14, 4), 1.2, 1.2);
        break;
    case TreeGlyph::Load:
        painter.drawLine(QPointF(8, 1.5), QPointF(8, 10));
        painter.drawLine(QPointF(5, 7), QPointF(8, 10));
        painter.drawLine(QPointF(11, 7), QPointF(8, 10));
        painter.setPen(QPen(accent, 1.5));
        painter.drawLine(QPointF(2, 13), QPointF(14, 13));
        break;
    case TreeGlyph::Constraint:
        painter.setBrush(accentFill);
        painter.drawPolygon(QPolygonF{QPointF(8, 3), QPointF(3, 11), QPointF(13, 11)});
        painter.setBrush(Qt::NoBrush);
        painter.drawLine(QPointF(2, 12.5), QPointF(14, 12.5));
        painter.drawLine(QPointF(4, 12.5), QPointF(2.5, 15));
        painter.drawLine(QPointF(8, 12.5), QPointF(6.5, 15));
        painter.drawLine(QPointF(12, 12.5), QPointF(10.5, 15));
        break;
    case TreeGlyph::SolveTask:
        painter.drawEllipse(QRectF(1.5, 1.5, 13, 13));
        painter.setBrush(accent);
        painter.drawPolygon(QPolygonF{QPointF(6, 4.5), QPointF(12, 8), QPointF(6, 11.5)});
        break;
    case TreeGlyph::StopTask:
        painter.drawEllipse(QRectF(1.5, 1.5, 13, 13));
        painter.setBrush(accent);
        painter.drawRoundedRect(QRectF(5.2, 5.2, 5.6, 5.6), 0.8, 0.8);
        break;
    case TreeGlyph::ResultFile:
        painter.drawRoundedRect(QRectF(2, 1.5, 12, 13), 1.2, 1.2);
        painter.setPen(QPen(accent, 1.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawPolyline(QPolygonF{QPointF(4, 11), QPointF(7, 8), QPointF(9, 10), QPointF(12, 5)});
        break;
    case TreeGlyph::ResultFrames:
        painter.drawRoundedRect(QRectF(1.5, 3, 11, 10), 1, 1);
        painter.drawRoundedRect(QRectF(4, 1, 10.5, 10), 1, 1);
        painter.setPen(QPen(accent, 1.3));
        painter.drawLine(QPointF(6, 4), QPointF(12, 4));
        painter.drawLine(QPointF(6, 7), QPointF(11, 7));
        break;
    case TreeGlyph::Displacement:
        painter.drawLine(QPointF(2, 12), QPointF(13, 3));
        painter.drawLine(QPointF(13, 3), QPointF(9, 3.5));
        painter.drawLine(QPointF(13, 3), QPointF(12.5, 7));
        painter.setPen(QPen(accent, 1.4));
        painter.drawLine(QPointF(3, 13.5), QPointF(10, 13.5));
        break;
    case TreeGlyph::Stress:
        painter.drawEllipse(QRectF(2, 2, 12, 12));
        painter.drawArc(QRectF(4, 4, 8, 8), 20 * 16, 140 * 16);
        painter.setPen(QPen(accent, 1.5, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(8, 8), QPointF(12, 5));
        break;
    case TreeGlyph::Strain:
        painter.drawRect(QRectF(2, 3, 9, 10));
        painter.setPen(QPen(accent, 1.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawPolygon(QPolygonF{QPointF(6, 1.5), QPointF(14, 4), QPointF(11, 14), QPointF(3, 11.5)});
        break;
    }

    return QIcon(pixmap);
}

void setTreeGlyph(QTreeWidgetItem* item, TreeGlyph glyph)
{
    if (item)
        item->setData(0, TreeGlyphRole, static_cast<int>(glyph));
}

void applyTreeIcons(QTreeWidget* tree, const QColor& color, const QColor& accent)
{
    if (!tree)
        return;

    QList<QTreeWidgetItem*> pending;
    for (int index = 0; index < tree->topLevelItemCount(); ++index)
        pending.append(tree->topLevelItem(index));

    while (!pending.isEmpty())
    {
        QTreeWidgetItem* item = pending.takeLast();
        const QVariant glyphValue = item->data(0, TreeGlyphRole);
        if (glyphValue.isValid())
            item->setIcon(0, treeIcon(static_cast<TreeGlyph>(glyphValue.toInt()), color, accent));
        for (int index = 0; index < item->childCount(); ++index)
            pending.append(item->child(index));
    }
}

struct ThemeColors
{
    QString rootBackground;
    QString header;
    QString navigation;
    QString panel;
    QString elevated;
    QString field;
    QString border;
    QString borderStrong;
    QString text;
    QString muted;
    QString accent;
    QString accentSecond;
    QString accentSoft;
    QString success;
    QString selectionText;
};

ThemeColors themeColors(int themeIndex)
{
    if (themeIndex == 3)
    {
        // Keep this palette aligned with D:\VS\THOP\FEM\FEM\FEMStyleBase.cpp.
        return {QStringLiteral("#F4F7FB"), QStringLiteral("#FFFFFF"), QStringLiteral("#F7FAFF"),
                QStringLiteral("#FFFFFF"), QStringLiteral("#F8FBFF"), QStringLiteral("#F8FBFF"),
                QStringLiteral("#D9E3EF"), QStringLiteral("#D9E3EF"), QStringLiteral("#1D2B3A"),
                QStringLiteral("#66758A"), QStringLiteral("#2F80ED"), QStringLiteral("#1E63C7"),
                QStringLiteral("#D8E9FF"), QStringLiteral("#2F9E68"), QStringLiteral("#FFFFFF")};
    }
    if (themeIndex == 1)
    {
        return {QStringLiteral("#0b0712"), QStringLiteral("#1d112e"), QStringLiteral("#140c20"),
                QStringLiteral("#1b102b"), QStringLiteral("#241638"), QStringLiteral("#180f26"),
                QStringLiteral("#3f285c"), QStringLiteral("#684681"), QStringLiteral("#f5edff"),
                QStringLiteral("#a995c4"), QStringLiteral("#8065ff"), QStringLiteral("#f05bff"),
                QStringLiteral("#38204e"), QStringLiteral("#59f0ce"), QStringLiteral("#ffffff")};
    }
    if (themeIndex == 2)
    {
        return {QStringLiteral("qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #0a2027,stop:0.52 #102f35,stop:1 #17313a)"),
                QStringLiteral("rgba(27,55,62,225)"),
                QStringLiteral("rgba(14,39,46,235)"),
                QStringLiteral("rgba(25,54,61,225)"),
                QStringLiteral("rgba(35,68,75,220)"),
                QStringLiteral("rgba(18,47,54,220)"),
                QStringLiteral("#55777f"),
                QStringLiteral("#78989f"),
                QStringLiteral("#efffff"),
                QStringLiteral("#9abdc3"),
                QStringLiteral("#55b7ff"),
                QStringLiteral("#55e0c2"),
                QStringLiteral("#294e55"),
                QStringLiteral("#6df1b7"),
                QStringLiteral("#07191d")};
    }
    return {QStringLiteral("#0b1016"), QStringLiteral("#111923"), QStringLiteral("#0f151d"), QStringLiteral("#121923"),
            QStringLiteral("#151d27"), QStringLiteral("#101720"), QStringLiteral("#263342"), QStringLiteral("#35485d"),
            QStringLiteral("#dce7f4"), QStringLiteral("#8190a2"), QStringLiteral("#4f8cff"), QStringLiteral("#30c7d9"),
            QStringLiteral("#1a2b39"), QStringLiteral("#3ddc97"), QStringLiteral("#071019")};
}

QString buildStyleSheet(const ThemeColors& c)
{
    QString style = QStringLiteral(R"QSS(
* { font-family: "Microsoft YaHei UI", "Segoe UI"; font-size: 13px; color: $TEXT; outline: none; }
QMainWindow, #centralWidget { background: $ROOT; }
QDialog, QMessageBox, QFileDialog { background: $PANEL; color: $TEXT; }
QDialog QWidget, QMessageBox QWidget, QFileDialog QWidget { color: $TEXT; }
QDialogButtonBox { background: transparent; border: none; }
QMessageBox QLabel,
QMessageBox QLabel#qt_msgbox_label,
QMessageBox QLabel#qt_msgbox_informativelabel { background: transparent; color: $TEXT; }
QMessageBox QPushButton { min-width: 76px; min-height: 26px; }
QDialog#analysisStepManagerDialog,
QDialog#analysisLoadManagerDialog,
QDialog#analysisConstraintManagerDialog,
QDialog#solveTaskManagerDialog,
QDialog#analysisStepEditorDialog,
QDialog#analysisLoadEditorDialog,
QDialog#analysisConstraintEditorDialog,
QDialog#nodeResultExportDialog {
    background: $PANEL;
    color: $TEXT;
}
QDialog#analysisStepManagerDialog QWidget#analysisManagerPage,
QDialog#analysisLoadManagerDialog QWidget#analysisManagerPage,
QDialog#analysisConstraintManagerDialog QWidget#analysisManagerPage {
    background: $FIELD;
    color: $TEXT;
}
QDialog#analysisStepManagerDialog QDialogButtonBox#analysisManagerButtonBox,
QDialog#analysisLoadManagerDialog QDialogButtonBox#analysisManagerButtonBox,
QDialog#analysisConstraintManagerDialog QDialogButtonBox#analysisManagerButtonBox {
    background: transparent;
    border: none;
}
QDialog#analysisStepEditorDialog QLabel,
QDialog#analysisLoadEditorDialog QLabel,
QDialog#analysisConstraintEditorDialog QLabel {
    background: transparent;
    color: $TEXT;
}
QDialog#nodeResultExportDialog QLabel { background: transparent; }
QDialog#nodeResultExportDialog #exportDialogTitle { font-size: 20px; font-weight: 750; color: $TEXT; }
QDialog#nodeResultExportDialog #exportDialogSubtitle,
QDialog#nodeResultExportDialog #exportFieldHint { color: $MUTED; }
QDialog#nodeResultExportDialog #exportSectionCard {
    background: $ELEVATED; border: 1px solid $BORDER; border-radius: 12px;
}
QDialog#nodeResultExportDialog #exportSectionTitle { color: $TEXT; font-weight: 700; }
QDialog#nodeResultExportDialog #exportFieldGroup {
    background: $FIELD; border: 1px solid $BORDER; border-radius: 9px;
    margin-top: 8px; padding-top: 4px;
}
QDialog#nodeResultExportDialog #exportFieldGroup::title {
    subcontrol-origin: margin; left: 10px; padding: 0 5px; color: $MUTED;
}
QDialog#nodeResultExportDialog #exportQuietButton { background: $FIELD; }
QDialog#nodeResultExportDialog #exportValidationLabel { color: #E05252; }
QDialog#nodeResultExportDialog QDialogButtonBox#exportDialogButtons { background: transparent; border: none; }
#appHeader { background: $HEADER; border-bottom: 1px solid $BORDER; }
#logoLabel { background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 $ACCENT,stop:1 $ACCENT2); color: $SELECTION_TEXT; border-radius: 9px; font-size: 16px; font-weight: 800; }
#appTitleLabel { font-size: 19px; font-weight: 750; }
#themeLabel, #loadStatisticsLabel, #incrementLabel { color: $MUTED; }
#navigationRail { background: $NAV; border-right: 1px solid $BORDER; }
#projectPanel { background: $PANEL; }
#projectPanel { border-right: 1px solid $BORDER; }
#workspacePanel { background: $ROOT; }
#statusFrame { background: $NAV; border-top: 1px solid $BORDER; }
#projectTitleLabel { font-size: 19px; font-weight: 700; }
#readyLabel { color: $MUTED; }
#viewportEmptyLabel { color: $MUTED; font-size: 14px; background: transparent; }

QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QPlainTextEdit {
    background: $FIELD; border: 1px solid $BORDER_STRONG; border-radius: 7px; padding: 6px 10px; selection-background-color: $ACCENT; min-height: 20px;
}
QComboBox { selection-background-color: $ACCENT_SOFT; selection-color: $TEXT; }
QLineEdit:hover, QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover { border-color: $ACCENT2; }
QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus { border: 1px solid $ACCENT; background: $ELEVATED; }
QComboBox::drop-down { border: none; width: 28px; }
QComboBox::down-arrow { image: url(:/YQY/icon_chevron_down.svg); width: 12px; height: 12px; margin-right: 8px; }
QComboBoxPrivateContainer {
    background: $ELEVATED;
    border: 1px solid $BORDER_STRONG;
    border-radius: 7px;
    padding: 0;
}
QComboBox QAbstractItemView {
    background: $ELEVATED;
    color: $TEXT;
    border: none;
    border-radius: 6px;
    selection-background-color: $ACCENT_SOFT;
    selection-color: $TEXT;
    padding: 4px;
    margin: 0;
}
QComboBox QAbstractItemView::item {
    background: transparent;
    color: $TEXT;
    border: none;
    border-radius: 5px;
    min-height: 28px;
    padding: 3px 8px;
}
QComboBox QAbstractItemView::item:hover,
QComboBox QAbstractItemView::item:selected {
    background: $ACCENT_SOFT;
    color: $TEXT;
}
QComboBoxPrivateContainer QAbstractItemView,
QAbstractItemView {
    selection-background-color: $ACCENT_SOFT;
    selection-color: $TEXT;
}
QComboBoxPrivateContainer QAbstractItemView::item:hover,
QComboBoxPrivateContainer QAbstractItemView::item:selected,
QAbstractItemView::item:hover,
QAbstractItemView::item:selected {
    background: $ACCENT_SOFT;
    color: $TEXT;
}
QDoubleSpinBox::up-button, QSpinBox::up-button,
QDoubleSpinBox::down-button, QSpinBox::down-button {
    subcontrol-origin: padding;
    width: 22px;
    background: transparent;
    border: none;
    border-left: 1px solid $BORDER;
}
QDoubleSpinBox::up-button, QSpinBox::up-button {
    subcontrol-position: top right;
    border-top-right-radius: 5px;
}
QDoubleSpinBox::down-button, QSpinBox::down-button {
    subcontrol-position: bottom right;
    border-bottom-right-radius: 5px;
}
QDoubleSpinBox::up-button:hover, QSpinBox::up-button:hover,
QDoubleSpinBox::down-button:hover, QSpinBox::down-button:hover { background: $ACCENT_SOFT; }
QDoubleSpinBox::up-arrow, QSpinBox::up-arrow {
    image: url(:/YQY/icon_chevron_up.svg); width: 10px; height: 10px;
}
QDoubleSpinBox::down-arrow, QSpinBox::down-arrow {
    image: url(:/YQY/icon_chevron_down.svg); width: 10px; height: 10px;
}

QMenu {
    background: $ELEVATED; color: $TEXT; border: 1px solid $BORDER_STRONG; border-radius: 7px; padding: 5px;
}
QMenu::item {
    background: transparent; color: $TEXT; border-radius: 5px; padding: 7px 28px 7px 10px; margin: 1px;
}
QMenu::item:selected { background: $ACCENT_SOFT; color: $TEXT; }
QMenu::item:disabled { background: transparent; color: $MUTED; }
QMenu::separator { height: 1px; background: $BORDER; margin: 5px 8px; }
QMenu::indicator { width: 14px; height: 14px; }
QMenu::indicator:checked { image: url(:/YQY/icon_check_small.svg); background: $ACCENT2; border: 1px solid $ACCENT; border-radius: 3px; }

QListView, QListWidget, QTreeView {
    background: $ELEVATED;
    color: $TEXT;
    border: 1px solid $BORDER_STRONG;
    selection-background-color: $ACCENT_SOFT;
    selection-color: $TEXT;
}
QListView::item, QListWidget::item, QTreeView::item { padding: 5px 8px; }
QListView::item:hover, QListWidget::item:hover,
QTreeView::item:hover, QListView::item:selected,
QListWidget::item:selected, QTreeView::item:selected {
    background: $ACCENT_SOFT;
    color: $TEXT;
}

QPushButton, QToolButton {
    background: $ELEVATED; border: 1px solid $BORDER_STRONG; border-radius: 7px; padding: 7px 13px; min-height: 20px;
}
QPushButton:hover, QToolButton:hover { background: $ACCENT_SOFT; border-color: $ACCENT2; }
QPushButton:pressed, QToolButton:pressed { background: $FIELD; border-color: $ACCENT; padding-top: 8px; padding-bottom: 6px; }
QPushButton:disabled, QToolButton:disabled, QLineEdit:disabled, QComboBox:disabled, QDoubleSpinBox:disabled { color: $MUTED; background: $FIELD; border-color: $BORDER; }
#startSolveButton { background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 $ACCENT,stop:1 $ACCENT2); color: $SELECTION_TEXT; border: none; font-weight: 750; padding-left: 18px; padding-right: 18px; }
#startSolveButton:hover { border: 1px solid $TEXT; }
#stopButton { min-width: 84px; }
#taskControlButton {
    min-width: 26px; max-width: 26px; min-height: 26px; max-height: 26px;
    border-radius: 13px; padding: 0; background: transparent; border: none;
}
#taskControlButton:hover { background: $ACCENT_SOFT; border: none; }
#taskControlButton:pressed { padding: 0; background: $ELEVATED; border: none; }
#taskControlButton:disabled { background: transparent; border: none; }
#solveTaskActionButton {
    padding: 5px 4px; text-align: center; min-width: 0;
}
#solveTaskActionButton:pressed { padding: 6px 6px 4px 6px; }
#solveCasesButton, #solveAllCasesButton, #solveRestartAllCasesButton {
    padding: 7px 3px; min-width: 0;
}
#solveCaseWidget, #solveCaseLabel { background: transparent; border: none; }
#solveCaseLabel { padding: 0; margin: 0; }

#navigationRail QPushButton { background: transparent; border: 1px solid transparent; border-radius: 9px; padding: 5px 2px; color: $MUTED; }
#navigationRail QPushButton:hover { background: $ELEVATED; color: $TEXT; border-color: $BORDER; }
#navigationRail QPushButton:checked { background: $ACCENT_SOFT; color: $TEXT; border-color: $ACCENT2; border-left: 3px solid $ACCENT2; }

#toolBarCard { background: $ELEVATED; border: 1px solid $BORDER; border-radius: 9px; }
#toolbarScrollArea, #toolbarScrollArea QWidget#qt_scrollarea_viewport { background: transparent; border: none; }
#analysisParameterCard { background: $ELEVATED; border: 1px solid $BORDER; border-radius: 9px; }
#toolBarCard QPushButton, #toolBarCard QToolButton { padding: 0; min-width: 36px; min-height: 34px; }
#toolBarCard #startSolveButton { padding: 0; }
#toolBarCard QToolButton:checked { background: $ACCENT_SOFT; border-color: $ACCENT2; color: $TEXT; }

QTreeWidget { background: transparent; border: none; alternate-background-color: transparent; padding: 2px; }
QTreeWidget::item { min-height: 27px; border-radius: 6px; padding: 2px 4px; }
QTreeWidget::item:hover { background: $ELEVATED; }
QTreeWidget::item:selected { background: $ACCENT_SOFT; color: $TEXT; }
QTreeWidget::branch { background: transparent; }
QTreeWidget::branch:hover { background: $ELEVATED; }
QTreeWidget::branch:selected { background: $ACCENT_SOFT; }
QTreeWidget#solveStepTree QHeaderView::section { min-height: 32px; padding: 3px 2px; }

QTableView, QTableWidget {
    background: $FIELD;
    alternate-background-color: $ELEVATED;
    color: $TEXT;
    border: 1px solid $BORDER_STRONG;
    border-radius: 6px;
    gridline-color: $BORDER;
    selection-background-color: $ACCENT_SOFT;
    selection-color: $TEXT;
}
QTableView::item, QTableWidget::item {
    color: $TEXT;
    border-bottom: 1px solid $BORDER;
    padding: 7px 9px;
}
QTableView::item:hover, QTableWidget::item:hover { background: $ELEVATED; }
QTableView::item:selected, QTableWidget::item:selected {
    background: $ACCENT_SOFT;
    color: $TEXT;
}
QTableView QLineEdit, QTableWidget QLineEdit {
    background: $PANEL;
    color: $TEXT;
    border: 1px solid $ACCENT2;
    border-radius: 3px;
    padding: 2px 5px;
}
QHeaderView { background: $PANEL; color: $TEXT; }
QHeaderView::section {
    background: $PANEL;
    color: $MUTED;
    border: none;
    border-right: 1px solid $BORDER;
    border-bottom: 1px solid $BORDER_STRONG;
    padding: 9px 8px;
    font-weight: 650;
}
QTableCornerButton::section { background: $PANEL; border: none; border-bottom: 1px solid $BORDER_STRONG; }
#propertyPage { background: $PANEL; }
#propertyTabs::pane { background: $FIELD; border: 1px solid $BORDER; border-radius: 9px; }
#propertyTabs QTabBar::tab:selected { color: $TEXT; border-bottom: 2px solid $ACCENT2; }

/* The conductor page is created at runtime, so every background layer must
   explicitly participate in the active theme instead of using the OS palette. */
#conductorPage,
#conductorScrollArea,
#conductorScrollArea QWidget#qt_scrollarea_viewport,
#conductorContent {
    background: $PANEL;
    color: $TEXT;
    border: none;
}
#conductorPropertyGroup, #conductorFormGroup {
    background: $ELEVATED;
    color: $TEXT;
    border: 1px solid $BORDER;
    border-radius: 9px;
    margin-top: 12px;
    padding-top: 8px;
}
#conductorPropertyGroup::title, #conductorFormGroup::title {
    color: $TEXT;
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    background: $ELEVATED;
}
#conductorPropertyGroup QLabel,
#conductorFormGroup QLabel,
#conductorFormGroup QCheckBox {
    background: transparent;
    color: $TEXT;
}
#conductorPage QLineEdit,
#conductorPage QComboBox,
#conductorPage QDoubleSpinBox,
#conductorPage QSpinBox {
    background: $FIELD;
    color: $TEXT;
    border-color: $BORDER_STRONG;
}
#conductorPage QLineEdit:focus,
#conductorPage QComboBox:focus,
#conductorPage QDoubleSpinBox:focus,
#conductorPage QSpinBox:focus {
    background: $ELEVATED;
    border-color: $ACCENT;
}
#conductorCreateButton {
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 $ACCENT,stop:1 $ACCENT2);
    color: $SELECTION_TEXT;
    border: none;
    font-weight: 700;
}
#conductorCreateButton:hover { border: 1px solid $TEXT; }

QTabBar::tab { background: transparent; color: $MUTED; border: 1px solid transparent; padding: 8px 16px; margin-right: 4px; border-radius: 7px; }
QTabBar::tab:hover { color: $TEXT; background: $ELEVATED; }
QTabBar::tab:selected { color: $TEXT; background: $ELEVATED; border-color: $BORDER_STRONG; }
QTabWidget::pane { background: $ELEVATED; border: 1px solid $BORDER; border-radius: 9px; top: -1px; }
QTabWidget QTabBar::tab { padding: 6px 13px; }
QTabBar::close-button { image: url(:/YQY/icon_close.svg); width: 14px; height: 14px; margin: 2px 4px; border-radius: 3px; }
QTabBar::close-button:hover { background: $ACCENT_SOFT; }

QProgressBar { background: $FIELD; border: none; border-radius: 4px; min-height: 8px; max-height: 8px; text-align: right; color: transparent; }
QProgressBar::chunk { border-radius: 4px; background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 $ACCENT,stop:1 $ACCENT2); }
QProgressBar#solveTaskProgress { border: 1px solid $BORDER; border-radius: 7px; min-height: 22px; max-height: 22px; text-align: center; color: $TEXT; font-weight: 650; }
QProgressBar#solveTaskProgress::chunk { border-radius: 6px; }
QSlider::groove:horizontal { height: 5px; border-radius: 2px; background: $BORDER_STRONG; }
QSlider::sub-page:horizontal { background: $ACCENT2; border-radius: 2px; }
QSlider::handle:horizontal { background: $TEXT; border: 2px solid $ACCENT2; width: 14px; height: 14px; margin: -6px 0; border-radius: 8px; }
QCheckBox { spacing: 10px; }
QCheckBox::indicator { width: 34px; height: 18px; border-radius: 9px; background: $BORDER_STRONG; border: 1px solid $BORDER; }
QCheckBox::indicator:checked { image: url(:/YQY/icon_check.svg); background: $ACCENT2; border-color: $ACCENT2; }
QSplitter::handle { background: $BORDER; }
QSplitter::handle:horizontal { width: 4px; }
QSplitter::handle:vertical { height: 6px; }
QSplitter::handle:hover { background: $ACCENT2; }
QScrollBar:vertical { width: 8px; background: transparent; margin: 2px; }
QScrollBar::handle:vertical { background: $BORDER_STRONG; border-radius: 4px; min-height: 28px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal { height: 8px; background: transparent; margin: 1px; }
QScrollBar::handle:horizontal { background: $BORDER_STRONG; border-radius: 4px; min-width: 28px; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QToolTip, QLabel#qtooltip_label {
    background: $ELEVATED;
    color: $TEXT;
    border: 1px solid $BORDER_STRONG;
    border-radius: 4px;
    padding: 5px 7px;
}
)QSS");

    style.replace(QStringLiteral("$ROOT"), c.rootBackground);
    style.replace(QStringLiteral("$HEADER"), c.header);
    style.replace(QStringLiteral("$NAV"), c.navigation);
    style.replace(QStringLiteral("$PANEL"), c.panel);
    style.replace(QStringLiteral("$ELEVATED"), c.elevated);
    style.replace(QStringLiteral("$FIELD"), c.field);
    style.replace(QStringLiteral("$BORDER_STRONG"), c.borderStrong);
    style.replace(QStringLiteral("$BORDER"), c.border);
    style.replace(QStringLiteral("$TEXT"), c.text);
    style.replace(QStringLiteral("$MUTED"), c.muted);
    style.replace(QStringLiteral("$ACCENT_SOFT"), c.accentSoft);
    style.replace(QStringLiteral("$ACCENT2"), c.accentSecond);
    style.replace(QStringLiteral("$ACCENT"), c.accent);
    style.replace(QStringLiteral("$SUCCESS"), c.success);
    style.replace(QStringLiteral("$SELECTION_TEXT"), c.selectionText);
    return style;
}
} // namespace

YQY::YQY(QWidget* parent)
    : QMainWindow(parent), m_modelController(new ModelController(this)),
      m_solveTaskController(new SolveTaskController(this))
{
    ui.setupUi(this);
    if (!qApp->findChild<QObject*>(QStringLiteral("popupThemeFilter")))
    {
        auto* popupThemeFilter = new PopupThemeFilter(qApp);
        popupThemeFilter->setObjectName(QStringLiteral("popupThemeFilter"));
        qApp->installEventFilter(popupThemeFilter);
    }
    m_propertyLibrary = std::make_unique<Conductor::PropertyLibrary>();
    QString propertyLibraryError;
    const bool propertyLibraryReady = m_propertyLibrary->load(propertyLibraryError);
    initializeConductorModule();
    initializePropertyModule();
    initializeResultModule();
    initializeSettingsModule();
    ui.startSolveButton->hide();
    ui.solvePageStartButton->hide();
    setMinimumSize(760, 520);
    initializeToolbarAppearance();

    // The conductor/property/analysis editors need enough room for their
    // labels and native sub-controls.  A narrower initial pane made fields
    // look broken even though the splitter itself was still valid.
    ui.mainSplitter->setSizes(m_normalMainSplitterSizes);
    ui.modelDataTree->header()->setStretchLastSection(true);
    ui.solveStepTree->setColumnCount(4);
    ui.solveStepTree->setHeaderHidden(false);
    ui.solveStepTree->setHeaderLabels(
        {QStringLiteral("算例"), QStringLiteral("进度"), QStringLiteral("结果"), QStringLiteral("耗时")});
    ui.solveStepTree->header()->setDefaultAlignment(Qt::AlignCenter);
    ui.solveStepTree->header()->setMinimumHeight(34);
    ui.solveStepTree->setRootIsDecorated(false);
    ui.solveStepTree->setIndentation(0);
    ui.solveStepTree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    ui.solveStepTree->header()->setSectionResizeMode(1, QHeaderView::Fixed);
    ui.solveStepTree->header()->setSectionResizeMode(2, QHeaderView::Fixed);
    ui.solveStepTree->header()->setSectionResizeMode(3, QHeaderView::Fixed);
    ui.solveStepTree->setColumnWidth(1, 92);
    ui.solveStepTree->setColumnWidth(2, 72);
    ui.solveStepTree->setColumnWidth(3, 72);
    m_solveCasesButton = ui.solveCasesButton;
    m_solveAllCasesButton = ui.solveAllCasesButton;
    m_solveRestartAllCasesButton = ui.solveRestartAllCasesButton;
    const QList<QPushButton*> solveBatchButtons = {
        m_solveCasesButton, m_solveAllCasesButton, m_solveRestartAllCasesButton};
    for (QPushButton* button : solveBatchButtons)
    {
        button->setMinimumHeight(54);
        button->setMinimumWidth(0);
        button->setIconSize(QSize(16, 16));
        button->setCursor(Qt::PointingHandCursor);
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }
    QFont solveCasesFont = m_solveCasesButton->font();
    solveCasesFont.setPointSizeF(qMax(10.0, solveCasesFont.pointSizeF()));
    solveCasesFont.setWeight(QFont::DemiBold);
    m_solveCasesButton->setFont(solveCasesFont);
    m_solveAllCasesButton->setFont(solveCasesFont);
    m_solveRestartAllCasesButton->setFont(solveCasesFont);
    m_solveCasesButton->setToolTip(QStringLiteral("打开全部分析步求解窗口"));
    m_solveAllCasesButton->setToolTip(QStringLiteral("将所有待运行分析步加入计算队列；正在计算、已排队和已有结果的算例不会重复计算；单个分析步失败不会停止其他队列任务"));
    m_solveRestartAllCasesButton->setToolTip(
        QStringLiteral("重新计算所有分析步；正在排队或计算的任务会先安全停止，再从头加入计算队列"));
    connect(m_solveCasesButton, &QPushButton::clicked, this, &YQY::openSolveTaskManager);
    connect(m_solveAllCasesButton, &QPushButton::clicked, this, &YQY::startAllReadySolveTasks);
    connect(m_solveRestartAllCasesButton, &QPushButton::clicked, this, &YQY::restartAllSolveTasks);
    ui.solveProgress->setObjectName(QStringLiteral("solveTaskProgress"));
    connect(ui.solveProgress, &QProgressBar::valueChanged, ui.solveProgress,
            [this](int value) { ui.solveProgress->setFormat(QStringLiteral("%1%").arg(value / 10.0, 0, 'f', 1)); });
    initializeAnalysisEditor();
    initializeResponsiveLayout();

    initializeEmptyState();
    initializeInteractions();

    if (propertyLibraryReady)
        ui.logEdit->appendPlainText(QStringLiteral("[INFO] 启动属性库已加载：%1 种材料，%2 种截面")
                                        .arg(m_propertyLibrary->materials().size())
                                        .arg(m_propertyLibrary->sections().size()));
    else
    {
        m_conductorModule->createButton()->setEnabled(false);
        ui.logEdit->appendPlainText(QStringLiteral("[ERROR] 启动属性库加载失败：%1").arg(propertyLibraryError));
    }

    QSettings settings;
    const int savedTheme = qBound(0, settings.value(QStringLiteral("appearance/theme"), 0).toInt(), 3);
    ui.themeComboBox->setCurrentIndex(savedTheme);
    applyTheme(savedTheme);
}

YQY::~YQY() = default;

void YQY::initializeSettingsModule()
{
    m_settingsPanel = new SettingsPanel(ui.settingsModulePage);
    auto* concurrencySpin = m_settingsPanel->concurrencySpin();
    const int availableThreads = SolveTaskController::availableThreadCount();
    const int allowedThreads = SolveTaskController::maximumAllowedThreadCount();
    concurrencySpin->setRange(SolveTaskController::MinimumThreadCount, allowedThreads);
    concurrencySpin->setSuffix(QStringLiteral(" 个计算区域"));
    concurrencySpin->setMinimumWidth(150);
    concurrencySpin->setToolTip(
        QStringLiteral("本机可选范围 1～%1；降低上限不会中断正在计算的任务").arg(allowedThreads));

    QSettings settings;
    const int savedConcurrency = qBound(
        SolveTaskController::MinimumThreadCount,
        settings.value(QStringLiteral("solver/maxConcurrentSteps"), SolveTaskController::defaultThreadCount()).toInt(),
        allowedThreads);
    concurrencySpin->setValue(savedConcurrency);
    m_solveTaskController->setMaximumThreadCount(savedConcurrency);
    auto* status = m_settingsPanel->statusLabel();
    const auto updateStatus = [status, availableThreads, allowedThreads](int count)
    {
        status->setText(
            QStringLiteral("检测到 %1 个逻辑线程，本机安全上限为 %2；当前每个分析步最多同时运行 %3 个计算区域。")
                .arg(availableThreads)
                .arg(allowedThreads)
                .arg(count));
    };
    updateStatus(savedConcurrency);
    connect(concurrencySpin, qOverload<int>(&QSpinBox::valueChanged), this,
            [this, status, updateStatus](int count)
            {
                m_solveTaskController->setMaximumThreadCount(count);
                const int applied = m_solveTaskController->maximumThreadCount();
                QSettings().setValue(QStringLiteral("solver/maxConcurrentSteps"), applied);
                updateStatus(applied);
                setWorkspaceMessage(QStringLiteral("最大并发计算区域已设置为 %1").arg(applied));
            });

    ui.settingsModuleLayout->insertWidget(1, m_settingsPanel);
}

bool YQY::event(QEvent* event)
{
    const bool handled = QMainWindow::event(event);
    if (event->type() == QEvent::ScreenChangeInternal || event->type() == QEvent::ApplicationFontChange ||
        event->type() == QEvent::StyleChange)
    {
        QTimer::singleShot(0, this, &YQY::updateResponsiveLayout);
    }
    return handled;
}

void YQY::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    updateResponsiveLayout();
}

bool YQY::openModel(const QString& filePath)
{
    return loadModelFile(filePath);
}

bool YQY::openResult(const QString& filePath)
{
    return loadHdf5Result(filePath, false);
}

bool YQY::saveAnalysisManagerPreview(const QString& filePath, int managerIndex)
{
    const auto structure = m_modelController->model();
    if (!structure || filePath.trimmed().isEmpty())
        return false;
    std::unique_ptr<AnalysisManagerDialog> manager;
    if (managerIndex == 1)
        manager = std::make_unique<AnalysisLoadManagerDialog>(structure, this);
    else if (managerIndex == 2)
        manager = std::make_unique<AnalysisConstraintManagerDialog>(structure, this);
    else
        manager = std::make_unique<AnalysisStepManagerDialog>(structure, this);
    manager->show();
    QCoreApplication::processEvents(QEventLoop::AllEvents);
    const bool saved = manager->grab().save(filePath);
    manager->close();
    return saved;
}

bool YQY::saveSolveTaskManagerPreview(const QString& filePath)
{
    if (!m_modelController->model() || m_solveTaskController->taskIds().isEmpty())
        return false;
    switchModule(Module::Solve);
    openSolveTaskManager();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    return m_solveTaskManager && m_solveTaskManager->grab().save(filePath);
}

bool YQY::saveNodeExportPreview(const QString& filePath)
{
    const auto structure = m_modelController->model();
    const QString& resultFilePath = m_resultPanel->resultFilePath();
    if (!structure || resultFilePath.isEmpty())
        return false;
    QSet<int> availableNodes;
    for (const auto& [nodeId, node] : structure->m_Nodes)
    {
        if (node)
            availableNodes.insert(nodeId);
    }
    NodeResultExportDialog dialog(resultFilePath, availableNodes, m_selectedNodeId, this);
    dialog.show();
    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
    const bool saved = dialog.grab().save(filePath);
    dialog.close();
    return saved;
}

void YQY::initializeResultModule()
{
    m_resultPanel = new ResultControlPanel(ui.resultModulePage);
    setActionGlyph(m_resultPanel->exportButton(), ActionGlyph::Export);
    ui.resultModuleLayout->insertWidget(3, m_resultPanel);
    m_resultPanel->setFrameChangedHandler([this](double framePosition)
    {
        displayResultPosition(framePosition);
    });
    m_resultPanel->setVisualizationChangedHandler([this]() { updateResultVisualization(); });
    m_resultPanel->setExportHandler([this]() { exportNodeResults(); });
}

void YQY::initializeConductorModule()
{
    m_conductorModule = new ConductorModule(ui.moduleStack);
    auto* viewLibraryButton = m_conductorModule->viewLibraryButton();

    setActionGlyph(viewLibraryButton, ActionGlyph::Library);
    setActionGlyph(m_conductorModule->createButton(), ActionGlyph::CreateConductor);
    m_conductorModule->setPropertyLibrary(m_propertyLibrary.get());
    ui.moduleStack->insertWidget(1, m_conductorModule);

    connect(viewLibraryButton, &QPushButton::clicked, this, [this]() { switchModule(Module::Properties); });
    connect(m_conductorModule->createButton(), &QPushButton::clicked, this, &YQY::createConductorModel);
}

void YQY::initializePropertyModule()
{
    m_propertyModule = new PropertyModule(ui.moduleStack);
    setActionGlyph(m_propertyModule->refreshButton(), ActionGlyph::Refresh);
    setActionGlyph(m_propertyModule->applyButton(), ActionGlyph::Apply);
    ui.moduleStack->insertWidget(1, m_propertyModule);

    connect(m_propertyModule->refreshButton(), &QPushButton::clicked, this, &YQY::refreshPropertyModule);
    connect(m_propertyModule->applyButton(), &QPushButton::clicked, this, &YQY::applyModelPropertyEdits);
    connect(m_propertyModule->materialTree(), &QTreeWidget::itemDoubleClicked, this, &YQY::editPropertyItem);
    connect(m_propertyModule->sectionTree(), &QTreeWidget::itemDoubleClicked, this, &YQY::editPropertyItem);
}

void YQY::refreshPropertyModule()
{
    auto* m_materialTable = m_propertyModule ? m_propertyModule->materialTable() : nullptr;
    auto* m_sectionTable = m_propertyModule ? m_propertyModule->sectionTable() : nullptr;
    auto* m_materialPropertyTree = m_propertyModule ? m_propertyModule->materialTree() : nullptr;
    auto* m_sectionPropertyTree = m_propertyModule ? m_propertyModule->sectionTree() : nullptr;
    auto* m_propertyTabs = m_propertyModule ? m_propertyModule->tabs() : nullptr;
    auto* m_applyPropertyButton = m_propertyModule ? m_propertyModule->applyButton() : nullptr;
    if (!m_materialTable || !m_sectionTable || !m_materialPropertyTree || !m_sectionPropertyTree)
        return;
    const QSignalBlocker materialBlocker(m_materialTable);
    const QSignalBlocker sectionBlocker(m_sectionTable);
    m_materialTable->setRowCount(0);
    m_sectionTable->setRowCount(0);
    m_materialPropertyTree->clear();
    m_sectionPropertyTree->clear();
    int materialItemCount = 0;
    int sectionItemCount = 0;

    auto* defaultMaterialGroup = new QTreeWidgetItem(m_materialPropertyTree, {QStringLiteral("默认属性库")});
    auto* defaultSectionGroup = new QTreeWidgetItem(m_sectionPropertyTree, {QStringLiteral("默认属性库")});
    defaultMaterialGroup->setToolTip(0, QStringLiteral("MaterialProperty.bdf"));
    defaultSectionGroup->setToolTip(0, QStringLiteral("MaterialProperty.bdf"));
    setTreeGlyph(defaultMaterialGroup, TreeGlyph::Project);
    setTreeGlyph(defaultSectionGroup, TreeGlyph::Project);
    if (m_propertyLibrary)
    {
        for (int index = 0; index < m_propertyLibrary->materials().size(); ++index)
        {
            const auto& material = m_propertyLibrary->materials().at(index);
            auto* item = new QTreeWidgetItem(defaultMaterialGroup,
                                             {QStringLiteral("材料 %1 · %2").arg(material.id).arg(material.name)});
            item->setToolTip(0, QStringLiteral("%1 | E=%2 Pa | ν=%3 | ρ=%4 kg/m³")
                                    .arg(material.category)
                                    .arg(material.young, 0, 'g', 6)
                                    .arg(material.poisson, 0, 'g', 4)
                                    .arg(material.density, 0, 'g', 6));
            item->setData(0, Qt::UserRole, 1);
            item->setData(0, Qt::UserRole + 1, index);
            setTreeGlyph(item, TreeGlyph::Material);
            ++materialItemCount;
        }
        for (int index = 0; index < m_propertyLibrary->sections().size(); ++index)
        {
            const auto& section = m_propertyLibrary->sections().at(index);
            auto* item = new QTreeWidgetItem(defaultSectionGroup,
                                             {QStringLiteral("截面 %1 · %2").arg(section.id).arg(section.name)});
            item->setToolTip(0, QStringLiteral("%1 | A=%2 m² | %3")
                                    .arg(section.category)
                                    .arg(section.area, 0, 'g', 8)
                                    .arg(section.description));
            item->setData(0, Qt::UserRole, 3);
            item->setData(0, Qt::UserRole + 1, index);
            setTreeGlyph(item, TreeGlyph::Section);
            ++sectionItemCount;
        }
    }

    auto setCell = [](QTableWidget* table, int row, int column, const QString& text, bool editable)
    {
        auto* cell = new QTableWidgetItem(text);
        if (!editable)
            cell->setFlags(cell->flags() & ~Qt::ItemIsEditable);
        table->setItem(row, column, cell);
        return cell;
    };

    for (int modelId : m_modelController->modelIds())
    {
        const auto structure = m_modelController->model(modelId);
        if (!structure)
            continue;
        const QString modelName = QFileInfo(m_modelController->filePath(modelId)).fileName();
        auto* materialGroup =
            new QTreeWidgetItem(m_materialPropertyTree, {QStringLiteral("模型 %1 · %2").arg(modelId).arg(modelName)});
        auto* sectionGroup =
            new QTreeWidgetItem(m_sectionPropertyTree, {QStringLiteral("模型 %1 · %2").arg(modelId).arg(modelName)});
        setTreeGlyph(materialGroup, TreeGlyph::Model);
        setTreeGlyph(sectionGroup, TreeGlyph::Model);
        for (const auto& [materialId, material] : structure->m_Material)
        {
            if (!material)
                continue;
            const int row = m_materialTable->rowCount();
            m_materialTable->insertRow(row);
            auto* modelCell = setCell(m_materialTable, row, 0, QString::number(modelId), false);
            modelCell->setData(Qt::UserRole, modelId);
            modelCell->setData(Qt::UserRole + 1, materialId);
            setCell(m_materialTable, row, 1, modelName, false);
            setCell(m_materialTable, row, 2, QString::number(materialId), false);
            setCell(m_materialTable, row, 3, QString::number(material->m_Young, 'g', 12), true);
            setCell(m_materialTable, row, 4, QString::number(material->m_Poisson, 'g', 12), true);
            setCell(m_materialTable, row, 5, QString::number(material->m_Density, 'g', 12), true);
            setCell(m_materialTable, row, 6, QString::number(material->m_MaxStress, 'g', 12), true);
            setCell(m_materialTable, row, 7, QString::number(material->m_Expansion, 'g', 12), true);
            auto* item = new QTreeWidgetItem(materialGroup, {QStringLiteral("材料 ID %1 · E=%2 · ρ=%3")
                                                                 .arg(materialId)
                                                                 .arg(material->m_Young, 0, 'g', 5)
                                                                 .arg(material->m_Density, 0, 'g', 5)});
            item->setData(0, Qt::UserRole, 2);
            item->setData(0, Qt::UserRole + 1, modelId);
            item->setData(0, Qt::UserRole + 2, materialId);
            setTreeGlyph(item, TreeGlyph::Material);
            ++materialItemCount;
        }

        for (const auto& [sectionId, section] : structure->m_Section)
        {
            if (!section)
                continue;
            const auto circular = std::dynamic_pointer_cast<SectionCircular>(section);
            const auto rectangle = std::dynamic_pointer_cast<SectionRectangle>(section);
            const QString type = rectangle  ? QStringLiteral("矩形")
                                 : circular ? QStringLiteral("圆形")
                                            : QStringLiteral("通用");
            const int row = m_sectionTable->rowCount();
            m_sectionTable->insertRow(row);
            auto* modelCell = setCell(m_sectionTable, row, 0, QString::number(modelId), false);
            modelCell->setData(Qt::UserRole, modelId);
            modelCell->setData(Qt::UserRole + 1, sectionId);
            setCell(m_sectionTable, row, 1, modelName, false);
            setCell(m_sectionTable, row, 2, QString::number(sectionId), false);
            setCell(m_sectionTable, row, 3, type, false);
            setCell(m_sectionTable, row, 4, QString::number(section->m_Area, 'g', 12), !rectangle);
            setCell(m_sectionTable, row, 5, circular ? QString::number(circular->m_Radius, 'g', 12) : QString(), false);
            setCell(m_sectionTable, row, 6, rectangle ? QString::number(rectangle->m_Width, 'g', 12) : QString(),
                    rectangle != nullptr);
            setCell(m_sectionTable, row, 7, rectangle ? QString::number(rectangle->m_Height, 'g', 12) : QString(),
                    rectangle != nullptr);
            auto* item = new QTreeWidgetItem(
                sectionGroup,
                {QStringLiteral("截面 ID %1 · %2 · A=%3").arg(sectionId).arg(type).arg(section->m_Area, 0, 'g', 6)});
            item->setData(0, Qt::UserRole, 4);
            item->setData(0, Qt::UserRole + 1, modelId);
            item->setData(0, Qt::UserRole + 2, sectionId);
            setTreeGlyph(item, TreeGlyph::Section);
            ++sectionItemCount;
        }
    }
    m_materialPropertyTree->expandAll();
    m_sectionPropertyTree->expandAll();
    const ThemeColors propertyColors = themeColors(ui.themeComboBox->currentIndex());
    applyTreeIcons(m_materialPropertyTree, QColor(propertyColors.text), QColor(propertyColors.accentSecond));
    applyTreeIcons(m_sectionPropertyTree, QColor(propertyColors.text), QColor(propertyColors.accentSecond));
    m_propertyTabs->setTabText(0, QStringLiteral("材料 (%1)").arg(materialItemCount));
    m_propertyTabs->setTabText(1, QStringLiteral("截面 (%1)").arg(sectionItemCount));
    m_applyPropertyButton->setEnabled(m_materialTable->rowCount() > 0 || m_sectionTable->rowCount() > 0);
}

void YQY::editPropertyItem(QTreeWidgetItem* item, int)
{
    if (!item)
        return;
    const int kind = item->data(0, Qt::UserRole).toInt();
    if (kind < 1 || kind > 4)
        return;

    PropertyItemEditorDialog dialog(this);

    if (kind == 1 || kind == 2)
    {
        Conductor::MaterialPreset preset;
        std::shared_ptr<Material> modelMaterial;
        int libraryIndex = -1;
        int modelId = -1;
        int materialId = -1;
        QString sourceText;
        if (kind == 1)
        {
            libraryIndex = item->data(0, Qt::UserRole + 1).toInt();
            if (!m_propertyLibrary || libraryIndex < 0 || libraryIndex >= m_propertyLibrary->materials().size())
                return;
            preset = m_propertyLibrary->materials().at(libraryIndex);
            materialId = preset.id;
            sourceText = QStringLiteral("默认属性库 · MaterialProperty.bdf");
        }
        else
        {
            modelId = item->data(0, Qt::UserRole + 1).toInt();
            materialId = item->data(0, Qt::UserRole + 2).toInt();
            const auto structure = m_modelController->model(modelId);
            const auto found = structure ? structure->m_Material.find(materialId)
                                         : std::map<int, std::shared_ptr<Material>>::iterator{};
            if (!structure || found == structure->m_Material.end() || !found->second)
                return;
            modelMaterial = found->second;
            preset.id = materialId;
            preset.name = QStringLiteral("材料 ID %1").arg(materialId);
            preset.young = modelMaterial->m_Young;
            preset.poisson = modelMaterial->m_Poisson;
            preset.density = modelMaterial->m_Density;
            preset.maxStress = modelMaterial->m_MaxStress;
            preset.expansion = modelMaterial->m_Expansion;
            sourceText = QStringLiteral("模型 %1 · %2")
                             .arg(modelId)
                             .arg(QFileInfo(m_modelController->filePath(modelId)).fileName());
        }
        dialog.setWindowTitle(QStringLiteral("编辑材料属性 · %1").arg(preset.name));
        dialog.configureMaterial(sourceText, materialId, preset.young, preset.poisson, preset.density, preset.maxStress,
                                 preset.expansion);
        auto* young = dialog.young();
        auto* poisson = dialog.poisson();
        auto* density = dialog.density();
        auto* stress = dialog.stress();
        auto* expansion = dialog.expansion();
        auto* buttons = dialog.buttons();
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        connect(buttons, &QDialogButtonBox::accepted, &dialog,
                [&]()
                {
                    if (kind == 1)
                    {
                        preset.young = young->value();
                        preset.poisson = poisson->value();
                        preset.density = density->value();
                        preset.maxStress = stress->value();
                        preset.expansion = expansion->value();
                        QString error;
                        if (!m_propertyLibrary->updateMaterial(libraryIndex, preset, error))
                        {
                            QMessageBox::warning(&dialog, QStringLiteral("保存失败"), error);
                            return;
                        }
                    }
                    else
                    {
                        modelMaterial->m_Young = young->value();
                        modelMaterial->m_Poisson = poisson->value();
                        modelMaterial->m_Density = density->value();
                        modelMaterial->m_MaxStress = stress->value();
                        modelMaterial->m_Expansion = expansion->value();
                    }
                    dialog.accept();
                });
    }
    else
    {
        Conductor::SectionPreset preset;
        std::shared_ptr<SectionBase> modelSection;
        std::shared_ptr<SectionRectangle> rectangle;
        int libraryIndex = -1;
        int modelId = -1;
        int sectionId = -1;
        QString sourceText;
        if (kind == 3)
        {
            libraryIndex = item->data(0, Qt::UserRole + 1).toInt();
            if (!m_propertyLibrary || libraryIndex < 0 || libraryIndex >= m_propertyLibrary->sections().size())
                return;
            preset = m_propertyLibrary->sections().at(libraryIndex);
            sectionId = preset.id;
            sourceText = QStringLiteral("默认属性库 · MaterialProperty.bdf");
        }
        else
        {
            modelId = item->data(0, Qt::UserRole + 1).toInt();
            sectionId = item->data(0, Qt::UserRole + 2).toInt();
            const auto structure = m_modelController->model(modelId);
            const auto found = structure ? structure->m_Section.find(sectionId)
                                         : std::map<int, std::shared_ptr<SectionBase>>::iterator{};
            if (!structure || found == structure->m_Section.end() || !found->second)
                return;
            modelSection = found->second;
            rectangle = std::dynamic_pointer_cast<SectionRectangle>(modelSection);
            preset.id = sectionId;
            preset.name = QStringLiteral("截面 ID %1").arg(sectionId);
            preset.area = modelSection->m_Area;
            sourceText = QStringLiteral("模型 %1 · %2")
                             .arg(modelId)
                             .arg(QFileInfo(m_modelController->filePath(modelId)).fileName());
        }
        dialog.setWindowTitle(QStringLiteral("编辑截面属性 · %1").arg(preset.name));
        dialog.configureSection(sourceText, sectionId, rectangle ? QStringLiteral("矩形") : QStringLiteral("圆形/通用"),
                                preset.area, static_cast<bool>(rectangle), rectangle ? rectangle->m_Width : 0.0,
                                rectangle ? rectangle->m_Height : 0.0);
        auto* area = dialog.area();
        auto* width = dialog.width();
        auto* height = dialog.height();
        auto* buttons = dialog.buttons();
        connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
        connect(buttons, &QDialogButtonBox::accepted, &dialog,
                [&]()
                {
                    if (kind == 3)
                    {
                        preset.area = area->value();
                        QString error;
                        if (!m_propertyLibrary->updateSection(libraryIndex, preset, error))
                        {
                            QMessageBox::warning(&dialog, QStringLiteral("保存失败"), error);
                            return;
                        }
                    }
                    else if (rectangle)
                    {
                        rectangle->m_Width = width->value();
                        rectangle->m_Height = height->value();
                        rectangle->Calculate_Area();
                    }
                    else
                    {
                        modelSection->m_Area = area->value();
                        if (auto circular = std::dynamic_pointer_cast<SectionCircular>(modelSection))
                            circular->Calculate_Radius();
                    }
                    dialog.accept();
                });
    }

    if (dialog.exec() == QDialog::Accepted)
    {
        ui.logEdit->appendPlainText(QStringLiteral("[模型属性] 已保存属性修改；未改写模型源文件"));
        refreshPropertyModule();
        setWorkspaceMessage(QStringLiteral("属性修改已保存，后续提交计算将使用新值"));
    }
}

void YQY::applyModelPropertyEdits()
{
    auto* m_materialTable = m_propertyModule->materialTable();
    auto* m_sectionTable = m_propertyModule->sectionTable();
    struct MaterialEdit
    {
        std::shared_ptr<Material> target;
        double young, poisson, density, stress, expansion;
    };
    struct SectionEdit
    {
        std::shared_ptr<SectionBase> target;
        double area, width, height;
        bool rectangle;
    };
    QVector<MaterialEdit> materialEdits;
    QVector<SectionEdit> sectionEdits;

    for (int row = 0; row < m_materialTable->rowCount(); ++row)
    {
        const int modelId = m_materialTable->item(row, 0)->data(Qt::UserRole).toInt();
        const int materialId = m_materialTable->item(row, 0)->data(Qt::UserRole + 1).toInt();
        const auto structure = m_modelController->model(modelId);
        const auto found =
            structure ? structure->m_Material.find(materialId) : std::map<int, std::shared_ptr<Material>>::iterator{};
        if (!structure || found == structure->m_Material.end() || !found->second)
            continue;
        bool ok[5] = {};
        MaterialEdit edit{found->second,
                          m_materialTable->item(row, 3)->text().toDouble(&ok[0]),
                          m_materialTable->item(row, 4)->text().toDouble(&ok[1]),
                          m_materialTable->item(row, 5)->text().toDouble(&ok[2]),
                          m_materialTable->item(row, 6)->text().toDouble(&ok[3]),
                          m_materialTable->item(row, 7)->text().toDouble(&ok[4])};
        if (!ok[0] || !ok[1] || !ok[2] || !ok[3] || !ok[4] || edit.young <= 0.0 || edit.poisson <= -1.0 ||
            edit.poisson >= 0.5 || edit.density <= 0.0 || edit.stress < 0.0)
        {
            QMessageBox::warning(this, QStringLiteral("材料属性无效"),
                                 QStringLiteral("模型 %1 的材料 ID %2 数值无效。").arg(modelId).arg(materialId));
            return;
        }
        materialEdits.push_back(edit);
    }

    for (int row = 0; row < m_sectionTable->rowCount(); ++row)
    {
        const int modelId = m_sectionTable->item(row, 0)->data(Qt::UserRole).toInt();
        const int sectionId = m_sectionTable->item(row, 0)->data(Qt::UserRole + 1).toInt();
        const auto structure = m_modelController->model(modelId);
        const auto found =
            structure ? structure->m_Section.find(sectionId) : std::map<int, std::shared_ptr<SectionBase>>::iterator{};
        if (!structure || found == structure->m_Section.end() || !found->second)
            continue;
        const bool rectangle = static_cast<bool>(std::dynamic_pointer_cast<SectionRectangle>(found->second));
        bool okArea = false, okWidth = !rectangle, okHeight = !rectangle;
        const double area = m_sectionTable->item(row, 4)->text().toDouble(&okArea);
        const double width = rectangle ? m_sectionTable->item(row, 6)->text().toDouble(&okWidth) : 0.0;
        const double height = rectangle ? m_sectionTable->item(row, 7)->text().toDouble(&okHeight) : 0.0;
        if (!okArea || !okWidth || !okHeight || area <= 0.0 || (rectangle && (width <= 0.0 || height <= 0.0)))
        {
            QMessageBox::warning(this, QStringLiteral("截面属性无效"),
                                 QStringLiteral("模型 %1 的截面 ID %2 数值无效。").arg(modelId).arg(sectionId));
            return;
        }
        sectionEdits.push_back({found->second, area, width, height, rectangle});
    }

    for (const auto& edit : materialEdits)
    {
        edit.target->m_Young = edit.young;
        edit.target->m_Poisson = edit.poisson;
        edit.target->m_Density = edit.density;
        edit.target->m_MaxStress = edit.stress;
        edit.target->m_Expansion = edit.expansion;
    }
    for (const auto& edit : sectionEdits)
    {
        if (edit.rectangle)
        {
            auto rectangle = std::dynamic_pointer_cast<SectionRectangle>(edit.target);
            rectangle->m_Width = edit.width;
            rectangle->m_Height = edit.height;
            rectangle->Calculate_Area();
        }
        else
        {
            edit.target->m_Area = edit.area;
            if (auto circular = std::dynamic_pointer_cast<SectionCircular>(edit.target))
                circular->Calculate_Radius();
        }
    }
    ui.logEdit->appendPlainText(QStringLiteral("[模型属性] 已更新 %1 个材料、%2 个截面；源模型文本未修改")
                                    .arg(materialEdits.size())
                                    .arg(sectionEdits.size()));
    setWorkspaceMessage(QStringLiteral("模型材料/截面修改已应用，后续提交计算将使用新值"));
    refreshPropertyModule();
}

void YQY::showPropertyLibrary()
{
    if (!m_propertyLibrary || !m_propertyLibrary->isReady())
    {
        QMessageBox::warning(this, QStringLiteral("属性库"), QStringLiteral("启动属性库尚未成功加载。"));
        return;
    }

    PropertyLibraryDialog dialog(this);
    auto* materialTable = dialog.materialTable();
    materialTable->setRowCount(m_propertyLibrary->materials().size());
    for (int row = 0; row < m_propertyLibrary->materials().size(); ++row)
    {
        const auto& item = m_propertyLibrary->materials().at(row);
        const QStringList values = {QString::number(item.id),
                                    item.category,
                                    item.name,
                                    QString::number(item.young, 'g', 8),
                                    QString::number(item.poisson),
                                    QString::number(item.density, 'g', 8),
                                    QString::number(item.maxStress, 'g', 8),
                                    QString::number(item.expansion, 'g', 8)};
        for (int column = 0; column < values.size(); ++column)
        {
            auto* cell = new QTableWidgetItem(values.at(column));
            if (column < 3)
                cell->setFlags(cell->flags() & ~Qt::ItemIsEditable);
            materialTable->setItem(row, column, cell);
        }
    }

    auto* sectionTable = dialog.sectionTable();
    sectionTable->setRowCount(m_propertyLibrary->sections().size());
    for (int row = 0; row < m_propertyLibrary->sections().size(); ++row)
    {
        const auto& item = m_propertyLibrary->sections().at(row);
        const QStringList values = {QString::number(item.id),
                                    item.category,
                                    item.name,
                                    QString::number(item.area, 'g', 8),
                                    QString::number(item.area * 1.0e6, 'g', 8),
                                    item.description};
        for (int column = 0; column < values.size(); ++column)
        {
            auto* cell = new QTableWidgetItem(values.at(column));
            if (column != 3)
                cell->setFlags(cell->flags() & ~Qt::ItemIsEditable);
            sectionTable->setItem(row, column, cell);
        }
    }
    dialog.tabs()->setTabText(0, QStringLiteral("材料 (%1)").arg(materialTable->rowCount()));
    dialog.tabs()->setTabText(1, QStringLiteral("截面 (%1)").arg(sectionTable->rowCount()));

    materialTable->setCurrentCell(
        qBound(0, m_conductorModule->materialCombo()->currentIndex(), materialTable->rowCount() - 1), 3);
    sectionTable->setCurrentCell(
        qBound(0, m_conductorModule->sectionCombo()->currentIndex(), sectionTable->rowCount() - 1), 3);

    const auto currentStructure = m_modelController->model();
    auto* syncCurrentModel = dialog.syncCheck();
    auto* targetMaterialCombo = dialog.targetMaterialCombo();
    auto* targetSectionCombo = dialog.targetSectionCombo();
    if (currentStructure)
    {
        for (const auto& [id, material] : currentStructure->m_Material)
            if (material)
                targetMaterialCombo->addItem(QStringLiteral("材料 ID %1").arg(id), id);
        for (const auto& [id, section] : currentStructure->m_Section)
            if (section)
                targetSectionCombo->addItem(QStringLiteral("截面 ID %1").arg(id), id);
    }
    const bool canSync = targetMaterialCombo->count() > 0 && targetSectionCombo->count() > 0;
    syncCurrentModel->setEnabled(canSync);
    targetMaterialCombo->setEnabled(canSync);
    targetSectionCombo->setEnabled(canSync);
    auto* buttons = dialog.buttons();
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    connect(
        buttons, &QDialogButtonBox::accepted, &dialog,
        [this, &dialog, materialTable, sectionTable, syncCurrentModel, targetMaterialCombo, targetSectionCombo,
         currentStructure]()
        {
            QVector<Conductor::MaterialPreset> materials = m_propertyLibrary->materials();
            QVector<Conductor::SectionPreset> sections = m_propertyLibrary->sections();
            for (int row = 0; row < materials.size(); ++row)
            {
                bool okYoung = false, okPoisson = false, okDensity = false, okStress = false, okExpansion = false;
                materials[row].young = materialTable->item(row, 3)->text().trimmed().toDouble(&okYoung);
                materials[row].poisson = materialTable->item(row, 4)->text().trimmed().toDouble(&okPoisson);
                materials[row].density = materialTable->item(row, 5)->text().trimmed().toDouble(&okDensity);
                materials[row].maxStress = materialTable->item(row, 6)->text().trimmed().toDouble(&okStress);
                materials[row].expansion = materialTable->item(row, 7)->text().trimmed().toDouble(&okExpansion);
                if (!okYoung || !okPoisson || !okDensity || !okStress || !okExpansion || materials[row].young <= 0.0 ||
                    materials[row].density <= 0.0 || materials[row].poisson <= -1.0 || materials[row].poisson >= 0.5 ||
                    materials[row].maxStress < 0.0 || materials[row].expansion < 0.0)
                {
                    QMessageBox::warning(
                        &dialog, QStringLiteral("属性无效"),
                        QStringLiteral("材料第 %1 行数据无效。E、密度必须大于 0，泊松比必须在 (-1, 0.5) 内。")
                            .arg(row + 1));
                    return;
                }
            }
            for (int row = 0; row < sections.size(); ++row)
            {
                bool okArea = false;
                sections[row].area = sectionTable->item(row, 3)->text().trimmed().toDouble(&okArea);
                if (!okArea || sections[row].area <= 0.0)
                {
                    QMessageBox::warning(&dialog, QStringLiteral("属性无效"),
                                         QStringLiteral("截面第 %1 行面积必须大于 0。").arg(row + 1));
                    return;
                }
            }

            QString error;
            for (int row = 0; row < materials.size(); ++row)
                if (!m_propertyLibrary->updateMaterial(row, materials.at(row), error))
                    return;
            for (int row = 0; row < sections.size(); ++row)
                if (!m_propertyLibrary->updateSection(row, sections.at(row), error))
                    return;

            bool modelUpdated = false;
            if (syncCurrentModel->isChecked() && currentStructure)
            {
                const int materialRow = materialTable->currentRow();
                const int sectionRow = sectionTable->currentRow();
                const auto materialTarget =
                    currentStructure->m_Material.find(targetMaterialCombo->currentData().toInt());
                const auto sectionTarget = currentStructure->m_Section.find(targetSectionCombo->currentData().toInt());
                if (materialRow >= 0 && sectionRow >= 0 && materialTarget != currentStructure->m_Material.end() &&
                    materialTarget->second && sectionTarget != currentStructure->m_Section.end() &&
                    sectionTarget->second)
                {
                    const auto& sourceMaterial = materials.at(materialRow);
                    materialTarget->second->m_Young = sourceMaterial.young;
                    materialTarget->second->m_Poisson = sourceMaterial.poisson;
                    materialTarget->second->m_Density = sourceMaterial.density;
                    materialTarget->second->m_MaxStress = sourceMaterial.maxStress;
                    materialTarget->second->m_Expansion = sourceMaterial.expansion;
                    sectionTarget->second->m_Area = sections.at(sectionRow).area;
                    if (auto circular = std::dynamic_pointer_cast<SectionCircular>(sectionTarget->second))
                        circular->Calculate_Radius();
                    modelUpdated = true;
                }
            }
            ui.logEdit->appendPlainText(
                modelUpdated ? QStringLiteral("[属性库] 会话预设已保存，并已同步到当前模型（未改写文本文件）")
                             : QStringLiteral("[属性库] 会话预设已保存（未改写文本文件）"));
            refreshModulePages();
            dialog.accept();
        });
    dialog.exec();
}

void YQY::createConductorModel()
{
    const QString generatedDirectory = QDir(importFileDirectory()).absoluteFilePath(QStringLiteral("Generated"));
    if (!m_propertyLibrary)
    {
        QMessageBox::critical(this, QStringLiteral("创建导线模型"), QStringLiteral("启动属性库不可用。"));
        return;
    }
    const ConductorModule::BuildResult result = m_conductorModule->buildModel(*m_propertyLibrary, generatedDirectory);
    if (!result.succeeded())
    {
        QMessageBox::critical(this, QStringLiteral("创建导线模型"), result.error);
        return;
    }

    ui.logEdit->appendPlainText(QStringLiteral("[导线建模] 已生成 %1 个节点、%2 个单元、%3 个相内间隔棒：%4")
                                    .arg(result.nodeCount)
                                    .arg(result.elementCount)
                                    .arg(result.spacerCount)
                                    .arg(result.filePath));
    const int modelId = m_modelController->adoptModel(result.structure, result.filePath);
    if (modelId > 0)
    {
        switchModule(Module::Model);
        setWorkspaceMessage(QStringLiteral("导线模型已生成并加入当前工作区"));
    }
    else
    {
        QMessageBox::warning(this, QStringLiteral("创建导线模型"), QStringLiteral("模型已生成并保存，但未能加入当前工作区。"));
    }
}

void YQY::initializeToolbarAppearance()
{
    const QList<QPair<QAbstractButton*, QString>> buttons = {
        {ui.importButton, QStringLiteral("导入模型")},
        {ui.undoButton, QStringLiteral("撤销上一次节点修改")},
        {ui.selectModeButton, QStringLiteral("选择模式")},
        {ui.rotateModeButton, QStringLiteral("旋转视图")},
        {ui.panModeButton, QStringLiteral("平移视图")},
        {ui.zoomModeButton, QStringLiteral("缩放视图")},
        {ui.showNodesButton, QStringLiteral("显示或隐藏节点")},
        {ui.showElementsButton, QStringLiteral("显示或隐藏单元")},
        {ui.showSolidButton, QStringLiteral("显示或隐藏单元实体")},
        {ui.showNodeIdsButton, QStringLiteral("显示或隐藏节点编号")},
        {ui.showElementIdsButton, QStringLiteral("显示或隐藏单元编号")},
        {ui.frontViewButton, QStringLiteral("前视图")},
        {ui.backViewButton, QStringLiteral("后视图")},
        {ui.leftViewButton, QStringLiteral("左视图")},
        {ui.rightViewButton, QStringLiteral("右视图")},
        {ui.topViewButton, QStringLiteral("顶视图")},
        {ui.bottomViewButton, QStringLiteral("底视图")},
        {ui.fitViewButton, QStringLiteral("适应窗口")},
        {ui.startSolveButton, QStringLiteral("开始求解")}};

    for (const auto& [button, toolTip] : buttons)
    {
        button->setText(QString());
        button->setToolTip(toolTip);
        button->setStatusTip(toolTip);
        button->setAccessibleName(toolTip);
        button->setIconSize(QSize(24, 24));
        button->setFixedSize(QSize(38, 36));
        button->setCursor(Qt::PointingHandCursor);
        button->setAttribute(Qt::WA_Hover, true);
        button->setMouseTracking(true);
        button->installEventFilter(new InstantToolTipFilter(button));
    }
    ui.toolBarLayout->setSpacing(4);

    const QList<QPair<QPushButton*, QString>> navigationButtons = {
        {ui.propertyButton, QStringLiteral("属性")}, {ui.conductorButton, QStringLiteral("导线")},
        {ui.modelButton, QStringLiteral("模型")},    {ui.analysisButton, QStringLiteral("分析")},
        {ui.solveButton, QStringLiteral("求解")},    {ui.resultButton, QStringLiteral("结果")},
        {ui.settingsButton, QStringLiteral("设置")}};
    for (const auto& [button, label] : navigationButtons)
    {
        button->setText(label);
        button->setIconSize(QSize(20, 20));
        button->setCursor(Qt::PointingHandCursor);
        button->setAccessibleName(label + QStringLiteral("模块"));
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    const QList<QPair<QPushButton*, QString>> actionButtons = {
        {ui.modelImportButton, QStringLiteral("导入模型")},
        {ui.modelFitButton, QStringLiteral("适应窗口")},
        {ui.openH5Button, QStringLiteral("打开 H5 结果文件")},
        {ui.solvePageStartButton, QStringLiteral("运行全部分析步")},
        {ui.stopButton, QStringLiteral("停止")}};
    for (const auto& [button, label] : actionButtons)
    {
        button->setText(label);
        button->setIconSize(QSize(20, 20));
        button->setCursor(Qt::PointingHandCursor);
        button->setAccessibleName(label);
        button->setMinimumHeight(qMax(38, fontMetrics().height() + 18));
    }
    setActionGlyph(ui.modelImportButton, ActionGlyph::Import);
    setActionGlyph(ui.modelFitButton, ActionGlyph::Fit);
    setActionGlyph(ui.openH5Button, ActionGlyph::OpenFile);
    setActionGlyph(ui.solvePageStartButton, ActionGlyph::Run);
    setActionGlyph(ui.stopButton, ActionGlyph::Stop);
}

void YQY::initializeResponsiveLayout()
{
    // These values keep the model usable on smaller laptop displays. The
    // previous fixed minimums added up to more than the available width.
    ui.navigationRail->setMinimumWidth(84);
    ui.navigationRail->setMaximumWidth(84);
    // The reference FEM shell reserves about 380 logical pixels for its whole
    // left work area. Our rail is outside the splitter and the property/
    // conductor pages contain longer Chinese labels, so 420 px is the first
    // width at which those pages stay aligned without wasting the viewport.
    ui.projectPanel->setMinimumWidth(320);
    ui.projectPanel->setMaximumWidth(520);
    ui.workspacePanel->setMinimumWidth(300);
    ui.mainSplitter->setHandleWidth(6);
    ui.mainSplitter->setCollapsible(0, false);
    ui.mainSplitter->setCollapsible(1, false);
    ui.mainSplitter->setStretchFactor(0, 0);
    ui.mainSplitter->setStretchFactor(1, 1);
    ui.mainSplitter->setSizes(m_normalMainSplitterSizes);
    ui.modelViewport->setMinimumSize(280, 160);
    ui.monitorTabs->setMinimumHeight(100);
    ui.workspaceVerticalSplitter->setChildrenCollapsible(false);
    ui.workspaceVerticalSplitter->setStretchFactor(0, 4);
    ui.workspaceVerticalSplitter->setStretchFactor(1, 1);
    ui.workspaceVerticalSplitter->setSizes({520, 160});
    connect(ui.mainSplitter, &QSplitter::splitterMoved, this, [this](int, int) { updateResponsiveLayout(); });

    const QList<QPushButton*> navigationButtons = {ui.propertyButton, ui.conductorButton, ui.modelButton,
                                                   ui.analysisButton, ui.solveButton,     ui.resultButton,
                                                   ui.settingsButton};
    for (auto* button : navigationButtons)
    {
        button->setMinimumWidth(66);
        button->setMinimumHeight(qMax(58, fontMetrics().height() + 36));
    }

    const QList<QTreeWidget*> trees = {ui.modelDataTree, ui.analysisTree, ui.solveStepTree, ui.resultTree};
    for (auto* tree : trees)
    {
        tree->setIconSize(QSize(20, 20));
        tree->setTextElideMode(Qt::ElideMiddle);
        tree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        tree->setUniformRowHeights(true);
    }
    ui.modelPropertyValue->setWordWrap(true);

    // Stretch the toolbar background on wide screens while preserving a
    // stable logical button size. Narrow screens scroll horizontally instead
    // of shrinking the buttons or growing the toolbar vertically.
    m_toolbarScrollArea = ui.toolbarScrollArea;
    ui.toolBarLayout->activate();
    m_toolbarNaturalWidth = ui.toolBarCard->sizeHint().width();
    m_toolbarCardHeight = ui.toolBarCard->sizeHint().height();
    ui.toolBarCard->setMinimumWidth(m_toolbarNaturalWidth);
    ui.toolBarCard->setFixedHeight(m_toolbarCardHeight);
    ui.toolBarCard->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_toolbarScrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_toolbarScrollArea->setFixedHeight(m_toolbarCardHeight);

    updateResponsiveLayout();
    QTimer::singleShot(0, this, &YQY::updateResponsiveLayout);
}

void YQY::updateResponsiveLayout()
{
    const int availableWidth = width();
    const int projectWidth = ui.projectPanel->width();

    // The task monitor already exposes status and elapsed time. Give the case
    // name priority as the project pane narrows: hide elapsed time first, then
    // status, instead of squeezing the name down to only a few characters.
    const bool compactSolveTree = projectWidth < 360;
    const bool mediumSolveTree = projectWidth < 480;
    ui.solveStepTree->setColumnHidden(2, compactSolveTree);
    ui.solveStepTree->setColumnHidden(3, mediumSolveTree);
    ui.solveStepTree->setColumnWidth(1, compactSolveTree ? 80 : 92);
    ui.solveStepTree->setColumnWidth(2, 72);
    ui.solveStepTree->setColumnWidth(3, 72);

    if (m_toolbarScrollArea && m_toolbarNaturalWidth > 0 && m_toolbarCardHeight > 0)
    {
        int viewportWidth = m_toolbarScrollArea->viewport()->width();
        if (viewportWidth <= 0)
            viewportWidth = qMax(0, ui.workspacePanel->width() - ui.workspaceLayout->contentsMargins().left() -
                                        ui.workspaceLayout->contentsMargins().right());

        const bool needsHorizontalScroll = viewportWidth < m_toolbarNaturalWidth;
        m_toolbarScrollArea->setHorizontalScrollBarPolicy(needsHorizontalScroll ? Qt::ScrollBarAlwaysOn
                                                                                : Qt::ScrollBarAlwaysOff);
        const int scrollBarHeight = needsHorizontalScroll ? style()->pixelMetric(QStyle::PM_ScrollBarExtent) : 0;
        m_toolbarScrollArea->setFixedHeight(m_toolbarCardHeight + scrollBarHeight);
    }

    ui.themeLabel->setVisible(availableWidth >= 820);
    ui.readyLabel->setVisible(availableWidth >= 900);
    ui.appTitleLabel->setVisible(availableWidth >= 680);
}

void YQY::updateToolbarIcons(int themeIndex)
{
    const ThemeColors colors = themeColors(themeIndex);
    const QColor foreground(colors.text);
    const QColor accent(colors.accentSecond);
    const QList<QPair<QAbstractButton*, QString>> toolbarButtons = {
        {ui.importButton, QStringLiteral(":/YQY/icon_importline.svg")},
        {ui.undoButton, QStringLiteral(":/YQY/icon_replay.svg")},
        {ui.selectModeButton, QStringLiteral(":/YQY/icon_cursor.svg")},
        {ui.rotateModeButton, QStringLiteral(":/YQY/icon_rotate.svg")},
        {ui.panModeButton, QStringLiteral(":/YQY/icon_pan.svg")},
        {ui.zoomModeButton, QStringLiteral(":/YQY/icon_zoom.svg")},
        {ui.showNodesButton, QStringLiteral(":/YQY/icon_node.svg")},
        {ui.showElementsButton, QStringLiteral(":/YQY/icon_element.svg")},
        {ui.showSolidButton, QStringLiteral(":/YQY/icon_solid.svg")},
        {ui.showNodeIdsButton, QStringLiteral(":/YQY/icon_node_id.svg")},
        {ui.showElementIdsButton, QStringLiteral(":/YQY/icon_element_id.svg")},
        {ui.frontViewButton, QStringLiteral(":/YQY/icon_front.svg")},
        {ui.backViewButton, QStringLiteral(":/YQY/icon_back.svg")},
        {ui.leftViewButton, QStringLiteral(":/YQY/icon_left.svg")},
        {ui.rightViewButton, QStringLiteral(":/YQY/icon_right.svg")},
        {ui.topViewButton, QStringLiteral(":/YQY/icon_upper.svg")},
        {ui.bottomViewButton, QStringLiteral(":/YQY/icon_under.svg")},
        {ui.fitViewButton, QStringLiteral(":/YQY/icon_scale.svg")},
        {ui.startSolveButton, QStringLiteral(":/YQY/icon_solve.svg")}};
    for (const auto& [button, resourcePath] : toolbarButtons)
        button->setIcon(QIcon(resourcePath));

    const QList<QPair<NavigationButton*, NavigationGlyph>> navigationButtons = {
        {ui.propertyButton, NavigationGlyph::Properties}, {ui.conductorButton, NavigationGlyph::Conductor},
        {ui.modelButton, NavigationGlyph::Model},         {ui.analysisButton, NavigationGlyph::Analysis},
        {ui.solveButton, NavigationGlyph::Solve},         {ui.resultButton, NavigationGlyph::Results},
        {ui.settingsButton, NavigationGlyph::Settings}};
    for (const auto& [button, glyph] : navigationButtons)
    {
        button->setIcon(navigationIcon(glyph, foreground, accent));
        QPalette palette = button->palette();
        palette.setColor(QPalette::ButtonText, QColor(colors.muted));
        palette.setColor(QPalette::BrightText, foreground);
        button->setPalette(palette);
    }

    applyActionIcons(this, foreground, accent);
}

void YQY::initializeEmptyState()
{
    ui.undoButton->setEnabled(false);
    ui.documentTabBar->hide();
    ui.modelDataTree->clear();
    auto* emptyModel = new QTreeWidgetItem(ui.modelDataTree, {QStringLiteral("尚未加载模型")});
    setTreeGlyph(emptyModel, TreeGlyph::Model);

    ui.showNodesButton->setEnabled(false);
    ui.showElementsButton->setEnabled(false);
    ui.showSolidButton->setEnabled(false);
    ui.showNodeIdsButton->setEnabled(false);
    ui.showElementIdsButton->setEnabled(false);
    ui.frontViewButton->setEnabled(false);
    ui.backViewButton->setEnabled(false);
    ui.leftViewButton->setEnabled(false);
    ui.rightViewButton->setEnabled(false);
    ui.topViewButton->setEnabled(false);
    ui.bottomViewButton->setEnabled(false);
    ui.fitViewButton->setEnabled(false);
    ui.selectModeButton->setEnabled(false);
    ui.rotateModeButton->setEnabled(false);
    ui.panModeButton->setEnabled(false);
    ui.zoomModeButton->setEnabled(false);
    ui.startSolveButton->setEnabled(false);
    ui.stopButton->setEnabled(false);
    ui.solveProgress->setValue(0);
    ui.incrementLabel->setText(QStringLiteral("尚未开始求解"));
    ui.convergenceLabel->setText(QStringLiteral("加载模型并配置分析步后可开始求解"));
    ui.logEdit->clear();
    updateLoadStatistics();
    ui.readyLabel->setText(QStringLiteral("等待导入模型"));
}

void YQY::initializeInteractions()
{
    connect(ui.themeComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &YQY::applyTheme);

    auto* interactionGroup = new QButtonGroup(this);
    interactionGroup->setExclusive(true);
    interactionGroup->addButton(ui.selectModeButton, static_cast<int>(ModelViewport::InteractionMode::Select));
    interactionGroup->addButton(ui.rotateModeButton, static_cast<int>(ModelViewport::InteractionMode::Rotate));
    interactionGroup->addButton(ui.panModeButton, static_cast<int>(ModelViewport::InteractionMode::Pan));
    interactionGroup->addButton(ui.zoomModeButton, static_cast<int>(ModelViewport::InteractionMode::Zoom));
    connect(interactionGroup, &QButtonGroup::idClicked, this,
            [this](int id)
            {
                const auto mode = static_cast<ModelViewport::InteractionMode>(id);
                ui.modelViewport->setInteractionMode(mode);
                const QStringList names = {QStringLiteral("选择"), QStringLiteral("旋转"), QStringLiteral("平移"),
                                           QStringLiteral("缩放")};
                setWorkspaceMessage(QStringLiteral("交互模式：%1").arg(names.at(id)));
            });

    connect(ui.showNodesButton, &QToolButton::toggled, ui.modelViewport, &ModelViewport::setNodesVisible);
    connect(ui.showElementsButton, &QToolButton::toggled, ui.modelViewport, &ModelViewport::setElementsVisible);
    connect(ui.showSolidButton, &QToolButton::toggled, this,
            [this](bool visible)
            {
                // 与参考程序保持一致：实体显示时隐藏会穿插在实体表面的节点、
                // 节点编号及单元轴线；退出实体显示时恢复节点和单元轴线。
                const QSignalBlocker nodesBlocker(ui.showNodesButton);
                const QSignalBlocker nodeIdsBlocker(ui.showNodeIdsButton);
                const QSignalBlocker elementsBlocker(ui.showElementsButton);

                ui.modelViewport->setSolidVisible(visible);
                ui.showNodesButton->setChecked(!visible);
                ui.showNodeIdsButton->setChecked(false);
                ui.showElementsButton->setChecked(!visible);
                ui.modelViewport->setNodesVisible(!visible);
                ui.modelViewport->setNodeLabelsVisible(false);
                ui.modelViewport->setElementsVisible(!visible);

                setWorkspaceMessage(visible ? QStringLiteral("实体模式：已隐藏节点、节点编号和单元轴线")
                                            : QStringLiteral("已退出实体模式"));
            });
    connect(ui.showNodeIdsButton, &QToolButton::toggled, ui.modelViewport, &ModelViewport::setNodeLabelsVisible);
    connect(ui.showElementIdsButton, &QToolButton::toggled, ui.modelViewport, &ModelViewport::setElementLabelsVisible);
    connect(ui.modelViewport, &ModelViewport::nodesVisibilityChanged, ui.showNodesButton, &QToolButton::setChecked);
    connect(ui.modelViewport, &ModelViewport::elementsVisibilityChanged, ui.showElementsButton,
            &QToolButton::setChecked);
    connect(ui.modelViewport, &ModelViewport::solidVisibilityChanged, ui.showSolidButton, &QToolButton::setChecked);
    connect(ui.modelViewport, &ModelViewport::nodeLabelsVisibilityChanged, ui.showNodeIdsButton,
            &QToolButton::setChecked);
    connect(ui.modelViewport, &ModelViewport::elementLabelsVisibilityChanged, ui.showElementIdsButton,
            &QToolButton::setChecked);
    connect(ui.modelViewport, &ModelViewport::interactionModeChanged, this,
            [this](ModelViewport::InteractionMode mode)
            {
                const QList<QAbstractButton*> buttons = {ui.selectModeButton, ui.rotateModeButton, ui.panModeButton,
                                                         ui.zoomModeButton};
                buttons.at(static_cast<int>(mode))->setChecked(true);
            });

    const QList<QPair<QToolButton*, ModelViewport::StandardView>> viewButtons = {
        {ui.frontViewButton, ModelViewport::StandardView::Front},
        {ui.backViewButton, ModelViewport::StandardView::Back},
        {ui.leftViewButton, ModelViewport::StandardView::Left},
        {ui.rightViewButton, ModelViewport::StandardView::Right},
        {ui.topViewButton, ModelViewport::StandardView::Top},
        {ui.bottomViewButton, ModelViewport::StandardView::Bottom}};
    for (const auto& [button, view] : viewButtons)
    {
        connect(button, &QToolButton::clicked, this, [this, view]() { ui.modelViewport->setStandardView(view); });
    }
    connect(ui.fitViewButton, &QToolButton::clicked, ui.modelViewport, &ModelViewport::resetCamera);

    connect(ui.undoButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_modelController->undoLastChange())
                {
                    if (m_selectedNodeId >= 0)
                        showNodeProperties(m_selectedNodeId);
                    setWorkspaceMessage(QStringLiteral("已撤销上一次节点坐标修改"));
                }
            });
    connect(ui.documentTabBar, &QTabBar::tabCloseRequested, this,
            [this](int tabIndex) { m_modelController->closeModel(ui.documentTabBar->tabData(tabIndex).toInt()); });
    connect(ui.documentTabBar, &QTabBar::currentChanged, this,
            [this](int tabIndex)
            {
                if (tabIndex >= 0)
                    m_modelController->setActiveModel(ui.documentTabBar->tabData(tabIndex).toInt());
            });
    const QList<QPair<QPushButton*, Module>> navigationItems = {
        {ui.propertyButton, Module::Properties}, {ui.conductorButton, Module::Conductor},
        {ui.modelButton, Module::Model},         {ui.analysisButton, Module::Analysis},
        {ui.solveButton, Module::Solve},         {ui.resultButton, Module::Result},
        {ui.settingsButton, Module::Settings}};
    for (const auto& item : navigationItems)
    {
        connect(item.first, &QPushButton::clicked, this, [this, module = item.second]() { switchModule(module); });
    }

    connect(ui.modelImportButton, &QPushButton::clicked, ui.importButton, &QPushButton::click);
    connect(ui.modelFitButton, &QPushButton::clicked, ui.modelViewport, &ModelViewport::resetCamera);
    connect(ui.openH5Button, &QPushButton::clicked, this, &YQY::openHdf5Result);
    connect(ui.startSolveButton, &QPushButton::clicked, this, &YQY::submitSolveTask);
    connect(ui.solvePageStartButton, &QPushButton::clicked, this, &YQY::submitSolveTask);
    connect(ui.stopButton, &QPushButton::clicked, this, &YQY::cancelSelectedSolveTask);
    connect(ui.solveStepTree, &QTreeWidget::itemSelectionChanged, this,
            [this]() { updateTaskMonitor(selectedSolveTaskId()); });
    connect(m_solveTaskController, &SolveTaskController::taskAdded, this,
            [this](int taskId)
            {
                refreshSolveTasks(taskId);
                ui.logEdit->appendPlainText(QStringLiteral("[INFO] 已创建待运行算例 %1，后台并发上限 %2")
                                                .arg(taskId)
                                                .arg(m_solveTaskController->maximumThreadCount()));
            });
    connect(m_solveTaskController, &SolveTaskController::taskUpdated, this,
            [this](int taskId)
            {
                updateSolveTaskRow(taskId);
                refreshSolveTaskManager();
                const auto info = m_solveTaskController->taskInfo(taskId);
                if (info.status == SolveTaskController::Status::Completed ||
                    info.status == SolveTaskController::Status::Failed ||
                    info.status == SolveTaskController::Status::Cancelled)
                {
                    const QString level = info.status == SolveTaskController::Status::Completed
                                              ? QStringLiteral("INFO")
                                              : QStringLiteral("WARN");
                    ui.logEdit->appendPlainText(QStringLiteral("[%1] %2：%3").arg(level, info.name, info.message));
                    if (info.hasUsableResult && QFileInfo::exists(info.outputFile))
                    {
                        // 成功、失败或主动停止都预加载本次已经落盘的有效帧；是否进入
                        // 结果模块仍由用户决定。
                        loadHdf5Result(info.outputFile, false, false, info.partialResult);
                    }
                }
            });

    connect(ui.importButton, &QPushButton::clicked, this, [this]()
    {
        ModelImportFileDialog dialog(importFileDirectory(), this);
        if (dialog.exec() != QDialog::Accepted)
            return;
        const QStringList paths = dialog.selectedFiles();
        if (paths.isEmpty())
            return;
        const int accepted = m_modelController->loadModels(paths);
        setWorkspaceMessage(QStringLiteral("已加入 %1 个模型的读取队列").arg(accepted));
    });

    connect(ui.modelViewport, &ModelViewport::nodeSelected, this, &YQY::showNodeProperties);
    connect(ui.modelViewport, &ModelViewport::elementSelected, this, &YQY::showElementProperties);
    connect(ui.modelViewport, &ModelViewport::selectionCleared, this, &YQY::clearSelectionProperties);
    connect(m_modelController, &ModelController::nodePositionChanged, ui.modelViewport,
            &ModelViewport::updateNodePosition);
    connect(m_modelController, &ModelController::loadStarted, this,
            [this](const QString& filePath)
            {
                ++m_modelLoadTotal;
                updateLoadStatistics();
                ui.importButton->setEnabled(false);
                setWorkspaceMessage(QStringLiteral("正在后台读取模型：%1").arg(QFileInfo(filePath).fileName()));
                ui.logEdit->appendPlainText(QStringLiteral("[模型读取] 开始：%1").arg(filePath));
            });
    connect(m_modelController, &ModelController::loadSucceeded, this, &YQY::handleModelLoaded);
    connect(m_modelController, &ModelController::loadFailed, this, &YQY::handleModelLoadFailed);
    connect(m_modelController, &ModelController::activeModelChanged, this, &YQY::handleActiveModelChanged);
    connect(m_modelController, &ModelController::modelClosed, this,
            [this](int modelId)
            {
                const QSignalBlocker blocker(ui.documentTabBar);
                for (int index = 0; index < ui.documentTabBar->count(); ++index)
                {
                    if (ui.documentTabBar->tabData(index).toInt() == modelId)
                    {
                        ui.documentTabBar->removeTab(index);
                        break;
                    }
                }
                m_resultFilesByModelId.remove(modelId);
                refreshModulePages();
            });
    connect(m_modelController, &ModelController::busyChanged, this,
            [this](bool busy) { ui.importButton->setEnabled(!busy); });
    connect(m_modelController, &ModelController::undoAvailabilityChanged, ui.undoButton, &QPushButton::setEnabled);
    connect(m_modelController, &ModelController::modelCleared, this,
            [this]()
            {
                closeAnalysisManagers();
                const QSignalBlocker blocker(ui.documentTabBar);
                while (ui.documentTabBar->count() > 0)
                    ui.documentTabBar->removeTab(0);
                m_resultFilesByModelId.clear();
                ui.modelViewport->clearModel();
                ui.modelDataTree->clear();
                new QTreeWidgetItem(ui.modelDataTree, {QStringLiteral("尚未加载模型")});
                ui.showNodesButton->setChecked(true);
                ui.showElementsButton->setChecked(true);
                ui.showSolidButton->setChecked(false);
                ui.showNodeIdsButton->setChecked(false);
                ui.showElementIdsButton->setChecked(false);
                setModelControlsEnabled(false);
                clearSelectionProperties();
                ui.logEdit->appendPlainText(QStringLiteral("[INFO] 已关闭全部模型"));
                refreshModulePages();
                setWorkspaceMessage(QStringLiteral("未加载模型"));
            });

    switchModule(Module::Model);
}

void YQY::applyTheme(int themeIndex)
{
    themeIndex = qBound(0, themeIndex, 3);
    const ThemeColors colors = themeColors(themeIndex);
    QPalette themedPalette = palette();
    themedPalette.setColor(QPalette::Window, QColor(colors.panel));
    themedPalette.setColor(QPalette::WindowText, QColor(colors.text));
    themedPalette.setColor(QPalette::Base, QColor(colors.field));
    themedPalette.setColor(QPalette::AlternateBase, QColor(colors.elevated));
    themedPalette.setColor(QPalette::Text, QColor(colors.text));
    themedPalette.setColor(QPalette::Button, QColor(colors.elevated));
    themedPalette.setColor(QPalette::ButtonText, QColor(colors.text));
    themedPalette.setColor(QPalette::Highlight, QColor(colors.accentSoft));
    themedPalette.setColor(QPalette::HighlightedText, QColor(colors.text));
    themedPalette.setColor(QPalette::Light, QColor(colors.border));
    themedPalette.setColor(QPalette::Midlight, QColor(colors.border));
    themedPalette.setColor(QPalette::Mid, QColor(colors.borderStrong));
    themedPalette.setColor(QPalette::Dark, QColor(colors.borderStrong));
    themedPalette.setColor(QPalette::Shadow, QColor(colors.rootBackground));
    themedPalette.setColor(QPalette::ToolTipBase, QColor(colors.elevated));
    themedPalette.setColor(QPalette::ToolTipText, QColor(colors.text));
    themedPalette.setColor(QPalette::PlaceholderText, QColor(colors.muted));
    themedPalette.setColor(QPalette::Disabled, QPalette::Text, QColor(colors.muted));
    themedPalette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(colors.muted));
    const QString popupViewStyle =
        QStringLiteral("QAbstractItemView { background: %1; color: %2; border: none; padding: 4px; "
                       "selection-background-color: %3; selection-color: %2; } "
                       "QAbstractItemView::item { background: transparent; color: %2; border: none; "
                       "border-radius: 5px; min-height: 28px; padding: 3px 8px; } "
                       "QAbstractItemView::item:hover, QAbstractItemView::item:selected { "
                       "background: %3; color: %2; }")
            .arg(colors.elevated, colors.text, colors.accentSoft);
    qApp->setProperty(PopupViewStyleProperty, popupViewStyle);
    QColor captionColor(colors.header);
    if (!captionColor.isValid() && themeIndex == 2)
        captionColor = QColor(QStringLiteral("#1b373e"));
    if (!captionColor.isValid())
        captionColor = QColor(colors.panel);
    qApp->setProperty(CaptionColorProperty, captionColor);
    qApp->setProperty(CaptionTextColorProperty, QColor(colors.text));
    qApp->setProperty(WindowBorderColorProperty, QColor(colors.borderStrong));
    // Popups and modal dialogs are separate top-level Qt windows. Applying the
    // theme only to this QMainWindow leaves their frame and selection palette
    // at the Windows defaults, which produces white gutters and system-blue
    // rows inside otherwise dark controls.
    QApplication::setPalette(themedPalette);
    qApp->setStyleSheet(buildStyleSheet(colors));
    for (QWidget* topLevelWidget : QApplication::topLevelWidgets())
        applyWindowsTitleBarTheme(topLevelWidget);
    updateToolbarIcons(themeIndex);
    ui.modelViewport->setThemeIndex(themeIndex);
    const QColor treeColor(colors.text);
    const QColor treeAccent(colors.accentSecond);
    applyTreeIcons(ui.modelDataTree, treeColor, treeAccent);
    applyTreeIcons(ui.analysisTree, treeColor, treeAccent);
    applyTreeIcons(ui.solveStepTree, treeColor, treeAccent);
    applyTreeIcons(ui.resultTree, treeColor, treeAccent);
    if (m_solveCasesButton)
        m_solveCasesButton->setIcon(treeIcon(TreeGlyph::AnalysisSteps, treeColor, treeAccent));
    if (m_solveAllCasesButton)
        m_solveAllCasesButton->setIcon(treeIcon(TreeGlyph::SolveTask, treeColor, treeAccent));
    if (m_solveRestartAllCasesButton)
        m_solveRestartAllCasesButton->setIcon(actionIcon(ActionGlyph::Refresh, treeColor, treeAccent));
    refreshAnalysisEditor();
    QSettings().setValue(QStringLiteral("appearance/theme"), themeIndex);

    const QStringList names = {QStringLiteral("黑色"), QStringLiteral("紫色"), QStringLiteral("绿色"),
                               QStringLiteral("浅色")};
    setWorkspaceMessage(QStringLiteral("已应用 %1 主题").arg(names.at(themeIndex)));
    QTimer::singleShot(0, this, &YQY::updateResponsiveLayout);
}

void YQY::switchModule(Module module)
{
    const int pageIndex = static_cast<int>(module);
    if (pageIndex < 0 || pageIndex >= ui.moduleStack->count())
        return;

    ui.moduleStack->setCurrentIndex(pageIndex);
    QStringList titles = {QStringLiteral("模型数据"), QStringLiteral("分析"), QStringLiteral("求解"),
                          QStringLiteral("结果"), QStringLiteral("设置")};
    titles.insert(1, QStringLiteral("材料与截面属性"));
    titles.insert(2, QStringLiteral("导线建模"));
    ui.projectTitleLabel->setText(titles.at(pageIndex));

    const bool modelEditing = module == Module::Model;
    ui.importButton->setVisible(modelEditing);
    ui.undoButton->setVisible(modelEditing);
    ui.startSolveButton->setVisible(false);
    ui.monitorTabs->setCurrentIndex(module == Module::Solve ? 0 : module == Module::Result ? 2 : 1);

    switch (module)
    {
    case Module::Properties:
        ui.propertyButton->setChecked(true);
        break;
    case Module::Conductor:
        ui.conductorButton->setChecked(true);
        break;
    case Module::Model:
        ui.modelButton->setChecked(true);
        break;
    case Module::Analysis:
        ui.analysisButton->setChecked(true);
        break;
    case Module::Solve:
        ui.solveButton->setChecked(true);
        break;
    case Module::Result:
        ui.resultButton->setChecked(true);
        break;
    case Module::Settings:
        ui.settingsButton->setChecked(true);
        break;
    }

    refreshModulePages();
    setWorkspaceMessage(QStringLiteral("已进入%1模块").arg(titles.at(pageIndex)));
}

void YQY::refreshModulePages()
{
    const auto structure = m_modelController->model();
    const bool hasModel = static_cast<bool>(structure);

    ui.modelFileValue->setText(hasModel ? QFileInfo(m_modelController->filePath()).fileName()
                                        : QStringLiteral("尚未加载模型"));
    ui.modelNodeValue->setText(hasModel ? QStringLiteral("节点 %1").arg(structure->m_Nodes.size())
                                        : QStringLiteral("节点 --"));
    ui.modelElementValue->setText(hasModel ? QStringLiteral("单元 %1").arg(structure->m_Elements.size())
                                           : QStringLiteral("单元 --"));
    ui.modelPropertyValue->setText(
        hasModel
            ? QStringLiteral("材料 %1 · 截面 %2").arg(structure->m_Material.size()).arg(structure->m_Section.size())
            : QStringLiteral("材料 -- · 截面 --"));
    ui.modelFitButton->setEnabled(hasModel);

    ui.analysisTree->clear();
    if (hasModel)
    {
        auto* steps =
            new QTreeWidgetItem(ui.analysisTree, {QStringLiteral("分析步  %1").arg(structure->m_AnalysisStep.size())});
        setTreeGlyph(steps, TreeGlyph::AnalysisSteps);
        for (const auto& [stepId, step] : structure->m_AnalysisStep)
        {
            const QString stepName = step && !step->m_Name.trimmed().isEmpty() ? step->m_Name.trimmed()
                                                                               : QStringLiteral("Step-%1").arg(stepId);
            const QString text =
                QStringLiteral("%1  ·  %2").arg(stepName).arg(step ? step->GetTypeName() : QStringLiteral("UNKNOWN"));
            auto* stepItem = new QTreeWidgetItem(steps, {text});
            setTreeGlyph(stepItem, TreeGlyph::AnalysisStep);
        }
        auto* loads = new QTreeWidgetItem(ui.analysisTree, {QStringLiteral("载荷  %1").arg(structure->m_Load.size())});
        setTreeGlyph(loads, TreeGlyph::Load);
        auto* constraints =
            new QTreeWidgetItem(ui.analysisTree, {QStringLiteral("约束  %1").arg(structure->m_Constraint.size())});
        setTreeGlyph(constraints, TreeGlyph::Constraint);
        auto* regions = new QTreeWidgetItem(ui.analysisTree,
            {QStringLiteral("计算区域  %1").arg(structure->m_ComputeRegions.size())});
        setTreeGlyph(regions, TreeGlyph::Model);
        for (const auto& [regionId, region] : structure->m_ComputeRegions)
        {
            Q_UNUSED(regionId);
            if (!region)
                continue;
            auto* regionItem = new QTreeWidgetItem(regions,
                {QStringLiteral("%1  ·  节点 %2  ·  单元 %3")
                    .arg(region->m_Name).arg(region->m_NodeIds.size()).arg(region->m_ElementIds.size())});
            setTreeGlyph(regionItem, TreeGlyph::Model);
        }
        ui.analysisTree->expandAll();
    }
    else
    {
        auto* emptyAnalysis = new QTreeWidgetItem(ui.analysisTree, {QStringLiteral("尚未加载模型")});
        setTreeGlyph(emptyAnalysis, TreeGlyph::Model);
    }

    const ThemeColors treeColors = themeColors(ui.themeComboBox->currentIndex());
    applyTreeIcons(ui.analysisTree, QColor(treeColors.text), QColor(treeColors.accentSecond));

    const bool canSolve = hasModel && !structure->m_AnalysisStep.empty();
    ui.solveReadinessLabel->setText(
        canSolve   ? QStringLiteral("模型已就绪，共 %1 个分析步").arg(structure->m_AnalysisStep.size())
        : hasModel ? QStringLiteral("当前模型没有可运行的分析步")
                   : QStringLiteral("请先加载包含分析步的模型"));
    ui.solvePageStartButton->setEnabled(canSolve);
    ui.startSolveButton->setEnabled(canSolve);
    refreshSolveTasks();
    refreshAnalysisEditor();
    refreshPropertyModule();
}

void YQY::initializeAnalysisEditor()
{
    m_analysisPanel = new AnalysisSummaryPanel(ui.analysisModulePage);
    m_analysisPanel->stepsButton()->setObjectName(QStringLiteral("analysisStepsButton"));
    m_analysisPanel->loadsButton()->setObjectName(QStringLiteral("analysisLoadsButton"));
    m_analysisPanel->constraintsButton()->setObjectName(QStringLiteral("analysisConstraintsButton"));
    m_analysisPanel->regionsButton()->setObjectName(QStringLiteral("analysisRegionsButton"));
    ui.analysisModuleLayout->insertWidget(1, m_analysisPanel);
    connect(m_analysisPanel->stepsButton(), &QPushButton::clicked, this,
            [this]() { openAnalysisManager(static_cast<int>(AnalysisManagerDialog::Page::Steps)); });
    connect(m_analysisPanel->loadsButton(), &QPushButton::clicked, this,
            [this]() { openAnalysisManager(static_cast<int>(AnalysisManagerDialog::Page::Loads)); });
    connect(m_analysisPanel->constraintsButton(), &QPushButton::clicked, this,
            [this]() { openAnalysisManager(static_cast<int>(AnalysisManagerDialog::Page::Constraints)); });
    connect(m_analysisPanel->regionsButton(), &QPushButton::clicked, this, &YQY::openComputeRegionManager);
    connect(ui.analysisTree, &QTreeWidget::itemDoubleClicked, this,
            [this](QTreeWidgetItem* item, int)
            {
                if (!item)
                    return;
                while (item->parent())
                    item = item->parent();
                const int page = ui.analysisTree->indexOfTopLevelItem(item);
                if (page >= 0 && page <= 2 && m_modelController->model())
                    openAnalysisManager(page);
                else if (page == 3 && m_modelController->model())
                    openComputeRegionManager();
            });
}

void YQY::openComputeRegionManager()
{
    const auto structure = m_modelController->model();
    if (!structure)
        return;
    ComputeRegionManagerDialog manager(structure, this);
    manager.exec();
    QSet<int> affectedStepIds;
    for (const auto& [stepId, step] : structure->m_AnalysisStep)
    {
        if (step)
            affectedStepIds.insert(stepId);
    }
    handleAnalysisResourcesChanged(affectedStepIds);
    refreshModulePages();
}

void YQY::refreshAnalysisEditor()
{
    if (!m_analysisPanel)
        return;
    const auto structure = m_modelController->model();
    const bool enabled = static_cast<bool>(structure);
    auto* stepsButton = m_analysisPanel->stepsButton();
    auto* loadsButton = m_analysisPanel->loadsButton();
    auto* constraintsButton = m_analysisPanel->constraintsButton();
    auto* regionsButton = m_analysisPanel->regionsButton();
    stepsButton->setEnabled(enabled);
    loadsButton->setEnabled(enabled);
    constraintsButton->setEnabled(enabled);
    regionsButton->setEnabled(enabled);
    stepsButton->setText(QStringLiteral("分析步 · %1").arg(structure ? structure->m_AnalysisStep.size() : 0));
    loadsButton->setText(QStringLiteral("荷载 · %1").arg(structure ? structure->m_Load.size() : 0));
    constraintsButton->setText(QStringLiteral("约束 · %1").arg(structure ? structure->m_Constraint.size() : 0));
    regionsButton->setText(QStringLiteral("计算区域 · %1").arg(structure ? structure->m_ComputeRegions.size() : 0));
    const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
    stepsButton->setIcon(treeIcon(TreeGlyph::AnalysisSteps, QColor(colors.text), QColor(colors.accentSecond)));
    loadsButton->setIcon(treeIcon(TreeGlyph::Load, QColor(colors.text), QColor(colors.accentSecond)));
    constraintsButton->setIcon(treeIcon(TreeGlyph::Constraint, QColor(colors.text), QColor(colors.accentSecond)));
    regionsButton->setIcon(treeIcon(TreeGlyph::Model, QColor(colors.text), QColor(colors.accentSecond)));
}

void YQY::openAnalysisManager(int initialPage)
{
    const auto structure = m_modelController->model();
    if (!structure)
    {
        QMessageBox::information(this, QStringLiteral("分析资源管理器"), QStringLiteral("请先加载或创建模型。"));
        return;
    }
    const int boundedPage = qBound(0, initialPage, 2);
    QPointer<AnalysisManagerDialog>* managerSlot = nullptr;
    if (boundedPage == static_cast<int>(AnalysisManagerDialog::Page::Steps))
        managerSlot = &m_analysisStepManager;
    else if (boundedPage == static_cast<int>(AnalysisManagerDialog::Page::Loads))
        managerSlot = &m_analysisLoadManager;
    else
        managerSlot = &m_analysisConstraintManager;

    if (*managerSlot)
    {
        (*managerSlot)->show();
        (*managerSlot)->raise();
        (*managerSlot)->activateWindow();
        return;
    }

    AnalysisManagerDialog* manager = nullptr;
    if (boundedPage == static_cast<int>(AnalysisManagerDialog::Page::Steps))
        manager = new AnalysisStepManagerDialog(structure, this);
    else if (boundedPage == static_cast<int>(AnalysisManagerDialog::Page::Loads))
        manager = new AnalysisLoadManagerDialog(structure, this);
    else
        manager = new AnalysisConstraintManagerDialog(structure, this);

    *managerSlot = manager;
    manager->setAttribute(Qt::WA_DeleteOnClose);
    manager->setWindowModality(Qt::NonModal);
    manager->setModelChangedCallback([this](const QSet<int>& affectedStepIds)
                                     { handleAnalysisResourcesChanged(affectedStepIds); });
    manager->setOpenManagerCallback([this](AnalysisManagerDialog::Page page)
                                    { openAnalysisManager(static_cast<int>(page)); });
    manager->show();
    manager->raise();
    manager->activateWindow();
}

void YQY::handleAnalysisResourcesChanged(const QSet<int>& affectedStepIds)
{
    const auto structure = m_modelController->model();
    if (!structure)
        return;

    const QString sourceFile = m_modelController->filePath();
    m_solveTaskController->removeUnavailablePreparedTasks(structure, sourceFile);
    int preparedCount = 0;
    for (const int stepId : affectedStepIds)
    {
        if (structure->m_AnalysisStep.find(stepId) == structure->m_AnalysisStep.end())
            continue;
        if (m_solveTaskController->prepare(structure, sourceFile, stepId) >= 0)
            ++preparedCount;
    }
    ui.logEdit->appendPlainText(
        QStringLiteral(
            "[INFO] 分析资源已增量更新：影响 %1 个分析步，准备 %2 个算例；当前共 %3 个分析步、%4 个荷载、%5 个约束")
            .arg(affectedStepIds.size())
            .arg(preparedCount)
            .arg(structure->m_AnalysisStep.size())
            .arg(structure->m_Load.size())
            .arg(structure->m_Constraint.size()));
    setWorkspaceMessage(QStringLiteral("分析资源已增量同步，未受影响的计算状态保持不变"));
    refreshModulePages();

    if (m_analysisStepManager)
        m_analysisStepManager->refreshFromModel();
    if (m_analysisLoadManager)
        m_analysisLoadManager->refreshFromModel();
    if (m_analysisConstraintManager)
        m_analysisConstraintManager->refreshFromModel();
}

void YQY::closeAnalysisManagers()
{
    if (m_analysisStepManager)
    {
        auto* manager = m_analysisStepManager.data();
        m_analysisStepManager.clear();
        manager->close();
    }
    if (m_analysisLoadManager)
    {
        auto* manager = m_analysisLoadManager.data();
        m_analysisLoadManager.clear();
        manager->close();
    }
    if (m_analysisConstraintManager)
    {
        auto* manager = m_analysisConstraintManager.data();
        m_analysisConstraintManager.clear();
        manager->close();
    }
}

void YQY::prepareAnalysisCases(const std::shared_ptr<StructureData>& structure, const QString& sourceFile,
                               int onlyStepId)
{
    if (!structure || sourceFile.trimmed().isEmpty())
        return;
    m_solveTaskController->removeUnavailablePreparedTasks(structure, sourceFile);
    if (onlyStepId > 0)
    {
        m_solveTaskController->prepare(structure, sourceFile, onlyStepId);
        return;
    }
    for (const auto& [stepId, step] : structure->m_AnalysisStep)
    {
        if (step)
            m_solveTaskController->prepare(structure, sourceFile, stepId);
    }
}

void YQY::submitSolveTask()
{
    const auto structure = m_modelController->model();
    if (!structure || structure->m_AnalysisStep.empty())
    {
        QMessageBox::information(this, QStringLiteral("无法提交"), QStringLiteral("请先加载包含分析步的模型。"));
        return;
    }

    const int taskId = m_solveTaskController->submit(structure, m_modelController->filePath());
    if (taskId < 0)
    {
        QMessageBox::warning(this, QStringLiteral("提交失败"), QStringLiteral("无法创建独立计算算例。"));
        return;
    }
    switchModule(Module::Solve);
    setWorkspaceMessage(QStringLiteral("算例 %1 已提交，可继续修改参数并提交新算例").arg(taskId));
}

void YQY::cancelSelectedSolveTask()
{
    const int taskId = selectedSolveTaskId();
    if (taskId < 0 || !m_solveTaskController->cancel(taskId))
        return;
    setWorkspaceMessage(QStringLiteral("已请求停止算例 %1").arg(taskId));
}

int YQY::selectedSolveTaskId() const
{
    const auto selected = ui.solveStepTree->selectedItems();
    return selected.isEmpty() ? -1 : selected.first()->data(0, Qt::UserRole).toInt();
}

void YQY::openSolveTaskManager()
{
    if (m_solveTaskManager)
    {
        refreshSolveTaskManager();
        m_solveTaskManager->show();
        m_solveTaskManager->raise();
        m_solveTaskManager->activateWindow();
        return;
    }

    auto* dialog = new SolveTaskManagerDialog(this);
    const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
    auto* solveAllButton = dialog->solveAllButton();
    auto* restartAllButton = dialog->restartAllButton();
    solveAllButton->setIcon(treeIcon(TreeGlyph::SolveTask, QColor(colors.text), QColor(colors.accentSecond)));
    restartAllButton->setIcon(actionIcon(ActionGlyph::Refresh, QColor(colors.text), QColor(colors.accentSecond)));
    solveAllButton->setToolTip(QStringLiteral("将所有待运行分析步加入队列；正在计算、已排队和已有结果的算例不会重复计"
                                               "算；单个分析步失败不会停止其他队列任务"));
    restartAllButton->setToolTip(
        QStringLiteral("重新计算所有分析步；正在排队或计算的任务会先安全停止，再从头加入计算队列"));
    connect(solveAllButton, &QPushButton::clicked, this, &YQY::startAllReadySolveTasks);
    connect(restartAllButton, &QPushButton::clicked, this, &YQY::restartAllSolveTasks);
    m_solveTaskManager = dialog;
    connect(dialog, &QObject::destroyed, this, [this]() { m_solveTaskManager = nullptr; });
    refreshSolveTaskManager();
    dialog->show();
}

void YQY::startAllReadySolveTasks()
{
    const int started = m_solveTaskController->startAllReady();
    setWorkspaceMessage(started > 0 ? QStringLiteral("已将 %1 个分析步加入计算队列 · 最大并发 %2")
                                          .arg(started)
                                          .arg(m_solveTaskController->maximumThreadCount())
                                    : QStringLiteral("没有需要加入队列的待运行分析步"));
    refreshSolveTaskManager();
}

void YQY::restartAllSolveTasks()
{
    const int requested = m_solveTaskController->restartAll();
    setWorkspaceMessage(
        requested > 0
            ? QStringLiteral("已请求重新计算 %1 个分析步 · 活动任务将先安全停止 · 最大并发 %2")
                  .arg(requested)
                  .arg(m_solveTaskController->maximumThreadCount())
            : QStringLiteral("没有可重新计算的分析步"));
    refreshSolveTaskManager();
}

void YQY::refreshSolveTaskManager()
{
    auto* m_solveAllButton = m_solveTaskManager ? m_solveTaskManager->solveAllButton() : nullptr;
    auto* restartAllButton = m_solveTaskManager ? m_solveTaskManager->restartAllButton() : nullptr;
    auto* m_solveTaskTable = m_solveTaskManager ? m_solveTaskManager->taskTable() : nullptr;
    const QList<int> ids = m_solveTaskController->taskIds();
    if (m_solveCasesButton)
        m_solveCasesButton->setText(QStringLiteral("全部分析步 · %1").arg(ids.size()));
    int readyCount = 0;
    for (int id : ids)
    {
        if (m_solveTaskController->taskInfo(id).status == SolveTaskController::Status::Ready)
            ++readyCount;
    }
    if (m_solveAllButton)
    {
        m_solveAllButton->setText(QStringLiteral("全部计算 · %1").arg(readyCount));
        m_solveAllButton->setEnabled(readyCount > 0);
    }
    if (m_solveAllCasesButton)
    {
        m_solveAllCasesButton->setText(QStringLiteral("全部计算 · %1").arg(readyCount));
        m_solveAllCasesButton->setEnabled(readyCount > 0);
    }
    if (restartAllButton)
    {
        restartAllButton->setText(QStringLiteral("重新计算 · %1").arg(ids.size()));
        restartAllButton->setEnabled(!ids.isEmpty());
    }
    if (m_solveRestartAllCasesButton)
    {
        m_solveRestartAllCasesButton->setText(QStringLiteral("重新计算 · %1").arg(ids.size()));
        m_solveRestartAllCasesButton->setEnabled(!ids.isEmpty());
    }
    if (!m_solveTaskManager || !m_solveTaskTable)
        return;

    const QSignalBlocker blocker(m_solveTaskTable);
    m_solveTaskTable->setUpdatesEnabled(false);
    m_solveTaskTable->clearSpans();
    m_solveTaskTable->clearContents();

    if (ids.isEmpty())
    {
        m_solveTaskTable->setRowCount(1);
        auto* emptyItem = new QTableWidgetItem(QStringLiteral("尚未提交算例"));
        emptyItem->setTextAlignment(Qt::AlignCenter);
        emptyItem->setFlags(emptyItem->flags() & ~Qt::ItemIsSelectable);
        m_solveTaskTable->setItem(0, 0, emptyItem);
        m_solveTaskTable->setSpan(0, 0, 1, m_solveTaskTable->columnCount());
        m_solveTaskTable->setUpdatesEnabled(true);
        return;
    }

    m_solveTaskTable->setRowCount(ids.size());
    const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
    for (int row = 0; row < ids.size(); ++row)
    {
        const int id = ids.at(row);
        const auto info = m_solveTaskController->taskInfo(id);
        const bool stoppable =
            info.status == SolveTaskController::Status::Queued || info.status == SolveTaskController::Status::Running;
        const bool stopping = info.status == SolveTaskController::Status::Cancelling;

        auto* action = new QPushButton(stoppable  ? QStringLiteral("停止")
                                       : stopping ? QStringLiteral("停止中")
                                                  : QStringLiteral("运行"),
                                       m_solveTaskTable);
        action->setObjectName(QStringLiteral("solveTaskActionButton"));
        action->setIconSize(QSize(16, 16));
        action->setMinimumWidth(0);
        action->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        action->setCursor(Qt::PointingHandCursor);
        action->setEnabled(!stopping);
        action->setIcon(treeIcon(stoppable || stopping ? TreeGlyph::StopTask : TreeGlyph::SolveTask,
                                 QColor(colors.text), QColor(colors.accentSecond)));
        action->setToolTip(stoppable ? QStringLiteral("停止当前算例") : QStringLiteral("运行当前算例"));
        connect(action, &QPushButton::clicked, this,
                [this, id]()
                {
                    const auto current = m_solveTaskController->taskInfo(id);
                    if (current.status == SolveTaskController::Status::Queued ||
                        current.status == SolveTaskController::Status::Running)
                        m_solveTaskController->cancel(id);
                    else if (current.status != SolveTaskController::Status::Cancelling)
                        m_solveTaskController->start(id);
                });
        m_solveTaskTable->setCellWidget(row, 0, action);

        auto* nameItem = new QTableWidgetItem(info.name);
        nameItem->setToolTip(info.name + QStringLiteral("\n") + info.message);
        m_solveTaskTable->setItem(row, 1, nameItem);

        auto* stepItem = new QTableWidgetItem(info.analysisStepId > 0 ? QString::number(info.analysisStepId)
                                                                      : QStringLiteral("全部"));
        stepItem->setTextAlignment(Qt::AlignCenter);
        m_solveTaskTable->setItem(row, 2, stepItem);

        auto* progress = new QProgressBar(m_solveTaskTable);
        progress->setObjectName(QStringLiteral("solveTaskProgress"));
        progress->setRange(0, 1000);
        progress->setValue(qRound(info.progress * 1000.0));
        progress->setFormat(QStringLiteral("%1%").arg(info.progress * 100.0, 0, 'f', 1));
        m_solveTaskTable->setCellWidget(row, 3, progress);

        auto* statusItem = new QTableWidgetItem(solveTaskDisplayStatus(info));
        statusItem->setTextAlignment(Qt::AlignCenter);
        if (info.status == SolveTaskController::Status::Completed)
            statusItem->setForeground(QColor(QStringLiteral("#2F9E68")));
        else if (info.status == SolveTaskController::Status::Failed)
            statusItem->setForeground(QColor(QStringLiteral("#E05252")));
        else if (info.status == SolveTaskController::Status::Cancelled)
            statusItem->setForeground(QColor(QStringLiteral("#D18B32")));
        m_solveTaskTable->setItem(row, 4, statusItem);

        const bool finished = info.status == SolveTaskController::Status::Completed ||
                              info.status == SolveTaskController::Status::Failed ||
                              info.status == SolveTaskController::Status::Cancelled;
        auto* elapsedItem =
            new QTableWidgetItem(finished ? QStringLiteral("%1 ms").arg(info.elapsedMs) : QStringLiteral("-- ms"));
        elapsedItem->setTextAlignment(Qt::AlignCenter);
        m_solveTaskTable->setItem(row, 5, elapsedItem);
    }
    m_solveTaskTable->setUpdatesEnabled(true);
}

void YQY::refreshSolveTasks(int preferredTaskId)
{
    int selectedId = preferredTaskId >= 0 ? preferredTaskId : selectedSolveTaskId();
    ui.solveStepTree->clear();

    const QList<int> ids = m_solveTaskController->taskIds();
    refreshSolveTaskManager();
    if (ids.isEmpty())
    {
        auto* placeholder =
            new QTreeWidgetItem(ui.solveStepTree, {QStringLiteral("尚未提交算例"), QStringLiteral("0.0%"),
                                                   QStringLiteral("就绪"), QStringLiteral("-- ms")});
        placeholder->setFlags(placeholder->flags() & ~Qt::ItemIsSelectable);
        for (int column = 0; column < ui.solveStepTree->columnCount(); ++column)
            placeholder->setTextAlignment(column, Qt::AlignCenter);
        setTreeGlyph(placeholder, TreeGlyph::SolveTask);
        const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
        applyTreeIcons(ui.solveStepTree, QColor(colors.text), QColor(colors.accentSecond));
        ui.stopButton->setEnabled(false);
        return;
    }

    QTreeWidgetItem* selectedItem = nullptr;
    for (int id : ids)
    {
        const auto info = m_solveTaskController->taskInfo(id);
        auto* item =
            new QTreeWidgetItem(ui.solveStepTree, {QString(), QString(), solveTaskDisplayStatus(info), QString()});
        item->setData(0, Qt::UserRole, id);
        item->setToolTip(0, info.message);
        item->setTextAlignment(2, Qt::AlignCenter);
        item->setTextAlignment(3, Qt::AlignCenter);

        auto* progress = new QProgressBar(ui.solveStepTree);
        progress->setObjectName(QStringLiteral("solveTaskProgress"));
        progress->setRange(0, 1000);
        progress->setValue(qRound(info.progress * 1000.0));
        progress->setTextVisible(true);
        progress->setFormat(QStringLiteral("%1%").arg(info.progress * 100.0, 0, 'f', 1));
        connect(progress, &QProgressBar::valueChanged, progress,
                [progress](int value) { progress->setFormat(QStringLiteral("%1%").arg(value / 10.0, 0, 'f', 1)); });
        ui.solveStepTree->setItemWidget(item, 1, progress);
        auto* caseWidget = new QWidget(ui.solveStepTree);
        caseWidget->setObjectName(QStringLiteral("solveCaseWidget"));
        auto* caseLayout = new QHBoxLayout(caseWidget);
        caseLayout->setContentsMargins(4, 0, 4, 0);
        caseLayout->setSpacing(5);
        caseLayout->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        auto* controlButton = new QToolButton(caseWidget);
        controlButton->setObjectName(QStringLiteral("taskControlButton"));
        controlButton->setIconSize(QSize(18, 18));
        controlButton->setCursor(Qt::PointingHandCursor);
        controlButton->setProperty("taskId", id);
        auto* caseLabel = new QLabel(info.name, caseWidget);
        caseLabel->setObjectName(QStringLiteral("solveCaseLabel"));
        caseLabel->setToolTip(info.name);
        caseLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        caseLabel->setMinimumWidth(0);
        caseLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        caseLayout->addWidget(controlButton);
        caseLayout->addWidget(caseLabel, 1);
        connect(controlButton, &QToolButton::clicked, this,
                [this, id, item]()
                {
                    ui.solveStepTree->setCurrentItem(item);
                    const auto current = m_solveTaskController->taskInfo(id);
                    if (current.status == SolveTaskController::Status::Queued ||
                        current.status == SolveTaskController::Status::Running)
                    {
                        if (m_solveTaskController->cancel(id))
                            setWorkspaceMessage(QStringLiteral("已请求停止算例 %1").arg(id));
                    }
                    else if (current.status == SolveTaskController::Status::Completed ||
                             current.status == SolveTaskController::Status::Failed ||
                             current.status == SolveTaskController::Status::Cancelled ||
                             current.status == SolveTaskController::Status::Ready)
                    {
                        if (m_solveTaskController->start(id))
                            setWorkspaceMessage(QStringLiteral("算例 %1 已加入计算队列").arg(id));
                    }
                });
        ui.solveStepTree->setItemWidget(item, 0, caseWidget);
        item->setSizeHint(0, QSize(0, 40));
        if (id == selectedId)
            selectedItem = item;
        updateSolveTaskRow(id);
    }

    if (!selectedItem)
        selectedItem = ui.solveStepTree->topLevelItem(ui.solveStepTree->topLevelItemCount() - 1);
    ui.solveStepTree->setCurrentItem(selectedItem);
    const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
    applyTreeIcons(ui.solveStepTree, QColor(colors.text), QColor(colors.accentSecond));
    updateTaskMonitor(selectedItem ? selectedItem->data(0, Qt::UserRole).toInt() : -1);

    const int running = m_solveTaskController->runningTaskCount();
    if (running > 0)
        ui.solveReadinessLabel->setText(QStringLiteral("%1 个算例正在运行 · 最多 %2 个并发线程")
                                            .arg(running)
                                            .arg(m_solveTaskController->maximumThreadCount()));
}

void YQY::updateSolveTaskRow(int taskId)
{
    QTreeWidgetItem* item = nullptr;
    for (int row = 0; row < ui.solveStepTree->topLevelItemCount(); ++row)
    {
        auto* candidate = ui.solveStepTree->topLevelItem(row);
        if (candidate && candidate->data(0, Qt::UserRole).toInt() == taskId)
        {
            item = candidate;
            break;
        }
    }
    if (!item)
        return;

    const auto info = m_solveTaskController->taskInfo(taskId);
    auto* progress = qobject_cast<QProgressBar*>(ui.solveStepTree->itemWidget(item, 1));
    if (progress)
        animateProgressBar(progress, qRound(info.progress * 1000.0));

    item->setText(2, solveTaskDisplayStatus(info));
    item->setToolTip(0, info.message);
    const bool finished = info.status == SolveTaskController::Status::Completed ||
                          info.status == SolveTaskController::Status::Failed ||
                          info.status == SolveTaskController::Status::Cancelled;
    item->setText(3, finished ? QStringLiteral("%1 ms").arg(info.elapsedMs) : QStringLiteral("-- ms"));

    QColor statusColor;
    if (info.status == SolveTaskController::Status::Completed)
        statusColor = QColor(QStringLiteral("#2F9E68"));
    else if (info.status == SolveTaskController::Status::Failed)
        statusColor = QColor(QStringLiteral("#E05252"));
    else if (info.status == SolveTaskController::Status::Cancelled)
        statusColor = QColor(QStringLiteral("#D18B32"));
    if (statusColor.isValid())
        item->setForeground(2, statusColor);

    QWidget* caseWidget = ui.solveStepTree->itemWidget(item, 0);
    if (caseWidget)
    {
        auto* controlButton = caseWidget->findChild<QToolButton*>(QStringLiteral("taskControlButton"));
        auto* caseLabel = caseWidget->findChild<QLabel*>(QStringLiteral("solveCaseLabel"));
        if (caseLabel)
        {
            caseLabel->setText(info.name);
            caseLabel->setToolTip(info.name);
        }
        if (!controlButton)
            return;
        const bool stoppable =
            info.status == SolveTaskController::Status::Queued || info.status == SolveTaskController::Status::Running;
        const bool restartable = info.status == SolveTaskController::Status::Ready ||
                                 info.status == SolveTaskController::Status::Completed ||
                                 info.status == SolveTaskController::Status::Failed ||
                                 info.status == SolveTaskController::Status::Cancelled;
        const bool stopping = info.status == SolveTaskController::Status::Cancelling;
        const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
        controlButton->setIcon(treeIcon((stoppable || stopping) ? TreeGlyph::StopTask : TreeGlyph::SolveTask,
                                        QColor(colors.text), QColor(colors.accentSecond)));
        controlButton->setEnabled(stoppable || restartable);
        controlButton->setToolTip(stoppable     ? QStringLiteral("仅停止算例 %1").arg(taskId)
                                  : restartable ? QStringLiteral("开始运行算例 %1").arg(taskId)
                                                : QStringLiteral("算例正在停止"));
        controlButton->setAccessibleName(controlButton->toolTip());
    }

    if (selectedSolveTaskId() == taskId)
        updateTaskMonitor(taskId);

    const int running = m_solveTaskController->runningTaskCount();
    if (running > 0)
        ui.solveReadinessLabel->setText(QStringLiteral("%1 个算例正在运行 · 最多 %2 个并发线程")
                                            .arg(running)
                                            .arg(m_solveTaskController->maximumThreadCount()));
    else
        ui.solveReadinessLabel->setText(QStringLiteral("所有已提交算例均已结束"));
}

void YQY::updateTaskMonitor(int taskId)
{
    if (taskId < 0)
    {
        ui.stopButton->setEnabled(false);
        return;
    }

    const auto info = m_solveTaskController->taskInfo(taskId);
    ui.solveProgress->setRange(0, 1000);
    animateProgressBar(ui.solveProgress, qRound(info.progress * 1000.0));
    ui.solveProgress->setFormat(QStringLiteral("%1%").arg(ui.solveProgress->value() / 10.0, 0, 'f', 1));
    ui.incrementLabel->setText(QStringLiteral("%1 · %2 · %3 ms")
                                   .arg(info.name, SolveTaskController::statusText(info.status))
                                   .arg(info.elapsedMs));
    ui.convergenceLabel->setText(info.message);
    const bool active =
        info.status == SolveTaskController::Status::Queued || info.status == SolveTaskController::Status::Running;
    ui.stopButton->setEnabled(active);
}

void YQY::openHdf5Result()
{
    const QString initialDirectory = hdf5ResultDirectory();

    ModelImportFileDialog dialog(initialDirectory, QStringLiteral("打开 H5 结果文件"),
                                 QStringLiteral("HDF5 结果文件 (*.h5 *.hdf5);;所有文件 (*.*)"),
                                 QFileDialog::ExistingFiles, this);

    if (dialog.exec() != QDialog::Accepted)
        return;

    const QStringList selectedFiles = dialog.selectedFiles();
    if (selectedFiles.isEmpty())
        return;

    int loadedCount = 0;
    QStringList failedFiles;
    for (const QString& filePath : selectedFiles)
    {
        if (loadHdf5Result(filePath, false))
            ++loadedCount;
        else
            failedFiles.append(QFileInfo(filePath).fileName());
    }

    if (!failedFiles.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("部分 H5 文件未能打开"),
                             QStringLiteral("以下文件没有有效的模型或结果数据：\n%1")
                                 .arg(failedFiles.join(QLatin1Char('\n'))));
    }
    if (loadedCount > 0)
        setWorkspaceMessage(QStringLiteral("已打开 %1 个 H5 结果文件").arg(loadedCount));
}

void YQY::exportNodeResults()
{
    auto* m_resultPlayButton = m_resultPanel->playButton();
    auto* m_resultFrameSlider = m_resultPanel->frameSlider();
    auto* m_exportNodeResultsButton = m_resultPanel->exportButton();
    const auto& m_resultFrames = m_resultPanel->frames();
    const QString& m_resultFilePath = m_resultPanel->resultFilePath();
    const auto structure = m_modelController->model();
    if (!structure || m_resultFilePath.isEmpty() || m_resultFrames.empty())
    {
        QMessageBox::information(this, QStringLiteral("尚无可导出结果"),
                                 QStringLiteral("请先加载模型及其 H5 结果文件。"));
        return;
    }

    QSet<int> availableNodes;
    for (const auto& [nodeId, node] : structure->m_Nodes)
    {
        if (node)
            availableNodes.insert(nodeId);
    }

    NodeResultExportDialog dialog(m_resultFilePath, availableNodes, m_selectedNodeId, this);
    if (dialog.exec() != QDialog::Accepted)
        return;

    const std::vector<int> nodeIds = dialog.nodeIds();
    const std::vector<EnumKeyword::NodeResultType> resultTypes = dialog.resultTypes();
    QString outputFile = dialog.outputFile();
    if (QFileInfo(outputFile).suffix().isEmpty())
        outputFile += QStringLiteral(".bdf");

    if (QFileInfo::exists(outputFile) &&
        QMessageBox::question(this, QStringLiteral("替换已有文件"),
                              QStringLiteral("文件已经存在，是否替换？\n%1").arg(outputFile),
                              QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
    {
        return;
    }

    m_resultPanel->stopPlayback();
    m_resultPlayButton->setEnabled(false);
    m_resultFrameSlider->setEnabled(false);
    m_exportNodeResultsButton->setEnabled(false);
    m_exportNodeResultsButton->setText(QStringLiteral("正在导出…"));
    setWorkspaceMessage(QStringLiteral("正在导出 %1 个节点的时程数据…").arg(nodeIds.size()));

    const QString sourceFile = m_resultFilePath;
    auto* watcher = new QFutureWatcher<bool>(this);
    connect(watcher, &QFutureWatcher<bool>::finished, this,
            [this, watcher, outputFile, m_resultPlayButton, m_resultFrameSlider, m_exportNodeResultsButton,
             nodeCount = nodeIds.size(), fieldCount = resultTypes.size()]()
            {
                const bool succeeded = watcher->result();
                watcher->deleteLater();
                m_exportNodeResultsButton->setText(QStringLiteral("导出节点数据…"));
                const auto& frames = m_resultPanel->frames();
                m_exportNodeResultsButton->setEnabled(!m_resultPanel->resultFilePath().isEmpty() && !frames.empty());
                m_resultFrameSlider->setEnabled(!frames.empty());
                m_resultPlayButton->setEnabled(frames.size() > 1);

                if (succeeded)
                {
                    ui.logEdit->appendPlainText(QStringLiteral("[INFO] 节点结果已导出：%1 个节点，%2 个分量 → %3")
                                                    .arg(nodeCount)
                                                    .arg(fieldCount)
                                                    .arg(outputFile));
                    setWorkspaceMessage(QStringLiteral("节点结果导出完成 · %1").arg(QFileInfo(outputFile).fileName()));
                }
                else
                {
                    setWorkspaceMessage(QStringLiteral("节点结果导出失败"));
                    QMessageBox::critical(this, QStringLiteral("导出失败"),
                                          QStringLiteral("无法导出节点结果，请检查结果文件和保存路径。"));
                }
            });
    watcher->setFuture(QtConcurrent::run(
        [sourceFile, outputFile, nodeIds, resultTypes]()
        {
            Hdf5ModelIO exporter;
            return exporter.ExportBdfResultFromHdf5(sourceFile, outputFile, nodeIds, resultTypes, {}, {});
        }));
}

bool YQY::loadHdf5Result(const QString& filePath, bool showErrors, bool activateResultModule, bool partialResult)
{
    auto* m_resultPlayButton = m_resultPanel->playButton();
    auto* m_resultFrameSlider = m_resultPanel->frameSlider();
    auto* m_exportNodeResultsButton = m_resultPanel->exportButton();
    auto* m_resultReader = m_resultPanel->reader();
    auto& m_resultFrames = m_resultPanel->frames();
    auto& m_resultFilePath = m_resultPanel->resultFilePath();
    auto& m_resultIsPartial = m_resultPanel->partialResult();
    auto& m_resultAutomaticScale = m_resultPanel->automaticScale();
    Hdf5ResultSummary summary;
    if (!m_resultReader->InspectHdf5(filePath, summary) || !summary.hasResult)
    {
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 读取失败"),
                                  QStringLiteral("文件中没有有效的 YQY 计算结果：\n%1").arg(filePath));
        return false;
    }
    partialResult = partialResult || summary.partialResult;
    std::vector<Hdf5ResultFrameInfo> indexedFrames;
    if (!m_resultReader->OpenResultFile(filePath, indexedFrames) || indexedFrames.empty())
    {
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 索引失败"),
                                  QStringLiteral("无法建立结果帧索引：\n%1").arg(filePath));
        return false;
    }

    if (!summary.hasModel)
    {
        m_resultReader->CloseResultFile();
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 模型缺失"),
                                  QStringLiteral("该 H5 文件不包含模型数据，无法可靠显示对应结果：\n%1")
                                      .arg(filePath));
        return false;
    }

    QElapsedTimer modelTimer;
    modelTimer.start();
    auto embeddedModel = std::make_shared<StructureData>();
    Hdf5ModelIO modelReader;
    if (!modelReader.ImportHdf5(filePath, embeddedModel.get()))
    {
        m_resultReader->CloseResultFile();
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 模型读取失败"),
                                  QStringLiteral("无法从 H5 文件恢复计算时使用的模型：\n%1").arg(filePath));
        return false;
    }

    const bool shouldActivateEmbeddedModel = activateResultModule
        || !modelsMatchForResults(m_modelController->model(), embeddedModel);
    if (shouldActivateEmbeddedModel
        && m_modelController->adoptModel(embeddedModel, filePath, modelTimer.elapsed()) < 0)
    {
        m_resultReader->CloseResultFile();
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 模型激活失败"),
                                  QStringLiteral("H5 模型已经读取，但无法切换为当前显示模型。"));
        return false;
    }

    m_resultFilePath = QFileInfo(filePath).absoluteFilePath();
    m_resultFilesByModelId.insert(m_modelController->activeModelId(), m_resultFilePath);
    m_resultIsPartial = partialResult;
    m_resultFrames = std::move(indexedFrames);
    if (!m_resultReader->ReadResultRanges(m_resultRanges))
        m_resultRanges = {};
    m_cachedResultFrameIndex = -1;
    m_cachedNextResultFrameIndex = -1;
    Hdf5ResultFrame scaleReferenceFrame;
    if (m_resultReader->ReadResultFrame(static_cast<int>(m_resultFrames.size()) - 1, scaleReferenceFrame) &&
        ui.modelViewport->displayResultFrame(scaleReferenceFrame, ModelViewport::ResultField::DisplacementMagnitude,
                                             0.0, false))
    {
        m_resultAutomaticScale = ui.modelViewport->activeDeformationScale();
    }
    else
        m_resultAutomaticScale = 1.0;
    m_resultPanel->stopPlayback();
    m_resultFrameSlider->setEnabled(true);
    m_resultFrameSlider->setRange(0, static_cast<int>(m_resultFrames.size()) - 1);
    m_resultPlayButton->setEnabled(m_resultFrames.size() > 1);
    m_exportNodeResultsButton->setEnabled(true);

    ui.resultFileLabel->setText(partialResult ? QStringLiteral("%1 · 部分结果").arg(QFileInfo(filePath).fileName())
                                              : QFileInfo(filePath).fileName());
    ui.resultTree->clear();
    auto* root = new QTreeWidgetItem(
        ui.resultTree, {partialResult ? QStringLiteral("部分结果（计算未完成）") : QStringLiteral("结果文件")});
    setTreeGlyph(root, TreeGlyph::ResultFile);
    auto* modelData = new QTreeWidgetItem(
        root, {summary.hasModel ? QStringLiteral("模型数据  可用") : QStringLiteral("模型数据  无")});
    setTreeGlyph(modelData, TreeGlyph::Model);
    const double resultEndTime = m_resultFrames.empty() ? 0.0 : m_resultFrames.back().time;
    auto* framesItem = new QTreeWidgetItem(
        root,
        {partialResult
             ? QStringLiteral("有效结果帧  %1 · 截止 t=%2 s").arg(summary.frameCount).arg(resultEndTime, 0, 'g', 10)
             : QStringLiteral("结果帧  %1").arg(summary.frameCount)});
    setTreeGlyph(framesItem, TreeGlyph::ResultFrames);
    auto* nodal = new QTreeWidgetItem(root, {QStringLiteral("节点结果")});
    setTreeGlyph(nodal, TreeGlyph::Nodes);
    auto* displacement =
        new QTreeWidgetItem(nodal, {QStringLiteral("位移记录  %1").arg(summary.displacementRecordCount)});
    setTreeGlyph(displacement, TreeGlyph::Displacement);
    auto* elemental = new QTreeWidgetItem(root, {QStringLiteral("单元结果")});
    setTreeGlyph(elemental, TreeGlyph::Elements);
    auto* stress = new QTreeWidgetItem(elemental, {QStringLiteral("应力记录  %1").arg(summary.stressRecordCount)});
    setTreeGlyph(stress, TreeGlyph::Stress);
    auto* strain = new QTreeWidgetItem(elemental, {QStringLiteral("应变记录  %1").arg(summary.strainRecordCount)});
    setTreeGlyph(strain, TreeGlyph::Strain);
    const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
    applyTreeIcons(ui.resultTree, QColor(colors.text), QColor(colors.accentSecond));
    ui.resultTree->expandAll();
    ui.logEdit->appendPlainText(QStringLiteral("[%1] 已加载%2 H5 后处理结果：%3，%4 帧，截止 t=%5 s")
                                    .arg(partialResult ? QStringLiteral("WARN") : QStringLiteral("INFO"),
                                         partialResult ? QStringLiteral("部分") : QString(), filePath)
                                    .arg(summary.frameCount)
                                    .arg(resultEndTime, 0, 'g', 10));
    if (shouldActivateEmbeddedModel)
    {
        ui.logEdit->appendPlainText(QStringLiteral("[INFO] 已切换到 H5 内嵌模型：节点 %1，单元 %2")
                                        .arg(embeddedModel->m_Nodes.size())
                                        .arg(embeddedModel->m_Elements.size()));
    }
    if (activateResultModule)
        switchModule(Module::Result);
    {
        const QSignalBlocker blocker(m_resultFrameSlider);
        m_resultFrameSlider->setValue(0);
    }
    displayResultFrame(0);
    setWorkspaceMessage(partialResult ? QStringLiteral("部分结果可用 · %1 帧 · 截止 t=%2 s")
                                            .arg(summary.frameCount)
                                            .arg(resultEndTime, 0, 'g', 10)
                                      : QStringLiteral("后处理已就绪 · %1 帧").arg(summary.frameCount));
    return true;
}

void YQY::displayResultFrame(int frameIndex)
{
    displayResultPosition(static_cast<double>(frameIndex));
}

bool YQY::cacheResultFramePair(int firstIndex, int secondIndex)
{
    auto* resultReader = m_resultPanel->reader();
    if (m_cachedResultFrameIndex != firstIndex)
    {
        if (m_cachedNextResultFrameIndex == firstIndex)
        {
            std::swap(m_cachedResultFrameIndex, m_cachedNextResultFrameIndex);
            std::swap(m_cachedResultFrame, m_cachedNextResultFrame);
        }
        else if (!resultReader->ReadResultFrame(firstIndex, m_cachedResultFrame))
        {
            return false;
        }
        m_cachedResultFrameIndex = firstIndex;
    }

    if (m_cachedNextResultFrameIndex != secondIndex)
    {
        if (!resultReader->ReadResultFrame(secondIndex, m_cachedNextResultFrame))
            return false;
        m_cachedNextResultFrameIndex = secondIndex;
    }
    return true;
}

void YQY::displayResultPosition(double framePosition)
{
    auto* m_resultFieldCombo = m_resultPanel->fieldCombo();
    auto* m_resultScaleSpin = m_resultPanel->scaleSpin();
    auto* m_resultOriginalCheck = m_resultPanel->originalCheck();
    auto* m_resultFrameLabel = m_resultPanel->frameLabel();
    auto* m_resultTimeLabel = m_resultPanel->timeValueLabel();
    auto* m_resultDeformationLabel = m_resultPanel->deformationValueLabel();
    const auto& m_resultFrames = m_resultPanel->frames();
    const bool m_resultIsPartial = m_resultPanel->partialResult();
    const double m_resultAutomaticScale = m_resultPanel->automaticScale();
    if (m_resultFrames.empty())
        return;

    const double frameCount = static_cast<double>(m_resultFrames.size());
    framePosition = std::fmod(std::max(0.0, framePosition), frameCount);
    const int frameIndex = static_cast<int>(std::floor(framePosition));
    const int nextFrameIndex = (frameIndex + 1) % static_cast<int>(m_resultFrames.size());
    const double interpolation = framePosition - static_cast<double>(frameIndex);
    if (!cacheResultFramePair(frameIndex, nextFrameIndex))
    {
        m_resultFrameLabel->setText(QStringLiteral("第 %1 帧读取失败").arg(frameIndex + 1));
        m_resultTimeLabel->setText(QStringLiteral("--"));
        m_resultDeformationLabel->setText(QStringLiteral("--"));
        return;
    }
    Hdf5ResultFrame frame = interpolation <= 1.0e-9
        ? m_cachedResultFrame
        : interpolateResultFrames(m_cachedResultFrame, m_cachedNextResultFrame, interpolation);
    const auto field = static_cast<ModelViewport::ResultField>(m_resultFieldCombo->currentIndex());
    const Hdf5ResultRange& scalarRange = resultRangeForField(m_resultRanges, field);
    if (scalarRange.valid)
        ui.modelViewport->setResultScalarRange(scalarRange.minimum, scalarRange.maximum);
    else
        ui.modelViewport->clearResultScalarRange();
    const double requestedScale =
        m_resultScaleSpin->value() <= 0.0 ? m_resultAutomaticScale : m_resultScaleSpin->value();
    if (!ui.modelViewport->displayResultFrame(frame, field, requestedScale, m_resultOriginalCheck->isChecked()))
        return;
    const QString scale = m_resultScaleSpin->value() <= 0.0
                              ? QStringLiteral("自动 %1×").arg(m_resultAutomaticScale, 0, 'g', 4)
                              : QStringLiteral("%1×").arg(m_resultScaleSpin->value(), 0, 'g', 6);
    m_resultFrameLabel->setText(QStringLiteral("%1 / %2%3")
                                    .arg(frameIndex + 1)
                                    .arg(m_resultFrames.size())
                                    .arg(m_resultIsPartial ? QStringLiteral("  ·  部分结果") : QString()));
    m_resultTimeLabel->setText(QStringLiteral("%1 s").arg(frame.info.time, 0, 'g', 8));
    m_resultDeformationLabel->setText(scale);
}

void YQY::updateResultVisualization()
{
    if (m_resultPanel && !m_resultPanel->frames().empty())
        displayResultFrame(m_resultPanel->frameSlider()->value());
}

bool YQY::loadModelFile(const QString& filePath)
{
    if (!m_modelController->loadModel(filePath))
    {
        if (m_modelController->isLoading())
            setWorkspaceMessage(QStringLiteral("已有模型正在读取，请稍候"));
        return false;
    }
    return true;
}

void YQY::handleModelLoaded(int modelId, const QString& filePath, qint64 elapsedMs)
{
    const auto structure = m_modelController->model(modelId);
    if (!structure)
        return;

    {
        const QSignalBlocker blocker(ui.documentTabBar);
        const int tabIndex = ui.documentTabBar->addTab(QFileInfo(filePath).fileName());
        ui.documentTabBar->setTabData(tabIndex, modelId);
        ui.documentTabBar->setTabToolTip(tabIndex, filePath);
    }

    ++m_modelLoadSucceeded;
    updateLoadStatistics();
    ui.logEdit->appendPlainText(QStringLiteral("[模型读取] 成功：%1").arg(filePath));
    ui.logEdit->appendPlainText(QStringLiteral("           数据：节点 %1，单元 %2，材料 %3，截面 %4；耗时 %5 ms")
                                    .arg(structure->m_Nodes.size())
                                    .arg(structure->m_Elements.size())
                                    .arg(structure->m_Material.size())
                                    .arg(structure->m_Section.size())
                                    .arg(elapsedMs));

    const QString suffix = QFileInfo(filePath).suffix();
    if (suffix.compare(QStringLiteral("h5"), Qt::CaseInsensitive) != 0
        && suffix.compare(QStringLiteral("hdf5"), Qt::CaseInsensitive) != 0)
    {
        prepareAnalysisCases(structure, filePath);
    }

    setWorkspaceMessage(QStringLiteral("模型加载完成 · %1 个节点 · %2 个单元 · %3 ms")
                            .arg(structure->m_Nodes.size())
                            .arg(structure->m_Elements.size())
                            .arg(elapsedMs));
}

void YQY::handleActiveModelChanged(int modelId)
{
    closeAnalysisManagers();
    const auto structure = m_modelController->model(modelId);
    if (!structure)
        return;

    {
        const QSignalBlocker blocker(ui.documentTabBar);
        for (int index = 0; index < ui.documentTabBar->count(); ++index)
        {
            if (ui.documentTabBar->tabData(index).toInt() == modelId)
            {
                ui.documentTabBar->setCurrentIndex(index);
                break;
            }
        }
    }

    m_selectedNodeId = -1;
    ui.modelViewport->displayModel(structure);
    populateModelDataTree(QFileInfo(m_modelController->filePath(modelId)).fileName());
    setModelControlsEnabled(true);
    clearSelectionProperties();
    refreshModulePages();
    setWorkspaceMessage(QStringLiteral("当前模型：%1 · 共打开 %2 个模型")
                            .arg(QFileInfo(m_modelController->filePath(modelId)).fileName())
                            .arg(m_modelController->modelCount()));

    const QString resultFilePath = m_resultFilesByModelId.value(modelId);
    if (!resultFilePath.isEmpty() && QFileInfo(resultFilePath) != QFileInfo(m_resultPanel->resultFilePath()))
    {
        QTimer::singleShot(0, this, [this, modelId, resultFilePath]()
        {
            if (m_modelController->activeModelId() == modelId
                && QFileInfo(resultFilePath) != QFileInfo(m_resultPanel->resultFilePath()))
            {
                loadHdf5Result(resultFilePath, false, false);
            }
        });
    }
}

void YQY::handleModelLoadFailed(const QString& filePath, const QString& errorMessage)
{
    ++m_modelLoadFailed;
    updateLoadStatistics();
    QMessageBox::critical(this, QStringLiteral("模型读取失败"),
                          QStringLiteral("无法读取文件：\n%1\n\n%2").arg(filePath, errorMessage));
    ui.logEdit->appendPlainText(QStringLiteral("[模型读取] 失败：%1").arg(filePath));
    ui.logEdit->appendPlainText(QStringLiteral("           原因：%1").arg(errorMessage));
    setWorkspaceMessage(QStringLiteral("模型读取失败：%1").arg(QFileInfo(filePath).fileName()));
}

void YQY::populateModelDataTree(const QString& fileName)
{
    const auto structure = m_modelController->model();
    ui.modelDataTree->clear();
    if (!structure)
    {
        auto* emptyModel = new QTreeWidgetItem(ui.modelDataTree, {QStringLiteral("尚未加载模型")});
        setTreeGlyph(emptyModel, TreeGlyph::Model);
        const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
        applyTreeIcons(ui.modelDataTree, QColor(colors.text), QColor(colors.accentSecond));
        return;
    }

    auto* model = new QTreeWidgetItem(ui.modelDataTree, {fileName});
    setTreeGlyph(model, TreeGlyph::Model);
    auto* nodes = new QTreeWidgetItem(model, {QStringLiteral("节点  %1").arg(structure->m_Nodes.size())});
    setTreeGlyph(nodes, TreeGlyph::Nodes);
    auto* elements = new QTreeWidgetItem(model, {QStringLiteral("单元  %1").arg(structure->m_Elements.size())});
    setTreeGlyph(elements, TreeGlyph::Elements);
    auto* materials = new QTreeWidgetItem(model, {QStringLiteral("材料  %1").arg(structure->m_Material.size())});
    setTreeGlyph(materials, TreeGlyph::Material);
    auto* sections = new QTreeWidgetItem(model, {QStringLiteral("截面  %1").arg(structure->m_Section.size())});
    setTreeGlyph(sections, TreeGlyph::Section);

    int trussCount = 0;
    int cableCount = 0;
    int beamCount = 0;
    int otherCount = 0;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        Q_UNUSED(elementId);
        if (dynamic_cast<ElementTruss*>(element.get()))
            ++trussCount;
        else if (dynamic_cast<ElementCable*>(element.get()))
            ++cableCount;
        else if (dynamic_cast<ElementBeam_CR*>(element.get()))
            ++beamCount;
        else
            ++otherCount;
    }

    auto* elementTypes = new QTreeWidgetItem(model, {QStringLiteral("单元类型")});
    setTreeGlyph(elementTypes, TreeGlyph::ElementTypes);
    if (trussCount > 0)
    {
        auto* item = new QTreeWidgetItem(elementTypes, {QStringLiteral("T3D2  %1").arg(trussCount)});
        setTreeGlyph(item, TreeGlyph::Truss);
    }
    if (cableCount > 0)
    {
        auto* item = new QTreeWidgetItem(elementTypes, {QStringLiteral("CABLE  %1").arg(cableCount)});
        setTreeGlyph(item, TreeGlyph::Cable);
    }
    if (beamCount > 0)
    {
        auto* item = new QTreeWidgetItem(elementTypes, {QStringLiteral("CR3D  %1").arg(beamCount)});
        setTreeGlyph(item, TreeGlyph::Beam);
    }
    if (otherCount > 0)
    {
        auto* item = new QTreeWidgetItem(elementTypes, {QStringLiteral("OTHER  %1").arg(otherCount)});
        setTreeGlyph(item, TreeGlyph::Other);
    }

    const ThemeColors colors = themeColors(ui.themeComboBox->currentIndex());
    applyTreeIcons(ui.modelDataTree, QColor(colors.text), QColor(colors.accentSecond));

    ui.modelDataTree->expandItem(model);
    ui.modelDataTree->expandItem(elementTypes);
    ui.modelDataTree->setCurrentItem(model);
}

void YQY::setModelControlsEnabled(bool enabled)
{
    const auto structure = m_modelController->model();
    enabled = enabled && static_cast<bool>(structure);
    ui.modelDataTree->setEnabled(enabled);

    ui.documentTabBar->setVisible(enabled);

    ui.showNodesButton->setEnabled(enabled);
    ui.showElementsButton->setEnabled(enabled);
    ui.showSolidButton->setEnabled(enabled && ui.modelViewport->hasSolidGeometry());
    ui.showNodeIdsButton->setEnabled(enabled);
    ui.showElementIdsButton->setEnabled(enabled);
    ui.frontViewButton->setEnabled(enabled);
    ui.backViewButton->setEnabled(enabled);
    ui.leftViewButton->setEnabled(enabled);
    ui.rightViewButton->setEnabled(enabled);
    ui.topViewButton->setEnabled(enabled);
    ui.bottomViewButton->setEnabled(enabled);
    ui.fitViewButton->setEnabled(enabled);
    ui.selectModeButton->setEnabled(enabled);
    ui.rotateModeButton->setEnabled(enabled);
    ui.panModeButton->setEnabled(enabled);
    ui.zoomModeButton->setEnabled(enabled);
    ui.startSolveButton->setEnabled(false);
    ui.stopButton->setEnabled(false);
}

void YQY::showNodeProperties(int nodeId)
{
    const auto structure = m_modelController->model();
    if (!structure)
        return;
    const auto node = structure->FindNode(nodeId);
    if (!node)
        return;

    m_selectedNodeId = nodeId;
    QStringList dofIds;
    for (const int dof : node->m_DOF)
        dofIds.append(QString::number(dof));

    ui.monitorTabs->setCurrentWidget(ui.logPage);
    ui.logEdit->appendPlainText(QStringLiteral("\n[选择] 节点 %1\n"
                                               "  坐标：X = %2，Y = %3，Z = %4\n"
                                               "  自由度：%5")
                                    .arg(nodeId)
                                    .arg(node->m_X, 0, 'g', 12)
                                    .arg(node->m_Y, 0, 'g', 12)
                                    .arg(node->m_Z, 0, 'g', 12)
                                    .arg(dofIds.isEmpty() ? QStringLiteral("--") : dofIds.join(QStringLiteral(", "))));
    setWorkspaceMessage(QStringLiteral("已选择节点 %1").arg(nodeId));
}

void YQY::showElementProperties(int elementId)
{
    const auto structure = m_modelController->model();
    if (!structure)
        return;
    const auto element = structure->FindElement(elementId);
    if (!element)
        return;

    QString typeName = QStringLiteral("UNKNOWN");
    if (dynamic_cast<ElementTruss*>(element.get()))
        typeName = QStringLiteral("T3D2");
    else if (dynamic_cast<ElementCable*>(element.get()))
        typeName = QStringLiteral("CABLE");
    else if (dynamic_cast<ElementBeam_CR*>(element.get()))
        typeName = QStringLiteral("CR3D");

    QStringList nodeIds;
    for (const auto& weakNode : element->m_pNode)
    {
        const auto node = weakNode.lock();
        if (node)
            nodeIds << QString::number(node->m_Id);
    }

    QString propertyId = QStringLiteral("--");
    QString materialId = QStringLiteral("--");
    QString sectionId = QStringLiteral("--");
    if (const auto property = element->m_pProperty.lock())
    {
        propertyId = QString::number(property->m_Id);
        if (const auto material = property->m_pMaterial.lock())
            materialId = QString::number(material->m_Id);
        if (const auto section = property->m_pSection.lock())
            sectionId = QString::number(section->m_Id);
    }

    m_selectedNodeId = -1;
    ui.monitorTabs->setCurrentWidget(ui.logPage);
    ui.logEdit->appendPlainText(QStringLiteral("\n[选择] 单元 %1\n"
                                               "  类型：%2\n"
                                               "  关联节点：%3\n"
                                               "  属性：%4，材料：%5，截面：%6\n"
                                               "  初始长度：%7，当前长度：%8\n"
                                               "  初始应力：%9，当前应力：%10")
                                    .arg(elementId)
                                    .arg(typeName)
                                    .arg(nodeIds.isEmpty() ? QStringLiteral("--") : nodeIds.join(QStringLiteral(", ")))
                                    .arg(propertyId)
                                    .arg(materialId)
                                    .arg(sectionId)
                                    .arg(element->L0, 0, 'g', 12)
                                    .arg(element->L, 0, 'g', 12)
                                    .arg(element->m_InitStress, 0, 'g', 12)
                                    .arg(element->m_Stress, 0, 'g', 12));
    setWorkspaceMessage(QStringLiteral("已选择单元 %1").arg(elementId));
}

void YQY::clearSelectionProperties()
{
    m_selectedNodeId = -1;
}

void YQY::setWorkspaceMessage(const QString& message)
{
    ui.readyLabel->setText(message);
}

void YQY::updateLoadStatistics()
{
    ui.loadStatisticsLabel->setText(QStringLiteral("模型读取  总计 %1  ·  成功 %2  ·  失败 %3")
                                        .arg(m_modelLoadTotal)
                                        .arg(m_modelLoadSucceeded)
                                        .arg(m_modelLoadFailed));
}
