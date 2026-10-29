#include "YQY.h"
#include <QVariant>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMainWindow>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSpacerItem>
#include <QSplitter>
#include <QStackedWidget>
#include <QTabBar>
#include <QTabWidget>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>
#include "Widgets/WorkspaceSplitter.h"
#include "Controllers/ModelController.h"
#include "Controllers/ResultDataController.h"
#include "Controllers/SolveTaskController.h"
#include "Controllers/WorkspaceController.h"
#include "Widgets/ModelViewport.h"
#include "Widgets/NavigationButton.h"
#include "Widgets/SingleLineLabel.h"
#include "Widgets/SettingsPanel.h"
#include "Widgets/ResultControlPanel.h"
#include "Widgets/NodeTimeHistoryPanel.h"
#include "Widgets/ResultControlInputWidgets.h"
#include "Widgets/AnalysisSummaryPanel.h"
#include "Widgets/PropertyModule.h"
#include "Widgets/ConductorModule.h"
#include "Dialogs/AnalysisManagerDialog.h"
#include "Dialogs/ComputeRegionManagerDialog.h"
#include "Dialogs/ModelSetManagerDialog.h"
#include "Dialogs/SolveTaskManagerDialog.h"
#include "Dialogs/PropertyItemEditorDialog.h"
#include "Dialogs/PropertyLibraryDialog.h"
#include "Dialogs/NodeResultExportDialog.h"
#include "Dialogs/ElementResultExportDialog.h"
#include "Dialogs/ResultPlotExportDialog.h"
#include "Dialogs/FileDialogService.h"
#include "Application/ApplicationPaths.h"
#include "Utility/ResultExportPaths.h"
#include "Solver/Assembly/AssemblySettings.h"
#include "Solver/Linear/Gpu/GpuSettings.h"
#include "Solver/Linear/LinearSolverSettings.h"
#include "Export/ResultOutputSettings.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementSpring1.h"
#include "DataStructure/Element/ElementSpring2.h"
#include "DataStructure/Element/ElementSpringA.h"
#include "Export/Hdf5ModelSerializer.h"
#include "Export/Hdf5ResultReader.h"
#include "Export/ResultExporter.h"
#include "Export/Outputter.h"
#include "Export/ResultFrameUtilities.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Section/SectionRectangle.h"
#include "Conductor/PropertyLibrary.h"
#include "Widgets/ProgressBarAnimation.h"
#include "Utility/UiIcons.h"
#include <cmath>
#include <QAction>
#include <QActionGroup>
#include <QAbstractSpinBox>
#include <QClipboard>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QEvent>
#include <QFile>
#include <QFileInfo>
#include <QFutureWatcher>
#include <QKeyEvent>
#include <QLocale>
#include <QMessageBox>
#include <QMenu>
#include <QPainter>
#include <QPointer>
#include <QProcess>
#include <QScopedValueRollback>
#include <QStyleOptionComboBox>
#include <QStyleOptionButton>
#include <QStyleOptionSpinBox>
#include <QScrollBar>
#include <QTextCursor>
#include <QTextStream>
#include <QTimer>
#include <QUrl>
#include <QtConcurrent/QtConcurrentRun>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace
{
class InputArrowOverlay final : public QWidget
{
public:
    explicit InputArrowOverlay(QWidget* target)
        : QWidget(target)
        , m_target(target)
    {
        setObjectName(QStringLiteral("inputArrowOverlay"));
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_NoSystemBackground);
        setFocusPolicy(Qt::NoFocus);
        m_target->installEventFilter(this);
        updateGeometry();
        show();
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (watched == m_target)
        {
            switch (event->type())
            {
                case QEvent::Resize:
                case QEvent::Show:
                case QEvent::StyleChange:
                case QEvent::EnabledChange:
                case QEvent::PaletteChange:
                case QEvent::LayoutDirectionChange:
                    updateGeometry();
                    break;
                case QEvent::Paint:
                    // 值、范围和按钮模式改变也会触发原控件重绘。
                    UpdateSpinState();
                    break;
                default:
                    break;
            }
        }
        return QWidget::eventFilter(watched, event);
    }

    void paintEvent(QPaintEvent*) override
    {
        if (!m_target)
            return;

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        if (auto* combo = qobject_cast<QComboBox*>(m_target))
        {
            QStyleOptionComboBox option;
            option.initFrom(combo);
            option.editable = combo->isEditable();
            option.frame = combo->hasFrame();
            option.iconSize = combo->iconSize();
            const QRect arrowRect =
                combo->style()->subControlRect(QStyle::CC_ComboBox, &option, QStyle::SC_ComboBoxArrow, combo);
            DrawChevron(painter, arrowRect, false, combo->isEnabled(), true);
            return;
        }

        auto* spin = qobject_cast<QAbstractSpinBox*>(m_target);
        if (!spin || spin->buttonSymbols() == QAbstractSpinBox::NoButtons)
            return;

        QStyleOptionSpinBox option;
        option.initFrom(spin);
        option.frame = spin->hasFrame();
        option.buttonSymbols = spin->buttonSymbols();
        option.stepEnabled = StepEnabled(spin);
        const QRect up = spin->style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxUp, spin);
        const QRect down = spin->style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxDown, spin);
        const bool upEnabled = spin->isEnabled() && option.stepEnabled.testFlag(QAbstractSpinBox::StepUpEnabled);
        const bool downEnabled = spin->isEnabled() && option.stepEnabled.testFlag(QAbstractSpinBox::StepDownEnabled);
        if (option.buttonSymbols == QAbstractSpinBox::PlusMinus)
        {
            DrawPlusMinus(painter, up, true, upEnabled);
            DrawPlusMinus(painter, down, false, downEnabled);
            return;
        }
        DrawChevron(painter, up, true, upEnabled, false);
        DrawChevron(painter, down, false, downEnabled, false);
    }

private:
    void UpdateSpinState()
    {
        const auto* spin = qobject_cast<QAbstractSpinBox*>(m_target);
        if (!spin)
            return;

        const int state = static_cast<int>(StepEnabled(spin)) | (static_cast<int>(spin->buttonSymbols()) << 4);
        if (state == m_spinVisualState)
            return;
        m_spinVisualState = state;
        update();
    }

    void updateGeometry()
    {
        if (!m_target)
            return;

        setGeometry(m_target->rect());
        raise();
        update();
    }

    static QAbstractSpinBox::StepEnabled StepEnabled(const QAbstractSpinBox* spin)
    {
        if (spin->isReadOnly())
            return QAbstractSpinBox::StepNone;

        bool up = true;
        bool down = true;
        if (const auto* number = qobject_cast<const QSpinBox*>(spin))
        {
            up = number->wrapping() || number->value() < number->maximum();
            down = number->wrapping() || number->value() > number->minimum();
        }
        else if (const auto* number = qobject_cast<const QDoubleSpinBox*>(spin))
        {
            up = number->wrapping() || number->value() < number->maximum();
            down = number->wrapping() || number->value() > number->minimum();
        }
        QAbstractSpinBox::StepEnabled result = QAbstractSpinBox::StepNone;
        if (up)
            result |= QAbstractSpinBox::StepUpEnabled;
        if (down)
            result |= QAbstractSpinBox::StepDownEnabled;
        return result;
    }

    void SetArrowPen(_OUT QPainter& painter, bool enabled, qreal width) const
    {
        const QPalette::ColorGroup group = enabled ? QPalette::Active : QPalette::Disabled;
        QColor color = m_target->palette().color(group, QPalette::Text);
        color.setAlphaF(enabled ? 0.86 : 0.60);
        painter.setPen(QPen(color, width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    }

    void DrawChevron(_OUT QPainter& painter, const QRect& rect, bool upward, bool enabled, bool combo) const
    {
        if (rect.isEmpty())
            return;

        SetArrowPen(painter, enabled, combo ? 1.6 : 1.45);
        const QPointF center = QRectF(rect).center();
        const qreal halfWidth = qMin(combo ? 4.0 : 3.2, rect.width() / 3.0);
        const qreal halfHeight = qMin(combo ? 2.0 : 1.6, rect.height() / 4.0);
        const qreal direction = upward ? -1.0 : 1.0;
        QPainterPath path;
        path.moveTo(center.x() - halfWidth, center.y() - direction * halfHeight);
        path.lineTo(center.x(), center.y() + direction * halfHeight);
        path.lineTo(center.x() + halfWidth, center.y() - direction * halfHeight);
        painter.drawPath(path);
    }

    void DrawPlusMinus(_OUT QPainter& painter, const QRect& rect, bool plus, bool enabled) const
    {
        if (rect.isEmpty())
            return;

        SetArrowPen(painter, enabled, 1.45);
        const QPointF center = QRectF(rect).center();
        painter.drawLine(center + QPointF(-3, 0), center + QPointF(3, 0));
        if (plus)
            painter.drawLine(center + QPointF(0, -3), center + QPointF(0, 3));
    }

    QPointer<QWidget> m_target;
    int m_spinVisualState = -1;
};

class CheckboxCheckmarkOverlay final : public QWidget
{
public:
    explicit CheckboxCheckmarkOverlay(QCheckBox* target)
        : QWidget(target)
        , m_target(target)
    {
        setObjectName(QStringLiteral("checkboxCheckmarkOverlay"));
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_NoSystemBackground);
        setFocusPolicy(Qt::NoFocus);
        m_target->installEventFilter(this);
        connect(m_target, &QCheckBox::checkStateChanged, this,
                [this](Qt::CheckState)
                {
                    update();
                });
        updateGeometry();
        show();
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (watched == m_target &&
            (event->type() == QEvent::Resize || event->type() == QEvent::Show || event->type() == QEvent::StyleChange ||
             event->type() == QEvent::EnabledChange || event->type() == QEvent::PaletteChange ||
             event->type() == QEvent::LayoutDirectionChange))
        {
            updateGeometry();
        }
        return QWidget::eventFilter(watched, event);
    }

    void paintEvent(QPaintEvent*) override
    {
        if (!m_target || m_target->checkState() == Qt::Unchecked)
            return;

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        QColor color =
            m_target->isEnabled() ? QColor(Qt::white) : m_target->palette().color(QPalette::Disabled, QPalette::Text);
        if (m_target->checkState() == Qt::PartiallyChecked)
            color = m_target->palette().color(m_target->isEnabled() ? QPalette::Active : QPalette::Disabled,
                                              QPalette::Text);
        painter.setPen(QPen(color, 2.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

        if (m_target->checkState() == Qt::PartiallyChecked)
        {
            painter.drawLine(QPointF(width() * 0.30, height() * 0.50), QPointF(width() * 0.70, height() * 0.50));
            return;
        }

        QPainterPath path;
        path.moveTo(width() * 0.31, height() * 0.50);
        path.lineTo(width() * 0.44, height() * 0.72);
        path.lineTo(width() * 0.70, height() * 0.28);
        painter.drawPath(path);
    }

private:
    void updateGeometry()
    {
        if (!m_target)
            return;

        QStyleOptionButton option;
        option.initFrom(m_target);
        option.text = m_target->text();
        option.icon = m_target->icon();
        option.iconSize = m_target->iconSize();
        setGeometry(m_target->style()->subElementRect(QStyle::SE_CheckBoxIndicator, &option, m_target));
        raise();
        update();
    }

    QPointer<QCheckBox> m_target;
};

static void installInputControlOverlays(QWidget* root)
{
    if (!root || root->objectName() == QStringLiteral("inputArrowOverlay") ||
        root->objectName() == QStringLiteral("checkboxCheckmarkOverlay"))
        return;

    const auto install = [](QWidget* target)
    {
        if (!target || target->property("inputArrowOverlayInstalled").toBool())
            return;
        target->setProperty("inputArrowOverlayInstalled", true);
        new InputArrowOverlay(target);
    };
    const auto installCheckmark = [](QCheckBox* target)
    {
        if (!target || target->property("checkboxCheckmarkOverlayInstalled").toBool())
            return;
        target->setProperty("checkboxCheckmarkOverlayInstalled", true);
        new CheckboxCheckmarkOverlay(target);
    };

    if (qobject_cast<QComboBox*>(root) || qobject_cast<QAbstractSpinBox*>(root))
        install(root);
    if (auto* checkBox = qobject_cast<QCheckBox*>(root))
        installCheckmark(checkBox);
    for (QComboBox* combo : root->findChildren<QComboBox*>())
        install(combo);
    for (QAbstractSpinBox* spin : root->findChildren<QAbstractSpinBox*>())
        install(spin);
    for (QCheckBox* checkBox : root->findChildren<QCheckBox*>())
        installCheckmark(checkBox);
}

QString solveTaskDisplayStatus(const SolveTaskController::TaskInfo& info)
{
    if (info.initialStatePreparation)
    {
        switch (info.status)
        {
            case SolveTaskController::Status::Ready:
                return QStringLiteral("初态待运行");
            case SolveTaskController::Status::Queued:
                return QStringLiteral("初态排队中");
            case SolveTaskController::Status::Running:
                return QStringLiteral("初态计算中");
            case SolveTaskController::Status::Completed:
                return QStringLiteral("初态完成");
            case SolveTaskController::Status::Failed:
                return QStringLiteral("初态失败");
            case SolveTaskController::Status::Cancelling:
                return QStringLiteral("初态停止中");
            case SolveTaskController::Status::Cancelled:
                return QStringLiteral("初态已停止");
        }
    }
    if (info.status == SolveTaskController::Status::Queued && info.waitingForInitialStatic)
        return QStringLiteral("等待静力");
    if (info.partialResult)
    {
        return info.status == SolveTaskController::Status::Cancelled ? QStringLiteral("已停止·部分结果")
                                                                     : QStringLiteral("失败·部分结果");
    }
    return SolveTaskController::statusText(info.status);
}

const Hdf5ResultRange& resultRangeForField(const Hdf5ResultRanges& ranges, ModelViewport::ResultField field)
{
    switch (field)
    {
        case ModelViewport::ResultField::DisplacementMagnitude:
            return ranges.displacementMagnitude;
        case ModelViewport::ResultField::DisplacementX:
            return ranges.displacementX;
        case ModelViewport::ResultField::DisplacementY:
            return ranges.displacementY;
        case ModelViewport::ResultField::DisplacementZ:
            return ranges.displacementZ;
        case ModelViewport::ResultField::AxialForce:
            return ranges.axialForce;
        case ModelViewport::ResultField::Stress:
            return ranges.stress;
        case ModelViewport::ResultField::Strain:
            return ranges.strain;
    }
    return ranges.displacementMagnitude;
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
    Solid,
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
    SpringBehavior,
    ElementTypes,
    Truss,
    Cable,
    Beam,
    SpringGround,
    Spring,
    SpringAxial,
    Other,
    AnalysisSteps,
    AnalysisStep,
    Load,
    Constraint,
    MPC,
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
    CreateMultiSpanConductor,
    Refresh,
    Apply,
    Export,
    Plot
};

constexpr auto ActionGlyphProperty = "semanticActionGlyph";

constexpr int TreeGlyphRole = Qt::UserRole + 20;

class InstantToolTipFilter final : public QObject
{
public:
    explicit InstantToolTipFilter(QAbstractButton* button)
        : QObject(button)
        , m_button(button)
    {
    }

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
void applyWindowsWindowIcon(QWidget* widget)
{
    if (!widget || !widget->isWindow())
        return;

    constexpr int applicationIconResourceId = 101;
    const HINSTANCE moduleHandle = GetModuleHandleW(nullptr);
    const int smallExtent = qRound(16 * widget->devicePixelRatioF());
    const int largeExtent = qRound(32 * widget->devicePixelRatioF());
    const HICON smallIcon = static_cast<HICON>(LoadImageW(moduleHandle, MAKEINTRESOURCEW(applicationIconResourceId),
                                                          IMAGE_ICON, smallExtent, smallExtent, LR_SHARED));
    const HICON largeIcon = static_cast<HICON>(LoadImageW(moduleHandle, MAKEINTRESOURCEW(applicationIconResourceId),
                                                          IMAGE_ICON, largeExtent, largeExtent, LR_SHARED));
    const HWND windowHandle = reinterpret_cast<HWND>(widget->winId());
    if (smallIcon)
        SendMessageW(windowHandle, WM_SETICON, ICON_SMALL, reinterpret_cast<LPARAM>(smallIcon));
    if (largeIcon)
        SendMessageW(windowHandle, WM_SETICON, ICON_BIG, reinterpret_cast<LPARAM>(largeIcon));
}

void applyWindowsTitleBarTheme(QWidget* widget)
{
    if (!widget || !widget->isWindow() || widget->windowType() == Qt::Popup || widget->windowType() == Qt::ToolTip)
        return;

    using DwmSetWindowAttributeFunction = HRESULT(WINAPI*)(HWND, DWORD, LPCVOID, DWORD);
    static const DwmSetWindowAttributeFunction setWindowAttribute = []()
    {
        const HMODULE dwmApi = LoadLibraryW(L"dwmapi.dll");
        return dwmApi ? reinterpret_cast<DwmSetWindowAttributeFunction>(GetProcAddress(dwmApi, "DwmSetWindowAttribute"))
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
void applyWindowsWindowIcon(QWidget*)
{
}

void applyWindowsTitleBarTheme(QWidget*)
{
}
#endif

class PopupThemeFilter final : public QObject
{
public:
    explicit PopupThemeFilter(QObject* parent)
        : QObject(parent)
    {
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() == QEvent::DevicePixelRatioChange)
        {
            auto* window = qobject_cast<QWidget*>(watched);
            if (window && window->isWindow() && window->windowType() != Qt::Popup &&
                window->windowType() != Qt::ToolTip)
                applyWindowsWindowIcon(window);
        }
        if (event->type() != QEvent::Show)
            return QObject::eventFilter(watched, event);

        auto* widget = qobject_cast<QWidget*>(watched);
        if (widget)
            installInputControlOverlays(widget);
        if (widget && widget->isWindow() && widget->windowType() != Qt::Popup && widget->windowType() != Qt::ToolTip)
        {
            widget->setWindowIcon(UiIcons::applicationIcon());
            QTimer::singleShot(0, widget,
                               [widget]()
                               {
                                   applyWindowsWindowIcon(widget);
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
    QPixmap pixmap(96, 96);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.scale(4.0, 4.0);
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
        case ToolbarGlyph::Solid:
            painter.setBrush(accentFill);
            painter.drawPolygon(QPolygonF{QPointF(4, 7), QPointF(12, 3), QPointF(20, 7), QPointF(12, 12)});
            painter.drawPolygon(QPolygonF{QPointF(4, 7), QPointF(12, 12), QPointF(12, 21), QPointF(4, 16)});
            painter.setBrush(accent);
            painter.drawPolygon(QPolygonF{QPointF(12, 12), QPointF(20, 7), QPointF(20, 16), QPointF(12, 21)});
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
    const QPixmap normal = renderToolbarGlyph(glyph, color, accent);
    const QPixmap checked = renderToolbarGlyph(glyph, accent, accent);
    icon.addPixmap(normal, QIcon::Normal, QIcon::Off);
    icon.addPixmap(checked, QIcon::Normal, QIcon::On);
    icon.addPixmap(renderToolbarGlyph(glyph, accent, accent), QIcon::Active, QIcon::Off);
    icon.addPixmap(checked, QIcon::Active, QIcon::On);
    // 未加载模型时保留图标原色，操作是否可用仍由按钮状态控制。
    icon.addPixmap(normal, QIcon::Disabled, QIcon::Off);
    icon.addPixmap(checked, QIcon::Disabled, QIcon::On);
    return icon;
}

QPixmap renderNavigationGlyph(NavigationGlyph glyph, const QColor& color, const QColor& accent)
{
    // 保持普通高分辨率位图，由 QIcon 完成最终 DPI 缩放，避免侧栏出现双重设备像素比裁剪。
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
        case ActionGlyph::CreateMultiSpanConductor:
        {
            for (int x : {3, 14, 25})
                painter.drawLine(QPointF(x, 5), QPointF(x, 27));
            QPainterPath wire;
            wire.moveTo(3, 9);
            wire.cubicTo(6, 23, 11, 23, 14, 9);
            wire.cubicTo(17, 23, 22, 23, 25, 9);
            painter.drawPath(wire);
            painter.setPen(QPen(accent, 2.4, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(QPointF(28, 20), QPointF(28, 28));
            painter.drawLine(QPointF(24, 24), QPointF(32, 24));
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
        case ActionGlyph::Plot:
            painter.drawPolyline(QPolygonF{QPointF(4, 4), QPointF(4, 28), QPointF(28, 28)});
            painter.setPen(QPen(accent, 2.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter.drawPolyline(QPolygonF{QPointF(7, 22), QPointF(12, 12), QPointF(18, 19), QPointF(26, 7)});
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
    // 按 64 像素渲染且不设置设备像素比；树控件请求 20 像素图标时由 Qt 在各缩放比例下清晰降采样。
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
        case TreeGlyph::SpringBehavior:
        case TreeGlyph::SpringGround:
        case TreeGlyph::Spring:
        case TreeGlyph::SpringAxial:
            painter.drawLine(QPointF(1.5, 8), QPointF(3.5, 8));
            painter.drawLine(QPointF(12.5, 8), QPointF(14.5, 8));
            painter.setPen(QPen(accent, 1.35, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter.drawPolyline(QPolygonF{QPointF(3.5, 8), QPointF(4.5, 4.5), QPointF(6.5, 11.5), QPointF(8.5, 4.5),
                                           QPointF(10.5, 11.5), QPointF(12.5, 8)});
            painter.setPen(QPen(color, 1.15, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter.setBrush(accent);
            if (glyph == TreeGlyph::SpringGround)
            {
                painter.drawLine(QPointF(1.5, 3), QPointF(1.5, 13));
                painter.drawLine(QPointF(1.5, 4), QPointF(0.5, 5.5));
                painter.drawLine(QPointF(1.5, 10), QPointF(0.5, 11.5));
            }
            else
                painter.drawEllipse(QPointF(1.5, 8), 0.8, 0.8);
            painter.drawEllipse(QPointF(14.5, 8), 0.8, 0.8);
            if (glyph == TreeGlyph::SpringAxial)
            {
                painter.drawLine(QPointF(5, 1.5), QPointF(11, 1.5));
                painter.drawLine(QPointF(5, 1.5), QPointF(6.5, 0.5));
                painter.drawLine(QPointF(11, 1.5), QPointF(9.5, 2.5));
            }
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
        case TreeGlyph::MPC:
            painter.drawEllipse(QPointF(3.5, 8), 2.0, 2.0);
            painter.drawEllipse(QPointF(12.5, 8), 2.0, 2.0);
            painter.drawLine(QPointF(5.7, 8), QPointF(10.3, 8));
            painter.setPen(QPen(accent, 1.5, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(QPointF(8, 4.5), QPointF(8, 11.5));
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
        return {QStringLiteral("#F3F5F8"), QStringLiteral("#FFFFFF"), QStringLiteral("#F7F9FC"),
                QStringLiteral("#FFFFFF"), QStringLiteral("#F7F9FC"), QStringLiteral("#FFFFFF"),
                QStringLiteral("#DFE4EC"), QStringLiteral("#B9C4D2"), QStringLiteral("#263445"),
                QStringLiteral("#617186"), QStringLiteral("#2563C6"), QStringLiteral("#1D53AD"),
                QStringLiteral("#E7EFFC"), QStringLiteral("#23835D"), QStringLiteral("#FFFFFF")};
    }
    if (themeIndex == 1)
    {
        return {QStringLiteral("#0b0712"), QStringLiteral("#1d112e"), QStringLiteral("#140c20"),
                QStringLiteral("#1b102b"), QStringLiteral("#241638"), QStringLiteral("#180f26"),
                QStringLiteral("#3f285c"), QStringLiteral("#684681"), QStringLiteral("#f5edff"),
                QStringLiteral("#b9a9ce"), QStringLiteral("#7660CC"), QStringLiteral("#8B78DB"),
                QStringLiteral("#38204e"), QStringLiteral("#59f0ce"), QStringLiteral("#ffffff")};
    }
    if (themeIndex == 2)
    {
        return {QStringLiteral("#10272D"), QStringLiteral("#1B373E"), QStringLiteral("#142E35"),
                QStringLiteral("#19363D"), QStringLiteral("#23444B"), QStringLiteral("#122F36"),
                QStringLiteral("#55777f"), QStringLiteral("#78989f"), QStringLiteral("#efffff"),
                QStringLiteral("#9abdc3"), QStringLiteral("#267B73"), QStringLiteral("#348E85"),
                QStringLiteral("#294e55"), QStringLiteral("#6df1b7"), QStringLiteral("#FFFFFF")};
    }
    return {QStringLiteral("#111720"), QStringLiteral("#19222E"), QStringLiteral("#151D27"), QStringLiteral("#1A232F"),
            QStringLiteral("#202B39"), QStringLiteral("#151E29"), QStringLiteral("#303D4E"), QStringLiteral("#4B5C71"),
            QStringLiteral("#E3EAF3"), QStringLiteral("#A2B0C2"), QStringLiteral("#316FCC"), QStringLiteral("#4681DC"),
            QStringLiteral("#243C5B"), QStringLiteral("#53B98A"), QStringLiteral("#FFFFFF")};
}

QString buildStyleSheet(const ThemeColors& c)
{
    QString style = QStringLiteral(R"QSS(
* { font-family: "Microsoft YaHei UI", "Segoe UI"; font-size: 13px; color: $TEXT; }
QMainWindow, #centralWidget { background: $ROOT; }
QDialog, QMessageBox { background: $PANEL; color: $TEXT; }
QDialog QWidget, QMessageBox QWidget { color: $TEXT; }
QDialogButtonBox { background: transparent; border: none; }
QMessageBox QLabel,
QMessageBox QLabel#qt_msgbox_label,
QMessageBox QLabel#qt_msgbox_informativelabel { background: transparent; color: $TEXT; }
QDialogButtonBox QPushButton, QMessageBox QPushButton { min-width: 76px; }
QGroupBox {
    background: $ELEVATED; border: 1px solid $BORDER; border-radius: 6px;
    margin-top: 12px; padding: 12px 10px 10px;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 10px; padding: 0 5px; color: $TEXT; font-weight: 600;
}
QScrollArea, QScrollArea > QWidget > QWidget { background: $PANEL; border: none; }
QLabel[secondaryText="true"] { color: $MUTED; background: transparent; }
#mutedText, #spacerStatusLabel { color: $MUTED; background: transparent; }
#cardTitle { font-size: 14px; font-weight: 600; background: transparent; }
QDialog#analysisStepManagerDialog,
QDialog#analysisLoadManagerDialog,
QDialog#analysisConstraintManagerDialog,
QDialog#analysisMPCManagerDialog,
QDialog#solveTaskManagerDialog,
QDialog#analysisStepEditorDialog,
QDialog#analysisLoadEditorDialog,
QDialog#analysisConstraintEditorDialog,
QDialog#analysisMPCEditorDialog,
QDialog#nodeResultExportDialog,
QDialog#resultPlotExportDialog {
    background: $PANEL;
    color: $TEXT;
}
QDialog#analysisStepManagerDialog QWidget#managerPage,
QDialog#analysisLoadManagerDialog QWidget#managerPage,
QDialog#analysisConstraintManagerDialog QWidget#managerPage,
QDialog#analysisMPCManagerDialog QWidget#managerPage {
    background: $FIELD;
    color: $TEXT;
}
QDialog#analysisStepManagerDialog QDialogButtonBox#analysisManagerButtonBox,
QDialog#analysisLoadManagerDialog QDialogButtonBox#analysisManagerButtonBox,
QDialog#analysisConstraintManagerDialog QDialogButtonBox#analysisManagerButtonBox,
QDialog#analysisMPCManagerDialog QDialogButtonBox#analysisManagerButtonBox {
    background: transparent;
    border: none;
}
QDialog#analysisStepEditorDialog QLabel,
QDialog#analysisLoadEditorDialog QLabel,
QDialog#analysisConstraintEditorDialog QLabel,
QDialog#analysisMPCEditorDialog QLabel {
    background: transparent;
}
QDialog#nodeResultExportDialog QLabel { background: transparent; }
QDialog #exportDialogTitle, QDialog #plotDialogTitle { font-size: 18px; font-weight: 600; color: $TEXT; }
QDialog #exportDialogSubtitle, QDialog #plotDialogSubtitle, QDialog #exportFieldHint { color: $MUTED; }
QDialog #exportValidationLabel { color: #D85A64; }
QDialog#nodeResultExportDialog #exportDialogSubtitle,
QDialog#nodeResultExportDialog #exportFieldHint { color: $MUTED; }
QDialog#nodeResultExportDialog #exportSectionCard {
    background: $ELEVATED; border: 1px solid $BORDER; border-radius: 6px;
}
QDialog#resultPlotExportDialog #resultPlotSectionCard {
    background: $ELEVATED; border: 1px solid $BORDER; border-radius: 6px;
    margin-top: 10px; padding: 10px 12px 12px 12px;
}
QDialog#resultPlotExportDialog QGroupBox#resultPlotSectionCard::title {
    subcontrol-origin: margin; left: 10px; padding: 0 5px; color: $TEXT; font-weight: 700;
}
QDialog#resultPlotExportDialog #plotDialogTitle { font-size: 18px; font-weight: 600; color: $TEXT; }
QDialog#resultPlotExportDialog #plotDialogSubtitle { color: $MUTED; }
QDialog#resultPlotExportDialog #plotDialogErrorLabel { color: #F08080; }
QDialog#resultPlotExportDialog #plotDialogQuietButton { background: $FIELD; }
QDialog#resultPlotExportDialog #plotDialogButtons { background: transparent; border: none; }
QDialog#nodeResultExportDialog QGroupBox#exportSectionCard {
    margin-top: 10px; padding: 10px 12px 12px 12px;
}
QDialog#nodeResultExportDialog QGroupBox#exportSectionCard::title {
    subcontrol-origin: margin; left: 10px; padding: 0 5px; color: $TEXT; font-weight: 700;
}
QDialog#nodeResultExportDialog #exportSectionTitle { color: $TEXT; font-weight: 700; }
QDialog#nodeResultExportDialog #exportFieldCaption { color: $MUTED; font-weight: 700; }
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
#logoLabel { background: $ACCENT; color: $SELECTION_TEXT; border-radius: 6px; font-size: 16px; font-weight: 700; }
#appTitleLabel { font-size: 17px; font-weight: 600; }
#themeLabel, #loadStatisticsLabel, #incrementLabel { color: $MUTED; }
#navigationRail { background: $NAV; border-right: 1px solid $BORDER; }
#projectPanel { background: $PANEL; }
#projectPanel { border-right: 1px solid $BORDER; }
#workspacePanel { background: $ROOT; }
#statusFrame { background: $NAV; border-top: 1px solid $BORDER; }
#projectTitleLabel { font-size: 17px; font-weight: 600; }
#modelModuleHint, #analysisModuleHint, #solveModuleHint, #resultModuleHint, #settingsModuleHint { color: $MUTED; }
#modelSummaryCard { background: $ELEVATED; border: 1px solid $BORDER; border-radius: 6px; }
#modelSummaryCard QLabel { background: transparent; }
#modelNodeValue, #modelElementValue, #modelPropertyValue { color: $MUTED; }
#readyLabel { color: $MUTED; }
#viewportEmptyLabel { color: $MUTED; font-size: 14px; background: transparent; }

QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QPlainTextEdit {
    background: $FIELD; border: 1px solid $BORDER_STRONG; border-radius: 5px; padding: 5px 9px;
    selection-background-color: $ACCENT; selection-color: $SELECTION_TEXT; min-height: 20px;
}
QComboBox, QDoubleSpinBox, QSpinBox { padding-right: 28px; }
QPlainTextEdit { padding: 8px; }
#logEdit { padding: 4px 8px; }
QComboBox { selection-background-color: $ACCENT_SOFT; selection-color: $TEXT; }
QLineEdit:hover, QComboBox:hover, QDoubleSpinBox:hover, QSpinBox:hover { border-color: $ACCENT2; }
QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus { border: 1px solid $ACCENT; background: $ELEVATED; }
QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; border: none; width: 28px; }
QComboBox::down-arrow {
    image: none;
    width: 0;
    height: 0;
}
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
    width: 28px;
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
QDoubleSpinBox[layoutDirection="1"]::up-button, QSpinBox[layoutDirection="1"]::up-button {
    border-left: none; border-right: 1px solid $BORDER;
    border-top-right-radius: 0; border-top-left-radius: 5px;
}
QDoubleSpinBox[layoutDirection="1"]::down-button, QSpinBox[layoutDirection="1"]::down-button {
    border-left: none; border-right: 1px solid $BORDER;
    border-bottom-right-radius: 0; border-bottom-left-radius: 5px;
}
QDoubleSpinBox::up-arrow, QSpinBox::up-arrow {
    image: none;
    width: 0;
    height: 0;
}
QDoubleSpinBox::down-arrow, QSpinBox::down-arrow {
    image: none;
    width: 0;
    height: 0;
}
QDoubleSpinBox::up-button:hover, QSpinBox::up-button:hover,
QDoubleSpinBox::down-button:hover, QSpinBox::down-button:hover { background: $ACCENT_SOFT; }

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
    background: $ELEVATED; border: 1px solid $BORDER_STRONG; border-radius: 5px; padding: 6px 12px; min-height: 20px;
}
QPushButton:hover, QToolButton:hover { background: $ACCENT_SOFT; border-color: $ACCENT2; }
QPushButton:focus, QToolButton:focus { border-color: $ACCENT; }
QPushButton:pressed, QToolButton:pressed { background: $FIELD; border-color: $ACCENT; }
QPushButton:disabled, QToolButton:disabled, QLineEdit:disabled, QComboBox:disabled,
QDoubleSpinBox:disabled, QSpinBox:disabled { color: $MUTED; background: $FIELD; border-color: $BORDER; }
QDialog QPushButton:default, QPushButton[primaryAction="true"] {
    background: $ACCENT; color: $SELECTION_TEXT; border-color: $ACCENT; font-weight: 600;
}
QDialog QPushButton:default:hover, QPushButton[primaryAction="true"]:hover { background: $ACCENT2; border-color: $ACCENT2; }
QPushButton[dangerAction="true"] { color: #D85A64; }
QPushButton[dangerAction="true"]:disabled { color: $MUTED; }
QDialog QPushButton:default:disabled, QPushButton[primaryAction="true"]:disabled {
    background: $FIELD; color: $MUTED; border-color: $BORDER;
}
#startSolveButton { background: $ACCENT; color: $SELECTION_TEXT; border: 1px solid $ACCENT; font-weight: 600; padding-left: 18px; padding-right: 18px; }
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
#toolBarCard QPushButton, #toolBarCard QToolButton { background: transparent; border-color: transparent; padding: 0; min-width: 34px; min-height: 34px; }
#toolBarCard QPushButton:hover, #toolBarCard QToolButton:hover { background: $ACCENT_SOFT; border-color: $BORDER_STRONG; }
#toolBarCard QPushButton:focus, #toolBarCard QToolButton:focus { border-color: $ACCENT; }
#toolBarCard #startSolveButton { padding: 0; }
#toolBarCard QToolButton:checked { background: $ACCENT_SOFT; border-color: $ACCENT2; color: $TEXT; }
#toolBarCard #toolbarSeparator { background: $BORDER_STRONG; border: none; }
#resultControlPanel QPushButton { padding: 6px 8px; }
#exportPlotButton {
    background: $ACCENT_SOFT; border-color: $ACCENT2; color: $TEXT; min-height: 20px;
}
#exportPlotButton:hover { background: $ACCENT2; }

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
    padding: 4px 8px;
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
    padding: 7px 8px;
    font-weight: 600;
}
QTableCornerButton::section { background: $PANEL; border: none; border-bottom: 1px solid $BORDER_STRONG; }
#propertyPage { background: $PANEL; }
#propertyTabs::pane { background: $FIELD; border: 1px solid $BORDER; border-radius: 9px; }
#propertyTabs QTabBar::tab:selected { color: $TEXT; border-bottom: 2px solid $ACCENT2; }
)QSS");

    style += QStringLiteral(R"QSS(

/* 导线入口与建模弹窗使用同一主题。 */
#conductorPage,
#conductorCreationDialog,
#conductorScrollArea,
#conductorScrollArea QWidget#qt_scrollarea_viewport,
#conductorContent {
    background: $PANEL;
    color: $TEXT;
    border: none;
}
#conductorPropertyGroup, #conductorFormGroup,
#conductorCreationDialog QGroupBox {
    background: $ELEVATED;
    color: $TEXT;
    border: 1px solid $BORDER;
    border-radius: 9px;
    margin-top: 12px;
    padding: 0;
}
#conductorPropertyGroup::title, #conductorFormGroup::title,
#conductorCreationDialog QGroupBox::title {
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
#conductorPage QSpinBox,
#conductorCreationDialog QLineEdit,
#conductorCreationDialog QComboBox,
#conductorCreationDialog QDoubleSpinBox,
#conductorCreationDialog QSpinBox {
    background: $FIELD;
    color: $TEXT;
    border-color: $BORDER_STRONG;
}
#conductorPage QLineEdit:focus,
#conductorPage QComboBox:focus,
#conductorPage QDoubleSpinBox:focus,
#conductorPage QSpinBox:focus,
#conductorCreationDialog QLineEdit:focus,
#conductorCreationDialog QComboBox:focus,
#conductorCreationDialog QDoubleSpinBox:focus,
#conductorCreationDialog QSpinBox:focus {
    background: $ELEVATED;
    border-color: $ACCENT;
}
#conductorCreationDialog QLineEdit:disabled,
#conductorCreationDialog QComboBox:disabled,
#conductorCreationDialog QDoubleSpinBox:disabled,
#conductorCreationDialog QSpinBox:disabled {
    background: $PANEL;
    color: $MUTED;
    border-color: $BORDER;
}
#conductorCreationDialog QLabel:disabled { color: $MUTED; }
#conductorCreateButton {
    background: $ACCENT;
    color: $SELECTION_TEXT;
    border: none;
    font-weight: 700;
}
#conductorCreateButton:hover { border: 1px solid $TEXT; }
QPushButton[conductorModeSwitch="true"] { padding: 7px 14px; font-weight: 600; }
QPushButton[conductorModeSwitch="true"]:checked { background: $ACCENT_SOFT; border-color: $ACCENT2; color: $TEXT; }
#conductorSettingsTabs::pane { background: $PANEL; border: none; }
#conductorSettingsTabs QTabBar::tab {
    background: transparent; border: none; border-bottom: 2px solid transparent;
    border-radius: 0px; padding: 7px 14px; margin-right: 10px;
}
#conductorSettingsTabs QTabBar::tab:selected { color: $TEXT; border-bottom: 2px solid $ACCENT2; }

QTabBar::tab { background: transparent; color: $MUTED; border: 1px solid transparent; padding: 8px 16px; margin-right: 4px; border-radius: 7px; }
QTabBar::tab:hover { color: $TEXT; background: $ELEVATED; }
QTabBar::tab:selected { color: $TEXT; background: $ELEVATED; border-color: $BORDER_STRONG; }
QTabWidget::pane { background: $ELEVATED; border: 1px solid $BORDER; border-radius: 9px; top: -1px; }
QTabWidget QTabBar::tab { padding: 6px 13px; }
QTabWidget#stepSettingsTabs QTabBar::tab {
    background: transparent; border: none; border-bottom: 2px solid transparent;
    border-radius: 0px; padding: 5px 12px; margin-right: 12px;
}
QTabWidget#stepSettingsTabs QTabBar::tab:selected { color: $TEXT; border-bottom: 2px solid $ACCENT2; }
QTabBar::close-button { image: url(:/YQY/icon_close.svg); width: 14px; height: 14px; margin: 2px 4px; border-radius: 3px; }
QTabBar::close-button:hover { background: $ACCENT_SOFT; }

QProgressBar { background: $FIELD; border: none; border-radius: 4px; min-height: 8px; max-height: 8px; text-align: right; color: transparent; }
QProgressBar::chunk { border-radius: 4px; background: qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 $ACCENT,stop:1 $ACCENT2); }
QProgressBar#solveTaskProgress { border: 1px solid $BORDER; border-radius: 7px; min-height: 22px; max-height: 22px; text-align: center; color: $TEXT; font-weight: 650; }
QProgressBar#solveTaskProgress::chunk { border-radius: 6px; }
QSlider::groove:horizontal { height: 5px; border-radius: 2px; background: $BORDER_STRONG; }
QSlider::sub-page:horizontal { background: $ACCENT2; border-radius: 2px; }
QSlider::handle:horizontal { background: $TEXT; border: 2px solid $ACCENT2; width: 14px; height: 14px; margin: -6px 0; border-radius: 8px; }
QCheckBox, QRadioButton { spacing: 8px; min-height: 24px; background: transparent; }
QCheckBox::indicator {
    width: 16px; height: 16px; border-radius: 3px; background: $FIELD; border: 1px solid $BORDER_STRONG;
}
QCheckBox::indicator:hover, QCheckBox::indicator:focus { border-color: $ACCENT; }
QCheckBox::indicator:checked {
    image: none; background: $ACCENT; border-color: $ACCENT;
}
QCheckBox::indicator:indeterminate { background: $ACCENT_SOFT; border: 2px solid $ACCENT; }
QCheckBox:disabled, QRadioButton:disabled { color: $MUTED; }
QCheckBox::indicator:disabled { background: $ELEVATED; border-color: $BORDER; }
QRadioButton::indicator { width: 16px; height: 16px; border-radius: 9px; background: $FIELD; border: 1px solid $BORDER_STRONG; }
QRadioButton::indicator:checked { background: $ACCENT; border: 4px solid $FIELD; }
QRadioButton::indicator:hover { border-color: $ACCENT; }
QSplitter::handle { background: $BORDER; }
QSplitter::handle:horizontal { width: 4px; }
QSplitter::handle:vertical { height: 6px; }
QSplitter::handle:hover { background: $ACCENT2; }
QSplitter#workspaceVerticalSplitter::handle { background: transparent; height: 12px; }
QScrollBar:vertical { width: 12px; background: transparent; margin: 2px; }
QScrollBar::handle:vertical { background: $BORDER_STRONG; border-radius: 4px; min-height: 28px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal { height: 12px; background: transparent; margin: 2px; }
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
    : QMainWindow(parent)
    , m_modelController(new ModelController(this))
    , m_solveTaskController(new SolveTaskController(this))
    , m_resultDataController(std::make_unique<ResultDataController>())
    , m_workspaceController(std::make_unique<WorkspaceController>())
{
    initializeUi();
    const bool isolatedSolveEnabled = QCoreApplication::arguments().contains(QStringLiteral("--isolated-solve"));
    m_solveTaskController->setIsolatedProcessEnabled(isolatedSolveEnabled);
    if (isolatedSolveEnabled)
        appendLogMessage(
            QStringLiteral("[INFO] 独立进程试验模式已启用；完整算例使用工作进程，指定分析步仍使用原执行方式。"));
    setWindowIcon(QApplication::windowIcon());
    applyWindowsWindowIcon(this);
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
    m_startSolveButton->hide();
    m_solvePageStartButton->hide();
    setMinimumSize(760, 520);
    initializeToolbarAppearance();

    // 导线、属性和分析编辑器需要为标签及原生子控件保留足够空间；初始面板过窄会造成字段显示错乱。
    m_mainSplitter->setSizes(m_normalMainSplitterSizes);
    m_modelDataTree->header()->setStretchLastSection(true);
    m_solveStepTree->setColumnCount(4);
    m_solveStepTree->setHeaderHidden(false);
    m_solveStepTree->setHeaderLabels(
        {QStringLiteral("算例"), QStringLiteral("进度"), QStringLiteral("结果"), QStringLiteral("耗时")});
    m_solveStepTree->header()->setDefaultAlignment(Qt::AlignCenter);
    m_solveStepTree->header()->setMinimumHeight(34);
    m_solveStepTree->setRootIsDecorated(false);
    m_solveStepTree->setIndentation(0);
    m_solveStepTree->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_solveStepTree->header()->setSectionResizeMode(1, QHeaderView::Fixed);
    m_solveStepTree->header()->setSectionResizeMode(2, QHeaderView::Fixed);
    m_solveStepTree->header()->setSectionResizeMode(3, QHeaderView::Fixed);
    m_solveStepTree->setColumnWidth(1, 92);
    m_solveStepTree->setColumnWidth(2, 72);
    m_solveStepTree->setColumnWidth(3, 72);
    const QList<QPushButton*> solveBatchButtons = {m_solveCasesButton, m_solveAllCasesButton,
                                                   m_solveRestartAllCasesButton};
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
    m_solveAllCasesButton->setToolTip(QStringLiteral("将所有待运行分析步加入计算队列；正在计算、已排队和已有结果的算例"
                                                     "不会重复计算；单个分析步失败不会停止其他队列任务"));
    m_solveRestartAllCasesButton->setToolTip(
        QStringLiteral("重新计算所有分析步；正在排队或计算的任务会先安全停止，再从头加入计算队列"));
    connect(m_solveCasesButton, &QPushButton::clicked, this, &YQY::openSolveTaskManager);
    connect(m_solveAllCasesButton, &QPushButton::clicked, this, &YQY::startAllReadySolveTasks);
    connect(m_solveRestartAllCasesButton, &QPushButton::clicked, this, &YQY::restartAllSolveTasks);
    m_solveProgress->setObjectName(QStringLiteral("solveTaskProgress"));
    connect(m_solveProgress, &QProgressBar::valueChanged, m_solveProgress,
            [this](int value)
            {
                m_solveProgress->setFormat(QStringLiteral("%1%").arg(value / 10.0, 0, 'f', 1));
            });
    initializeAnalysisEditor();
    initializeResponsiveLayout();

    initializeEmptyState();
    initializeInteractions();

    if (propertyLibraryReady)
        appendLogMessage(QStringLiteral("[INFO] 启动属性库已加载：%1 种材料，%2 种截面")
                             .arg(m_propertyLibrary->materials().size())
                             .arg(m_propertyLibrary->sections().size()));
    else
    {
        m_conductorModule->setCreationEnabled(false);
        appendLogMessage(QStringLiteral("[ERROR] 启动属性库加载失败：%1").arg(propertyLibraryError));
    }

    QSettings settings;
    const int savedTheme = qBound(0, settings.value(QStringLiteral("appearance/theme"), 0).toInt(), 3);
    m_themeComboBox->setCurrentIndex(savedTheme);
    applyTheme(savedTheme);
}

YQY::~YQY() = default;

void YQY::initializeSettingsModule()
{
    m_settingsPanel = new SettingsPanel(m_settingsModulePage);
    auto* concurrencySpin = m_settingsPanel->concurrencySpin();
    auto* assemblyThreadsSpin = m_settingsPanel->assemblyThreadsSpin();
    auto* resultBatchFramesSpin = m_settingsPanel->resultBatchFramesSpin();
    auto* backgroundResultWriteCheckBox = m_settingsPanel->backgroundResultWriteCheckBox();
    auto* gpuSolverCheckBox = m_settingsPanel->gpuSolverCheckBox();
    auto* linearSolverModeCombo = m_settingsPanel->linearSolverModeCombo();
    auto* nodeLabelModeCombo = m_settingsPanel->nodeLabelModeCombo();
    const int availableThreads = SolveTaskController::availableThreadCount();
    const int allowedThreads = SolveTaskController::maximumAllowedThreadCount();
    concurrencySpin->setRange(SolveTaskController::MinimumThreadCount, allowedThreads);
    concurrencySpin->setSuffix(QStringLiteral(" 个计算区域"));
    concurrencySpin->setMinimumWidth(150);
    concurrencySpin->setToolTip(
        QStringLiteral("本机可选范围 1～%1；降低上限不会中断正在计算的任务").arg(allowedThreads));
    assemblyThreadsSpin->setRange(1, allowedThreads);
    assemblyThreadsSpin->setSuffix(QStringLiteral(" 个线程"));
    assemblyThreadsSpin->setMinimumWidth(150);
    assemblyThreadsSpin->setToolTip(QStringLiteral("单个分析步的单元装配工作线程数。大模型可提高该值；多个分析步同时计"
                                                   "算时，请避免两项设置的乘积超过本机线程数。"));

    QSettings settings;
    const bool adaptiveNodeLabels = settings.value(QStringLiteral("view/adaptiveNodeLabels"), false).toBool();
    nodeLabelModeCombo->setCurrentIndex(adaptiveNodeLabels ? 1 : 0);
    m_modelViewport->setAdaptiveNodeLabels(adaptiveNodeLabels);
    connect(nodeLabelModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int index)
            {
                const bool adaptive = index == 1;
                m_modelViewport->setAdaptiveNodeLabels(adaptive);
                QSettings().setValue(QStringLiteral("view/adaptiveNodeLabels"), adaptive);
                setWorkspaceMessage(adaptive ? QStringLiteral("节点编号显示已切换为自适应")
                                             : QStringLiteral("节点编号显示已切换为全部"));
            });
    const int savedConcurrency = qBound(
        SolveTaskController::MinimumThreadCount,
        settings.value(QStringLiteral("solver/maxConcurrentSteps"), SolveTaskController::defaultThreadCount()).toInt(),
        allowedThreads);
    concurrencySpin->setValue(savedConcurrency);
    m_solveTaskController->setMaximumThreadCount(savedConcurrency);
    const int savedAssemblyThreads =
        qBound(1, settings.value(QStringLiteral("solver/assemblyThreads"), 1).toInt(), allowedThreads);
    assemblyThreadsSpin->setValue(savedAssemblyThreads);
    SolverNameSpace::AssemblySettings::SetThreadCount(savedAssemblyThreads);
    const int savedResultBatchFrames = qBound(
        ResultOutputSettings::MinimumFrameBatchSize,
        settings.value(QStringLiteral("output/resultBatchFrames"), ResultOutputSettings::DefaultFrameBatchSize).toInt(),
        ResultOutputSettings::MaximumFrameBatchSize);
    resultBatchFramesSpin->setValue(savedResultBatchFrames);
    ResultOutputSettings::SetFrameBatchSize(savedResultBatchFrames);
    const bool backgroundResultWrite = settings.value(QStringLiteral("output/backgroundResultWrite"), true).toBool();
    backgroundResultWriteCheckBox->setChecked(backgroundResultWrite);
    ResultOutputSettings::SetBackgroundWriteEnabled(backgroundResultWrite);
    const bool useGpuSolver = settings.value(QStringLiteral("solver/useGpuSparseSolver"), false).toBool();
    gpuSolverCheckBox->setChecked(useGpuSolver);
    SolverNameSpace::GpuSettings::SetEnabled(useGpuSolver);
    const int savedLinearSolverMode =
        qBound(0, settings.value(QStringLiteral("solver/linearSolverMode"), 0).toInt(), 5);
    linearSolverModeCombo->setCurrentIndex(savedLinearSolverMode);
    SolverNameSpace::LinearSolverSettings::SetMode(
        static_cast<SolverNameSpace::LinearSolverMode>(savedLinearSolverMode));
    gpuSolverCheckBox->setEnabled(savedLinearSolverMode == 0);
    auto* status = m_settingsPanel->statusLabel();
    const auto updateStatus = [status, availableThreads, allowedThreads](int concurrency, int assemblyThreads)
    {
        status->setText(
            QStringLiteral(
                "检测到 %1 个逻辑线程，本机安全上限为 %2；当前最多并发 %3 个分析步，每个分析步使用 %4 个装配线程。")
                .arg(availableThreads)
                .arg(allowedThreads)
                .arg(concurrency)
                .arg(assemblyThreads));
    };
    updateStatus(savedConcurrency, savedAssemblyThreads);
    connect(concurrencySpin, qOverload<int>(&QSpinBox::valueChanged), this,
            [this, assemblyThreadsSpin, updateStatus](int count)
            {
                m_solveTaskController->setMaximumThreadCount(count);
                const int applied = m_solveTaskController->maximumThreadCount();
                QSettings().setValue(QStringLiteral("solver/maxConcurrentSteps"), applied);
                updateStatus(applied, assemblyThreadsSpin->value());
                setWorkspaceMessage(QStringLiteral("最大并发计算区域已设置为 %1").arg(applied));
            });
    connect(assemblyThreadsSpin, qOverload<int>(&QSpinBox::valueChanged), this,
            [this, concurrencySpin, updateStatus](int count)
            {
                SolverNameSpace::AssemblySettings::SetThreadCount(count);
                QSettings().setValue(QStringLiteral("solver/assemblyThreads"), count);
                updateStatus(concurrencySpin->value(), count);
                setWorkspaceMessage(QStringLiteral("单步单元装配线程已设置为 %1").arg(count));
            });
    connect(resultBatchFramesSpin, qOverload<int>(&QSpinBox::valueChanged), this,
            [this](int frameCount)
            {
                ResultOutputSettings::SetFrameBatchSize(frameCount);
                QSettings().setValue(QStringLiteral("output/resultBatchFrames"), frameCount);
                setWorkspaceMessage(QStringLiteral("动力结果已设置为每 %1 帧批量写入一次").arg(frameCount));
            });
    connect(backgroundResultWriteCheckBox, &QCheckBox::toggled, this,
            [this](bool enabled)
            {
                ResultOutputSettings::SetBackgroundWriteEnabled(enabled);
                QSettings().setValue(QStringLiteral("output/backgroundResultWrite"), enabled);
                setWorkspaceMessage(enabled ? QStringLiteral("动力结果后台写入已启用")
                                            : QStringLiteral("动力结果后台写入已关闭"));
            });
    connect(gpuSolverCheckBox, &QCheckBox::toggled, this,
            [this](bool enabled)
            {
                SolverNameSpace::GpuSettings::SetEnabled(enabled);
                QSettings().setValue(QStringLiteral("solver/useGpuSparseSolver"), enabled);
                setWorkspaceMessage(enabled ? SolverNameSpace::GpuSettings::CalibrationDescription()
                                            : QStringLiteral("GPU 加速已关闭"));
            });
    connect(linearSolverModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int index)
            {
                const auto mode = static_cast<SolverNameSpace::LinearSolverMode>(index);
                SolverNameSpace::LinearSolverSettings::SetMode(mode);
                QSettings().setValue(QStringLiteral("solver/linearSolverMode"), index);
                m_settingsPanel->gpuSolverCheckBox()->setEnabled(mode == SolverNameSpace::LinearSolverMode::Automatic);
                switch (mode)
                {
                    case SolverNameSpace::LinearSolverMode::Automatic:
                        setWorkspaceMessage(QStringLiteral("线性求解方法已设置为自动选择"));
                        break;
                    case SolverNameSpace::LinearSolverMode::Ldlt:
                        setWorkspaceMessage(QStringLiteral("线性求解方法已设置为 LDLT，仅适用于对称矩阵"));
                        break;
                    case SolverNameSpace::LinearSolverMode::Lu:
                        setWorkspaceMessage(QStringLiteral("线性求解方法已设置为 Eigen LU，可处理非对称矩阵"));
                        break;
                    case SolverNameSpace::LinearSolverMode::Pardiso:
                        setWorkspaceMessage(QStringLiteral(
                            "线性求解方法已设置为 oneMKL PARDISO，对称矩阵使用 LDLT，非对称矩阵使用 LU"));
                        break;
                    case SolverNameSpace::LinearSolverMode::CudaIterative:
                        setWorkspaceMessage(QStringLiteral("线性求解方法已设置为 CUDA BiCGSTAB 通用非对称迭代法"));
                        break;
                    case SolverNameSpace::LinearSolverMode::Cudss:
                        setWorkspaceMessage(QStringLiteral("线性求解方法已设置为 NVIDIA cuDSS 通用非对称直接法"));
                        break;
                }
            });

    m_settingsModuleLayout->insertWidget(1, m_settingsPanel);
    m_settingsModuleLayout->setStretch(1, 1);
    m_settingsModuleLayout->setStretch(2, 0);
}

bool YQY::event(QEvent* event)
{
    const bool handled = QMainWindow::event(event);
    if (event->type() == QEvent::DevicePixelRatioChange)
    {
        applyWindowsWindowIcon(this);
    }
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

void YQY::initializeResultModule()
{
    m_resultPanel = new ResultControlPanel(m_resultModulePage);
    m_resultCaseCombo->setMinimumWidth(0);
    m_resultCaseCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    m_resultCaseCombo->setMinimumContentsLength(8);
    m_resultCaseCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    connect(m_resultCaseCombo, &QComboBox::currentTextChanged, m_resultCaseCombo, &QWidget::setToolTip);
    connect(m_resultCaseCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int index)
            {
                if (index < 0)
                    return;
                loadSolveTaskResult(m_resultCaseCombo->itemData(index).toInt());
            });
    connect(m_resultPanel->stepCombo(), qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int index)
            {
                if (index >= 0)
                    applyResultStepFilter(m_resultPanel->stepCombo()->itemData(index).toInt());
            });
    setActionGlyph(m_resultPanel->exportButton(), ActionGlyph::Export);
    setActionGlyph(m_resultPanel->exportElementButton(), ActionGlyph::Export);
    setActionGlyph(m_resultPanel->exportIterationButton(), ActionGlyph::Export);
    setActionGlyph(m_resultPanel->exportPlotButton(), ActionGlyph::Plot);
    setActionGlyph(m_resultPanel->exportEnergyButton(), ActionGlyph::Export);
    m_resultModuleLayout->insertWidget(3, m_resultPanel);
    m_resultFileLabel->hide();
    m_resultTree->hide();
    m_resultTree->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
    m_resultTree->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Minimum);
    auto* sidebarScroll = new QScrollArea(m_resultModulePage);
    sidebarScroll->setObjectName(QStringLiteral("resultSidebarScroll"));
    sidebarScroll->setFrameShape(QFrame::NoFrame);
    sidebarScroll->setWidgetResizable(true);
    sidebarScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    sidebarScroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* sidebarBody = new QWidget(sidebarScroll);
    sidebarBody->setObjectName(QStringLiteral("resultSidebarBody"));
    auto* sidebarLayout = new QVBoxLayout(sidebarBody);
    sidebarLayout->setContentsMargins(0, 0, 0, 0);
    sidebarLayout->setSpacing(10);
    sidebarLayout->setSizeConstraint(QLayout::SetMinimumSize);
    while (QLayoutItem* item = m_resultModuleLayout->takeAt(0))
    {
        if (QWidget* widget = item->widget())
        {
            delete item;
            sidebarLayout->addWidget(widget);
        }
        else if (QLayout* childLayout = item->layout())
        {
            childLayout->setParent(nullptr);
            sidebarLayout->addLayout(childLayout);
        }
        else
        {
            sidebarLayout->addItem(item);
        }
    }
    sidebarLayout->addStretch();
    sidebarScroll->setWidget(sidebarBody);
    m_resultModuleLayout->addWidget(sidebarScroll, 1);
    m_nodeTimeHistoryPanel = new NodeTimeHistoryPanel(m_workspaceDisplayStack);
    m_workspaceDisplayStack->addWidget(m_nodeTimeHistoryPanel);
    m_nodeTimeHistoryPanel->setReturnToModelHandler(
        [this]()
        {
            m_workspaceDisplayStack->setCurrentWidget(m_modelViewport);
            setWorkspaceMessage(QStringLiteral("已返回模型视图"));
        });
    m_nodeTimeHistoryPanel->setEnergyStepChangedHandler(
        [this](int stepId)
        {
            auto* combo = m_resultPanel->stepCombo();
            const int index = combo->findData(stepId);
            if (index >= 0 && combo->currentIndex() != index)
                combo->setCurrentIndex(index);
        });
    m_nodeTimeHistoryPanel->setEnergyExportStateHandler(
        [this](bool enabled, bool busy)
        {
            auto* button = m_resultPanel->exportEnergyButton();
            button->setEnabled(enabled);
            button->setText(busy ? QStringLiteral("正在导出…") : QStringLiteral("导出能量"));
        });
    m_resultPanel->setFrameChangedHandler(
        [this](double framePosition)
        {
            displayResultPosition(framePosition);
        });
    m_resultPanel->setVisualizationChangedHandler(
        [this]()
        {
            updateResultVisualization();
        });
    m_resultPanel->setPlaybackStateChangedHandler(
        [this](bool playing)
        {
            // 大模型的标签生成和渲染开销较高，因此整个运行期间强制隐藏并禁用编号标签。
            const QSignalBlocker nodeIdsBlocker(m_showNodeIdsButton);
            const QSignalBlocker elementIdsBlocker(m_showElementIdsButton);
            m_modelViewport->setIdLabelsPlaybackLocked(playing);
            if (playing)
            {
                m_showNodeIdsButton->setChecked(false);
                m_showElementIdsButton->setChecked(false);
            }
            const bool idsAvailable = !playing && m_modelViewport->hasModel();
            m_showNodeIdsButton->setEnabled(idsAvailable);
            m_showElementIdsButton->setEnabled(idsAvailable);
            // 播放期间跳过实体重建以保持帧率；暂停后仅重建当前帧一次，
            // 保证实体显示与最终动画位置一致。
            if (!playing && !m_resultPanel->frames().empty())
                updateResultVisualization();
        });
    m_resultPanel->setExportHandler(
        [this]()
        {
            exportNodeResults();
        });
    m_resultPanel->setElementExportHandler(
        [this]()
        {
            exportElementResults();
        });
    m_resultPanel->setIterationExportHandler(
        [this]()
        {
            exportIterationHistory();
        });
    m_resultPanel->setPlotExportHandler(
        [this]()
        {
            showTimeHistory();
        });
    m_resultPanel->setEnergyExportHandler(
        [this]()
        {
            m_nodeTimeHistoryPanel->exportEnergyHistory();
        });
}

void YQY::initializeConductorModule()
{
    m_conductorModule = new ConductorModule(this);
    m_conductorModule->hide();
    auto* viewLibraryButton = m_conductorModule->viewLibraryButton();

    setActionGlyph(viewLibraryButton, ActionGlyph::Library);
    setActionGlyph(m_conductorModule->createButton(), ActionGlyph::CreateConductor);
    setActionGlyph(m_conductorModule->singleSpanButton(), ActionGlyph::CreateConductor);
    setActionGlyph(m_conductorModule->multiSpanButton(), ActionGlyph::CreateMultiSpanConductor);
    m_conductorModule->setPropertyLibrary(m_propertyLibrary.get());

    connect(viewLibraryButton, &QPushButton::clicked, this,
            [this]()
            {
                switchModule(Module::Properties);
            });
    connect(m_conductorModule->createButton(), &QPushButton::clicked, this, &YQY::createConductorModel);
}

void YQY::initializePropertyModule()
{
    m_propertyModule = new PropertyModule(m_moduleStack);
    setActionGlyph(m_propertyModule->refreshButton(), ActionGlyph::Refresh);
    setActionGlyph(m_propertyModule->applyButton(), ActionGlyph::Apply);
    m_moduleStack->insertWidget(1, m_propertyModule);

    connect(m_propertyModule->refreshButton(), &QPushButton::clicked, this, &YQY::refreshPropertyModule);
    connect(m_propertyModule->applyButton(), &QPushButton::clicked, this, &YQY::applyModelPropertyEdits);
    connect(m_propertyModule->materialTree(), &QTreeWidget::itemDoubleClicked, this, &YQY::editPropertyItem);
    connect(m_propertyModule->sectionTree(), &QTreeWidget::itemDoubleClicked, this, &YQY::editPropertyItem);
    connect(m_propertyModule->springBehaviorTree(), &QTreeWidget::itemDoubleClicked, this, &YQY::editPropertyItem);
    connect(m_modelDataTree, &QTreeWidget::itemDoubleClicked, this, &YQY::editPropertyItem);
}

void YQY::refreshPropertyModule()
{
    auto* m_materialTable = m_propertyModule ? m_propertyModule->materialTable() : nullptr;
    auto* m_sectionTable = m_propertyModule ? m_propertyModule->sectionTable() : nullptr;
    auto* m_materialPropertyTree = m_propertyModule ? m_propertyModule->materialTree() : nullptr;
    auto* m_sectionPropertyTree = m_propertyModule ? m_propertyModule->sectionTree() : nullptr;
    auto* m_springBehaviorTree = m_propertyModule ? m_propertyModule->springBehaviorTree() : nullptr;
    auto* m_propertyTabs = m_propertyModule ? m_propertyModule->tabs() : nullptr;
    auto* m_applyPropertyButton = m_propertyModule ? m_propertyModule->applyButton() : nullptr;
    if (!m_materialTable || !m_sectionTable || !m_materialPropertyTree || !m_sectionPropertyTree ||
        !m_springBehaviorTree || !m_propertyTabs || !m_applyPropertyButton)
        return;
    const QSignalBlocker materialBlocker(m_materialTable);
    const QSignalBlocker sectionBlocker(m_sectionTable);
    m_materialTable->setRowCount(0);
    m_sectionTable->setRowCount(0);
    m_materialPropertyTree->clear();
    m_sectionPropertyTree->clear();
    m_springBehaviorTree->clear();
    int materialItemCount = 0;
    int sectionItemCount = 0;
    int springBehaviorItemCount = 0;

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
        QTreeWidgetItem* materialGroup = nullptr;
        if (!structure->m_Material.empty())
        {
            materialGroup = new QTreeWidgetItem(m_materialPropertyTree,
                                                {QStringLiteral("模型 %1 · %2").arg(modelId).arg(modelName)});
            setTreeGlyph(materialGroup, TreeGlyph::Model);
        }
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

        QTreeWidgetItem* sectionGroup = nullptr;
        if (!structure->m_Section.empty())
        {
            sectionGroup = new QTreeWidgetItem(m_sectionPropertyTree,
                                               {QStringLiteral("模型 %1 · %2").arg(modelId).arg(modelName)});
            setTreeGlyph(sectionGroup, TreeGlyph::Model);
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

        if (!structure->m_SpringBehaviors.empty())
        {
            auto* behaviorGroup =
                new QTreeWidgetItem(m_springBehaviorTree, {QStringLiteral("模型 %1 · %2").arg(modelId).arg(modelName)});
            setTreeGlyph(behaviorGroup, TreeGlyph::Model);
            for (const auto& [behaviorId, behavior] : structure->m_SpringBehaviors)
            {
                if (!behavior)
                    continue;
                auto* item = new QTreeWidgetItem(
                    behaviorGroup,
                    {QStringLiteral("行为 %1 · 力-位移点 %2").arg(behaviorId).arg(behavior->m_Points.size())});
                item->setToolTip(0, QStringLiteral("双击查看力-位移数据"));
                item->setData(0, Qt::UserRole, 5);
                item->setData(0, Qt::UserRole + 1, modelId);
                item->setData(0, Qt::UserRole + 2, behaviorId);
                setTreeGlyph(item, TreeGlyph::SpringBehavior);
                ++springBehaviorItemCount;
            }
        }
    }
    m_materialPropertyTree->expandAll();
    m_sectionPropertyTree->expandAll();
    m_springBehaviorTree->expandAll();
    const ThemeColors propertyColors = themeColors(m_themeComboBox->currentIndex());
    applyTreeIcons(m_materialPropertyTree, QColor(propertyColors.text), QColor(propertyColors.accentSecond));
    applyTreeIcons(m_sectionPropertyTree, QColor(propertyColors.text), QColor(propertyColors.accentSecond));
    applyTreeIcons(m_springBehaviorTree, QColor(propertyColors.text), QColor(propertyColors.accentSecond));
    m_propertyTabs->setTabText(0, QStringLiteral("材料 (%1)").arg(materialItemCount));
    m_propertyTabs->setTabText(1, QStringLiteral("截面 (%1)").arg(sectionItemCount));
    m_propertyTabs->setTabText(2, QStringLiteral("弹簧行为 (%1)").arg(springBehaviorItemCount));
    m_applyPropertyButton->setEnabled(m_materialTable->rowCount() > 0 || m_sectionTable->rowCount() > 0);
}

void YQY::editPropertyItem(QTreeWidgetItem* item, int)
{
    if (!item)
        return;
    const int kind = item->data(0, Qt::UserRole).toInt();
    if (kind == 5)
    {
        const int modelId = item->data(0, Qt::UserRole + 1).toInt();
        const int behaviorId = item->data(0, Qt::UserRole + 2).toInt();
        const auto structure = m_modelController->model(modelId);
        const auto found = structure ? structure->m_SpringBehaviors.find(behaviorId)
                                     : std::map<int, std::shared_ptr<SpringBehavior>>::iterator{};
        if (!structure || found == structure->m_SpringBehaviors.end() || !found->second)
            return;

        QStringList points;
        for (const SpringForceDisplacementPoint& point : found->second->m_Points)
        {
            points.append(
                QStringLiteral("力 = %1，位移 = %2").arg(point.force, 0, 'g', 12).arg(point.displacement, 0, 'g', 12));
        }
        QMessageBox dialog(QMessageBox::Information, QStringLiteral("弹簧行为 %1").arg(behaviorId),
                           QStringLiteral("模型：%1\n\n力-位移数据：\n%2")
                               .arg(QFileInfo(m_modelController->filePath(modelId)).fileName())
                               .arg(points.isEmpty() ? QStringLiteral("--") : points.join(QStringLiteral("\n"))),
                           QMessageBox::Ok, this);
        dialog.setWindowIcon(UiIcons::applicationIcon());
        dialog.exec();
        return;
    }
    if (kind < 1 || kind > 4)
        return;

    PropertyItemEditorDialog dialog(this);
    dialog.setWindowIcon(UiIcons::applicationIcon());

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
                    young->interpretText();
                    poisson->interpretText();
                    density->interpretText();
                    stress->interpretText();
                    expansion->interpretText();
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
                    area->interpretText();
                    width->interpretText();
                    height->interpretText();
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
        appendLogMessage(QStringLiteral("[模型属性] 已保存属性修改；未改写模型源文件"));
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
    appendLogMessage(QStringLiteral("[模型属性] 已更新 %1 个材料、%2 个截面；源模型文本未修改")
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
            appendLogMessage(modelUpdated
                                 ? QStringLiteral("[属性库] 会话预设已保存，并已同步到当前模型（未改写文本文件）")
                                 : QStringLiteral("[属性库] 会话预设已保存（未改写文本文件）"));
            refreshModulePages();
            dialog.accept();
        });
    dialog.exec();
}

void YQY::createConductorModel()
{
    const QString generatedDirectory = ApplicationPaths::generatedModelDirectory();
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

    appendLogMessage(QStringLiteral("[导线建模] 已生成 %1 个节点、%2 个单元、%3 个相内间隔棒：%4")
                         .arg(result.nodeCount)
                         .arg(result.elementCount)
                         .arg(result.spacerCount)
                         .arg(QFileInfo(result.filePath).fileName()));
    appendLogMessage(QStringLiteral("[路径] 生成模型：%1").arg(result.filePath), true);
    const int modelId = m_modelController->adoptModel(result.structure, result.filePath);
    if (modelId > 0)
    {
        m_conductorModule->closeCreationDialog();
        switchModule(Module::Model);
        setWorkspaceMessage(QStringLiteral("导线模型已生成并加入当前工作区"));
    }
    else
    {
        QMessageBox::warning(this, QStringLiteral("创建导线模型"),
                             QStringLiteral("模型已生成并保存，但未能加入当前工作区。"));
    }
}

void YQY::initializeToolbarAppearance()
{
    const QList<QPair<QAbstractButton*, QString>> buttons = {
        {m_importButton, QStringLiteral("导入模型")},
        {m_undoButton, QStringLiteral("撤销上一次节点修改")},
        {m_selectModeButton, QStringLiteral("普通光标：默认单击或框选节点、单元；可拖动中心标记")},
        {m_rotateModeButton, QStringLiteral("旋转视图：拖动一次后返回普通光标")},
        {m_panModeButton, QStringLiteral("平移视图：拖动一次后返回普通光标")},
        {m_zoomModeButton, QStringLiteral("缩放：拖动或框选放大一次后返回普通光标")},
        {m_showNodesButton, QStringLiteral("显示或隐藏节点")},
        {m_showElementsButton, QStringLiteral("显示或隐藏单元")},
        {m_showSolidButton, QStringLiteral("显示或隐藏单元实体")},
        {m_showNodeIdsButton, QStringLiteral("显示或隐藏节点编号")},
        {m_showElementIdsButton, QStringLiteral("显示或隐藏单元编号")},
        {m_frontViewButton, QStringLiteral("前视图")},
        {m_backViewButton, QStringLiteral("后视图")},
        {m_leftViewButton, QStringLiteral("左视图")},
        {m_rightViewButton, QStringLiteral("右视图")},
        {m_topViewButton, QStringLiteral("顶视图")},
        {m_bottomViewButton, QStringLiteral("底视图")},
        {m_fitViewButton, QStringLiteral("适应窗口")},
        {m_startSolveButton, QStringLiteral("开始求解")}};

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
    m_toolBarLayout->setSpacing(4);
    const QList<QWidget*> groupStarts = {m_selectModeButton, m_showNodesButton, m_frontViewButton, m_fitViewButton};
    for (QWidget* groupStart : groupStarts)
    {
        auto* separator = new QFrame(m_toolBarCard);
        separator->setObjectName(QStringLiteral("toolbarSeparator"));
        separator->setFixedSize(1, 22);
        m_toolBarLayout->insertWidget(m_toolBarLayout->indexOf(groupStart), separator, 0, Qt::AlignVCenter);
    }
    m_modelImportButton->setProperty("primaryAction", true);
    m_stopButton->setProperty("dangerAction", true);
    m_logEdit->setMaximumBlockCount(3000);
    m_monitorLayout->setAlignment(Qt::AlignTop);
    m_incrementLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_convergenceLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    connect(m_solveProgress, &QProgressBar::valueChanged, this,
            [this](int value)
            {
                m_solveProgress->setFormat(QStringLiteral("%1%").arg(value / 10.0, 0, 'f', 1));
            });
    connect(m_logDetailsCheck, &QCheckBox::toggled, this, &YQY::rebuildLogView);
    connect(m_copyLogButton, &QPushButton::clicked, this,
            [this]()
            {
                QApplication::clipboard()->setText(m_logEdit->toPlainText());
            });
    connect(m_clearLogButton, &QPushButton::clicked, this,
            [this]()
            {
                m_logEntries.clear();
                m_logEdit->clear();
            });

    const QList<QPair<QPushButton*, QString>> navigationButtons = {
        {m_propertyButton, QStringLiteral("属性")},
        {m_modelButton, QStringLiteral("模型")},    {m_analysisButton, QStringLiteral("分析")},
        {m_solveButton, QStringLiteral("求解")},    {m_resultButton, QStringLiteral("结果")},
        {m_settingsButton, QStringLiteral("设置")}};
    for (const auto& [button, label] : navigationButtons)
    {
        button->setText(label);
        button->setIconSize(QSize(20, 20));
        button->setCursor(Qt::PointingHandCursor);
        button->setAccessibleName(label + QStringLiteral("模块"));
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    const QList<QPair<QPushButton*, QString>> actionButtons = {
        {m_modelImportButton, QStringLiteral("导入模型")},
        {m_createConductorButton, QStringLiteral("创建导线")},
        {m_openH5Button, QStringLiteral("打开 H5 结果文件")},
        {m_solvePageStartButton, QStringLiteral("运行全部分析步")},
        {m_stopButton, QStringLiteral("停止")}};
    for (const auto& [button, label] : actionButtons)
    {
        button->setText(label);
        button->setIconSize(QSize(20, 20));
        button->setCursor(Qt::PointingHandCursor);
        button->setAccessibleName(label);
        button->setMinimumHeight(qMax(38, fontMetrics().height() + 18));
    }
    setActionGlyph(m_importButton, ActionGlyph::Import);
    setActionGlyph(m_modelImportButton, ActionGlyph::Import);
    setActionGlyph(m_createConductorButton, ActionGlyph::CreateConductor);
    setActionGlyph(m_openH5Button, ActionGlyph::OpenFile);
    setActionGlyph(m_solvePageStartButton, ActionGlyph::Run);
    setActionGlyph(m_stopButton, ActionGlyph::Stop);
}

void YQY::initializeResponsiveLayout()
{
    // 这些尺寸保证模型可在较小的笔记本屏幕上使用，旧版固定最小宽度之和超过了可用宽度。
    m_navigationRail->setMinimumWidth(84);
    m_navigationRail->setMaximumWidth(84);
    // 侧栏可拖动，表单在较窄宽度下自行换行，保留模型视口的可用空间。
    m_projectPanel->setMinimumWidth(320);
    m_projectPanel->setMaximumWidth(520);
    m_workspacePanel->setMinimumWidth(300);
    m_mainSplitter->setHandleWidth(6);
    m_mainSplitter->setCollapsible(0, false);
    m_mainSplitter->setCollapsible(1, false);
    m_mainSplitter->setStretchFactor(0, 0);
    m_mainSplitter->setStretchFactor(1, 1);
    m_mainSplitter->setSizes(m_normalMainSplitterSizes);
    m_modelViewport->setMinimumSize(280, 160);
    m_monitorTabs->setMinimumHeight(100);
    m_workspaceDisplayStack->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_workspaceVerticalSplitter->setChildrenCollapsible(false);
    m_workspaceVerticalSplitter->setStretchFactor(0, 1);
    m_workspaceVerticalSplitter->setStretchFactor(1, 0);
    connect(m_monitorTabs, &QTabWidget::currentChanged, this,
            [this](int)
            {
                updateMonitorPanelHeight();
            });
    updateMonitorPanelHeight();
    connect(m_mainSplitter, &QSplitter::splitterMoved, this,
            [this](int, int)
            {
                updateResponsiveLayout();
            });

    const QList<QPushButton*> navigationButtons = {m_propertyButton, m_modelButton,
                                                   m_analysisButton, m_solveButton,     m_resultButton,
                                                   m_settingsButton};
    for (auto* button : navigationButtons)
    {
        button->setMinimumWidth(66);
        button->setMinimumHeight(qMax(58, fontMetrics().height() + 36));
    }

    const QList<QTreeWidget*> trees = {m_modelDataTree, m_analysisTree, m_solveStepTree, m_resultTree};
    for (auto* tree : trees)
    {
        tree->setIconSize(QSize(20, 20));
        tree->setTextElideMode(Qt::ElideMiddle);
        tree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        tree->setUniformRowHeights(true);
    }
    m_modelPropertyValue->setWordWrap(true);

    // 宽屏时拉伸工具栏背景但保持按钮逻辑尺寸稳定；窄屏使用水平滚动，不缩小按钮或增加工具栏高度。
    m_toolBarLayout->activate();
    m_toolbarNaturalWidth = m_toolBarCard->sizeHint().width();
    m_toolbarCardHeight = m_toolBarCard->sizeHint().height();
    m_toolBarCard->setMinimumWidth(m_toolbarNaturalWidth);
    m_toolBarCard->setFixedHeight(m_toolbarCardHeight);
    m_toolBarCard->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_toolbarScrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_toolbarScrollArea->setFixedHeight(m_toolbarCardHeight);

    updateResponsiveLayout();
    QTimer::singleShot(0, this, &YQY::updateResponsiveLayout);
}

void YQY::updateResponsiveLayout()
{
    const int availableWidth = width();
    const int projectWidth = m_projectPanel->width();

    // 任务监视器已经显示状态和耗时；项目面板变窄时优先保留算例名称，依次隐藏耗时和状态。
    const bool compactSolveTree = projectWidth < 360;
    const bool mediumSolveTree = projectWidth < 480;
    m_solveStepTree->setColumnHidden(2, compactSolveTree);
    m_solveStepTree->setColumnHidden(3, mediumSolveTree);
    m_solveStepTree->setColumnWidth(1, compactSolveTree ? 80 : 92);
    m_solveStepTree->setColumnWidth(2, 72);
    m_solveStepTree->setColumnWidth(3, 72);

    if (m_toolbarScrollArea && m_toolbarNaturalWidth > 0 && m_toolbarCardHeight > 0)
    {
        int viewportWidth = m_toolbarScrollArea->viewport()->width();
        if (viewportWidth <= 0)
            viewportWidth = qMax(0, m_workspacePanel->width() - m_workspaceLayout->contentsMargins().left() -
                                        m_workspaceLayout->contentsMargins().right());

        const bool needsHorizontalScroll = viewportWidth < m_toolbarNaturalWidth;
        m_toolbarScrollArea->setHorizontalScrollBarPolicy(needsHorizontalScroll ? Qt::ScrollBarAlwaysOn
                                                                                : Qt::ScrollBarAlwaysOff);
        const int scrollBarHeight = needsHorizontalScroll ? style()->pixelMetric(QStyle::PM_ScrollBarExtent) : 0;
        m_toolbarScrollArea->setFixedHeight(m_toolbarCardHeight + scrollBarHeight);
    }

    m_themeLabel->setVisible(availableWidth >= 820);
    m_readyLabel->setVisible(availableWidth >= 900);
    m_appTitleLabel->setVisible(availableWidth >= 680);
    updateSolveBatchActionsLayout();
    updateMonitorPanelHeight();
}

void YQY::updateSolveBatchActionsLayout()
{
    const QList<QPushButton*> buttons = {m_solveCasesButton, m_solveAllCasesButton, m_solveRestartAllCasesButton};
    if (!buttons.first())
        return;
    const int available = m_solveModulePage->width();
    const int spacing = m_solveBatchActionsLayout->horizontalSpacing();
    const int actionWidth = qMax(buttons.at(1)->sizeHint().width(), buttons.at(2)->sizeHint().width());
    const int allWidth = 3 * qMax(buttons.at(0)->sizeHint().width(), actionWidth) + 2 * spacing;
    const int actionsWidth = 2 * actionWidth + spacing;
    const int columns = available >= allWidth ? 3 : available >= actionsWidth ? 2 : 1;
    if (m_solveBatchActionColumns == columns)
        return;
    m_solveBatchActionColumns = columns;
    for (auto* button : buttons)
        m_solveBatchActionsLayout->removeWidget(button);
    for (int index = 0; index < 3; ++index)
        m_solveBatchActionsLayout->setColumnStretch(index, index < columns ? 1 : 0);
    if (columns == 3)
    {
        for (int index = 0; index < buttons.size(); ++index)
            m_solveBatchActionsLayout->addWidget(buttons.at(index), 0, index);
    }
    else if (columns == 2)
    {
        m_solveBatchActionsLayout->addWidget(buttons.at(0), 0, 0, 1, 2);
        m_solveBatchActionsLayout->addWidget(buttons.at(1), 1, 0);
        m_solveBatchActionsLayout->addWidget(buttons.at(2), 1, 1);
    }
    else
    {
        for (int index = 0; index < buttons.size(); ++index)
            m_solveBatchActionsLayout->addWidget(buttons.at(index), index, 0);
    }
}

void YQY::updateMonitorPanelHeight()
{
    if (m_monitorHeightUpdatePending)
        return;
    m_monitorHeightUpdatePending = true;
    QTimer::singleShot(0, this,
                       [this]()
                       {
                           m_monitorHeightUpdatePending = false;
                           m_monitorLayout->activate();
                           const int contentHeight = qMax(m_monitorLayout->sizeHint().height(),
                                                          m_logPage->minimumSizeHint().height());
                           const int height = contentHeight + m_monitorTabs->tabBar()->sizeHint().height() + 4;
                           m_monitorTabs->setMinimumHeight(height);
                           if (m_monitorPanelInitialized)
                               return;

                           // 两个页签共用紧凑初始高度，后续保留用户拖动的分割位置。
                           const int viewportHeight = m_workspaceVerticalSplitter->height() -
                                                      m_workspaceVerticalSplitter->handleWidth() - height;
                           m_workspaceVerticalSplitter->setSizes({qMax(160, viewportHeight), height});
                           m_monitorPanelInitialized = true;
                       });
}

QString YQY::elapsedTimeText(qint64 milliseconds)
{
    if (milliseconds < 1000)
        return QStringLiteral("%1 ms").arg(qMax<qint64>(0, milliseconds));
    if (milliseconds < 60000)
        return QStringLiteral("%1 秒").arg(milliseconds / 1000.0, 0, 'f', 1);
    return QStringLiteral("%1 分 %2 秒").arg(milliseconds / 60000).arg((milliseconds % 60000) / 1000);
}

void YQY::appendLogMessage(const QString& message, bool diagnostic)
{
    const QString text = message.trimmed();
    if (text.isEmpty())
        return;
    const QString entry =
        QStringLiteral("%1  %2").arg(QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss")), text);
    m_logEntries.push_back({entry, diagnostic});
    if (m_logEntries.size() > 3000)
        m_logEntries.removeFirst();
    if (diagnostic && !m_logDetailsCheck->isChecked())
        return;
    const auto* scrollBar = m_logEdit->verticalScrollBar();
    const int previousPosition = scrollBar->value();
    const bool atBottom = previousPosition >= scrollBar->maximum() - 2;
    m_logEdit->appendPlainText(entry);
    if (!atBottom)
        m_logEdit->verticalScrollBar()->setValue(previousPosition);
}

void YQY::rebuildLogView()
{
    QStringList lines;
    const bool detailed = m_logDetailsCheck->isChecked();
    for (const auto& entry : m_logEntries)
    {
        if (detailed || !entry.diagnostic)
            lines.push_back(entry.text);
    }
    m_logEdit->setPlainText(lines.join(QLatin1Char('\n')));
    m_logEdit->moveCursor(QTextCursor::End);
}

void YQY::updateToolbarIcons(int themeIndex)
{
    const ThemeColors colors = themeColors(themeIndex);
    const QColor foreground(colors.text);
    const QColor accent(colors.accentSecond);
    const QList<QPair<QAbstractButton*, ToolbarGlyph>> toolbarButtons = {
        {m_undoButton, ToolbarGlyph::Undo},
        {m_selectModeButton, ToolbarGlyph::Select},
        {m_rotateModeButton, ToolbarGlyph::Rotate},
        {m_panModeButton, ToolbarGlyph::Pan},
        {m_zoomModeButton, ToolbarGlyph::Zoom},
        {m_showNodesButton, ToolbarGlyph::Nodes},
        {m_showElementsButton, ToolbarGlyph::Elements},
        {m_showSolidButton, ToolbarGlyph::Solid},
        {m_showNodeIdsButton, ToolbarGlyph::NodeIds},
        {m_showElementIdsButton, ToolbarGlyph::ElementIds},
        {m_frontViewButton, ToolbarGlyph::Front},
        {m_backViewButton, ToolbarGlyph::Back},
        {m_leftViewButton, ToolbarGlyph::Left},
        {m_rightViewButton, ToolbarGlyph::Right},
        {m_topViewButton, ToolbarGlyph::Top},
        {m_bottomViewButton, ToolbarGlyph::Bottom},
        {m_fitViewButton, ToolbarGlyph::Fit},
        {m_startSolveButton, ToolbarGlyph::Solve}};
    for (const auto& [button, glyph] : toolbarButtons)
        button->setIcon(toolbarIcon(glyph, foreground, accent));

    const QList<QPair<NavigationButton*, NavigationGlyph>> navigationButtons = {
        {m_propertyButton, NavigationGlyph::Properties},
        {m_modelButton, NavigationGlyph::Model},         {m_analysisButton, NavigationGlyph::Analysis},
        {m_solveButton, NavigationGlyph::Solve},         {m_resultButton, NavigationGlyph::Results},
        {m_settingsButton, NavigationGlyph::Settings}};
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
    m_undoButton->setEnabled(false);
    m_documentTabBar->hide();
    m_modelDataTree->clear();
    auto* emptyModel = new QTreeWidgetItem(m_modelDataTree, {QStringLiteral("尚未加载模型")});
    setTreeGlyph(emptyModel, TreeGlyph::Model);

    m_showNodesButton->setEnabled(false);
    m_showElementsButton->setEnabled(false);
    m_showSolidButton->setEnabled(false);
    m_showNodeIdsButton->setEnabled(false);
    m_showElementIdsButton->setEnabled(false);
    m_frontViewButton->setEnabled(false);
    m_backViewButton->setEnabled(false);
    m_leftViewButton->setEnabled(false);
    m_rightViewButton->setEnabled(false);
    m_topViewButton->setEnabled(false);
    m_bottomViewButton->setEnabled(false);
    m_fitViewButton->setEnabled(false);
    m_selectModeButton->setEnabled(false);
    m_rotateModeButton->setEnabled(false);
    m_panModeButton->setEnabled(false);
    m_zoomModeButton->setEnabled(false);
    m_startSolveButton->setEnabled(false);
    m_stopButton->setEnabled(false);
    m_solveProgress->setValue(0);
    m_incrementLabel->setText(QStringLiteral("尚未开始求解"));
    m_convergenceLabel->setText(QStringLiteral("加载模型并配置分析步后可开始求解"));
    m_logEntries.clear();
    m_logEdit->clear();
    updateLoadStatistics();
    m_readyLabel->setText(QStringLiteral("等待导入模型"));
}

void YQY::initializeInteractions()
{
    connect(m_themeComboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, &YQY::applyTheme);

    auto* interactionGroup = new QButtonGroup(this);
    interactionGroup->setExclusive(true);
    interactionGroup->addButton(m_selectModeButton, static_cast<int>(ModelViewport::InteractionMode::Select));
    interactionGroup->addButton(m_rotateModeButton, static_cast<int>(ModelViewport::InteractionMode::Rotate));
    interactionGroup->addButton(m_panModeButton, static_cast<int>(ModelViewport::InteractionMode::Pan));
    interactionGroup->addButton(m_zoomModeButton, static_cast<int>(ModelViewport::InteractionMode::Zoom));

    connect(interactionGroup, &QButtonGroup::idClicked, this,
            [this](int id)
            {
                const auto mode = static_cast<ModelViewport::InteractionMode>(id);
                m_modelViewport->setInteractionMode(mode);
                const QStringList names = {QStringLiteral("选择"), QStringLiteral("旋转"), QStringLiteral("平移"),
                                           QStringLiteral("缩放")};
                setWorkspaceMessage(QStringLiteral("交互模式：%1").arg(names.at(id)));
            });

    connect(m_showNodesButton, &QToolButton::toggled, m_modelViewport, &ModelViewport::setNodesVisible);
    connect(m_showElementsButton, &QToolButton::toggled, m_modelViewport, &ModelViewport::setElementsVisible);
    connect(m_showSolidButton, &QToolButton::toggled, this,
            [this](bool visible)
            {
                // 与参考程序保持一致：实体显示时隐藏会穿插在实体表面的节点、
                // 节点编号及单元轴线；退出实体显示时恢复节点和单元轴线。
                const QSignalBlocker nodesBlocker(m_showNodesButton);
                const QSignalBlocker nodeIdsBlocker(m_showNodeIdsButton);
                const QSignalBlocker elementsBlocker(m_showElementsButton);

                m_modelViewport->setSolidVisible(visible);
                m_showNodesButton->setChecked(!visible);
                m_showNodeIdsButton->setChecked(false);
                m_showElementsButton->setChecked(!visible);
                m_modelViewport->setNodesVisible(!visible);
                m_modelViewport->setNodeLabelsVisible(false);
                m_modelViewport->setElementsVisible(!visible);

                setWorkspaceMessage(visible ? QStringLiteral("实体模式：已隐藏节点、节点编号和单元轴线")
                                            : QStringLiteral("已退出实体模式"));
            });
    connect(m_showNodeIdsButton, &QToolButton::toggled, m_modelViewport, &ModelViewport::setNodeLabelsVisible);
    connect(m_showElementIdsButton, &QToolButton::toggled, m_modelViewport, &ModelViewport::setElementLabelsVisible);
    connect(m_modelViewport, &ModelViewport::nodesVisibilityChanged, m_showNodesButton, &QToolButton::setChecked);
    connect(m_modelViewport, &ModelViewport::elementsVisibilityChanged, m_showElementsButton, &QToolButton::setChecked);
    connect(m_modelViewport, &ModelViewport::solidVisibilityChanged, m_showSolidButton, &QToolButton::setChecked);
    connect(m_modelViewport, &ModelViewport::nodeLabelsVisibilityChanged, m_showNodeIdsButton,
            &QToolButton::setChecked);
    connect(m_modelViewport, &ModelViewport::elementLabelsVisibilityChanged, m_showElementIdsButton,
            &QToolButton::setChecked);
    connect(m_modelViewport, &ModelViewport::interactionModeChanged, this,
            [this](ModelViewport::InteractionMode mode)
            {
                const QList<QAbstractButton*> buttons = {m_selectModeButton, m_rotateModeButton, m_panModeButton,
                                                         m_zoomModeButton};
                const int buttonIndex = mode == ModelViewport::InteractionMode::ZoomToRect
                                            ? static_cast<int>(ModelViewport::InteractionMode::Zoom)
                                            : static_cast<int>(mode);
                buttons.at(buttonIndex)->setChecked(true);
            });

    const QList<QPair<QToolButton*, ModelViewport::StandardView>> viewButtons = {
        {m_frontViewButton, ModelViewport::StandardView::Front},
        {m_backViewButton, ModelViewport::StandardView::Back},
        {m_leftViewButton, ModelViewport::StandardView::Left},
        {m_rightViewButton, ModelViewport::StandardView::Right},
        {m_topViewButton, ModelViewport::StandardView::Top},
        {m_bottomViewButton, ModelViewport::StandardView::Bottom}};
    for (const auto& [button, view] : viewButtons)
    {
        connect(button, &QToolButton::clicked, this,
                [this, view]()
                {
                    m_modelViewport->setStandardView(view);
                });
    }
    connect(m_fitViewButton, &QToolButton::clicked, m_modelViewport, &ModelViewport::resetCamera);

    connect(m_undoButton, &QPushButton::clicked, this,
            [this]()
            {
                if (m_modelController->undoLastChange())
                {
                    if (m_selectedNodeId >= 0)
                        showNodeProperties(m_selectedNodeId);
                    setWorkspaceMessage(QStringLiteral("已撤销上一次节点坐标修改"));
                }
            });
    connect(m_documentTabBar, &QTabBar::tabCloseRequested, this,
            [this](int tabIndex)
            {
                m_modelController->closeModel(m_documentTabBar->tabData(tabIndex).toInt());
            });
    connect(m_documentTabBar, &QTabBar::currentChanged, this,
            [this](int tabIndex)
            {
                if (tabIndex >= 0)
                    m_modelController->setActiveModel(m_documentTabBar->tabData(tabIndex).toInt());
            });
    const QList<QPair<QPushButton*, Module>> navigationItems = {
        {m_propertyButton, Module::Properties},
        {m_modelButton, Module::Model},         {m_analysisButton, Module::Analysis},
        {m_solveButton, Module::Solve},         {m_resultButton, Module::Result},
        {m_settingsButton, Module::Settings}};
    for (const auto& item : navigationItems)
    {
        connect(item.first, &QPushButton::clicked, this,
                [this, module = item.second]()
                {
                    switchModule(module);
                });
    }

    connect(m_modelImportButton, &QPushButton::clicked, m_importButton, &QPushButton::click);
    connect(m_createConductorButton, &QPushButton::clicked, this,
            [this]() { m_conductorModule->openCreationDialog(); });
    connect(m_openH5Button, &QPushButton::clicked, this, &YQY::openHdf5Result);
    connect(m_startSolveButton, &QPushButton::clicked, this, &YQY::submitSolveTask);
    connect(m_solvePageStartButton, &QPushButton::clicked, this, &YQY::submitSolveTask);
    connect(m_stopButton, &QPushButton::clicked, this, &YQY::cancelSelectedSolveTask);
    connect(m_solveStepTree, &QTreeWidget::itemSelectionChanged, this,
            [this]()
            {
                const int taskId = selectedSolveTaskId();
                updateTaskMonitor(taskId);
                loadSolveTaskResult(taskId);
            });
    connect(m_solveTaskController, &SolveTaskController::taskAdded, this,
            [this](int taskId)
            {
                refreshSolveTasks(taskId);
                appendLogMessage(QStringLiteral("[准备] 已创建待运行算例 %1，后台并发上限 %2")
                                     .arg(taskId)
                                     .arg(m_solveTaskController->maximumThreadCount()),
                                 true);
            });
    connect(m_solveTaskController, &SolveTaskController::taskUpdated, this,
            [this](int taskId)
            {
                updateSolveTaskRow(taskId);
                refreshSolveTaskManager();
                refreshResultCases();
                const auto info = m_solveTaskController->taskInfo(taskId);
                const bool finished = info.status == SolveTaskController::Status::Completed ||
                                      info.status == SolveTaskController::Status::Failed ||
                                      info.status == SolveTaskController::Status::Cancelled;
                if (!finished)
                    m_loggedFinishedTasks.remove(taskId);
                if (info.status == SolveTaskController::Status::Ready)
                    m_loggedStartedTasks.remove(taskId);
                if ((info.status == SolveTaskController::Status::Queued ||
                     info.status == SolveTaskController::Status::Running) &&
                    !m_loggedStartedTasks.contains(taskId))
                {
                    m_loggedStartedTasks.insert(taskId);
                    appendLogMessage(QStringLiteral("[求解] %1：%2").arg(info.name, info.message));
                }
                if (info.status == SolveTaskController::Status::Queued && m_resultPanel &&
                    QFileInfo(m_resultPanel->resultFilePath()).absoluteFilePath() ==
                        QFileInfo(info.outputFile).absoluteFilePath())
                {
                    // 结果读取器仍通过 ASCII 硬链接占用同一 NTFS 文件时，Windows 无法替换该 H5 文件。
                    m_workspaceController->removeModel(m_modelController->activeModelId());
                    clearActiveResultContext();
                }
                if (finished && !m_loggedFinishedTasks.contains(taskId))
                {
                    // 先登记结束事件，结果载入触发的嵌套更新不再重复输出或加载。
                    m_loggedFinishedTasks.insert(taskId);
                    m_loggedStartedTasks.remove(taskId);
                    QString loadFailure;
                    bool loaded = info.resultLoadRecorded;
                    if (!info.initialStatePreparation && info.hasUsableResult && !info.resultLoadRecorded &&
                        QFileInfo::exists(info.outputFile))
                    {
                        // 成功、失败或主动停止都预加载本次已经落盘的有效帧；是否进入
                        // 结果模块仍由用户决定。
                        if (loadHdf5Result(info.outputFile, false, false, info.partialResult, &loadFailure, false))
                        {
                            m_solveTaskController->recordResultLoaded(taskId);
                            loaded = true;
                        }
                    }
                    const auto finalInfo = m_solveTaskController->taskInfo(taskId);
                    const QString level = info.status == SolveTaskController::Status::Completed ? QStringLiteral("完成")
                                          : info.status == SolveTaskController::Status::Failed  ? QStringLiteral("错误")
                                                                                               : QStringLiteral("停止");
                    QString summary = QStringLiteral("[%1] %2：%3").arg(level, info.name, info.message.simplified());
                    if (info.solverElapsedMs > 0)
                        summary += QStringLiteral(" · 求解 %1").arg(elapsedTimeText(info.solverElapsedMs));
                    if (loaded)
                        summary += QStringLiteral(" · 结果 %1 帧 · 总耗时 %2")
                                       .arg(finalInfo.resultFrameCount)
                                       .arg(elapsedTimeText(finalInfo.elapsedMs));
                    appendLogMessage(summary);
                    if (!info.diagnostics.isEmpty())
                        appendLogMessage(QStringLiteral("[诊断] %1\n%2").arg(info.name, info.diagnostics), true);
                    if (!loadFailure.isEmpty())
                        appendLogMessage(QStringLiteral("[警告] 结果加载失败：%1").arg(loadFailure));
                }
            });

    connect(
        m_importButton, &QPushButton::clicked, this,
        [this]()
        {
            const QStringList paths = FileDialogService::selectOpenFiles(
                this, QStringLiteral("导入模型"), ApplicationPaths::importFileDirectory(),
                QStringLiteral("模型文件 (*.bdf);;H5模型文件 (*.h5 *.hdf5);;杆塔模型文件 (*.txt);;ABAQUS文件 (*.inp)"),
                QStringLiteral("modelImport"));
            if (paths.isEmpty())
                return;
            int accepted = 0;
            for (const QString& path : paths)
                if (loadModelFile(path))
                    ++accepted;
            setWorkspaceMessage(QStringLiteral("已加入 %1 个模型的读取队列").arg(accepted));
        });

    connect(m_modelViewport, &ModelViewport::nodeSelected, this, &YQY::showNodeProperties);
    connect(m_modelViewport, &ModelViewport::elementSelected, this, &YQY::showElementProperties);
    connect(m_modelViewport, &ModelViewport::selectionCleared, this, &YQY::clearSelectionProperties);
    connect(m_modelViewport, &ModelViewport::selectionChanged, this,
            [this](int nodeCount, int elementCount)
            {
                if (nodeCount + elementCount <= 1)
                    return;
                setWorkspaceMessage(QStringLiteral("已选择 %1 个节点、%2 个单元（Ctrl 点选可切换，Ctrl 框选可追加）")
                                        .arg(nodeCount)
                                        .arg(elementCount));
            });
    connect(m_modelController, &ModelController::nodePositionChanged, m_modelViewport,
            &ModelViewport::updateNodePosition);
    connect(m_modelController, &ModelController::loadStarted, this,
            [this](const QString& filePath)
            {
                ++m_modelLoadTotal;
                updateLoadStatistics();
                m_importButton->setEnabled(false);
                setWorkspaceMessage(QStringLiteral("正在后台读取模型：%1").arg(QFileInfo(filePath).fileName()));
                appendLogMessage(QStringLiteral("[模型] 开始读取：%1").arg(QFileInfo(filePath).fileName()));
                appendLogMessage(QStringLiteral("[路径] 源模型：%1").arg(filePath), true);
            });
    connect(m_modelController, &ModelController::loadSucceeded, this, &YQY::handleModelLoaded);
    connect(m_modelController, &ModelController::loadFailed, this, &YQY::handleModelLoadFailed);
    connect(m_modelController, &ModelController::activeModelChanged, this, &YQY::handleActiveModelChanged);
    connect(m_modelController, &ModelController::modelClosed, this,
            [this](int modelId)
            {
                const QSignalBlocker blocker(m_documentTabBar);
                for (int index = 0; index < m_documentTabBar->count(); ++index)
                {
                    if (m_documentTabBar->tabData(index).toInt() == modelId)
                    {
                        m_documentTabBar->removeTab(index);
                        break;
                    }
                }
                m_workspaceController->removeModel(modelId);
                refreshModulePages();
            });
    connect(m_modelController, &ModelController::busyChanged, this,
            [this](bool busy)
            {
                m_importButton->setEnabled(!busy);
            });
    connect(m_modelController, &ModelController::undoAvailabilityChanged, m_undoButton, &QPushButton::setEnabled);
    connect(m_modelController, &ModelController::modelCleared, this,
            [this]()
            {
                closeAnalysisManagers();
                const QSignalBlocker blocker(m_documentTabBar);
                while (m_documentTabBar->count() > 0)
                    m_documentTabBar->removeTab(0);
                m_workspaceController->clear();
                m_modelViewport->clearModel();
                m_modelDataTree->clear();
                new QTreeWidgetItem(m_modelDataTree, {QStringLiteral("尚未加载模型")});
                m_showNodesButton->setChecked(true);
                m_showElementsButton->setChecked(true);
                m_showSolidButton->setChecked(false);
                m_showNodeIdsButton->setChecked(false);
                m_showElementIdsButton->setChecked(false);
                setModelControlsEnabled(false);
                clearSelectionProperties();
                appendLogMessage(QStringLiteral("[INFO] 已关闭全部模型"));
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
    // 弹出窗口和模态对话框是独立的 Qt 顶层窗口。若主题仅应用于主窗口，它们仍使用 Windows 默认边框和
    // 选中调色板，会在深色控件中出现白色边缘和系统蓝色行。
    QApplication::setPalette(themedPalette);
    qApp->setStyleSheet(buildStyleSheet(colors));
    for (QWidget* topLevelWidget : QApplication::topLevelWidgets())
        applyWindowsTitleBarTheme(topLevelWidget);
    updateToolbarIcons(themeIndex);
    m_modelViewport->setThemeIndex(themeIndex);
    const QColor treeColor(colors.text);
    const QColor treeAccent(colors.accentSecond);
    applyTreeIcons(m_modelDataTree, treeColor, treeAccent);
    applyTreeIcons(m_analysisTree, treeColor, treeAccent);
    applyTreeIcons(m_solveStepTree, treeColor, treeAccent);
    applyTreeIcons(m_resultTree, treeColor, treeAccent);
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
    if (pageIndex < 0 || pageIndex >= m_moduleStack->count())
        return;

    m_moduleStack->setCurrentIndex(pageIndex);
    QStringList titles = {QStringLiteral("模型数据"), QStringLiteral("分析"), QStringLiteral("求解"),
                          QStringLiteral("结果"), QStringLiteral("设置")};
    titles.insert(1, QStringLiteral("材料与截面属性"));
    m_projectTitleLabel->setText(titles.at(pageIndex));

    const bool modelEditing = module == Module::Model;
    m_importButton->setVisible(modelEditing);
    m_undoButton->setVisible(modelEditing);
    m_startSolveButton->setVisible(false);
    m_monitorTabs->setCurrentIndex(module == Module::Solve ? 0 : 1);
    if (module != Module::Result)
        m_workspaceDisplayStack->setCurrentWidget(m_modelViewport);

    switch (module)
    {
        case Module::Properties:
            m_propertyButton->setChecked(true);
            break;
        case Module::Model:
            m_modelButton->setChecked(true);
            break;
        case Module::Analysis:
            m_analysisButton->setChecked(true);
            break;
        case Module::Solve:
            m_solveButton->setChecked(true);
            break;
        case Module::Result:
            m_resultButton->setChecked(true);
            break;
        case Module::Settings:
            m_settingsButton->setChecked(true);
            break;
    }

    refreshModulePages();
    setWorkspaceMessage(QStringLiteral("已进入%1模块").arg(titles.at(pageIndex)));
    QTimer::singleShot(0, this, &YQY::updateSolveBatchActionsLayout);
}

void YQY::refreshModulePages()
{
    const auto structure = m_modelController->model();
    const bool hasModel = static_cast<bool>(structure);

    m_modelFileValue->setText(hasModel ? QFileInfo(m_modelController->filePath()).fileName()
                                       : QStringLiteral("尚未加载模型"));
    m_modelNodeValue->setText(hasModel ? QStringLiteral("节点 %1").arg(structure->Nodes().size())
                                       : QStringLiteral("节点 --"));
    m_modelElementValue->setText(hasModel ? QStringLiteral("单元 %1").arg(structure->Elements().size())
                                          : QStringLiteral("单元 --"));
    QStringList modelDataSummary;
    if (hasModel)
    {
        if (!structure->m_Material.empty())
            modelDataSummary.append(QStringLiteral("材料 %1").arg(structure->m_Material.size()));
        if (!structure->m_Section.empty())
            modelDataSummary.append(QStringLiteral("截面 %1").arg(structure->m_Section.size()));
        if (!structure->m_SpringBehaviors.empty())
            modelDataSummary.append(QStringLiteral("弹簧行为 %1").arg(structure->m_SpringBehaviors.size()));
    }
    m_modelPropertyValue->setText(hasModel ? modelDataSummary.join(QStringLiteral(" · "))
                                           : QStringLiteral("材料 -- · 截面 --"));
    m_createConductorButton->setEnabled(m_propertyLibrary && m_propertyLibrary->isReady());

    m_analysisTree->clear();
    if (hasModel)
    {
        auto* steps =
            new QTreeWidgetItem(m_analysisTree, {QStringLiteral("分析步  %1").arg(structure->AnalysisSteps().size())});
        setTreeGlyph(steps, TreeGlyph::AnalysisSteps);
        for (const auto& [stepId, step] : structure->AnalysisSteps())
        {
            const QString stepName = step && !step->m_Name.trimmed().isEmpty() ? step->m_Name.trimmed()
                                                                               : QStringLiteral("Step-%1").arg(stepId);
            const QString text =
                QStringLiteral("%1  ·  %2").arg(stepName).arg(step ? step->GetTypeName() : QStringLiteral("UNKNOWN"));
            auto* stepItem = new QTreeWidgetItem(steps, {text});
            setTreeGlyph(stepItem, TreeGlyph::AnalysisStep);
        }
        auto* loads = new QTreeWidgetItem(m_analysisTree, {QStringLiteral("载荷  %1").arg(structure->m_Load.size())});
        setTreeGlyph(loads, TreeGlyph::Load);
        auto* constraints =
            new QTreeWidgetItem(m_analysisTree, {QStringLiteral("约束  %1").arg(structure->m_Constraint.size())});
        setTreeGlyph(constraints, TreeGlyph::Constraint);
        auto* mpcs =
            new QTreeWidgetItem(m_analysisTree, {QStringLiteral("MPC  %1").arg(structure->m_MPCConstraints.size())});
        setTreeGlyph(mpcs, TreeGlyph::MPC);
        for (const auto& [mpcId, mpc] : structure->m_MPCConstraints)
        {
            if (!mpc)
                continue;
            const std::vector<int> nodeIds = mpc->GetNodeIds();
            const QString directions = [&mpc]()
            {
                QString value;
                for (const int direction : mpc->m_SlaveDirections)
                    value.append(QString::number(direction));
                return value;
            }();
            auto* mpcItem = new QTreeWidgetItem(
                mpcs, {QStringLiteral("%1  ·  %2 → %3  ·  %4")
                           .arg(mpc->m_Name.trimmed().isEmpty() ? QStringLiteral("MPC-%1").arg(mpcId) : mpc->m_Name)
                           .arg(nodeIds.size() == 2 ? QString::number(nodeIds[0]) : QStringLiteral("--"))
                           .arg(nodeIds.size() == 2 ? QString::number(nodeIds[1]) : QStringLiteral("--"))
                           .arg(directions)});
            setTreeGlyph(mpcItem, TreeGlyph::MPC);
        }
        auto* regions = new QTreeWidgetItem(m_analysisTree,
                                            {QStringLiteral("计算区域  %1").arg(structure->m_ComputeRegions.size())});
        setTreeGlyph(regions, TreeGlyph::Model);
        for (const auto& [regionId, region] : structure->m_ComputeRegions)
        {
            Q_UNUSED(regionId);
            if (!region)
                continue;
            auto* regionItem = new QTreeWidgetItem(regions, {QStringLiteral("%1  ·  节点 %2  ·  单元 %3")
                                                                 .arg(region->m_Name)
                                                                 .arg(region->m_NodeIds.size())
                                                                 .arg(region->m_ElementIds.size())});
            setTreeGlyph(regionItem, TreeGlyph::Model);
        }
        m_analysisTree->expandAll();
    }
    else
    {
        auto* emptyAnalysis = new QTreeWidgetItem(m_analysisTree, {QStringLiteral("尚未加载模型")});
        setTreeGlyph(emptyAnalysis, TreeGlyph::Model);
    }

    const ThemeColors treeColors = themeColors(m_themeComboBox->currentIndex());
    applyTreeIcons(m_analysisTree, QColor(treeColors.text), QColor(treeColors.accentSecond));

    const bool canSolve = hasModel && !structure->AnalysisSteps().empty();
    m_solveReadinessLabel->setText(
        canSolve   ? QStringLiteral("模型已就绪，共 %1 个分析步").arg(structure->AnalysisSteps().size())
        : hasModel ? QStringLiteral("当前模型没有可运行的分析步")
                   : QStringLiteral("请先加载包含分析步的模型"));
    m_solvePageStartButton->setEnabled(canSolve);
    m_startSolveButton->setEnabled(canSolve);
    refreshSolveTasks();
    refreshAnalysisEditor();
    refreshPropertyModule();
}

void YQY::initializeAnalysisEditor()
{
    m_analysisPanel = new AnalysisSummaryPanel(m_analysisModulePage);
    m_analysisPanel->stepsButton()->setObjectName(QStringLiteral("analysisStepsButton"));
    m_analysisPanel->loadsButton()->setObjectName(QStringLiteral("analysisLoadsButton"));
    m_analysisPanel->constraintsButton()->setObjectName(QStringLiteral("analysisConstraintsButton"));
    m_analysisPanel->mpcsButton()->setObjectName(QStringLiteral("analysisMPCsButton"));
    m_analysisPanel->regionsButton()->setObjectName(QStringLiteral("analysisRegionsButton"));
    m_analysisModuleLayout->insertWidget(1, m_analysisPanel);
    connect(m_analysisPanel->stepsButton(), &QPushButton::clicked, this,
            [this]()
            {
                openAnalysisManager(static_cast<int>(AnalysisManagerDialog::Page::Steps));
            });
    connect(m_analysisPanel->loadsButton(), &QPushButton::clicked, this,
            [this]()
            {
                openAnalysisManager(static_cast<int>(AnalysisManagerDialog::Page::Loads));
            });
    connect(m_analysisPanel->constraintsButton(), &QPushButton::clicked, this,
            [this]()
            {
                openAnalysisManager(static_cast<int>(AnalysisManagerDialog::Page::Constraints));
            });
    connect(m_analysisPanel->mpcsButton(), &QPushButton::clicked, this,
            [this]()
            {
                openAnalysisManager(static_cast<int>(AnalysisManagerDialog::Page::MPCs));
            });
    connect(m_analysisPanel->regionsButton(), &QPushButton::clicked, this, &YQY::openComputeRegionManager);
    connect(m_analysisTree, &QTreeWidget::itemDoubleClicked, this,
            [this](QTreeWidgetItem* item, int)
            {
                if (!item)
                    return;
                while (item->parent())
                    item = item->parent();
                const int page = m_analysisTree->indexOfTopLevelItem(item);
                if (page >= 0 && page <= 3 && m_modelController->model())
                    openAnalysisManager(page);
                else if (page == 4 && m_modelController->model())
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
    for (const auto& [stepId, step] : structure->AnalysisSteps())
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
    auto* mpcsButton = m_analysisPanel->mpcsButton();
    auto* regionsButton = m_analysisPanel->regionsButton();
    stepsButton->setEnabled(enabled);
    loadsButton->setEnabled(enabled);
    constraintsButton->setEnabled(enabled);
    mpcsButton->setEnabled(enabled);
    regionsButton->setEnabled(enabled);
    stepsButton->setText(QStringLiteral("分析步 · %1").arg(structure ? structure->AnalysisSteps().size() : 0));
    loadsButton->setText(QStringLiteral("荷载 · %1").arg(structure ? structure->m_Load.size() : 0));
    constraintsButton->setText(QStringLiteral("约束 · %1").arg(structure ? structure->m_Constraint.size() : 0));
    mpcsButton->setText(QStringLiteral("MPC · %1").arg(structure ? structure->m_MPCConstraints.size() : 0));
    regionsButton->setText(QStringLiteral("计算区域 · %1").arg(structure ? structure->m_ComputeRegions.size() : 0));
    const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
    stepsButton->setIcon(treeIcon(TreeGlyph::AnalysisSteps, QColor(colors.text), QColor(colors.accentSecond)));
    loadsButton->setIcon(treeIcon(TreeGlyph::Load, QColor(colors.text), QColor(colors.accentSecond)));
    constraintsButton->setIcon(treeIcon(TreeGlyph::Constraint, QColor(colors.text), QColor(colors.accentSecond)));
    mpcsButton->setIcon(treeIcon(TreeGlyph::MPC, QColor(colors.text), QColor(colors.accentSecond)));
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
    const int boundedPage = qBound(0, initialPage, 3);
    QPointer<AnalysisManagerDialog>* managerSlot = nullptr;
    if (boundedPage == static_cast<int>(AnalysisManagerDialog::Page::Steps))
        managerSlot = &m_analysisStepManager;
    else if (boundedPage == static_cast<int>(AnalysisManagerDialog::Page::Loads))
        managerSlot = &m_analysisLoadManager;
    else if (boundedPage == static_cast<int>(AnalysisManagerDialog::Page::Constraints))
        managerSlot = &m_analysisConstraintManager;
    else
        managerSlot = &m_analysisMPCManager;

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
    else if (boundedPage == static_cast<int>(AnalysisManagerDialog::Page::Constraints))
        manager = new AnalysisConstraintManagerDialog(structure, this);
    else
        manager = new AnalysisMPCManagerDialog(structure, this);

    *managerSlot = manager;
    manager->setAttribute(Qt::WA_DeleteOnClose);
    manager->setWindowModality(Qt::NonModal);
    manager->setModelChangedCallback(
        [this](const QSet<int>& affectedStepIds)
        {
            handleAnalysisResourcesChanged(affectedStepIds);
        });
    manager->setOpenManagerCallback(
        [this](AnalysisManagerDialog::Page page)
        {
            openAnalysisManager(static_cast<int>(page));
        });
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
    appendLogMessage(QStringLiteral("[INFO] 分析资源已增量更新：影响 %1 个分析步，准备 %2 个算例；"
                                    "当前共 %3 个分析步、%4 个荷载、%5 个约束、%6 个 MPC")
                         .arg(affectedStepIds.size())
                         .arg(preparedCount)
                         .arg(structure->AnalysisSteps().size())
                         .arg(structure->m_Load.size())
                         .arg(structure->m_Constraint.size())
                         .arg(structure->m_MPCConstraints.size()));
    setWorkspaceMessage(QStringLiteral("分析资源已增量同步，未受影响的计算状态保持不变"));
    refreshModulePages();

    if (m_analysisStepManager)
        m_analysisStepManager->refreshFromModel();
    if (m_analysisLoadManager)
        m_analysisLoadManager->refreshFromModel();
    if (m_analysisConstraintManager)
        m_analysisConstraintManager->refreshFromModel();
    if (m_analysisMPCManager)
        m_analysisMPCManager->refreshFromModel();
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
    if (m_analysisMPCManager)
    {
        auto* manager = m_analysisMPCManager.data();
        m_analysisMPCManager.clear();
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
    for (const auto& [stepId, step] : structure->AnalysisSteps())
    {
        if (step)
            m_solveTaskController->prepare(structure, sourceFile, stepId);
    }
}

void YQY::submitSolveTask()
{
    const auto structure = m_modelController->model();
    if (!structure || structure->AnalysisSteps().empty())
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
    const QVariant monitoredTask = m_solveProgress->property("monitoredTaskId");
    const int taskId = monitoredTask.isValid() ? monitoredTask.toInt() : selectedSolveTaskId();
    if (taskId < 0 || !m_solveTaskController->cancel(taskId))
        return;
    setWorkspaceMessage(QStringLiteral("已请求停止算例 %1").arg(taskId));
}

int YQY::selectedSolveTaskId() const
{
    const auto selected = m_solveStepTree->selectedItems();
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
    const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
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
    connect(dialog, &QObject::destroyed, this,
            [this]()
            {
                m_solveTaskManager = nullptr;
            });
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
    setWorkspaceMessage(requested > 0
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
    const QList<int> ids = m_solveTaskController->analysisTaskIds();
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
    updateSolveBatchActionsLayout();
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
    const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
    for (int row = 0; row < ids.size(); ++row)
    {
        const int id = ids.at(row);
        const auto info = m_solveTaskController->displayTaskInfo(id);
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
        if (info.initialStatePreparation)
        {
            action->setToolTip(stoppable  ? QStringLiteral("停止无风初态（等待该初态的动力步将不启动）")
                               : stopping ? QStringLiteral("无风初态正在停止")
                                          : QStringLiteral("单独运行原静力步"));
        }
        else
            action->setToolTip(stoppable ? QStringLiteral("停止当前算例") : QStringLiteral("运行当前算例"));
        connect(action, &QPushButton::clicked, this,
                [this, id]()
                {
                    const auto current = m_solveTaskController->displayTaskInfo(id);
                    if (current.status == SolveTaskController::Status::Queued ||
                        current.status == SolveTaskController::Status::Running)
                        m_solveTaskController->cancel(current.id);
                    else if (current.status != SolveTaskController::Status::Cancelling)
                        m_solveTaskController->start(id);
                });
        m_solveTaskTable->setCellWidget(row, 0, action);

        auto* nameItem = new QTableWidgetItem(info.name);
        nameItem->setData(Qt::UserRole, id);
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
        statusItem->setToolTip(info.message);
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
    // 重建列表时不触发结果加载，避免切换结果模型后嵌套刷新并删除当前正在使用的条目。
    const QSignalBlocker selectionBlocker(m_solveStepTree);
    m_solveStepTree->clear();

    const QList<int> ids = m_solveTaskController->analysisTaskIds();
    refreshSolveTaskManager();
    if (ids.isEmpty())
    {
        auto* placeholder =
            new QTreeWidgetItem(m_solveStepTree, {QStringLiteral("尚未提交算例"), QStringLiteral("0.0%"),
                                                  QStringLiteral("就绪"), QStringLiteral("-- ms")});
        placeholder->setFlags(placeholder->flags() & ~Qt::ItemIsSelectable);
        for (int column = 0; column < m_solveStepTree->columnCount(); ++column)
            placeholder->setTextAlignment(column, Qt::AlignCenter);
        setTreeGlyph(placeholder, TreeGlyph::SolveTask);
        const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
        applyTreeIcons(m_solveStepTree, QColor(colors.text), QColor(colors.accentSecond));
        m_stopButton->setEnabled(false);
        return;
    }

    QTreeWidgetItem* selectedItem = nullptr;
    for (int id : ids)
    {
        const auto info = m_solveTaskController->displayTaskInfo(id);
        auto* item =
            new QTreeWidgetItem(m_solveStepTree, {QString(), QString(), solveTaskDisplayStatus(info), QString()});
        item->setData(0, Qt::UserRole, id);
        item->setToolTip(0, info.message);
        item->setTextAlignment(2, Qt::AlignCenter);
        item->setTextAlignment(3, Qt::AlignCenter);

        auto* progress = new QProgressBar(m_solveStepTree);
        progress->setObjectName(QStringLiteral("solveTaskProgress"));
        progress->setRange(0, 1000);
        progress->setValue(qRound(info.progress * 1000.0));
        progress->setTextVisible(true);
        progress->setFormat(QStringLiteral("%1%").arg(info.progress * 100.0, 0, 'f', 1));
        connect(progress, &QProgressBar::valueChanged, progress,
                [progress](int value)
                {
                    progress->setFormat(QStringLiteral("%1%").arg(value / 10.0, 0, 'f', 1));
                });
        m_solveStepTree->setItemWidget(item, 1, progress);
        auto* caseWidget = new QWidget(m_solveStepTree);
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
        auto* caseLabel = new SingleLineLabel(caseWidget);
        caseLabel->setText(info.name);
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
                    m_solveStepTree->setCurrentItem(item);
                    const auto current = m_solveTaskController->displayTaskInfo(id);
                    if (current.status == SolveTaskController::Status::Queued ||
                        current.status == SolveTaskController::Status::Running)
                    {
                        if (m_solveTaskController->cancel(current.id))
                            setWorkspaceMessage(QStringLiteral("已请求停止算例 %1").arg(current.id));
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
        m_solveStepTree->setItemWidget(item, 0, caseWidget);
        item->setSizeHint(0, QSize(0, 40));
        if (id == selectedId)
            selectedItem = item;
        updateSolveTaskRow(id);
    }

    if (!selectedItem)
        selectedItem = m_solveStepTree->topLevelItem(m_solveStepTree->topLevelItemCount() - 1);
    m_solveStepTree->setCurrentItem(selectedItem);
    const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
    applyTreeIcons(m_solveStepTree, QColor(colors.text), QColor(colors.accentSecond));
    updateTaskMonitor(selectedItem ? selectedItem->data(0, Qt::UserRole).toInt() : -1);

    const int running = m_solveTaskController->runningTaskCount();
    if (running > 0)
        m_solveReadinessLabel->setText(QStringLiteral("%1 个后台任务正在运行 · 最多 %2 个并发线程")
                                           .arg(running)
                                           .arg(m_solveTaskController->maximumThreadCount()));
}

void YQY::updateSolveTaskRow(int taskId)
{
    const int selectedTaskId = selectedSolveTaskId();
    const auto selectedInfo = m_solveTaskController->taskInfo(selectedTaskId);
    const auto selectedDisplayInfo = m_solveTaskController->displayTaskInfo(selectedTaskId);
    if (selectedTaskId == taskId || selectedDisplayInfo.id == taskId || selectedInfo.initialStaticTaskId == taskId)
        updateTaskMonitor(selectedTaskId);

    const int running = m_solveTaskController->runningTaskCount();
    if (running > 0)
        m_solveReadinessLabel->setText(QStringLiteral("%1 个后台任务正在运行 · 最多 %2 个并发线程")
                                           .arg(running)
                                           .arg(m_solveTaskController->maximumThreadCount()));
    else
        m_solveReadinessLabel->setText(QStringLiteral("所有已提交算例均已结束"));

    for (int row = 0; row < m_solveStepTree->topLevelItemCount(); ++row)
    {
        auto* item = m_solveStepTree->topLevelItem(row);
        if (!item || !item->data(0, Qt::UserRole).isValid())
            continue;
        const int logicalTaskId = item->data(0, Qt::UserRole).toInt();
        const auto rawInfo = m_solveTaskController->taskInfo(logicalTaskId);
        const auto info = m_solveTaskController->displayTaskInfo(logicalTaskId);
        if (logicalTaskId != taskId && info.id != taskId && rawInfo.initialStaticTaskId != taskId)
            continue;

        auto* progress = qobject_cast<QProgressBar*>(m_solveStepTree->itemWidget(item, 1));
        if (progress)
            ProgressBarAnimation::animateTo(progress, qRound(info.progress * 1000.0));

        item->setText(2, solveTaskDisplayStatus(info));
        item->setToolTip(0, info.name + QStringLiteral("\n") + info.message);
        item->setToolTip(2, info.message);
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
        item->setForeground(2, statusColor.isValid() ? QBrush(statusColor) : QBrush());

        QWidget* caseWidget = m_solveStepTree->itemWidget(item, 0);
        if (!caseWidget)
            continue;
        auto* controlButton = caseWidget->findChild<QToolButton*>(QStringLiteral("taskControlButton"));
        auto* caseLabel = caseWidget->findChild<QLabel*>(QStringLiteral("solveCaseLabel"));
        if (caseLabel)
        {
            caseLabel->setText(info.name);
            caseLabel->setToolTip(info.name + QStringLiteral("\n") + info.message);
        }
        if (!controlButton)
            continue;
        const bool stoppable =
            info.status == SolveTaskController::Status::Queued || info.status == SolveTaskController::Status::Running;
        const bool restartable = info.status == SolveTaskController::Status::Ready ||
                                 info.status == SolveTaskController::Status::Completed ||
                                 info.status == SolveTaskController::Status::Failed ||
                                 info.status == SolveTaskController::Status::Cancelled;
        const bool stopping = info.status == SolveTaskController::Status::Cancelling;
        const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
        controlButton->setIcon(treeIcon((stoppable || stopping) ? TreeGlyph::StopTask : TreeGlyph::SolveTask,
                                        QColor(colors.text), QColor(colors.accentSecond)));
        controlButton->setEnabled(stoppable || restartable);
        if (info.initialStatePreparation)
        {
            controlButton->setToolTip(stoppable  ? QStringLiteral("停止无风初态（等待该初态的动力步将不启动）")
                                      : stopping ? QStringLiteral("无风初态正在停止")
                                                 : QStringLiteral("单独运行原静力步"));
        }
        else
        {
            controlButton->setToolTip(stoppable     ? QStringLiteral("仅停止算例 %1").arg(logicalTaskId)
                                      : restartable ? QStringLiteral("开始运行算例 %1").arg(logicalTaskId)
                                                    : QStringLiteral("算例正在停止"));
        }
        controlButton->setAccessibleName(controlButton->toolTip());
    }
}

void YQY::updateTaskMonitor(int taskId)
{
    if (taskId < 0)
    {
        m_solveProgress->setProperty("monitoredTaskId", QVariant());
        m_stopButton->setEnabled(false);
        return;
    }

    auto info = m_solveTaskController->displayTaskInfo(taskId);
    if (info.status != SolveTaskController::Status::Running && info.initialStaticTaskId >= 0)
    {
        const auto staticInfo = m_solveTaskController->taskInfo(info.initialStaticTaskId);
        if (staticInfo.status == SolveTaskController::Status::Queued ||
            staticInfo.status == SolveTaskController::Status::Running ||
            staticInfo.status == SolveTaskController::Status::Cancelling)
            info = staticInfo;
    }
    m_solveProgress->setProperty("monitoredTaskId", info.id);
    m_solveProgress->setRange(0, 1000);
    ProgressBarAnimation::animateTo(m_solveProgress, qRound(info.progress * 1000.0));
    m_solveProgress->setFormat(QStringLiteral("%1%").arg(m_solveProgress->value() / 10.0, 0, 'f', 1));
    m_incrementLabel->setText(QStringLiteral("%1 · %2 · 已用时 %3")
                                  .arg(info.name, solveTaskDisplayStatus(info))
                                  .arg(elapsedTimeText(info.elapsedMs)));
    m_convergenceLabel->setText(info.message);
    const bool active =
        info.status == SolveTaskController::Status::Queued || info.status == SolveTaskController::Status::Running;
    m_stopButton->setEnabled(active);
    m_stopButton->setToolTip(info.initialStatePreparation ? QStringLiteral("停止无风初态（等待该初态的动力步将不启动）")
                                                          : QStringLiteral("停止当前算例"));
}

void YQY::openHdf5Result()
{
    const QString initialDirectory = ApplicationPaths::hdf5ResultDirectory(m_modelController->filePath());

    const QStringList selectedFiles = FileDialogService::selectOpenFiles(
        this, QStringLiteral("打开 H5 结果文件"), initialDirectory,
        QStringLiteral("HDF5 结果文件 (*.h5 *.hdf5);;所有文件 (*.*)"), QStringLiteral("hdf5ResultImport"));
    if (selectedFiles.isEmpty())
        return;

    int loadedCount = 0;
    QStringList failedFiles;
    for (const QString& filePath : selectedFiles)
    {
        QString failureReason;
        if (loadHdf5Result(filePath, false, true, false, &failureReason))
            ++loadedCount;
        else
        {
            const QString description = failureReason.isEmpty() ? QStringLiteral("未知读取错误") : failureReason;
            failedFiles.append(QStringLiteral("%1：%2").arg(QFileInfo(filePath).fileName(), description));
            appendLogMessage(QStringLiteral("[WARN] H5结果读取失败：%1；%2").arg(filePath, description));
        }
    }

    if (!failedFiles.isEmpty())
    {
        QMessageBox::warning(
            this, QStringLiteral("部分 H5 文件未能打开"),
            QStringLiteral("以下文件没有有效的模型或结果数据：\n%1").arg(failedFiles.join(QLatin1Char('\n'))));
    }
    if (loadedCount > 0)
        setWorkspaceMessage(QStringLiteral("已打开 %1 个 H5 结果文件").arg(loadedCount));
}

bool YQY::resultTaskBelongsToActiveModel(const QString& sourceFile) const
{
    if (!m_modelController || sourceFile.trimmed().isEmpty())
        return false;

    const QString sourcePath = QFileInfo(sourceFile).absoluteFilePath();
    const QString activeModelPath = QFileInfo(m_modelController->filePath()).absoluteFilePath();
    if (!activeModelPath.isEmpty() && sourcePath.compare(activeModelPath, Qt::CaseInsensitive) == 0)
    {
        return true;
    }

    if (m_resultSourceModelName.isEmpty())
        return false;
    return QFileInfo(sourceFile)
               .completeBaseName()
               .compare(QFileInfo(m_resultSourceModelName).completeBaseName(), Qt::CaseInsensitive) == 0;
}

void YQY::refreshResultCases(int preferredTaskId)
{
    if (!m_solveTaskController || !m_resultCaseCombo)
        return;

    auto* combo = m_resultCaseCombo;
    int selectedTaskId = preferredTaskId > 0 ? preferredTaskId : m_resultTaskId;
    if (selectedTaskId <= 0 && combo->currentIndex() >= 0)
        selectedTaskId = combo->currentData().toInt();

    const QSignalBlocker blocker(combo);
    combo->clear();
    for (const int taskId : m_solveTaskController->analysisTaskIds())
    {
        const auto info = m_solveTaskController->taskInfo(taskId);
        if (!info.hasUsableResult || !QFileInfo::exists(info.outputFile) ||
            !resultTaskBelongsToActiveModel(info.sourceFile))
        {
            continue;
        }

        const QString stepText = info.analysisStepId > 0 ? QStringLiteral("分析步 %1").arg(info.analysisStepId)
                                                         : QStringLiteral("全部分析步");
        combo->addItem(QStringLiteral("%1 · %2").arg(stepText, QFileInfo(info.outputFile).completeBaseName()), taskId);
        combo->setItemData(combo->count() - 1, combo->itemText(combo->count() - 1), Qt::ToolTipRole);
    }

    if (combo->count() == 0)
    {
        combo->setEnabled(false);
        combo->setToolTip(QStringLiteral("切换当前模型下不同分析步的计算结果"));
        return;
    }

    int selectedIndex = combo->findData(selectedTaskId);
    if (selectedIndex < 0 && m_resultPanel && !m_resultPanel->resultFilePath().isEmpty())
    {
        for (int index = 0; index < combo->count(); ++index)
        {
            const auto info = m_solveTaskController->taskInfo(combo->itemData(index).toInt());
            if (QFileInfo(info.outputFile)
                    .absoluteFilePath()
                    .compare(m_resultPanel->resultFilePath(), Qt::CaseInsensitive) == 0)
            {
                selectedIndex = index;
                break;
            }
        }
    }
    combo->setCurrentIndex(selectedIndex >= 0 ? selectedIndex : combo->count() - 1);
    combo->setToolTip(combo->currentText());
    combo->setEnabled(true);
}

void YQY::loadSolveTaskResult(int taskId)
{
    if (taskId <= 0 || !m_solveTaskController)
        return;

    const auto info = m_solveTaskController->taskInfo(taskId);
    if (!info.hasUsableResult || !QFileInfo::exists(info.outputFile))
        return;

    const QString resultFile = QFileInfo(info.outputFile).absoluteFilePath();
    if (resultFile.compare(m_resultPanel->resultFilePath(), Qt::CaseInsensitive) == 0)
        return;

    loadHdf5Result(resultFile, false, false, info.partialResult);
}

void YQY::refreshResultSteps(int preferredStepId)
{
    auto* combo = m_resultPanel->stepCombo();
    const QSignalBlocker blocker(combo);
    combo->clear();

    std::vector<int> stepIds;
    for (const auto& frame : m_allResultFrames)
    {
        if (std::find(stepIds.cbegin(), stepIds.cend(), frame.stepId) == stepIds.cend())
            stepIds.push_back(frame.stepId);
    }
    if (stepIds.empty())
    {
        combo->setEnabled(false);
        return;
    }

    if (stepIds.size() > 1)
        combo->addItem(QStringLiteral("全部分析步"), 0);
    const auto structure = m_modelController->model();
    for (const int stepId : stepIds)
    {
        QString label = QStringLiteral("分析步 %1").arg(stepId);
        if (structure)
        {
            const auto stepIt = structure->m_AnalysisStep.find(stepId);
            if (stepIt != structure->m_AnalysisStep.cend() && stepIt->second &&
                !stepIt->second->m_Name.trimmed().isEmpty())
            {
                label += QStringLiteral(" · %1").arg(stepIt->second->m_Name.trimmed());
            }
        }
        combo->addItem(label, stepId);
    }

    int selectedIndex = combo->findData(preferredStepId);
    if (selectedIndex < 0)
        selectedIndex = combo->count() - 1;
    combo->setCurrentIndex(selectedIndex);
    combo->setEnabled(combo->count() > 1);
}

void YQY::applyResultStepFilter(int stepId)
{
    if (!m_resultPanel)
        return;

    m_resultStepId = stepId;
    auto& frames = m_resultPanel->frames();
    frames.clear();
    m_resultFrameSourceIndices.clear();
    clearResultFrameCache();
    for (int index = 0; index < static_cast<int>(m_allResultFrames.size()); ++index)
    {
        const auto& frame = m_allResultFrames[static_cast<std::size_t>(index)];
        if (stepId > 0 && frame.stepId != stepId)
            continue;
        frames.push_back(frame);
        m_resultFrameSourceIndices.push_back(index);
    }

    m_resultPanel->stopPlayback();
    auto* slider = m_resultPanel->frameSlider();
    const QSignalBlocker blocker(slider);
    slider->setRange(0, std::max(0, static_cast<int>(frames.size()) - 1));
    slider->setValue(0);
    slider->setEnabled(!frames.empty());
    m_resultPanel->playButton()->setEnabled(frames.size() > 1);
    if (!frames.empty())
        displayResultFrame(0);
    if (m_nodeTimeHistoryPanel)
        m_nodeTimeHistoryPanel->setAnalysisStep(stepId);
}

bool YQY::readResultFrame(int frameIndex, _OUT Hdf5ResultFrame& frame)
{
    const Hdf5ResultFrame* cached = cachedResultFrame(frameIndex);
    if (!cached)
        return false;
    frame = *cached;
    return true;
}

const Hdf5ResultFrame* YQY::cachedResultFrame(int frameIndex)
{
    if (frameIndex < 0 || frameIndex >= static_cast<int>(m_resultFrameSourceIndices.size()))
        return nullptr;
    const int sourceIndex = m_resultFrameSourceIndices[static_cast<std::size_t>(frameIndex)];
    for (const ResultFrameCacheEntry& entry : m_resultFrameCache)
    {
        if (entry.sourceIndex == sourceIndex)
            return &entry.frame;
    }
    ResultFrameCacheEntry& entry = m_resultFrameCache[static_cast<std::size_t>(m_nextResultFrameCacheSlot)];
    entry.sourceIndex = -1;
    entry.frame = {};
    if (!m_resultDataController->readFrame(sourceIndex, entry.frame))
        return nullptr;
    entry.sourceIndex = sourceIndex;
    m_nextResultFrameCacheSlot = (m_nextResultFrameCacheSlot + 1) % static_cast<int>(m_resultFrameCache.size());
    return &entry.frame;
}

void YQY::clearResultFrameCache()
{
    for (ResultFrameCacheEntry& entry : m_resultFrameCache)
    {
        entry.sourceIndex = -1;
        entry.frame = {};
    }
    m_nextResultFrameCacheSlot = 0;
}

void YQY::exportNodeResults()
{
    auto* m_resultPlayButton = m_resultPanel->playButton();
    auto* m_resultFrameSlider = m_resultPanel->frameSlider();
    auto* m_exportNodeResultsButton = m_resultPanel->exportButton();
    auto* m_exportElementResultsButton = m_resultPanel->exportElementButton();
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
    for (const auto& [nodeId, node] : structure->Nodes())
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
    m_exportElementResultsButton->setEnabled(false);
    m_exportNodeResultsButton->setText(QStringLiteral("正在导出…"));
    setWorkspaceMessage(QStringLiteral("正在导出 %1 个节点的时程数据…").arg(nodeIds.size()));

    const QString sourceFile = m_resultFilePath;
    auto* watcher = new QFutureWatcher<bool>(this);
    connect(watcher, &QFutureWatcher<bool>::finished, this,
            [this, watcher, outputFile, m_resultPlayButton, m_resultFrameSlider, m_exportNodeResultsButton,
             m_exportElementResultsButton, nodeCount = nodeIds.size(), fieldCount = resultTypes.size()]()
            {
                const bool succeeded = watcher->result();
                watcher->deleteLater();
                m_exportNodeResultsButton->setText(QStringLiteral("导出节点数据…"));
                const auto& frames = m_resultPanel->frames();
                m_exportNodeResultsButton->setEnabled(!m_resultPanel->resultFilePath().isEmpty() && !frames.empty());
                m_exportElementResultsButton->setEnabled(!m_resultPanel->resultFilePath().isEmpty() && !frames.empty());
                m_resultFrameSlider->setEnabled(!frames.empty());
                m_resultPlayButton->setEnabled(frames.size() > 1);

                if (succeeded)
                {
                    appendLogMessage(QStringLiteral("[INFO] 节点结果已导出：%1 个节点，%2 个分量 → %3")
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
            ResultExporter exporter;
            return exporter.exportBdf(sourceFile, outputFile, nodeIds, resultTypes, {}, {});
        }));
}

void YQY::exportElementResults()
{
    auto* playButton = m_resultPanel->playButton();
    auto* frameSlider = m_resultPanel->frameSlider();
    auto* exportButton = m_resultPanel->exportElementButton();
    const auto& frames = m_resultPanel->frames();
    const QString& resultFilePath = m_resultPanel->resultFilePath();
    const auto structure = m_modelController->model();
    if (!structure || resultFilePath.isEmpty() || frames.empty())
    {
        QMessageBox::information(this, QStringLiteral("尚无可导出结果"),
                                 QStringLiteral("请先加载模型及其 H5 结果文件。"));
        return;
    }

    QSet<int> availableElements;
    for (const auto& [elementId, element] : structure->Elements())
        if (element)
            availableElements.insert(elementId);
    ElementResultExportDialog dialog(resultFilePath, availableElements, this);
    if (dialog.exec() != QDialog::Accepted)
        return;

    const std::vector<int> elementIds = dialog.elementIds();
    const std::vector<EnumKeyword::ElementResultType> resultTypes = dialog.resultTypes();
    QString outputFile = dialog.outputFile();
    if (QFileInfo(outputFile).suffix().isEmpty())
        outputFile += QStringLiteral(".bdf");
    if (QFileInfo::exists(outputFile) &&
        QMessageBox::question(this, QStringLiteral("替换已有文件"),
                              QStringLiteral("文件已经存在，是否替换？\n%1").arg(outputFile),
                              QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
        return;

    m_resultPanel->stopPlayback();
    playButton->setEnabled(false);
    frameSlider->setEnabled(false);
    exportButton->setEnabled(false);
    m_resultPanel->exportButton()->setEnabled(false);
    exportButton->setText(QStringLiteral("正在导出…"));
    const QString sourceFile = resultFilePath;
    auto* watcher = new QFutureWatcher<bool>(this);
    connect(watcher, &QFutureWatcher<bool>::finished, this,
            [this, watcher, outputFile, playButton, frameSlider, exportButton, elementCount = elementIds.size(),
             fieldCount = resultTypes.size()]()
            {
                const bool succeeded = watcher->result();
                watcher->deleteLater();
                exportButton->setText(QStringLiteral("导出单元数据…"));
                const auto& currentFrames = m_resultPanel->frames();
                const bool enabled = !m_resultPanel->resultFilePath().isEmpty() && !currentFrames.empty();
                exportButton->setEnabled(enabled);
                m_resultPanel->exportButton()->setEnabled(enabled);
                frameSlider->setEnabled(!currentFrames.empty());
                playButton->setEnabled(currentFrames.size() > 1);
                if (succeeded)
                {
                    appendLogMessage(QStringLiteral("[INFO] 单元结果已导出：%1 个单元，%2 个分量 → %3")
                                         .arg(elementCount)
                                         .arg(fieldCount)
                                         .arg(outputFile));
                    setWorkspaceMessage(QStringLiteral("单元结果导出完成 · %1").arg(QFileInfo(outputFile).fileName()));
                }
                else
                {
                    setWorkspaceMessage(QStringLiteral("单元结果导出失败"));
                    QMessageBox::critical(this, QStringLiteral("导出失败"),
                                          QStringLiteral("无法导出单元结果，请检查结果文件和保存路径。"));
                }
            });
    watcher->setFuture(QtConcurrent::run(
        [sourceFile, outputFile, elementIds, resultTypes]()
        {
            ResultExporter exporter;
            return exporter.exportBdf(sourceFile, outputFile, {}, {}, elementIds, resultTypes);
        }));
}

void YQY::exportIterationHistory()
{
    const QString sourceFile = m_resultPanel->resultFilePath();
    if (sourceFile.isEmpty())
    {
        QMessageBox::information(this, QStringLiteral("暂无迭代步"), QStringLiteral("请先加载计算结果文件。"));
        return;
    }

    Hdf5ResultReader reader;
    std::vector<Hdf5ResultFrameInfo> frames;
    std::vector<SolverIterationRecord> records;
    const bool readOk = reader.open(sourceFile, frames) && reader.readIterationHistory(records);
    reader.close();
    if (!readOk || records.empty())
    {
        QMessageBox::information(this, QStringLiteral("暂无迭代步"),
                                 QStringLiteral("当前 H5 结果文件中没有迭代步数据。"));
        return;
    }

    const QString defaultOutputFile =
        ResultExportPaths::filePath(sourceFile, QStringLiteral("迭代步"), QStringLiteral("bdf"));
    if (defaultOutputFile.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("导出失败"), QStringLiteral("无法创建结果导出目录。"));
        return;
    }
    QString outputFile = FileDialogService::selectSaveFile(this, QStringLiteral("保存迭代步"), defaultOutputFile,
                                                           QStringLiteral("BDF 文件 (*.bdf);;所有文件 (*.*)"),
                                                           QStringLiteral("iterationResultExport"), false);
    if (outputFile.isEmpty())
        return;
    if (QFileInfo(outputFile).suffix().isEmpty())
        outputFile += QStringLiteral(".bdf");
    if (QFileInfo::exists(outputFile) &&
        QMessageBox::question(this, QStringLiteral("替换已有文件"),
                              QStringLiteral("文件已经存在，是否替换？\n%1").arg(outputFile),
                              QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
    {
        return;
    }

    Outputter exporter;
    exporter.SetSolverIterationRecords(std::move(records));
    if (exporter.ExportSolverIterationHistory(outputFile))
    {
        appendLogMessage(QStringLiteral("[INFO] 迭代步已导出：%1").arg(outputFile));
        setWorkspaceMessage(QStringLiteral("迭代步导出完成 · %1").arg(QFileInfo(outputFile).fileName()));
    }
    else
    {
        QMessageBox::critical(this, QStringLiteral("导出失败"), QStringLiteral("无法导出迭代步，请检查保存路径。"));
    }
}

void YQY::exportResultPlot()
{
    const QString sourceFile = m_resultPanel->resultFilePath();
    const auto& frames = m_resultPanel->frames();
    if (sourceFile.isEmpty() || frames.empty())
    {
        QMessageBox::information(this, QStringLiteral("尚无可绘图结果"),
                                 QStringLiteral("请先加载包含结果帧的 H5 结果文件。"));
        return;
    }
    QSet<int> availableNodes;
    if (const auto structure = m_modelController->model())
    {
        for (const auto& [nodeId, node] : structure->Nodes())
        {
            if (node)
                availableNodes.insert(nodeId);
        }
    }
    ResultPlotExportDialog dialog(sourceFile, availableNodes, m_selectedNodeId, this);
    if (dialog.exec() != QDialog::Accepted)
        return;
    const ResultPlotSelection selection = dialog.selection();
    QString outputFile = selection.outputFile;
    if (QFileInfo(outputFile).suffix().isEmpty())
        outputFile += QStringLiteral(".bdf");
    if (QFileInfo::exists(outputFile) &&
        QMessageBox::question(this, QStringLiteral("替换已有文件"),
                              QStringLiteral("文件已经存在，是否替换？\n%1").arg(outputFile),
                              QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
        return;

    auto* plotButton = m_resultPanel->exportPlotButton();
    plotButton->setEnabled(false);
    plotButton->setText(QStringLiteral("正在导出 BDF…"));
    setWorkspaceMessage(QStringLiteral("正在导出节点 %1 的时程数据…").arg(selection.nodeId));

    const int nodeId = selection.nodeId;
    auto* watcher = new QFutureWatcher<bool>(this);
    connect(
        watcher, &QFutureWatcher<bool>::finished, this,
        [this, watcher, plotButton, outputFile, nodeId]()
        {
            const bool succeeded = watcher->result();
            watcher->deleteLater();
            if (!succeeded)
            {
                plotButton->setEnabled(true);
                plotButton->setText(QStringLiteral("导出数据并生成曲线"));
                setWorkspaceMessage(QStringLiteral("时程数据导出失败"));
                QMessageBox::critical(this, QStringLiteral("导出失败"),
                                      QStringLiteral("无法直接读取该节点的 H5 位移时程，或无法写入 BDF 文件。\n"
                                                     "请检查结果文件的节点记录布局和保存路径。"));
                return;
            }

            QString scriptFile;
            const QStringList candidates = {
                QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("Tools/result_plot.py")),
                QDir::current().absoluteFilePath(QStringLiteral("YQY/Tools/result_plot.py")),
                QDir(QCoreApplication::applicationDirPath())
                    .absoluteFilePath(QStringLiteral("../../YQY/Tools/result_plot.py"))};
            for (const QString& candidate : candidates)
            {
                if (QFileInfo(candidate).isFile())
                {
                    scriptFile = QFileInfo(candidate).absoluteFilePath();
                    break;
                }
            }
            if (scriptFile.isEmpty())
            {
                plotButton->setEnabled(true);
                plotButton->setText(QStringLiteral("导出数据并生成曲线"));
                setWorkspaceMessage(QStringLiteral("BDF 已导出，但未找到 Python 绘图脚本"));
                QMessageBox::warning(this, QStringLiteral("BDF 已导出"),
                                     QStringLiteral("BDF 已保存，但未找到 YQY/Tools/result_plot.py。"));
                return;
            }

            const QString imageFile = QFileInfo(outputFile).absolutePath() + QDir::separator() +
                                      QFileInfo(outputFile).completeBaseName() + QStringLiteral(".png");
            auto* process = new QProcess(this);
            process->setProcessChannelMode(QProcess::MergedChannels);
            const QStringList arguments = {scriptFile, QStringLiteral("--input"), outputFile,
                                           QStringLiteral("--output"), imageFile};
            plotButton->setText(QStringLiteral("正在生成曲线…"));
            setWorkspaceMessage(QStringLiteral("正在调用 Python 生成结果图…"));
            connect(process, &QProcess::errorOccurred, this,
                    [this, process, plotButton](QProcess::ProcessError error)
                    {
                        if (error != QProcess::FailedToStart)
                            return;
                        plotButton->setEnabled(true);
                        plotButton->setText(QStringLiteral("导出数据并生成曲线"));
                        setWorkspaceMessage(QStringLiteral("BDF 已导出，但 Python 未能启动"));
                        QMessageBox::warning(
                            this, QStringLiteral("绘图未完成"),
                            QStringLiteral("BDF 已导出，但无法启动 Python。请确认 Python 已安装并已加入 PATH。"));
                        process->deleteLater();
                    });
            connect(
                process, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
                [this, process, plotButton, outputFile, imageFile, nodeId](int exitCode,
                                                                           QProcess::ExitStatus exitStatus)
                {
                    const bool plotted =
                        exitStatus == QProcess::NormalExit && exitCode == 0 && QFileInfo(imageFile).isFile();
                    plotButton->setEnabled(true);
                    plotButton->setText(QStringLiteral("导出数据并生成曲线"));
                    if (plotted)
                    {
                        appendLogMessage(QStringLiteral("[INFO] 节点 %1 曲线已生成：%2").arg(nodeId).arg(imageFile));
                        setWorkspaceMessage(QStringLiteral("结果图生成完成 · %1").arg(QFileInfo(imageFile).fileName()));
                        QDesktopServices::openUrl(QUrl::fromLocalFile(imageFile));
                    }
                    else
                    {
                        setWorkspaceMessage(QStringLiteral("BDF 已导出，但 Python 绘图失败"));
                        const QString details = QString::fromLocal8Bit(process->readAllStandardOutput());
                        QMessageBox::warning(
                            this, QStringLiteral("绘图未完成"),
                            QStringLiteral("BDF 已导出，但 Python 绘图失败。\n%1").arg(details.trimmed()));
                    }
                    process->deleteLater();
                });
            process->start(QStringLiteral("python"), arguments);
        });
    watcher->setFuture(QtConcurrent::run(
        [sourceFile, outputFile, selection, nodeId]()
        {
            Hdf5ResultReader reader;
            std::vector<Hdf5ResultFrameInfo> loadedFrames;
            if (!reader.open(sourceFile, loadedFrames))
                return false;

            std::vector<Hdf5NodalDisplacementHistorySample> samples;
            const bool historyRead = reader.readNodalDisplacementHistory(nodeId, samples);
            reader.close();
            if (!historyRead || samples.empty())
                return false;

            return ResultExporter().exportNodalHistoryBdf(outputFile, nodeId, samples,
                                                          {selection.displacementX, selection.displacementY,
                                                           selection.displacementZ, selection.rotationX,
                                                           selection.rotationY, selection.rotationZ});
        }));
}

void YQY::showTimeHistory()
{
    if (!m_nodeTimeHistoryPanel || !m_resultPanel || m_resultPanel->resultFilePath().isEmpty() ||
        m_resultPanel->frames().empty())
    {
        QMessageBox::information(this, QStringLiteral("尚无可绘制结果"),
                                 QStringLiteral("请先加载包含结果帧的 H5 结果文件。"));
        return;
    }
    m_resultPanel->stopPlayback();
    m_nodeTimeHistoryPanel->showTimeHistory(m_selectedNodeId);
    m_workspaceDisplayStack->setCurrentWidget(m_nodeTimeHistoryPanel);
    setWorkspaceMessage(QStringLiteral("时程曲线：可切换节点位移与能量，选择分量后绘制"));
}

bool YQY::loadHdf5Result(const QString& filePath, bool showErrors, bool activateResultModule, bool partialResult,
                         QString* failureReason, bool reportToLog)
{
    if (failureReason)
        failureReason->clear();
    const auto fail = [failureReason](const QString& reason)
    {
        if (failureReason)
            *failureReason = reason;
        return false;
    };
    auto* m_resultPlayButton = m_resultPanel->playButton();
    auto* m_resultFrameSlider = m_resultPanel->frameSlider();
    auto* m_exportNodeResultsButton = m_resultPanel->exportButton();
    auto* m_exportElementResultsButton = m_resultPanel->exportElementButton();
    auto* m_exportIterationButton = m_resultPanel->exportIterationButton();
    auto* m_exportPlotButton = m_resultPanel->exportPlotButton();
    auto& m_resultFrames = m_resultPanel->frames();
    auto& m_resultFilePath = m_resultPanel->resultFilePath();
    auto& m_resultIsPartial = m_resultPanel->partialResult();
    auto& m_resultAutomaticScale = m_resultPanel->automaticScale();
    Hdf5ResultSummary summary;
    if (!m_resultDataController->inspect(filePath, summary) || !summary.hasResult)
    {
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 读取失败"),
                                  QStringLiteral("文件中没有有效的 YQY 计算结果：\n%1").arg(filePath));
        return fail(QStringLiteral("文件不含有效的YQY结果数据或结果结构不完整"));
    }
    partialResult = partialResult || summary.partialResult;
    if (!summary.hasModel)
    {
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 模型缺失"),
                                  QStringLiteral("该 H5 文件不包含模型数据，无法可靠显示对应结果：\n%1").arg(filePath));
        return fail(QStringLiteral("结果文件缺少嵌入模型"));
    }

    QElapsedTimer modelTimer;
    modelTimer.start();
    auto embeddedModel = std::make_shared<StructureData>();
    Hdf5ModelSerializer modelReader;
    if (!modelReader.importModel(filePath, embeddedModel.get()))
    {
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 模型读取失败"),
                                  QStringLiteral("无法从 H5 文件恢复计算时使用的模型：\n%1").arg(filePath));
        return fail(QStringLiteral("嵌入模型读取失败"));
    }

    const auto currentModel = m_modelController->model();
    const bool sameModelAsCurrent = ResultFrameUtilities::modelsMatch(currentModel, embeddedModel);
    const bool shouldActivateEmbeddedModel = activateResultModule || !sameModelAsCurrent;
    if (shouldActivateEmbeddedModel)
    {
        // 采用结果内嵌模型只更新显示文档，同步加载回调不能再为原分析步创建待运行任务。
        int adoptedModelId = -1;
        {
            const QScopedValueRollback<bool> resultModelGuard(m_loadingResultModel, true);
            adoptedModelId = m_modelController->adoptModel(embeddedModel, filePath, modelTimer.elapsed());
        }
        if (adoptedModelId < 0)
        {
            if (showErrors)
                QMessageBox::critical(this, QStringLiteral("H5 模型激活失败"),
                                      QStringLiteral("H5 模型已经读取，但无法切换为当前显示模型。"));
            return fail(QStringLiteral("嵌入模型无法切换为当前显示模型"));
        }
    }

    // adoptModel() 同步发出 activeModelChanged；该回调会清理旧的结果上下文。
    // 所以必须在模型切换完成后才打开本 H5 的结果流，否则首帧显示时读取器已经被关闭。
    std::vector<Hdf5ResultFrameInfo> indexedFrames;
    if (!m_resultDataController->open(filePath, indexedFrames, m_resultRanges))
    {
        if (showErrors)
            QMessageBox::critical(this, QStringLiteral("H5 索引失败"),
                                  QStringLiteral("无法建立结果帧索引：\n%1").arg(filePath));
        return fail(QStringLiteral("结果帧索引无法建立"));
    }

    m_resultFilePath = QFileInfo(filePath).absoluteFilePath();
    m_workspaceController->associateResult(m_modelController->activeModelId(), m_resultFilePath);
    m_resultSourceModelName = embeddedModel->m_OutputControl.m_SourceModelName.trimmed();
    if (m_resultSourceModelName.isEmpty())
        m_resultSourceModelName = m_modelController->filePath();
    ResultExportPaths::setSourceModel(m_resultFilePath, m_resultSourceModelName);
    m_resultTaskId = -1;
    for (const int taskId : m_solveTaskController->analysisTaskIds())
    {
        const auto taskInfo = m_solveTaskController->taskInfo(taskId);
        if (QFileInfo(taskInfo.outputFile).absoluteFilePath().compare(m_resultFilePath, Qt::CaseInsensitive) == 0)
        {
            m_resultTaskId = taskId;
            break;
        }
    }
    m_resultIsPartial = partialResult;
    m_allResultFrames = std::move(indexedFrames);
    const int preferredStepId = m_allResultFrames.empty() ? 0 : m_allResultFrames.back().stepId;
    if (m_nodeTimeHistoryPanel)
        m_nodeTimeHistoryPanel->setResultContext(m_resultFilePath, m_allResultFrames, preferredStepId,
                                                 summary.energyRecordCount > 0);
    refreshResultSteps(preferredStepId);
    applyResultStepFilter(preferredStepId);
    refreshResultCases(m_resultTaskId);
    if (m_resultRanges.displacementMagnitude.valid)
    {
        m_resultAutomaticScale =
            m_modelViewport->automaticDeformationScale(std::max(0.0, m_resultRanges.displacementMagnitude.maximum));
    }
    else
    {
        Hdf5ResultFrame scaleReferenceFrame;
        if (readResultFrame(static_cast<int>(m_resultFrames.size()) - 1, scaleReferenceFrame) &&
            m_modelViewport->displayResultFrame(scaleReferenceFrame, ModelViewport::ResultField::DisplacementMagnitude,
                                                0.0, false))
        {
            m_resultAutomaticScale = m_modelViewport->activeDeformationScale();
        }
        else
        {
            m_resultAutomaticScale = 1.0;
        }
    }
    m_resultPanel->stopPlayback();
    m_resultFrameSlider->setEnabled(true);
    m_resultFrameSlider->setRange(0, static_cast<int>(m_resultFrames.size()) - 1);
    m_resultPlayButton->setEnabled(m_resultFrames.size() > 1);
    m_exportNodeResultsButton->setEnabled(true);
    m_exportElementResultsButton->setEnabled(true);
    m_exportIterationButton->setEnabled(true);
    m_exportPlotButton->setEnabled(true);
    m_resultPanel->exportEnergyButton()->setToolTip(
        summary.energyRecordCount > 0 ? QStringLiteral("将当前分析步的能量时程导出为 BDF 数据文件；能量视图中按勾选分量导出")
                                      : QStringLiteral("当前结果未包含能量输出，请重新计算后查看。"));

    m_resultFileLabel->setText(partialResult ? QStringLiteral("%1 · 部分结果").arg(QFileInfo(filePath).fileName())
                                             : QFileInfo(filePath).fileName());
    m_resultTree->clear();
    auto* root = new QTreeWidgetItem(
        m_resultTree, {partialResult ? QStringLiteral("部分结果（计算未完成）") : QStringLiteral("结果文件")});
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
    auto* energy = new QTreeWidgetItem(
        root, {summary.energyRecordCount > 0 ? QStringLiteral("能量记录  %1").arg(summary.energyRecordCount)
                                            : QStringLiteral("能量记录  无")});
    setTreeGlyph(energy, TreeGlyph::ResultFile);
    if (summary.energyRecordCount > 0)
    {
        QStringList components;
        if (summary.availableEnergyMask & EnergyResult::Kinetic)
            components.append(QStringLiteral("动能"));
        if (summary.availableEnergyMask & EnergyResult::Strain)
            components.append(QStringLiteral("应变能"));
        if (summary.availableEnergyMask & EnergyResult::Gravity)
            components.append(QStringLiteral("重力势能"));
        if (summary.availableEnergyMask & EnergyResult::Mechanical)
            components.append(QStringLiteral("总机械能"));
        energy->setToolTip(0, components.isEmpty()
                                 ? QStringLiteral("已有能量记录，当前模型未提供有效能量分量。")
                                 : QStringLiteral("文件内可用分量：%1（J）；各分析步以实际记录为准。")
                                       .arg(components.join(QStringLiteral("、"))));
    }
    else
    {
        energy->setToolTip(0, QStringLiteral("当前结果未包含能量输出，请重新计算后查看。"));
    }
    const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
    applyTreeIcons(m_resultTree, QColor(colors.text), QColor(colors.accentSecond));
    m_resultTree->expandAll();
    m_resultFileLabel->show();
    m_resultTree->show();
    m_resultTree->updateGeometry();
    if (reportToLog)
    {
        appendLogMessage(QStringLiteral("[%1] 已加载%2 H5 后处理结果：%3，%4 帧，截止 t=%5 s")
                             .arg(partialResult ? QStringLiteral("WARN") : QStringLiteral("INFO"),
                                  partialResult ? QStringLiteral("部分") : QString(), QFileInfo(filePath).fileName())
                             .arg(summary.frameCount)
                             .arg(resultEndTime, 0, 'g', 10));
        appendLogMessage(QStringLiteral("[路径] 结果文件：%1").arg(filePath), true);
    }
    if (shouldActivateEmbeddedModel && reportToLog)
    {
        appendLogMessage(QStringLiteral("[INFO] 已切换到 H5 内嵌模型：节点 %1，单元 %2")
                             .arg(embeddedModel->Nodes().size())
                             .arg(embeddedModel->Elements().size()));
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
    const Hdf5ResultFrame* currentFrame = cachedResultFrame(frameIndex);
    const Hdf5ResultFrame* nextFrame = cachedResultFrame(nextFrameIndex);
    if (!currentFrame || !nextFrame)
    {
        m_resultFrameLabel->setText(QStringLiteral("第 %1 帧读取失败").arg(frameIndex + 1));
        m_resultTimeLabel->setText(QStringLiteral("--"));
        m_resultDeformationLabel->setText(QStringLiteral("--"));
        return;
    }
    Hdf5ResultFrame interpolatedFrame;
    const Hdf5ResultFrame* frame = currentFrame;
    if (interpolation > 1.0e-9)
    {
        interpolatedFrame = ResultFrameUtilities::interpolate(*currentFrame, *nextFrame, interpolation);
        frame = &interpolatedFrame;
    }
    constexpr std::array resultFields = {ModelViewport::ResultField::DisplacementMagnitude,
                                         ModelViewport::ResultField::AxialForce, ModelViewport::ResultField::Stress,
                                         ModelViewport::ResultField::Strain};
    const int fieldIndex = m_resultFieldCombo->currentIndex();
    if (fieldIndex < 0 || fieldIndex >= static_cast<int>(resultFields.size()))
        return;
    const ModelViewport::ResultField field = resultFields[static_cast<std::size_t>(fieldIndex)];
    const Hdf5ResultRange& scalarRange = resultRangeForField(m_resultRanges, field);
    if (scalarRange.valid)
        m_modelViewport->setResultScalarRange(scalarRange.minimum, scalarRange.maximum);
    else
        m_modelViewport->clearResultScalarRange();
    const double requestedScale =
        m_resultScaleSpin->value() <= 0.0 ? m_resultAutomaticScale : m_resultScaleSpin->value();
    const bool refreshSolidGeometry = !m_resultPanel->playbackTimer()->isActive();
    if (!m_modelViewport->displayResultFrame(*frame, field, requestedScale, m_resultOriginalCheck->isChecked(),
                                             refreshSolidGeometry))
        return;
    const QString scale = m_resultScaleSpin->value() <= 0.0
                              ? QStringLiteral("自动 %1×").arg(m_resultAutomaticScale, 0, 'g', 4)
                              : QStringLiteral("%1×").arg(m_resultScaleSpin->value(), 0, 'g', 6);
    m_resultFrameLabel->setText(QStringLiteral("%1 / %2%3")
                                    .arg(frameIndex + 1)
                                    .arg(m_resultFrames.size())
                                    .arg(m_resultIsPartial ? QStringLiteral("  ·  部分结果") : QString()));
    m_resultTimeLabel->setText(QStringLiteral("%1 s").arg(frame->info.time, 0, 'g', 8));
    m_resultDeformationLabel->setText(scale);
}

void YQY::updateResultVisualization()
{
    if (m_resultPanel && !m_resultPanel->frames().empty())
        displayResultFrame(m_resultPanel->frameSlider()->value());
}

void YQY::clearActiveResultContext()
{
    if (!m_resultPanel)
        return;

    // 结果读取器及播放定时器只属于一个模型；切换文档后继续保留会把旧节点编号和标量应用到新几何体。
    m_resultPanel->stopPlayback();
    m_resultDataController->close();
    clearResultFrameCache();
    m_resultPanel->frames().clear();
    m_allResultFrames.clear();
    m_resultFrameSourceIndices.clear();
    m_resultStepId = 0;
    m_resultPanel->resultFilePath().clear();
    if (m_nodeTimeHistoryPanel)
        m_nodeTimeHistoryPanel->clearHistory();
    m_resultTaskId = -1;
    m_resultSourceModelName.clear();
    m_resultPanel->partialResult() = false;
    m_resultPanel->automaticScale() = 1.0;
    m_resultRanges = {};

    {
        const QSignalBlocker blocker(m_resultPanel->frameSlider());
        m_resultPanel->frameSlider()->setRange(0, 0);
        m_resultPanel->frameSlider()->setValue(0);
    }
    m_resultPanel->frameSlider()->setEnabled(false);
    m_resultPanel->playButton()->setEnabled(false);
    m_resultPanel->exportButton()->setEnabled(false);
    m_resultPanel->exportElementButton()->setEnabled(false);
    m_resultPanel->exportIterationButton()->setEnabled(false);
    m_resultPanel->exportPlotButton()->setEnabled(false);
    m_resultPanel->exportEnergyButton()->setEnabled(false);
    m_resultPanel->exportEnergyButton()->setToolTip(QStringLiteral("请先加载包含能量输出的 H5 结果文件。"));
    {
        const QSignalBlocker blocker(m_resultCaseCombo);
        m_resultCaseCombo->clear();
    }
    m_resultCaseCombo->setEnabled(false);
    {
        const QSignalBlocker blocker(m_resultPanel->stepCombo());
        m_resultPanel->stepCombo()->clear();
    }
    m_resultPanel->stepCombo()->setEnabled(false);
    m_resultPanel->frameLabel()->setText(QStringLiteral("--"));
    m_resultPanel->timeValueLabel()->setText(QStringLiteral("--"));
    m_resultPanel->deformationValueLabel()->setText(QStringLiteral("--"));
    m_resultFileLabel->setText(QStringLiteral("--"));
    m_resultTree->clear();
    m_resultFileLabel->hide();
    m_resultTree->hide();
}

bool YQY::loadModelFile(const QString& filePath)
{
    if (!m_modelController->loadModel(filePath))
    {
        if (m_modelController->isLoading())
            setWorkspaceMessage(QStringLiteral("已有模型正在读取，请稍候"));
        return false;
    }
    // 已作为结果打开的同路径文档不会再次发出加载信号；显式导入为计算模型时补齐任务。
    const QFileInfo requestedFile(filePath);
    for (const int modelId : m_modelController->modelIds())
    {
        if (QFileInfo(m_modelController->filePath(modelId)) != requestedFile)
            continue;
        for (const int taskId : m_solveTaskController->analysisTaskIds())
            if (QFileInfo(m_solveTaskController->taskInfo(taskId).sourceFile) == requestedFile)
                return true;
        prepareAnalysisCases(m_modelController->model(modelId), m_modelController->filePath(modelId));
        break;
    }
    return true;
}

void YQY::handleModelLoaded(int modelId, const QString& filePath, qint64 elapsedMs)
{
    const auto structure = m_modelController->model(modelId);
    if (!structure)
        return;

    {
        const QSignalBlocker blocker(m_documentTabBar);
        const int tabIndex = m_documentTabBar->addTab(QFileInfo(filePath).fileName());
        m_documentTabBar->setTabData(tabIndex, modelId);
        m_documentTabBar->setTabToolTip(tabIndex, filePath);
    }

    ++m_modelLoadSucceeded;
    updateLoadStatistics();
    appendLogMessage(QStringLiteral("[模型] 已读取 %1：节点 %2，单元 %3，材料 %4，截面 %5 · 用时 %6")
                         .arg(QFileInfo(filePath).fileName())
                         .arg(structure->Nodes().size())
                         .arg(structure->Elements().size())
                         .arg(structure->m_Material.size())
                         .arg(structure->m_Section.size())
                         .arg(elapsedTimeText(elapsedMs)));

    // 正常导入 BDF/H5 模型均准备任务；为查看结果而采用内嵌模型时保留原计算任务。
    if (!m_loadingResultModel)
        prepareAnalysisCases(structure, filePath);

    setWorkspaceMessage(QStringLiteral("模型加载完成 · %1 个节点 · %2 个单元 · %3 ms")
                            .arg(structure->Nodes().size())
                            .arg(structure->Elements().size())
                            .arg(elapsedMs));
}

void YQY::handleActiveModelChanged(int modelId)
{
    closeAnalysisManagers();
    const auto structure = m_modelController->model(modelId);
    if (!structure)
        return;

    // 重建共享 VTK 视口前停止并分离旧模型后处理，随后恢复当前选中模型的结果文件。
    clearActiveResultContext();
    m_modelViewport->clearResultDisplay();

    {
        const QSignalBlocker blocker(m_documentTabBar);
        for (int index = 0; index < m_documentTabBar->count(); ++index)
        {
            if (m_documentTabBar->tabData(index).toInt() == modelId)
            {
                m_documentTabBar->setCurrentIndex(index);
                break;
            }
        }
    }

    m_selectedNodeId = -1;
    m_modelViewport->displayModel(structure);
    populateModelDataTree(QFileInfo(m_modelController->filePath(modelId)).fileName());
    setModelControlsEnabled(true);
    clearSelectionProperties();
    refreshModulePages();
    setWorkspaceMessage(QStringLiteral("当前模型：%1 · 共打开 %2 个模型")
                            .arg(QFileInfo(m_modelController->filePath(modelId)).fileName())
                            .arg(m_modelController->modelCount()));

    const QString resultFilePath = m_workspaceController->resultForModel(modelId);
    if (!resultFilePath.isEmpty())
    {
        QTimer::singleShot(0, this,
                           [this, modelId, resultFilePath]()
                           {
                               if (m_modelController->activeModelId() == modelId &&
                                   m_resultPanel->resultFilePath().isEmpty())
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
    appendLogMessage(QStringLiteral("[错误] 模型 %1 读取失败：%2").arg(QFileInfo(filePath).fileName(), errorMessage));
    setWorkspaceMessage(QStringLiteral("模型读取失败：%1").arg(QFileInfo(filePath).fileName()));
}

void YQY::populateModelDataTree(const QString& fileName)
{
    const auto structure = m_modelController->model();
    m_modelDataTree->clear();
    if (!structure)
    {
        auto* emptyModel = new QTreeWidgetItem(m_modelDataTree, {QStringLiteral("尚未加载模型")});
        setTreeGlyph(emptyModel, TreeGlyph::Model);
        const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
        applyTreeIcons(m_modelDataTree, QColor(colors.text), QColor(colors.accentSecond));
        return;
    }

    auto* model = new QTreeWidgetItem(m_modelDataTree, {fileName});
    setTreeGlyph(model, TreeGlyph::Model);
    auto* nodes = new QTreeWidgetItem(model, {QStringLiteral("节点  %1").arg(structure->Nodes().size())});
    setTreeGlyph(nodes, TreeGlyph::Nodes);
    auto* elements = new QTreeWidgetItem(model, {QStringLiteral("单元  %1").arg(structure->Elements().size())});
    setTreeGlyph(elements, TreeGlyph::Elements);
    const int modelId = m_modelController->activeModelId();
    QTreeWidgetItem* materials = nullptr;
    if (!structure->Materials().empty())
    {
        materials = new QTreeWidgetItem(model, {QStringLiteral("材料  %1").arg(structure->Materials().size())});
        setTreeGlyph(materials, TreeGlyph::Material);
        for (const auto& [materialId, material] : structure->Materials())
        {
            if (!material)
                continue;
            auto* item = new QTreeWidgetItem(materials, {QStringLiteral("材料 ID %1 · E=%2 · ρ=%3")
                                                             .arg(materialId)
                                                             .arg(material->m_Young, 0, 'g', 5)
                                                             .arg(material->m_Density, 0, 'g', 5)});
            item->setToolTip(0, QStringLiteral("双击查看或修改材料参数"));
            item->setData(0, Qt::UserRole, 2);
            item->setData(0, Qt::UserRole + 1, modelId);
            item->setData(0, Qt::UserRole + 2, materialId);
            setTreeGlyph(item, TreeGlyph::Material);
        }
    }
    QTreeWidgetItem* sections = nullptr;
    if (!structure->Sections().empty())
    {
        sections = new QTreeWidgetItem(model, {QStringLiteral("截面  %1").arg(structure->Sections().size())});
        setTreeGlyph(sections, TreeGlyph::Section);
        for (const auto& [sectionId, section] : structure->Sections())
        {
            if (!section)
                continue;
            const auto rectangle = std::dynamic_pointer_cast<SectionRectangle>(section);
            const auto circular = std::dynamic_pointer_cast<SectionCircular>(section);
            const QString type = rectangle  ? QStringLiteral("矩形")
                                 : circular ? QStringLiteral("圆形")
                                            : QStringLiteral("通用");
            auto* item = new QTreeWidgetItem(
                sections,
                {QStringLiteral("截面 ID %1 · %2 · A=%3").arg(sectionId).arg(type).arg(section->m_Area, 0, 'g', 6)});
            item->setToolTip(0, QStringLiteral("双击查看或修改截面参数"));
            item->setData(0, Qt::UserRole, 4);
            item->setData(0, Qt::UserRole + 1, modelId);
            item->setData(0, Qt::UserRole + 2, sectionId);
            setTreeGlyph(item, TreeGlyph::Section);
        }
    }
    if (!structure->m_SpringBehaviors.empty())
    {
        auto* behaviors =
            new QTreeWidgetItem(model, {QStringLiteral("弹簧行为  %1").arg(structure->m_SpringBehaviors.size())});
        setTreeGlyph(behaviors, TreeGlyph::SpringBehavior);
        for (const auto& [behaviorId, behavior] : structure->m_SpringBehaviors)
        {
            const int pointCount = behavior ? static_cast<int>(behavior->m_Points.size()) : 0;
            auto* item = new QTreeWidgetItem(
                behaviors, {QStringLiteral("行为 %1  ·  力-位移点 %2").arg(behaviorId).arg(pointCount)});
            item->setToolTip(0, QStringLiteral("双击查看力-位移数据"));
            item->setData(0, Qt::UserRole, 5);
            item->setData(0, Qt::UserRole + 1, modelId);
            item->setData(0, Qt::UserRole + 2, behaviorId);
            setTreeGlyph(item, TreeGlyph::SpringBehavior);
        }
    }

    int trussCount = 0;
    int cableCount = 0;
    int beamCount = 0;
    int spring1Count = 0;
    int spring2Count = 0;
    int springACount = 0;
    int otherCount = 0;
    for (const auto& [elementId, element] : structure->Elements())
    {
        Q_UNUSED(elementId);
        if (dynamic_cast<ElementTruss*>(element.get()))
            ++trussCount;
        else if (dynamic_cast<ElementCable*>(element.get()))
            ++cableCount;
        else if (dynamic_cast<ElementBeam_CR*>(element.get()))
            ++beamCount;
        else if (dynamic_cast<ElementSpring1*>(element.get()))
            ++spring1Count;
        else if (dynamic_cast<ElementSpring2*>(element.get()))
            ++spring2Count;
        else if (dynamic_cast<ElementSpringA*>(element.get()))
            ++springACount;
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
    if (spring1Count > 0)
    {
        auto* item = new QTreeWidgetItem(elementTypes, {QStringLiteral("SPRING1  %1").arg(spring1Count)});
        setTreeGlyph(item, TreeGlyph::SpringGround);
    }
    if (spring2Count > 0)
    {
        auto* item = new QTreeWidgetItem(elementTypes, {QStringLiteral("SPRING2  %1").arg(spring2Count)});
        setTreeGlyph(item, TreeGlyph::Spring);
    }
    if (springACount > 0)
    {
        auto* item = new QTreeWidgetItem(elementTypes, {QStringLiteral("SPRINGA  %1").arg(springACount)});
        setTreeGlyph(item, TreeGlyph::SpringAxial);
    }
    if (otherCount > 0)
    {
        auto* item = new QTreeWidgetItem(elementTypes, {QStringLiteral("OTHER  %1").arg(otherCount)});
        setTreeGlyph(item, TreeGlyph::Other);
    }

    const ThemeColors colors = themeColors(m_themeComboBox->currentIndex());
    applyTreeIcons(m_modelDataTree, QColor(colors.text), QColor(colors.accentSecond));

    m_modelDataTree->expandItem(model);
    if (materials)
        m_modelDataTree->expandItem(materials);
    if (sections)
        m_modelDataTree->expandItem(sections);
    m_modelDataTree->expandItem(elementTypes);
    m_modelDataTree->setCurrentItem(model);
}

void YQY::setModelControlsEnabled(bool enabled)
{
    const auto structure = m_modelController->model();
    enabled = enabled && static_cast<bool>(structure);
    m_modelDataTree->setEnabled(enabled);

    m_documentTabBar->setVisible(enabled);

    m_showNodesButton->setEnabled(enabled);
    m_showElementsButton->setEnabled(enabled);
    m_showSolidButton->setEnabled(enabled && m_modelViewport->hasSolidGeometry());
    m_showNodeIdsButton->setEnabled(enabled);
    m_showElementIdsButton->setEnabled(enabled);
    m_frontViewButton->setEnabled(enabled);
    m_backViewButton->setEnabled(enabled);
    m_leftViewButton->setEnabled(enabled);
    m_rightViewButton->setEnabled(enabled);
    m_topViewButton->setEnabled(enabled);
    m_bottomViewButton->setEnabled(enabled);
    m_fitViewButton->setEnabled(enabled);
    m_selectModeButton->setEnabled(enabled);
    m_rotateModeButton->setEnabled(enabled);
    m_panModeButton->setEnabled(enabled);
    m_zoomModeButton->setEnabled(enabled);
    m_startSolveButton->setEnabled(false);
    m_stopButton->setEnabled(false);
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

    m_monitorTabs->setCurrentWidget(m_logPage);
    appendLogMessage(QStringLiteral("\n[选择] 节点 %1\n"
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
    else if (dynamic_cast<ElementSpring1*>(element.get()))
        typeName = QStringLiteral("SPRING1（对地固定方向弹簧）");
    else if (dynamic_cast<ElementSpring2*>(element.get()))
        typeName = QStringLiteral("SPRING2（两节点固定方向弹簧）");
    else if (dynamic_cast<ElementSpringA*>(element.get()))
        typeName = QStringLiteral("SPRINGA（两节点随动轴向弹簧）");

    QStringList nodeIds;
    for (const auto& weakNode : element->m_pNode)
    {
        const auto node = weakNode.lock();
        if (node)
            nodeIds << QString::number(node->m_Id);
    }

    if (const auto spring = dynamic_cast<ElementSpringBase*>(element.get()))
    {
        QString behaviorId = QStringLiteral("--");
        QStringList behaviorPoints;
        double tangent = 0.0;
        if (const auto behavior = spring->m_pSpringBehavior.lock())
        {
            behaviorId = QString::number(behavior->m_Id);
            for (const SpringForceDisplacementPoint& point : behavior->m_Points)
            {
                behaviorPoints.append(
                    QStringLiteral("(%1, %2)").arg(point.force, 0, 'g', 8).arg(point.displacement, 0, 'g', 8));
            }
            tangent = spring->GetCurrentTangent();
        }

        QString directionText;
        if (const auto spring1 = dynamic_cast<ElementSpring1*>(element.get()))
            directionText = QStringLiteral("节点 DOF：%1（%2）")
                                .arg(spring1->m_DOF)
                                .arg(spring1->m_DOF < 3 ? QStringLiteral("平动") : QStringLiteral("转动"));
        else if (const auto spring2 = dynamic_cast<ElementSpring2*>(element.get()))
        {
            directionText = QStringLiteral("节点1 DOF：%1（%2），节点2 DOF：%3（%4）")
                                .arg(spring2->m_FirstDOF)
                                .arg(spring2->m_FirstDOF < 3 ? QStringLiteral("平动") : QStringLiteral("转动"))
                                .arg(spring2->m_SecondDOF)
                                .arg(spring2->m_SecondDOF < 3 ? QStringLiteral("平动") : QStringLiteral("转动"));
        }
        else
            directionText = QStringLiteral("当前长度：%1").arg(element->L, 0, 'g', 12);

        m_selectedNodeId = -1;
        m_monitorTabs->setCurrentWidget(m_logPage);
        appendLogMessage(
            QStringLiteral("\n[选择] 单元 %1\n"
                           "  类型：%2\n"
                           "  关联节点：%3\n"
                           "  %4\n"
                           "  弹簧行为：%5\n"
                           "  当前相对位移：%6，当前弹簧力：%7，当前切线刚度：%8\n"
                           "  力-相对位移点（力, 位移）：%9")
                .arg(elementId)
                .arg(typeName)
                .arg(nodeIds.isEmpty() ? QStringLiteral("--") : nodeIds.join(QStringLiteral(", ")))
                .arg(directionText)
                .arg(behaviorId)
                .arg(spring->m_CurrentRelativeDisplacement, 0, 'g', 12)
                .arg(spring->m_CurrentForce, 0, 'g', 12)
                .arg(tangent, 0, 'g', 12)
                .arg(behaviorPoints.isEmpty() ? QStringLiteral("--") : behaviorPoints.join(QStringLiteral("，"))));
        setWorkspaceMessage(QStringLiteral("已选择弹簧单元 %1").arg(elementId));
        return;
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
    m_monitorTabs->setCurrentWidget(m_logPage);
    appendLogMessage(QStringLiteral("\n[选择] 单元 %1\n"
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
    m_readyLabel->setText(message);
}

void YQY::updateLoadStatistics()
{
    m_loadStatisticsLabel->setText(QStringLiteral("模型读取  总计 %1  ·  成功 %2  ·  失败 %3")
                                       .arg(m_modelLoadTotal)
                                       .arg(m_modelLoadSucceeded)
                                       .arg(m_modelLoadFailed));
}
