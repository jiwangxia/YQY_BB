#include "DialogScreenAdapter.h"

#include <QDialog>
#include <QEvent>
#include <QGuiApplication>
#include <QLayout>
#include <QScreen>
#include <QScopedValueRollback>
#include <QTimer>
#include <QWindow>

DialogScreenAdapter::DialogScreenAdapter(QDialog* dialog)
    : QObject(dialog)
    , m_dialog(dialog)
{
    setObjectName(QStringLiteral("dialogScreenAdapter"));
    dialog->installEventFilter(this);
}

void DialogScreenAdapter::install(QDialog* dialog, const QSize& preferredMinimum, const QSize& preferredMaximum)
{
    auto* adapter = dynamic_cast<DialogScreenAdapter*>(
        dialog->findChild<QObject*>(QStringLiteral("dialogScreenAdapter"), Qt::FindDirectChildrenOnly));
    if (!adapter)
        adapter = new DialogScreenAdapter(dialog);
    adapter->m_preferredMinimum = preferredMinimum.expandedTo(QSize(1, 1));
    adapter->m_preferredMaximum = preferredMaximum;
    if (dialog->layout())
        dialog->layout()->setSizeConstraint(QLayout::SetNoConstraint);
    adapter->scheduleFit();
}

bool DialogScreenAdapter::eventFilter(QObject* watched, QEvent* event)
{
    if (watched != m_dialog)
        return QObject::eventFilter(watched, event);
    if (event->type() == QEvent::Show || event->type() == QEvent::WinIdChange ||
        event->type() == QEvent::ScreenChangeInternal)
    {
        scheduleFit();
    }
    return QObject::eventFilter(watched, event);
}

void DialogScreenAdapter::scheduleFit()
{
    if (m_pending)
        return;
    m_pending = true;
    QTimer::singleShot(0, this,
                       [this]()
                       {
                           m_pending = false;
                           fitToScreen();
                       });
}

void DialogScreenAdapter::observeScreen()
{
    QWindow* window = m_dialog->windowHandle();
    if (m_window != window)
    {
        disconnect(m_windowConnection);
        m_window = window;
        if (window)
            m_windowConnection = connect(window, &QWindow::screenChanged, this,
                                         [this]() { scheduleFit(); });
    }
    QScreen* screen = m_dialog->screen();
    if (!screen)
        screen = QGuiApplication::primaryScreen();
    if (m_screen == screen)
        return;
    disconnect(m_geometryConnection);
    disconnect(m_dpiConnection);
    m_screen = screen;
    if (screen)
    {
        m_geometryConnection = connect(screen, &QScreen::availableGeometryChanged, this,
                                       [this]() { scheduleFit(); });
        m_dpiConnection = connect(screen, &QScreen::logicalDotsPerInchChanged, this,
                                  [this]() { scheduleFit(); });
    }
}

void DialogScreenAdapter::fitToScreen()
{
    if (m_applyingGeometry)
        return;
    const QScopedValueRollback<bool> applyingGeometry(m_applyingGeometry, true);
    observeScreen();
    if (!m_screen)
        return;
    const QRect workArea = m_screen->availableGeometry().adjusted(8, 8, -8, -8);
    const QMargins frame = m_window ? m_window->frameMargins() : QMargins(0, 32, 0, 0);
    const QSize clientArea(qMax(1, workArea.width() - frame.left() - frame.right()),
                           qMax(1, workArea.height() - frame.top() - frame.bottom()));
    const QSize maximum = m_preferredMaximum.isValid()
                              ? m_preferredMaximum.expandedTo(QSize(1, 1)).boundedTo(clientArea) : clientArea;
    const QSize minimum = m_preferredMinimum.boundedTo(maximum);
    const QSize targetSize = m_dialog->size().boundedTo(maximum).expandedTo(minimum);
    if (m_dialog->minimumSize() != minimum)
        m_dialog->setMinimumSize(minimum);
    if (m_dialog->maximumSize() != maximum)
        m_dialog->setMaximumSize(maximum);
    if (m_dialog->isMaximized() || m_dialog->isFullScreen())
        return;
    if (m_dialog->isVisible() && m_dialog->size() != targetSize)
        m_dialog->resize(targetSize);
    if (!m_dialog->isVisible())
        return;
    const QRect currentFrame = m_dialog->frameGeometry();
    const int left = qBound(workArea.left(), currentFrame.left(),
                            qMax(workArea.left(), workArea.right() - currentFrame.width() + 1));
    const int top = qBound(workArea.top(), currentFrame.top(),
                           qMax(workArea.top(), workArea.bottom() - currentFrame.height() + 1));
    const QPoint offset(left - currentFrame.left(), top - currentFrame.top());
    if (!offset.isNull())
        m_dialog->move(m_dialog->pos() + offset);
}
