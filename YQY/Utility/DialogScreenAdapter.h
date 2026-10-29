#pragma once

#include <QObject>
#include <QPointer>
#include <QSize>

class QDialog;
class QScreen;
class QWindow;

// 打开或切换屏幕时适配可用区域，不干预正常拖动。
class DialogScreenAdapter final : public QObject
{
public:
    static void install(QDialog* dialog, const QSize& preferredMinimum, const QSize& preferredMaximum = QSize());

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    explicit DialogScreenAdapter(QDialog* dialog);
    void scheduleFit();
    void fitToScreen();
    void observeScreen();

    QDialog* m_dialog = nullptr;
    QSize m_preferredMinimum;
    QSize m_preferredMaximum;
    QPointer<QWindow> m_window;
    QPointer<QScreen> m_screen;
    QMetaObject::Connection m_windowConnection;
    QMetaObject::Connection m_geometryConnection;
    QMetaObject::Connection m_dpiConnection;
    bool m_pending = false;
    bool m_applyingGeometry = false;
};
