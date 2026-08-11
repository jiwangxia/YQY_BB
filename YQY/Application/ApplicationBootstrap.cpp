#include "Application/ApplicationBootstrap.h"

#include "GUI/YQY.h"

#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>

void ApplicationBootstrap::prepareEnvironment()
{
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
}

void ApplicationBootstrap::configureApplication(QApplication& application)
{
    Q_UNUSED(application);//消除未使用参数警告
    QApplication::setOrganizationName(QStringLiteral("YQY"));
    QApplication::setApplicationName(QStringLiteral("YQY"));
    QApplication::setWindowIcon(QIcon(QStringLiteral(":/YQY/app_icon.ico")));
    QApplication::setStyle(QStringLiteral("Fusion"));//默认风格
    QApplication::setFont(QFont(QStringLiteral("Microsoft YaHei UI"), 9));//默认字体
}

void ApplicationBootstrap::prepareMainWindow(YQY& window)
{
    const QScreen* screen = QGuiApplication::primaryScreen();
    if (!screen) return;

    const QRect available = screen->availableGeometry();
    constexpr int margin = 24;
    const int maximumWidth = qMax(760, available.width() - margin * 2);
    const int maximumHeight = qMax(520, available.height() - margin * 2);
    const int initialWidth = qMin(maximumWidth, qMax(760, qRound(available.width() * 0.68)));
    const int initialHeight = qMin(maximumHeight, qMax(520, qRound(available.height() * 0.72)));
    window.resize(initialWidth, initialHeight);
    window.move(available.center() - window.rect().center());
}
