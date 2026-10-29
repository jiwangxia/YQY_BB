#include "Application/ApplicationBootstrap.h"

#include "GUI/YQY.h"
#include "Utility/UiIcons.h"

#include <QSurfaceFormat>
#include <QTranslator>
#include <QVTKOpenGLNativeWidget.h>

namespace
{
class StandardButtonTranslator final : public QTranslator
{
public:
    using QTranslator::QTranslator;

    QString translate(const char* context, const char* sourceText, const char* disambiguation = nullptr,
                      int pluralCount = -1) const override
    {
        Q_UNUSED(context);
        Q_UNUSED(disambiguation);
        Q_UNUSED(pluralCount);

        const QString source = QString::fromUtf8(sourceText);
        if (source == QStringLiteral("OK"))
            return QStringLiteral("确定");
        if (source == QStringLiteral("Save"))
            return QStringLiteral("保存");
        if (source == QStringLiteral("Save All"))
            return QStringLiteral("全部保存");
        if (source == QStringLiteral("Open"))
            return QStringLiteral("打开");
        if (source == QStringLiteral("Yes"))
            return QStringLiteral("是");
        if (source == QStringLiteral("Yes to All"))
            return QStringLiteral("全部是");
        if (source == QStringLiteral("No"))
            return QStringLiteral("否");
        if (source == QStringLiteral("No to All"))
            return QStringLiteral("全部否");
        if (source == QStringLiteral("Abort"))
            return QStringLiteral("中止");
        if (source == QStringLiteral("Retry"))
            return QStringLiteral("重试");
        if (source == QStringLiteral("Ignore"))
            return QStringLiteral("忽略");
        if (source == QStringLiteral("Close"))
            return QStringLiteral("关闭");
        if (source == QStringLiteral("Cancel"))
            return QStringLiteral("取消");
        if (source == QStringLiteral("Discard"))
            return QStringLiteral("放弃");
        if (source == QStringLiteral("Help"))
            return QStringLiteral("帮助");
        if (source == QStringLiteral("Apply"))
            return QStringLiteral("应用");
        if (source == QStringLiteral("Reset"))
            return QStringLiteral("重置");
        if (source == QStringLiteral("Restore Defaults"))
            return QStringLiteral("恢复默认值");
        return {};
    }
};
}

void ApplicationBootstrap::prepareEnvironment()
{
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
}

void ApplicationBootstrap::configureApplication(QApplication& application)
{
    application.installTranslator(new StandardButtonTranslator(&application));
    QApplication::setOrganizationName(QStringLiteral("YQY"));
    QApplication::setApplicationName(QStringLiteral("YQY"));
    QApplication::setWindowIcon(UiIcons::applicationIcon());
    QApplication::setStyle(QStringLiteral("Fusion"));                      //默认风格
    QApplication::setFont(QFont(QStringLiteral("Microsoft YaHei UI"), 9)); //默认字体
}

void ApplicationBootstrap::prepareMainWindow(YQY& window)
{
    const QScreen* screen = QGuiApplication::primaryScreen();
    if (!screen)
        return;

    const QRect available = screen->availableGeometry();
    constexpr int margin = 24;
    const int maximumWidth = qMax(760, available.width() - margin * 2);
    const int maximumHeight = qMax(520, available.height() - margin * 2);
    const int initialWidth = qMin(maximumWidth, qMax(760, qRound(available.width() * 0.68)));
    const int initialHeight = qMin(maximumHeight, qMax(520, qRound(available.height() * 0.72)));
    window.resize(initialWidth, initialHeight);
    window.move(available.center() - window.rect().center());
}
