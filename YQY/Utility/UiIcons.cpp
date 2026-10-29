#include "Utility/UiIcons.h"

#include <QPixmap>

QIcon UiIcons::applicationIcon()
{
    static const QIcon icon = []()
    {
        // Qt 窗口直接使用 PNG，避免程序标识依赖 ICO 解码插件。
        const QPixmap source(QStringLiteral(":/YQY/app_icon_source.png"));
        QIcon result;
        for (const int extent : {16, 20, 24, 32, 40, 48, 64, 128, 256})
            result.addPixmap(source.scaled(extent, extent, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        return result;
    }();
    return icon;
}
