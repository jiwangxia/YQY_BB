#pragma once

#include "DialogFormView.h"
#include "Utility/DialogScreenAdapter.h"
#include <QDialog>
#include <QLayout>
#include <QTimer>

namespace DialogSizing
{
// 仅设置首次打开尺寸，后续由原生布局和滚动区处理窗口大小变化。
inline void fitFormToScreen(QDialog* dialog, bool preserveInitialHeight = false, bool compactForm = false)
{
    QTimer::singleShot(0, dialog, [dialog, preserveInitialHeight, compactForm]()
    {
        QWidget* content = DialogFormView::contentWidget(dialog);
        const bool installed = content != dialog;
        content->ensurePolished();
        if (!content->layout())
        {
            return;
        }
        content->layout()->activate();
        int width = qMax(dialog->width(), content->sizeHint().width());
        if (compactForm)
        {
            width = qMax(content->minimumSizeHint().width(), qMin(width, 720));
        }
        const int naturalHeight = qMax(content->sizeHint().height(),
                                        content->layout()->totalHeightForWidth(width));
        const int height = preserveInitialHeight ? dialog->height() : naturalHeight;
        auto* view = DialogFormView::install(dialog);
        if (view && !installed)
        {
            dialog->resize(width, height);
        }
        DialogScreenAdapter::install(dialog, QSize(360, 260));
    });
}

inline void lockHeightToContents(QDialog* dialog) // 按内容确定首次打开高度
{
    fitFormToScreen(dialog);
}

inline void fitCompactFormToScreen(QDialog* dialog) // 使用紧凑的首次打开宽度
{
    fitFormToScreen(dialog, false, true);
}

inline void lockCurrentHeight(QDialog* dialog) // 保留调用方指定的首次打开高度
{
    fitFormToScreen(dialog, true);
}
}
