#pragma once

#include <QDialog>
#include <QLayout>
#include <QSizePolicy>
#include <QTimer>

namespace DialogSizing
{
// Form-only dialogs have no meaningful vertically expanding content. Lock their
// height after Qt has completed polish/layout, while leaving the width resizable.
inline void lockHeightToContents(QDialog* dialog)
{
    dialog->setWindowFlag(Qt::WindowMaximizeButtonHint, false);
    QTimer::singleShot(0, dialog, [dialog]()
    {
        if (QLayout* layout = dialog->layout())
            layout->activate();

        const int previousWidth = dialog->width();
        dialog->adjustSize();
        dialog->resize(qMax(previousWidth, dialog->width()), dialog->height());

        const int contentHeight = dialog->height();
        dialog->setMinimumHeight(contentHeight);
        dialog->setMaximumHeight(contentHeight);
        dialog->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    });
}

// Use this for deliberately composed dialogs whose designed initial height is
// already correct but whose contents do not benefit from vertical resizing.
inline void lockCurrentHeight(QDialog* dialog)
{
    dialog->setWindowFlag(Qt::WindowMaximizeButtonHint, false);
    QTimer::singleShot(0, dialog, [dialog]()
    {
        if (QLayout* layout = dialog->layout())
            layout->activate();

        const int designedHeight = dialog->height();
        dialog->setMinimumHeight(designedHeight);
        dialog->setMaximumHeight(designedHeight);
        dialog->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    });
}
}
