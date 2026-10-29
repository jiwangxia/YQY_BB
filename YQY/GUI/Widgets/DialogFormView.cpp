#include "DialogFormView.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QLayout>
#include <QVBoxLayout>

DialogFormView::DialogFormView(QWidget* parent)
    : QScrollArea(parent)
{
    setObjectName(QStringLiteral("dialogFormScroll"));
    setFrameShape(QFrame::NoFrame);
    setWidgetResizable(true);
    setAlignment(Qt::AlignLeft | Qt::AlignTop);
}

DialogFormView* DialogFormView::install(QDialog* dialog)
{
    auto* view = dynamic_cast<DialogFormView*>(dialog->findChild<QScrollArea*>(
        QStringLiteral("dialogFormScroll"), Qt::FindDirectChildrenOnly));
    if (view || !dialog->layout())
    {
        return view;
    }

    QLayout* contentLayout = dialog->layout();
    QDialogButtonBox* buttons = nullptr;
    for (auto* candidate : dialog->findChildren<QDialogButtonBox*>(QString(), Qt::FindDirectChildrenOnly))
    {
        if (contentLayout->indexOf(candidate) >= 0)
        {
            buttons = candidate;
            contentLayout->removeWidget(buttons);
            break;
        }
    }
    const QMargins margins = contentLayout->contentsMargins();
    auto* body = new QWidget;
    body->setObjectName(QStringLiteral("dialogFormBody"));
    body->setLayout(contentLayout);
    contentLayout->setSizeConstraint(QLayout::SetMinimumSize);
    view = new DialogFormView(dialog);
    view->setWidget(body);
    auto* root = new QVBoxLayout(dialog);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    root->setSizeConstraint(QLayout::SetNoConstraint);
    root->addWidget(view, 1);
    if (buttons)
    {
        // 确认与取消留在滚动区外，小窗口下仍可直接操作。
        auto* footer = new QHBoxLayout;
        footer->setContentsMargins(margins.left(), 0, margins.right(), margins.bottom());
        footer->addWidget(buttons);
        root->addLayout(footer);
    }
    return view;
}

QWidget* DialogFormView::contentWidget(QDialog* dialog)
{
    auto* view = dynamic_cast<DialogFormView*>(dialog->findChild<QScrollArea*>(
        QStringLiteral("dialogFormScroll"), Qt::FindDirectChildrenOnly));
    return view ? view->widget() : dialog;
}
