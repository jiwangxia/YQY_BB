#include "StepAlgorithmStack.h"
#include <QLayout>
#include <QSize>
#include <QStackedWidget>
#include <QWidget>

StepAlgorithmStack::StepAlgorithmStack(QWidget* parent)
    : QStackedWidget(parent)
{
    layout()->setSizeConstraint(QLayout::SetNoConstraint);
}

QSize StepAlgorithmStack::sizeHint() const
{
    return currentWidget() ? currentWidget()->sizeHint() : QSize();
}

QSize StepAlgorithmStack::minimumSizeHint() const
{
    return currentWidget() ? currentWidget()->minimumSizeHint() : QSize();
}

bool StepAlgorithmStack::hasHeightForWidth() const
{
    return currentWidget() && currentWidget()->hasHeightForWidth();
}

int StepAlgorithmStack::heightForWidth(int width) const
{
    return currentWidget() ? currentWidget()->heightForWidth(width) : -1;
}
