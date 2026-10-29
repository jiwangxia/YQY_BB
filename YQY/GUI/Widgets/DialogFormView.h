#pragma once

#include <QScrollArea>

class QDialog;

// 使用原生控件布局，空间不足时滚动；不改变文字或控件的显示比例。
class DialogFormView final : public QScrollArea
{
public:
    explicit DialogFormView(QWidget* parent);
    static DialogFormView* install(QDialog* dialog);
    static QWidget* contentWidget(QDialog* dialog);
};
