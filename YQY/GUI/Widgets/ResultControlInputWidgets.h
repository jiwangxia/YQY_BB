#pragma once

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QStylePainter>
#include <QStyleOptionComboBox>

// 结果名称过长时只省略绘制文本，列表内容和当前值保持完整。
class ResultSelectionComboBox final : public QComboBox
{
public:
    explicit ResultSelectionComboBox(QWidget* parent = nullptr) // 创建结果字段选择框
        : QComboBox(parent)
    {
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QStylePainter painter(this);
        QStyleOptionComboBox option;
        initStyleOption(&option);
        painter.drawComplexControl(QStyle::CC_ComboBox, option);
        const QRect field = style()->subControlRect(QStyle::CC_ComboBox, &option,
                                                    QStyle::SC_ComboBoxEditField, this);
        const int iconWidth = option.currentIcon.isNull() ? 0 : option.iconSize.width() + 4;
        option.currentText = fontMetrics().elidedText(option.currentText, Qt::ElideRight,
                                                      qMax(0, field.width() - iconWidth - 4));
        painter.drawControl(QStyle::CE_ComboBoxLabel, option);
    }
};

class ResultScaleSpinBox final : public QDoubleSpinBox
{
public:
    explicit ResultScaleSpinBox(QWidget* parent = nullptr)      // 创建结果变形比例输入框
        : QDoubleSpinBox(parent)
    {
    }
};

class ChevronComboBox final : public QComboBox
{
public:
    explicit ChevronComboBox(QWidget* parent = nullptr)         // 创建使用统一箭头样式的下拉框
        : QComboBox(parent)
    {
    }
};

class ChevronDoubleSpinBox final : public QDoubleSpinBox
{
public:
    explicit ChevronDoubleSpinBox(QWidget* parent = nullptr)    // 创建使用统一箭头样式的小数输入框
        : QDoubleSpinBox(parent)
    {
    }
};

class ChevronSpinBox final : public QSpinBox
{
public:
    explicit ChevronSpinBox(QWidget* parent = nullptr)          // 创建使用统一箭头样式的整数输入框
        : QSpinBox(parent)
    {
    }
};
