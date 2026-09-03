#pragma once

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QPaintEvent>
#include <QPainter>
#include <QSpinBox>
#include <QStyle>
#include <QStyleOptionComboBox>
#include <QStyleOptionSpinBox>

// 使用自绘下拉箭头的结果选择框。
class ResultSelectionComboBox final : public QComboBox
{
public:
    explicit ResultSelectionComboBox(QWidget* parent = nullptr) // 创建结果选择框
        : QComboBox(parent)
    {
    }

protected:
    void paintEvent(QPaintEvent* event) override // 绘制原控件和下拉箭头
    {
        QComboBox::paintEvent(event);

        QStyleOptionComboBox option;
        initStyleOption(&option);
        const QRect arrowRect = style()->subControlRect(QStyle::CC_ComboBox, &option, QStyle::SC_ComboBoxArrow, this);
        DrawChevron(arrowRect);
    }

private:
    void DrawChevron(const QRect& rect) // 在给定区域绘制向下箭头
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        QColor color = palette().color(isEnabled() ? QPalette::Active : QPalette::Disabled, QPalette::Text);
        color.setAlphaF(0.72F);
        QPen pen(color, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter.setPen(pen);

        const QPoint center = rect.center();
        QPainterPath path;
        path.moveTo(center.x() - 4, center.y() - 2);
        path.lineTo(center.x(), center.y() + 2);
        path.lineTo(center.x() + 4, center.y() - 2);
        painter.drawPath(path);
    }
};

// 使用自绘上下箭头的结果缩放系数输入框。
class ResultScaleSpinBox final : public QDoubleSpinBox
{
public:
    explicit ResultScaleSpinBox(QWidget* parent = nullptr) // 创建缩放系数输入框
        : QDoubleSpinBox(parent)
    {
    }

protected:
    void paintEvent(QPaintEvent* event) override // 绘制原控件和上下箭头
    {
        QDoubleSpinBox::paintEvent(event);

        QStyleOptionSpinBox option;
        initStyleOption(&option);
        DrawChevron(style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxUp, this), true);
        DrawChevron(style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxDown, this), false);
    }

private:
    void DrawChevron(const QRect& rect, bool upward) // 按方向绘制箭头
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        QColor color = palette().color(isEnabled() ? QPalette::Active : QPalette::Disabled, QPalette::Text);
        color.setAlphaF(0.72F);
        QPen pen(color, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
        painter.setPen(pen);

        const QPoint center = rect.center();
        QPainterPath path;
        if (upward)
        {
            path.moveTo(center.x() - 3, center.y() + 1);
            path.lineTo(center.x(), center.y() - 2);
            path.lineTo(center.x() + 3, center.y() + 1);
        }
        else
        {
            path.moveTo(center.x() - 3, center.y() - 1);
            path.lineTo(center.x(), center.y() + 2);
            path.lineTo(center.x() + 3, center.y() - 1);
        }
        painter.drawPath(path);
    }
};

// 使用与结果面板一致的细线箭头，供普通参数编辑页使用。
class ChevronComboBox final : public QComboBox
{
public:
    explicit ChevronComboBox(QWidget* parent = nullptr)
        : QComboBox(parent)
    {
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        QComboBox::paintEvent(event);

        QStyleOptionComboBox option;
        initStyleOption(&option);
        DrawChevron(style()->subControlRect(QStyle::CC_ComboBox, &option, QStyle::SC_ComboBoxArrow, this));
    }

private:
    void DrawChevron(const QRect& rect)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        QColor color = palette().color(isEnabled() ? QPalette::Active : QPalette::Disabled, QPalette::Text);
        color.setAlphaF(0.72F);
        painter.setPen(QPen(color, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

        const QPoint center = rect.center();
        QPainterPath path;
        path.moveTo(center.x() - 4, center.y() - 2);
        path.lineTo(center.x(), center.y() + 2);
        path.lineTo(center.x() + 4, center.y() - 2);
        painter.drawPath(path);
    }
};

class ChevronDoubleSpinBox final : public QDoubleSpinBox
{
public:
    explicit ChevronDoubleSpinBox(QWidget* parent = nullptr)
        : QDoubleSpinBox(parent)
    {
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        QDoubleSpinBox::paintEvent(event);

        QStyleOptionSpinBox option;
        initStyleOption(&option);
        DrawChevron(style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxUp, this), true);
        DrawChevron(style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxDown, this), false);
    }

private:
    void DrawChevron(const QRect& rect, bool upward)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        QColor color = palette().color(isEnabled() ? QPalette::Active : QPalette::Disabled, QPalette::Text);
        color.setAlphaF(0.72F);
        painter.setPen(QPen(color, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

        const QPoint center = rect.center();
        QPainterPath path;
        if (upward)
        {
            path.moveTo(center.x() - 3, center.y() + 1);
            path.lineTo(center.x(), center.y() - 2);
            path.lineTo(center.x() + 3, center.y() + 1);
        }
        else
        {
            path.moveTo(center.x() - 3, center.y() - 1);
            path.lineTo(center.x(), center.y() + 2);
            path.lineTo(center.x() + 3, center.y() - 1);
        }
        painter.drawPath(path);
    }
};

class ChevronSpinBox final : public QSpinBox
{
public:
    explicit ChevronSpinBox(QWidget* parent = nullptr)
        : QSpinBox(parent)
    {
    }

protected:
    void paintEvent(QPaintEvent* event) override
    {
        QSpinBox::paintEvent(event);

        QStyleOptionSpinBox option;
        initStyleOption(&option);
        DrawChevron(style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxUp, this), true);
        DrawChevron(style()->subControlRect(QStyle::CC_SpinBox, &option, QStyle::SC_SpinBoxDown, this), false);
    }

private:
    void DrawChevron(const QRect& rect, bool upward)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        QColor color = palette().color(isEnabled() ? QPalette::Active : QPalette::Disabled, QPalette::Text);
        color.setAlphaF(0.72F);
        painter.setPen(QPen(color, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

        const QPoint center = rect.center();
        QPainterPath path;
        if (upward)
        {
            path.moveTo(center.x() - 3, center.y() + 1);
            path.lineTo(center.x(), center.y() - 2);
            path.lineTo(center.x() + 3, center.y() + 1);
        }
        else
        {
            path.moveTo(center.x() - 3, center.y() - 1);
            path.lineTo(center.x(), center.y() + 2);
            path.lineTo(center.x() + 3, center.y() - 1);
        }
        painter.drawPath(path);
    }
};
