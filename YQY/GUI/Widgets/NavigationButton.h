#pragma once

#include <QPushButton>
#include <QStyleOptionButton>
#include <QStylePainter>

// 固定图标在上、文本在下的导航布局，避免字体或缩放改变两者的对齐。
class NavigationButton final : public QPushButton
{
public:
    explicit NavigationButton(QWidget* parent = nullptr)                      // 创建侧栏导航按钮
        : QPushButton(parent)
    {
    }

    explicit NavigationButton(const QString& text, QWidget* parent = nullptr) // 创建侧栏导航按钮
        : QPushButton(text, parent)
    {
    }

    QSize sizeHint() const override                                           // 计算满足侧栏图标和文字排布的建议尺寸
    {
        const QSize base = QPushButton::sizeHint();
        return QSize(qMax(base.width(), 64), qMax(base.height(), 58));
    }

protected:
    void paintEvent(QPaintEvent*) override                                    // 绘制导航图标、文字和焦点状态
    {
        QStyleOptionButton option;
        initStyleOption(&option);

        QStylePainter painter(this);
        painter.drawControl(QStyle::CE_PushButtonBevel, option);

        const QSize requestedIconSize = iconSize().boundedTo(QSize(width() - 12, 20));
        const int contentHeight = requestedIconSize.height() + 5 + fontMetrics().height();
        const int contentTop = qMax(4, (height() - contentHeight) / 2);
        const QRect iconRect((width() - requestedIconSize.width()) / 2, contentTop, requestedIconSize.width(),
                             requestedIconSize.height());
        const QIcon::Mode mode = isEnabled() ? (underMouse() ? QIcon::Active : QIcon::Normal) : QIcon::Disabled;
        const QIcon::State state = isChecked() ? QIcon::On : QIcon::Off;
        icon().paint(&painter, iconRect, Qt::AlignCenter, mode, state);

        const QRect textRect(4, iconRect.bottom() + 5, width() - 8, fontMetrics().height());
        painter.setPen(palette().color(isChecked() ? QPalette::BrightText : QPalette::ButtonText));
        painter.drawText(textRect, Qt::AlignHCenter | Qt::AlignVCenter | Qt::TextSingleLine, text());

        if (option.state.testFlag(QStyle::State_HasFocus))
        {
            QStyleOptionFocusRect focus;
            focus.QStyleOption::operator=(option);
            focus.rect = rect().adjusted(3, 3, -3, -3);
            focus.state |= QStyle::State_KeyboardFocusChange;
            style()->drawPrimitive(QStyle::PE_FrameFocusRect, &focus, &painter, this);
        }
    }
};
