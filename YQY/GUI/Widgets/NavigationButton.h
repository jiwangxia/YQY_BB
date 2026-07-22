#pragma once

#include <QPushButton>
#include <QStyleOptionButton>
#include <QStylePainter>

// Navigation rail button with a stable icon-above-text layout.  Keeping the
// layout in painting code avoids font/DPI dependent newline glyph alignment.
class NavigationButton final : public QPushButton
{
public:
    explicit NavigationButton(QWidget* parent = nullptr) : QPushButton(parent) {}

    explicit NavigationButton(const QString& text, QWidget* parent = nullptr) : QPushButton(text, parent) {}

    QSize sizeHint() const override
    {
        const QSize base = QPushButton::sizeHint();
        return QSize(qMax(base.width(), 64), qMax(base.height(), 58));
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QStyleOptionButton option;
        initStyleOption(&option);

        QStylePainter painter(this);
        painter.drawControl(QStyle::CE_PushButtonBevel, option);

        const QSize requestedIconSize = iconSize().boundedTo(QSize(width() - 12, 20));
        const QRect iconRect(
            (width() - requestedIconSize.width()) / 2,
            3,
            requestedIconSize.width(),
            requestedIconSize.height());
        const QIcon::Mode mode = isEnabled()
            ? (underMouse() ? QIcon::Active : QIcon::Normal)
            : QIcon::Disabled;
        const QIcon::State state = isChecked() ? QIcon::On : QIcon::Off;
        icon().paint(&painter, iconRect, Qt::AlignCenter, mode, state);

        const QRect textRect = rect().adjusted(4, iconRect.bottom() + 1, -4, -2);
        painter.setPen(palette().color(isChecked() ? QPalette::BrightText : QPalette::ButtonText));
        painter.drawText(textRect,
            Qt::AlignHCenter | Qt::AlignVCenter | Qt::TextSingleLine, text());

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
