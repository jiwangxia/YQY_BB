#pragma once

#include <QEnterEvent>
#include <QPainter>
#include <QSplitter>

// 保留整行拖动范围，用居中的短柄提示上下区域可调整。
class WorkspaceSplitter final : public QSplitter
{
public:
    explicit WorkspaceSplitter(QWidget* parent = nullptr) // 创建视图与监视面板的垂直分隔器
        : QSplitter(Qt::Vertical, parent)
    {
        setHandleWidth(12);
    }

protected:
    QSplitterHandle* createHandle() override              // 创建工作区自定义分隔手柄
    {
        return new Handle(this);
    }

private:
    class Handle final : public QSplitterHandle
    {
    public:
        explicit Handle(QSplitter* parent)                // 创建可拖动的面板高度调节手柄
            : QSplitterHandle(Qt::Vertical, parent)
        {
            setCursor(Qt::SplitVCursor);
            setToolTip(QStringLiteral("上下拖动调整区域高度"));
            setAccessibleName(QStringLiteral("图形区与日志区高度调节柄"));
        }

    protected:
        void enterEvent(QEnterEvent* event) override      // 在鼠标进入时更新手柄悬停外观
        {
            QSplitterHandle::enterEvent(event);
            update();
        }

        void leaveEvent(QEvent* event) override           // 在鼠标离开时恢复手柄外观
        {
            QSplitterHandle::leaveEvent(event);
            update();
        }

        void paintEvent(QPaintEvent* event) override      // 绘制面板高度调节手柄
        {
            QSplitterHandle::paintEvent(event);
            QPainter painter(this);
            painter.setRenderHint(QPainter::Antialiasing);
            painter.setPen(Qt::NoPen);
            painter.setBrush(palette().color(underMouse() ? QPalette::WindowText : QPalette::Mid));
            const QRectF grip((width() - 40.0) / 2.0, (height() - 4.0) / 2.0, 40.0, 4.0);
            painter.drawRoundedRect(grip, 2.0, 2.0);
        }
    };
};
