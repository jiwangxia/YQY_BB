#pragma once

#include <QLabel>
#include <QStylePainter>

// 状态文字使用整行空间；空间不足时省略，完整内容可悬停查看。
class SingleLineLabel final : public QLabel
{
public:
    explicit SingleLineLabel(QWidget* parent = nullptr) // 创建可省略长文本的单行标签
        : QLabel(parent)
    {
        setWordWrap(false);
        setTextFormat(Qt::PlainText);
        setMinimumWidth(0);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    }

    void setText(const QString& text)                   // 简化显示文本并保留完整工具提示
    {
        QLabel::setText(text.simplified());
        setToolTip(text);
    }

    QSize sizeHint() const override                     // 提供可水平收缩的单行标签建议尺寸
    {
        QSize size = QLabel::sizeHint();
        size.setWidth(0);
        return size;
    }

    QSize minimumSizeHint() const override              // 允许标签随容器压缩宽度
    {
        return sizeHint();
    }

protected:
    void paintEvent(QPaintEvent*) override              // 绘制单行文本并省略超出宽度的部分
    {
        QStylePainter painter(this);
        const QString visibleText =
            fontMetrics().elidedText(text().simplified(), Qt::ElideRight, contentsRect().width());
        painter.drawItemText(contentsRect(), alignment(), palette(), isEnabled(), visibleText, QPalette::WindowText);
    }
};
