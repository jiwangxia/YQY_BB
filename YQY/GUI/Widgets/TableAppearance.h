#pragma once

#include <QHeaderView>
#include <QStyledItemDelegate>
#include <QTableView>

// 将表格单元格文本居中显示的委托。
class CenteredTableItemDelegate final : public QStyledItemDelegate
{
public:
    explicit CenteredTableItemDelegate(QObject* parent = nullptr)                               // 创建表格内容居中绘制代理
        : QStyledItemDelegate(parent)
    {
    }

    void initStyleOption(QStyleOptionViewItem* option, const QModelIndex& index) const override // 设置表格单元格文字居中对齐
    {
        QStyledItemDelegate::initStyleOption(option, index);
        option->displayAlignment = Qt::AlignCenter;
    }
};

// 统一设置表头和单元格的居中显示。
inline void applyCenteredTableAppearance(QTableView* table)                                     // 统一表头和单元格的居中显示
{
    if (!table)
        return;

    table->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
    table->setItemDelegate(new CenteredTableItemDelegate(table));
}
