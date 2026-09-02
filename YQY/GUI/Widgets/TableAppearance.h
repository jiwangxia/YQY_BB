#pragma once

#include <QHeaderView>
#include <QStyledItemDelegate>
#include <QTableView>

// 将表格单元格文本居中显示的委托。
class CenteredTableItemDelegate final : public QStyledItemDelegate
{
public:
    explicit CenteredTableItemDelegate(QObject* parent = nullptr)
        : QStyledItemDelegate(parent)
    {
    }

    void initStyleOption(QStyleOptionViewItem* option, const QModelIndex& index) const override
    {
        QStyledItemDelegate::initStyleOption(option, index);
        option->displayAlignment = Qt::AlignCenter;
    }
};

// 统一设置表头和单元格的居中显示。
inline void applyCenteredTableAppearance(QTableView* table)
{
    if (!table)
        return;

    table->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
    table->setItemDelegate(new CenteredTableItemDelegate(table));
}
