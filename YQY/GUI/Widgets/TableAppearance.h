#pragma once

#include <QHeaderView>
#include <QStyledItemDelegate>
#include <QTableView>

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

inline void applyCenteredTableAppearance(QTableView* table)
{
    if (!table)
        return;

    table->horizontalHeader()->setDefaultAlignment(Qt::AlignCenter);
    table->setItemDelegate(new CenteredTableItemDelegate(table));
}
