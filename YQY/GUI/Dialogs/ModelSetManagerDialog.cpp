#include "ModelSetManagerDialog.h"
#include "Widgets/DialogSizing.h"
#include "Utility/UiIcons.h"

#include "DataStructure/Region/ComputeRegion.h"
#include "DataStructure/Region/ModelSet.h"
#include "DataStructure/Structure/StructureData.h"

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include <map>
#include <set>
#include <vector>

namespace
{

struct LogicalCollection
{
    QString name;
    int nodeSetId = 0;
    int elementSetId = 0;
    std::set<int> nodeIds;
    std::set<int> elementIds;
};

QString logicalName(const ModelSet& modelSet)
{
    const QString suffix =
        modelSet.m_Type == ModelSetType::Node ? QStringLiteral(" · 节点") : QStringLiteral(" · 单元");
    return modelSet.m_Name.endsWith(suffix) ? modelSet.m_Name.left(modelSet.m_Name.size() - suffix.size()).trimmed()
                                            : modelSet.m_Name.trimmed();
}

std::vector<LogicalCollection> collectLogicalCollections(const StructureData& structure)
{
    std::map<QString, LogicalCollection> grouped;
    for (const auto& [setId, modelSet] : structure.m_ModelSets)
    {
        if (!modelSet)
            continue;
        const QString name = logicalName(*modelSet);
        if (name.isEmpty())
            continue;
        auto& collection = grouped[name];
        collection.name = name;
        if (modelSet->m_Type == ModelSetType::Node)
        {
            collection.nodeSetId = setId;
            collection.nodeIds = modelSet->m_Ids;
        }
        else
        {
            collection.elementSetId = setId;
            collection.elementIds = modelSet->m_Ids;
        }
    }

    std::vector<LogicalCollection> result;
    result.reserve(grouped.size());
    for (auto& [name, collection] : grouped)
    {
        Q_UNUSED(name);
        result.push_back(std::move(collection));
    }
    return result;
}

int regionReferenceCount(const StructureData& structure, int nodeSetId, int elementSetId)
{
    int count = 0;
    for (const auto& [regionId, region] : structure.m_ComputeRegions)
    {
        Q_UNUSED(regionId);
        if (region && ((nodeSetId > 0 && region->m_SourceSetIds.contains(nodeSetId)) ||
                       (elementSetId > 0 && region->m_SourceSetIds.contains(elementSetId))))
            ++count;
    }
    return count;
}

class CombineCollectionsDialog final : public QDialog
{
public:
    CombineCollectionsDialog(const std::vector<LogicalCollection>& collections, QWidget* parent)
        : QDialog(parent)
    {
        setWindowTitle(QStringLiteral("新建组合集合"));
        setWindowIcon(UiIcons::applicationIcon());
        setMinimumSize(500, 360);
        resize(620, 440);
        DialogSizing::lockCurrentHeight(this);

        auto* root = new QVBoxLayout(this);
        root->setContentsMargins(16, 16, 16, 16);
        root->setSpacing(12);
        auto* hint = new QLabel(
            QStringLiteral(
                "选择一个或多个已有线路集合，程序自动合并其节点和单元。集合成员由模型对象维护，不需要输入 ID。"),
            this);
        hint->setWordWrap(true);
        hint->setProperty("secondaryText", true);
        root->addWidget(hint);

        auto* form = new QFormLayout;
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        m_name = new QLineEdit(this);
        m_name->setPlaceholderText(QStringLiteral("例如：耐张段 1 两根地线"));
        form->addRow(QStringLiteral("集合名称"), m_name);
        root->addLayout(form);

        m_sources = new QListWidget(this);
        m_sources->setSpacing(2);
        m_sources->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        for (const auto& collection : collections)
        {
            if (collection.nodeSetId <= 0 || collection.elementSetId <= 0 || collection.elementIds.empty())
                continue;
            auto* item = new QListWidgetItem(QStringLiteral("%1（节点 %2，单元 %3）")
                                                 .arg(collection.name)
                                                 .arg(collection.nodeIds.size())
                                                 .arg(collection.elementIds.size()),
                                             m_sources);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(Qt::Unchecked);
            item->setData(Qt::UserRole, collection.nodeSetId);
            item->setData(Qt::UserRole + 1, collection.elementSetId);
        }
        root->addWidget(m_sources, 1);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("创建"));
        buttons->button(QDialogButtonBox::Ok)->setDefault(true);
        buttons->button(QDialogButtonBox::Ok)->setProperty("primaryAction", true);
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
        connect(buttons, &QDialogButtonBox::accepted, this,
                [this]()
                {
                    if (name().isEmpty())
                    {
                        QMessageBox::information(this, QStringLiteral("缺少名称"), QStringLiteral("请填写集合名称。"));
                        m_name->setFocus();
                        return;
                    }
                    if (sourceSetIds().empty())
                    {
                        QMessageBox::information(this, QStringLiteral("未选择来源"),
                                                 QStringLiteral("请至少选择一个已有线路集合。"));
                        m_sources->setFocus();
                        return;
                    }
                    accept();
                });
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        root->addWidget(buttons);
        m_name->setFocus();
    }

    QString name() const
    {
        return m_name->text().trimmed();
    }

    std::set<int> sourceSetIds() const
    {
        std::set<int> result;
        for (int row = 0; row < m_sources->count(); ++row)
        {
            const auto* item = m_sources->item(row);
            if (item->checkState() != Qt::Checked)
                continue;
            result.insert(item->data(Qt::UserRole).toInt());
            result.insert(item->data(Qt::UserRole + 1).toInt());
        }
        return result;
    }

private:
    QLineEdit* m_name = nullptr;
    QListWidget* m_sources = nullptr;
};

} // namespace

ModelSetManagerDialog::ModelSetManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent)
    : QDialog(parent)
    , m_structure(structure)
{
    setWindowTitle(QStringLiteral("集合管理器"));
    setWindowIcon(UiIcons::applicationIcon());
    setObjectName(QStringLiteral("modelSetManagerDialog"));
    setMinimumSize(720, 380);
    resize(880, 520);
    DialogSizing::lockCurrentHeight(this);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(12);
    auto* hint = new QLabel(QStringLiteral("一个逻辑集合由节点集和单元集成对组成。FEM "
                                           "导入的相线、地线集合可直接使用；也可以将多个已有集合组合成新的业务集合。"),
                            this);
    hint->setWordWrap(true);
    hint->setProperty("secondaryText", true);
    root->addWidget(hint);

    m_table = new QTableWidget(this);
    m_table->setColumnCount(6);
    m_table->setHorizontalHeaderLabels({QStringLiteral("集合名称"), QStringLiteral("完整性"), QStringLiteral("节点数"),
                                        QStringLiteral("单元数"), QStringLiteral("引用区域"), QStringLiteral("来源")});
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->setShowGrid(false);
    m_table->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    m_table->verticalHeader()->setVisible(false);
    m_table->verticalHeader()->setDefaultSectionSize(36);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    root->addWidget(m_table, 1);

    auto* actions = new QHBoxLayout;
    auto* addButton = new QPushButton(QStringLiteral("组合新集合"), this);
    auto* renameButton = new QPushButton(QStringLiteral("重命名"), this);
    auto* deleteButton = new QPushButton(QStringLiteral("删除"), this);
    auto* closeButton = new QPushButton(QStringLiteral("关闭"), this);
    actions->setSpacing(8);
    addButton->setProperty("primaryAction", true);
    deleteButton->setProperty("dangerAction", true);
    renameButton->setEnabled(false);
    deleteButton->setEnabled(false);
    for (QPushButton* button : {addButton, renameButton, deleteButton, closeButton})
        button->setAutoDefault(false);
    actions->addWidget(addButton);
    actions->addWidget(renameButton);
    actions->addWidget(deleteButton);
    actions->addStretch(1);
    actions->addWidget(closeButton);
    root->addLayout(actions);

    connect(addButton, &QPushButton::clicked, this, &ModelSetManagerDialog::createCombinedCollection);
    connect(renameButton, &QPushButton::clicked, this, &ModelSetManagerDialog::renameSelectedCollection);
    connect(deleteButton, &QPushButton::clicked, this, &ModelSetManagerDialog::deleteSelectedCollection);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(m_table, &QTableWidget::itemSelectionChanged, this,
            [this, renameButton, deleteButton]()
            {
                const bool selected = selectedNodeSetId() > 0 || selectedElementSetId() > 0;
                renameButton->setEnabled(selected);
                deleteButton->setEnabled(selected);
            });
    connect(m_table, &QTableWidget::cellDoubleClicked, this,
            [this](int, int)
            {
                renameSelectedCollection();
            });
    refreshTable();
}

void ModelSetManagerDialog::refreshTable(const QString& preferredName)
{
    m_table->setRowCount(0);
    if (!m_structure)
        return;

    for (const auto& collection : collectLogicalCollections(*m_structure))
    {
        const int row = m_table->rowCount();
        m_table->insertRow(row);
        auto* nameItem = new QTableWidgetItem(collection.name);
        nameItem->setData(Qt::UserRole, collection.nodeSetId);
        nameItem->setData(Qt::UserRole + 1, collection.elementSetId);
        m_table->setItem(row, 0, nameItem);
        const bool complete = collection.nodeSetId > 0 && collection.elementSetId > 0 && !collection.elementIds.empty();
        m_table->setItem(row, 1,
                         new QTableWidgetItem(complete ? QStringLiteral("可用于计算区域") : QStringLiteral("不完整")));
        m_table->setItem(row, 2, new QTableWidgetItem(QString::number(collection.nodeIds.size())));
        m_table->setItem(row, 3, new QTableWidgetItem(QString::number(collection.elementIds.size())));
        m_table->setItem(row, 4,
                         new QTableWidgetItem(QString::number(
                             regionReferenceCount(*m_structure, collection.nodeSetId, collection.elementSetId))));
        const bool importedLine =
            collection.name.contains(QStringLiteral("_相线_")) || collection.name.contains(QStringLiteral("_地线_"));
        m_table->setItem(
            row, 5,
            new QTableWidgetItem(importedLine ? QStringLiteral("FEM 线路对象") : QStringLiteral("用户组合/模型")));
        for (int column = 2; column <= 4; ++column)
            m_table->item(row, column)->setTextAlignment(Qt::AlignCenter);
        for (int column = 0; column < m_table->columnCount(); ++column)
            m_table->item(row, column)->setToolTip(m_table->item(row, column)->text());
        if (!preferredName.isEmpty() && collection.name == preferredName)
            m_table->selectRow(row);
    }
    if (m_table->currentRow() < 0 && m_table->rowCount() > 0)
        m_table->selectRow(0);
}

void ModelSetManagerDialog::createCombinedCollection()
{
    if (!m_structure)
        return;
    const auto collections = collectLogicalCollections(*m_structure);
    CombineCollectionsDialog editor(collections, this);
    if (editor.exec() != QDialog::Accepted)
        return;

    for (const auto& collection : collections)
    {
        if (collection.name.compare(editor.name(), Qt::CaseInsensitive) == 0)
        {
            QMessageBox::information(this, QStringLiteral("名称重复"), QStringLiteral("已存在同名集合。"));
            return;
        }
    }

    std::set<int> nodeIds;
    std::set<int> elementIds;
    for (int setId : editor.sourceSetIds())
    {
        const auto found = m_structure->m_ModelSets.find(setId);
        if (found == m_structure->m_ModelSets.end() || !found->second)
            continue;
        if (found->second->m_Type == ModelSetType::Node)
            nodeIds.insert(found->second->m_Ids.begin(), found->second->m_Ids.end());
        else
            elementIds.insert(found->second->m_Ids.begin(), found->second->m_Ids.end());
    }
    // 单元集才是计算范围的核心。节点集即使来自旧文件且不完整，也按单元连接关系自动补齐。
    for (int elementId : elementIds)
    {
        const auto elementIt = m_structure->m_Elements.find(elementId);
        if (elementIt == m_structure->m_Elements.cend() || !elementIt->second)
            continue;
        for (const auto& nodeRef : elementIt->second->m_pNode)
        {
            if (const auto node = nodeRef.lock())
                nodeIds.insert(node->m_Id);
        }
    }
    if (nodeIds.empty() || elementIds.empty())
    {
        QMessageBox::critical(this, QStringLiteral("无法创建集合"),
                              QStringLiteral("所选来源没有形成完整的节点和单元集合。"));
        return;
    }

    QString error;
    const int nodeSetId =
        m_structure->AddModelSet(editor.name() + QStringLiteral(" · 节点"), ModelSetType::Node, nodeIds, &error);
    if (nodeSetId <= 0)
    {
        QMessageBox::critical(this, QStringLiteral("无法创建集合"), error);
        return;
    }
    const int elementSetId =
        m_structure->AddModelSet(editor.name() + QStringLiteral(" · 单元"), ModelSetType::Element, elementIds, &error);
    if (elementSetId <= 0)
    {
        m_structure->m_ModelSets.erase(nodeSetId);
        QMessageBox::critical(this, QStringLiteral("无法创建集合"), error);
        return;
    }
    refreshTable(editor.name());
}

void ModelSetManagerDialog::renameSelectedCollection()
{
    if (!m_structure)
        return;
    const int nodeSetId = selectedNodeSetId();
    const int elementSetId = selectedElementSetId();
    if (nodeSetId <= 0 && elementSetId <= 0)
        return;

    QDialog editor(this);
    editor.setWindowTitle(QStringLiteral("重命名集合"));
    editor.setWindowIcon(UiIcons::applicationIcon());
    editor.setMinimumWidth(440);
    auto* layout = new QVBoxLayout(&editor);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(16);
    auto* form = new QFormLayout;
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    auto* nameEdit = new QLineEdit(selectedCollectionName(), &editor);
    nameEdit->selectAll();
    nameEdit->setFocus();
    form->addRow(QStringLiteral("新名称"), nameEdit);
    layout->addLayout(form);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &editor);
    buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
    buttons->button(QDialogButtonBox::Ok)->setDefault(true);
    buttons->button(QDialogButtonBox::Ok)->setProperty("primaryAction", true);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    layout->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &editor, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &editor, &QDialog::reject);
    DialogSizing::fitCompactFormToScreen(&editor);
    if (editor.exec() != QDialog::Accepted)
        return;

    const QString newName = nameEdit->text().trimmed();
    if (newName.isEmpty())
        return;
    for (const auto& collection : collectLogicalCollections(*m_structure))
    {
        if (collection.name.compare(newName, Qt::CaseInsensitive) == 0 &&
            collection.name.compare(selectedCollectionName(), Qt::CaseSensitive) != 0)
        {
            QMessageBox::information(this, QStringLiteral("名称重复"), QStringLiteral("已存在同名集合。"));
            return;
        }
    }
    if (nodeSetId > 0)
        m_structure->m_ModelSets.at(nodeSetId)->m_Name = newName + QStringLiteral(" · 节点");
    if (elementSetId > 0)
        m_structure->m_ModelSets.at(elementSetId)->m_Name = newName + QStringLiteral(" · 单元");
    refreshTable(newName);
}

void ModelSetManagerDialog::deleteSelectedCollection()
{
    if (!m_structure)
        return;
    const int nodeSetId = selectedNodeSetId();
    const int elementSetId = selectedElementSetId();
    if (nodeSetId <= 0 && elementSetId <= 0)
        return;
    const int references = regionReferenceCount(*m_structure, nodeSetId, elementSetId);
    if (references > 0)
    {
        QMessageBox::information(
            this, QStringLiteral("集合正在使用"),
            QStringLiteral("该集合仍被 %1 个计算区域引用。请先删除对应计算区域。").arg(references));
        return;
    }
    if (QMessageBox::question(
            this, QStringLiteral("删除集合"),
            QStringLiteral("确定删除“%1”的节点集和单元集吗？\n模型实体不会被删除。").arg(selectedCollectionName()),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;
    if (nodeSetId > 0)
        m_structure->m_ModelSets.erase(nodeSetId);
    if (elementSetId > 0)
        m_structure->m_ModelSets.erase(elementSetId);
    refreshTable();
}

int ModelSetManagerDialog::selectedNodeSetId() const
{
    const int row = m_table ? m_table->currentRow() : -1;
    return row >= 0 && m_table->item(row, 0) ? m_table->item(row, 0)->data(Qt::UserRole).toInt() : 0;
}

int ModelSetManagerDialog::selectedElementSetId() const
{
    const int row = m_table ? m_table->currentRow() : -1;
    return row >= 0 && m_table->item(row, 0) ? m_table->item(row, 0)->data(Qt::UserRole + 1).toInt() : 0;
}

QString ModelSetManagerDialog::selectedCollectionName() const
{
    const int row = m_table ? m_table->currentRow() : -1;
    return row >= 0 && m_table->item(row, 0) ? m_table->item(row, 0)->text() : QString{};
}
