#include "ComputeRegionManagerDialog.h"
#include "Widgets/DialogSizing.h"
#include "Utility/UiIcons.h"

#include "ModelSetManagerDialog.h"

#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Region/ModelSet.h"
#include "DataStructure/Structure/StructureData.h"

#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
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

struct TopologyCollection
{
    QString name;
    int nodeSetId = 0;
    int elementSetId = 0;
    int nodeCount = 0;
    int elementCount = 0;
};

QString collectionName(const QString& setName, ModelSetType type)
{
    const QString suffix = type == ModelSetType::Node ? QStringLiteral(" · 节点") : QStringLiteral(" · 单元");
    return setName.endsWith(suffix) ? setName.left(setName.size() - suffix.size()).trimmed() : setName.trimmed();
}

std::vector<TopologyCollection> buildTopologyCollections(const StructureData& structure)
{
    std::map<QString, TopologyCollection> grouped;
    for (const auto& [setId, set] : structure.m_ModelSets)
    {
        if (!set)
            continue;

        const QString name = collectionName(set->m_Name, set->m_Type);
        if (name.isEmpty())
            continue;

        auto& collection = grouped[name];
        collection.name = name;
        if (set->m_Type == ModelSetType::Node)
        {
            collection.nodeSetId = setId;
            collection.nodeCount = static_cast<int>(set->m_Ids.size());
        }
        else
        {
            collection.elementSetId = setId;
            collection.elementCount = static_cast<int>(set->m_Ids.size());
        }
    }

    std::vector<TopologyCollection> result;
    for (const auto& [name, collection] : grouped)
    {
        Q_UNUSED(name);
        // 计算集合必须包含单元；仅含节点的集合不能独立组成计算区域，因此不在此显示。
        if (collection.elementSetId > 0 && collection.elementCount > 0)
            result.push_back(collection);
    }
    return result;
}

int convertedFemTrussCount(const StructureData& structure)
{
    int count = 0;
    for (const auto& [elementId, element] : structure.m_Elements)
    {
        Q_UNUSED(elementId);
        const auto truss = std::dynamic_pointer_cast<ElementTruss>(element);
        if (truss && truss->m_UseEngineeringStrain)
            ++count;
    }
    return count;
}

class RegionFromCollectionsDialog final : public QDialog
{
public:
    RegionFromCollectionsDialog(const std::vector<TopologyCollection>& collections, QWidget* parent)
        : QDialog(parent)
    {
        setWindowTitle(QStringLiteral("由集合新建计算区域"));
        setWindowIcon(UiIcons::applicationIcon());
        setMinimumSize(500, 360);
        resize(620, 460);
        DialogSizing::lockCurrentHeight(this);

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(16, 16, 16, 16);
        layout->setSpacing(12);
        auto* intro = new QLabel(QStringLiteral("勾选导入 H5 "
                                                "中的线路对象即可组成计算区域。程序会自动展开节点、单元、端点约束和关联"
                                                "关系；不需要输入节点或单元 ID。"),
                                 this);
        intro->setWordWrap(true);
        intro->setProperty("secondaryText", true);
        layout->addWidget(intro);

        auto* form = new QFormLayout;
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        m_nameEdit = new QLineEdit(this);
        m_nameEdit->setPlaceholderText(QStringLiteral("例如：耐张段 1 快速计算区域"));
        form->addRow(QStringLiteral("区域名称"), m_nameEdit);
        layout->addLayout(form);

        layout->addWidget(new QLabel(QStringLiteral("选择线路对象"), this));
        m_collectionList = new QListWidget(this);
        m_collectionList->setSpacing(2);
        m_collectionList->setSelectionMode(QAbstractItemView::SingleSelection);
        m_collectionList->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        for (const auto& collection : collections)
        {
            auto* item = new QListWidgetItem(QStringLiteral("%1（节点 %2，单元 %3）")
                                                 .arg(collection.name)
                                                 .arg(collection.nodeCount)
                                                 .arg(collection.elementCount),
                                             m_collectionList);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(Qt::Unchecked);
            item->setData(Qt::UserRole, collection.nodeSetId);
            item->setData(Qt::UserRole + 1, collection.elementSetId);
        }
        layout->addWidget(m_collectionList, 1);

        auto* note = new QLabel(QStringLiteral("相连的线路对象会自动合并为一个完整的物理计算区域。删除区域只删除计算配"
                                               "置，不会删除模型、节点、单元或集合。"),
                                this);
        note->setWordWrap(true);
        note->setProperty("secondaryText", true);
        layout->addWidget(note);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("创建"));
        buttons->button(QDialogButtonBox::Ok)->setDefault(true);
        buttons->button(QDialogButtonBox::Ok)->setProperty("primaryAction", true);
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
        connect(buttons, &QDialogButtonBox::accepted, this,
                [this]()
                {
                    if (m_nameEdit->text().trimmed().isEmpty())
                    {
                        QMessageBox::warning(this, QStringLiteral("缺少名称"), QStringLiteral("请填写计算区域名称。"));
                        m_nameEdit->setFocus();
                        return;
                    }
                    if (sourceSetIds().empty())
                    {
                        QMessageBox::warning(this, QStringLiteral("未选择集合"),
                                             QStringLiteral("请至少选择一个线路对象。"));
                        m_collectionList->setFocus();
                        return;
                    }
                    accept();
                });
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        layout->addWidget(buttons);
        m_nameEdit->setFocus();
    }

    QString regionName() const
    {
        return m_nameEdit->text().trimmed();
    }

    std::set<int> sourceSetIds() const
    {
        std::set<int> result;
        for (int row = 0; row < m_collectionList->count(); ++row)
        {
            const auto* item = m_collectionList->item(row);
            if (item->checkState() != Qt::Checked)
                continue;
            const int nodeSetId = item->data(Qt::UserRole).toInt();
            const int elementSetId = item->data(Qt::UserRole + 1).toInt();
            if (nodeSetId > 0)
                result.insert(nodeSetId);
            if (elementSetId > 0)
                result.insert(elementSetId);
        }
        return result;
    }

private:
    QLineEdit* m_nameEdit = nullptr;
    QListWidget* m_collectionList = nullptr;
};

} // namespace

ComputeRegionManagerDialog::ComputeRegionManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent)
    : QDialog(parent)
    , m_structure(structure)
{
    setWindowTitle(QStringLiteral("线路计算区域"));
    setWindowIcon(UiIcons::applicationIcon());
    setObjectName(QStringLiteral("computeRegionManagerDialog"));
    setMinimumSize(660, 380);
    resize(840, 520);
    DialogSizing::lockCurrentHeight(this);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(16, 16, 16, 16);
    layout->setSpacing(12);
    auto* hint = new QLabel(QStringLiteral("导入 FEM H5 "
                                           "后，耐张段、相线、地线等线路对象会成为可选集合。由一个或多个集合创建计算区"
                                           "域；具体计算哪几个区域，请在“分析步 → 计算区域”中勾选。"),
                            this);
    hint->setWordWrap(true);
    hint->setProperty("secondaryText", true);
    layout->addWidget(hint);

    const int convertedCount = m_structure ? convertedFemTrussCount(*m_structure) : 0;
    auto* stressStatus = new QLabel(this);
    if (convertedCount > 0)
    {
        stressStatus->setText(
            QStringLiteral(
                "预张力状态：已识别 %1 个 FEM 预张力桁架。初始张力已转换为无应力长度参与求解，因此“附加初始应力”显示为 "
                "0 不代表没有导入。")
                .arg(convertedCount));
    }
    else
    {
        stressStatus->setText(
            QStringLiteral("预张力状态：当前模型未识别到 FEM 预张力桁架；请用新版 FEM 导出的 H5 重新导入后再检查。"));
    }
    stressStatus->setWordWrap(true);
    layout->addWidget(stressStatus);

    m_table = new QTableWidget(this);
    m_table->setColumnCount(5);
    m_table->setHorizontalHeaderLabels({QStringLiteral("参与"), QStringLiteral("计算区域"), QStringLiteral("节点数"),
                                        QStringLiteral("单元数"), QStringLiteral("来源集合")});
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setAlternatingRowColors(true);
    m_table->setShowGrid(false);
    m_table->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    m_table->verticalHeader()->setVisible(false);
    m_table->verticalHeader()->setDefaultSectionSize(36);
    layout->addWidget(m_table, 1);

    auto* actions = new QHBoxLayout;
    auto* collectionsButton = new QPushButton(QStringLiteral("集合管理器"), this);
    auto* createButton = new QPushButton(QStringLiteral("由集合新建区域"), this);
    auto* deleteButton = new QPushButton(QStringLiteral("删除区域"), this);
    createButton->setProperty("primaryAction", true);
    deleteButton->setProperty("dangerAction", true);
    deleteButton->setEnabled(false);
    actions->setSpacing(8);
    actions->addWidget(createButton);
    actions->addWidget(collectionsButton);
    actions->addWidget(deleteButton);
    actions->addStretch(1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    buttons->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
    for (QPushButton* button : {collectionsButton, createButton, deleteButton, buttons->button(QDialogButtonBox::Close)})
        button->setAutoDefault(false);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    actions->addWidget(buttons);
    layout->addLayout(actions);

    connect(collectionsButton, &QPushButton::clicked, this,
            [this]()
            {
                ModelSetManagerDialog manager(m_structure, this);
                manager.exec();
            });
    connect(createButton, &QPushButton::clicked, this, &ComputeRegionManagerDialog::createRegionFromCollections);
    connect(deleteButton, &QPushButton::clicked, this, &ComputeRegionManagerDialog::deleteSelectedRegion);
    connect(m_table, &QTableWidget::itemSelectionChanged, this,
            [this, deleteButton]()
            {
                deleteButton->setEnabled(selectedRegionId() > 0);
            });
    connect(m_table, &QTableWidget::itemChanged, this,
            [this](QTableWidgetItem* item)
            {
                if (m_refreshing || !m_structure || item->column() != 0)
                    return;
                const int regionId = item->data(Qt::UserRole).toInt();
                const auto it = m_structure->m_ComputeRegions.find(regionId);
                if (it == m_structure->m_ComputeRegions.end() || !it->second)
                    return;

                const bool enabled = item->checkState() == Qt::Checked;
                it->second->m_Enabled = enabled;
                for (auto& [stepId, step] : m_structure->m_AnalysisStep)
                {
                    Q_UNUSED(stepId);
                    if (!step)
                        continue;
                    if (enabled)
                    {
                        // “参与”表示该区域重新加入现有分析配置；全部区域模式本来就会
                        // 自动包含它，显式选择模式则补上引用。
                        if (step->m_RegionScope == AnalysisRegionScope::SelectedRegions)
                            step->m_ComputeRegionIds.insert(regionId);
                        continue;
                    }

                    // 全部区域模式若直接保留，会在区域再次启用时悄悄重新勾选它。
                    // 转成显式列表并保存其余启用区域，使两个界面的状态始终一致。
                    if (step->m_RegionScope == AnalysisRegionScope::AllEnabledRegions)
                    {
                        step->m_RegionScope = AnalysisRegionScope::SelectedRegions;
                        step->m_ComputeRegionIds.clear();
                        for (const auto& [candidateId, candidate] : m_structure->m_ComputeRegions)
                        {
                            if (candidateId != regionId && candidate && candidate->m_Enabled)
                                step->m_ComputeRegionIds.insert(candidateId);
                        }
                    }
                    else
                    {
                        step->m_ComputeRegionIds.erase(regionId);
                    }
                }
                m_structure->SynchronizeComputeRegionEnablementFromSteps();
            });

    refreshTable();
}

void ComputeRegionManagerDialog::refreshTable(int preferredRegionId)
{
    m_refreshing = true;
    m_table->setRowCount(0);
    if (m_structure)
    {
        int row = 0;
        for (const auto& [regionId, region] : m_structure->m_ComputeRegions)
        {
            if (!region)
                continue;
            m_table->insertRow(row);
            auto* enabled = new QTableWidgetItem;
            enabled->setFlags(enabled->flags() | Qt::ItemIsUserCheckable);
            enabled->setCheckState(region->m_Enabled ? Qt::Checked : Qt::Unchecked);
            enabled->setData(Qt::UserRole, regionId);
            m_table->setItem(row, 0, enabled);
            m_table->setItem(row, 1, new QTableWidgetItem(region->m_Name));
            m_table->setItem(row, 2, new QTableWidgetItem(QString::number(region->m_NodeIds.size())));
            m_table->setItem(row, 3, new QTableWidgetItem(QString::number(region->m_ElementIds.size())));
            m_table->setItem(row, 4, new QTableWidgetItem(QString::number(region->m_SourceSetIds.size())));
            for (int column = 2; column < m_table->columnCount(); ++column)
                m_table->item(row, column)->setTextAlignment(Qt::AlignCenter);
            for (int column = 1; column < m_table->columnCount(); ++column)
                m_table->item(row, column)->setToolTip(m_table->item(row, column)->text());
            if (regionId == preferredRegionId)
                m_table->selectRow(row);
            ++row;
        }
    }
    if (m_table->currentRow() < 0 && m_table->rowCount() > 0)
        m_table->selectRow(0);
    m_refreshing = false;
}

int ComputeRegionManagerDialog::selectedRegionId() const
{
    const int row = m_table->currentRow();
    if (row < 0 || !m_table->item(row, 0))
        return 0;
    return m_table->item(row, 0)->data(Qt::UserRole).toInt();
}

void ComputeRegionManagerDialog::createRegionFromCollections()
{
    if (!m_structure)
        return;
    const auto collections = buildTopologyCollections(*m_structure);
    if (collections.empty())
    {
        QMessageBox::information(
            this, QStringLiteral("没有可用集合"),
            QStringLiteral("当前模型没有从 FEM H5 导入的线路集合。请用新版 FEM 导出 H5 后重新导入模型。"));
        return;
    }

    RegionFromCollectionsDialog editor(collections, this);
    if (editor.exec() != QDialog::Accepted)
        return;

    QString error;
    const int regionId =
        m_structure->AddComputeRegionFromSets(editor.regionName(), editor.sourceSetIds(), true, &error);
    if (regionId <= 0)
    {
        QMessageBox::critical(this, QStringLiteral("无法创建计算区域"),
                              error.isEmpty() ? QStringLiteral("所选集合无法形成有效的计算区域。") : error);
        return;
    }
    for (auto& [stepId, step] : m_structure->m_AnalysisStep)
    {
        Q_UNUSED(stepId);
        if (step && step->m_RegionScope == AnalysisRegionScope::SelectedRegions)
            step->m_ComputeRegionIds.insert(regionId);
    }
    m_structure->SynchronizeComputeRegionEnablementFromSteps();
    refreshTable(regionId);
}

void ComputeRegionManagerDialog::deleteSelectedRegion()
{
    if (!m_structure)
        return;
    const int regionId = selectedRegionId();
    const auto it = m_structure->m_ComputeRegions.find(regionId);
    if (regionId <= 0 || it == m_structure->m_ComputeRegions.end() || !it->second)
    {
        QMessageBox::information(this, QStringLiteral("未选择区域"), QStringLiteral("请先在表格中选择一个计算区域。"));
        return;
    }

    if (QMessageBox::question(
            this, QStringLiteral("删除计算区域"),
            QStringLiteral("确定删除“%1”吗？\n这只删除计算区域配置和分析步引用，不会删除模型、节点、单元或导入集合。")
                .arg(it->second->m_Name),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;

    m_structure->RemoveComputeRegion(regionId);
    refreshTable();
}
