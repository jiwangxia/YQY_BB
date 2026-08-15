#include "ComputeRegionManagerDialog.h"
#include "ModelSetManagerDialog.h"

#include "DataStructure/Structure/StructureData.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QRegularExpression>
#include <QTableWidget>
#include <QTextEdit>
#include <QVBoxLayout>

static bool ParseIds(const QString& text, std::set<int>& ids, QString& errorMessage)
{
    ids.clear();
    const QStringList tokens = text.split(QRegularExpression(QStringLiteral("[,;\\s]+")), Qt::SkipEmptyParts);
    for (const QString& token : tokens)
    {
        const QStringList range = token.split(QLatin1Char('-'), Qt::SkipEmptyParts);
        bool firstOk = false;
        const int first = range.value(0).toInt(&firstOk);
        if (!firstOk || first <= 0 || range.size() > 2)
        {
            errorMessage = QStringLiteral("无法识别 ID：%1").arg(token);
            return false;
        }
        if (range.size() == 1)
        {
            ids.insert(first);
            continue;
        }
        bool lastOk = false;
        const int last = range.at(1).toInt(&lastOk);
        if (!lastOk || last < first)
        {
            errorMessage = QStringLiteral("无效 ID 范围：%1").arg(token);
            return false;
        }
        for (int id = first;; ++id)
        {
            ids.insert(id);
            if (id == last)
            {
                break;
            }
        }
    }
    return true;
}

static QString FormatIds(const std::set<int>& ids)
{
    QStringList values;
    for (int id : ids)
        values.append(QString::number(id));
    return values.join(QStringLiteral(", "));
}

class ComputeRegionEditorDialog final : public QDialog
{
public:
    ComputeRegionEditorDialog(const std::shared_ptr<StructureData>& structure,
                              const std::shared_ptr<ComputeRegion>& region, QWidget* parent)
        : QDialog(parent)
    {
        setWindowTitle(region ? QStringLiteral("编辑计算区域") : QStringLiteral("新增计算区域"));
        setMinimumWidth(520);
        auto* root = new QVBoxLayout(this);
        auto* form = new QFormLayout();
        m_name = new QLineEdit(this);
        m_enabled = new QCheckBox(QStringLiteral("参与计算"), this);
        m_nodeIds = new QTextEdit(this);
        m_elementIds = new QTextEdit(this);
        m_sourceSets = new QListWidget(this);
        m_nodeIds->setPlaceholderText(QStringLiteral("节点 ID，例如：1, 2, 10-20；可留空并由单元自动补齐"));
        m_elementIds->setPlaceholderText(QStringLiteral("单元 ID，例如：1, 2, 10-20"));
        m_nodeIds->setMaximumHeight(90);
        m_elementIds->setMaximumHeight(90);
        form->addRow(QStringLiteral("名称"), m_name);
        form->addRow(QStringLiteral("状态"), m_enabled);
        form->addRow(QStringLiteral("节点 ID"), m_nodeIds);
        form->addRow(QStringLiteral("单元 ID"), m_elementIds);
        form->addRow(QStringLiteral("来源集合"), m_sourceSets);
        root->addLayout(form);
        root->addWidget(new QLabel(QStringLiteral("共享节点或单元的区域将在保存后自动合并。"), this));
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        root->addWidget(buttons);
        connect(buttons, &QDialogButtonBox::accepted, this,
                [this]()
                {
                    QString error;
                    if (!ParseIds(m_nodeIds->toPlainText(), m_nodes, error) ||
                        !ParseIds(m_elementIds->toPlainText(), m_elements, error))
                    {
                        QMessageBox::information(this, QStringLiteral("ID 格式错误"), error);
                        return;
                    }
                    if (m_nodes.empty() && m_elements.empty() && sourceSetIds().empty())
                    {
                        QMessageBox::information(this, QStringLiteral("区域为空"),
                                                 QStringLiteral("请至少输入一个节点 ID 或单元 ID。"));
                        return;
                    }
                    accept();
                });
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

        m_enabled->setChecked(true);
        if (structure)
        {
            for (const auto& [setId, modelSet] : structure->m_ModelSets)
            {
                if (!modelSet)
                    continue;
                auto* item = new QListWidgetItem(QStringLiteral("%1（%2，%3 项）")
                                                     .arg(modelSet->m_Name, modelSet->m_Type == ModelSetType::Node
                                                                                ? QStringLiteral("节点集")
                                                                                : QStringLiteral("单元集"))
                                                     .arg(modelSet->m_Ids.size()),
                                                 m_sourceSets);
                item->setData(Qt::UserRole, setId);
                item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
                item->setCheckState(region && region->m_SourceSetIds.find(setId) != region->m_SourceSetIds.cend()
                                        ? Qt::Checked
                                        : Qt::Unchecked);
            }
        }
        if (region)
        {
            m_name->setText(region->m_Name);
            m_enabled->setChecked(region->m_Enabled);
            m_nodeIds->setPlainText(FormatIds(region->m_DirectNodeIds));
            m_elementIds->setPlainText(FormatIds(region->m_DirectElementIds));
        }
    }

    QString name() const
    {
        return m_name->text().trimmed();
    }

    bool enabled() const
    {
        return m_enabled->isChecked();
    }

    const std::set<int>& nodeIds() const
    {
        return m_nodes;
    }

    const std::set<int>& elementIds() const
    {
        return m_elements;
    }

    std::set<int> sourceSetIds() const
    {
        std::set<int> result;
        for (int row = 0; row < m_sourceSets->count(); ++row)
        {
            const auto* item = m_sourceSets->item(row);
            if (item->checkState() == Qt::Checked)
                result.insert(item->data(Qt::UserRole).toInt());
        }
        return result;
    }

private:
    QLineEdit* m_name = nullptr;
    QCheckBox* m_enabled = nullptr;
    QTextEdit* m_nodeIds = nullptr;
    QTextEdit* m_elementIds = nullptr;
    QListWidget* m_sourceSets = nullptr;
    std::set<int> m_nodes;
    std::set<int> m_elements;
};

ComputeRegionManagerDialog::ComputeRegionManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent)
    : QDialog(parent)
    , m_structure(structure)
{
    setWindowTitle(QStringLiteral("计算区域管理器"));
    resize(820, 460);
    auto* root = new QVBoxLayout(this);
    m_table = new QTableWidget(this);
    m_table->setColumnCount(5);
    m_table->setHorizontalHeaderLabels({QStringLiteral("名称"), QStringLiteral("启用"), QStringLiteral("节点数"),
                                        QStringLiteral("单元数"), QStringLiteral("来源集合")});
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->verticalHeader()->setVisible(false);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    root->addWidget(m_table);

    auto* actions = new QHBoxLayout();
    auto* addButton = new QPushButton(QStringLiteral("新增"), this);
    auto* editButton = new QPushButton(QStringLiteral("编辑"), this);
    auto* deleteButton = new QPushButton(QStringLiteral("删除"), this);
    auto* setsButton = new QPushButton(QStringLiteral("集合管理"), this);
    auto* closeButton = new QPushButton(QStringLiteral("关闭"), this);
    actions->addWidget(addButton);
    actions->addWidget(editButton);
    actions->addWidget(deleteButton);
    actions->addWidget(setsButton);
    actions->addStretch();
    actions->addWidget(closeButton);
    root->addLayout(actions);

    connect(addButton, &QPushButton::clicked, this,
            [this]()
            {
                editRegion();
            });
    connect(editButton, &QPushButton::clicked, this,
            [this]()
            {
                editRegion(selectedRegionId());
            });
    connect(deleteButton, &QPushButton::clicked, this, &ComputeRegionManagerDialog::deleteSelectedRegion);
    connect(setsButton, &QPushButton::clicked, this, &ComputeRegionManagerDialog::manageSets);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(m_table, &QTableWidget::cellDoubleClicked, this,
            [this](int, int)
            {
                editRegion(selectedRegionId());
            });
    refreshTable();
}

void ComputeRegionManagerDialog::refreshTable(int preferredRegionId)
{
    m_table->setRowCount(0);
    if (!m_structure)
        return;
    for (const auto& [regionId, region] : m_structure->m_ComputeRegions)
    {
        if (!region)
            continue;
        const int row = m_table->rowCount();
        m_table->insertRow(row);
        auto* nameItem = new QTableWidgetItem(region->m_Name);
        nameItem->setData(Qt::UserRole, regionId);
        m_table->setItem(row, 0, nameItem);
        m_table->setItem(row, 1, new QTableWidgetItem(region->m_Enabled ? QStringLiteral("是") : QStringLiteral("否")));
        m_table->setItem(row, 2, new QTableWidgetItem(QString::number(region->m_NodeIds.size())));
        m_table->setItem(row, 3, new QTableWidgetItem(QString::number(region->m_ElementIds.size())));
        m_table->setItem(row, 4, new QTableWidgetItem(QString::number(region->m_SourceSetIds.size())));
        if (regionId == preferredRegionId)
            m_table->selectRow(row);
    }
    if (m_table->currentRow() < 0 && m_table->rowCount() > 0)
        m_table->selectRow(0);
}

void ComputeRegionManagerDialog::editRegion(int regionId)
{
    if (!m_structure)
        return;
    std::shared_ptr<ComputeRegion> existing;
    const auto existingIt = m_structure->m_ComputeRegions.find(regionId);
    if (existingIt != m_structure->m_ComputeRegions.end())
        existing = existingIt->second;

    ComputeRegionEditorDialog editor(m_structure, existing, this);
    if (editor.exec() != QDialog::Accepted)
        return;

    QString error;
    int selectedId = regionId;
    if (!existing)
    {
        selectedId = m_structure->AddComputeRegion(editor.name(), editor.nodeIds(), editor.elementIds(),
                                                   editor.sourceSetIds(), editor.enabled(), &error);
        if (selectedId <= 0)
        {
            QMessageBox::critical(this, QStringLiteral("无法创建计算区域"), error);
            return;
        }
    }
    else
    {
        const ComputeRegion backup = *existing;
        existing->m_Name = editor.name().isEmpty() ? backup.m_Name : editor.name();
        existing->m_Enabled = editor.enabled();
        existing->m_AutoGenerated = false;
        existing->m_DirectNodeIds = editor.nodeIds();
        existing->m_DirectElementIds = editor.elementIds();
        existing->m_SourceSetIds = editor.sourceSetIds();
        if (!m_structure->RebuildAndMergeComputeRegions(&error))
        {
            *existing = backup;
            QMessageBox::critical(this, QStringLiteral("无法修改计算区域"), error);
            return;
        }
    }
    refreshTable(selectedId);
}

void ComputeRegionManagerDialog::deleteSelectedRegion()
{
    const int regionId = selectedRegionId();
    if (!m_structure || regionId <= 0)
        return;
    if (QMessageBox::question(this, QStringLiteral("删除计算区域"),
                              QStringLiteral("确定删除选中的计算区域吗？分析步中的对应引用也会删除。")) !=
        QMessageBox::Yes)
    {
        return;
    }
    m_structure->RemoveComputeRegion(regionId);
    refreshTable();
}

void ComputeRegionManagerDialog::manageSets()
{
    if (!m_structure)
    {
        return;
    }
    ModelSetManagerDialog manager(m_structure, this);
    manager.exec();
    refreshTable();
}

int ComputeRegionManagerDialog::selectedRegionId() const
{
    const int row = m_table ? m_table->currentRow() : -1;
    return row >= 0 && m_table->item(row, 0) ? m_table->item(row, 0)->data(Qt::UserRole).toInt() : -1;
}
