#include "ModelSetManagerDialog.h"

#include "DataStructure/Structure/StructureData.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QTableWidget>
#include <QVBoxLayout>

static bool ParseSetIds(const QString& text, std::set<int>& ids, QString& errorMessage)
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
    if (ids.empty())
    {
        errorMessage = QStringLiteral("集合不能是空集合。");
        return false;
    }
    return true;
}

static QString FormatSetIds(const std::set<int>& ids)
{
    QStringList values;
    for (int id : ids)
    {
        values.append(QString::number(id));
    }
    return values.join(QStringLiteral(", "));
}

static bool ValidateSetIds(const StructureData& structure, ModelSetType type, const std::set<int>& ids,
                           QString& errorMessage)
{
    for (int id : ids)
    {
        const bool exists = type == ModelSetType::Node ? structure.m_Nodes.find(id) != structure.m_Nodes.cend()
                                                       : structure.m_Elements.find(id) != structure.m_Elements.cend();
        if (!exists)
        {
            errorMessage = QStringLiteral("集合引用了不存在的%1 ID %2。")
                               .arg(type == ModelSetType::Node ? QStringLiteral("节点") : QStringLiteral("单元"))
                               .arg(id);
            return false;
        }
    }
    return true;
}

class ModelSetEditorDialog final : public QDialog
{
public:
    explicit ModelSetEditorDialog(const std::shared_ptr<ModelSet>& modelSet, QWidget* parent)
        : QDialog(parent)
    {
        setWindowTitle(modelSet ? QStringLiteral("编辑集合") : QStringLiteral("新增集合"));
        setMinimumWidth(520);

        auto* root = new QVBoxLayout(this);
        auto* form = new QFormLayout();
        m_name = new QLineEdit(this);
        m_type = new QComboBox(this);
        m_type->addItem(QStringLiteral("节点集合"), static_cast<int>(ModelSetType::Node));
        m_type->addItem(QStringLiteral("单元集合"), static_cast<int>(ModelSetType::Element));
        m_ids = new QPlainTextEdit(this);
        m_ids->setPlaceholderText(QStringLiteral("支持逗号、空格、分号和范围，例如：1, 2, 10-20"));
        m_ids->setMinimumHeight(120);
        form->addRow(QStringLiteral("名称"), m_name);
        form->addRow(QStringLiteral("类型"), m_type);
        form->addRow(QStringLiteral("ID"), m_ids);
        root->addLayout(form);
        root->addWidget(
            new QLabel(QStringLiteral("修改被计算区域引用的集合后，相关计算区域将自动重新构建和合并。"), this));

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        root->addWidget(buttons);
        connect(buttons, &QDialogButtonBox::accepted, this,
                [this]()
                {
                    QString error;
                    if (!ParseSetIds(m_ids->toPlainText(), m_parsedIds, error))
                    {
                        QMessageBox::information(this, QStringLiteral("ID 格式错误"), error);
                        return;
                    }
                    accept();
                });
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

        if (modelSet)
        {
            m_name->setText(modelSet->m_Name);
            m_type->setCurrentIndex(modelSet->m_Type == ModelSetType::Node ? 0 : 1);
            m_ids->setPlainText(FormatSetIds(modelSet->m_Ids));
        }
    }

    QString name() const
    {
        return m_name->text().trimmed();
    }

    ModelSetType type() const
    {
        return static_cast<ModelSetType>(m_type->currentData().toInt());
    }

    const std::set<int>& ids() const
    {
        return m_parsedIds;
    }

private:
    QLineEdit* m_name = nullptr;
    QComboBox* m_type = nullptr;
    QPlainTextEdit* m_ids = nullptr;
    std::set<int> m_parsedIds;
};

ModelSetManagerDialog::ModelSetManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent)
    : QDialog(parent)
    , m_structure(structure)
{
    setWindowTitle(QStringLiteral("集合管理器"));
    resize(760, 440);

    auto* root = new QVBoxLayout(this);
    m_table = new QTableWidget(this);
    m_table->setColumnCount(5);
    m_table->setHorizontalHeaderLabels({QStringLiteral("ID"), QStringLiteral("名称"), QStringLiteral("类型"),
                                        QStringLiteral("成员数"), QStringLiteral("引用区域数")});
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->verticalHeader()->setVisible(false);
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    root->addWidget(m_table);

    auto* actions = new QHBoxLayout();
    auto* addButton = new QPushButton(QStringLiteral("新增"), this);
    auto* editButton = new QPushButton(QStringLiteral("编辑"), this);
    auto* deleteButton = new QPushButton(QStringLiteral("删除"), this);
    auto* closeButton = new QPushButton(QStringLiteral("关闭"), this);
    actions->addWidget(addButton);
    actions->addWidget(editButton);
    actions->addWidget(deleteButton);
    actions->addStretch();
    actions->addWidget(closeButton);
    root->addLayout(actions);

    connect(addButton, &QPushButton::clicked, this,
            [this]()
            {
                editSet();
            });
    connect(editButton, &QPushButton::clicked, this,
            [this]()
            {
                editSet(selectedSetId());
            });
    connect(deleteButton, &QPushButton::clicked, this, &ModelSetManagerDialog::deleteSelectedSet);
    connect(closeButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(m_table, &QTableWidget::cellDoubleClicked, this,
            [this](int, int)
            {
                editSet(selectedSetId());
            });
    refreshTable();
}

void ModelSetManagerDialog::refreshTable(int preferredSetId)
{
    m_table->setRowCount(0);
    if (!m_structure)
    {
        return;
    }

    for (const auto& [setId, modelSet] : m_structure->m_ModelSets)
    {
        if (!modelSet)
        {
            continue;
        }
        int referenceCount = 0;
        for (const auto& [regionId, region] : m_structure->m_ComputeRegions)
        {
            Q_UNUSED(regionId);
            if (region && region->m_SourceSetIds.find(setId) != region->m_SourceSetIds.cend())
            {
                ++referenceCount;
            }
        }

        const int row = m_table->rowCount();
        m_table->insertRow(row);
        auto* idItem = new QTableWidgetItem(QString::number(setId));
        idItem->setData(Qt::UserRole, setId);
        m_table->setItem(row, 0, idItem);
        m_table->setItem(row, 1, new QTableWidgetItem(modelSet->m_Name));
        m_table->setItem(row, 2,
                         new QTableWidgetItem(modelSet->m_Type == ModelSetType::Node ? QStringLiteral("节点")
                                                                                     : QStringLiteral("单元")));
        m_table->setItem(row, 3, new QTableWidgetItem(QString::number(modelSet->m_Ids.size())));
        m_table->setItem(row, 4, new QTableWidgetItem(QString::number(referenceCount)));
        if (setId == preferredSetId)
        {
            m_table->selectRow(row);
        }
    }
    if (m_table->currentRow() < 0 && m_table->rowCount() > 0)
    {
        m_table->selectRow(0);
    }
}

void ModelSetManagerDialog::editSet(int setId)
{
    if (!m_structure)
    {
        return;
    }
    std::shared_ptr<ModelSet> existing;
    const auto existingIt = m_structure->m_ModelSets.find(setId);
    if (existingIt != m_structure->m_ModelSets.cend())
    {
        existing = existingIt->second;
    }

    ModelSetEditorDialog editor(existing, this);
    if (editor.exec() != QDialog::Accepted)
    {
        return;
    }

    QString error;
    if (!ValidateSetIds(*m_structure, editor.type(), editor.ids(), error))
    {
        QMessageBox::critical(this, QStringLiteral("无法保存集合"), error);
        return;
    }

    int selectedId = setId;
    if (!existing)
    {
        selectedId = m_structure->AddModelSet(editor.name(), editor.type(), editor.ids(), &error);
        if (selectedId <= 0)
        {
            QMessageBox::critical(this, QStringLiteral("无法创建集合"), error);
            return;
        }
    }
    else
    {
        const ModelSet setBackup = *existing;
        std::map<int, std::shared_ptr<ComputeRegion>> regionBackup;
        for (const auto& [regionId, region] : m_structure->m_ComputeRegions)
        {
            regionBackup.emplace(regionId, region ? std::make_shared<ComputeRegion>(*region) : nullptr);
        }
        std::map<int, std::set<int>> stepRegionBackup;
        for (const auto& [stepId, step] : m_structure->m_AnalysisStep)
        {
            if (step)
            {
                stepRegionBackup.emplace(stepId, step->m_ComputeRegionIds);
            }
        }

        existing->m_Name = editor.name().isEmpty() ? setBackup.m_Name : editor.name();
        existing->m_Type = editor.type();
        existing->m_Ids = editor.ids();
        if (!m_structure->RebuildAndMergeComputeRegions(&error))
        {
            *existing = setBackup;
            m_structure->m_ComputeRegions = std::move(regionBackup);
            for (const auto& [stepId, regionIds] : stepRegionBackup)
            {
                const auto stepIt = m_structure->m_AnalysisStep.find(stepId);
                if (stepIt != m_structure->m_AnalysisStep.cend() && stepIt->second)
                {
                    stepIt->second->m_ComputeRegionIds = regionIds;
                }
            }
            QMessageBox::critical(this, QStringLiteral("无法修改集合"), error);
            return;
        }
    }
    refreshTable(selectedId);
}

void ModelSetManagerDialog::deleteSelectedSet()
{
    const int setId = selectedSetId();
    if (!m_structure || setId <= 0)
    {
        return;
    }
    for (const auto& [regionId, region] : m_structure->m_ComputeRegions)
    {
        Q_UNUSED(regionId);
        if (region && region->m_SourceSetIds.find(setId) != region->m_SourceSetIds.cend())
        {
            QMessageBox::information(
                this, QStringLiteral("无法删除集合"),
                QStringLiteral("该集合仍被计算区域“%1”引用，请先修改该计算区域。").arg(region->m_Name));
            return;
        }
    }
    if (QMessageBox::question(this, QStringLiteral("删除集合"), QStringLiteral("确定删除选中的集合吗？")) !=
        QMessageBox::Yes)
    {
        return;
    }
    m_structure->m_ModelSets.erase(setId);
    refreshTable();
}

int ModelSetManagerDialog::selectedSetId() const
{
    const int row = m_table ? m_table->currentRow() : -1;
    const auto* item = row >= 0 ? m_table->item(row, 0) : nullptr;
    return item ? item->data(Qt::UserRole).toInt() : -1;
}
