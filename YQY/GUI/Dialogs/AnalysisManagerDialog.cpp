#include "AnalysisManagerDialog.h"
#include "Utility/AnalysisDisplay.h"
#include "StepEditorDialog.h"
#include "LoadEditorDialog.h"
#include "MPCEditorDialog.h"
#include "ConstraintEditorDialog.h"
#include <QVariant>
#include <QAbstractButton>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QVBoxLayout>
#include "Widgets/CompactDoubleSpinBox.h"
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QSpacerItem>
#include <QTableWidget>
#include <QWidget>
#include "AnalysisEditor.h"
#include "Utility/UiIcons.h"
#include "Widgets/TableAppearance.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Aerodynamics/AeroManager.h"
#include "DataStructure/Factory/LoadFactory.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Wind.h"
#include "Solver/Factory/SolverFactory.h"
#include "Widgets/DialogSizing.h"
#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QResizeEvent>
#include <QSignalBlocker>
#include <QScreen>
#include <QScrollArea>
#include <QScrollBar>
#include <QSplitter>
#include <QStackedWidget>
#include <QStackedLayout>
#include <QStyle>
#include <QStyledItemDelegate>
#include <QTabBar>
#include <QTabWidget>
#include <QTimer>
#include <algorithm>
#include <array>
#include <limits>
#include <set>
#include <variant>
#include <vector>

enum class ManagerGlyph
{
    Add,
    Edit,
    Delete
};

static QIcon managerIcon(ManagerGlyph glyph, const QPalette& palette)
{
    QPixmap pixmap(48, 48);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.scale(3.0, 3.0);
    const QColor foreground = palette.color(QPalette::Text);
    const QColor accent = palette.color(QPalette::Highlight);
    painter.setPen(QPen(foreground, 1.35, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.setBrush(Qt::NoBrush);
    switch (glyph)
    {
        case ManagerGlyph::Add:
            painter.drawEllipse(QPointF(8, 8), 5.5, 5.5);
            painter.setPen(QPen(accent, 1.7, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(QPointF(8, 4.8), QPointF(8, 11.2));
            painter.drawLine(QPointF(4.8, 8), QPointF(11.2, 8));
            break;
        case ManagerGlyph::Edit:
            painter.drawRoundedRect(QRectF(2.5, 3, 8.5, 10), 1.2, 1.2);
            painter.setPen(QPen(accent, 1.7, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter.drawLine(QPointF(6, 11), QPointF(13.2, 3.8));
            painter.drawLine(QPointF(11.6, 3.4), QPointF(13.6, 5.4));
            break;
        case ManagerGlyph::Delete:
            painter.drawRoundedRect(QRectF(4, 5, 8, 8.5), 1, 1);
            painter.drawLine(QPointF(3, 4), QPointF(13, 4));
            painter.drawLine(QPointF(6, 2.5), QPointF(10, 2.5));
            painter.setPen(QPen(accent, 1.35, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(QPointF(6.5, 7), QPointF(6.5, 11));
            painter.drawLine(QPointF(9.5, 7), QPointF(9.5, 11));
            break;
    }
    return QIcon(pixmap);
}

template <typename Map> static int nextId(const Map& values)
{
    return values.empty() ? 1 : values.rbegin()->first + 1;
}

template <typename Resource>
static bool isEffectiveAtStep(const std::shared_ptr<Resource>& resource, int stepId,
                              const std::shared_ptr<AnalysisStep>& targetStep)
{
    // StepId=0 表示初始/全局资源；更早分析步的资源也会被后续分析步继承。
    if (!resource)
        return false;
    const int sourceStepId = resource->m_StepId;
    if (sourceStepId <= 0 || sourceStepId == stepId)
        return true;
    // 动力分析步是所选静力平衡状态的分支，只继承静力历史，不继承同级动力分支。
    if (targetStep && targetStep->m_Type == EnumKeyword::StepType::DYNAMIC && targetStep->m_InitialStaticStepId > 0)
    {
        return sourceStepId <= targetStep->m_InitialStaticStepId;
    }
    return sourceStepId < stepId;
}

static QTableWidgetItem* tableItem(const QString& text, int id = -1)
{
    auto* item = new QTableWidgetItem(text);
    if (id >= 0)
        item->setData(Qt::UserRole, id);
    return item;
}
std::unique_ptr<AnalysisStepEditor> AnalysisEditorFactory::CreateStep(int id,
                                                                      const std::shared_ptr<StructureData>& structure,
                                                                      const std::shared_ptr<AnalysisStep>& existing,
                                                                      QWidget* parent)
{
    return std::make_unique<StepEditorDialog>(id, structure, existing, parent);
}

std::unique_ptr<AnalysisLoadEditor> AnalysisEditorFactory::CreateLoad(int id,
                                                                      const std::shared_ptr<StructureData>& structure,
                                                                      const std::shared_ptr<LoadBase>& existing,
                                                                      QWidget* parent)
{
    return std::make_unique<LoadEditorDialog>(id, structure, existing, parent);
}

std::unique_ptr<AnalysisConstraintEditor> AnalysisEditorFactory::CreateConstraint(
    int id, const std::shared_ptr<StructureData>& structure, const std::shared_ptr<Constraint>& existing,
    QWidget* parent)
{
    return std::make_unique<ConstraintEditorDialog>(id, structure, existing, parent);
}

std::unique_ptr<AnalysisMPCEditor> AnalysisEditorFactory::CreateMPC(
    int id, const std::shared_ptr<StructureData>& structure, const std::shared_ptr<NonlinearMPCConstraint>& existing,
    QWidget* parent)
{
    return std::make_unique<MPCEditorDialog>(id, structure, existing, parent);
}

AnalysisManagerDialog::AnalysisManagerDialog(const std::shared_ptr<StructureData>& structure, Page initialPage,
                                             QWidget* parent)
    : QDialog(parent)
    , m_structure(structure)
    , m_page(initialPage)
{
    initializeUi();
    const bool isStepManager = initialPage == Page::Steps;
    const bool isLoadManager = initialPage == Page::Loads;
    const bool isConstraintManager = initialPage == Page::Constraints;
    setObjectName(isStepManager         ? QStringLiteral("analysisStepManagerDialog")
                  : isLoadManager       ? QStringLiteral("analysisLoadManagerDialog")
                  : isConstraintManager ? QStringLiteral("analysisConstraintManagerDialog")
                                        : QStringLiteral("analysisMPCManagerDialog"));
    setWindowTitle(isStepManager         ? QStringLiteral("分析步管理器")
                   : isLoadManager       ? QStringLiteral("荷载管理器")
                   : isConstraintManager ? QStringLiteral("边界条件管理器")
                                         : QStringLiteral("MPC 管理器"));
    setWindowIcon(UiIcons::applicationIcon());
    setModal(false);
    setWindowModality(Qt::NonModal);
    resize(isLoadManager ? 1120 : 920, 500);
    DialogSizing::lockCurrentHeight(this);
    QTableWidget* table = m_resourceTable;
    applyCenteredTableAppearance(table);
    table->setItemDelegateForColumn(0, new QStyledItemDelegate(table));
    QPushButton* addButton = m_addButton;
    QPushButton* editButton = m_editButton;
    QPushButton* deleteButton = m_deleteButton;
    std::function<void()> addHandler;
    std::function<void()> editHandler;
    std::function<void()> deleteHandler;
    QStringList headers;
    if (isStepManager)
    {
        m_stepTable = table;
        headers = {QStringLiteral("名称"),       QStringLiteral("类型"),       QStringLiteral("总时间"),
                   QStringLiteral("增量"),       QStringLiteral("生效荷载数"), QStringLiteral("生效约束数"),
                   QStringLiteral("生效 MPC 数")};
        headers.insert(4, QStringLiteral("计算区域"));
        addHandler = [this]()
        {
            editStep();
        };
        editHandler = [this]()
        {
            editStep(currentId(m_stepTable));
        };
        deleteHandler = [this]()
        {
            deleteSelectedStep();
        };
    }
    else if (isLoadManager)
    {
        m_loadTable = table;
        headers = {QStringLiteral("名称"),     QStringLiteral("分析步"),  QStringLiteral("物理类型"),
                   QStringLiteral("目标"),     QStringLiteral("分量"),    QStringLiteral("基值"),
                   QStringLiteral("作用时间"), QStringLiteral("时程函数")};
        addHandler = [this]()
        {
            editLoad();
        };
        editHandler = [this]()
        {
            editLoad(currentId(m_loadTable));
        };
        deleteHandler = [this]()
        {
            deleteSelectedLoad();
        };
    }
    else if (isConstraintManager)
    {
        m_constraintTable = table;
        headers = {QStringLiteral("名称"), QStringLiteral("分析步"), QStringLiteral("节点"), QStringLiteral("方向"),
                   QStringLiteral("位移值")};
        addHandler = [this]()
        {
            editConstraint();
        };
        editHandler = [this]()
        {
            editConstraint(currentId(m_constraintTable));
        };
        deleteHandler = [this]()
        {
            deleteSelectedConstraint();
        };
    }
    else
    {
        m_mpcTable = table;
        headers = {QStringLiteral("名称"),   QStringLiteral("分析步"),     QStringLiteral("主节点"),
                   QStringLiteral("从节点"), QStringLiteral("从节点方向"), QStringLiteral("约束关系")};
        addHandler = [this]()
        {
            editMPC();
        };
        editHandler = [this]()
        {
            editMPC(currentId(m_mpcTable));
        };
        deleteHandler = [this]()
        {
            deleteSelectedMPC();
        };
    }
    table->setColumnCount(headers.size());
    table->setHorizontalHeaderLabels(headers);
    table->verticalHeader()->setVisible(false);
    table->verticalHeader()->setDefaultSectionSize(36);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    addButton->setObjectName(QStringLiteral("analysisAddButton"));
    editButton->setObjectName(QStringLiteral("analysisEditButton"));
    deleteButton->setObjectName(QStringLiteral("analysisDeleteButton"));
    addButton->setProperty("primaryAction", true);
    deleteButton->setProperty("dangerAction", true);
    addButton->setIcon(managerIcon(ManagerGlyph::Add, palette()));
    editButton->setIcon(managerIcon(ManagerGlyph::Edit, palette()));
    deleteButton->setIcon(managerIcon(ManagerGlyph::Delete, palette()));
    addButton->setIconSize(QSize(18, 18));
    editButton->setIconSize(QSize(18, 18));
    deleteButton->setIconSize(QSize(18, 18));
    for (QPushButton* button : {addButton, editButton, deleteButton})
        button->setMinimumWidth(qMax(92, button->sizeHint().width() + 4));
    connect(addButton, &QPushButton::clicked, this, addHandler);
    connect(editButton, &QPushButton::clicked, this, editHandler);
    connect(deleteButton, &QPushButton::clicked, this, deleteHandler);
    connect(table, &QTableWidget::itemDoubleClicked, this,
            [this, table, editHandler](QTableWidgetItem* item)
            {
                if (table == m_stepTable && item && item->column() == 5)
                {
                    if (m_openManagerCallback)
                        m_openManagerCallback(Page::Loads);
                    return;
                }
                if (table == m_stepTable && item && item->column() == 6)
                {
                    if (m_openManagerCallback)
                        m_openManagerCallback(Page::Constraints);
                    return;
                }
                if (table == m_stepTable && item && item->column() == 7)
                {
                    if (m_openManagerCallback)
                        m_openManagerCallback(Page::MPCs);
                    return;
                }
                editHandler();
            });
    connect(table, &QTableWidget::itemSelectionChanged, this,
            [table, editButton, deleteButton]()
            {
                const bool selected = table->currentRow() >= 0;
                editButton->setEnabled(selected);
                deleteButton->setEnabled(selected);
            });
    m_buttonBox->setObjectName(QStringLiteral("analysisManagerButtonBox"));
    m_buttonBox->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
    m_buttonBox->button(QDialogButtonBox::Close)->setAutoDefault(false);
    connect(m_buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    refreshAll();
}

void AnalysisManagerDialog::setModelChangedCallback(std::function<void(const QSet<int>&)> callback)
{
    m_modelChangedCallback = std::move(callback);
}

void AnalysisManagerDialog::setOpenManagerCallback(std::function<void(Page)> callback)
{
    m_openManagerCallback = std::move(callback);
}

void AnalysisManagerDialog::refreshFromModel()
{
    refreshAll();
}

QSet<int> AnalysisManagerDialog::affectedStepsFrom(int firstStepId) const
{
    QSet<int> affected;
    if (!m_structure)
        return affected;
    for (const auto& [stepId, step] : m_structure->m_AnalysisStep)
    {
        if (step && (firstStepId <= 0 || stepId >= firstStepId))
            affected.insert(stepId);
    }
    return affected;
}

void AnalysisManagerDialog::markModelChanged(const QSet<int>& affectedStepIds)
{
    m_modelChanged = true;
    refreshAll();
    if (m_modelChangedCallback)
        m_modelChangedCallback(affectedStepIds);
}

void AnalysisManagerDialog::refreshAll()
{
    refreshStepTable();
    refreshLoadTable();
    refreshConstraintTable();
    refreshMPCTable();
}

int AnalysisManagerDialog::currentId(QTableWidget* table) const
{
    if (!table || table->currentRow() < 0 || !table->item(table->currentRow(), 0))
        return -1;
    return table->item(table->currentRow(), 0)->data(Qt::UserRole).toInt();
}

void AnalysisManagerDialog::selectId(QTableWidget* table, int id) const
{
    if (!table)
        return;
    for (int row = 0; row < table->rowCount(); ++row)
    {
        if (table->item(row, 0) && table->item(row, 0)->data(Qt::UserRole).toInt() == id)
        {
            table->selectRow(row);
            table->scrollToItem(table->item(row, 0));
            return;
        }
    }
    table->clearSelection();
}

void AnalysisManagerDialog::refreshStepTable(int preferredId)
{
    if (!m_stepTable)
        return;
    if (preferredId < 0)
        preferredId = currentId(m_stepTable);
    m_stepTable->setRowCount(0);
    if (!m_structure)
        return;
    for (const auto& [stepId, step] : m_structure->m_AnalysisStep)
    {
        if (!step)
            continue;
        int loadCount = 0;
        int constraintCount = 0;
        int mpcCount = 0;
        for (const auto& [loadId, load] : m_structure->m_Load)
        {
            Q_UNUSED(loadId);
            if (isEffectiveAtStep(load, stepId, step))
                ++loadCount;
        }
        for (const auto& [constraintId, constraint] : m_structure->m_Constraint)
        {
            Q_UNUSED(constraintId);
            if (isEffectiveAtStep(constraint, stepId, step))
                ++constraintCount;
        }
        for (const auto& [mpcId, mpc] : m_structure->m_MPCConstraints)
        {
            Q_UNUSED(mpcId);
            if (isEffectiveAtStep(mpc, stepId, step))
                ++mpcCount;
        }
        const int row = m_stepTable->rowCount();
        m_stepTable->insertRow(row);
        m_stepTable->setItem(row, 0, tableItem(AnalysisDisplay::stepDisplayName(stepId, step), stepId));
        QString typeText = step->m_Type == EnumKeyword::StepType::STATIC    ? QStringLiteral("静力")
                           : step->m_Type == EnumKeyword::StepType::DYNAMIC ? QStringLiteral("动力")
                                                                            : QStringLiteral("未知");
        if (step->m_Type == EnumKeyword::StepType::DYNAMIC && step->m_InitialStaticStepId > 0)
        {
            const auto initialIt = m_structure->m_AnalysisStep.find(step->m_InitialStaticStepId);
            typeText += QStringLiteral("（初态：%1）")
                            .arg(initialIt != m_structure->m_AnalysisStep.cend()
                                     ? AnalysisDisplay::stepDisplayName(initialIt->first, initialIt->second)
                                     : QStringLiteral("Step-%1 缺失").arg(step->m_InitialStaticStepId));
        }
        m_stepTable->setItem(row, 1, tableItem(typeText));
        m_stepTable->setItem(row, 2, tableItem(QString::number(step->m_Time, 'g', 10)));
        m_stepTable->setItem(row, 3, tableItem(QString::number(step->m_StepSize, 'g', 10)));
        QString regionText = QStringLiteral("全部启用区域");
        if (step->m_RegionScope == AnalysisRegionScope::SelectedRegions)
        {
            QStringList regionNames;
            for (int regionId : step->m_ComputeRegionIds)
            {
                const auto regionIt = m_structure->m_ComputeRegions.find(regionId);
                regionNames.append(regionIt != m_structure->m_ComputeRegions.cend() && regionIt->second
                                       ? regionIt->second->m_Name
                                       : QStringLiteral("区域 %1（缺失）").arg(regionId));
            }
            regionText = regionNames.join(QStringLiteral("、"));
        }
        m_stepTable->setItem(row, 4, tableItem(regionText));
        m_stepTable->setItem(row, 5, tableItem(QString::number(loadCount)));
        m_stepTable->setItem(row, 6, tableItem(QString::number(constraintCount)));
        m_stepTable->setItem(row, 7, tableItem(QString::number(mpcCount)));
    }
    selectId(m_stepTable, preferredId);
}
void AnalysisManagerDialog::refreshLoadTable(int preferredId)
{
    if (!m_loadTable)
        return;
    if (preferredId < 0)
        preferredId = currentId(m_loadTable);
    m_loadTable->setRowCount(0);
    if (!m_structure)
        return;
    for (const auto& [loadId, load] : m_structure->m_Load)
    {
        if (!load)
            continue;
        QString stepName = QStringLiteral("初始 / 全局");
        if (load->m_StepId > 0)
        {
            const auto step = m_structure->m_AnalysisStep.find(load->m_StepId);
            stepName = step != m_structure->m_AnalysisStep.end()
                           ? AnalysisDisplay::stepDisplayName(step->first, step->second)
                           : QStringLiteral("Step-%1（缺失）").arg(load->m_StepId);
        }
        // 每个荷载只有一个所有者，不为每个动力分支复制重力荷载，只列出继承该荷载的分支。
        QStringList inheritedBy;
        for (const auto& [stepId, step] : m_structure->m_AnalysisStep)
        {
            if (!step || step->m_Type != EnumKeyword::StepType::DYNAMIC || step->m_InitialStaticStepId <= 0 ||
                stepId == load->m_StepId)
            {
                continue;
            }
            if (load->m_StepId <= step->m_InitialStaticStepId)
                inheritedBy << AnalysisDisplay::stepDisplayName(stepId, step);
        }
        const QString stepScope = inheritedBy.isEmpty()
                                      ? stepName
                                      : QStringLiteral("%1  \u2192  %2\uFF08\u7EE7\u627F\uFF09")
                                            .arg(stepName, inheritedBy.join(QStringLiteral("\u3001")));
        const int targetId = AnalysisDisplay::loadTargetId(load);
        const QString target = targetId > 0 ? (load->m_LoadType == EnumKeyword::LoadType::FORCE_NODE
                                                   ? QStringLiteral("节点 %1").arg(targetId)
                                                   : QStringLiteral("单元 %1").arg(targetId))
                                            : QStringLiteral("全模型");
        const int row = m_loadTable->rowCount();
        m_loadTable->insertRow(row);
        m_loadTable->setItem(row, 0, tableItem(AnalysisDisplay::loadDisplayName(loadId, load), loadId));
        auto* stepItem = tableItem(stepScope);
        if (!inheritedBy.isEmpty())
        {
            stepItem->setToolTip(
                QStringLiteral("\u6B64\u8377\u8F7D\u5B9A\u4E49\u5C5E\u4E8E %1\u3002\u5728\u6C42\u89E3 %2 \u65F6\uFF0C"
                               "\u5B83\u4F1A\u4EE5\u5168\u503C\u6301\u7EED\u4F5C\u7528\uFF0C\u4E0D\u9700\u8981\u91CD"
                               "\u590D\u521B\u5EFA\u3002")
                    .arg(stepName, inheritedBy.join(QStringLiteral("\u3001"))));
        }
        m_loadTable->setItem(row, 1, stepItem);
        m_loadTable->setItem(row, 2, tableItem(AnalysisDisplay::loadPhysicalTypeName(load)));
        m_loadTable->setItem(row, 3, tableItem(target));
        m_loadTable->setItem(row, 4, tableItem(AnalysisDisplay::loadComponentText(load)));
        m_loadTable->setItem(row, 5, tableItem(AnalysisDisplay::loadValueText(load)));
        m_loadTable->setItem(row, 6, tableItem(AnalysisDisplay::loadTimeWindowText(load)));
        m_loadTable->setItem(row, 7, tableItem(AnalysisDisplay::timeFunctionName(load->m_FunctionType)));
    }
    selectId(m_loadTable, preferredId);
}
void AnalysisManagerDialog::refreshConstraintTable(int preferredId)
{
    if (!m_constraintTable)
        return;
    if (preferredId < 0)
        preferredId = currentId(m_constraintTable);
    m_constraintTable->setRowCount(0);
    if (!m_structure)
        return;
    for (const auto& [constraintId, constraint] : m_structure->m_Constraint)
    {
        if (!constraint)
            continue;
        QString stepName = QStringLiteral("初始 / 全局");
        if (constraint->m_StepId > 0)
        {
            const auto step = m_structure->m_AnalysisStep.find(constraint->m_StepId);
            stepName = step != m_structure->m_AnalysisStep.end()
                           ? AnalysisDisplay::stepDisplayName(step->first, step->second)
                           : QStringLiteral("Step-%1（缺失）").arg(constraint->m_StepId);
        }
        const auto node = constraint->m_pNode.lock();
        const int row = m_constraintTable->rowCount();
        m_constraintTable->insertRow(row);
        m_constraintTable->setItem(
            row, 0, tableItem(AnalysisDisplay::constraintDisplayName(constraintId, constraint), constraintId));
        m_constraintTable->setItem(row, 1, tableItem(stepName));
        m_constraintTable->setItem(row, 2, tableItem(node ? QString::number(node->m_Id) : QStringLiteral("--")));
        m_constraintTable->setItem(row, 3, tableItem(AnalysisDisplay::directionName(constraint->m_Direction)));
        m_constraintTable->setItem(row, 4, tableItem(QString::number(constraint->m_Value, 'g', 12)));
    }
    selectId(m_constraintTable, preferredId);
}

void AnalysisManagerDialog::refreshMPCTable(int preferredId)
{
    if (!m_mpcTable)
        return;
    if (preferredId < 0)
        preferredId = currentId(m_mpcTable);
    m_mpcTable->setRowCount(0);
    if (!m_structure)
        return;
    for (const auto& [mpcId, mpc] : m_structure->m_MPCConstraints)
    {
        if (!mpc)
            continue;
        QString stepName = QStringLiteral("初始 / 全局");
        if (mpc->m_StepId > 0)
        {
            const auto step = m_structure->m_AnalysisStep.find(mpc->m_StepId);
            stepName = step != m_structure->m_AnalysisStep.cend()
                           ? AnalysisDisplay::stepDisplayName(step->first, step->second)
                           : QStringLiteral("Step-%1（缺失）").arg(mpc->m_StepId);
        }
        const std::vector<int> nodeIds = mpc->GetNodeIds();
        const int row = m_mpcTable->rowCount();
        m_mpcTable->insertRow(row);
        m_mpcTable->setItem(row, 0, tableItem(AnalysisDisplay::mpcDisplayName(mpcId, mpc), mpcId));
        m_mpcTable->setItem(row, 1, tableItem(stepName));
        m_mpcTable->setItem(row, 2,
                            tableItem(nodeIds.size() == 2 ? QString::number(nodeIds[0]) : QStringLiteral("--")));
        m_mpcTable->setItem(row, 3,
                            tableItem(nodeIds.size() == 2 ? QString::number(nodeIds[1]) : QStringLiteral("--")));
        m_mpcTable->setItem(row, 4, tableItem(AnalysisDisplay::mpcDirectionText(mpc)));
        m_mpcTable->setItem(row, 5, tableItem(AnalysisDisplay::mpcRelationText(mpc)));
    }
    selectId(m_mpcTable, preferredId);
}

void AnalysisManagerDialog::editStep(int stepId)
{
    if (!m_structure)
        return;
    std::shared_ptr<AnalysisStep> existing;
    if (stepId >= 0)
    {
        const auto found = m_structure->m_AnalysisStep.find(stepId);
        if (found == m_structure->m_AnalysisStep.end() || !found->second)
            return;
        existing = found->second;
    }
    else
    {
        stepId = nextId(m_structure->m_AnalysisStep);
    }

    auto editor = AnalysisEditorFactory::CreateStep(stepId, m_structure, existing, this);
    if (editor->exec() != QDialog::Accepted)
        return;
    const AnalysisStepConfig value = editor->config();
    m_structure->m_AeroManager.setShapePreservingInterpolation(editor->shapePreservingAerodynamicInterpolation());
    if (value.enableGalloping &&
        value.gallopingProfileBindingMode == GallopingProfileBindingMode::ExplicitElementProfile)
    {
        const auto profiles = editor->quadWireProfiles();
        for (const auto& [elementId, element] : m_structure->m_Elements)
        {
            Q_UNUSED(elementId);
            if (element && element->m_AeroBundleCount == 4 && element->m_WireId >= 0 && element->m_WireId < 4 &&
                profiles[element->m_WireId] >= 0)
                element->m_AeroProfileId = profiles[element->m_WireId];
        }
    }
    if (existing)
    {
        existing->m_Name = value.name.isEmpty() ? QStringLiteral("Step-%1").arg(stepId) : value.name;
        existing->m_Type = value.type;
        existing->isDynamic = value.type == EnumKeyword::StepType::DYNAMIC;
        existing->m_Time = value.totalTime;
        existing->m_StepSize = value.stepSize;
        existing->m_Tolerance = value.tolerance;
        existing->m_MaxIterations = value.maxIterations;
        existing->m_DynamicSolverType = value.dynamicSolverType;
        existing->m_GallopingAerodynamicTangentMode = value.gallopingAerodynamicTangentMode;
        existing->m_InitialStaticStepId = existing->isDynamic ? value.initialStaticStepId : 0;
        existing->m_AdaptiveTssbn = value.adaptiveTssbn;
        existing->m_AdaptiveTimeStep = value.adaptiveTimeStep;
        existing->m_StructuralDamping.settings = value.structuralDamping;
        existing->m_StructuralDamping.Reset();
        existing->m_EnableGalloping = value.enableGalloping;
        existing->m_GallopingProfileBindingMode = value.gallopingProfileBindingMode;
        existing->m_GallopingIceThickness = value.gallopingIceThickness;
        existing->m_GallopingInitialAttackDegrees = value.gallopingInitialAttackDegrees;
        existing->m_GallopingInitialVerticalPerturbationMeters = value.gallopingInitialVerticalPerturbationMeters;
        existing->m_GallopingInitialVerticalPerturbationWaves = value.gallopingInitialVerticalPerturbationWaves;
        existing->m_RegionScope = value.regionScope;
        existing->m_ComputeRegionIds = value.computeRegionIds;
    }
    else
    {
        m_structure->AddAnalysisStep(value);
    }

    // 区域参与状态是全部分析步引用关系的汇总，只允许这一处公共规则计算，
    // 避免导入、编辑或删除分析步后两个窗口出现不同状态。
    m_structure->SynchronizeComputeRegionEnablementFromSteps();

    QSet<int> affectedStepIds;
    for (const auto& [affectedStepId, affectedStep] : m_structure->m_AnalysisStep)
    {
        if (affectedStep)
            affectedStepIds.insert(affectedStepId);
    }
    markModelChanged(affectedStepIds);
    selectId(m_stepTable, stepId);
}
void AnalysisManagerDialog::editLoad(int loadId)
{
    if (!m_structure)
        return;
    if (loadId < 0 && m_structure->m_AnalysisStep.empty())
    {
        QMessageBox::information(this, QStringLiteral("无法新增荷载"), QStringLiteral("请先创建至少一个分析步。"));
        return;
    }
    std::shared_ptr<LoadBase> existing;
    if (loadId >= 0)
    {
        const auto found = m_structure->m_Load.find(loadId);
        if (found == m_structure->m_Load.end() || !found->second)
            return;
        existing = found->second;
    }
    else
    {
        loadId = nextId(m_structure->m_Load);
    }
    const int oldStepId = existing ? existing->m_StepId : -1;
    auto editor = AnalysisEditorFactory::CreateLoad(loadId, m_structure, existing, this);
    if (editor->exec() != QDialog::Accepted || !editor->value())
        return;
    m_structure->m_Load[loadId] = editor->value();
    QSet<int> affected = affectedStepsFrom(editor->value()->m_StepId);
    if (oldStepId >= 0)
        affected.unite(affectedStepsFrom(oldStepId));
    markModelChanged(affected);
    selectId(m_loadTable, loadId);
}
void AnalysisManagerDialog::editConstraint(int constraintId)
{
    if (!m_structure)
        return;
    std::shared_ptr<Constraint> existing;
    if (constraintId >= 0)
    {
        const auto found = m_structure->m_Constraint.find(constraintId);
        if (found == m_structure->m_Constraint.end() || !found->second)
            return;
        existing = found->second;
    }
    else
    {
        constraintId = nextId(m_structure->m_Constraint);
    }
    const int oldStepId = existing ? existing->m_StepId : -1;
    auto editor = AnalysisEditorFactory::CreateConstraint(constraintId, m_structure, existing, this);
    if (editor->exec() != QDialog::Accepted || !editor->node())
        return;
    auto constraint = existing ? existing : std::make_shared<Constraint>();
    constraint->m_Id = constraintId;
    constraint->m_Name = editor->name();
    constraint->m_StepId = editor->stepId();
    constraint->m_pNode = editor->node();
    constraint->m_Direction = editor->direction();
    constraint->m_Value = editor->displacement();
    m_structure->m_Constraint[constraintId] = constraint;
    QSet<int> affected = affectedStepsFrom(constraint->m_StepId);
    if (oldStepId >= 0)
        affected.unite(affectedStepsFrom(oldStepId));
    markModelChanged(affected);
    selectId(m_constraintTable, constraintId);
}

void AnalysisManagerDialog::editMPC(int mpcId)
{
    if (!m_structure)
        return;
    std::shared_ptr<NonlinearMPCConstraint> existing;
    if (mpcId >= 0)
    {
        const auto found = m_structure->m_MPCConstraints.find(mpcId);
        if (found == m_structure->m_MPCConstraints.end() || !found->second)
            return;
        existing = found->second;
    }
    else
    {
        mpcId = nextId(m_structure->m_MPCConstraints);
    }
    const int oldStepId = existing ? existing->m_StepId : -1;
    auto editor = AnalysisEditorFactory::CreateMPC(mpcId, m_structure, existing, this);
    if (editor->exec() != QDialog::Accepted || !editor->value())
        return;

    m_structure->m_MPCConstraints[mpcId] = editor->value();
    QString regionError;
    if (!m_structure->RebuildAndMergeComputeRegions(&regionError))
    {
        if (existing)
            m_structure->m_MPCConstraints[mpcId] = existing;
        else
            m_structure->m_MPCConstraints.erase(mpcId);
        m_structure->RebuildAndMergeComputeRegions();
        QMessageBox::warning(this, QStringLiteral("无法更新 MPC"), regionError);
        return;
    }

    QSet<int> affected = affectedStepsFrom(editor->value()->m_StepId);
    if (oldStepId >= 0)
        affected.unite(affectedStepsFrom(oldStepId));
    markModelChanged(affected);
    selectId(m_mpcTable, mpcId);
}

void AnalysisManagerDialog::deleteSelectedStep()
{
    if (!m_structure)
        return;
    const int stepId = currentId(m_stepTable);
    const auto found = m_structure->m_AnalysisStep.find(stepId);
    if (found == m_structure->m_AnalysisStep.end())
        return;
    int loadCount = 0;
    int constraintCount = 0;
    int mpcCount = 0;
    int dependentStepCount = 0;
    for (const auto& [id, load] : m_structure->m_Load)
    {
        Q_UNUSED(id);
        if (load && load->m_StepId == stepId)
            ++loadCount;
    }
    for (const auto& [id, constraint] : m_structure->m_Constraint)
    {
        Q_UNUSED(id);
        if (constraint && constraint->m_StepId == stepId)
            ++constraintCount;
    }
    for (const auto& [id, mpc] : m_structure->m_MPCConstraints)
    {
        Q_UNUSED(id);
        if (mpc && mpc->m_StepId == stepId)
            ++mpcCount;
    }
    for (const auto& [candidateId, candidate] : m_structure->m_AnalysisStep)
    {
        Q_UNUSED(candidateId);
        if (candidate && candidate->m_InitialStaticStepId == stepId)
            ++dependentStepCount;
    }
    const QString prompt = QStringLiteral("确定删除“%1”吗？\n同时会删除归属于该步的 %2 个荷载、"
                                          "%3 个约束和 %4 个 MPC。\n另有 %5 个动力步继承该静力步，删除后将改为不继承。")
                               .arg(AnalysisDisplay::stepDisplayName(stepId, found->second))
                               .arg(loadCount)
                               .arg(constraintCount)
                               .arg(mpcCount)
                               .arg(dependentStepCount);
    if (QMessageBox::question(this, QStringLiteral("删除分析步"), prompt, QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::No) != QMessageBox::Yes)
        return;

    // 当前分析步引入的荷载和约束会被后续分析步继承；删除前记录依赖范围，仅使真正受影响的后续算例失效。
    const QSet<int> affectedStepIds = affectedStepsFrom(stepId);
    m_structure->m_AnalysisStep.erase(found);
    for (auto& [candidateId, candidate] : m_structure->m_AnalysisStep)
    {
        Q_UNUSED(candidateId);
        if (candidate && candidate->m_InitialStaticStepId == stepId)
            candidate->m_InitialStaticStepId = 0;
    }
    for (auto it = m_structure->m_Load.begin(); it != m_structure->m_Load.end();)
        it = it->second && it->second->m_StepId == stepId ? m_structure->m_Load.erase(it) : std::next(it);
    for (auto it = m_structure->m_Constraint.begin(); it != m_structure->m_Constraint.end();)
        it = it->second && it->second->m_StepId == stepId ? m_structure->m_Constraint.erase(it) : std::next(it);
    for (auto it = m_structure->m_MPCConstraints.begin(); it != m_structure->m_MPCConstraints.end();)
    {
        it = it->second && it->second->m_StepId == stepId ? m_structure->m_MPCConstraints.erase(it) : std::next(it);
    }
    m_structure->SynchronizeComputeRegionEnablementFromSteps();
    markModelChanged(affectedStepIds);
}
void AnalysisManagerDialog::deleteSelectedLoad()
{
    if (!m_structure)
        return;
    const int loadId = currentId(m_loadTable);
    const auto found = m_structure->m_Load.find(loadId);
    if (found == m_structure->m_Load.end())
        return;
    if (QMessageBox::question(
            this, QStringLiteral("删除荷载"),
            QStringLiteral("确定删除“%1”吗？").arg(AnalysisDisplay::loadDisplayName(loadId, found->second)),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;
    const int firstAffectedStep = found->second ? found->second->m_StepId : 0;
    m_structure->m_Load.erase(found);
    markModelChanged(affectedStepsFrom(firstAffectedStep));
}
void AnalysisManagerDialog::deleteSelectedConstraint()
{
    if (!m_structure)
        return;
    const int constraintId = currentId(m_constraintTable);
    const auto found = m_structure->m_Constraint.find(constraintId);
    if (found == m_structure->m_Constraint.end())
        return;
    if (QMessageBox::question(
            this, QStringLiteral("删除约束"),
            QStringLiteral("确定删除“%1”吗？").arg(AnalysisDisplay::constraintDisplayName(constraintId, found->second)),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;
    const int firstAffectedStep = found->second ? found->second->m_StepId : 0;
    m_structure->m_Constraint.erase(found);
    markModelChanged(affectedStepsFrom(firstAffectedStep));
}

void AnalysisManagerDialog::deleteSelectedMPC()
{
    if (!m_structure)
        return;
    const int mpcId = currentId(m_mpcTable);
    const auto found = m_structure->m_MPCConstraints.find(mpcId);
    if (found == m_structure->m_MPCConstraints.end())
        return;
    if (QMessageBox::question(
            this, QStringLiteral("删除 MPC"),
            QStringLiteral("确定删除“%1”吗？").arg(AnalysisDisplay::mpcDisplayName(mpcId, found->second)),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }
    const int firstAffectedStep = found->second ? found->second->m_StepId : 0;
    m_structure->m_MPCConstraints.erase(found);
    m_structure->RebuildAndMergeComputeRegions();
    markModelChanged(affectedStepsFrom(firstAffectedStep));
}

void AnalysisManagerDialog::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("AnalysisManagerDialogClass");
    }
    setMinimumSize(QSize(720, 380));
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setSpacing(12);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(16, 16, 16, 16);
    QWidget* managerPage = new QWidget(this);
    managerPage->setObjectName("managerPage");
    QVBoxLayout* pageLayout = new QVBoxLayout(managerPage);
    pageLayout->setObjectName("pageLayout");
    pageLayout->setContentsMargins(0, 0, 0, 0);
    m_resourceTable = new QTableWidget(managerPage);
    m_resourceTable->setObjectName("resourceTable");
    m_resourceTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_resourceTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_resourceTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_resourceTable->setAlternatingRowColors(true);
    m_resourceTable->setShowGrid(false);
    m_resourceTable->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    pageLayout->addWidget(m_resourceTable);

    rootLayout->addWidget(managerPage);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->setSpacing(8);
    actionLayout->setObjectName("actionLayout");
    m_addButton = new QPushButton(this);
    m_addButton->setObjectName("addButton");
    m_addButton->setAutoDefault(false);

    actionLayout->addWidget(m_addButton);

    m_editButton = new QPushButton(this);
    m_editButton->setObjectName("editButton");
    m_editButton->setEnabled(false);
    m_editButton->setAutoDefault(false);

    actionLayout->addWidget(m_editButton);

    m_deleteButton = new QPushButton(this);
    m_deleteButton->setObjectName("deleteButton");
    m_deleteButton->setEnabled(false);
    m_deleteButton->setAutoDefault(false);

    actionLayout->addWidget(m_deleteButton);

    QSpacerItem* actionSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

    actionLayout->addItem(actionSpacer);

    m_buttonBox = new QDialogButtonBox(this);
    m_buttonBox->setObjectName("buttonBox");
    m_buttonBox->setStandardButtons(QDialogButtonBox::Close);

    actionLayout->addWidget(m_buttonBox);

    rootLayout->addLayout(actionLayout);

    QWidget::setTabOrder(m_resourceTable, m_addButton);
    QWidget::setTabOrder(m_addButton, m_editButton);
    QWidget::setTabOrder(m_editButton, m_deleteButton);

    m_addButton->setText(QCoreApplication::translate("AnalysisManagerDialogClass", "新增", nullptr));
    m_editButton->setText(QCoreApplication::translate("AnalysisManagerDialogClass", "编辑", nullptr));
    m_deleteButton->setText(QCoreApplication::translate("AnalysisManagerDialogClass", "删除", nullptr));
}
