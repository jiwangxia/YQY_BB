#include "MPCEditorDialog.h"
#include "Widgets/DialogSizing.h"
#include "Utility/UiIcons.h"
#include "Utility/AnalysisDisplay.h"
#include "DataStructure/Structure/StructureData.h"
#include <QChar>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QSpinBox>
#include <QString>
#include <QStringLiteral>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <variant>
#include <vector>

MPCEditorDialog::MPCEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
                                 const std::shared_ptr<NonlinearMPCConstraint>& mpc, QWidget* parent)
    : AnalysisMPCEditor(parent)
    , m_id(id)
    , m_structure(structure)
{
    setObjectName(QStringLiteral("analysisMPCEditorDialog"));
    setWindowTitle(mpc ? QStringLiteral("编辑 MPC") : QStringLiteral("新增 MPC"));
    setWindowIcon(UiIcons::applicationIcon());
    setModal(true);
    resize(480, 310);
    DialogSizing::fitCompactFormToScreen(this);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(14);
    auto* form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    form->setHorizontalSpacing(12);
    form->setVerticalSpacing(8);
    m_name = new QLineEdit(this);
    m_name->setPlaceholderText(QStringLiteral("MPC-%1").arg(id));
    m_step = new QComboBox(this);
    m_step->addItem(QStringLiteral("初始 / 全局"), 0);
    if (structure)
    {
        for (const auto& [stepId, step] : structure->m_AnalysisStep)
            if (step)
                m_step->addItem(AnalysisDisplay::stepDisplayName(stepId, step), stepId);
    }
    m_master = new QSpinBox(this);
    m_slave = new QSpinBox(this);
    m_master->setRange(1, std::numeric_limits<int>::max());
    m_slave->setRange(1, std::numeric_limits<int>::max());
    m_direction = new QComboBox(this);
    m_direction->setEditable(true);
    m_direction->addItems(
        {QStringLiteral("0"), QStringLiteral("1"), QStringLiteral("2"), QStringLiteral("05"), QStringLiteral("012")});
    m_direction->setToolTip(QStringLiteral("输入从节点消元自由度。当前求解器支持单个平动方向 0/1/2，"
                                           "非线性剪力释放 05，以及完整平动方向 012。"));
    form->addRow(QStringLiteral("名称"), m_name);
    form->addRow(QStringLiteral("生效分析步"), m_step);
    form->addRow(QStringLiteral("主节点"), m_master);
    form->addRow(QStringLiteral("从节点"), m_slave);
    form->addRow(QStringLiteral("从节点方向"), m_direction);
    root->addLayout(form);

    auto* hint = new QLabel(QStringLiteral("关系由程序根据 SlaveDirection 自动建立：单方向使用初始定长关系，"
                                           "05 使用论文非线性剪力释放关系，012 使用大转动刚性偏置关系。"),
                            this);
    hint->setWordWrap(true);
    hint->setObjectName(QStringLiteral("analysisMPCHint"));
    hint->setProperty("secondaryText", true);
    root->addWidget(hint);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
    buttons->button(QDialogButtonBox::Ok)->setDefault(true);
    buttons->button(QDialogButtonBox::Ok)->setProperty("primaryAction", true);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    root->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, this, &MPCEditorDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    if (mpc)
    {
        m_name->setText(mpc->m_Name);
        int stepIndex = m_step->findData(mpc->m_StepId);
        if (stepIndex < 0)
        {
            m_step->addItem(QStringLiteral("Step-%1（缺失）").arg(mpc->m_StepId), mpc->m_StepId);
            stepIndex = m_step->count() - 1;
        }
        m_step->setCurrentIndex(stepIndex);
        const std::vector<int> nodeIds = mpc->GetNodeIds();
        if (nodeIds.size() == 2)
        {
            m_master->setValue(nodeIds[0]);
            m_slave->setValue(nodeIds[1]);
        }
        m_direction->setCurrentText(AnalysisDisplay::mpcDirectionText(mpc));
    }
    else if (structure && !structure->m_Nodes.empty())
    {
        m_master->setValue(structure->m_Nodes.cbegin()->first);
        auto slaveIt = structure->m_Nodes.cbegin();
        if (slaveIt != structure->m_Nodes.cend())
            ++slaveIt;
        m_slave->setValue(slaveIt != structure->m_Nodes.cend() ? slaveIt->first : structure->m_Nodes.cbegin()->first);
    }
}

std::shared_ptr<NonlinearMPCConstraint> MPCEditorDialog::value() const
{
    return m_result;
}

void MPCEditorDialog::accept()
{
    if (!m_structure)
        return;
    const auto master = m_structure->FindNode(m_master->value());
    const auto slave = m_structure->FindNode(m_slave->value());
    if (!master || !slave || master == slave)
    {
        QMessageBox::warning(this, QStringLiteral("无效主从节点"),
                             QStringLiteral("主节点和从节点必须存在，并且不能是同一个节点。"));
        return;
    }

    const QString directionText = m_direction->currentText().trimmed();
    std::vector<int> directions;
    std::set<int> uniqueDirections;
    for (const QChar character : directionText)
    {
        if (!character.isDigit())
        {
            QMessageBox::warning(this, QStringLiteral("无效从节点方向"),
                                 QStringLiteral("SlaveDirection 只能由数字 0 到 5 组成。"));
            return;
        }
        const int direction = character.digitValue();
        if (direction < 0 || direction > 5 || !uniqueDirections.insert(direction).second)
        {
            QMessageBox::warning(this, QStringLiteral("无效从节点方向"),
                                 QStringLiteral("SlaveDirection 包含越界或重复自由度。"));
            return;
        }
        directions.push_back(direction);
    }
    if (!((directions.size() == 1 && directions.front() < 3) || directions == std::vector<int>({0, 5}) ||
          directions == std::vector<int>({0, 1, 2})))
    {
        QMessageBox::information(this, QStringLiteral("当前求解范围"),
                                 QStringLiteral("当前阶段支持单个平动方向 0/1/2、非线性剪力释放 05，"
                                                "或完整平动方向 012。"));
        return;
    }
    for (const int direction : directions)
    {
        if (direction >= slave->m_DOF.size())
        {
            QMessageBox::warning(this, QStringLiteral("从自由度越界"),
                                 QStringLiteral("节点 %1 没有自由度 %2。").arg(slave->m_Id).arg(direction));
            return;
        }
        for (const auto& [constraintId, constraint] : m_structure->m_Constraint)
        {
            const auto constrainedNode = constraint ? constraint->m_pNode.lock() : nullptr;
            if (constrainedNode && constrainedNode->m_Id == slave->m_Id &&
                static_cast<int>(constraint->m_Direction) == direction)
            {
                QMessageBox::warning(this, QStringLiteral("从自由度冲突"),
                                     QStringLiteral("节点 %1 的自由度 %2 已被位移约束 %3 固定。")
                                         .arg(slave->m_Id)
                                         .arg(direction)
                                         .arg(constraintId));
                return;
            }
        }
    }

    for (const auto& [otherId, other] : m_structure->m_MPCConstraints)
    {
        if (otherId == m_id || !other)
            continue;
        const std::vector<int> otherNodes = other->GetNodeIds();
        if (otherNodes.size() != 2 || otherNodes[1] != slave->m_Id)
            continue;
        for (const int direction : directions)
        {
            if (std::find(other->m_SlaveDirections.cbegin(), other->m_SlaveDirections.cend(), direction) !=
                other->m_SlaveDirections.cend())
            {
                QMessageBox::warning(this, QStringLiteral("从自由度重复"),
                                     QStringLiteral("节点 %1 的自由度 %2 已被 MPC %3 使用。")
                                         .arg(slave->m_Id)
                                         .arg(direction)
                                         .arg(otherId));
                return;
            }
        }
    }

    std::shared_ptr<NonlinearMPCConstraint> result;
    if (directions.size() == 1)
    {
        auto distance = std::make_shared<DistanceMPCConstraint>();
        distance->m_pNodeA = slave;
        distance->m_pNodeB = master;
        distance->m_pSlaveNode = slave;
        distance->m_SlaveDirection = directions.front();
        distance->m_Length =
            Eigen::Vector3d(slave->m_X - master->m_X, slave->m_Y - master->m_Y, slave->m_Z - master->m_Z).norm();
        result = std::move(distance);
    }
    else if (directions == std::vector<int>({0, 5}))
    {
        auto release = std::make_shared<PlanarShearReleaseMPCConstraint>();
        release->m_pMasterNode = master;
        release->m_pSlaveNode = slave;
        result = std::move(release);
    }
    else
    {
        auto rigid = std::make_shared<RigidOffsetMPCConstraint>();
        rigid->m_pMasterNode = master;
        rigid->m_pSlaveNode = slave;
        rigid->m_Offset = Eigen::Vector3d(slave->m_X - master->m_X, slave->m_Y - master->m_Y, slave->m_Z - master->m_Z);
        result = std::move(rigid);
    }
    result->m_Id = m_id;
    result->m_Name = m_name->text().trimmed();
    result->m_StepId = m_step->currentData().toInt();
    result->m_SlaveDirections = std::move(directions);
    m_result = std::move(result);
    QDialog::accept();
}
