#include "ConstraintEditorDialog.h"
#include "Widgets/CompactDoubleSpinBox.h"
#include "Widgets/DialogSizing.h"
#include "Utility/UiIcons.h"
#include "Utility/AnalysisDisplay.h"
#include "DataStructure/Structure/StructureData.h"
#include <QComboBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QSize>
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

ConstraintEditorDialog::ConstraintEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
                                               const std::shared_ptr<Constraint>& constraint, QWidget* parent)
    : AnalysisConstraintEditor(parent)
    , m_structure(structure)
{
    initializeUi();
    setObjectName(QStringLiteral("analysisConstraintEditorDialog"));
    setWindowTitle(constraint ? QStringLiteral("编辑约束") : QStringLiteral("新增约束"));
    setWindowIcon(UiIcons::applicationIcon());
    setModal(true);
    DialogSizing::fitCompactFormToScreen(this);
    m_formLayout->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_formLayout->setHorizontalSpacing(12);
    m_formLayout->setVerticalSpacing(8);
    m_name->setPlaceholderText(QStringLiteral("Constraint-%1").arg(id));
    m_step->addItem(QStringLiteral("初始 / 全局"), 0);
    for (const auto& [stepId, step] : structure->m_AnalysisStep)
        if (step)
            m_step->addItem(AnalysisDisplay::stepDisplayName(stepId, step), stepId);
    for (int value = static_cast<int>(EnumKeyword::Direction::X); value <= static_cast<int>(EnumKeyword::Direction::RZ);
         ++value)
    {
        const auto direction = static_cast<EnumKeyword::Direction>(value);
        m_direction->addItem(AnalysisDisplay::directionName(direction), value);
    }
    auto* buttons = m_buttonBox;
    buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
    buttons->button(QDialogButtonBox::Ok)->setDefault(true);
    buttons->button(QDialogButtonBox::Ok)->setProperty("primaryAction", true);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    connect(buttons, &QDialogButtonBox::accepted, this, &ConstraintEditorDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    if (constraint)
    {
        m_name->setText(constraint->m_Name);
        m_step->setCurrentIndex(qMax(0, m_step->findData(constraint->m_StepId)));
        const auto node = constraint->m_pNode.lock();
        if (node)
            m_node->setValue(node->m_Id);
        m_direction->setCurrentIndex(qMax(0, m_direction->findData(static_cast<int>(constraint->m_Direction))));
        m_value->setValue(constraint->m_Value);
    }
    else
    {
        if (m_step->count() > 1)
            m_step->setCurrentIndex(1);
        if (!structure->m_Nodes.empty())
            m_node->setValue(structure->m_Nodes.begin()->first);
    }
}

QString ConstraintEditorDialog::name() const
{
    return m_name->text().trimmed();
}

int ConstraintEditorDialog::stepId() const
{
    return m_step->currentData().toInt();
}

std::shared_ptr<Node> ConstraintEditorDialog::node() const
{
    return m_nodeValue;
}

EnumKeyword::Direction ConstraintEditorDialog::direction() const
{
    return static_cast<EnumKeyword::Direction>(m_direction->currentData().toInt());
}

double ConstraintEditorDialog::displacement() const
{
    return m_value->value();
}

void ConstraintEditorDialog::accept()
{
    m_nodeValue = m_structure->FindNode(m_node->value());
    if (!m_nodeValue)
    {
        QMessageBox::warning(this, QStringLiteral("无效节点"), QStringLiteral("节点 %1 不存在。").arg(m_node->value()));
        return;
    }
    QDialog::accept();
}

void ConstraintEditorDialog::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("ConstraintEditorDialogClass");
    }
    setMinimumSize(QSize(440, 0));
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setSpacing(14);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(18, 18, 18, 18);
    m_formLayout = new QFormLayout();
    m_formLayout->setObjectName("formLayout");
    m_formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    QLabel* nameLabel = new QLabel(this);
    nameLabel->setObjectName("nameLabel");

    m_formLayout->setWidget(0, QFormLayout::LabelRole, nameLabel);

    m_name = new QLineEdit(this);
    m_name->setObjectName("nameEdit");

    m_formLayout->setWidget(0, QFormLayout::FieldRole, m_name);

    QLabel* stepLabel = new QLabel(this);
    stepLabel->setObjectName("stepLabel");

    m_formLayout->setWidget(1, QFormLayout::LabelRole, stepLabel);

    m_step = new QComboBox(this);
    m_step->setObjectName("stepCombo");

    m_formLayout->setWidget(1, QFormLayout::FieldRole, m_step);

    QLabel* nodeLabel = new QLabel(this);
    nodeLabel->setObjectName("nodeLabel");

    m_formLayout->setWidget(2, QFormLayout::LabelRole, nodeLabel);

    m_node = new QSpinBox(this);
    m_node->setObjectName("nodeSpin");
    m_node->setMinimum(1);
    m_node->setMaximum(2147483647);

    m_formLayout->setWidget(2, QFormLayout::FieldRole, m_node);

    QLabel* directionLabel = new QLabel(this);
    directionLabel->setObjectName("directionLabel");

    m_formLayout->setWidget(3, QFormLayout::LabelRole, directionLabel);

    m_direction = new QComboBox(this);
    m_direction->setObjectName("directionCombo");

    m_formLayout->setWidget(3, QFormLayout::FieldRole, m_direction);

    QLabel* valueLabel = new QLabel(this);
    valueLabel->setObjectName("valueLabel");

    m_formLayout->setWidget(4, QFormLayout::LabelRole, valueLabel);

    m_value = new CompactDoubleSpinBox(this);
    m_value->setObjectName("valueSpin");
    m_value->setMinimum(-1000000000000000.000000000000000);
    m_value->setMaximum(1000000000000000.000000000000000);
    m_value->setDecimals(12);

    m_formLayout->setWidget(4, QFormLayout::FieldRole, m_value);

    rootLayout->addLayout(m_formLayout);

    m_buttonBox = new QDialogButtonBox(this);
    m_buttonBox->setObjectName("buttonBox");
    m_buttonBox->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);

    rootLayout->addWidget(m_buttonBox);

    nameLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "名称", nullptr));
    stepLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "首次生效分析步", nullptr));
    nodeLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "节点编号", nullptr));
    directionLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "方向", nullptr));
    valueLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "位移值", nullptr));
}
