#include "AnalysisManagerDialog.h"
#include "Widgets/TableAppearance.h"

#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Aerodynamics/AeroManager.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "Widgets/CompactDoubleSpinBox.h"
#include "Widgets/DialogSizing.h"
#include "ui_AnalysisManagerDialog.h"
#include "ui_StepEditorDialog.h"
#include "ui_LoadEditorDialog.h"
#include "ui_ConstraintEditorDialog.h"

#include <QDialogButtonBox>
#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <limits>
#include <variant>
#include <vector>

namespace
{
enum class ManagerGlyph { Add, Edit, Delete, Steps, Load, Constraint, MPC };

QIcon managerIcon(ManagerGlyph glyph, const QPalette& palette)
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
    case ManagerGlyph::Steps:
        painter.drawPolyline(QPolygonF{ QPointF(2, 12), QPointF(5, 12), QPointF(5, 8),
            QPointF(9, 8), QPointF(9, 4), QPointF(14, 4) });
        painter.setBrush(accent);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPointF(5, 12), 1.25, 1.25);
        painter.drawEllipse(QPointF(9, 8), 1.25, 1.25);
        painter.drawEllipse(QPointF(14, 4), 1.25, 1.25);
        break;
    case ManagerGlyph::Load:
        painter.drawLine(QPointF(8, 1.5), QPointF(8, 10));
        painter.drawLine(QPointF(5, 7), QPointF(8, 10));
        painter.drawLine(QPointF(11, 7), QPointF(8, 10));
        painter.setPen(QPen(accent, 1.4, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(3, 13), QPointF(13, 13));
        break;
    case ManagerGlyph::Constraint:
        painter.drawLine(QPointF(3, 2), QPointF(3, 14));
        painter.drawLine(QPointF(3, 5), QPointF(8, 5));
        painter.drawLine(QPointF(3, 11), QPointF(8, 11));
        painter.setPen(QPen(accent, 1.5, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(8, 3), QPointF(8, 13));
        painter.drawLine(QPointF(11, 3), QPointF(11, 13));
        break;
    case ManagerGlyph::MPC:
        painter.drawEllipse(QPointF(4, 8), 2.2, 2.2);
        painter.drawEllipse(QPointF(12, 8), 2.2, 2.2);
        painter.drawLine(QPointF(6.3, 8), QPointF(9.7, 8));
        painter.setPen(QPen(accent, 1.6, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(QPointF(8, 4), QPointF(8, 12));
        break;
    }
    return QIcon(pixmap);
}

template <typename Map>
int nextId(const Map& values)
{
    return values.empty() ? 1 : values.rbegin()->first + 1;
}

template <typename Resource>
bool isEffectiveAtStep(const std::shared_ptr<Resource>& resource,
    int stepId, const std::shared_ptr<AnalysisStep>& targetStep)
{
    // StepId=0 表示初始/全局资源；更早分析步的资源也会被后续分析步继承。
    if (!resource)
        return false;
    const int sourceStepId = resource->m_StepId;
    if (sourceStepId <= 0 || sourceStepId == stepId)
        return true;
    // A dynamic step is a branch from its selected static equilibrium:
    // inherit its static history, but never a sibling dynamic branch.
    if (targetStep && targetStep->m_Type == EnumKeyword::StepType::DYNAMIC
        && targetStep->m_InitialStaticStepId > 0)
    {
        return sourceStepId <= targetStep->m_InitialStaticStepId;
    }
    return sourceStepId < stepId;
}

QString stepDisplayName(int id, const std::shared_ptr<AnalysisStep>& step)
{
    return step && !step->m_Name.trimmed().isEmpty()
        ? step->m_Name.trimmed() : QStringLiteral("Step-%1").arg(id);
}

class AdaptiveTssbnSettingsEditor final : public QGroupBox
{
public:
    using Settings = SolverNameSpace::AdaptiveTssbnSettings;

    explicit AdaptiveTssbnSettingsEditor(QWidget* parent)
        : QGroupBox(QStringLiteral("自适应 TSSBN 参数"), parent)
    {
        auto* form = new QGridLayout(this);
        form->setHorizontalSpacing(10);
        form->setVerticalSpacing(6);
        form->setColumnStretch(1, 1);
        form->setColumnStretch(3, 1);
        const std::array<FieldSpec, 13> fields{{
            { QStringLiteral("高频半径"), QStringLiteral("高频谱半径 ρ∞"), &Settings::spectralRadiusInfinity, 0.0, 1.0, 6 },
            { QStringLiteral("最小步长"), QStringLiteral("最小时间步"), &Settings::minimumTimeStep, 1.0e-12, 1.0e12, 12 },
            { QStringLiteral("最大步长"), QStringLiteral("最大时间步"), &Settings::maximumTimeStep, 1.0e-12, 1.0e12, 12 },
            { QStringLiteral("相对 LTE"), QStringLiteral("相对 LTE 容限"), &Settings::relativeTolerance, 1.0e-12, 1.0, 12 },
            { QStringLiteral("绝对 LTE"), QStringLiteral("绝对 LTE 容限"), &Settings::absoluteTolerance, 1.0e-14, 1.0, 14 },
            { QStringLiteral("安全系数"), QStringLiteral("步长安全系数"), &Settings::safetyFactor, 0.01, 1.0, 6 },
            { QStringLiteral("缩步系数"), QStringLiteral("最小缩步系数"), &Settings::shrinkFactor, 0.01, 0.99, 6 },
            { QStringLiteral("增长系数"), QStringLiteral("最大增长系数"), &Settings::maximumGrowthFactor, 1.0, 100.0, 6 },
            { QStringLiteral("牛顿目标"), QStringLiteral("目标牛顿迭代数"), &Settings::targetNewtonIterations, 1.0, 1000.0, 0 },
            { QStringLiteral("误差增益"), QStringLiteral("误差变化增益"), &Settings::derivativeGain, 0.0, 10.0, 6 },
            { QStringLiteral("因子下限"), QStringLiteral("误差变化因子下限"), &Settings::minimumDerivativeFactor, 0.01, 100.0, 6 },
            { QStringLiteral("因子上限"), QStringLiteral("误差变化因子上限"), &Settings::maximumDerivativeFactor, 0.01, 100.0, 6 },
            { QStringLiteral("拒绝上限"), QStringLiteral("最大连续拒绝次数"), &Settings::maximumRejectedAttempts, 1.0, 1000.0, 0 }
        }};

        for (std::size_t index = 0; index < fields.size(); ++index)
        {
            const FieldSpec& field = fields[index];
            QAbstractSpinBox* editor = nullptr;
            if (std::holds_alternative<DoubleMember>(field.member))
            {
                auto* spin = new CompactDoubleSpinBox(this);
                spin->setRange(field.minimum, field.maximum);
                spin->setDecimals(field.decimals);
                spin->setKeyboardTracking(false);
                editor = spin;
            }
            else
            {
                auto* spin = new QSpinBox(this);
                spin->setRange(
                    static_cast<int>(field.minimum),
                    static_cast<int>(field.maximum));
                editor = spin;
            }
            editor->setToolTip(field.toolTip);
            editor->setFixedWidth(112);
            bindings_.push_back({ field.member, editor });
            auto* label = new QLabel(field.label, this);
            label->setToolTip(field.toolTip);
            label->setFixedWidth(68);
            label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            const int row = static_cast<int>(index / 2);
            const int column = static_cast<int>(index % 2) * 2;
            form->addWidget(label, row, column);
            form->addWidget(editor, row, column + 1);
        }
        setSettings(Settings{});
    }

    void setSettings(const Settings& settings)
    {
        for (const Binding& binding : bindings_)
        {
            if (const auto member = std::get_if<DoubleMember>(&binding.member))
                static_cast<QDoubleSpinBox*>(binding.editor)->setValue(
                    settings.*(*member));
            else
                static_cast<QSpinBox*>(binding.editor)->setValue(
                    settings.*std::get<IntMember>(binding.member));
        }
    }

    Settings settings() const
    {
        Settings value;
        for (const Binding& binding : bindings_)
        {
            if (const auto member = std::get_if<DoubleMember>(&binding.member))
                value.*(*member) =
                    static_cast<QDoubleSpinBox*>(binding.editor)->value();
            else
                value.*std::get<IntMember>(binding.member) =
                    static_cast<QSpinBox*>(binding.editor)->value();
        }
        return value;
    }

private:
    using DoubleMember = double Settings::*;
    using IntMember = int Settings::*;
    using Member = std::variant<DoubleMember, IntMember>;

    struct FieldSpec
    {
        QString label;
        QString toolTip;
        Member member;
        double minimum;
        double maximum;
        int decimals;
    };

    struct Binding
    {
        Member member;
        QAbstractSpinBox* editor;
    };

    std::vector<Binding> bindings_;
};

class StepEditorDialog final : public QDialog
{
public:
    explicit StepEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
        const std::shared_ptr<AnalysisStep>& step, QWidget* parent)
        : QDialog(parent)
        , m_id(id)
    {
        Ui::StepEditorDialogClass form;
        form.setupUi(this);
        setObjectName(QStringLiteral("analysisStepEditorDialog"));
        setWindowTitle(step ? QStringLiteral("编辑分析步") : QStringLiteral("新增分析步"));
        setModal(true);
        m_name = form.nameEdit;
        m_name->setPlaceholderText(QStringLiteral("Step-%1").arg(id));
        m_type = form.typeCombo;
        m_type->setItemData(0, static_cast<int>(EnumKeyword::StepType::STATIC));
        m_type->setItemData(1, static_cast<int>(EnumKeyword::StepType::DYNAMIC));
        m_time = form.timeSpin;
        m_increment = form.incrementSpin;
        m_tolerance = form.toleranceSpin;
        m_iterations = form.iterationsSpin;
        // Rebuild the Designer form in a new container.  Moving its layout
        // directly leaves its label items owned by the old parent layout.
        auto* sharedParametersGroup = new QGroupBox(
            QString::fromUtf8("\xE9\x80\x9A\xE7\x94\xA8\xE5\x8F\x82\xE6\x95\xB0"), this);
        QLayoutItem* sharedFormItem = form.rootLayout->takeAt(0);
        auto* previousSharedLayout = sharedFormItem
            ? sharedFormItem->layout() : nullptr;
        if (previousSharedLayout)
        {
            previousSharedLayout->removeWidget(form.nameLabel);
            previousSharedLayout->removeWidget(m_name);
            previousSharedLayout->removeWidget(form.typeLabel);
            previousSharedLayout->removeWidget(m_type);
            previousSharedLayout->removeWidget(form.timeLabel);
            previousSharedLayout->removeWidget(m_time);
            previousSharedLayout->removeWidget(form.incrementLabel);
            previousSharedLayout->removeWidget(m_increment);
            previousSharedLayout->removeWidget(form.toleranceLabel);
            previousSharedLayout->removeWidget(m_tolerance);
            previousSharedLayout->removeWidget(form.iterationsLabel);
            previousSharedLayout->removeWidget(m_iterations);
        }
        auto* sharedParametersLayout = new QFormLayout(sharedParametersGroup);
        sharedParametersLayout->setFieldGrowthPolicy(
            QFormLayout::AllNonFixedFieldsGrow);
        sharedParametersLayout->addRow(form.nameLabel, m_name);
        sharedParametersLayout->addRow(form.typeLabel, m_type);
        sharedParametersLayout->addRow(form.timeLabel, m_time);
        sharedParametersLayout->addRow(form.incrementLabel, m_increment);
        sharedParametersLayout->addRow(form.toleranceLabel, m_tolerance);
        sharedParametersLayout->addRow(form.iterationsLabel, m_iterations);
        delete sharedFormItem;
        auto* columns = new QWidget(this);
        auto* columnsLayout = new QHBoxLayout(columns);
        columnsLayout->setContentsMargins(0, 0, 0, 0);
        columnsLayout->setSpacing(14);
        auto* commonColumn = new QWidget(columns);
        auto* commonColumnLayout = new QVBoxLayout(commonColumn);
        commonColumnLayout->setContentsMargins(0, 0, 0, 0);
        commonColumnLayout->setSpacing(14);
        commonColumnLayout->addWidget(sharedParametersGroup);
        commonColumn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        auto* solverColumn = new QWidget(columns);
        auto* solverColumnLayout = new QVBoxLayout(solverColumn);
        solverColumnLayout->setContentsMargins(0, 0, 0, 0);
        solverColumnLayout->setSpacing(14);
        solverColumn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        auto* tssbnColumn = new QWidget(columns);
        auto* tssbnColumnLayout = new QVBoxLayout(tssbnColumn);
        tssbnColumnLayout->setContentsMargins(0, 0, 0, 0);
        tssbnColumnLayout->setSpacing(14);
        tssbnColumn->setFixedWidth(430);
        tssbnColumn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        columnsLayout->addWidget(commonColumn, 1);
        columnsLayout->addWidget(solverColumn, 1);
        columnsLayout->addWidget(tssbnColumn, 0);
        columns->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
        form.rootLayout->insertWidget(0, columns);
        setMinimumWidth(760);
        auto* solverGroup = new QGroupBox(QStringLiteral("动力求解器"), this);
        auto* solverLayout = new QFormLayout(solverGroup);
        m_dynamicSolver = new QComboBox(solverGroup);
        m_dynamicSolver->addItem(
            QStringLiteral("Newmark-β"),
            static_cast<int>(SolverNameSpace::SolverType::Newmark));
        m_dynamicSolver->addItem(
            QStringLiteral("自适应 TSSBN"),
            static_cast<int>(SolverNameSpace::SolverType::AdaptiveTSSBN));
        solverLayout->addRow(QStringLiteral("积分方法"), m_dynamicSolver);
        auto* initialStateGroup = new QGroupBox(QStringLiteral("初始平衡状态"), this);
        auto* initialStateLayout = new QFormLayout(initialStateGroup);
        m_inheritStaticState = new QCheckBox(QStringLiteral("使用静力步作为初始状态"), initialStateGroup);
        m_initialStaticStep = new QComboBox(initialStateGroup);
        if (structure)
        {
            for (const auto& [candidateId, candidate] : structure->m_AnalysisStep)
            {
                if (candidateId >= id || !candidate ||
                    candidate->m_Type != EnumKeyword::StepType::STATIC)
                    continue;
                m_initialStaticStep->addItem(stepDisplayName(candidateId, candidate), candidateId);
            }
        }
        m_inheritStaticState->setEnabled(m_initialStaticStep->count() > 0);
        m_inheritStaticState->setToolTip(m_initialStaticStep->count() > 0
            ? QStringLiteral("先独立计算所选静力步，再复制其平衡构型和内力启动当前动力分支。"
                             "前置静力步中有效的重力及其他基础荷载会持续作用；不会包含其他动力步的荷载。")
            : QStringLiteral("当前模型没有可用的静力初态，请先创建静力分析步。"));
        initialStateLayout->addRow(m_inheritStaticState);
        initialStateLayout->addRow(QStringLiteral("初态来源"), m_initialStaticStep);
        m_inheritedBaseLoads = new QLabel(initialStateGroup);
        m_inheritedBaseLoads->setWordWrap(true);
        initialStateLayout->addRow(QStringLiteral("基础荷载"), m_inheritedBaseLoads);
        auto* gallopingGroup = new QGroupBox(QStringLiteral("舞动工况"), this);
        auto* gallopingLayout = new QFormLayout(gallopingGroup);
        m_enableGalloping = new QCheckBox(QStringLiteral("启用舞动气动力"), gallopingGroup);
        m_gallopingIceThickness = new QComboBox(gallopingGroup);
        for (int thickness : AeroManager::supportedIceThicknesses())
            m_gallopingIceThickness->addItem(QStringLiteral("%1 mm").arg(thickness), thickness);
        m_gallopingInitialAttack = new QDoubleSpinBox(gallopingGroup);
        m_gallopingInitialAttack->setRange(-1000000.0, 1000000.0);
        m_gallopingInitialAttack->setDecimals(6);
        m_gallopingInitialAttack->setValue(45.0);
        m_gallopingInitialAttack->setSuffix(QStringLiteral("°"));
        gallopingLayout->addRow(m_enableGalloping);
        gallopingLayout->addRow(QStringLiteral("覆冰厚度"), m_gallopingIceThickness);
        gallopingLayout->addRow(QStringLiteral("初始风攻角"), m_gallopingInitialAttack);
        auto* regionGroup = new QGroupBox(QStringLiteral("计算区域"), this);
        auto* regionLayout = new QVBoxLayout(regionGroup);
        m_allRegions = new QCheckBox(QStringLiteral("全部启用计算区域"), regionGroup);
        m_regionList = new QListWidget(regionGroup);
        regionLayout->addWidget(m_allRegions);
        regionLayout->addWidget(m_regionList);
        commonColumnLayout->addWidget(regionGroup);

        // The first two columns are always present.  Adaptive TSSBN opens a
        // dedicated third column to their right instead of increasing height.
        solverColumnLayout->addWidget(solverGroup);
        solverColumnLayout->addWidget(initialStateGroup);
        solverColumnLayout->addWidget(gallopingGroup);
        solverColumnLayout->addStretch(1);
        m_tssbnEditor = new AdaptiveTssbnSettingsEditor(tssbnColumn);
        auto* tssbnGroup = m_tssbnEditor;
        tssbnColumnLayout->addWidget(tssbnGroup);
        tssbnColumnLayout->addStretch(1);
        if (structure)
        {
            for (const auto& [regionId, region] : structure->m_ComputeRegions)
            {
                if (!region)
                    continue;
                auto* item = new QListWidgetItem(
                    QStringLiteral("%1（节点 %2，单元 %3）")
                        .arg(region->m_Name).arg(region->m_NodeIds.size()).arg(region->m_ElementIds.size()),
                    m_regionList);
                item->setData(Qt::UserRole, regionId);
                item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
                item->setCheckState(Qt::Unchecked);
            }
        }
        // Region selection is normally a short list.  Keep it sized to its
        // rows (up to four) instead of letting it stretch to the dialog bottom.
        const int regionRows = std::clamp(m_regionList->count(), 1, 4);
        const int regionRowHeight = std::max(26, m_regionList->sizeHintForRow(0));
        m_regionList->setFixedHeight(regionRows * regionRowHeight
            + 2 * m_regionList->frameWidth() + 4);
        connect(m_allRegions, &QCheckBox::toggled, m_regionList, &QListWidget::setDisabled);

        const auto resizeToVisibleContent = [this, columns]()
        {
            // Visibility changes are fully processed on the next event-loop
            // turn.  Measuring then makes both expansion and collapse reliable.
            QTimer::singleShot(0, this, [this, columns]()
            {
                columns->updateGeometry();
                if (auto* rootLayout = layout())
                {
                    rootLayout->invalidate();
                    rootLayout->activate();
                }
                setMinimumHeight(0);
                setMaximumHeight(QWIDGETSIZE_MAX);
                const QSize target = sizeHint();
                resize(std::max(minimumWidth(), target.width()), target.height());
            });
        };
        const auto updateInheritedBaseLoads = [this, structure]()
        {
            if (!m_inheritStaticState->isChecked() || m_initialStaticStep->currentIndex() < 0)
            {
                m_inheritedBaseLoads->setText(QStringLiteral("未继承前置静力荷载"));
                return;
            }
            const int sourceStepId = m_initialStaticStep->currentData().toInt();
            int gravityCount = 0;
            int otherLoadCount = 0;
            if (structure)
            {
                for (const auto& [loadId, load] : structure->m_Load)
                {
                    Q_UNUSED(loadId);
                    if (!load || (load->m_StepId > 0 && load->m_StepId > sourceStepId))
                        continue;
                    if (std::dynamic_pointer_cast<Force_Gravity>(load))
                        ++gravityCount;
                    else
                        ++otherLoadCount;
                }
            }
            QStringList parts;
            if (gravityCount > 0)
                parts.push_back(QStringLiteral("重力 ×%1（持续作用）").arg(gravityCount));
            if (otherLoadCount > 0)
                parts.push_back(QStringLiteral("其他前置荷载 ×%1").arg(otherLoadCount));
            m_inheritedBaseLoads->setText(parts.isEmpty()
                ? QStringLiteral("未发现前置基础荷载")
                : parts.join(QStringLiteral("；")));
        };
        const auto updateDynamicState =
            [this, commonColumn, solverColumn, solverGroup, tssbnGroup,
                tssbnColumn, gallopingGroup, initialStateGroup,
                resizeToVisibleContent, updateInheritedBaseLoads]()
        {
            const bool dynamic = static_cast<EnumKeyword::StepType>(
                m_type->currentData().toInt()) == EnumKeyword::StepType::DYNAMIC;
            solverGroup->setEnabled(dynamic);
            initialStateGroup->setEnabled(dynamic);
            m_initialStaticStep->setEnabled(dynamic
                && m_inheritStaticState->isChecked()
                && m_initialStaticStep->count() > 0);
            const bool adaptiveTssbn =
                dynamic
                && static_cast<SolverNameSpace::SolverType>(
                    m_dynamicSolver->currentData().toInt())
                    == SolverNameSpace::SolverType::AdaptiveTSSBN;
            // On TSSBN, preserve the dynamic-control column's normal reading
            // width and make room by compacting only the common left column.
            commonColumn->setMaximumWidth(
                adaptiveTssbn ? 320 : QWIDGETSIZE_MAX);
            solverColumn->setMinimumWidth(adaptiveTssbn ? 440 : 0);
            tssbnColumn->setVisible(adaptiveTssbn);
            tssbnGroup->setEnabled(adaptiveTssbn);
            gallopingGroup->setEnabled(dynamic);
            m_gallopingIceThickness->setEnabled(m_enableGalloping->isChecked());
            m_gallopingInitialAttack->setEnabled(m_enableGalloping->isChecked());
            updateInheritedBaseLoads();
            resizeToVisibleContent();
        };
        connect(m_type, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [updateDynamicState](int) { updateDynamicState(); });
        connect(m_dynamicSolver,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this, [updateDynamicState](int) { updateDynamicState(); });
        connect(m_enableGalloping, &QCheckBox::toggled,
            this, [updateDynamicState](bool) { updateDynamicState(); });
        connect(m_inheritStaticState, &QCheckBox::toggled,
            this, [updateDynamicState](bool) { updateDynamicState(); });
        connect(m_initialStaticStep, qOverload<int>(&QComboBox::currentIndexChanged),
            this, [updateDynamicState](int) { updateDynamicState(); });
        m_time->setRange(1.0e-12, 1.0e12);
        m_increment->setRange(1.0e-12, 1.0e12);
        m_tolerance->setRange(1.0e-14, 1.0);
        auto* buttons = form.buttonBox;
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        connect(buttons, &QDialogButtonBox::accepted, this, [this]()
        {
            const bool adaptiveTssbn =
                static_cast<EnumKeyword::StepType>(
                    m_type->currentData().toInt())
                    == EnumKeyword::StepType::DYNAMIC
                && static_cast<SolverNameSpace::SolverType>(
                    m_dynamicSolver->currentData().toInt())
                    == SolverNameSpace::SolverType::AdaptiveTSSBN;
            const auto tssbn = m_tssbnEditor->settings();
            if (adaptiveTssbn
                && (tssbn.minimumTimeStep > tssbn.maximumTimeStep
                    || m_increment->value() < tssbn.minimumTimeStep
                    || m_increment->value() > tssbn.maximumTimeStep))
            {
                QMessageBox::warning(
                    this,
                    QStringLiteral("TSSBN 参数无效"),
                    QStringLiteral(
                        "必须满足：最小时间步 ≤ 初始时间步 ≤ 最大时间步。"));
                return;
            }
            const bool dynamic = static_cast<EnumKeyword::StepType>(
                m_type->currentData().toInt()) == EnumKeyword::StepType::DYNAMIC;
            if (dynamic && m_inheritStaticState->isChecked()
                && m_initialStaticStep->currentIndex() < 0)
            {
                QMessageBox::information(this, QStringLiteral("缺少前置静力步"),
                    QStringLiteral("请先创建静力分析步，或取消“使用静力步作为初始状态”。"));
                return;
            }
            if (adaptiveTssbn
                && tssbn.minimumDerivativeFactor
                    > tssbn.maximumDerivativeFactor)
            {
                QMessageBox::warning(
                    this,
                    QStringLiteral("TSSBN 参数无效"),
                    QStringLiteral("误差变化因子下限不能大于上限。"));
                return;
            }
            if (!m_allRegions->isChecked())
            {
                bool hasCheckedRegion = false;
                for (int row = 0; row < m_regionList->count(); ++row)
                {
                    if (m_regionList->item(row)->checkState() == Qt::Checked)
                    {
                        hasCheckedRegion = true;
                        break;
                    }
                }
                if (!hasCheckedRegion)
                {
                    QMessageBox::information(this, QStringLiteral("请选择计算区域"),
                        QStringLiteral("分析步至少需要选择一个计算区域。"));
                    return;
                }
            }
            accept();
        });
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        if (step)
        {
            m_name->setText(step->m_Name);
            m_type->setCurrentIndex(qMax(0, m_type->findData(static_cast<int>(step->m_Type))));
            m_time->setValue(step->m_Time);
            m_increment->setValue(step->m_StepSize);
            m_tolerance->setValue(step->m_Tolerance);
            m_iterations->setValue(step->m_MaxIterations);
            m_dynamicSolver->setCurrentIndex(qMax(0,
                m_dynamicSolver->findData(
                    static_cast<int>(step->m_DynamicSolverType))));
            const int inheritedIndex = m_initialStaticStep->findData(step->m_InitialStaticStepId);
            m_inheritStaticState->setChecked(step->m_Type == EnumKeyword::StepType::DYNAMIC
                && inheritedIndex >= 0);
            if (inheritedIndex >= 0)
                m_initialStaticStep->setCurrentIndex(inheritedIndex);
            m_tssbnEditor->setSettings(step->m_AdaptiveTssbn);
            m_enableGalloping->setChecked(step->m_EnableGalloping);
            m_gallopingIceThickness->setCurrentIndex(qMax(0,
                m_gallopingIceThickness->findData(step->m_GallopingIceThickness)));
            m_gallopingInitialAttack->setValue(
                step->m_GallopingInitialAttackDegrees);
            m_allRegions->setChecked(step->m_RegionScope == AnalysisRegionScope::AllEnabledRegions);
            for (int row = 0; row < m_regionList->count(); ++row)
            {
                auto* item = m_regionList->item(row);
                const int regionId = item->data(Qt::UserRole).toInt();
                item->setCheckState(step->m_ComputeRegionIds.find(regionId)
                    != step->m_ComputeRegionIds.cend() ? Qt::Checked : Qt::Unchecked);
            }
        }
        else
        {
            m_time->setValue(1.0);
            m_increment->setValue(0.01);
            m_tolerance->setValue(1.0e-5);
            m_iterations->setValue(50);
            m_dynamicSolver->setCurrentIndex(
                m_dynamicSolver->findData(
                    static_cast<int>(
                        SolverNameSpace::SolverType::Newmark)));
            if (m_initialStaticStep->count() > 0)
            {
                m_initialStaticStep->setCurrentIndex(m_initialStaticStep->count() - 1);
                m_inheritStaticState->setChecked(true);
            }
            m_allRegions->setChecked(true);
        }
        updateDynamicState();
    }

    AnalysisStepConfig config() const
    {
        AnalysisStepConfig value;
        value.id = m_id;
        value.name = m_name->text().trimmed();
        value.type = static_cast<EnumKeyword::StepType>(m_type->currentData().toInt());
        value.totalTime = m_time->value();
        value.stepSize = m_increment->value();
        value.tolerance = m_tolerance->value();
        value.maxIterations = m_iterations->value();
        value.dynamicSolverType =
            value.type == EnumKeyword::StepType::DYNAMIC
            ? static_cast<SolverNameSpace::SolverType>(
                m_dynamicSolver->currentData().toInt())
            : SolverNameSpace::SolverType::Newmark;
        value.initialStaticStepId = value.type == EnumKeyword::StepType::DYNAMIC
            && m_inheritStaticState->isChecked()
            ? m_initialStaticStep->currentData().toInt() : 0;
        value.adaptiveTssbn = m_tssbnEditor->settings();
        value.enableGalloping = value.type == EnumKeyword::StepType::DYNAMIC
            && m_enableGalloping->isChecked();
        value.gallopingIceThickness = m_gallopingIceThickness->currentData().toInt();
        value.gallopingInitialAttackDegrees =
            m_gallopingInitialAttack->value();
        value.regionScope = m_allRegions->isChecked()
            ? AnalysisRegionScope::AllEnabledRegions : AnalysisRegionScope::SelectedRegions;
        for (int row = 0; row < m_regionList->count(); ++row)
        {
            const auto* item = m_regionList->item(row);
            if (item->checkState() == Qt::Checked)
                value.computeRegionIds.insert(item->data(Qt::UserRole).toInt());
        }
        return value;
    }

private:
    int m_id = 0;
    QLineEdit* m_name = nullptr;
    QComboBox* m_type = nullptr;
    QDoubleSpinBox* m_time = nullptr;
    QDoubleSpinBox* m_increment = nullptr;
    QDoubleSpinBox* m_tolerance = nullptr;
    QSpinBox* m_iterations = nullptr;
    QComboBox* m_dynamicSolver = nullptr;
    QCheckBox* m_inheritStaticState = nullptr;
    QComboBox* m_initialStaticStep = nullptr;
    QLabel* m_inheritedBaseLoads = nullptr;
    AdaptiveTssbnSettingsEditor* m_tssbnEditor = nullptr;
    QCheckBox* m_enableGalloping = nullptr;
    QComboBox* m_gallopingIceThickness = nullptr;
    QDoubleSpinBox* m_gallopingInitialAttack = nullptr;
    QCheckBox* m_allRegions = nullptr;
    QListWidget* m_regionList = nullptr;
};

QString directionName(EnumKeyword::Direction direction)
{
    return EnumKeyword::MapDirection.key(direction, QStringLiteral("--"));
}

QString loadTypeName(EnumKeyword::LoadType type)
{
    switch (type)
    {
    case EnumKeyword::LoadType::FORCE_NODE: return QStringLiteral("节点力");
    case EnumKeyword::LoadType::FORCE_ELEMENT: return QStringLiteral("单元荷载");
    case EnumKeyword::LoadType::FORCE_GRAVITY: return QStringLiteral("重力");
    case EnumKeyword::LoadType::FORCE_WIND: return QStringLiteral("风荷载");
    default: return QStringLiteral("未知");
    }
}

QString loadDisplayName(int id, const std::shared_ptr<LoadBase>& load)
{
    return load && !load->m_Name.trimmed().isEmpty()
        ? load->m_Name.trimmed() : QStringLiteral("Load-%1").arg(id);
}

int loadTargetId(const std::shared_ptr<LoadBase>& load)
{
    if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(load))
    {
        const auto node = nodeLoad->m_pNode.lock();
        return node ? node->m_Id : -1;
    }
    if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(load))
    {
        const auto element = elementLoad->m_pElement.lock();
        return element ? element->m_Id : -1;
    }
    return -1;
}

double loadValue(const std::shared_ptr<LoadBase>& load)
{
    if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(load))
        return nodeLoad->m_Value;
    if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(load))
        return elementLoad->m_Value;
    if (const auto gravity = std::dynamic_pointer_cast<Force_Gravity>(load))
        return gravity->m_g;
    if (const auto wind = std::dynamic_pointer_cast<Force_Wind>(load))
        return wind->m_velocity;
    return 0.0;
}

class LoadEditorDialog final : public QDialog
{
public:
    LoadEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
        const std::shared_ptr<LoadBase>& load, QWidget* parent)
        : QDialog(parent)
        , m_id(id)
        , m_structure(structure)
        , m_original(load)
    {
        Ui::LoadEditorDialogClass form;
        form.setupUi(this);
        setObjectName(QStringLiteral("analysisLoadEditorDialog"));
        setWindowTitle(load ? QStringLiteral("编辑荷载") : QStringLiteral("新增荷载"));
        setModal(true);
        DialogSizing::lockHeightToContents(this);
        m_name = form.nameEdit;
        m_name->setPlaceholderText(QStringLiteral("Load-%1").arg(id));
        m_step = form.stepCombo;
        m_step->addItem(QStringLiteral("初始 / 全局"), 0);
        for (const auto& [stepId, step] : structure->m_AnalysisStep)
            if (step)
                m_step->addItem(stepDisplayName(stepId, step), stepId);
        m_type = form.typeCombo;
        m_type->setItemData(0, static_cast<int>(EnumKeyword::LoadType::FORCE_NODE));
        m_type->setItemData(1, static_cast<int>(EnumKeyword::LoadType::FORCE_ELEMENT));
        m_type->setItemData(2, static_cast<int>(EnumKeyword::LoadType::FORCE_GRAVITY));
        m_type->setItemData(3, static_cast<int>(EnumKeyword::LoadType::FORCE_WIND));
        m_target = form.targetSpin;
        m_direction = form.directionCombo;
        for (int value = static_cast<int>(EnumKeyword::Direction::X);
             value <= static_cast<int>(EnumKeyword::Direction::RZ); ++value)
        {
            const auto direction = static_cast<EnumKeyword::Direction>(value);
            m_direction->addItem(directionName(direction), value);
        }
        m_directionLabel = form.directionLabel;
        m_directionStack = new QStackedWidget(this);
        form.formLayout->removeWidget(m_direction);
        m_directionStack->addWidget(m_direction);
        auto* windDirectionWidget = new QWidget(m_directionStack);
        auto* windDirectionLayout = new QHBoxLayout(windDirectionWidget);
        windDirectionLayout->setContentsMargins(0, 0, 0, 0);
        for (int component = 0; component < 3; ++component)
        {
            m_windDirection[component] = new QDoubleSpinBox(windDirectionWidget);
            m_windDirection[component]->setRange(-1.0e6, 1.0e6);
            m_windDirection[component]->setDecimals(6);
            m_windDirection[component]->setPrefix(
                QString(component == 0 ? "X " : component == 1 ? "Y " : "Z "));
            windDirectionLayout->addWidget(m_windDirection[component]);
        }
        // Conductor X / vertical Z uses -Y as the reference inflow.  This is
        // the right-handed mapping of the legacy TSSBN X/Y(vertical)/Z(wind)
        // convention and makes positive Cl point toward +Z.
        m_windDirection[1]->setValue(-1.0);
        m_directionStack->addWidget(windDirectionWidget);
        form.formLayout->setWidget(4, QFormLayout::FieldRole, m_directionStack);
        m_value = form.valueSpin;
        m_valueLabel = form.valueLabel;
        m_valueStack = new QStackedWidget(this);
        form.formLayout->removeWidget(m_value);
        m_valueStack->addWidget(m_value);
        m_windSpeed = new QComboBox(m_valueStack);
        for (int speed : AeroManager::supportedWindSpeeds())
            m_windSpeed->addItem(QStringLiteral("%1 m/s").arg(speed), speed);
        m_valueStack->addWidget(m_windSpeed);
        form.formLayout->setWidget(5, QFormLayout::FieldRole, m_valueStack);
        auto* buttons = form.buttonBox;
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        connect(buttons, &QDialogButtonBox::accepted, this, &LoadEditorDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(m_type, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int) { updateTargetState(); });
        if (load)
        {
            m_name->setText(load->m_Name);
            int stepIndex = m_step->findData(load->m_StepId);
            if (stepIndex < 0)
            {
                m_step->addItem(QStringLiteral("Step-%1（缺失）").arg(load->m_StepId), load->m_StepId);
                stepIndex = m_step->count() - 1;
            }
            m_step->setCurrentIndex(stepIndex);
            m_type->setCurrentIndex(qMax(0, m_type->findData(static_cast<int>(load->m_LoadType))));
            m_direction->setCurrentIndex(qMax(0,
                m_direction->findData(static_cast<int>(load->m_Direction))));
            const int targetId = loadTargetId(load);
            if (targetId > 0)
                m_target->setValue(targetId);
            m_value->setValue(loadValue(load));
            if (const auto wind = std::dynamic_pointer_cast<Force_Wind>(load))
            {
                for (int component = 0; component < 3; ++component)
                    m_windDirection[component]->setValue(wind->m_direction[component]);
                int windIndex = m_windSpeed->findData(wind->m_velocity);
                if (windIndex < 0)
                {
                    m_windSpeed->addItem(
                        QStringLiteral("%1 m/s（旧模型，不支持舞动）").arg(wind->m_velocity),
                        wind->m_velocity);
                    windIndex = m_windSpeed->count() - 1;
                }
                m_windSpeed->setCurrentIndex(windIndex);
            }
        }
        else
        {
            if (m_step->count() > 1)
                m_step->setCurrentIndex(1);
            if (!structure->m_Nodes.empty())
                m_target->setValue(structure->m_Nodes.begin()->first);
        }
        updateTargetState();
    }

    std::shared_ptr<LoadBase> value() const
    {
        return m_result;
    }

protected:
    void accept() override
    {
        const auto type = static_cast<EnumKeyword::LoadType>(m_type->currentData().toInt());
        std::shared_ptr<LoadBase> result;
        if (type == EnumKeyword::LoadType::FORCE_NODE)
        {
            const auto node = m_structure->FindNode(m_target->value());
            if (!node)
            {
                QMessageBox::warning(this, QStringLiteral("无效目标"),
                    QStringLiteral("节点 %1 不存在。").arg(m_target->value()));
                return;
            }
            auto item = std::make_shared<Force_Node>();
            item->m_pNode = node;
            item->m_Value = m_value->value();
            result = item;
        }
        else if (type == EnumKeyword::LoadType::FORCE_ELEMENT)
        {
            const auto element = m_structure->FindElement(m_target->value());
            if (!element)
            {
                QMessageBox::warning(this, QStringLiteral("无效目标"),
                    QStringLiteral("单元 %1 不存在。").arg(m_target->value()));
                return;
            }
            auto item = std::make_shared<Force_Element>();
            item->m_pElement = element;
            item->m_Value = m_value->value();
            result = item;
        }
        else if (type == EnumKeyword::LoadType::FORCE_GRAVITY)
        {
            auto item = std::make_shared<Force_Gravity>();
            item->m_g = m_value->value();
            result = item;
        }
        else if (type == EnumKeyword::LoadType::FORCE_WIND)
        {
            const Eigen::Vector3d direction(
                m_windDirection[0]->value(),
                m_windDirection[1]->value(),
                m_windDirection[2]->value());
            if (!direction.allFinite() || direction.norm() <= 1.0e-12)
            {
                QMessageBox::warning(this, QStringLiteral("无效风向"),
                    QStringLiteral("全局风向向量不能为零。"));
                return;
            }
            auto item = std::make_shared<Force_Wind>();
            item->m_velocity = m_windSpeed->currentData().toDouble();
            item->m_direction = direction.normalized();
            result = item;
        }
        if (!result)
            return;
        result->m_Id = m_id;
        result->m_Name = m_name->text().trimmed();
        result->m_StepId = m_step->currentData().toInt();
        result->m_Direction = type == EnumKeyword::LoadType::FORCE_WIND
            ? EnumKeyword::Direction::UNKNOWN
            : static_cast<EnumKeyword::Direction>(m_direction->currentData().toInt());
        if (m_original)
        {
            result->m_StartTime = m_original->m_StartTime;
            result->m_EndTime = m_original->m_EndTime;
            result->m_FunctionType = m_original->m_FunctionType;
            result->m_Amplitude = m_original->m_Amplitude;
            result->m_Frequency = m_original->m_Frequency;
            result->m_Phase = m_original->m_Phase;
            result->m_Offset = m_original->m_Offset;
            result->m_RampT0 = m_original->m_RampT0;
            result->m_RampT1 = m_original->m_RampT1;
            result->m_Decay = m_original->m_Decay;
            result->m_Period = m_original->m_Period;
            result->m_DutyCycle = m_original->m_DutyCycle;
            const auto oldWind = std::dynamic_pointer_cast<Force_Wind>(m_original);
            const auto newWind = std::dynamic_pointer_cast<Force_Wind>(result);
            if (oldWind && newWind)
                newWind->m_windDensity = oldWind->m_windDensity;
        }
        m_result = result;
        QDialog::accept();
    }

private:
    void updateTargetState()
    {
        const auto type = static_cast<EnumKeyword::LoadType>(m_type->currentData().toInt());
        const bool hasTarget = type == EnumKeyword::LoadType::FORCE_NODE
            || type == EnumKeyword::LoadType::FORCE_ELEMENT;
        m_target->setEnabled(hasTarget);
        m_target->setToolTip(type == EnumKeyword::LoadType::FORCE_NODE
            ? QStringLiteral("节点编号") : type == EnumKeyword::LoadType::FORCE_ELEMENT
            ? QStringLiteral("单元编号") : QStringLiteral("该荷载作用于整个模型"));
        const bool wind = type == EnumKeyword::LoadType::FORCE_WIND;
        m_directionStack->setCurrentIndex(wind ? 1 : 0);
        m_directionLabel->setText(wind ? QStringLiteral("全局风向") : QStringLiteral("方向"));
        m_valueStack->setCurrentIndex(wind ? 1 : 0);
        m_valueLabel->setText(wind ? QStringLiteral("风速") : QStringLiteral("数值"));
    }

    int m_id = 0;
    std::shared_ptr<StructureData> m_structure;
    std::shared_ptr<LoadBase> m_original;
    std::shared_ptr<LoadBase> m_result;
    QLineEdit* m_name = nullptr;
    QComboBox* m_step = nullptr;
    QComboBox* m_type = nullptr;
    QSpinBox* m_target = nullptr;
    QComboBox* m_direction = nullptr;
    QLabel* m_directionLabel = nullptr;
    QStackedWidget* m_directionStack = nullptr;
    QDoubleSpinBox* m_windDirection[3] = {};
    QDoubleSpinBox* m_value = nullptr;
    QLabel* m_valueLabel = nullptr;
    QStackedWidget* m_valueStack = nullptr;
    QComboBox* m_windSpeed = nullptr;
};

QString constraintDisplayName(int id, const std::shared_ptr<Constraint>& constraint)
{
    return constraint && !constraint->m_Name.trimmed().isEmpty()
        ? constraint->m_Name.trimmed() : QStringLiteral("Constraint-%1").arg(id);
}

QString mpcDisplayName(
    int id, const std::shared_ptr<NonlinearMPCConstraint>& mpc)
{
    return mpc && !mpc->m_Name.trimmed().isEmpty()
        ? mpc->m_Name.trimmed() : QStringLiteral("MPC-%1").arg(id);
}

QString mpcDirectionText(
    const std::shared_ptr<NonlinearMPCConstraint>& mpc)
{
    QString result;
    if (mpc)
        for (const int direction : mpc->m_SlaveDirections)
            result.append(QString::number(direction));
    return result;
}

QString mpcRelationText(
    const std::shared_ptr<NonlinearMPCConstraint>& mpc)
{
    if (std::dynamic_pointer_cast<DistanceMPCConstraint>(mpc))
        return QStringLiteral("定长");
    if (std::dynamic_pointer_cast<RigidOffsetMPCConstraint>(mpc))
        return QStringLiteral("刚性偏置");
    if (std::dynamic_pointer_cast<PlanarShearReleaseMPCConstraint>(mpc))
        return QStringLiteral("非线性剪力释放");
    return QStringLiteral("非线性关系");
}

class MPCEditorDialog final : public QDialog
{
public:
    MPCEditorDialog(
        int id,
        const std::shared_ptr<StructureData>& structure,
        const std::shared_ptr<NonlinearMPCConstraint>& mpc,
        QWidget* parent)
        : QDialog(parent)
        , m_id(id)
        , m_structure(structure)
    {
        setObjectName(QStringLiteral("analysisMPCEditorDialog"));
        setWindowTitle(mpc ? QStringLiteral("编辑 MPC") : QStringLiteral("新增 MPC"));
        setModal(true);
        resize(480, 310);
        DialogSizing::lockHeightToContents(this);

        auto* root = new QVBoxLayout(this);
        auto* form = new QFormLayout();
        m_name = new QLineEdit(this);
        m_name->setPlaceholderText(QStringLiteral("MPC-%1").arg(id));
        m_step = new QComboBox(this);
        m_step->addItem(QStringLiteral("初始 / 全局"), 0);
        if (structure)
        {
            for (const auto& [stepId, step] : structure->m_AnalysisStep)
                if (step)
                    m_step->addItem(stepDisplayName(stepId, step), stepId);
        }
        m_master = new QSpinBox(this);
        m_slave = new QSpinBox(this);
        m_master->setRange(1, std::numeric_limits<int>::max());
        m_slave->setRange(1, std::numeric_limits<int>::max());
        m_direction = new QComboBox(this);
        m_direction->setEditable(true);
        m_direction->addItems({
            QStringLiteral("0"), QStringLiteral("1"), QStringLiteral("2"),
            QStringLiteral("05"), QStringLiteral("012") });
        m_direction->setToolTip(QStringLiteral(
            "输入从节点消元自由度。当前求解器支持单个平动方向 0/1/2，"
            "非线性剪力释放 05，以及完整平动方向 012。"));
        form->addRow(QStringLiteral("名称"), m_name);
        form->addRow(QStringLiteral("生效分析步"), m_step);
        form->addRow(QStringLiteral("主节点"), m_master);
        form->addRow(QStringLiteral("从节点"), m_slave);
        form->addRow(QStringLiteral("从节点方向"), m_direction);
        root->addLayout(form);

        auto* hint = new QLabel(QStringLiteral(
            "关系由程序根据 SlaveDirection 自动建立：单方向使用初始定长关系，"
            "05 使用论文非线性剪力释放关系，012 使用大转动刚性偏置关系。"), this);
        hint->setWordWrap(true);
        hint->setObjectName(QStringLiteral("analysisMPCHint"));
        root->addWidget(hint);
        auto* buttons = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        root->addWidget(buttons);
        connect(buttons, &QDialogButtonBox::accepted, this, &MPCEditorDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

        if (mpc)
        {
            m_name->setText(mpc->m_Name);
            int stepIndex = m_step->findData(mpc->m_StepId);
            if (stepIndex < 0)
            {
                m_step->addItem(
                    QStringLiteral("Step-%1（缺失）").arg(mpc->m_StepId),
                    mpc->m_StepId);
                stepIndex = m_step->count() - 1;
            }
            m_step->setCurrentIndex(stepIndex);
            const std::vector<int> nodeIds = mpc->GetNodeIds();
            if (nodeIds.size() == 2)
            {
                m_master->setValue(nodeIds[0]);
                m_slave->setValue(nodeIds[1]);
            }
            m_direction->setCurrentText(mpcDirectionText(mpc));
        }
        else if (structure && !structure->m_Nodes.empty())
        {
            m_master->setValue(structure->m_Nodes.cbegin()->first);
            auto slaveIt = structure->m_Nodes.cbegin();
            if (slaveIt != structure->m_Nodes.cend())
                ++slaveIt;
            m_slave->setValue(slaveIt != structure->m_Nodes.cend()
                ? slaveIt->first : structure->m_Nodes.cbegin()->first);
        }
    }

    std::shared_ptr<NonlinearMPCConstraint> value() const { return m_result; }

protected:
    void accept() override
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
            if (direction < 0 || direction > 5
                || !uniqueDirections.insert(direction).second)
            {
                QMessageBox::warning(this, QStringLiteral("无效从节点方向"),
                    QStringLiteral("SlaveDirection 包含越界或重复自由度。"));
                return;
            }
            directions.push_back(direction);
        }
        if (!((directions.size() == 1 && directions.front() < 3)
            || directions == std::vector<int>({0, 5})
            || directions == std::vector<int>({0, 1, 2})))
        {
            QMessageBox::information(this, QStringLiteral("当前求解范围"),
                QStringLiteral(
                    "当前阶段支持单个平动方向 0/1/2、非线性剪力释放 05，"
                    "或完整平动方向 012。"));
            return;
        }
        for (const int direction : directions)
        {
            if (direction >= slave->m_DOF.size())
            {
                QMessageBox::warning(this, QStringLiteral("从自由度越界"),
                    QStringLiteral("节点 %1 没有自由度 %2。")
                        .arg(slave->m_Id).arg(direction));
                return;
            }
            for (const auto& [constraintId, constraint] :
                 m_structure->m_Constraint)
            {
                const auto constrainedNode =
                    constraint ? constraint->m_pNode.lock() : nullptr;
                if (constrainedNode
                    && constrainedNode->m_Id == slave->m_Id
                    && static_cast<int>(constraint->m_Direction) == direction)
                {
                    QMessageBox::warning(this, QStringLiteral("从自由度冲突"),
                        QStringLiteral(
                            "节点 %1 的自由度 %2 已被位移约束 %3 固定。")
                            .arg(slave->m_Id).arg(direction).arg(constraintId));
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
                if (std::find(other->m_SlaveDirections.cbegin(),
                        other->m_SlaveDirections.cend(), direction)
                    != other->m_SlaveDirections.cend())
                {
                    QMessageBox::warning(this, QStringLiteral("从自由度重复"),
                        QStringLiteral("节点 %1 的自由度 %2 已被 MPC %3 使用。")
                            .arg(slave->m_Id).arg(direction).arg(otherId));
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
            distance->m_Length = Eigen::Vector3d(
                slave->m_X - master->m_X,
                slave->m_Y - master->m_Y,
                slave->m_Z - master->m_Z).norm();
            result = std::move(distance);
        }
        else if (directions == std::vector<int>({0, 5}))
        {
            auto release =
                std::make_shared<PlanarShearReleaseMPCConstraint>();
            release->m_pMasterNode = master;
            release->m_pSlaveNode = slave;
            result = std::move(release);
        }
        else
        {
            auto rigid = std::make_shared<RigidOffsetMPCConstraint>();
            rigid->m_pMasterNode = master;
            rigid->m_pSlaveNode = slave;
            rigid->m_Offset = Eigen::Vector3d(
                slave->m_X - master->m_X,
                slave->m_Y - master->m_Y,
                slave->m_Z - master->m_Z);
            result = std::move(rigid);
        }
        result->m_Id = m_id;
        result->m_Name = m_name->text().trimmed();
        result->m_StepId = m_step->currentData().toInt();
        result->m_SlaveDirections = std::move(directions);
        m_result = std::move(result);
        QDialog::accept();
    }

private:
    int m_id = 0;
    std::shared_ptr<StructureData> m_structure;
    std::shared_ptr<NonlinearMPCConstraint> m_result;
    QLineEdit* m_name = nullptr;
    QComboBox* m_step = nullptr;
    QSpinBox* m_master = nullptr;
    QSpinBox* m_slave = nullptr;
    QComboBox* m_direction = nullptr;
};

class ConstraintEditorDialog final : public QDialog
{
public:
    ConstraintEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
        const std::shared_ptr<Constraint>& constraint, QWidget* parent)
        : QDialog(parent)
        , m_structure(structure)
    {
        Ui::ConstraintEditorDialogClass form;
        form.setupUi(this);
        setObjectName(QStringLiteral("analysisConstraintEditorDialog"));
        setWindowTitle(constraint ? QStringLiteral("编辑约束") : QStringLiteral("新增约束"));
        setModal(true);
        DialogSizing::lockHeightToContents(this);
        m_name = form.nameEdit;
        m_name->setPlaceholderText(QStringLiteral("Constraint-%1").arg(id));
        m_step = form.stepCombo;
        m_step->addItem(QStringLiteral("初始 / 全局"), 0);
        for (const auto& [stepId, step] : structure->m_AnalysisStep)
            if (step)
                m_step->addItem(stepDisplayName(stepId, step), stepId);
        m_node = form.nodeSpin;
        m_direction = form.directionCombo;
        for (int value = static_cast<int>(EnumKeyword::Direction::X);
             value <= static_cast<int>(EnumKeyword::Direction::RZ); ++value)
        {
            const auto direction = static_cast<EnumKeyword::Direction>(value);
            m_direction->addItem(directionName(direction), value);
        }
        m_value = form.valueSpin;
        auto* buttons = form.buttonBox;
        buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        connect(buttons, &QDialogButtonBox::accepted, this, &ConstraintEditorDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        if (constraint)
        {
            m_name->setText(constraint->m_Name);
            m_step->setCurrentIndex(qMax(0, m_step->findData(constraint->m_StepId)));
            const auto node = constraint->m_pNode.lock();
            if (node)
                m_node->setValue(node->m_Id);
            m_direction->setCurrentIndex(qMax(0,
                m_direction->findData(static_cast<int>(constraint->m_Direction))));
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

    QString name() const
    {
        return m_name->text().trimmed();
    }
    int stepId() const
    {
        return m_step->currentData().toInt();
    }
    std::shared_ptr<Node> node() const
    {
        return m_nodeValue;
    }
    EnumKeyword::Direction direction() const
    {
        return static_cast<EnumKeyword::Direction>(m_direction->currentData().toInt());
    }
    double displacement() const
    {
        return m_value->value();
    }

protected:
    void accept() override
    {
        m_nodeValue = m_structure->FindNode(m_node->value());
        if (!m_nodeValue)
        {
            QMessageBox::warning(this, QStringLiteral("无效节点"),
                QStringLiteral("节点 %1 不存在。").arg(m_node->value()));
            return;
        }
        QDialog::accept();
    }

private:
    std::shared_ptr<StructureData> m_structure;
    std::shared_ptr<Node> m_nodeValue;
    QLineEdit* m_name = nullptr;
    QComboBox* m_step = nullptr;
    QSpinBox* m_node = nullptr;
    QComboBox* m_direction = nullptr;
    QDoubleSpinBox* m_value = nullptr;
};

QTableWidgetItem* tableItem(const QString& text, int id = -1)
{
    auto* item = new QTableWidgetItem(text);
    if (id >= 0)
        item->setData(Qt::UserRole, id);
    return item;
}
}

AnalysisManagerDialog::AnalysisManagerDialog(
    const std::shared_ptr<StructureData>& structure,
    Page initialPage,
    QWidget* parent)
    : QDialog(parent)
    , m_structure(structure)
    , m_page(initialPage)
{
    Ui::AnalysisManagerDialogClass form;
    form.setupUi(this);
    const bool isStepManager = initialPage == Page::Steps;
    const bool isLoadManager = initialPage == Page::Loads;
    const bool isConstraintManager = initialPage == Page::Constraints;
    setObjectName(isStepManager ? QStringLiteral("analysisStepManagerDialog")
        : isLoadManager ? QStringLiteral("analysisLoadManagerDialog")
        : isConstraintManager ? QStringLiteral("analysisConstraintManagerDialog")
        : QStringLiteral("analysisMPCManagerDialog"));
    setWindowTitle(isStepManager ? QStringLiteral("分析步管理器")
        : isLoadManager ? QStringLiteral("荷载管理器")
        : isConstraintManager ? QStringLiteral("边界条件管理器")
        : QStringLiteral("MPC 管理器"));
    setWindowIcon(managerIcon(isStepManager ? ManagerGlyph::Steps
        : isLoadManager ? ManagerGlyph::Load
        : isConstraintManager ? ManagerGlyph::Constraint
        : ManagerGlyph::MPC, palette()));
    setModal(false);
    setWindowModality(Qt::NonModal);
    resize(920, 560);
    QTableWidget* table = form.resourceTable;
    applyCenteredTableAppearance(table);
    QPushButton* addButton = form.addButton;
    QPushButton* editButton = form.editButton;
    QPushButton* deleteButton = form.deleteButton;
    std::function<void()> addHandler;
    std::function<void()> editHandler;
    std::function<void()> deleteHandler;
    QStringList headers;
    if (isStepManager)
    {
        m_stepTable = table;
        headers = { QStringLiteral("名称"), QStringLiteral("类型"), QStringLiteral("总时间"),
            QStringLiteral("增量"), QStringLiteral("生效荷载数"),
            QStringLiteral("生效约束数"), QStringLiteral("生效 MPC 数") };
        headers.insert(4, QStringLiteral("计算区域"));
        addHandler = [this]() { editStep(); };
        editHandler = [this]() { editStep(currentId(m_stepTable)); };
        deleteHandler = [this]() { deleteSelectedStep(); };
    }
    else if (isLoadManager)
    {
        m_loadTable = table;
        headers = { QStringLiteral("名称"), QStringLiteral("分析步"), QStringLiteral("类型"),
            QStringLiteral("目标"), QStringLiteral("方向"), QStringLiteral("数值") };
        addHandler = [this]() { editLoad(); };
        editHandler = [this]() { editLoad(currentId(m_loadTable)); };
        deleteHandler = [this]() { deleteSelectedLoad(); };
    }
    else if (isConstraintManager)
    {
        m_constraintTable = table;
        headers = { QStringLiteral("名称"), QStringLiteral("分析步"), QStringLiteral("节点"),
            QStringLiteral("方向"), QStringLiteral("位移值") };
        addHandler = [this]() { editConstraint(); };
        editHandler = [this]() { editConstraint(currentId(m_constraintTable)); };
        deleteHandler = [this]() { deleteSelectedConstraint(); };
    }
    else
    {
        m_mpcTable = table;
        headers = { QStringLiteral("名称"), QStringLiteral("分析步"),
            QStringLiteral("主节点"), QStringLiteral("从节点"),
            QStringLiteral("SlaveDirection"), QStringLiteral("约束关系") };
        addHandler = [this]() { editMPC(); };
        editHandler = [this]() { editMPC(currentId(m_mpcTable)); };
        deleteHandler = [this]() { deleteSelectedMPC(); };
    }
    table->setColumnCount(headers.size());
    table->setHorizontalHeaderLabels(headers);
    table->verticalHeader()->setVisible(false);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    addButton->setObjectName(QStringLiteral("analysisAddButton"));
    editButton->setObjectName(QStringLiteral("analysisEditButton"));
    deleteButton->setObjectName(QStringLiteral("analysisDeleteButton"));
    addButton->setIcon(managerIcon(ManagerGlyph::Add, palette()));
    editButton->setIcon(managerIcon(ManagerGlyph::Edit, palette()));
    deleteButton->setIcon(managerIcon(ManagerGlyph::Delete, palette()));
    addButton->setIconSize(QSize(18, 18));
    editButton->setIconSize(QSize(18, 18));
    deleteButton->setIconSize(QSize(18, 18));
    for (QPushButton* button : { addButton, editButton, deleteButton })
        button->setMinimumWidth(qMax(92, button->sizeHint().width() + 4));
    connect(addButton, &QPushButton::clicked, this, addHandler);
    connect(editButton, &QPushButton::clicked, this, editHandler);
    connect(deleteButton, &QPushButton::clicked, this, deleteHandler);
    connect(table, &QTableWidget::itemDoubleClicked, this,
        [this, table, editHandler](QTableWidgetItem* item) {
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
        [table, editButton, deleteButton]() {
            const bool selected = table->currentRow() >= 0;
            editButton->setEnabled(selected);
            deleteButton->setEnabled(selected);
        });
    form.buttonBox->setObjectName(QStringLiteral("analysisManagerButtonBox"));
    form.buttonBox->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
    connect(form.buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    refreshAll();
}

void AnalysisManagerDialog::setModelChangedCallback(
    std::function<void(const QSet<int>&)> callback)
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
        m_stepTable->setItem(row, 0, tableItem(stepDisplayName(stepId, step), stepId));
        QString typeText = step->m_Type == EnumKeyword::StepType::STATIC
            ? QStringLiteral("静力") : step->m_Type == EnumKeyword::StepType::DYNAMIC
            ? QStringLiteral("动力") : QStringLiteral("未知");
        if (step->m_Type == EnumKeyword::StepType::DYNAMIC && step->m_InitialStaticStepId > 0)
        {
            const auto initialIt = m_structure->m_AnalysisStep.find(step->m_InitialStaticStepId);
            typeText += QStringLiteral("（初态：%1）").arg(
                initialIt != m_structure->m_AnalysisStep.cend()
                ? stepDisplayName(initialIt->first, initialIt->second)
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
                    ? regionIt->second->m_Name : QStringLiteral("区域 %1（缺失）").arg(regionId));
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
                ? stepDisplayName(step->first, step->second)
                : QStringLiteral("Step-%1（缺失）").arg(load->m_StepId);
        }
        // A load has exactly one owner. Do not duplicate a gravity load for
        // each dynamic branch; list the branches that inherit it instead.
        QStringList inheritedBy;
        for (const auto& [stepId, step] : m_structure->m_AnalysisStep)
        {
            if (!step || step->m_Type != EnumKeyword::StepType::DYNAMIC
                || step->m_InitialStaticStepId <= 0 || stepId == load->m_StepId)
            {
                continue;
            }
            if (load->m_StepId <= step->m_InitialStaticStepId)
                inheritedBy << stepDisplayName(stepId, step);
        }
        const QString stepScope = inheritedBy.isEmpty()
            ? stepName
            : QStringLiteral("%1  \u2192  %2\uFF08\u7EE7\u627F\uFF09")
                .arg(stepName, inheritedBy.join(QStringLiteral("\u3001")));
        const int targetId = loadTargetId(load);
        const QString target = targetId > 0
            ? (load->m_LoadType == EnumKeyword::LoadType::FORCE_NODE
                ? QStringLiteral("节点 %1").arg(targetId) : QStringLiteral("单元 %1").arg(targetId))
            : QStringLiteral("全模型");
        const int row = m_loadTable->rowCount();
        m_loadTable->insertRow(row);
        m_loadTable->setItem(row, 0, tableItem(loadDisplayName(loadId, load), loadId));
        auto* stepItem = tableItem(stepScope);
        if (!inheritedBy.isEmpty())
        {
            stepItem->setToolTip(QStringLiteral("\u6B64\u8377\u8F7D\u5B9A\u4E49\u5C5E\u4E8E %1\u3002\u5728\u6C42\u89E3 %2 \u65F6\uFF0C"
                "\u5B83\u4F1A\u4EE5\u5168\u503C\u6301\u7EED\u4F5C\u7528\uFF0C\u4E0D\u9700\u8981\u91CD\u590D\u521B\u5EFA\u3002")
                .arg(stepName, inheritedBy.join(QStringLiteral("\u3001"))));
        }
        m_loadTable->setItem(row, 1, stepItem);
        m_loadTable->setItem(row, 2, tableItem(loadTypeName(load->m_LoadType)));
        m_loadTable->setItem(row, 3, tableItem(target));
        m_loadTable->setItem(row, 4, tableItem(directionName(load->m_Direction)));
        m_loadTable->setItem(row, 5, tableItem(QString::number(loadValue(load), 'g', 12)));
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
                ? stepDisplayName(step->first, step->second)
                : QStringLiteral("Step-%1（缺失）").arg(constraint->m_StepId);
        }
        const auto node = constraint->m_pNode.lock();
        const int row = m_constraintTable->rowCount();
        m_constraintTable->insertRow(row);
        m_constraintTable->setItem(row, 0,
            tableItem(constraintDisplayName(constraintId, constraint), constraintId));
        m_constraintTable->setItem(row, 1, tableItem(stepName));
        m_constraintTable->setItem(row, 2,
            tableItem(node ? QString::number(node->m_Id) : QStringLiteral("--")));
        m_constraintTable->setItem(row, 3, tableItem(directionName(constraint->m_Direction)));
        m_constraintTable->setItem(row, 4,
            tableItem(QString::number(constraint->m_Value, 'g', 12)));
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
                ? stepDisplayName(step->first, step->second)
                : QStringLiteral("Step-%1（缺失）").arg(mpc->m_StepId);
        }
        const std::vector<int> nodeIds = mpc->GetNodeIds();
        const int row = m_mpcTable->rowCount();
        m_mpcTable->insertRow(row);
        m_mpcTable->setItem(
            row, 0, tableItem(mpcDisplayName(mpcId, mpc), mpcId));
        m_mpcTable->setItem(row, 1, tableItem(stepName));
        m_mpcTable->setItem(row, 2, tableItem(nodeIds.size() == 2
            ? QString::number(nodeIds[0]) : QStringLiteral("--")));
        m_mpcTable->setItem(row, 3, tableItem(nodeIds.size() == 2
            ? QString::number(nodeIds[1]) : QStringLiteral("--")));
        m_mpcTable->setItem(row, 4, tableItem(mpcDirectionText(mpc)));
        m_mpcTable->setItem(row, 5, tableItem(mpcRelationText(mpc)));
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

    StepEditorDialog editor(stepId, m_structure, existing, this);
    if (editor.exec() != QDialog::Accepted)
        return;
    const AnalysisStepConfig value = editor.config();
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
        existing->m_InitialStaticStepId = existing->isDynamic
            ? value.initialStaticStepId : 0;
        existing->m_AdaptiveTssbn = value.adaptiveTssbn;
        existing->m_EnableGalloping = existing->isDynamic && value.enableGalloping;
        existing->m_GallopingIceThickness = value.gallopingIceThickness;
        existing->m_GallopingInitialAttackDegrees =
            value.gallopingInitialAttackDegrees;
        existing->m_RegionScope = value.regionScope;
        existing->m_ComputeRegionIds = value.computeRegionIds;
    }
    else
    {
        m_structure->AddAnalysisStep(value);
    }
    markModelChanged({ stepId });
    selectId(m_stepTable, stepId);
}
void AnalysisManagerDialog::editLoad(int loadId)
{
    if (!m_structure)
        return;
    if (loadId < 0 && m_structure->m_AnalysisStep.empty())
    {
        QMessageBox::information(this, QStringLiteral("无法新增荷载"),
            QStringLiteral("请先创建至少一个分析步。"));
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
    LoadEditorDialog editor(loadId, m_structure, existing, this);
    if (editor.exec() != QDialog::Accepted || !editor.value())
        return;
    m_structure->m_Load[loadId] = editor.value();
    QSet<int> affected = affectedStepsFrom(editor.value()->m_StepId);
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
    ConstraintEditorDialog editor(constraintId, m_structure, existing, this);
    if (editor.exec() != QDialog::Accepted || !editor.node())
        return;
    auto constraint = existing ? existing : std::make_shared<Constraint>();
    constraint->m_Id = constraintId;
    constraint->m_Name = editor.name();
    constraint->m_StepId = editor.stepId();
    constraint->m_pNode = editor.node();
    constraint->m_Direction = editor.direction();
    constraint->m_Value = editor.displacement();
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
    MPCEditorDialog editor(mpcId, m_structure, existing, this);
    if (editor.exec() != QDialog::Accepted || !editor.value())
        return;

    m_structure->m_MPCConstraints[mpcId] = editor.value();
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

    QSet<int> affected = affectedStepsFrom(editor.value()->m_StepId);
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
    const QString prompt = QStringLiteral(
        "确定删除“%1”吗？\n同时会删除归属于该步的 %2 个荷载、"
        "%3 个约束和 %4 个 MPC。\n另有 %5 个动力步继承该静力步，删除后将改为不继承。")
        .arg(stepDisplayName(stepId, found->second))
        .arg(loadCount).arg(constraintCount).arg(mpcCount).arg(dependentStepCount);
    if (QMessageBox::question(this, QStringLiteral("删除分析步"), prompt,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
        return;

    // Loads and constraints introduced by this step are inherited by later
    // steps.  Capture that dependency range before erasing the step so only
    // the genuinely affected later cases are invalidated.
    const QSet<int> affectedStepIds = affectedStepsFrom(stepId);
    m_structure->m_AnalysisStep.erase(found);
    for (auto& [candidateId, candidate] : m_structure->m_AnalysisStep)
    {
        Q_UNUSED(candidateId);
        if (candidate && candidate->m_InitialStaticStepId == stepId)
            candidate->m_InitialStaticStepId = 0;
    }
    for (auto it = m_structure->m_Load.begin(); it != m_structure->m_Load.end(); )
        it = it->second && it->second->m_StepId == stepId ? m_structure->m_Load.erase(it) : std::next(it);
    for (auto it = m_structure->m_Constraint.begin(); it != m_structure->m_Constraint.end(); )
        it = it->second && it->second->m_StepId == stepId ? m_structure->m_Constraint.erase(it) : std::next(it);
    for (auto it = m_structure->m_MPCConstraints.begin();
         it != m_structure->m_MPCConstraints.end(); )
    {
        it = it->second && it->second->m_StepId == stepId
            ? m_structure->m_MPCConstraints.erase(it) : std::next(it);
    }
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
    if (QMessageBox::question(this, QStringLiteral("删除荷载"),
        QStringLiteral("确定删除“%1”吗？").arg(loadDisplayName(loadId, found->second)),
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
    if (QMessageBox::question(this, QStringLiteral("删除约束"),
        QStringLiteral("确定删除“%1”吗？").arg(constraintDisplayName(constraintId, found->second)),
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
    if (QMessageBox::question(this, QStringLiteral("删除 MPC"),
        QStringLiteral("确定删除“%1”吗？").arg(mpcDisplayName(mpcId, found->second)),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes)
    {
        return;
    }
    const int firstAffectedStep = found->second ? found->second->m_StepId : 0;
    m_structure->m_MPCConstraints.erase(found);
    m_structure->RebuildAndMergeComputeRegions();
    markModelChanged(affectedStepsFrom(firstAffectedStep));
}
