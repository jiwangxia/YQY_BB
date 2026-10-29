#include "StepEditorDialog.h"
#include "Widgets/CompactDoubleSpinBox.h"
#include "Utility/DialogScreenAdapter.h"
#include "Utility/UiIcons.h"
#include "Utility/AnalysisDisplay.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Aerodynamics/AeroManager.h"
#include "Solver/Factory/SolverFactory.h"
#include "AdaptiveTssbnSettingsEditor.h"
#include "AdaptiveTimeStepSettingsEditor.h"
#include "StepAlgorithmStack.h"
#include <QAbstractSpinBox>
#include <QBoxLayout>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QObject>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSize>
#include <QSizePolicy>
#include <QSpinBox>
#include <QString>
#include <QStringList>
#include <QStringLiteral>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <variant>
#include <vector>

StepEditorDialog::StepEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
                                   const std::shared_ptr<AnalysisStep>& step, QWidget* parent)
    : AnalysisStepEditor(parent)
    , m_id(id)
    , m_structure(structure)
{
    initializeUi();
    setObjectName(QStringLiteral("analysisStepEditorDialog"));
    setStyleSheet(QStringLiteral("QGroupBox[stepEditorSection=\"true\"] { padding: 0px; margin-top: %1px; }"
                                 "QGroupBox[stepEditorSection=\"true\"]::title { subcontrol-origin: margin;"
                                 " subcontrol-position: top left; position: absolute; top: 0px; left: 16px;"
                                 " padding: 0px; background: transparent; font-weight: 600; }"
                                 "QTabWidget#stepSettingsTabs::pane { border: none; background: transparent; }")
                      .arg(fontMetrics().height() + 8));
    setWindowTitle(step ? QStringLiteral("编辑分析步") : QStringLiteral("新增分析步"));
    setWindowIcon(UiIcons::applicationIcon());
    setModal(true);
    m_name->setPlaceholderText(QStringLiteral("Step-%1").arg(id));
    m_type->setItemData(0, static_cast<int>(EnumKeyword::StepType::STATIC));
    m_type->setItemData(1, static_cast<int>(EnumKeyword::StepType::DYNAMIC));
    auto* sharedParametersGroup = new QGroupBox(QStringLiteral("时间与收敛"), this);
    sharedParametersGroup->setObjectName(QStringLiteral("stepTimeParametersGroup"));
    auto* sharedParametersLayout = new QFormLayout(sharedParametersGroup);
    sharedParametersLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    sharedParametersLayout->setHorizontalSpacing(12);
    sharedParametersLayout->setVerticalSpacing(8);
    sharedParametersLayout->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    sharedParametersLayout->addRow(m_timeLabel, m_time);
    sharedParametersLayout->addRow(m_incrementLabel, m_increment);
    sharedParametersLayout->addRow(m_toleranceLabel, m_tolerance);
    sharedParametersLayout->addRow(m_iterationsLabel, m_iterations);
    auto* columns = new QWidget(this);
    // 同时控制“名称”字段和正文左栏的宽度比例；右侧类型、积分方法及设置栏使用另一个权重。
    constexpr int nameColumnStretch = 1;
    constexpr int settingsColumnStretch = 2;
    auto* columnsLayout = new QHBoxLayout(columns);
    columnsLayout->setContentsMargins(0, 8, 0, 0);
    columnsLayout->setSpacing(12);
    columnsLayout->setSizeConstraint(QLayout::SetMinimumSize);
    auto* commonColumn = new QWidget(columns);
    auto* commonColumnLayout = new QVBoxLayout(commonColumn);
    commonColumnLayout->setContentsMargins(0, 0, 0, 0);
    commonColumnLayout->setSpacing(12);
    commonColumnLayout->setAlignment(Qt::AlignTop);
    commonColumn->setMinimumWidth(280);
    commonColumn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    auto* solverColumn = new QWidget(columns);
    auto* solverColumnLayout = new QVBoxLayout(solverColumn);
    solverColumnLayout->setContentsMargins(0, 0, 0, 0);
    solverColumnLayout->setSpacing(12);
    solverColumnLayout->setAlignment(Qt::AlignTop);
    solverColumn->setMinimumWidth(510);
    solverColumn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    auto* tssbnColumn = new QWidget(columns);
    tssbnColumn->setObjectName(QStringLiteral("stepAlgorithmColumn"));
    auto* tssbnColumnLayout = new QVBoxLayout(tssbnColumn);
    tssbnColumnLayout->setContentsMargins(0, 8, 0, 0);
    tssbnColumnLayout->setSpacing(14);
    tssbnColumn->setMinimumWidth(420);
    tssbnColumn->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    m_gallopingDialog = new QDialog(this);
    m_gallopingDialog->setObjectName(QStringLiteral("analysisStepGallopingDialog"));
    m_gallopingDialog->setWindowTitle(QStringLiteral("覆冰与舞动设置"));
    m_gallopingDialog->setWindowIcon(UiIcons::applicationIcon());
    m_gallopingDialog->setModal(true);
    auto* gallopingDialogLayout = new QVBoxLayout(m_gallopingDialog);
    gallopingDialogLayout->setContentsMargins(18, 18, 18, 18);
    gallopingDialogLayout->setSpacing(14);
    auto* gallopingScroll = new QScrollArea(m_gallopingDialog);
    gallopingScroll->setObjectName(QStringLiteral("stepGallopingSettingsScroll"));
    gallopingScroll->setFrameShape(QFrame::NoFrame);
    gallopingScroll->setWidgetResizable(true);
    auto* gallopingBody = new QWidget(gallopingScroll);
    auto* gallopingBodyLayout = new QVBoxLayout(gallopingBody);
    gallopingBodyLayout->setContentsMargins(0, 0, 0, 0);
    gallopingBodyLayout->setAlignment(Qt::AlignTop);
    gallopingBodyLayout->setSizeConstraint(QLayout::SetMinimumSize);
    gallopingScroll->setWidget(gallopingBody);
    gallopingDialogLayout->addWidget(gallopingScroll, 1);
    auto* gallopingButtons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, m_gallopingDialog);
    gallopingButtons->setObjectName(QStringLiteral("stepGallopingButtonBox"));
    gallopingButtons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
    gallopingButtons->button(QDialogButtonBox::Ok)->setDefault(true);
    gallopingButtons->button(QDialogButtonBox::Ok)->setProperty("primaryAction", true);
    gallopingButtons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    gallopingButtons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    gallopingDialogLayout->addWidget(gallopingButtons);
    connect(gallopingButtons, &QDialogButtonBox::accepted, m_gallopingDialog, &QDialog::accept);
    connect(gallopingButtons, &QDialogButtonBox::rejected, m_gallopingDialog, &QDialog::reject);
    columnsLayout->addWidget(commonColumn, nameColumnStretch);
    columnsLayout->addWidget(solverColumn, settingsColumnStretch);
    columnsLayout->setAlignment(commonColumn, Qt::AlignTop);
    columnsLayout->setAlignment(solverColumn, Qt::AlignTop);
    columns->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    auto* formScroll = new QScrollArea(this);
    formScroll->setObjectName(QStringLiteral("analysisStepFormScroll"));
    formScroll->setFrameShape(QFrame::NoFrame);
    formScroll->setWidgetResizable(true);
    auto* formBody = new QWidget(formScroll);
    formBody->setObjectName(QStringLiteral("stepFormBody"));
    auto* formBodyLayout = new QVBoxLayout(formBody);
    formBodyLayout->setContentsMargins(0, 0, 0, 0);
    formBodyLayout->setSpacing(12);
    formBodyLayout->setSizeConstraint(QLayout::SetMinimumSize);
    formBodyLayout->addWidget(columns);
    formBodyLayout->addStretch(1);
    formScroll->setWidget(formBody);
    auto* settingsTabs = new QTabWidget(solverColumn);
    settingsTabs->setObjectName(QStringLiteral("stepSettingsTabs"));
    settingsTabs->tabBar()->setFixedHeight(30);
    auto* stateSettings = new QWidget(settingsTabs);
    auto* stateSettingsLayout = new QVBoxLayout(stateSettings);
    stateSettingsLayout->setContentsMargins(0, 8, 0, 0);
    stateSettingsLayout->setSpacing(16);
    stateSettingsLayout->setAlignment(Qt::AlignTop);
    settingsTabs->addTab(stateSettings, QStringLiteral("初态与阻尼"));
    auto* algorithmScroll = new QScrollArea(settingsTabs);
    algorithmScroll->setObjectName(QStringLiteral("stepAlgorithmScroll"));
    algorithmScroll->setFrameShape(QFrame::NoFrame);
    algorithmScroll->setWidgetResizable(true);
    algorithmScroll->setWidget(tssbnColumn);
    settingsTabs->addTab(algorithmScroll, QStringLiteral("算法参数"));
    solverColumnLayout->addWidget(settingsTabs, 1);
    m_rootLayout->insertWidget(0, formScroll, 1);
    m_rootLayout->setContentsMargins(16, 16, 16, 16);
    m_rootLayout->setSpacing(12);
    m_rootLayout->setSizeConstraint(QLayout::SetNoConstraint);
    auto* commonHeading = new QLabel(QStringLiteral("求解与工况"), commonColumn);
    commonHeading->setObjectName(QStringLiteral("stepCommonHeading"));
    commonHeading->setFixedHeight(30);
    QFont headingFont = commonHeading->font();
    headingFont.setWeight(QFont::DemiBold);
    commonHeading->setFont(headingFont);
    commonHeading->setContentsMargins(16, 0, 0, 0);
    commonColumnLayout->addWidget(commonHeading);
    auto* identity = new QWidget(this);
    auto* identityLayout = new QHBoxLayout(identity);
    identityLayout->setContentsMargins(0, 0, 0, 0);
    identityLayout->setSpacing(12);
    auto* nameField = new QWidget(identity);
    auto* nameLayout = new QVBoxLayout(nameField);
    nameLayout->setContentsMargins(0, 0, 0, 0);
    nameLayout->setSpacing(6);
    nameLayout->addWidget(m_nameLabel);
    nameLayout->addWidget(m_name);
    auto* typeField = new QWidget(identity);
    auto* typeLayout = new QVBoxLayout(typeField);
    typeLayout->setContentsMargins(0, 0, 0, 0);
    typeLayout->setSpacing(6);
    typeLayout->addWidget(m_typeLabel);
    typeLayout->addWidget(m_type);
    auto* solverGroup = new QWidget(identity);
    auto* solverLayout = new QVBoxLayout(solverGroup);
    solverLayout->setContentsMargins(0, 0, 0, 0);
    solverLayout->setSpacing(6);
    auto* identitySettings = new QWidget(identity);
    auto* identitySettingsLayout = new QHBoxLayout(identitySettings);
    identitySettingsLayout->setContentsMargins(0, 0, 0, 0);
    identitySettingsLayout->setSpacing(12);
    identitySettingsLayout->addWidget(typeField, 1);
    identitySettingsLayout->addWidget(solverGroup, 2);
    identityLayout->addWidget(nameField, nameColumnStretch);
    identityLayout->addWidget(identitySettings, settingsColumnStretch);
    formBodyLayout->insertWidget(0, identity);
    m_dynamicSolver = new QComboBox(solverGroup);
    m_dynamicSolver->setObjectName(QStringLiteral("dynamicSolverCombo"));
    for (const auto& algorithm : SolverNameSpace::SolverFactory::DynamicAlgorithms())
        m_dynamicSolver->addItem(QString::fromUtf8(algorithm.displayName), static_cast<int>(algorithm.type));
    m_dynamicSolver->setItemData(
        m_dynamicSolver->findData(static_cast<int>(SolverNameSpace::SolverType::ReferenceTSSBN)),
        QStringLiteral("采用参考程序的静力初态、气动力公式、Newton 迭代和自适应控制；"
                       "索单元保留参考公式的负轴力，气动系数使用本模型查表插值。"),
        Qt::ToolTipRole);
    auto* integrationLabel = new QLabel(QStringLiteral("积分方法"), solverGroup);
    solverLayout->addWidget(integrationLabel);
    solverLayout->addWidget(m_dynamicSolver);
    auto* initialStateGroup = new QGroupBox(QStringLiteral("初始平衡状态"), this);
    initialStateGroup->setObjectName(QStringLiteral("stepInitialStateGroup"));
    auto* initialStateLayout = new QFormLayout(initialStateGroup);
    m_inheritStaticState = new QCheckBox(QStringLiteral("使用静力步作为初始状态"), initialStateGroup);
    m_inheritStaticState->setObjectName(QStringLiteral("inheritStaticStateCheck"));
    m_initialStaticStep = new QComboBox(initialStateGroup);
    if (structure)
    {
        for (const auto& [candidateId, candidate] : structure->m_AnalysisStep)
        {
            if (candidateId >= id || !candidate || candidate->m_Type != EnumKeyword::StepType::STATIC)
                continue;
            m_initialStaticStep->addItem(AnalysisDisplay::stepDisplayName(candidateId, candidate), candidateId);
        }
    }
    if (m_initialStaticStep->count() == 0)
        m_initialStaticStep->setPlaceholderText(QStringLiteral("暂无可用静力步"));
    m_inheritStaticState->setEnabled(m_initialStaticStep->count() > 0);
    m_inheritStaticState->setToolTip(
        m_initialStaticStep->count() > 0
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
    m_aerodynamicTangentMode = new QComboBox(gallopingGroup);
    m_aerodynamicTangentMode->setObjectName(QStringLiteral("aerodynamicTangentModeCombo"));
    m_aerodynamicTangentMode->addItem(QStringLiteral("关闭（准 Newton）"),
                                      static_cast<int>(SolverNameSpace::AerodynamicTangentMode::Disabled));
    m_aerodynamicTangentMode->addItem(
        QStringLiteral("每时间步/隐式阶段一次"),
        static_cast<int>(SolverNameSpace::AerodynamicTangentMode::OncePerTimeStepOrStage));
    m_aerodynamicTangentMode->addItem(QStringLiteral("每轮 Newton"),
                                      static_cast<int>(SolverNameSpace::AerodynamicTangentMode::EveryNewtonIteration));
    m_aerodynamicTangentMode->setToolTip(QStringLiteral("关闭仅表示不组装气动力切线；气动力仍在每轮 Newton "
                                                        "按当前状态刷新。可用于快速筛选，定量结果请做步长对照。"));
    m_enableGalloping = new QCheckBox(QStringLiteral("启用舞动气动力"), gallopingGroup);
    m_enableGalloping->setObjectName(QStringLiteral("enableGallopingCheck"));
    m_enableGalloping->setToolTip(QStringLiteral("静力步可先求平均风平衡；动力步可继承该静力步并计算舞动。"));
    m_gallopingProfileBindingMode = new QComboBox(gallopingGroup);
    m_gallopingProfileBindingMode->setObjectName(QStringLiteral("gallopingProfileBindingCombo"));
    m_gallopingProfileBindingMode->addItem(QStringLiteral("按风向和几何自动选列"),
                                           static_cast<int>(GallopingProfileBindingMode::AutoGeometry));
    m_gallopingProfileBindingMode->addItem(QStringLiteral("按单元气动剖面编号选列"),
                                           static_cast<int>(GallopingProfileBindingMode::ExplicitElementProfile));
    m_gallopingProfileBindingMode->setToolTip(
        QStringLiteral("显式模式固定使用导线单元的 AERO_PROFILE_ID；四分裂的物理 WIRE_ID 不会因此改变。"
                       "单根导线只使用 P1。"));
    m_quadProfileRows = new QWidget(gallopingGroup);
    auto* quadProfileLayout = new QFormLayout(m_quadProfileRows);
    quadProfileLayout->setContentsMargins(0, 0, 0, 0);
    const auto* structureData = m_structure.get();
    std::array<int, 4> currentProfiles{-2, -2, -2, -2};
    std::array<bool, 4> mixedProfiles{};
    if (structureData)
    {
        for (const auto& [elementId, element] : structureData->m_Elements)
        {
            Q_UNUSED(elementId);
            if (!element || element->m_AeroBundleCount != 4 || element->m_WireId < 0 || element->m_WireId >= 4)
                continue;
            const int wireId = element->m_WireId;
            ++m_quadWireElementCounts[wireId];
            if (currentProfiles[wireId] == -2)
                currentProfiles[wireId] = element->m_AeroProfileId;
            else if (currentProfiles[wireId] != element->m_AeroProfileId)
                mixedProfiles[wireId] = true;
        }
    }
    for (int wireId = 0; wireId < 4; ++wireId)
    {
        if (m_quadWireElementCounts[wireId] == 0)
            continue;
        m_hasQuadWireElements = true;
        auto* profile = new QComboBox(m_quadProfileRows);
        profile->addItem(QStringLiteral("请选择 P1–P4"), -1);
        for (int profileId = 0; profileId < 4; ++profileId)
            profile->addItem(QStringLiteral("P%1").arg(profileId + 1), profileId);
        if (!mixedProfiles[wireId] && currentProfiles[wireId] >= 0 && currentProfiles[wireId] < 4)
            profile->setCurrentIndex(profile->findData(currentProfiles[wireId]));
        profile->setToolTip(QStringLiteral("物理子导线 WIRE_ID=%1 沿跨全部单元共用此气动曲线列。").arg(wireId));
        m_quadWireProfiles[wireId] = profile;
        profile->setObjectName(QStringLiteral("quadWireProfileCombo%1").arg(wireId + 1));
        quadProfileLayout->addRow(
            QStringLiteral("子导线 %1（%2 单元）").arg(wireId + 1).arg(m_quadWireElementCounts[wireId]), profile);
    }
    auto* profileNote =
        new QLabel(QStringLiteral("P1–P4 的对应关系作用于整个模型的同编号子导线，所有分析步共用。"), m_quadProfileRows);
    profileNote->setWordWrap(true);
    quadProfileLayout->addRow(profileNote);
    m_gallopingIceThickness = new QComboBox(gallopingGroup);
    m_gallopingIceThickness->setObjectName(QStringLiteral("gallopingIceThicknessCombo"));
    for (int thickness : AeroManager::supportedIceThicknesses())
        m_gallopingIceThickness->addItem(QStringLiteral("%1 mm").arg(thickness), thickness);
    m_gallopingIceThickness->setToolTip(
        QStringLiteral("用于选择风洞气动系数表；结构覆冰质量、惯量和几何需在导线建模时单独设置，并使用相同冰厚。"));
    m_gallopingInitialAttack = new CompactDoubleSpinBox(gallopingGroup);
    m_gallopingInitialAttack->setObjectName(QStringLiteral("gallopingInitialAttackSpin"));
    m_gallopingInitialAttack->setRange(-1000000.0, 1000000.0);
    m_gallopingInitialAttack->setDecimals(6);
    m_gallopingInitialAttack->setValue(45.0);
    m_gallopingInitialAttack->setSuffix(QStringLiteral("°"));
    m_gallopingInitialAttack->setToolTip(QStringLiteral(
        "输入的是名义初始攻角。计算中的实际攻角随导线扭转及相对风偏角实时变化，风载平衡后也可能与输入值不同。"));
    m_gallopingInitialVerticalPerturbation = new CompactDoubleSpinBox(gallopingGroup);
    m_gallopingInitialVerticalPerturbation->setObjectName(QStringLiteral("gallopingInitialVerticalPerturbationSpin"));
    m_gallopingInitialVerticalPerturbation->setRange(-1000000.0, 1000000.0);
    m_gallopingInitialVerticalPerturbation->setDecimals(6);
    m_gallopingInitialVerticalPerturbation->setSingleStep(0.001);
    m_gallopingInitialVerticalPerturbation->setSuffix(QStringLiteral(" m"));
    m_gallopingInitialVerticalPerturbation->setToolTip(
        QStringLiteral("继承静力平衡状态后，在动力初态施加有符号的竖向位移扰动；0 m 表示不施加。"));
    m_gallopingInitialPerturbWaves = new QComboBox(gallopingGroup);
    m_gallopingInitialPerturbWaves->setObjectName(QStringLiteral("gallopingInitialPerturbWavesCombo"));
    m_gallopingInitialPerturbWaves->addItem(QStringLiteral("1 个半波（跨中最大）"), 1);
    m_gallopingInitialPerturbWaves->addItem(QStringLiteral("2 个半波（跨中为零）"), 2);
    m_gallopingInitialPerturbWaves->setToolTip(
        QStringLiteral("选择初始竖向扰动沿档距的空间波形；2 个半波可激发跨中位移为零的反对称模态。"));
    m_aeroInterpolation = new QComboBox(gallopingGroup);
    m_aeroInterpolation->setObjectName(QStringLiteral("aeroInterpolationCombo"));
    m_aeroInterpolation->addItem(QStringLiteral("线性"), false);
    m_aeroInterpolation->addItem(QStringLiteral("周期保形三次"), true);
    m_aeroInterpolation->setCurrentIndex(m_structure->m_AeroManager.isShapePreservingInterpolationEnabled() ? 1 : 0);
    m_aeroInterpolation->setToolTip(
        QStringLiteral("这是整个模型的气动系数插值方式，会影响舞动增长率和幅值；保存模型后在独立求解进程中保持一致。"));
    gallopingLayout->addRow(m_enableGalloping);
    gallopingLayout->addRow(QStringLiteral("气动曲线绑定"), m_gallopingProfileBindingMode);
    gallopingLayout->addRow(m_quadProfileRows);
    gallopingLayout->addRow(QStringLiteral("气动曲线覆冰厚度"), m_gallopingIceThickness);
    gallopingLayout->addRow(QStringLiteral("初始风攻角"), m_gallopingInitialAttack);
    gallopingLayout->addRow(QStringLiteral("初始竖向扰动幅值"), m_gallopingInitialVerticalPerturbation);
    gallopingLayout->addRow(QStringLiteral("初始扰动空间半波数"), m_gallopingInitialPerturbWaves);
    gallopingLayout->addRow(QStringLiteral("气动系数插值（全模型）"), m_aeroInterpolation);
    gallopingLayout->addRow(QStringLiteral("气动切线"), m_aerodynamicTangentMode);
    auto* referenceGallopingNote =
        new QLabel(QStringLiteral("参考流程独立重算所选静力步的无风平衡，再在动力步启风。"
                                  "气动系数按所选覆冰厚度查表插值，采用固定风速动压和固定升阻力方向；"
                                  "四分裂可按风向和几何自动选列，或显式绑定 P1–P4。"
                                  "气动力每轮 Newton 更新，气动力切线固定关闭；索按参考公式保留负轴力。"),
                   gallopingGroup);
    referenceGallopingNote->setWordWrap(true);
    referenceGallopingNote->setToolTip(referenceGallopingNote->text());
    referenceGallopingNote->setText(QStringLiteral("参考流程：继承无风静力平衡，动力步启风；气动切线固定关闭。"));
    referenceGallopingNote->setProperty("secondaryText", true);
    referenceGallopingNote->setVisible(false);
    gallopingLayout->addRow(referenceGallopingNote);
    auto* structuralDampingGroup = new QGroupBox(QStringLiteral("局部瑞丽阻尼"), this);
    structuralDampingGroup->setObjectName(QStringLiteral("stepStructuralDampingGroup"));
    auto* structuralDampingLayout = new QVBoxLayout(structuralDampingGroup);
    structuralDampingLayout->setSpacing(12);
    m_enableStructuralDamping = new QCheckBox(QStringLiteral("启用结构阻尼"), structuralDampingGroup);
    m_enableStructuralDamping->setObjectName(QStringLiteral("enableStructuralDampingCheck"));
    m_translationDampingRatio = new CompactDoubleSpinBox(structuralDampingGroup);
    m_translationDampingRatio->setObjectName(QStringLiteral("translationDampingRatioSpin"));
    m_translationDampingRatio->setRange(0.0, 100.0);
    m_translationDampingRatio->setSingleStep(0.05);
    m_translationDampingRatio->setSuffix(QStringLiteral(" %"));
    m_torsionDampingRatio = new CompactDoubleSpinBox(structuralDampingGroup);
    m_torsionDampingRatio->setRange(0.0, 100.0);
    m_torsionDampingRatio->setSingleStep(0.1);
    m_torsionDampingRatio->setSuffix(QStringLiteral(" %"));
    m_useExplicitRayleighCoefficients =
        new QCheckBox(QStringLiteral("直接输入局部 Rayleigh 系数"), structuralDampingGroup);
    m_useExplicitRayleighCoefficients->setObjectName(QStringLiteral("explicitRayleighCoefficientsCheck"));
    auto* rayleighCoefficients = new QWidget(structuralDampingGroup);
    auto* rayleighLayout = new QGridLayout(rayleighCoefficients);
    rayleighLayout->setContentsMargins(0, 0, 0, 0);
    rayleighLayout->setHorizontalSpacing(10);
    rayleighLayout->setVerticalSpacing(6);
    m_translationMassCoefficient = new CompactDoubleSpinBox(rayleighCoefficients);
    m_translationStiffnessCoefficient = new CompactDoubleSpinBox(rayleighCoefficients);
    m_torsionMassCoefficient = new CompactDoubleSpinBox(rayleighCoefficients);
    m_torsionStiffnessCoefficient = new CompactDoubleSpinBox(rayleighCoefficients);
    const std::array<QDoubleSpinBox*, 4> coefficientEditors{m_translationMassCoefficient,
                                                            m_translationStiffnessCoefficient, m_torsionMassCoefficient,
                                                            m_torsionStiffnessCoefficient};
    for (auto* editor : coefficientEditors)
        editor->setRange(0.0, 1.0e12);
    const std::array<QString, 4> coefficientLabels{
        QStringLiteral("平动质量系数 y1 (1/s)"), QStringLiteral("平动刚度系数 y2 (s)"),
        QStringLiteral("扭转质量系数 c1 (1/s)"), QStringLiteral("扭转刚度系数 c2 (s)")};
    for (int index = 0; index < 4; ++index)
    {
        const int row = 2 * (index / 2);
        rayleighLayout->addWidget(new QLabel(coefficientLabels[index], rayleighCoefficients), row, index % 2);
        rayleighLayout->addWidget(coefficientEditors[index], row + 1, index % 2);
    }
    rayleighLayout->setColumnStretch(0, 1);
    rayleighLayout->setColumnStretch(1, 1);
    auto* dampingParameters = new QWidget(structuralDampingGroup);
    auto* dampingParametersLayout = new QGridLayout(dampingParameters);
    dampingParametersLayout->setContentsMargins(0, 0, 0, 0);
    dampingParametersLayout->setHorizontalSpacing(10);
    dampingParametersLayout->setVerticalSpacing(6);
    auto* translationDampingLabel = new QLabel(QStringLiteral("平动阻尼比"), dampingParameters);
    auto* torsionDampingLabel = new QLabel(QStringLiteral("扭转阻尼比"), dampingParameters);
    dampingParametersLayout->addWidget(translationDampingLabel, 0, 0);
    dampingParametersLayout->addWidget(torsionDampingLabel, 0, 1);
    dampingParametersLayout->addWidget(m_translationDampingRatio, 1, 0);
    dampingParametersLayout->addWidget(m_torsionDampingRatio, 1, 1);
    const std::array<QString, 4> referenceNames{
        QStringLiteral("translationReferenceMode1"), QStringLiteral("translationReferenceMode2"),
        QStringLiteral("torsionReferenceMode1"), QStringLiteral("torsionReferenceMode2")};
    for (int family = 0; family < 2; ++family)
    {
        dampingParametersLayout->addWidget(
            new QLabel(family == 0 ? QStringLiteral("平动参考阶次（0 自动）")
                                  : QStringLiteral("扭转参考阶次（0 自动）"), dampingParameters), 2, family);
        auto* references = new QWidget(dampingParameters);
        references->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
        auto* referenceLayout = new QHBoxLayout(references);
        referenceLayout->setContentsMargins(0, 0, 0, 0);
        for (int index = 0; index < 2; ++index)
        {
            const int editorIndex = 2 * family + index;
            auto* editor = new QSpinBox(references);
            editor->setRange(0, 1000000000);
            editor->setSpecialValueText(QStringLiteral("自动"));
            editor->setObjectName(referenceNames[editorIndex]);
            editor->setToolTip(
                QStringLiteral("两项均为自动时，选该类最低两个有效模态；手动时填写两个不同的整体模态阶次。"
                               "参考模态用于标定局部瑞丽系数并回代实际阻尼比。"));
            m_dampingReferenceModes[editorIndex] = editor;
            referenceLayout->addWidget(editor);
        }
        dampingParametersLayout->addWidget(references, 3, family);
    }
    dampingParametersLayout->setColumnStretch(0, 1);
    dampingParametersLayout->setColumnStretch(1, 1);
    m_translationDampingRatio->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_torsionDampingRatio->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    m_translationDampingRatio->setMinimumWidth(120);
    m_torsionDampingRatio->setMinimumWidth(120);
    structuralDampingLayout->addWidget(m_enableStructuralDamping);
    structuralDampingLayout->addWidget(dampingParameters);
    structuralDampingLayout->addWidget(m_useExplicitRayleighCoefficients);
    structuralDampingLayout->addWidget(rayleighCoefficients);
    auto* regionGroup = new QGroupBox(QStringLiteral("计算区域"), this);
    regionGroup->setObjectName(QStringLiteral("stepRegionGroup"));
    regionGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    auto* regionLayout = new QVBoxLayout(regionGroup);
    m_allRegions = new QCheckBox(QStringLiteral("全部启用计算区域"), regionGroup);
    m_allRegions->setObjectName(QStringLiteral("stepAllRegionsCheck"));
    m_regionList = new QListWidget(regionGroup);
    regionLayout->addWidget(m_allRegions);
    regionLayout->addWidget(m_regionList);

    commonColumnLayout->addWidget(sharedParametersGroup);
    stateSettingsLayout->addWidget(initialStateGroup);
    stateSettingsLayout->addWidget(structuralDampingGroup);
    gallopingBodyLayout->addWidget(gallopingGroup);
    m_tssbnEditor = new AdaptiveTssbnSettingsEditor(tssbnColumn);
    auto* tssbnGroup = m_tssbnEditor;
    m_adaptiveTimeStepEditor = new AdaptiveTimeStepSettingsEditor(tssbnColumn);
    auto* adaptiveTimeStepGroup = m_adaptiveTimeStepEditor;
    auto* algorithmStack = new StepAlgorithmStack(tssbnColumn);
    algorithmStack->setObjectName(QStringLiteral("stepAlgorithmStack"));
    auto* algorithmSummaryPage = new QWidget(algorithmStack);
    auto* adaptiveTimeStepPage = new QWidget(algorithmStack);
    auto* adaptiveTimeStepPageLayout = new QVBoxLayout(adaptiveTimeStepPage);
    adaptiveTimeStepPageLayout->setContentsMargins(0, 0, 0, 0);
    adaptiveTimeStepPageLayout->addWidget(adaptiveTimeStepGroup, 0, Qt::AlignTop);
    adaptiveTimeStepPageLayout->addStretch();
    auto* tssbnPage = new QWidget(algorithmStack);
    auto* tssbnPageLayout = new QVBoxLayout(tssbnPage);
    tssbnPageLayout->setContentsMargins(0, 0, 0, 0);
    tssbnPageLayout->addWidget(tssbnGroup, 0, Qt::AlignTop);
    tssbnPageLayout->addStretch();
    algorithmStack->addWidget(algorithmSummaryPage);
    algorithmStack->addWidget(adaptiveTimeStepPage);
    algorithmStack->addWidget(tssbnPage);
    tssbnColumnLayout->addWidget(algorithmStack);
    auto* gallopingEntry = new QGroupBox(QStringLiteral("覆冰与舞动"), commonColumn);
    gallopingEntry->setObjectName(QStringLiteral("stepGallopingEntryGroup"));
    gallopingEntry->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    auto* gallopingEntryLayout = new QBoxLayout(QBoxLayout::TopToBottom, gallopingEntry);
    gallopingEntryLayout->setSpacing(8);
    m_gallopingSummary = new QLabel(gallopingEntry);
    m_gallopingSummary->setProperty("secondaryText", true);
    m_gallopingSummary->setWordWrap(true);
    auto* gallopingButton = new QPushButton(QStringLiteral("覆冰舞动设置…"), gallopingEntry);
    gallopingButton->setObjectName(QStringLiteral("stepGallopingSettingsButton"));
    gallopingButton->setAutoDefault(false);
    gallopingButton->setToolTip(QStringLiteral("设置舞动气动力、覆冰厚度、初始攻角及气动曲线。"));
    gallopingEntryLayout->addWidget(m_gallopingSummary);
    gallopingEntryLayout->addWidget(gallopingButton);
    auto* additionalSettings = new QWidget(commonColumn);
    additionalSettings->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    auto* additionalSettingsLayout = new QHBoxLayout(additionalSettings);
    additionalSettingsLayout->setDirection(QBoxLayout::TopToBottom);
    additionalSettingsLayout->setContentsMargins(0, 0, 0, 0);
    additionalSettingsLayout->setSpacing(12);
    additionalSettingsLayout->setAlignment(Qt::AlignTop);
    additionalSettingsLayout->addWidget(regionGroup);
    additionalSettingsLayout->addWidget(gallopingEntry);
    commonColumnLayout->addWidget(additionalSettings);
    auto* staticSettings = new QWidget(solverColumn);
    staticSettings->setObjectName(QStringLiteral("stepStaticSettings"));
    staticSettings->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
    staticSettings->hide();
    auto* staticSettingsLayout = new QVBoxLayout(staticSettings);
    staticSettingsLayout->setContentsMargins(0, 0, 0, 0);
    staticSettingsLayout->setSpacing(12);
    staticSettingsLayout->setAlignment(Qt::AlignTop);
    auto* staticHeading = new QLabel(QStringLiteral("区域与舞动"), staticSettings);
    staticHeading->setFixedHeight(30);
    staticHeading->setFont(headingFont);
    staticHeading->setContentsMargins(16, 0, 0, 0);
    staticSettingsLayout->addWidget(staticHeading);
    solverColumnLayout->addWidget(staticSettings);
    for (QLabel* label : {m_nameLabel, m_typeLabel, integrationLabel, translationDampingLabel, torsionDampingLabel})
        label->setProperty("secondaryText", true);
    const int labelWidth = std::max(84, fontMetrics().horizontalAdvance(QStringLiteral("最大迭代数")));
    for (QFormLayout* mainForm : {sharedParametersLayout, initialStateLayout})
    {
        mainForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        mainForm->setRowWrapPolicy(QFormLayout::DontWrapRows);
        mainForm->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        mainForm->setHorizontalSpacing(12);
        mainForm->setVerticalSpacing(8);
        for (int row = 0; row < mainForm->rowCount(); ++row)
        {
            auto* item = mainForm->itemAt(row, QFormLayout::LabelRole);
            if (item && item->widget())
            {
                item->widget()->setFixedWidth(labelWidth);
                item->widget()->setProperty("secondaryText", true);
            }
        }
    }
    for (QGroupBox* group :
         {sharedParametersGroup, initialStateGroup, structuralDampingGroup, regionGroup, gallopingEntry,
          static_cast<QGroupBox*>(m_tssbnEditor), static_cast<QGroupBox*>(m_adaptiveTimeStepEditor)})
    {
        group->setProperty("stepEditorSection", true);
        group->layout()->setContentsMargins(16, 8, 16, 12);
        group->layout()->setAlignment(Qt::AlignTop);
    }
    for (QFormLayout* formLayout : findChildren<QFormLayout*>())
    {
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        formLayout->setRowWrapPolicy(formLayout == sharedParametersLayout || formLayout == initialStateLayout
                                         ? QFormLayout::DontWrapRows
                                         : QFormLayout::WrapLongRows);
        formLayout->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        formLayout->setHorizontalSpacing(12);
        formLayout->setVerticalSpacing(8);
    }
    for (QComboBox* combo : findChildren<QComboBox*>())
    {
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setMinimumContentsLength(12);
    }
    for (QWidget* input : findChildren<QWidget*>())
    {
        if (input->window() == this &&
            ((qobject_cast<QLineEdit*>(input) && !qobject_cast<QAbstractSpinBox*>(input->parentWidget()) &&
              !qobject_cast<QComboBox*>(input->parentWidget())) ||
             qobject_cast<QComboBox*>(input) || qobject_cast<QAbstractSpinBox*>(input)))
            input->setFixedHeight(34);
    }
    setTabOrder(m_name, m_type);
    setTabOrder(m_type, m_dynamicSolver);
    setTabOrder(m_dynamicSolver, m_time);
    setTabOrder(m_time, m_increment);
    setTabOrder(m_increment, m_tolerance);
    setTabOrder(m_tolerance, m_iterations);
    setTabOrder(m_iterations, m_allRegions);
    setTabOrder(m_allRegions, gallopingButton);
    setTabOrder(gallopingButton, m_inheritStaticState);
    setTabOrder(m_inheritStaticState, m_initialStaticStep);
    setTabOrder(m_initialStaticStep, m_enableStructuralDamping);
    if (structure)
    {
        for (const auto& [regionId, region] : structure->m_ComputeRegions)
        {
            if (!region)
                continue;
            auto* item = new QListWidgetItem(QStringLiteral("%1（节点 %2，单元 %3）")
                                                 .arg(region->m_Name)
                                                 .arg(region->m_NodeIds.size())
                                                 .arg(region->m_ElementIds.size()),
                                             m_regionList);
            item->setData(Qt::UserRole, regionId);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(Qt::Unchecked);
        }
    }
    // 区域选择通常是短列表，按最多四行设置高度，不让其拉伸至对话框底部。
    const int regionRows = std::clamp(m_regionList->count(), 1, 4);
    const int regionRowHeight = std::max(26, m_regionList->sizeHintForRow(0));
    m_regionList->setFixedHeight(regionRows * regionRowHeight + 2 * m_regionList->frameWidth() + 4);
    connect(m_allRegions, &QCheckBox::toggled, this,
            [this](bool allRegions)
            {
                m_regionList->setDisabled(allRegions);
                m_regionList->setVisible(!allRegions);
                if (allRegions)
                    return;

                bool hasSelection = false;
                for (int row = 0; row < m_regionList->count(); ++row)
                    hasSelection = hasSelection || m_regionList->item(row)->checkState() == Qt::Checked;
                if (hasSelection)
                    return;

                // 从“全部区域”切换为“指定区域”时继承当前参与状态，避免得到空计算范围。
                for (int row = 0; row < m_regionList->count(); ++row)
                {
                    auto* item = m_regionList->item(row);
                    const int regionId = item->data(Qt::UserRole).toInt();
                    if (!m_structure)
                        continue;
                    const auto found = m_structure->m_ComputeRegions.find(regionId);
                    if (found != m_structure->m_ComputeRegions.cend() && found->second && found->second->m_Enabled)
                        item->setCheckState(Qt::Checked);
                }
            });

    const auto updateInheritedBaseLoads = [this, structure]()
    {
        if (!m_inheritStaticState->isChecked() || m_initialStaticStep->currentIndex() < 0)
        {
            m_inheritedBaseLoads->setText(QStringLiteral("未继承前置静力荷载"));
            return;
        }
        const int sourceStepId = m_initialStaticStep->currentData().toInt();
        const bool referenceTssbn = static_cast<SolverNameSpace::SolverType>(m_dynamicSolver->currentData().toInt()) ==
                                    SolverNameSpace::SolverType::ReferenceTSSBN;
        int gravityCount = 0;
        int otherLoadCount = 0;
        if (structure)
        {
            for (const auto& [loadId, load] : structure->m_Load)
            {
                Q_UNUSED(loadId);
                if (!load || (load->m_StepId > 0 && load->m_StepId > sourceStepId))
                    continue;
                if (referenceTssbn && std::dynamic_pointer_cast<Force_Wind>(load))
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
        m_inheritedBaseLoads->setText(parts.isEmpty() ? QStringLiteral("未发现前置基础荷载")
                                                      : parts.join(QStringLiteral("；")));
    };
    const auto updateDynamicState = [this, solverGroup, settingsTabs, adaptiveTimeStepGroup, tssbnGroup,
                                     algorithmStack, adaptiveTimeStepPage, tssbnPage, algorithmSummaryPage,
                                     gallopingGroup, structuralDampingGroup, dampingParameters, rayleighCoefficients,
                                     initialStateGroup, referenceGallopingNote, identityLayout,
                                     nameColumnStretch, settingsColumnStretch,
                                     commonColumnLayout, additionalSettings, additionalSettingsLayout,
                                     staticSettings, staticSettingsLayout, gallopingButton,
                                     sharedParametersLayout, initialStateLayout,
                                     structuralDampingLayout, updateInheritedBaseLoads]()
    {
        if (m_loadingStep)
            return;
        const bool dynamic =
            static_cast<EnumKeyword::StepType>(m_type->currentData().toInt()) == EnumKeyword::StepType::DYNAMIC;
        const auto solverType = static_cast<SolverNameSpace::SolverType>(m_dynamicSolver->currentData().toInt());
        const bool referenceTssbn = dynamic && solverType == SolverNameSpace::SolverType::ReferenceTSSBN;
        if (referenceTssbn != m_referenceTssbn)
        {
            const QSignalBlocker gallopingBlocker(m_enableGalloping);
            const QSignalBlocker initialStateBlocker(m_inheritStaticState);
            const QSignalBlocker tangentBlocker(m_aerodynamicTangentMode);
            if (referenceTssbn)
            {
                m_regularEnableGalloping = m_enableGalloping->isChecked();
                m_regularInheritStaticState = m_inheritStaticState->isChecked();
                m_regularAerodynamicTangentMode = static_cast<SolverNameSpace::AerodynamicTangentMode>(
                    m_aerodynamicTangentMode->currentData().toInt());
            }
            else
            {
                m_enableGalloping->setChecked(m_regularEnableGalloping);
                m_inheritStaticState->setChecked(m_regularInheritStaticState);
                m_aerodynamicTangentMode->setCurrentIndex(
                    m_aerodynamicTangentMode->findData(static_cast<int>(m_regularAerodynamicTangentMode)));
            }
            m_referenceTssbn = referenceTssbn;
        }
        if (referenceTssbn)
        {
            const QSignalBlocker gallopingBlocker(m_enableGalloping);
            const QSignalBlocker initialStateBlocker(m_inheritStaticState);
            const QSignalBlocker tangentBlocker(m_aerodynamicTangentMode);
            m_enableGalloping->setChecked(true);
            m_inheritStaticState->setChecked(true);
            m_aerodynamicTangentMode->setCurrentIndex(m_aerodynamicTangentMode->findData(
                static_cast<int>(SolverNameSpace::AerodynamicTangentMode::Disabled)));
        }
        m_enableGalloping->setEnabled(!referenceTssbn);
        m_inheritStaticState->setEnabled(!referenceTssbn && m_initialStaticStep->count() > 0);
        m_aeroInterpolation->setEnabled(true);
        referenceGallopingNote->setVisible(referenceTssbn);
        m_tssbnEditor->setReferenceMode(referenceTssbn);
        solverGroup->setEnabled(dynamic);
        solverGroup->setVisible(dynamic);
        settingsTabs->setVisible(dynamic);
        staticSettings->setVisible(!dynamic);
        QVBoxLayout* additionalLayout = dynamic ? commonColumnLayout : staticSettingsLayout;
        if (additionalSettings->parentWidget() != additionalLayout->parentWidget())
        {
            commonColumnLayout->removeWidget(additionalSettings);
            staticSettingsLayout->removeWidget(additionalSettings);
            additionalLayout->addWidget(additionalSettings);
            additionalSettings->show();
            setTabOrder(m_iterations, m_allRegions);
            setTabOrder(m_allRegions, gallopingButton);
        }
        commonColumnLayout->setSpacing(12);
        identityLayout->setStretch(0, nameColumnStretch);
        identityLayout->setStretch(1, settingsColumnStretch);
        additionalSettingsLayout->setDirection(QBoxLayout::TopToBottom);
        sharedParametersLayout->setVerticalSpacing(16);
        initialStateLayout->setVerticalSpacing(16);
        structuralDampingLayout->setSpacing(16);
        settingsTabs->tabBar()->setVisible(dynamic);
        initialStateGroup->setEnabled(dynamic);
        m_initialStaticStep->setEnabled(dynamic && m_inheritStaticState->isChecked() &&
                                        m_initialStaticStep->count() > 0);
        const bool adaptiveTssbn =
            dynamic && (solverType == SolverNameSpace::SolverType::AdaptiveTSSBN || referenceTssbn);
        const bool adaptiveTimeStep = dynamic && (solverType == SolverNameSpace::SolverType::AdaptiveNewmark ||
                                                  solverType == SolverNameSpace::SolverType::AdaptiveRungeKutta4);
        algorithmStack->setCurrentWidget(adaptiveTssbn      ? tssbnPage
                                         : adaptiveTimeStep ? adaptiveTimeStepPage
                                                            : algorithmSummaryPage);
        settingsTabs->setTabEnabled(1, adaptiveTssbn || adaptiveTimeStep);
        settingsTabs->setTabVisible(1, dynamic && (adaptiveTssbn || adaptiveTimeStep));
        settingsTabs->setTabToolTip(1, adaptiveTssbn || adaptiveTimeStep
                                           ? QStringLiteral("设置所选积分算法的误差与自适应步长控制。")
                                           : QStringLiteral("所选算法使用基本设置中的固定步长，无附加参数。"));
        if (!dynamic || (!adaptiveTssbn && !adaptiveTimeStep))
            settingsTabs->setCurrentIndex(0);
        adaptiveTimeStepGroup->setTitle(solverType == SolverNameSpace::SolverType::AdaptiveNewmark
                                            ? QStringLiteral("自适应 Newmark 参数")
                                            : QStringLiteral("自适应 RK4 参数"));
        adaptiveTimeStepGroup->setEnabled(adaptiveTimeStep);
        tssbnGroup->setEnabled(adaptiveTssbn);
        gallopingGroup->setEnabled(true);
        structuralDampingGroup->setEnabled(dynamic);
        const bool dampingEnabled = dynamic && m_enableStructuralDamping->isChecked();
        const bool explicitRayleigh = m_useExplicitRayleighCoefficients->isChecked();
        dampingParameters->setVisible(!explicitRayleigh);
        dampingParameters->setEnabled(dampingEnabled);
        m_useExplicitRayleighCoefficients->setEnabled(dampingEnabled);
        rayleighCoefficients->setVisible(explicitRayleigh);
        rayleighCoefficients->setEnabled(dampingEnabled);
        m_gallopingIceThickness->setEnabled(m_enableGalloping->isChecked());
        m_gallopingInitialAttack->setEnabled(m_enableGalloping->isChecked());
        m_gallopingInitialVerticalPerturbation->setEnabled(dynamic && m_enableGalloping->isChecked() &&
                                                           m_inheritStaticState->isChecked() &&
                                                           m_initialStaticStep->currentIndex() >= 0);
        m_gallopingInitialPerturbWaves->setEnabled(dynamic && m_enableGalloping->isChecked() &&
                                                   m_inheritStaticState->isChecked() &&
                                                   m_initialStaticStep->currentIndex() >= 0);
        m_aerodynamicTangentMode->setEnabled(m_enableGalloping->isChecked() && !referenceTssbn);
        m_gallopingProfileBindingMode->setEnabled(m_enableGalloping->isChecked());
        m_quadProfileRows->setVisible(
            m_hasQuadWireElements && m_enableGalloping->isChecked() &&
            static_cast<GallopingProfileBindingMode>(m_gallopingProfileBindingMode->currentData().toInt()) ==
                GallopingProfileBindingMode::ExplicitElementProfile);
        updateInheritedBaseLoads();
        updateGallopingSummary();
    };
    connect(gallopingButton, &QPushButton::clicked, this,
            [this, updateDynamicState]()
            {
                editGallopingSettings();
                updateDynamicState();
            });
    connect(m_type, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [updateDynamicState](int)
            {
                updateDynamicState();
            });
    connect(m_dynamicSolver, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [updateDynamicState](int)
            {
                updateDynamicState();
            });
    connect(m_enableGalloping, &QCheckBox::toggled, this,
            [updateDynamicState](bool)
            {
                updateDynamicState();
            });
    connect(m_gallopingProfileBindingMode, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [updateDynamicState](int)
            {
                updateDynamicState();
            });
    connect(m_enableStructuralDamping, &QCheckBox::toggled, this,
            [updateDynamicState](bool)
            {
                updateDynamicState();
            });
    connect(m_useExplicitRayleighCoefficients, &QCheckBox::toggled, this,
            [updateDynamicState](bool)
            {
                updateDynamicState();
            });
    connect(m_inheritStaticState, &QCheckBox::toggled, this,
            [updateDynamicState](bool)
            {
                updateDynamicState();
            });
    connect(m_initialStaticStep, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [updateDynamicState](int)
            {
                updateDynamicState();
            });
    m_time->setRange(1.0e-12, 1.0e12);
    m_increment->setRange(1.0e-12, 1.0e12);
    m_tolerance->setRange(1.0e-14, 1.0);
    auto* buttons = m_buttonBox;
    buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
    buttons->button(QDialogButtonBox::Ok)->setDefault(true);
    buttons->button(QDialogButtonBox::Ok)->setProperty("primaryAction", true);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    connect(
        buttons, &QDialogButtonBox::accepted, this,
        [this, gallopingButton, algorithmScroll, settingsTabs]()
        {
            commitNumericEditors();
            const auto dampingSettings = config().structuralDamping;
            const auto validReferencePair = [](const std::array<int, 2>& modes)
            {
                return (modes[0] == 0 && modes[1] == 0) || (modes[0] > 0 && modes[1] > 0 && modes[0] != modes[1]);
            };
            if (!validReferencePair(dampingSettings.translationReferenceModes) ||
                !validReferencePair(dampingSettings.torsionReferenceModes))
            {
                QMessageBox::warning(this, QStringLiteral("阻尼参考阶次无效"),
                                     QStringLiteral("每类的两个参考阶次必须同时为自动，或为两个不同的正整数。"));
                settingsTabs->setCurrentIndex(0);
                m_dampingReferenceModes[0]->setFocus();
                return;
            }
            const auto solverType = static_cast<SolverNameSpace::SolverType>(m_dynamicSolver->currentData().toInt());
            const bool referenceTssbn = solverType == SolverNameSpace::SolverType::ReferenceTSSBN;
            const bool adaptiveTssbn =
                static_cast<EnumKeyword::StepType>(m_type->currentData().toInt()) == EnumKeyword::StepType::DYNAMIC &&
                (solverType == SolverNameSpace::SolverType::AdaptiveTSSBN || referenceTssbn);
            const auto tssbn = m_tssbnEditor->settings();
            const auto adaptiveTimeStepSettings = m_adaptiveTimeStepEditor->settings();
            const bool adaptiveTimeStep =
                static_cast<EnumKeyword::StepType>(m_type->currentData().toInt()) == EnumKeyword::StepType::DYNAMIC &&
                (solverType == SolverNameSpace::SolverType::AdaptiveNewmark ||
                 solverType == SolverNameSpace::SolverType::AdaptiveRungeKutta4);
            if (adaptiveTimeStep &&
                (adaptiveTimeStepSettings.minimumTimeStep > adaptiveTimeStepSettings.maximumTimeStep ||
                 m_increment->value() < adaptiveTimeStepSettings.minimumTimeStep ||
                 m_increment->value() > adaptiveTimeStepSettings.maximumTimeStep))
            {
                QMessageBox::warning(this, QStringLiteral("自适应步长参数无效"),
                                     QStringLiteral("必须满足：最小时间步 ≤ 初始时间步 ≤ 最大时间步。"));
                settingsTabs->setCurrentIndex(1);
                algorithmScroll->ensureWidgetVisible(m_adaptiveTimeStepEditor);
                return;
            }
            if (adaptiveTssbn &&
                (tssbn.minimumTimeStep > tssbn.maximumTimeStep || m_increment->value() < tssbn.minimumTimeStep ||
                 m_increment->value() > tssbn.maximumTimeStep))
            {
                QMessageBox::warning(this, QStringLiteral("TSSBN 参数无效"),
                                     QStringLiteral("必须满足：最小时间步 ≤ 初始时间步 ≤ 最大时间步。"));
                settingsTabs->setCurrentIndex(1);
                algorithmScroll->ensureWidgetVisible(m_tssbnEditor);
                return;
            }
            const bool dynamic =
                static_cast<EnumKeyword::StepType>(m_type->currentData().toInt()) == EnumKeyword::StepType::DYNAMIC;
            if (dynamic && !SolverNameSpace::SolverFactory::GetDynamicExecutionPolicy(solverType).implemented)
            {
                QMessageBox::warning(this, QStringLiteral("积分方法不可用"),
                                     QStringLiteral("当前积分方法尚未实现，请选择列表中的可用算法。"));
                m_dynamicSolver->setFocus();
                return;
            }
            if (dynamic && m_inheritStaticState->isChecked() && m_initialStaticStep->currentIndex() < 0)
            {
                QMessageBox::information(this, QStringLiteral("缺少前置静力步"),
                                         referenceTssbn
                                             ? QStringLiteral("参考 TSSBN 舞动需要前置静力步，请先创建静力分析步。")
                                             : QStringLiteral("请先创建静力分析步，或取消“使用静力步作为初始状态”。"));
                settingsTabs->setCurrentIndex(0);
                m_inheritStaticState->setFocus();
                return;
            }
            if (adaptiveTssbn && tssbn.minimumDerivativeFactor > tssbn.maximumDerivativeFactor)
            {
                QMessageBox::warning(this, QStringLiteral("TSSBN 参数无效"),
                                     QStringLiteral("误差变化因子下限不能大于上限。"));
                settingsTabs->setCurrentIndex(1);
                algorithmScroll->ensureWidgetVisible(m_tssbnEditor);
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
                    settingsTabs->setCurrentIndex(0);
                    m_allRegions->setFocus();
                    return;
                }
            }
            if (m_enableGalloping->isChecked() &&
                static_cast<GallopingProfileBindingMode>(m_gallopingProfileBindingMode->currentData().toInt()) ==
                    GallopingProfileBindingMode::ExplicitElementProfile)
            {
                std::set<int> usedProfiles;
                for (const auto* profile : m_quadWireProfiles)
                {
                    if (!profile)
                        continue;
                    const int profileId = profile->currentData().toInt();
                    if (profileId < 0 || !usedProfiles.insert(profileId).second)
                    {
                        QMessageBox::warning(this, QStringLiteral("气动曲线绑定无效"),
                                             QStringLiteral("每根四分裂子导线需选择不同的 P1–P4 曲线。"));
                        gallopingButton->click();
                        return;
                    }
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
        int solverIndex = m_dynamicSolver->findData(static_cast<int>(step->m_DynamicSolverType));
        if (solverIndex < 0 && step->m_Type == EnumKeyword::StepType::DYNAMIC)
        {
            m_dynamicSolver->addItem(
                QStringLiteral("不可用积分器（类型 %1）").arg(static_cast<int>(step->m_DynamicSolverType)),
                static_cast<int>(step->m_DynamicSolverType));
            solverIndex = m_dynamicSolver->count() - 1;
        }
        m_dynamicSolver->setCurrentIndex(qMax(0, solverIndex));
        m_aerodynamicTangentMode->setCurrentIndex(
            qMax(0, m_aerodynamicTangentMode->findData(static_cast<int>(step->m_GallopingAerodynamicTangentMode))));
        const int inheritedIndex = m_initialStaticStep->findData(step->m_InitialStaticStepId);
        m_inheritStaticState->setChecked(step->m_Type == EnumKeyword::StepType::DYNAMIC && inheritedIndex >= 0);
        if (inheritedIndex >= 0)
            m_initialStaticStep->setCurrentIndex(inheritedIndex);
        m_tssbnEditor->setSettings(step->m_AdaptiveTssbn);
        m_adaptiveTimeStepEditor->setSettings(step->m_AdaptiveTimeStep);
        m_enableStructuralDamping->setChecked(step->m_StructuralDamping.settings.enabled);
        m_useExplicitRayleighCoefficients->setChecked(
            step->m_StructuralDamping.settings.useExplicitRayleighCoefficients);
        m_translationDampingRatio->setValue(step->m_StructuralDamping.settings.translationDampingRatio * 100.0);
        m_torsionDampingRatio->setValue(step->m_StructuralDamping.settings.torsionDampingRatio * 100.0);
        for (int index = 0; index < 2; ++index)
        {
            m_dampingReferenceModes[index]->setValue(
                step->m_StructuralDamping.settings.translationReferenceModes[index]);
            m_dampingReferenceModes[index + 2]->setValue(
                step->m_StructuralDamping.settings.torsionReferenceModes[index]);
        }
        m_translationMassCoefficient->setValue(step->m_StructuralDamping.settings.translationMassCoefficient);
        m_translationStiffnessCoefficient->setValue(step->m_StructuralDamping.settings.translationStiffnessCoefficient);
        m_torsionMassCoefficient->setValue(step->m_StructuralDamping.settings.torsionMassCoefficient);
        m_torsionStiffnessCoefficient->setValue(step->m_StructuralDamping.settings.torsionStiffnessCoefficient);
        m_enableGalloping->setChecked(step->m_EnableGalloping);
        m_gallopingProfileBindingMode->setCurrentIndex(
            qMax(0, m_gallopingProfileBindingMode->findData(static_cast<int>(step->m_GallopingProfileBindingMode))));
        m_gallopingIceThickness->setCurrentIndex(
            qMax(0, m_gallopingIceThickness->findData(step->m_GallopingIceThickness)));
        m_gallopingInitialAttack->setValue(step->m_GallopingInitialAttackDegrees);
        m_gallopingInitialVerticalPerturbation->setValue(step->m_GallopingInitialVerticalPerturbationMeters);
        m_gallopingInitialPerturbWaves->setCurrentIndex(
            qMax(0, m_gallopingInitialPerturbWaves->findData(step->m_GallopingInitialVerticalPerturbationWaves)));
        m_allRegions->setChecked(step->m_RegionScope == AnalysisRegionScope::AllEnabledRegions);
        for (int row = 0; row < m_regionList->count(); ++row)
        {
            auto* item = m_regionList->item(row);
            const int regionId = item->data(Qt::UserRole).toInt();
            item->setCheckState(step->m_ComputeRegionIds.find(regionId) != step->m_ComputeRegionIds.cend()
                                    ? Qt::Checked
                                    : Qt::Unchecked);
        }
    }
    else
    {
        m_time->setValue(1.0);
        m_increment->setValue(0.01);
        m_tolerance->setValue(1.0e-5);
        m_iterations->setValue(50);
        m_dynamicSolver->setCurrentIndex(
            m_dynamicSolver->findData(static_cast<int>(SolverNameSpace::SolverType::Newmark)));
        m_translationDampingRatio->setValue(0.5);
        m_torsionDampingRatio->setValue(3.8);
        for (auto* editor : m_dampingReferenceModes)
            editor->setValue(0);
        if (m_initialStaticStep->count() > 0)
        {
            m_initialStaticStep->setCurrentIndex(m_initialStaticStep->count() - 1);
            m_inheritStaticState->setChecked(true);
        }
        m_allRegions->setChecked(true);
    }
    m_loadingStep = false;
    updateDynamicState();
    resize(1000, 620);
    DialogScreenAdapter::install(this, QSize(600, 300));
    m_gallopingDialog->resize(620, 620);
    DialogScreenAdapter::install(m_gallopingDialog, QSize(420, 300));
}

AnalysisStepConfig StepEditorDialog::config() const
{
    AnalysisStepConfig value;
    value.id = m_id;
    value.name = m_name->text().trimmed();
    value.type = static_cast<EnumKeyword::StepType>(m_type->currentData().toInt());
    value.totalTime = m_time->value();
    value.stepSize = m_increment->value();
    value.tolerance = m_tolerance->value();
    value.maxIterations = m_iterations->value();
    value.dynamicSolverType = value.type == EnumKeyword::StepType::DYNAMIC
                                  ? static_cast<SolverNameSpace::SolverType>(m_dynamicSolver->currentData().toInt())
                                  : SolverNameSpace::SolverType::Newmark;
    value.gallopingAerodynamicTangentMode =
        static_cast<SolverNameSpace::AerodynamicTangentMode>(m_aerodynamicTangentMode->currentData().toInt());
    value.initialStaticStepId = value.type == EnumKeyword::StepType::DYNAMIC && m_inheritStaticState->isChecked()
                                    ? m_initialStaticStep->currentData().toInt()
                                    : 0;
    value.adaptiveTssbn = m_tssbnEditor->settings();
    value.adaptiveTimeStep = m_adaptiveTimeStepEditor->settings();
    value.structuralDamping.enabled =
        value.type == EnumKeyword::StepType::DYNAMIC && m_enableStructuralDamping->isChecked();
    value.structuralDamping.formulation = SolverNameSpace::StructuralDampingFormulation::LocalRayleigh;
    value.structuralDamping.translationDampingRatio = m_translationDampingRatio->value() / 100.0;
    value.structuralDamping.torsionDampingRatio = m_torsionDampingRatio->value() / 100.0;
    value.structuralDamping.translationReferenceModes = {m_dampingReferenceModes[0]->value(),
                                                         m_dampingReferenceModes[1]->value()};
    value.structuralDamping.torsionReferenceModes = {m_dampingReferenceModes[2]->value(),
                                                     m_dampingReferenceModes[3]->value()};
    value.structuralDamping.useExplicitRayleighCoefficients = m_useExplicitRayleighCoefficients->isChecked();
    if (!value.structuralDamping.enabled || value.structuralDamping.useExplicitRayleighCoefficients)
    {
        for (auto* modes :
             {&value.structuralDamping.translationReferenceModes, &value.structuralDamping.torsionReferenceModes})
        {
            if (!((*modes)[0] == 0 && (*modes)[1] == 0) &&
                !((*modes)[0] > 0 && (*modes)[1] > 0 && (*modes)[0] != (*modes)[1]))
                *modes = {};
        }
    }
    value.structuralDamping.translationMassCoefficient = m_translationMassCoefficient->value();
    value.structuralDamping.translationStiffnessCoefficient = m_translationStiffnessCoefficient->value();
    value.structuralDamping.torsionMassCoefficient = m_torsionMassCoefficient->value();
    value.structuralDamping.torsionStiffnessCoefficient = m_torsionStiffnessCoefficient->value();
    value.enableGalloping = m_enableGalloping->isChecked();
    value.gallopingProfileBindingMode =
        static_cast<GallopingProfileBindingMode>(m_gallopingProfileBindingMode->currentData().toInt());
    value.gallopingIceThickness = m_gallopingIceThickness->currentData().toInt();
    value.gallopingInitialAttackDegrees = m_gallopingInitialAttack->value();
    value.gallopingInitialVerticalPerturbationMeters =
        value.type == EnumKeyword::StepType::DYNAMIC && value.enableGalloping && value.initialStaticStepId > 0
            ? m_gallopingInitialVerticalPerturbation->value()
            : 0.0;
    value.gallopingInitialVerticalPerturbationWaves =
        value.type == EnumKeyword::StepType::DYNAMIC && value.enableGalloping && value.initialStaticStepId > 0
            ? m_gallopingInitialPerturbWaves->currentData().toInt()
            : 1;
    value.regionScope =
        m_allRegions->isChecked() ? AnalysisRegionScope::AllEnabledRegions : AnalysisRegionScope::SelectedRegions;
    for (int row = 0; row < m_regionList->count(); ++row)
    {
        const auto* item = m_regionList->item(row);
        if (item->checkState() == Qt::Checked)
            value.computeRegionIds.insert(item->data(Qt::UserRole).toInt());
    }
    return value;
}

bool StepEditorDialog::shapePreservingAerodynamicInterpolation() const
{
    return m_aeroInterpolation->currentData().toBool();
}

std::array<int, 4> StepEditorDialog::quadWireProfiles() const
{
    std::array<int, 4> profiles{-1, -1, -1, -1};
    for (int wireId = 0; wireId < 4; ++wireId)
    {
        if (m_quadWireProfiles[wireId])
            profiles[wireId] = m_quadWireProfiles[wireId]->currentData().toInt();
    }
    return profiles;
}

void StepEditorDialog::updateGallopingSummary()
{
    m_gallopingSummary->setText(
        m_enableGalloping->isChecked()
            ? QStringLiteral("已启用 · 覆冰 %1 mm").arg(m_gallopingIceThickness->currentData().toInt())
            : QStringLiteral("舞动未启用"));
}

void StepEditorDialog::editGallopingSettings()
{
    std::vector<std::pair<QDoubleSpinBox*, double>> doubles;
    std::vector<std::pair<QComboBox*, int>> combos;
    std::vector<std::pair<QCheckBox*, bool>> checks;
    QWidget* content = m_gallopingDialog;
    for (auto* editor : content->findChildren<QDoubleSpinBox*>())
        doubles.emplace_back(editor, editor->value());
    for (auto* editor : content->findChildren<QComboBox*>())
        combos.emplace_back(editor, editor->currentIndex());
    for (auto* editor : content->findChildren<QCheckBox*>())
        checks.emplace_back(editor, editor->isChecked());
    if (m_gallopingDialog->exec() == QDialog::Accepted)
    {
        m_gallopingInitialAttack->interpretText();
        m_gallopingInitialVerticalPerturbation->interpretText();
        return;
    }
    // 取消只恢复覆冰与舞动草稿，主窗的初态、阻尼和积分参数不受影响。
    m_loadingStep = true;
    for (const auto& [editor, value] : doubles)
    {
        const QSignalBlocker blocker(editor);
        editor->setValue(value);
    }
    for (const auto& [editor, value] : combos)
    {
        const QSignalBlocker blocker(editor);
        editor->setCurrentIndex(value);
    }
    for (const auto& [editor, value] : checks)
    {
        const QSignalBlocker blocker(editor);
        editor->setChecked(value);
    }
    m_loadingStep = false;
}

void StepEditorDialog::commitNumericEditors()
{
    m_time->interpretText();
    m_increment->interpretText();
    m_tolerance->interpretText();
    m_translationDampingRatio->interpretText();
    m_torsionDampingRatio->interpretText();
    for (auto* editor : m_dampingReferenceModes)
        editor->interpretText();
    m_translationMassCoefficient->interpretText();
    m_translationStiffnessCoefficient->interpretText();
    m_torsionMassCoefficient->interpretText();
    m_torsionStiffnessCoefficient->interpretText();
    m_gallopingInitialAttack->interpretText();
    m_gallopingInitialVerticalPerturbation->interpretText();
    m_tssbnEditor->commitEditors();
    m_adaptiveTimeStepEditor->commitEditors();
}

void StepEditorDialog::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("StepEditorDialogClass");
    }
    setMinimumSize(QSize(420, 0));
    m_rootLayout = new QVBoxLayout(this);
    m_rootLayout->setSpacing(14);
    m_rootLayout->setObjectName("rootLayout");
    m_rootLayout->setContentsMargins(18, 18, 18, 18);
    m_nameLabel = new QLabel(this);
    m_nameLabel->setObjectName("nameLabel");
    m_name = new QLineEdit(this);
    m_name->setObjectName("nameEdit");
    m_typeLabel = new QLabel(this);
    m_typeLabel->setObjectName("typeLabel");
    m_type = new QComboBox(this);
    m_type->addItem(QString());
    m_type->addItem(QString());
    m_type->setObjectName("typeCombo");
    m_timeLabel = new QLabel(this);
    m_timeLabel->setObjectName("timeLabel");
    m_time = new CompactDoubleSpinBox(this);
    m_time->setObjectName("timeSpin");
    m_incrementLabel = new QLabel(this);
    m_incrementLabel->setObjectName("incrementLabel");
    m_increment = new CompactDoubleSpinBox(this);
    m_increment->setObjectName("incrementSpin");
    m_toleranceLabel = new QLabel(this);
    m_toleranceLabel->setObjectName("toleranceLabel");
    m_tolerance = new CompactDoubleSpinBox(this);
    m_tolerance->setObjectName("toleranceSpin");
    m_iterationsLabel = new QLabel(this);
    m_iterationsLabel->setObjectName("iterationsLabel");
    m_iterations = new QSpinBox(this);
    m_iterations->setObjectName("iterationsSpin");
    m_iterations->setMinimum(1);
    m_iterations->setMaximum(100000);
    m_buttonBox = new QDialogButtonBox(this);
    m_buttonBox->setObjectName("buttonBox");
    m_buttonBox->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);

    m_rootLayout->addWidget(m_buttonBox);

    m_nameLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "名称", nullptr));
    m_typeLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "类型", nullptr));
    m_type->setItemText(0, QCoreApplication::translate("StepEditorDialogClass", "静力分析", nullptr));
    m_type->setItemText(1, QCoreApplication::translate("StepEditorDialogClass", "动力分析", nullptr));

    m_timeLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "总时间", nullptr));
    m_incrementLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "步长 / 增量", nullptr));
    m_toleranceLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "收敛容差", nullptr));
    m_iterationsLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "最大迭代数", nullptr));
}
