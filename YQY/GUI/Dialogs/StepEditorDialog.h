#pragma once

#include "AnalysisEditor.h"
#include "Base/EmptyOUT.h"
#include "Solver/Interface/ISolver.h"
#include <array>
#include <vector>
#include <functional>

class AdaptiveTimeStepSettingsEditor;
class AdaptiveTssbnSettingsEditor;
class QCheckBox;
class QComboBox;
class QDialog;
class QDialogButtonBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QSpinBox;
class QVBoxLayout;
class QWidget;

class StepEditorDialog final : public AnalysisStepEditor
{
public:
    explicit StepEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
                              const std::shared_ptr<AnalysisStep>& step, QWidget* parent); // 创建分析步编辑器并载入已有配置

    AnalysisStepConfig config() const override;                                            // 读取当前编辑的分析步配置

    bool shapePreservingAerodynamicInterpolation() const override;                         // 读取气动系数是否使用保形插值

    std::array<int, 4> quadWireProfiles() const override;                                  // 读取四分裂导线各子线的气动曲线绑定

private:
    void initializeUi();                                                                   // 创建控件和布局

    void updateGallopingSummary();                                                         // 更新当前舞动配置的摘要

    void editGallopingSettings();                                                          // 打开舞动参数设置窗口

    void commitNumericEditors();                                                           // 提交分析步各数值框中尚未确认的输入

    int m_id = 0;
    std::shared_ptr<StructureData> m_structure;
    QLineEdit* m_name = nullptr;
    QComboBox* m_type = nullptr;
    QDoubleSpinBox* m_time = nullptr;
    QDoubleSpinBox* m_increment = nullptr;
    QDoubleSpinBox* m_tolerance = nullptr;
    QSpinBox* m_iterations = nullptr;
    QComboBox* m_dynamicSolver = nullptr;
    QDialog* m_gallopingDialog = nullptr;
    QLabel* m_gallopingSummary = nullptr;
    bool m_loadingStep = true;
    bool m_referenceTssbn = false;
    bool m_regularEnableGalloping = false;
    bool m_regularInheritStaticState = false;
    SolverNameSpace::AerodynamicTangentMode m_regularAerodynamicTangentMode =
        SolverNameSpace::AerodynamicTangentMode::EveryNewtonIteration;
    QComboBox* m_aerodynamicTangentMode = nullptr;
    QCheckBox* m_inheritStaticState = nullptr;
    QComboBox* m_initialStaticStep = nullptr;
    QLabel* m_inheritedBaseLoads = nullptr;
    AdaptiveTssbnSettingsEditor* m_tssbnEditor = nullptr;
    AdaptiveTimeStepSettingsEditor* m_adaptiveTimeStepEditor = nullptr;
    QCheckBox* m_enableStructuralDamping = nullptr;
    QDoubleSpinBox* m_translationDampingRatio = nullptr;
    QDoubleSpinBox* m_torsionDampingRatio = nullptr;
    std::array<QSpinBox*, 4> m_dampingReferenceModes{};
    QCheckBox* m_useExplicitRayleighCoefficients = nullptr;
    QDoubleSpinBox* m_translationMassCoefficient = nullptr;
    QDoubleSpinBox* m_translationStiffnessCoefficient = nullptr;
    QDoubleSpinBox* m_torsionMassCoefficient = nullptr;
    QDoubleSpinBox* m_torsionStiffnessCoefficient = nullptr;
    QCheckBox* m_enableGalloping = nullptr;
    QComboBox* m_gallopingProfileBindingMode = nullptr;
    QWidget* m_quadProfileRows = nullptr;
    std::array<QComboBox*, 4> m_quadWireProfiles{};
    std::array<int, 4> m_quadWireElementCounts{};
    bool m_hasQuadWireElements = false;
    QComboBox* m_gallopingIceThickness = nullptr;
    QDoubleSpinBox* m_gallopingInitialAttack = nullptr;
    QDoubleSpinBox* m_gallopingInitialVerticalPerturbation = nullptr;
    QComboBox* m_gallopingInitialPerturbWaves = nullptr;
    QComboBox* m_aeroInterpolation = nullptr;
    QCheckBox* m_allRegions = nullptr;
    QListWidget* m_regionList = nullptr;

    QVBoxLayout* m_rootLayout = nullptr;
    QLabel* m_nameLabel = nullptr;
    QLabel* m_typeLabel = nullptr;
    QLabel* m_timeLabel = nullptr;
    QLabel* m_incrementLabel = nullptr;
    QLabel* m_toleranceLabel = nullptr;
    QLabel* m_iterationsLabel = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
