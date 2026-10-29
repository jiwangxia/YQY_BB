#pragma once

#include "Base/Base.h"
#include <memory>
#include <utility>
#include <vector>
#include <QWidget>
#include <QList>

class ChevronComboBox;
class ChevronDoubleSpinBox;
class ChevronSpinBox;
class CompactDoubleSpinBox;
class QFormLayout;
class QEvent;
class QGridLayout;
class QGroupBox;
class QHBoxLayout;
class QScrollArea;
class QTabWidget;
class QVBoxLayout;
class QWidget;

class QCheckBox;
class QComboBox;
class QDialog;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class StructureData;
namespace Conductor
{
class PropertyLibrary;
}
class ConductorModule final : public QWidget
{
public:
    struct BuildResult
    {
        std::shared_ptr<StructureData> structure;
        QString filePath;
        QString error;
        int nodeCount = 0;
        int elementCount = 0;
        int spacerCount = 0;

        bool succeeded() const                                                                            // 检查模型和输出路径是否生成成功
        {
            return structure && !filePath.isEmpty();
        }
    };

    explicit ConductorModule(QWidget* p = nullptr);                                                       // 创建导线建模入口及参数面板
    ~ConductorModule() override;                                                                          // 销毁导线建模入口及参数面板
    void setPropertyLibrary(Conductor::PropertyLibrary* library);                                         // 绑定建模使用的材料与截面属性库
    void setExplicitInnerSpacerPositions(std::vector<double> positions)                                   // 设置显式给定的档内间隔棒位置
    {
        m_explicitInnerSpacerPositions = std::move(positions);
    }
    BuildResult buildModel(Conductor::PropertyLibrary& library, const QString& generatedDirectory) const; // 根据界面参数生成导线模型并返回生成信息
    QPushButton* singleSpanButton() const;                                                                // 返回单档导线建模入口按钮
    QPushButton* multiSpanButton() const;                                                                 // 返回多档导线建模入口按钮
    void setCreationEnabled(bool enabled);                                                                // 设置导线建模入口是否可用
    void openCreationDialog();                                                                            // 打开导线参数窗口并选择建模模式
    void openCreationDialog(bool multiSpan);                                                              // 打开导线参数窗口并选择建模模式
    void closeCreationDialog();                                                                           // 关闭导线参数窗口
    QDialog* creationDialog() const;                                                                      // 返回导线参数窗口
    QPushButton* viewLibraryButton() const;                                                               // 返回查看材料与截面属性库的按钮
    QLineEdit* nameEdit() const;                                                                          // 返回模型名称输入框
    QComboBox* materialCombo() const;                                                                     // 返回导线材料选择框
    QComboBox* sectionCombo() const;                                                                      // 返回导线截面选择框
    QComboBox* elementCombo() const;                                                                      // 返回导线单元类型选择框
    QDoubleSpinBox* startX() const;                                                                       // 返回导线起点 X 坐标输入框
    QDoubleSpinBox* startY() const;                                                                       // 返回导线起点 Y 坐标输入框
    QDoubleSpinBox* startZ() const;                                                                       // 返回导线起点 Z 坐标输入框
    QDoubleSpinBox* endX() const;                                                                         // 返回导线终点 X 坐标输入框
    QDoubleSpinBox* endY() const;                                                                         // 返回导线终点 Y 坐标输入框
    QDoubleSpinBox* endZ() const;                                                                         // 返回导线终点 Z 坐标输入框
    QComboBox* bundleCombo() const;                                                                       // 返回导线分裂数选择框
    QDoubleSpinBox* spacingSpin() const;                                                                  // 返回分裂导线间距输入框
    QDoubleSpinBox* bundleRotationSpin() const;                                                           // 返回分裂导线截面转角输入框
    QSpinBox* segmentsSpin() const;                                                                       // 返回导线离散段数输入框
    QDoubleSpinBox* endTransitionSpin() const;                                                            // 返回端部过渡长度输入框
    QDoubleSpinBox* stressSpin() const;                                                                   // 返回导线初始应力输入框
    QCheckBox* innerSpacerCheck() const;                                                                  // 返回生成档内间隔棒的选项
    QComboBox* spacerLayoutCombo() const;                                                                 // 返回间隔棒布置方式选择框
    QSpinBox* spacerCountSpin() const;                                                                    // 返回间隔棒数量输入框
    QDoubleSpinBox* spacerMassSpin() const;                                                               // 返回间隔棒质量输入框
    QComboBox* spacerPresetCombo() const;                                                                 // 返回间隔棒型号预设选择框
    QDoubleSpinBox* spacerSpacingSpin() const;                                                            // 返回间隔棒布置间距输入框
    QLineEdit* spacerModelEdit() const;                                                                   // 返回间隔棒型号名称输入框
    QComboBox* spacerElementCombo() const;                                                                // 返回间隔棒单元类型选择框
    QComboBox* spacerMaterialCombo() const;                                                               // 返回间隔棒材料选择框
    QComboBox* spacerSectionCombo() const;                                                                // 返回间隔棒截面选择框
    QCheckBox* analysisCheck() const;                                                                     // 返回生成默认分析配置的选项
    QPushButton* createButton() const;                                                                    // 返回确认生成导线模型的按钮

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;                                           // 监听参数窗口尺寸变化并调整布局

private:
    void updateResponsiveLayout();                                                                        // 按可用空间调整界面布局
    void arrangeSections(QGridLayout* grid, const QList<QWidget*>& sections, bool singleColumn);          // 按单列或多列方式排列参数分组
    void initializeCreationDialog();                                                                      // 初始化导线参数窗口及操作连接
    void initializeStationTable();                                                                        // 初始化多档导线站点参数表
    void updateModelModeUi();                                                                             // 切换单档与多档参数的显示状态
    QString defaultModelName() const;                                                                     // 按当前建模模式生成默认模型名称
    void updateDefaultModelName();                                                                        // 在名称未被自定义时更新默认名称
    void refreshStationTypes();                                                                           // 刷新多档站点类型及端点约束
    std::vector<Vector3d> stationCenters(_OUT QString& error) const;                                      // 读取站点中心坐标并返回校验错误
    void updateSpacerUi();                                                                                // 按间隔棒类型和布置方式更新输入状态
    void updateSpacerPreview();                                                                           // 更新间隔棒数量和布置预览
    void refreshSpacerPresets();                                                                          // 刷新可选的间隔棒型号预设
    void applySpacerPreset(int index);                                                                    // 将所选型号预设填入间隔棒参数
    void updateSpacerPresetDescription();                                                                 // 更新当前间隔棒型号说明
    void updateEndTopologyUi();                                                                           // 按端部连接形式更新相关输入项

    void initializeUi();                                                                                  // 创建界面控件并组织布局
    void initializePropertyGroup();                                                                       // 创建模型名称、材料和截面参数组
    void initializeGeometryGroup();                                                                       // 创建分裂导线几何与离散参数组
    void initializeSpanGroup();                                                                           // 创建单档坐标与多档站点参数组
    void initializeEndConnectionGroup();                                                                  // 创建端部连接及初始应力参数组
    void initializeStructuralIceGroup();                                                                  // 创建结构覆冰参数组
    void initializeTorsionGroup();                                                                        // 创建扭转刚度与覆冰角度参数组
    void initializeSpacerPropertyGroup();                                                                 // 创建间隔棒型号和材料参数组
    void initializeSpacerGroup();                                                                         // 创建间隔棒结构与布置参数组
    void initializeBasicTab();                                                                            // 组织导线基础参数选项卡
    void initializeAdvancedTab();                                                                         // 组织覆冰与扭转参数选项卡
    void initializeSpacerTab();                                                                           // 组织间隔棒参数选项卡

    QVBoxLayout* m_rootLayout = nullptr;
    QPushButton* m_singleSpanButton = nullptr;
    QPushButton* m_multiSpanButton = nullptr;
    QWidget* m_editorContainer = nullptr;
    QVBoxLayout* m_editorLayout = nullptr;
    QHBoxLayout* m_globalBundleLayout = nullptr;
    ChevronComboBox* m_bundleCombo = nullptr;
    QTabWidget* m_creationTabs = nullptr;
    QWidget* m_basicTab = nullptr;
    QScrollArea* m_scrollArea = nullptr;
    QWidget* m_content = nullptr;
    QGridLayout* m_contentLayout = nullptr;
    QGroupBox* m_propertyGroup = nullptr;
    QVBoxLayout* m_propertyLayout = nullptr;
    QFormLayout* m_materialFormLayout = nullptr;
    QLineEdit* m_nameEdit = nullptr;
    ChevronComboBox* m_materialCombo = nullptr;
    ChevronComboBox* m_sectionCombo = nullptr;
    ChevronComboBox* m_elementCombo = nullptr;
    QLabel* m_propertySummary = nullptr;
    QGroupBox* m_formGroup = nullptr;
    QFormLayout* m_formLayout = nullptr;
    ChevronDoubleSpinBox* m_spacingSpin = nullptr;
    QLabel* m_bundleRotationLabel = nullptr;
    ChevronDoubleSpinBox* m_bundleRotationSpin = nullptr;
    QLabel* m_segmentsLabel = nullptr;
    ChevronSpinBox* m_segmentsSpin = nullptr;
    QGroupBox* m_spanDefinitionGroup = nullptr;
    QVBoxLayout* m_spanDefinitionLayout = nullptr;
    ChevronComboBox* m_modelModeCombo = nullptr;
    QWidget* m_singleSpanEditor = nullptr;
    QGridLayout* m_coordinateFormLayout = nullptr;
    QLabel* m_startLabel = nullptr;
    QWidget* m_startCoordinates = nullptr;
    QVBoxLayout* m_startLayout = nullptr;
    ChevronDoubleSpinBox* m_startX = nullptr;
    ChevronDoubleSpinBox* m_startY = nullptr;
    ChevronDoubleSpinBox* m_startZ = nullptr;
    QLabel* m_endLabel = nullptr;
    QWidget* m_endCoordinates = nullptr;
    QVBoxLayout* m_endLayout = nullptr;
    ChevronDoubleSpinBox* m_endX = nullptr;
    ChevronDoubleSpinBox* m_endY = nullptr;
    ChevronDoubleSpinBox* m_endZ = nullptr;
    QWidget* m_multiSpanEditor = nullptr;
    QLabel* m_stationHelpLabel = nullptr;
    QTableWidget* m_stationTable = nullptr;
    QHBoxLayout* m_stationButtonLayout = nullptr;
    QPushButton* m_addStationButton = nullptr;
    QPushButton* m_removeStationButton = nullptr;
    QFormLayout* m_suspensionFormLayout = nullptr;
    ChevronDoubleSpinBox* m_suspensionLengthSpin = nullptr;
    QGroupBox* m_endConnectionGroup = nullptr;
    QFormLayout* m_endConnectionFormLayout = nullptr;
    ChevronComboBox* m_endTopologyCombo = nullptr;
    QLabel* m_dualSupportSpacingLabel = nullptr;
    ChevronDoubleSpinBox* m_dualSupportSpacingSpin = nullptr;
    ChevronDoubleSpinBox* m_endTransitionSpin = nullptr;
    ChevronDoubleSpinBox* m_stressSpin = nullptr;
    QScrollArea* m_advancedScrollArea = nullptr;
    QWidget* m_advancedContent = nullptr;
    QGridLayout* m_advancedContentLayout = nullptr;
    QGroupBox* m_structuralIceGroup = nullptr;
    QFormLayout* m_structuralIceFormLayout = nullptr;
    QCheckBox* m_structuralIceCheck = nullptr;
    ChevronDoubleSpinBox* m_iceThicknessSpin = nullptr;
    ChevronDoubleSpinBox* m_bareDiameterSpin = nullptr;
    QCheckBox* m_iceEccentricityCheck = nullptr;
    QGroupBox* m_torsionGroup = nullptr;
    QFormLayout* m_torsionFormLayout = nullptr;
    ChevronDoubleSpinBox* m_torsionalRigiditySpin = nullptr;
    ChevronDoubleSpinBox* m_iceCenterAngleSpin = nullptr;
    ChevronDoubleSpinBox* m_iceHalfAngleSpin = nullptr;
    QWidget* m_spacerTab = nullptr;
    QVBoxLayout* m_spacerTabLayout = nullptr;
    QHBoxLayout* m_spacerCreationLayout = nullptr;
    QCheckBox* m_innerSpacerCheck = nullptr;
    QLabel* m_spacerCountPreview = nullptr;
    QScrollArea* m_spacerScrollArea = nullptr;
    QWidget* m_spacerContent = nullptr;
    QGridLayout* m_spacerContentLayout = nullptr;
    QGroupBox* m_spacerPropertyGroup = nullptr;
    QFormLayout* m_spacerPropertyFormLayout = nullptr;
    QLabel* m_spacerPresetLabel = nullptr;
    ChevronComboBox* m_spacerPresetCombo = nullptr;
    QLabel* m_spacerModelLabel = nullptr;
    QLineEdit* m_spacerModelEdit = nullptr;
    QLabel* m_spacerMassLabel = nullptr;
    CompactDoubleSpinBox* m_spacerMassSpin = nullptr;
    QLabel* m_spacerSpacingLabel = nullptr;
    ChevronDoubleSpinBox* m_spacerSpacingSpin = nullptr;
    QLabel* m_spacerMaterialLabel = nullptr;
    ChevronComboBox* m_spacerMaterialCombo = nullptr;
    QGroupBox* m_spacerGroup = nullptr;
    QFormLayout* m_spacerFormLayout = nullptr;
    ChevronComboBox* m_spacerStyleCombo = nullptr;
    QLabel* m_spacerElementLabel = nullptr;
    ChevronComboBox* m_spacerElementCombo = nullptr;
    ChevronComboBox* m_spacerLayoutCombo = nullptr;
    QLabel* m_spacerCountLabel = nullptr;
    ChevronSpinBox* m_spacerCountSpin = nullptr;
    QLabel* m_spacerSectionLabel = nullptr;
    ChevronComboBox* m_spacerSectionCombo = nullptr;
    QLabel* m_spacerStatusLabel = nullptr;
    QHBoxLayout* m_footerLayout = nullptr;
    QPushButton* m_viewLibraryButton = nullptr;
    QCheckBox* m_analysisCheck = nullptr;
    QPushButton* m_cancelButton = nullptr;
    QPushButton* m_createButton = nullptr;

    QDialog* m_creationDialog = nullptr;
    bool m_creationDialogInitialized = false;
    bool m_singleColumnLayout = false;
    bool m_footerStacked = false;
    bool m_dualSupportSpacingCustomized = false;
    bool m_modelNameCustomized = false;
    bool m_applyingSpacerPreset = false;
    Conductor::PropertyLibrary* m_propertyLibrary = nullptr;
    QMetaObject::Connection m_materialLibraryConnection;
    QMetaObject::Connection m_spacerMaterialLibraryConnection;
    QString m_defaultModelName = QStringLiteral("单导线");
    std::vector<double> m_explicitInnerSpacerPositions;
};
