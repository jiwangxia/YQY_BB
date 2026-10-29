#pragma once

#include "AnalysisEditor.h"
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "DataStructure/Load/LoadBase.h"
#include "Base/EmptyOUT.h"
#include <array>
#include <vector>
#include <functional>

class LoadTimeFunctionPreview;
class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QLineEdit;
class QScrollArea;
class QResizeEvent;
class QSplitter;
class QSpinBox;
class QVBoxLayout;
class QWidget;

class LoadEditorDialog final : public AnalysisLoadEditor
{
private:
    void initializeUi();                                                                                               // 创建控件和布局

    // 隐藏页不参与尺寸计算，常量函数不再为正弦函数的四行参数保留空间。
    class ParameterStack;

public:
    LoadEditorDialog(int id, const std::shared_ptr<StructureData>& structure, const std::shared_ptr<LoadBase>& load,
                     QWidget* parent);                                                                                 // 创建荷载编辑器并载入已有荷载

    std::shared_ptr<LoadBase> value() const override;                                                                  // 返回确认后的荷载对象

protected:
    void accept() override;                                                                                            // 校验作用目标和时间函数并确认荷载
    void resizeEvent(QResizeEvent* event) override;                                                                    // 在窗口尺寸变化时更新荷载表单布局

private:
    QWidget* createField(QLabel* label, QWidget* editor);                                                              // 将标签和编辑控件组合为表单字段

    QScrollArea* createFormScroll(QWidget* body, const QString& name);                                                 // 为参数表单创建滚动容器

    static QDoubleSpinBox* createTimeSpinBox(QWidget* parent);                                                         // 创建时间参数输入框

    static QDoubleSpinBox* createParameterSpinBox(QWidget* parent, double minimum = -1.0e12, double maximum = 1.0e12); // 创建指定取值范围的函数参数输入框

    void addFunctionType(TimeFunctionType type);                                                                       // 向时间函数选择框添加指定类型

    QWidget* createConstantParametersPage();                                                                           // 创建恒定时间函数参数页

    QWidget* createHarmonicParametersPage();                                                                           // 创建谐波时间函数参数页

    QWidget* createRampParametersPage();                                                                               // 创建斜坡时间函数参数页

    QWidget* createExponentialParametersPage();                                                                        // 创建指数时间函数参数页

    QWidget* createTriangularParametersPage();                                                                         // 创建三角波时间函数参数页

    QWidget* createSquareParametersPage();                                                                             // 创建方波时间函数参数页

    double suggestedEndTime() const;                                                                                   // 根据分析步给出建议荷载结束时间

    std::shared_ptr<LoadBase> previewLoad() const;                                                                     // 按当前输入构造用于预览的荷载

    void connectTimeFunctionControls();                                                                                // 连接时间函数参数与预览更新事件

    void updateTimeFunctionDisplay();                                                                                  // 更新时间函数参数页、表达式和预览

    void updateTargetState();                                                                                          // 按荷载类型调整目标及分量输入状态

    int m_id = 0;
    std::shared_ptr<StructureData> m_structure;
    std::shared_ptr<LoadBase> m_original;
    std::shared_ptr<LoadBase> m_result;
    QLineEdit* m_name = nullptr;
    QScrollArea* m_editorScroll = nullptr;
    QScrollArea* m_previewScroll = nullptr;
    QSplitter* m_contentSplitter = nullptr;
    ParameterStack* m_targetStack = nullptr;
    QLabel* m_targetLabel = nullptr;
    QComboBox* m_step = nullptr;
    QComboBox* m_type = nullptr;
    QSpinBox* m_target = nullptr;
    QComboBox* m_direction = nullptr;
    QLabel* m_directionLabel = nullptr;
    ParameterStack* m_directionStack = nullptr;
    QDoubleSpinBox* m_windDirection[3] = {};
    QDoubleSpinBox* m_value = nullptr;
    QLabel* m_valueLabel = nullptr;
    ParameterStack* m_valueStack = nullptr;
    QComboBox* m_windSpeed = nullptr;
    QDoubleSpinBox* m_startTime = nullptr;
    QDoubleSpinBox* m_endTime = nullptr;
    QCheckBox* m_continuous = nullptr;
    QComboBox* m_function = nullptr;
    ParameterStack* m_functionParameters = nullptr;
    QDoubleSpinBox* m_amplitude = nullptr;
    QDoubleSpinBox* m_frequency = nullptr;
    QDoubleSpinBox* m_phase = nullptr;
    QDoubleSpinBox* m_offset = nullptr;
    QDoubleSpinBox* m_rampT0 = nullptr;
    QDoubleSpinBox* m_rampT1 = nullptr;
    QDoubleSpinBox* m_decay = nullptr;
    QDoubleSpinBox* m_trianglePeriod = nullptr;
    QDoubleSpinBox* m_squarePeriod = nullptr;
    QDoubleSpinBox* m_dutyCycle = nullptr;
    QLabel* m_typeSummary = nullptr;
    QLabel* m_expression = nullptr;
    QLabel* m_timeSummary = nullptr;
    LoadTimeFunctionPreview* m_preview = nullptr;
    QDoubleSpinBox* m_previewTime = nullptr;
    QLabel* m_previewValue = nullptr;

    QVBoxLayout* m_rootLayout = nullptr;

    QLabel* m_nameLabel = nullptr;
    QLabel* m_stepLabel = nullptr;
    QLabel* m_typeLabel = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
