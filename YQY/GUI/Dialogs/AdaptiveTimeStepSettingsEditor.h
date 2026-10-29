#pragma once

#include <QGroupBox>
#include "Solver/Dynamic/TimeStepping/TimeStepControl.h"
#include "Base/EmptyOUT.h"
#include <array>
#include <vector>
#include <functional>

class QDoubleSpinBox;
class QGridLayout;
class QLabel;
class QSpinBox;
class QWidget;

class AdaptiveTimeStepSettingsEditor final : public QGroupBox
{
public:
    using Settings = SolverNameSpace::AdaptiveTimeStepSettings;

    explicit AdaptiveTimeStepSettingsEditor(QWidget* parent);        // 创建自适应时间步参数编辑面板

    void setSettings(const Settings& settings);                      // 将已有算法参数填入编辑控件

    Settings settings() const;                                       // 读取编辑后的算法参数

    void commitEditors();                                            // 提交数值框中尚未确认的输入

private:
    QLabel* createSectionTitle(const QString& text);                 // 创建时间步参数分组标题

    void addDoubleField(QGridLayout* layout, int row, int column, const QString& text, double minimum, double maximum,
                        int decimals, _OUT QDoubleSpinBox*& editor); // 向网格添加带范围和精度限制的数值字段

    QDoubleSpinBox* m_minimumTimeStep = nullptr;
    QDoubleSpinBox* m_maximumTimeStep = nullptr;
    QDoubleSpinBox* m_relativeTolerance = nullptr;
    QDoubleSpinBox* m_absoluteTolerance = nullptr;
    QDoubleSpinBox* m_velocityRelativeTolerance = nullptr;
    QDoubleSpinBox* m_velocityAbsoluteTolerance = nullptr;
    QDoubleSpinBox* m_safetyFactor = nullptr;
    QDoubleSpinBox* m_minimumScaleFactor = nullptr;
    QDoubleSpinBox* m_maximumGrowthFactor = nullptr;
    QDoubleSpinBox* m_solveFailureFactor = nullptr;
    QSpinBox* m_maximumRejectedAttempts = nullptr;
    std::vector<QDoubleSpinBox*> m_doubleEditors;
};
