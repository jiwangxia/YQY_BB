#pragma once

#include <QGroupBox>
#include "Solver/Dynamic/TimeStepping/AdaptiveTssbnSettings.h"
#include "Base/EmptyOUT.h"
#include <array>
#include <vector>
#include <variant>
#include <functional>

class QAbstractSpinBox;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QWidget;

class AdaptiveTssbnSettingsEditor final : public QGroupBox
{
public:
    using Settings = SolverNameSpace::AdaptiveTssbnSettings;

    explicit AdaptiveTssbnSettingsEditor(QWidget* parent); // 创建自适应 TSSBN 参数编辑面板

    void setSettings(const Settings& settings);            // 将已有算法参数填入编辑控件

    void setReferenceMode(bool enabled);                   // 切换参考算法模式及参数可编辑状态

    Settings settings() const;                             // 读取编辑后的算法参数

    void commitEditors();                                  // 提交数值框中尚未确认的输入

private:
    void applyReferenceSettings();                         // 将参考算法参数应用到编辑控件

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
        std::size_t section;
        int row;
        int column;
    };

    struct Binding
    {
        Member member;
        QAbstractSpinBox* editor;
    };

    std::vector<Binding> m_bindings;
    QComboBox* m_errorNormMode = nullptr;
    QCheckBox* m_referenceIndependentError = nullptr;
    QDoubleSpinBox* m_absoluteTolerance = nullptr;
    QDoubleSpinBox* m_velocityAbsoluteTolerance = nullptr;
    bool m_referenceMode = false;
    SolverNameSpace::TssbnErrorNormMode m_regularErrorNormMode = Settings{}.errorNormMode;
    double m_regularVelocityTolerance = Settings{}.velocityAbsoluteTolerance;
    double m_referenceIndependentVelocityTolerance = Settings{}.velocityAbsoluteTolerance;
};
