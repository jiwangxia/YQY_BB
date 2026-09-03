#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <QString>
#include "Base/EmptyOUT.h"
#include <vector>

namespace SolverNameSpace
{
struct StructuralDampingSettings
{
    bool enabled = false;
    double translationDampingRatio = 5.0e-3;
    double torsionDampingRatio = 3.8e-2;
    double maximumFrequencyHz = 3.0;
};

struct StructuralDampingReport
{
    // 原始完整模型的自由度数量。
    int originalDofCount = 0;
    // MPC 约化后用于模态求解的自由度数量。
    int modalDofCount = 0;
    // 最终参与结构阻尼构造的模态数量。
    int selectedModeCount = 0;
    // 用于确定扭转 Rayleigh 系数的参考模态数量。
    int torsionReferenceModeCount = 0;
    // 第一个平动参考模态频率，单位 Hz。
    double translationReferenceFrequency1Hz = 0.0;
    // 第二个平动参考模态频率，单位 Hz。
    double translationReferenceFrequency2Hz = 0.0;
    // 第一个扭转参考模态频率，单位 Hz。
    double torsionReferenceFrequency1Hz = 0.0;
    // 第二个扭转参考模态频率，单位 Hz。
    double torsionReferenceFrequency2Hz = 0.0;
    // 根据平动参考模态和目标阻尼比计算的质量比例系数。
    double translationMassCoefficient = 0.0;
    // 根据平动参考模态和目标阻尼比计算的刚度比例系数。
    double translationStiffnessCoefficient = 0.0;
    // 根据扭转参考模态和目标阻尼比计算的质量比例系数。
    double torsionMassCoefficient = 0.0;
    // 根据扭转参考模态和目标阻尼比计算的刚度比例系数。
    double torsionStiffnessCoefficient = 0.0;
    // 各选中模态回代得到的阻尼比与目标阻尼比之间的最大绝对误差。
    double maximumBackCheckError = 0.0;
    // 结构阻尼计算过程的摘要信息。
    QString summary;
};

class StructuralDampingModel
{
public:
    StructuralDampingSettings settings;
    StructuralDampingReport report;

    bool Calculate(const Eigen::SparseMatrix<double>& stiffness, const Eigen::SparseMatrix<double>& mass,
                   const std::vector<bool>& torsionDofs, const Eigen::SparseMatrix<double>& modalTransformation,
                   _OUT QString& errorMessage);
    void Reset();
    void Disable();

    bool IsComputed() const
    {
        return computed_;
    }

    const Eigen::MatrixXd& Factor() const
    {
        return factor_;
    }

    Eigen::VectorXd Apply(const Eigen::VectorXd& velocity) const;

private:
    bool computed_ = false;
    Eigen::MatrixXd factor_;
};
}
