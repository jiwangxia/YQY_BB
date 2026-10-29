#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <QString>
#include "Base/EmptyOUT.h"
#include <array>
#include <functional>
#include <vector>

namespace SolverNameSpace
{
enum class StructuralDampingFormulation
{
    LocalRayleigh = 1
};

struct StructuralDampingSettings
{
    bool enabled = false;
    StructuralDampingFormulation formulation = StructuralDampingFormulation::LocalRayleigh;
    double translationDampingRatio = 5.0e-3;
    double torsionDampingRatio = 3.8e-2;
    bool useExplicitRayleighCoefficients = false;
    double translationMassCoefficient = 0.0;
    double translationStiffnessCoefficient = 0.0;
    double torsionMassCoefficient = 0.0;
    double torsionStiffnessCoefficient = 0.0;
    // 全部为零表示各族最低两个有效模态；否则为用户指定的两个完整模态阶号。
    std::array<int, 2> translationReferenceModes{};
    std::array<int, 2> torsionReferenceModes{};
};

struct StructuralDampingModeReport
{
    int modeNumber = 0;
    double frequencyHz = 0.0;
    bool torsion = false;
    double torsionFraction = 0.0;
    double residual = 0.0;
    bool valid = false;
    QString rejectionReason;
    bool isReferenceMode = false;
    double targetDampingRatio = 0.0;
    double projectedDampingRatio = 0.0;
};

struct LocalRayleighGroupBasis
{
    QString label;
    std::array<Eigen::SparseMatrix<double>, 4> basis;
};

struct LocalRayleighGroupReport
{
    QString label;
    std::array<double, 4> coefficients{};
};

struct StructuralDampingReport
{
    // 原始完整模型的自由度数量。
    int originalDofCount = 0;
    // MPC 约化后用于模态求解的自由度数量。
    int modalDofCount = 0;
    // 最终用于确定 Rayleigh 系数的参考模态数量。
    int selectedModeCount = 0;
    // 用于确定扭转 Rayleigh 系数的参考模态数量。
    int torsionReferenceModeCount = 0;
    // 一次模态求解的请求、收敛与通过残差及质量检查的数量。
    int requestedModeCount = 0;
    int convergedModeCount = 0;
    int validModeCount = 0;
    // 第一个平动参考模态频率，单位 Hz。
    double translationReferenceFrequency1Hz = 0.0;
    // 第二个平动参考模态频率，单位 Hz。
    double translationReferenceFrequency2Hz = 0.0;
    // 第一个扭转参考模态频率，单位 Hz。
    double torsionReferenceFrequency1Hz = 0.0;
    // 第二个扭转参考模态频率，单位 Hz。
    double torsionReferenceFrequency2Hz = 0.0;
    // 完整非索单元及索平动通道的质量比例系数。
    double translationMassCoefficient = 0.0;
    // 完整非索单元及索平动通道的刚度比例系数。
    double translationStiffnessCoefficient = 0.0;
    // 索自扭转通道的质量比例系数。
    double torsionMassCoefficient = 0.0;
    // 索自扭转通道的刚度比例系数。
    double torsionStiffnessCoefficient = 0.0;
    // 分组标定时各组独立使用固定系数；空列表表示使用统一四系数。
    std::vector<LocalRayleighGroupReport> localGroups;
    // 系数和实际矩阵在参考模态上的最大绝对回代误差。
    double maximumBackCheckError = 0.0;
    double maximumCoefficientBackCheckError = 0.0;
    double maximumReferenceProjectionError = 0.0;
    double minimumMassNormalizedDampingEigenvalue = 0.0;
    double dissipationEigenvalueTolerance = 0.0;
    double dampingMatrixRelativeAsymmetry = 0.0;
    bool matrixValidationPassed = false;
    double referenceProjectionTolerance = 1.0e-8;
    int translationGroupModeCount = 0;
    int torsionGroupModeCount = 0;
    int supplementalModeCount = 0;
    int degenerateModeRotationCount = 0;
    int degenerateModeRotationRejectedCount = 0;
    std::vector<StructuralDampingModeReport> modeSpectrum;
    // N−1 阶之外补全的末阶也参与分类、参考点选择和实际矩阵回代。
    std::vector<StructuralDampingModeReport> supplementalModeSpectrum;
    // 结构阻尼计算过程的摘要信息。
    QString summary;
};

class StructuralDampingModel
{
public:
    StructuralDampingSettings settings;
    StructuralDampingReport report;

    bool Calculate(const Eigen::SparseMatrix<double>& stiffness, const Eigen::SparseMatrix<double>& mass,
                   const std::array<Eigen::SparseMatrix<double>, 4>& localDampingBasis,
                   const std::vector<bool>& torsionDofs, const Eigen::SparseMatrix<double>& modalTransformation,
                   _OUT QString& errorMessage,
                   const std::function<double(const Eigen::VectorXd&)>& torsionClassifier = {},
                   const Eigen::SparseMatrix<double>& torsionMetric = {},
                   const std::vector<LocalRayleighGroupBasis>& groupedBasis = {});
    bool ValidateRayleighMatrix(const Eigen::SparseMatrix<double>& mass, const Eigen::SparseMatrix<double>& damping,
                                const Eigen::SparseMatrix<double>& modalTransformation, _OUT QString& errorMessage);
    void Reset();
    void Disable();

    bool IsComputed() const
    {
        return computed_;
    }

private:
    bool CalibrateLocalRayleigh(const Eigen::SparseMatrix<double>& reducedStiffness,
                                const Eigen::SparseMatrix<double>& reducedMass, const Eigen::SparseMatrix<double>& mass,
                                const std::array<Eigen::SparseMatrix<double>, 4>& localDampingBasis,
                                const std::vector<bool>& torsionDofs,
                                const Eigen::SparseMatrix<double>& modalTransformation, _OUT QString& errorMessage,
                                const std::function<double(const Eigen::VectorXd&)>& torsionClassifier,
                                const Eigen::SparseMatrix<double>& torsionMetric,
                                const std::vector<LocalRayleighGroupBasis>& groupedBasis);
    void RotateDegenerateModeGroups(const Eigen::SparseMatrix<double>& reducedStiffness,
                                    const Eigen::SparseMatrix<double>& reducedMass,
                                    const Eigen::SparseMatrix<double>& modalTransformation,
                                    const Eigen::SparseMatrix<double>& torsionMetric, _OUT Eigen::VectorXd& eigenvalues,
                                    _OUT Eigen::MatrixXd& eigenvectors);
    StructuralDampingModeReport& ModeReport(int index);
    const StructuralDampingModeReport& ModeReport(int index) const;
    bool SelectReferenceModes(const std::array<int, 2>& requestedModes, const std::vector<int>& groupIndices,
                              const QString& familyName, _OUT std::vector<int>& selectedIndices,
                              _OUT QString& errorMessage) const;
    bool FitLocalCoefficients(const Eigen::SparseMatrix<double>& mass,
                              const std::array<Eigen::SparseMatrix<double>, 4>& localDampingBasis,
                              const std::vector<int>& referenceIndices, _OUT QString& errorMessage);
    bool FitGroupedLocalCoefficients(const Eigen::SparseMatrix<double>& mass,
                                     const std::vector<LocalRayleighGroupBasis>& groupedBasis,
                                     const std::vector<int>& referenceIndices, _OUT QString& errorMessage);
    static bool FindNonnegativeFeasiblePoint(const Eigen::MatrixXd& response, const Eigen::VectorXd& target,
                                             _OUT Eigen::VectorXd& coefficients);
    static bool MinimizeNonnegativeNorm(const Eigen::MatrixXd& response, const Eigen::VectorXd& target,
                                        _OUT Eigen::VectorXd& coefficients);
    bool calibrationReady_ = false;
    bool computed_ = false;
    // 仅在初始化矩阵回代完成前保留振型。
    Eigen::MatrixXd rayleighModeShapes_;
};
}
