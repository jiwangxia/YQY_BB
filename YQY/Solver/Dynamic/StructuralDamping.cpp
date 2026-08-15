#include "StructuralDamping.h"

#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/MatOp/SymShiftInvert.h>
#include <Spectra/SymGEigsShiftSolver.h>

#include <Eigen/SparseCholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>

namespace SolverNameSpace
{
namespace
{
constexpr double kTwoPi = 6.283185307179586476925286766559;

struct ModalCandidate
{
    double omega = 0.0;
    double frequencyHz = 0.0;
    double torsionFraction = 0.0;
    Eigen::VectorXd shape;
};

bool IsValidRatio(double value)
{
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

bool ExtractModes(const Eigen::SparseMatrix<double>& reducedStiffness, const Eigen::SparseMatrix<double>& reducedMass,
                  const Eigen::SparseMatrix<double>& fullMass, const Eigen::SparseMatrix<double>& transformation,
                  const std::vector<bool>& torsionDofs, int requestedModes,
                  _OUT std::vector<ModalCandidate>& translationModes, _OUT std::vector<ModalCandidate>& torsionModes,
                  _OUT QString& errorMessage)
{
    const int matrixSize = reducedStiffness.rows();
    const int modeCount = std::min(requestedModes, matrixSize <= 32 ? matrixSize : matrixSize - 1);
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenvectors;
    int converged = 0;
    if (matrixSize <= 32)
    {
        const Eigen::MatrixXd denseStiffness(reducedStiffness);
        const Eigen::MatrixXd denseMass(reducedMass);
        Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver(denseStiffness, denseMass);
        if (solver.info() == Eigen::Success)
        {
            converged = modeCount;
            eigenvalues = solver.eigenvalues().head(modeCount);
            eigenvectors = solver.eigenvectors().leftCols(modeCount);
        }
    }
    else
    {
        const int subspaceSize = std::min(matrixSize, std::max(modeCount + 2, 2 * modeCount + 1));
        using ShiftOperation = Spectra::SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
        using MassOperation = Spectra::SparseSymMatProd<double>;
        ShiftOperation shiftOperation(reducedStiffness, reducedMass);
        MassOperation massOperation(reducedMass);
        Spectra::SymGEigsShiftSolver<ShiftOperation, MassOperation, Spectra::GEigsMode::ShiftInvert> solver(
            shiftOperation, massOperation, modeCount, subspaceSize, 0.0);
        solver.init();
        converged = solver.compute(Spectra::SortRule::LargestMagn, 1000, 1.0e-10,
                                   Spectra::SortRule::SmallestAlge);
        if (solver.info() == Spectra::CompInfo::Successful)
        {
            eigenvalues = solver.eigenvalues();
            eigenvectors = solver.eigenvectors();
        }
    }
    if (converged < std::min(4, modeCount))
    {
        errorMessage = QStringLiteral("广义模态求解未收敛：请求 %1 个，收敛 %2 个。").arg(modeCount).arg(converged);
        return false;
    }

    translationModes.clear();
    torsionModes.clear();
    for (int modeIndex = 0; modeIndex < eigenvalues.size(); ++modeIndex)
    {
        const double eigenvalue = eigenvalues[modeIndex];
        if (!std::isfinite(eigenvalue) || eigenvalue <= 1.0e-10)
            continue;
        const Eigen::VectorXd reducedShape = eigenvectors.col(modeIndex);
        const Eigen::VectorXd stiffnessProduct = reducedStiffness * reducedShape;
        const double residual =
            (stiffnessProduct - eigenvalue * (reducedMass * reducedShape)).norm() /
            std::max(1.0, stiffnessProduct.norm());
        if (!std::isfinite(residual) || residual > 1.0e-6)
            continue;

        const Eigen::VectorXd fullShape = transformation * reducedShape;
        Eigen::VectorXd torsionShape = Eigen::VectorXd::Zero(fullShape.size());
        Eigen::VectorXd translationShape = fullShape;
        for (int dof = 0; dof < fullShape.size(); ++dof)
        {
            if (!torsionDofs[static_cast<std::size_t>(dof)])
                continue;
            torsionShape[dof] = fullShape[dof];
            translationShape[dof] = 0.0;
        }
        const double translationWeight = std::max(0.0, translationShape.dot(fullMass * translationShape));
        const double torsionWeight = std::max(0.0, torsionShape.dot(fullMass * torsionShape));
        const double classifiedWeight = translationWeight + torsionWeight;
        if (!std::isfinite(classifiedWeight) || classifiedWeight <= 1.0e-20)
            continue;
        const double omega = std::sqrt(eigenvalue);
        ModalCandidate candidate{omega, omega / kTwoPi, torsionWeight / classifiedWeight, fullShape};
        (candidate.torsionFraction >= 0.5 ? torsionModes : translationModes).push_back(std::move(candidate));
    }
    return true;
}

void SetReferenceCoefficients(double firstFrequency, double secondFrequency, double ratio,
                              _OUT double& massCoefficient, _OUT double& stiffnessCoefficient)
{
    const double firstOmega = kTwoPi * firstFrequency;
    const double secondOmega = kTwoPi * secondFrequency;
    massCoefficient = 2.0 * ratio * firstOmega * secondOmega / (firstOmega + secondOmega);
    stiffnessCoefficient = 2.0 * ratio / (firstOmega + secondOmega);
}
}

bool StructuralDampingModel::Calculate(const Eigen::SparseMatrix<double>& stiffness,
                                       const Eigen::SparseMatrix<double>& mass,
                                       const std::vector<bool>& torsionDofs,
                                       const Eigen::SparseMatrix<double>& modalTransformation, QString& errorMessage)
{
    Reset();
    if (!settings.enabled)
        return true;
    if (!IsValidRatio(settings.translationDampingRatio) || !IsValidRatio(settings.torsionDampingRatio) ||
        !std::isfinite(settings.maximumFrequencyHz) || settings.maximumFrequencyHz <= 0.0)
    {
        errorMessage = QStringLiteral("结构阻尼参数无效：阻尼比应在 0% 到 100% 之间，最高关注频率必须大于零。");
        return false;
    }
    if (stiffness.rows() != stiffness.cols() || mass.rows() != mass.cols() || stiffness.rows() != mass.rows() ||
        stiffness.rows() < 2 || static_cast<int>(torsionDofs.size()) != stiffness.rows() ||
        modalTransformation.rows() != stiffness.rows() || modalTransformation.cols() < 4)
    {
        errorMessage = QStringLiteral("结构阻尼矩阵尺寸与自由度不一致，或独立自由度不足五个。");
        return false;
    }

    Eigen::SparseMatrix<double> reducedStiffness = modalTransformation.transpose() * stiffness * modalTransformation;
    Eigen::SparseMatrix<double> reducedMass = modalTransformation.transpose() * mass * modalTransformation;
    reducedStiffness = 0.5 * (reducedStiffness + Eigen::SparseMatrix<double>(reducedStiffness.transpose()));
    reducedMass = 0.5 * (reducedMass + Eigen::SparseMatrix<double>(reducedMass.transpose()));
    reducedStiffness.makeCompressed();
    reducedMass.makeCompressed();

    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> massCheck;
    massCheck.compute(reducedMass);
    if (massCheck.info() != Eigen::Success)
    {
        errorMessage = QStringLiteral("MPC 约化后的自由质量矩阵不是正定矩阵，请检查零质量自由度和间隔棒质量。");
        return false;
    }

    const int availableModes = reducedStiffness.rows() <= 32 ? reducedStiffness.rows() : reducedStiffness.rows() - 1;
    int requestedModes = std::min(20, availableModes);
    std::vector<ModalCandidate> translationModes;
    std::vector<ModalCandidate> torsionModes;
    while (true)
    {
        if (!ExtractModes(reducedStiffness, reducedMass, mass, modalTransformation, torsionDofs, requestedModes,
                          translationModes, torsionModes, errorMessage))
            return false;
        double highestFrequency = 0.0;
        if (!translationModes.empty())
            highestFrequency = translationModes.back().frequencyHz;
        if (!torsionModes.empty())
            highestFrequency = std::max(highestFrequency, torsionModes.back().frequencyHz);
        const bool referenceModesReady = translationModes.size() >= 2 && torsionModes.size() >= 2;
        if (referenceModesReady &&
            (highestFrequency > settings.maximumFrequencyHz || requestedModes >= availableModes))
            break;
        if (requestedModes >= availableModes)
        {
            errorMessage = QStringLiteral("已搜索全部 %1 个可用模态，仍未越过 %2 Hz 或未找到两个平动、两个扭转模态（%3/%4）。")
                               .arg(availableModes)
                               .arg(settings.maximumFrequencyHz, 0, 'g', 8)
                               .arg(translationModes.size())
                               .arg(torsionModes.size());
            return false;
        }
        requestedModes = std::min(availableModes, requestedModes + 20);
    }

    struct SelectedMode
    {
        const ModalCandidate* mode = nullptr;
        bool torsion = false;
    };
    std::vector<SelectedMode> selectedModes;
    int selectedTorsionCount = 0;
    for (const ModalCandidate& mode : translationModes)
        if (mode.frequencyHz <= settings.maximumFrequencyHz)
            selectedModes.push_back({&mode, false});
    for (const ModalCandidate& mode : torsionModes)
        if (mode.frequencyHz <= settings.maximumFrequencyHz)
        {
            selectedModes.push_back({&mode, true});
            ++selectedTorsionCount;
        }
    for (const ModalCandidate& mode : torsionModes)
        if (mode.frequencyHz > settings.maximumFrequencyHz && selectedTorsionCount < 2)
        {
            selectedModes.push_back({&mode, true});
            ++selectedTorsionCount;
        }
    if (selectedModes.empty() || selectedTorsionCount < 2)
    {
        errorMessage = QStringLiteral("关注频段内没有可用模态，或缺少两个扭转参考模态。");
        return false;
    }
    std::sort(selectedModes.begin(), selectedModes.end(), [](const SelectedMode& left, const SelectedMode& right)
              { return left.mode->frequencyHz < right.mode->frequencyHz; });

    factor_.resize(stiffness.rows(), static_cast<int>(selectedModes.size()));
    int factorColumn = 0;
    for (const SelectedMode& selected : selectedModes)
    {
        const double modalMass = selected.mode->shape.dot(mass * selected.mode->shape);
        if (!std::isfinite(modalMass) || modalMass <= std::numeric_limits<double>::epsilon())
        {
            errorMessage = QStringLiteral("结构阻尼模态的广义质量无效。");
            return false;
        }
        const double targetRatio =
            selected.torsion ? settings.torsionDampingRatio : settings.translationDampingRatio;
        const Eigen::VectorXd massMode = mass * selected.mode->shape;
        const double factorScale = std::sqrt(2.0 * targetRatio * selected.mode->omega / modalMass);
        factor_.col(factorColumn++) = factorScale * massMode;
    }

    double maximumBackCheckError = 0.0;
    for (const SelectedMode& selected : selectedModes)
    {
        const double modalMass = selected.mode->shape.dot(mass * selected.mode->shape);
        const double targetRatio =
            selected.torsion ? settings.torsionDampingRatio : settings.translationDampingRatio;
        const double projectedNorm = (factor_.transpose() * selected.mode->shape).squaredNorm();
        const double actualRatio = projectedNorm / (2.0 * selected.mode->omega * modalMass);
        maximumBackCheckError = std::max(maximumBackCheckError, std::abs(actualRatio - targetRatio));
    }
    if (!std::isfinite(maximumBackCheckError) || maximumBackCheckError > 1.0e-8)
    {
        errorMessage = QStringLiteral("结构阻尼回代校核失败，最大阻尼比误差为 %1。")
                           .arg(maximumBackCheckError, 0, 'g', 8);
        Reset();
        return false;
    }

    report.originalDofCount = stiffness.rows();
    report.modalDofCount = modalTransformation.cols();
    report.selectedModeCount = static_cast<int>(selectedModes.size());
    report.torsionReferenceModeCount = 2;
    report.translationReferenceFrequency1Hz = translationModes[0].frequencyHz;
    report.translationReferenceFrequency2Hz = translationModes[1].frequencyHz;
    report.torsionReferenceFrequency1Hz = torsionModes[0].frequencyHz;
    report.torsionReferenceFrequency2Hz = torsionModes[1].frequencyHz;
    SetReferenceCoefficients(report.translationReferenceFrequency1Hz, report.translationReferenceFrequency2Hz,
                             settings.translationDampingRatio, report.translationMassCoefficient,
                             report.translationStiffnessCoefficient);
    SetReferenceCoefficients(report.torsionReferenceFrequency1Hz, report.torsionReferenceFrequency2Hz,
                             settings.torsionDampingRatio, report.torsionMassCoefficient,
                             report.torsionStiffnessCoefficient);
    report.maximumBackCheckError = maximumBackCheckError;
    report.summary =
        QStringLiteral("结构阻尼已生成：自由度 %1→%2，搜索 %3 个模态，使用 %4 个；扭转参考 %5/%6 Hz，最大回代误差 %7。")
            .arg(report.originalDofCount)
            .arg(report.modalDofCount)
            .arg(requestedModes)
            .arg(report.selectedModeCount)
            .arg(report.torsionReferenceFrequency1Hz, 0, 'g', 8)
            .arg(report.torsionReferenceFrequency2Hz, 0, 'g', 8)
            .arg(maximumBackCheckError, 0, 'g', 8);
    computed_ = true;
    return true;
}

void StructuralDampingModel::Reset()
{
    computed_ = false;
    factor_.resize(0, 0);
    report = {};
}

Eigen::VectorXd StructuralDampingModel::Apply(const Eigen::VectorXd& velocity) const
{
    if (!computed_ || velocity.size() != factor_.rows())
        return Eigen::VectorXd::Zero(velocity.size());
    return factor_ * (factor_.transpose() * velocity);
}

void StructuralDampingModel::Disable()
{
    settings.enabled = false;
    Reset();
}
}
