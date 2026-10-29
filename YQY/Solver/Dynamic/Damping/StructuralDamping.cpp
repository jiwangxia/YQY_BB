#include "StructuralDamping.h"

#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/MatOp/SymShiftInvert.h>
#include <Spectra/SymGEigsShiftSolver.h>

#include <Eigen/SparseCholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>

namespace SolverNameSpace
{
static constexpr double kTwoPi = 6.283185307179586476925286766559;

static bool IsValidRatio(double value)
{
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

StructuralDampingModeReport& StructuralDampingModel::ModeReport(int index)
{
    if (index < report.requestedModeCount)
        return report.modeSpectrum[index];
    return report.supplementalModeSpectrum[index - report.requestedModeCount];
}

const StructuralDampingModeReport& StructuralDampingModel::ModeReport(int index) const
{
    if (index < report.requestedModeCount)
        return report.modeSpectrum[index];
    return report.supplementalModeSpectrum[index - report.requestedModeCount];
}

bool StructuralDampingModel::SelectReferenceModes(const std::array<int, 2>& requestedModes,
                                                  const std::vector<int>& groupIndices, const QString& familyName,
                                                  _OUT std::vector<int>& selectedIndices,
                                                  _OUT QString& errorMessage) const
{
    selectedIndices.clear();
    if (requestedModes[0] == 0 && requestedModes[1] == 0)
    {
        const int count = std::min(2, static_cast<int>(groupIndices.size()));
        selectedIndices.assign(groupIndices.begin(), groupIndices.begin() + count);
        return true;
    }
    if (requestedModes[0] <= 0 || requestedModes[1] <= 0 || requestedModes[0] == requestedModes[1])
    {
        errorMessage = QStringLiteral("%1参考模态应同时自动选择，或指定两个不同的正整数阶号。").arg(familyName);
        return false;
    }
    for (int number : requestedModes)
    {
        const int index = number - 1;
        if (std::find(groupIndices.begin(), groupIndices.end(), index) == groupIndices.end())
        {
            errorMessage =
                QStringLiteral("指定第 %1 阶不是有效%2模态，不能作为该族阻尼参考点。").arg(number).arg(familyName);
            selectedIndices.clear();
            return false;
        }
        selectedIndices.push_back(index);
    }
    return true;
}

bool StructuralDampingModel::Calculate(
    const Eigen::SparseMatrix<double>& stiffness, const Eigen::SparseMatrix<double>& mass,
    const std::array<Eigen::SparseMatrix<double>, 4>& localDampingBasis, const std::vector<bool>& torsionDofs,
    const Eigen::SparseMatrix<double>& modalTransformation, _OUT QString& errorMessage,
    const std::function<double(const Eigen::VectorXd&)>& torsionClassifier,
    const Eigen::SparseMatrix<double>& torsionMetric, const std::vector<LocalRayleighGroupBasis>& groupedBasis)
{
    Reset();
    if (!settings.enabled)
        return true;
    if (!IsValidRatio(settings.translationDampingRatio) || !IsValidRatio(settings.torsionDampingRatio))
    {
        errorMessage = QStringLiteral("结构阻尼参数无效：阻尼比应在 0% 到 100% 之间。");
        return false;
    }
    if (stiffness.rows() != stiffness.cols() || mass.rows() != mass.cols() || stiffness.rows() != mass.rows() ||
        stiffness.rows() < 1 || static_cast<int>(torsionDofs.size()) != stiffness.rows() ||
        modalTransformation.rows() != stiffness.rows() || modalTransformation.cols() < 1)
    {
        errorMessage = QStringLiteral("结构阻尼矩阵尺寸与自由度不一致，或独立自由度不满足模态求解要求。");
        return false;
    }
    if (torsionMetric.size() != 0 && (torsionMetric.rows() != mass.rows() || torsionMetric.cols() != mass.cols()))
    {
        errorMessage = QStringLiteral("结构阻尼物理扭转分类矩阵尺寸与原始自由度不一致。");
        return false;
    }

    for (const auto& basis : localDampingBasis)
    {
        if (basis.rows() != mass.rows() || basis.cols() != mass.cols())
        {
            errorMessage = QStringLiteral("局部 Rayleigh 单位系数矩阵尺寸与自由度不一致。");
            return false;
        }
        for (int column = 0; column < basis.outerSize(); ++column)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator entry(basis, column); entry; ++entry)
            {
                if (std::isfinite(entry.value()))
                    continue;
                errorMessage = QStringLiteral("局部 Rayleigh 单位系数矩阵含非有限值。");
                return false;
            }
        }
    }
    for (const auto& group : groupedBasis)
    {
        for (const auto& basis : group.basis)
        {
            if (basis.rows() != mass.rows() || basis.cols() != mass.cols())
            {
                errorMessage = QStringLiteral("局部 Rayleigh 分组 %1 的单位矩阵尺寸不一致。").arg(group.label);
                return false;
            }
            for (int column = 0; column < basis.outerSize(); ++column)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator entry(basis, column); entry; ++entry)
                {
                    if (std::isfinite(entry.value()))
                        continue;
                    errorMessage = QStringLiteral("局部 Rayleigh 分组 %1 的单位矩阵含非有限值。").arg(group.label);
                    return false;
                }
            }
        }
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

    return CalibrateLocalRayleigh(reducedStiffness, reducedMass, mass, localDampingBasis, torsionDofs,
                                  modalTransformation, errorMessage, torsionClassifier, torsionMetric, groupedBasis);
}

void StructuralDampingModel::RotateDegenerateModeGroups(const Eigen::SparseMatrix<double>& reducedStiffness,
                                                        const Eigen::SparseMatrix<double>& reducedMass,
                                                        const Eigen::SparseMatrix<double>& modalTransformation,
                                                        const Eigen::SparseMatrix<double>& torsionMetric,
                                                        _OUT Eigen::VectorXd& eigenvalues,
                                                        _OUT Eigen::MatrixXd& eigenvectors)
{
    if (torsionMetric.size() == 0)
        return;
    for (int first = 0; first < eigenvalues.size();)
    {
        int last = first + 1;
        if (std::isfinite(eigenvalues[first]) && eigenvalues[first] > 1.0e-10)
        {
            while (last < eigenvalues.size() && std::isfinite(eigenvalues[last]) &&
                   std::abs(eigenvalues[last] - eigenvalues[first]) <=
                       1.0e-8 * std::max(std::abs(eigenvalues[first]), std::abs(eigenvalues[last])))
                ++last;
        }
        const int count = last - first;
        if (count == 1)
        {
            first = last;
            continue;
        }
        // 仅在真正重根子空间内按物理扭转指标选基，避免任意混合基决定平动/扭转族。
        const Eigen::MatrixXd fullShapes = modalTransformation * eigenvectors.middleCols(first, count);
        const Eigen::MatrixXd rawMetric = fullShapes.transpose() * (torsionMetric * fullShapes);
        const Eigen::MatrixXd modalMetric = 0.5 * (rawMetric + rawMetric.transpose());
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> rotationSolver(modalMetric);
        if (rotationSolver.info() != Eigen::Success || !rotationSolver.eigenvectors().allFinite())
        {
            report.degenerateModeRotationRejectedCount += count;
            first = last;
            continue;
        }
        Eigen::MatrixXd rotatedShapes = eigenvectors.middleCols(first, count) * rotationSolver.eigenvectors();
        Eigen::VectorXd rotatedEigenvalues(count);
        bool validRotation = true;
        for (int index = 0; index < count; ++index)
        {
            const double modalMass = rotatedShapes.col(index).dot(reducedMass * rotatedShapes.col(index));
            if (!std::isfinite(modalMass) || modalMass <= 0.0)
            {
                validRotation = false;
                break;
            }
            rotatedShapes.col(index) /= std::sqrt(modalMass);
            const Eigen::VectorXd stiffnessProduct = reducedStiffness * rotatedShapes.col(index);
            rotatedEigenvalues[index] = rotatedShapes.col(index).dot(stiffnessProduct);
            const double residual =
                (stiffnessProduct - rotatedEigenvalues[index] * (reducedMass * rotatedShapes.col(index))).norm() /
                std::max(1.0, stiffnessProduct.norm());
            if (!std::isfinite(rotatedEigenvalues[index]) || !std::isfinite(residual) || residual > 1.0e-6)
            {
                validRotation = false;
                break;
            }
        }
        if (validRotation)
        {
            eigenvectors.middleCols(first, count) = rotatedShapes;
            eigenvalues.segment(first, count) = rotatedEigenvalues;
            report.degenerateModeRotationCount += count;
        }
        else
            report.degenerateModeRotationRejectedCount += count;
        first = last;
    }
}

bool StructuralDampingModel::CalibrateLocalRayleigh(
    const Eigen::SparseMatrix<double>& reducedStiffness, const Eigen::SparseMatrix<double>& reducedMass,
    const Eigen::SparseMatrix<double>& mass, const std::array<Eigen::SparseMatrix<double>, 4>& localDampingBasis,
    const std::vector<bool>& torsionDofs, const Eigen::SparseMatrix<double>& modalTransformation,
    _OUT QString& errorMessage, const std::function<double(const Eigen::VectorXd&)>& torsionClassifier,
    const Eigen::SparseMatrix<double>& torsionMetric, const std::vector<LocalRayleighGroupBasis>& groupedBasis)
{
    const int matrixSize = reducedStiffness.rows();
    report.originalDofCount = mass.rows();
    report.modalDofCount = matrixSize;
    report.requestedModeCount = matrixSize > 1 ? matrixSize - 1 : 1;
    const int requestedModes = report.requestedModeCount;
    Eigen::VectorXd eigenvalues;
    Eigen::MatrixXd eigenvectors;
    if (matrixSize <= 512)
    {
        const Eigen::MatrixXd denseStiffness(reducedStiffness);
        const Eigen::MatrixXd denseMass(reducedMass);
        Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> solver(denseStiffness, denseMass);
        if (solver.info() == Eigen::Success)
        {
            report.convergedModeCount = requestedModes;
            eigenvalues = solver.eigenvalues();
            eigenvectors = solver.eigenvectors();
        }
    }
    else
    {
        try
        {
            using ShiftOperation = Spectra::SymShiftInvert<double, Eigen::Sparse, Eigen::Sparse>;
            using MassOperation = Spectra::SparseSymMatProd<double>;
            ShiftOperation shiftOperation(reducedStiffness, reducedMass);
            MassOperation massOperation(reducedMass);
            Spectra::SymGEigsShiftSolver<ShiftOperation, MassOperation, Spectra::GEigsMode::ShiftInvert> solver(
                shiftOperation, massOperation, requestedModes, matrixSize, 0.0);
            solver.init();
            report.convergedModeCount =
                solver.compute(Spectra::SortRule::LargestMagn, 1000, 1.0e-10, Spectra::SortRule::SmallestAlge);
            if (solver.info() == Spectra::CompInfo::Successful && report.convergedModeCount == requestedModes)
            {
                eigenvalues = solver.eigenvalues();
                eigenvectors = solver.eigenvectors();
            }
        }
        catch (const std::exception& exception)
        {
            errorMessage =
                QStringLiteral("局部 Rayleigh 一次 N−1 阶模态求解失败：%1").arg(QString::fromUtf8(exception.what()));
            return false;
        }
    }
    if (report.convergedModeCount != requestedModes || eigenvalues.size() < requestedModes ||
        eigenvectors.cols() < requestedModes)
    {
        errorMessage = QStringLiteral("局部 Rayleigh 全模态求解未完成：独立自由度 %1，请求 N−1=%2 阶，收敛 %3 阶。")
                           .arg(matrixSize)
                           .arg(requestedModes)
                           .arg(report.convergedModeCount);
        return false;
    }

    for (int index = 0; index < eigenvectors.cols(); ++index)
    {
        const double modalMass = eigenvectors.col(index).dot(reducedMass * eigenvectors.col(index));
        if (!std::isfinite(modalMass) || modalMass <= 0.0)
        {
            errorMessage = QStringLiteral("结构阻尼第 %1 阶模态质量无效。").arg(index + 1);
            return false;
        }
        eigenvectors.col(index) /= std::sqrt(modalMass);
    }
    if (eigenvectors.cols() < matrixSize)
    {
        // 对称广义特征问题剩余一维质量正交补空间就是末阶振型，随后仍检查特征方程残差。
        const Eigen::MatrixXd massShapes = reducedMass * eigenvectors;
        const Eigen::VectorXd complementaryDiagonal =
            Eigen::VectorXd::Ones(matrixSize) - (eigenvectors.array() * massShapes.array()).rowwise().sum().matrix();
        Eigen::Index seedIndex = 0;
        complementaryDiagonal.maxCoeff(&seedIndex);
        Eigen::VectorXd finalShape = Eigen::VectorXd::Unit(matrixSize, seedIndex);
        for (int pass = 0; pass < 3; ++pass)
        {
            const Eigen::VectorXd weights = eigenvectors.transpose() * (reducedMass * finalShape);
            finalShape.noalias() -= eigenvectors * weights;
        }
        const double finalMass = finalShape.dot(reducedMass * finalShape);
        if (!std::isfinite(finalMass) || finalMass <= std::numeric_limits<double>::min())
        {
            errorMessage = QStringLiteral("结构阻尼末阶质量正交补全失败，不能建立完整模态分组。");
            return false;
        }
        finalShape /= std::sqrt(finalMass);
        const double finalEigenvalue = finalShape.dot(reducedStiffness * finalShape);
        const Eigen::VectorXd stiffnessProduct = reducedStiffness * finalShape;
        const double residual = (stiffnessProduct - finalEigenvalue * (reducedMass * finalShape)).norm() /
                                std::max(1.0, stiffnessProduct.norm());
        if (!std::isfinite(residual) || residual > 1.0e-6)
        {
            errorMessage = QStringLiteral("结构阻尼末阶质量正交补全残差 %1 超过 1e−6。").arg(residual, 0, 'g', 17);
            return false;
        }
        eigenvectors.conservativeResize(matrixSize, matrixSize);
        eigenvectors.col(matrixSize - 1) = finalShape;
        eigenvalues.conservativeResize(matrixSize);
        eigenvalues[matrixSize - 1] = finalEigenvalue;
    }
    RotateDegenerateModeGroups(reducedStiffness, reducedMass, modalTransformation, torsionMetric, eigenvalues,
                               eigenvectors);

    const Eigen::VectorXd massDiagonal = mass.diagonal();
    std::vector<int> translationIndices;
    std::vector<int> torsionIndices;
    report.modeSpectrum.resize(requestedModes);
    report.supplementalModeCount = matrixSize - requestedModes;
    report.supplementalModeSpectrum.resize(report.supplementalModeCount);
    rayleighModeShapes_ = Eigen::MatrixXd::Zero(mass.rows(), matrixSize);
    for (int index = 0; index < matrixSize; ++index)
    {
        StructuralDampingModeReport& mode = ModeReport(index);
        mode.modeNumber = index + 1;
        const double eigenvalue = eigenvalues[index];
        if (!std::isfinite(eigenvalue) || eigenvalue <= 1.0e-10)
        {
            mode.residual = std::numeric_limits<double>::infinity();
            mode.rejectionReason = QStringLiteral("特征值不是有限正数或低于有效阈值");
            continue;
        }
        mode.frequencyHz = std::sqrt(eigenvalue) / kTwoPi;
        const Eigen::VectorXd reducedShape = eigenvectors.col(index);
        const Eigen::VectorXd stiffnessProduct = reducedStiffness * reducedShape;
        mode.residual = (stiffnessProduct - eigenvalue * (reducedMass * reducedShape)).norm() /
                        std::max(1.0, stiffnessProduct.norm());
        if (!std::isfinite(mode.residual) || mode.residual > 1.0e-6)
        {
            mode.rejectionReason = QStringLiteral("特征方程相对残差超过 1e−6");
            continue;
        }
        const Eigen::VectorXd fullShape = modalTransformation * reducedShape;
        double totalWeight = 0.0;
        double torsionWeight = 0.0;
        for (int dof = 0; dof < fullShape.size(); ++dof)
        {
            const double weight = std::max(0.0, massDiagonal[dof]) * fullShape[dof] * fullShape[dof];
            totalWeight += weight;
            if (torsionDofs[static_cast<std::size_t>(dof)])
                torsionWeight += weight;
        }
        if (!std::isfinite(totalWeight) || totalWeight <= std::numeric_limits<double>::epsilon())
        {
            mode.rejectionReason = QStringLiteral("振型的质量对角权重无效");
            continue;
        }
        mode.torsionFraction = torsionClassifier ? torsionClassifier(fullShape) : torsionWeight / totalWeight;
        if (!std::isfinite(mode.torsionFraction) || mode.torsionFraction < 0.0 || mode.torsionFraction > 1.0)
        {
            errorMessage = QStringLiteral("结构阻尼第 %1 阶物理模态分类指标不在 [0,1] 内。").arg(mode.modeNumber);
            rayleighModeShapes_.resize(0, 0);
            return false;
        }
        mode.torsion = mode.torsionFraction >= 0.5;
        mode.valid = true;
        mode.targetDampingRatio = mode.torsion ? settings.torsionDampingRatio : settings.translationDampingRatio;
        rayleighModeShapes_.col(index) = fullShape;
        (mode.torsion ? torsionIndices : translationIndices).push_back(index);
        if (index < requestedModes)
            ++report.validModeCount;
    }
    const auto sortByFrequency = [this](int left, int right)
    {
        return ModeReport(left).frequencyHz < ModeReport(right).frequencyHz;
    };
    std::sort(translationIndices.begin(), translationIndices.end(), sortByFrequency);
    std::sort(torsionIndices.begin(), torsionIndices.end(), sortByFrequency);
    if (translationIndices.empty() && torsionIndices.empty())
    {
        errorMessage = QStringLiteral("结构阻尼完整谱中没有可用于标定的有效正频率模态。");
        rayleighModeShapes_.resize(0, 0);
        return false;
    }

    std::vector<int> translationReferences;
    std::vector<int> torsionReferences;
    if (!SelectReferenceModes(settings.translationReferenceModes, translationIndices, QStringLiteral("平动"),
                              translationReferences, errorMessage) ||
        !SelectReferenceModes(settings.torsionReferenceModes, torsionIndices, QStringLiteral("扭转"), torsionReferences,
                              errorMessage))
    {
        rayleighModeShapes_.resize(0, 0);
        return false;
    }
    std::vector<int> referenceIndices = translationReferences;
    referenceIndices.insert(referenceIndices.end(), torsionReferences.begin(), torsionReferences.end());
    for (int index : referenceIndices)
        ModeReport(index).isReferenceMode = true;
    report.selectedModeCount = static_cast<int>(referenceIndices.size());
    report.torsionReferenceModeCount = static_cast<int>(torsionReferences.size());
    report.translationGroupModeCount = static_cast<int>(translationIndices.size());
    report.torsionGroupModeCount = static_cast<int>(torsionIndices.size());
    if (!translationReferences.empty())
        report.translationReferenceFrequency1Hz = ModeReport(translationReferences[0]).frequencyHz;
    if (translationReferences.size() > 1)
        report.translationReferenceFrequency2Hz = ModeReport(translationReferences[1]).frequencyHz;
    if (!torsionReferences.empty())
        report.torsionReferenceFrequency1Hz = ModeReport(torsionReferences[0]).frequencyHz;
    if (torsionReferences.size() > 1)
        report.torsionReferenceFrequency2Hz = ModeReport(torsionReferences[1]).frequencyHz;
    if (!FitLocalCoefficients(mass, localDampingBasis, referenceIndices, errorMessage))
    {
        if (groupedBasis.empty() || !FitGroupedLocalCoefficients(mass, groupedBasis, referenceIndices, errorMessage))
        {
            rayleighModeShapes_.resize(0, 0);
            return false;
        }
    }
    errorMessage.clear();
    report.maximumBackCheckError = report.maximumCoefficientBackCheckError;
    report.summary = QStringLiteral("局部 Rayleigh 标定完成：请求/收敛/有效 %1/%2/%3 阶，参考阻尼比拟合误差 %4。")
                         .arg(report.requestedModeCount)
                         .arg(report.convergedModeCount)
                         .arg(report.validModeCount)
                         .arg(report.maximumCoefficientBackCheckError, 0, 'g', 17);
    if (!report.localGroups.empty())
        report.summary += QStringLiteral(" %1 个单元组采用独立固定系数，系数按全谱响应均方根加权最小范数确定。")
                              .arg(static_cast<int>(report.localGroups.size()));
    calibrationReady_ = true;
    return true;
}

bool StructuralDampingModel::FitLocalCoefficients(const Eigen::SparseMatrix<double>& mass,
                                                  const std::array<Eigen::SparseMatrix<double>, 4>& localDampingBasis,
                                                  const std::vector<int>& referenceIndices, _OUT QString& errorMessage)
{
    const int referenceCount = static_cast<int>(referenceIndices.size());
    Eigen::MatrixXd response(referenceCount, 4);
    Eigen::VectorXd target(referenceCount);
    for (int row = 0; row < referenceCount; ++row)
    {
        const int index = referenceIndices[row];
        const auto& mode = ModeReport(index);
        const Eigen::VectorXd shape = rayleighModeShapes_.col(index);
        const double modalMass = shape.dot(mass * shape);
        const double denominator = 2.0 * kTwoPi * mode.frequencyHz * modalMass;
        if (!std::isfinite(denominator) || denominator <= 0.0)
        {
            errorMessage = QStringLiteral("局部 Rayleigh 第 %1 阶参考模态质量或频率无效。").arg(mode.modeNumber);
            return false;
        }
        target[row] = mode.targetDampingRatio;
        for (int column = 0; column < 4; ++column)
            response(row, column) = shape.dot(localDampingBasis[column] * shape) / denominator;
    }
    if (!response.allFinite())
    {
        errorMessage = QStringLiteral("局部 Rayleigh 参考阻尼比方程含非有限值。");
        return false;
    }

    // A_ij = phi_i^T C_j phi_i / (2 omega_i phi_i^T M phi_i)，四列对应四个单位系数。
    Eigen::MatrixXd scaledResponse = response;
    Eigen::VectorXd scaledTarget = target;
    for (int row = 0; row < referenceCount; ++row)
    {
        const double scale = std::max(target[row], 1.0e-6);
        scaledResponse.row(row) /= scale;
        scaledTarget[row] /= scale;
    }
    Eigen::Vector4d columnScale;
    for (int column = 0; column < 4; ++column)
    {
        columnScale[column] = scaledResponse.col(column).norm();
        if (columnScale[column] > 0.0)
            scaledResponse.col(column) /= columnScale[column];
    }

    Eigen::Vector4d coefficients = Eigen::Vector4d::Zero();
    double bestError = scaledTarget.squaredNorm();
    // 非负解位于某个变量面上；四变量共有十六个活跃子集。
    for (unsigned mask = 1; mask < 16; ++mask)
    {
        std::vector<int> active;
        for (int column = 0; column < 4; ++column)
        {
            if ((mask & (1U << column)) != 0 && columnScale[column] > 0.0)
                active.push_back(column);
        }
        if (active.empty())
            continue;
        Eigen::MatrixXd activeResponse(referenceCount, active.size());
        for (int column = 0; column < active.size(); ++column)
            activeResponse.col(column) = scaledResponse.col(active[column]);
        Eigen::JacobiSVD<Eigen::MatrixXd> solver(activeResponse, Eigen::ComputeThinU | Eigen::ComputeThinV);
        solver.setThreshold(1.0e-12);
        Eigen::VectorXd activeCoefficients = solver.solve(scaledTarget);
        if (!activeCoefficients.allFinite())
            continue;
        const double nonnegativeTolerance = 1.0e-12 * std::max(1.0, activeCoefficients.cwiseAbs().maxCoeff());
        if (activeCoefficients.minCoeff() < -nonnegativeTolerance)
            continue;
        activeCoefficients = activeCoefficients.cwiseMax(0.0);
        const double error = (activeResponse * activeCoefficients - scaledTarget).squaredNorm();
        if (!std::isfinite(error) || error >= bestError || bestError <= 1.0e-24)
            continue;
        coefficients.setZero();
        for (int column = 0; column < active.size(); ++column)
            coefficients[active[column]] = activeCoefficients[column] / columnScale[active[column]];
        bestError = error;
    }
    if (!coefficients.allFinite())
    {
        errorMessage = QStringLiteral("局部 Rayleigh 标定产生非有限系数。");
        return false;
    }
    report.translationMassCoefficient = coefficients[0];
    report.translationStiffnessCoefficient = coefficients[1];
    report.torsionMassCoefficient = coefficients[2];
    report.torsionStiffnessCoefficient = coefficients[3];
    const Eigen::VectorXd actual = response * coefficients;
    Eigen::Index worstReference = 0;
    report.maximumCoefficientBackCheckError = (actual - target).cwiseAbs().maxCoeff(&worstReference);
    for (int row = 0; row < referenceCount; ++row)
        ModeReport(referenceIndices[row]).projectedDampingRatio = actual[row];
    if (report.maximumCoefficientBackCheckError > report.referenceProjectionTolerance)
    {
        const auto& mode = ModeReport(referenceIndices[worstReference]);
        errorMessage = QStringLiteral("局部 Rayleigh 的非负四系数不能满足参考阻尼比：第 %1 阶目标 %2，"
                                      "拟合 %3，最大偏差 %4，容差 %5。")
                           .arg(mode.modeNumber)
                           .arg(mode.targetDampingRatio, 0, 'g', 17)
                           .arg(actual[worstReference], 0, 'g', 17)
                           .arg(report.maximumCoefficientBackCheckError, 0, 'g', 17)
                           .arg(report.referenceProjectionTolerance, 0, 'g', 17);
        report.maximumBackCheckError = report.maximumCoefficientBackCheckError;
        return false;
    }
    return true;
}

bool StructuralDampingModel::FindNonnegativeFeasiblePoint(const Eigen::MatrixXd& response,
                                                          const Eigen::VectorXd& target,
                                                          _OUT Eigen::VectorXd& coefficients)
{
    const int variableCount = response.cols();
    Eigen::MatrixXd normalized = response;
    Eigen::VectorXd columnScale(variableCount);
    for (int column = 0; column < variableCount; ++column)
    {
        columnScale[column] = normalized.col(column).stableNorm();
        if (columnScale[column] > 0.0)
            normalized.col(column) /= columnScale[column];
    }
    Eigen::VectorXd solution = Eigen::VectorXd::Zero(variableCount);
    std::vector<bool> passive(static_cast<std::size_t>(variableCount), false);
    const double tolerance = 1.0e-12 * std::max(1.0, target.norm());
    const int iterationLimit = std::max(1000, 20 * variableCount * variableCount);
    int iteration = 0;
    while (++iteration <= iterationLimit)
    {
        const Eigen::VectorXd gradient = normalized.transpose() * (target - normalized * solution);
        int entering = -1;
        double largestGradient = tolerance;
        for (int column = 0; column < variableCount; ++column)
        {
            if (!passive[column] && columnScale[column] > 0.0 && gradient[column] > largestGradient)
            {
                entering = column;
                largestGradient = gradient[column];
            }
        }
        if (entering < 0)
            break;
        passive[entering] = true;
        while (++iteration <= iterationLimit)
        {
            std::vector<int> indices;
            for (int column = 0; column < variableCount; ++column)
                if (passive[column])
                    indices.push_back(column);
            if (indices.empty())
                break;
            Eigen::MatrixXd restricted(response.rows(), indices.size());
            for (int column = 0; column < indices.size(); ++column)
                restricted.col(column) = normalized.col(indices[column]);
            Eigen::JacobiSVD<Eigen::MatrixXd> solver(restricted, Eigen::ComputeThinU | Eigen::ComputeThinV);
            solver.setThreshold(1.0e-12);
            const Eigen::VectorXd restrictedSolution = solver.solve(target);
            if (!restrictedSolution.allFinite())
                return false;
            Eigen::VectorXd candidate = Eigen::VectorXd::Zero(variableCount);
            for (int column = 0; column < indices.size(); ++column)
                candidate[indices[column]] = restrictedSolution[column];
            if (restrictedSolution.minCoeff() > 0.0)
            {
                solution = candidate;
                break;
            }
            double fraction = 1.0;
            for (int column : indices)
                if (candidate[column] <= 0.0 && solution[column] > candidate[column])
                    fraction = std::min(fraction, solution[column] / (solution[column] - candidate[column]));
            solution += fraction * (candidate - solution);
            for (int column : indices)
            {
                if (solution[column] > tolerance)
                    continue;
                solution[column] = 0.0;
                passive[column] = false;
            }
        }
    }
    coefficients = Eigen::VectorXd::Zero(variableCount);
    for (int column = 0; column < variableCount; ++column)
        if (columnScale[column] > 0.0)
            coefficients[column] = solution[column] / columnScale[column];
    return iteration <= iterationLimit && coefficients.allFinite() &&
           (response * coefficients - target).norm() <= 1.0e-9 * std::max(1.0, target.norm());
}

bool StructuralDampingModel::MinimizeNonnegativeNorm(const Eigen::MatrixXd& response, const Eigen::VectorXd& target,
                                                     _OUT Eigen::VectorXd& coefficients)
{
    const int variableCount = response.cols();
    std::vector<bool> fixed(static_cast<std::size_t>(variableCount), false);
    const int iterationLimit = std::max(1000, 20 * variableCount * variableCount);
    for (int iteration = 0; iteration < iterationLimit; ++iteration)
    {
        std::vector<int> free;
        for (int column = 0; column < variableCount; ++column)
            if (!fixed[column])
                free.push_back(column);
        if (free.empty())
            return target.norm() <= 1.0e-12;
        Eigen::MatrixXd restricted(response.rows(), free.size());
        for (int column = 0; column < free.size(); ++column)
            restricted.col(column) = response.col(free[column]);
        Eigen::JacobiSVD<Eigen::MatrixXd> solver(restricted, Eigen::ComputeThinU | Eigen::ComputeThinV);
        solver.setThreshold(1.0e-12);
        const Eigen::VectorXd restrictedMinimum = solver.solve(target);
        if (!restrictedMinimum.allFinite() ||
            (restricted * restrictedMinimum - target).norm() > 1.0e-9 * std::max(1.0, target.norm()))
            return false;
        Eigen::VectorXd candidate = Eigen::VectorXd::Zero(variableCount);
        for (int column = 0; column < free.size(); ++column)
            candidate[free[column]] = restrictedMinimum[column];

        // 从可行点沿当前自由面的最小范数方向移动，触及下界时加入活动约束。
        const double tolerance = 1.0e-12 * std::max(1.0, coefficients.norm());
        double fraction = 1.0;
        int blocking = -1;
        for (int column : free)
        {
            const double contribution = std::abs(candidate[column]) * response.col(column).stableNorm();
            if (candidate[column] >= -tolerance || contribution <= 1.0e-12 * std::max(1.0, target.norm()))
                continue;
            const double step = coefficients[column] / (coefficients[column] - candidate[column]);
            if (step < fraction)
            {
                fraction = std::max(0.0, step);
                blocking = column;
            }
        }
        if (blocking >= 0)
        {
            coefficients += fraction * (candidate - coefficients);
            coefficients[blocking] = 0.0;
            fixed[blocking] = true;
            continue;
        }
        coefficients = candidate.cwiseMax(0.0);

        // 下界乘子为负时释放该约束；只增不减活动集不能保证凸二次问题的最优性。
        const Eigen::MatrixXd transposed = restricted.transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> multiplierSolver(transposed, Eigen::ComputeThinU | Eigen::ComputeThinV);
        multiplierSolver.setThreshold(1.0e-12);
        const Eigen::VectorXd multipliers = multiplierSolver.solve(-restrictedMinimum);
        if (!multipliers.allFinite())
            return false;
        int leaving = -1;
        double mostNegative = -1.0e-11 * std::max(1.0, coefficients.norm());
        for (int column = 0; column < variableCount; ++column)
        {
            if (!fixed[column])
                continue;
            const double multiplier = response.col(column).dot(multipliers);
            if (multiplier < mostNegative)
            {
                mostNegative = multiplier;
                leaving = column;
            }
        }
        if (leaving < 0)
            return coefficients.allFinite() &&
                   (response * coefficients - target).norm() <= 1.0e-9 * std::max(1.0, target.norm());
        if (solver.rank() < response.rows())
        {
            // 退化自由面上的等式乘子不唯一，需检查是否存在满足所有零变量界条件的KKT乘子。
            std::vector<int> zeroIndices;
            for (int column = 0; column < variableCount; ++column)
                if (coefficients[column] <= tolerance)
                    zeroIndices.push_back(column);
            const int equalityCount = response.rows();
            Eigen::MatrixXd certificateMatrix =
                Eigen::MatrixXd::Zero(variableCount, 2 * equalityCount + zeroIndices.size());
            certificateMatrix.leftCols(equalityCount) = response.transpose();
            certificateMatrix.middleCols(equalityCount, equalityCount) = -response.transpose();
            for (int index = 0; index < zeroIndices.size(); ++index)
                certificateMatrix(zeroIndices[index], 2 * equalityCount + index) = -1.0;
            Eigen::VectorXd certificate;
            if (FindNonnegativeFeasiblePoint(certificateMatrix, -coefficients, certificate))
            {
                double complementarity = 0.0;
                for (int index = 0; index < zeroIndices.size(); ++index)
                    complementarity += coefficients[zeroIndices[index]] * certificate[2 * equalityCount + index];
                const double stationarity = (certificateMatrix * certificate + coefficients).norm();
                if (stationarity <= 1.0e-10 * std::max(1.0, coefficients.norm()) &&
                    complementarity <= 1.0e-10 * std::max(1.0, coefficients.squaredNorm()) &&
                    (response * coefficients - target).norm() <= 1.0e-9 * std::max(1.0, target.norm()))
                    return true;
            }
        }
        fixed[leaving] = false;
    }
    return false;
}

bool StructuralDampingModel::FitGroupedLocalCoefficients(const Eigen::SparseMatrix<double>& mass,
                                                         const std::vector<LocalRayleighGroupBasis>& groupedBasis,
                                                         const std::vector<int>& referenceIndices,
                                                         _OUT QString& errorMessage)
{
    const int variableCount = 4 * static_cast<int>(groupedBasis.size());
    const int modeCount = rayleighModeShapes_.cols();
    Eigen::MatrixXd allResponse = Eigen::MatrixXd::Zero(modeCount, variableCount);
    Eigen::VectorXd denominators = Eigen::VectorXd::Ones(modeCount);
    int validCount = 0;
    for (int index = 0; index < modeCount; ++index)
    {
        const auto& mode = ModeReport(index);
        if (!mode.valid)
            continue;
        const Eigen::VectorXd shape = rayleighModeShapes_.col(index);
        denominators[index] = 2.0 * kTwoPi * mode.frequencyHz * shape.dot(mass * shape);
        if (!std::isfinite(denominators[index]) || denominators[index] <= 0.0)
        {
            errorMessage = QStringLiteral("局部 Rayleigh 分组标定第 %1 阶模态质量或频率无效。").arg(mode.modeNumber);
            return false;
        }
        ++validCount;
    }
    for (int group = 0; group < groupedBasis.size(); ++group)
    {
        for (int component = 0; component < 4; ++component)
        {
            const Eigen::MatrixXd products = groupedBasis[group].basis[component] * rayleighModeShapes_;
            for (int index = 0; index < modeCount; ++index)
                if (ModeReport(index).valid)
                    allResponse(index, 4 * group + component) =
                        rayleighModeShapes_.col(index).dot(products.col(index)) / denominators[index];
        }
    }
    if (!allResponse.allFinite() || validCount < 1)
    {
        errorMessage = QStringLiteral("局部 Rayleigh 分组全谱响应含非有限值或没有有效模态。");
        return false;
    }
    Eigen::VectorXd weights(variableCount);
    for (int column = 0; column < variableCount; ++column)
    {
        weights[column] = allResponse.col(column).stableNorm() / std::sqrt(static_cast<double>(validCount));
        if (weights[column] == 0.0)
            weights[column] = 1.0;
    }
    if (!weights.allFinite())
    {
        errorMessage = QStringLiteral("局部 Rayleigh 分组全谱响应权重无效。");
        return false;
    }
    const int referenceCount = static_cast<int>(referenceIndices.size());
    Eigen::MatrixXd response(referenceCount, variableCount);
    Eigen::VectorXd target(referenceCount);
    for (int row = 0; row < referenceCount; ++row)
    {
        response.row(row) = allResponse.row(referenceIndices[row]);
        target[row] = ModeReport(referenceIndices[row]).targetDampingRatio;
    }
    // y_j=w_j*c_j，最小化全部有效模态响应均方根加权的系数范数，参考点仍为硬等式。
    Eigen::MatrixXd scaledResponse = response * weights.cwiseInverse().asDiagonal();
    Eigen::VectorXd scaledTarget = target;
    for (int row = 0; row < referenceCount; ++row)
    {
        const double scale = std::max(target[row], 1.0e-6);
        scaledResponse.row(row) /= scale;
        scaledTarget[row] /= scale;
    }
    Eigen::VectorXd weightedCoefficients;
    if (!FindNonnegativeFeasiblePoint(scaledResponse, scaledTarget, weightedCoefficients))
    {
        errorMessage = QStringLiteral("局部 Rayleigh 的 %1 个单元组仍无法以非负系数满足原参考模态的阻尼比。")
                           .arg(static_cast<int>(groupedBasis.size()));
        return false;
    }
    if (!MinimizeNonnegativeNorm(scaledResponse, scaledTarget, weightedCoefficients))
    {
        errorMessage = QStringLiteral("局部 Rayleigh 分组等式约束最小范数标定未收敛。");
        return false;
    }
    const Eigen::VectorXd coefficients = weightedCoefficients.cwiseQuotient(weights);
    const Eigen::VectorXd actual = response * coefficients;
    report.maximumCoefficientBackCheckError = (actual - target).cwiseAbs().maxCoeff();
    if (!coefficients.allFinite() || coefficients.minCoeff() < 0.0 ||
        report.maximumCoefficientBackCheckError > report.referenceProjectionTolerance)
    {
        errorMessage = QStringLiteral("局部 Rayleigh 分组系数回代偏差 %1 超过容差 %2，或系数无效。")
                           .arg(report.maximumCoefficientBackCheckError, 0, 'g', 17)
                           .arg(report.referenceProjectionTolerance, 0, 'g', 17);
        return false;
    }
    report.translationMassCoefficient = 0.0;
    report.translationStiffnessCoefficient = 0.0;
    report.torsionMassCoefficient = 0.0;
    report.torsionStiffnessCoefficient = 0.0;
    report.localGroups.resize(groupedBasis.size());
    for (int group = 0; group < groupedBasis.size(); ++group)
    {
        report.localGroups[group].label = groupedBasis[group].label;
        for (int component = 0; component < 4; ++component)
            report.localGroups[group].coefficients[component] = coefficients[4 * group + component];
    }
    for (int row = 0; row < referenceCount; ++row)
        ModeReport(referenceIndices[row]).projectedDampingRatio = actual[row];
    return true;
}

bool StructuralDampingModel::ValidateRayleighMatrix(const Eigen::SparseMatrix<double>& mass,
                                                    const Eigen::SparseMatrix<double>& damping,
                                                    const Eigen::SparseMatrix<double>& modalTransformation,
                                                    _OUT QString& errorMessage)
{
    report.matrixValidationPassed = false;
    const bool wasCalibrated = calibrationReady_;
    calibrationReady_ = false;
    computed_ = false;
    if (!wasCalibrated || settings.formulation != StructuralDampingFormulation::LocalRayleigh ||
        rayleighModeShapes_.rows() != mass.rows() || rayleighModeShapes_.cols() != report.modalDofCount ||
        mass.rows() != mass.cols() || damping.rows() != mass.rows() || damping.cols() != mass.cols() ||
        modalTransformation.rows() != mass.rows() || modalTransformation.cols() != report.modalDofCount)
    {
        errorMessage = QStringLiteral("局部 Rayleigh 矩阵回代的尺寸、振型缓存或标定状态无效。");
        rayleighModeShapes_.resize(0, 0);
        return false;
    }
    const Eigen::SparseMatrix<double> reducedDamping = modalTransformation.transpose() * damping * modalTransformation;
    const Eigen::SparseMatrix<double> reducedMass = modalTransformation.transpose() * mass * modalTransformation;
    const Eigen::MatrixXd denseDamping(reducedDamping);
    const Eigen::MatrixXd denseReducedMass(reducedMass);
    const Eigen::MatrixXd denseMass = 0.5 * (denseReducedMass + denseReducedMass.transpose());
    const Eigen::MatrixXd symmetricDamping = 0.5 * (denseDamping + denseDamping.transpose());
    if (!denseDamping.allFinite() || !denseMass.allFinite())
    {
        errorMessage = QStringLiteral("局部 Rayleigh 实际阻尼或质量矩阵含非有限值。");
        rayleighModeShapes_.resize(0, 0);
        return false;
    }
    report.dampingMatrixRelativeAsymmetry =
        (denseDamping - denseDamping.transpose()).norm() / std::max(1.0, denseDamping.norm());
    if (report.dampingMatrixRelativeAsymmetry > 1.0e-10)
    {
        errorMessage = QStringLiteral("实际结构阻尼矩阵不对称，相对误差 %1 超过 1e−10。")
                           .arg(report.dampingMatrixRelativeAsymmetry, 0, 'g', 17);
        rayleighModeShapes_.resize(0, 0);
        return false;
    }
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> dissipationSolver(symmetricDamping, denseMass,
                                                                                Eigen::EigenvaluesOnly);
    if (dissipationSolver.info() != Eigen::Success || !dissipationSolver.eigenvalues().allFinite())
    {
        errorMessage = QStringLiteral("局部 Rayleigh 实际阻尼的质量归一化耗散校验未收敛。");
        rayleighModeShapes_.resize(0, 0);
        return false;
    }
    report.minimumMassNormalizedDampingEigenvalue = dissipationSolver.eigenvalues().minCoeff();
    report.dissipationEigenvalueTolerance =
        1.0e-10 * std::max(1.0, dissipationSolver.eigenvalues().cwiseAbs().maxCoeff());
    if (report.minimumMassNormalizedDampingEigenvalue < -report.dissipationEigenvalueTolerance)
    {
        errorMessage = QStringLiteral("局部 Rayleigh 实际阻尼存在负耗散：最小质量归一化特征值 %1，容差 %2。")
                           .arg(report.minimumMassNormalizedDampingEigenvalue, 0, 'g', 17)
                           .arg(report.dissipationEigenvalueTolerance, 0, 'g', 17);
        rayleighModeShapes_.resize(0, 0);
        return false;
    }
    report.maximumReferenceProjectionError = 0.0;
    for (int index = 0; index < rayleighModeShapes_.cols(); ++index)
    {
        StructuralDampingModeReport& mode = ModeReport(index);
        if (!mode.valid)
            continue;
        const Eigen::VectorXd shape = rayleighModeShapes_.col(index);
        const double modalMass = shape.dot(mass * shape);
        const double denominator = 2.0 * kTwoPi * mode.frequencyHz * modalMass;
        const double projectedRatio = shape.dot(damping * shape) / denominator;
        if (!std::isfinite(modalMass) || modalMass <= 0.0 || !std::isfinite(projectedRatio))
        {
            errorMessage = QStringLiteral("局部 Rayleigh 第 %1 阶实际矩阵投影无效。").arg(mode.modeNumber);
            rayleighModeShapes_.resize(0, 0);
            return false;
        }
        mode.projectedDampingRatio = projectedRatio;
        if (mode.isReferenceMode)
        {
            const double error = std::abs(projectedRatio - mode.targetDampingRatio);
            report.maximumReferenceProjectionError = std::max(report.maximumReferenceProjectionError, error);
            if (error > report.referenceProjectionTolerance)
            {
                errorMessage = QStringLiteral("结构阻尼第 %1 阶实际回代为 %2，目标 %3，偏差 %4 超过容差 %5。")
                                   .arg(mode.modeNumber)
                                   .arg(projectedRatio, 0, 'g', 17)
                                   .arg(mode.targetDampingRatio, 0, 'g', 17)
                                   .arg(error, 0, 'g', 17)
                                   .arg(report.referenceProjectionTolerance, 0, 'g', 17);
                rayleighModeShapes_.resize(0, 0);
                return false;
            }
        }
    }
    rayleighModeShapes_.resize(0, 0);
    report.maximumBackCheckError =
        std::max(report.maximumCoefficientBackCheckError, report.maximumReferenceProjectionError);
    report.matrixValidationPassed = true;
    computed_ = true;
    report.summary += QStringLiteral(" 实际矩阵回代完成：标定点投影最大偏差 %1，最小耗散特征值 %2。")
                          .arg(report.maximumReferenceProjectionError, 0, 'g', 17)
                          .arg(report.minimumMassNormalizedDampingEigenvalue, 0, 'g', 17);
    return true;
}

void StructuralDampingModel::Reset()
{
    calibrationReady_ = false;
    computed_ = false;
    rayleighModeShapes_.resize(0, 0);
    report = {};
}

void StructuralDampingModel::Disable()
{
    settings.enabled = false;
    Reset();
}
}
