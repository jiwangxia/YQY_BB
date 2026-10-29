#include "NonlinearMPC.h"
#include "Solver/Linear/LinearSystemSolver.h"

//#include <algorithm>
//#include <cmath>
//#include <set>

namespace SolverNameSpace
{
namespace
{
bool SparseAllFinite(const Eigen::SparseMatrix<double>& matrix)
{
    for (int outer = 0; outer < matrix.outerSize(); ++outer)
        for (Eigen::SparseMatrix<double>::InnerIterator entry(matrix, outer); entry; ++entry)
            if (!std::isfinite(entry.value()))
                return false;
    return true;
}
}

void NonlinearMPCData::Clear()
{
    value.resize(0);
    jacobian.resize(0, 0);
    hessians.clear();
    hessianEntries.clear();
    slaveDofs.clear();
}

bool NonlinearMPCData::IsValid(int numberOfDofs) const
{
    const int nc = static_cast<int>(value.size());
    const bool hasSparseHessians = static_cast<int>(hessians.size()) == nc;
    const bool hasLocalHessianEntries = static_cast<int>(hessianEntries.size()) == nc;
    if (numberOfDofs <= 0 || nc <= 0 || nc >= numberOfDofs || jacobian.rows() != nc ||
        jacobian.cols() != numberOfDofs || (!hasSparseHessians && !hasLocalHessianEntries) ||
        static_cast<int>(slaveDofs.size()) != nc || !value.allFinite() || !jacobian.allFinite())
        return false;

    std::set<int> uniqueSlaves;
    for (int i = 0; i < nc; ++i)
    {
        if (slaveDofs[i] < 0 || slaveDofs[i] >= numberOfDofs || !uniqueSlaves.insert(slaveDofs[i]).second)
            return false;
        if (hasSparseHessians && !((hessians[i].rows() == 0 && hessians[i].cols() == 0) ||
                                   (hessians[i].rows() == numberOfDofs && hessians[i].cols() == numberOfDofs &&
                                    SparseAllFinite(hessians[i]))))
            return false;
        if (hasLocalHessianEntries)
            for (const auto& entry : hessianEntries[i])
                if (entry.row() < 0 || entry.row() >= numberOfDofs || entry.col() < 0 || entry.col() >= numberOfDofs ||
                    !std::isfinite(entry.value()))
                    return false;
    }
    return true;
}

Eigen::VectorXd NonlinearMPCReduction::RecoverFullIncrement(const Eigen::VectorXd& masterIncrement) const
{
    if (masterIncrement.size() != masterTransformation.cols())
        return Eigen::VectorXd();
    return particularCorrection + masterTransformation * masterIncrement;
}

namespace
{
Eigen::VectorXd Gather(const Eigen::VectorXd& source, const std::vector<int>& indices)
{
    Eigen::VectorXd result(indices.size());
    for (int i = 0; i < static_cast<int>(indices.size()); ++i)
        result[i] = source[indices[i]];
    return result;
}

bool IsDiagonal(const Eigen::MatrixXd& matrix, double tolerance)
{
    for (int i = 0; i < matrix.rows(); ++i)
        for (int j = 0; j < matrix.cols(); ++j)
            if (i != j && std::abs(matrix(i, j)) > tolerance)
                return false;
    return true;
}

Eigen::VectorXd ComputeConstraintCurvature(const NonlinearMPCData& constraints, const Eigen::VectorXd& velocity)
{
    const int constraintCount = static_cast<int>(constraints.value.size());
    const bool hasSparseHessians = static_cast<int>(constraints.hessians.size()) == constraintCount;
    const bool hasLocalHessianEntries = static_cast<int>(constraints.hessianEntries.size()) == constraintCount;
    Eigen::VectorXd curvature = Eigen::VectorXd::Zero(constraintCount);
    for (int row = 0; row < constraintCount; ++row)
    {
        if (hasSparseHessians && constraints.hessians[row].rows() == velocity.size())
            curvature[row] += velocity.dot(constraints.hessians[row] * velocity);
        if (hasLocalHessianEntries)
            for (const auto& entry : constraints.hessianEntries[row])
                curvature[row] += entry.value() * velocity[entry.row()] * velocity[entry.col()];
    }
    return curvature;
}
}

bool NonlinearMPC::ValidateInitialKinematics(const NonlinearMPCData& constraints,
                                              const Eigen::VectorXd& displacement,
                                              const Eigen::VectorXd& velocity,
                                              const Eigen::VectorXd& acceleration, _OUT QString& errorMessage,
                                              double tolerance)
{
    errorMessage.clear();
    if (constraints.Empty())
        return true;

    const int dofCount = static_cast<int>(displacement.size());
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || !constraints.IsValid(dofCount))
    {
        errorMessage = QStringLiteral("初始 MPC 约束数据或检查容限无效");
        return false;
    }
    if (velocity.size() != dofCount || !displacement.allFinite() || !velocity.allFinite())
    {
        errorMessage = QStringLiteral("初始 MPC 位移或速度的维数、数值无效");
        return false;
    }
    if (acceleration.size() != dofCount || !acceleration.allFinite())
    {
        errorMessage = QStringLiteral("初始 MPC 加速度缺失或数值无效；请先求解质量与约束方程，提供一致的初始加速度");
        return false;
    }

    const double positionResidual = constraints.value.norm();
    if (positionResidual > tolerance)
    {
        errorMessage = QStringLiteral("初始 MPC 位置约束 c(q) 不一致：残差=%1，容限=%2")
                           .arg(positionResidual, 0, 'g', 10)
                           .arg(tolerance, 0, 'g', 10);
        return false;
    }

    const double velocityResidual = (constraints.jacobian * velocity).norm();
    const double velocityLimit = tolerance * std::max(1.0, velocity.norm());
    if (!std::isfinite(velocityResidual) || velocityResidual > velocityLimit)
    {
        errorMessage = QStringLiteral("初始 MPC 速度约束 Gv 不一致：残差=%1，容限=%2")
                           .arg(velocityResidual, 0, 'g', 10)
                           .arg(velocityLimit, 0, 'g', 10);
        return false;
    }

    Eigen::VectorXd accelerationResidual = constraints.jacobian * acceleration;
    const Eigen::VectorXd curvature = ComputeConstraintCurvature(constraints, velocity);
    const double accelerationLimit =
        tolerance * std::max({1.0, accelerationResidual.norm(), curvature.norm()});
    accelerationResidual += curvature;
    const double accelerationResidualNorm = accelerationResidual.norm();
    if (!curvature.allFinite() || !std::isfinite(accelerationResidualNorm) ||
        accelerationResidualNorm > accelerationLimit)
    {
        errorMessage = QStringLiteral("初始 MPC 加速度约束 Ga+H[v,v] 不一致：残差=%1，容限=%2；"
                                      "请先求解质量与约束方程，提供一致的初始加速度")
                           .arg(accelerationResidualNorm, 0, 'g', 10)
                           .arg(accelerationLimit, 0, 'g', 10);
        return false;
    }
    return true;
}

bool NonlinearMPC::RecoverConsistentAcceleration(const Eigen::SparseMatrix<double>& mass,
                                                  const Eigen::VectorXd& residualAtGuess,
                                                  const Eigen::VectorXd& velocity,
                                                  const Eigen::VectorXd& accelerationGuess,
                                                  const NonlinearMPCData& constraints, LinearSystemSolver& solver,
                                                  _OUT Eigen::VectorXd& acceleration,
                                                  _OUT Eigen::VectorXd& multipliers, _OUT QString& errorMessage)
{
    acceleration.resize(0);
    multipliers.resize(0);
    errorMessage.clear();
    const auto fail = [&](const QString& reason)
    {
        acceleration.resize(0);
        multipliers.resize(0);
        errorMessage = reason;
        return false;
    };

    const int dofCount = static_cast<int>(mass.rows());
    if (dofCount <= 0 || mass.cols() != dofCount || residualAtGuess.size() != dofCount ||
        velocity.size() != dofCount || accelerationGuess.size() != dofCount || !SparseAllFinite(mass) ||
        !residualAtGuess.allFinite() || !velocity.allFinite() || !accelerationGuess.allFinite())
        return fail(QStringLiteral("质量方程输入的维数或数值无效"));

    const bool hasConstraints = !constraints.Empty();
    if (hasConstraints && !constraints.IsValid(dofCount))
        return fail(QStringLiteral("MPC 约束数据无效"));

    const int constraintCount = static_cast<int>(constraints.value.size());
    Eigen::VectorXd curvature = Eigen::VectorXd::Zero(constraintCount);
    if (hasConstraints)
    {
        curvature = ComputeConstraintCurvature(constraints, velocity);
        if (!curvature.allFinite())
            return fail(QStringLiteral("MPC 约束曲率项包含非有限数值"));
    }

    Eigen::SparseMatrix<double> solveMatrix = mass;
    Eigen::VectorXd solveRhs = residualAtGuess;
    if (hasConstraints)
    {
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(static_cast<std::size_t>(mass.nonZeros()) +
                         2 * static_cast<std::size_t>(constraints.jacobian.rows() * constraints.jacobian.cols()));
        for (int outer = 0; outer < mass.outerSize(); ++outer)
            for (Eigen::SparseMatrix<double>::InnerIterator entry(mass, outer); entry; ++entry)
                triplets.emplace_back(entry.row(), entry.col(), entry.value());
        for (int row = 0; row < constraintCount; ++row)
            for (int column = 0; column < dofCount; ++column)
            {
                const double value = constraints.jacobian(row, column);
                if (value == 0.0)
                    continue;
                triplets.emplace_back(column, dofCount + row, value);
                triplets.emplace_back(dofCount + row, column, value);
            }
        solveMatrix.resize(dofCount + constraintCount, dofCount + constraintCount);
        solveMatrix.setFromTriplets(triplets.begin(), triplets.end());
        solveRhs = Eigen::VectorXd::Zero(dofCount + constraintCount);
        solveRhs.head(dofCount) = residualAtGuess;
        solveRhs.tail(constraintCount) = -constraints.jacobian * accelerationGuess - curvature;
    }

    solver.SetPreferLdlt(!hasConstraints);
    Eigen::VectorXd solution;
    if (!solver.Solve(solveMatrix, solveRhs, solution) || solution.size() != solveRhs.size() ||
        !solution.allFinite())
        return fail(QStringLiteral("质量与 MPC 约束方程求解失败"));

    const Eigen::VectorXd accelerationCorrection = solution.head(dofCount);
    acceleration = accelerationGuess + accelerationCorrection;
    if (!acceleration.allFinite())
        return fail(QStringLiteral("恢复的加速度包含非有限数值"));

    Eigen::VectorXd forceBalance = residualAtGuess - mass * accelerationCorrection;
    const double forceScale = std::max({1.0, residualAtGuess.norm(), (mass * accelerationCorrection).norm()});
    double constraintError = 0.0;
    double constraintScale = 1.0;
    if (hasConstraints)
    {
        const Eigen::VectorXd kktMultipliers = solution.tail(constraintCount);
        const Eigen::VectorXd constraintForce = constraints.jacobian.transpose() * kktMultipliers;
        forceBalance -= constraintForce;
        multipliers = -kktMultipliers;
        constraintError = (constraints.jacobian * acceleration + curvature).norm();
        constraintScale = std::max({1.0, (constraints.jacobian * acceleration).norm(), curvature.norm()});
    }
    if (!forceBalance.allFinite() || !std::isfinite(constraintError) ||
        forceBalance.norm() > 1.0e-8 * forceScale || constraintError > 1.0e-8 * constraintScale)
        return fail(QStringLiteral("加速度恢复后的动力或 MPC 约束残差过大"));
    return true;
}

bool NonlinearMPC::Reduce(const Eigen::SparseMatrix<double>& tangent, const Eigen::VectorXd& rhs,
                          const NonlinearMPCData& constraints, NonlinearMPCReduction& reduction, double rankTolerance)
{
    const int ndof = tangent.rows();
    const int nc = static_cast<int>(constraints.value.size());
    if (tangent.cols() != ndof || rhs.size() != ndof || rankTolerance <= 0.0 || !rhs.allFinite() ||
        !SparseAllFinite(tangent) || !constraints.IsValid(ndof))
        return false;

    reduction.slaveDofs = constraints.slaveDofs;
    std::vector<bool> slaveMask(ndof, false);
    for (const int dof : reduction.slaveDofs)
        slaveMask[dof] = true;

    reduction.masterDofs.clear();
    reduction.masterDofs.reserve(ndof - nc);
    for (int dof = 0; dof < ndof; ++dof)
        if (!slaveMask[dof])
            reduction.masterDofs.push_back(dof);

    const int nm = static_cast<int>(reduction.masterDofs.size());
    Eigen::MatrixXd Gm(nc, nm);
    Eigen::MatrixXd Gs(nc, nc);
    for (int i = 0; i < nc; ++i)
    {
        for (int j = 0; j < nm; ++j)
            Gm(i, j) = constraints.jacobian(i, reduction.masterDofs[j]);
        for (int j = 0; j < nc; ++j)
            Gs(i, j) = constraints.jacobian(i, reduction.slaveDofs[j]);
    }

    Eigen::MatrixXd slaveFromMaster;
    Eigen::VectorXd slaveParticular;
    const Eigen::VectorXd structuralResidual = -rhs;
    const Eigen::VectorXd residualSlave = Gather(structuralResidual, reduction.slaveDofs);
    Eigen::VectorXd multiplier;

    const bool diagonalSlaveBlock = IsDiagonal(Gs, rankTolerance);
    const bool useDirectDiagonalProjection = diagonalSlaveBlock;
    if (useDirectDiagonalProjection)
    {
        Eigen::VectorXd diagonal = Gs.diagonal();
        if ((diagonal.array().abs() <= rankTolerance).any())
            return false;
        const Eigen::VectorXd inverseDiagonal = diagonal.cwiseInverse();
        slaveFromMaster = inverseDiagonal.asDiagonal() * Gm;
        slaveFromMaster *= -1.0;
        slaveParticular = inverseDiagonal.asDiagonal() * constraints.value;
        slaveParticular *= -1.0;
        multiplier = inverseDiagonal.asDiagonal() * residualSlave;
    }
    else
    {
        Eigen::FullPivLU<Eigen::MatrixXd> factor(Gs);
        factor.setThreshold(rankTolerance);
        if (factor.rank() != nc)
            return false;
        Eigen::FullPivLU<Eigen::MatrixXd> transposeFactor(Gs.transpose());
        transposeFactor.setThreshold(rankTolerance);
        slaveFromMaster = -factor.solve(Gm);
        slaveParticular = -factor.solve(constraints.value);
        multiplier = transposeFactor.solve(residualSlave);
    }
    if (!slaveFromMaster.allFinite() || !slaveParticular.allFinite() || !multiplier.allFinite())
        return false;
    reduction.multipliers = multiplier;

    std::vector<Eigen::Triplet<double>> transformationTriplets;
    transformationTriplets.reserve(nm + nc * 8);
    for (int i = 0; i < nm; ++i)
        transformationTriplets.emplace_back(reduction.masterDofs[i], i, 1.0);
    for (int i = 0; i < nc; ++i)
        for (int j = 0; j < nm; ++j)
            if (std::abs(slaveFromMaster(i, j)) > 1.0e-15)
                transformationTriplets.emplace_back(reduction.slaveDofs[i], j, slaveFromMaster(i, j));

    reduction.masterTransformation.resize(ndof, nm);
    reduction.masterTransformation.setFromTriplets(transformationTriplets.begin(), transformationTriplets.end());
    reduction.particularCorrection = Eigen::VectorXd::Zero(ndof);
    for (int i = 0; i < nc; ++i)
        reduction.particularCorrection[reduction.slaveDofs[i]] = slaveParticular[i];

    Eigen::SparseMatrix<double> lagrangianTangent = tangent;
    const bool hasSparseHessians = static_cast<int>(constraints.hessians.size()) == nc;
    const bool hasLocalHessianEntries = static_cast<int>(constraints.hessianEntries.size()) == nc;
    std::size_t hessianNonZeros = 0;
    for (int i = 0; i < nc; ++i)
        if (multiplier[i] != 0.0)
        {
            if (hasSparseHessians)
                hessianNonZeros += static_cast<std::size_t>(constraints.hessians[i].nonZeros());
            if (hasLocalHessianEntries)
                hessianNonZeros += constraints.hessianEntries[i].size();
        }
    if (hessianNonZeros > 0)
    {
        // MPC Hessian 仅含局部项，刚性偏置只贡献主节点转动的 3x3 块。
        // 先汇总局部块，再一次合并全局切线，避免按标量约束反复复制稀疏矩阵。
        std::vector<Eigen::Triplet<double>> hessianTriplets;
        hessianTriplets.reserve(hessianNonZeros);
        for (int i = 0; i < nc; ++i)
        {
            if (multiplier[i] == 0.0)
                continue;
            if (hasSparseHessians)
            {
                const auto& hessian = constraints.hessians[i];
                for (int outer = 0; outer < hessian.outerSize(); ++outer)
                    for (Eigen::SparseMatrix<double>::InnerIterator entry(hessian, outer); entry; ++entry)
                        hessianTriplets.emplace_back(entry.row(), entry.col(), multiplier[i] * entry.value());
            }
            if (hasLocalHessianEntries)
                for (const auto& entry : constraints.hessianEntries[i])
                    hessianTriplets.emplace_back(entry.row(), entry.col(), multiplier[i] * entry.value());
        }
        Eigen::SparseMatrix<double> lagrangianCorrection(ndof, ndof);
        lagrangianCorrection.setFromTriplets(hessianTriplets.begin(), hessianTriplets.end());
        lagrangianTangent -= lagrangianCorrection;
    }
    lagrangianTangent.makeCompressed();

    if (diagonalSlaveBlock)
    {
        // 刚性偏置和位移绑定 MPC 的从自由度 Jacobian 为对角阵，变换仅在少量从自由度行偏离单位阵。
        // 直接按稀疏项装配 T^T*L*T，避免两次全局乘法重复处理单位阵列。
        std::vector<int> masterIndex(ndof, -1);
        for (int master = 0; master < nm; ++master)
            masterIndex[reduction.masterDofs[master]] = master;
        std::vector<int> slaveRow(ndof, -1);
        std::vector<std::vector<std::pair<int, double>>> slaveTerms(nc);
        for (int slave = 0; slave < nc; ++slave)
        {
            slaveRow[reduction.slaveDofs[slave]] = slave;
            for (int master = 0; master < nm; ++master)
            {
                const double coefficient = slaveFromMaster(slave, master);
                if (std::abs(coefficient) > 1.0e-15)
                    slaveTerms[slave].emplace_back(master, coefficient);
            }
        }

        const auto accumulateTranspose = [&](const Eigen::VectorXd& full) -> Eigen::VectorXd
        {
            Eigen::VectorXd reduced = Eigen::VectorXd::Zero(nm);
            for (int dof = 0; dof < ndof; ++dof)
            {
                const int master = masterIndex[dof];
                if (master >= 0)
                    reduced[master] += full[dof];
                else
                {
                    const int slave = slaveRow[dof];
                    for (const auto& [column, coefficient] : slaveTerms[slave])
                        reduced[column] += coefficient * full[dof];
                }
            }
            return reduced;
        };

        std::vector<Eigen::Triplet<double>> tangentTriplets;
        tangentTriplets.reserve(static_cast<std::size_t>(lagrangianTangent.nonZeros()) +
                                static_cast<std::size_t>(nc) * 64);
        for (int outer = 0; outer < lagrangianTangent.outerSize(); ++outer)
        {
            for (Eigen::SparseMatrix<double>::InnerIterator entry(lagrangianTangent, outer); entry; ++entry)
            {
                const int row = entry.row();
                const int column = entry.col();
                const int rowMaster = masterIndex[row];
                const int columnMaster = masterIndex[column];
                if (rowMaster >= 0 && columnMaster >= 0)
                {
                    tangentTriplets.emplace_back(rowMaster, columnMaster, entry.value());
                }
                else if (rowMaster >= 0)
                {
                    for (const auto& [reducedColumn, coefficient] : slaveTerms[slaveRow[column]])
                        tangentTriplets.emplace_back(rowMaster, reducedColumn, entry.value() * coefficient);
                }
                else if (columnMaster >= 0)
                {
                    for (const auto& [reducedRow, coefficient] : slaveTerms[slaveRow[row]])
                        tangentTriplets.emplace_back(reducedRow, columnMaster, coefficient * entry.value());
                }
                else
                {
                    for (const auto& [reducedRow, rowCoefficient] : slaveTerms[slaveRow[row]])
                        for (const auto& [reducedColumn, columnCoefficient] : slaveTerms[slaveRow[column]])
                            tangentTriplets.emplace_back(reducedRow, reducedColumn,
                                                         rowCoefficient * entry.value() * columnCoefficient);
                }
            }
        }
        reduction.tangent.resize(nm, nm);
        reduction.tangent.setFromTriplets(tangentTriplets.begin(), tangentTriplets.end());

        const Eigen::VectorXd reducedResidual = accumulateTranspose(structuralResidual);
        const Eigen::VectorXd correction = lagrangianTangent * reduction.particularCorrection;
        reduction.rhs = -reducedResidual - accumulateTranspose(correction);
    }
    else
    {
        const Eigen::SparseMatrix<double> projected = lagrangianTangent * reduction.masterTransformation;
        reduction.tangent = reduction.masterTransformation.transpose() * projected;

        const Eigen::VectorXd reducedResidual = reduction.masterTransformation.transpose() * structuralResidual;
        reduction.rhs = -reducedResidual - reduction.masterTransformation.transpose() *
                                               (lagrangianTangent * reduction.particularCorrection);
    }
    reduction.constraintNorm = constraints.value.norm();
    return reduction.rhs.allFinite() && SparseAllFinite(reduction.tangent);
}
}
