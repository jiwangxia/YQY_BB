#include "NonlinearMPC.h"

#include <algorithm>
#include <cmath>
#include <set>

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
    const bool hasLocalHessianEntries = static_cast<int>(constraints.hessianEntries.size()) == nc;
    std::size_t hessianNonZeros = 0;
    for (int i = 0; i < nc; ++i)
        if (multiplier[i] != 0.0)
        {
            hessianNonZeros += static_cast<std::size_t>(constraints.hessians[i].nonZeros());
            if (hasLocalHessianEntries)
                hessianNonZeros += constraints.hessianEntries[i].size();
        }
    if (hessianNonZeros > 0)
    {
        // Each MPC Hessian is local (rigid offsets contribute only a
        // 3x3 master-rotation block), but repeated sparse subtraction
        // copies and merges the full global tangent once per scalar
        // constraint.  Combine all local blocks first and merge once.
        std::vector<Eigen::Triplet<double>> hessianTriplets;
        hessianTriplets.reserve(hessianNonZeros);
        for (int i = 0; i < nc; ++i)
        {
            if (multiplier[i] == 0.0)
                continue;
            const auto& hessian = constraints.hessians[i];
            for (int outer = 0; outer < hessian.outerSize(); ++outer)
                for (Eigen::SparseMatrix<double>::InnerIterator entry(hessian, outer); entry; ++entry)
                    hessianTriplets.emplace_back(entry.row(), entry.col(), multiplier[i] * entry.value());
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
        // Rigid-offset and translational-tie MPCs have a diagonal
        // slave Jacobian.  Their transformation is identity except for
        // a few slave rows.  Forming L*T and then T^T*(L*T) globally
        // spends most of the solve time multiplying identity columns.
        // Assemble T^T*L*T directly from the sparse entries instead.
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
