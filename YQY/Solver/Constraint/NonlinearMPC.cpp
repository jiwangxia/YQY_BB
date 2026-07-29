#include "NonlinearMPC.h"

#include <algorithm>
#include <set>

namespace SolverNameSpace
{
    void NonlinearMPCData::Clear()
    {
        value.resize(0);
        jacobian.resize(0, 0);
        hessians.clear();
        slaveDofs.clear();
    }

    bool NonlinearMPCData::IsValid(int numberOfDofs) const
    {
        const int nc = static_cast<int>(value.size());
        if (numberOfDofs <= 0 || nc <= 0 || nc >= numberOfDofs
            || jacobian.rows() != nc || jacobian.cols() != numberOfDofs
            || static_cast<int>(hessians.size()) != nc
            || static_cast<int>(slaveDofs.size()) != nc
            || !value.allFinite() || !jacobian.allFinite())
            return false;

        std::set<int> uniqueSlaves;
        for (int i = 0; i < nc; ++i)
        {
            if (slaveDofs[i] < 0 || slaveDofs[i] >= numberOfDofs
                || !uniqueSlaves.insert(slaveDofs[i]).second
                || hessians[i].rows() != numberOfDofs
                || hessians[i].cols() != numberOfDofs)
                return false;
        }
        return true;
    }

    Eigen::VectorXd NonlinearMPCReduction::RecoverFullIncrement(
        const Eigen::VectorXd& masterIncrement) const
    {
        if (masterIncrement.size() != masterTransformation.cols())
            return Eigen::VectorXd();
        return particularCorrection
            + masterTransformation * masterIncrement;
    }

    namespace
    {
        Eigen::VectorXd Gather(
            const Eigen::VectorXd& source,
            const std::vector<int>& indices)
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

    bool NonlinearMPC::Reduce(
        const Eigen::SparseMatrix<double>& tangent,
        const Eigen::VectorXd& rhs,
        const NonlinearMPCData& constraints,
        NonlinearMPCReduction& reduction,
        double rankTolerance)
    {
        const int ndof = tangent.rows();
        const int nc = static_cast<int>(constraints.value.size());
        if (tangent.cols() != ndof || rhs.size() != ndof
            || rankTolerance <= 0.0 || !constraints.IsValid(ndof))
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
                Gm(i, j) = constraints.jacobian(
                    i, reduction.masterDofs[j]);
            for (int j = 0; j < nc; ++j)
                Gs(i, j) = constraints.jacobian(
                    i, reduction.slaveDofs[j]);
        }

        Eigen::MatrixXd slaveFromMaster;
        Eigen::VectorXd slaveParticular;
        const Eigen::VectorXd structuralResidual = -rhs;
        const Eigen::VectorXd residualSlave =
            Gather(structuralResidual, reduction.slaveDofs);
        Eigen::VectorXd multiplier;

        if (IsDiagonal(Gs, rankTolerance))
        {
            Eigen::VectorXd diagonal = Gs.diagonal();
            if ((diagonal.array().abs() <= rankTolerance).any())
                return false;
            const Eigen::VectorXd inverseDiagonal =
                diagonal.cwiseInverse();
            slaveFromMaster = inverseDiagonal.asDiagonal() * Gm;
            slaveFromMaster *= -1.0;
            slaveParticular =
                inverseDiagonal.asDiagonal() * constraints.value;
            slaveParticular *= -1.0;
            multiplier =
                inverseDiagonal.asDiagonal() * residualSlave;
        }
        else
        {
            Eigen::FullPivLU<Eigen::MatrixXd> factor(Gs);
            factor.setThreshold(rankTolerance);
            if (factor.rank() != nc)
                return false;
            Eigen::FullPivLU<Eigen::MatrixXd> transposeFactor(
                Gs.transpose());
            transposeFactor.setThreshold(rankTolerance);
            slaveFromMaster = -factor.solve(Gm);
            slaveParticular = -factor.solve(constraints.value);
            multiplier = transposeFactor.solve(residualSlave);
        }
        if (!slaveFromMaster.allFinite() || !slaveParticular.allFinite()
            || !multiplier.allFinite())
            return false;
        reduction.multipliers = multiplier;

        std::vector<Eigen::Triplet<double>> transformationTriplets;
        transformationTriplets.reserve(nm + nc * 8);
        for (int i = 0; i < nm; ++i)
            transformationTriplets.emplace_back(
                reduction.masterDofs[i], i, 1.0);
        for (int i = 0; i < nc; ++i)
            for (int j = 0; j < nm; ++j)
                if (std::abs(slaveFromMaster(i, j)) > 1.0e-15)
                    transformationTriplets.emplace_back(
                        reduction.slaveDofs[i], j,
                        slaveFromMaster(i, j));

        reduction.masterTransformation.resize(ndof, nm);
        reduction.masterTransformation.setFromTriplets(
            transformationTriplets.begin(),
            transformationTriplets.end());
        reduction.particularCorrection =
            Eigen::VectorXd::Zero(ndof);
        for (int i = 0; i < nc; ++i)
            reduction.particularCorrection[reduction.slaveDofs[i]]
                = slaveParticular[i];

        Eigen::SparseMatrix<double> lagrangianTangent = tangent;
        for (int i = 0; i < nc; ++i)
        {
            if (multiplier[i] != 0.0)
                lagrangianTangent -=
                    multiplier[i] * constraints.hessians[i];
        }
        lagrangianTangent.makeCompressed();

        const Eigen::SparseMatrix<double> projected =
            lagrangianTangent
            * reduction.masterTransformation;
        reduction.tangent =
            reduction.masterTransformation.transpose() * projected;

        const Eigen::VectorXd reducedResidual =
            reduction.masterTransformation.transpose()
            * structuralResidual;
        reduction.rhs =
            -reducedResidual
            - reduction.masterTransformation.transpose()
                * (lagrangianTangent
                    * reduction.particularCorrection);
        reduction.constraintNorm = constraints.value.norm();
        return reduction.rhs.allFinite();
    }
}
