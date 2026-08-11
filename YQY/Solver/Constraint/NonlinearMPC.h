#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include "Base/EmptyOUT.h"

namespace SolverNameSpace
{
    /**
     * Global representation of smooth constraints c(q) = 0.
     *
     * The columns use the current free-DOF numbering. One unique slave DOF
     * must be supplied for every scalar constraint.
     */
    struct NonlinearMPCData
    {
        Eigen::VectorXd value;
        Eigen::MatrixXd jacobian;
        std::vector<Eigen::SparseMatrix<double>> hessians;
        // Local Hessian entries.  This avoids allocating a full global
        // sparse matrix for constraints whose second derivative is only a
        // small rotational block (for example a rigid offset MPC).
        std::vector<std::vector<Eigen::Triplet<double>>> hessianEntries;
        std::vector<int> slaveDofs;

        void Clear();
        bool Empty() const { return value.size() == 0; }
        bool IsValid(int numberOfDofs) const;
    };

    struct NonlinearMPCReduction
    {
        Eigen::SparseMatrix<double> tangent;
        Eigen::VectorXd rhs;
        Eigen::SparseMatrix<double> masterTransformation;
        Eigen::VectorXd particularCorrection;
        Eigen::VectorXd multipliers;
        std::vector<int> masterDofs;
        std::vector<int> slaveDofs;
        double constraintNorm = 0.0;

        Eigen::VectorXd RecoverFullIncrement(
            const Eigen::VectorXd& masterIncrement) const;
    };

    /**
     * Consistently linearized nonlinear master-slave elimination.
     *
     * Implements the reduced-Hessian form equivalent to Eqs. (35)-(37) of
     * Boungard & Wackerfuss (2024). `rhs` follows this codebase's convention
     * K*dq = Fext-Fint, i.e. it is the negative structural residual.
     */
    class NonlinearMPC
    {
    public:
        static bool Reduce(
            const Eigen::SparseMatrix<double>& tangent,
            const Eigen::VectorXd& rhs,
            const NonlinearMPCData& constraints,
            _OUT NonlinearMPCReduction& reduction,
            double rankTolerance = 1.0e-12);
    };
}
