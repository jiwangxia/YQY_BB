#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include "Base/EmptyOUT.h"

namespace SolverNameSpace
{
/**
 * 光滑约束 c(q) = 0 的全局表示。
 * 矩阵列使用当前自由自由度编号，每个标量约束必须指定唯一的从自由度。
 */
struct NonlinearMPCData
{
    Eigen::VectorXd value;
    Eigen::MatrixXd jacobian;
    std::vector<Eigen::SparseMatrix<double>> hessians;
    // 局部 Hessian 项用于避免为二阶导数仅占小型转动块的约束分配完整全局稀疏矩阵。
    std::vector<std::vector<Eigen::Triplet<double>>> hessianEntries;
    std::vector<int> slaveDofs;

    void Clear();
    bool Empty() const
    {
        return value.size() == 0;
    }
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

    Eigen::VectorXd RecoverFullIncrement(const Eigen::VectorXd& masterIncrement) const;
};

/**
 * 一致线性化的非线性主从消元。
 * 采用与 Boungard & Wackerfuss（2024）公式（35）至（37）等价的约化 Hessian 形式。
 * rhs 遵循 K*dq = Fext-Fint，即结构残差的相反数。
 */
class NonlinearMPC
{
public:
    static bool Reduce(const Eigen::SparseMatrix<double>& tangent, const Eigen::VectorXd& rhs,
                       const NonlinearMPCData& constraints, _OUT NonlinearMPCReduction& reduction,
                       double rankTolerance = 1.0e-12);
};
}
