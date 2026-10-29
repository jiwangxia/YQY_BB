#include "NewtonPrecisionScale.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace SolverNameSpace
{
double EstimateNewtonPrecisionResidual(const Eigen::VectorXd& rhs, const Eigen::VectorXd& independentExternalForce,
                                       const Eigen::SparseMatrix<double>& tangent,
                                       const Eigen::VectorXd& coordinateScale)
{
    if (rhs.size() == 0 || independentExternalForce.size() != rhs.size() || coordinateScale.size() != rhs.size() ||
        tangent.rows() != rhs.size() || tangent.cols() != rhs.size() ||
        !rhs.allFinite() || !independentExternalForce.allFinite() || !coordinateScale.allFinite() ||
        coordinateScale.minCoeff() <= 0.0)
        return std::numeric_limits<double>::infinity();

    // |K|s 覆盖当前有效切线中相邻节点及不同坐标的耦合。
    Eigen::VectorXd rowScale = tangent.cwiseAbs() * coordinateScale;
    if (!rowScale.allFinite())
        return std::numeric_limits<double>::infinity();
    double maximum = 0.0;
    for (Eigen::Index index = 0; index < rhs.size(); ++index)
    {
        const double scale = std::max({1.0e-12, rowScale[index], std::abs(independentExternalForce[index]),
                                       std::abs(independentExternalForce[index] - rhs[index])});
        if (!std::isfinite(scale))
            return std::numeric_limits<double>::infinity();
        maximum = std::max(maximum, std::abs(rhs[index]) / scale);
    }
    return maximum;
}
}
