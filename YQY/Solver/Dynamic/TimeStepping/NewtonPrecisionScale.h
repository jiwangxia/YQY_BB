#pragma once

#include <Eigen/Dense>
#include <Eigen/SparseCore>

namespace SolverNameSpace
{
/**
 * @brief 当前 Newton 增量坐标扰动的整行残差尺度，仅供精度受限停止候选使用。
 *
 * coordinateScale 必须与完整修正的缩放使用同一坐标和单位。
 * 此局部一阶预算不能代替完整修正、LTE、约束及实际切线的检查。
 */
double EstimateNewtonPrecisionResidual(const Eigen::VectorXd& rhs, const Eigen::VectorXd& independentExternalForce,
                                       const Eigen::SparseMatrix<double>& tangent,
                                       const Eigen::VectorXd& coordinateScale);
}
