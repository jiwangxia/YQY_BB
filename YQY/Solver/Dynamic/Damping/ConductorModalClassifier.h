#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <QString>
#include "Base/EmptyOUT.h"
#include <functional>

class StructureData;

namespace SolverNameSpace
{
/** @brief 静力参考构型的导线物理振型指标，使用完整自由自由度布局。
 * @details 指标描述导线运动，不是整个结构质量矩阵的能量分块；
 *          传入振型必须已经通过 MPC 变换展开。
 * @author yangqiuyou
 * @date 2026.10.17
 */
struct ConductorModalMetrics
{
    Eigen::SparseMatrix<double> bundleTorsion;
    Eigen::SparseMatrix<double> transverseMotion;
    Eigen::SparseMatrix<double> axialTwist;
    int conductorCount = 0;
    int bundleSectionCount = 0;
    int bundleSpanCount = 0;
    int skippedBundleCount = 0;
    QString summary;

    double TorsionFraction(const Eigen::VectorXd& shape, const Eigen::SparseMatrix<double>& mass) const;
};

class ConductorModalClassifier
{
public:
    static bool Build(const StructureData& data, int fixedDofCount, int freeDofCount,
                      _OUT ConductorModalMetrics& metrics, _OUT QString& errorMessage,
                      const std::function<bool(int)>& isStepActive = {});
};
}
