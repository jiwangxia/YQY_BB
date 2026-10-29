#pragma once

#include "Base/EmptyOUT.h"
#include "Solver/Linear/LinearSystemSolver.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace SolverNameSpace
{
struct TssbnLinearPrecisionResult
{
    double backwardError = std::numeric_limits<double>::infinity();
    int refinementSolves = 0;
};

/**
 * @brief 普通 TSSBN 精度候选使用真实非对称系统残差作有限次线性精化。
 * @details 检查已装配有效切线 A 的逐行后向误差；原门不满足时，最多三次求解实际残差并累加修正。
 *          复用现有求解路径；当前后端会重新数值分解，不宣称复用分解。
 *          返回 true 表示计算有限且求解成功，是否满足停止门仍由 backwardError 独立判定。
 * @param linearSolver 当前候选所用线性求解器
 * @param tangent 实际非对称阶段切线 A
 * @param rhs 实际右端
 * @param tolerance 原逐行后向误差门
 * @param maximumRefinements 允许的精化次数，必须位于 0 到 3
 * @param patternVersion 当前实际矩阵图样版本
 * @param[out] solution 输入当前候选解，返回有限次精化后的解
 * @param[out] result 后向误差和实际额外线性求解次数
 * @author   yangqiuyou
 * @date     2026.10.22
 */
inline bool RefineTssbnPrecisionCorrection(LinearSystemSolver& linearSolver, const SpMat& tangent,
                                           const Vec& rhs,
                                           double tolerance, int maximumRefinements, std::uint64_t patternVersion,
                                           _OUT Vec& solution, _OUT TssbnLinearPrecisionResult& result)
{
    result = TssbnLinearPrecisionResult{};
    if (tangent.rows() != rhs.size() || tangent.cols() != rhs.size() ||
        solution.size() != rhs.size() || !rhs.allFinite() || !solution.allFinite() ||
        !std::isfinite(tolerance) || tolerance <= 0.0 || maximumRefinements < 0 ||
        maximumRefinements > 3)
        return false;
    for (int attempt = 0; attempt <= maximumRefinements; ++attempt)
    {
        Vec residual = rhs - tangent * solution;
        Vec scale = rhs.cwiseAbs() + tangent.cwiseAbs() * solution.cwiseAbs();
        if (!residual.allFinite() || !scale.allFinite())
            return false;
        result.backwardError = 0.0;
        for (Eigen::Index row = 0; row < residual.size(); ++row)
            result.backwardError =
                std::max(result.backwardError,
                         std::abs(residual[row]) / std::max(std::numeric_limits<double>::min(), scale[row]));
        if (result.backwardError <= tolerance || attempt == maximumRefinements)
            return true;
        Vec increment;
        ++result.refinementSolves;
        if (!linearSolver.Solve(tangent, residual, increment, patternVersion) ||
            increment.size() != solution.size() || !increment.allFinite())
            return false;
        solution += increment;
        if (!solution.allFinite())
            return false;
    }
    return false;
}
}
