#pragma once

#include "Solver/Interface/IAnalysisModel.h"

#include <cstdint>
#include <memory>

namespace SolverNameSpace
{
// Intel oneMKL PARDISO 稀疏直接求解器封装。
class PardisoSolver final
{
public:
    PardisoSolver();  // 创建 PARDISO 上下文
    ~PardisoSolver(); // 释放 PARDISO 上下文
    PardisoSolver(const PardisoSolver&) = delete;
    PardisoSolver& operator=(const PardisoSolver&) = delete;

    // 求解单右端项稀疏线性方程组。
    bool Solve(const SpMat& matrix, const Vec& rhs, bool symmetric, std::uint64_t pattern, _OUT Vec& solution);
    // 求解多右端项稀疏线性方程组。
    bool Solve(const SpMat& matrix, const Eigen::MatrixXd& rhs, bool symmetric, std::uint64_t pattern,
               _OUT Eigen::MatrixXd& solution);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl; // 隐藏 PARDISO 资源细节
};
}
