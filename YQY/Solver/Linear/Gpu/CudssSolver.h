#pragma once

#include "Solver/Interface/IAnalysisModel.h"

#include <memory>

namespace SolverNameSpace
{
// cuDSS 稀疏线性方程组求解器封装。
class CudssSolver final
{
public:
    CudssSolver();  // 创建 cuDSS 上下文
    ~CudssSolver(); // 释放 cuDSS 上下文
    CudssSolver(const CudssSolver&) = delete;
    CudssSolver& operator=(const CudssSolver&) = delete;

    bool Solve(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution); // 求解稀疏线性方程组

private:
    class Impl;
    std::unique_ptr<Impl> m_impl; // 隐藏 cuDSS 资源细节
};
}
