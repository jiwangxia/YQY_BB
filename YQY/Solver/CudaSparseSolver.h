#pragma once

#include "Solver/Interface/IAnalysisModel.h"

#include <memory>

namespace SolverNameSpace
{
// GPU BiCGSTAB 后端会向调用方报告不收敛状态，由既有 CPU 直接求解器安全回退。
class CudaSparseSolver final
{
public:
    CudaSparseSolver();
    ~CudaSparseSolver();
    CudaSparseSolver(const CudaSparseSolver&) = delete;
    CudaSparseSolver& operator=(const CudaSparseSolver&) = delete;

    bool Solve(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, double relativeTolerance = 1.0e-10,
               int maximumIterations = 0);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
}
