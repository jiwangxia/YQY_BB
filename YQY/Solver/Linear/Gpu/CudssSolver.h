#pragma once

#include "Solver/Interface/IAnalysisModel.h"

#include <memory>

namespace SolverNameSpace
{
// 单次 cuDSS 调用的设备端耗时。数值由 CUDA Event 统计，不包含 CPU 侧 CSR 转换。
struct CudssSolveTiming
{
    double uploadMilliseconds = 0.0;
    double analysisMilliseconds = 0.0;
    double factorizationMilliseconds = 0.0;
    double solveMilliseconds = 0.0;
    double downloadMilliseconds = 0.0;
    bool reusedAnalysis = false;
};

// cuDSS 稀疏线性方程组求解器封装。
class CudssSolver final
{
public:
    CudssSolver();  // 创建 cuDSS 上下文
    ~CudssSolver(); // 释放 cuDSS 上下文
    CudssSolver(const CudssSolver&) = delete;
    CudssSolver& operator=(const CudssSolver&) = delete;

    bool Solve(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution); // 求解稀疏线性方程组
    bool SolveDeviceCscValues(const SpMat& matrix, const double* deviceValues, int valueCount, const Vec& rhs,
                              _OUT Vec& solution);

    // 返回最近一次调用的 GPU 分阶段耗时，供性能统计和自动后端选择使用。
    const CudssSolveTiming& LastTiming() const;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl; // 隐藏 cuDSS 资源细节
};
}
