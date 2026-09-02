#pragma once

#include <atomic>

namespace SolverNameSpace
{
// 线性方程组求解器选择模式。
enum class LinearSolverMode
{
    Automatic = 0,
    Ldlt = 1,
    Lu = 2,
    Pardiso = 3,
    CudaIterative = 4,
    Cudss = 5
};

// 保存当前进程的线性求解器选择。
class LinearSolverSettings final
{
public:
    // 设置首选线性求解器。
    static void SetMode(LinearSolverMode mode)
    {
        s_mode.store(mode, std::memory_order_relaxed);
    }

    // 返回当前首选线性求解器。
    static LinearSolverMode Mode()
    {
        return s_mode.load(std::memory_order_relaxed);
    }

private:
    inline static std::atomic<LinearSolverMode> s_mode{LinearSolverMode::Automatic}; // 当前选择模式
};
}
