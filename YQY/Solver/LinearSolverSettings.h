#pragma once

#include <atomic>

namespace SolverNameSpace
{
enum class LinearSolverMode
{
    Automatic = 0,
    Ldlt = 1,
    Lu = 2,
    Pardiso = 3,
    CudaIterative = 4,
    Cudss = 5
};

class LinearSolverSettings final
{
public:
    static void SetMode(LinearSolverMode mode)
    {
        s_mode.store(mode, std::memory_order_relaxed);
    }

    static LinearSolverMode Mode()
    {
        return s_mode.load(std::memory_order_relaxed);
    }

private:
    inline static std::atomic<LinearSolverMode> s_mode{LinearSolverMode::Automatic};
};
}
