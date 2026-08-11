#pragma once

#include <atomic>

namespace SolverNameSpace
{
class GpuSettings final
{
public:
    static constexpr int MinimumGpuDofs = 10000;
    static void SetEnabled(bool enabled)
    {
        s_enabled.store(enabled, std::memory_order_relaxed);
    }

    static bool IsEnabled()
    {
        return s_enabled.load(std::memory_order_relaxed);
    }

    static void ResetStatistics()
    {
        s_attempts.store(0, std::memory_order_relaxed);
        s_successes.store(0, std::memory_order_relaxed);
        s_fallbacks.store(0, std::memory_order_relaxed);
        s_maximumMatrixDofs.store(0, std::memory_order_relaxed);
    }
    static void RecordAttempt() { s_attempts.fetch_add(1, std::memory_order_relaxed); }
    static void RecordSuccess() { s_successes.fetch_add(1, std::memory_order_relaxed); }
    static void RecordFallback() { s_fallbacks.fetch_add(1, std::memory_order_relaxed); }
    static int Attempts() { return s_attempts.load(std::memory_order_relaxed); }
    static int Successes() { return s_successes.load(std::memory_order_relaxed); }
    static int Fallbacks() { return s_fallbacks.load(std::memory_order_relaxed); }
    static void ObserveMatrixDofs(int dofs)
    {
        int current = s_maximumMatrixDofs.load(std::memory_order_relaxed);
        while (current < dofs && !s_maximumMatrixDofs.compare_exchange_weak(
            current, dofs, std::memory_order_relaxed)) {}
    }
    static int MaximumMatrixDofs() { return s_maximumMatrixDofs.load(std::memory_order_relaxed); }

private:
    inline static std::atomic_bool s_enabled{ false };
    inline static std::atomic_int s_attempts{ 0 };
    inline static std::atomic_int s_successes{ 0 };
    inline static std::atomic_int s_fallbacks{ 0 };
    inline static std::atomic_int s_maximumMatrixDofs{ 0 };
};
}
