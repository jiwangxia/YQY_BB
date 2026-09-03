#pragma once

#include <atomic>

namespace SolverNameSpace
{
// 管理 GPU 求解开关和运行统计。
class GpuSettings final
{
public:
    static constexpr int MinimumGpuDofs = 10000; // 启用 GPU 的最小自由度数
    // 设置 GPU 求解开关。
    static void SetEnabled(bool enabled)
    {
        s_enabled.store(enabled, std::memory_order_relaxed);
    }

    // 返回 GPU 求解是否启用。
    static bool IsEnabled()
    {
        return s_enabled.load(std::memory_order_relaxed);
    }

    // 清空本次运行的 GPU 统计。
    static void ResetStatistics()
    {
        s_attempts.store(0, std::memory_order_relaxed);
        s_successes.store(0, std::memory_order_relaxed);
        s_fallbacks.store(0, std::memory_order_relaxed);
        s_maximumMatrixDofs.store(0, std::memory_order_relaxed);
    }
    // 记录一次 GPU 求解尝试。
    static void RecordAttempt()
    {
        s_attempts.fetch_add(1, std::memory_order_relaxed);
    }
    // 记录一次 GPU 求解成功。
    static void RecordSuccess()
    {
        s_successes.fetch_add(1, std::memory_order_relaxed);
    }
    // 记录一次回退到 CPU 求解。
    static void RecordFallback()
    {
        s_fallbacks.fetch_add(1, std::memory_order_relaxed);
    }
    // 返回 GPU 求解尝试次数。
    static int Attempts()
    {
        return s_attempts.load(std::memory_order_relaxed);
    }
    // 返回 GPU 求解成功次数。
    static int Successes()
    {
        return s_successes.load(std::memory_order_relaxed);
    }
    // 返回 GPU 求解回退次数。
    static int Fallbacks()
    {
        return s_fallbacks.load(std::memory_order_relaxed);
    }
    // 更新已处理矩阵的最大自由度数。
    static void ObserveMatrixDofs(int dofs)
    {
        int current = s_maximumMatrixDofs.load(std::memory_order_relaxed);
        while (current < dofs && !s_maximumMatrixDofs.compare_exchange_weak(current, dofs, std::memory_order_relaxed))
        {
        }
    }
    // 返回已处理矩阵的最大自由度数。
    static int MaximumMatrixDofs()
    {
        return s_maximumMatrixDofs.load(std::memory_order_relaxed);
    }

private:
    static std::atomic_bool s_enabled;           // GPU 求解开关
    static std::atomic_int s_attempts;           // 尝试次数
    static std::atomic_int s_successes;          // 成功次数
    static std::atomic_int s_fallbacks;          // 回退次数
    static std::atomic_int s_maximumMatrixDofs;  // 最大矩阵自由度数
};
}
