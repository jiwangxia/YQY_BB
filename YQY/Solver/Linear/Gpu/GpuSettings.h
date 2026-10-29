#pragma once

#include <atomic>
#include <cstdint>

#include <QString>

#include "CudssSolver.h"

namespace SolverNameSpace
{
// 管理 GPU 求解开关和运行统计。
class GpuSettings final
{
public:
    struct ThreadStatistics
    {
        int linearAttempts = 0;
        int linearSuccesses = 0;
        int cpuFallbacks = 0;
        int assemblyUses = 0;
        int skippedUses = 0;
    };

    static int MinimumGpuDofs();
    static int MinimumGpuTrussElements();
    static int MinimumGpuCableElements();
    static int MinimumGpuSharedCableElements();
    static int MinimumGpuBeamElements();
    static QString CalibrationDescription();
    // 调整共享节点索批处理的启用阈值，保留原校准与下限约束。
    static void SetMinimumGpuSharedCableElements(int minimumElements);

    // 设置 GPU 求解开关，首次启用时完成一次轻量设备校准。
    static void SetEnabled(bool enabled);

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
        s_cudssUploadMicroseconds.store(0, std::memory_order_relaxed);
        s_cudssAnalysisMicroseconds.store(0, std::memory_order_relaxed);
        s_cudssFactorizationMicroseconds.store(0, std::memory_order_relaxed);
        s_cudssSolveMicroseconds.store(0, std::memory_order_relaxed);
        s_cudssDownloadMicroseconds.store(0, std::memory_order_relaxed);
        s_cudssAnalysisReuses.store(0, std::memory_order_relaxed);
        ResetThreadReport();
    }
    // 记录一次 GPU 求解尝试。
    static void RecordAttempt();
    // 记录一次 GPU 求解成功。
    static void RecordSuccess();
    // 记录一次回退到 CPU 求解。
    static void RecordFallback(const QString& reason = QString());
    static void RecordSkipped(const QString& reason);
    static void RecordGpuUse(const QString& description);
    static void ResetThreadReport();
    static QString ThreadReport();
    static ThreadStatistics GetThreadStatistics();
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

    // 累计 cuDSS 分阶段耗时；以微秒整数保存，避免跨线程累计浮点数。
    static void RecordCudssTiming(const CudssSolveTiming& timing)
    {
        s_cudssUploadMicroseconds.fetch_add(ToMicroseconds(timing.uploadMilliseconds), std::memory_order_relaxed);
        s_cudssAnalysisMicroseconds.fetch_add(ToMicroseconds(timing.analysisMilliseconds),
                                               std::memory_order_relaxed);
        s_cudssFactorizationMicroseconds.fetch_add(ToMicroseconds(timing.factorizationMilliseconds),
                                                    std::memory_order_relaxed);
        s_cudssSolveMicroseconds.fetch_add(ToMicroseconds(timing.solveMilliseconds), std::memory_order_relaxed);
        s_cudssDownloadMicroseconds.fetch_add(ToMicroseconds(timing.downloadMilliseconds),
                                               std::memory_order_relaxed);
        if (timing.reusedAnalysis)
            s_cudssAnalysisReuses.fetch_add(1, std::memory_order_relaxed);
    }
    static double CudssUploadMilliseconds()
    {
        return ToMilliseconds(s_cudssUploadMicroseconds.load(std::memory_order_relaxed));
    }
    static double CudssAnalysisMilliseconds()
    {
        return ToMilliseconds(s_cudssAnalysisMicroseconds.load(std::memory_order_relaxed));
    }
    static double CudssFactorizationMilliseconds()
    {
        return ToMilliseconds(s_cudssFactorizationMicroseconds.load(std::memory_order_relaxed));
    }
    static double CudssSolveMilliseconds()
    {
        return ToMilliseconds(s_cudssSolveMicroseconds.load(std::memory_order_relaxed));
    }
    static double CudssDownloadMilliseconds()
    {
        return ToMilliseconds(s_cudssDownloadMicroseconds.load(std::memory_order_relaxed));
    }
    static int CudssAnalysisReuses()
    {
        return s_cudssAnalysisReuses.load(std::memory_order_relaxed);
    }

private:
    static void EnsureCalibrated();
    static void AppendThreadReport(const QString& message);
    static std::int64_t ToMicroseconds(double milliseconds)
    {
        return milliseconds > 0.0 ? static_cast<std::int64_t>(milliseconds * 1000.0 + 0.5) : 0;
    }
    static double ToMilliseconds(std::int64_t microseconds)
    {
        return static_cast<double>(microseconds) / 1000.0;
    }

    static std::atomic_bool s_enabled;           // GPU 求解开关
    static std::atomic_int s_minimumGpuDofs;
    static std::atomic_int s_minimumGpuTrussElements;
    static std::atomic_int s_minimumGpuCableElements;
    static std::atomic_int s_minimumGpuSharedCableElements;
    static std::atomic_int s_minimumGpuBeamElements;
    static std::atomic_int s_attempts;           // 尝试次数
    static std::atomic_int s_successes;          // 成功次数
    static std::atomic_int s_fallbacks;          // 回退次数
    static std::atomic_int s_maximumMatrixDofs;  // 最大矩阵自由度数
    static std::atomic_llong s_cudssUploadMicroseconds;
    static std::atomic_llong s_cudssAnalysisMicroseconds;
    static std::atomic_llong s_cudssFactorizationMicroseconds;
    static std::atomic_llong s_cudssSolveMicroseconds;
    static std::atomic_llong s_cudssDownloadMicroseconds;
    static std::atomic_int s_cudssAnalysisReuses;
};
}
