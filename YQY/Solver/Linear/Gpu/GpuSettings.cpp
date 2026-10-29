#include "GpuSettings.h"

#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

namespace SolverNameSpace
{
namespace
{
constexpr int DefaultMinimumGpuDofs = 10000;
constexpr int DefaultMinimumGpuTrussElements = 65536;
constexpr int DefaultMinimumGpuCableElements = 32768;
constexpr int DefaultMinimumGpuBeamElements = 4096;

std::once_flag calibrationFlag;
QString calibrationDescription = QStringLiteral("GPU 尚未校准");
thread_local QString threadReport;
thread_local GpuSettings::ThreadStatistics threadStatistics;

int ScaleThreshold(int value, double scale, int lowerBound)
{
    const int scaled = static_cast<int>(std::lround(static_cast<double>(value) * scale));
    return std::max(lowerBound, ((scaled + 255) / 256) * 256);
}
}

std::atomic_bool GpuSettings::s_enabled{ false };
std::atomic_int GpuSettings::s_minimumGpuDofs{ DefaultMinimumGpuDofs };
std::atomic_int GpuSettings::s_minimumGpuTrussElements{ DefaultMinimumGpuTrussElements };
std::atomic_int GpuSettings::s_minimumGpuCableElements{ DefaultMinimumGpuCableElements };
std::atomic_int GpuSettings::s_minimumGpuSharedCableElements{ DefaultMinimumGpuCableElements };
std::atomic_int GpuSettings::s_minimumGpuBeamElements{ DefaultMinimumGpuBeamElements };
std::atomic_int GpuSettings::s_attempts{ 0 };
std::atomic_int GpuSettings::s_successes{ 0 };
std::atomic_int GpuSettings::s_fallbacks{ 0 };
std::atomic_int GpuSettings::s_maximumMatrixDofs{ 0 };
std::atomic_llong GpuSettings::s_cudssUploadMicroseconds{ 0 };
std::atomic_llong GpuSettings::s_cudssAnalysisMicroseconds{ 0 };
std::atomic_llong GpuSettings::s_cudssFactorizationMicroseconds{ 0 };
std::atomic_llong GpuSettings::s_cudssSolveMicroseconds{ 0 };
std::atomic_llong GpuSettings::s_cudssDownloadMicroseconds{ 0 };
std::atomic_int GpuSettings::s_cudssAnalysisReuses{ 0 };

void GpuSettings::EnsureCalibrated()
{
    std::call_once(calibrationFlag,
                   []()
                   {
                       int device = 0;
                       cudaDeviceProp properties{};
                       if (cudaGetDevice(&device) != cudaSuccess ||
                           cudaGetDeviceProperties(&properties, device) != cudaSuccess)
                       {
                           calibrationDescription = QStringLiteral("未检测到可校准的 CUDA 设备，沿用保守阈值");
                           cudaGetLastError();
                           return;
                       }

                       constexpr std::size_t transferBytes = 8U * 1024U * 1024U;
                       constexpr int repetitions = 4;
                       void* hostMemory = nullptr;
                       void* deviceMemory = nullptr;
                       double transferBandwidth = 0.0;
                       if (cudaHostAlloc(&hostMemory, transferBytes, cudaHostAllocDefault) == cudaSuccess &&
                           cudaMalloc(&deviceMemory, transferBytes) == cudaSuccess)
                       {
                           cudaMemset(deviceMemory, 0, transferBytes);
                           cudaMemcpy(hostMemory, deviceMemory, transferBytes, cudaMemcpyDeviceToHost);
                           const auto start = std::chrono::steady_clock::now();
                           for (int repetition = 0; repetition < repetitions; ++repetition)
                           {
                               cudaMemcpy(deviceMemory, hostMemory, transferBytes, cudaMemcpyHostToDevice);
                               cudaMemcpy(hostMemory, deviceMemory, transferBytes, cudaMemcpyDeviceToHost);
                           }
                           const double seconds =
                               std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
                           if (seconds > 0.0)
                           {
                               transferBandwidth = 2.0 * repetitions * static_cast<double>(transferBytes) /
                                                   seconds / 1.0e9;
                           }
                       }
                       if (deviceMemory)
                           cudaFree(deviceMemory);
                       if (hostMemory)
                           cudaFreeHost(hostMemory);

                       int memoryClockRate = 0;
                       cudaDeviceGetAttribute(&memoryClockRate, cudaDevAttrMemoryClockRate, device);
                       const double deviceBandwidth =
                           memoryClockRate > 0 && properties.memoryBusWidth > 0
                               ? 2.0 * static_cast<double>(memoryClockRate) *
                                     static_cast<double>(properties.memoryBusWidth) / 8.0e6
                               : 400.0;
                       const double transferScale = transferBandwidth > 0.0
                                                        ? std::clamp(std::sqrt(12.0 / transferBandwidth), 0.75, 1.5)
                                                        : 1.0;
                       const double deviceScale =
                           std::clamp(std::sqrt(400.0 / std::max(100.0, deviceBandwidth)), 0.75, 1.25);
                       const double rawThresholdScale = std::clamp(transferScale * deviceScale, 0.6, 1.6);
                       const double thresholdScale = std::round(rawThresholdScale * 4.0) / 4.0;

                       s_minimumGpuDofs.store(ScaleThreshold(DefaultMinimumGpuDofs, thresholdScale, 6144),
                                              std::memory_order_relaxed);
                       s_minimumGpuTrussElements.store(
                           ScaleThreshold(DefaultMinimumGpuTrussElements, thresholdScale, 32768),
                           std::memory_order_relaxed);
                       s_minimumGpuCableElements.store(
                           ScaleThreshold(DefaultMinimumGpuCableElements, thresholdScale, 16384),
                           std::memory_order_relaxed);
                       s_minimumGpuSharedCableElements.store(
                           ScaleThreshold(DefaultMinimumGpuCableElements, thresholdScale, 16384),
                           std::memory_order_relaxed);
                       s_minimumGpuBeamElements.store(
                           ScaleThreshold(DefaultMinimumGpuBeamElements, thresholdScale, 2048),
                           std::memory_order_relaxed);
                       calibrationDescription =
                           QStringLiteral("GPU 校准：%1，双向传输 %2 GB/s；阈值 DOF=%3、桁架=%4、索=%5、梁=%6")
                               .arg(QString::fromLatin1(properties.name))
                               .arg(transferBandwidth, 0, 'f', 2)
                               .arg(s_minimumGpuDofs.load(std::memory_order_relaxed))
                               .arg(s_minimumGpuTrussElements.load(std::memory_order_relaxed))
                               .arg(s_minimumGpuCableElements.load(std::memory_order_relaxed))
                               .arg(s_minimumGpuBeamElements.load(std::memory_order_relaxed));
                   });
}

void GpuSettings::SetEnabled(bool enabled)
{
    if (enabled)
        EnsureCalibrated();
    s_enabled.store(enabled, std::memory_order_relaxed);
}

int GpuSettings::MinimumGpuDofs()
{
    EnsureCalibrated();
    return s_minimumGpuDofs.load(std::memory_order_relaxed);
}

int GpuSettings::MinimumGpuTrussElements()
{
    EnsureCalibrated();
    return s_minimumGpuTrussElements.load(std::memory_order_relaxed);
}

int GpuSettings::MinimumGpuCableElements()
{
    EnsureCalibrated();
    return s_minimumGpuCableElements.load(std::memory_order_relaxed);
}

int GpuSettings::MinimumGpuSharedCableElements()
{
    EnsureCalibrated();
    return s_minimumGpuSharedCableElements.load(std::memory_order_relaxed);
}

void GpuSettings::SetMinimumGpuSharedCableElements(int minimumElements)
{
    EnsureCalibrated();
    s_minimumGpuSharedCableElements.store(std::max(1, minimumElements), std::memory_order_relaxed);
}

int GpuSettings::MinimumGpuBeamElements()
{
    EnsureCalibrated();
    return s_minimumGpuBeamElements.load(std::memory_order_relaxed);
}

QString GpuSettings::CalibrationDescription()
{
    EnsureCalibrated();
    return calibrationDescription;
}

void GpuSettings::AppendThreadReport(const QString& message)
{
    if (message.isEmpty() || threadReport.contains(message))
        return;
    if (!threadReport.isEmpty())
        threadReport += QStringLiteral("；");
    threadReport += message;
}

void GpuSettings::RecordSuccess()
{
    s_successes.fetch_add(1, std::memory_order_relaxed);
    ++threadStatistics.linearSuccesses;
    AppendThreadReport(QStringLiteral("GPU 线性求解已实际执行"));
}

void GpuSettings::RecordAttempt()
{
    s_attempts.fetch_add(1, std::memory_order_relaxed);
    ++threadStatistics.linearAttempts;
}

void GpuSettings::RecordFallback(const QString& reason)
{
    s_fallbacks.fetch_add(1, std::memory_order_relaxed);
    ++threadStatistics.cpuFallbacks;
    AppendThreadReport(reason.isEmpty() ? QStringLiteral("GPU 失败后已回退 CPU") : reason);
}

void GpuSettings::RecordSkipped(const QString& reason)
{
    ++threadStatistics.skippedUses;
    AppendThreadReport(reason);
}

void GpuSettings::RecordGpuUse(const QString& description)
{
    ++threadStatistics.assemblyUses;
    AppendThreadReport(description);
}

void GpuSettings::ResetThreadReport()
{
    threadReport.clear();
    threadStatistics = {};
}

QString GpuSettings::ThreadReport()
{
    QString report = QStringLiteral("GPU统计：线性尝试 %1 次、成功 %2 次，单元批计算 %3 次，CPU回退 %4 次")
                         .arg(threadStatistics.linearAttempts)
                         .arg(threadStatistics.linearSuccesses)
                         .arg(threadStatistics.assemblyUses)
                         .arg(threadStatistics.cpuFallbacks);
    if (threadStatistics.skippedUses > 0)
        report += QStringLiteral("，因阈值跳过 %1 次").arg(threadStatistics.skippedUses);
    if (!threadReport.isEmpty())
        report += QStringLiteral("；%1").arg(threadReport);
    return report;
}

GpuSettings::ThreadStatistics GpuSettings::GetThreadStatistics()
{
    return threadStatistics;
}
}
