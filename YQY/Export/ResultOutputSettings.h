#pragma once

#include <algorithm>
#include <atomic>

// 管理结果流写入的进程内配置。
class ResultOutputSettings final
{
public:
    static constexpr int DefaultFrameBatchSize = 8;   // 默认单批帧数量
    static constexpr int MinimumFrameBatchSize = 1;   // 最小单批帧数量
    static constexpr int MaximumFrameBatchSize = 128; // 最大单批帧数量

    // 设置单次写入的帧数量。
    static void SetFrameBatchSize(int frameCount)
    {
        s_frameBatchSize.store(std::clamp(frameCount, MinimumFrameBatchSize, MaximumFrameBatchSize),
                               std::memory_order_relaxed);
    }

    // 返回当前单次写入的帧数量。
    static int FrameBatchSize()
    {
        return s_frameBatchSize.load(std::memory_order_relaxed);
    }

    // 设置是否在后台线程写入结果。
    static void SetBackgroundWriteEnabled(bool enabled)
    {
        s_backgroundWriteEnabled.store(enabled, std::memory_order_relaxed);
    }

    // 返回后台写入是否启用。
    static bool IsBackgroundWriteEnabled()
    {
        return s_backgroundWriteEnabled.load(std::memory_order_relaxed);
    }

private:
    inline static std::atomic<int> s_frameBatchSize{DefaultFrameBatchSize}; // 帧批量大小
    inline static std::atomic<bool> s_backgroundWriteEnabled{true}; // 后台写入开关
};
