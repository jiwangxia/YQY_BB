#pragma once

#include <algorithm>
#include <atomic>

class ResultOutputSettings final
{
public:
    static constexpr int DefaultFrameBatchSize = 8;
    static constexpr int MinimumFrameBatchSize = 1;
    static constexpr int MaximumFrameBatchSize = 128;

    static void SetFrameBatchSize(int frameCount)
    {
        s_frameBatchSize.store(std::clamp(frameCount, MinimumFrameBatchSize, MaximumFrameBatchSize),
                               std::memory_order_relaxed);
    }

    static int FrameBatchSize()
    {
        return s_frameBatchSize.load(std::memory_order_relaxed);
    }

    static void SetBackgroundWriteEnabled(bool enabled)
    {
        s_backgroundWriteEnabled.store(enabled, std::memory_order_relaxed);
    }

    static bool IsBackgroundWriteEnabled()
    {
        return s_backgroundWriteEnabled.load(std::memory_order_relaxed);
    }

private:
    inline static std::atomic<int> s_frameBatchSize{DefaultFrameBatchSize};
    inline static std::atomic<bool> s_backgroundWriteEnabled{true};
};
