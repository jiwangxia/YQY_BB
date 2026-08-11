#pragma once

#include <algorithm>
#include <atomic>

namespace SolverNameSpace
{
// Process-wide settings for the shared-memory element assembly workers.
class AssemblySettings final
{
public:
    static void SetThreadCount(int threadCount)
    {
        s_threadCount.store(std::max(1, threadCount), std::memory_order_relaxed);
    }

    static int ThreadCount()
    {
        return s_threadCount.load(std::memory_order_relaxed);
    }

private:
    inline static std::atomic<int> s_threadCount{ 1 };
};
}
