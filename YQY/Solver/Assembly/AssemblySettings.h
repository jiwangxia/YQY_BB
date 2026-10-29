#pragma once

#include <algorithm>
#include <atomic>
#include <QThreadPool>

namespace SolverNameSpace
{
// 共享内存单元装配线程的全局设置。
class AssemblySettings final
{
public:
    static void SetThreadCount(int threadCount)
    {
        const int applied = std::max(1, threadCount);
        s_threadCount.store(applied, std::memory_order_relaxed);
        ThreadPool().setMaxThreadCount(applied);
    }

    static int ThreadCount()
    {
        return s_threadCount.load(std::memory_order_relaxed);
    }

    static QThreadPool& ThreadPool()
    {
        static QThreadPool threadPool;
        return threadPool;
    }

private:
    inline static std::atomic<int> s_threadCount{1};
};
}
