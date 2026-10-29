#pragma once

#include "SystemEvaluation.h"

#include <cstddef>
#include <functional>
#include <vector>

class QThreadPool;

namespace SolverNameSpace
{
/// @brief 一个工作任务负责的连续单元区间。
struct ParallelWorkRange
{
    std::size_t first = 0;
    std::size_t last = 0;
};

/// @brief 与时间积分器无关的共享内存单元并行执行器。
class ParallelAssemblyEngine final
{
public:
    using IndexedWorker = std::function<void(std::size_t workerIndex)>;
    using RangeWorker = std::function<void(std::size_t workerIndex, const ParallelWorkRange& range)>;

    /// @brief 执行固定数量的工作者；单个工作者时保持串行执行。
    void ExecuteWorkers(const SystemEvaluationRequest& request, std::size_t workerCount, QThreadPool& threadPool,
                        const IndexedWorker& worker) const;

    /// @brief 按预先生成的单元区间执行工作者。
    void ExecuteRanges(const SystemEvaluationRequest& request, const std::vector<ParallelWorkRange>& ranges,
                       QThreadPool& threadPool, const RangeWorker& worker) const;
};
}
