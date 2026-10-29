#include "ParallelAssemblyEngine.h"

namespace SolverNameSpace
{
void ParallelAssemblyEngine::ExecuteWorkers(const SystemEvaluationRequest& request, std::size_t workerCount,
                                            QThreadPool& threadPool, const IndexedWorker& worker) const
{
    if (workerCount == 0 || !worker || request.contributions == ToMask(SystemContribution::None))
        return;
    if (workerCount == 1)
    {
        worker(0);
        return;
    }

    std::vector<QFuture<void>> futures;
    futures.reserve(workerCount);
    for (std::size_t workerIndex = 0; workerIndex < workerCount; ++workerIndex)
        futures.push_back(QtConcurrent::run(&threadPool, worker, workerIndex));
    for (QFuture<void>& future : futures)
        future.waitForFinished();
}

void ParallelAssemblyEngine::ExecuteRanges(const SystemEvaluationRequest& request,
                                           const std::vector<ParallelWorkRange>& ranges, QThreadPool& threadPool,
                                           const RangeWorker& worker) const
{
    if (ranges.empty() || !worker)
        return;
    ExecuteWorkers(request, ranges.size(), threadPool,
                   [&ranges, &worker](std::size_t workerIndex) { worker(workerIndex, ranges[workerIndex]); });
}
}
