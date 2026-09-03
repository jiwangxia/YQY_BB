#include "AnalysisSolve.h"

#include "DataStructure/AnalysisStep/AnalysisStep.h"

#include <QDebug>
#include <QThread>
#include <QThreadPool>
#include <QtConcurrentRun>

#include <algorithm>
#include <map>
#include <mutex>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>

void AnalysisRunner::SetStructure(std::shared_ptr<StructureData> pStructure)
{
    m_pStructure = pStructure;
}

void AnalysisRunner::SetRuntimeCallbacks(ProgressCallback progressCallback, CancelCallback cancelCallback)
{
    m_progressCallback = std::move(progressCallback);
    m_cancelCallback = std::move(cancelCallback);
}

void AnalysisRunner::SetMaximumRegionThreads(int count)
{
    m_maximumRegionThreads = std::max(1, count);
}

bool AnalysisRunner::RunAll()
{
    auto structure = m_pStructure.lock();
    if (!structure || structure->m_AnalysisStep.empty())
        return false;

    std::vector<int> stepIds;
    stepIds.reserve(structure->m_AnalysisStep.size());
    for (const auto& [stepId, step] : structure->m_AnalysisStep)
    {
        if (step)
            stepIds.push_back(stepId);
    }
    return RunSelectedByRegions(stepIds);
}

bool AnalysisRunner::RunStep(int stepId)
{
    return RunSelectedByRegions({stepId});
}

bool AnalysisRunner::RunStepFromCurrentState(int stepId)
{
    m_wasCancelled = false;
    auto structure = m_pStructure.lock();
    if (!structure)
        return false;
    structure->EnsureDefaultAnalysisConfiguration();
    const auto stepIt = structure->m_AnalysisStep.find(stepId);
    if (stepIt == structure->m_AnalysisStep.end() || !stepIt->second)
        return false;
    stepIt->second->SetInitializeFromCurrentState(true);
    return RunStepDirect(stepId);
}

bool AnalysisRunner::RunSelectedByRegions(const std::vector<int>& stepIds)
{
    m_wasCancelled = false;
    auto structure = m_pStructure.lock();
    if (!structure)
        return false;

    // Programmatically generated models may already contain analysis steps but
    // no explicit compute region.  Normalize them exactly as imported models
    // before building either an independent or a dependent solve schedule.
    structure->EnsureDefaultAnalysisConfiguration();

    const bool hasStaticDependency = std::any_of(stepIds.cbegin(), stepIds.cend(),
                                                 [&structure](int stepId)
                                                 {
                                                     const auto found = structure->m_AnalysisStep.find(stepId);
                                                     return found != structure->m_AnalysisStep.cend() &&
                                                            found->second && found->second->m_InitialStaticStepId > 0;
                                                 });
    if (hasStaticDependency)
        return RunWithStaticDependencies(stepIds);

    struct WorkItem
    {
        int stepId = 0;
        int regionId = 0;
    };
    struct WorkResult
    {
        bool succeeded = false;
        bool cancelled = false;
        QString errorMessage;
        std::shared_ptr<StructureData> model;
    };

    std::vector<WorkItem> workItems;
    for (int stepId : stepIds)
    {
        const auto stepIt = structure->m_AnalysisStep.find(stepId);
        if (stepIt == structure->m_AnalysisStep.cend() || !stepIt->second)
            return false;
        for (int regionId : structure->ResolveAnalysisStepRegionIds(*stepIt->second))
            workItems.push_back({stepId, regionId});
    }
    if (workItems.empty())
    {
        qDebug().noquote() << QStringLiteral("Error: 分析步没有可运行的启用计算区域。");
        return false;
    }

    QString validationError;
    if (!structure->ValidateComputeRegions(&validationError))
    {
        qDebug().noquote() << QStringLiteral("Error:") << validationError;
        return false;
    }

    if (workItems.size() == 1)
    {
        const WorkItem work = workItems.front();
        const auto regionIt = structure->m_ComputeRegions.find(work.regionId);
        if (regionIt != structure->m_ComputeRegions.cend() && regionIt->second &&
            regionIt->second->m_NodeIds.size() == structure->m_Nodes.size() &&
            regionIt->second->m_ElementIds.size() == structure->m_Elements.size())
        {
            return RunStepDirect(work.stepId);
        }
    }

    structure->GetOutputter().Clear();
    std::mutex progressMutex;
    std::vector<double> progressValues(workItems.size(), 0.0);
    std::vector<WorkResult> results(workItems.size());

    const auto runWorkItem = [this, structure, &workItems, &progressValues,
                              &progressMutex](std::size_t index) -> WorkResult
    {
        WorkResult result;
        const WorkItem work = workItems.at(index);
        QString cloneError;
        result.model = structure->CloneRegionForAnalysis(work.regionId, work.stepId, &cloneError);
        if (!result.model)
        {
            result.errorMessage = cloneError;
            return result;
        }

        result.model->m_OutputControl.m_StreamResult = false;
        AnalysisRunner child;
        child.SetStructure(result.model);
        child.SetRuntimeCallbacks(
            [this, index, work, &progressValues, &progressMutex](double progress, const QString& message)
            {
                double overall = 0.0;
                {
                    std::lock_guard<std::mutex> lock(progressMutex);
                    progressValues.at(index) = progress;
                    for (double value : progressValues)
                        overall += value;
                    overall /= static_cast<double>(progressValues.size());
                }
                if (m_progressCallback)
                {
                    m_progressCallback(
                        overall,
                        QStringLiteral("区域 %1 · 分析步 %2 · %3").arg(work.regionId).arg(work.stepId).arg(message));
                }
            },
            m_cancelCallback);
        result.succeeded = child.RunStepDirect(work.stepId, false);
        result.cancelled = child.WasCancelled() || (m_cancelCallback && m_cancelCallback());
        if (!result.succeeded && !result.cancelled)
        {
            result.errorMessage = QStringLiteral("区域 %1 的分析步 %2 求解失败。").arg(work.regionId).arg(work.stepId);
        }
        return result;
    };

    const int idealThreads = std::max(1, QThread::idealThreadCount());
    const int reservedThreads = std::max(1, (idealThreads + 3) / 4);
    const int automaticLimit = std::max(1, std::min(12, idealThreads - reservedThreads));
    const int configuredLimit = m_maximumRegionThreads > 0 ? m_maximumRegionThreads : automaticLimit;
    const int parallelLimit = std::max(1, std::min(configuredLimit, static_cast<int>(workItems.size())));
    QThreadPool regionThreadPool;
    regionThreadPool.setMaxThreadCount(parallelLimit);

    for (std::size_t first = 0; first < workItems.size(); first += parallelLimit)
    {
        if (m_cancelCallback && m_cancelCallback())
        {
            m_wasCancelled = true;
            break;
        }
        const std::size_t last = std::min(workItems.size(), first + parallelLimit);
        std::vector<QFuture<WorkResult>> futures;
        futures.reserve(last - first);
        for (std::size_t index = first; index < last; ++index)
            futures.push_back(QtConcurrent::run(&regionThreadPool, runWorkItem, index));
        for (std::size_t index = first; index < last; ++index)
            results.at(index) = futures.at(index - first).result();
    }

    bool succeeded = !m_wasCancelled;
    for (const WorkResult& result : results)
    {
        if (result.model)
            structure->GetOutputter().MergeFramesFrom(result.model->GetOutputter());
        succeeded = succeeded && result.succeeded;
        m_wasCancelled = m_wasCancelled || result.cancelled;
        if (!result.errorMessage.isEmpty())
            qDebug().noquote() << QStringLiteral("Error:") << result.errorMessage;
    }

    if (structure->GetOutputter().GetFrameCount() > 0)
    {
        succeeded = structure->GetOutputter().SaveHdf5File(structure->m_OutputControl.m_Hdf5FileName, structure.get(),
                                                           structure->m_OutputControl.m_SourceModelName,
                                                           succeeded && !m_wasCancelled) &&
                    succeeded;
    }
    else
    {
        qDebug().noquote() << QStringLiteral("Error: 分析没有可写入 H5 的结果帧");
        succeeded = false;
    }
    if (m_progressCallback && succeeded)
        m_progressCallback(1.0, QStringLiteral("全部计算区域完成"));
    return succeeded && !m_wasCancelled;
}

bool AnalysisRunner::RunWithStaticDependencies(const std::vector<int>& stepIds)
{
    auto structure = m_pStructure.lock();
    if (!structure)
        return false;

    struct WorkItem
    {
        int stepId = 0;
        int regionId = 0;
    };
    struct DependencyKey
    {
        int staticStepId = 0;
        int regionId = 0;
        bool operator<(const DependencyKey& other) const
        {
            return std::tie(staticStepId, regionId) < std::tie(other.staticStepId, other.regionId);
        }
    };

    QString validationError;
    if (!structure->ValidateComputeRegions(&validationError))
    {
        qDebug().noquote() << QStringLiteral("Error:") << validationError;
        return false;
    }

    std::set<int> requested(stepIds.cbegin(), stepIds.cend());
    std::map<DependencyKey, std::vector<int>> dynamicByEquilibrium;
    std::vector<WorkItem> standaloneItems;

    for (int stepId : requested)
    {
        const auto stepIt = structure->m_AnalysisStep.find(stepId);
        if (stepIt == structure->m_AnalysisStep.cend() || !stepIt->second)
            return false;
        const auto& step = *stepIt->second;
        const std::vector<int> regionIds = structure->ResolveAnalysisStepRegionIds(step);
        if (regionIds.empty())
        {
            qDebug().noquote() << QStringLiteral("Error: 分析步 %1 没有可运行的启用计算区域。").arg(stepId);
            return false;
        }
        if (step.m_InitialStaticStepId <= 0)
        {
            for (int regionId : regionIds)
                standaloneItems.push_back({stepId, regionId});
            continue;
        }
        const auto staticIt = structure->m_AnalysisStep.find(step.m_InitialStaticStepId);
        if (step.m_Type != EnumKeyword::StepType::DYNAMIC || staticIt == structure->m_AnalysisStep.cend() ||
            !staticIt->second || staticIt->second->m_Type != EnumKeyword::StepType::STATIC || staticIt->first >= stepId)
        {
            qDebug().noquote() << QStringLiteral("Error: 动力步 %1 引用的前置静力步无效。").arg(stepId);
            return false;
        }
        const std::vector<int> staticRegionVector = structure->ResolveAnalysisStepRegionIds(*staticIt->second);
        const std::set<int> staticRegions(staticRegionVector.cbegin(), staticRegionVector.cend());
        for (int regionId : regionIds)
        {
            if (staticRegions.find(regionId) == staticRegions.cend())
            {
                qDebug().noquote() << QStringLiteral("Error: 动力步 %1 的计算区域 %2 未包含在前置静力步 %3 中。")
                                          .arg(stepId)
                                          .arg(regionId)
                                          .arg(staticIt->first);
                return false;
            }
            dynamicByEquilibrium[{staticIt->first, regionId}].push_back(stepId);
        }
    }

    // A static step/region pair referenced by a dynamic branch is solved by the
    // dependency chain and must not also remain as an independent work item.
    standaloneItems.erase(std::remove_if(standaloneItems.begin(), standaloneItems.end(),
                                         [&dynamicByEquilibrium](const WorkItem& work)
                                         {
                                             return dynamicByEquilibrium.find({work.stepId, work.regionId}) !=
                                                    dynamicByEquilibrium.cend();
                                         }),
                          standaloneItems.end());

    const int operationCount = std::max(
        1, static_cast<int>(standaloneItems.size() + dynamicByEquilibrium.size() +
                            std::accumulate(dynamicByEquilibrium.cbegin(), dynamicByEquilibrium.cend(), std::size_t{0},
                                            [](std::size_t total, const auto& item)
                                            {
                                                return total + item.second.size();
                                            })));
    int completedOperations = 0;
    structure->GetOutputter().Clear();

    const auto runOne = [this, structure, operationCount,
                         &completedOperations](const std::shared_ptr<StructureData>& model, int stepId, int regionId,
                                               bool inheritedState, bool publishResults) -> bool
    {
        if (!model)
            return false;
        model->m_OutputControl.m_StreamResult = false;
        const auto stepIt = model->m_AnalysisStep.find(stepId);
        if (stepIt == model->m_AnalysisStep.cend() || !stepIt->second)
            return false;
        stepIt->second->SetInitializeFromCurrentState(inheritedState);
        AnalysisRunner child;
        child.SetStructure(model);
        child.SetRuntimeCallbacks(
            [this, operationCount, &completedOperations, stepId, regionId](double progress, const QString& message)
            {
                if (m_progressCallback)
                {
                    m_progressCallback(
                        (completedOperations + std::clamp(progress, 0.0, 1.0)) / operationCount,
                        QStringLiteral("区域 %1 · 分析步 %2 · %3").arg(regionId).arg(stepId).arg(message));
                }
            },
            m_cancelCallback);
        const bool succeeded = child.RunStepDirect(stepId, false);
        m_wasCancelled = m_wasCancelled || child.WasCancelled();
        if (!succeeded)
            return false;
        if (publishResults)
            structure->GetOutputter().MergeFramesFrom(model->GetOutputter());
        ++completedOperations;
        return true;
    };

    for (const WorkItem& work : standaloneItems)
    {
        QString cloneError;
        auto model = structure->CloneRegionForAnalysis(work.regionId, work.stepId, &cloneError);
        if (!model || !runOne(model, work.stepId, work.regionId, false, true))
        {
            if (!cloneError.isEmpty())
                qDebug().noquote() << QStringLiteral("Error:") << cloneError;
            return false;
        }
    }

    for (const auto& [key, dynamicIds] : dynamicByEquilibrium)
    {
        QString cloneError;
        std::set<int> chainStepIds{key.staticStepId};
        chainStepIds.insert(dynamicIds.cbegin(), dynamicIds.cend());
        auto equilibriumModel = structure->CloneRegionForAnalysis(key.regionId, chainStepIds, &cloneError);
        if (!equilibriumModel || !runOne(equilibriumModel, key.staticStepId, key.regionId, false,
                                         requested.find(key.staticStepId) != requested.cend()))
        {
            if (!cloneError.isEmpty())
                qDebug().noquote() << QStringLiteral("Error:") << cloneError;
            return false;
        }
        for (int dynamicStepId : dynamicIds)
        {
            auto dynamicModel = equilibriumModel->CloneForAnalysis(&cloneError);
            if (!dynamicModel || !runOne(dynamicModel, dynamicStepId, key.regionId, true, true))
            {
                if (!cloneError.isEmpty())
                    qDebug().noquote() << QStringLiteral("Error:") << cloneError;
                return false;
            }
        }
    }

    const bool hasFrames = structure->GetOutputter().GetFrameCount() > 0;
    const bool saved = hasFrames && structure->GetOutputter().SaveHdf5File(
                                        structure->m_OutputControl.m_Hdf5FileName, structure.get(),
                                        structure->m_OutputControl.m_SourceModelName, !m_wasCancelled);
    if (saved && m_progressCallback)
        m_progressCallback(1.0, QStringLiteral("静力平衡与动力分析链计算完成"));
    return saved && !m_wasCancelled;
}

bool AnalysisRunner::RunStepDirect(int stepId, bool persistHdf5)
{
    auto structure = m_pStructure.lock();
    if (!structure)
        return false;
    const auto stepIt = structure->m_AnalysisStep.find(stepId);
    if (stepIt == structure->m_AnalysisStep.end() || !stepIt->second)
        return false;

    auto& step = stepIt->second;
    step->m_Id = stepId;
    step->SetStructure(structure);
    step->SetRuntimeCallbacks(m_progressCallback, m_cancelCallback);
    const bool succeeded = step->Solve(persistHdf5);
    step->ClearRuntimeCallbacks();
    m_wasCancelled = !succeeded && m_cancelCallback && m_cancelCallback();
    return succeeded;
}
