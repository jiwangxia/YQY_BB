#include "AnalysisSolve.h"

#include "DataStructure/AnalysisStep/AnalysisStep.h"

#include <QDebug>
#include <QThread>

#include <algorithm>
#include <future>
#include <mutex>
#include <stdexcept>
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

bool AnalysisRunner::RunSelectedByRegions(const std::vector<int>& stepIds)
{
    m_wasCancelled = false;
    auto structure = m_pStructure.lock();
    if (!structure)
        return false;

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
        if (regionIt != structure->m_ComputeRegions.cend() && regionIt->second
            && regionIt->second->m_NodeIds.size() == structure->m_Nodes.size()
            && regionIt->second->m_ElementIds.size() == structure->m_Elements.size())
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

        result.model->m_OutputControl.m_EnableHdf5 = false;
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
                m_progressCallback(overall, QStringLiteral("区域 %1 · 分析步 %2 · %3")
                    .arg(work.regionId).arg(work.stepId).arg(message));
            }
        }, m_cancelCallback);
        result.succeeded = child.RunStepDirect(work.stepId);
        result.cancelled = child.WasCancelled()
            || (m_cancelCallback && m_cancelCallback());
        if (!result.succeeded && !result.cancelled)
        {
            result.errorMessage = QStringLiteral("区域 %1 的分析步 %2 求解失败。")
                .arg(work.regionId).arg(work.stepId);
        }
        return result;
    };

    const int idealThreads = std::max(1, QThread::idealThreadCount());
    const int reservedThreads = std::max(1, (idealThreads + 3) / 4);
    const int automaticLimit = std::max(1, std::min(12, idealThreads - reservedThreads));
    const int configuredLimit = m_maximumRegionThreads > 0
        ? m_maximumRegionThreads : automaticLimit;
    const int parallelLimit = std::max(1,
        std::min(configuredLimit, static_cast<int>(workItems.size())));

    for (std::size_t first = 0; first < workItems.size(); first += parallelLimit)
    {
        if (m_cancelCallback && m_cancelCallback())
        {
            m_wasCancelled = true;
            break;
        }
        const std::size_t last = std::min(workItems.size(), first + parallelLimit);
        std::vector<std::future<WorkResult>> futures;
        futures.reserve(last - first);
        for (std::size_t index = first; index < last; ++index)
            futures.push_back(std::async(std::launch::async, runWorkItem, index));
        for (std::size_t index = first; index < last; ++index)
            results.at(index) = futures.at(index - first).get();
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

    if (structure->m_OutputControl.m_EnableHdf5
        && structure->GetOutputter().GetFrameCount() > 0)
    {
        succeeded = structure->GetOutputter().SaveHdf5File(
            structure->m_OutputControl.m_Hdf5FileName,
            structure.get(), structure->m_OutputControl.m_SourceModelName,
            succeeded && !m_wasCancelled) && succeeded;
    }
    if (m_progressCallback && succeeded)
        m_progressCallback(1.0, QStringLiteral("全部计算区域完成"));
    return succeeded && !m_wasCancelled;
}

bool AnalysisRunner::RunStepDirect(int stepId)
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
    const bool succeeded = step->Solve();
    step->ClearRuntimeCallbacks();
    m_wasCancelled = !succeeded && m_cancelCallback && m_cancelCallback();
    return succeeded;
}
