#include "AnalysisSolve.h"

#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "DataStructure/Structure/StructureData.h"
#include "Solver/Assembly/AssemblySettings.h"
#include "Solver/Factory/SolverFactory.h"
#include "Utility/Logger/Logger.h"

#include <QThread>
#include <Eigen/Eigenvalues>
#include <numbers>

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

void AnalysisRunner::SetExecutionThreadBudget(int count)
{
    m_executionThreadBudget = count > 0 ? std::max(1, count) : 0;
}

void AnalysisRunner::SetInitialRegionStates(const std::map<int, std::shared_ptr<StructureData>>& states)
{
    m_initialRegionStates = states;
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
    if (!m_initialRegionStates.empty())
        return RunSelectedByRegions({stepId});
    m_wasCancelled = false;
    m_solvedRegionStates.clear();
    auto structure = m_pStructure.lock();
    if (!structure)
        return false;
    structure->EnsureDefaultAnalysisConfiguration();
    const auto stepIt = structure->m_AnalysisStep.find(stepId);
    if (stepIt == structure->m_AnalysisStep.end() || !stepIt->second)
        return false;
    stepIt->second->SetInitializeFromCurrentState(true);
    return RunStepDirect(stepId, true, true);
}

bool AnalysisRunner::RunSelectedByRegions(const std::vector<int>& stepIds)
{
    m_wasCancelled = false;
    m_solvedRegionStates.clear();
    const bool inheritedStaticState = !m_initialRegionStates.empty();
    auto structure = m_pStructure.lock();
    if (!structure)
        return false;

    // 生成模型可能已有分析步但尚未定义计算区域；构建求解调度前，按导入模型的规则补齐默认配置。
    structure->EnsureDefaultAnalysisConfiguration();

    const bool hasStaticDependency = std::any_of(stepIds.cbegin(), stepIds.cend(),
                                                 [&structure](int stepId)
                                                 {
                                                     const auto found = structure->m_AnalysisStep.find(stepId);
                                                     return found != structure->m_AnalysisStep.cend() &&
                                                            found->second && found->second->m_InitialStaticStepId > 0;
                                                 });
    if (hasStaticDependency && !inheritedStaticState)
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

    if (workItems.size() == 1 && !inheritedStaticState)
    {
        const WorkItem work = workItems.front();
        const auto regionIt = structure->m_ComputeRegions.find(work.regionId);
        if (regionIt != structure->m_ComputeRegions.cend() && regionIt->second &&
            regionIt->second->m_NodeIds.size() == structure->m_Nodes.size() &&
            regionIt->second->m_ElementIds.size() == structure->m_Elements.size())
        {
            const bool succeeded = RunStepDirect(work.stepId);
            if (succeeded)
                m_solvedRegionStates.emplace(work.regionId, structure);
            return succeeded;
        }
    }

    if (!inheritedStaticState)
        structure->GetOutputter().Clear();
    std::mutex progressMutex;
    std::vector<double> progressValues(workItems.size(), 0.0);
    std::vector<WorkResult> results(workItems.size());

    const int idealThreads = std::max(1, QThread::idealThreadCount());
    const int reservedThreads = std::max(1, (idealThreads + 3) / 4);
    const int availableCpuThreads = m_executionThreadBudget > 0
                                        ? m_executionThreadBudget
                                        : std::max(1, idealThreads - reservedThreads);
    const int automaticLimit = std::max(1, std::min(12, availableCpuThreads));
    const int configuredLimit = m_maximumRegionThreads > 0 ? m_maximumRegionThreads : automaticLimit;
    const int parallelLimit = std::max(
        1, std::min({configuredLimit, availableCpuThreads, static_cast<int>(workItems.size())}));
    // 区域任务与单任务内的单元装配共用同一 CPU 预算。若不拆分，N 个区域各自
    // 再启动 N 个装配线程会形成嵌套并发，反而拉长总用时并影响交互响应。
    const int assemblyThreadsPerRegion =
        std::max(1, std::min(SolverNameSpace::AssemblySettings::ThreadCount(), availableCpuThreads / parallelLimit));
    // PARDISO 自身还会创建 OpenMP 工作线程。区域并行时，以可用计算预算均分，
    // 并通过 oneMKL 的 thread-local 设置限制到当前区域任务，避免 N 倍超订阅。
    const int pardisoThreadsPerRegion = std::max(1, availableCpuThreads / parallelLimit);

    const auto runWorkItem = [this, structure, &workItems, &progressValues, &progressMutex,
                              assemblyThreadsPerRegion, pardisoThreadsPerRegion,
                              inheritedStaticState](std::size_t index) -> WorkResult
    {
        WorkResult result;
        const WorkItem work = workItems.at(index);
        QString cloneError;
        if (inheritedStaticState)
        {
            const auto equilibrium = m_initialRegionStates.find(work.regionId);
            if (equilibrium == m_initialRegionStates.cend() || !equilibrium->second)
            {
                result.errorMessage = QStringLiteral("区域 %1 缺少已收敛的前置静力状态。").arg(work.regionId);
                return result;
            }
            result.model = equilibrium->second->CloneForAnalysis(&cloneError);
            const auto selectedStep = structure->m_AnalysisStep.at(work.stepId);
            std::set<int> retainedStepIds{work.stepId};
            if (selectedStep->m_InitialStaticStepId > 0)
                retainedStepIds.insert(selectedStep->m_InitialStaticStepId);
            auto definitionModel = structure->CloneRegionForAnalysis(work.regionId, retainedStepIds, &cloneError);
            if (!result.model || !definitionModel)
            {
                result.errorMessage = cloneError;
                return result;
            }
            if (!result.model->CopyAnalysisBindingsFrom(*definitionModel, &cloneError))
            {
                result.errorMessage =
                    QStringLiteral("区域 %1 的动力荷载及约束更新失败：%2").arg(work.regionId).arg(cloneError);
                return result;
            }
            // 保留该区域的节点和单元平衡状态，动力定义取当前任务副本。
            result.model->m_AnalysisStep = std::move(definitionModel->m_AnalysisStep);
            result.model->m_OutputControl = definitionModel->m_OutputControl;
            result.model->m_AeroManager = definitionModel->m_AeroManager;
        }
        else
            result.model = structure->CloneRegionForAnalysis(work.regionId, work.stepId, &cloneError);
        if (!result.model)
        {
            result.errorMessage = cloneError;
            return result;
        }

        const auto childStep = result.model->m_AnalysisStep.find(work.stepId);
        if (childStep != result.model->m_AnalysisStep.cend() && childStep->second)
        {
            childStep->second->SetAssemblyThreadCountOverride(assemblyThreadsPerRegion);
            childStep->second->SetPardisoThreadCountOverride(pardisoThreadsPerRegion);
            childStep->second->SetInitializeFromCurrentState(inheritedStaticState);
        }

        result.model->m_OutputControl.m_StreamResult = false;
        AnalysisRunner child;
        child.SetStructure(result.model);
        child.SetExecutionThreadBudget(pardisoThreadsPerRegion);
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
        result.succeeded = child.RunStepDirect(work.stepId, false, inheritedStaticState);
        result.cancelled = child.WasCancelled() || (m_cancelCallback && m_cancelCallback());
        if (!result.succeeded && !result.cancelled)
        {
            const auto failedStep = result.model->m_AnalysisStep.find(work.stepId);
            const QString failureReason = failedStep != result.model->m_AnalysisStep.cend() && failedStep->second
                                              ? failedStep->second->LastFailureReason()
                                              : QString();
            result.errorMessage = failureReason.isEmpty()
                                      ? QStringLiteral("区域 %1 的分析步 %2 求解失败。").arg(work.regionId).arg(work.stepId)
                                      : QStringLiteral("区域 %1：%2").arg(work.regionId).arg(failureReason);
        }
        return result;
    };

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
    QString firstFailureReason;
    for (std::size_t index = 0; index < results.size(); ++index)
    {
        const WorkResult& result = results.at(index);
        if (result.model)
            structure->GetOutputter().MergeFramesFrom(result.model->GetOutputter());
        if (result.succeeded && result.model)
            m_solvedRegionStates[workItems.at(index).regionId] = result.model;
        succeeded = succeeded && result.succeeded;
        m_wasCancelled = m_wasCancelled || result.cancelled;
        if (!result.errorMessage.isEmpty())
        {
            qDebug().noquote() << QStringLiteral("Error:") << result.errorMessage;
            if (firstFailureReason.isEmpty())
                firstFailureReason = result.errorMessage;
        }
    }

    for (std::size_t index = 0; index < results.size(); ++index)
        if (!results[index].succeeded)
            structure->GetOutputter().InvalidateEnergy(workItems[index].stepId);

    if (!succeeded && !firstFailureReason.isEmpty())
    {
        for (const int stepId : stepIds)
        {
            const auto step = structure->m_AnalysisStep.find(stepId);
            if (step != structure->m_AnalysisStep.cend() && step->second)
                step->second->SetLastFailureReason(firstFailureReason);
        }
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
        SolverNameSpace::InitialStaticMode initialStaticMode = SolverNameSpace::InitialStaticMode::Configured;
        bool publishInitialStaticResults = true;
        bool operator<(const DependencyKey& other) const
        {
            return std::tie(staticStepId, regionId, initialStaticMode, publishInitialStaticResults) <
                   std::tie(other.staticStepId, other.regionId, other.initialStaticMode,
                            other.publishInitialStaticResults);
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
    int requestedOperationCount = 0;

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
        requestedOperationCount += static_cast<int>(regionIds.size());
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
            const auto executionPolicy =
                SolverNameSpace::SolverFactory::GetDynamicExecutionPolicy(step.m_DynamicSolverType);
            dynamicByEquilibrium[{staticIt->first, regionId, executionPolicy.initialStaticMode,
                                  executionPolicy.publishInitialStaticResults}]
                .push_back(stepId);
        }
    }

    // 只有原配置的静力结果可由依赖链代替；专用初态不覆盖用户独立选择的静力结果。
    standaloneItems.erase(std::remove_if(standaloneItems.begin(), standaloneItems.end(),
                                         [&dynamicByEquilibrium](const WorkItem& work)
                                         {
                                             return dynamicByEquilibrium.find(
                                                        {work.stepId, work.regionId,
                                                         SolverNameSpace::InitialStaticMode::Configured, true}) !=
                                                    dynamicByEquilibrium.cend();
                                         }),
                          standaloneItems.end());

    // 自动计算的前置静力只准备初始状态，不占用所选分析步的进度。
    const int operationCount = std::max(1, requestedOperationCount);
    int completedOperations = 0;
    structure->GetOutputter().Clear();

    const auto finalize = [this, structure](bool succeeded)
    {
        m_wasCancelled = m_wasCancelled || (m_cancelCallback && m_cancelCallback());
        const bool complete = succeeded && !m_wasCancelled;
        if (!complete)
            for (const auto& [stepId, step] : structure->m_AnalysisStep)
                structure->GetOutputter().InvalidateEnergy(stepId);
        const bool hasFrames = structure->GetOutputter().GetFrameCount() > 0;
        const bool saved = hasFrames && structure->GetOutputter().SaveHdf5File(
                                            structure->m_OutputControl.m_Hdf5FileName, structure.get(),
                                            structure->m_OutputControl.m_SourceModelName, complete);
        if (saved && complete && m_progressCallback)
            m_progressCallback(1.0, QStringLiteral("静力平衡与动力分析链计算完成"));
        return saved && complete;
    };

    const auto runOne = [this, structure, operationCount,
                         &completedOperations](const std::shared_ptr<StructureData>& model, int stepId, int regionId,
                                               bool inheritedState, bool publishResults) -> bool
    {
        if (m_cancelCallback && m_cancelCallback())
        {
            m_wasCancelled = true;
            return false;
        }
        if (!model)
            return false;
        model->m_OutputControl.m_StreamResult = false;
        const auto stepIt = model->m_AnalysisStep.find(stepId);
        if (stepIt == model->m_AnalysisStep.cend() || !stepIt->second)
            return false;
        stepIt->second->SetInitializeFromCurrentState(inheritedState);
        AnalysisRunner child;
        child.SetStructure(model);
        child.SetExecutionThreadBudget(m_executionThreadBudget);
        child.SetRuntimeCallbacks(
            [this, operationCount, &completedOperations, stepId, regionId, publishResults](double progress,
                                                                                           const QString& message)
            {
                if (m_progressCallback)
                {
                    m_progressCallback(
                        (completedOperations + (publishResults ? std::clamp(progress, 0.0, 1.0) : 0.0)) /
                            operationCount,
                        QStringLiteral("区域 %1 · 分析步 %2 · %3").arg(regionId).arg(stepId).arg(message));
                }
            },
            m_cancelCallback);
        const bool succeeded = child.RunStepDirect(stepId, false, inheritedState);
        m_wasCancelled = m_wasCancelled || child.WasCancelled();
        if (publishResults)
            structure->GetOutputter().MergeFramesFrom(model->GetOutputter());
        if (!succeeded)
        {
            const auto originalStep = structure->m_AnalysisStep.find(stepId);
            if (originalStep != structure->m_AnalysisStep.cend() && originalStep->second)
                originalStep->second->SetLastFailureReason(
                    stepIt->second->LastFailureReason().isEmpty()
                        ? QStringLiteral("区域 %1 的分析步 %2 求解失败。").arg(regionId).arg(stepId)
                        : stepIt->second->LastFailureReason());
            qInfo().noquote() << QStringLiteral("YQY_STEP_FAILURE 区域 %1，分析步 %2：%3")
                                     .arg(regionId)
                                     .arg(stepId)
                                     .arg(stepIt->second->LastFailureReason());
            return false;
        }
        if (publishResults)
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
            return finalize(false);
        }
    }

    for (const auto& [key, dynamicIds] : dynamicByEquilibrium)
    {
        QString cloneError;
        std::set<int> chainStepIds{key.staticStepId};
        chainStepIds.insert(dynamicIds.cbegin(), dynamicIds.cend());
        auto equilibriumModel = structure->CloneRegionForAnalysis(key.regionId, chainStepIds, &cloneError);
        if (equilibriumModel && key.initialStaticMode == SolverNameSpace::InitialStaticMode::ReferenceWindless)
        {
            // 独立无风平衡副本只供参考动力分支继承，不覆盖用户原来的静力分析定义或结果。
            const auto equilibriumStep = equilibriumModel->m_AnalysisStep.at(key.staticStepId);
            equilibriumStep->m_EnableGalloping = false;
            equilibriumStep->SetReferenceGravityEquilibrium(true);
            qInfo().noquote() << QStringLiteral("参考 TSSBN 舞动：区域 %1 单独计算无风静力平衡，静力步 %2。")
                                     .arg(key.regionId)
                                     .arg(key.staticStepId);
        }
        if (!equilibriumModel ||
            !runOne(equilibriumModel, key.staticStepId, key.regionId, false,
                    key.publishInitialStaticResults && requested.find(key.staticStepId) != requested.cend()))
        {
            if (!cloneError.isEmpty())
            {
                if (const auto step = structure->m_AnalysisStep.find(key.staticStepId);
                    step != structure->m_AnalysisStep.cend() && step->second)
                    step->second->SetLastFailureReason(cloneError);
                qDebug().noquote() << QStringLiteral("Error:") << cloneError;
            }
            return finalize(false);
        }
        for (int dynamicStepId : dynamicIds)
        {
            auto dynamicModel = equilibriumModel->CloneForAnalysis(&cloneError);
            if (!dynamicModel || !runOne(dynamicModel, dynamicStepId, key.regionId, true, true))
            {
                if (!cloneError.isEmpty())
                {
                    if (const auto step = structure->m_AnalysisStep.find(dynamicStepId);
                        step != structure->m_AnalysisStep.cend() && step->second)
                        step->second->SetLastFailureReason(cloneError);
                    qDebug().noquote() << QStringLiteral("Error:") << cloneError;
                }
                return finalize(false);
            }
        }
    }

    return finalize(true);
}

bool ApplyGallopingInitialVerticalPerturbation(StructureData& structure, AnalysisStep& step)
{
    const double amplitude = step.m_GallopingInitialVerticalPerturbationMeters;
    const auto fail = [&step](const QString& reason)
    {
        step.SetLastFailureReason(reason);
        qWarning().noquote() << QStringLiteral("Error:") << reason;
        return false;
    };
    const int waves = step.m_GallopingInitialVerticalPerturbationWaves;
    if (waves != 1 && waves != 2)
        return fail(QStringLiteral("竖向初始扰动的空间半波数必须为 1 或 2。"));
    if (amplitude == 0.0)
        return true;
    if (!std::isfinite(amplitude) || !step.m_EnableGalloping || step.m_Type != EnumKeyword::StepType::DYNAMIC ||
        step.m_InitialStaticStepId <= 0)
        return fail(QStringLiteral("竖向初始扰动只适用于继承静力平衡的舞动动力步，且幅值必须有限。"));

    std::map<int, Eigen::Vector3d> conductorPositions;
    std::set<int> affectedNodeIds;
    for (const auto& [elementId, element] : structure.m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad())
            continue;
        if (element->m_pNode.size() != 2)
            return fail(QStringLiteral("气动导线单元 %1 不是双节点单元，无法定义跨内扰动。").arg(elementId));
        for (const auto& weakNode : element->m_pNode)
        {
            const auto node = weakNode.lock();
            if (!node)
                return fail(QStringLiteral("气动导线单元 %1 的端节点缺失。").arg(elementId));
            const Eigen::Vector3d position(node->m_X, node->m_Y, node->m_Z);
            if (!position.allFinite() || node->m_Displacement.size() < 3)
                return fail(QStringLiteral("气动导线节点 %1 的坐标或平移自由度无效。").arg(node->m_Id));
            conductorPositions.emplace(node->m_Id, position);
            affectedNodeIds.insert(node->m_Id);
        }
    }
    if (conductorPositions.size() < 3)
        return fail(QStringLiteral("气动导线跨内节点不足，无法施加半波初始扰动。"));

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& [nodeId, position] : conductorPositions)
        center += position;
    center /= static_cast<double>(conductorPositions.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& [nodeId, position] : conductorPositions)
    {
        const Eigen::Vector3d offset = position - center;
        covariance.noalias() += offset * offset.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covariance);
    if (eigenSolver.info() != Eigen::Success || eigenSolver.eigenvalues()[2] <= 1.0e-12)
        return fail(QStringLiteral("气动导线跨向几何退化，无法施加半波初始扰动。"));
    Eigen::Vector3d spanAxis = eigenSolver.eigenvectors().col(2);
    // 第一半波对跨轴翻转不敏感，第二半波会变号；规范 PCA 特征向量的符号。
    Eigen::Index dominant = 0;
    spanAxis.cwiseAbs().maxCoeff(&dominant);
    if (spanAxis[dominant] < 0.0)
        spanAxis = -spanAxis;
    double minimumProjection = std::numeric_limits<double>::infinity();
    double maximumProjection = -std::numeric_limits<double>::infinity();
    for (const auto& [nodeId, position] : conductorPositions)
    {
        const double projection = spanAxis.dot(position);
        minimumProjection = std::min(minimumProjection, projection);
        maximumProjection = std::max(maximumProjection, projection);
    }
    const double span = maximumProjection - minimumProjection;
    if (!std::isfinite(span) || span <= 1.0e-9)
        return fail(QStringLiteral("气动导线跨度无效，无法施加半波初始扰动。"));

    // 间隔棒和 MPC 相连的节点使用相同半波相位，避免只移动导线端节点。
    bool changed = true;
    while (changed)
    {
        changed = false;
        for (const auto& [elementId, element] : structure.m_Elements)
        {
            if (!element || (element->m_Role != ElementRole::IntraPhaseSpacer &&
                             element->m_Role != ElementRole::InterPhaseSpacer))
                continue;
            bool connected = false;
            for (const auto& weakNode : element->m_pNode)
            {
                const auto node = weakNode.lock();
                connected |= node && affectedNodeIds.contains(node->m_Id);
            }
            if (!connected)
                continue;
            for (const auto& weakNode : element->m_pNode)
            {
                const auto node = weakNode.lock();
                if (!node)
                    return fail(QStringLiteral("间隔棒单元 %1 的端节点缺失。").arg(elementId));
                changed |= affectedNodeIds.insert(node->m_Id).second;
            }
        }
        for (const auto& [constraintId, mpc] : structure.m_MPCConstraints)
        {
            if (!mpc || !step.IsStepScopedDataActive(mpc->m_StepId))
                continue;
            const std::vector<int> ids = mpc->GetNodeIds();
            const bool connected = std::any_of(ids.cbegin(), ids.cend(), [&affectedNodeIds](int id)
                                               { return affectedNodeIds.contains(id); });
            if (!connected)
                continue;
            for (int id : ids)
                changed |= affectedNodeIds.insert(id).second;
        }
    }

    Eigen::Vector3d modelUp = Eigen::Vector3d::UnitZ();
    for (const auto& [loadId, load] : structure.m_Load)
    {
        const auto gravity = std::dynamic_pointer_cast<Force_Gravity>(load);
        if (!gravity || !step.IsStepScopedDataActive(gravity->m_StepId))
            continue;
        const int component = static_cast<int>(gravity->m_Direction);
        if (component >= 0 && component < 3 && std::isfinite(gravity->m_g) && std::abs(gravity->m_g) > 1.0e-12)
        {
            modelUp = -std::copysign(1.0, gravity->m_g) * Eigen::Vector3d::Unit(component);
            break;
        }
    }

    std::map<int, double> increments;
    double maximumIncrement = 0.0;
    for (int nodeId : affectedNodeIds)
    {
        const auto found = structure.m_Nodes.find(nodeId);
        if (found == structure.m_Nodes.cend() || !found->second)
            return fail(QStringLiteral("半波扰动关联节点 %1 缺失。").arg(nodeId));
        const auto& node = found->second;
        const Eigen::Vector3d position(node->m_X, node->m_Y, node->m_Z);
        if (!position.allFinite() || node->m_Displacement.size() < 3)
            return fail(QStringLiteral("半波扰动关联节点 %1 的坐标或平移自由度无效。").arg(nodeId));
        const double fraction = std::clamp((spanAxis.dot(position) - minimumProjection) / span, 0.0, 1.0);
        // PCA 的跨轴可能因垂度产生极微小倾斜。端点在跨长的 1e-8 内
        // 视为同一支座截面，避免给固定自由度施加舍入级的假位移。
        const double increment = fraction <= 1.0e-8 || fraction >= 1.0 - 1.0e-8
                                     ? 0.0
                                     : amplitude * std::sin(waves * std::numbers::pi * fraction);
        increments.emplace(nodeId, increment);
        maximumIncrement = std::max(maximumIncrement, std::abs(increment));
    }
    if (maximumIncrement <= 1.0e-14)
        return fail(QStringLiteral("气动导线没有可扰动的跨内节点。"));

    const double constraintTolerance = std::max(1.0e-12, std::abs(amplitude) * 1.0e-9);
    for (const auto& [constraintId, constraint] : structure.m_Constraint)
    {
        if (!constraint || !step.IsStepScopedDataActive(constraint->m_StepId))
            continue;
        const auto node = constraint->m_pNode.lock();
        if (!node)
            continue;
        const int component = static_cast<int>(constraint->m_Direction);
        if (component >= 0 && component < 3 &&
            std::abs(increments[node->m_Id] * modelUp[component]) > constraintTolerance)
            return fail(QStringLiteral("节点 %1 的固定自由度与竖向半波扰动冲突。").arg(node->m_Id));
    }
    for (const auto& [constraintId, mpc] : structure.m_MPCConstraints)
    {
        if (!mpc || !step.IsStepScopedDataActive(mpc->m_StepId))
            continue;
        const std::vector<int> ids = mpc->GetNodeIds();
        if (ids.empty())
            continue;
        const double firstIncrement = increments[ids.front()];
        for (int nodeId : ids)
        {
            if (std::abs(increments[nodeId] - firstIncrement) > constraintTolerance)
                return fail(QStringLiteral("MPC %1 关联节点的扰动不一致。").arg(constraintId));
        }
    }
    for (const auto& [nodeId, increment] : increments)
    {
        auto& displacement = structure.m_Nodes.at(nodeId)->m_Displacement;
        for (int component = 0; component < 3; ++component)
            displacement[component] += increment * modelUp[component];
    }
    qInfo().noquote() << QStringLiteral("舞动初始竖向扰动：分析步 %1，幅值 %2 m，空间半波数 %3，跨度 %4 m，关联节点 %5。")
                             .arg(step.m_Id)
                             .arg(amplitude, 0, 'g', 8)
                             .arg(waves)
                             .arg(span, 0, 'g', 8)
                             .arg(static_cast<qulonglong>(increments.size()));
    return true;
}

bool AnalysisRunner::RunStepDirect(int stepId, bool persistHdf5, bool inheritedStaticState)
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
    if (inheritedStaticState && step->m_GallopingInitialVerticalPerturbationMeters != 0.0)
        step->SetInitialDynamicStateCallback(
            [structure, step](QString& error)
            {
                if (ApplyGallopingInitialVerticalPerturbation(*structure, *step))
                    return true;
                error = step->LastFailureReason();
                return false;
            });
    if (m_executionThreadBudget > 0)
    {
        const int assemblyThreads =
            std::min(SolverNameSpace::AssemblySettings::ThreadCount(), m_executionThreadBudget);
        step->SetAssemblyThreadCountOverride(assemblyThreads);
        step->SetPardisoThreadCountOverride(m_executionThreadBudget);
        Logger::Instance().Diagnostic(QStringLiteral("执行线程策略：CPU 预算 %1，装配上限 %2，PARDISO 上限 %3")
                                          .arg(m_executionThreadBudget)
                                          .arg(assemblyThreads)
                                          .arg(m_executionThreadBudget));
    }
    step->SetRuntimeCallbacks(m_progressCallback, m_cancelCallback);
    const bool succeeded = step->Solve(persistHdf5);
    step->ClearInitialDynamicStateCallback();
    step->ClearRuntimeCallbacks();
    m_wasCancelled = !succeeded && m_cancelCallback && m_cancelCallback();
    return succeeded;
}
