#include "Controllers/SolveTaskController.h"

#include "Application/ApplicationPaths.h"
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include "Export/Hdf5ResultReader.h"
#include "Export/ResultOutputSettings.h"
#include "Solver/Assembly/AssemblySettings.h"
#include "Solver/Factory/SolverFactory.h"
#include "Solver/Linear/Gpu/GpuSettings.h"
#include "Solver/Linear/LinearSolverSettings.h"
#include "Solver/Workflow/AnalysisSolve.h"
#include "Utility/Logger/Logger.h"
#include "Utility/AnalysisDiagnostics.h"

#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QProcess>
#include <QRegularExpression>
#include <QStringList>
#include <QTemporaryDir>
#include <QTextStream>
#include <QThread>

#include <algorithm>
#include <chrono>
#include <exception>
#include <set>

SolveTaskController::SolveTaskController(QObject* parent)
    : QObject(parent)
{
    m_threadPool.setMaxThreadCount(defaultThreadCount());
    m_threadPool.setExpiryTimeout(30000);
}

SolveTaskController::~SolveTaskController()
{
    cancelAll();
    m_threadPool.waitForDone();
}

int SolveTaskController::submit(const std::shared_ptr<StructureData>& model, const QString& sourceFile)
{
    const int taskId = prepare(model, sourceFile, 0);
    return taskId >= 0 && start(taskId) ? taskId : -1;
}

int SolveTaskController::prepare(const std::shared_ptr<StructureData>& model, const QString& sourceFile,
                                 int analysisStepId)
{
    return prepareTask(model, sourceFile, analysisStepId, false);
}

int SolveTaskController::prepareTask(const std::shared_ptr<StructureData>& model, const QString& sourceFile,
                                     int analysisStepId, bool referenceGravityEquilibrium)
{
    if (model)
        model->EnsureDefaultAnalysisConfiguration();
    if (!model || sourceFile.trimmed().isEmpty() || model->m_AnalysisStep.empty() || analysisStepId < 0)
        return -1;
    if (analysisStepId > 0 && model->m_AnalysisStep.find(analysisStepId) == model->m_AnalysisStep.end())
        return -1;

    for (const auto& [candidateId, selected] : model->m_AnalysisStep)
    {
        if (analysisStepId > 0 && candidateId != analysisStepId)
            continue;
        if (!selected)
            return -1;
        QString validationError;
        if (selected->m_Type != EnumKeyword::StepType::STATIC && selected->m_Type != EnumKeyword::StepType::DYNAMIC)
            validationError = QStringLiteral("分析步类型不受支持，无法创建求解任务。");
        else if (referenceGravityEquilibrium && selected->m_Type != EnumKeyword::StepType::STATIC)
            validationError = QStringLiteral("初态准备必须引用静力分析步。");
        if (selected->m_Type == EnumKeyword::StepType::DYNAMIC)
        {
            const auto policy =
                SolverNameSpace::SolverFactory::GetDynamicExecutionPolicy(selected->m_DynamicSolverType);
            if (!policy.implemented)
                validationError = QStringLiteral("所选动力算法尚未实现，无法创建求解任务。");
            else if (policy.requiresStaticState && selected->m_InitialStaticStepId <= 0)
                validationError = QStringLiteral("所选动力算法需要关联前置静力步。");
            else if (policy.requiresGalloping && !selected->m_EnableGalloping)
                validationError = QStringLiteral("所选动力算法需要启用覆冰舞动。");
        }
        if (selected->m_InitialStaticStepId > 0)
        {
            const auto initial = model->m_AnalysisStep.find(selected->m_InitialStaticStepId);
            if (selected->m_Type != EnumKeyword::StepType::DYNAMIC || initial == model->m_AnalysisStep.cend() ||
                !initial->second || initial->second->m_Type != EnumKeyword::StepType::STATIC ||
                initial->first >= candidateId || initial->second->m_InitialStaticStepId > 0)
                validationError = QStringLiteral("动力步必须关联编号更早且有效的静力步，静力步自身不能引用初态。");
        }
        if (!validationError.isEmpty())
        {
            selected->SetLastFailureReason(validationError);
            return -1;
        }
    }

    const QString absoluteSource = QFileInfo(sourceFile).absoluteFilePath();
    std::shared_ptr<TaskContext> task;
    for (const auto& existing : m_tasks)
    {
        if (existing && existing->info.sourceFile == absoluteSource &&
            existing->info.analysisStepId == analysisStepId &&
            existing->referenceGravityEquilibrium == referenceGravityEquilibrium)
        {
            task = existing;
            break;
        }
    }
    const bool isNew = !task;
    if (!task)
    {
        task = std::make_shared<TaskContext>();
        task->info.id = m_nextTaskId++;
    }
    else if (task->info.status == Status::Queued || task->info.status == Status::Running ||
             task->info.status == Status::Cancelling)
    {
        return task->info.id;
    }
    QString cloneError;
    auto modelTemplate = model->CloneForAnalysis(&cloneError);
    if (!modelTemplate)
        return -1;
    if (analysisStepId > 0)
    {
        std::set<int> retainedStepIds{analysisStepId};
        const auto selectedCloneStep = modelTemplate->m_AnalysisStep.find(analysisStepId);
        if (selectedCloneStep != modelTemplate->m_AnalysisStep.cend() && selectedCloneStep->second &&
            selectedCloneStep->second->m_InitialStaticStepId > 0)
        {
            retainedStepIds.insert(selectedCloneStep->second->m_InitialStaticStepId);
        }
        if (selectedCloneStep != modelTemplate->m_AnalysisStep.cend() && selectedCloneStep->second &&
            selectedCloneStep->second->m_Type == EnumKeyword::StepType::STATIC)
        {
            for (const auto& [candidateId, candidate] : modelTemplate->m_AnalysisStep)
            {
                if (candidate && candidate->m_Type == EnumKeyword::StepType::DYNAMIC &&
                    candidate->m_InitialStaticStepId == analysisStepId)
                    retainedStepIds.insert(candidateId);
            }
        }
        for (auto it = modelTemplate->m_AnalysisStep.begin(); it != modelTemplate->m_AnalysisStep.end();)
        {
            if (retainedStepIds.find(it->first) == retainedStepIds.cend())
                it = modelTemplate->m_AnalysisStep.erase(it);
            else
                ++it;
        }
    }

    task->cancelRequested.store(false, std::memory_order_relaxed);
    task->lastProgressReportMs.store(0, std::memory_order_relaxed);
    task->workerScheduled = false;
    task->solvedModel.reset();
    task->info.sourceFile = absoluteSource;
    task->solvedRegionStates.clear();
    task->info.analysisStepId = analysisStepId;
    task->info.initialStaticTaskId = -1;
    task->info.waitingForInitialStatic = false;
    task->info.initialStatePreparation = referenceGravityEquilibrium;
    task->referenceGravityEquilibrium = referenceGravityEquilibrium;
    const auto selectedStep =
        analysisStepId > 0 ? model->m_AnalysisStep.find(analysisStepId) : model->m_AnalysisStep.end();
    const int requiredStaticStepId = selectedStep != model->m_AnalysisStep.end() && selectedStep->second
                                         ? selectedStep->second->m_InitialStaticStepId
                                         : 0;
    const QString selectedStepName = selectedStep != model->m_AnalysisStep.end() && selectedStep->second &&
                                             !selectedStep->second->m_Name.trimmed().isEmpty()
                                         ? selectedStep->second->m_Name.trimmed()
                                         : QStringLiteral("Step-%1").arg(analysisStepId);
    task->info.name = analysisStepId > 0 ? QStringLiteral("%1 · %2 · 算例 %3")
                                               .arg(QFileInfo(sourceFile).completeBaseName())
                                               .arg(selectedStepName)
                                               .arg(task->info.id)
                                         : QStringLiteral("%1 · 全部分析步 · 算例 %2")
                                               .arg(QFileInfo(sourceFile).completeBaseName())
                                               .arg(task->info.id);
    if (referenceGravityEquilibrium)
    {
        const auto staticStep = modelTemplate->m_AnalysisStep.at(analysisStepId);
        staticStep->m_EnableGalloping = false;
        staticStep->SetReferenceGravityEquilibrium(true);
        task->info.name += QStringLiteral("（初态准备：无风静力平衡）");
    }
    task->info.status = Status::Ready;
    task->info.progress = 0.0;
    task->info.elapsedMs = 0;
    task->info.solverElapsedMs = 0;
    task->info.workerProcessId = 0;
    task->info.resultLoadRecorded = false;
    task->info.hasUsableResult = false;
    task->info.partialResult = false;
    task->info.resultFrameCount = 0;
    task->info.resultEndTime = 0.0;
    task->info.message = QStringLiteral("参数已就绪，点击算例按钮开始计算");
    task->info.diagnostics.clear();

    const QFileInfo sourceInfo(task->info.sourceFile);
    // H5 导入模型可能没有原始源文件名，此时按实际导入文件归属结果目录。
    const QString resultSource = model->m_OutputControl.m_SourceModelName.trimmed().isEmpty()
                                     ? task->info.sourceFile
                                     : model->m_OutputControl.m_SourceModelName;
    const QDir outputDir(ApplicationPaths::hdf5ResultDirectory(resultSource, model->m_OutputControl.m_Hdf5FileName));
    task->info.outputFile = outputDir.filePath(
        QStringLiteral("%1_case_%2.h5").arg(sourceInfo.completeBaseName()).arg(task->info.id, 3, 10, QLatin1Char('0')));
    modelTemplate->m_OutputControl.m_Hdf5FileName = task->info.outputFile;
    task->modelTemplate = std::move(modelTemplate);

    m_tasks.insert(task->info.id, task);
    if (isNew)
        emit taskAdded(task->info.id);
    else
        emit taskUpdated(task->info.id);

    // 导线模型创建时会先准备静力任务，动力步通常稍后由分析编辑器加入。
    // 此时刷新静力模板，保证继承平衡状态前已经包含新的动力步定义。
    if (requiredStaticStepId > 0 && requiredStaticStepId != analysisStepId)
    {
        const bool reference =
            SolverNameSpace::SolverFactory::GetDynamicExecutionPolicy(selectedStep->second->m_DynamicSolverType)
                .initialStaticMode == SolverNameSpace::InitialStaticMode::ReferenceWindless;
        task->info.initialStaticTaskId = prepareTask(model, sourceFile, requiredStaticStepId, reference);
        emit taskUpdated(task->info.id);
    }
    return task->info.id;
}

void SolveTaskController::removeUnavailablePreparedTasks(const std::shared_ptr<StructureData>& model,
                                                         const QString& sourceFile)
{
    if (!model || sourceFile.trimmed().isEmpty())
        return;
    const QString absoluteSource = QFileInfo(sourceFile).absoluteFilePath();
    for (auto it = m_tasks.begin(); it != m_tasks.end();)
    {
        const auto& task = it.value();
        const bool active = task && (task->info.status == Status::Queued || task->info.status == Status::Running ||
                                     task->info.status == Status::Cancelling);
        const bool missingStep = task && task->info.sourceFile == absoluteSource && task->info.analysisStepId > 0 &&
                                 model->m_AnalysisStep.find(task->info.analysisStepId) == model->m_AnalysisStep.end();
        const bool unusedReferenceEquilibrium =
            task && task->info.sourceFile == absoluteSource && task->referenceGravityEquilibrium &&
            std::none_of(
                model->m_AnalysisStep.cbegin(), model->m_AnalysisStep.cend(),
                [&task](const auto& entry)
                {
                    return entry.second && entry.second->m_Type == EnumKeyword::StepType::DYNAMIC &&
                           SolverNameSpace::SolverFactory::GetDynamicExecutionPolicy(entry.second->m_DynamicSolverType)
                                   .initialStaticMode == SolverNameSpace::InitialStaticMode::ReferenceWindless &&
                           entry.second->m_InitialStaticStepId == task->info.analysisStepId;
                });
        if ((missingStep || unusedReferenceEquilibrium) && !active)
        {
            m_restartBatch.removeAll(task->info.id);
            it = m_tasks.erase(it);
        }
        else
            ++it;
    }
}

bool SolveTaskController::start(int taskId)
{
    if (!m_restartBatch.isEmpty() || m_stagingRestartBatch)
        return false;
    const auto task = m_tasks.value(taskId);
    if (!task || (task->info.status != Status::Ready && task->info.status != Status::Completed &&
                  task->info.status != Status::Failed && task->info.status != Status::Cancelled))
        return false;
    task->totalTimer.start();
    task->cancelRequested.store(false, std::memory_order_relaxed);
    task->lastProgressReportMs.store(0, std::memory_order_relaxed);
    task->info.status = Status::Queued;
    task->info.waitingForInitialStatic = false;
    task->info.progress = 0.0;
    task->info.elapsedMs = 0;
    task->info.solverElapsedMs = 0;
    task->info.workerProcessId = 0;
    task->info.resultLoadRecorded = false;
    task->info.hasUsableResult = false;
    task->info.partialResult = false;
    task->info.resultFrameCount = 0;
    task->info.resultEndTime = 0.0;
    task->workerScheduled = false;
    task->solvedModel.reset();
    task->info.message = QStringLiteral("等待计算线程");
    task->info.diagnostics.clear();
    task->solvedRegionStates.clear();
    const QFileInfo previousOutput(task->info.outputFile);
    task->previousOutputModifiedMs = previousOutput.exists() ? previousOutput.lastModified().toMSecsSinceEpoch() : -1;
    task->previousOutputSize = previousOutput.exists() ? previousOutput.size() : -1;
    emit taskUpdated(taskId);

    const auto dependency = dependencyTask(task);
    if (initialStaticStepId(task) > 0 && !dependency)
    {
        finishTask(taskId, Status::Failed, QStringLiteral("前置静力任务不可用，请重新准备分析步"));
        return true;
    }
    if (dependency)
    {
        if (dependency->info.status == Status::Completed && dependency->solvedModel)
            launchTask(task);
        else
        {
            if (dependency->info.status == Status::Completed)
            {
                finishTask(taskId, Status::Failed, QStringLiteral("前置静力步未生成可用的平衡状态，动力步未启动"));
                return true;
            }
            task->info.waitingForInitialStatic = true;
            task->info.message = QStringLiteral("等待前置静力步 %1 完成").arg(dependency->info.analysisStepId);
            emit taskUpdated(taskId);
            if (dependency->info.status == Status::Ready || dependency->info.status == Status::Failed ||
                dependency->info.status == Status::Cancelled)
                start(dependency->info.id);
        }
    }
    else
        launchTask(task);
    return true;
}

void SolveTaskController::recordResultLoaded(int taskId)
{
    const auto task = m_tasks.value(taskId);
    if (!task || !task->info.hasUsableResult || task->info.resultLoadRecorded || !task->totalTimer.isValid())
        return;
    if (task->info.status != Status::Completed && task->info.status != Status::Failed &&
        task->info.status != Status::Cancelled)
        return;
    task->info.elapsedMs = task->totalTimer.elapsed();
    task->info.resultLoadRecorded = true;
    emit taskUpdated(taskId);
}

int SolveTaskController::initialStaticStepId(const std::shared_ptr<TaskContext>& task) const
{
    if (!task || !task->modelTemplate || task->info.analysisStepId <= 0)
        return 0;
    const auto stepIt = task->modelTemplate->m_AnalysisStep.find(task->info.analysisStepId);
    return stepIt != task->modelTemplate->m_AnalysisStep.end() && stepIt->second ? stepIt->second->m_InitialStaticStepId
                                                                                 : 0;
}

std::shared_ptr<SolveTaskController::TaskContext> SolveTaskController::dependencyTask(
    const std::shared_ptr<TaskContext>& task) const
{
    return task ? m_tasks.value(task->info.initialStaticTaskId) : nullptr;
}

void SolveTaskController::launchTask(const std::shared_ptr<TaskContext>& task)
{
    if (!task || task->workerScheduled || task->info.status != Status::Queued)
        return;
    const auto dependency = dependencyTask(task);
    const auto initialStaticModel = dependency ? dependency->solvedModel : nullptr;
    const auto initialRegionStates =
        dependency ? dependency->solvedRegionStates : std::map<int, std::shared_ptr<StructureData>>{};
    bool publishInitialStaticResults = true;
    if (task->modelTemplate && task->info.analysisStepId > 0)
    {
        const auto selected = task->modelTemplate->m_AnalysisStep.find(task->info.analysisStepId);
        if (selected != task->modelTemplate->m_AnalysisStep.cend() && selected->second &&
            selected->second->m_Type == EnumKeyword::StepType::DYNAMIC)
        {
            publishInitialStaticResults =
                SolverNameSpace::SolverFactory::GetDynamicExecutionPolicy(selected->second->m_DynamicSolverType)
                    .publishInitialStaticResults;
        }
    }
    task->info.waitingForInitialStatic = false;
    task->executionThreadBudget = executionThreadBudgetPerTask();
    task->assemblyThreadLimit = std::min(SolverNameSpace::AssemblySettings::ThreadCount(), task->executionThreadBudget);
    task->isolatedProcess = m_isolatedProcessEnabled && task->info.analysisStepId == 0;
    task->workerScheduled = true;
    m_threadPool.start(
        [this, task, initialStaticModel, initialRegionStates, publishInitialStaticResults]()
        {
            runTask(task, initialStaticModel, initialRegionStates, publishInitialStaticResults);
        });
    emit taskUpdated(task->info.id);
}

void SolveTaskController::tryStartRestartBatch()
{
    if (m_restartBatch.isEmpty() || m_stagingRestartBatch)
        return;
    for (const auto& task : m_tasks)
    {
        if (task && (task->workerScheduled || task->info.status == Status::Queued ||
                     task->info.status == Status::Running || task->info.status == Status::Cancelling))
            return;
    }

    const QList<int> ids = m_restartBatch;
    m_restartBatch.clear();
    for (const int id : ids)
    {
        const auto task = m_tasks.value(id);
        if (!task)
            continue;
        task->solvedModel.reset();
        task->solvedRegionStates.clear();
        task->restartRequested = false;
        task->info.status = Status::Ready;
        task->info.waitingForInitialStatic = false;
        task->info.progress = 0.0;
    }
    // 先将全部初态依赖失效并启动，再提交动力，创建 ID 不参与求解顺序。
    for (const int id : ids)
    {
        const auto task = m_tasks.value(id);
        if (task && task->info.initialStaticTaskId < 0)
            start(id);
    }
    for (const int id : ids)
    {
        const auto task = m_tasks.value(id);
        if (task && task->info.initialStaticTaskId >= 0)
            start(id);
    }
}

void SolveTaskController::releaseDependentTasks(const std::shared_ptr<TaskContext>& dependency, Status status)
{
    if (!dependency || !m_restartBatch.isEmpty() || m_stagingRestartBatch)
        return;
    const QList<int> ids = taskIds();
    for (const int id : ids)
    {
        const auto candidate = m_tasks.value(id);
        if (!candidate || candidate->info.status != Status::Queued || candidate->workerScheduled ||
            dependencyTask(candidate) != dependency)
            continue;
        if (status == Status::Completed && dependency->solvedModel)
        {
            candidate->info.message = QStringLiteral("前置静力步已完成，开始动力分析");
            emit taskUpdated(candidate->info.id);
            launchTask(candidate);
        }
        else
        {
            const bool cancelled = status == Status::Cancelled;
            finishTask(candidate->info.id, cancelled ? Status::Cancelled : Status::Failed,
                       cancelled ? QStringLiteral("前置静力步已停止，动力步未启动")
                                 : QStringLiteral("前置静力步求解失败，动力步未启动"));
        }
    }
}

int SolveTaskController::startAllReady()
{
    if (!m_restartBatch.isEmpty() || m_stagingRestartBatch)
        return 0;
    QList<int> readyIds;
    for (const int id : analysisTaskIds())
    {
        const auto task = m_tasks.value(id);
        if (task && task->info.status == Status::Ready)
            readyIds.append(id);
    }
    for (const int id : readyIds)
        start(id);
    int startedCount = 0;
    for (const int id : readyIds)
    {
        const auto task = m_tasks.value(id);
        if (task && task->info.status != Status::Ready)
            ++startedCount;
    }
    return startedCount;
}

int SolveTaskController::restartAll()
{
    const QList<int> analysisIds = analysisTaskIds();
    if (analysisIds.isEmpty())
        return 0;
    m_stagingRestartBatch = true;
    m_restartBatch = analysisIds;
    for (const int id : analysisIds)
    {
        const auto task = m_tasks.value(id);
        const auto dependency = dependencyTask(task);
        if (dependency && !m_restartBatch.contains(dependency->info.id))
            m_restartBatch.append(dependency->info.id);
    }
    const QList<int> allIds = taskIds();
    for (const int id : allIds)
    {
        const auto task = m_tasks.value(id);
        if (task)
            task->restartRequested = false;
    }
    for (const int id : allIds)
        cancel(id);
    m_stagingRestartBatch = false;
    tryStartRestartBatch();
    return analysisIds.size();
}

bool SolveTaskController::cancel(int taskId)
{
    const auto task = m_tasks.value(taskId);
    if (!task || (task->info.status != Status::Queued && task->info.status != Status::Running))
        return false;

    if (task->info.status == Status::Queued && !task->workerScheduled)
    {
        task->cancelRequested.store(true, std::memory_order_relaxed);
        finishTask(taskId, Status::Cancelled, QStringLiteral("已取消等待前置静力步的动力任务"));
        return true;
    }

    task->cancelRequested.store(true, std::memory_order_relaxed);
    task->info.status = Status::Cancelling;
    task->info.message = QStringLiteral("正在停止（将在当前增量结束后退出）");
    emit taskUpdated(taskId);
    return true;
}

bool SolveTaskController::restart(int taskId)
{
    return start(taskId);
}

void SolveTaskController::cancelAll()
{
    m_restartBatch.clear();
    for (const auto& task : m_tasks)
    {
        if (task)
            task->restartRequested = false;
    }
    const QList<int> ids = taskIds();
    for (const int id : ids)
        cancel(id);
}

QList<int> SolveTaskController::taskIds() const
{
    auto ids = m_tasks.keys();
    std::sort(ids.begin(), ids.end());
    return ids;
}

QList<int> SolveTaskController::analysisTaskIds() const
{
    QList<int> ids;
    for (auto it = m_tasks.cbegin(); it != m_tasks.cend(); ++it)
    {
        if (it.value() && !it.value()->referenceGravityEquilibrium)
            ids.append(it.key());
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

SolveTaskController::TaskInfo SolveTaskController::taskInfo(int taskId) const
{
    const auto task = m_tasks.value(taskId);
    return task ? task->info : TaskInfo{};
}

SolveTaskController::TaskInfo SolveTaskController::displayTaskInfo(int taskId) const
{
    const auto task = m_tasks.value(taskId);
    if (!task)
        return TaskInfo{};
    if (task->info.status != Status::Ready || task->info.initialStatePreparation || task->info.analysisStepId <= 0)
        return task->info;

    for (const auto& candidate : m_tasks)
    {
        if (!candidate || candidate->info.sourceFile != task->info.sourceFile)
            continue;
        const auto preparation = dependencyTask(candidate);
        if (preparation && preparation->referenceGravityEquilibrium &&
            preparation->info.analysisStepId == task->info.analysisStepId && preparation->info.status != Status::Ready)
            return preparation->info;
    }
    return task->info;
}

int SolveTaskController::runningTaskCount() const
{
    int count = 0;
    for (const auto& task : m_tasks)
    {
        if (task->info.status == Status::Queued || task->info.status == Status::Running ||
            task->info.status == Status::Cancelling)
            ++count;
    }
    return count;
}

int SolveTaskController::maximumThreadCount() const
{
    return m_threadPool.maxThreadCount();
}

int SolveTaskController::executionThreadBudgetPerTask() const
{
    const int available = availableThreadCount();
    const int reserved = std::max(1, (available + 3) / 4);
    const int usable = std::max(1, available - reserved);
    return std::max(1, usable / maximumThreadCount());
}

void SolveTaskController::setMaximumThreadCount(int count)
{
    // 降低线程上限不会终止正在运行的任务；已有任务正常结束，后续调度采用新上限。
    m_threadPool.setMaxThreadCount(std::clamp(count, MinimumThreadCount, maximumAllowedThreadCount()));
}

void SolveTaskController::setIsolatedProcessEnabled(bool enabled)
{
    m_isolatedProcessEnabled = enabled;
}

int SolveTaskController::availableThreadCount()
{
    return std::max(1, QThread::idealThreadCount());
}

int SolveTaskController::maximumAllowedThreadCount()
{
    const int available = availableThreadCount();
    // 为界面、渲染、I/O 和系统预留约四分之一逻辑处理器，且全局最多使用十二线程。
    const int reserved = std::max(1, (available + 3) / 4);
    return std::clamp(available - reserved, MinimumThreadCount, MaximumThreadCount);
}

int SolveTaskController::defaultThreadCount()
{
    return std::min(DefaultThreadCount, maximumAllowedThreadCount());
}

QString SolveTaskController::statusText(Status status)
{
    switch (status)
    {
        case Status::Ready:
            return QStringLiteral("待运行");
        case Status::Queued:
            return QStringLiteral("排队中");
        case Status::Running:
            return QStringLiteral("计算中");
        case Status::Completed:
            return QStringLiteral("成功");
        case Status::Failed:
            return QStringLiteral("失败");
        case Status::Cancelling:
            return QStringLiteral("停止中");
        case Status::Cancelled:
            return QStringLiteral("已停止");
    }
    return QStringLiteral("未知");
}

void SolveTaskController::runTask(const std::shared_ptr<TaskContext>& task,
                                  const std::shared_ptr<StructureData>& initialStaticModel,
                                  const std::map<int, std::shared_ptr<StructureData>>& initialRegionStates,
                                  bool publishInitialStaticResults)
{
    if (task->isolatedProcess)
    {
        runIsolatedTask(task);
        return;
    }
    if (task->cancelRequested.load(std::memory_order_relaxed))
    {
        QMetaObject::invokeMethod(
            this,
            [this, id = task->info.id]()
            {
                finishTask(id, Status::Cancelled, QStringLiteral("任务在启动前已停止"));
            },
            Qt::QueuedConnection);
        return;
    }

    QMetaObject::invokeMethod(
        this,
        [this, id = task->info.id]()
        {
            const auto current = m_tasks.value(id);
            if (!current || current->info.status == Status::Cancelling)
                return;
            current->info.status = Status::Running;
            current->info.message = QStringLiteral("正在创建独立计算模型 · CPU预算 %1 · 装配上限 %2")
                                        .arg(current->executionThreadBudget)
                                        .arg(current->assemblyThreadLimit);
            emit taskUpdated(id);
        },
        Qt::QueuedConnection);

    Status finalStatus = Status::Failed;
    QString finalMessage;
    QString stageReport;
    qint64 solverElapsedMs = -1;
    QElapsedTimer solverTimer;
    SolverNameSpace::GpuSettings::ResetThreadReport();
    try
    {
        QString cloneError;
        const bool startsFromStaticState = static_cast<bool>(initialStaticModel);
        auto structure = task->modelTemplate ? task->modelTemplate->CloneForAnalysis(&cloneError) : nullptr;
        if (!structure)
            throw std::runtime_error(cloneError.isEmpty() ? "无法创建独立计算模型" : cloneError.toUtf8().constData());

        if (startsFromStaticState && initialRegionStates.empty())
        {
            auto equilibrium = initialStaticModel->CloneForAnalysis(&cloneError);
            if (!equilibrium)
                throw std::runtime_error(cloneError.isEmpty() ? "无法复制前置静力状态"
                                                              : cloneError.toUtf8().constData());
            if (!equilibrium->CopyAnalysisBindingsFrom(*structure, &cloneError))
                throw std::runtime_error(cloneError.isEmpty() ? "无法复制当前分析荷载及约束"
                                                              : cloneError.toUtf8().constData());
            equilibrium->m_AnalysisStep = std::move(structure->m_AnalysisStep);
            equilibrium->m_OutputControl = structure->m_OutputControl;
            equilibrium->m_AeroManager = structure->m_AeroManager;
            structure = std::move(equilibrium);
        }
        if (startsFromStaticState && publishInitialStaticResults)
            structure->GetOutputter().MergeFramesFrom(initialStaticModel->GetOutputter());

        structure->m_OutputControl.m_Hdf5FileName = task->info.outputFile;

        AnalysisRunner runner;
        runner.SetStructure(structure);
        runner.SetMaximumRegionThreads(task->executionThreadBudget);
        runner.SetExecutionThreadBudget(task->executionThreadBudget);
        if (startsFromStaticState)
            runner.SetInitialRegionStates(initialRegionStates);
        runner.SetRuntimeCallbacks(
            [this, task, id = task->info.id](double progress, const QString& message)
            {
                const qint64 nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::steady_clock::now().time_since_epoch())
                                         .count();
                const qint64 previousMs = task->lastProgressReportMs.load(std::memory_order_relaxed);
                if (progress < 1.0 && previousMs > 0 && nowMs - previousMs < 33)
                    return;
                task->lastProgressReportMs.store(nowMs, std::memory_order_relaxed);
                QMetaObject::invokeMethod(
                    this,
                    [this, id, progress, message]()
                    {
                        reportProgress(id, progress, message);
                    },
                    Qt::QueuedConnection);
            },
            [task]()
            {
                return task->cancelRequested.load(std::memory_order_relaxed);
            });

        solverTimer.start();
        // 单步任务先复用已完成的前置静力状态，并将前置静力结果帧合并到当前分支 H5。
        // 这里只显式求解当前步，避免重复计算前置静力步。
        const bool succeeded = task->info.analysisStepId > 0
                                   ? (startsFromStaticState ? runner.RunStepFromCurrentState(task->info.analysisStepId)
                                                            : runner.RunStep(task->info.analysisStepId))
                                   : runner.RunAll();
        solverElapsedMs = solverTimer.elapsed();
        if (runner.WasCancelled() || task->cancelRequested.load(std::memory_order_relaxed))
        {
            finalStatus = Status::Cancelled;
            finalMessage = QStringLiteral("任务已停止");
        }
        else if (succeeded)
        {
            if (initialStaticStepId(task) <= 0)
            {
                task->solvedModel = structure;
                task->solvedRegionStates = runner.GetSolvedRegionStates();
            }
            finalStatus = Status::Completed;
            finalMessage = QStringLiteral("计算完成");
        }
        else
        {
            finalMessage = QStringLiteral("求解器返回失败，请查看日志和模型参数");
            if (task->info.analysisStepId > 0)
            {
                const auto failedStep = structure->m_AnalysisStep.find(task->info.analysisStepId);
                if (failedStep != structure->m_AnalysisStep.cend() && failedStep->second &&
                    !failedStep->second->LastFailureReason().isEmpty())
                    finalMessage = failedStep->second->LastFailureReason();
            }
            else
            {
                for (const auto& [stepId, step] : structure->m_AnalysisStep)
                {
                    if (!step || step->LastFailureReason().isEmpty())
                        continue;
                    finalMessage = QStringLiteral("分析步 %1：%2").arg(stepId).arg(step->LastFailureReason());
                    break;
                }
            }
        }
        stageReport = AnalysisDiagnostics::PerformanceReport(*structure, task->info.analysisStepId);
    }
    catch (const std::exception& exception)
    {
        if (solverTimer.isValid())
            solverElapsedMs = solverTimer.elapsed();
        finalMessage = QString::fromUtf8(exception.what());
    }
    catch (...)
    {
        if (solverTimer.isValid())
            solverElapsedMs = solverTimer.elapsed();
        finalMessage = QStringLiteral("计算过程中发生未知异常");
    }

    const QString gpuReport = SolverNameSpace::GpuSettings::ThreadReport();
    finalMessage = completionMessage(finalStatus, finalMessage, gpuReport);
    const QString diagnostics = completionDiagnostics(task->info.outputFile, solverElapsedMs, gpuReport, stageReport,
                                                      AnalysisDiagnostics::ProcessPeakMemoryMegabytes(), 0);

    QMetaObject::invokeMethod(
        this,
        [this, id = task->info.id, finalStatus, finalMessage, solverElapsedMs, diagnostics]()
        {
            finishTask(id, finalStatus, finalMessage, solverElapsedMs, diagnostics);
        },
        Qt::QueuedConnection);
}

void SolveTaskController::runIsolatedTask(const std::shared_ptr<TaskContext>& task)
{
    Status finalStatus = Status::Failed;
    QString finalMessage = QStringLiteral("独立计算进程未启动");
    QString diagnostics = completionDiagnostics(task->info.outputFile, -1, QString(), QString(), 0.0, 0);
    qint64 solverElapsedMs = -1;
    qint64 workerProcessId = 0;
    if (task->cancelRequested.load(std::memory_order_relaxed))
    {
        finalStatus = Status::Cancelled;
        finalMessage = QStringLiteral("任务在启动前已停止");
    }
    else
    {
        QMetaObject::invokeMethod(
            this,
            [this, id = task->info.id]()
            {
                const auto current = m_tasks.value(id);
                if (!current)
                    return;
                current->info.status = Status::Running;
                current->info.message = QStringLiteral("正在准备独立计算进程");
                emit taskUpdated(id);
            },
            Qt::QueuedConnection);

        QTemporaryDir temporaryDirectory;
        if (!temporaryDirectory.isValid() || !task->modelTemplate)
        {
            finalMessage = QStringLiteral("无法建立独立计算进程的模型快照");
        }
        else
        {
            const QString modelPath = temporaryDirectory.filePath(QStringLiteral("model.h5"));
            const QString cancelPath = temporaryDirectory.filePath(QStringLiteral("cancel"));
            QString aerodynamicSnapshotError;
            for (const auto& [stepId, step] : task->modelTemplate->m_AnalysisStep)
            {
                if (!step || !step->m_EnableGalloping)
                    continue;
                if (!step->PreloadGallopingDataForSnapshot(task->modelTemplate, &aerodynamicSnapshotError))
                {
                    if (aerodynamicSnapshotError.isEmpty())
                        aerodynamicSnapshotError = QStringLiteral("分析步 %1 的气动参数准备失败").arg(stepId);
                    break;
                }
            }
            Hdf5ModelIO modelIo;
            if (!aerodynamicSnapshotError.isEmpty())
            {
                finalMessage = aerodynamicSnapshotError;
            }
            else if (!modelIo.ExportModelHdf5(modelPath, task->modelTemplate.get(), task->info.sourceFile, true))
            {
                finalMessage = QStringLiteral("无法导出独立计算进程的模型快照：%1").arg(modelPath);
            }
            else
            {
                QStringList arguments{QStringLiteral("--solve-worker"),
                                      modelPath,
                                      task->info.outputFile,
                                      cancelPath,
                                      QString::number(task->executionThreadBudget),
                                      QString::number(task->assemblyThreadLimit),
                                      task->modelTemplate->m_OutputControl.m_SourceModelName,
                                      task->modelTemplate->m_OutputControl.m_StreamResult ? QStringLiteral("1")
                                                                                          : QStringLiteral("0")};
                if (SolverNameSpace::GpuSettings::IsEnabled())
                    arguments.push_back(QStringLiteral("--gpu-solver"));
                const auto linearMode = SolverNameSpace::LinearSolverSettings::Mode();
                switch (linearMode)
                {
                    case SolverNameSpace::LinearSolverMode::Ldlt:
                        arguments.push_back(QStringLiteral("--linear-solver=ldlt"));
                        break;
                    case SolverNameSpace::LinearSolverMode::Lu:
                        arguments.push_back(QStringLiteral("--linear-solver=lu"));
                        break;
                    case SolverNameSpace::LinearSolverMode::Pardiso:
                        arguments.push_back(QStringLiteral("--linear-solver=pardiso"));
                        break;
                    case SolverNameSpace::LinearSolverMode::CudaIterative:
                        arguments.push_back(QStringLiteral("--linear-solver=cuda"));
                        break;
                    case SolverNameSpace::LinearSolverMode::Cudss:
                        arguments.push_back(QStringLiteral("--linear-solver=cudss"));
                        break;
                    case SolverNameSpace::LinearSolverMode::Automatic:
                        break;
                }
                arguments.push_back(
                    QStringLiteral("--result-batch-frames=%1").arg(ResultOutputSettings::FrameBatchSize()));
                arguments.push_back(ResultOutputSettings::IsBackgroundWriteEnabled()
                                        ? QStringLiteral("--background-result-write")
                                        : QStringLiteral("--no-background-result-write"));
                arguments.push_back(Logger::DetailedOutputEnabled() ? QStringLiteral("--verbose-log")
                                                                    : QStringLiteral("--quiet-log"));

                QProcess process;
                process.setProgram(QCoreApplication::applicationFilePath());
                process.setArguments(arguments);
                process.setProcessChannelMode(QProcess::SeparateChannels);
                process.start();
                if (!process.waitForStarted(30000))
                {
                    finalMessage = QStringLiteral("独立计算进程启动失败：%1").arg(process.errorString());
                }
                else
                {
                    workerProcessId = static_cast<qint64>(process.processId());
                    bool receivedResult = false;
                    QByteArray pendingOutput;
                    QByteArray diagnosticOutput;
                    QStringList profileBindingMessages;
                    bool cancelSent = false;
                    const auto consumeLines = [&]()
                    {
                        pendingOutput += process.readAllStandardOutput();
                        qsizetype newline = 0;
                        while ((newline = pendingOutput.indexOf('\n')) >= 0)
                        {
                            const QByteArray line = pendingOutput.left(newline).trimmed();
                            pendingOutput.remove(0, newline + 1);
                            if (line.startsWith("YQY_AERO_BINDING "))
                            {
                                if (profileBindingMessages.size() < 24)
                                    profileBindingMessages.push_back(QString::fromUtf8(line.mid(17)));
                                continue;
                            }
                            if (!line.startsWith("YQY_WORKER "))
                                continue;
                            const QJsonDocument document = QJsonDocument::fromJson(line.mid(11));
                            if (!document.isObject())
                                continue;
                            const QJsonObject event = document.object();
                            const QString type = event.value(QStringLiteral("type")).toString();
                            if (type == QStringLiteral("progress"))
                            {
                                const double progress = event.value(QStringLiteral("progress")).toDouble();
                                const QString message = event.value(QStringLiteral("message")).toString();
                                QMetaObject::invokeMethod(
                                    this,
                                    [this, id = task->info.id, progress, message]()
                                    {
                                        reportProgress(id, progress, message);
                                    },
                                    Qt::QueuedConnection);
                            }
                            else if (type == QStringLiteral("result"))
                            {
                                receivedResult = true;
                                finalStatus =
                                    event.value(QStringLiteral("cancelled")).toBool()
                                        ? Status::Cancelled
                                        : (event.value(QStringLiteral("succeeded")).toBool() ? Status::Completed
                                                                                             : Status::Failed);
                                solverElapsedMs =
                                    static_cast<qint64>(event.value(QStringLiteral("solverMs")).toDouble(-1));
                                finalMessage = event.value(QStringLiteral("message")).toString();
                                const QString gpuReport = event.value(QStringLiteral("gpuReport")).toString();
                                const QString stageReport = event.value(QStringLiteral("stageReport")).toString();
                                const double peakMemory = event.value(QStringLiteral("peakWorkingSetMb")).toDouble();
                                finalMessage = completionMessage(finalStatus, finalMessage, gpuReport);
                                diagnostics = completionDiagnostics(task->info.outputFile, solverElapsedMs, gpuReport,
                                                                    stageReport, peakMemory, workerProcessId);
                            }
                        }
                    };
                    while (process.state() != QProcess::NotRunning)
                    {
                        process.waitForReadyRead(100);
                        consumeLines();
                        diagnosticOutput += process.readAllStandardError();
                        if (diagnosticOutput.size() > 4096)
                            diagnosticOutput = diagnosticOutput.right(4096);
                        if (!cancelSent && task->cancelRequested.load(std::memory_order_relaxed))
                        {
                            QFile marker(cancelPath);
                            cancelSent = marker.open(QIODevice::WriteOnly);
                        }
                    }
                    consumeLines();
                    diagnosticOutput += process.readAllStandardError();
                    if (!receivedResult)
                    {
                        finalStatus = cancelSent ? Status::Cancelled : Status::Failed;
                        finalMessage =
                            cancelSent ? QStringLiteral("任务已停止") : QStringLiteral("独立计算进程未返回结果");
                        const QString reason =
                            QString::fromUtf8(diagnosticOutput).trimmed().section(QLatin1Char('\n'), 0, 0).trimmed();
                        if (!cancelSent && !reason.isEmpty())
                        {
                            finalMessage =
                                QStringLiteral("工作进程失败：%1%2")
                                    .arg(reason.left(160), reason.size() > 160 ? QStringLiteral("…") : QString());
                        }
                    }
                    else if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
                    {
                        const bool wasCompleted = finalStatus == Status::Completed;
                        finalStatus = Status::Failed;
                        finalMessage = wasCompleted ? QStringLiteral("独立计算进程异常退出")
                                                    : finalMessage + QStringLiteral("；独立计算进程异常退出");
                    }
                    if (!diagnosticOutput.trimmed().isEmpty())
                    {
                        diagnostics += QStringLiteral("\n工作进程输出：\n%1")
                                           .arg(QString::fromUtf8(diagnosticOutput).trimmed().right(4096));
                    }
                    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
                        diagnostics += QStringLiteral("\n工作进程退出代码：%1").arg(process.exitCode());
                    if (!profileBindingMessages.isEmpty())
                        diagnostics +=
                            QStringLiteral("\n气动剖面绑定：\n") + profileBindingMessages.join(QLatin1Char('\n'));
                }
            }
        }
    }
    QMetaObject::invokeMethod(
        this,
        [this, id = task->info.id, finalStatus, finalMessage, solverElapsedMs, workerProcessId, diagnostics]()
        {
            const auto current = m_tasks.value(id);
            if (current)
                current->info.workerProcessId = workerProcessId;
            finishTask(id, finalStatus, finalMessage, solverElapsedMs, diagnostics);
        },
        Qt::QueuedConnection);
}

void SolveTaskController::reportProgress(int taskId, double progress, const QString& message)
{
    const auto task = m_tasks.value(taskId);
    if (!task || (task->info.status != Status::Queued && task->info.status != Status::Running &&
                  task->info.status != Status::Cancelling))
        return;
    task->info.progress = std::clamp(progress, 0.0, 1.0);
    if (!message.isEmpty() && task->info.status != Status::Cancelling)
        task->info.message = message;
    if (task->totalTimer.isValid())
        task->info.elapsedMs = task->totalTimer.elapsed();
    emit taskUpdated(taskId);
}

QString SolveTaskController::completionMessage(Status status, const QString& message, const QString& gpuReport)
{
    if (status != Status::Completed)
        return message.trimmed().isEmpty() ? statusText(status) : message.trimmed();
    const QRegularExpression fallbackPattern(QStringLiteral("CPU回退 ([1-9][0-9]*) 次"));
    if (fallbackPattern.match(gpuReport).hasMatch() || gpuReport.contains(QStringLiteral("失败")) ||
        gpuReport.contains(QStringLiteral("异常")))
        return QStringLiteral("计算完成（GPU 已回退 CPU，详见诊断）");
    return QStringLiteral("计算完成");
}

QString SolveTaskController::gpuDiagnostics(const QString& report)
{
    if (report.trimmed().isEmpty())
        return QString();
    const QRegularExpression statisticsPattern(
        QStringLiteral("^GPU统计：线性尝试 ([0-9]+) 次、成功 ([0-9]+) 次，单元批计算 ([0-9]+) 次，CPU回退 ([0-9]+) 次"
                       "(?:，因阈值跳过 ([0-9]+) 次)?"));
    const auto match = statisticsPattern.match(report);
    if (!match.hasMatch())
    {
        QString formatted = report.trimmed();
        formatted.replace(QStringLiteral("；"), QStringLiteral("\n  "));
        return QStringLiteral("GPU：%1").arg(formatted);
    }

    const qint64 attempts = match.captured(1).toLongLong();
    const qint64 successes = match.captured(2).toLongLong();
    const qint64 assemblies = match.captured(3).toLongLong();
    const qint64 fallbacks = match.captured(4).toLongLong();
    const qint64 skipped = match.captured(5).toLongLong();
    QStringList statistics;
    if (attempts > 0)
        statistics.push_back(QStringLiteral("线性尝试 %1 次").arg(attempts));
    if (successes > 0)
        statistics.push_back(QStringLiteral("线性成功 %1 次").arg(successes));
    if (assemblies > 0)
        statistics.push_back(QStringLiteral("单元批计算 %1 次").arg(assemblies));
    if (fallbacks > 0)
        statistics.push_back(QStringLiteral("CPU 回退 %1 次").arg(fallbacks));
    if (skipped > 0)
        statistics.push_back(QStringLiteral("阈值跳过 %1 次").arg(skipped));

    QString summary = statistics.join(QStringLiteral("，"));
    if (successes == 0 && assemblies == 0)
    {
        const QString state = attempts > 0 ? QStringLiteral("未完成 GPU 计算") : QStringLiteral("未使用 GPU");
        summary = summary.isEmpty() ? state : QStringLiteral("%1（%2）").arg(state, summary);
    }
    QStringList lines{QStringLiteral("GPU（任务线程）：%1").arg(summary)};
    const auto notes = report.mid(match.capturedEnd()).split(QStringLiteral("；"), Qt::SkipEmptyParts);
    for (const auto& note : notes)
        lines.push_back(QStringLiteral("  %1").arg(note.trimmed()));
    return lines.join(QLatin1Char('\n'));
}

QString SolveTaskController::stageDiagnostics(const QString& report)
{
    if (report.trimmed().isEmpty())
        return QString();
    QString formatted = report.trimmed();
    formatted.replace(QStringLiteral("GPU批次：有效切线 0、残差 0"), QStringLiteral("GPU批次：未使用 GPU 批计算"));
    formatted.replace(QStringLiteral("内部耗时(ms)："), QStringLiteral("\n  内部耗时（ms）：\n    "));
    formatted.replace(QStringLiteral("；系统评估 "), QStringLiteral("\n    系统评估："));
    formatted.replace(QStringLiteral("（自身 "), QStringLiteral("\n      自身："));
    formatted.replace(QStringLiteral("；包含："), QStringLiteral("\n      包含："));
    formatted.replace(QStringLiteral("）；结果输出 "), QStringLiteral("\n    结果输出："));
    formatted.replace(QStringLiteral("。包含项不可与系统评估或求解步直接相加；GPU批次："),
                      QStringLiteral("\n    计时说明：包含项不可与系统评估或求解步直接相加。\n    GPU 批次："));
    formatted.replace(QStringLiteral("求解步 "), QStringLiteral("求解步："));
    formatted.replace(QStringLiteral("、"), QStringLiteral("\n        "));
    formatted.replace(QStringLiteral("；"), QStringLiteral("\n\n"));
    return QStringLiteral("分析步诊断：\n%1").arg(formatted);
}

QString SolveTaskController::completionDiagnostics(const QString& outputFile, qint64 solverElapsedMs,
                                                   const QString& gpuReport, const QString& stageReport,
                                                   double peakMemoryMegabytes, qint64 workerProcessId)
{
    QStringList lines;
    if (!outputFile.isEmpty())
        lines.push_back(QStringLiteral("结果路径：%1").arg(QDir::toNativeSeparators(outputFile)));
    if (solverElapsedMs >= 0)
        lines.push_back(QStringLiteral("求解器内部耗时：%1 ms").arg(solverElapsedMs));
    const QString gpu = gpuDiagnostics(gpuReport);
    if (!gpu.isEmpty())
        lines.push_back(gpu);
    const QString stages = stageDiagnostics(stageReport);
    if (!stages.isEmpty())
        lines.push_back(stages);
    if (peakMemoryMegabytes > 0.0)
    {
        lines.push_back(
            workerProcessId > 0
                ? QStringLiteral("内存：工作进程 %1 峰值工作集 %2 MB（含进程开销）")
                      .arg(workerProcessId)
                      .arg(peakMemoryMegabytes, 0, 'f', 1)
                : QStringLiteral("内存：进程历史峰值工作集 %1 MB（非本算例独占）").arg(peakMemoryMegabytes, 0, 'f', 1));
    }
    return lines.join(QLatin1Char('\n'));
}

void SolveTaskController::finishTask(int taskId, Status status, const QString& message, qint64 elapsedMs,
                                     const QString& diagnostics)
{
    const auto task = m_tasks.value(taskId);
    if (!task)
        return;
    const bool completionWasCancelled =
        status == Status::Completed && task->cancelRequested.load(std::memory_order_relaxed);
    if (completionWasCancelled)
        status = Status::Cancelled;
    if (status != Status::Completed)
    {
        task->solvedModel.reset();
        task->solvedRegionStates.clear();
    }
    task->info.status = status;
    task->info.waitingForInitialStatic = false;
    task->workerScheduled = false;
    task->info.message = completionWasCancelled ? QStringLiteral("任务已停止（完成通知前已接受停止请求）") : message;
    task->info.diagnostics = diagnostics.trimmed();
    if (status == Status::Completed)
        task->info.progress = 1.0;
    if (elapsedMs >= 0)
        task->info.solverElapsedMs = elapsedMs;
    if (task->totalTimer.isValid())
        task->info.elapsedMs = task->totalTimer.elapsed();

    const QFileInfo resultFile(task->info.outputFile);
    const bool outputWasUpdated =
        resultFile.exists() && (task->previousOutputModifiedMs < 0 ||
                                resultFile.lastModified().toMSecsSinceEpoch() != task->previousOutputModifiedMs ||
                                resultFile.size() != task->previousOutputSize);
    if (outputWasUpdated)
    {
        Hdf5ResultReader reader;
        std::vector<Hdf5ResultFrameInfo> frames;
        if (reader.open(task->info.outputFile, frames) && !frames.empty())
        {
            task->info.hasUsableResult = true;
            task->info.partialResult = status != Status::Completed;
            task->info.resultFrameCount = static_cast<qint64>(frames.size());
            task->info.resultEndTime = frames.back().time;
            if (task->info.partialResult)
            {
                task->info.message += QStringLiteral("；已保留 %1 帧部分结果，截止 t=%2 s")
                                          .arg(task->info.resultFrameCount)
                                          .arg(task->info.resultEndTime, 0, 'g', 10);
            }
        }
    }
    if (task->totalTimer.isValid())
        task->info.elapsedMs = task->totalTimer.elapsed();
    if (task->totalTimer.isValid())
    {
        if (!task->info.diagnostics.isEmpty())
            task->info.diagnostics += QLatin1Char('\n');
        task->info.diagnostics += QStringLiteral("任务耗时：%1 ms（含排队、准备与结果检查）").arg(task->info.elapsedMs);
    }
    const bool shouldRestart = task->restartRequested;
    task->restartRequested = false;
    emit taskUpdated(taskId);
    if (!m_restartBatch.isEmpty() || m_stagingRestartBatch)
    {
        tryStartRestartBatch();
        return;
    }
    if (shouldRestart)
    {
        start(taskId);
        return;
    }
    releaseDependentTasks(task, status);
}
