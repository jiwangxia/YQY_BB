#include "Controllers/SolveTaskController.h"

#include "Application/ApplicationPaths.h"
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include "Solver/AnalysisSolve.h"

#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QMetaObject>
#include <QThread>

#include <algorithm>
#include <chrono>
#include <exception>
#include <set>

namespace
{
QString hdf5OutputDirectory(const StructureData& model)
{
    const QString configuredFileName = model.m_OutputControl.m_Hdf5FileName.trimmed();
    if (!configuredFileName.isEmpty())
        return QFileInfo(configuredFileName).absoluteDir().absolutePath();

    return ApplicationPaths::hdf5ResultDirectory();
}
}

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
    if (model)
        model->EnsureDefaultAnalysisConfiguration();
    if (!model || sourceFile.trimmed().isEmpty() || model->m_AnalysisStep.empty())
        return -1;
    if (analysisStepId > 0 && model->m_AnalysisStep.find(analysisStepId) == model->m_AnalysisStep.end())
        return -1;

    const QString absoluteSource = QFileInfo(sourceFile).absoluteFilePath();
    std::shared_ptr<TaskContext> task;
    for (const auto& existing : m_tasks)
    {
        if (existing && existing->info.sourceFile == absoluteSource && existing->info.analysisStepId == analysisStepId)
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
    task->startedAtMs.store(0, std::memory_order_relaxed);
    task->lastProgressReportMs.store(0, std::memory_order_relaxed);
    task->workerScheduled = false;
    task->solvedModel.reset();
    task->info.sourceFile = absoluteSource;
    task->info.analysisStepId = analysisStepId;
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
    task->info.status = Status::Ready;
    task->info.progress = 0.0;
    task->info.elapsedMs = 0;
    task->info.message = QStringLiteral("参数已就绪，点击算例按钮开始计算");

    const QFileInfo sourceInfo(task->info.sourceFile);
    QDir outputDir(hdf5OutputDirectory(*model));
    if (!outputDir.exists())
        QDir().mkpath(outputDir.absolutePath());
    task->info.outputFile = outputDir.filePath(
        QStringLiteral("%1_case_%2.h5").arg(sourceInfo.completeBaseName()).arg(task->info.id, 3, 10, QLatin1Char('0')));
    modelTemplate->m_OutputControl.m_Hdf5FileName = task->info.outputFile;
    task->modelTemplate = std::move(modelTemplate);

    m_tasks.insert(task->info.id, task);
    if (isNew)
        emit taskAdded(task->info.id);
    else
        emit taskUpdated(task->info.id);

    // A static task is prepared when the conductor model is created, while a
    // dynamic step is often added later from the analysis editor. Refresh the
    // source static template at that point so its cached equilibrium model
    // contains the new dynamic-step definition before state inheritance.
    if (requiredStaticStepId > 0 && requiredStaticStepId != analysisStepId)
        prepare(model, sourceFile, requiredStaticStepId);
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
        if (missingStep && !active)
            it = m_tasks.erase(it);
        else
            ++it;
    }
}

bool SolveTaskController::start(int taskId)
{
    const auto task = m_tasks.value(taskId);
    if (!task || (task->info.status != Status::Ready && task->info.status != Status::Completed &&
                  task->info.status != Status::Failed && task->info.status != Status::Cancelled))
        return false;
    task->cancelRequested.store(false, std::memory_order_relaxed);
    task->startedAtMs.store(0, std::memory_order_relaxed);
    task->lastProgressReportMs.store(0, std::memory_order_relaxed);
    task->info.status = Status::Queued;
    task->info.progress = 0.0;
    task->info.elapsedMs = 0;
    task->info.hasUsableResult = false;
    task->info.partialResult = false;
    task->info.resultFrameCount = 0;
    task->info.resultEndTime = 0.0;
    task->workerScheduled = false;
    task->solvedModel.reset();
    task->info.message = QStringLiteral("等待计算线程");
    const QFileInfo previousOutput(task->info.outputFile);
    task->previousOutputModifiedMs = previousOutput.exists() ? previousOutput.lastModified().toMSecsSinceEpoch() : -1;
    task->previousOutputSize = previousOutput.exists() ? previousOutput.size() : -1;
    emit taskUpdated(taskId);

    const auto dependency = dependencyTask(task);
    if (dependency)
    {
        if (dependency->info.status == Status::Completed && dependency->solvedModel)
            launchTask(task);
        else
        {
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
    const int staticStepId = initialStaticStepId(task);
    if (staticStepId <= 0 || !task)
        return nullptr;
    for (const auto& candidate : m_tasks)
    {
        if (candidate && candidate->info.sourceFile == task->info.sourceFile &&
            candidate->info.analysisStepId == staticStepId)
            return candidate;
    }
    return nullptr;
}

void SolveTaskController::launchTask(const std::shared_ptr<TaskContext>& task)
{
    if (!task || task->workerScheduled || task->info.status != Status::Queued)
        return;
    task->workerScheduled = true;
    m_threadPool.start(
        [this, task]()
        {
            runTask(task);
        });
}

void SolveTaskController::releaseDependentTasks(const std::shared_ptr<TaskContext>& dependency, Status status)
{
    if (!dependency)
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
            finishTask(candidate->info.id, Status::Failed, QStringLiteral("前置静力步求解失败，动力步未启动"));
        }
    }
}

int SolveTaskController::startAllReady()
{
    int startedCount = 0;
    const QList<int> ids = taskIds();
    for (int id : ids)
    {
        const auto task = m_tasks.value(id);
        if (task && task->info.status == Status::Ready && start(id))
            ++startedCount;
    }
    return startedCount;
}

int SolveTaskController::restartAll()
{
    int requestedCount = 0;
    const QList<int> ids = taskIds();
    for (int id : ids)
    {
        const auto task = m_tasks.value(id);
        if (!task)
            continue;

        ++requestedCount;
        if (task->info.status == Status::Queued || task->info.status == Status::Running)
        {
            task->restartRequested = true;
            cancel(id);
        }
        else if (task->info.status == Status::Cancelling)
        {
            task->restartRequested = true;
        }
        else
        {
            start(id);
        }
    }
    return requestedCount;
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
    for (auto it = m_tasks.begin(); it != m_tasks.end(); ++it)
        it.value()->cancelRequested.store(true, std::memory_order_relaxed);
}

QList<int> SolveTaskController::taskIds() const
{
    auto ids = m_tasks.keys();
    std::sort(ids.begin(), ids.end());
    return ids;
}

SolveTaskController::TaskInfo SolveTaskController::taskInfo(int taskId) const
{
    const auto task = m_tasks.value(taskId);
    return task ? task->info : TaskInfo{};
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

void SolveTaskController::setMaximumThreadCount(int count)
{
    // QThreadPool does not terminate workers that are already running when the
    // limit is lowered.  They finish normally and subsequent scheduling obeys
    // the new limit, so changing this setting cannot damage an active solve.
    m_threadPool.setMaxThreadCount(std::clamp(count, MinimumThreadCount, maximumAllowedThreadCount()));
}

int SolveTaskController::availableThreadCount()
{
    return std::max(1, QThread::idealThreadCount());
}

int SolveTaskController::maximumAllowedThreadCount()
{
    const int available = availableThreadCount();
    // Reserve roughly one quarter of the logical processors (at least one)
    // for the UI, rendering, I/O and the operating system.  Twelve remains the
    // product-wide hard limit even on a high-core-count workstation.
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

void SolveTaskController::runTask(const std::shared_ptr<TaskContext>& task)
{
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
            if (!current)
                return;
            current->info.status = Status::Running;
            current->info.message = QStringLiteral("正在创建独立计算模型");
            emit taskUpdated(id);
        },
        Qt::QueuedConnection);

    Status finalStatus = Status::Failed;
    QString finalMessage;
    qint64 solverElapsedMs = -1;
    QElapsedTimer solverTimer;
    try
    {
        QString cloneError;
        const auto dependency = dependencyTask(task);
        const bool startsFromStaticState = dependency && dependency->solvedModel;
        auto structure = startsFromStaticState
                             ? dependency->solvedModel->CloneForAnalysis(&cloneError)
                             : (task->modelTemplate ? task->modelTemplate->CloneForAnalysis(&cloneError) : nullptr);
        if (!structure)
            throw std::runtime_error(cloneError.isEmpty() ? "无法创建独立计算模型" : cloneError.toUtf8().constData());

        structure->m_OutputControl.m_Hdf5FileName = task->info.outputFile;

        AnalysisRunner runner;
        runner.SetStructure(structure);
        runner.SetMaximumRegionThreads(maximumThreadCount());
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

        task->startedAtMs.store(QDateTime::currentMSecsSinceEpoch(), std::memory_order_relaxed);
        solverTimer.start();
        // A prepared single-step task may retain its referenced static step as
        // an internal dependency.  Run the selected step explicitly so the
        // dependency is solved first without turning it into a user-visible
        // result step in the dynamic task's H5 file.
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
                task->solvedModel = structure;
            finalStatus = Status::Completed;
            finalMessage = QStringLiteral("计算完成，结果：%1").arg(task->info.outputFile);
        }
        else
        {
            const auto failedStep = structure->m_AnalysisStep.find(task->info.analysisStepId);
            finalMessage = failedStep != structure->m_AnalysisStep.cend() && failedStep->second &&
                                   !failedStep->second->LastFailureReason().isEmpty()
                               ? failedStep->second->LastFailureReason()
                               : QStringLiteral("求解器返回失败，请查看日志和模型参数");
        }
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

    QMetaObject::invokeMethod(
        this,
        [this, id = task->info.id, finalStatus, finalMessage, solverElapsedMs]()
        {
            finishTask(id, finalStatus, finalMessage, solverElapsedMs);
        },
        Qt::QueuedConnection);
}

void SolveTaskController::reportProgress(int taskId, double progress, const QString& message)
{
    const auto task = m_tasks.value(taskId);
    if (!task)
        return;
    task->info.progress = std::clamp(progress, 0.0, 1.0);
    if (!message.isEmpty())
        task->info.message = message;
    const qint64 startedAtMs = task->startedAtMs.load(std::memory_order_relaxed);
    if (startedAtMs > 0)
        task->info.elapsedMs = QDateTime::currentMSecsSinceEpoch() - startedAtMs;
    emit taskUpdated(taskId);
}

void SolveTaskController::finishTask(int taskId, Status status, const QString& message, qint64 elapsedMs)
{
    const auto task = m_tasks.value(taskId);
    if (!task)
        return;
    task->info.status = status;
    task->workerScheduled = false;
    task->info.message = message;
    if (status == Status::Completed)
        task->info.progress = 1.0;
    if (elapsedMs >= 0)
        task->info.elapsedMs = elapsedMs;
    else
    {
        const qint64 startedAtMs = task->startedAtMs.load(std::memory_order_relaxed);
        if (startedAtMs > 0)
            task->info.elapsedMs = QDateTime::currentMSecsSinceEpoch() - startedAtMs;
    }

    const QFileInfo resultFile(task->info.outputFile);
    const bool outputWasUpdated =
        resultFile.exists() && (task->previousOutputModifiedMs < 0 ||
                                resultFile.lastModified().toMSecsSinceEpoch() != task->previousOutputModifiedMs ||
                                resultFile.size() != task->previousOutputSize);
    if (outputWasUpdated)
    {
        Hdf5ModelIO reader;
        std::vector<Hdf5ResultFrameInfo> frames;
        if (reader.OpenResultFile(task->info.outputFile, frames) && !frames.empty())
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
    const bool shouldRestart = task->restartRequested;
    task->restartRequested = false;
    emit taskUpdated(taskId);
    if (shouldRestart)
    {
        start(taskId);
        return;
    }
    releaseDependentTasks(task, status);
}
