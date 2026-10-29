#include "Application/SolveWorker.h"

#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelSerializer.h"
#include "Solver/Assembly/AssemblySettings.h"
#include "Solver/Linear/Gpu/GpuSettings.h"
#include "Solver/Workflow/AnalysisSolve.h"
#include "Utility/AnalysisDiagnostics.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>

#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>

static void WriteIsolatedSolveEvent(const QJsonObject& event)
{
    QTextStream stream(stdout);
    stream << "YQY_WORKER " << QString::fromUtf8(QJsonDocument(event).toJson(QJsonDocument::Compact)) << Qt::endl;
}

int RunIsolatedSolveWorker(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--solve-worker"));
    if (index < 0 || index + 7 >= arguments.size())
        return 2;

    const QString modelPath = arguments.at(index + 1);
    const QString outputPath = arguments.at(index + 2);
    const QString cancelPath = arguments.at(index + 3);
    bool threadBudgetValid = false;
    bool assemblyLimitValid = false;
    const int threadBudget = arguments.at(index + 4).toInt(&threadBudgetValid);
    const int assemblyLimit = arguments.at(index + 5).toInt(&assemblyLimitValid);
    const QString sourceModelName = arguments.at(index + 6);
    const bool streamResult = arguments.at(index + 7) == QStringLiteral("1");
    if (!threadBudgetValid || !assemblyLimitValid || threadBudget <= 0 || assemblyLimit <= 0)
        return 2;

    SolverNameSpace::AssemblySettings::SetThreadCount(assemblyLimit);
    SolverNameSpace::GpuSettings::ResetThreadReport();
    bool succeeded = false;
    bool cancelled = false;
    QString message;
    QString stageReport;
    qint64 solverMilliseconds = -1;
    try
    {
        auto structure = std::make_shared<StructureData>();
        Hdf5ModelSerializer serializer;
        if (!serializer.importModel(modelPath, structure.get()))
            throw std::runtime_error("工作进程无法读取模型快照");
        structure->m_OutputControl.m_Hdf5FileName = outputPath;
        structure->m_OutputControl.m_SourceModelName = sourceModelName;
        structure->m_OutputControl.m_StreamResult = streamResult;

        AnalysisRunner runner;
        runner.SetStructure(structure);
        runner.SetMaximumRegionThreads(threadBudget);
        runner.SetExecutionThreadBudget(threadBudget);
        qint64 lastReportMilliseconds = -1000;
        QElapsedTimer progressTimer;
        progressTimer.start();
        qint64 lastCancelCheckMilliseconds = -1000;
        bool cancelRequested = false;
        std::mutex progressMutex;
        std::mutex cancelMutex;
        runner.SetRuntimeCallbacks(
            [&progressTimer, &lastReportMilliseconds, &progressMutex](double progress, const QString& progressMessage)
            {
                const std::lock_guard lock(progressMutex);
                const qint64 now = progressTimer.elapsed();
                if (progress < 1.0 && now - lastReportMilliseconds < 100)
                    return;
                lastReportMilliseconds = now;
                WriteIsolatedSolveEvent({{QStringLiteral("type"), QStringLiteral("progress")},
                                         {QStringLiteral("progress"), progress},
                                         {QStringLiteral("message"), progressMessage}});
            },
            [&progressTimer, &lastCancelCheckMilliseconds, &cancelRequested, &cancelPath, &cancelMutex]()
            {
                const std::lock_guard lock(cancelMutex);
                const qint64 now = progressTimer.elapsed();
                if (now - lastCancelCheckMilliseconds >= 100)
                {
                    lastCancelCheckMilliseconds = now;
                    cancelRequested = QFileInfo::exists(cancelPath);
                }
                return cancelRequested;
            });

        QElapsedTimer solverTimer;
        solverTimer.start();
        succeeded = runner.RunAll();
        solverMilliseconds = solverTimer.elapsed();
        cancelled = runner.WasCancelled() || cancelRequested;
        message = cancelled ? QStringLiteral("任务已停止")
                            : (succeeded ? QStringLiteral("计算完成，结果：%1").arg(outputPath)
                                         : QStringLiteral("求解器返回失败，请查看日志和模型参数"));
        stageReport = AnalysisDiagnostics::PerformanceReport(*structure, 0);
    }
    catch (const std::exception& exception)
    {
        message = QString::fromUtf8(exception.what());
    }
    catch (...)
    {
        message = QStringLiteral("独立计算进程发生未知异常");
    }

    WriteIsolatedSolveEvent({{QStringLiteral("type"), QStringLiteral("result")},
                             {QStringLiteral("succeeded"), succeeded},
                             {QStringLiteral("cancelled"), cancelled},
                             {QStringLiteral("message"), message},
                             {QStringLiteral("solverMs"), static_cast<double>(solverMilliseconds)},
                             {QStringLiteral("gpuReport"), SolverNameSpace::GpuSettings::ThreadReport()},
                             {QStringLiteral("stageReport"), stageReport},
                             {QStringLiteral("peakWorkingSetMb"), AnalysisDiagnostics::ProcessPeakMemoryMegabytes()}});
    return 0;
}
