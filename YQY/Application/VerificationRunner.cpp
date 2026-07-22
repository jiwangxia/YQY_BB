#include "Application/VerificationRunner.h"

#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ResultIO.h"
#include "GUI/Controllers/ModelController.h"
#include "GUI/Controllers/SolveTaskController.h"

namespace
{
std::optional<int> verifyNodeExport(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-node-export"));
    if (index < 0)
        return std::nullopt;
    if (index + 2 >= arguments.size())
        return 1;

    const QString source = arguments.at(index + 1);
    const QString target = arguments.at(index + 2);
    Hdf5ResultIO exporter;
    const bool exported =
        exporter.ExportBdfResultFromHdf5(source, target, {1},
                                         {EnumKeyword::NodeResultType::U1, EnumKeyword::NodeResultType::U2,
                                          EnumKeyword::NodeResultType::U3, EnumKeyword::NodeResultType::MagnitudeU},
                                         {}, {});
    QTextStream(stdout) << "node export=" << (exported ? "success" : "failed") << " file=" << target << Qt::endl;
    return exported ? 0 : 2;
}

std::optional<int> verifyResultRead(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-result-read"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    Hdf5ResultIO reader;
    std::vector<Hdf5ResultFrameInfo> frames;
    if (!reader.OpenResultFile(arguments.at(index + 1), frames) || frames.empty())
        return 2;
    Hdf5ResultFrame first;
    Hdf5ResultFrame last;
    if (!reader.ReadResultFrame(0, first) || !reader.ReadResultFrame(static_cast<int>(frames.size()) - 1, last))
    {
        return 3;
    }
    QTextStream(stdout) << "result frames=" << frames.size() << " first_step=" << first.info.stepId
                        << " first_nodes=" << first.nodes.size() << " first_elements=" << first.elements.size()
                        << " last_time=" << last.info.time << Qt::endl;
    return first.nodes.empty() || first.elements.empty() ? 4 : 0;
}

std::optional<int> verifyHdf5Model(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-hdf5-model"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Hdf5ResultIO reader;
    if (!reader.ImportHdf5(arguments.at(index + 1), structure.get()))
        return 2;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        if (!element || element->m_pNode.size() != 2 || element->m_pNode[0].expired()
            || element->m_pNode[1].expired())
        {
            QTextStream(stderr) << "invalid element=" << elementId << Qt::endl;
            return 3;
        }
    }
    QTextStream(stdout) << "hdf5 model nodes=" << structure->m_Nodes.size()
                        << " elements=" << structure->m_Elements.size()
                        << " materials=" << structure->m_Material.size()
                        << " sections=" << structure->m_Section.size() << Qt::endl;
    return structure->m_Nodes.empty() || structure->m_Elements.empty() ? 4 : 0;
}

std::optional<int> verifySolveTask(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-solve-task"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    const QString filePath = arguments.at(index + 1);
    ModelController modelController;
    SolveTaskController taskController;
    int taskId = -1;
    QObject::connect(
        &modelController, &ModelController::loadSucceeded,
        [&modelController, &taskController, &taskId, &application](int modelId, const QString& loadedFile, qint64)
        {
            taskId = taskController.prepare(modelController.model(modelId), loadedFile, 0);
            if (taskId < 0 || !taskController.start(taskId))
                application.exit(2);
        });
    QObject::connect(&modelController, &ModelController::loadFailed,
                     [&application](const QString&, const QString&) { application.exit(3); });
    QObject::connect(&taskController, &SolveTaskController::taskUpdated,
                     [&taskController, &taskId, &application](int updatedTaskId)
                     {
                         if (updatedTaskId != taskId)
                             return;
                         const auto info = taskController.taskInfo(updatedTaskId);
                         if (info.status != SolveTaskController::Status::Completed &&
                             info.status != SolveTaskController::Status::Failed &&
                             info.status != SolveTaskController::Status::Cancelled)
                         {
                             return;
                         }
                         QTextStream(stdout)
                             << "solve status=" << SolveTaskController::statusText(info.status)
                             << " elapsed_ms=" << info.elapsedMs << " message=" << info.message << Qt::endl;
                         application.exit(info.status == SolveTaskController::Status::Completed ? 0 : 4);
                     });
    QTimer::singleShot(120000, &application, [&application]() { application.exit(5); });
    if (!modelController.loadModel(filePath))
        return 1;
    return application.exec();
}

std::optional<int> verifyPartialResult(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-partial-result"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    const QString filePath = arguments.at(index + 1);
    ModelController modelController;
    SolveTaskController taskController;
    int taskId = -1;
    bool cancellationRequested = false;
    QObject::connect(
        &modelController, &ModelController::loadSucceeded,
        [&modelController, &taskController, &taskId, &application](int modelId, const QString& loadedFile, qint64)
        {
            taskId = taskController.prepare(modelController.model(modelId), loadedFile, 0);
            if (taskId < 0 || !taskController.start(taskId))
                application.exit(2);
        });
    QObject::connect(&modelController, &ModelController::loadFailed,
                     [&application](const QString&, const QString&) { application.exit(3); });
    QObject::connect(
        &taskController, &SolveTaskController::taskUpdated,
        [&taskController, &taskId, &cancellationRequested, &application](int updatedTaskId)
        {
            if (updatedTaskId != taskId)
                return;
            const auto info = taskController.taskInfo(updatedTaskId);
            if (!cancellationRequested && info.status == SolveTaskController::Status::Running && info.progress >= 0.05)
            {
                cancellationRequested = taskController.cancel(updatedTaskId);
                return;
            }
            if (info.status != SolveTaskController::Status::Completed &&
                info.status != SolveTaskController::Status::Failed &&
                info.status != SolveTaskController::Status::Cancelled)
            {
                return;
            }
            QTextStream(stdout) << "partial status=" << SolveTaskController::statusText(info.status)
                                << " available=" << (info.hasUsableResult ? "true" : "false")
                                << " partial=" << (info.partialResult ? "true" : "false")
                                << " frames=" << info.resultFrameCount << " end_time=" << info.resultEndTime
                                << " file=" << info.outputFile << Qt::endl;
            application.exit(info.hasUsableResult && info.partialResult && info.resultFrameCount > 0 ? 0 : 4);
        });
    QTimer::singleShot(120000, &application, [&application]() { application.exit(5); });
    if (!modelController.loadModel(filePath))
        return 1;
    return application.exec();
}

std::optional<int> verifyModelImport(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-model-import"));
    if (index < 0)
        return std::nullopt;

    const QStringList files = arguments.mid(index + 1);
    ModelController controller;
    int loadedCount = 0;
    int failedCount = 0;
    int cloneFailedCount = 0;
    QObject::connect(
        &controller, &ModelController::loadSucceeded,
        [&controller, &loadedCount, &cloneFailedCount](int modelId, const QString& filePath, qint64 elapsedMs)
        {
            ++loadedCount;
            QString cloneError;
            const auto firstClone = controller.model(modelId)->CloneForAnalysis(&cloneError);
            const auto secondClone = firstClone ? firstClone->CloneForAnalysis(&cloneError) : nullptr;
            if (!secondClone)
            {
                ++cloneFailedCount;
                QTextStream(stderr) << "clone failed model=" << modelId << " error=" << cloneError << Qt::endl;
            }
            QTextStream(stdout) << "loaded model=" << modelId << " file=" << filePath << " elapsed_ms=" << elapsedMs
                                << Qt::endl;
        });
    QObject::connect(&controller, &ModelController::loadFailed,
                     [&failedCount](const QString& filePath, const QString& errorMessage)
                     {
                         ++failedCount;
                         QTextStream(stderr) << "failed file=" << filePath << " error=" << errorMessage << Qt::endl;
                     });
    QObject::connect(
        &controller, &ModelController::busyChanged,
        [&controller, &application, &loadedCount, &failedCount, &cloneFailedCount, expected = files.size()](bool busy)
        {
            if (busy)
                return;
            const QList<int> ids = controller.modelIds();
            bool isolated = ids.size() == loadedCount;
            if (ids.size() >= 2)
            {
                const auto first = controller.model(ids.at(0));
                const auto second = controller.model(ids.at(1));
                if (first && second)
                {
                    const auto firstNode = first->FindNode(1);
                    const auto secondNode = second->FindNode(1);
                    if (firstNode && secondNode)
                        isolated = isolated && firstNode.get() != secondNode.get();
                }
            }
            QTextStream(stdout) << "summary loaded=" << loadedCount << " failed=" << failedCount
                                << " clone_failed=" << cloneFailedCount << " documents=" << controller.modelCount()
                                << " isolated=" << (isolated ? "true" : "false") << Qt::endl;
            application.exit(loadedCount == expected && failedCount == 0 && cloneFailedCount == 0 && isolated ? 0 : 1);
        });
    QTimer::singleShot(30000, &application, [&application]() { application.exit(2); });
    if (files.isEmpty() || controller.loadModels(files) != files.size())
        return 1;
    return application.exec();
}
} // namespace

std::optional<int> VerificationRunner::run(QApplication& application, const QStringList& arguments)
{
    if (const auto result = verifyNodeExport(arguments))
        return result;
    if (const auto result = verifyResultRead(arguments))
        return result;
    if (const auto result = verifyHdf5Model(arguments))
        return result;
    if (const auto result = verifySolveTask(application, arguments))
        return result;
    if (const auto result = verifyPartialResult(application, arguments))
        return result;
    return verifyModelImport(application, arguments);
}
