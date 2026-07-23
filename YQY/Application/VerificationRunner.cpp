#include "Application/VerificationRunner.h"

#include "Conductor/ConductorModelBuilder.h"
#include "Conductor/PropertyLibrary.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include "GUI/Controllers/ModelController.h"
#include "GUI/Controllers/SolveTaskController.h"
#include "Import/Input_Model.h"
#include "Solver/AnalysisSolve.h"

#include <QTemporaryDir>

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
    Hdf5ModelIO exporter;
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

    Hdf5ModelIO reader;
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
    Hdf5ModelIO reader;
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

std::optional<int> verifyHdf5ModelContract(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-hdf5-model-contract"));
    if (index < 0)
        return std::nullopt;
    if (index + 2 >= arguments.size())
        return 1;

    auto source = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), source))
        return 2;
    source->m_ModelSets.clear();
    source->m_ComputeRegions.clear();
    QString error;
    const int firstSetId = source->AddModelSet(QStringLiteral("区域A单元"), ModelSetType::Element, {1}, &error);
    const int secondSetId = source->AddModelSet(QStringLiteral("区域B单元"), ModelSetType::Element, {2}, &error);
    const int firstRegionId = source->AddComputeRegionFromSets(
        QStringLiteral("区域A"), {firstSetId}, true, &error);
    const int secondRegionId = source->AddComputeRegionFromSets(
        QStringLiteral("区域B"), {secondSetId}, true, &error);
    if (firstSetId <= 0 || secondSetId <= 0 || firstRegionId <= 0 || secondRegionId <= 0)
        return 3;

    source->EnsureDefaultAnalysisConfiguration();
    if (source->m_AnalysisStep.empty() || !source->m_AnalysisStep.begin()->second)
        return 4;
    auto sourceStep = source->m_AnalysisStep.begin()->second;
    sourceStep->m_Name = QStringLiteral("导线区域分析");
    sourceStep->m_RegionScope = AnalysisRegionScope::SelectedRegions;
    sourceStep->m_ComputeRegionIds = {firstRegionId, secondRegionId};

    Hdf5ModelIO writer;
    if (!writer.ExportModelHdf5(arguments.at(index + 2), source.get(), arguments.at(index + 1)))
        return 5;
    Hdf5ResultSummary summary;
    if (!writer.InspectHdf5(arguments.at(index + 2), summary) || !summary.hasModel || summary.hasResult)
        return 6;

    auto restored = std::make_shared<StructureData>();
    if (!writer.ImportHdf5(arguments.at(index + 2), restored.get()))
        return 7;
    const auto restoredStep = restored->m_AnalysisStep.find(sourceStep->m_Id);
    if (restored->m_ModelSets.size() != 2 || restored->m_ComputeRegions.size() != 2
        || restoredStep == restored->m_AnalysisStep.cend() || !restoredStep->second
        || restoredStep->second->m_Name != sourceStep->m_Name
        || restoredStep->second->m_RegionScope != AnalysisRegionScope::SelectedRegions
        || restoredStep->second->m_ComputeRegionIds.size() != 2)
    {
        return 8;
    }
    QTextStream(stdout) << "hdf5 contract sets=" << restored->m_ModelSets.size()
        << " regions=" << restored->m_ComputeRegions.size()
        << " steps=" << restored->m_AnalysisStep.size()
        << " result=" << (summary.hasResult ? 1 : 0) << Qt::endl;
    return 0;
}

std::optional<int> verifyComputeRegions(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-compute-regions"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), structure))
        return 2;
    structure->m_ComputeRegions.clear();
    QString error;
    const int firstRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 A"), {}, {1}, {}, true, &error);
    const int secondRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 B"), {}, {2}, {}, true, &error);
    if (firstRegionId <= 0 || secondRegionId <= 0 || structure->m_ComputeRegions.size() != 2)
    {
        QTextStream(stderr) << "region setup failed: " << error << Qt::endl;
        return 3;
    }
    const int mergedRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 A 重叠候选"), {}, {1}, {}, true, &error);
    if (mergedRegionId != firstRegionId || structure->m_ComputeRegions.size() != 2)
    {
        QTextStream(stderr) << "overlap merge failed: " << error << Qt::endl;
        return 4;
    }

    structure->m_OutputControl.m_StreamResult = false;
    AnalysisRunner runner;
    runner.SetStructure(structure);
    if (!runner.RunAll() || structure->GetOutputter().GetDataSet().empty())
        return 5;
    const auto& frame = structure->GetOutputter().GetDataSet().back();
    QTextStream(stdout) << "compute regions=" << structure->m_ComputeRegions.size()
        << " merged_id=" << mergedRegionId
        << " result_nodes=" << frame.GetNodeDatas().size()
        << " result_elements=" << frame.GetElementDatas().size() << Qt::endl;
    return frame.GetNodeDatas().size() == 4 && frame.GetElementDatas().size() == 2 ? 0 : 6;
}

std::optional<int> verifyConductorBundle(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-conductor-bundle")))
        return std::nullopt;

    auto structure = std::make_shared<StructureData>();
    Conductor::PropertyLibrary library;
    QString propertyError;
    if (!library.load(propertyError) || !library.isReady())
    {
        QTextStream(stderr) << "conductor property library failed: " << propertyError << Qt::endl;
        return 1;
    }

    auto property = library.instantiateProperty(0, 0, *structure, propertyError);
    if (!property)
    {
        QTextStream(stderr) << "conductor property failed: " << propertyError << Qt::endl;
        return 2;
    }

    Conductor::LineBuildConfig config;
    config.start = Vector3d(0.0, 0.0, 20.0);
    config.end = Vector3d(300.0, 0.0, 20.0);
    config.property = property;
    config.elementType = EnumKeyword::ElementType::T3D2;
    config.conductor.nBundle = 4;
    config.conductor.spacing = 0.45;
    config.conductor.segments = 10;
    config.conductor.stress0 = 50.0e6;
    config.conductor.connecttype = Conductor::ConnectionMode::Parallel;
    config.convergeBundleEnds = true;
    config.setNamePrefix = QStringLiteral("验证单档");

    Conductor::ConductorModelBuilder builder(structure);
    Conductor::LineBuildResult result;
    std::string buildError;
    Conductor::SpanConductorBuildConfig spanConfig;
    spanConfig.line = config;
    spanConfig.useInnerSpacerLayout = true;
    spanConfig.innerSpacerLayout.useEqualSpacing = false;
    spanConfig.innerSpacerLayout.spacer.elementType = EnumKeyword::ElementType::CR3D;
    spanConfig.innerSpacerLayout.spacer.property = property;
    if (!builder.BuildSpanConductor(spanConfig, result, buildError))
    {
        QTextStream(stderr) << "conductor build failed: " << QString::fromStdString(buildError) << Qt::endl;
        return 3;
    }

    if (result.subConductors.size() != 4
        || result.leftSupportNodeId <= 0
        || result.rightSupportNodeId <= 0
        || result.leftTensionEnd.groupNodeIds.size() != 2
        || result.rightTensionEnd.groupNodeIds.size() != 2
        || result.leftTensionEnd.yokeElementIds.size() != 2
        || result.rightTensionEnd.yokeElementIds.size() != 2
        || result.leftTensionEnd.stabilizerElementId <= 0
        || result.rightTensionEnd.stabilizerElementId <= 0
        || result.innerSpacers.empty()
        || structure->m_ModelSets.size() != 8)
    {
        return 4;
    }

    for (const auto& [wireId, sub] : result.subConductors)
    {
        if (sub.nodeIds.empty()
            || sub.nodeSetId <= 0
            || sub.elementSetId <= 0)
        {
            return 5;
        }
    }
    if (result.subConductors.at(0).nodeIds.front() != result.subConductors.at(1).nodeIds.front()
        || result.subConductors.at(2).nodeIds.front() != result.subConductors.at(3).nodeIds.front()
        || result.subConductors.at(0).nodeIds.front() == result.subConductors.at(2).nodeIds.front())
        return 10;

    int expectedConductorNodeId = 1;
    for (std::size_t groupIndex = 0; groupIndex < result.leftTensionEnd.groupNodeIds.size(); ++groupIndex)
    {
        const int leftGroupId = result.leftTensionEnd.groupNodeIds[groupIndex];
        const int rightGroupId = result.rightTensionEnd.groupNodeIds[groupIndex];
        if (leftGroupId != expectedConductorNodeId++)
            return 12;
        for (const auto& [wireId, sub] : result.subConductors)
        {
            if (sub.nodeIds.front() != leftGroupId)
                continue;
            for (std::size_t index = 1; index + 1 < sub.nodeIds.size(); ++index)
            {
                if (sub.nodeIds[index] != expectedConductorNodeId++)
                    return 13;
            }
            if (sub.nodeIds.back() != rightGroupId)
                return 14;
        }
        if (rightGroupId != expectedConductorNodeId++)
            return 15;
    }
    if (result.leftSupportNodeId != expectedConductorNodeId++
        || result.rightSupportNodeId != expectedConductorNodeId++)
        return 16;

    int expectedId = 1;
    for (const auto& [nodeId, node] : structure->m_Nodes)
    {
        if (!node || nodeId != expectedId++)
            return 6;
    }
    expectedId = 1;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        if (!element || elementId != expectedId++)
            return 7;
    }

    int expectedConductorElementId = 1;
    for (const auto& [wireId, sub] : result.subConductors)
    {
        for (int elementId : sub.elementIds)
        {
            const auto element = structure->FindElement(elementId);
            if (elementId != expectedConductorElementId++
                || !element
                || element->m_Role != ElementRole::Conductor
                || element->m_WireId != wireId
                || element->m_AeroProfileId != wireId
                || !element->HasAerodynamicLoad())
                return 17;
        }
    }
    for (const auto* end : {&result.leftTensionEnd, &result.rightTensionEnd})
    {
        std::vector<int> hardwareIds = end->yokeElementIds;
        if (end->stabilizerElementId > 0)
            hardwareIds.push_back(end->stabilizerElementId);
        for (int elementId : hardwareIds)
        {
            const auto element = structure->FindElement(elementId);
            if (!element
                || element->m_Role != ElementRole::TensionHardware
                || element->HasAerodynamicLoad())
                return 18;
        }
    }
    for (const auto& spacer : result.innerSpacers)
    {
        for (int elementId : spacer.elementIds)
        {
            const auto element = structure->FindElement(elementId);
            if (!element
                || element->m_Role != ElementRole::IntraPhaseSpacer
                || element->HasAerodynamicLoad())
                return 19;
        }
    }

    const auto firstWire = result.subConductors.find(0);
    if (firstWire == result.subConductors.cend() || firstWire->second.nodeIds.size() < 3)
        return 8;
    const auto rightBottomNode = structure->FindNode(firstWire->second.nodeIds[1]);
    if (!rightBottomNode || rightBottomNode->m_Y >= 0.0 || rightBottomNode->m_Z >= config.start.z())
        return 9;
    const double transitionDx = rightBottomNode->m_X - config.start.x();
    const double transitionDy = rightBottomNode->m_Y - config.start.y();
    if (std::hypot(transitionDx, transitionDy) < 0.5)
        return 11;

    QTemporaryDir temporaryDirectory;
    const QString h5Path = temporaryDirectory.filePath(QStringLiteral("conductor_tags.h5"));
    Hdf5ModelIO hdf5;
    StructureData restored;
    if (!temporaryDirectory.isValid()
        || !hdf5.ExportModelHdf5(h5Path, structure.get(), QStringLiteral("verification"))
        || !hdf5.ImportHdf5(h5Path, &restored)
        || restored.m_Elements.size() != structure->m_Elements.size())
        return 20;
    for (const auto& [elementId, source] : structure->m_Elements)
    {
        const auto target = restored.FindElement(elementId);
        if (!source || !target
            || target->m_Role != source->m_Role
            || target->m_WireId != source->m_WireId
            || target->m_AeroProfileId != source->m_AeroProfileId)
            return 21;
    }

    QTextStream(stdout)
        << "conductor bundle nodes=" << structure->m_Nodes.size()
        << " elements=" << structure->m_Elements.size()
        << " sets=" << structure->m_ModelSets.size()
        << " supports=" << result.leftSupportNodeId << "," << result.rightSupportNodeId
        << Qt::endl;
    return 0;
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
    if (const auto result = verifyHdf5ModelContract(arguments))
        return result;
    if (const auto result = verifyComputeRegions(arguments))
        return result;
    if (const auto result = verifyConductorBundle(arguments))
        return result;
    if (const auto result = verifySolveTask(application, arguments))
        return result;
    if (const auto result = verifyPartialResult(application, arguments))
        return result;
    return verifyModelImport(application, arguments);
}
