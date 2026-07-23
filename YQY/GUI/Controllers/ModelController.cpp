#include "Controllers/ModelController.h"

#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include "Import/Input_Model.h"

#include <QElapsedTimer>
#include <QFileInfo>
#include <QtConcurrent/QtConcurrentRun>

#include <cmath>
#include <exception>

ModelController::ModelController(QObject* parent)
    : QObject(parent)
{
    connect(&m_loadWatcher, &QFutureWatcher<LoadResult>::finished,
        this, &ModelController::finishLoading);
}

ModelController::~ModelController()
{
    m_loadWatcher.disconnect(this);
    m_loadWatcher.cancel();
}

bool ModelController::loadModel(const QString& filePath)
{
    if (filePath.trimmed().isEmpty())
        return false;

    const QString absolutePath = QFileInfo(filePath).absoluteFilePath();
    for (auto it = m_documents.cbegin(); it != m_documents.cend(); ++it)
    {
        if (QFileInfo(it->filePath) == QFileInfo(absolutePath))
        {
            setActiveModel(it.key());
            return true;
        }
    }
    if (m_pendingFiles.contains(absolutePath))
        return true;

    const bool wasIdle = !isLoading();
    m_pendingFiles.enqueue(absolutePath);
    if (wasIdle)
        emit busyChanged(true);
    startNextLoading();
    return true;
}

int ModelController::loadModels(const QStringList& filePaths)
{
    int accepted = 0;
    for (const QString& filePath : filePaths)
    {
        if (loadModel(filePath))
            ++accepted;
    }
    return accepted;
}

int ModelController::adoptModel(
    const std::shared_ptr<StructureData>& structure, const QString& filePath, qint64 elapsedMs)
{
    if (!structure || structure->m_Nodes.empty() || filePath.trimmed().isEmpty())
        return -1;

    structure->EnsureDefaultAnalysisConfiguration();

    const QString absolutePath = QFileInfo(filePath).absoluteFilePath();
    for (auto it = m_documents.begin(); it != m_documents.end(); ++it)
    {
        if (QFileInfo(it->filePath) != QFileInfo(absolutePath))
            continue;

        it->structure = structure;
        it->editHistory.clear();
        if (m_activeModelId == it.key())
            emit activeModelChanged(it.key());
        else
            setActiveModel(it.key());
        return it.key();
    }

    ModelDocument document;
    document.id = m_nextModelId++;
    document.filePath = absolutePath;
    document.structure = structure;
    const int modelId = document.id;
    m_documents.insert(modelId, std::move(document));
    emit busyChanged(true);
    emit loadStarted(absolutePath);
    emit loadSucceeded(modelId, absolutePath, elapsedMs);
    setActiveModel(modelId);
    emit busyChanged(false);
    return modelId;
}

void ModelController::startNextLoading()
{
    if (m_loadWatcher.isRunning() || m_pendingFiles.isEmpty())
        return;

    const QString absolutePath = m_pendingFiles.dequeue();
    emit loadStarted(absolutePath);

    m_loadWatcher.setFuture(QtConcurrent::run([absolutePath]() {
        LoadResult result;
        result.filePath = absolutePath;
        QElapsedTimer timer;
        timer.start();

        try
        {
            auto structure = std::make_shared<StructureData>();
            if (QFileInfo(absolutePath).suffix().compare(QStringLiteral("h5"), Qt::CaseInsensitive) == 0
                || QFileInfo(absolutePath).suffix().compare(QStringLiteral("hdf5"), Qt::CaseInsensitive) == 0)
            {
                Hdf5ModelIO importer;
                result.success = importer.ImportHdf5(absolutePath, structure.get());
                if (!result.success)
                {
                    result.errorMessage = QStringLiteral("H5模型格式不符合YQY统一模型协议。");
                }
            }
            else
            {
                Input_Model importer;
                result.success = importer.InputData(absolutePath, structure);
                if (!result.success)
                {
                    result.errorMessage = importer.LastError();
                }
            }
            if (result.success && structure->m_Nodes.empty())
            {
                result.success = false;
                result.errorMessage = QStringLiteral("文件中没有读取到有效节点。");
            }
            if (result.success)
                result.structure = std::move(structure);
            else if (result.errorMessage.isEmpty())
                result.errorMessage = QStringLiteral("模型格式不受支持或文件内容不完整。");
        }
        catch (const std::exception& exception)
        {
            result.errorMessage = QString::fromUtf8(exception.what());
        }
        catch (...)
        {
            result.errorMessage = QStringLiteral("读取模型时发生未知异常。");
        }

        result.elapsedMs = timer.elapsed();
        return result;
    }));
}

bool ModelController::updateNodePosition(int nodeId, double x, double y, double z)
{
    auto document = m_documents.find(m_activeModelId);
    if (document == m_documents.end() || !document->structure ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        return false;

    const auto node = document->structure->FindNode(nodeId);
    if (!node)
        return false;

    if (node->m_X == x && node->m_Y == y && node->m_Z == z)
        return true;

    NodeEdit edit;
    edit.nodeId = nodeId;
    edit.before[0] = node->m_X;
    edit.before[1] = node->m_Y;
    edit.before[2] = node->m_Z;
    edit.after[0] = x;
    edit.after[1] = y;
    edit.after[2] = z;
    document->editHistory.push_back(edit);

    node->m_X = x;
    node->m_Y = y;
    node->m_Z = z;
    emit nodePositionChanged(nodeId, x, y, z);
    emit undoAvailabilityChanged(true);
    return true;
}

bool ModelController::undoLastChange()
{
    auto document = m_documents.find(m_activeModelId);
    if (document == m_documents.end() || !document->structure || document->editHistory.empty())
        return false;

    const NodeEdit edit = document->editHistory.back();
    document->editHistory.pop_back();
    const auto node = document->structure->FindNode(edit.nodeId);
    if (!node)
        return false;

    node->m_X = edit.before[0];
    node->m_Y = edit.before[1];
    node->m_Z = edit.before[2];
    emit nodePositionChanged(edit.nodeId, edit.before[0], edit.before[1], edit.before[2]);
    emit undoAvailabilityChanged(!document->editHistory.empty());
    return true;
}

void ModelController::clearModel()
{
    closeModel(m_activeModelId);
}

bool ModelController::closeModel(int modelId)
{
    if (!m_documents.contains(modelId))
        return false;

    const bool wasActive = modelId == m_activeModelId;
    m_documents.remove(modelId);
    emit modelClosed(modelId);

    if (!wasActive)
        return true;

    m_activeModelId = -1;
    if (!m_documents.isEmpty())
    {
        setActiveModel(m_documents.firstKey());
    }
    else
    {
        emit undoAvailabilityChanged(false);
        emit modelCleared();
    }
    return true;
}

bool ModelController::setActiveModel(int modelId)
{
    const auto found = m_documents.constFind(modelId);
    if (found == m_documents.cend())
        return false;
    if (m_activeModelId == modelId)
        return true;

    m_activeModelId = modelId;
    emit undoAvailabilityChanged(!found->editHistory.empty());
    emit activeModelChanged(modelId);
    return true;
}

std::shared_ptr<StructureData> ModelController::model() const
{
    return model(m_activeModelId);
}

std::shared_ptr<StructureData> ModelController::model(int modelId) const
{
    const auto found = m_documents.constFind(modelId);
    return found == m_documents.cend() ? nullptr : found->structure;
}

QString ModelController::filePath() const
{
    return filePath(m_activeModelId);
}

QString ModelController::filePath(int modelId) const
{
    const auto found = m_documents.constFind(modelId);
    return found == m_documents.cend() ? QString() : found->filePath;
}

QList<int> ModelController::modelIds() const
{
    return m_documents.keys();
}

int ModelController::activeModelId() const
{
    return m_activeModelId;
}

int ModelController::modelCount() const
{
    return m_documents.size();
}

bool ModelController::isLoading() const
{
    return m_loadWatcher.isRunning() || !m_pendingFiles.isEmpty();
}

void ModelController::finishLoading()
{
    const LoadResult result = m_loadWatcher.result();
    if (result.success && result.structure)
    {
        ModelDocument document;
        document.id = m_nextModelId++;
        document.filePath = result.filePath;
        document.structure = result.structure;
        const int modelId = document.id;
        m_documents.insert(modelId, std::move(document));
        emit loadSucceeded(modelId, result.filePath, result.elapsedMs);
        setActiveModel(modelId);
    }
    else
    {
        emit loadFailed(result.filePath, result.errorMessage);
    }
    if (!m_pendingFiles.isEmpty())
    {
        startNextLoading();
    }
    else
    {
        emit busyChanged(false);
    }
}
