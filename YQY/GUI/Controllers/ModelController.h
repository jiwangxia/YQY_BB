#pragma once

#include <QObject>
#include <QFutureWatcher>
#include <QMap>
#include <QQueue>
#include <QStringList>

#include <memory>
#include <vector>

class StructureData;

class ModelController final : public QObject
{
    Q_OBJECT

public:
    explicit ModelController(QObject* parent = nullptr);
    ~ModelController() override;

    bool loadModel(const QString& filePath);
    int loadModels(const QStringList& filePaths);
    int adoptModel(const std::shared_ptr<StructureData>& structure, const QString& filePath, qint64 elapsedMs = 0);
    bool updateNodePosition(int nodeId, double x, double y, double z);
    bool undoLastChange();
    void clearModel();
    bool closeModel(int modelId);
    bool setActiveModel(int modelId);

    std::shared_ptr<StructureData> model() const;
    std::shared_ptr<StructureData> model(int modelId) const;
    QString filePath() const;
    QString filePath(int modelId) const;
    QList<int> modelIds() const;
    int activeModelId() const;
    int modelCount() const;
    bool isLoading() const;

signals:
    void loadStarted(const QString& filePath);
    void loadSucceeded(int modelId, const QString& filePath, qint64 elapsedMs);
    void loadFailed(const QString& filePath, const QString& errorMessage);
    void activeModelChanged(int modelId);
    void modelClosed(int modelId);
    void nodePositionChanged(int nodeId, double x, double y, double z);
    void modelCleared();
    void undoAvailabilityChanged(bool available);
    void busyChanged(bool busy);

private:
    struct LoadResult
    {
        QString filePath;
        QString errorMessage;
        std::shared_ptr<StructureData> structure;
        qint64 elapsedMs = 0;
        bool success = false;
    };

    void startNextLoading();
    void finishLoading();

    struct NodeEdit
    {
        int nodeId = -1;
        double before[3] = {0.0, 0.0, 0.0};
        double after[3] = {0.0, 0.0, 0.0};
    };

    struct ModelDocument
    {
        int id = -1;
        QString filePath;
        std::shared_ptr<StructureData> structure;
        std::vector<NodeEdit> editHistory;
    };

    QFutureWatcher<LoadResult> m_loadWatcher;
    QQueue<QString> m_pendingFiles;
    QMap<int, ModelDocument> m_documents;
    int m_activeModelId = -1;
    int m_nextModelId = 1;
};
