#pragma once

#include <QObject>
#include <QHash>
#include <QThreadPool>

#include <atomic>
#include <memory>

class StructureData;

class SolveTaskController final : public QObject
{
    Q_OBJECT

public:
    static constexpr int MinimumThreadCount = 1;
    static constexpr int MaximumThreadCount = 12;
    static constexpr int DefaultThreadCount = 5;

    enum class Status
    {
        Ready,
        Queued,
        Running,
        Completed,
        Failed,
        Cancelling,
        Cancelled
    };

    struct TaskInfo
    {
        int id = -1;
        QString name;
        QString sourceFile;
        QString outputFile;
        QString message;
        Status status = Status::Ready;
        double progress = 0.0;
        qint64 elapsedMs = 0;
        int analysisStepId = 0;
        bool hasUsableResult = false;
        bool partialResult = false;
        qint64 resultFrameCount = 0;
        double resultEndTime = 0.0;
    };

    explicit SolveTaskController(QObject* parent = nullptr);
    ~SolveTaskController() override;

    int submit(const std::shared_ptr<StructureData>& model, const QString& sourceFile);
    int prepare(const std::shared_ptr<StructureData>& model, const QString& sourceFile, int analysisStepId);
    void removeUnavailablePreparedTasks(const std::shared_ptr<StructureData>& model, const QString& sourceFile);
    bool start(int taskId);
    int startAllReady();
    int restartAll();
    bool cancel(int taskId);
    bool restart(int taskId);
    void cancelAll();

    QList<int> taskIds() const;
    TaskInfo taskInfo(int taskId) const;
    int runningTaskCount() const;
    int maximumThreadCount() const;
    void setMaximumThreadCount(int count);
    static int availableThreadCount();
    static int maximumAllowedThreadCount();
    static int defaultThreadCount();
    static QString statusText(Status status);

signals:
    void taskAdded(int taskId);
    void taskUpdated(int taskId);

private:
    struct TaskContext
    {
        TaskInfo info;
        std::shared_ptr<StructureData> modelTemplate;
        std::atomic<bool> cancelRequested{ false };
        std::atomic<qint64> startedAtMs{ 0 };
        std::atomic<qint64> lastProgressReportMs{ 0 };
        bool restartRequested = false;
        qint64 previousOutputModifiedMs = -1;
        qint64 previousOutputSize = -1;
    };

    void runTask(const std::shared_ptr<TaskContext>& task);
    void reportProgress(int taskId, double progress, const QString& message);
    void finishTask(int taskId, Status status, const QString& message, qint64 elapsedMs = -1);

    QThreadPool m_threadPool;
    QHash<int, std::shared_ptr<TaskContext>> m_tasks;
    int m_nextTaskId = 1;
};
