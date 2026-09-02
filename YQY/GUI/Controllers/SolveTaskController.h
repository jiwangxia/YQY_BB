#pragma once

#include <QObject>
#include <QHash>
#include <QThreadPool>

#include <atomic>
#include <memory>

class StructureData;

// 管理多模型、多分析步的并行求解任务。
class SolveTaskController final : public QObject
{
    Q_OBJECT

public:
    static constexpr int MinimumThreadCount = 1;  // 最少并行线程数
    static constexpr int MaximumThreadCount = 12; // 最多并行线程数
    static constexpr int DefaultThreadCount = 5;  // 默认并行线程数

    enum class Status // 求解任务状态
    {
        Ready,
        Queued,
        Running,
        Completed,
        Failed,
        Cancelling,
        Cancelled
    };

    struct TaskInfo // 供界面展示的任务快照
    {
        int id = -1;                    // 任务编号
        QString name;                   // 任务显示名称
        QString sourceFile;             // 源模型路径
        QString outputFile;             // 结果文件路径
        QString message;                // 当前状态说明
        Status status = Status::Ready;  // 当前任务状态
        double progress = 0.0;          // 求解进度，范围 0 至 1
        qint64 elapsedMs = 0;           // 已耗时毫秒数
        int analysisStepId = 0;         // 对应分析步编号
        bool hasUsableResult = false;   // 是否可打开已有结果
        bool partialResult = false;     // 结果是否为中断后的部分结果
        qint64 resultFrameCount = 0;    // 已写入结果帧数量
        double resultEndTime = 0.0;     // 结果结束时间
    };

    explicit SolveTaskController(QObject* parent = nullptr); // 创建任务线程池
    ~SolveTaskController() override; // 取消并等待运行中的任务

    int submit(const std::shared_ptr<StructureData>& model, const QString& sourceFile); // 创建并启动全模型任务
    int prepare(const std::shared_ptr<StructureData>& model, const QString& sourceFile, int analysisStepId);
    // 创建指定分析步的待启动任务。
    void removeUnavailablePreparedTasks(const std::shared_ptr<StructureData>& model, const QString& sourceFile);
    bool start(int taskId); // 启动一个待执行任务
    int startAllReady(); // 启动所有可执行任务
    int restartAll(); // 重新启动全部任务
    bool cancel(int taskId); // 请求取消一个任务
    bool restart(int taskId); // 重新执行一个任务
    void cancelAll(); // 请求取消全部任务

    QList<int> taskIds() const; // 返回全部任务编号
    TaskInfo taskInfo(int taskId) const; // 返回一个任务快照
    int runningTaskCount() const; // 返回运行中任务数
    int maximumThreadCount() const; // 返回线程池上限
    void setMaximumThreadCount(int count); // 设置线程池上限
    static int availableThreadCount(); // 返回本机建议并行线程数
    static int maximumAllowedThreadCount(); // 返回允许的最大线程数
    static int defaultThreadCount(); // 返回默认线程数
    static QString statusText(Status status); // 返回状态显示文本

signals:
    void taskAdded(int taskId); // 新任务已创建
    void taskUpdated(int taskId); // 任务状态或进度已更新

private:
    struct TaskContext // 任务运行期间的内部状态
    {
        TaskInfo info;                         // 界面任务快照
        std::shared_ptr<StructureData> modelTemplate; // 求解前模型副本
        std::atomic<bool> cancelRequested{false}; // 取消请求标记
        std::atomic<qint64> startedAtMs{0};    // 开始时间戳
        std::atomic<qint64> lastProgressReportMs{0}; // 上次进度上报时间戳
        bool restartRequested = false;         // 完成后是否重新执行
        bool workerScheduled = false;          // 是否已提交到线程池
        std::shared_ptr<StructureData> solvedModel; // 已完成求解的模型
        qint64 previousOutputModifiedMs = -1;  // 重启前结果文件修改时间
        qint64 previousOutputSize = -1;        // 重启前结果文件大小
    };

    int initialStaticStepId(const std::shared_ptr<TaskContext>& task) const; // 查询首个静力分析步
    std::shared_ptr<TaskContext> dependencyTask(const std::shared_ptr<TaskContext>& task) const; // 查询前置任务
    void launchTask(const std::shared_ptr<TaskContext>& task); // 提交任务到线程池
    void releaseDependentTasks(const std::shared_ptr<TaskContext>& dependency, Status status); // 处理依赖任务
    void runTask(const std::shared_ptr<TaskContext>& task); // 在线程池中执行求解
    void reportProgress(int taskId, double progress, const QString& message); // 上报求解进度
    void finishTask(int taskId, Status status, const QString& message, qint64 elapsedMs = -1); // 完成任务收尾

    QThreadPool m_threadPool; // 并行求解线程池
    QHash<int, std::shared_ptr<TaskContext>> m_tasks; // 任务编号到运行状态的映射
    int m_nextTaskId = 1; // 下一个任务编号
};
