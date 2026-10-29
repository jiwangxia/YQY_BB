#pragma once

#include <QObject>
#include <QHash>
#include <QElapsedTimer>
#include <QStringList>
#include <QThreadPool>

#include <atomic>
#include <map>
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
        int id = -1;                          // 任务编号
        QString name;                         // 任务显示名称
        QString sourceFile;                   // 源模型路径
        QString outputFile;                   // 结果文件路径
        QString message;                      // 当前状态说明
        QString diagnostics;                  // 完成后的分行诊断，供界面按需展开
        Status status = Status::Ready;        // 当前任务状态
        double progress = 0.0;                // 求解进度，范围 0 至 1
        qint64 elapsedMs = 0;                 // 已耗时毫秒数
        qint64 solverElapsedMs = 0;           // 求解器内部耗时，不含排队与结果加载
        qint64 workerProcessId = 0;           // 独立计算进程编号，线程模式为零
        bool resultLoadRecorded = false;      // 结果是否已完成自动加载
        int analysisStepId = 0;               // 对应分析步编号
        int initialStaticTaskId = -1;         // 独立前置静力任务编号
        bool waitingForInitialStatic = false; // 正在等待关联静力初态
        bool initialStatePreparation = false; // 内部初态准备任务，不单列为用户分析步
        bool hasUsableResult = false;         // 是否可打开已有结果
        bool partialResult = false;           // 结果是否为中断后的部分结果
        qint64 resultFrameCount = 0;          // 已写入结果帧数量
        double resultEndTime = 0.0;           // 结果结束时间
    };

    explicit SolveTaskController(QObject* parent = nullptr);                                                     // 创建任务线程池
    ~SolveTaskController() override;                                                                             // 取消并等待运行中的任务

    int submit(const std::shared_ptr<StructureData>& model, const QString& sourceFile);                          // 创建并启动全模型任务
    int prepare(const std::shared_ptr<StructureData>& model, const QString& sourceFile, int analysisStepId);     // 为指定分析步创建待执行任务
    // 创建指定分析步的待启动任务。
    void removeUnavailablePreparedTasks(const std::shared_ptr<StructureData>& model, const QString& sourceFile); // 移除模型中已失效的待执行任务
    bool start(int taskId);                                                                                      // 启动一个待执行任务
    void setIsolatedProcessEnabled(bool enabled);                                                                // 试验性启用完整算例的独立进程执行
    void recordResultLoaded(int taskId);                                                                         // 结果及首帧显示完成后记录端到端耗时
    int startAllReady();                                                                                         // 启动待运行任务，返回用户分析步数量
    int restartAll();                                                                                            // 重启全部任务，返回用户分析步数量
    bool cancel(int taskId);                                                                                     // 请求取消一个任务
    bool restart(int taskId);                                                                                    // 重新执行一个任务
    void cancelAll();                                                                                            // 请求取消全部任务

    QList<int> taskIds() const;                                                                                  // 返回全部任务编号
    QList<int> analysisTaskIds() const;                                                                          // 返回用户分析步任务，排除内部初态准备
    TaskInfo taskInfo(int taskId) const;                                                                         // 返回一个任务快照
    TaskInfo displayTaskInfo(int taskId) const;                                                                  // 展示关联无风初态；返回的 id 可能是内部执行任务
    int runningTaskCount() const;                                                                                // 返回全部运行中任务数，包含初态准备
    int maximumThreadCount() const;                                                                              // 返回线程池上限
    void setMaximumThreadCount(int count);                                                                       // 设置线程池上限
    static int availableThreadCount();                                                                           // 返回本机建议并行线程数
    static int maximumAllowedThreadCount();                                                                      // 返回允许的最大线程数
    static int defaultThreadCount();                                                                             // 返回默认线程数
    static QString statusText(Status status);                                                                    // 返回状态显示文本

signals:
    void taskAdded(int taskId);                                                                                  // 新任务已创建
    void taskUpdated(int taskId);                                                                                // 任务状态或进度已更新

private:
    struct TaskContext // 任务运行期间的内部状态
    {
        TaskInfo info;                                                    // 界面任务快照
        std::shared_ptr<StructureData> modelTemplate;                     // 求解前模型副本
        std::atomic<bool> cancelRequested{false};                         // 取消请求标记
        QElapsedTimer totalTimer;                                         // 从点击启动至结果可用的单调时钟
        std::atomic<qint64> lastProgressReportMs{0};                      // 上次进度上报时间戳
        bool restartRequested = false;                                    // 完成后是否重新执行
        bool workerScheduled = false;                                     // 是否已提交到线程池
        bool isolatedProcess = false;                                     // 提交时冻结的执行方式
        std::shared_ptr<StructureData> solvedModel;                       // 已完成求解的模型
        std::map<int, std::shared_ptr<StructureData>> solvedRegionStates; // 各区域已收敛状态
        bool referenceGravityEquilibrium = false;                         // 参考动力分支的无风静力任务
        qint64 previousOutputModifiedMs = -1;                             // 重启前结果文件修改时间
        qint64 previousOutputSize = -1;                                   // 重启前结果文件大小
        int executionThreadBudget = 1;                                    // 当前任务可使用的 CPU 线程预算
        int assemblyThreadLimit = 1;                                      // 当前任务单元装配的线程上限
    };

    int prepareTask(const std::shared_ptr<StructureData>& model, const QString& sourceFile, int analysisStepId,
                    bool referenceGravityEquilibrium);                                                           // 创建分析任务及所需的重力平衡准备任务
    int initialStaticStepId(const std::shared_ptr<TaskContext>& task) const;                                     // 查询首个静力分析步
    std::shared_ptr<TaskContext> dependencyTask(const std::shared_ptr<TaskContext>& task) const;                 // 查询前置任务
    void launchTask(const std::shared_ptr<TaskContext>& task);                                                   // 提交任务到线程池
    void tryStartRestartBatch();                                                                                 // 旧批次停止后统一重启
    void releaseDependentTasks(const std::shared_ptr<TaskContext>& dependency, Status status);                   // 处理依赖任务
    void runTask(const std::shared_ptr<TaskContext>& task, const std::shared_ptr<StructureData>& initialStaticModel,
                 const std::map<int, std::shared_ptr<StructureData>>& initialRegionStates,
                 bool publishInitialStaticResults);                                                              // 使用提交时冻结的初态在线程池中执行求解
    void runIsolatedTask(const std::shared_ptr<TaskContext>& task);                                              // 在独立进程中执行完整算例
    void reportProgress(int taskId, double progress, const QString& message);                                    // 上报求解进度
    void finishTask(int taskId, Status status, const QString& message, qint64 elapsedMs = -1,
                    const QString& diagnostics = QString());                                                     // 完成任务收尾
    static QString completionMessage(Status status, const QString& message, const QString& gpuReport);           // 组合任务完成状态和 GPU 执行摘要
    static QString gpuDiagnostics(const QString& report);                                                        // 整理 GPU 求解诊断信息
    static QString stageDiagnostics(const QString& report);                                                      // 整理各求解阶段的诊断信息
    static QString completionDiagnostics(const QString& outputFile, qint64 solverElapsedMs, const QString& gpuReport,
                                         const QString& stageReport, double peakMemoryMegabytes,
                                         qint64 workerProcessId);                                                // 汇总输出文件、耗时、内存和执行诊断
    int executionThreadBudgetPerTask() const;                                                                    // 计算每个并发任务的稳定 CPU 预算

    QThreadPool m_threadPool;                         // 并行求解线程池
    QHash<int, std::shared_ptr<TaskContext>> m_tasks; // 任务编号到运行状态的映射
    QList<int> m_restartBatch;                        // 等待旧 worker 全部停止后重算的用户任务及依赖
    bool m_stagingRestartBatch = false;               // 阻止同步取消回调提前启动新批次
    int m_nextTaskId = 1;                             // 下一个任务编号
    bool m_isolatedProcessEnabled = false;            // 独立进程模式需显式启用
};
