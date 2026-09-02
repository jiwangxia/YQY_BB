#pragma once

#include <QObject>
#include <QFutureWatcher>
#include <QMap>
#include <QQueue>
#include <QStringList>

#include <memory>
#include <vector>

class StructureData;

// 管理模型文档的异步加载、激活和节点坐标撤销。
class ModelController final : public QObject
{
    Q_OBJECT

public:
    explicit ModelController(QObject* parent = nullptr); // 创建模型文档控制器
    ~ModelController() override; // 确保后台加载任务结束

    bool loadModel(const QString& filePath); // 异步加载一个模型文件
    int loadModels(const QStringList& filePaths); // 将多个模型文件加入加载队列
    int adoptModel(const std::shared_ptr<StructureData>& structure, const QString& filePath, qint64 elapsedMs = 0);
    // 将已构建的模型纳入文档管理。
    bool updateNodePosition(int nodeId, double x, double y, double z); // 修改活动模型中的节点坐标
    bool undoLastChange(); // 撤销最近一次节点坐标修改
    void clearModel(); // 关闭全部模型文档
    bool closeModel(int modelId); // 关闭指定模型文档
    bool setActiveModel(int modelId); // 切换活动模型文档

    std::shared_ptr<StructureData> model() const; // 返回活动模型
    std::shared_ptr<StructureData> model(int modelId) const; // 返回指定模型
    QString filePath() const; // 返回活动模型文件路径
    QString filePath(int modelId) const; // 返回指定模型文件路径
    QList<int> modelIds() const; // 返回全部模型编号
    int activeModelId() const; // 返回活动模型编号
    int modelCount() const; // 返回已打开模型数量
    bool isLoading() const; // 返回是否仍有后台加载任务

signals:
    void loadStarted(const QString& filePath); // 开始加载模型
    void loadSucceeded(int modelId, const QString& filePath, qint64 elapsedMs); // 模型加载成功
    void loadFailed(const QString& filePath, const QString& errorMessage); // 模型加载失败
    void activeModelChanged(int modelId); // 活动模型已切换
    void modelClosed(int modelId); // 模型文档已关闭
    void nodePositionChanged(int nodeId, double x, double y, double z); // 节点坐标已修改
    void modelCleared(); // 全部模型已关闭
    void undoAvailabilityChanged(bool available); // 撤销能力发生变化
    void busyChanged(bool busy); // 后台加载状态发生变化

private:
    struct LoadResult // 一次后台模型加载的结果
    {
        QString filePath;                    // 请求加载的文件
        QString errorMessage;                // 失败原因
        std::shared_ptr<StructureData> structure; // 解析出的结构数据
        qint64 elapsedMs = 0;                // 加载耗时
        bool success = false;                // 是否加载成功
    };

    void startNextLoading(); // 启动队列中的下一个加载任务
    void finishLoading(); // 接收并处理当前加载任务结果

    struct NodeEdit // 节点坐标的一次可撤销修改
    {
        int nodeId = -1;                    // 被修改的节点编号
        double before[3] = {0.0, 0.0, 0.0}; // 修改前坐标
        double after[3] = {0.0, 0.0, 0.0};  // 修改后坐标
    };

    struct ModelDocument // 一个已打开的模型文档
    {
        int id = -1;                         // 文档编号
        QString filePath;                    // 模型来源路径
        std::shared_ptr<StructureData> structure; // 文档结构数据
        std::vector<NodeEdit> editHistory;   // 可撤销的节点修改记录
    };

    QFutureWatcher<LoadResult> m_loadWatcher; // 当前后台加载任务观察器
    QQueue<QString> m_pendingFiles;           // 待加载文件队列
    QMap<int, ModelDocument> m_documents;     // 已打开模型文档
    int m_activeModelId = -1;                 // 当前活动文档编号
    int m_nextModelId = 1;                    // 下一个文档编号
};
