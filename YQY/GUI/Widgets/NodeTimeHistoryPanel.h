#pragma once

#include "Export/Hdf5ResultData.h"

#include <array>
#include <functional>
#include <memory>
#include <vector>

#include <QWidget>

class QCheckBox;
class QComboBox;
class QFutureWatcherBase;
class QVBoxLayout;
class QLabel;
class QPushButton;
class QResizeEvent;
class QScrollArea;
class QSpinBox;
class NodeTimeHistoryChart;
class SingleLineLabel;

class NodeTimeHistoryPanel final : public QWidget
{
public:
    explicit NodeTimeHistoryPanel(QWidget* parent = nullptr);    // 创建节点与能量时程绘图面板

    void setResultFile(const QString& resultFile);               // 设置节点时程查询的结果文件
    void setResultContext(const QString& resultFile, const std::vector<Hdf5ResultFrameInfo>& frames,
                          int currentStepId, bool hasEnergyRecords); // 更新完整结果上下文并清除旧查询缓存
    void setAnalysisStep(int stepId);                            // 同步能量分析步，显示时自动刷新
    void showEnergyHistory(int stepId);                          // 切换到指定分析步的能量时程
    void showNodeHistory(int nodeId = -1);                       // 切换到节点时程，有节点编号时直接绘制
    void showTimeHistory(int nodeId = -1);                       // 打开统一时程入口并保留当前绘图类型
    void exportEnergyHistory();                                 // 导出当前分析步的能量，无需先绘图
    void setNodeId(int nodeId);                                  // 设置待查询的节点编号
    void drawCurrentNode();                                      // 绘制当前节点选定分量的时程曲线
    void clearHistory();                                         // 清空时程曲线和查询状态
    void setReturnToModelHandler(std::function<void()> handler); // 设置返回模型视图的回调
    void setEnergyStepChangedHandler(std::function<void(int)> handler); // 同步用户选择的能量分析步
    void setEnergyExportStateHandler(std::function<void(bool, bool)> handler); // 通知侧栏导出可用性和忙碌状态

protected:
    void resizeEvent(QResizeEvent* event) override;              // 窄窗口将查询控件换行
    bool eventFilter(QObject* watched, QEvent* event) override; // 查询栏宽度变化后重新换行

private:
    unsigned int selectedEnergyMask() const;                    // 能量模式使用勾选项，节点模式使用全部分量
    bool canExportEnergy() const;                               // 检查当前结果和所选分量能否发起导出
    void updateEnergyExportState();                             // 更新外部侧栏的能量导出按钮状态
    bool m_energyExportInProgress = false;
    struct LoadResult
    {
        int nodeId = -1;
        int stepId = 0;
        bool energy = false;
        QString error;
        QString emptyMessage;
        unsigned int availableEnergyMask = 0;
        std::vector<int> energyStepIds;
        std::shared_ptr<const std::vector<Hdf5NodalDisplacementHistorySample>> samples;
        std::shared_ptr<const std::vector<Hdf5EnergyHistorySample>> energySamples;
    };

    std::array<bool, 6> selectedComponents() const;              // 读取当前绘图类型的勾选分量
    void updatePlotType();                                      // 切换节点或能量查询控件
    void updateQueryLayout();                                   // 按可用宽度排列查询控件
    int resolvedAnalysisStep(int stepId) const;                 // 将全部分析步解析为有效的单个分析步
    void resetCache();                                          // 释放上一个结果上下文的缓存
    void updateEnergyAvailability(unsigned int mask, bool known); // 标明当前分析步支持的能量分量
    QString energyStatus(int stepId, std::size_t sampleCount) const; // 生成当前能量时程状态
    void invalidateQuery();                                     // 输入改变后使旧的异步查询失效
    void setStatus(const QString& text, bool error = false);     // 更新状态文字与错误颜色
    void requestHistory();                                      // 后台读取时程，保留最新的查询请求
    void applyHistory(const LoadResult& result);                 // 将读取结果应用到曲线与状态显示

    QSpinBox* m_nodeId = nullptr;
    QScrollArea* m_scrollArea = nullptr;
    QWidget* m_queryHost = nullptr;
    QVBoxLayout* m_queryLayout = nullptr;
    std::vector<int> m_queryRows;
    bool m_queryEnergy = false;
    QLabel* m_nodeLabel = nullptr;
    QComboBox* m_plotType = nullptr;
    QPushButton* m_drawButton = nullptr;
    QPushButton* m_returnButton = nullptr;
    SingleLineLabel* m_status = nullptr;
    std::array<QCheckBox*, 6> m_componentChecks{};
    QWidget* m_nodeQueryPanel = nullptr;
    QWidget* m_energyQueryPanel = nullptr;
    QComboBox* m_energyStep = nullptr;
    std::array<QCheckBox*, 4> m_energyChecks{};
    NodeTimeHistoryChart* m_chart = nullptr;
    QFutureWatcherBase* m_loader = nullptr;
    QString m_resultFile;
    QString m_cachedFile;
    int m_cachedNodeId = -1;
    int m_cachedStepId = 0;
    int m_requestSerial = 0;
    bool m_queryPending = false;
    bool m_energyAvailabilityKnown = false;
    bool m_hasEnergyRecords = true;
    unsigned int m_availableEnergyMask = 0;
    std::shared_ptr<const std::vector<Hdf5NodalDisplacementHistorySample>> m_cachedSamples;
    std::shared_ptr<const std::vector<Hdf5EnergyHistorySample>> m_cachedEnergySamples;
    std::function<void()> m_returnToModelHandler;
    std::function<void(int)> m_energyStepChangedHandler;
    std::function<void(bool, bool)> m_energyExportStateHandler;
};
