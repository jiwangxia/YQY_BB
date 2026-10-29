#pragma once

#include <array>
#include <QtWidgets/QMainWindow>
#include "Base/EmptyOUT.h"
#include "Export/Hdf5ResultData.h"

class ModelViewport;
class NavigationButton;
class QCheckBox;
class QComboBox;
class QFrame;
class QGridLayout;
class QHBoxLayout;
class QLabel;
class QPlainTextEdit;
class QProgressBar;
class QSplitter;
class QStackedWidget;
class QTabBar;
class QTabWidget;
class QToolButton;
class QTreeWidget;
class QVBoxLayout;
class QWidget;
class SingleLineLabel;
class WorkspaceSplitter;

class StructureData;
class ModelController;
class SolveTaskController;
class ResultDataController;
class WorkspaceController;
class QEvent;
class QResizeEvent;
class QPushButton;
class QScrollArea;
class QTreeWidgetItem;
class AnalysisManagerDialog;
class SettingsPanel;
class ResultControlPanel;
class NodeTimeHistoryPanel;
class AnalysisSummaryPanel;
class PropertyModule;
class ConductorModule;
class SolveTaskManagerDialog;
namespace Conductor
{
class PropertyLibrary;
}

class WindowAudit;

class YQY final : public QMainWindow
{
    Q_OBJECT

public:
    explicit YQY(QWidget* parent = nullptr);                                                                    // 创建主窗口并初始化各功能模块
    ~YQY() override;                                                                                            // 销毁主窗口
    bool openModel(const QString& filePath);                                                                    // 从指定路径打开模型文档
    bool openResult(const QString& filePath);                                                                   // 从指定路径打开结果文件

protected:
    bool event(QEvent* event) override;                                                                         // 处理主窗口事件并触发布局更新
    void resizeEvent(QResizeEvent* event) override;                                                             // 在主窗口尺寸变化时调整布局

private:
    friend class WindowAudit;

    enum class Module
    {
        Model = 0,
        Properties,
        Analysis,
        Solve,
        Result,
        Settings
    };

    void initializeEmptyState();                                                                                // 初始化尚未加载模型时的界面状态
    void initializeToolbarAppearance();                                                                         // 设置工具栏按钮的图标和外观
    void initializeConductorModule();                                                                           // 创建导线面板并连接建模操作
    void initializePropertyModule();                                                                            // 创建属性面板并连接编辑操作
    void initializeResultModule();                                                                              // 创建结果面板并连接播放与导出操作
    void initializeSettingsModule();                                                                            // 创建设置面板并绑定运行参数
    void updateToolbarIcons(int themeIndex);                                                                    // 按当前主题更新工具栏图标
    void initializeInteractions();                                                                              // 连接导航、模型操作和控制器事件
    void initializeResponsiveLayout();                                                                          // 配置主窗口的自适应布局规则
    void updateResponsiveLayout();                                                                              // 按可用空间调整界面布局
    void updateSolveBatchActionsLayout();                                                                       // 按可用宽度排列求解批量操作按钮
    void updateMonitorPanelHeight();                                                                            // 调整求解监视和日志面板高度
    void appendLogMessage(const QString& message, bool diagnostic = false);                                     // 追加运行日志并区分详细诊断信息
    void rebuildLogView();                                                                                      // 按当前诊断显示选项重建日志内容
    static QString elapsedTimeText(qint64 milliseconds);                                                        // 将毫秒耗时转换为界面显示文本
    void applyTheme(int themeIndex);                                                                            // 应用所选界面主题并同步各模块外观
    void switchModule(Module module);                                                                           // 切换主窗口当前功能模块
    void refreshModulePages();                                                                                  // 刷新各功能模块的内容和可用状态
    void openHdf5Result();                                                                                      // 选择并打开 HDF5 结果文件
    bool loadHdf5Result(const QString& filePath, bool showErrors = true, bool activateResultModule = true,
                        bool partialResult = false, QString* failureReason = nullptr, bool reportToLog = true); // 加载结果文件并更新显示、错误提示和日志
    void refreshResultCases(int preferredTaskId = -1);                                                          // 刷新可查看结果的任务列表
    void loadSolveTaskResult(int taskId);                                                                       // 加载指定求解任务的结果
    bool resultTaskBelongsToActiveModel(const QString& sourceFile) const;                                       // 检查结果任务是否属于当前活动模型
    void refreshResultSteps(int preferredStepId = 0);                                                           // 刷新结果分析步列表并恢复指定选择
    void applyResultStepFilter(int stepId);                                                                     // 按分析步筛选可显示的结果帧
    bool readResultFrame(int frameIndex, _OUT Hdf5ResultFrame& frame);                                          // 读取指定结果帧并报告读取状态
    const Hdf5ResultFrame* cachedResultFrame(int frameIndex);                                                   // 从缓存获取结果帧，缺失时读取
    void clearResultFrameCache();                                                                               // 清空结果帧缓存
    void displayResultFrame(int frameIndex);                                                                    // 显示指定编号的结果帧
    void displayResultPosition(double framePosition);                                                           // 按连续帧位置插值显示动画结果
    void updateResultVisualization();                                                                           // 按当前结果场和比例刷新三维显示
    void clearActiveResultContext();                                                                            // 清除当前结果文件、帧和界面状态
    bool loadModelFile(const QString& filePath);                                                                // 提交模型文件加载并更新界面状态
    void handleModelLoaded(int modelId, const QString& filePath, qint64 elapsedMs);                             // 处理模型加载成功并更新文档与统计
    void handleModelLoadFailed(const QString& filePath, const QString& errorMessage);                           // 显示模型加载错误并更新统计
    void handleActiveModelChanged(int modelId);                                                                 // 同步活动模型切换后的各模块状态
    void populateModelDataTree(const QString& fileName);                                                        // 根据活动模型重建模型数据树
    void setModelControlsEnabled(bool enabled);                                                                 // 统一设置模型相关操作的可用状态
    void showNodeProperties(int nodeId);                                                                        // 显示指定节点的属性
    void showElementProperties(int elementId);                                                                  // 显示指定单元的属性
    void clearSelectionProperties();                                                                            // 清除当前选中实体的属性显示
    void updateLoadStatistics();                                                                                // 更新模型加载数量、成功和失败统计
    void setWorkspaceMessage(const QString& message);                                                           // 更新工作区状态提示
    void submitSolveTask();                                                                                     // 提交当前模型的分析求解任务
    void prepareAnalysisCases(const std::shared_ptr<StructureData>& structure, const QString& sourceFile,
                              int onlyStepId = 0);                                                              // 为模型中的指定或全部分析步准备任务
    void cancelSelectedSolveTask();                                                                             // 请求取消当前选中的求解任务
    void refreshSolveTasks(int preferredTaskId = -1);                                                           // 刷新求解任务列表并恢复指定选择
    void updateSolveTaskRow(int taskId);                                                                        // 刷新指定求解任务的表格内容
    void updateTaskMonitor(int taskId);                                                                         // 更新指定任务的进度和收敛监视信息
    int selectedSolveTaskId() const;                                                                            // 读取当前选中的求解任务编号
    void openSolveTaskManager();                                                                                // 打开求解任务管理窗口
    void startAllReadySolveTasks();                                                                             // 启动全部已准备好的求解任务
    void restartAllSolveTasks();                                                                                // 重新执行全部求解任务
    void refreshSolveTaskManager();                                                                             // 刷新任务管理窗口中的任务信息
    void initializeAnalysisEditor();                                                                            // 连接分析资源概览与编辑入口
    void refreshAnalysisEditor();                                                                               // 刷新分析步、荷载和约束概览
    void openAnalysisManager(int initialPage);                                                                  // 打开指定类别的分析资源管理器
    void openComputeRegionManager();                                                                            // 打开计算区域管理窗口
    void handleAnalysisResourcesChanged(const QSet<int>& affectedStepIds);                                      // 按分析资源变更刷新受影响的任务与界面
    void closeAnalysisManagers();                                                                               // 关闭当前模型的分析资源管理窗口
    void showPropertyLibrary();                                                                                 // 打开材料与截面属性库
    void createConductorModel();                                                                                // 生成导线模型并加入当前工作区
    void exportNodeResults();                                                                                   // 选择并导出节点结果数据
    void exportElementResults();                                                                                // 选择并导出单元结果数据
    void exportIterationHistory();                                                                              // 导出求解迭代历史
    void exportResultPlot();                                                                                    // 选择并导出节点时程曲线
    void showTimeHistory();                                                                                     // 打开可切换节点位移与能量的时程面板
    void refreshPropertyModule();                                                                               // 刷新当前模型的材料、截面和弹簧属性
    void applyModelPropertyEdits();                                                                             // 将属性面板中的修改应用到模型
    void editPropertyItem(QTreeWidgetItem* item, int column);                                                   // 打开所选材料或截面的属性编辑窗口

    void initializeUi();                                                                                        // 创建界面控件并组织布局
    void initializeHeader(QWidget* centralWidget, QVBoxLayout* rootLayout);                                     // 创建主窗口标题与主题选择区域
    void initializeNavigation(QWidget* centralWidget, QHBoxLayout* bodyLayout);                                 // 创建功能模块导航栏
    void initializeModelPage();                                                                                 // 创建模型信息与操作页面
    void initializeAnalysisPage();                                                                              // 创建分析资源概览页面
    void initializeSolvePage();                                                                                 // 创建求解任务与批量操作页面
    void initializeResultPage();                                                                                // 创建结果文件与分析步选择页面
    void initializeSettingsPage();                                                                              // 创建程序设置页面容器
    void initializeViewportToolbar();                                                                           // 创建三维视口操作工具栏
    void initializeMonitorPages();                                                                              // 创建求解监视与日志选项卡
    void initializeStatusBar(QWidget* centralWidget, QVBoxLayout* rootLayout);                                  // 创建加载统计与工作状态显示区域
    void initializeProjectPanel();                                                                              // 组织左侧功能模块面板
    void initializeWorkspacePanel();                                                                            // 组织文档标签、视口和监视面板

    QLabel* m_appTitleLabel = nullptr;
    QLabel* m_themeLabel = nullptr;
    QComboBox* m_themeComboBox = nullptr;
    QFrame* m_navigationRail = nullptr;
    NavigationButton* m_modelButton = nullptr;
    NavigationButton* m_propertyButton = nullptr;
    NavigationButton* m_analysisButton = nullptr;
    NavigationButton* m_solveButton = nullptr;
    NavigationButton* m_resultButton = nullptr;
    NavigationButton* m_settingsButton = nullptr;
    QSplitter* m_mainSplitter = nullptr;
    QFrame* m_projectPanel = nullptr;
    QLabel* m_projectTitleLabel = nullptr;
    QStackedWidget* m_moduleStack = nullptr;
    QLabel* m_modelFileValue = nullptr;
    QLabel* m_modelNodeValue = nullptr;
    QLabel* m_modelElementValue = nullptr;
    QLabel* m_modelPropertyValue = nullptr;
    QTreeWidget* m_modelDataTree = nullptr;
    QPushButton* m_modelImportButton = nullptr;
    QPushButton* m_createConductorButton = nullptr;
    QWidget* m_analysisModulePage = nullptr;
    QVBoxLayout* m_analysisModuleLayout = nullptr;
    QTreeWidget* m_analysisTree = nullptr;
    QWidget* m_solveModulePage = nullptr;
    QLabel* m_solveReadinessLabel = nullptr;
    QGridLayout* m_solveBatchActionsLayout = nullptr;
    QTreeWidget* m_solveStepTree = nullptr;
    QPushButton* m_solvePageStartButton = nullptr;
    QWidget* m_resultModulePage = nullptr;
    QVBoxLayout* m_resultModuleLayout = nullptr;
    QPushButton* m_openH5Button = nullptr;
    QComboBox* m_resultCaseCombo = nullptr;
    SingleLineLabel* m_resultFileLabel = nullptr;
    QTreeWidget* m_resultTree = nullptr;
    QWidget* m_settingsModulePage = nullptr;
    QVBoxLayout* m_settingsModuleLayout = nullptr;
    QFrame* m_workspacePanel = nullptr;
    QVBoxLayout* m_workspaceLayout = nullptr;
    QTabBar* m_documentTabBar = nullptr;
    QFrame* m_toolBarCard = nullptr;
    QHBoxLayout* m_toolBarLayout = nullptr;
    QPushButton* m_importButton = nullptr;
    QPushButton* m_undoButton = nullptr;
    QToolButton* m_selectModeButton = nullptr;
    QToolButton* m_rotateModeButton = nullptr;
    QToolButton* m_panModeButton = nullptr;
    QToolButton* m_zoomModeButton = nullptr;
    QToolButton* m_showNodesButton = nullptr;
    QToolButton* m_showElementsButton = nullptr;
    QToolButton* m_showSolidButton = nullptr;
    QToolButton* m_showNodeIdsButton = nullptr;
    QToolButton* m_showElementIdsButton = nullptr;
    QToolButton* m_frontViewButton = nullptr;
    QToolButton* m_backViewButton = nullptr;
    QToolButton* m_leftViewButton = nullptr;
    QToolButton* m_rightViewButton = nullptr;
    QToolButton* m_topViewButton = nullptr;
    QToolButton* m_bottomViewButton = nullptr;
    QToolButton* m_fitViewButton = nullptr;
    QPushButton* m_startSolveButton = nullptr;
    WorkspaceSplitter* m_workspaceVerticalSplitter = nullptr;
    QStackedWidget* m_workspaceDisplayStack = nullptr;
    ModelViewport* m_modelViewport = nullptr;
    QTabWidget* m_monitorTabs = nullptr;
    QWidget* m_monitorPage = nullptr;
    QVBoxLayout* m_monitorLayout = nullptr;
    SingleLineLabel* m_incrementLabel = nullptr;
    QProgressBar* m_solveProgress = nullptr;
    SingleLineLabel* m_convergenceLabel = nullptr;
    QPushButton* m_stopButton = nullptr;
    QWidget* m_logPage = nullptr;
    QCheckBox* m_logDetailsCheck = nullptr;
    QPushButton* m_copyLogButton = nullptr;
    QPushButton* m_clearLogButton = nullptr;
    QPlainTextEdit* m_logEdit = nullptr;
    QLabel* m_loadStatisticsLabel = nullptr;
    QLabel* m_readyLabel = nullptr;

    ModelController* m_modelController = nullptr;
    SolveTaskController* m_solveTaskController = nullptr;
    std::unique_ptr<ResultDataController> m_resultDataController;
    std::unique_ptr<WorkspaceController> m_workspaceController;
    std::unique_ptr<Conductor::PropertyLibrary> m_propertyLibrary;
    SettingsPanel* m_settingsPanel = nullptr;
    ResultControlPanel* m_resultPanel = nullptr;
    NodeTimeHistoryPanel* m_nodeTimeHistoryPanel = nullptr;
    AnalysisSummaryPanel* m_analysisPanel = nullptr;
    PropertyModule* m_propertyModule = nullptr;
    ConductorModule* m_conductorModule = nullptr;
    QScrollArea* m_toolbarScrollArea = nullptr;
    int m_toolbarNaturalWidth = 0;
    int m_toolbarCardHeight = 0;
    QList<int> m_normalMainSplitterSizes{420, 940};
    bool m_monitorPanelInitialized = false;
    bool m_monitorHeightUpdatePending = false;
    struct LogEntry
    {
        QString text;
        bool diagnostic = false;
    };
    QList<LogEntry> m_logEntries;
    QSet<int> m_loggedFinishedTasks;
    QSet<int> m_loggedStartedTasks;
    int m_selectedNodeId = -1;
    QPushButton* m_solveCasesButton = nullptr;
    int m_solveBatchActionColumns = 0;
    QPushButton* m_solveAllCasesButton = nullptr;
    QPushButton* m_solveRestartAllCasesButton = nullptr;
    QPointer<SolveTaskManagerDialog> m_solveTaskManager;
    QPointer<AnalysisManagerDialog> m_analysisStepManager;
    QPointer<AnalysisManagerDialog> m_analysisLoadManager;
    QPointer<AnalysisManagerDialog> m_analysisConstraintManager;
    QPointer<AnalysisManagerDialog> m_analysisMPCManager;
    int m_modelLoadTotal = 0;
    int m_modelLoadSucceeded = 0;
    int m_modelLoadFailed = 0;
    bool m_loadingResultModel = false;
    Hdf5ResultRanges m_resultRanges;
    std::vector<Hdf5ResultFrameInfo> m_allResultFrames;
    std::vector<int> m_resultFrameSourceIndices;
    struct ResultFrameCacheEntry
    {
        int sourceIndex = -1;
        Hdf5ResultFrame frame;
    };
    std::array<ResultFrameCacheEntry, 2> m_resultFrameCache;
    int m_nextResultFrameCacheSlot = 0;
    int m_resultStepId = 0;
    int m_resultTaskId = -1;
    QString m_resultSourceModelName;
};
