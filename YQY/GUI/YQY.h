#pragma once

#include <QtWidgets/QMainWindow>

#include "Export/Hdf5ResultIO.h"
#include "ui_YQY.h"

class StructureData;
class ModelController;
class SolveTaskController;
class QEvent;
class QResizeEvent;
class QPushButton;
class QScrollArea;
class QTreeWidgetItem;
class AnalysisManagerDialog;
class SettingsPanel;
class ResultControlPanel;
class AnalysisSummaryPanel;
class PropertyModule;
class ConductorModule;
class SolveTaskManagerDialog;
namespace Conductor
{
class PropertyLibrary;
}

class YQY final : public QMainWindow
{
    Q_OBJECT

public:
    explicit YQY(QWidget* parent = nullptr);
    ~YQY() override;
    bool openModel(const QString& filePath);
    bool openResult(const QString& filePath);
    bool saveAnalysisManagerPreview(const QString& filePath, int managerIndex = 0);
    bool saveSolveTaskManagerPreview(const QString& filePath);
    bool saveNodeExportPreview(const QString& filePath);

protected:
    bool event(QEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    enum class Module
    {
        Model = 0,
        Properties,
        Conductor,
        Analysis,
        Solve,
        Result,
        Settings
    };

    void initializeEmptyState();
    void initializeToolbarAppearance();
    void initializeConductorModule();
    void initializePropertyModule();
    void initializeResultModule();
    void initializeSettingsModule();
    void updateToolbarIcons(int themeIndex);
    void initializeInteractions();
    void initializeResponsiveLayout();
    void updateResponsiveLayout();
    void applyTheme(int themeIndex);
    void switchModule(Module module);
    void refreshModulePages();
    void openHdf5Result();
    bool loadHdf5Result(const QString& filePath, bool showErrors = true, bool activateResultModule = true,
                        bool partialResult = false);
    void displayResultFrame(int frameIndex);
    void displayResultPosition(double framePosition);
    bool cacheResultFramePair(int firstIndex, int secondIndex);
    void updateResultVisualization();
    bool loadModelFile(const QString& filePath);
    void handleModelLoaded(int modelId, const QString& filePath, qint64 elapsedMs);
    void handleModelLoadFailed(const QString& filePath, const QString& errorMessage);
    void handleActiveModelChanged(int modelId);
    void populateModelDataTree(const QString& fileName);
    void setModelControlsEnabled(bool enabled);
    void showNodeProperties(int nodeId);
    void showElementProperties(int elementId);
    void clearSelectionProperties();
    void updateLoadStatistics();
    void setWorkspaceMessage(const QString& message);
    void submitSolveTask();
    void prepareAnalysisCases(const std::shared_ptr<StructureData>& structure, const QString& sourceFile,
                              int onlyStepId = 0);
    void cancelSelectedSolveTask();
    void refreshSolveTasks(int preferredTaskId = -1);
    void updateSolveTaskRow(int taskId);
    void updateTaskMonitor(int taskId);
    int selectedSolveTaskId() const;
    void openSolveTaskManager();
    void startAllReadySolveTasks();
    void restartAllSolveTasks();
    void refreshSolveTaskManager();
    void initializeAnalysisEditor();
    void refreshAnalysisEditor();
    void openAnalysisManager(int initialPage);
    void handleAnalysisResourcesChanged(const QSet<int>& affectedStepIds);
    void closeAnalysisManagers();
    void showPropertyLibrary();
    void createConductorModel();
    void exportNodeResults();
    void refreshPropertyModule();
    void applyModelPropertyEdits();
    void editPropertyItem(QTreeWidgetItem* item, int column);

    Ui::YQYClass ui;
    ModelController* m_modelController = nullptr;
    SolveTaskController* m_solveTaskController = nullptr;
    std::unique_ptr<Conductor::PropertyLibrary> m_propertyLibrary;
    SettingsPanel* m_settingsPanel = nullptr;
    ResultControlPanel* m_resultPanel = nullptr;
    AnalysisSummaryPanel* m_analysisPanel = nullptr;
    PropertyModule* m_propertyModule = nullptr;
    ConductorModule* m_conductorModule = nullptr;
    QScrollArea* m_toolbarScrollArea = nullptr;
    int m_toolbarNaturalWidth = 0;
    int m_toolbarCardHeight = 0;
    QList<int> m_normalMainSplitterSizes{420, 940};
    int m_selectedNodeId = -1;
    QPushButton* m_solveCasesButton = nullptr;
    QPushButton* m_solveAllCasesButton = nullptr;
    QPushButton* m_solveRestartAllCasesButton = nullptr;
    QPointer<SolveTaskManagerDialog> m_solveTaskManager;
    QPointer<AnalysisManagerDialog> m_analysisStepManager;
    QPointer<AnalysisManagerDialog> m_analysisLoadManager;
    QPointer<AnalysisManagerDialog> m_analysisConstraintManager;
    int m_modelLoadTotal = 0;
    int m_modelLoadSucceeded = 0;
    int m_modelLoadFailed = 0;
    int m_cachedResultFrameIndex = -1;
    int m_cachedNextResultFrameIndex = -1;
    Hdf5ResultFrame m_cachedResultFrame;
    Hdf5ResultFrame m_cachedNextResultFrame;
    Hdf5ResultRanges m_resultRanges;
};
