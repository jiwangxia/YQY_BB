#pragma once

#include <QDialog>
#include <QSet>
#include <QStringList>
#include <functional>
#include <memory>
#include <utility>

class QDialogButtonBox;
class QPushButton;

class StructureData;
class QTableWidget;

class AnalysisManagerDialog : public QDialog
{
public:
    enum class Page
    {
        Steps = 0,
        Loads = 1,
        Constraints = 2,
        MPCs = 3
    };

    explicit AnalysisManagerDialog(const std::shared_ptr<StructureData>& structure, Page initialPage,
                                   QWidget* parent = nullptr);                                                           // 创建分析资源管理器并显示指定页面

    bool modelChanged() const                                                                                            // 查询管理器是否修改过模型
    {
        return m_modelChanged;
    }
    void setModelChangedCallback(std::function<void(const QSet<int>&)> callback);                                        // 设置模型变更及受影响分析步的通知回调
    void setOpenManagerCallback(std::function<void(Page)> callback);                                                     // 设置切换分析资源管理页面的回调
    void refreshFromModel();                                                                                             // 从当前模型重新加载管理器内容

private:
    void initializeUi();                                                                                                 // 创建界面控件并组织布局

    void refreshAll();                                                                                                   // 刷新分析步、荷载、约束和 MPC 列表
    void refreshStepTable(int preferredId = -1);                                                                         // 刷新分析步列表并恢复指定项选择
    void refreshLoadTable(int preferredId = -1);                                                                         // 刷新荷载列表并恢复指定项选择
    void refreshConstraintTable(int preferredId = -1);                                                                   // 刷新约束列表并恢复指定项选择
    void refreshMPCTable(int preferredId = -1);                                                                          // 刷新 MPC 列表并恢复指定项选择
    void editStep(int stepId = -1);                                                                                      // 新增或编辑指定分析步
    void editLoad(int loadId = -1);                                                                                      // 新增或编辑指定荷载
    void editConstraint(int constraintId = -1);                                                                          // 新增或编辑指定约束
    void editMPC(int mpcId = -1);                                                                                        // 新增或编辑指定 MPC 关系
    void deleteSelectedStep();                                                                                           // 删除选中的分析步
    void deleteSelectedLoad();                                                                                           // 删除选中的荷载
    void deleteSelectedConstraint();                                                                                     // 删除选中的约束
    void deleteSelectedMPC();                                                                                            // 删除选中的 MPC 关系
    QSet<int> affectedStepsFrom(int firstStepId) const;                                                                  // 收集从指定分析步起受变更影响的步骤
    void markModelChanged(const QSet<int>& affectedStepIds);                                                             // 标记模型变更并通知受影响的分析步
    int currentId(QTableWidget* table) const;                                                                            // 读取表格当前选中记录的编号
    void selectId(QTableWidget* table, int id) const;                                                                    // 在表格中选中指定编号的记录

    QTableWidget* m_resourceTable = nullptr;
    QPushButton* m_addButton = nullptr;
    QPushButton* m_editButton = nullptr;
    QPushButton* m_deleteButton = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;

    std::shared_ptr<StructureData> m_structure;
    Page m_page = Page::Steps;
    QTableWidget* m_stepTable = nullptr;
    QTableWidget* m_loadTable = nullptr;
    QTableWidget* m_constraintTable = nullptr;
    QTableWidget* m_mpcTable = nullptr;
    bool m_modelChanged = false;
    std::function<void(const QSet<int>&)> m_modelChangedCallback;
    std::function<void(Page)> m_openManagerCallback;
};

class AnalysisStepManagerDialog final : public AnalysisManagerDialog
{
public:
    explicit AnalysisStepManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr)       // 创建分析步管理窗口
        : AnalysisManagerDialog(structure, Page::Steps, parent)
    {
    }
};

class AnalysisLoadManagerDialog final : public AnalysisManagerDialog
{
public:
    explicit AnalysisLoadManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr)       // 创建荷载管理窗口
        : AnalysisManagerDialog(structure, Page::Loads, parent)
    {
    }
};

class AnalysisConstraintManagerDialog final : public AnalysisManagerDialog
{
public:
    explicit AnalysisConstraintManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr) // 创建约束管理窗口
        : AnalysisManagerDialog(structure, Page::Constraints, parent)
    {
    }
};

class AnalysisMPCManagerDialog final : public AnalysisManagerDialog
{
public:
    explicit AnalysisMPCManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr)        // 创建 MPC 管理窗口
        : AnalysisManagerDialog(structure, Page::MPCs, parent)
    {
    }
};
