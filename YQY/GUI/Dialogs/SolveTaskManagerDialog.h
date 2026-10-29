#pragma once

#include <QDialog>

class QDialogButtonBox;
class QLabel;
class QPushButton;
class QTableWidget;
class SolveTaskManagerDialog final : public QDialog
{
public:
    explicit SolveTaskManagerDialog(QWidget* parent = nullptr); // 创建求解任务管理窗口
    ~SolveTaskManagerDialog() override;                         // 销毁求解任务管理窗口
    QPushButton* solveAllButton() const;                        // 返回启动全部待执行任务的按钮
    QPushButton* restartAllButton() const;                      // 返回重新执行全部任务的按钮
    QTableWidget* taskTable() const;                            // 返回求解任务状态表格

private:
    void initializeUi();                                        // 创建界面控件并组织布局

    QLabel* m_hintLabel = nullptr;
    QTableWidget* m_taskTable = nullptr;
    QPushButton* m_solveAllButton = nullptr;
    QPushButton* m_restartAllButton = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
