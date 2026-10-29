#pragma once

#include <QWidget>

class QPushButton;
class AnalysisSummaryPanel final : public QWidget
{
public:
    explicit AnalysisSummaryPanel(QWidget* parent = nullptr); // 创建分析资源概览面板
    ~AnalysisSummaryPanel() override;                         // 销毁分析资源概览面板
    QPushButton* stepsButton() const;                         // 返回分析步管理入口按钮
    QPushButton* loadsButton() const;                         // 返回荷载管理入口按钮
    QPushButton* constraintsButton() const;                   // 返回约束管理入口按钮
    QPushButton* mpcsButton() const;                          // 返回 MPC 管理入口按钮
    QPushButton* regionsButton() const;                       // 返回计算区域管理入口按钮

private:
    void initializeUi();                                      // 创建界面控件并组织布局
    void initializeAppearance();                              // 设置控件图标、样式和显示属性

    QPushButton* m_stepsButton = nullptr;
    QPushButton* m_loadsButton = nullptr;
    QPushButton* m_constraintsButton = nullptr;
    QPushButton* m_mpcsButton = nullptr;
    QPushButton* m_regionsButton = nullptr;
};
