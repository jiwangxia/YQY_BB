#pragma once

#include <QWidget>

class QVBoxLayout;
class QTabWidget;
class QTableWidget;
class QTreeWidget;
class QPushButton;
class PropertyModule final : public QWidget
{
public:
    explicit PropertyModule(QWidget* p = nullptr); // 创建模型属性查看与编辑面板
    ~PropertyModule() override;                    // 销毁模型属性查看与编辑面板
    QTabWidget* tabs() const;                      // 返回属性分类选项卡
    QTableWidget* materialTable() const;           // 返回材料属性表格
    QTableWidget* sectionTable() const;            // 返回截面属性表格
    QTreeWidget* materialTree() const;             // 返回模型材料属性树
    QTreeWidget* sectionTree() const;              // 返回模型截面属性树
    QTreeWidget* springBehaviorTree() const;       // 返回弹簧本构属性树
    QPushButton* refreshButton() const;            // 返回刷新属性的按钮
    QPushButton* applyButton() const;              // 返回应用属性修改的按钮

private:
    void initializeUi();                           // 创建界面控件并组织布局
    void initializeAppearance();                   // 设置控件图标、样式和显示属性

    QVBoxLayout* m_rootLayout = nullptr;
    QTabWidget* m_propertyTabs = nullptr;
    QTreeWidget* m_materialTree = nullptr;
    QTreeWidget* m_sectionTree = nullptr;
    QTreeWidget* m_springBehaviorTree = nullptr;
    QTableWidget* m_materialTable = nullptr;
    QTableWidget* m_sectionTable = nullptr;
    QPushButton* m_refreshButton = nullptr;
    QPushButton* m_applyButton = nullptr;
};
