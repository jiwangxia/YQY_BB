#pragma once

#include <QDialog>

class QLabel;
class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QTableWidget;
class QTabWidget;
class PropertyLibraryDialog final : public QDialog
{
public:
    explicit PropertyLibraryDialog(QWidget* p = nullptr); // 创建属性库选择与同步窗口
    ~PropertyLibraryDialog() override;                    // 销毁属性库选择与同步窗口
    QTabWidget* tabs() const;                             // 返回属性分类选项卡
    QTableWidget* materialTable() const;                  // 返回材料属性表格
    QTableWidget* sectionTable() const;                   // 返回截面属性表格
    QCheckBox* syncCheck() const;                         // 返回是否同步到模型的选项
    QComboBox* targetMaterialCombo() const;               // 返回目标模型材料选择框
    QComboBox* targetSectionCombo() const;                // 返回目标模型截面选择框
    QDialogButtonBox* buttons() const;                    // 返回确认和取消按钮组

private:
    void updateSelectionLabel();                          // 更新当前所选属性的摘要
    void updateSyncControls();                            // 按同步选项更新目标选择控件状态
    void initializeUi();                                  // 创建界面控件并组织布局

    QTabWidget* m_tabs = nullptr;
    QTableWidget* m_materialTable = nullptr;
    QTableWidget* m_sectionTable = nullptr;
    QLabel* m_selectionLabel = nullptr;
    QCheckBox* m_syncCheck = nullptr;
    QLabel* m_materialLabel = nullptr;
    QComboBox* m_targetMaterialCombo = nullptr;
    QLabel* m_sectionLabel = nullptr;
    QComboBox* m_targetSectionCombo = nullptr;
    QLabel* m_noteLabel = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
