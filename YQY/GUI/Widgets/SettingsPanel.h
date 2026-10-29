#pragma once

#include <QWidget>

class QFormLayout;
class QFrame;
class QVBoxLayout;

class QLabel;
class QCheckBox;
class QComboBox;
class QSpinBox;

class SettingsPanel final : public QWidget
{
public:
    explicit SettingsPanel(QWidget* parent = nullptr); // 创建程序运行与显示设置面板
    ~SettingsPanel() override;                         // 销毁程序运行与显示设置面板

    QSpinBox* concurrencySpin() const;                 // 返回求解任务并发数输入框
    QSpinBox* assemblyThreadsSpin() const;             // 返回矩阵装配线程数输入框
    QCheckBox* gpuSolverCheckBox() const;              // 返回启用 GPU 求解的选项
    QComboBox* linearSolverModeCombo() const;          // 返回线性求解器模式选择框
    QSpinBox* resultBatchFramesSpin() const;           // 返回结果批量写入帧数输入框
    QCheckBox* backgroundResultWriteCheckBox() const;  // 返回后台写入结果的选项
    QComboBox* nodeLabelModeCombo() const;             // 返回节点编号显示模式选择框
    QLabel* statusLabel() const;                       // 返回运行设置状态说明标签

private:
    void initializeUi();                               // 创建界面控件并组织布局
    void initializeAppearance();                       // 设置控件图标、样式和显示属性

    QVBoxLayout* m_rootLayout = nullptr;
    QFrame* m_infoCard = nullptr;
    QVBoxLayout* m_cardLayout = nullptr;
    QLabel* m_titleLabel = nullptr;
    QLabel* m_descriptionLabel = nullptr;
    QFormLayout* m_concurrencyForm = nullptr;
    QSpinBox* m_concurrencySpin = nullptr;
    QSpinBox* m_assemblyThreadsSpin = nullptr;
    QComboBox* m_linearSolverModeCombo = nullptr;
    QCheckBox* m_gpuSolverCheckBox = nullptr;
    QSpinBox* m_resultBatchFramesSpin = nullptr;
    QCheckBox* m_backgroundResultWriteCheckBox = nullptr;
    QComboBox* m_nodeLabelModeCombo = nullptr;
    QLabel* m_statusLabel = nullptr;
};
