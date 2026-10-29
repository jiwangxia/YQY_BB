#pragma once

#include <QDialog>

class CompactDoubleSpinBox;
class QLabel;
class QDialogButtonBox;
class QDoubleSpinBox;
class PropertyItemEditorDialog final : public QDialog
{
public:
    explicit PropertyItemEditorDialog(QWidget* p = nullptr);    // 创建材料与截面属性编辑窗口
    ~PropertyItemEditorDialog() override;                       // 销毁材料与截面属性编辑窗口
    void configureMaterial(const QString& source, int id, double young, double poisson, double density, double stress,
                           double expansion);                   // 填入材料属性并切换到材料编辑模式
    void configureSection(const QString& source, int id, const QString& type, double area, bool rectangle,
                          double width = 0, double height = 0); // 填入截面属性并配置截面尺寸输入
    QDoubleSpinBox* young() const;                              // 返回弹性模量输入框
    QDoubleSpinBox* poisson() const;                            // 返回泊松比输入框
    QDoubleSpinBox* density() const;                            // 返回密度输入框
    QDoubleSpinBox* stress() const;                             // 返回应力参数输入框
    QDoubleSpinBox* expansion() const;                          // 返回热膨胀系数输入框
    QDoubleSpinBox* area() const;                               // 返回截面面积输入框
    QDoubleSpinBox* width() const;                              // 返回矩形截面宽度输入框
    QDoubleSpinBox* height() const;                             // 返回矩形截面高度输入框
    QDialogButtonBox* buttons() const;                          // 返回确认和取消按钮组

private:
    void showMaterial(bool);                                    // 切换材料字段与截面字段的显示状态
    void initializeUi();                                        // 创建界面控件并组织布局

    QLabel* m_sourceValue = nullptr;
    QLabel* m_idCaption = nullptr;
    QLabel* m_idValue = nullptr;
    QLabel* m_typeCaption = nullptr;
    QLabel* m_typeValue = nullptr;
    QLabel* m_youngCaption = nullptr;
    CompactDoubleSpinBox* m_youngSpin = nullptr;
    QLabel* m_poissonCaption = nullptr;
    CompactDoubleSpinBox* m_poissonSpin = nullptr;
    QLabel* m_densityCaption = nullptr;
    CompactDoubleSpinBox* m_densitySpin = nullptr;
    QLabel* m_stressCaption = nullptr;
    CompactDoubleSpinBox* m_stressSpin = nullptr;
    QLabel* m_expansionCaption = nullptr;
    CompactDoubleSpinBox* m_expansionSpin = nullptr;
    QLabel* m_areaCaption = nullptr;
    CompactDoubleSpinBox* m_areaSpin = nullptr;
    QLabel* m_widthCaption = nullptr;
    CompactDoubleSpinBox* m_widthSpin = nullptr;
    QLabel* m_heightCaption = nullptr;
    CompactDoubleSpinBox* m_heightSpin = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
