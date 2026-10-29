#pragma once

#include "AnalysisEditor.h"
#include "Base/EmptyOUT.h"
#include <array>
#include <vector>
#include <functional>

class QComboBox;
class QDialogButtonBox;
class QDoubleSpinBox;
class QFormLayout;
class QLineEdit;
class QSpinBox;
class QWidget;

class ConstraintEditorDialog final : public AnalysisConstraintEditor
{
public:
    ConstraintEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
                           const std::shared_ptr<Constraint>& constraint, QWidget* parent); // 创建约束编辑器并载入已有约束

    QString name() const override;                                                          // 读取约束名称
    int stepId() const override;                                                            // 读取约束首次生效的分析步编号
    std::shared_ptr<Node> node() const override;                                            // 返回校验通过的约束节点
    EnumKeyword::Direction direction() const override;                                      // 读取约束自由度方向
    double displacement() const override;                                                   // 读取给定的约束位移值

protected:
    void accept() override;                                                                 // 检查节点和约束参数并确认编辑结果

private:
    void initializeUi();                                                                    // 创建控件和布局

    std::shared_ptr<StructureData> m_structure;
    std::shared_ptr<Node> m_nodeValue;
    QLineEdit* m_name = nullptr;
    QComboBox* m_step = nullptr;
    QSpinBox* m_node = nullptr;
    QComboBox* m_direction = nullptr;
    QDoubleSpinBox* m_value = nullptr;

    QFormLayout* m_formLayout = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
