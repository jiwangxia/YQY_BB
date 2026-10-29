#pragma once

#include "AnalysisEditor.h"
#include "Base/EmptyOUT.h"
#include <array>
#include <vector>
#include <functional>

class QComboBox;
class QLineEdit;
class QSpinBox;
class QWidget;

class MPCEditorDialog final : public AnalysisMPCEditor
{
public:
    MPCEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
                    const std::shared_ptr<NonlinearMPCConstraint>& mpc, QWidget* parent); // 创建 MPC 编辑器并载入已有关系

    std::shared_ptr<NonlinearMPCConstraint> value() const override;                       // 返回确认后的 MPC 约束对象

protected:
    void accept() override;                                                               // 校验主从节点、方向和关系冲突并确认

private:
    int m_id = 0;
    std::shared_ptr<StructureData> m_structure;
    std::shared_ptr<NonlinearMPCConstraint> m_result;
    QLineEdit* m_name = nullptr;
    QComboBox* m_step = nullptr;
    QSpinBox* m_master = nullptr;
    QSpinBox* m_slave = nullptr;
    QComboBox* m_direction = nullptr;
};
