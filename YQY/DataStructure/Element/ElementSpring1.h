#pragma once

#include "DataStructure/Element/ElementSpringBase.h"

// 节点对地固定方向弹簧，支持节点的平动和转动自由度。
class ElementSpring1 : public ElementSpringBase
{
public:
    int m_DOF = 0; // 受弹簧约束的节点内部自由度编号（0 至 5）

    ElementSpring1();
    void GetNodeLocalDOFCounts(_OUT std::vector<int>& counts) const override;
    void Get_ke(_OUT MatrixXd& ke) override;
    void Get_L0() override; // 固定方向弹簧不使用几何初始长度
};
