#pragma once

#include "DataStructure/Element/ElementSpringBase.h"

// 两节点固定方向弹簧，两个端点可分别选用平动或转动自由度。
class ElementSpring2 : public ElementSpringBase
{
public:
    int m_FirstDOF = 0;  // 第一个节点的内部自由度编号（0 至 5）
    int m_SecondDOF = 0; // 第二个节点的内部自由度编号（0 至 5）

    ElementSpring2();
    void GetNodeLocalDOFCounts(_OUT std::vector<int>& counts) const override;
    void Get_ke(_OUT MatrixXd& ke) override;
    void Get_L0() override; // 固定方向弹簧不使用几何初始长度
};
