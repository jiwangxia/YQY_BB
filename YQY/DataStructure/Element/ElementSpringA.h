#pragma once

#include "DataStructure/Element/ElementSpringBase.h"

// 两节点轴向弹簧，沿当前节点连线方向受力。
class ElementSpringA : public ElementSpringBase
{
public:
    ElementSpringA();
    void Get_ke(_OUT MatrixXd& ke) override;
    void Get_L0() override; // 由初始节点坐标计算初始长度
};
