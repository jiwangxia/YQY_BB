#pragma once
#include "Base/Base.h"
class Node;

/**
 * @brief 约束类 - 存储边界条件信息
 */
class Constraint : public Base
{
public:
    Constraint() {}

    std::weak_ptr<Node> m_pNode;  ///< 约束所在节点
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN;  ///< 约束方向
    double m_Value = 0.0;  ///< 约束位移值
};

