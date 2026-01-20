#pragma once
#include "LoadBase.h"
class Node;

/**
 * @brief 节点力荷载类 - 施加在节点上的集中力
 */
class Force_Node : public LoadBase
{
public:
    Force_Node() { m_LoadType = EnumKeyword::LoadType::FORCE_NODE; }

    std::weak_ptr<Node> m_pNode;  ///< 荷载所在节点
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN;  ///< 荷载方向

    double m_Value = 0.0;  ///< 荷载值
};

