#pragma once
#include "LoadBase.h"
class Node;
class Force_Node : public LoadBase
{
public:
    Force_Node() { m_LoadType = EnumKeyword::LoadType::FORCE_NODE; }

    std::weak_ptr<Node> m_pNode;
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN;

    double m_Value = 0.0;
};

