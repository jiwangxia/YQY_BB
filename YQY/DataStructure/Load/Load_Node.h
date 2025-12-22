#pragma once
#include "LoadBase.h"
class Node;
class Load_Node : public LoadBase
{
public:
    Load_Node() { m_LoadType = EnumKeyword::LoadType::NODE_FORCE; }

    std::weak_ptr<Node> m_pNode;
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN;

    double m_Value = 0.0;
};

