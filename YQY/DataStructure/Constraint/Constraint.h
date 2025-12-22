#pragma once
#include "Base/Base.h"
class Node;
class Constraint : public Base
{
public:
    Constraint() {}

    std::weak_ptr<Node> m_pNode;
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN;
    double m_Value = 0.0;
};

