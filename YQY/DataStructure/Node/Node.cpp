#include "Node.h"

Node::Node() : m_X(0.0), m_Y(0.0), m_Z(0.0)
{
    m_DOF.resize(3, -1);    //默认3个自由度，均未约束
    m_Displacement.resize(3, 0.0);
}
