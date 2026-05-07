#include "Node.h"

Node::Node() : m_X(0.0), m_Y(0.0), m_Z(0.0)
{
    m_DOF.resize(3, -1);    //默认3个自由度，均未约束
    m_Displacement.resize(3, 0.0);
}

void Node::SetNumDOFs(int num_dofs)
{
    if (num_dofs > m_DOF.size()) 
    {
        m_DOF.resize(num_dofs, -1);
        m_Displacement.resize(num_dofs, 0.0);
        m_Displacement_n.resize(num_dofs, 0.0);
        m_Acceleration.resize(num_dofs, 0.0);
        m_Velocity.resize(num_dofs, 0.0);
        m_Force.resize(num_dofs, 0.0);
    }
}
