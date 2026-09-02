#include "ElementSpring1.h"

#include <stdexcept>

ElementSpring1::ElementSpring1()
{
    m_pNode.resize(1);
}

void ElementSpring1::GetNodeLocalDOFCounts(_OUT std::vector<int>& counts) const
{
    const auto node = m_pNode[0].lock();
    counts = {node ? static_cast<int>(node->m_DOF.size()) : Get_NodeDOF()};
}

void ElementSpring1::Get_ke(_OUT MatrixXd& ke)
{
    const auto node = m_pNode[0].lock();
    if (!node || m_DOF < 0 || m_DOF >= static_cast<int>(node->m_DOF.size()) ||
        m_DOF >= static_cast<int>(node->m_Displacement.size()))
        throw std::runtime_error("SPRING1 node or degree of freedom is invalid.");

    double force = 0.0;
    double tangent = 0.0;
    EvaluateBehavior(node->m_Displacement[m_DOF], force, tangent);

    const int nodeDofCount = static_cast<int>(node->m_DOF.size());
    ke = MatrixXd::Zero(nodeDofCount, nodeDofCount);
    ke(m_DOF, m_DOF) = tangent;
    m_inforce = VectorXd::Zero(nodeDofCount);
    m_inforce[m_DOF] = force;
}

void ElementSpring1::Get_L0()
{
}
