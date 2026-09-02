#include "ElementSpring2.h"

#include <stdexcept>

ElementSpring2::ElementSpring2()
{
    m_pNode.resize(2);
}

void ElementSpring2::GetNodeLocalDOFCounts(_OUT std::vector<int>& counts) const
{
    counts.clear();
    counts.reserve(m_pNode.size());
    for (const auto& nodeReference : m_pNode)
    {
        const auto node = nodeReference.lock();
        counts.push_back(node ? static_cast<int>(node->m_DOF.size()) : Get_NodeDOF());
    }
}

void ElementSpring2::Get_ke(_OUT MatrixXd& ke)
{
    const auto firstNode = m_pNode[0].lock();
    const auto secondNode = m_pNode[1].lock();
    if (!firstNode || !secondNode || m_FirstDOF < 0 || m_FirstDOF >= static_cast<int>(firstNode->m_DOF.size()) ||
        m_SecondDOF < 0 || m_SecondDOF >= static_cast<int>(secondNode->m_DOF.size()) ||
        m_FirstDOF >= static_cast<int>(firstNode->m_Displacement.size()) ||
        m_SecondDOF >= static_cast<int>(secondNode->m_Displacement.size()))
    {
        throw std::runtime_error("SPRING2 node or degree of freedom is invalid.");
    }

    const double relativeDisplacement = firstNode->m_Displacement[m_FirstDOF] - secondNode->m_Displacement[m_SecondDOF];
    double force = 0.0;
    double tangent = 0.0;
    EvaluateBehavior(relativeDisplacement, force, tangent);

    const int firstNodeDofCount = static_cast<int>(firstNode->m_DOF.size());
    const int secondNodeDofCount = static_cast<int>(secondNode->m_DOF.size());
    ke = MatrixXd::Zero(firstNodeDofCount + secondNodeDofCount, firstNodeDofCount + secondNodeDofCount);
    ke(m_FirstDOF, m_FirstDOF) = tangent;
    ke(m_FirstDOF, firstNodeDofCount + m_SecondDOF) = -tangent;
    ke(firstNodeDofCount + m_SecondDOF, m_FirstDOF) = -tangent;
    ke(firstNodeDofCount + m_SecondDOF, firstNodeDofCount + m_SecondDOF) = tangent;
    m_inforce = VectorXd::Zero(firstNodeDofCount + secondNodeDofCount);
    m_inforce[m_FirstDOF] = force;
    m_inforce[firstNodeDofCount + m_SecondDOF] = -force;
}

void ElementSpring2::Get_L0()
{
}
