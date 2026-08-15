#include "ElementBase.h"
#include "DataStructure/Node/Node.h"

#include <algorithm>
#include <numeric>

ElementBase::ElementBase()
{
}

void ElementBase::Get_InertiaForce(_OUT VectorXd& inertiaForce)
{
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    const int elementDofs = std::accumulate(localDofCounts.cbegin(), localDofCounts.cend(), 0);
    MatrixXd mass = MatrixXd::Zero(elementDofs, elementDofs);
    Get_me_Consistent(mass);

    VectorXd acceleration = VectorXd::Zero(mass.rows());
    int offset = 0;
    for (int nodeIndex = 0; nodeIndex < m_pNode.size(); ++nodeIndex)
    {
        const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIndex)];
        const auto node = m_pNode[nodeIndex].lock();
        if (!node)
        {
            offset += localDofs;
            continue;
        }

        const int nodeDofs = std::min(localDofs, static_cast<int>(node->m_Acceleration.size()));
        for (int i = 0; i < nodeDofs && offset + i < acceleration.size(); ++i)
            acceleration(offset + i) = node->m_Acceleration[i];
        offset += localDofs;
    }
    inertiaForce = mass * acceleration;
}

void ElementBase::Get_GyroscopicMatrix(_OUT MatrixXd& gyroscopicMatrix)
{
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    const int elementDofs = std::accumulate(localDofCounts.cbegin(), localDofCounts.cend(), 0);
    gyroscopicMatrix = MatrixXd::Zero(elementDofs, elementDofs);
}

void ElementBase::Get_CentrifugalMatrix(_OUT MatrixXd& centrifugalMatrix)
{
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    const int elementDofs = std::accumulate(localDofCounts.cbegin(), localDofCounts.cend(), 0);
    centrifugalMatrix = MatrixXd::Zero(elementDofs, elementDofs);
}

void ElementBase::GetDynamicContributions(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce,
                                          _OUT MatrixXd& gyroscopicMatrix, _OUT MatrixXd& centrifugalMatrix)
{
    Get_me_Consistent(massMatrix);
    const int elementDofs = static_cast<int>(massMatrix.rows());
    VectorXd acceleration = VectorXd::Zero(elementDofs);
    int offset = 0;
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    for (int nodeIndex = 0; nodeIndex < m_pNode.size(); ++nodeIndex)
    {
        const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIndex)];
        const auto node = m_pNode[nodeIndex].lock();
        if (!node)
        {
            offset += localDofs;
            continue;
        }

        const int nodeDofs = std::min(localDofs, static_cast<int>(node->m_Acceleration.size()));
        for (int i = 0; i < nodeDofs && offset + i < elementDofs; ++i)
            acceleration[offset + i] = node->m_Acceleration[i];
        offset += localDofs;
    }
    inertiaForce = massMatrix * acceleration;
    Get_GyroscopicMatrix(gyroscopicMatrix);
    Get_CentrifugalMatrix(centrifugalMatrix);
}

void ElementBase::GetNodeLocalDOFCounts(_OUT std::vector<int>& counts) const
{
    counts.assign(static_cast<std::size_t>(m_pNode.size()), Get_NodeDOF());
}

void ElementBase::GetDOFs(std::vector<int>& DOFs) const
{
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    const int numDOFs = std::accumulate(localDofCounts.cbegin(), localDofCounts.cend(), 0);
    DOFs.resize(numDOFs);

    int index = 0;
    for (int nodeIndex = 0; nodeIndex < m_pNode.size(); ++nodeIndex)
    {
        auto pNode = m_pNode[nodeIndex].lock();
        const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIndex)];
        for (int i = 0; i < localDofs; ++i)
        {
            DOFs[index++] = pNode->m_DOF[i];
        }
    }
}
