#include "ElementBase.h"
#include "DataStructure/Node/Node.h"

ElementBase::ElementBase()
{
}

void ElementBase::Get_InertiaForce(VectorXd& inertiaForce)
{
    const int elementDofs =
        Get_NodeDOF() * static_cast<int>(m_pNode.size());
    MatrixXd mass = MatrixXd::Zero(elementDofs, elementDofs);
    Get_me_Consistent(mass);

    VectorXd acceleration = VectorXd::Zero(mass.rows());
    int offset = 0;
    for (const auto& nodeReference : m_pNode)
    {
        const auto node = nodeReference.lock();
        if (!node)
            continue;

        const int nodeDofs = std::min(
            Get_NodeDOF(), static_cast<int>(node->m_Acceleration.size()));
        for (int i = 0; i < nodeDofs && offset + i < acceleration.size(); ++i)
            acceleration(offset + i) = node->m_Acceleration[i];
        offset += Get_NodeDOF();
    }
    inertiaForce = mass * acceleration;
}

void ElementBase::Get_GyroscopicMatrix(MatrixXd& gyroscopicMatrix)
{
    const int elementDofs =
        Get_NodeDOF() * static_cast<int>(m_pNode.size());
    gyroscopicMatrix = MatrixXd::Zero(elementDofs, elementDofs);
}

void ElementBase::Get_CentrifugalMatrix(MatrixXd& centrifugalMatrix)
{
    const int elementDofs =
        Get_NodeDOF() * static_cast<int>(m_pNode.size());
    centrifugalMatrix = MatrixXd::Zero(elementDofs, elementDofs);
}

void ElementBase::GetDynamicContributions(
    MatrixXd& massMatrix,
    VectorXd& inertiaForce,
    MatrixXd& gyroscopicMatrix,
    MatrixXd& centrifugalMatrix)
{
    Get_me_Consistent(massMatrix);
    Get_InertiaForce(inertiaForce);
    Get_GyroscopicMatrix(gyroscopicMatrix);
    Get_CentrifugalMatrix(centrifugalMatrix);
}

void ElementBase::GetDOFs(std::vector<int>& DOFs) const
{
    int NodeDOF = Get_NodeDOF();
    int numDOFs = m_pNode.size() * NodeDOF;
    DOFs.resize(numDOFs);

    int index = 0;
    for (const auto& node : m_pNode)
    {
        auto pNode = node.lock();
        for(int i = 0; i < NodeDOF; i++)
        {
            DOFs[index++] = pNode->m_DOF[i];
        }
    }
}
