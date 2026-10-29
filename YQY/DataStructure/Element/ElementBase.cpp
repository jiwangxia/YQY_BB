#include "ElementBase.h"
#include "DataStructure/Node/Node.h"
#include "Solver/Dynamic/Damping/LocalRayleighMatrix.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

ElementBase::ElementBase()
{
}

bool ElementBase::TryGetEnergySectionInertia(_OUT double& Iy, _OUT double& Iz, _OUT double& J) const
{
    const auto property = m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    return section && section->TryGetEnergyInertia(Iy, Iz, J);
}

bool ElementBase::TryGetStrainEnergy(_OUT double& energy) const
{
    energy = std::numeric_limits<double>::quiet_NaN();
    return false;
}

bool ElementBase::TryGetKineticEnergy(_OUT double& energy)
{
    energy = std::numeric_limits<double>::quiet_NaN();
    std::vector<int> counts;
    GetNodeLocalDOFCounts(counts);
    if (counts.size() != static_cast<std::size_t>(m_pNode.size()))
        return false;
    const int dofCount = std::accumulate(counts.cbegin(), counts.cend(), 0);
    if (dofCount <= 0)
        return false;
    VectorXd velocity(dofCount);
    int offset = 0;
    for (int index = 0; index < m_pNode.size(); ++index)
    {
        const auto node = m_pNode[index].lock();
        const int count = counts[static_cast<std::size_t>(index)];
        if (!node || count < 0 || node->m_Velocity.size() < static_cast<std::size_t>(count))
            return false;
        for (int component = 0; component < count; ++component)
            velocity[offset++] = node->m_Velocity[component];
    }
    if (!velocity.allFinite())
        return false;
    if (velocity.isZero(0.0))
    {
        energy = 0.0;
        return true;
    }
    MatrixXd mass;
    Get_me_Physical(mass);
    if (mass.rows() != dofCount || mass.cols() != dofCount || !mass.allFinite())
        return false;
    const double value = 0.5 * velocity.dot(mass * velocity);
    if (!std::isfinite(value) || value < 0.0)
        return false;
    energy = value;
    return true;
}

void ElementBase::Get_me_Physical(_OUT MatrixXd& me)
{
    Get_me_Consistent(me);
}

void ElementBase::GetRayleighDamping(const std::array<double, 4>& coefficients, _OUT MatrixXd& damping)
{
    MatrixXd stiffness;
    MatrixXd mass;
    Get_ke(stiffness);
    Get_me_Physical(mass);
    GetRayleighDamping(coefficients, stiffness, mass, damping);
}

void ElementBase::GetRayleighDamping(const std::array<double, 4>& coefficients, const MatrixXd& stiffness,
                                   const MatrixXd& physicalMass, _OUT MatrixXd& damping) const
{
    SolverNameSpace::BuildProportionalRayleighMatrix(stiffness, physicalMass, coefficients[0], coefficients[1],
                                                     damping);
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
