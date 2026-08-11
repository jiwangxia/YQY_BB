#include "NonlinearMPCConstraint.h"

#include "DataStructure/Node/Node.h"

#include <cmath>

namespace
{
    Eigen::Vector3d ReferencePosition(const Node& node)
    {
        return Eigen::Vector3d(node.m_X, node.m_Y, node.m_Z);
    }

    Eigen::Vector3d CurrentPosition(const Node& node)
    {
        Eigen::Vector3d result = ReferencePosition(node);
        for (int i = 0;
             i < 3 && i < static_cast<int>(node.m_Displacement.size());
             ++i)
            result[i] += node.m_Displacement[i];
        return result;
    }

    int FreeIndex(
        const Node& node,
        int direction,
        int fixedDofs,
        int freeDofs)
    {
        if (direction < 0 || direction >= node.m_DOF.size())
            return -1;
        const int index = node.m_DOF[direction] - fixedDofs;
        return index >= 0 && index < freeDofs ? index : -1;
    }

    void AddEntry(
        std::vector<Eigen::Triplet<double>>& entries,
        int row,
        int column,
        double value)
    {
        if (row >= 0 && column >= 0 && std::abs(value) > 1.0e-16)
            entries.emplace_back(row, column, value);
    }

    void AddFixedReaction(
        Node& node,
        int direction,
        int fixedDofs,
        double value)
    {
        if (direction < 0 || direction >= node.m_DOF.size())
            return;
        const int dof = node.m_DOF[direction];
        if (dof < 0 || dof >= fixedDofs)
            return;
        if (node.m_ReactionForce.size()
            < static_cast<std::size_t>(node.m_DOF.size()))
            node.m_ReactionForce.resize(node.m_DOF.size(), 0.0);
        node.m_ReactionForce[direction] += value;
    }
}

bool TranslationalTieMPCConstraint::Evaluate(
    int fixedDofs,
    int freeDofs,
    SolverNameSpace::NonlinearMPCData& data) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave || freeDofs < 3)
        return false;

    data.Clear();
    data.value = Eigen::VectorXd::Zero(3);
    data.jacobian = Eigen::MatrixXd::Zero(3, freeDofs);
    data.hessians.resize(3);
    data.slaveDofs.reserve(3);

    for (int direction = 0; direction < 3; ++direction)
    {
        const double masterDisplacement =
            direction < static_cast<int>(master->m_Displacement.size())
            ? master->m_Displacement[direction] : 0.0;
        const double slaveDisplacement =
            direction < static_cast<int>(slave->m_Displacement.size())
            ? slave->m_Displacement[direction] : 0.0;
        data.value[direction] = slaveDisplacement - masterDisplacement;

        const int masterDof =
            FreeIndex(*master, direction, fixedDofs, freeDofs);
        const int slaveDof =
            FreeIndex(*slave, direction, fixedDofs, freeDofs);
        if (masterDof >= 0)
            data.jacobian(direction, masterDof) = -1.0;
        if (slaveDof >= 0)
            data.jacobian(direction, slaveDof) = 1.0;
        data.hessians[direction].resize(freeDofs, freeDofs);
        data.slaveDofs.push_back(slaveDof);
    }
    return data.IsValid(freeDofs);
}

std::shared_ptr<NonlinearMPCConstraint>
TranslationalTieMPCConstraint::Clone(
    const std::map<int, std::shared_ptr<Node>>& nodes) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave)
        return nullptr;
    const auto foundMaster = nodes.find(master->m_Id);
    const auto foundSlave = nodes.find(slave->m_Id);
    if (foundMaster == nodes.end() || foundSlave == nodes.end())
        return nullptr;
    auto result = std::make_shared<TranslationalTieMPCConstraint>(*this);
    result->m_pMasterNode = foundMaster->second;
    result->m_pSlaveNode = foundSlave->second;
    return result;
}

std::vector<int> TranslationalTieMPCConstraint::GetNodeIds() const
{
    std::vector<int> result;
    if (const auto node = m_pMasterNode.lock())
        result.push_back(node->m_Id);
    if (const auto node = m_pSlaveNode.lock())
        result.push_back(node->m_Id);
    return result;
}

void TranslationalTieMPCConstraint::AccumulateReactions(
    int fixedDofs,
    const Eigen::VectorXd& multipliers) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave || multipliers.size() != 3)
        return;
    for (int direction = 0; direction < 3; ++direction)
    {
        AddFixedReaction(
            *master, direction, fixedDofs, multipliers[direction]);
        AddFixedReaction(
            *slave, direction, fixedDofs, -multipliers[direction]);
    }
}

bool DistanceMPCConstraint::Evaluate(
    int fixedDofs,
    int freeDofs,
    SolverNameSpace::NonlinearMPCData& data) const
{
    const auto nodeA = m_pNodeA.lock();
    const auto nodeB = m_pNodeB.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!nodeA || !nodeB || !slave || freeDofs <= 1)
        return false;

    const Eigen::Vector3d referenceDelta =
        ReferencePosition(*nodeA) - ReferencePosition(*nodeB);
    const double targetLength =
        m_Length >= 0.0 ? m_Length : referenceDelta.norm();
    const Eigen::Vector3d delta =
        CurrentPosition(*nodeA) - CurrentPosition(*nodeB);
    const double length = delta.norm();
    if (!std::isfinite(targetLength) || targetLength < 0.0
        || !std::isfinite(length) || length <= 1.0e-14)
        return false;

    data.Clear();
    data.value = Eigen::VectorXd::Constant(1, length - targetLength);
    data.jacobian = Eigen::MatrixXd::Zero(1, freeDofs);
    data.hessians.emplace_back(freeDofs, freeDofs);

    const Eigen::Vector3d direction = delta / length;
    const Eigen::Matrix3d curvature =
        (Eigen::Matrix3d::Identity()
            - direction * direction.transpose()) / length;
    int dofA[3] = { -1, -1, -1 };
    int dofB[3] = { -1, -1, -1 };
    for (int i = 0; i < 3; ++i)
    {
        dofA[i] = FreeIndex(*nodeA, i, fixedDofs, freeDofs);
        dofB[i] = FreeIndex(*nodeB, i, fixedDofs, freeDofs);
        if (dofA[i] >= 0) data.jacobian(0, dofA[i]) += direction[i];
        if (dofB[i] >= 0) data.jacobian(0, dofB[i]) -= direction[i];
    }

    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(36);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            AddEntry(entries, dofA[i], dofA[j], curvature(i, j));
            AddEntry(entries, dofA[i], dofB[j], -curvature(i, j));
            AddEntry(entries, dofB[i], dofA[j], -curvature(i, j));
            AddEntry(entries, dofB[i], dofB[j], curvature(i, j));
        }
    }
    data.hessians[0].setFromTriplets(entries.begin(), entries.end());
    data.slaveDofs.push_back(
        FreeIndex(*slave, m_SlaveDirection, fixedDofs, freeDofs));
    return data.IsValid(freeDofs);
}

std::shared_ptr<NonlinearMPCConstraint> DistanceMPCConstraint::Clone(
    const std::map<int, std::shared_ptr<Node>>& nodes) const
{
    const auto nodeA = m_pNodeA.lock();
    const auto nodeB = m_pNodeB.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!nodeA || !nodeB || !slave)
        return nullptr;
    const auto foundA = nodes.find(nodeA->m_Id);
    const auto foundB = nodes.find(nodeB->m_Id);
    const auto foundSlave = nodes.find(slave->m_Id);
    if (foundA == nodes.end() || foundB == nodes.end()
        || foundSlave == nodes.end())
        return nullptr;
    auto result = std::make_shared<DistanceMPCConstraint>(*this);
    result->m_pNodeA = foundA->second;
    result->m_pNodeB = foundB->second;
    result->m_pSlaveNode = foundSlave->second;
    return result;
}

std::vector<int> DistanceMPCConstraint::GetNodeIds() const
{
    std::vector<int> result;
    // 对文本MPC保持统一顺序：主节点、从节点。
    if (const auto node = m_pNodeB.lock()) result.push_back(node->m_Id);
    if (const auto node = m_pNodeA.lock()) result.push_back(node->m_Id);
    return result;
}

void DistanceMPCConstraint::AccumulateReactions(
    int fixedDofs,
    const Eigen::VectorXd& multipliers) const
{
    const auto nodeA = m_pNodeA.lock();
    const auto nodeB = m_pNodeB.lock();
    if (!nodeA || !nodeB || multipliers.size() != 1)
        return;
    const Eigen::Vector3d delta =
        CurrentPosition(*nodeA) - CurrentPosition(*nodeB);
    const double length = delta.norm();
    if (length <= 1.0e-14)
        return;
    const Eigen::Vector3d gradient = delta / length;
    // The reduced equations use R - G^T*Y. Therefore the physical
    // constraint-force contribution is -G^T*Y.
    for (int direction = 0; direction < 3; ++direction)
    {
        AddFixedReaction(
            *nodeA, direction, fixedDofs,
            -gradient[direction] * multipliers[0]);
        AddFixedReaction(
            *nodeB, direction, fixedDofs,
            gradient[direction] * multipliers[0]);
    }
}

bool RigidOffsetMPCConstraint::Evaluate(
    int fixedDofs,
    int freeDofs,
    SolverNameSpace::NonlinearMPCData& data) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave || master->m_DOF.size() < 3
        || slave->m_DOF.size() < 3 || freeDofs <= 3)
        return false;

    const Eigen::Vector3d offset = m_Offset.allFinite()
        ? m_Offset
        : ReferencePosition(*slave) - ReferencePosition(*master);
    const bool masterHasRotation = master->m_DOF.size() >= 6;
    const Eigen::Vector3d rotatedOffset =
        masterHasRotation ? master->m_Rg * offset : offset;
    const Eigen::Vector3d value =
        CurrentPosition(*slave)
        - CurrentPosition(*master)
        - rotatedOffset;

    data.Clear();
    data.value = value;
    data.jacobian = Eigen::MatrixXd::Zero(3, freeDofs);
    data.hessianEntries.resize(3);

    int masterTranslation[3] = { -1, -1, -1 };
    int masterRotation[3] = { -1, -1, -1 };
    int slaveTranslation[3] = { -1, -1, -1 };
    for (int i = 0; i < 3; ++i)
    {
        masterTranslation[i] =
            FreeIndex(*master, i, fixedDofs, freeDofs);
        masterRotation[i] = masterHasRotation
            ? FreeIndex(*master, i + 3, fixedDofs, freeDofs)
            : -1;
        slaveTranslation[i] =
            FreeIndex(*slave, i, fixedDofs, freeDofs);
        if (masterTranslation[i] >= 0)
            data.jacobian(i, masterTranslation[i]) = -1.0;
        if (slaveTranslation[i] >= 0)
            data.jacobian(i, slaveTranslation[i]) = 1.0;
    }

    Eigen::Matrix3d skew;
    skew <<
        0.0, -rotatedOffset.z(), rotatedOffset.y(),
        rotatedOffset.z(), 0.0, -rotatedOffset.x(),
        -rotatedOffset.y(), rotatedOffset.x(), 0.0;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            if (masterRotation[column] >= 0)
                data.jacobian(row, masterRotation[column])
                    = skew(row, column);

    for (int component = 0; component < 3; ++component)
    {
        Eigen::Vector3d basis = Eigen::Vector3d::Zero();
        basis[component] = 1.0;
        const Eigen::Matrix3d rotationHessian =
            rotatedOffset[component] * Eigen::Matrix3d::Identity()
            - 0.5 * (
                basis * rotatedOffset.transpose()
                + rotatedOffset * basis.transpose());
        auto& entries = data.hessianEntries[component];
        entries.reserve(9);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                AddEntry(
                    entries, masterRotation[i], masterRotation[j],
                    rotationHessian(i, j));
    }

    for (int i = 0; i < 3; ++i)
        data.slaveDofs.push_back(slaveTranslation[i]);
    return data.IsValid(freeDofs);
}

std::shared_ptr<NonlinearMPCConstraint> RigidOffsetMPCConstraint::Clone(
    const std::map<int, std::shared_ptr<Node>>& nodes) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave)
        return nullptr;
    const auto foundMaster = nodes.find(master->m_Id);
    const auto foundSlave = nodes.find(slave->m_Id);
    if (foundMaster == nodes.end() || foundSlave == nodes.end())
        return nullptr;
    auto result = std::make_shared<RigidOffsetMPCConstraint>(*this);
    result->m_pMasterNode = foundMaster->second;
    result->m_pSlaveNode = foundSlave->second;
    return result;
}

std::vector<int> RigidOffsetMPCConstraint::GetNodeIds() const
{
    std::vector<int> result;
    if (const auto node = m_pMasterNode.lock())
        result.push_back(node->m_Id);
    if (const auto node = m_pSlaveNode.lock())
        result.push_back(node->m_Id);
    return result;
}

void RigidOffsetMPCConstraint::AccumulateReactions(
    int fixedDofs,
    const Eigen::VectorXd& multipliers) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave || multipliers.size() != 3)
        return;

    for (int direction = 0; direction < 3; ++direction)
    {
        AddFixedReaction(
            *master, direction, fixedDofs, multipliers[direction]);
        AddFixedReaction(
            *slave, direction, fixedDofs, -multipliers[direction]);
    }

    if (master->m_DOF.size() >= 6)
    {
        const Eigen::Vector3d offset = m_Offset.allFinite()
            ? m_Offset
            : ReferencePosition(*slave) - ReferencePosition(*master);
        const Eigen::Vector3d rotatedOffset = master->m_Rg * offset;
        Eigen::Matrix3d skew;
        skew <<
            0.0, -rotatedOffset.z(), rotatedOffset.y(),
            rotatedOffset.z(), 0.0, -rotatedOffset.x(),
            -rotatedOffset.y(), rotatedOffset.x(), 0.0;
        const Eigen::Vector3d moment =
            -skew.transpose() * multipliers;
        for (int direction = 0; direction < 3; ++direction)
            AddFixedReaction(
                *master, direction + 3, fixedDofs, moment[direction]);
    }
}

bool PlanarShearReleaseMPCConstraint::Evaluate(
    int fixedDofs,
    int freeDofs,
    SolverNameSpace::NonlinearMPCData& data) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave || master->m_DOF.size() < 6
        || slave->m_DOF.size() < 6 || freeDofs <= 2)
        return false;

    const double phiMaster =
        std::atan2(master->m_Rg(1, 0), master->m_Rg(0, 0));
    const double phiSlave =
        std::atan2(slave->m_Rg(1, 0), slave->m_Rg(0, 0));
    const double cm = std::cos(phiMaster);
    const double sm = std::sin(phiMaster);
    const double cs = std::cos(phiSlave);
    const double ss = std::sin(phiSlave);
    const double um = master->m_Displacement[0];
    const double wm = master->m_Displacement[1];
    const double us = slave->m_Displacement[0];
    const double ws = slave->m_Displacement[1];

    // Use the relative rotation for the equality equation so that a full
    // revolution does not create an artificial atan2 branch jump.
    const Eigen::Matrix3d relative =
        master->m_Rg.transpose() * slave->m_Rg;
    const double relativeAngle =
        std::atan2(relative(1, 0), relative(0, 0));

    data.Clear();
    data.value = Eigen::VectorXd::Zero(2);
    data.value[0] = cm * um + sm * wm - cs * us - ss * ws;
    data.value[1] = -relativeAngle;
    data.jacobian = Eigen::MatrixXd::Zero(2, freeDofs);
    data.hessians.resize(2);

    const int mX = FreeIndex(*master, 0, fixedDofs, freeDofs);
    const int mY = FreeIndex(*master, 1, fixedDofs, freeDofs);
    const int mRz = FreeIndex(*master, 5, fixedDofs, freeDofs);
    const int sX = FreeIndex(*slave, 0, fixedDofs, freeDofs);
    const int sY = FreeIndex(*slave, 1, fixedDofs, freeDofs);
    const int sRz = FreeIndex(*slave, 5, fixedDofs, freeDofs);

    auto setJacobian = [&data](int row, int column, double value)
    {
        if (column >= 0)
            data.jacobian(row, column) += value;
    };
    setJacobian(0, mX, cm);
    setJacobian(0, mY, sm);
    setJacobian(0, mRz, -sm * um + cm * wm);
    setJacobian(0, sX, -cs);
    setJacobian(0, sY, -ss);
    setJacobian(0, sRz, ss * us - cs * ws);
    setJacobian(1, mRz, 1.0);
    setJacobian(1, sRz, -1.0);

    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(10);
    AddEntry(entries, mX, mRz, -sm);
    AddEntry(entries, mRz, mX, -sm);
    AddEntry(entries, mY, mRz, cm);
    AddEntry(entries, mRz, mY, cm);
    AddEntry(entries, mRz, mRz, -cm * um - sm * wm);
    AddEntry(entries, sX, sRz, ss);
    AddEntry(entries, sRz, sX, ss);
    AddEntry(entries, sY, sRz, -cs);
    AddEntry(entries, sRz, sY, -cs);
    AddEntry(entries, sRz, sRz, cs * us + ss * ws);
    data.hessians[0].resize(freeDofs, freeDofs);
    data.hessians[0].setFromTriplets(entries.begin(), entries.end());
    data.hessians[1].resize(freeDofs, freeDofs);
    data.hessians[1].setZero();

    // The paper's constraint is fully coupled. Select the translational
    // slave column with the larger pivot so the elimination remains regular
    // while the beam rolls through 90 and 270 degrees.
    data.slaveDofs = {
        std::abs(cs) >= std::abs(ss) ? sX : sY,
        sRz
    };
    return data.IsValid(freeDofs);
}

std::shared_ptr<NonlinearMPCConstraint>
PlanarShearReleaseMPCConstraint::Clone(
    const std::map<int, std::shared_ptr<Node>>& nodes) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave)
        return nullptr;
    const auto foundMaster = nodes.find(master->m_Id);
    const auto foundSlave = nodes.find(slave->m_Id);
    if (foundMaster == nodes.end() || foundSlave == nodes.end())
        return nullptr;
    auto result =
        std::make_shared<PlanarShearReleaseMPCConstraint>(*this);
    result->m_pMasterNode = foundMaster->second;
    result->m_pSlaveNode = foundSlave->second;
    return result;
}

std::vector<int> PlanarShearReleaseMPCConstraint::GetNodeIds() const
{
    std::vector<int> result;
    if (const auto node = m_pMasterNode.lock())
        result.push_back(node->m_Id);
    if (const auto node = m_pSlaveNode.lock())
        result.push_back(node->m_Id);
    return result;
}

void PlanarShearReleaseMPCConstraint::AccumulateReactions(
    int fixedDofs,
    const Eigen::VectorXd& multipliers) const
{
    const auto master = m_pMasterNode.lock();
    const auto slave = m_pSlaveNode.lock();
    if (!master || !slave || multipliers.size() != 2)
        return;

    const double phiMaster =
        std::atan2(master->m_Rg(1, 0), master->m_Rg(0, 0));
    const double phiSlave =
        std::atan2(slave->m_Rg(1, 0), slave->m_Rg(0, 0));
    const double cm = std::cos(phiMaster);
    const double sm = std::sin(phiMaster);
    const double cs = std::cos(phiSlave);
    const double ss = std::sin(phiSlave);
    const double um = master->m_Displacement[0];
    const double wm = master->m_Displacement[1];
    const double us = slave->m_Displacement[0];
    const double ws = slave->m_Displacement[1];
    const double rotationGradientMaster = -sm * um + cm * wm;
    const double rotationGradientSlave = ss * us - cs * ws;

    AddFixedReaction(*master, 0, fixedDofs, -cm * multipliers[0]);
    AddFixedReaction(*master, 1, fixedDofs, -sm * multipliers[0]);
    AddFixedReaction(
        *master, 5, fixedDofs,
        -rotationGradientMaster * multipliers[0] - multipliers[1]);
    AddFixedReaction(*slave, 0, fixedDofs, cs * multipliers[0]);
    AddFixedReaction(*slave, 1, fixedDofs, ss * multipliers[0]);
    AddFixedReaction(
        *slave, 5, fixedDofs,
        -rotationGradientSlave * multipliers[0] + multipliers[1]);
}
