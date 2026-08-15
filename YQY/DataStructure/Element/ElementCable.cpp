#include "ElementCable.h"
#include "Utility/CR.h"

#include <numeric>
#include <stdexcept>

static bool UsesSpatialRotation(const Node& node)
{
    return node.m_DOF.size() >= 6;
}

static Eigen::Vector3d CurrentCableAxis(const ElementCable& cable)
{
    const auto first = cable.m_pNode[0].lock();
    const auto second = cable.m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("ElementCable node reference is invalid");
    const Eigen::Vector3d firstPosition(first->m_X + first->m_Displacement[0], first->m_Y + first->m_Displacement[1],
                                        first->m_Z + first->m_Displacement[2]);
    const Eigen::Vector3d secondPosition(second->m_X + second->m_Displacement[0],
                                         second->m_Y + second->m_Displacement[1],
                                         second->m_Z + second->m_Displacement[2]);
    const Eigen::Vector3d chord = secondPosition - firstPosition;
    if (chord.norm() <= 1.0e-12)
        throw std::runtime_error("ElementCable length must be positive");
    return Utility::CR::CanonicalAxis(chord);
}

static MatrixXd BuildCableDofTransform(const ElementCable& cable)
{
    std::vector<int> localDofCounts;
    cable.GetNodeLocalDOFCounts(localDofCounts);
    const int secondOffset = localDofCounts[0];
    const int elementDofs = secondOffset + localDofCounts[1];
    const Eigen::Vector3d axis = CurrentCableAxis(cable);
    MatrixXd transform = MatrixXd::Zero(8, elementDofs);
    transform.block<3, 3>(0, 0).setIdentity();
    transform.block<3, 3>(4, secondOffset).setIdentity();
    if (localDofCounts[0] == 4)
        transform(3, 3) = 1.0;
    else
        transform.block<1, 3>(3, 3) = axis.transpose();
    if (localDofCounts[1] == 4)
        transform(7, secondOffset + 3) = 1.0;
    else
        transform.block<1, 3>(7, secondOffset + 3) = axis.transpose();
    return transform;
}

ElementCable::ElementCable()
{
    m_pNode.resize(2);
}

void ElementCable::GetNodeLocalDOFCounts(_OUT std::vector<int>& counts) const
{
    counts.clear();
    counts.reserve(static_cast<std::size_t>(m_pNode.size()));
    for (const auto& nodeReference : m_pNode)
    {
        const auto node = nodeReference.lock();
        counts.push_back(node && UsesSpatialRotation(*node) ? 6 : 4);
    }
}

double ElementCable::GetNodalTwist(int nodeIndex) const
{
    if (nodeIndex < 0 || nodeIndex >= m_pNode.size())
        throw std::out_of_range("ElementCable node index is out of range");
    const auto node = m_pNode[nodeIndex].lock();
    if (!node)
        throw std::runtime_error("ElementCable node reference is invalid");
    if (!UsesSpatialRotation(*node))
        return node->m_Displacement[3];
    const Eigen::Matrix3d relativeRotation = node->m_Rg * node->m_Rg_n.transpose();
    return m_CommittedSpatialTwist[static_cast<std::size_t>(nodeIndex)] +
           Utility::CR::ExtractAxialTwist(relativeRotation, CurrentCableAxis(*this));
}

double ElementCable::GetNodalTwistRate(int nodeIndex) const
{
    if (nodeIndex < 0 || nodeIndex >= m_pNode.size())
        throw std::out_of_range("ElementCable node index is out of range");
    const auto node = m_pNode[nodeIndex].lock();
    if (!node)
        throw std::runtime_error("ElementCable node reference is invalid");
    if (!UsesSpatialRotation(*node))
        return node->m_Velocity.size() > 3 ? node->m_Velocity[3] : 0.0;
    if (node->m_Velocity.size() < 6)
        return 0.0;
    const Eigen::Vector3d angularVelocity(node->m_Velocity[3], node->m_Velocity[4], node->m_Velocity[5]);
    return angularVelocity.dot(CurrentCableAxis(*this));
}

void ElementCable::CopyRuntimeState(const ElementCable& source)
{
    m_CommittedSpatialTwist = source.m_CommittedSpatialTwist;
}

void ElementCable::CommitState()
{
    for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_pNode.size()); ++nodeIndex)
    {
        const auto node = m_pNode[static_cast<std::size_t>(nodeIndex)].lock();
        if (node && UsesSpatialRotation(*node))
            m_CommittedSpatialTwist[static_cast<std::size_t>(nodeIndex)] = GetNodalTwist(nodeIndex);
    }
}

void ElementCable::AddNodalAxialTorque(int nodeIndex, double torque, _OUT VectorXd& elementForce) const
{
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    if (nodeIndex < 0 || nodeIndex >= static_cast<int>(localDofCounts.size()))
        throw std::out_of_range("ElementCable node index is out of range");
    const int offset = std::accumulate(localDofCounts.cbegin(), localDofCounts.cbegin() + nodeIndex, 0);
    const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIndex)];
    if (elementForce.size() < offset + localDofs)
        throw std::invalid_argument("ElementCable force vector size does not match its DOFs");
    if (localDofs == 4)
    {
        elementForce[offset + 3] += torque;
        return;
    }
    elementForce.segment<3>(offset + 3) += torque * CurrentCableAxis(*this);
}

void ElementCable::Get_ke(MatrixXd& ke)
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    auto pProperty = m_pProperty.lock();
    if (!pNode0 || !pNode1 || !pProperty)
        throw std::runtime_error("ElementCable has incomplete node/property references");

    auto pMaterial = pProperty->m_pMaterial.lock();
    auto pSection = pProperty->m_pSection.lock();
    if (!pMaterial || !pSection)
        throw std::runtime_error("ElementCable has incomplete material/section references");

    Get_L0();
    const Eigen::Vector3d x0(pNode0->m_X + pNode0->m_Displacement[0], pNode0->m_Y + pNode0->m_Displacement[1],
                             pNode0->m_Z + pNode0->m_Displacement[2]);
    const Eigen::Vector3d x1(pNode1->m_X + pNode1->m_Displacement[0], pNode1->m_Y + pNode1->m_Displacement[1],
                             pNode1->m_Z + pNode1->m_Displacement[2]);
    const Eigen::Vector3d chord = x1 - x0;
    const double currentLength = chord.norm();
    if (currentLength <= 1.0e-12 || L0 <= 1.0e-12)
        throw std::runtime_error("ElementCable length must be positive");

    const Eigen::Vector3d axis = chord / currentLength;
    const Eigen::Vector3d torsionAxis = Utility::CR::CanonicalAxis(chord);
    const double area = pSection->m_Area;
    if (area <= 0.0)
        throw std::runtime_error("ElementCable section area must be positive");

    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    pSection->Calculate_I(Iy, Iz, polarMoment);
    if (polarMoment <= 0.0 && pSection->m_Radius > 0.0)
        polarMoment = 0.5 * area * pSection->m_Radius * pSection->m_Radius;
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    if (1.0 + pMaterial->m_Poisson <= 1.0e-12)
        throw std::runtime_error("ElementCable Poisson ratio must be greater than -1");

    const double shearModulus = pMaterial->m_Young / (2.0 * (1.0 + pMaterial->m_Poisson));
    // 与 TSSBN 参考实现 Element_Cable_CR::Calculate_ke_TSSBN 保持一致：
    // 采用初始构形（材料坐标）上的工程应变和扭转率。这样
    // N = N0 + EA/L0*(L-L0)、M = M0 + GJ/L0*theta 的导数与下方
    // 材料切线严格一致。旧实现使用当前长度作分母，却仍把 EA/L
    // 直接当作切线，内力与切线并不共轭，会破坏 Newton 收敛和动力频率。
    const double axialStiffness = pMaterial->m_Young * area / L0;
    const double torsionalStiffness = shearModulus * polarMoment / L0;
    const double extension = currentLength - L0;
    const double twist = GetNodalTwist(1) - GetNodalTwist(0);

    Eigen::Vector2d generalizedForce;
    // 索不能传递压力。轴向力被截断到零后，轴向材料刚度和几何刚度
    // 也必须同时移除，否则松弛索会在压缩状态下产生非物理反力。
    const double trialAxialForce = m_InitStress * area + axialStiffness * extension;
    const bool isTaut = trialAxialForce > 0.0;
    generalizedForce(0) = isTaut ? trialAxialForce : 0.0;
    generalizedForce(1) = torsionalStiffness * twist;
    m_Stress = area > 0.0 ? generalizedForce(0) / area : 0.0;

    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    const int secondOffset = localDofCounts[0];
    const int elementDofs = secondOffset + localDofCounts[1];
    MatrixXd B = MatrixXd::Zero(2, elementDofs);
    B.block<1, 3>(0, 0) = -axis.transpose();
    B.block<1, 3>(0, secondOffset) = axis.transpose();
    if (localDofCounts[0] == 4)
        B(1, 3) = -1.0;
    else
        B.block<1, 3>(1, 3) = -torsionAxis.transpose();
    if (localDofCounts[1] == 4)
        B(1, secondOffset + 3) = 1.0;
    else
        B.block<1, 3>(1, secondOffset + 3) = torsionAxis.transpose();

    Eigen::Matrix2d material = Eigen::Matrix2d::Zero();
    material(0, 0) = isTaut ? axialStiffness : 0.0;
    material(1, 1) = torsionalStiffness;
    ke = B.transpose() * material * B;

    const Eigen::Matrix3d geometricBlock =
        generalizedForce(0) / currentLength * (Eigen::Matrix3d::Identity() - axis * axis.transpose());
    ke.block<3, 3>(0, 0) += geometricBlock;
    ke.block<3, 3>(secondOffset, secondOffset) += geometricBlock;
    ke.block<3, 3>(0, secondOffset) -= geometricBlock;
    ke.block<3, 3>(secondOffset, 0) -= geometricBlock;

    m_inforce = B.transpose() * generalizedForce;
    m_ke = ke;
    L = currentLength;
}

void ElementCable::Get_me_Lumped(MatrixXd& me) //集中质量矩阵
{
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!material || !section)
        throw std::runtime_error("ElementCable has incomplete material/section references");

    Get_L0();
    if (section->m_Area <= 0.0 || material->m_Density < 0.0)
        throw std::runtime_error("ElementCable mass properties must be nonnegative");
    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    section->Calculate_I(Iy, Iz, polarMoment);
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    const double linearDensity = section->m_Area * material->m_Density;
    const double rotaryDensity = polarMoment * material->m_Density;
    constexpr double Sy = 0.0;
    constexpr double Sz = 0.0;

    MatrixXd canonicalMass = MatrixXd::Zero(8, 8);

    canonicalMass(0, 0) = canonicalMass(1, 1) = canonicalMass(2, 2) = canonicalMass(4, 4) = canonicalMass(5, 5) =
        canonicalMass(6, 6) = linearDensity;
    canonicalMass(3, 3) = canonicalMass(7, 7) = rotaryDensity;
    canonicalMass(1, 3) = canonicalMass(3, 1) = canonicalMass(5, 7) = canonicalMass(7, 5) = -Sy;
    canonicalMass(2, 3) = canonicalMass(3, 2) = canonicalMass(6, 7) = canonicalMass(7, 6) = Sz;

    canonicalMass *= (L0 / 2.0);
    const MatrixXd transform = BuildCableDofTransform(*this);
    me = transform.transpose() * canonicalMass * transform;
}

void ElementCable::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!material || !section)
        throw std::runtime_error("ElementCable has incomplete material/section references");

    Get_L0();
    if (section->m_Area <= 0.0 || material->m_Density < 0.0)
        throw std::runtime_error("ElementCable mass properties must be nonnegative");
    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    section->Calculate_I(Iy, Iz, polarMoment);
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    const double linearDensity = section->m_Area * material->m_Density;
    const double rotaryDensity = polarMoment * material->m_Density;
    constexpr double Sy = 0.0;
    constexpr double Sz = 0.0; // 当前只考虑质心与剪心重合的对称截面
    MatrixXd canonicalMass = MatrixXd::Zero(8, 8);

    MatrixXd mu = MatrixXd::Zero(4, 4);
    mu(0, 0) = mu(1, 1) = mu(2, 2) = linearDensity;
    mu(3, 3) = rotaryDensity;
    mu(1, 3) = mu(3, 1) = -Sy;
    mu(2, 3) = mu(3, 2) = Sz;

    canonicalMass.block<4, 4>(0, 0) = 2.0 * mu;
    canonicalMass.block<4, 4>(0, 4) = mu;
    canonicalMass.block<4, 4>(4, 4) = 2.0 * mu;
    canonicalMass.block<4, 4>(4, 0) = mu;
    canonicalMass *= (L0 / 6.0);
    const MatrixXd transform = BuildCableDofTransform(*this);
    me = transform.transpose() * canonicalMass * transform;
}

void ElementCable::Get_L0()
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pNode0 || !pNode1)
        throw std::runtime_error("ElementCable node reference is invalid");

    const Eigen::Vector3d x0(pNode0->m_X, pNode0->m_Y, pNode0->m_Z);
    const Eigen::Vector3d x1(pNode1->m_X, pNode1->m_Y, pNode1->m_Z);
    L0 = (x1 - x0).norm();
    if (L0 <= 1.0e-12)
        throw std::runtime_error("ElementCable initial length must be positive");
    if (L <= 0.0)
        L = L0;
}

void ElementCable::Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce)
{
    if (damping.size() < 4)
    {
        // 处理错误：抛出异常或设置空矩阵并返回
        throw std::invalid_argument("damping vector must have at least 4 elements");
    }

    MatrixXd ke, me;
    Get_ke(ke);
    Get_me_Consistent(me); //一致质量矩阵

    ce = damping[0] * me + damping[1] * ke;
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    std::vector<int> rotationalIndices;
    int offset = 0;
    for (int localDofs : localDofCounts)
    {
        for (int localDof = 3; localDof < localDofs; ++localDof)
            rotationalIndices.push_back(offset + localDof);
        offset += localDofs;
    }
    for (int row : rotationalIndices)
    {
        for (int column : rotationalIndices)
            ce(row, column) = damping[2] * me(row, column) + damping[3] * ke(row, column);
    }
}
