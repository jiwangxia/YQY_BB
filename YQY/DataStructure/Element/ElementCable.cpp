#include "ElementCable.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Section/SectionRectangle.h"
#include "DataStructure/Section/Section_Ice.h"
#include "Solver/Dynamic/Damping/LocalRayleighMatrix.h"
#include "Utility/CR.h"

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <numeric>
#include <limits>
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

namespace
{
constexpr int kMaximumSpatialCableDofs = 12;
using CableGradient = Eigen::Matrix<double, kMaximumSpatialCableDofs, 1>;
using CableHessian = Eigen::Matrix<double, kMaximumSpatialCableDofs, kMaximumSpatialCableDofs>;

struct TwistDerivatives
{
    CableGradient gradient = CableGradient::Zero();
    CableHessian hessian = CableHessian::Zero();
};

TwistDerivatives ComputeTwistDerivatives(const ElementCable& cable, int nodeIndex,
                                         const std::vector<int>& localDofCounts, const Eigen::Vector3d& chord,
                                         bool computeHessian = true)
{
    TwistDerivatives result;
    const int secondOffset = localDofCounts[0];
    const int dofCount = secondOffset + localDofCounts[1];
    const int rotationOffset = nodeIndex == 0 ? 3 : secondOffset + 3;
    if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
    {
        result.gradient[rotationOffset] = 1.0;
        return result;
    }

    const auto node = cable.m_pNode[static_cast<std::size_t>(nodeIndex)].lock();
    if (!node)
        throw std::runtime_error("ElementCable spatial twist node reference is invalid");
    const double length = chord.norm();
    const Eigen::Vector3d unitAxis = chord / length;
    const Eigen::Vector3d canonicalAxis = Utility::CR::CanonicalAxis(chord);
    const double axisSign = canonicalAxis.dot(unitAxis) < 0.0 ? -1.0 : 1.0;
    const Eigen::Matrix3d projector = Eigen::Matrix3d::Identity() - unitAxis * unitAxis.transpose();
    std::array<Eigen::Vector3d, kMaximumSpatialCableDofs> chordDerivative;
    std::array<Eigen::Vector3d, kMaximumSpatialCableDofs> projectedDerivative;
    std::array<Eigen::Vector3d, kMaximumSpatialCableDofs> axisDerivative;
    std::array<double, kMaximumSpatialCableDofs> longitudinalDerivative{};
    for (int column = 0; column < dofCount; ++column)
    {
        chordDerivative[column].setZero();
        if (column < 3)
            chordDerivative[column][column] = -1.0;
        else if (column >= secondOffset && column < secondOffset + 3)
            chordDerivative[column][column - secondOffset] = 1.0;
        projectedDerivative[column] = projector * chordDerivative[column];
        axisDerivative[column] = axisSign / length * projectedDerivative[column];
        longitudinalDerivative[column] = unitAxis.dot(chordDerivative[column]);
    }

    Eigen::Quaterniond quaternion(node->m_Rg * node->m_Rg_n.transpose());
    quaternion.normalize();
    if (quaternion.w() < 0.0)
        quaternion.coeffs() *= -1.0;
    const double w = quaternion.w();
    const Eigen::Vector3d v = quaternion.vec();
    const double z = v.dot(canonicalAxis);
    const double denominator = w * w + z * z;
    if (denominator <= 1.0e-14)
        throw std::runtime_error("ElementCable axial twist is singular near a transverse half-turn");
    const double inverseDenominator = 1.0 / denominator;
    const double inverseDenominatorSquared = inverseDenominator * inverseDenominator;
    const double thetaW = -2.0 * z * inverseDenominator;
    const double thetaZ = 2.0 * w * inverseDenominator;
    const double thetaWW = 4.0 * w * z * inverseDenominatorSquared;
    const double thetaWZ = 2.0 * (z * z - w * w) * inverseDenominatorSquared;
    const double thetaZZ = -thetaWW;

    std::array<double, kMaximumSpatialCableDofs> wDerivative{};
    std::array<double, kMaximumSpatialCableDofs> zDerivative{};
    std::array<Eigen::Vector3d, kMaximumSpatialCableDofs> vDerivative;
    for (int column = 0; column < dofCount; ++column)
        vDerivative[column].setZero();
    for (int component = 0; component < 3; ++component)
    {
        const int column = rotationOffset + component;
        Eigen::Vector3d basis = Eigen::Vector3d::Zero();
        basis[component] = 1.0;
        wDerivative[column] = -0.5 * v[component];
        vDerivative[column] = 0.5 * (w * basis + basis.cross(v));
    }
    for (int column = 0; column < dofCount; ++column)
    {
        zDerivative[column] = vDerivative[column].dot(canonicalAxis) + v.dot(axisDerivative[column]);
        result.gradient[column] = thetaW * wDerivative[column] + thetaZ * zDerivative[column];
    }
    if (!computeHessian)
        return result;
    for (int row = 0; row < dofCount; ++row)
        for (int column = row; column < dofCount; ++column)
        {
            const bool sameSpatialRotation = row == column && row >= rotationOffset && row < rotationOffset + 3;
            const double wSecondDerivative = sameSpatialRotation ? -0.25 * w : 0.0;
            Eigen::Vector3d vSecondDerivative = Eigen::Vector3d::Zero();
            if (sameSpatialRotation)
                vSecondDerivative = -0.25 * v;
            const Eigen::Vector3d axisSecondDerivative =
                -axisSign / (length * length) *
                (longitudinalDerivative[row] * projectedDerivative[column] +
                 longitudinalDerivative[column] * projectedDerivative[row] +
                 unitAxis * chordDerivative[row].dot(projectedDerivative[column]));
            const double zSecondDerivative = vSecondDerivative.dot(canonicalAxis) +
                                             vDerivative[row].dot(axisDerivative[column]) +
                                             vDerivative[column].dot(axisDerivative[row]) +
                                             v.dot(axisSecondDerivative);
            const double value = thetaW * wSecondDerivative + thetaZ * zSecondDerivative +
                                 thetaWW * wDerivative[row] * wDerivative[column] +
                                 thetaWZ * (wDerivative[row] * zDerivative[column] +
                                            zDerivative[row] * wDerivative[column]) +
                                 thetaZZ * zDerivative[row] * zDerivative[column];
            result.hessian(row, column) = value;
            result.hessian(column, row) = value;
        }
    return result;
}
} // namespace

ElementCable::ElementCable()
{
    m_pNode.resize(2);
}

bool ElementCable::TryGetStrainEnergy(_OUT double& energy) const
{
    energy = std::numeric_limits<double>::quiet_NaN();
    // 参考静力的当前长度本构未包含轴扭交叉势能导数，不能报告保守应变能。
    if (m_ReferenceTssbnStaticEquilibrium)
        return false;
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!first || !second || !material || !section || L0 <= 0.0 || section->m_Area <= 0.0 ||
        material->m_Young <= 0.0 || material->m_Poisson <= -1.0 ||
        first->m_Displacement.size() < 4 || second->m_Displacement.size() < 4)
        return false;
    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    if (!TryGetEnergySectionInertia(Iy, Iz, polarMoment))
        return false;
    if (polarMoment <= 0.0 && section->m_Radius > 0.0)
        polarMoment = 0.5 * section->m_Area * section->m_Radius * section->m_Radius;
    const Vector3d initial(second->m_X - first->m_X, second->m_Y - first->m_Y, second->m_Z - first->m_Z);
    const Vector3d delta(second->m_Displacement[0] - first->m_Displacement[0],
                         second->m_Displacement[1] - first->m_Displacement[1],
                         second->m_Displacement[2] - first->m_Displacement[2]);
    const double length = (initial + delta).norm();
    const double referenceLength = m_ReferenceTssbnConstitutive ? initial.norm() : L0;
    if (length <= 0.0 || referenceLength <= 0.0)
        return false;
    const double extension = length - referenceLength;
    const double axialStiffness = material->m_Young * section->m_Area / referenceLength;
    const double initialForce = m_InitStress * section->m_Area;
    double axialEnergy = 0.5 * axialStiffness * extension * extension + initialForce * extension;
    if (!m_ReferenceTssbnConstitutive)
    {
        // 对截断拉力 max(0, N0+k*u) 积分，并保持 u=0 时能量为零。
        const double initialTension = std::max(0.0, initialForce);
        const double tension = std::max(0.0, initialForce + axialStiffness * extension);
        axialEnergy = (tension - initialTension) * (tension + initialTension) / (2.0 * axialStiffness);
    }
    const double twist = GetNodalTwist(1) - GetNodalTwist(0);
    const double torsionalStiffness = material->m_Young * polarMoment /
                                     (2.0 * (1.0 + material->m_Poisson) * referenceLength);
    const double value = axialEnergy + 0.5 * torsionalStiffness * twist * twist;
    if (!std::isfinite(value))
        return false;
    energy = value;
    return true;
}

bool ElementCable::TryGetKineticEnergy(_OUT double& energy)
{
    energy = std::numeric_limits<double>::quiet_NaN();
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!first || !second || !material || !section || L0 <= 0.0 ||
        first->m_Velocity.size() < (UsesSpatialRotation(*first) ? 6u : 4u) ||
        second->m_Velocity.size() < (UsesSpatialRotation(*second) ? 6u : 4u) ||
        first->m_Displacement.size() < 4 || second->m_Displacement.size() < 4)
        return false;
    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    if (!TryGetEnergySectionInertia(Iy, Iz, polarMoment))
        return false;
    const double lineMass = material->m_Density * section->m_Area;
    const double rotaryDensity = section->m_MassInertiaPerLength.x() > 0.0
                                     ? section->m_MassInertiaPerLength.x()
                                     : material->m_Density * polarMoment;
    const Vector3d coupling = m_ReferenceTssbnConstitutive || section->m_DiagnosticReferenceFixedInertialCoupling
                                  ? Vector3d(Vector3d::UnitX().cross(section->m_MassFirstMomentPerLength))
                                  : Vector3d(CurrentCableAxis(*this).cross(GetMassFirstMomentGlobal()));
    const Vector3d firstVelocity(first->m_Velocity[0], first->m_Velocity[1], first->m_Velocity[2]);
    const Vector3d secondVelocity(second->m_Velocity[0], second->m_Velocity[1], second->m_Velocity[2]);
    // 参考共享梁节点使用材料角速度 x，与其惯性力的材料角加速度约定一致。
    const double firstSpin = GetNodalTwistRate(0);
    const double secondSpin = GetNodalTwistRate(1);
    const double translation = lineMass *
                               (firstVelocity.squaredNorm() + firstVelocity.dot(secondVelocity) +
                                 secondVelocity.squaredNorm());
    const double rotation = rotaryDensity *
                            (firstSpin * firstSpin + firstSpin * secondSpin + secondSpin * secondSpin);
    const double coupled = coupling.dot((2.0 * firstSpin + secondSpin) * firstVelocity +
                                         (firstSpin + 2.0 * secondSpin) * secondVelocity);
    const double value = L0 / 6.0 * (translation + rotation + coupled);
    if (!std::isfinite(value) || value < 0.0)
        return false;
    energy = value;
    return true;
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

int ElementCable::GetReferenceDofMapping(_OUT std::array<int, 8>& referenceToNative) const
{
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("ElementCable node reference is invalid");
    const int secondOffset = UsesSpatialRotation(*first) ? 6 : 4;
    // 参考索只连接两端前三平动和全局 RotX，梁节点的 RotY/RotZ 不进入索单元。
    referenceToNative = {0, 2, 1, 3, secondOffset, secondOffset + 2, secondOffset + 1, secondOffset + 3};
    return secondOffset + (UsesSpatialRotation(*second) ? 6 : 4);
}

double ElementCable::GetNodalTwist(int nodeIndex) const
{
    if (nodeIndex < 0 || nodeIndex >= m_pNode.size())
        throw std::out_of_range("ElementCable node index is out of range");
    const auto node = m_pNode[nodeIndex].lock();
    if (!node)
        throw std::runtime_error("ElementCable node reference is invalid");
    if (m_ReferenceTssbnConstitutive || !UsesSpatialRotation(*node))
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
    if (m_ReferenceTssbnConstitutive)
        return (node->m_Rg.transpose() * angularVelocity).x();
    return angularVelocity.dot(CurrentCableAxis(*this));
}

void ElementCable::GetMeanTwistDerivatives(_OUT VectorXd& gradient, _OUT MatrixXd* hessian) const
{
    std::vector<int> counts;
    GetNodeLocalDOFCounts(counts);
    const int dofCount = counts[0] + counts[1];
    if (m_ReferenceTssbnConstitutive)
    {
        gradient.setZero(dofCount);
        gradient[3] = 0.5;
        gradient[counts[0] + 3] = 0.5;
        if (hessian)
            hessian->setZero(dofCount, dofCount);
        return;
    }
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("ElementCable mean twist node reference is invalid");
    const Eigen::Vector3d firstPosition(first->m_X + first->m_Displacement[0],
                                        first->m_Y + first->m_Displacement[1],
                                        first->m_Z + first->m_Displacement[2]);
    const Eigen::Vector3d secondPosition(second->m_X + second->m_Displacement[0],
                                         second->m_Y + second->m_Displacement[1],
                                         second->m_Z + second->m_Displacement[2]);
    const Eigen::Vector3d chord = secondPosition - firstPosition;
    if (chord.norm() <= 1.0e-12)
        throw std::runtime_error("ElementCable mean twist requires positive length");
    const bool needHessian = hessian != nullptr;
    const TwistDerivatives firstTwist = ComputeTwistDerivatives(*this, 0, counts, chord, needHessian);
    const TwistDerivatives secondTwist = ComputeTwistDerivatives(*this, 1, counts, chord, needHessian);
    gradient = 0.5 * (firstTwist.gradient + secondTwist.gradient).head(dofCount);
    if (hessian)
        *hessian = 0.5 * (firstTwist.hessian + secondTwist.hessian).topLeftCorner(dofCount, dofCount);
}

Eigen::Vector3d ElementCable::GetMassFirstMomentGlobal() const
{
    const auto property = m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!section)
        throw std::runtime_error("ElementCable section reference is invalid");
    const Eigen::Vector3d reference = section->m_MassFirstMomentPerLength;
    if (reference.squaredNorm() == 0.0)
        return Eigen::Vector3d::Zero();
    const Eigen::Vector3d axis = CurrentCableAxis(*this);
    const Eigen::Vector3d transverse = reference - axis.dot(reference) * axis;
    const double meanTwist = 0.5 * (GetNodalTwist(0) + GetNodalTwist(1));
    return Eigen::AngleAxisd(meanTwist, axis) * transverse;
}

void ElementCable::CopyRuntimeState(const ElementCable& source)
{
    m_CommittedSpatialTwist = source.m_CommittedSpatialTwist;
    m_ReferenceTssbnConstitutive = source.m_ReferenceTssbnConstitutive;
    m_ReferenceTssbnStaticEquilibrium = source.m_ReferenceTssbnStaticEquilibrium;
}

void ElementCable::SetReferenceTssbnConstitutiveMode(bool enabled, bool staticEquilibrium)
{
    ClearPreparedSectionInertia();
    m_ReferenceCenteredMassOrderCacheReady = false;
    m_ReferenceTssbnConstitutive = enabled;
    m_ReferenceTssbnStaticEquilibrium = enabled && staticEquilibrium;
}

void ElementCable::SetPreparedSectionInertia(const SectionBase* sectionIdentity, double Iy, double Iz, double J)
{
    m_PreparedSectionInertia = {Iy, Iz, J};
    m_PreparedSectionIdentity = sectionIdentity;
}

void ElementCable::ClearPreparedSectionInertia()
{
    m_PreparedSectionIdentity = nullptr;
}

void ElementCable::GetSectionInertia(SectionBase& section, _OUT double& Iy, _OUT double& Iz, _OUT double& J) const
{
    if (m_PreparedSectionIdentity == &section)
    {
        Iy = m_PreparedSectionInertia[0];
        Iz = m_PreparedSectionInertia[1];
        J = m_PreparedSectionInertia[2];
        return;
    }
    section.Calculate_I(Iy, Iz, J);
}

bool ElementCable::RestoreCommittedSpatialTwist(const std::array<double, 2>& twist)
{
    if (!std::isfinite(twist[0]) || !std::isfinite(twist[1]))
        return false;
    m_CommittedSpatialTwist = twist;
    m_BackupCommittedSpatialTwist = twist;
    m_CheckpointCommittedSpatialTwist.reset();
    return true;
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

void ElementCable::BackupState()
{
    m_BackupCommittedSpatialTwist = m_CommittedSpatialTwist;
}

void ElementCable::RestoreState()
{
    m_CommittedSpatialTwist = m_BackupCommittedSpatialTwist;
}

bool ElementCable::PushStateCheckpoint()
{
    if (m_CheckpointCommittedSpatialTwist.has_value())
        return false;
    m_CheckpointCommittedSpatialTwist = SpatialTwistCheckpoint{m_CommittedSpatialTwist,
                                                              m_BackupCommittedSpatialTwist};
    return true;
}

bool ElementCable::RestoreStateCheckpoint()
{
    if (!m_CheckpointCommittedSpatialTwist.has_value())
        return false;
    m_CommittedSpatialTwist = m_CheckpointCommittedSpatialTwist->committed;
    // 两半步试算会覆盖单步备份；恢复后拒绝整步仍须回到原始历史。
    m_BackupCommittedSpatialTwist = m_CheckpointCommittedSpatialTwist->backup;
    return true;
}

void ElementCable::DiscardStateCheckpoint()
{
    m_CheckpointCommittedSpatialTwist.reset();
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
    if (m_ReferenceTssbnConstitutive || localDofs == 4)
    {
        elementForce[offset + 3] += torque;
        return;
    }
    elementForce.segment<3>(offset + 3) += torque * CurrentCableAxis(*this);
}

void ElementCable::GetReferenceStaticCalibrationStiffness(_OUT MatrixXd& ke) const
{
    const auto property = m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!property || !section)
        throw std::runtime_error("ElementCable has incomplete calibration property/section references");

    // Calculate_I 会更新截面缓存，因此连同属性一起复制，节点和材料仅共享只读数据。
    std::shared_ptr<SectionBase> calibrationSection;
    if (const auto circular = std::dynamic_pointer_cast<SectionCircular>(section))
        calibrationSection = std::make_shared<SectionCircular>(*circular);
    else if (const auto rectangle = std::dynamic_pointer_cast<SectionRectangle>(section))
        calibrationSection = std::make_shared<SectionRectangle>(*rectangle);
    else if (const auto ice = std::dynamic_pointer_cast<Section_Ice>(section))
        calibrationSection = std::make_shared<Section_Ice>(*ice);
    else
        throw std::runtime_error("unsupported ElementCable calibration section type");

    const auto calibrationProperty = std::make_shared<Property>(*property);
    calibrationProperty->m_pSection = calibrationSection;
    ElementCable calibrationCopy(*this);
    calibrationCopy.m_pProperty = calibrationProperty;
    calibrationCopy.SetReferenceTssbnConstitutiveMode(true, true);
    calibrationCopy.Get_ke(ke);
}

void ElementCable::Get_ke(_OUT MatrixXd& ke)
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
    if (m_ReferenceTssbnConstitutive)
    {
        // 参考局部顺序为 [轴向, 竖向, 横向, 扭转]，运算完成后再映回当前坐标。
        const Eigen::Vector3d initial0(pNode0->m_X, pNode0->m_Z, -pNode0->m_Y);
        const Eigen::Vector3d initial1(pNode1->m_X, pNode1->m_Z, -pNode1->m_Y);
        const Eigen::Vector3d initialChord = initial1 - initial0;
        const Eigen::Vector3d displacement0(pNode0->m_Displacement[0], pNode0->m_Displacement[2],
                                             -pNode0->m_Displacement[1]);
        const Eigen::Vector3d displacement1(pNode1->m_Displacement[0], pNode1->m_Displacement[2],
                                             -pNode1->m_Displacement[1]);
        const Eigen::Vector3d chord = initialChord + displacement1 - displacement0;
        const double currentLength = chord.norm();
        if (currentLength <= 1.0e-12 || L0 <= 1.0e-12)
            throw std::runtime_error("ElementCable length must be positive");
        const double area = pSection->m_Area;
        if (area <= 0.0)
            throw std::runtime_error("ElementCable section area must be positive");
        double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
        GetSectionInertia(*pSection, Iy, Iz, polarMoment);
        if (!pSection->m_HasExplicitSectionInertia && dynamic_cast<const SectionCircular*>(pSection.get()))
        {
            const double diameter = pSection->m_Radius * 2.0;
            polarMoment = PI * diameter * diameter * diameter * diameter / 32.0;
        }
        if (polarMoment <= 0.0 && pSection->m_Radius > 0.0)
            polarMoment = 0.5 * area * pSection->m_Radius * pSection->m_Radius;
        if (polarMoment < 0.0)
            throw std::runtime_error("ElementCable polar moment must not be negative");
        if (1.0 + pMaterial->m_Poisson <= 1.0e-12)
            throw std::runtime_error("ElementCable Poisson ratio must be greater than -1");

        const double shearModulus = pMaterial->m_Young / (2.0 + 2.0 * pMaterial->m_Poisson);
        const double constitutiveLength = m_ReferenceTssbnStaticEquilibrium ? currentLength : initialChord.norm();
        const double axialStiffness = pMaterial->m_Young * area / constitutiveLength;
        const double torsionalStiffness = shearModulus * polarMoment / constitutiveLength;
        const double extension = currentLength - initialChord.norm();
        const double twist = pNode1->m_Displacement[3] - pNode0->m_Displacement[3];
        Eigen::Matrix2d material = Eigen::Matrix2d::Zero();
        material(0, 0) = axialStiffness;
        material(1, 1) = torsionalStiffness;
        Eigen::Vector2d generalizedForce = Eigen::Vector2d::Zero();
        if (m_ReferenceTssbnStaticEquilibrium)
        {
            generalizedForce(0) = m_InitStress * area;
            generalizedForce(0) += axialStiffness * extension;
            generalizedForce(1) += torsionalStiffness * twist;
        }
        else
        {
            generalizedForce(0) = m_InitStress * area + axialStiffness * extension;
            generalizedForce(1) = 0.0 + torsionalStiffness * twist;
        }
        const Eigen::Vector3d axis = chord.normalized();
        VectorXd& r = m_ReferenceAxialGradientScratch;
        r.resize(8);
        r << -axis[0], -axis[1], -axis[2], 0.0, axis[0], axis[1], axis[2], 0.0;
        MatrixXd& B = m_ReferenceStrainDisplacementScratch;
        B.setZero(2, 8);
        B.block<1, 8>(0, 0) = r;
        B(1, 3) = -1.0;
        B(1, 7) = 1.0;
        MatrixXd& geometricBlock = m_ReferenceGeometricBlockScratch;
        geometricBlock.setZero(3, 3);
        if (m_ReferenceTssbnStaticEquilibrium)
            geometricBlock = 1.0 / currentLength * (Eigen::Matrix3d::Identity() - axis * axis.transpose());
        else
        {
            const Eigen::Matrix3d referenceBlock =
                (Eigen::Matrix3d::Identity() - axis * axis.transpose()) / currentLength;
            geometricBlock = referenceBlock;
        }
        MatrixXd& geometric = m_ReferenceGeometricScratch;
        geometric.setZero(8, 8);
        geometric.block<3, 3>(0, 0) = geometricBlock;
        geometric.block<3, 3>(4, 0) = -geometricBlock;
        geometric.block<3, 3>(0, 4) = -geometricBlock;
        geometric.block<3, 3>(4, 4) = geometricBlock;
        VectorXd& referenceForce = m_ReferenceForceScratch;
        referenceForce.noalias() = B.transpose() * generalizedForce;
        MatrixXd& referenceStiffness = m_ReferenceStiffnessScratch;
        if (m_ReferenceTssbnStaticEquilibrium)
        {
            MatrixXd& geometricStiffness = m_ReferenceGeometricStiffnessScratch;
            geometricStiffness.setZero(8, 8);
            geometricStiffness = geometric * generalizedForce(0);
            referenceStiffness = B.transpose() * material * B + geometricStiffness;
        }
        else
            referenceStiffness = B.transpose() * material * B + geometric * generalizedForce(0);

        std::array<int, 8> referenceToNative;
        const int elementDofs = GetReferenceDofMapping(referenceToNative);
        constexpr std::array<double, 8> referenceSigns{1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0};
        ke.setZero(elementDofs, elementDofs);
        m_inforce.setZero(elementDofs);
        for (int row = 0; row < 8; ++row)
        {
            m_inforce[referenceToNative[row]] = referenceSigns[row] * referenceForce[row];
            for (int column = 0; column < 8; ++column)
                ke(referenceToNative[row], referenceToNative[column]) =
                    referenceSigns[row] * referenceSigns[column] * referenceStiffness(row, column);
        }
        m_Stress = generalizedForce(0) / area;
        m_ke = ke;
        L = currentLength;
        return;
    }
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
    GetSectionInertia(*pSection, Iy, Iz, polarMoment);
    if (polarMoment <= 0.0 && pSection->m_Radius > 0.0)
        polarMoment = 0.5 * area * pSection->m_Radius * pSection->m_Radius;
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    if (1.0 + pMaterial->m_Poisson <= 1.0e-12)
        throw std::runtime_error("ElementCable Poisson ratio must be greater than -1");

    const double shearModulus = pMaterial->m_Young / (2.0 * (1.0 + pMaterial->m_Poisson));
    // 默认线路和参考动力线路使用 L0；参考静力线路按原实现使用当前长度，
    // 并保留原实现直接采用 EA/L、GJ/L 的切线，避免改变参考 Newton 迭代。
    const double constitutiveLength = m_ReferenceTssbnStaticEquilibrium ? currentLength : L0;
    const double axialStiffness = pMaterial->m_Young * area / constitutiveLength;
    const double torsionalStiffness = shearModulus * polarMoment / constitutiveLength;
    const double extension = currentLength - L0;
    const double twist = GetNodalTwist(1) - GetNodalTwist(0);

    Eigen::Vector2d generalizedForce;
    // 默认索只受拉；参考线路保留原实现的负轴力及其材料和几何刚度。
    const double trialAxialForce = m_InitStress * area + axialStiffness * extension;
    const bool isTaut = trialAxialForce > 0.0;
    const bool retainAxialStiffness = m_ReferenceTssbnConstitutive || isTaut;
    generalizedForce(0) = retainAxialStiffness ? trialAxialForce : 0.0;
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
    material(0, 0) = retainAxialStiffness ? axialStiffness : 0.0;
    const bool spatialRotation = localDofCounts[0] == 6 || localDofCounts[1] == 6;
    material(1, 1) = spatialRotation ? 0.0 : torsionalStiffness;
    ke = B.transpose() * material * B;

    const Eigen::Matrix3d geometricBlock =
        generalizedForce(0) / currentLength * (Eigen::Matrix3d::Identity() - axis * axis.transpose());
    ke.block<3, 3>(0, 0) += geometricBlock;
    ke.block<3, 3>(secondOffset, secondOffset) += geometricBlock;
    ke.block<3, 3>(0, secondOffset) -= geometricBlock;
    ke.block<3, 3>(secondOffset, 0) -= geometricBlock;

    if (spatialRotation)
    {
        Eigen::Vector2d axialForce(generalizedForce(0), 0.0);
        m_inforce = B.transpose() * axialForce;
        const TwistDerivatives firstTwist = ComputeTwistDerivatives(*this, 0, localDofCounts, chord);
        const TwistDerivatives secondTwist = ComputeTwistDerivatives(*this, 1, localDofCounts, chord);
        const Eigen::VectorXd twistGradient =
            (secondTwist.gradient - firstTwist.gradient).head(elementDofs);
        const Eigen::MatrixXd twistHessian =
            (secondTwist.hessian - firstTwist.hessian).topLeftCorner(elementDofs, elementDofs);
        m_inforce.noalias() += generalizedForce(1) * twistGradient;
        ke.noalias() += torsionalStiffness * twistGradient * twistGradient.transpose();
        ke.noalias() += generalizedForce(1) * twistHessian;
        // 上述 Hessian 基于固定指数坐标；当前空间坐标中的左乘转角增量
        // 还需计入 SO(3) 运输项，才能得到力矩分量的切线。
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            if (localDofCounts[static_cast<std::size_t>(nodeIndex)] != 6)
                continue;
            const int rotationOffset = (nodeIndex == 0 ? 0 : secondOffset) + 3;
            const Eigen::Vector3d rotationalGradient = twistGradient.segment<3>(rotationOffset);
            Eigen::Matrix3d skewGradient;
            Utility::CR::SkewSymmetric(rotationalGradient, skewGradient);
            ke.block<3, 3>(rotationOffset, rotationOffset) -= 0.5 * generalizedForce(1) * skewGradient;
        }
    }
    else
    {
        m_inforce = B.transpose() * generalizedForce;
    }
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
    GetSectionInertia(*section, Iy, Iz, polarMoment);
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    const double linearDensity = section->m_Area * material->m_Density;
    const double rotaryDensity = section->m_MassInertiaPerLength.x() > 0.0
                                     ? section->m_MassInertiaPerLength.x()
                                     : polarMoment * material->m_Density;
    const Eigen::Vector3d translationTwistMass = section->m_DiagnosticReferenceFixedInertialCoupling
                                                      ? Eigen::Vector3d::UnitX().cross(
                                                            section->m_MassFirstMomentPerLength)
                                                      : CurrentCableAxis(*this).cross(GetMassFirstMomentGlobal());

    MatrixXd canonicalMass = MatrixXd::Zero(8, 8);

    canonicalMass(0, 0) = canonicalMass(1, 1) = canonicalMass(2, 2) = canonicalMass(4, 4) = canonicalMass(5, 5) =
        canonicalMass(6, 6) = linearDensity;
    canonicalMass(3, 3) = canonicalMass(7, 7) = rotaryDensity;
    for (int component = 0; component < 3; ++component)
    {
        canonicalMass(component, 3) = canonicalMass(3, component) = translationTwistMass[component];
        canonicalMass(4 + component, 7) = canonicalMass(7, 4 + component) = translationTwistMass[component];
    }

    canonicalMass *= (L0 / 2.0);
    const MatrixXd transform = BuildCableDofTransform(*this);
    me = transform.transpose() * canonicalMass * transform;
}

void ElementCable::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
    m_ReferenceCenteredMassOrderCacheReady = false;
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!material || !section)
        throw std::runtime_error("ElementCable has incomplete material/section references");

    Get_L0();
    if (section->m_Area <= 0.0 || material->m_Density < 0.0)
        throw std::runtime_error("ElementCable mass properties must be nonnegative");
    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    GetSectionInertia(*section, Iy, Iz, polarMoment);
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (m_ReferenceTssbnConstitutive && first && second)
    {
        std::array<int, 8> referenceToNative;
        const int elementDofs = GetReferenceDofMapping(referenceToNative);
        const Eigen::Vector3d currentChord(
            second->m_X + second->m_Displacement[0] - first->m_X - first->m_Displacement[0],
            second->m_Y + second->m_Displacement[1] - first->m_Y - first->m_Displacement[1],
            second->m_Z + second->m_Displacement[2] - first->m_Z - first->m_Displacement[2]);
        // 固定参考质量的缓存命中也不能接受已经坍缩的单元几何。
        if (!currentChord.allFinite() || currentChord.norm() <= 1.0e-12)
            throw std::runtime_error("ElementCable current length must be positive");
        if (!section->m_HasExplicitSectionInertia && dynamic_cast<const SectionCircular*>(section.get()))
        {
            const double diameter = section->m_Radius * 2.0;
            polarMoment = PI * diameter * diameter * diameter * diameter / 32.0;
        }
        const double translationMass = section->m_Area * material->m_Density * L0 / 6.0;
        const double rotaryMass = section->m_MassInertiaPerLength.x() > 0.0
                                      ? section->m_MassInertiaPerLength.x() * L0 / 6.0
                                      : polarMoment * material->m_Density * L0 / 6.0;
        const bool centeredMass = section->m_MassFirstMomentPerLength.x() == 0.0 &&
                                  section->m_MassFirstMomentPerLength.y() == 0.0 &&
                                  section->m_MassFirstMomentPerLength.z() == 0.0;
        if (centeredMass && m_ReferenceCenteredMass.rows() == elementDofs &&
            m_ReferenceCenteredMass.cols() == elementDofs &&
            m_ReferenceCenteredMassInReferenceOrder.rows() == 8 &&
            m_ReferenceCenteredMassInReferenceOrder.cols() == 8 &&
            referenceToNative == m_ReferenceCenteredMassDofMapping &&
            translationMass == m_ReferenceCenteredMassCoefficients[0] &&
            rotaryMass == m_ReferenceCenteredMassCoefficients[1] &&
            std::signbit(translationMass) == std::signbit(m_ReferenceCenteredMassCoefficients[0]) &&
            std::signbit(rotaryMass) == std::signbit(m_ReferenceCenteredMassCoefficients[1]))
        {
            me = m_ReferenceCenteredMass;
            m_ReferenceCenteredMassOrderCacheReady = true;
            return;
        }
        // 参考质量始终采用固定全局 X 扭转耦合；偏心一阶矩仍由截面参数给定。
        const Eigen::Vector3d coupling = Eigen::Vector3d::UnitX().cross(section->m_MassFirstMomentPerLength);
        const Eigen::Vector3d referenceCoupling(coupling[0], coupling[2], -coupling[1]);
        const double massSy = referenceCoupling[1] == 0.0 ? 0.0 : -referenceCoupling[1] * L0 / 6.0;
        const double massSz = referenceCoupling[2] == 0.0 ? 0.0 : referenceCoupling[2] * L0 / 6.0;
        MatrixXd referenceMass = MatrixXd::Zero(8, 8);
        referenceMass(0, 0) = referenceMass(1, 1) = referenceMass(2, 2) = referenceMass(4, 4) = referenceMass(5, 5) =
            referenceMass(6, 6) = 2.0 * translationMass;
        referenceMass(3, 3) = referenceMass(7, 7) = 2.0 * rotaryMass;
        referenceMass(0, 4) = referenceMass(4, 0) = translationMass;
        referenceMass(1, 3) = referenceMass(3, 1) = -2.0 * massSy;
        referenceMass(1, 5) = referenceMass(5, 1) = translationMass;
        referenceMass(1, 7) = referenceMass(7, 1) = -1.0 * massSy;
        referenceMass(2, 3) = referenceMass(3, 2) = 2.0 * massSz;
        referenceMass(2, 6) = referenceMass(6, 2) = translationMass;
        referenceMass(2, 7) = referenceMass(7, 2) = massSz;
        referenceMass(3, 5) = referenceMass(5, 3) = -1.0 * massSy;
        referenceMass(3, 6) = referenceMass(6, 3) = massSz;
        referenceMass(3, 7) = referenceMass(7, 3) = rotaryMass;
        referenceMass(5, 7) = referenceMass(7, 5) = -2.0 * massSy;
        referenceMass(6, 7) = referenceMass(7, 6) = 2.0 * massSz;
        if (referenceCoupling[0] != 0.0)
        {
            const double massSx = referenceCoupling[0] * L0 / 6.0;
            referenceMass(0, 3) = referenceMass(3, 0) = 2.0 * massSx;
            referenceMass(0, 7) = referenceMass(7, 0) = massSx;
            referenceMass(4, 3) = referenceMass(3, 4) = massSx;
            referenceMass(4, 7) = referenceMass(7, 4) = 2.0 * massSx;
        }
        constexpr std::array<double, 8> referenceSigns{1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0};
        me.setZero(elementDofs, elementDofs);
        for (int row = 0; row < 8; ++row)
            for (int column = 0; column < 8; ++column)
                me(referenceToNative[row], referenceToNative[column]) =
                    referenceSigns[row] * referenceSigns[column] * referenceMass(row, column);
        if (centeredMass)
        {
            m_ReferenceCenteredMass = me;
            m_ReferenceCenteredMassCoefficients = {translationMass, rotaryMass};
            m_ReferenceCenteredMassDofMapping = referenceToNative;
            // 从原生矩阵按原惯性路径反变换，保留矩阵零项的符号。
            m_ReferenceCenteredMassInReferenceOrder.resize(8, 8);
            for (int row = 0; row < 8; ++row)
                for (int column = 0; column < 8; ++column)
                    m_ReferenceCenteredMassInReferenceOrder(row, column) =
                        referenceSigns[row] * referenceSigns[column] *
                        me(referenceToNative[row], referenceToNative[column]);
            m_ReferenceCenteredMassOrderCacheReady = true;
        }
        return;
    }

    const double linearDensity = section->m_Area * material->m_Density;
    const double rotaryDensity = section->m_MassInertiaPerLength.x() > 0.0
                                     ? section->m_MassInertiaPerLength.x()
                                     : polarMoment * material->m_Density;
    const Eigen::Vector3d translationTwistMass = section->m_DiagnosticReferenceFixedInertialCoupling
                                                      ? Eigen::Vector3d::UnitX().cross(
                                                            section->m_MassFirstMomentPerLength)
                                                      : CurrentCableAxis(*this).cross(GetMassFirstMomentGlobal());
    MatrixXd canonicalMass = MatrixXd::Zero(8, 8);

    MatrixXd mu = MatrixXd::Zero(4, 4);
    mu(0, 0) = mu(1, 1) = mu(2, 2) = linearDensity;
    mu(3, 3) = rotaryDensity;
    for (int component = 0; component < 3; ++component)
        mu(component, 3) = mu(3, component) = translationTwistMass[component];

    canonicalMass.block<4, 4>(0, 0) = 2.0 * mu;
    canonicalMass.block<4, 4>(0, 4) = mu;
    canonicalMass.block<4, 4>(4, 4) = 2.0 * mu;
    canonicalMass.block<4, 4>(4, 0) = mu;
    canonicalMass *= (L0 / 6.0);
    const MatrixXd transform = BuildCableDofTransform(*this);
    me = transform.transpose() * canonicalMass * transform;
}

void ElementCable::GetDynamicContributions(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce,
                                           _OUT MatrixXd& gyroscopicMatrix, _OUT MatrixXd& centrifugalMatrix)
{
    EvaluateDynamicContributions(massMatrix, inertiaForce, gyroscopicMatrix, centrifugalMatrix, true);
}

void ElementCable::EvaluateDynamicContributions(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce,
                                                _OUT MatrixXd& gyroscopicMatrix, _OUT MatrixXd& centrifugalMatrix,
                                                bool includeTangents)
{
    Get_me_Consistent(massMatrix);
    const int elementDofs = static_cast<int>(massMatrix.rows());
    // 参考固定质量惯性仅使用加速度，偏心截面同样无需构造速度工作向量。
    const bool skipUnusedVelocity = m_ReferenceTssbnConstitutive;
    VectorXd velocity;
    if (!skipUnusedVelocity)
        velocity = VectorXd::Zero(elementDofs);
    VectorXd& acceleration = m_DynamicAccelerationScratch;
    acceleration.setZero(elementDofs);
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    int offset = 0;
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto node = m_pNode[nodeIndex].lock();
        if (!node)
            throw std::runtime_error("ElementCable node reference is invalid");
        const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIndex)];
        for (int component = 0; component < localDofs; ++component)
        {
            if (!skipUnusedVelocity && component < static_cast<int>(node->m_Velocity.size()))
                velocity[offset + component] = node->m_Velocity[static_cast<std::size_t>(component)];
            if (component < static_cast<int>(node->m_Acceleration.size()))
                acceleration[offset + component] = node->m_Acceleration[static_cast<std::size_t>(component)];
        }
        offset += localDofs;
    }
    if (m_ReferenceTssbnConstitutive)
    {
        std::array<int, 8> referenceToNative;
        GetReferenceDofMapping(referenceToNative);
        constexpr std::array<double, 8> referenceSigns{1.0, 1.0, -1.0, 1.0, 1.0, 1.0, -1.0, 1.0};
        MatrixXd referenceMass;
        if (!m_ReferenceCenteredMassOrderCacheReady)
            referenceMass.resize(8, 8);
        VectorXd& referenceAcceleration = m_ReferenceAccelerationScratch;
        referenceAcceleration.resize(8);
        for (int row = 0; row < 8; ++row)
        {
            referenceAcceleration[row] = referenceSigns[row] * acceleration[referenceToNative[row]];
            if (!m_ReferenceCenteredMassOrderCacheReady)
                for (int column = 0; column < 8; ++column)
                    referenceMass(row, column) = referenceSigns[row] * referenceSigns[column] *
                                                 massMatrix(referenceToNative[row], referenceToNative[column]);
        }
        // 参考共享梁节点的索惯性取材料角加速度 x，不取沿当前索轴的空间投影。
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            if (localDofCounts[static_cast<std::size_t>(nodeIndex)] != 6)
                continue;
            const auto node = m_pNode[static_cast<std::size_t>(nodeIndex)].lock();
            const int nodeOffset = nodeIndex == 0 ? 0 : localDofCounts[0];
            const Eigen::Vector3d angularAcceleration = acceleration.segment<3>(nodeOffset + 3);
            referenceAcceleration[nodeIndex * 4 + 3] = (node->m_Rg.transpose() * angularAcceleration).x();
        }
        const MatrixXd& massInReferenceOrder = m_ReferenceCenteredMassOrderCacheReady
                                                   ? m_ReferenceCenteredMassInReferenceOrder
                                                   : referenceMass;
        VectorXd& referenceForce = m_ReferenceForceScratch;
        referenceForce.noalias() = massInReferenceOrder * referenceAcceleration;
        inertiaForce.setZero(elementDofs);
        for (int row = 0; row < 8; ++row)
            inertiaForce[referenceToNative[row]] = referenceSigns[row] * referenceForce[row];
    }
    else
        inertiaForce = massMatrix * acceleration;
    if (includeTangents)
    {
        gyroscopicMatrix = MatrixXd::Zero(elementDofs, elementDofs);
        centrifugalMatrix = MatrixXd::Zero(elementDofs, elementDofs);
    }
    // 参考源索使用固定 M*a，没有沿当前索轴投影产生的附加惯性和构型切线。
    if (m_ReferenceTssbnConstitutive)
        return;

    // 共享 6-DOF 节点仅以角速度在当前索轴上的投影参与转动惯性。
    // 即使截面无偏心，端点移动也会改变投影；此处对前述一致质量的转动部分求导。
    // 偏心耦合项仍由后续流程处理。
    if (localDofCounts[0] == 6 || localDofCounts[1] == 6)
    {
        const int secondOffset = localDofCounts[0];
        const int firstRotation = 3;
        const double rotaryWeight =
            localDofCounts[0] == 4 ? 0.5 * massMatrix(firstRotation, firstRotation)
                                   : 0.5 * massMatrix.block<3, 3>(firstRotation, firstRotation).trace();
        if (rotaryWeight > 0.0)
        {
            const auto first = m_pNode[0].lock();
            const auto second = m_pNode[1].lock();
            const Eigen::Vector3d chord(second->m_X + second->m_Displacement[0] - first->m_X -
                                            first->m_Displacement[0],
                                        second->m_Y + second->m_Displacement[1] - first->m_Y -
                                            first->m_Displacement[1],
                                        second->m_Z + second->m_Displacement[2] - first->m_Z -
                                            first->m_Displacement[2]);
            const double length = chord.norm();
            const Eigen::Vector3d axis = CurrentCableAxis(*this);
            const double orientation = chord.dot(axis) >= 0.0 ? 1.0 : -1.0;
            const Eigen::Matrix3d projection = Eigen::Matrix3d::Identity() - axis * axis.transpose();
            const Eigen::Matrix3d axisGradient = orientation * projection / length;
            const Eigen::Vector3d relativeVelocity = velocity.segment<3>(secondOffset) - velocity.head<3>();
            const Eigen::Vector3d axisRate = axisGradient * relativeVelocity;
            const std::array<int, 2> nodeOffsets{0, secondOffset};
            std::array<Eigen::Vector3d, 2> angularVelocity{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
            std::array<Eigen::Vector3d, 2> angularAcceleration{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
            Eigen::Vector2d spin = Eigen::Vector2d::Zero();
            Eigen::Vector2d spinAcceleration = Eigen::Vector2d::Zero();
            Eigen::Vector2d spinAxisRate = Eigen::Vector2d::Zero();
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
            {
                const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
                {
                    spin[nodeIndex] = velocity[rotationOffset];
                    spinAcceleration[nodeIndex] = acceleration[rotationOffset];
                }
                else
                {
                    angularVelocity[static_cast<std::size_t>(nodeIndex)] = velocity.segment<3>(rotationOffset);
                    angularAcceleration[static_cast<std::size_t>(nodeIndex)] = acceleration.segment<3>(rotationOffset);
                    spin[nodeIndex] = axis.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]);
                    spinAcceleration[nodeIndex] = axis.dot(angularAcceleration[static_cast<std::size_t>(nodeIndex)]);
                    spinAxisRate[nodeIndex] = axisRate.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]);
                }
            }
            Eigen::Matrix2d rotaryWeights;
            rotaryWeights << 2.0, 1.0, 1.0, 2.0;
            rotaryWeights *= rotaryWeight;
            const Eigen::Vector2d angularMomentum = rotaryWeights * spin;
            const Eigen::Vector2d momentumRate = rotaryWeights * (spinAcceleration + spinAxisRate);
            const Eigen::Vector2d convectiveMomentumRate = rotaryWeights * spinAxisRate;
            Eigen::Vector3d axisMomentumGradient = Eigen::Vector3d::Zero();
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
                    axisMomentumGradient += angularMomentum[nodeIndex] *
                                            angularVelocity[static_cast<std::size_t>(nodeIndex)];
            const Eigen::Vector3d positionForce = axisGradient * axisMomentumGradient;
            inertiaForce.head<3>() += positionForce;
            inertiaForce.segment<3>(secondOffset) -= positionForce;
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
            {
                const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
                    inertiaForce[rotationOffset] += convectiveMomentumRate[nodeIndex];
                else
                    inertiaForce.segment<3>(rotationOffset) += axis * convectiveMomentumRate[nodeIndex] +
                                                               angularMomentum[nodeIndex] * axisRate -
                                                               angularVelocity[static_cast<std::size_t>(nodeIndex)]
                                                                   .cross(axis * angularMomentum[nodeIndex]);
            }

            if (includeTangents)
            {
                // 用方向导数精确计算该转动质量项对速度与位置的切线，无需有限差分。
                for (int column = 0; column < elementDofs; ++column)
                {
                    Eigen::Vector3d relativeVelocityVariation = Eigen::Vector3d::Zero();
                    Eigen::Vector2d spinVariation = Eigen::Vector2d::Zero();
                    std::array<Eigen::Vector3d, 2> angularVelocityVariation{Eigen::Vector3d::Zero(),
                                                                            Eigen::Vector3d::Zero()};
                    if (column < 3)
                        relativeVelocityVariation[column] = -1.0;
                    else if (column >= secondOffset && column < secondOffset + 3)
                        relativeVelocityVariation[column - secondOffset] = 1.0;
                    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                    {
                        const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4 && column == rotationOffset)
                            spinVariation[nodeIndex] = 1.0;
                        else if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6 && column >= rotationOffset &&
                                 column < rotationOffset + 3)
                        {
                            angularVelocityVariation[static_cast<std::size_t>(nodeIndex)][column - rotationOffset] =
                                1.0;
                            spinVariation[nodeIndex] = axis[column - rotationOffset];
                        }
                    }
                    const Eigen::Vector3d axisRateVariation = axisGradient * relativeVelocityVariation;
                    const Eigen::Vector2d momentumVariation = rotaryWeights * spinVariation;
                    Eigen::Vector2d spinAxisRateVariation = Eigen::Vector2d::Zero();
                    Eigen::Vector3d momentumGradientVariation = Eigen::Vector3d::Zero();
                    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
                        {
                            spinAxisRateVariation[nodeIndex] =
                                axisRateVariation.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]) +
                                axisRate.dot(angularVelocityVariation[static_cast<std::size_t>(nodeIndex)]);
                            momentumGradientVariation +=
                                momentumVariation[nodeIndex] * angularVelocity[static_cast<std::size_t>(nodeIndex)] +
                                angularMomentum[nodeIndex] *
                                    angularVelocityVariation[static_cast<std::size_t>(nodeIndex)];
                        }
                    const Eigen::Vector2d convectiveRateVariation = rotaryWeights * spinAxisRateVariation;
                    const Eigen::Vector3d positionForceVariation = axisGradient * momentumGradientVariation;
                    gyroscopicMatrix.block<3, 1>(0, column) += positionForceVariation;
                    gyroscopicMatrix.block<3, 1>(secondOffset, column) -= positionForceVariation;
                    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                    {
                        const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
                            gyroscopicMatrix(rotationOffset, column) += convectiveRateVariation[nodeIndex];
                        else
                            gyroscopicMatrix.block<3, 1>(rotationOffset, column) +=
                                axis * convectiveRateVariation[nodeIndex] + momentumVariation[nodeIndex] * axisRate +
                                angularMomentum[nodeIndex] * axisRateVariation -
                                angularVelocityVariation[static_cast<std::size_t>(nodeIndex)]
                                    .cross(axis * angularMomentum[nodeIndex]) -
                                angularVelocity[static_cast<std::size_t>(nodeIndex)]
                                    .cross(axis * momentumVariation[nodeIndex]);
                    }
                }

                for (int column : {0, 1, 2, secondOffset, secondOffset + 1, secondOffset + 2})
                {
                    Eigen::Vector3d chordVariation = Eigen::Vector3d::Zero();
                    chordVariation[column < 3 ? column : column - secondOffset] = column < 3 ? -1.0 : 1.0;
                    const Eigen::Vector3d axisVariation = axisGradient * chordVariation;
                    const double lengthVariation = orientation * axis.dot(chordVariation);
                    const Eigen::Matrix3d axisGradientVariation =
                        -orientation / length * (axisVariation * axis.transpose() + axis * axisVariation.transpose()) -
                        orientation * lengthVariation / (length * length) * projection;
                    const Eigen::Vector3d axisRateVariation = axisGradientVariation * relativeVelocity;
                    Eigen::Vector2d spinVariation = Eigen::Vector2d::Zero();
                    Eigen::Vector2d spinRateVariation = Eigen::Vector2d::Zero();
                    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
                        {
                            spinVariation[nodeIndex] =
                                axisVariation.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]);
                            spinRateVariation[nodeIndex] =
                                axisVariation.dot(angularAcceleration[static_cast<std::size_t>(nodeIndex)]) +
                                axisRateVariation.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]);
                        }
                    const Eigen::Vector2d momentumVariation = rotaryWeights * spinVariation;
                    const Eigen::Vector2d momentumRateVariation = rotaryWeights * spinRateVariation;
                    Eigen::Vector3d momentumGradientVariation = Eigen::Vector3d::Zero();
                    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
                            momentumGradientVariation +=
                                momentumVariation[nodeIndex] * angularVelocity[static_cast<std::size_t>(nodeIndex)];
                    const Eigen::Vector3d positionForceVariation =
                        axisGradientVariation * axisMomentumGradient + axisGradient * momentumGradientVariation;
                    centrifugalMatrix.block<3, 1>(0, column) += positionForceVariation;
                    centrifugalMatrix.block<3, 1>(secondOffset, column) -= positionForceVariation;
                    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                    {
                        const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
                            centrifugalMatrix(rotationOffset, column) += momentumRateVariation[nodeIndex];
                        else
                            centrifugalMatrix.block<3, 1>(rotationOffset, column) +=
                                axisVariation * momentumRate[nodeIndex] + axis * momentumRateVariation[nodeIndex] +
                                momentumVariation[nodeIndex] * axisRate +
                                angularMomentum[nodeIndex] * axisRateVariation -
                                angularVelocity[static_cast<std::size_t>(nodeIndex)]
                                    .cross(axisVariation * angularMomentum[nodeIndex] +
                                           axis * momentumVariation[nodeIndex]);
                    }
                }
            }
        }
    }

    const auto property = m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!section || section->m_MassFirstMomentPerLength.squaredNorm() == 0.0 ||
        section->m_DiagnosticReferenceFixedInertialCoupling)
        return;

    // 偏心动能同时随弦轴和截面平均扭角变化；空间角速度使用当前弦轴投影。
    const Eigen::Vector3d axis = CurrentCableAxis(*this);
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    const Eigen::Vector3d chord(second->m_X + second->m_Displacement[0] - first->m_X -
                                    first->m_Displacement[0],
                                second->m_Y + second->m_Displacement[1] - first->m_Y -
                                    first->m_Displacement[1],
                                second->m_Z + second->m_Displacement[2] - first->m_Z -
                                    first->m_Displacement[2]);
    const double length = chord.norm();
    const double orientation = chord.dot(axis) >= 0.0 ? 1.0 : -1.0;
    const Eigen::Matrix3d projection = Eigen::Matrix3d::Identity() - axis * axis.transpose();
    const Eigen::Matrix3d axisGradient = orientation * projection / length;
    const int secondOffset = localDofCounts[0];
    const std::array<int, 2> nodeOffsets{0, secondOffset};
    const Eigen::Vector3d reference = section->m_MassFirstMomentPerLength;
    const Eigen::Vector3d coupling = axis.cross(GetMassFirstMomentGlobal());
    const Eigen::Vector3d couplingPrime = axis.cross(coupling);
    const double meanTwist = 0.5 * (GetNodalTwist(0) + GetNodalTwist(1));
    const double cosine = std::cos(meanTwist);
    const double sine = std::sin(meanTwist);
    Eigen::Matrix3d skewReference;
    Utility::CR::SkewSymmetric(reference, skewReference);
    const Eigen::Matrix3d axisCouplingGradient =
        -cosine * skewReference +
        sine * (axis.dot(reference) * Eigen::Matrix3d::Identity() + axis * reference.transpose());
    const Eigen::Matrix3d axisCouplingTwistGradient =
        sine * skewReference +
        cosine * (axis.dot(reference) * Eigen::Matrix3d::Identity() + axis * reference.transpose());
    Eigen::Matrix2d weights;
    weights << 2.0, 1.0, 1.0, 2.0;
    weights *= L0 / 6.0;
    VectorXd meanTwistGradient;
    MatrixXd meanTwistHessian;
    GetMeanTwistDerivatives(meanTwistGradient, includeTangents ? &meanTwistHessian : nullptr);

    std::array<Eigen::Vector3d, 2> translationVelocity;
    std::array<Eigen::Vector3d, 2> angularVelocity{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    std::array<Eigen::Vector3d, 2> translationAcceleration;
    std::array<Eigen::Vector3d, 2> angularAcceleration{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector2d spin = Eigen::Vector2d::Zero();
    Eigen::Vector2d spinAcceleration = Eigen::Vector2d::Zero();
    Eigen::Vector2d spinAxisRate = Eigen::Vector2d::Zero();
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
        translationVelocity[static_cast<std::size_t>(nodeIndex)] =
            velocity.segment<3>(nodeOffsets[static_cast<std::size_t>(nodeIndex)]);
        translationAcceleration[static_cast<std::size_t>(nodeIndex)] =
            acceleration.segment<3>(nodeOffsets[static_cast<std::size_t>(nodeIndex)]);
        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
        {
            spin[nodeIndex] = velocity[rotationOffset];
            spinAcceleration[nodeIndex] = acceleration[rotationOffset];
        }
        else
        {
            angularVelocity[static_cast<std::size_t>(nodeIndex)] = velocity.segment<3>(rotationOffset);
            angularAcceleration[static_cast<std::size_t>(nodeIndex)] = acceleration.segment<3>(rotationOffset);
            spin[nodeIndex] = axis.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]);
            spinAcceleration[nodeIndex] = axis.dot(angularAcceleration[static_cast<std::size_t>(nodeIndex)]);
        }
    }
    const Eigen::Vector3d relativeVelocity = translationVelocity[1] - translationVelocity[0];
    const Eigen::Vector3d axisRate = axisGradient * relativeVelocity;
    const double meanTwistRate = meanTwistGradient.dot(velocity);
    const Eigen::Vector3d couplingRate = axisCouplingGradient * axisRate + couplingPrime * meanTwistRate;
    const Eigen::Vector2d weightedSpin = weights * spin;
    Eigen::Vector2d translationWork = Eigen::Vector2d::Zero();
    Eigen::Vector3d weightedVelocity = Eigen::Vector3d::Zero();
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        weightedVelocity += weightedSpin[nodeIndex] * translationVelocity[static_cast<std::size_t>(nodeIndex)];
        for (int translationNode = 0; translationNode < 2; ++translationNode)
            translationWork[nodeIndex] +=
                weights(translationNode, nodeIndex) *
                translationVelocity[static_cast<std::size_t>(translationNode)].dot(coupling);
        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
            spinAxisRate[nodeIndex] = axisRate.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]);
    }
    const double twistWork = weightedVelocity.dot(couplingPrime);
    Eigen::Vector3d axisWork = axisCouplingGradient.transpose() * weightedVelocity;
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
            axisWork += translationWork[nodeIndex] * angularVelocity[static_cast<std::size_t>(nodeIndex)];
    const Eigen::Vector3d positionWork = axisGradient * axisWork;
    const Eigen::Vector2d weightedSpinAxisRate = weights * spinAxisRate;
    Eigen::Vector2d angularWork = Eigen::Vector2d::Zero();
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        for (int translationNode = 0; translationNode < 2; ++translationNode)
            angularWork[nodeIndex] += weights(translationNode, nodeIndex) *
                                      translationVelocity[static_cast<std::size_t>(translationNode)].dot(couplingRate);

    inertiaForce.head<3>() += positionWork - twistWork * meanTwistGradient.head<3>() +
                              couplingRate * weightedSpin[0] + coupling * weightedSpinAxisRate[0];
    inertiaForce.segment<3>(secondOffset) += -positionWork -
                                              twistWork * meanTwistGradient.segment<3>(secondOffset) +
                                              couplingRate * weightedSpin[1] + coupling * weightedSpinAxisRate[1];
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
        if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
            inertiaForce[rotationOffset] += angularWork[nodeIndex] -
                                            twistWork * meanTwistGradient[rotationOffset];
        else
            inertiaForce.segment<3>(rotationOffset) +=
                axis * angularWork[nodeIndex] + axisRate * translationWork[nodeIndex] -
                angularVelocity[static_cast<std::size_t>(nodeIndex)]
                    .cross(axis * translationWork[nodeIndex]) -
                twistWork * meanTwistGradient.segment<3>(rotationOffset);
    }

    if (!includeTangents)
        return;

    // 对位形和速度分别求方向导数，包含平均扭角 Hessian 与空间转角运输项。
    for (int derivativeKind = 0; derivativeKind < 2; ++derivativeKind)
        for (int column = 0; column < elementDofs; ++column)
        {
            const bool configurationDerivative = derivativeKind == 1;
            MatrixXd& tangent = configurationDerivative ? centrifugalMatrix : gyroscopicMatrix;
            std::array<Eigen::Vector3d, 2> velocityVariation{Eigen::Vector3d::Zero(),
                                                                Eigen::Vector3d::Zero()};
            std::array<Eigen::Vector3d, 2> angularVelocityVariation{Eigen::Vector3d::Zero(),
                                                                       Eigen::Vector3d::Zero()};
            Eigen::Vector3d chordVariation = Eigen::Vector3d::Zero();
            if (column < 3)
            {
                if (configurationDerivative)
                    chordVariation[column] = -1.0;
                else
                    velocityVariation[0][column] = 1.0;
            }
            else if (column >= secondOffset && column < secondOffset + 3)
            {
                if (configurationDerivative)
                    chordVariation[column - secondOffset] = 1.0;
                else
                    velocityVariation[1][column - secondOffset] = 1.0;
            }
            else if (!configurationDerivative)
                for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                {
                    const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                    if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6 && column >= rotationOffset &&
                        column < rotationOffset + 3)
                        angularVelocityVariation[static_cast<std::size_t>(nodeIndex)][column - rotationOffset] = 1.0;
                }
            Eigen::Vector3d axisVariation = Eigen::Vector3d::Zero();
            Eigen::Matrix3d axisGradientVariation = Eigen::Matrix3d::Zero();
            if (configurationDerivative)
                axisVariation = axisGradient * chordVariation;
            const double lengthVariation = orientation * axis.dot(chordVariation);
            if (configurationDerivative)
                axisGradientVariation =
                    -orientation / length * (axisVariation * axis.transpose() + axis * axisVariation.transpose()) -
                    orientation * lengthVariation / (length * length) * projection;
            const double twistVariation = configurationDerivative ? meanTwistGradient[column] : 0.0;
            VectorXd twistGradientVariation = VectorXd::Zero(elementDofs);
            if (configurationDerivative)
            {
                twistGradientVariation = meanTwistHessian.col(column);
                for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                {
                    const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                    if (localDofCounts[static_cast<std::size_t>(nodeIndex)] != 6 || column < rotationOffset ||
                        column >= rotationOffset + 3)
                        continue;
                    Eigen::Vector3d basis = Eigen::Vector3d::Zero();
                    basis[column - rotationOffset] = 1.0;
                    twistGradientVariation.segment<3>(rotationOffset) -=
                        0.5 * meanTwistGradient.segment<3>(rotationOffset).cross(basis);
                }
            }
            const Eigen::Vector3d couplingVariation =
                axisCouplingGradient * axisVariation + couplingPrime * twistVariation;
            const Eigen::Vector3d couplingPrimeVariation =
                axisCouplingTwistGradient * axisVariation - coupling * twistVariation;
            const Eigen::Matrix3d couplingGradientVariation =
                axisCouplingTwistGradient * twistVariation +
                sine * (axisVariation.dot(reference) * Eigen::Matrix3d::Identity() +
                        axisVariation * reference.transpose());
            Eigen::Vector2d spinVariation = Eigen::Vector2d::Zero();
            Eigen::Vector2d spinAxisRateVariation = Eigen::Vector2d::Zero();
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
            {
                const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
                    spinVariation[nodeIndex] = !configurationDerivative && column == rotationOffset ? 1.0 : 0.0;
                else
                    spinVariation[nodeIndex] =
                        axisVariation.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]) +
                        axis.dot(angularVelocityVariation[static_cast<std::size_t>(nodeIndex)]);
            }
            const Eigen::Vector2d weightedSpinVariation = weights * spinVariation;
            Eigen::Vector2d translationWorkVariation = Eigen::Vector2d::Zero();
            Eigen::Vector3d weightedVelocityVariation = Eigen::Vector3d::Zero();
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
            {
                weightedVelocityVariation +=
                    weightedSpinVariation[nodeIndex] * translationVelocity[static_cast<std::size_t>(nodeIndex)] +
                    weightedSpin[nodeIndex] * velocityVariation[static_cast<std::size_t>(nodeIndex)];
                for (int translationNode = 0; translationNode < 2; ++translationNode)
                    translationWorkVariation[nodeIndex] += weights(translationNode, nodeIndex) *
                                                           (velocityVariation[static_cast<std::size_t>(translationNode)]
                                                                .dot(coupling) +
                                                            translationVelocity[static_cast<std::size_t>(translationNode)]
                                                                .dot(couplingVariation));
            }
            const double twistWorkVariation = weightedVelocityVariation.dot(couplingPrime) +
                                              weightedVelocity.dot(couplingPrimeVariation);
            Eigen::Vector3d axisWorkVariation = couplingGradientVariation.transpose() * weightedVelocity +
                                                axisCouplingGradient.transpose() * weightedVelocityVariation;
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
                    axisWorkVariation +=
                        translationWorkVariation[nodeIndex] *
                            angularVelocity[static_cast<std::size_t>(nodeIndex)] +
                        translationWork[nodeIndex] *
                            angularVelocityVariation[static_cast<std::size_t>(nodeIndex)];
            const Eigen::Vector3d axisRateVariation = axisGradientVariation * relativeVelocity +
                                                       axisGradient * (velocityVariation[1] - velocityVariation[0]);
            const double meanTwistRateVariation =
                configurationDerivative ? twistGradientVariation.dot(velocity) : meanTwistGradient[column];
            const Eigen::Vector3d couplingRateVariation =
                couplingGradientVariation * axisRate + axisCouplingGradient * axisRateVariation +
                couplingPrimeVariation * meanTwistRate + couplingPrime * meanTwistRateVariation;
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
                    spinAxisRateVariation[nodeIndex] =
                        axisRateVariation.dot(angularVelocity[static_cast<std::size_t>(nodeIndex)]) +
                        axisRate.dot(angularVelocityVariation[static_cast<std::size_t>(nodeIndex)]);
            const Eigen::Vector2d weightedSpinAxisRateVariation = weights * spinAxisRateVariation;
            Eigen::Vector2d angularWorkVariation = Eigen::Vector2d::Zero();
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                for (int translationNode = 0; translationNode < 2; ++translationNode)
                    angularWorkVariation[nodeIndex] +=
                        weights(translationNode, nodeIndex) *
                        (velocityVariation[static_cast<std::size_t>(translationNode)].dot(couplingRate) +
                         translationVelocity[static_cast<std::size_t>(translationNode)].dot(couplingRateVariation));

            Eigen::Vector3d positionWorkVariation = axisGradientVariation * axisWork +
                                                    axisGradient * axisWorkVariation;
            tangent.block<3, 1>(0, column) +=
                positionWorkVariation - twistWorkVariation * meanTwistGradient.head<3>() -
                twistWork * twistGradientVariation.head<3>() + couplingRateVariation * weightedSpin[0] +
                couplingRate * weightedSpinVariation[0] + couplingVariation * weightedSpinAxisRate[0] +
                coupling * weightedSpinAxisRateVariation[0];
            tangent.block<3, 1>(secondOffset, column) +=
                -positionWorkVariation - twistWorkVariation * meanTwistGradient.segment<3>(secondOffset) -
                twistWork * twistGradientVariation.segment<3>(secondOffset) +
                couplingRateVariation * weightedSpin[1] + couplingRate * weightedSpinVariation[1] +
                couplingVariation * weightedSpinAxisRate[1] + coupling * weightedSpinAxisRateVariation[1];
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
            {
                const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
                    tangent(rotationOffset, column) += angularWorkVariation[nodeIndex] -
                                                       twistWorkVariation * meanTwistGradient[rotationOffset] -
                                                       twistWork * twistGradientVariation[rotationOffset];
                else
                    tangent.block<3, 1>(rotationOffset, column) +=
                        axisVariation * angularWork[nodeIndex] + axis * angularWorkVariation[nodeIndex] +
                        axisRateVariation * translationWork[nodeIndex] +
                        axisRate * translationWorkVariation[nodeIndex] -
                        angularVelocityVariation[static_cast<std::size_t>(nodeIndex)]
                            .cross(axis * translationWork[nodeIndex]) -
                        angularVelocity[static_cast<std::size_t>(nodeIndex)]
                            .cross(axisVariation * translationWork[nodeIndex] +
                                   axis * translationWorkVariation[nodeIndex]) -
                        twistWorkVariation * meanTwistGradient.segment<3>(rotationOffset) -
                        twistWork * twistGradientVariation.segment<3>(rotationOffset);
            }

            if (!configurationDerivative)
                continue;
            Eigen::Vector2d spinAccelerationVariation = Eigen::Vector2d::Zero();
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 6)
                    spinAccelerationVariation[nodeIndex] =
                        axisVariation.dot(angularAcceleration[static_cast<std::size_t>(nodeIndex)]);
            const Eigen::Vector2d weightedSpinAcceleration = weights * spinAcceleration;
            const Eigen::Vector2d weightedSpinAccelerationVariation = weights * spinAccelerationVariation;
            tangent.block<3, 1>(0, column) += couplingVariation * weightedSpinAcceleration[0] +
                                                coupling * weightedSpinAccelerationVariation[0];
            tangent.block<3, 1>(secondOffset, column) +=
                couplingVariation * weightedSpinAcceleration[1] +
                coupling * weightedSpinAccelerationVariation[1];
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
            {
                const int rotationOffset = nodeOffsets[static_cast<std::size_t>(nodeIndex)] + 3;
                double accelerationWork = 0.0;
                double accelerationWorkVariation = 0.0;
                for (int translationNode = 0; translationNode < 2; ++translationNode)
                {
                    accelerationWork += weights(translationNode, nodeIndex) *
                                        translationAcceleration[static_cast<std::size_t>(translationNode)]
                                            .dot(coupling);
                    accelerationWorkVariation +=
                        weights(translationNode, nodeIndex) *
                        translationAcceleration[static_cast<std::size_t>(translationNode)].dot(couplingVariation);
                }
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] == 4)
                    tangent(rotationOffset, column) += accelerationWorkVariation;
                else
                    tangent.block<3, 1>(rotationOffset, column) +=
                        axisVariation * accelerationWork + axis * accelerationWorkVariation;
            }
        }
}

void ElementCable::Get_InertiaForce(_OUT VectorXd& inertiaForce)
{
    MatrixXd mass, velocityTangent, configurationTangent;
    EvaluateDynamicContributions(mass, inertiaForce, velocityTangent, configurationTangent, false);
}

void ElementCable::GetMassAndInertiaForce(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce)
{
    MatrixXd velocityTangent, configurationTangent;
    EvaluateDynamicContributions(massMatrix, inertiaForce, velocityTangent, configurationTangent, false);
}

void ElementCable::Get_GyroscopicMatrix(_OUT MatrixXd& gyroscopicMatrix)
{
    MatrixXd mass, configurationTangent;
    VectorXd inertiaForce;
    GetDynamicContributions(mass, inertiaForce, gyroscopicMatrix, configurationTangent);
}

void ElementCable::Get_CentrifugalMatrix(_OUT MatrixXd& centrifugalMatrix)
{
    MatrixXd mass, velocityTangent;
    VectorXd inertiaForce;
    GetDynamicContributions(mass, inertiaForce, velocityTangent, centrifugalMatrix);
}

void ElementCable::Get_L0()
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pNode0 || !pNode1)
        throw std::runtime_error("ElementCable node reference is invalid");

    const Eigen::Vector3d x0(pNode0->m_X, pNode0->m_Y, pNode0->m_Z);
    const Eigen::Vector3d x1(pNode1->m_X, pNode1->m_Y, pNode1->m_Z);
    if (m_ReferenceTssbnConstitutive)
    {
        const Eigen::Vector3d reference0(x0[0], x0[2], -x0[1]);
        const Eigen::Vector3d reference1(x1[0], x1[2], -x1[1]);
        L0 = (reference1 - reference0).norm();
    }
    else
        L0 = (x1 - x0).norm();
    if (L0 <= 1.0e-12)
        throw std::runtime_error("ElementCable initial length must be positive");
    if (L <= 0.0)
        L = L0;
}

void ElementCable::GetRayleighDamping(const std::array<double, 4>& coefficients, const MatrixXd& stiffness,
                                    const MatrixXd& physicalMass, _OUT MatrixXd& damping) const
{
    std::vector<int> localDofCounts;
    GetNodeLocalDOFCounts(localDofCounts);
    const int size = std::accumulate(localDofCounts.cbegin(), localDofCounts.cend(), 0);
    std::vector<bool> rotational(static_cast<std::size_t>(size), false);
    int offset = 0;
    for (int count : localDofCounts)
    {
        // 共享节点的轴扭转已由索的K/M映射到空间转动分量，不引入梁的弯曲刚度。
        for (int component = 3; component < count; ++component)
            rotational[static_cast<std::size_t>(offset + component)] = true;
        offset += count;
    }
    SolverNameSpace::BuildLocalRayleighMatrix(stiffness, physicalMass, rotational, coefficients, damping);
}

void ElementCable::Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce)
{
    if (damping.size() < 4)
        throw std::invalid_argument("damping vector must have at least 4 elements");
    GetRayleighDamping({damping[0], damping[1], damping[2], damping[3]}, ce);
}

void ElementCable::Assemble(const std::vector<double>& damping, const MatrixXd& ke, const MatrixXd& me,
                            _OUT MatrixXd& ce)
{
    if (damping.size() < 4)
        throw std::invalid_argument("damping vector must have at least 4 elements");
    GetRayleighDamping({damping[0], damping[1], damping[2], damping[3]}, ke, me, ce);
}
