#include "LoadAssembler.h"

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <QElapsedTimer>

#include "DataStructure/Aerodynamics/AeroManager.h"
#include "DataStructure/Aerodynamics/BundleAeroMapper.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Element/ElementSpringBase.h"
#include "DataStructure/Load/AerodynamicLoadCalculator.h"
#include "DataStructure/Load/Force_Element.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Node.h"
#include "DataStructure/Load/Force_Wind.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Node/Node.h"
#include "DataStructure/Structure/StructureData.h"
#include "Solver/Assembly/AssemblySettings.h"
#include "Solver/Assembly/ParallelAssemblyEngine.h"
#include "Utility/CR.h"

static const std::vector<BladeModel>* ResolveGallopingCaseData(const StructureData& structure, int bundleCount,
                                                              double windSpeed, int iceThickness)
{
    const AeroCaseKey key{bundleCount, static_cast<int>(std::llround(windSpeed)), iceThickness};
    return structure.m_AeroManager.findCaseModels(key);
}

namespace
{
using Vec = Eigen::VectorXd;
using Vec3 = Eigen::Vector3d;

AeroCoefficients ReferenceSingleConductorFit(double radians, AeroCoefficients* slopes = nullptr,
                                            const AeroCoefficients* outsideDomain = nullptr)
{
    const double maximumFitAngle = 100.0 * std::acos(-1.0) / 180.0;
    if (!outsideDomain && (radians < 0.0 || radians > maximumFitAngle))
    {
        static std::atomic_bool warned = false;
        if (!warned.exchange(true))
            std::cerr << "Warning: reference fifth-order aerodynamic fit exceeded 0-100 degree domain; "
                         "use the bounded-fit diagnostic with 360-degree data.\n";
    }
    // 旧参考源码的全域外推仅用于对照；论文诊断在有效域外改用360°数据。
    double distanceToFitBoundary = 0.0;
    if (outsideDomain)
    {
        const double fullTurn = 2.0 * std::acos(-1.0);
        radians = std::fmod(radians, fullTurn);
        if (radians < 0.0)
            radians += fullTurn;
        if (radians > maximumFitAngle)
            return *outsideDomain;
        distanceToFitBoundary = std::min(radians, maximumFitAngle - radians);
    }
    // Reference Wind_method/AerodynamicData.cpp, 12 mm iced single conductor.
    const auto polynomial = [radians](const std::array<double, 6>& a, double* slope)
    {
        double value = a[5];
        double derivative = 0.0;
        for (int i = 4; i >= 0; --i)
        {
            derivative = derivative * radians + value;
            value = value * radians + a[static_cast<std::size_t>(i)];
        }
        if (slope)
            *slope = derivative;
        return value;
    };
    AeroCoefficients result;
    result.lift = polynomial({0.2543936, 0.9188796, 3.780506, -8.131483, 4.886842, -0.9613859},
                             slopes ? &slopes->lift : nullptr);
    result.drag = polynomial({0.8800010, -0.9692127, 6.373413, -6.923727, 3.253336, -0.5900608},
                             slopes ? &slopes->drag : nullptr);
    result.moment = polynomial({0.1037828, 1.457832, 0.1624594, -2.235872, 1.620168, -0.3619016},
                               slopes ? &slopes->moment : nullptr);
    if (outsideDomain)
    {
        const double transitionWidth = 5.0 * std::acos(-1.0) / 180.0;
        const double fraction = std::clamp(distanceToFitBoundary / transitionWidth, 0.0, 1.0);
        const double weight = fraction * fraction * (3.0 - 2.0 * fraction);
        result.lift = (1.0 - weight) * outsideDomain->lift + weight * result.lift;
        result.drag = (1.0 - weight) * outsideDomain->drag + weight * result.drag;
        result.moment = (1.0 - weight) * outsideDomain->moment + weight * result.moment;
    }
    return result;
}

bool TranslationComponent(EnumKeyword::Direction direction, int& component)
{
    component = static_cast<int>(direction);
    return component >= 0 && component < 3;
}

void AddElementForce(const ElementBase& element, const Vec& force, int fixedDofs, Vec& fixed, Vec& free)
{
    std::vector<int> dofs;
    element.GetDOFs(dofs);
    if (dofs.size() != static_cast<std::size_t>(force.size()))
        throw std::runtime_error("Element load vector size does not match its DOFs");

    for (int i = 0; i < force.size(); ++i)
    {
        const int dof = dofs[static_cast<std::size_t>(i)];
        if (dof < 0)
            continue;
        if (dof < fixedDofs)
        {
            if (dof >= fixed.size())
                throw std::runtime_error("Element load fixed DOF is out of range");
            fixed[dof] += force[i];
        }
        else
        {
            if (dof - fixedDofs >= free.size())
                throw std::runtime_error("Element load free DOF is out of range");
            free[dof - fixedDofs] += force[i];
        }
    }
}

void AddNodeForce(const Node& node, int component, double value, int fixedDofs, Vec& fixed, Vec& free)
{
    if (component < 0 || component >= node.m_DOF.size())
        return;
    const int dof = node.m_DOF[component];
    if (dof < 0)
        return;
    if (dof < fixedDofs)
    {
        if (dof >= fixed.size())
            throw std::runtime_error("Node load fixed DOF is out of range");
        fixed[dof] += value;
    }
    else
    {
        if (dof - fixedDofs >= free.size())
            throw std::runtime_error("Node load free DOF is out of range");
        free[dof - fixedDofs] += value;
    }
}

Vec SpatialUniformLineLoad(const ElementBase& element, const Vec3& firstPosition, const Vec3& secondPosition,
                           const Vec3& q)
{
    if (element.m_pNode.size() != 2)
        throw std::runtime_error("Distributed load requires a two-node element");
    const Vec3 chord = secondPosition - firstPosition;
    const double length = chord.norm();
    if (length <= 1.0e-12)
        throw std::runtime_error("Element current length must be positive");

    std::vector<int> localDofCounts;
    element.GetNodeLocalDOFCounts(localDofCounts);
    if (localDofCounts.size() != 2 || localDofCounts[0] < 3 || localDofCounts[1] < 3)
        throw std::runtime_error("Distributed load requires translational DOFs");

    const int secondOffset = localDofCounts[0];
    Vec result = Vec::Zero(localDofCounts[0] + localDofCounts[1]);
    result.segment<3>(0) = 0.5 * length * q;
    result.segment<3>(secondOffset) = 0.5 * length * q;

    if (element.Get_NodeDOF() >= 6 && localDofCounts[0] >= 6 && localDofCounts[1] >= 6)
    {
        const Vec3 endMoment = length * length / 12.0 * chord.normalized().cross(q);
        result.segment<3>(3) = endMoment;
        result.segment<3>(secondOffset + 3) = -endMoment;
    }
    return result;
}

Vec SpatialUniformLineLoad(const ElementBase& element, const Vec3& q)
{
    const auto first = element.m_pNode[0].lock();
    const auto second = element.m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("Element node reference is invalid");
    const Vec3 firstPosition(first->m_X + first->m_Displacement[0], first->m_Y + first->m_Displacement[1],
                             first->m_Z + first->m_Displacement[2]);
    const Vec3 secondPosition(second->m_X + second->m_Displacement[0], second->m_Y + second->m_Displacement[1],
                              second->m_Z + second->m_Displacement[2]);
    return SpatialUniformLineLoad(element, firstPosition, secondPosition, q);
}

Eigen::MatrixXd SpatialUniformLineLoadTangent(const ElementBase& element, const Vec3& firstPosition,
                                              const Vec3& secondPosition, const Vec3& q)
{
    const Vec3 chord = secondPosition - firstPosition;
    const double length = chord.norm();
    if (length <= 1.0e-12)
        throw std::runtime_error("Element current length must be positive");
    std::vector<int> localDofCounts;
    element.GetNodeLocalDOFCounts(localDofCounts);
    if (localDofCounts.size() != 2 || localDofCounts[0] < 3 || localDofCounts[1] < 3)
        throw std::runtime_error("Distributed load requires translational DOFs");

    const int secondOffset = localDofCounts[0];
    const int elementDofs = secondOffset + localDofCounts[1];
    const bool hasEndMoments = element.Get_NodeDOF() >= 6 &&
                               localDofCounts[0] >= 6 && localDofCounts[1] >= 6;
    Eigen::MatrixXd tangent = Eigen::MatrixXd::Zero(elementDofs, elementDofs);
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const int columnOffset = nodeIndex == 0 ? 0 : secondOffset;
        const double chordSign = nodeIndex == 0 ? -1.0 : 1.0;
        for (int component = 0; component < 3; ++component)
        {
            const Vec3 chordDerivative = chordSign * Vec3::Unit(component);
            const double lengthDerivative = chord.dot(chordDerivative) / length;
            const Vec3 nodalForceDerivative = 0.5 * lengthDerivative * q;
            const int column = columnOffset + component;
            tangent.block<3, 1>(0, column) = nodalForceDerivative;
            tangent.block<3, 1>(secondOffset, column) = nodalForceDerivative;
            if (hasEndMoments)
            {
                const Vec3 endMomentDerivative =
                    (lengthDerivative * chord.cross(q) + length * chordDerivative.cross(q)) / 12.0;
                tangent.block<3, 1>(3, column) = endMomentDerivative;
                tangent.block<3, 1>(secondOffset + 3, column) = -endMomentDerivative;
            }
        }
    }
    return tangent;
}

Vec3 GlobalLineLoad(EnumKeyword::Direction direction, double magnitude)
{
    int component = -1;
    if (!TranslationComponent(direction, component))
        throw std::runtime_error("Distributed load direction must be X, Y, or Z");
    Vec3 q = Vec3::Zero();
    q[component] = magnitude;
    return q;
}

struct EccentricGravityState
{
    std::array<Vec3, 2> positions;
    std::array<double, 2> twists;
};

Vec EccentricGravityElementForce(const ElementCable& cable, const Vec3& referenceFirstMoment,
                                 const Vec3& gravityAcceleration, const EccentricGravityState& state,
                                 _OUT double* totalTwistDerivative = nullptr)
{
    std::vector<int> localDofCounts;
    cable.GetNodeLocalDOFCounts(localDofCounts);
    const int secondOffset = localDofCounts[0];
    Vec force = Vec::Zero(secondOffset + localDofCounts[1]);
    const Vec3 chord = state.positions[1] - state.positions[0];
    const double length = chord.norm();
    if (length <= 1.0e-12)
        throw std::runtime_error("Eccentric ice gravity requires positive cable length");
    const Vec3 axis = Utility::CR::CanonicalAxis(chord);
    // 势能用不随单元节点次序变化的规范轴定义偏心方向。对节点位置求导时，
    // 实际 chord 与规范轴可能反向，长度及轴方向的导数都需保留这个符号。
    const double chordOrientation = chord.dot(axis) / length;
    const Vec3 transverseMoment = referenceFirstMoment - axis.dot(referenceFirstMoment) * axis;
    const double meanTwist = 0.5 * (state.twists[0] + state.twists[1]);
    const Vec3 currentMoment = Eigen::AngleAxisd(meanTwist, axis) * transverseMoment;
    const double axialMoment = axis.dot(referenceFirstMoment);
    const double axialGravity = axis.dot(gravityAcceleration);
    const Vec3 chordForce =
        std::cos(meanTwist) * ((referenceFirstMoment.dot(gravityAcceleration) + axialMoment * axialGravity) * axis -
                               axialGravity * referenceFirstMoment - axialMoment * gravityAcceleration) +
        std::sin(meanTwist) * referenceFirstMoment.cross(gravityAcceleration);
    force.segment<3>(0) = -chordOrientation * chordForce;
    force.segment<3>(secondOffset) = chordOrientation * chordForce;
    const double nodalTorque = 0.5 * length * axis.dot(currentMoment.cross(gravityAcceleration));
    if (totalTwistDerivative)
        *totalTwistDerivative = 2.0 * nodalTorque;
    if (localDofCounts[0] == 4)
        force[3] = nodalTorque;
    else
        force.segment<3>(3) = nodalTorque * axis;
    if (localDofCounts[1] == 4)
        force[secondOffset + 3] = nodalTorque;
    else
        force.segment<3>(secondOffset + 3) = nodalTorque * axis;
    return force;
}

EccentricGravityState CurrentEccentricGravityState(const ElementCable& cable)
{
    EccentricGravityState state;
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto node = cable.m_pNode[static_cast<std::size_t>(nodeIndex)].lock();
        if (!node)
            throw std::runtime_error("Eccentric ice gravity node is invalid");
        state.positions[static_cast<std::size_t>(nodeIndex)] =
            Vec3(node->m_X + node->m_Displacement[0], node->m_Y + node->m_Displacement[1],
                 node->m_Z + node->m_Displacement[2]);
        state.twists[static_cast<std::size_t>(nodeIndex)] = cable.GetNodalTwist(nodeIndex);
    }
    return state;
}

Vec EccentricGravityConsistentElementForce(const ElementCable& cable, const Vec3& referenceFirstMoment,
                                           const Vec3& gravityAcceleration)
{
    const EccentricGravityState state = CurrentEccentricGravityState(cable);
    double twistDerivative = 0.0;
    Vec force = EccentricGravityElementForce(cable, referenceFirstMoment, gravityAcceleration,
                                            state, &twistDerivative);
    std::vector<int> counts;
    cable.GetNodeLocalDOFCounts(counts);
    if (counts[0] == 4 && counts[1] == 4)
        return force;
    // 上方位置偏导固定平均扭角；空间节点扭角还随左乘旋转和弦线变化。
    // 与索单元内扭转力共用同一扭角梯度。
    force.segment(3, counts[0] - 3).setZero();
    force.segment(counts[0] + 3, counts[1] - 3).setZero();
    Vec twistGradient;
    cable.GetMeanTwistDerivatives(twistGradient);
    force.noalias() += twistDerivative * twistGradient;
    return force;
}

void AssembleGravity(const Force_Gravity& load, const StructureData& structure, int fixedDofs, double scale, Vec& fixed,
                     Vec& free)
{
    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element)
            continue;
        const auto property = element->m_pProperty.lock();
        const auto material = property ? property->m_pMaterial.lock() : nullptr;
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!material || !section)
            throw std::runtime_error("Gravity load requires material and section");

        const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
        const auto truss = std::dynamic_pointer_cast<ElementTruss>(element);
        const double q = material->m_Density * section->m_Area * load.m_g * scale;
        const Vec3 gravityPerLength = GlobalLineLoad(load.m_Direction, q);
        Vec elementForce;
        if (truss)
        {
            // 桁架重力作用于恒定参考质量；拉伸、压缩和转动均不改变总重。
            Eigen::MatrixXd mass;
            truss->Get_me_Physical(mass);
            const Vec3 gravityAcceleration = GlobalLineLoad(load.m_Direction, load.m_g * scale);
            Vec acceleration(6);
            acceleration << gravityAcceleration, gravityAcceleration;
            elementForce = mass * acceleration;
        }
        else if (cable)
        {
            const auto first = cable->m_pNode[0].lock();
            const auto second = cable->m_pNode[1].lock();
            if (!first || !second)
                throw std::runtime_error("Gravity cable node reference is invalid");
            if (cable->IsReferenceTssbnConstitutiveMode())
            {
                const Vec3 firstReference(first->m_X, first->m_Z, -first->m_Y);
                const Vec3 secondReference(second->m_X, second->m_Z, -second->m_Y);
                // 参考静力冻结重力，按 G*L*rho*A 后再分配到两端。
                const double gravity = load.m_g * (secondReference - firstReference).norm() *
                                       material->m_Density * section->m_Area;
                const Vec3 nodalForce = GlobalLineLoad(load.m_Direction, 0.5 * gravity * scale);
                std::vector<int> localDofCounts;
                cable->GetNodeLocalDOFCounts(localDofCounts);
                elementForce = Vec::Zero(localDofCounts[0] + localDofCounts[1]);
                elementForce.segment<3>(0) = nodalForce;
                elementForce.segment<3>(localDofCounts[0]) = nodalForce;
            }
            else
            {
                const Vec3 firstReference(first->m_X, first->m_Y, first->m_Z);
                const Vec3 secondReference(second->m_X, second->m_Y, second->m_Z);
                elementForce = SpatialUniformLineLoad(*cable, firstReference, secondReference, gravityPerLength);
            }
        }
        else
        {
            const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(element);
            if (beam && beam->IsReferenceTssbnConstitutiveMode())
            {
                // 参考梁重力按初始长度均分到节点，不产生一致线载荷的梁端力矩。
                const auto first = beam->m_pNode[0].lock();
                const auto second = beam->m_pNode[1].lock();
                if (!first || !second)
                    throw std::runtime_error("Gravity beam node reference is invalid");
                const Vec3 firstReference(first->m_X, first->m_Z, -first->m_Y);
                const Vec3 secondReference(second->m_X, second->m_Z, -second->m_Y);
                const double gravity = load.m_g * (secondReference - firstReference).norm() *
                                       material->m_Density * section->m_Area;
                const Vec3 nodalForce = GlobalLineLoad(load.m_Direction, 0.5 * gravity * scale);
                elementForce = Vec::Zero(12);
                elementForce.segment<3>(0) = nodalForce;
                elementForce.segment<3>(6) = nodalForce;
            }
            else
                elementForce = SpatialUniformLineLoad(*element, gravityPerLength);
        }
        if (cable && !cable->IsReferenceTssbnConstitutiveMode() && !load.m_DiagnosticIgnoreEccentricMoment &&
            section->m_MassFirstMomentPerLength.squaredNorm() > 0.0)
        {
            const Vec3 gravityAcceleration = GlobalLineLoad(load.m_Direction, load.m_g * scale);
            elementForce += EccentricGravityConsistentElementForce(*cable, section->m_MassFirstMomentPerLength,
                                                                   gravityAcceleration);
        }
        AddElementForce(*element, elementForce, fixedDofs, fixed, free);
    }
    const int component = static_cast<int>(load.m_Direction);
    for (const auto& [id, inertia] : structure.m_RigidBodyInertias)
    {
        Q_UNUSED(id);
        const auto node = inertia ? inertia->m_pNode.lock() : nullptr;
        if (node)
            AddNodeForce(*node, component, inertia->m_Mass * load.m_g * scale, fixedDofs, fixed, free);
    }
}

void AssembleWind(const Force_Wind& load, const StructureData& structure, int fixedDofs, double scale, Vec& fixed,
                  Vec& free)
{
    const Vec3 windVelocity = load.GetWindVelocityGlobal();
    if (windVelocity.norm() <= 1.0e-12)
        throw std::runtime_error("Wind direction vector must be nonzero");
    const Vec3 windDirection = windVelocity.normalized();
    const bool useAeroTags = std::any_of(structure.m_Elements.cbegin(), structure.m_Elements.cend(),
                                         [](const auto& entry)
                                         {
                                             return entry.second && entry.second->HasAerodynamicLoad();
                                         });

    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element || (useAeroTags && !element->HasAerodynamicLoad()))
            continue;
        const auto property = element->m_pProperty.lock();
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!section)
            throw std::runtime_error("Wind load requires a section");

        // Phase 1: wind pressure only, deliberately without Cd or aeroelastic terms.
        const double q =
            0.5 * load.m_windDensity * load.m_velocity * load.m_velocity * (2.0 * section->m_Radius) * scale;
        AddElementForce(*element, SpatialUniformLineLoad(*element, q * windDirection), fixedDofs, fixed, free);
    }
}

Vec3 CurrentNodePosition(const Node& node)
{
    return Vec3(node.m_X + node.m_Displacement[0], node.m_Y + node.m_Displacement[1],
                node.m_Z + node.m_Displacement[2]);
}

void ValidateBundledGallopingAxis(int elementId, const ElementBase& element, const Vec3& firstPosition,
                                  const Vec3& secondPosition, const Vec3& modelUp)
{
    if (element.m_AeroBundleCount <= 1)
        return;
    try
    {
        BundleAeroMapper::ValidateAxisConvention(secondPosition - firstPosition, modelUp);
    }
    catch (const std::exception& exception)
    {
        throw std::runtime_error("Galloping element " + std::to_string(elementId) + ": " + exception.what());
    }
}

AerodynamicSectionState BuildGallopingSectionState(const ElementBase& element, const ElementCable* cable,
                                                   const Force_Wind& wind, double initialAttackDegrees,
                                                   const Eigen::Vector3d& modelUp, const Node& first,
                                                   const Node& second, bool referenceTssbn = false)
{
    AerodynamicSectionState state;
    state.firstPosition = Vec3(first.m_X + first.m_Displacement[0], first.m_Y + first.m_Displacement[1],
                               first.m_Z + first.m_Displacement[2]);
    state.secondPosition = Vec3(second.m_X + second.m_Displacement[0], second.m_Y + second.m_Displacement[1],
                                second.m_Z + second.m_Displacement[2]);
    state.firstVelocity = Vec3(first.m_Velocity[0], first.m_Velocity[1], first.m_Velocity[2]);
    state.secondVelocity = Vec3(second.m_Velocity[0], second.m_Velocity[1], second.m_Velocity[2]);
    state.windVelocity = wind.GetWindVelocityGlobal();
    state.modelUp = modelUp;
    state.initialAttack = initialAttackDegrees * std::acos(-1.0) / 180.0;
    state.referenceTssbn = referenceTssbn;
    state.diagnosticReferenceFixedWindSpeed = wind.m_DiagnosticReferenceFixedWindSpeed;
    ValidateBundledGallopingAxis(element.m_Id, element, state.firstPosition, state.secondPosition, modelUp);
    const Vec3 localAxis = Utility::CR::CanonicalAxis(state.secondPosition - state.firstPosition);
    if (cable)
    {
        const auto originalFirst = element.m_pNode[0].lock();
        const auto originalSecond = element.m_pNode[1].lock();
        if (!originalFirst || !originalSecond)
            throw std::runtime_error("Galloping element node reference is invalid");
        const Vec3 originalFirstPosition(originalFirst->m_X + originalFirst->m_Displacement[0],
                                         originalFirst->m_Y + originalFirst->m_Displacement[1],
                                         originalFirst->m_Z + originalFirst->m_Displacement[2]);
        const Vec3 originalSecondPosition(originalSecond->m_X + originalSecond->m_Displacement[0],
                                          originalSecond->m_Y + originalSecond->m_Displacement[1],
                                          originalSecond->m_Z + originalSecond->m_Displacement[2]);
        const Vec3 originalAxis = Utility::CR::CanonicalAxis(originalSecondPosition - originalFirstPosition);
        if (referenceTssbn)
        {
            // 参考索在共享梁节点仍取绝对全局 RotX 和材料角速度 X。
            state.firstTwist = first.m_Displacement[3];
            state.secondTwist = second.m_Displacement[3];
            state.firstTwistRate = first.m_Velocity[3];
            state.secondTwistRate = second.m_Velocity[3];
            if (first.m_DOF.size() >= 6)
            {
                const Vec3 angularVelocity(first.m_Velocity[3], first.m_Velocity[4], first.m_Velocity[5]);
                state.firstTwistRate = (first.m_Rg.transpose() * angularVelocity).x();
            }
            if (second.m_DOF.size() >= 6)
            {
                const Vec3 angularVelocity(second.m_Velocity[3], second.m_Velocity[4], second.m_Velocity[5]);
                state.secondTwistRate = (second.m_Rg.transpose() * angularVelocity).x();
            }
            return state;
        }
        if (first.m_DOF.size() >= 6)
        {
            const Eigen::Matrix3d firstOriginalRotation = originalFirst->m_Rg * originalFirst->m_Rg_n.transpose();
            const Eigen::Matrix3d firstCurrentRotation = first.m_Rg * first.m_Rg_n.transpose();
            state.firstTwist = cable->GetNodalTwist(0) -
                               Utility::CR::ExtractAxialTwist(firstOriginalRotation, originalAxis) +
                               Utility::CR::ExtractAxialTwist(firstCurrentRotation, localAxis);
            state.firstTwistRate = Vec3(first.m_Velocity[3], first.m_Velocity[4], first.m_Velocity[5]).dot(localAxis);
        }
        else
        {
            state.firstTwist = first.m_Displacement[3];
            state.firstTwistRate = first.m_Velocity.size() > 3 ? first.m_Velocity[3] : 0.0;
        }
        if (second.m_DOF.size() >= 6)
        {
            const Eigen::Matrix3d secondOriginalRotation = originalSecond->m_Rg * originalSecond->m_Rg_n.transpose();
            const Eigen::Matrix3d secondCurrentRotation = second.m_Rg * second.m_Rg_n.transpose();
            state.secondTwist = cable->GetNodalTwist(1) -
                                Utility::CR::ExtractAxialTwist(secondOriginalRotation, originalAxis) +
                                Utility::CR::ExtractAxialTwist(secondCurrentRotation, localAxis);
            state.secondTwistRate =
                Vec3(second.m_Velocity[3], second.m_Velocity[4], second.m_Velocity[5]).dot(localAxis);
        }
        else
        {
            state.secondTwist = second.m_Displacement[3];
            state.secondTwistRate = second.m_Velocity.size() > 3 ? second.m_Velocity[3] : 0.0;
        }
    }
    else if (first.m_Displacement.size() >= 6 && second.m_Displacement.size() >= 6)
    {
        const auto* beam = dynamic_cast<const ElementBeam_CR*>(&element);
        if (beam && !referenceTssbn)
        {
            AerodynamicSectionState reference;
            reference.firstPosition = Vec3(first.m_X, first.m_Y, first.m_Z);
            reference.secondPosition = Vec3(second.m_X, second.m_Y, second.m_Z);
            reference.windVelocity = state.windVelocity;
            reference.modelUp = modelUp;
            reference.diagnosticReferenceFixedWindSpeed = state.diagnosticReferenceFixedWindSpeed;
            const AerodynamicSectionResult referenceFrame = AerodynamicLoadCalculator::ComputeKinematics(reference);
            if (!beam->R0.allFinite() ||
                (beam->R0.transpose() * beam->R0 - Eigen::Matrix3d::Identity()).norm() > 1.0e-8 ||
                beam->R0.col(0).cross(referenceFrame.axis).norm() > 1.0e-8)
                throw std::runtime_error("Beam aerodynamic initial material frame is not initialized");

            Vec3 initialDirection = referenceFrame.transverse -
                                    referenceFrame.transverse.dot(referenceFrame.axis) * referenceFrame.axis;
            if (!initialDirection.allFinite() || initialDirection.norm() <= 1.0e-12)
                throw std::runtime_error("Beam aerodynamic initial zero direction is degenerate");
            initialDirection.normalize();
            // 用初始风/up截面定义零点，initialAttack 仍只在气动计算器中计入一次。
            const Vec3 materialDirection = beam->R0.transpose() * initialDirection;
            const double axisSign = beam->R0.col(0).dot(referenceFrame.axis) >= 0.0 ? 1.0 : -1.0;
            const Eigen::Matrix3d firstSection = first.m_Rg * beam->R0;
            const Eigen::Matrix3d secondSection = second.m_Rg * beam->R0;
            // 必须使用传入节点副本，使中心差分的姿态扰动进入同一荷载路径。
            state.firstSectionAxis = axisSign * firstSection.col(0);
            state.secondSectionAxis = axisSign * secondSection.col(0);
            state.firstSectionDirection = firstSection * materialDirection;
            state.secondSectionDirection = secondSection * materialDirection;
            state.useBeamSectionDirections = true;
        }
        else
        {
            // 参考 TSSBN 及其他六自由度单元保留原有角度约定。
            const Vec3 firstRotation(first.m_Displacement[3], first.m_Displacement[4], first.m_Displacement[5]);
            const Vec3 secondRotation(second.m_Displacement[3], second.m_Displacement[4], second.m_Displacement[5]);
            state.firstTwist = firstRotation.dot(localAxis);
            state.secondTwist = secondRotation.dot(localAxis);
        }
        if (first.m_Velocity.size() >= 6 && second.m_Velocity.size() >= 6)
        {
            const Vec3 firstAngularVelocity(first.m_Velocity[3], first.m_Velocity[4], first.m_Velocity[5]);
            const Vec3 secondAngularVelocity(second.m_Velocity[3], second.m_Velocity[4], second.m_Velocity[5]);
            state.firstTwistRate = firstAngularVelocity.dot(localAxis);
            state.secondTwistRate = secondAngularVelocity.dot(localAxis);
        }
    }
    return state;
}

AeroCoefficients GetGallopingCoefficients(const Force_Wind& wind, const StructureData& structure,
                                           const std::vector<BladeModel>& caseData, int profileId,
                                           double attackRadians, bool referenceTssbn)
{
    if (!referenceTssbn && wind.m_DiagnosticReferenceExactFittedAero)
        return ReferenceSingleConductorFit(attackRadians);

    const double attackDegrees =
        AeroManager::normalizeAngleDegrees(attackRadians * 180.0 / std::acos(-1.0));
    AeroCoefficients coefficients = structure.m_AeroManager.getCoefficients(caseData, profileId, attackDegrees);
    if (!referenceTssbn && wind.m_DiagnosticPaperBoundedFittedAero)
        coefficients = ReferenceSingleConductorFit(attackRadians, nullptr, &coefficients);
    return coefficients;
}

double GallopingFittedWeight(const Force_Wind& wind, double normalizedAttackDegrees)
{
    if (wind.m_DiagnosticReferenceExactFittedAero)
        return 1.0;
    if (!wind.m_DiagnosticPaperBoundedFittedAero || normalizedAttackDegrees > 100.0)
        return 0.0;
    const double fraction = std::clamp(std::min(normalizedAttackDegrees, 100.0 - normalizedAttackDegrees) / 5.0,
                                       0.0, 1.0);
    return fraction * fraction * (3.0 - 2.0 * fraction);
}

Vec EvaluateGallopingElementForce(const ElementBase& element, const ElementCable* cable, const Force_Wind& wind,
                                  const StructureData& structure, const std::vector<BladeModel>& caseData,
                                  int profileId, double initialAttackDegrees, const Eigen::Vector3d& modelUp,
                                  double scale, const Node& first, const Node& second, bool referenceTssbn)
{
    const auto property = element.m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!section)
        throw std::runtime_error("Galloping element section is invalid");

    const Vec3 windVelocity = wind.GetWindVelocityGlobal();
    if (referenceTssbn && cable && (first.m_DOF.size() == 4 || first.m_DOF.size() == 6) &&
        (second.m_DOF.size() == 4 || second.m_DOF.size() == 6) &&
        modelUp.isApprox(Vec3::UnitZ(), 1.0e-12) &&
        windVelocity.normalized().isApprox(-Vec3::UnitY(), 1.0e-12))
    {
        // 原程序的八维索方程嵌入实际节点布局；共享梁节点多出的两个转角不承受索气动力矩。
        const int secondOffset = static_cast<int>(first.m_DOF.size());
        const int elementDofs = secondOffset + static_cast<int>(second.m_DOF.size());
        const Vec3 firstPosition(first.m_X + first.m_Displacement[0], first.m_Y + first.m_Displacement[1],
                                 first.m_Z + first.m_Displacement[2]);
        const Vec3 secondPosition(second.m_X + second.m_Displacement[0], second.m_Y + second.m_Displacement[1],
                                  second.m_Z + second.m_Displacement[2]);
        ValidateBundledGallopingAxis(element.m_Id, element, firstPosition, secondPosition, modelUp);
        const double windSpeed = windVelocity.norm();
        if (windSpeed <= 1.0e-12)
            return Vec::Zero(elementDofs);
        const double verticalVelocity = (first.m_Velocity[2] + second.m_Velocity[2]) * 0.5;
        double firstTwistRate = first.m_Velocity[3];
        double secondTwistRate = second.m_Velocity[3];
        if (first.m_DOF.size() == 6)
        {
            const Vec3 angularVelocity(first.m_Velocity[3], first.m_Velocity[4], first.m_Velocity[5]);
            firstTwistRate = (first.m_Rg.transpose() * angularVelocity).x();
        }
        if (second.m_DOF.size() == 6)
        {
            const Vec3 angularVelocity(second.m_Velocity[3], second.m_Velocity[4], second.m_Velocity[5]);
            secondTwistRate = (second.m_Rg.transpose() * angularVelocity).x();
        }
        const double twistRate = (firstTwistRate + secondTwistRate) * 0.5;
        const double meanTwist = (first.m_Displacement[3] + second.m_Displacement[3]) * 0.5;
        const double attack = initialAttackDegrees * std::acos(-1.0) / 180.0 + meanTwist -
                              (section->m_Radius * twistRate + verticalVelocity) / windSpeed;
        const AeroCoefficients coefficients =
            GetGallopingCoefficients(wind, structure, caseData, profileId, attack, true);
        const double diameter = 2.0 * section->m_Radius;
        const double lift = 0.5 * wind.m_windDensity * diameter * windSpeed * windSpeed * coefficients.lift;
        const double drag = 0.5 * wind.m_windDensity * diameter * windSpeed * windSpeed * coefficients.drag;
        const double moment = wind.m_EnableAerodynamicMoment
            ? 0.5 * wind.m_windDensity * diameter * diameter * windSpeed * windSpeed * coefficients.moment
            : 0.0;
        const Vec3 initial0(first.m_X, first.m_Z, -first.m_Y);
        const Vec3 initial1(second.m_X, second.m_Z, -second.m_Y);
        const Vec3 displacement0(first.m_Displacement[0], first.m_Displacement[2], -first.m_Displacement[1]);
        const Vec3 displacement1(second.m_Displacement[0], second.m_Displacement[2], -second.m_Displacement[1]);
        const Vec3 initialChord = initial1 - initial0;
        const double length = (initialChord + displacement1 - displacement0).norm();
        const double halfLift = (0.5 * length * lift) * scale;
        const double halfDrag = (0.5 * length * drag) * scale;
        const double halfMoment = (0.5 * length * moment) * scale;
        Vec referenceForce = Vec::Zero(elementDofs);
        referenceForce.segment<4>(0) << 0.0, -halfDrag, halfLift, halfMoment;
        referenceForce.segment<4>(secondOffset) << 0.0, -halfDrag, halfLift, halfMoment;
        return referenceForce;
    }

    AerodynamicSectionState state = BuildGallopingSectionState(
        element, cable, wind, initialAttackDegrees, modelUp, first, second, referenceTssbn);
    state.radius = section->m_Radius;

    AerodynamicSectionResult result = AerodynamicLoadCalculator::ComputeKinematics(state);
    if (result.relativeSpeed <= 1.0e-12)
    {
        std::vector<int> localDofCounts;
        element.GetNodeLocalDOFCounts(localDofCounts);
        return Vec::Zero(localDofCounts[0] + localDofCounts[1]);
    }
    const AeroCoefficients coefficients =
        GetGallopingCoefficients(wind, structure, caseData, profileId, result.attackAngle, referenceTssbn);
    const double momentCoefficient = wind.m_EnableAerodynamicMoment ? coefficients.moment : 0.0;
    AerodynamicLoadCalculator::ComputeLineLoad(result, wind.m_windDensity, 2.0 * section->m_Radius,
                                               coefficients.lift, coefficients.drag, momentCoefficient,
                                               referenceTssbn || wind.m_DiagnosticReferenceFixedForceDirections);

    Vec elementForce = SpatialUniformLineLoad(
        element, state.firstPosition, state.secondPosition, scale * result.lineForce);
    const double currentLength = (state.secondPosition - state.firstPosition).norm();
    if (cable)
    {
        const double nodalTorque = 0.5 * currentLength * scale * result.lineMoment.dot(result.axis);
        std::vector<int> localDofCounts;
        element.GetNodeLocalDOFCounts(localDofCounts);
        int offset = 0;
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIndex)];
            if (localDofs == 4 || referenceTssbn)
                elementForce[offset + 3] += nodalTorque;
            else
                elementForce.segment<3>(offset + 3) += nodalTorque * result.axis;
            offset += localDofs;
        }
    }
    else if (element.Get_NodeDOF() >= 6)
    {
        std::vector<int> localDofCounts;
        element.GetNodeLocalDOFCounts(localDofCounts);
        const int secondOffset = localDofCounts[0];
        const Vec3 nodalMoment = 0.5 * currentLength * scale * result.lineMoment;
        elementForce.segment<3>(3) += nodalMoment;
        elementForce.segment<3>(secondOffset + 3) += nodalMoment;
    }
    return elementForce;
}

Vec EvaluateGallopingElementForce(const ElementBase& element, const ElementCable* cable, const Force_Wind& wind,
                                  const StructureData& structure, const std::vector<BladeModel>& caseData,
                                  int profileId, double initialAttackDegrees, const Eigen::Vector3d& modelUp,
                                  double scale, bool referenceTssbn)
{
    const auto first = element.m_pNode[0].lock();
    const auto second = element.m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("Galloping element node reference is invalid");
    return EvaluateGallopingElementForce(element, cable, wind, structure, caseData, profileId, initialAttackDegrees,
                                         modelUp, scale, *first, *second, referenceTssbn);
}

void PerturbNodeCorrection(Node& node, int localDirection, double increment, double velocityDerivative)
{
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
    if (localDirection < 3)
        translation[localDirection] = increment;
    else if (node.m_DOF.size() == 4 && localDirection == 3)
        rotation[0] = increment;
    else if (node.m_DOF.size() >= 6 && localDirection < 6)
        rotation[localDirection - 3] = increment;
    node.ApplyNewmarkCorrection(translation, rotation, 0.0, velocityDerivative);
}
}

LoadAssembler::GallopingElementAeroSample LoadAssembler::SampleGallopingElementAero(
    const Force_Wind& wind, const StructureData& structure, const ElementBase& element, int profileId,
    int iceThickness, double initialAttackDegrees, const Eigen::Vector3d& modelUp, bool referenceTssbn)
{
    const auto property = element.m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    const auto first = element.m_pNode[0].lock();
    const auto second = element.m_pNode[1].lock();
    if (!section || !first || !second || !element.HasAerodynamicLoad())
        throw std::runtime_error("Galloping aerodynamic diagnostic element is incomplete");

    const auto* caseData =
        ResolveGallopingCaseData(structure, element.m_AeroBundleCount, wind.m_velocity, iceThickness);
    if (!caseData)
        throw std::runtime_error("Galloping aerodynamic diagnostic case is not loaded");

    const auto* cable = dynamic_cast<const ElementCable*>(&element);
    AerodynamicSectionState state =
        BuildGallopingSectionState(element, cable, wind, initialAttackDegrees, modelUp, *first, *second,
                                    referenceTssbn);
    state.radius = section->m_Radius;
    AerodynamicSectionResult result = AerodynamicLoadCalculator::ComputeKinematics(state);
    const double radiansToDegrees = 180.0 / std::acos(-1.0);
    const double normalizedAttackDegrees =
        AeroManager::normalizeAngleDegrees(result.attackAngle * radiansToDegrees);
    const AeroCoefficients coefficients =
        GetGallopingCoefficients(wind, structure, *caseData, profileId, result.attackAngle, referenceTssbn);
    const double momentCoefficient = wind.m_EnableAerodynamicMoment ? coefficients.moment : 0.0;
    AerodynamicLoadCalculator::ComputeLineLoad(result, wind.m_windDensity, 2.0 * section->m_Radius,
                                               coefficients.lift, coefficients.drag, momentCoefficient,
                                               referenceTssbn || wind.m_DiagnosticReferenceFixedForceDirections);

    GallopingElementAeroSample sample;
    sample.attackDegrees = result.attackAngle * radiansToDegrees;
    sample.normalizedAttackDegrees = normalizedAttackDegrees;
    sample.flowAngleDegrees = result.flowAngle * radiansToDegrees;
    sample.relativeSpeed = result.relativeSpeed;
    sample.liftCoefficient = coefficients.lift;
    sample.dragCoefficient = coefficients.drag;
    sample.momentCoefficient = momentCoefficient;
    sample.lineTorque = result.lineMoment.dot(result.axis);
    sample.currentLength = (state.secondPosition - state.firstPosition).norm();
    sample.meanTwistRate = 0.5 * (state.firstTwistRate + state.secondTwistRate);
    sample.torsionalPowerPerLength = sample.lineTorque * sample.meanTwistRate;
    sample.translationalPowerPerLength =
        result.lineForce.dot(0.5 * (state.firstVelocity + state.secondVelocity));
    const Vec elementForce = EvaluateGallopingElementForce(element, cable, wind, structure, *caseData, profileId,
                                                            initialAttackDegrees, modelUp, 1.0, *first, *second,
                                                            referenceTssbn);
    std::vector<int> localDofCounts;
    element.GetNodeLocalDOFCounts(localDofCounts);
    if (localDofCounts.size() != 2 || first->m_Velocity.size() < localDofCounts[0] ||
        second->m_Velocity.size() < localDofCounts[1])
        throw std::runtime_error("Galloping aerodynamic diagnostic velocity DOFs are incomplete");
    Vec elementVelocity(elementForce.size());
    for (int i = 0; i < localDofCounts[0]; ++i)
        elementVelocity[i] = first->m_Velocity[static_cast<std::size_t>(i)];
    for (int i = 0; i < localDofCounts[1]; ++i)
        elementVelocity[localDofCounts[0] + i] = second->m_Velocity[static_cast<std::size_t>(i)];
    if (referenceTssbn && cable)
    {
        elementVelocity[3] = state.firstTwistRate;
        elementVelocity[localDofCounts[0] + 3] = state.secondTwistRate;
    }
    sample.totalPowerPerLength = elementForce.dot(elementVelocity) / sample.currentLength;
    sample.rotationalPowerPerLength = sample.totalPowerPerLength - sample.translationalPowerPerLength;
    sample.fittedWeight = referenceTssbn ? 0.0 : GallopingFittedWeight(wind, normalizedAttackDegrees);
    return sample;
}

static std::vector<SolverNameSpace::ParallelWorkRange> BuildUniformWorkRanges(std::size_t itemCount,
                                                                                int requestedWorkers)
{
    const int workerCount = std::max(1, std::min(requestedWorkers, static_cast<int>(itemCount)));
    std::vector<SolverNameSpace::ParallelWorkRange> ranges;
    ranges.reserve(static_cast<std::size_t>(workerCount));
    for (int worker = 0; worker < workerCount; ++worker)
    {
        const std::size_t first = itemCount * static_cast<std::size_t>(worker) / static_cast<std::size_t>(workerCount);
        const std::size_t last =
            itemCount * static_cast<std::size_t>(worker + 1) / static_cast<std::size_t>(workerCount);
        ranges.push_back({first, last});
    }
    return ranges;
}

static bool TryCalculateCableAerodynamicTangent(const ElementCable& cable, const Force_Wind& wind,
                                                const StructureData& structure, const std::vector<BladeModel>& caseData,
                                                int profileId, double initialAttackDegrees,
                                                const Eigen::Vector3d& modelUp, double scale, double velocityDerivative,
                                                double correctionIncrement, const Node& first, const Node& second,
                                                int firstDofs, int secondDofs,
                                                _OUT Eigen::Matrix<double, 12, 12>& tangent, bool referenceTssbn)
{
    // 拼接区的拟合权重随攻角变化，由通用中心差分求完整切线。
    if (!referenceTssbn && wind.m_DiagnosticPaperBoundedFittedAero)
        return false;
    // 共享节点的参考攻角采用材料角速度，沿轴投影切线不再适用，交由中心差分处理。
    if (referenceTssbn && (firstDofs != 4 || secondDofs != 4))
        return false;
    // 参考索保持动压和全局升阻力方向，只对攻角及当前长度求导。
    if (firstDofs == 4 && secondDofs == 4 &&
        (referenceTssbn ||
         (wind.m_DiagnosticReferenceFixedWindSpeed && wind.m_DiagnosticReferenceFixedForceDirections)))
    {
        const auto property = cable.m_pProperty.lock();
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!section)
            throw std::runtime_error("Galloping element section is invalid");
        AerodynamicSectionState state =
            BuildGallopingSectionState(cable, &cable, wind, initialAttackDegrees, modelUp, first, second,
                                        referenceTssbn);
        state.radius = section->m_Radius;
        const AerodynamicSectionResult result = AerodynamicLoadCalculator::ComputeKinematics(state);
        if (result.relativeSpeed <= 1.0e-12)
            return false;

        const Vec3 chord = state.secondPosition - state.firstPosition;
        const double length = chord.norm();
        const Vec3 chordDirection = chord / length;
        const double diameter = 2.0 * section->m_Radius;
        const double pressureWidth = 0.5 * wind.m_windDensity * diameter * result.relativeSpeed * result.relativeSpeed;
        AeroCoefficients slopes;
        AeroCoefficients coefficients;
        if (!referenceTssbn && wind.m_DiagnosticReferenceExactFittedAero)
        {
            coefficients = ReferenceSingleConductorFit(result.attackAngle, &slopes);
        }
        else
        {
            const double degreesPerRadian = 180.0 / std::acos(-1.0);
            const double attackDegrees = AeroManager::normalizeAngleDegrees(result.attackAngle * degreesPerRadian);
            double largestAngleDerivative = 0.0;
            for (int column = 0; column < 8; ++column)
            {
                const int component = column % 4;
                const double flowAngleDerivative = 0.5 * velocityDerivative / result.relativeSpeed *
                                                   (component == 3 ? section->m_Radius : result.transverse[component]);
                const double attackDerivative = (component == 3 ? 0.5 : 0.0) - flowAngleDerivative;
                largestAngleDerivative = std::max(largestAngleDerivative, std::abs(attackDerivative));
            }
            const double knotDistance = std::abs(std::remainder(attackDegrees, 5.0));
            if (knotDistance <= 2.0 * correctionIncrement * largestAngleDerivative * degreesPerRadian + 1.0e-9)
                return false;
            coefficients = structure.m_AeroManager.getCoefficients(caseData, profileId, attackDegrees);
            slopes = structure.m_AeroManager.getCoefficientSlopes(caseData, profileId, attackDegrees);
        }
        const double moment = wind.m_EnableAerodynamicMoment ? coefficients.moment : 0.0;
        const double momentSlope = wind.m_EnableAerodynamicMoment ? slopes.moment : 0.0;
        const Vec3 lineForce =
            pressureWidth * (coefficients.drag * result.windNormal + coefficients.lift * result.transverse);
        const double lineTorque = pressureWidth * diameter * moment;
        tangent.setZero();
        for (int column = 0; column < 8; ++column)
        {
            const int component = column % 4;
            const double lengthDerivative =
                component == 3 ? 0.0 : (column < 4 ? -chordDirection[component] : chordDirection[component]);
            const double flowAngleDerivative = 0.5 * velocityDerivative / result.relativeSpeed *
                                               (component == 3 ? section->m_Radius : result.transverse[component]);
            const double attackDerivative = (component == 3 ? 0.5 : 0.0) - flowAngleDerivative;
            const Vec3 lineForceDerivative =
                pressureWidth * attackDerivative * (slopes.drag * result.windNormal + slopes.lift * result.transverse);
            const double lineTorqueDerivative = pressureWidth * diameter * momentSlope * attackDerivative;
            const Vec3 nodalForceDerivative =
                0.5 * scale * (lengthDerivative * lineForce + length * lineForceDerivative);
            const double nodalTorqueDerivative =
                0.5 * scale * (lengthDerivative * lineTorque + length * lineTorqueDerivative);
            tangent.block<3, 1>(0, column) = nodalForceDerivative;
            tangent.block<3, 1>(4, column) = nodalForceDerivative;
            tangent(3, column) = nodalTorqueDerivative;
            tangent(7, column) = nodalTorqueDerivative;
        }
        return tangent.allFinite();
    }

    // 空间索解析切线同时支持完整相对风及参考固定方向流程；旧诊断组合沿用中心差分。
    if (!referenceTssbn && (wind.m_DiagnosticReferenceFixedWindSpeed || wind.m_DiagnosticReferenceExactFittedAero))
        return false;
    const auto property = cable.m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!section)
        throw std::runtime_error("Galloping element section is invalid");

    AerodynamicSectionState state =
        BuildGallopingSectionState(cable, &cable, wind, initialAttackDegrees, modelUp, first, second, referenceTssbn);
    state.radius = section->m_Radius;
    const AerodynamicSectionResult result = AerodynamicLoadCalculator::ComputeKinematics(state);
    const Vec3 chord = state.secondPosition - state.firstPosition;
    const double length = chord.norm();
    const Vec3 chordDirection = chord / length;
    const double orientation = result.axis.dot(chordDirection);
    const Vec3 absAxis = result.axis.cwiseAbs();
    Eigen::Index dominant = 0;
    absAxis.maxCoeff(&dominant);
    double secondLargest = 0.0;
    for (int direction = 0; direction < 3; ++direction)
        if (direction != dominant)
            secondLargest = std::max(secondLargest, absAxis[direction]);

    // 规范轴、局部横向方向和零相对风附近存在分段切换，继续使用中心差分。
    const Vec3 projectedWind = state.windVelocity - state.windVelocity.dot(result.axis) * result.axis;
    const double normalWindSpeed = projectedWind.norm();
    const Vec3 upNormal = state.modelUp - state.modelUp.dot(result.axis) * result.axis;
    if (absAxis[dominant] - secondLargest <= 4.0 * correctionIncrement / length ||
        (!referenceTssbn &&
         (normalWindSpeed <= std::max(1.0e-10, 4.0 * correctionIncrement * state.windVelocity.norm() / length) ||
          std::abs(result.transverse.dot(upNormal)) <= 4.0 * correctionIncrement * state.modelUp.norm() / length)) ||
        result.relativeSpeed <= 1.0e-10)
        return false;

    const double transverseSign = result.transverse.dot(result.axis.cross(result.windNormal)) >= 0.0 ? 1.0 : -1.0;
    const Vec3 midpointVelocity = 0.5 * (state.firstVelocity + state.secondVelocity);
    const double relativeTransverse = -midpointVelocity.dot(result.transverse) -
                                      0.5 * section->m_Radius * (state.firstTwistRate + state.secondTwistRate);
    const double relativeNormal = normalWindSpeed - midpointVelocity.dot(result.windNormal);
    const double speed = result.relativeSpeed;
    const double axisWind = result.axis.dot(state.windVelocity);
    const std::array<const Node*, 2> nodes{&first, &second};
    const std::array<int, 2> nodeDofs{firstDofs, secondDofs};
    std::array<Eigen::Quaterniond, 2> relativeRotations;
    std::array<Vec3, 2> angularVelocities{Vec3::Zero(), Vec3::Zero()};
    std::array<Eigen::Matrix3d, 2> rotationVelocityMaps;
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        if (nodeDofs[nodeIndex] != 6)
            continue;
        const Node& node = *nodes[nodeIndex];
        const Eigen::Matrix3d stepRotation = node.m_Rg * node.m_Rg_n.transpose();
        relativeRotations[nodeIndex] = Eigen::Quaterniond(stepRotation).normalized();
        if (relativeRotations[nodeIndex].w() < 0.0)
            relativeRotations[nodeIndex].coeffs() *= -1.0;
        const double axialQuaternion = relativeRotations[nodeIndex].vec().dot(result.axis);
        if (relativeRotations[nodeIndex].w() <= 1.0e-5 ||
            relativeRotations[nodeIndex].w() * relativeRotations[nodeIndex].w() + axialQuaternion * axialQuaternion <=
                1.0e-10)
            return false;
        angularVelocities[nodeIndex] = Vec3(node.m_Velocity[3], node.m_Velocity[4], node.m_Velocity[5]);
        Vec3 stepRotationVector;
        Utility::CR::Extract_RotationVector(stepRotation, stepRotationVector);
        Eigen::Matrix3d spatialSpinInverse;
        Utility::CR::Calculate_Ts_Inv(stepRotationVector, spatialSpinInverse);
        // 左乘空间转角修正的角速度导数与 Newmark、TSSBN 阶段更新一致。
        rotationVelocityMaps[nodeIndex] = stepRotation * spatialSpinInverse;
    }

    std::array<Vec3, 12> transverseDerivatives;
    std::array<Vec3, 12> normalDerivatives;
    std::array<Vec3, 12> axisDerivatives;
    std::array<double, 12> lengthDerivatives{};
    std::array<double, 12> transverseSpeedDerivatives{};
    std::array<double, 12> normalSpeedDerivatives{};
    std::array<double, 12> speedDerivatives{};
    std::array<double, 12> angleDerivatives{};
    const int elementDofs = firstDofs + secondDofs;
    double largestAngleDerivative = 0.0;
    for (int column = 0; column < elementDofs; ++column)
    {
        const int nodeIndex = column < firstDofs ? 0 : 1;
        const int component = column - (nodeIndex == 0 ? 0 : firstDofs);
        Vec3 chordDerivative = Vec3::Zero();
        Vec3 midpointVelocityDerivative = Vec3::Zero();
        if (component < 3)
        {
            chordDerivative[component] = nodeIndex == 0 ? -1.0 : 1.0;
            midpointVelocityDerivative[component] = 0.5 * velocityDerivative;
        }

        const Vec3 axisDerivative =
            orientation / length * (chordDerivative - result.axis * result.axis.dot(chordDerivative));
        double meanTwistDerivative = 0.0;
        double twistRateDerivative = 0.0;
        for (int end = 0; end < 2; ++end)
        {
            if (nodeDofs[end] == 4)
            {
                if (nodeIndex == end && component == 3)
                {
                    meanTwistDerivative += 0.5;
                    twistRateDerivative += 0.5 * velocityDerivative;
                }
                continue;
            }

            Vec3 spatialCorrection = Vec3::Zero();
            if (nodeIndex == end && component >= 3)
                spatialCorrection[component - 3] = 1.0;
            const Eigen::Quaterniond& rotation = relativeRotations[end];
            const double scalar = rotation.w();
            const Vec3 vector = rotation.vec();
            const double axial = vector.dot(result.axis);
            const double scalarDerivative = -0.5 * spatialCorrection.dot(vector);
            const Vec3 vectorDerivative = 0.5 * (scalar * spatialCorrection + spatialCorrection.cross(vector));
            const double axialDerivative = vectorDerivative.dot(result.axis) + vector.dot(axisDerivative);
            // 轴向扭角取四元数的摆动-扭转分量，弦轴旋转也改变该分量。
            meanTwistDerivative +=
                (scalar * axialDerivative - axial * scalarDerivative) / (scalar * scalar + axial * axial);

            const Vec3 angularVelocityDerivative = spatialCorrection.cross(angularVelocities[end]) +
                                                   velocityDerivative * rotationVelocityMaps[end] * spatialCorrection;
            twistRateDerivative +=
                0.5 * (angularVelocityDerivative.dot(result.axis) + angularVelocities[end].dot(axisDerivative));
        }
        const Vec3 projectedWindDerivative =
            -axisWind * axisDerivative - result.axis * state.windVelocity.dot(axisDerivative);
        const double normalWindDerivative = result.windNormal.dot(projectedWindDerivative);
        Vec3 windNormalDerivative =
            (projectedWindDerivative - result.windNormal * normalWindDerivative) /
            (referenceTssbn ? 1.0 : normalWindSpeed);
        Vec3 transverseDerivative =
            transverseSign * (axisDerivative.cross(result.windNormal) + result.axis.cross(windNormalDerivative));
        const double transverseSpeedDerivative = -midpointVelocityDerivative.dot(result.transverse) -
                                                 midpointVelocity.dot(transverseDerivative) -
                                                 section->m_Radius * twistRateDerivative;
        const double relativeNormalDerivative = normalWindDerivative -
                                                midpointVelocityDerivative.dot(result.windNormal) -
                                                midpointVelocity.dot(windNormalDerivative);
        double speedDerivative =
            (relativeTransverse * transverseSpeedDerivative + relativeNormal * relativeNormalDerivative) / speed;
        double flowAngleDerivative =
            (-relativeNormal * transverseSpeedDerivative + relativeTransverse * relativeNormalDerivative) /
            (speed * speed);
        if (referenceTssbn)
        {
            // 名义风速与固定全局升阻力方向不随单元姿态及平动速度变化。
            windNormalDerivative.setZero();
            transverseDerivative.setZero();
            speedDerivative = 0.0;
            flowAngleDerivative =
                (midpointVelocityDerivative.dot(result.transverse) + section->m_Radius * twistRateDerivative) / speed;
        }
        transverseDerivatives[column] = transverseDerivative;
        normalDerivatives[column] = windNormalDerivative;
        axisDerivatives[column] = axisDerivative;
        lengthDerivatives[column] = chordDirection.dot(chordDerivative);
        transverseSpeedDerivatives[column] = transverseSpeedDerivative;
        normalSpeedDerivatives[column] = relativeNormalDerivative;
        speedDerivatives[column] = speedDerivative;
        angleDerivatives[column] = meanTwistDerivative - flowAngleDerivative;
        largestAngleDerivative = std::max(largestAngleDerivative, std::abs(angleDerivatives[column]));
    }

    const double degreesPerRadian = 180.0 / std::acos(-1.0);
    const double attackDegrees = AeroManager::normalizeAngleDegrees(result.attackAngle * degreesPerRadian);
    const double knotDistance = std::abs(std::remainder(attackDegrees, 5.0));
    if (knotDistance <= 2.0 * correctionIncrement * largestAngleDerivative * degreesPerRadian + 1.0e-9)
        return false;

    const AeroCoefficients coefficients =
        structure.m_AeroManager.getCoefficients(caseData, profileId, attackDegrees);
    const AeroCoefficients slopes =
        structure.m_AeroManager.getCoefficientSlopes(caseData, profileId, attackDegrees);
    const double lift = coefficients.lift;
    const double drag = coefficients.drag;
    const double moment = wind.m_EnableAerodynamicMoment ? coefficients.moment : 0.0;
    const double liftSlope = slopes.lift;
    const double dragSlope = slopes.drag;
    const double momentSlope = wind.m_EnableAerodynamicMoment ? slopes.moment : 0.0;
    const double diameter = 2.0 * section->m_Radius;
    const double pressureWidth = 0.5 * wind.m_windDensity * diameter;
    const double forceTransverse = drag * relativeTransverse + lift * relativeNormal;
    const double forceNormal = drag * relativeNormal - lift * relativeTransverse;
    const Vec3 lineForce =
        (referenceTssbn || wind.m_DiagnosticReferenceFixedForceDirections)
            ? pressureWidth * speed * speed * (drag * result.windNormal + lift * result.transverse)
            : pressureWidth * speed * (forceTransverse * result.transverse + forceNormal * result.windNormal);
    const double lineTorque = pressureWidth * diameter * speed * speed * moment;

    tangent.setZero();
    for (int column = 0; column < elementDofs; ++column)
    {
        const double coefficientAngleDerivative = angleDerivatives[column];
        const double dragDerivative = dragSlope * coefficientAngleDerivative;
        const double liftDerivative = liftSlope * coefficientAngleDerivative;
        const double momentDerivative = momentSlope * coefficientAngleDerivative;
        Vec3 lineForceDerivative;
        if (referenceTssbn || wind.m_DiagnosticReferenceFixedForceDirections)
        {
            lineForceDerivative =
                pressureWidth *
                (2.0 * speed * speedDerivatives[column] * (drag * result.windNormal + lift * result.transverse) +
                 speed * speed *
                     (dragDerivative * result.windNormal + drag * normalDerivatives[column] +
                      liftDerivative * result.transverse + lift * transverseDerivatives[column]));
        }
        else
        {
            const double forceTransverseDerivative =
                dragDerivative * relativeTransverse + drag * transverseSpeedDerivatives[column] +
                liftDerivative * relativeNormal + lift * normalSpeedDerivatives[column];
            const double forceNormalDerivative =
                dragDerivative * relativeNormal + drag * normalSpeedDerivatives[column] -
                liftDerivative * relativeTransverse - lift * transverseSpeedDerivatives[column];
            lineForceDerivative =
                pressureWidth *
                (speedDerivatives[column] * (forceTransverse * result.transverse + forceNormal * result.windNormal) +
                 speed *
                     (forceTransverseDerivative * result.transverse + forceTransverse * transverseDerivatives[column] +
                      forceNormalDerivative * result.windNormal + forceNormal * normalDerivatives[column]));
        }

        const Vec3 nodalForceDerivative =
            0.5 * scale * (lengthDerivatives[column] * lineForce + length * lineForceDerivative);
        const double lineTorqueDerivative =
            pressureWidth * diameter *
            (2.0 * speed * speedDerivatives[column] * moment + speed * speed * momentDerivative);
        const double nodalTorqueDerivative =
            0.5 * scale * (lengthDerivatives[column] * lineTorque + length * lineTorqueDerivative);
        tangent.block<3, 1>(0, column) = nodalForceDerivative;
        tangent.block<3, 1>(firstDofs, column) = nodalForceDerivative;
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            const int row = (nodeIndex == 0 ? 0 : firstDofs) + 3;
            if (nodeDofs[nodeIndex] == 4)
                tangent(row, column) = nodalTorqueDerivative;
            else
                tangent.block<3, 1>(row, column) =
                    nodalTorqueDerivative * result.axis + 0.5 * scale * length * lineTorque * axisDerivatives[column];
        }
    }
    return tangent.allFinite();
}

static void AssembleGallopingTangentRange(std::size_t first, std::size_t last,
                                          const std::vector<std::pair<int, std::shared_ptr<ElementBase>>>& workItems,
                                          const std::map<int, const std::vector<BladeModel>*>& boundCases,
                                          const std::map<int, int>& elementProfileBindings, const Force_Wind& wind,
                                          const StructureData& structure, double initialAttackDegrees,
                                          const Eigen::Vector3d& modelUp, int fixedDofCount, int freeDofCount,
                                          double correctionIncrement, double velocityDerivative, double scale,
                                          _OUT std::vector<Eigen::Triplet<double>>& entries, bool referenceTssbn)
{
    for (std::size_t index = first; index < last; ++index)
    {
        const auto& [id, element] = workItems[index];
        const auto caseIt = boundCases.find(element->m_AeroBundleCount);
        const auto binding = elementProfileBindings.find(id);
        if (caseIt == boundCases.end() || !caseIt->second || binding == elementProfileBindings.end())
            throw std::runtime_error("Galloping tangent work item is incomplete");

        const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
        std::vector<int> dofs;
        std::vector<int> localDofCounts;
        element->GetDOFs(dofs);
        element->GetNodeLocalDOFCounts(localDofCounts);
        const int elementDofCount = localDofCounts.size() == 2 ? localDofCounts[0] + localDofCounts[1] : 0;
        if (dofs.size() != static_cast<std::size_t>(elementDofCount) || localDofCounts.size() != 2)
            throw std::runtime_error("Galloping tangent element layout is inconsistent");
        const auto firstNode = element->m_pNode[0].lock();
        const auto secondNode = element->m_pNode[1].lock();
        if (!firstNode || !secondNode)
            throw std::runtime_error("Galloping tangent node reference is invalid");

        Eigen::Matrix<double, 12, 12> cableTangent;
        const bool analytic = cable && (localDofCounts[0] == 4 || localDofCounts[0] == 6) &&
                              (localDofCounts[1] == 4 || localDofCounts[1] == 6) &&
                              TryCalculateCableAerodynamicTangent(
                                  *cable, wind, structure, *caseIt->second, binding->second, initialAttackDegrees,
                                  modelUp, scale, velocityDerivative, correctionIncrement, *firstNode, *secondNode,
                                  localDofCounts[0], localDofCounts[1], cableTangent, referenceTssbn);
        if (analytic)
        {
            for (int localColumn = 0; localColumn < elementDofCount; ++localColumn)
            {
                const int freeColumn = dofs[static_cast<std::size_t>(localColumn)] - fixedDofCount;
                if (freeColumn < 0 || freeColumn >= freeDofCount)
                    continue;
                for (int localRow = 0; localRow < elementDofCount; ++localRow)
                {
                    const int freeRow = dofs[static_cast<std::size_t>(localRow)] - fixedDofCount;
                    const double derivative = cableTangent(localRow, localColumn);
                    if (freeRow >= 0 && freeRow < freeDofCount && std::abs(derivative) > 1.0e-14)
                        entries.emplace_back(freeRow, freeColumn, derivative);
                }
            }
            continue;
        }

        int localColumn = 0;
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            for (int direction = 0; direction < localDofCounts[static_cast<std::size_t>(nodeIndex)]; ++direction)
            {
                const int freeColumn = dofs[static_cast<std::size_t>(localColumn)] - fixedDofCount;
                if (freeColumn < 0 || freeColumn >= freeDofCount)
                {
                    ++localColumn;
                    continue;
                }

                Node firstPlus = *firstNode;
                Node secondPlus = *secondNode;
                PerturbNodeCorrection(nodeIndex == 0 ? firstPlus : secondPlus, direction, correctionIncrement,
                                      velocityDerivative);
                const Eigen::VectorXd forcePlus = EvaluateGallopingElementForce(
                    *element, cable.get(), wind, structure, *caseIt->second, binding->second, initialAttackDegrees,
                    modelUp, scale, firstPlus, secondPlus, referenceTssbn);
                Node firstMinus = *firstNode;
                Node secondMinus = *secondNode;
                PerturbNodeCorrection(nodeIndex == 0 ? firstMinus : secondMinus, direction, -correctionIncrement,
                                      velocityDerivative);
                const Eigen::VectorXd forceMinus = EvaluateGallopingElementForce(
                    *element, cable.get(), wind, structure, *caseIt->second, binding->second, initialAttackDegrees,
                    modelUp, scale, firstMinus, secondMinus, referenceTssbn);
                const Eigen::VectorXd derivative = (forcePlus - forceMinus) / (2.0 * correctionIncrement);
                if (!derivative.allFinite())
                    throw std::runtime_error("Galloping tangent contains non-finite values");
                for (int localRow = 0; localRow < derivative.size(); ++localRow)
                {
                    const int freeRow = dofs[static_cast<std::size_t>(localRow)] - fixedDofCount;
                    if (freeRow >= 0 && freeRow < freeDofCount && std::abs(derivative[localRow]) > 1.0e-14)
                        entries.emplace_back(freeRow, freeColumn, derivative[localRow]);
                }
                ++localColumn;
            }
        }
    }
}

void LoadAssembler::Assemble(const LoadBase& load, const StructureData& structure, int fixedDofs, double scale,
                             _OUT Vec& fixed, _OUT Vec& free)
{
    switch (load.m_LoadType)
    {
        case EnumKeyword::LoadType::FORCE_NODE:
        {
            const auto& item = static_cast<const Force_Node&>(load);
            const auto node = item.m_pNode.lock();
            const int component = static_cast<int>(item.m_Direction);
            if (node)
                AddNodeForce(*node, component, item.m_Value * scale, fixedDofs, fixed, free);
            break;
        }
        case EnumKeyword::LoadType::FORCE_ELEMENT:
        {
            const auto& item = static_cast<const Force_Element&>(load);
            const auto element = item.m_pElement.lock();
            const int component = static_cast<int>(item.m_Direction);
            if (element && component >= 0 && component < element->Get_NodeDOF())
            {
                std::vector<int> localDofCounts;
                element->GetNodeLocalDOFCounts(localDofCounts);
                if (element->m_pNode.size() != 2 || localDofCounts.size() != 2 ||
                    component >= localDofCounts[0] || component >= localDofCounts[1])
                    throw std::runtime_error("Element load requires two nodes with the requested DOF");
                for (std::size_t nodeIndex = 0; nodeIndex < localDofCounts.size(); ++nodeIndex)
                {
                    const auto node = element->m_pNode[nodeIndex].lock();
                    if (!node || node->m_DOF.size() < localDofCounts[nodeIndex])
                        throw std::runtime_error("Element load node DOFs are not initialized");
                }
                const int secondOffset = localDofCounts[0];
                Vec force = Vec::Zero(localDofCounts[0] + localDofCounts[1]);
                const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
                if (cable && component == 3)
                {
                    cable->AddNodalAxialTorque(0, 0.5 * item.m_Value * scale, force);
                    cable->AddNodalAxialTorque(1, 0.5 * item.m_Value * scale, force);
                }
                else
                {
                    force[component] = force[secondOffset + component] = 0.5 * item.m_Value * scale;
                }
                AddElementForce(*element, force, fixedDofs, fixed, free);
            }
            break;
        }
        case EnumKeyword::LoadType::FORCE_GRAVITY:
            AssembleGravity(static_cast<const Force_Gravity&>(load), structure, fixedDofs, scale, fixed, free);
            break;
        case EnumKeyword::LoadType::FORCE_WIND:
            AssembleWind(static_cast<const Force_Wind&>(load), structure, fixedDofs, scale, fixed, free);
            break;
        default:
            break;
    }
}

bool LoadAssembler::EvaluateGravityPotential(const StructureData& structure, const Eigen::Vector3d& acceleration,
                                              const Eigen::Vector3d& eccentricAcceleration, _OUT double& energy)
{
    energy = 0.0;
    if (!acceleration.allFinite() || !eccentricAcceleration.allFinite())
        return false;
    if (acceleration.isZero(0.0) && eccentricAcceleration.isZero(0.0))
        return true;

    for (const auto& [id, element] : structure.m_Elements)
    {
        Q_UNUSED(id);
        if (!element)
            return false;
        if (dynamic_cast<const ElementSpringBase*>(element.get()))
            continue;
        if (element->m_pNode.size() != 2)
            return false;
        const auto first = element->m_pNode[0].lock();
        const auto second = element->m_pNode[1].lock();
        const auto property = element->m_pProperty.lock();
        const auto material = property ? property->m_pMaterial.lock() : nullptr;
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!first || !second || !material || !section || first->m_Displacement.size() < 3 ||
            second->m_Displacement.size() < 3)
            return false;

        const Eigen::Vector3d firstDisplacement(first->m_Displacement[0], first->m_Displacement[1],
                                                first->m_Displacement[2]);
        const Eigen::Vector3d secondDisplacement(second->m_Displacement[0], second->m_Displacement[1],
                                                 second->m_Displacement[2]);
        const Eigen::Vector3d referenceChord(second->m_X - first->m_X, second->m_Y - first->m_Y,
                                             second->m_Z - first->m_Z);
        const double referenceLength = element->L0 > 0.0 ? element->L0 : referenceChord.norm();
        const double mass = material->m_Density * section->m_Area * referenceLength;
        if (!std::isfinite(mass) || mass < 0.0 || !std::isfinite(referenceLength) || referenceLength <= 0.0)
            return false;
        // 与物理质量的线性平动插值一致，不构造节点力向量或整体质量矩阵。
        energy -= 0.5 * mass * acceleration.dot(firstDisplacement + secondDisplacement);

        const auto* cable = dynamic_cast<const ElementCable*>(element.get());
        if (!cable || cable->IsReferenceTssbnConstitutiveMode() || eccentricAcceleration.isZero(0.0) ||
            section->m_MassFirstMomentPerLength.isZero(0.0))
            continue;
        const Eigen::Vector3d currentChord = referenceChord + secondDisplacement - firstDisplacement;
        const double initialLength = referenceChord.norm();
        const double currentLength = currentChord.norm();
        if (initialLength <= 1.0e-12 || currentLength <= 1.0e-12)
            return false;
        const Eigen::Vector3d referenceAxis = Utility::CR::CanonicalAxis(referenceChord);
        const Eigen::Vector3d initialMoment = section->m_MassFirstMomentPerLength -
            referenceAxis.dot(section->m_MassFirstMomentPerLength) * referenceAxis;
        // 现有偏心重力取 -l*g·s(q)，保留当前长度及扭角，减去初始几何常数。
        energy -= eccentricAcceleration.dot(currentLength * cable->GetMassFirstMomentGlobal() -
                                              initialLength * initialMoment);
    }
    for (const auto& [id, inertia] : structure.m_RigidBodyInertias)
    {
        Q_UNUSED(id);
        const auto node = inertia ? inertia->m_pNode.lock() : nullptr;
        if (!node || node->m_Displacement.size() < 3 || !std::isfinite(inertia->m_Mass) || inertia->m_Mass < 0.0)
            return false;
        const Eigen::Vector3d displacement(node->m_Displacement[0], node->m_Displacement[1], node->m_Displacement[2]);
        energy -= inertia->m_Mass * acceleration.dot(displacement);
    }
    return std::isfinite(energy);
}

void LoadAssembler::AssembleEccentricGravityTangent(const Force_Gravity& gravity, const StructureData& structure,
                                                     int fixedDofCount, int freeDofCount, double scale,
                                                     _OUT Eigen::SparseMatrix<double>& externalTangent)
{
    externalTangent.resize(freeDofCount, freeDofCount);
    externalTangent.setZero();
    if (scale == 0.0 || gravity.m_DiagnosticIgnoreEccentricMoment)
        return;

    const Vec3 gravityAcceleration = GlobalLineLoad(gravity.m_Direction, gravity.m_g * scale);
    std::vector<Eigen::Triplet<double>> entries;
    for (const auto& [elementId, element] : structure.m_Elements)
    {
        Q_UNUSED(elementId);
        const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
        if (!cable || cable->IsReferenceTssbnConstitutiveMode())
            continue;
        const auto property = cable->m_pProperty.lock();
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!section || section->m_MassFirstMomentPerLength.squaredNorm() == 0.0)
            continue;

        std::vector<int> dofs;
        std::vector<int> localDofCounts;
        cable->GetDOFs(dofs);
        cable->GetNodeLocalDOFCounts(localDofCounts);
        const EccentricGravityState state = CurrentEccentricGravityState(*cable);
        if (localDofCounts[0] == 6 || localDofCounts[1] == 6)
        {
            // W(p,θ(p,R)) 需包含扭角随弦线变化及 SO(3) 转动连接项。
            // 4/4 路径仍走下方旧公式；本分支与索内扭转共用梯度和 Hessian。
            const int elementDofs = localDofCounts[0] + localDofCounts[1];
            Vec twistGradient;
            Eigen::MatrixXd twistHessian;
            cable->GetMeanTwistDerivatives(twistGradient, &twistHessian);
            const Vec3 chord = state.positions[1] - state.positions[0];
            const double length = chord.norm();
            if (length <= 1.0e-12)
                throw std::runtime_error("Eccentric ice gravity requires positive cable length");
            const Vec3 axis = Utility::CR::CanonicalAxis(chord);
            const double orientation = chord.dot(axis) / length;
            const Eigen::Matrix3d transverseProjector = Eigen::Matrix3d::Identity() - axis * axis.transpose();
            const Eigen::Matrix3d axisChordPartial = orientation / length * transverseProjector;
            const Vec3& firstMoment = section->m_MassFirstMomentPerLength;
            const Vec3 momentCrossGravity = firstMoment.cross(gravityAcceleration);
            const double axialMoment = axis.dot(firstMoment);
            const double axialGravity = axis.dot(gravityAcceleration);
            const double momentGravity = firstMoment.dot(gravityAcceleration);
            const double meanTwist = 0.5 * (state.twists[0] + state.twists[1]);
            const double cosine = std::cos(meanTwist);
            const double sine = std::sin(meanTwist);
            const Vec3 axialProductGradient = axialGravity * firstMoment + axialMoment * gravityAcceleration;
            const Vec3 untwistedForce = (momentGravity + axialMoment * axialGravity) * axis -
                                         axialGravity * firstMoment - axialMoment * gravityAcceleration;
            const Eigen::Matrix3d forceAxisPartial = cosine *
                ((momentGravity + axialMoment * axialGravity) * Eigen::Matrix3d::Identity() +
                 axis * axialProductGradient.transpose() - firstMoment * gravityAcceleration.transpose() -
                 gravityAcceleration * firstMoment.transpose());
            // 规范轴的方向符号在两次位置求导中相消，节点两端保留相反的弦线导数。
            const Eigen::Matrix3d forceChordPartial = orientation * forceAxisPartial * axisChordPartial;
            const Vec3 twistForce = orientation * (cosine * momentCrossGravity - sine * untwistedForce);
            const double axialCrossGravity = axis.dot(momentCrossGravity);
            const double torqueShape = cosine * axialCrossGravity +
                                       sine * (axialMoment * axialGravity - momentGravity);
            const double torque = length * torqueShape;
            const Vec3 torqueChordPartial = chord / length * torqueShape +
                length * axisChordPartial * (cosine * momentCrossGravity + sine * axialProductGradient);
            const double torqueTwist = length * (-sine * axialCrossGravity +
                cosine * (axialMoment * axialGravity - momentGravity));
            Eigen::MatrixXd positionPartial = Eigen::MatrixXd::Zero(elementDofs, elementDofs);
            Vec torquePositionPartial = Vec::Zero(elementDofs);
            const int secondOffset = localDofCounts[0];
            positionPartial.block<3, 3>(0, 0) = forceChordPartial;
            positionPartial.block<3, 3>(0, secondOffset) = -forceChordPartial;
            positionPartial.block<3, 3>(secondOffset, 0) = -forceChordPartial;
            positionPartial.block<3, 3>(secondOffset, secondOffset) = forceChordPartial;
            torquePositionPartial.segment<3>(0) = -torqueChordPartial;
            torquePositionPartial.segment<3>(secondOffset) = torqueChordPartial;
            Vec partialTwist = Vec::Zero(elementDofs);
            partialTwist.segment<3>(0) = -twistForce;
            partialTwist.segment<3>(secondOffset) = twistForce;
            Eigen::MatrixXd localTangent = positionPartial;
            localTangent.noalias() += partialTwist * twistGradient.transpose();
            localTangent.noalias() += twistGradient * torquePositionPartial.transpose();
            localTangent.noalias() += torqueTwist * twistGradient * twistGradient.transpose();
            localTangent.noalias() += torque * twistHessian;
            for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
            {
                if (localDofCounts[static_cast<std::size_t>(nodeIndex)] != 6)
                    continue;
                const int offset = (nodeIndex == 0 ? 0 : localDofCounts[0]) + 3;
                Eigen::Matrix3d transport;
                Utility::CR::SkewSymmetric(twistGradient.segment<3>(offset), transport);
                localTangent.block<3, 3>(offset, offset) -= 0.5 * torque * transport;
            }
            for (int column = 0; column < elementDofs; ++column)
            {
                const int freeColumn = dofs[static_cast<std::size_t>(column)] - fixedDofCount;
                if (freeColumn < 0 || freeColumn >= freeDofCount)
                    continue;
                for (int row = 0; row < elementDofs; ++row)
                {
                    const int freeRow = dofs[static_cast<std::size_t>(row)] - fixedDofCount;
                    const double derivative = localTangent(row, column);
                    if (freeRow >= 0 && freeRow < freeDofCount && std::abs(derivative) > 1.0e-14)
                        entries.emplace_back(freeRow, freeColumn, derivative);
                }
            }
            continue;
        }
        const Vec3 axis = Utility::CR::CanonicalAxis(state.positions[1] - state.positions[0]);
        constexpr double increment = 1.0e-6;
        int localColumn = 0;
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIndex)];
            for (int direction = 0; direction < localDofs; ++direction, ++localColumn)
            {
                const int freeColumn = dofs[static_cast<std::size_t>(localColumn)] - fixedDofCount;
                if (freeColumn < 0 || freeColumn >= freeDofCount)
                    continue;
                EccentricGravityState plus = state;
                EccentricGravityState minus = state;
                if (direction < 3)
                {
                    plus.positions[static_cast<std::size_t>(nodeIndex)][direction] += increment;
                    minus.positions[static_cast<std::size_t>(nodeIndex)][direction] -= increment;
                }
                else
                {
                    const double twistIncrement = localDofs == 4 ? increment : axis[direction - 3] * increment;
                    plus.twists[static_cast<std::size_t>(nodeIndex)] += twistIncrement;
                    minus.twists[static_cast<std::size_t>(nodeIndex)] -= twistIncrement;
                }
                const Vec derivative =
                    (EccentricGravityElementForce(*cable, section->m_MassFirstMomentPerLength, gravityAcceleration,
                                                  plus) -
                     EccentricGravityElementForce(*cable, section->m_MassFirstMomentPerLength, gravityAcceleration,
                                                  minus)) /
                    (2.0 * increment);
                for (int localRow = 0; localRow < derivative.size(); ++localRow)
                {
                    const int freeRow = dofs[static_cast<std::size_t>(localRow)] - fixedDofCount;
                    if (freeRow >= 0 && freeRow < freeDofCount && std::abs(derivative[localRow]) > 1.0e-14)
                        entries.emplace_back(freeRow, freeColumn, derivative[localRow]);
                }
            }
        }
    }
    externalTangent.setFromTriplets(entries.cbegin(), entries.cend());
    externalTangent.makeCompressed();
}

void LoadAssembler::AssembleDeformedGravityTangent(const Force_Gravity& gravity, const StructureData& structure,
                                                   int fixedDofCount, int freeDofCount, double scale,
                                                   _OUT Eigen::SparseMatrix<double>& externalTangent)
{
    externalTangent.resize(freeDofCount, freeDofCount);
    externalTangent.setZero();
    if (scale == 0.0)
        return;

    std::vector<Eigen::Triplet<double>> entries;
    for (const auto& [elementId, element] : structure.m_Elements)
    {
        Q_UNUSED(elementId);
        if (!element || std::dynamic_pointer_cast<ElementCable>(element) ||
            std::dynamic_pointer_cast<ElementTruss>(element))
            continue;
        const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(element);
        if (beam && beam->IsReferenceTssbnConstitutiveMode())
            continue;
        const auto property = element->m_pProperty.lock();
        const auto material = property ? property->m_pMaterial.lock() : nullptr;
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!material || !section)
            throw std::runtime_error("Gravity load requires material and section");
        if (element->m_pNode.size() != 2)
            throw std::runtime_error("Distributed load requires a two-node element");
        const auto first = element->m_pNode[0].lock();
        const auto second = element->m_pNode[1].lock();
        if (!first || !second)
            throw std::runtime_error("Element node reference is invalid");

        const Vec3 firstPosition(first->m_X + first->m_Displacement[0], first->m_Y + first->m_Displacement[1],
                                 first->m_Z + first->m_Displacement[2]);
        const Vec3 secondPosition(second->m_X + second->m_Displacement[0], second->m_Y + second->m_Displacement[1],
                                  second->m_Z + second->m_Displacement[2]);
        const double lineLoadMagnitude = material->m_Density * section->m_Area * gravity.m_g * scale;
        const Vec3 lineLoad = GlobalLineLoad(gravity.m_Direction, lineLoadMagnitude);
        const Eigen::MatrixXd localTangent =
            SpatialUniformLineLoadTangent(*element, firstPosition, secondPosition, lineLoad);
        std::vector<int> dofs;
        element->GetDOFs(dofs);
        if (dofs.size() != static_cast<std::size_t>(localTangent.rows()))
            throw std::runtime_error("Element load tangent size does not match its DOFs");
        for (int column = 0; column < localTangent.cols(); ++column)
        {
            const int freeColumn = dofs[static_cast<std::size_t>(column)] - fixedDofCount;
            if (freeColumn < 0 || freeColumn >= freeDofCount)
                continue;
            for (int row = 0; row < localTangent.rows(); ++row)
            {
                const int freeRow = dofs[static_cast<std::size_t>(row)] - fixedDofCount;
                const double derivative = localTangent(row, column);
                if (freeRow >= 0 && freeRow < freeDofCount && std::abs(derivative) > 1.0e-14)
                    entries.emplace_back(freeRow, freeColumn, derivative);
            }
        }
    }
    externalTangent.setFromTriplets(entries.cbegin(), entries.cend());
    externalTangent.makeCompressed();
}

void LoadAssembler::AssembleGalloping(const Force_Wind& wind, const StructureData& structure,
                                      const std::map<int, int>& elementProfileBindings, int iceThickness,
                                      double initialAttackDegrees, const Eigen::Vector3d& modelUp, int fixedDofs,
                                     double scale, _OUT Vec& fixed, _OUT Vec& free,
                                      _OUT GallopingAssemblyTiming* timing, bool referenceTssbn)
{
    if (timing)
        *timing = {};
    const Vec3 windVelocity = wind.GetWindVelocityGlobal();
    if (windVelocity.norm() <= 1.0e-12)
        throw std::runtime_error("Galloping wind direction vector must be nonzero");

    // 每种分裂数在一次装配中只查询一次工况，单元循环内不读取文件。
    std::map<int, const std::vector<BladeModel>*> boundCases;
    std::vector<std::pair<int, std::shared_ptr<ElementBase>>> workItems;
    workItems.reserve(structure.m_Elements.size());
    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad() || element->m_pNode.size() != 2)
            continue;
        const int bundleCount = element->m_AeroBundleCount;
        if (bundleCount <= 0 || element->m_WireId < 0)
            throw std::runtime_error("Galloping element has incomplete aerodynamic metadata");
        auto caseIt = boundCases.find(bundleCount);
        if (caseIt == boundCases.end())
        {
            caseIt = boundCases.emplace(bundleCount, ResolveGallopingCaseData(structure, bundleCount, wind.m_velocity,
                                                                              iceThickness)).first;
        }
        if (!caseIt->second)
            throw std::runtime_error("Galloping aerodynamic case is not loaded");

        const auto binding = elementProfileBindings.find(id);
        if (binding == elementProfileBindings.end())
            throw std::runtime_error("Galloping element has no step-level aerodynamic profile binding");
        const auto first = element->m_pNode[0].lock();
        const auto second = element->m_pNode[1].lock();
        if (!first || !second)
            throw std::runtime_error("Galloping element node reference is invalid");
        ValidateBundledGallopingAxis(id, *element, CurrentNodePosition(*first), CurrentNodePosition(*second), modelUp);
        workItems.emplace_back(id, element);
    }
    if (workItems.empty())
        return;
    if (timing)
        timing->elementEvaluations = static_cast<int>(workItems.size());

    // 参考线路的小批气动力计算保持串行，避免任务调度超过单元求值成本。
    constexpr std::size_t minimumReferenceParallelItems = 128;
    const int requestedWorkers = referenceTssbn && workItems.size() < minimumReferenceParallelItems
                                     ? 1
                                     : SolverNameSpace::AssemblySettings::ThreadCount();
    const auto ranges = BuildUniformWorkRanges(workItems.size(), requestedWorkers);
    std::vector<Vec> fixedBuffers(ranges.size(), Vec::Zero(fixed.size()));
    std::vector<Vec> freeBuffers(ranges.size(), Vec::Zero(free.size()));
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ExternalForce)};
    SolverNameSpace::ParallelAssemblyEngine engine;
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    QElapsedTimer elementLoopTimer;
    if (timing)
        elementLoopTimer.start();
    engine.ExecuteRanges(
        request, ranges, threadPool,
        [&workItems, &boundCases, &elementProfileBindings, &wind, &structure, initialAttackDegrees, modelUp, fixedDofs,
         scale, referenceTssbn, &fixedBuffers,
         &freeBuffers](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        {
            Vec& fixedBuffer = fixedBuffers[worker];
            Vec& freeBuffer = freeBuffers[worker];
            for (std::size_t index = range.first; index < range.last; ++index)
            {
                const auto& [id, element] = workItems[index];
                const auto caseIt = boundCases.find(element->m_AeroBundleCount);
                const auto binding = elementProfileBindings.find(id);
                if (caseIt == boundCases.end() || !caseIt->second || binding == elementProfileBindings.end())
                    throw std::runtime_error("Galloping aerodynamic work item is incomplete");
                const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
                const Vec elementForce = EvaluateGallopingElementForce(*element, cable.get(), wind, structure,
                                                                       *caseIt->second, binding->second,
                                                                       initialAttackDegrees, modelUp, scale,
                                                                       referenceTssbn);
                AddElementForce(*element, elementForce, fixedDofs, fixedBuffer, freeBuffer);
            }
        });
    if (timing)
        timing->elementLoopMilliseconds = static_cast<double>(elementLoopTimer.nsecsElapsed()) / 1.0e6;
    for (std::size_t worker = 0; worker < ranges.size(); ++worker)
    {
        fixed += fixedBuffers[worker];
        free += freeBuffers[worker];
    }
}

void LoadAssembler::AssembleGallopingTangent(const Force_Wind& wind, const StructureData& structure,
                                             const std::map<int, int>& elementProfileBindings, int iceThickness,
                                             double initialAttackDegrees, const Eigen::Vector3d& modelUp,
                                             int fixedDofCount, int freeDofCount, double scale,
                                             double velocityDerivative,
                                             _OUT Eigen::SparseMatrix<double>& externalTangent, bool referenceTssbn)
{
    externalTangent.resize(freeDofCount, freeDofCount);
    externalTangent.setZero();
    if (scale == 0.0 || freeDofCount <= 0)
        return;
    if (wind.GetWindVelocityGlobal().norm() <= 1.0e-12)
        throw std::runtime_error("Galloping wind direction vector must be nonzero");

    const double correctionIncrement =
        std::min(1.0e-6, 1.0e-3 / std::max(1.0, std::abs(velocityDerivative)));
    std::map<int, const std::vector<BladeModel>*> boundCases;
    std::vector<std::pair<int, std::shared_ptr<ElementBase>>> workItems;
    workItems.reserve(structure.m_Elements.size());
    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad() || element->m_pNode.size() != 2)
            continue;
        const int bundleCount = element->m_AeroBundleCount;
        if (bundleCount <= 0 || element->m_WireId < 0)
            throw std::runtime_error("Galloping element has incomplete aerodynamic metadata");
        auto caseIt = boundCases.find(bundleCount);
        if (caseIt == boundCases.end())
        {
            caseIt = boundCases.emplace(bundleCount, ResolveGallopingCaseData(structure, bundleCount, wind.m_velocity,
                                                                              iceThickness)).first;
        }
        if (!caseIt->second)
            throw std::runtime_error("Galloping aerodynamic case is not loaded");
        const auto binding = elementProfileBindings.find(id);
        if (binding == elementProfileBindings.end())
            throw std::runtime_error("Galloping element has no step-level aerodynamic profile binding");
        const auto first = element->m_pNode[0].lock();
        const auto second = element->m_pNode[1].lock();
        if (!first || !second)
            throw std::runtime_error("Galloping element node reference is invalid");
        const Vec3 chord = CurrentNodePosition(*second) - CurrentNodePosition(*first);
        ValidateBundledGallopingAxis(id, *element, Vec3::Zero(), chord, modelUp);
        if (bundleCount > 1)
        {
            // 数值切线会扰动两端平动自由度。跨越规范轴切换面时差分无定义，
            // 在并行装配之前明确报错，避免异常停留在工作线程中。
            for (int component = 0; component < 3; ++component)
            {
                const Vec3 correction = correctionIncrement * Vec3::Unit(component);
                ValidateBundledGallopingAxis(id, *element, Vec3::Zero(), chord + correction, modelUp);
                ValidateBundledGallopingAxis(id, *element, Vec3::Zero(), chord - correction, modelUp);
            }
        }
        workItems.emplace_back(id, element);
    }
    if (workItems.empty())
        return;

    const auto ranges = BuildUniformWorkRanges(workItems.size(), SolverNameSpace::AssemblySettings::ThreadCount());
    std::vector<std::vector<Eigen::Triplet<double>>> entryBuffers(ranges.size());
    for (std::vector<Eigen::Triplet<double>>& buffer : entryBuffers)
        buffer.reserve(std::max<std::size_t>(64, workItems.size() * 64 / ranges.size()));
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ExternalTangent), 0.0, 1.0, 0.0,
        velocityDerivative};
    SolverNameSpace::ParallelAssemblyEngine engine;
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    engine.ExecuteRanges(
        request, ranges, threadPool,
        [&workItems, &boundCases, &elementProfileBindings, &wind, &structure, initialAttackDegrees, modelUp,
         fixedDofCount, freeDofCount, correctionIncrement, velocityDerivative, scale, referenceTssbn,
         &entryBuffers](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        {
            AssembleGallopingTangentRange(range.first, range.last, workItems, boundCases, elementProfileBindings, wind,
                                          structure, initialAttackDegrees, modelUp, fixedDofCount, freeDofCount,
                                          correctionIncrement, velocityDerivative, scale, entryBuffers[worker],
                                          referenceTssbn);
        });

    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(workItems.size() * 64);
    for (const std::vector<Eigen::Triplet<double>>& buffer : entryBuffers)
        entries.insert(entries.end(), buffer.cbegin(), buffer.cend());
    externalTangent.setFromTriplets(entries.begin(), entries.end());
    externalTangent.makeCompressed();
}
