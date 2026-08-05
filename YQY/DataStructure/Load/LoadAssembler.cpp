#include "LoadAssembler.h"

#include <algorithm>
#include <stdexcept>

#include "DataStructure/Aerodynamics/AeroManager.h"
#include "DataStructure/Aerodynamics/BundleAeroMapper.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Load/AerodynamicLoadCalculator.h"
#include "DataStructure/Load/Force_Element.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Node.h"
#include "DataStructure/Load/Force_Wind.h"
#include "DataStructure/Node/Node.h"
#include "DataStructure/Structure/StructureData.h"

namespace
{
using Vec = Eigen::VectorXd;
using Vec3 = Eigen::Vector3d;

bool TranslationComponent(EnumKeyword::Direction direction, int& component)
{
    component = static_cast<int>(direction);
    return component >= 0 && component < 3;
}

void AddElementForce(const ElementBase& element, const Vec& force,
    int fixedDofs, Vec& fixed, Vec& free)
{
    std::vector<int> dofs;
    element.GetDOFs(dofs);
    if (dofs.size() != static_cast<std::size_t>(force.size()))
        throw std::runtime_error("Element load vector size does not match its DOFs");

    for (int i = 0; i < force.size(); ++i)
    {
        const int dof = dofs[static_cast<std::size_t>(i)];
        if (dof < 0) continue;
        if (dof < fixedDofs) fixed[dof] += force[i];
        else free[dof - fixedDofs] += force[i];
    }
}

void AddNodeForce(const Node& node, int component, double value,
    int fixedDofs, Vec& fixed, Vec& free)
{
    if (component < 0 || component >= node.m_DOF.size()) return;
    const int dof = node.m_DOF[component];
    if (dof < 0) return;
    if (dof < fixedDofs) fixed[dof] += value;
    else free[dof - fixedDofs] += value;
}

// For a spatial uniform load q, this is equivalent to THOP's sequence:
// q_local = R_current^T q -> local consistent load -> R_current transform.
// Writing its invariant global form avoids duplicating every element's local
// frame definition while still using the current chord and producing beam end
// moments correctly.
Vec SpatialUniformLineLoad(const ElementBase& element, const Vec3& q)
{
    if (element.m_pNode.size() != 2)
        throw std::runtime_error("Distributed load requires a two-node element");
    const auto first = element.m_pNode[0].lock();
    const auto second = element.m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("Element node reference is invalid");

    const Vec3 x1(first->m_X + first->m_Displacement[0],
        first->m_Y + first->m_Displacement[1], first->m_Z + first->m_Displacement[2]);
    const Vec3 x2(second->m_X + second->m_Displacement[0],
        second->m_Y + second->m_Displacement[1], second->m_Z + second->m_Displacement[2]);
    const Vec3 chord = x2 - x1;
    const double length = chord.norm();
    if (length <= 1.0e-12)
        throw std::runtime_error("Element current length must be positive");

    const int nodeDofs = element.Get_NodeDOF();
    if (nodeDofs < 3)
        throw std::runtime_error("Distributed load requires translational DOFs");

    Vec result = Vec::Zero(2 * nodeDofs);
    result.segment<3>(0) = 0.5 * length * q;
    result.segment<3>(nodeDofs) = 0.5 * length * q;

    if (nodeDofs >= 6)
    {
        const Vec3 endMoment = length * length / 12.0 * chord.normalized().cross(q);
        result.segment<3>(3) = endMoment;
        result.segment<3>(nodeDofs + 3) = -endMoment;
    }
    return result;
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

void AssembleGravity(const Force_Gravity& load, const StructureData& structure,
    int fixedDofs, double scale, Vec& fixed, Vec& free)
{
    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element) continue;
        const auto property = element->m_pProperty.lock();
        const auto material = property ? property->m_pMaterial.lock() : nullptr;
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!material || !section)
            throw std::runtime_error("Gravity load requires material and section");

        const double q = material->m_Density * section->m_Area * load.m_g * scale;
        AddElementForce(*element, SpatialUniformLineLoad(*element, GlobalLineLoad(load.m_Direction, q)),
            fixedDofs, fixed, free);
    }
}

void AssembleWind(const Force_Wind& load, const StructureData& structure,
    int fixedDofs, double scale, Vec& fixed, Vec& free)
{
    const Vec3 windVelocity = load.GetWindVelocityGlobal();
    if (windVelocity.norm() <= 1.0e-12)
        throw std::runtime_error("Wind direction vector must be nonzero");
    const Vec3 windDirection = windVelocity.normalized();
    const bool useAeroTags = std::any_of(structure.m_Elements.cbegin(), structure.m_Elements.cend(),
        [](const auto& entry) { return entry.second && entry.second->HasAerodynamicLoad(); });

    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element || (useAeroTags && !element->HasAerodynamicLoad())) continue;
        const auto property = element->m_pProperty.lock();
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!section)
            throw std::runtime_error("Wind load requires a section");

        // Phase 1: wind pressure only, deliberately without Cd or aeroelastic terms.
        const double q = 0.5 * load.m_windDensity * load.m_velocity * load.m_velocity
            * (2.0 * section->m_Radius) * scale;
        AddElementForce(*element, SpatialUniformLineLoad(*element, q * windDirection),
            fixedDofs, fixed, free);
    }
}
}

void LoadAssembler::Assemble(const LoadBase& load, const StructureData& structure,
    int fixedDofs, double scale, Vec& fixed, Vec& free)
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
            Vec force = Vec::Zero(2 * element->Get_NodeDOF());
            force[component] = force[element->Get_NodeDOF() + component] = 0.5 * item.m_Value * scale;
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

void LoadAssembler::AssembleGalloping(const Force_Wind& wind,
    const StructureData& structure, int iceThickness,
    double initialAttackDegrees,
    const Eigen::Vector3d& modelUp, int fixedDofs, double scale,
    Vec& fixed, Vec& free)
{
    const Vec3 windVelocity = wind.GetWindVelocityGlobal();
    if (windVelocity.norm() <= 1.0e-12)
        throw std::runtime_error("Galloping wind direction vector must be nonzero");

    // Compatibility for older models that only stored profile IDs.
    int legacyBundleCount = 1;
    for (const auto& [id, element] : structure.m_Elements)
        if (element && element->HasAerodynamicLoad())
            legacyBundleCount = std::max(
                legacyBundleCount, element->m_AeroProfileId + 1);
    if (legacyBundleCount > 1)
        legacyBundleCount = 4;

    // One case lookup per bundle count and assembly pass; no file access occurs
    // in the element loop.
    std::map<int, const std::vector<BladeModel>*> boundCases;
    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad()
            || element->m_pNode.size() != 2)
            continue;
        const auto first = element->m_pNode[0].lock();
        const auto second = element->m_pNode[1].lock();
        const auto property = element->m_pProperty.lock();
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!first || !second || !section)
            throw std::runtime_error(
                "Galloping element has incomplete node/section data");

        const int bundleCount = element->m_AeroBundleCount > 0
            ? element->m_AeroBundleCount : legacyBundleCount;
        auto caseIt = boundCases.find(bundleCount);
        if (caseIt == boundCases.end())
        {
            const AeroCaseKey key{
                bundleCount,
                static_cast<int>(std::llround(wind.m_velocity)),
                iceThickness };
            caseIt = boundCases.emplace(
                bundleCount, structure.m_AeroManager.findCaseModels(key)).first;
        }
        if (!caseIt->second)
            throw std::runtime_error(
                "Galloping aerodynamic case is not loaded");

        AerodynamicSectionState state;
        state.firstPosition = Vec3(
            first->m_X + first->m_Displacement[0],
            first->m_Y + first->m_Displacement[1],
            first->m_Z + first->m_Displacement[2]);
        state.secondPosition = Vec3(
            second->m_X + second->m_Displacement[0],
            second->m_Y + second->m_Displacement[1],
            second->m_Z + second->m_Displacement[2]);
        state.firstVelocity = Vec3(
            first->m_Velocity[0], first->m_Velocity[1],
            first->m_Velocity[2]);
        state.secondVelocity = Vec3(
            second->m_Velocity[0], second->m_Velocity[1],
            second->m_Velocity[2]);
        state.windVelocity = windVelocity;
        state.radius = section->m_Radius;
        state.initialAttack =
            initialAttackDegrees * std::acos(-1.0) / 180.0;

        const int nodeDofs = element->Get_NodeDOF();
        if (nodeDofs == 4)
        {
            state.firstTwist = first->m_Displacement[3];
            state.secondTwist = second->m_Displacement[3];
            state.firstTwistRate = first->m_Velocity[3];
            state.secondTwistRate = second->m_Velocity[3];
        }

        AerodynamicSectionResult result =
            AerodynamicLoadCalculator::ComputeKinematics(state);
        if (result.relativeSpeed <= 1.0e-12)
            continue;
        const int profileId = BundleAeroMapper::ResolveProfile(
            bundleCount, element->m_WireId, result.axis,
            modelUp, windVelocity);
        const AeroCoefficients coefficients =
            structure.m_AeroManager.getCoefficients(
                *caseIt->second, profileId,
                AeroManager::normalizeAngleDegrees(
                    result.attackAngle * 180.0 / std::acos(-1.0)));
        AerodynamicLoadCalculator::ComputeLineLoad(
            result, wind.m_windDensity, 2.0 * section->m_Radius,
            coefficients.lift, coefficients.drag, coefficients.moment);

        Vec elementForce = SpatialUniformLineLoad(
            *element, scale * result.lineForce);
        const double currentLength =
            (state.secondPosition - state.firstPosition).norm();
        if (nodeDofs == 4)
        {
            const double nodalTorque = 0.5 * currentLength * scale
                * result.lineMoment.dot(result.axis);
            elementForce[3] += nodalTorque;
            elementForce[7] += nodalTorque;
        }
        else if (nodeDofs >= 6)
        {
            const Vec3 nodalMoment =
                0.5 * currentLength * scale * result.lineMoment;
            elementForce.segment<3>(3) += nodalMoment;
            elementForce.segment<3>(nodeDofs + 3) += nodalMoment;
        }
        AddElementForce(
            *element, elementForce, fixedDofs, fixed, free);
    }
}
