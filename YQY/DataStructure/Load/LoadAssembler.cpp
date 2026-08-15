#include "LoadAssembler.h"

#include <algorithm>
#include <stdexcept>

#include "DataStructure/Aerodynamics/AeroManager.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Load/AerodynamicLoadCalculator.h"
#include "DataStructure/Load/Force_Element.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Node.h"
#include "DataStructure/Load/Force_Wind.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Node/Node.h"
#include "DataStructure/Structure/StructureData.h"
#include "Utility/CR.h"

namespace
{
using Vec = Eigen::VectorXd;
using Vec3 = Eigen::Vector3d;

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
            fixed[dof] += force[i];
        else
            free[dof - fixedDofs] += force[i];
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
        fixed[dof] += value;
    else
        free[dof - fixedDofs] += value;
}

Vec SpatialUniformLineLoad(const ElementBase& element, const Vec3& q)
{
    if (element.m_pNode.size() != 2)
        throw std::runtime_error("Distributed load requires a two-node element");
    const auto first = element.m_pNode[0].lock();
    const auto second = element.m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("Element node reference is invalid");

    const Vec3 x1(first->m_X + first->m_Displacement[0], first->m_Y + first->m_Displacement[1],
                  first->m_Z + first->m_Displacement[2]);
    const Vec3 x2(second->m_X + second->m_Displacement[0], second->m_Y + second->m_Displacement[1],
                  second->m_Z + second->m_Displacement[2]);
    const Vec3 chord = x2 - x1;
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

Vec3 GlobalLineLoad(EnumKeyword::Direction direction, double magnitude)
{
    int component = -1;
    if (!TranslationComponent(direction, component))
        throw std::runtime_error("Distributed load direction must be X, Y, or Z");
    Vec3 q = Vec3::Zero();
    q[component] = magnitude;
    return q;
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

        const double q = material->m_Density * section->m_Area * load.m_g * scale;
        AddElementForce(*element, SpatialUniformLineLoad(*element, GlobalLineLoad(load.m_Direction, q)), fixedDofs,
                        fixed, free);
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

Vec EvaluateGallopingElementForce(const ElementBase& element, const ElementCable* cable, const Force_Wind& wind,
                                  const StructureData& structure, const std::vector<BladeModel>& caseData,
                                  int profileId, double initialAttackDegrees, double scale)
{
    const auto first = element.m_pNode[0].lock();
    const auto second = element.m_pNode[1].lock();
    const auto property = element.m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!first || !second || !section)
        throw std::runtime_error("Galloping element has incomplete node/section data");

    AerodynamicSectionState state;
    state.firstPosition = Vec3(first->m_X + first->m_Displacement[0], first->m_Y + first->m_Displacement[1],
                               first->m_Z + first->m_Displacement[2]);
    state.secondPosition = Vec3(second->m_X + second->m_Displacement[0], second->m_Y + second->m_Displacement[1],
                                second->m_Z + second->m_Displacement[2]);
    state.firstVelocity = Vec3(first->m_Velocity[0], first->m_Velocity[1], first->m_Velocity[2]);
    state.secondVelocity = Vec3(second->m_Velocity[0], second->m_Velocity[1], second->m_Velocity[2]);
    state.windVelocity = wind.GetWindVelocityGlobal();
    state.radius = section->m_Radius;
    state.initialAttack = initialAttackDegrees * std::acos(-1.0) / 180.0;
    if (cable)
    {
        state.firstTwist = cable->GetNodalTwist(0);
        state.secondTwist = cable->GetNodalTwist(1);
        state.firstTwistRate = cable->GetNodalTwistRate(0);
        state.secondTwistRate = cable->GetNodalTwistRate(1);
    }

    AerodynamicSectionResult result = AerodynamicLoadCalculator::ComputeKinematics(state);
    if (result.relativeSpeed <= 1.0e-12)
    {
        std::vector<int> localDofCounts;
        element.GetNodeLocalDOFCounts(localDofCounts);
        return Vec::Zero(localDofCounts[0] + localDofCounts[1]);
    }
    const AeroCoefficients coefficients = structure.m_AeroManager.getCoefficients(
        caseData, profileId,
        AeroManager::normalizeAngleDegrees(result.attackAngle * 180.0 / std::acos(-1.0)));
    AerodynamicLoadCalculator::ComputeLineLoad(result, wind.m_windDensity, 2.0 * section->m_Radius,
                                               coefficients.lift, coefficients.drag, coefficients.moment);

    Vec elementForce = SpatialUniformLineLoad(element, scale * result.lineForce);
    const double currentLength = (state.secondPosition - state.firstPosition).norm();
    if (cable)
    {
        const double nodalTorque = 0.5 * currentLength * scale * result.lineMoment.dot(result.axis);
        cable->AddNodalAxialTorque(0, nodalTorque, elementForce);
        cable->AddNodalAxialTorque(1, nodalTorque, elementForce);
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

void LoadAssembler::AssembleGalloping(const Force_Wind& wind, const StructureData& structure,
                                      const std::map<int, int>& elementProfileBindings, int iceThickness,
                                      double initialAttackDegrees, const Eigen::Vector3d& modelUp, int fixedDofs,
                                      double scale, _OUT Vec& fixed, _OUT Vec& free)
{
    const Vec3 windVelocity = wind.GetWindVelocityGlobal();
    if (windVelocity.norm() <= 1.0e-12)
        throw std::runtime_error("Galloping wind direction vector must be nonzero");

    // Compatibility for older models that only stored profile IDs.
    int legacyBundleCount = 1;
    for (const auto& [id, element] : structure.m_Elements)
        if (element && element->HasAerodynamicLoad())
            legacyBundleCount = std::max(legacyBundleCount, element->m_AeroProfileId + 1);
    if (legacyBundleCount > 1)
        legacyBundleCount = 4;

    // One case lookup per bundle count and assembly pass; no file access occurs
    // in the element loop.
    std::map<int, const std::vector<BladeModel>*> boundCases;
    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad() || element->m_pNode.size() != 2)
            continue;
        const int bundleCount = element->m_AeroBundleCount > 0 ? element->m_AeroBundleCount : legacyBundleCount;
        auto caseIt = boundCases.find(bundleCount);
        if (caseIt == boundCases.end())
        {
            const AeroCaseKey key{bundleCount, static_cast<int>(std::llround(wind.m_velocity)), iceThickness};
            caseIt = boundCases.emplace(bundleCount, structure.m_AeroManager.findCaseModels(key)).first;
        }
        if (!caseIt->second)
            throw std::runtime_error("Galloping aerodynamic case is not loaded");

        const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
        const auto binding = elementProfileBindings.find(id);
        if (binding == elementProfileBindings.end())
            throw std::runtime_error("Galloping element has no step-level aerodynamic profile binding");
        const Vec elementForce = EvaluateGallopingElementForce(*element, cable.get(), wind, structure, *caseIt->second,
                                                               binding->second, initialAttackDegrees, scale);
        AddElementForce(*element, elementForce, fixedDofs, fixed, free);
    }
}

void LoadAssembler::AssembleGallopingTangent(const Force_Wind& wind, const StructureData& structure,
                                              const std::map<int, int>& elementProfileBindings, int iceThickness,
                                              double initialAttackDegrees, int fixedDofCount, int freeDofCount,
                                              double scale, double velocityDerivative,
                                              _OUT Eigen::SparseMatrix<double>& externalTangent)
{
    externalTangent.resize(freeDofCount, freeDofCount);
    externalTangent.setZero();
    if (scale == 0.0 || freeDofCount <= 0)
        return;
    if (wind.GetWindVelocityGlobal().norm() <= 1.0e-12)
        throw std::runtime_error("Galloping wind direction vector must be nonzero");

    int legacyBundleCount = 1;
    for (const auto& [id, element] : structure.m_Elements)
        if (element && element->HasAerodynamicLoad())
            legacyBundleCount = std::max(legacyBundleCount, element->m_AeroProfileId + 1);
    if (legacyBundleCount > 1)
        legacyBundleCount = 4;

    const double correctionIncrement =
        std::min(1.0e-6, 1.0e-3 / std::max(1.0, std::abs(velocityDerivative)));
    std::map<int, const std::vector<BladeModel>*> boundCases;
    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(structure.m_Elements.size() * 64);

    for (const auto& [id, element] : structure.m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad() || element->m_pNode.size() != 2)
            continue;
        const int bundleCount = element->m_AeroBundleCount > 0 ? element->m_AeroBundleCount : legacyBundleCount;
        auto caseIt = boundCases.find(bundleCount);
        if (caseIt == boundCases.end())
        {
            const AeroCaseKey key{bundleCount, static_cast<int>(std::llround(wind.m_velocity)), iceThickness};
            caseIt = boundCases.emplace(bundleCount, structure.m_AeroManager.findCaseModels(key)).first;
        }
        if (!caseIt->second)
            throw std::runtime_error("Galloping aerodynamic case is not loaded");
        const auto binding = elementProfileBindings.find(id);
        if (binding == elementProfileBindings.end())
            throw std::runtime_error("Galloping element has no step-level aerodynamic profile binding");

        const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
        std::vector<int> dofs;
        std::vector<int> localDofCounts;
        element->GetDOFs(dofs);
        element->GetNodeLocalDOFCounts(localDofCounts);
        const int elementDofCount =
            localDofCounts.size() == 2 ? localDofCounts[0] + localDofCounts[1] : 0;
        if (dofs.size() != static_cast<std::size_t>(elementDofCount) || localDofCounts.size() != 2)
            throw std::runtime_error("Galloping tangent element layout is inconsistent");

        int localColumn = 0;
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            const auto node = element->m_pNode[static_cast<std::size_t>(nodeIndex)].lock();
            if (!node)
                throw std::runtime_error("Galloping tangent node reference is invalid");
            for (int direction = 0; direction < localDofCounts[static_cast<std::size_t>(nodeIndex)]; ++direction)
            {
                const int freeColumn = dofs[static_cast<std::size_t>(localColumn)] - fixedDofCount;
                if (freeColumn < 0 || freeColumn >= freeDofCount)
                {
                    ++localColumn;
                    continue;
                }

                const Node savedNode = *node;
                PerturbNodeCorrection(*node, direction, correctionIncrement, velocityDerivative);
                const Vec forcePlus = EvaluateGallopingElementForce(
                    *element, cable.get(), wind, structure, *caseIt->second, binding->second, initialAttackDegrees,
                    scale);
                *node = savedNode;
                PerturbNodeCorrection(*node, direction, -correctionIncrement, velocityDerivative);
                const Vec forceMinus = EvaluateGallopingElementForce(
                    *element, cable.get(), wind, structure, *caseIt->second, binding->second, initialAttackDegrees,
                    scale);
                *node = savedNode;
                const Vec derivative = (forcePlus - forceMinus) / (2.0 * correctionIncrement);
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
    externalTangent.setFromTriplets(entries.begin(), entries.end());
    externalTangent.makeCompressed();
}
