#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionCircular.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
void CheckNear(double actual, double expected, double tolerance, const char* message)
{
    if (std::abs(actual - expected) > tolerance)
        throw std::runtime_error(message);
}
}

int RunTests()
{
    auto node0 = std::make_shared<Node>();
    auto node1 = std::make_shared<Node>();
    node0->SetNumDOFs(4);
    node1->SetNumDOFs(4);
    node1->m_X = 2.0;

    auto material = std::make_shared<Material>();
    material->m_Young = 200.0;
    material->m_Poisson = 0.25;
    material->m_Density = 3.0;

    auto section = std::make_shared<SectionCircular>();
    const double radius = 0.1;
    section->m_Area = PI * radius * radius;
    section->Calculate_Radius();

    auto property = std::make_shared<Property>();
    property->m_pMaterial = material;
    property->m_pSection = section;

    ElementCable cable;
    cable.m_pNode[0] = node0;
    cable.m_pNode[1] = node1;
    cable.m_pProperty = property;

    Eigen::MatrixXd stiffness;
    cable.Get_ke(stiffness);
    CheckNear(cable.L0, 2.0, 1.0e-12, "initial cable length");
    if ((stiffness - stiffness.transpose()).norm() > 1.0e-12)
        throw std::runtime_error("cable stiffness must be symmetric");

    Eigen::VectorXd rigidMotion = Eigen::VectorXd::Zero(8);
    rigidMotion.segment<3>(0) = Eigen::Vector3d(0.2, -0.1, 0.3);
    rigidMotion.segment<3>(4) = rigidMotion.segment<3>(0);
    rigidMotion(3) = rigidMotion(7) = 0.4;
    if ((stiffness * rigidMotion).norm() > 1.0e-12)
        throw std::runtime_error("rigid cable motion must not create elastic force");

    node1->m_Displacement[0] = 0.1;
    node1->m_Displacement[3] = 0.02;
    cable.Get_ke(stiffness);
    if (cable.m_inforce.norm() <= 0.0)
        throw std::runtime_error("extended/twisted cable must create internal force");
    CheckNear(cable.m_inforce.head<4>().sum() + cable.m_inforce.tail<4>().sum(), 0.0, 1.0e-12,
              "cable internal force equilibrium");

    Eigen::MatrixXd mass;
    cable.Get_me_Consistent(mass);
    if ((mass - mass.transpose()).norm() > 1.0e-12)
        throw std::runtime_error("cable mass must be symmetric");
    const double translationalMass = material->m_Density * section->m_Area * cable.L0;
    CheckNear(mass(0, 0) + mass(0, 4) + mass(4, 0) + mass(4, 4), translationalMass, 1.0e-12,
              "consistent translational mass");
    const double polarMoment = 0.5 * section->m_Area * radius * radius;
    CheckNear(mass(3, 3) + mass(3, 7) + mass(7, 3) + mass(7, 7), material->m_Density * polarMoment * cable.L0, 1.0e-12,
              "consistent torsional mass");

    // Cable 与 Beam 共节点时，普通端保留标量扭转，共节点端使用三个空间转角。
    const Eigen::Vector3d sharedAxis = Eigen::Vector3d(1.0, 2.0, 3.0).normalized();
    auto scalarNode = std::make_shared<Node>();
    auto sharedNode = std::make_shared<Node>();
    scalarNode->SetNumDOFs(4);
    sharedNode->SetNumDOFs(6);
    sharedNode->m_X = 2.0 * sharedAxis.x();
    sharedNode->m_Y = 2.0 * sharedAxis.y();
    sharedNode->m_Z = 2.0 * sharedAxis.z();
    constexpr double sharedTwist = 0.03;
    const Eigen::Vector3d transverse = sharedAxis.unitOrthogonal();
    sharedNode->m_Rg = Eigen::AngleAxisd(0.35, transverse).toRotationMatrix() *
                       Eigen::AngleAxisd(sharedTwist, sharedAxis).toRotationMatrix();
    sharedNode->m_Velocity[3] = 0.4 * sharedAxis.x();
    sharedNode->m_Velocity[4] = 0.4 * sharedAxis.y();
    sharedNode->m_Velocity[5] = 0.4 * sharedAxis.z();

    ElementCable sharedCable;
    sharedCable.m_pNode[0] = scalarNode;
    sharedCable.m_pNode[1] = sharedNode;
    sharedCable.m_pProperty = property;
    sharedCable.m_InitStress = 100.0;
    Eigen::MatrixXd sharedStiffness;
    Eigen::MatrixXd sharedMass;
    sharedCable.Get_ke(sharedStiffness);
    sharedCable.Get_me_Consistent(sharedMass);
    std::vector<int> sharedDofCounts;
    sharedCable.GetNodeLocalDOFCounts(sharedDofCounts);
    if (sharedDofCounts != std::vector<int>({4, 6}) || sharedStiffness.rows() != 10 || sharedMass.rows() != 10)
        throw std::runtime_error("mixed cable DOF layout must be 4+6");

    const double sharedTorsionalStiffness = material->m_Young / (2.0 * (1.0 + material->m_Poisson)) * polarMoment / 2.0;
    const Eigen::Vector3d sharedMoment = sharedCable.m_inforce.segment<3>(7);
    CheckNear(sharedCable.GetNodalTwist(1), sharedTwist, 1.0e-12, "shared-node axial twist");
    CheckNear(sharedCable.GetNodalTwistRate(1), 0.4, 1.0e-12, "shared-node axial twist rate");
    CheckNear(sharedCable.m_inforce[3], -sharedTorsionalStiffness * sharedTwist, 1.0e-12, "scalar-end torsional force");
    CheckNear((sharedMoment - sharedTorsionalStiffness * sharedTwist * sharedAxis).norm(), 0.0, 1.0e-12,
              "shared-node torsional moment vector");
    CheckNear((sharedMoment - sharedAxis * sharedMoment.dot(sharedAxis)).norm(), 0.0, 1.0e-12,
              "Cable must not transmit transverse moment to the Beam node");
    CheckNear(sharedAxis.dot(sharedStiffness.block<3, 3>(7, 7) * sharedAxis), sharedTorsionalStiffness, 1.0e-12,
              "shared-node torsional stiffness");
    CheckNear((sharedStiffness.block<3, 3>(7, 7) * transverse).norm(), 0.0, 1.0e-12,
              "Cable transverse rotational stiffness");
    Eigen::VectorXd axialTorque = Eigen::VectorXd::Zero(10);
    sharedCable.AddNodalAxialTorque(1, 2.5, axialTorque);
    CheckNear((axialTorque.segment<3>(7) - 2.5 * sharedAxis).norm(), 0.0, 1.0e-12,
              "shared-node external axial torque mapping");

    ElementCable reversedCable;
    reversedCable.m_pNode[0] = sharedNode;
    reversedCable.m_pNode[1] = scalarNode;
    reversedCable.m_pProperty = property;
    reversedCable.m_InitStress = sharedCable.m_InitStress;
    Eigen::MatrixXd reversedStiffness;
    reversedCable.Get_ke(reversedStiffness);
    CheckNear((reversedCable.m_inforce.head<6>() - sharedCable.m_inforce.tail<6>()).norm(), 0.0, 1.0e-12,
              "reversed inclined Cable force at shared node");
    CheckNear((reversedCable.m_inforce.tail<4>() - sharedCable.m_inforce.head<4>()).norm(), 0.0, 1.0e-12,
              "reversed inclined Cable force at scalar node");

    for (int index = 0; index < scalarNode->m_DOF.size(); ++index)
        scalarNode->m_DOF[index] = index;
    for (int index = 0; index < sharedNode->m_DOF.size(); ++index)
        sharedNode->m_DOF[index] = 4 + index;
    auto beamEndNode = std::make_shared<Node>();
    beamEndNode->SetNumDOFs(6);
    for (int index = 0; index < beamEndNode->m_DOF.size(); ++index)
        beamEndNode->m_DOF[index] = 10 + index;
    ElementBeam_CR beam;
    beam.m_pNode[0] = sharedNode;
    beam.m_pNode[1] = beamEndNode;
    std::vector<int> cableDofs;
    std::vector<int> beamDofs;
    sharedCable.GetDOFs(cableDofs);
    beam.GetDOFs(beamDofs);
    if (cableDofs.size() != 10 || beamDofs.size() != 12 ||
        !std::equal(cableDofs.cbegin() + 7, cableDofs.cbegin() + 10, beamDofs.cbegin() + 3))
        throw std::runtime_error("Cable and Beam must assemble to the same three shared-node rotation DOFs");

    std::cout << "cable stiffness, mass, and shared Beam-node tests passed\n";
    return 0;
}

int main()
{
    try
    {
        return RunTests();
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
