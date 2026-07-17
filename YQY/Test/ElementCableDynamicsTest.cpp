#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionCircular.h"

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

int main()
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
    CheckNear(cable.m_inforce.head<4>().sum() + cable.m_inforce.tail<4>().sum(),
        0.0, 1.0e-12, "cable internal force equilibrium");

    Eigen::MatrixXd mass;
    cable.Get_me_Consistent(mass);
    if ((mass - mass.transpose()).norm() > 1.0e-12)
        throw std::runtime_error("cable mass must be symmetric");
    const double translationalMass = material->m_Density * section->m_Area * cable.L0;
    CheckNear(mass(0, 0) + mass(0, 4) + mass(4, 0) + mass(4, 4),
        translationalMass, 1.0e-12, "consistent translational mass");
    const double polarMoment = 0.5 * section->m_Area * radius * radius;
    CheckNear(mass(3, 3) + mass(3, 7) + mass(7, 3) + mass(7, 7),
        material->m_Density * polarMoment * cable.L0,
        1.0e-12, "consistent torsional mass");

    std::cout << "cable stiffness and mass tests passed\n";
    return 0;
}
