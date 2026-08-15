#include "ElementBeam_Simo.h"

#include <stdexcept>

namespace
{
Eigen::Matrix3d Skew(const Eigen::Vector3d& value)
{
    Eigen::Matrix3d result;
    Utility::CR::SkewSymmetric(value, result);
    return result;
}

Eigen::Matrix3d Exp(const Eigen::Vector3d& value)
{
    Eigen::Matrix3d result;
    Utility::CR::Calculate_RotationMatrix(value, result);
    return result;
}

Eigen::Vector3d Log(const Eigen::Matrix3d& value)
{
    Eigen::Vector3d result;
    Utility::CR::Extract_RotationVector(value, result);
    return result;
}
}

ElementBeam_Simo::ElementBeam_Simo()
{
    m_pNode.resize(2);
    m_inforce = Eigen::VectorXd::Zero(12);
}

void ElementBeam_Simo::GetSectionData(double& area, double& Iy, double& Iz, double& J, double& young, double& shear,
                                      double& density) const
{
    auto property = m_pProperty.lock();
    if (!property)
        throw std::runtime_error("ElementBeam_Simo property reference is invalid");
    auto material = property->m_pMaterial.lock();
    auto section = property->m_pSection.lock();
    if (!material || !section)
        throw std::runtime_error("ElementBeam_Simo material/section reference is invalid");

    area = section->m_Area;
    section->Calculate_I(Iy, Iz, J);
    young = material->m_Young;
    density = material->m_Density;
    if (area <= 0.0 || young <= 0.0 || density < 0.0 || 1.0 + material->m_Poisson <= 0.0)
        throw std::runtime_error("ElementBeam_Simo has invalid material/section data");
    shear = young / (2.0 * (1.0 + material->m_Poisson));
}

void ElementBeam_Simo::Get_L0()
{
    auto node0 = m_pNode[0].lock();
    auto node1 = m_pNode[1].lock();
    if (!node0 || !node1)
        throw std::runtime_error("ElementBeam_Simo node reference is invalid");

    const Eigen::Vector3d x0(node0->m_X, node0->m_Y, node0->m_Z);
    const Eigen::Vector3d x1(node1->m_X, node1->m_Y, node1->m_Z);
    const Eigen::Vector3d referenceChord = x1 - x0;
    L0 = referenceChord.norm();
    if (L0 <= 1.0e-12)
        throw std::runtime_error("ElementBeam_Simo initial length must be positive");

    const Eigen::Vector3d e1 = referenceChord / L0;
    Eigen::Vector3d e2 = q0 - e1 * e1.dot(q0);
    if (e2.norm() <= 1.0e-10)
    {
        const Eigen::Vector3d auxiliary = std::abs(e1.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        e2 = auxiliary - e1 * e1.dot(auxiliary);
    }
    e2.normalize();
    const Eigen::Vector3d e3 = e1.cross(e2).normalized();
    e2 = e3.cross(e1).normalized();

    R0.col(0) = e1;
    R0.col(1) = e2;
    R0.col(2) = e3;
    if (L <= 0.0)
        L = L0;
}

ElementBeam_Simo::GaussState ElementBeam_Simo::EvaluateGaussState(double coordinate) const
{
    auto node0 = m_pNode[0].lock();
    auto node1 = m_pNode[1].lock();
    if (!node0 || !node1 || L0 <= 1.0e-12)
        throw std::runtime_error("ElementBeam_Simo is not initialized");

    const Eigen::Vector3d x0(node0->m_X + node0->m_Displacement[0], node0->m_Y + node0->m_Displacement[1],
                             node0->m_Z + node0->m_Displacement[2]);
    const Eigen::Vector3d x1(node1->m_X + node1->m_Displacement[0], node1->m_Y + node1->m_Displacement[1],
                             node1->m_Z + node1->m_Displacement[2]);

    // m_Rg is the nodal spatial rotation relative to the initial section frame.
    const Eigen::Matrix3d rotation0 = node0->m_Rg * R0;
    const Eigen::Matrix3d rotation1 = node1->m_Rg * R0;
    const Eigen::Vector3d relativeRotation = Log(rotation1 * rotation0.transpose());
    const double N1 = 0.5 * (1.0 - coordinate);
    const double N2 = 0.5 * (1.0 + coordinate);

    GaussState state;
    state.chord = x1 - x0;
    state.rotation = Exp(N2 * relativeRotation) * rotation0;
    state.curvature = relativeRotation / L0;
    state.strain = state.chord / L0 - state.rotation * Eigen::Vector3d::UnitX();
    return state;
}

void ElementBeam_Simo::Get_ke(Eigen::MatrixXd& ke)
{
    Get_L0();

    double area, Iy, Iz, J, young, shear, density;
    GetSectionData(area, Iy, Iz, J, young, shear, density);

    Eigen::Matrix<double, 6, 6> constitutive = Eigen::Matrix<double, 6, 6>::Zero();
    constitutive(0, 0) = young * area;
    constitutive(1, 1) = shear * area;
    constitutive(2, 2) = shear * area;
    constitutive(3, 3) = shear * J;
    constitutive(4, 4) = young * Iz;
    constitutive(5, 5) = young * Iy;

    // As in the reference implementation, one reduced Gauss point is used for
    // the static Simo-Reissner strain terms to avoid shear locking.
    const GaussState state = EvaluateGaussState(0.0);
    const double N[2] = {0.5, 0.5};
    const double dN[2] = {-1.0 / L0, 1.0 / L0};
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> rotationBlock = Eigen::Matrix<double, 6, 6>::Zero();
    rotationBlock.block<3, 3>(0, 0) = state.rotation;
    rotationBlock.block<3, 3>(3, 3) = state.rotation;

    Eigen::Matrix<double, 6, 1> stressResultant = Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Matrix3d forceConstitutive = constitutive.block<3, 3>(0, 0);
    const Eigen::Matrix3d momentConstitutive = constitutive.block<3, 3>(3, 3);
    const Eigen::Vector3d force = state.rotation * forceConstitutive * state.rotation.transpose() * state.strain;
    const Eigen::Vector3d moment = state.rotation * momentConstitutive * state.rotation.transpose() * state.curvature;
    stressResultant.head<3>() = force;
    stressResultant.tail<3>() = moment;

    Eigen::Matrix<double, 6, 12> B = Eigen::Matrix<double, 6, 12>::Zero();
    Eigen::Matrix<double, 6, 6> nodalB[2];
    const Eigen::Vector3d tangent = state.chord / L0;
    for (int i = 0; i < 2; ++i)
    {
        nodalB[i].setZero();
        nodalB[i].block<3, 3>(0, 0) = dN[i] * identity;
        nodalB[i].block<3, 3>(3, 3) = dN[i] * identity;
        nodalB[i].block<3, 3>(0, 3) = N[i] * Skew(tangent);
        B.block<6, 6>(0, 6 * i) = nodalB[i];
    }

    m_inforce = Eigen::VectorXd::Zero(12);
    for (int i = 0; i < 2; ++i)
        m_inforce.segment<6>(6 * i) = L0 * nodalB[i].transpose() * stressResultant;

    const Eigen::Matrix<double, 6, 6> spatialConstitutive = rotationBlock * constitutive * rotationBlock.transpose();
    Eigen::Matrix<double, 12, 12> geometric = Eigen::Matrix<double, 12, 12>::Zero();
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            Eigen::Matrix<double, 6, 6> block = Eigen::Matrix<double, 6, 6>::Zero();
            block.block<3, 3>(0, 3) = Skew(-force) * dN[i] * N[j];
            block.block<3, 3>(3, 0) = Skew(force) * dN[j] * N[i];
            block.block<3, 3>(3, 3) = Skew(-moment) * dN[i] * N[j] +
                                      (force * tangent.transpose() - force.dot(tangent) * identity) * N[i] * N[j];
            geometric.block<6, 6>(6 * i, 6 * j) = block;
        }
    }

    ke = L0 * (B.transpose() * spatialConstitutive * B + geometric);
    m_ke = ke;
    L = state.chord.norm();
    m_Stress = area > 0.0 ? force.dot(state.rotation.col(0)) / area : 0.0;
}

void ElementBeam_Simo::Get_me_Consistent(Eigen::MatrixXd& me)
{
    Get_L0();
    double area, Iy, Iz, J, young, shear, density;
    GetSectionData(area, Iy, Iz, J, young, shear, density);

    Eigen::Matrix3d materialInertia = Eigen::Matrix3d::Zero();
    materialInertia(0, 0) = density * (Iy + Iz);
    materialInertia(1, 1) = density * Iz;
    materialInertia(2, 2) = density * Iy;

    me = Eigen::MatrixXd::Zero(12, 12);
    const double points[2] = {-1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)};
    for (double point : points)
    {
        const double N[2] = {0.5 * (1.0 - point), 0.5 * (1.0 + point)};
        const GaussState state = EvaluateGaussState(point);
        const Eigen::Matrix3d spatialInertia = state.rotation * materialInertia * state.rotation.transpose();
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                const double weight = 0.5 * L0 * N[i] * N[j];
                me.block<3, 3>(6 * i, 6 * j) += weight * density * area * Eigen::Matrix3d::Identity();
                me.block<3, 3>(6 * i + 3, 6 * j + 3) += weight * spatialInertia;
            }
        }
    }
    m_me = me;
}

void ElementBeam_Simo::Get_me_Lumped(Eigen::MatrixXd& me)
{
    Eigen::MatrixXd consistent;
    Get_me_Consistent(consistent);
    me = Eigen::MatrixXd::Zero(12, 12);
    for (int i = 0; i < 12; ++i)
        me(i, i) = consistent.row(i).sum();
}

void ElementBeam_Simo::Assemble(const std::vector<double>& damping, Eigen::MatrixXd& ce)
{
    if (damping.size() < 4)
        throw std::invalid_argument("damping vector must have four coefficients");

    Eigen::MatrixXd stiffness, mass;
    Get_ke(stiffness);
    Get_me_Consistent(mass);
    ce = Eigen::MatrixXd::Zero(12, 12);
    for (int i = 0; i < 12; ++i)
    {
        for (int j = 0; j < 12; ++j)
        {
            const bool rotational = (i % 6 >= 3) || (j % 6 >= 3);
            const double alpha = rotational ? damping[2] : damping[0];
            const double beta = rotational ? damping[3] : damping[1];
            ce(i, j) = alpha * mass(i, j) + beta * stiffness(i, j);
        }
    }
    m_ce = ce;
}
