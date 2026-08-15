#include "RigidBodyInertia.h"
#include "Utility/CR.h"

#include <Eigen/Eigenvalues>
#include <cmath>

static Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& vector)
{
    Eigen::Matrix3d result;
    result << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(), -vector.y(), vector.x(), 0.0;
    return result;
}

bool RigidBodyInertia::IsValid() const
{
    const auto node = m_pNode.lock();
    if (!node || node->m_DOF.size() < 6 || !std::isfinite(m_Mass) || m_Mass <= 0.0 || !m_RotaryInertia.allFinite() ||
        !m_RotaryInertia.isApprox(m_RotaryInertia.transpose(), 1.0e-10))
    {
        return false;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(m_RotaryInertia);
    return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() > 0.0;
}

void RigidBodyInertia::GetDOFs(_OUT std::vector<int>& dofs) const
{
    dofs.clear();
    const auto node = m_pNode.lock();
    if (!node || node->m_DOF.size() < 6)
        return;
    dofs.assign(node->m_DOF.cbegin(), node->m_DOF.cbegin() + 6);
}

void RigidBodyInertia::GetDynamicContributions(_OUT Eigen::MatrixXd& massMatrix, _OUT Eigen::VectorXd& inertiaForce,
                                               _OUT Eigen::MatrixXd& gyroscopicMatrix,
                                               _OUT Eigen::MatrixXd& configurationTangent) const
{
    massMatrix = Eigen::MatrixXd::Zero(6, 6);
    inertiaForce = Eigen::VectorXd::Zero(6);
    gyroscopicMatrix = Eigen::MatrixXd::Zero(6, 6);
    configurationTangent = Eigen::MatrixXd::Zero(6, 6);

    const auto node = m_pNode.lock();
    if (!node || node->m_DOF.size() < 6)
        return;

    massMatrix.block<3, 3>(0, 0) = m_Mass * Eigen::Matrix3d::Identity();

    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularVelocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d angularAcceleration = Eigen::Vector3d::Zero();
    for (int component = 0; component < 3; ++component)
    {
        if (component < static_cast<int>(node->m_Acceleration.size()))
            acceleration[component] = node->m_Acceleration[component];
        if (component + 3 < static_cast<int>(node->m_Velocity.size()))
            angularVelocity[component] = node->m_Velocity[component + 3];
        if (component + 3 < static_cast<int>(node->m_Acceleration.size()))
            angularAcceleration[component] = node->m_Acceleration[component + 3];
    }

    const Eigen::Vector3d materialAngularVelocity = node->m_Rg.transpose() * angularVelocity;
    const Eigen::Vector3d materialAngularAcceleration = node->m_Rg.transpose() * angularAcceleration;
    const Eigen::Vector3d materialAngularMomentum = m_RotaryInertia * materialAngularVelocity;
    const Eigen::Vector3d materialDynamicMoment =
        m_RotaryInertia * materialAngularAcceleration + materialAngularVelocity.cross(materialAngularMomentum);
    inertiaForce.head<3>() = m_Mass * acceleration;
    inertiaForce.tail<3>() = node->m_Rg * materialDynamicMoment;

    Eigen::Matrix3d inverseSpatialSpin;
    Utility::CR::Calculate_Ts_Inv(node->m_StepRotation, inverseSpatialSpin);
    const Eigen::Matrix3d rotationalMapping = node->m_Rg_n.transpose() * inverseSpatialSpin;
    massMatrix.block<3, 3>(3, 3) = node->m_Rg * m_RotaryInertia * rotationalMapping;

    const Eigen::Matrix3d angularVelocitySkew = CrossProductMatrix(materialAngularVelocity);
    gyroscopicMatrix.block<3, 3>(3, 3) =
        node->m_Rg * (angularVelocitySkew * m_RotaryInertia - m_RotaryInertia * angularVelocitySkew) *
        rotationalMapping;
    configurationTangent.block<3, 3>(3, 3) = -node->m_Rg * CrossProductMatrix(materialDynamicMoment);
}
