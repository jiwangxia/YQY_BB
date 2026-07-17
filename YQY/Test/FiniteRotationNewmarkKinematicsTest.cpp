#include "DataStructure/Node/Node.h"
#include "Utility/CR.h"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
    void CheckVector(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
        double tolerance, const char* message)
    {
        if ((actual - expected).norm() > tolerance)
            throw std::runtime_error(message);
    }

    void CheckMatrix(const Eigen::Matrix3d& actual, const Eigen::Matrix3d& expected,
        double tolerance, const char* message)
    {
        if ((actual - expected).norm() > tolerance)
            throw std::runtime_error(message);
    }
}

int main()
{
    constexpr double dt = 0.1;
    constexpr double beta = 0.25;
    constexpr double gamma = 0.5;
    const double a0 = 1.0 / (beta * dt * dt);
    const double a1 = gamma / (beta * dt);

    // 3 DOF truss node: ordinary translational Newmark update.
    Node trussNode;
    trussNode.m_Velocity = { 1.0, -2.0, 0.5 };
    trussNode.m_Acceleration = { 0.2, 0.4, -0.6 };
    const std::array<bool, 3> active{ true, true, true };
    const std::array<bool, 3> inactive{ false, false, false };
    trussNode.BeginNewmarkStep(dt, beta, gamma, active, inactive);
    CheckVector(Eigen::Vector3d(trussNode.m_Displacement[0], trussNode.m_Displacement[1],
        trussNode.m_Displacement[2]),
        dt * Eigen::Vector3d(1.0, -2.0, 0.5)
            + dt * dt * (0.5 - beta) * Eigen::Vector3d(0.2, 0.4, -0.6),
        1.0e-12, "3 DOF predictor");
    trussNode.ApplyNewmarkCorrection(Eigen::Vector3d(0.01, 0.02, -0.03),
        Eigen::Vector3d::Zero(), a0, a1);
    trussNode.RollbackNewmarkStep();
    CheckVector(Eigen::Vector3d(trussNode.m_Velocity[0], trussNode.m_Velocity[1],
        trussNode.m_Velocity[2]), Eigen::Vector3d(1.0, -2.0, 0.5),
        1.0e-12, "3 DOF rollback");

    // 4 DOF cable node: translations plus a scalar torsional DOF.
    Node cableNode;
    cableNode.SetNumDOFs(4);
    cableNode.m_Displacement[3] = 0.1;
    cableNode.m_Velocity[3] = 0.3;
    cableNode.m_Acceleration[3] = -0.2;
    const std::array<bool, 3> torsionActive{ true, false, false };
    cableNode.BeginNewmarkStep(dt, beta, gamma, active, torsionActive);
    const double twistPredictor = 0.1 + dt * 0.3
        + dt * dt * (0.5 - beta) * -0.2;
    if (std::abs(cableNode.m_Displacement[3] - twistPredictor) > 1.0e-12)
        throw std::runtime_error("4 DOF torsion predictor");
    const double deltaTwist = 0.015;
    cableNode.ApplyNewmarkCorrection(Eigen::Vector3d::Zero(),
        Eigen::Vector3d(deltaTwist, 0.0, 0.0), a0, a1);
    if (std::abs(cableNode.m_Velocity[3]
        - (0.3 + dt * (1.0 - gamma) * -0.2 + a1 * deltaTwist)) > 1.0e-12)
        throw std::runtime_error("4 DOF torsional velocity correction");
    if (std::abs(cableNode.m_Acceleration[3] - a0 * deltaTwist) > 1.0e-12)
        throw std::runtime_error("4 DOF torsional acceleration correction");
    cableNode.RollbackNewmarkStep();
    if (std::abs(cableNode.m_Displacement[3] - 0.1) > 1.0e-12
        || std::abs(cableNode.m_Velocity[3] - 0.3) > 1.0e-12
        || std::abs(cableNode.m_Acceleration[3] + 0.2) > 1.0e-12)
        throw std::runtime_error("4 DOF rollback");

    Node node;
    node.SetNumDOFs(6);
    Eigen::Matrix3d initialRotation;
    Utility::CR::Calculate_RotationMatrix(Eigen::Vector3d(0.2, -0.1, 0.15), initialRotation);
    node.m_Rg = initialRotation;

    const Eigen::Vector3d omegaMaterial_n(0.2, -0.1, 0.3);
    const Eigen::Vector3d alphaMaterial_n(0.05, 0.02, -0.04);
    const Eigen::Vector3d omegaSpatial_n = initialRotation * omegaMaterial_n;
    const Eigen::Vector3d alphaSpatial_n = initialRotation * alphaMaterial_n;
    for (int i = 0; i < 3; ++i)
    {
        node.m_Velocity[i + 3] = omegaSpatial_n(i);
        node.m_Acceleration[i + 3] = alphaSpatial_n(i);
    }

    node.BeginNewmarkStep(dt, beta, gamma, active, active);

    const Eigen::Vector3d Hpred = dt * omegaMaterial_n
        + dt * dt * (0.5 - beta) * alphaMaterial_n;
    const Eigen::Vector3d thetaPred = initialRotation * Hpred;
    Eigen::Matrix3d predictedRotation;
    Utility::CR::Update_NodalRotation(thetaPred, initialRotation, predictedRotation);
    CheckMatrix(node.m_Rg, predictedRotation, 1.0e-12, "rotation predictor");
    CheckVector(node.m_OmegaMaterial,
        omegaMaterial_n + dt * (1.0 - gamma) * alphaMaterial_n,
        1.0e-12, "material angular velocity predictor");
    CheckVector(node.m_AlphaMaterial, Eigen::Vector3d::Zero(),
        1.0e-12, "material angular acceleration predictor");

    const Eigen::Vector3d oldStepRotation = node.m_StepRotation;
    const Eigen::Vector3d deltaRotation(0.01, -0.02, 0.015);
    Eigen::Matrix3d correctedRotation;
    Utility::CR::Update_NodalRotation(deltaRotation, predictedRotation, correctedRotation);
    Eigen::Vector3d correctedStepRotation;
    Utility::CR::Extract_RotationVector(
        correctedRotation * initialRotation.transpose(), correctedStepRotation);
    const Eigen::Vector3d deltaH = initialRotation.transpose()
        * (correctedStepRotation - oldStepRotation);
    const Eigen::Vector3d omegaExpected = node.m_OmegaMaterial + a1 * deltaH;
    const Eigen::Vector3d alphaExpected = node.m_AlphaMaterial + a0 * deltaH;

    node.ApplyNewmarkCorrection(Eigen::Vector3d(0.001, -0.002, 0.003),
        deltaRotation, a0, a1);
    CheckMatrix(node.m_Rg, correctedRotation, 1.0e-12, "multiplicative correction");
    CheckVector(node.m_OmegaMaterial, omegaExpected, 1.0e-12,
        "material angular velocity correction");
    CheckVector(node.m_AlphaMaterial, alphaExpected, 1.0e-12,
        "material angular acceleration correction");
    CheckMatrix(node.m_Rg.transpose() * node.m_Rg, Eigen::Matrix3d::Identity(),
        1.0e-12, "rotation orthogonality");

    node.RollbackNewmarkStep();
    CheckMatrix(node.m_Rg, initialRotation, 1.0e-12, "rotation rollback");
    CheckVector(Eigen::Vector3d(node.m_Velocity[3], node.m_Velocity[4], node.m_Velocity[5]),
        omegaSpatial_n, 1.0e-12, "angular velocity rollback");
    CheckVector(Eigen::Vector3d(node.m_Acceleration[3], node.m_Acceleration[4], node.m_Acceleration[5]),
        alphaSpatial_n, 1.0e-12, "angular acceleration rollback");

    std::cout << "finite-rotation Newmark kinematics test passed\n";
    return 0;
}
