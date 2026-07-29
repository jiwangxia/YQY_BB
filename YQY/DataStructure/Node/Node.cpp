#include "Node.h"
#include "Utility/CR.h"
#include <algorithm>

Node::Node() : m_X(0.0), m_Y(0.0), m_Z(0.0)
{
    m_DOF.resize(3, -1);    //默认3个自由度，均未约束
    m_Displacement.resize(3, 0.0);
}

void Node::SetNumDOFs(int num_dofs)
{
    if (num_dofs > m_DOF.size()) 
    {
        m_DOF.resize(num_dofs, -1);
        m_Displacement.resize(num_dofs, 0.0);
        m_Displacement_n.resize(num_dofs, 0.0);
        m_Acceleration.resize(num_dofs, 0.0);
        m_Velocity.resize(num_dofs, 0.0);
        m_Force.resize(num_dofs, 0.0);
        m_ReactionForce.resize(num_dofs, 0.0);
    }
}

void Node::BeginNewmarkStep(double dt, double beta, double gamma,
    const std::array<bool, 3>& translationActive,
    const std::array<bool, 3>& rotationActive)
{
    const int numDOF = static_cast<int>(m_DOF.size());
    if (m_Displacement.size() < numDOF) m_Displacement.resize(numDOF, 0.0);
    if (m_Velocity.size() < numDOF) m_Velocity.resize(numDOF, 0.0);
    if (m_Acceleration.size() < numDOF) m_Acceleration.resize(numDOF, 0.0);

    m_Displacement_n = m_Displacement;
    m_Velocity_n = m_Velocity;
    m_Acceleration_n = m_Acceleration;
    m_Rg_n = m_Rg;

    for (int i = 0; i < std::min(3, numDOF); ++i)
    {
        if (!translationActive[i]) continue;
        m_Displacement[i] = m_Displacement_n[i]
            + dt * m_Velocity_n[i]
            + dt * dt * (0.5 - beta) * m_Acceleration_n[i];
        m_Velocity[i] = m_Velocity_n[i]
            + dt * (1.0 - gamma) * m_Acceleration_n[i];
        m_Acceleration[i] = 0.0;
    }

    // 四自由度索：第 4 项是绕索轴的标量扭转角，不使用 SO(3)。
    if (numDOF == 4)
    {
        if (rotationActive[0])
        {
            m_Displacement[3] = m_Displacement_n[3]
                + dt * m_Velocity_n[3]
                + dt * dt * (0.5 - beta) * m_Acceleration_n[3];
            m_Velocity[3] = m_Velocity_n[3]
                + dt * (1.0 - gamma) * m_Acceleration_n[3];
            m_Acceleration[3] = 0.0;
        }
        return;
    }

    if (numDOF < 6) return;

    Eigen::Vector3d omegaSpatial_n;
    Eigen::Vector3d alphaSpatial_n;
    for (int i = 0; i < 3; ++i)
    {
        omegaSpatial_n(i) = m_Velocity_n[i + 3];
        alphaSpatial_n(i) = m_Acceleration_n[i + 3];
    }
    m_OmegaMaterial_n = m_Rg_n.transpose() * omegaSpatial_n;
    m_AlphaMaterial_n = m_Rg_n.transpose() * alphaSpatial_n;

    // Newmark 预测在材料增量转角 H 上进行，再转为空间转角更新姿态。
    const Eigen::Vector3d Hpred = dt * m_OmegaMaterial_n
        + dt * dt * (0.5 - beta) * m_AlphaMaterial_n;
    Eigen::Vector3d thetaPred = m_Rg_n * Hpred;
    for (int i = 0; i < 3; ++i)
    {
        if (!rotationActive[i]) thetaPred(i) = 0.0;
    }

    Utility::CR::Update_NodalRotation(thetaPred, m_Rg_n, m_Rg);
    Utility::CR::Extract_RotationVector(m_Rg * m_Rg_n.transpose(), m_StepRotation);

    m_OmegaMaterial = m_OmegaMaterial_n
        + dt * (1.0 - gamma) * m_AlphaMaterial_n;
    m_AlphaMaterial.setZero();

    Eigen::Vector3d omegaSpatial = m_Rg * m_OmegaMaterial;
    Eigen::Vector3d alphaSpatial = m_Rg * m_AlphaMaterial;
    for (int i = 0; i < 3; ++i)
    {
        if (!rotationActive[i])
        {
            omegaSpatial(i) = 0.0;
            alphaSpatial(i) = 0.0;
        }
    }
    m_OmegaMaterial = m_Rg.transpose() * omegaSpatial;
    m_AlphaMaterial = m_Rg.transpose() * alphaSpatial;

    Eigen::Vector3d absoluteRotation;
    Utility::CR::Extract_RotationVector(m_Rg, absoluteRotation);
    for (int i = 0; i < 3; ++i)
    {
        m_Displacement[i + 3] = absoluteRotation(i);
        m_Velocity[i + 3] = omegaSpatial(i);
        m_Acceleration[i + 3] = alphaSpatial(i);
    }
}

void Node::ApplyNewmarkCorrection(const Eigen::Vector3d& deltaTranslation,
    const Eigen::Vector3d& deltaRotation, double a0, double a1)
{
    const int numDOF = static_cast<int>(m_DOF.size());
    for (int i = 0; i < std::min(3, numDOF); ++i)
    {
        m_Displacement[i] += deltaTranslation(i);
        m_Velocity[i] += a1 * deltaTranslation(i);
        m_Acceleration[i] += a0 * deltaTranslation(i);
    }

    if (numDOF == 4)
    {
        m_Displacement[3] += deltaRotation(0);
        m_Velocity[3] += a1 * deltaRotation(0);
        m_Acceleration[3] += a0 * deltaRotation(0);
        return;
    }

    if (numDOF < 6) return;

    Eigen::Matrix3d updatedRotation;
    Utility::CR::Update_NodalRotation(deltaRotation, m_Rg, updatedRotation);
    m_Rg = updatedRotation;
    Utility::CR::Extract_RotationVector(m_Rg * m_Rg_n.transpose(), m_StepRotation);

    // Le 等（2012）式 (43)~(46)：Newton 求得的是空间自旋修正，
    // 需先经 Ts^{-1} 转为空间增量转角，再转到步初材料坐标系。
    Eigen::Matrix3d spatialSpinInverse;
    Utility::CR::Calculate_Ts_Inv(m_StepRotation, spatialSpinInverse);
    const Eigen::Vector3d deltaH = m_Rg_n.transpose()
        * spatialSpinInverse * deltaRotation;
    m_OmegaMaterial += a1 * deltaH;
    m_AlphaMaterial += a0 * deltaH;

    const Eigen::Vector3d omegaSpatial = m_Rg * m_OmegaMaterial;
    const Eigen::Vector3d alphaSpatial = m_Rg * m_AlphaMaterial;
    Eigen::Vector3d absoluteRotation;
    Utility::CR::Extract_RotationVector(m_Rg, absoluteRotation);
    for (int i = 0; i < 3; ++i)
    {
        m_Displacement[i + 3] = absoluteRotation(i);
        m_Velocity[i + 3] = omegaSpatial(i);
        m_Acceleration[i + 3] = alphaSpatial(i);
    }
}

void Node::RollbackNewmarkStep()
{
    if (!m_Displacement_n.empty()) m_Displacement = m_Displacement_n;
    if (!m_Velocity_n.empty()) m_Velocity = m_Velocity_n;
    if (!m_Acceleration_n.empty()) m_Acceleration = m_Acceleration_n;
    m_Rg = m_Rg_n;
    m_OmegaMaterial = m_OmegaMaterial_n;
    m_AlphaMaterial = m_AlphaMaterial_n;
    m_StepRotation.setZero();
}
