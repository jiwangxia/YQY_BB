#include "Node.h"
#include "Utility/CR.h"
#include "Solver/Dynamic/TimeStepping/TssbnRotationIncrement.h"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

static void AccumulateTssbnStageVector(const Eigen::Vector3d& increment, _OUT Eigen::Vector3d& value,
                                       _OUT Eigen::Vector3d& compensation)
{
    const Eigen::Vector3d corrected = increment - compensation;
    const Eigen::Vector3d sum = value + corrected;
    compensation = (sum - value) - corrected;
    value = sum;
}

Node::Node()
    : m_X(0.0)
    , m_Y(0.0)
    , m_Z(0.0)
{
    m_DOF.resize(3, -1); //默认3个自由度，均未约束
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

void Node::SetExactNumDOFs(int num_dofs)
{
    const int count = std::max(3, num_dofs);
    m_DOF.resize(count, -1);
    m_Displacement.resize(count, 0.0);
    m_Displacement_n.resize(count, 0.0);
    m_Acceleration.resize(count, 0.0);
    m_Acceleration_n.resize(count, 0.0);
    m_Velocity.resize(count, 0.0);
    m_Velocity_n.resize(count, 0.0);
    m_Force.resize(count, 0.0);
    m_ReactionForce.resize(count, 0.0);
}

void Node::BeginNewmarkStep(double dt, double beta, double gamma, const std::array<bool, 3>& translationActive,
                            const std::array<bool, 3>& rotationActive)
{
    m_TssbnImplicitStage = TssbnImplicitStageState{};
    const int numDOF = static_cast<int>(m_DOF.size());
    if (m_Displacement.size() < numDOF)
        m_Displacement.resize(numDOF, 0.0);
    if (m_Velocity.size() < numDOF)
        m_Velocity.resize(numDOF, 0.0);
    if (m_Acceleration.size() < numDOF)
        m_Acceleration.resize(numDOF, 0.0);

    m_Displacement_n = m_Displacement;
    m_Velocity_n = m_Velocity;
    m_Acceleration_n = m_Acceleration;
    m_Rg_n = m_Rg;
    m_StepTranslation.setZero();
    m_StepScalarRotation = 0.0;

    for (int i = 0; i < std::min(3, numDOF); ++i)
    {
        if (!translationActive[i])
            continue;
        m_StepTranslation[i] = dt * m_Velocity_n[i] + dt * dt * (0.5 - beta) * m_Acceleration_n[i];
        m_Displacement[i] = m_Displacement_n[i] + m_StepTranslation[i];
        m_Velocity[i] = m_Velocity_n[i] + dt * (1.0 - gamma) * m_Acceleration_n[i];
        m_Acceleration[i] = 0.0;
    }

    // 四自由度索：第 4 项是绕索轴的标量扭转角，不使用 SO(3)。
    if (numDOF == 4)
    {
        if (rotationActive[0])
        {
            m_StepScalarRotation = dt * m_Velocity_n[3] + dt * dt * (0.5 - beta) * m_Acceleration_n[3];
            m_Displacement[3] = m_Displacement_n[3] + m_StepScalarRotation;
            m_Velocity[3] = m_Velocity_n[3] + dt * (1.0 - gamma) * m_Acceleration_n[3];
            m_Acceleration[3] = 0.0;
        }
        return;
    }

    if (numDOF < 6)
        return;

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
    const Eigen::Vector3d Hpred = dt * m_OmegaMaterial_n + dt * dt * (0.5 - beta) * m_AlphaMaterial_n;
    Eigen::Vector3d thetaPred = m_Rg_n * Hpred;
    for (int i = 0; i < 3; ++i)
    {
        if (!rotationActive[i])
            thetaPred(i) = 0.0;
    }

    Utility::CR::Update_NodalRotation(thetaPred, m_Rg_n, m_Rg);
    m_StepRotation = thetaPred;
    if (thetaPred.norm() >= std::acos(-1.0))
    {
        Eigen::Matrix3d predictedStepRotation;
        Utility::CR::Calculate_RotationMatrix(thetaPred, predictedStepRotation);
        Utility::CR::Extract_RotationVector(predictedStepRotation, m_StepRotation);
        Utility::CR::Calculate_RotationMatrix(m_StepRotation, predictedStepRotation);
        m_Rg = predictedStepRotation * m_Rg_n;
    }

    m_OmegaMaterial = m_OmegaMaterial_n + dt * (1.0 - gamma) * m_AlphaMaterial_n;
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

void Node::ApplyAdditiveIncrement(int component, double increment)
{
    double* stepIncrement = component < 3 ? &m_StepTranslation[component]
                                        : (m_DOF.size() == 4 && component == 3 ? &m_StepScalarRotation : nullptr);
    if (!stepIncrement)
    {
        m_Displacement[component] += increment;
        return;
    }
    const double previous = component < m_Displacement_n.size() ? m_Displacement_n[component] : 0.0;
    // 兼容直接设置初态的既有入口；正常试算的构型等于步初加小增量，不从累计位移重新恢复小增量。
    if (m_Displacement[component] != previous + *stepIncrement)
        *stepIncrement = m_Displacement[component] - previous;
    *stepIncrement += increment;
    m_Displacement[component] = previous + *stepIncrement;
}

void Node::SetAdditiveDisplacement(int component, double value)
{
    m_Displacement[component] = value;
    const double previous = component < m_Displacement_n.size() ? m_Displacement_n[component] : 0.0;
    if (component < 3)
        m_StepTranslation[component] = value - previous;
    else if (m_DOF.size() == 4 && component == 3)
        m_StepScalarRotation = value - previous;
}

void Node::ApplyNewmarkCorrection(const Eigen::Vector3d& deltaTranslation, const Eigen::Vector3d& deltaRotation,
                                  double a0, double a1)
{
    const int numDOF = static_cast<int>(m_DOF.size());
    for (int i = 0; i < std::min(3, numDOF); ++i)
    {
        ApplyAdditiveIncrement(i, deltaTranslation(i));
        m_Velocity[i] += a1 * deltaTranslation(i);
        m_Acceleration[i] += a0 * deltaTranslation(i);
    }

    if (numDOF == 4)
    {
        ApplyAdditiveIncrement(3, deltaRotation(0));
        m_Velocity[3] += a1 * deltaRotation(0);
        m_Acceleration[3] += a0 * deltaRotation(0);
        return;
    }

    if (numDOF < 6)
        return;

    Eigen::Matrix3d stepRotationMatrix;
    Utility::CR::Calculate_RotationMatrix(m_StepRotation, stepRotationMatrix);
    const Eigen::Matrix3d cachedRotation = stepRotationMatrix * m_Rg_n;
    const bool externallyChangedRotation = (cachedRotation.array() != m_Rg.array()).any();
    if (externallyChangedRotation)
    {
        // 兼容直接设置姿态的调用；正常 Newmark 试算从独立步内转角恢复相对姿态。
        Utility::CR::Extract_RotationVector(m_Rg * m_Rg_n.transpose(), m_StepRotation);
        Utility::CR::Calculate_RotationMatrix(m_StepRotation, stepRotationMatrix);
    }
    if (deltaRotation.isZero(0.0) && !externallyChangedRotation)
        return;
    const Eigen::Vector3d oldStepRotation = m_StepRotation;
    Eigen::Vector3d stepRotationChange;
    const Eigen::Vector3d additiveRotation = oldStepRotation + deltaRotation;
    if (oldStepRotation.cross(deltaRotation).isZero(0.0) && additiveRotation.norm() < std::acos(-1.0))
    {
        // 共轴转动的指数可交换，完整有限修正严格退化为标量 Newmark。
        m_StepRotation = additiveRotation;
        stepRotationChange = deltaRotation;
    }
    else
    {
        Eigen::Matrix3d correctionRotation;
        Utility::CR::Calculate_RotationMatrix(deltaRotation, correctionRotation);
        Utility::CR::Extract_RotationVector(correctionRotation * stepRotationMatrix, m_StepRotation);
        stepRotationChange = m_StepRotation - oldStepRotation;
    }
    Utility::CR::Calculate_RotationMatrix(m_StepRotation, stepRotationMatrix);
    m_Rg = stepRotationMatrix * m_Rg_n;

    // 材料增量 Newmark 的状态更新使用完整有限转角差；
    // Ts^{-1} 只用于 Newton 切线的微分映射。
    const Eigen::Vector3d deltaH = m_Rg_n.transpose() * stepRotationChange;
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
    // 新阶段必须显式 Begin；C2 仍需已完成 C1 的独立阶段历史，不能在此清掉它。
    m_TssbnImplicitStage = TssbnImplicitStageState{};
    if (!m_Displacement_n.empty())
        m_Displacement = m_Displacement_n;
    if (!m_Velocity_n.empty())
        m_Velocity = m_Velocity_n;
    if (!m_Acceleration_n.empty())
        m_Acceleration = m_Acceleration_n;
    m_Rg = m_Rg_n;
    m_OmegaMaterial = m_OmegaMaterial_n;
    m_AlphaMaterial = m_AlphaMaterial_n;
    m_StepTranslation.setZero();
    m_StepScalarRotation = 0.0;
    m_StepRotation.setZero();
}

void Node::BeginTssbnImplicitStage(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                   double secondStageDiagonalFraction)
{
    m_TssbnImplicitStage = TssbnImplicitStageState{};
    if (m_DOF.size() < 6)
        return;
    const double diagonalTime =
        timeStep * (stageIndex == 1 ? firstStageTime : secondStageTime * secondStageDiagonalFraction);
    if ((stageIndex != 1 && stageIndex != 2) || !std::isfinite(diagonalTime) || diagonalTime <= 0.0)
        throw std::invalid_argument("普通 TSSBN 隐式阶段初始化参数无效");
    // 当前实际预测构型只按旧关系锚定一次；承认首轮除法与相消的舍入，不声称消除全部浮点下限。
    Eigen::Vector3d omega;
    Eigen::Vector3d alpha;
    SetTssbnStageKinematics(stageIndex, timeStep, firstStageTime, secondStageTime, secondStageDiagonalFraction, omega,
                            alpha, true);
    m_TssbnImplicitStage.active = true;
    m_TssbnImplicitStage.index = stageIndex;
    m_TssbnImplicitStage.diagonalTime = diagonalTime;
    m_TssbnImplicitStage.omega = m_OmegaMaterial;
    m_TssbnImplicitStage.alpha = m_AlphaMaterial;
}

void Node::ApplyTssbnImplicitStageCorrection(const Eigen::Vector3d& deltaTranslation,
                                             const Eigen::Vector3d& deltaRotation)
{
    const int numDofs = static_cast<int>(m_DOF.size());
    for (int component = 0; component < std::min(3, numDofs); ++component)
        ApplyAdditiveIncrement(component, deltaTranslation[component]);
    if (numDofs == 4)
        ApplyAdditiveIncrement(3, deltaRotation[0]);
    if (numDofs < 6)
        return;
    if (!m_TssbnImplicitStage.active)
        throw std::logic_error("普通 TSSBN 完整修正缺少显式阶段初始化");
    if (deltaRotation.isZero(0.0))
        return;

    Eigen::Vector3d updatedPhi;
    Eigen::Vector3d deltaPhi;
    if (!SolverNameSpace::ComputeTssbnRotationIncrement(m_StepRotation, deltaRotation, updatedPhi, deltaPhi))
        throw std::runtime_error("普通 TSSBN 完整转动修正超出可用主值图");
    if (deltaRotation.norm() <= 1.0e-4)
    {
        // 微小完整转角差单独累积；补偿不进入仅保存高位的几何姿态，阶段关系缺口须另行核验。
        AccumulateTssbnStageVector(deltaPhi, m_StepRotation, m_TssbnImplicitStage.rotationCompensation);
    }
    else
    {
        m_StepRotation = updatedPhi;
        m_TssbnImplicitStage.rotationCompensation.setZero();
    }
    const Eigen::Vector3d deltaMaterial = m_Rg_n.transpose() * deltaPhi;
    const double inverseTime = 1.0 / m_TssbnImplicitStage.diagonalTime;
    AccumulateTssbnStageVector(inverseTime * deltaMaterial, m_TssbnImplicitStage.omega,
                               m_TssbnImplicitStage.omegaCompensation);
    AccumulateTssbnStageVector((inverseTime * inverseTime) * deltaMaterial, m_TssbnImplicitStage.alpha,
                               m_TssbnImplicitStage.alphaCompensation);
    Eigen::Matrix3d relative;
    Utility::CR::Calculate_RotationMatrix(m_StepRotation, relative);
    m_Rg = relative * m_Rg_n;
    m_OmegaMaterial = m_TssbnImplicitStage.omega;
    m_AlphaMaterial = m_TssbnImplicitStage.alpha;
    const Eigen::Vector3d omega = m_Rg * m_OmegaMaterial;
    const Eigen::Vector3d alpha = m_Rg * m_AlphaMaterial;
    Eigen::Vector3d absolute;
    Utility::CR::Extract_RotationVector(m_Rg, absolute);
    for (int component = 0; component < 3; ++component)
    {
        m_Displacement[component + 3] = absolute[component];
        m_Velocity[component + 3] = omega[component];
        m_Acceleration[component + 3] = alpha[component];
    }
}

bool Node::IsTssbnAcceptedRotationValid() const
{
    if (m_DOF.size() < 6)
        return true;
    if (!m_Rg.allFinite() || !m_Rg_n.allFinite() || m_Displacement.size() < 6 || m_Velocity.size() < 6 ||
        m_Acceleration.size() < 6)
        return false;
    for (int component = 0; component < 6; ++component)
        if (!std::isfinite(m_Displacement[component]) || !std::isfinite(m_Velocity[component]) ||
            !std::isfinite(m_Acceleration[component]))
            return false;
    // 只接受 SO(3) 附近的数值漂移；该域检查不代替用户精度或独立正交性验收门。
    constexpr double domainTolerance = 1.0e-6;
    for (const Eigen::Matrix3d* rotation : {&m_Rg, &m_Rg_n})
    {
        const double determinant = rotation->determinant();
        if (!std::isfinite(determinant) || determinant <= 0.0 || std::abs(determinant - 1.0) > domainTolerance ||
            ((*rotation) * rotation->transpose() - Eigen::Matrix3d::Identity()).norm() > domainTolerance)
            return false;
    }
    return true;
}

bool Node::NormalizeTssbnAcceptedRotation()
{
    if (!IsTssbnAcceptedRotationValid())
        return false;
    if (m_DOF.size() < 6)
        return true;
    const Eigen::JacobiSVD<Eigen::Matrix3d> decomposition(m_Rg, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (decomposition.info() != Eigen::Success || !decomposition.singularValues().allFinite())
        return false;
    const Eigen::Matrix3d rotation = decomposition.matrixU() * decomposition.matrixV().transpose();
    if (!rotation.allFinite() || rotation.determinant() <= 0.0)
        return false;
    const Eigen::Vector3d velocity(m_Velocity[3], m_Velocity[4], m_Velocity[5]);
    const Eigen::Vector3d acceleration(m_Acceleration[3], m_Acceleration[4], m_Acceleration[5]);
    const Eigen::Vector3d materialOmega = rotation.transpose() * velocity;
    const Eigen::Vector3d materialAlpha = rotation.transpose() * acceleration;
    Eigen::Vector3d absoluteRotation;
    Eigen::Vector3d stepRotation;
    Utility::CR::Extract_RotationVector(rotation, absoluteRotation);
    Utility::CR::Extract_RotationVector(rotation * m_Rg_n.transpose(), stepRotation);
    if (!materialOmega.allFinite() || !materialAlpha.allFinite() || !absoluteRotation.allFinite() ||
        !stepRotation.allFinite())
        return false;
    // 先完成全部局部计算再提交；空间速度/加速度和所有步初备份均逐位保留。
    m_Rg = rotation;
    m_OmegaMaterial = materialOmega;
    m_AlphaMaterial = materialAlpha;
    m_StepRotation = stepRotation;
    for (int component = 0; component < 3; ++component)
        m_Displacement[component + 3] = absoluteRotation[component];
    m_TssbnImplicitStage = TssbnImplicitStageState{};
    return true;
}

void Node::SetTssbnStageKinematics(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                   double secondStageDiagonalFraction, _OUT Eigen::Vector3d& spatialAngularVelocity,
                                   _OUT Eigen::Vector3d& spatialAngularAcceleration, bool useStoredIncrement)
{
    // 普通路径由完整左修正维护独立小转角；参考默认仍从绝对姿态重建，保持其历史运算顺序。
    if (!useStoredIncrement)
        Utility::CR::Extract_RotationVector(m_Rg * m_Rg_n.transpose(), m_StepRotation);
    const Eigen::Vector3d materialRotationIncrement = m_Rg_n.transpose() * m_StepRotation;

    if (useStoredIncrement && m_TssbnImplicitStage.active)
    {
        if (m_TssbnImplicitStage.index != stageIndex)
            throw std::logic_error("普通 TSSBN 阶段切换缺少显式初始化");
        m_OmegaMaterial = m_TssbnImplicitStage.omega;
        m_AlphaMaterial = m_TssbnImplicitStage.alpha;
        if (stageIndex == 1)
        {
            m_TssbnOmegaMaterial1 = m_OmegaMaterial;
            m_TssbnAlphaMaterial1 = m_AlphaMaterial;
        }
        else
        {
            m_TssbnOmegaMaterial2 = m_OmegaMaterial;
            m_TssbnAlphaMaterial2 = m_AlphaMaterial;
        }
    }
    else if (stageIndex == 1)
    {
        const double diagonalTime = firstStageTime * timeStep;
        m_TssbnOmegaMaterial1 = materialRotationIncrement / diagonalTime;
        m_TssbnAlphaMaterial1 = (m_TssbnOmegaMaterial1 - m_OmegaMaterial_n) / diagonalTime;
        m_OmegaMaterial = m_TssbnOmegaMaterial1;
        m_AlphaMaterial = m_TssbnAlphaMaterial1;
    }
    else
    {
        const double previousCoefficient = secondStageTime * (1.0 - secondStageDiagonalFraction);
        const double diagonalCoefficient = secondStageTime * secondStageDiagonalFraction;
        const double diagonalTime = diagonalCoefficient * timeStep;
        m_TssbnOmegaMaterial2 =
            (materialRotationIncrement - timeStep * previousCoefficient * m_TssbnOmegaMaterial1) / diagonalTime;
        m_TssbnAlphaMaterial2 =
            (m_TssbnOmegaMaterial2 - m_OmegaMaterial_n - timeStep * previousCoefficient * m_TssbnAlphaMaterial1) /
            diagonalTime;
        m_OmegaMaterial = m_TssbnOmegaMaterial2;
        m_AlphaMaterial = m_TssbnAlphaMaterial2;
    }

    spatialAngularVelocity = m_Rg * m_OmegaMaterial;
    spatialAngularAcceleration = m_Rg * m_AlphaMaterial;
    for (int component = 0; component < 3; ++component)
    {
        m_Velocity[component + 3] = spatialAngularVelocity[component];
        m_Acceleration[component + 3] = spatialAngularAcceleration[component];
    }
}

Node::TssbnRotationState Node::IntegrateTssbnRotation(double timeStep, double stageAccelerationExtrapolation,
                                                      double baseFirstWeight, double embeddedFirstWeight,
                                                      double embeddedSecondWeight, double embeddedLastWeight,
                                                      double lastStageFirstCoefficient,
                                                      double lastStageSecondCoefficient) const
{
    const Eigen::Vector3d lastOmegaMaterial =
        m_OmegaMaterial_n + timeStep * (lastStageFirstCoefficient * m_TssbnAlphaMaterial1 +
                                        lastStageSecondCoefficient * m_TssbnAlphaMaterial2);
    const Eigen::Vector3d lastAlphaMaterial =
        m_TssbnAlphaMaterial2 + stageAccelerationExtrapolation * (m_TssbnAlphaMaterial2 - m_TssbnAlphaMaterial1);

    const Eigen::Vector3d baseMaterialIncrement =
        timeStep * (baseFirstWeight * m_TssbnOmegaMaterial1 + (1.0 - baseFirstWeight) * m_TssbnOmegaMaterial2);
    const Eigen::Vector3d embeddedMaterialIncrement =
        timeStep * (embeddedFirstWeight * m_TssbnOmegaMaterial1 + embeddedSecondWeight * m_TssbnOmegaMaterial2 +
                    embeddedLastWeight * lastOmegaMaterial);

    TssbnRotationState result;
    result.baseSpatialIncrement = m_Rg_n * baseMaterialIncrement;
    result.embeddedSpatialIncrement = m_Rg_n * embeddedMaterialIncrement;

    Eigen::Matrix3d baseRotation;
    Eigen::Matrix3d embeddedRotation;
    Utility::CR::Update_NodalRotation(result.baseSpatialIncrement, m_Rg_n, baseRotation);
    Utility::CR::Update_NodalRotation(result.embeddedSpatialIncrement, m_Rg_n, embeddedRotation);

    const Eigen::Vector3d baseOmegaMaterial =
        m_OmegaMaterial_n +
        timeStep * (baseFirstWeight * m_TssbnAlphaMaterial1 + (1.0 - baseFirstWeight) * m_TssbnAlphaMaterial2);
    const Eigen::Vector3d embeddedOmegaMaterial =
        m_OmegaMaterial_n +
        timeStep * (embeddedFirstWeight * m_TssbnAlphaMaterial1 + embeddedSecondWeight * m_TssbnAlphaMaterial2 +
                    embeddedLastWeight * lastAlphaMaterial);
    result.baseSpatialVelocity = baseRotation * baseOmegaMaterial;
    result.embeddedSpatialVelocity = embeddedRotation * embeddedOmegaMaterial;
    result.acceptedSpatialAcceleration = baseRotation * lastAlphaMaterial;
    return result;
}

Node::TssbnExplicitRotationState Node::IntegrateTssbnExplicitLastStage(double timeStep,
                                                                       double lastStageFirstCoefficient,
                                                                       double lastStageSecondCoefficient) const
{
    const Eigen::Vector3d materialIncrement =
        timeStep * (lastStageFirstCoefficient * m_TssbnOmegaMaterial1 +
                    lastStageSecondCoefficient * m_TssbnOmegaMaterial2);
    const Eigen::Vector3d materialVelocity =
        m_OmegaMaterial_n + timeStep * (lastStageFirstCoefficient * m_TssbnAlphaMaterial1 +
                                        lastStageSecondCoefficient * m_TssbnAlphaMaterial2);

    TssbnExplicitRotationState result;
    result.spatialIncrement = m_Rg_n * materialIncrement;
    Eigen::Matrix3d explicitRotation;
    Utility::CR::Update_NodalRotation(result.spatialIncrement, m_Rg_n, explicitRotation);
    result.spatialVelocity = explicitRotation * materialVelocity;
    return result;
}

Node::TssbnReferenceErrorState Node::GetReferenceTssbnErrorState(
    double timeStep, double baseFirstWeight, double embeddedFirstWeight, double embeddedSecondWeight,
    double embeddedLastWeight, double lastStageFirstCoefficient, double lastStageSecondCoefficient) const
{
    TssbnReferenceErrorState result;
    Eigen::Quaterniond stepStartQuaternion(m_Rg_n);
    stepStartQuaternion.normalize();
    // 矩阵不保存四元数历史符号；统一取 w>=0 的主转角，再沿用参考的 asin 公式。
    if (stepStartQuaternion.w() < 0.0)
        stepStartQuaternion.coeffs() *= -1.0;
    const double vectorNorm = stepStartQuaternion.vec().norm();
    if (vectorNorm > 1.0e-12)
        result.stepStartRotationVector =
            2.0 * std::asin(std::min(1.0, vectorNorm)) * stepStartQuaternion.vec() / vectorNorm;

    const Eigen::Vector3d lastOmegaMaterial =
        m_OmegaMaterial_n + timeStep * (lastStageFirstCoefficient * m_TssbnAlphaMaterial1 +
                                        lastStageSecondCoefficient * m_TssbnAlphaMaterial2);
    // LTE 按参考逐项乘加；接受态仍使用原有分组更新。
    result.baseMaterialIncrement = (timeStep * baseFirstWeight) * m_TssbnOmegaMaterial1;
    result.baseMaterialIncrement += (timeStep * (1.0 - baseFirstWeight)) * m_TssbnOmegaMaterial2;
    result.embeddedMaterialIncrement = (timeStep * embeddedFirstWeight) * m_TssbnOmegaMaterial1;
    result.embeddedMaterialIncrement += (timeStep * embeddedSecondWeight) * m_TssbnOmegaMaterial2;
    result.embeddedMaterialIncrement += (timeStep * embeddedLastWeight) * lastOmegaMaterial;
    return result;
}

Eigen::Vector3d Node::IntegrateTssbnEmbeddedVelocity(double timeStep, double embeddedFirstWeight,
                                                      double embeddedSecondWeight, double embeddedLastWeight,
                                                      double lastStageFirstCoefficient,
                                                      double lastStageSecondCoefficient,
    const Eigen::Vector3d& explicitSpatialAcceleration) const
{
    const TssbnExplicitRotationState explicitStage =
        IntegrateTssbnExplicitLastStage(timeStep, lastStageFirstCoefficient, lastStageSecondCoefficient);
    Eigen::Matrix3d explicitRotation;
    Utility::CR::Update_NodalRotation(explicitStage.spatialIncrement, m_Rg_n, explicitRotation);
    const Eigen::Vector3d explicitMaterialAcceleration = explicitRotation.transpose() * explicitSpatialAcceleration;
    const Eigen::Vector3d explicitMaterialVelocity =
        m_OmegaMaterial_n + timeStep * (lastStageFirstCoefficient * m_TssbnAlphaMaterial1 +
                                        lastStageSecondCoefficient * m_TssbnAlphaMaterial2);

    const Eigen::Vector3d embeddedMaterialIncrement =
        timeStep * (embeddedFirstWeight * m_TssbnOmegaMaterial1 +
                    embeddedSecondWeight * m_TssbnOmegaMaterial2 +
                    embeddedLastWeight * explicitMaterialVelocity);
    Eigen::Matrix3d embeddedRotation;
    Utility::CR::Update_NodalRotation(m_Rg_n * embeddedMaterialIncrement, m_Rg_n, embeddedRotation);
    const Eigen::Vector3d embeddedMaterialVelocity =
        m_OmegaMaterial_n + timeStep * (embeddedFirstWeight * m_TssbnAlphaMaterial1 +
                                        embeddedSecondWeight * m_TssbnAlphaMaterial2 +
                                        embeddedLastWeight * explicitMaterialAcceleration);
    return embeddedRotation * embeddedMaterialVelocity;
}
