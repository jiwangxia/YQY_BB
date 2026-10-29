#pragma once
#include "Base/Base.h"
#include <array>

/**
 * @brief 节点类 - 存储有限元节点信息
 */
class Node : public Base
{
public:
    struct TssbnRotationState
    {
        Eigen::Vector3d baseSpatialIncrement = Eigen::Vector3d::Zero();
        Eigen::Vector3d embeddedSpatialIncrement = Eigen::Vector3d::Zero();
        Eigen::Vector3d baseSpatialVelocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d embeddedSpatialVelocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d acceptedSpatialAcceleration = Eigen::Vector3d::Zero();
    };

    struct TssbnExplicitRotationState
    {
        Eigen::Vector3d spatialIncrement = Eigen::Vector3d::Zero();
        Eigen::Vector3d spatialVelocity = Eigen::Vector3d::Zero();
    };

    struct TssbnReferenceErrorState
    {
        Eigen::Vector3d stepStartRotationVector = Eigen::Vector3d::Zero();
        Eigen::Vector3d baseMaterialIncrement = Eigen::Vector3d::Zero();
        Eigen::Vector3d embeddedMaterialIncrement = Eigen::Vector3d::Zero();
    };

    /** @brief 普通隐式阶段的未投影材料状态及补偿；随完整节点事务复制。 */
    struct TssbnImplicitStageState
    {
        bool active = false;
        int index = 0;
        double diagonalTime = 0.0;
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
        Eigen::Vector3d rotationCompensation = Eigen::Vector3d::Zero();
        Eigen::Vector3d omegaCompensation = Eigen::Vector3d::Zero();
        Eigen::Vector3d alphaCompensation = Eigen::Vector3d::Zero();
    };

    Node();

    double m_X, m_Y, m_Z; ///< 节点坐标
    QVector<int> m_DOF;   ///< 节点自由度编号数组
    std::vector<double> m_Displacement;
    std::vector<double> m_Acceleration;
    std::vector<double> m_Velocity;
    std::vector<double> m_Force;         ///< 节点力(内力)
    std::vector<double> m_ReactionForce; ///< 节点力反力)

    void SetNumDOFs(int num_dofs);
    // 区域分析会删除与节点相连的部分单元；此时必须允许收缩到实际需要的自由度数，
    // 否则被删除梁单元留下的无刚度转角自由度会使静力矩阵奇异。
    void SetExactNumDOFs(int num_dofs);

    Eigen::Matrix3d m_Rg = Eigen::Matrix3d::Identity();

    std::vector<double> m_Displacement_n;                 ///< 上一时间步初的位移备份
    Eigen::Matrix3d m_Rg_n = Eigen::Matrix3d::Identity(); ///< 上一时间步初的旋转矩阵备份

    // Newmark 步初备份及有限转动状态。材料角速度、角加速度用于积分，
    // m_Velocity[3..5] / m_Acceleration[3..5] 保存供单元使用的空间量。
    std::vector<double> m_Velocity_n;
    std::vector<double> m_Acceleration_n;
    Eigen::Vector3d m_OmegaMaterial = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_AlphaMaterial = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_OmegaMaterial_n = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_AlphaMaterial_n = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_StepTranslation = Eigen::Vector3d::Zero(); ///< 独立累计小步平移，避免累计位移相减丢失修正
    double m_StepScalarRotation = 0.0;                           ///< 四自由度索的步内标量扭转
    Eigen::Vector3d m_StepRotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_TssbnOmegaMaterial1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_TssbnAlphaMaterial1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_TssbnOmegaMaterial2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_TssbnAlphaMaterial2 = Eigen::Vector3d::Zero();
    TssbnImplicitStageState m_TssbnImplicitStage;

    void BeginNewmarkStep(double dt, double beta, double gamma, const std::array<bool, 3>& translationActive,
                          const std::array<bool, 3>& rotationActive);
    void ApplyAdditiveIncrement(int component, double increment);
    void SetAdditiveDisplacement(int component, double value);
    void ApplyNewmarkCorrection(const Eigen::Vector3d& deltaTranslation, const Eigen::Vector3d& deltaRotation,
                                double a0, double a1);
    void RollbackNewmarkStep();
    void BeginTssbnImplicitStage(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                 double secondStageDiagonalFraction);
    void ApplyTssbnImplicitStageCorrection(const Eigen::Vector3d& deltaTranslation,
                                           const Eigen::Vector3d& deltaRotation);
    bool IsTssbnAcceptedRotationValid() const;
    bool NormalizeTssbnAcceptedRotation();
    void SetTssbnStageKinematics(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                 double secondStageDiagonalFraction, _OUT Eigen::Vector3d& spatialAngularVelocity,
                                 _OUT Eigen::Vector3d& spatialAngularAcceleration, bool useStoredIncrement = false);
    TssbnRotationState IntegrateTssbnRotation(double timeStep, double stageAccelerationExtrapolation,
                                              double baseFirstWeight, double embeddedFirstWeight,
                                              double embeddedSecondWeight, double embeddedLastWeight,
                                              double lastStageFirstCoefficient,
                                              double lastStageSecondCoefficient) const;
    TssbnExplicitRotationState IntegrateTssbnExplicitLastStage(double timeStep, double lastStageFirstCoefficient,
                                                               double lastStageSecondCoefficient) const;
    TssbnReferenceErrorState GetReferenceTssbnErrorState(double timeStep, double baseFirstWeight,
                                                         double embeddedFirstWeight, double embeddedSecondWeight,
                                                         double embeddedLastWeight, double lastStageFirstCoefficient,
                                                         double lastStageSecondCoefficient) const;
    Eigen::Vector3d IntegrateTssbnEmbeddedVelocity(double timeStep, double embeddedFirstWeight,
                                                   double embeddedSecondWeight, double embeddedLastWeight,
                                                   double lastStageFirstCoefficient, double lastStageSecondCoefficient,
                                                   const Eigen::Vector3d& explicitSpatialAcceleration) const;
};
