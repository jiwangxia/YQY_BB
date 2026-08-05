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
        Eigen::Vector3d acceptedSpatialAcceleration =
            Eigen::Vector3d::Zero();
    };

    Node();

    double m_X, m_Y, m_Z;  ///< 节点坐标
    QVector<int> m_DOF;    ///< 节点自由度编号数组
    std::vector<double>  m_Displacement;
    std::vector<double>  m_Acceleration;
    std::vector<double>  m_Velocity;
    std::vector<double>  m_Force;  ///< 节点力(内力)
    std::vector<double>  m_ReactionForce;  ///< 节点力反力)

    void SetNumDOFs(int num_dofs);

    Eigen::Matrix3d m_Rg = Eigen::Matrix3d::Identity();

    std::vector<double>  m_Displacement_n;                ///< 上一时间步初的位移备份
    Eigen::Matrix3d m_Rg_n = Eigen::Matrix3d::Identity(); ///< 上一时间步初的旋转矩阵备份

    // Newmark 步初备份及有限转动状态。材料角速度、角加速度用于积分，
    // m_Velocity[3..5] / m_Acceleration[3..5] 保存供单元使用的空间量。
    std::vector<double> m_Velocity_n;
    std::vector<double> m_Acceleration_n;
    Eigen::Vector3d m_OmegaMaterial = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_AlphaMaterial = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_OmegaMaterial_n = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_AlphaMaterial_n = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_StepRotation = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_TssbnOmegaMaterial1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_TssbnAlphaMaterial1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_TssbnOmegaMaterial2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_TssbnAlphaMaterial2 = Eigen::Vector3d::Zero();

    void BeginNewmarkStep(double dt, double beta, double gamma,
        const std::array<bool, 3>& translationActive,
        const std::array<bool, 3>& rotationActive);
    void ApplyNewmarkCorrection(const Eigen::Vector3d& deltaTranslation,
        const Eigen::Vector3d& deltaRotation, double a0, double a1);
    void RollbackNewmarkStep();
    void SetTssbnStageKinematics(
        int stageIndex,
        double timeStep,
        double firstStageTime,
        double secondStageTime,
        double secondStageDiagonalFraction,
        Eigen::Vector3d& spatialAngularVelocity,
        Eigen::Vector3d& spatialAngularAcceleration);
    TssbnRotationState IntegrateTssbnRotation(
        double timeStep,
        double stageAccelerationExtrapolation,
        double baseFirstWeight,
        double embeddedFirstWeight,
        double embeddedSecondWeight,
        double embeddedLastWeight,
        double lastStageFirstCoefficient,
        double lastStageSecondCoefficient) const;
};

