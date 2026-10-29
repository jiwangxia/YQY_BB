#pragma once

#include "Base/EmptyOUT.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <map>

class StructureData;
class LoadBase;
class Force_Wind;
class Force_Gravity;
class ElementBase;

// 将模型荷载转换为按约束自由度和自由自由度划分的全局力向量。
// 分析步只负责自由度编号和荷载作用时序。
class LoadAssembler
{
public:
    struct GallopingElementAeroSample
    {
        double attackDegrees = 0.0;
        double normalizedAttackDegrees = 0.0;
        double flowAngleDegrees = 0.0;
        double relativeSpeed = 0.0;
        double liftCoefficient = 0.0;
        double dragCoefficient = 0.0;
        double momentCoefficient = 0.0;
        double lineTorque = 0.0;
        double currentLength = 0.0;
        double meanTwistRate = 0.0;
        double torsionalPowerPerLength = 0.0;
        double translationalPowerPerLength = 0.0;
        double rotationalPowerPerLength = 0.0;
        double totalPowerPerLength = 0.0;
        double fittedWeight = 0.0;
        double appliedScale = 0.0;
    };

    struct GallopingAssemblyTiming
    {
        double elementLoopMilliseconds = 0.0;
        int elementEvaluations = 0;
    };

    static void Assemble(const LoadBase& load, const StructureData& structure, int fixedDofCount, double scale,
                         _OUT Eigen::VectorXd& fixedForces, _OUT Eigen::VectorXd& freeForces);

    // 参考质量中心线的重力势能，以未变形几何为零；偏心索另计现有重力模型的质量一阶矩势能。
    static bool EvaluateGravityPotential(const StructureData& structure, const Eigen::Vector3d& acceleration,
                                         const Eigen::Vector3d& eccentricAcceleration, _OUT double& energy);

    // referenceTssbn 仅作用于本次装配：名义动压和固定升阻力方向；系数沿用表格插值。
    static void AssembleGalloping(const Force_Wind& wind, const StructureData& structure,
                                  const std::map<int, int>& elementProfileBindings, int iceThickness,
                                  double initialAttackDegrees, const Eigen::Vector3d& modelUp, int fixedDofCount,
                                  double scale, _OUT Eigen::VectorXd& fixedForces, _OUT Eigen::VectorXd& freeForces,
                                  _OUT GallopingAssemblyTiming* timing = nullptr, bool referenceTssbn = false);

    static void AssembleGallopingTangent(const Force_Wind& wind, const StructureData& structure,
                                         const std::map<int, int>& elementProfileBindings, int iceThickness,
                                         double initialAttackDegrees, const Eigen::Vector3d& modelUp, int fixedDofCount,
                                         int freeDofCount, double scale, double velocityDerivative,
                                         _OUT Eigen::SparseMatrix<double>& externalTangent,
                                         bool referenceTssbn = false);

    static void AssembleEccentricGravityTangent(const Force_Gravity& gravity, const StructureData& structure,
                                                int fixedDofCount, int freeDofCount, double scale,
                                                _OUT Eigen::SparseMatrix<double>& externalTangent);

    // 与普通梁单元的当前弦线重力等效节点力保持一致；桁架和索不在此列。
    static void AssembleDeformedGravityTangent(const Force_Gravity& gravity, const StructureData& structure,
                                               int fixedDofCount, int freeDofCount, double scale,
                                               _OUT Eigen::SparseMatrix<double>& externalTangent);

    // 只读取当前已提交的节点状态；诊断采样与实际气动力采用同一攻角和系数查询。
    static GallopingElementAeroSample SampleGallopingElementAero(
        const Force_Wind& wind, const StructureData& structure, const ElementBase& element, int profileId,
        int iceThickness, double initialAttackDegrees, const Eigen::Vector3d& modelUp, bool referenceTssbn = false);
};
