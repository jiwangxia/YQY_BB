#pragma once

#include "Base/EmptyOUT.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <map>

class StructureData;
class LoadBase;
class Force_Wind;

// 将模型荷载转换为按约束自由度和自由自由度划分的全局力向量。
// 分析步只负责自由度编号和荷载作用时序。
class LoadAssembler
{
public:
    static void Assemble(const LoadBase& load, const StructureData& structure, int fixedDofCount, double scale,
                         _OUT Eigen::VectorXd& fixedForces, _OUT Eigen::VectorXd& freeForces);

    static void AssembleGalloping(const Force_Wind& wind, const StructureData& structure,
                                  const std::map<int, int>& elementProfileBindings, int iceThickness,
                                  double initialAttackDegrees, const Eigen::Vector3d& modelUp, int fixedDofCount,
                                  double scale, _OUT Eigen::VectorXd& fixedForces, _OUT Eigen::VectorXd& freeForces);

    static void AssembleGallopingTangent(const Force_Wind& wind, const StructureData& structure,
                                         const std::map<int, int>& elementProfileBindings, int iceThickness,
                                         double initialAttackDegrees, int fixedDofCount, int freeDofCount, double scale,
                                         double velocityDerivative,
                                         _OUT Eigen::SparseMatrix<double>& externalTangent);
};
