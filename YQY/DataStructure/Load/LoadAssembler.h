#pragma once

#include <Eigen/Dense>

class StructureData;
class LoadBase;
class Force_Wind;

// Converts a model load into the global partitioned force vectors.  Analysis
// steps retain only DOF numbering and load-step scheduling.
class LoadAssembler
{
public:
    static void Assemble(const LoadBase& load,
        const StructureData& structure,
        int fixedDofCount,
        double scale,
        Eigen::VectorXd& fixedForces,
        Eigen::VectorXd& freeForces);

    static void AssembleGalloping(const Force_Wind& wind,
        const StructureData& structure,
        int iceThickness,
        double initialAttackDegrees,
        const Eigen::Vector3d& modelUp,
        int fixedDofCount,
        double scale,
        Eigen::VectorXd& fixedForces,
        Eigen::VectorXd& freeForces);
};
