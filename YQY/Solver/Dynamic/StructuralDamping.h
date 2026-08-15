#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <QString>
#include "Base/EmptyOUT.h"
#include <vector>

namespace SolverNameSpace
{
struct StructuralDampingSettings
{
    bool enabled = false;
    double translationDampingRatio = 5.0e-3;
    double torsionDampingRatio = 3.8e-2;
    double maximumFrequencyHz = 3.0;
};

struct StructuralDampingReport
{
    int originalDofCount = 0;
    int modalDofCount = 0;
    int selectedModeCount = 0;
    int torsionReferenceModeCount = 0;
    double translationReferenceFrequency1Hz = 0.0;
    double translationReferenceFrequency2Hz = 0.0;
    double torsionReferenceFrequency1Hz = 0.0;
    double torsionReferenceFrequency2Hz = 0.0;
    double translationMassCoefficient = 0.0;
    double translationStiffnessCoefficient = 0.0;
    double torsionMassCoefficient = 0.0;
    double torsionStiffnessCoefficient = 0.0;
    double maximumBackCheckError = 0.0;
    QString summary;
};

class StructuralDampingModel
{
public:
    StructuralDampingSettings settings;
    StructuralDampingReport report;

    bool Calculate(const Eigen::SparseMatrix<double>& stiffness, const Eigen::SparseMatrix<double>& mass,
                   const std::vector<bool>& torsionDofs, const Eigen::SparseMatrix<double>& modalTransformation,
                   _OUT QString& errorMessage);
    void Reset();
    void Disable();

    bool IsComputed() const
    {
        return computed_;
    }

    const Eigen::MatrixXd& Factor() const
    {
        return factor_;
    }

    Eigen::VectorXd Apply(const Eigen::VectorXd& velocity) const;

private:
    bool computed_ = false;
    Eigen::MatrixXd factor_;
};
}
