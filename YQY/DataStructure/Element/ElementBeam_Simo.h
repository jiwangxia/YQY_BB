#pragma once

#include "ElementBase.h"

/**
 * @brief Two-node geometrically exact Simo-Reissner beam.
 *
 * The nodal layout is identical to ElementBeam_CR: three translations followed
 * by three spatial rotation DOFs.  Gauss-point states are rebuilt from the
 * current nodal positions and orientations during assembly, so trial-state
 * rollback remains owned by Node/AnalysisStep.
 */
class ElementBeam_Simo : public ElementBase
{
public:
    ElementBeam_Simo();

	int Get_NodeDOF() const override { return 6; }

    Eigen::Vector3d q0 = Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();

    void Get_ke(Eigen::MatrixXd& ke) override;
    void Get_me_Lumped(Eigen::MatrixXd& me) override;
    void Get_me_Consistent(Eigen::MatrixXd& me) override;
    void Get_L0() override;
    void Assemble(const std::vector<double>& damping, Eigen::MatrixXd& ce) override;

private:
    struct GaussState
    {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        Eigen::Vector3d strain = Eigen::Vector3d::Zero();
        Eigen::Vector3d curvature = Eigen::Vector3d::Zero();
        Eigen::Vector3d chord = Eigen::Vector3d::Zero();
    };

    GaussState EvaluateGaussState(double coordinate) const;
    void GetSectionData(double& area, double& Iy, double& Iz, double& J,
        double& young, double& shear, double& density) const;
};
