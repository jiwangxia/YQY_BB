#pragma once

#include "ElementBase.h"

/**
 * @brief 两节点几何精确 Simo-Reissner 梁。
 *
 * 节点自由度顺序与 ElementBeam_CR 相同：三个平动自由度后接三个空间转动自由度。
 * 装配时由当前节点位置与方向重建 Gauss 点状态，试探状态回滚仍由 Node/AnalysisStep 负责。
 */
class ElementBeam_Simo : public ElementBase
{
public:
    ElementBeam_Simo();

    int Get_NodeDOF() const override
    {
        return 6;
    }

    Eigen::Vector3d q0 = Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();

    void Get_ke(_OUT Eigen::MatrixXd& ke) override;
    bool TryGetStrainEnergy(_OUT double& energy) const override;
    bool TryGetKineticEnergy(_OUT double& energy) override;
    void Get_me_Lumped(_OUT Eigen::MatrixXd& me) override;
    void Get_me_Consistent(_OUT Eigen::MatrixXd& me) override;
    void Get_L0() override;
    void Assemble(const std::vector<double>& damping, _OUT Eigen::MatrixXd& ce) override;

private:
    struct GaussState
    {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        Eigen::Vector3d strain = Eigen::Vector3d::Zero();
        Eigen::Vector3d curvature = Eigen::Vector3d::Zero();
        Eigen::Vector3d chord = Eigen::Vector3d::Zero();
    };

    GaussState EvaluateGaussState(double coordinate) const;
    void GetSectionData(_OUT double& area, _OUT double& Iy, _OUT double& Iz, _OUT double& J, _OUT double& young,
                        _OUT double& shear, _OUT double& density) const;
};
