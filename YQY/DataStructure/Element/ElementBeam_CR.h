#pragma once
#include "ElementBase.h"
class ElementBeam_CR : public ElementBase
{
public:
    ElementBeam_CR();

    Vector3d q0;    // 单元参考向量,截面向量

	int Get_NodeDOF() const override { return 6; };
    Eigen::Matrix3d R0 = Matrix3d::Identity();

    void Get_ke(_OUT MatrixXd& ke);
    void Get_me_Lumped(_OUT MatrixXd& me);         //集中质量矩阵
    void Get_me_Consistent(_OUT MatrixXd& me);     //一致质量矩阵
    void Get_InertiaForce(_OUT VectorXd& inertiaForce) override;
    void Get_GyroscopicMatrix(_OUT MatrixXd& gyroscopicMatrix) override;
    void Get_CentrifugalMatrix(_OUT MatrixXd& centrifugalMatrix) override;
    void GetDynamicContributions(
        _OUT MatrixXd& massMatrix,
        _OUT VectorXd& inertiaForce,
        _OUT MatrixXd& gyroscopicMatrix,
        _OUT MatrixXd& centrifugalMatrix) override;
    void Get_L0();
    void Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce);

private:
    Vector3d def_p1, def_p2;  //当前节点坐标
    Matrix3d Rr;  // 局部随动坐标系

    // 按相同的共回转运动学同时计算一致质量、惯性残量和速度切线。
    // 三个输出对应论文离散式中的 M、f_in 和 C_k；允许传入空指针跳过输出。
    void EvaluateDynamicSystem(_OUT MatrixXd* massMatrix,
        _OUT VectorXd* inertiaForce, _OUT MatrixXd* velocityTangent,
        _OUT MatrixXd* configurationTangent = nullptr);

    void Get_kl(const VectorXd& pl, const double& L, _OUT MatrixXd& kl, _OUT VectorXd& fl); // 局部刚度矩阵
    void ComputeDeformedState(_OUT Vector3d& def_p1, _OUT Vector3d& def_p2,
        _OUT Matrix3d& Rg_1, _OUT Matrix3d& Rg_2,
        _OUT Vector3d& q1, _OUT Vector3d& q2, _OUT Vector3d& q);

    // 计算局部随动坐标系 Rr
    Matrix3d ComputeLocalFrame(const Vector3d& def_p1, const Vector3d& def_p2,
        const Vector3d& q);

    // 计算局部变形向量 pl (7x1)
    VectorXd ComputeLocalDeformation(const Vector3d& def_p1, const Vector3d& def_p2,
        const Matrix3d& Rr, const Matrix3d& Rg_1, const Matrix3d& Rg_2);

    // 计算局部材料刚度与内力 (kl, fl, fa, ka)
    void ComputeMaterialStiffness(const VectorXd& pl, const double& L,
        _OUT MatrixXd& kl, _OUT VectorXd& fl,
        _OUT VectorXd& fa, _OUT MatrixXd& ka);

    // 计算全局投影矩阵与材料刚度贡献
    void ComputeGlobalProjection(const Vector3d& def_p1, const Vector3d& def_p2,
        const Vector3d& q1, const Vector3d& q2,
        const Matrix3d& Rr, const MatrixXd& ka, const VectorXd& fa,
        _OUT MatrixXd& K_material, _OUT VectorXd& fp,
        _OUT MatrixXd& P, _OUT MatrixXd& G);
}; 

