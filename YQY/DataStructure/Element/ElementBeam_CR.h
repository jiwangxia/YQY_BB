#pragma once
#include "ElementBase.h"
class ElementBeam_CR : public ElementBase
{
public:
    ElementBeam_CR();

    Vector3d q0;    // 单元参考向量,截面向量

    int Get_NodeDOF() const override { return 6; };
    Eigen::Matrix3d R0 = Matrix3d::Identity();

    void Get_ke(MatrixXd& ke);
    void Get_me_Lumped(MatrixXd& me);         //集中质量矩阵
    void Get_me_Consistent(MatrixXd& me);     //一致质量矩阵
    void Get_L0();
    void Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce);

private:
    Vector3d def_p1, def_p2;  //当前节点坐标
    Matrix3d Rr;  // 局部随动坐标系

    void Get_kl(const VectorXd& pl, const double& L, MatrixXd& _OUT kl, VectorXd& _OUT fl); // 局部刚度矩阵
    void ComputeDeformedState(Vector3d& def_p1, Vector3d& def_p2,
        Matrix3d& Rg_1, Matrix3d& Rg_2,
        Vector3d& q1, Vector3d& q2, Vector3d& q);

    // 计算局部随动坐标系 Rr
    Matrix3d ComputeLocalFrame(const Vector3d& def_p1, const Vector3d& def_p2,
        const Vector3d& q);

    // 计算局部变形向量 pl (7x1)
    VectorXd ComputeLocalDeformation(const Vector3d& def_p1, const Vector3d& def_p2,
        const Matrix3d& Rr, const Matrix3d& Rg_1, const Matrix3d& Rg_2);

    // 计算局部材料刚度与内力 (kl, fl, fa, ka)
    void ComputeMaterialStiffness(const VectorXd& pl, const double& L,
        MatrixXd& kl, VectorXd& fl,
        VectorXd& fa, MatrixXd& ka);

    // 计算全局投影矩阵与材料刚度贡献
    void ComputeGlobalProjection(const Vector3d& def_p1, const Vector3d& def_p2,
        const Vector3d& q1, const Vector3d& q2,
        const Matrix3d& Rr, const MatrixXd& ka, const VectorXd& fa,
        MatrixXd& K_material, VectorXd& fp,
        MatrixXd& P, MatrixXd& G);
}; 

