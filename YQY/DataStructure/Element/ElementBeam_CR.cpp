#include "ElementBeam_CR.h"

ElementBeam_CR::ElementBeam_CR()
{
    m_pNode.resize(2);
}

void ElementBeam_CR::Get_ke(MatrixXd& ke)
{
}

void ElementBeam_CR::Get_ke_non(MatrixXd& ke)
{
    // ---- 计算变形状态 ----
    Vector3d def_p1, def_p2;
    Matrix3d Rg_1, Rg_2;
    Vector3d q1, q2, q;
    //std::cout << (VectorXd(q0).transpose()) << std::endl;
    ComputeDeformedState(def_p1, def_p2, Rg_1, Rg_2, q1, q2, q);

    // ---- 局部坐标系 ----
    Matrix3d Rr = ComputeLocalFrame(def_p1, def_p2, q);
    Vector3d r1 = (def_p2 - def_p1).normalized();

    // ---- 局部变形向量 pl ----
    VectorXd pl = ComputeLocalDeformation(def_p1, def_p2, Rr, Rg_1, Rg_2, L0);
    double L = (def_p2 - def_p1).norm();

    // ---- 局部材料刚度与内力 ----
    MatrixXd kl, ka;
    VectorXd fl, fa;
    ComputeMaterialStiffness(pl, L, kl, fl, fa, ka);

    // ---- 全局投影与材料刚度贡献 ----
    MatrixXd K_material, P, G, E;
    VectorXd fg;
    ComputeGlobalProjection(def_p1, def_p2, q1, q2, Rr, ka, fa, K_material, fg, P, G, E);
    m_inforce = fg;
    //std::cout << MatrixXd(P) << std::endl<< std::endl;
    // ---- 应力刚度矩阵（几何刚度） ----
    MatrixXd K_sigma;
    Utility::CR::Assemble_stress_k(L, fa, G, P, Rr, q1, q2, r1, K_sigma);

    // ---- 总刚度矩阵 ----
    ke = K_material + K_sigma;
    //ke = 0.5 * (ke + ke.transpose());
}

void ElementBeam_CR::Get_me_Lumped(MatrixXd& me)//集中质量矩阵
{
}

void ElementBeam_CR::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
}

void ElementBeam_CR::Get_L0()
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();

    if (pNode0 == nullptr || pNode1 == nullptr)
    {
        qDebug().noquote() << QStringLiteral("Error: ElementBeam_CR 节点指针为空");
        return;
    }

    double dx0 = pNode1->m_X - pNode0->m_X;
    double dy0 = pNode1->m_Y - pNode0->m_Y;
    double dz0 = pNode1->m_Z - pNode0->m_Z;

    L0 = sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
}

void ElementBeam_CR::Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce)
{
}

void ElementBeam_CR::Get_kl(const VectorXd& pl, const double& L, MatrixXd& _OUT kl, VectorXd& _OUT fl)
{
    // 1. 提取变形分量
    const double u = pl(0);
    const double t11 = pl(1), t21 = pl(2), t31 = pl(3);
    const double t12 = pl(4), t22 = pl(5), t32 = pl(6);

    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();
    auto pSection = pProperty->m_pSection.lock();

    double A = pSection->m_Area;
    double E = pMaterial->m_Young;
    double G = E / (2. * (1 + pMaterial->m_Poisson));

    double Iy = 0.0, Iz = 0.0, J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);
    const double Io = Iy + Iz;

    double Irr = 0.0;
    const double r = pSection->m_Radius;
    if (r > 1e-12) 
    {
        Irr = M_PI * std::pow(r, 6) / 3.0;
    }

    const double L2 = L * L;
    const double L3 = L2 * L;

    const double dt1 = t11 - t12;
    const double dt1_sq = dt1 * dt1;

    // 常用线性组合
    const double t2_41 = 4.0 * t21 - t22;
    const double t2_14 = t21 - 4.0 * t22;
    const double t3_41 = 4.0 * t31 - t32;
    const double t3_14 = t31 - 4.0 * t32;

    // 常用多项式项 (弓弦效应耦合项)
    const double poly_y = 2.0 * t21 * t21 - t21 * t22 + 2.0 * t22 * t22;
    const double poly_z = 2.0 * t31 * t31 - t31 * t32 + 2.0 * t32 * t32;
    const double poly_yz = poly_y + poly_z;

    const double axial_strain_term = L * poly_yz + 30.0 * u;

    // 提取非线性耦合内核 (这是 fl 和 kl 共享的最复杂的项)
    const double N_nonlin_core = 15.0 * Io * dt1_sq + A * L * axial_strain_term;


    // 计算局部内力向量 fl (7 x 1)

    fl.resize(7);
    fl.setZero();
    const double E_900L = E / (900.0 * L);
    // 轴力
    fl(0) = (E / (30.0 * L2)) * N_nonlin_core;

    // 扭矩
    fl(1) = dt1 * (30.0 * G * J * L2 + 15.0 * E * Irr * dt1_sq + E * Io * L * axial_strain_term) / (30.0 * L3);
    fl(4) = -fl(1);

    // Y轴弯矩（绕 y 轴弯曲，用 Iz）

    fl(2) = E_900L * (1800.0 * Iz * (2.0 * t21 + t22) + t2_41 * N_nonlin_core);
    fl(5) = E_900L * (1800.0 * Iz * (t21 + 2.0 * t22) - t2_14 * N_nonlin_core);

    // Z轴弯矩（绕 z 轴弯曲，用 Iy）
    fl(3) = E_900L * (1800.0 * Iy * (2.0 * t31 + t32) + t3_41 * N_nonlin_core);
    fl(6) = E_900L * (1800.0 * Iy * (t31 + 2.0 * t32) - t3_14 * N_nonlin_core);


    // 计算局部切线刚度矩阵 kl (7 x 7)

    kl.resize(7, 7);
    kl.setZero();

    // 常用乘法系数
    const double AE_30 = A * E / 30.0;
    const double AE_L_900 = A * E * L / 900.0;
    const double EIo_L2 = E * Io / L2;
    const double EIo_30L = E * Io / (30.0 * L);

    // 填充上三角 (Upper Triangular Part) 

    // ----- 第 0 行 (轴向 u) -----
    kl(0, 0) = (A * E) / L;
    kl(0, 1) = EIo_L2 * dt1;
    kl(0, 2) = AE_30 * t2_41;
    kl(0, 3) = AE_30 * t3_41;
    kl(0, 4) = -kl(0, 1);
    kl(0, 5) = -AE_30 * t2_14;
    kl(0, 6) = -AE_30 * t3_14;

    // ----- 第 1 行 (扭转 t11) -----
    kl(1, 1) = (30.0 * G * J * L2 + 45.0 * E * Irr * dt1_sq + E * Io * L * (L * poly_yz + 30.0 * u)) / (30.0 * L3);
    kl(1, 2) = EIo_30L * dt1 * t2_41;
    kl(1, 3) = EIo_30L * dt1 * t3_41;
    kl(1, 4) = -(G * J) / L - (E * (45.0 * Irr * dt1_sq + Io * L * (L * poly_yz + 30.0 * u))) / (30.0 * L3);
    kl(1, 5) = -EIo_30L * dt1 * t2_14;
    kl(1, 6) = -EIo_30L * dt1 * t3_14;

    // ----- 第 2 行 (弯曲 t21) -----
    kl(2, 2) = (E * (3600.0 * Iz + 60.0 * Io * dt1_sq + A * L * (L * (24.0 * t21 * t21 - 12.0 * t21 * t22 + 9.0 * t22 * t22 + 4.0 * poly_z) + 120.0 * u))) / (900.0 * L);
    kl(2, 3) = AE_L_900 * t2_41 * t3_41;
    kl(2, 4) = -kl(1, 2);
    kl(2, 5) = (E * (1800.0 * Iz - 15.0 * Io * dt1_sq - A * L * (L * (6.0 * t21 * t21 - 18.0 * t21 * t22 + 6.0 * t22 * t22 + poly_z) + 30.0 * u))) / (900.0 * L);
    kl(2, 6) = -AE_L_900 * t2_41 * t3_14;

    // ----- 第 3 行 (弯曲 t31) -----
    kl(3, 3) = (E * (3600.0 * Iy + 60.0 * Io * dt1_sq + A * L * (L * (4.0 * poly_y + 24.0 * t31 * t31 - 12.0 * t31 * t32 + 9.0 * t32 * t32) + 120.0 * u))) / (900.0 * L);
    kl(3, 4) = -kl(1, 3);
    kl(3, 5) = -AE_L_900 * t2_14 * t3_41;
    kl(3, 6) = (E * (1800.0 * Iy - 15.0 * Io * dt1_sq - A * L * (L * (poly_y + 6.0 * (t31 * t31 - 3.0 * t31 * t32 + t32 * t32)) + 30.0 * u))) / (900.0 * L);

    // ----- 第 4 行 (扭转 t12) -----
    kl(4, 4) = kl(1, 1);
    kl(4, 5) = EIo_30L * dt1 * t2_14;
    kl(4, 6) = EIo_30L * dt1 * t3_14;

    // ----- 第 5 行 (弯曲 t22) -----
    kl(5, 5) = (E * (3600.0 * Iz + 60.0 * Io * dt1_sq + A * L * (L * (9.0 * t21 * t21 - 12.0 * t21 * t22 + 24.0 * t22 * t22 + 4.0 * poly_z) + 120.0 * u))) / (900.0 * L);
    kl(5, 6) = AE_L_900 * t2_14 * t3_14;

    // ----- 第 6 行 (弯曲 t32) -----
    kl(6, 6) = (E * (3600.0 * Iy + 60.0 * Io * dt1_sq + A * L * (L * (4.0 * poly_y + 9.0 * t31 * t31 - 12.0 * t31 * t32 + 24.0 * t32 * t32) + 120.0 * u))) / (900.0 * L);


    //利用 Eigen 的底层机制，把上三角的值自动复制到下三角

    kl.triangularView<Eigen::Lower>() = kl.transpose();
}

void ElementBeam_CR::ComputeDeformedState(Vector3d& def_p1, Vector3d& def_p2,
    Matrix3d& Rg_1, Matrix3d& Rg_2,
    Vector3d& q1, Vector3d& q2, Vector3d& q)
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pNode0 || !pNode1) return;

    // 节点初始坐标
    Vector3d init_p1(pNode0->m_X, pNode0->m_Y, pNode0->m_Z);
    Vector3d init_p2(pNode1->m_X, pNode1->m_Y, pNode1->m_Z);

    // 平动位移
    Vector3d ug_p1(pNode0->m_Displacement[0], pNode0->m_Displacement[1], pNode0->m_Displacement[2]);
    Vector3d ug_p2(pNode1->m_Displacement[0], pNode1->m_Displacement[1], pNode1->m_Displacement[2]);

    // 转动位移（旋转向量）
    Vector3d thetag_1(pNode0->m_Displacement[3], pNode0->m_Displacement[4], pNode0->m_Displacement[5]);
    Vector3d thetag_2(pNode1->m_Displacement[3], pNode1->m_Displacement[4], pNode1->m_Displacement[5]);

    // 变形后坐标
    def_p1 = init_p1 + ug_p1;
    def_p2 = init_p2 + ug_p2;

    // 全局旋转矩阵
    Utility::CR::Calculate_RotationMatrix(thetag_1, Rg_1);
    Utility::CR::Calculate_RotationMatrix(thetag_2, Rg_2);

    // 截面方向向量
    q0.normalize(); // 确保初始截面方向向量是单位向量
    Vector3d beam_axis = init_p2 - init_p1;
    beam_axis.normalize();
    if (q0.cross(beam_axis).norm() < 1e-6)
    {
        qDebug().noquote() << QStringLiteral("初始截面向量与轴线重合");
        q0 = Vector3d(0, 0, 1); // 如果重合，选择一个默认的垂直向量
    }

    q1 = Rg_1 * q0;
    q2 = Rg_2 * q0;
    q = 0.5 * (q1 + q2);
}

Matrix3d ElementBeam_CR::ComputeLocalFrame(const Vector3d& def_p1, const Vector3d& def_p2,
    const Vector3d& q)
{
    Matrix3d Rr;
    Utility::CR::Calculate_Rr(def_p1, def_p2, q, Rr);
    return Rr;
}

VectorXd ElementBeam_CR::ComputeLocalDeformation(const Vector3d& def_p1, const Vector3d& def_p2, const Matrix3d& Rr, const Matrix3d& Rg_1, const Matrix3d& Rg_2, double L0)
{
    // 当前长度与轴向伸长
    double L = (def_p2 - def_p1).norm();
    double u_ = L - L0;

    // 局部旋转矩阵
    Matrix3d R1_ = Rr.transpose() * Rg_1;
    Matrix3d R2_ = Rr.transpose() * Rg_2;

    // 提取局部旋转向量
    Vector3d vartheta1, vartheta2;
    Utility::CR::Extract_RotationVector(R1_, vartheta1);
    Utility::CR::Extract_RotationVector(R2_, vartheta2);

    VectorXd pl(7);
    pl(0) = u_;
    pl.segment<3>(1) = vartheta1;
    pl.segment<3>(4) = vartheta2;
    return pl;
}

void ElementBeam_CR::ComputeMaterialStiffness(const VectorXd& pl, const double& L, MatrixXd& kl, VectorXd& fl, VectorXd& fa, MatrixXd& ka)
{
    // 局部刚度与内力
    Get_kl(pl, L, kl, fl);

    // 变形映射矩阵 Ba
    Vector3d vartheta1 = pl.segment<3>(1);
    Vector3d vartheta2 = pl.segment<3>(4);
    MatrixXd Ba;
    Utility::CR::Assemble_Matrix_Ba(vartheta1, vartheta2, Ba);

    // 内力投影 fa = Ba^T * fl
    fa = Ba.transpose() * fl;
    if (fa.size() != 7)
    {
        qDebug().noquote() << QStringLiteral("梁 内力向量 fa 大小不为 7");
        return;
    }
    // 局部几何刚度 Kh 和材料刚度 Ka
    MatrixXd Kh;
    Utility::CR::Assemble_Matrix_Kh(vartheta1, vartheta2, fl, Kh);
    ka = Ba.transpose() * kl * Ba + Kh;
}

void ElementBeam_CR::ComputeGlobalProjection(const Vector3d& def_p1, const Vector3d& def_p2,
    const Vector3d& q1, const Vector3d& q2,
    const Matrix3d& Rr, const MatrixXd& ka, const VectorXd& fa,
    MatrixXd& K_material, VectorXd& fp,
    MatrixXd& P, MatrixXd& G, MatrixXd& E)
{
    // 组装转换矩阵 E (12x12) 和投影矩阵 P (7x12)、自旋矩阵 G (3x12)
    Utility::CR::Assemble_Matrix_E(Rr, E);
    Utility::CR::Assemble_Matrix_PG(def_p1, def_p2, q1, q2, Rr, P, G);

    // Bg = P * E
    MatrixXd Bg = P * E;

    // 全局内力向量 fp = Bg^T * fa
    fp = Bg.transpose() * fa;

    // 全局材料刚度矩阵 K_material = Bg^T * ka * Bg
    K_material = Bg.transpose() * ka * Bg;
}