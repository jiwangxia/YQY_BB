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
    Matrix3d Rg_1, Rg_2;
    Vector3d q1, q2, q;
    //std::cout << (VectorXd(q0).transpose()) << std::endl;
    ComputeDeformedState(def_p1, def_p2, Rg_1, Rg_2, q1, q2, q);

    // ---- 局部坐标系 ----
    Rr = ComputeLocalFrame(def_p1, def_p2, q);
    Vector3d r1 = (def_p2 - def_p1).normalized();

    // ---- 局部变形向量 pl ----
    VectorXd pl = ComputeLocalDeformation(def_p1, def_p2, Rr, Rg_1, Rg_2);
    L = (def_p2 - def_p1).norm();

    // ---- 局部材料刚度与内力 ----
    MatrixXd kl, ka;
    VectorXd fl, fa;
    ComputeMaterialStiffness(pl, L, kl, fl, fa, ka);

    // ---- 全局投影与材料刚度贡献 ----
    MatrixXd K_material, P, G;
    VectorXd fg;
    ComputeGlobalProjection(def_p1, def_p2, q1, q2, Rr, ka, fa, K_material, fg, P, G);
    m_inforce = fg;

    // ---- 应力刚度矩阵（几何刚度） ----
    MatrixXd K_sigma;
    Utility::CR::Assemble_stress_k(L, fa, G, P, Rr, q1, q2, r1, K_sigma);

    // ---- 总刚度矩阵 ----
    ke = K_material + K_sigma;
    m_ke = ke;
}

void ElementBeam_CR::Get_me_Lumped(MatrixXd& me)//集中质量矩阵
{
}

void ElementBeam_CR::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
    me.setZero(12, 12);

    // ==========================================
    // 1.1 获取基本属性和材料参数
    // ==========================================
    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();
    auto pSection = pProperty->m_pSection.lock();

    double A = pSection->m_Area;
    double p = pMaterial->m_Density;
    double Ap = p * A; // 线质量密度 (\rho * A)

    double Iy = 0.0, Iz = 0.0, J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);
    const double Io = pSection->Io;
    double Irr = pSection->Irr;
    Matrix3d JJ = Matrix3d::Zero();
    JJ(0, 0) = 20;//p * (Iy + Iz); // 极质量惯性矩
    JJ(1, 1) = 10;//p * Iz;
    JJ(2, 2) = 10;//p * Iy;

    // ==========================================
    // 1.2 获取当前变形状态和局部坐标系
    // ==========================================
    Matrix3d Rg_1, Rg_2;
    Vector3d def_p1, def_p2, q1, q2, q;
    ComputeDeformedState(def_p1, def_p2, Rg_1, Rg_2, q1, q2, q);

    Matrix3d Rr = ComputeLocalFrame(def_p1, def_p2, q);

    // 局部变形参数 (仅提取后6个分量作为局部转角)
    VectorXd pl = ComputeLocalDeformation(def_p1, def_p2, Rr, Rg_1, Rg_2);
    VectorXd thetaL = pl.segment<6>(1);

    // ==========================================
    // 1.3 调用 CR.cpp 工具获取核心转换矩阵
    // ==========================================
    MatrixXd P, G; // P 自动是 6x12 (仅旋转), G 是 3x12 (刚体自旋)
    Utility::CR::Assemble_Matrix_PG(def_p1, def_p2, q1, q2, Rr, P, G);

    // E_mat 对角块是 Rr^T，因此它是 全局 -> 局部 的转换矩阵
    MatrixXd E_mat;
    Utility::CR::Assemble_Matrix_E(Rr, E_mat);

    // ==========================================
    // 1.4 两点高斯积分计算局部质量矩阵 Me
    // ==========================================
    double gpt[2] = { -sqrt(3.0) / 3.0, sqrt(3.0) / 3.0 }; // 高斯点坐标
    double gh[2] = { 1.0, 1.0 };                          // 高斯点权重
    MatrixXd Me = MatrixXd::Zero(12, 12);

    for (int i = 0; i < 2; i++)
    {
        double c = gpt[i];

        // 形函数 N 及其导数
        double N[6];
        N[0] = 0.5 - 0.5 * c;
        N[1] = 0.5 + 0.5 * c;
        N[2] = (0.5 - 0.5 * c) * (0.5 - 0.5 * c) * (0.5 + 0.5 * c) * L0;
        N[3] = (-0.5 + 0.5 * c) * (0.5 + 0.5 * c) * (0.5 + 0.5 * c) * L0;
        N[4] = -0.25 - 0.5 * c + 0.75 * c * c;
        N[5] = -0.25 + 0.5 * c + 0.75 * c * c;

        MatrixXd P1 = MatrixXd::Zero(3, 6);
        P1(1, 2) = N[2]; P1(1, 5) = N[3];
        P1(2, 1) = -N[2]; P1(2, 4) = -N[3];

        MatrixXd P2 = MatrixXd::Zero(3, 6);
        P2(0, 0) = N[0]; P2(0, 3) = N[1];
        P2(1, 1) = N[4]; P2(1, 4) = N[5];
        P2(2, 2) = N[4]; P2(2, 5) = N[5];

        // 高斯点处的局部变形与惯性主矩 Ip
        Vector3d ulG = P1 * thetaL;
        Vector3d thetaG = P2 * thetaL;

        Matrix3d MthetaG;
        Utility::CR::SkewSymmetric(thetaG, MthetaG);
        Matrix3d R_G = Matrix3d::Identity() + MthetaG;
        Matrix3d Ip = R_G * JJ * R_G.transpose(); // 考虑大转角后的截面质量惯性

        // 速度映射矩阵 H1 (平动), H2 (转动)
        MatrixXd N_M = MatrixXd::Zero(3, 12);
        N_M.block<3, 3>(0, 0) = N[0] * Matrix3d::Identity();
        N_M.block<3, 3>(0, 6) = N[1] * Matrix3d::Identity();

        Matrix3d MulG;
        Utility::CR::SkewSymmetric(ulG, MulG);
        MatrixXd H1 = N_M + P1 * P - MulG * G;
        MatrixXd H2 = P2 * P + G;

        // 累加高斯点贡献
        Me += gh[i] * (H1.transpose() * Ap * H1 + H2.transpose() * Ip * H2);
    }

    // ==========================================
    // 1.5 坐标转换回全局坐标系
    // E_mat 是 全局->局部，因此 E_mat.transpose() 是 局部->全局
    // ==========================================
    me = 0.5 * L0 * E_mat * Me * E_mat.transpose();
    m_me = me; // 缓存质量矩阵



    //me.setZero(12, 12);

    //double L = (def_p2 - def_p1).norm();//当前长度

    ////获取材料和截面属性
    //auto pProperty = m_pProperty.lock();
    //auto pMaterial = pProperty->m_pMaterial.lock();
    //auto pSection = pProperty->m_pSection.lock();

    //double A = pSection->m_Area;
    //double E = pMaterial->m_Young;
    //double density = pMaterial->m_Density;

    ////double Iyy = 0.0, Izz = 0.0, J = 0.0;
    ////pSection->Calculate_I(Iyy, Izz, J);
    ////const double Io = pSection->Io;
    ////double Sy = pSection->Sy;
    ////double Sz = pSection->Sz;

    //double mu = density * A;             // 线质量密度 (kg/m)

    //double mass_Iy = 10;// density* Iyy;
    //double mass_Iz = 10;// density* Izz;
    //double mass_Io = 20;// density* Io;
    //double mass_Sy = 0;//density * Sy;
    //double mass_Sz = 0;//density * Sz; 
    //double mass_Iyz = 0.0;

    //// 预计算公共项以极致优化执行速度
    //double muL = mu * L;
    //double L2 = L * L;
    //double L3 = L2 * L;

    //// ==========================================
    //// 3. 填充一致质量矩阵上三角部分
    //// ==========================================

    //// 第 1 行 (u1)
    //me(0, 0) = muL / 3.0;
    //me(0, 1) = 0.5 * mass_Sz;
    //me(0, 2) = 0.5 * mass_Sy;
    //me(0, 4) = mass_Sy * L / 12.0;
    //me(0, 5) = -mass_Sz * L / 12.0;
    //me(0, 6) = muL / 6.0;
    //me(0, 7) = -0.5 * mass_Sz;
    //me(0, 8) = -0.5 * mass_Sy;
    //me(0, 10) = -mass_Sy * L / 12.0;
    //me(0, 11) = mass_Sz * L / 12.0;

    //// 第 2 行 (v1)
    //me(1, 1) = (13.0 / 35.0) * muL + (6.0 / (5.0 * L)) * mass_Iz;
    //me(1, 2) = (6.0 / (5.0 * L)) * mass_Iyz;
    //me(1, 3) = -(7.0 / 20.0) * mass_Sy * L;
    //me(1, 4) = 0.1 * mass_Iyz;
    //me(1, 5) = (11.0 / 210.0) * mu * L2 + 0.1 * mass_Iz;
    //me(1, 6) = 0.5 * mass_Sz;
    //me(1, 7) = (9.0 / 70.0) * muL - (6.0 / (5.0 * L)) * mass_Iz;
    //me(1, 8) = -(6.0 / (5.0 * L)) * mass_Iyz;
    //me(1, 9) = -(3.0 / 20.0) * mass_Sy * L;
    //me(1, 10) = 0.1 * mass_Iyz;
    //me(1, 11) = -(13.0 / 420.0) * mu * L2 + 0.1 * mass_Iz;
    //// 第 3 行 (w1)
    //me(2, 2) = (13.0 / 35.0) * muL + (6.0 / (5.0 * L)) * mass_Iy;
    //me(2, 3) = (7.0 / 20.0) * mass_Sz * L;
    //me(2, 4) = -(11.0 / 210.0) * mu * L2 - 0.1 * mass_Iy;
    //me(2, 5) = -0.1 * mass_Iyz;
    //me(2, 6) = 0.5 * mass_Sy;
    //me(2, 7) = -(6.0 / (5.0 * L)) * mass_Iyz;
    //me(2, 8) = (9.0 / 70.0) * muL - (6.0 / (5.0 * L)) * mass_Iy;
    //me(2, 9) = (3.0 / 20.0) * mass_Sz * L;
    //me(2, 10) = -(13.0 / 420.0) * mu * L2 - 0.1 * mass_Iy;
    //me(2, 11) = 0.1 * mass_Iyz;

    //// 第 4 行 (theta_x1)
    //me(3, 3) = mass_Io * L / 3.0;
    //me(3, 4) = -mass_Sz * L2 / 20.0;
    //me(3, 5) = -mass_Sy * L2 / 20.0;
    //me(3, 7) = -3.0 * mass_Sy * L / 20.0;
    //me(3, 8) = 3.0 * mass_Sz * L / 20.0;
    //me(3, 9) = mass_Io * L / 6.0;
    //me(3, 10) = mass_Sz * L2 / 30.0;
    //me(3, 11) = mass_Sy * L2 / 30.0;

    //// 第 5 行 (theta_y1)
    //me(4, 4) = mu * L3 / 105.0 + (2.0 / 15.0) * mass_Iy * L;
    //me(4, 5) = -(2.0 / 15.0) * mass_Iyz * L;
    //me(4, 6) = -mass_Sy * L / 12.0;
    //me(4, 7) = 0.1 * mass_Iyz;
    //me(4, 8) = -(13.0 / 420.0) * mu * L2 + 0.1 * mass_Iy;
    //me(4, 9) = -mass_Sz * L2 / 30.0;
    //me(4, 10) = -mu * L3 / 140.0 - (1.0 / 30.0) * mass_Iy * L;
    //me(4, 11) = mass_Iyz * L / 30.0;

    //// 第 6 行 (theta_z1)
    //me(5, 5) = mu * L3 / 105.0 + (2.0 / 15.0) * mass_Iz * L;
    //me(5, 6) = -mass_Sz * L / 12.0;
    //me(5, 7) = (13.0 / 420.0) * mu * L2 - 0.1 * mass_Iz;
    //me(5, 8) = -0.1 * mass_Iyz;
    //me(5, 9) = -mass_Sy * L2 / 30.0;
    //me(5, 10) = mass_Iyz * L / 30.0;
    //me(5, 11) = -mu * L3 / 140.0 - (1.0 / 30.0) * mass_Iz * L;

    //// 第 7 行 (u2)
    //me(6, 6) = muL / 3.0;
    //me(6, 7) = -0.5 * mass_Sz;
    //me(6, 8) = -0.5 * mass_Sy;
    //me(6, 10) = mass_Sy * L / 12.0;
    //me(6, 11) = -mass_Sz * L / 12.0;

    //// 第 8 行 (v2)
    //me(7, 7) = (13.0 / 35.0) * muL + (6.0 / (5.0 * L)) * mass_Iz;
    //me(7, 8) = (6.0 / (5.0 * L)) * mass_Iyz;
    //me(7, 9) = -(7.0 / 20.0) * mass_Sy * L;
    //me(7, 10) = 0.1 * mass_Iyz;
    //me(7, 11) = -(11.0 / 210.0) * mu * L2 - 0.1 * mass_Iz;

    //// 第 9 行 (w2)
    //me(8, 8) = (13.0 / 35.0) * muL + (6.0 / (5.0 * L)) * mass_Iy;
    //me(8, 9) = (7.0 / 20.0) * mass_Sz * L;
    //me(8, 10) = (11.0 / 210.0) * mu * L2 + 0.1 * mass_Iy;
    //me(8, 11) = -0.1 * mass_Iyz;

    //// 第 10 行 (theta_x2)
    //me(9, 9) = mass_Io * L / 3.0;
    //me(9, 10) = mass_Sz * L2 / 20.0;
    //me(9, 11) = mass_Sy * L2 / 20.0;

    //// 第 11 行 (theta_y2)
    //me(10, 10) = mu * L3 / 105.0 + (2.0 / 15.0) * mass_Iy * L;
    //me(10, 11) = -(2.0 / 15.0) * mass_Iyz * L;

    //// 第 12 行 (theta_z2)
    //me(11, 11) = mu * L3 / 105.0 + (2.0 / 15.0) * mass_Iz * L;

    //// ==========================================
    //// 4. 强制对称化映射（将上三角复制到下三角）
    //// ==========================================
    //me.triangularView<Eigen::StrictlyLower>() = me.transpose();

    //MatrixXd ET;
    //Utility::CR::Assemble_Matrix_E(Rr, ET);
    //me = ET * me * ET.transpose();

    //m_me = me;
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

    Vector3d p1(pNode0->m_X, pNode0->m_Y, pNode0->m_Z);
    Vector3d p2(pNode1->m_X, pNode1->m_Y, pNode1->m_Z);

    Vector3d d = p2 - p1;
    L0 = d.norm();

    Vector3d r1 = d.normalized();
    Vector3d cross_result = r1.cross(q0);
    Vector3d r3 = cross_result.normalized();
    Vector3d r2 = r3.cross(r1);

    R0.col(0) = r1;
    R0.col(1) = r2;
    R0.col(2) = r3;
    q0 = r2;
}

void ElementBeam_CR::Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce)
{
    ce.setZero(12, 12);

    // ==========================================
    // 1. 获取基本属性和材料参数
    // ==========================================
    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();
    auto pSection = pProperty->m_pSection.lock();

    double A = pSection->m_Area;
    double p = pMaterial->m_Density;
    double Ap = p * A; // 线质量密度 (\rho * A)

    double Iy = 0.0, Iz = 0.0, J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);
    const double Io = pSection->Io;
    double Irr = pSection->Irr;
    Matrix3d JJ = Matrix3d::Zero();
    JJ(0, 0) = p * (Iy + Iz); // 极质量惯性矩
    JJ(1, 1) = p * Iz;
    JJ(2, 2) = p * Iy;

    // ==========================================
    // 2. 获取当前变形状态和局部坐标系
    // ==========================================
    Matrix3d Rg_1, Rg_2;
    Vector3d def_p1, def_p2, q1, q2, q;
    ComputeDeformedState(def_p1, def_p2, Rg_1, Rg_2, q1, q2, q);

    Matrix3d Rr = ComputeLocalFrame(def_p1, def_p2, q);

    // 局部变形参数 (仅提取后6个分量作为局部转角)
    VectorXd pl = ComputeLocalDeformation(def_p1, def_p2, Rr, Rg_1, Rg_2);
    VectorXd thetaL = pl.segment<6>(1);

    // ==========================================
    // 3. 调用 CR.cpp 工具获取核心转换矩阵
    // ==========================================
    // P 自动是 6x12 (仅旋转), G 是 3x12 (刚体自旋)
    MatrixXd P, G;
    Utility::CR::Assemble_Matrix_PG(def_p1, def_p2, q1, q2, Rr, P, G);

    // E_mat 对角块是 Rr^T，它是 全局 -> 局部 的转换矩阵
    MatrixXd E_mat;
    Utility::CR::Assemble_Matrix_E(Rr, E_mat);

    // ==========================================
    // 4. 获取单元的全局速度向量并转换到局部
    // ==========================================
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    VectorXd vg = VectorXd::Zero(12);
    for (int i = 0; i < 6; i++) {
        vg(i) = pNode0->m_Velocity[i];
        vg(i + 6) = pNode1->m_Velocity[i];
    }

    // 利用 E_mat 直接得到局部(随动)坐标系下的速度 ve
    VectorXd ve = E_mat * vg;

    // ==========================================
    // 5. 预计算高斯积分恒定矩阵
    // ==========================================
    double LL2 = L * L;

    VectorXd r = VectorXd::Zero(12);
    Vector3d r1 = (def_p2 - def_p1).normalized();
    r.segment<3>(0) = -r1;
    r.segment<3>(6) = r1;

    MatrixXd A1 = MatrixXd::Zero(3, 12);
    A1(1, 1) = -1; A1(2, 2) = -1; A1(1, 7) = 1; A1(2, 8) = 1;

    MatrixXd A2 = MatrixXd::Zero(3, 12);
    A2(1, 2) = 1;  A2(2, 1) = -1; A2(1, 8) = -1; A2(2, 7) = 1;

    // 利用 CR 工具计算速度反对称矩阵 F1
    MatrixXd F1 = MatrixXd::Zero(12, 3);
    Matrix3d skew_temp;

    Utility::CR::SkewSymmetric(ve.segment<3>(0), skew_temp);
    F1.block<3, 3>(0, 0) = skew_temp;

    Utility::CR::SkewSymmetric(ve.segment<3>(3), skew_temp);
    F1.block<3, 3>(3, 0) = skew_temp;

    Utility::CR::SkewSymmetric(ve.segment<3>(6), skew_temp);
    F1.block<3, 3>(6, 0) = skew_temp;

    Utility::CR::SkewSymmetric(ve.segment<3>(9), skew_temp);
    F1.block<3, 3>(9, 0) = skew_temp;

    // ==========================================
    // 6. 两点高斯积分计算局部切线阻尼 Cke
    // ==========================================
    double gpt[2] = { -sqrt(3.0) / 3.0, sqrt(3.0) / 3.0 };
    double gh[2] = { 1.0, 1.0 };
    MatrixXd Cke = MatrixXd::Zero(12, 12);

    for (int i = 0; i < 2; i++)
    {
        double c = gpt[i];

        // 6.1 形函数
        double N[6];
        N[0] = 0.5 - 0.5 * c;
        N[1] = 0.5 + 0.5 * c;
        N[2] = (0.5 - 0.5 * c) * (0.5 - 0.5 * c) * (0.5 + 0.5 * c) * L0;
        N[3] = (-0.5 + 0.5 * c) * (0.5 + 0.5 * c) * (0.5 + 0.5 * c) * L0;
        N[4] = -0.25 - 0.5 * c + 0.75 * c * c;
        N[5] = -0.25 + 0.5 * c + 0.75 * c * c;

        MatrixXd P1 = MatrixXd::Zero(3, 6);
        P1(1, 2) = N[2]; P1(1, 5) = N[3];
        P1(2, 1) = -N[2]; P1(2, 4) = -N[3];

        MatrixXd P2 = MatrixXd::Zero(3, 6);
        P2(0, 0) = N[0]; P2(0, 3) = N[1];
        P2(1, 1) = N[4]; P2(1, 4) = N[5];
        P2(2, 2) = N[4]; P2(2, 5) = N[5];

        // 6.2 局部变量与大转角惯性主矩更新
        Vector3d ulG = P1 * thetaL;
        Vector3d thetaG = P2 * thetaL;

        Matrix3d thetaG_skew;
        Utility::CR::SkewSymmetric(thetaG, thetaG_skew);
        Matrix3d R_G = Matrix3d::Identity() + thetaG_skew;
        Matrix3d Ip = R_G * JJ * R_G.transpose();

        // 6.3 速度映射矩阵
        MatrixXd N_M = MatrixXd::Zero(3, 12);
        N_M.block<3, 3>(0, 0) = N[0] * Matrix3d::Identity();
        N_M.block<3, 3>(0, 6) = N[1] * Matrix3d::Identity();

        Matrix3d ulG_skew;
        Utility::CR::SkewSymmetric(ulG, ulG_skew);

        // P 在 Assemble_Matrix_PG 中已经被构建为 6x12
        MatrixXd H1 = N_M + P1 * P - ulG_skew * G;
        MatrixXd H2 = P2 * P + G;

        // 6.4 映射矩阵对时间的导数
        Vector3d vlG = P1 * P * ve;
        Matrix3d vlG_skew;
        Utility::CR::SkewSymmetric(vlG, vlG_skew);

        MatrixXd NLA1 = (N[2] + N[3]) / LL2 * A1;
        MatrixXd NLA2 = (N[4] + N[5] - 1.0) / LL2 * A2;

        double r_vg = r.transpose() * vg;
        MatrixXd dH1 = NLA1 * r_vg - vlG_skew * G;
        MatrixXd dH2 = NLA2 * r_vg;

        // 6.5 对流耦合项 C1 ~ C4 (高度非线性)
        Vector3d thetaE = G * ve; // 刚体自旋角速度
        Matrix3d thetaE_s;
        Utility::CR::SkewSymmetric(thetaE, thetaE_s);

        MatrixXd Et = MatrixXd::Zero(12, 12);
        Et.block<3, 3>(0, 0) = thetaE_s; Et.block<3, 3>(3, 3) = thetaE_s;
        Et.block<3, 3>(6, 6) = thetaE_s; Et.block<3, 3>(9, 9) = thetaE_s;

        MatrixXd C1 = thetaE_s * H1 + dH1 - H1 * Et;
        MatrixXd C2 = thetaE_s * H2 + dH2 - H2 * Et;

        VectorXd h1 = H1 * ve; // 高斯点平动速度
        VectorXd h2 = H2 * ve; // 高斯点转动速度

        Matrix3d h1_skew, h2_skew;
        Utility::CR::SkewSymmetric(h1, h1_skew);
        Utility::CR::SkewSymmetric(h2, h2_skew);

        VectorXd re = VectorXd::Zero(12);
        re[0] = -1; re[6] = 1;

        MatrixXd C3 = -h1_skew * G + NLA1 * ve * re.transpose() + thetaE_s * P1 * P + H1 * F1 * G;
        MatrixXd C4 = -h2_skew * G + NLA2 * ve * re.transpose() + H2 * F1 * G;

        // 6.6 陀螺力矩项: skew(w) * I - I * skew(w)
        Matrix3d Gyro_term = h2_skew * Ip - Ip * h2_skew;

        // 累加局部切线阻尼
        Cke += gh[i] * (H1.transpose() * Ap * (C1 + C3) +
            H2.transpose() * Ip * (C2 + C4) +
            H2.transpose() * Gyro_term * H2);
    }

    // ==========================================
    // 7. 局部 -> 全局 转换
    // E_mat 是 全局->局部，因此 E_mat.transpose() 是 局部->全局
    // ==========================================
    ce = 0.5 * L0 * E_mat * Cke * E_mat.transpose();
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

    //double Iy = 0.0, Iz = 0.0, J = 0.0;
    //pSection->Calculate_I(Iy, Iz, J);
    //const double Io = pSection->Io;
    //double Irr = pSection->Irr;
    double Iy = 0.001, Iz = 0.001, J = 0.001;
    double Irr = 0.001;//1.5045055561273500985282118708287;
    double Io = 0.002;

    double EA_L = E * A / L;
    double GJ_L = G * J / L;
    double EIy_L = E * Iy / L;
    double EIz_L = E * Iz / L;

    // 组装局部切线刚度矩阵 kl (7 x 7)
    kl = MatrixXd::Zero(7, 7); // 必须清零初始化

    // 轴向拉压刚度
    kl(0, 0) = EA_L;

    // 扭转刚度 (绕局部 x 轴)
    kl(1, 1) = GJ_L; kl(1, 4) = -GJ_L;
    kl(4, 1) = -GJ_L; kl(4, 4) = GJ_L;

    // 弯曲刚度 - 局部 xz 平面 (绕局部 y 轴旋转)
    kl(2, 2) = 4.0 * EIy_L; kl(2, 5) = 2.0 * EIy_L;
    kl(5, 2) = 2.0 * EIy_L; kl(5, 5) = 4.0 * EIy_L;

    // 弯曲刚度 - 局部 xy 平面 (绕局部 z 轴旋转)
    kl(3, 3) = 4.0 * EIz_L; kl(3, 6) = 2.0 * EIz_L;
    kl(6, 3) = 2.0 * EIz_L; kl(6, 6) = 4.0 * EIz_L;

    // 计算局部内力向量 fl (7 x 1)
    fl = VectorXd::Zero(7);

    // 轴向力
    fl(0) = EA_L * u;

    // 扭矩
    fl(1) = GJ_L * (t11 - t12);
    fl(4) = GJ_L * (t12 - t11);

    // 绕 y 轴的端弯矩
    fl(2) = 2.0 * EIy_L * (2.0 * t21 + t22);
    fl(5) = 2.0 * EIy_L * (t21 + 2.0 * t22);

    // 绕 z 轴的端弯矩
    fl(3) = 2.0 * EIz_L * (2.0 * t31 + t32);
    fl(6) = 2.0 * EIz_L * (t31 + 2.0 * t32);

    m_Stress = fl(0) / A; // 轴向应力
    //const double L2 = L * L;
    //const double L3 = L2 * L;

    //const double dt1 = t11 - t12;
    //const double dt1_sq = dt1 * dt1;

    //// 常用线性组合
    //const double t2_41 = 4.0 * t21 - t22;
    //const double t2_14 = t21 - 4.0 * t22;
    //const double t3_41 = 4.0 * t31 - t32;
    //const double t3_14 = t31 - 4.0 * t32;

    //const double poly_y = 2.0 * t21 * t21 - t21 * t22 + 2.0 * t22 * t22;
    //const double poly_z = 2.0 * t31 * t31 - t31 * t32 + 2.0 * t32 * t32;
    //const double poly_yz = poly_y + poly_z;

    //const double axial_strain_term = L * poly_yz + 30.0 * u;

    //const double N_nonlin_core = 15.0 * Io * dt1_sq + A * L * axial_strain_term;


    //// 计算局部内力向量 fl (7 x 1)
    //fl.resize(7);
    //fl.setZero();
    //const double E_900L = E / (900.0 * L);
    //// 轴力
    //fl(0) = (E / (30.0 * L2)) * N_nonlin_core;

    //// 扭矩
    //fl(1) = dt1 * (30.0 * G * J * L2 + 15.0 * E * Irr * dt1_sq + E * Io * L * axial_strain_term) / (30.0 * L3);
    //fl(4) = -fl(1);

    //// Y轴弯矩（绕 y 轴弯曲，用 Iz）

    //fl(2) = E_900L * (1800.0 * Iz * (2.0 * t21 + t22) + t2_41 * N_nonlin_core);
    //fl(5) = E_900L * (1800.0 * Iz * (t21 + 2.0 * t22) - t2_14 * N_nonlin_core);

    //// Z轴弯矩（绕 z 轴弯曲，用 Iy）
    //fl(3) = E_900L * (1800.0 * Iy * (2.0 * t31 + t32) + t3_41 * N_nonlin_core);
    //fl(6) = E_900L * (1800.0 * Iy * (t31 + 2.0 * t32) - t3_14 * N_nonlin_core);
    //m_Stress = fl(0) / A; // 轴向应力

    //// 计算局部切线刚度矩阵 kl (7 x 7)

    //kl.resize(7, 7);
    //kl.setZero();

    //// 常用乘法系数
    //const double AE_30 = A * E / 30.0;
    //const double AE_L_900 = A * E * L / 900.0;
    //const double EIo_L2 = E * Io / L2;
    //const double EIo_30L = E * Io / (30.0 * L);

    //// 填充上三角 (Upper Triangular Part) 

    //// ----- 第 0 行 (轴向 u) -----
    //kl(0, 0) = (A * E) / L;
    //kl(0, 1) = EIo_L2 * dt1;
    //kl(0, 2) = AE_30 * t2_41;
    //kl(0, 3) = AE_30 * t3_41;
    //kl(0, 4) = -kl(0, 1);
    //kl(0, 5) = -AE_30 * t2_14;
    //kl(0, 6) = -AE_30 * t3_14;

    //// ----- 第 1 行 (扭转 t11) -----
    //kl(1, 1) = (30.0 * G * J * L2 + 45.0 * E * Irr * dt1_sq + E * Io * L * (L * poly_yz + 30.0 * u)) / (30.0 * L3);
    //kl(1, 2) = EIo_30L * dt1 * t2_41;
    //kl(1, 3) = EIo_30L * dt1 * t3_41;
    //kl(1, 4) = -(G * J) / L - (E * (45.0 * Irr * dt1_sq + Io * L * (L * poly_yz + 30.0 * u))) / (30.0 * L3);
    //kl(1, 5) = -EIo_30L * dt1 * t2_14;
    //kl(1, 6) = -EIo_30L * dt1 * t3_14;

    //// ----- 第 2 行 (弯曲 t21) -----
    //kl(2, 2) = (E * (3600.0 * Iz + 60.0 * Io * dt1_sq + A * L * (L * (24.0 * t21 * t21 - 12.0 * t21 * t22 + 9.0 * t22 * t22 + 4.0 * poly_z) + 120.0 * u))) / (900.0 * L);
    //kl(2, 3) = AE_L_900 * t2_41 * t3_41;
    //kl(2, 4) = -kl(1, 2);
    //kl(2, 5) = (E * (1800.0 * Iz - 15.0 * Io * dt1_sq - A * L * (L * (6.0 * t21 * t21 - 18.0 * t21 * t22 + 6.0 * t22 * t22 + poly_z) + 30.0 * u))) / (900.0 * L);
    //kl(2, 6) = -AE_L_900 * t2_41 * t3_14;

    //// ----- 第 3 行 (弯曲 t31) -----
    //kl(3, 3) = (E * (3600.0 * Iy + 60.0 * Io * dt1_sq + A * L * (L * (4.0 * poly_y + 24.0 * t31 * t31 - 12.0 * t31 * t32 + 9.0 * t32 * t32) + 120.0 * u))) / (900.0 * L);
    //kl(3, 4) = -kl(1, 3);
    //kl(3, 5) = -AE_L_900 * t2_14 * t3_41;
    //kl(3, 6) = (E * (1800.0 * Iy - 15.0 * Io * dt1_sq - A * L * (L * (poly_y + 6.0 * (t31 * t31 - 3.0 * t31 * t32 + t32 * t32)) + 30.0 * u))) / (900.0 * L);

    //// ----- 第 4 行 (扭转 t12) -----
    //kl(4, 4) = kl(1, 1);
    //kl(4, 5) = EIo_30L * dt1 * t2_14;
    //kl(4, 6) = EIo_30L * dt1 * t3_14;

    //// ----- 第 5 行 (弯曲 t22) -----
    //kl(5, 5) = (E * (3600.0 * Iz + 60.0 * Io * dt1_sq + A * L * (L * (9.0 * t21 * t21 - 12.0 * t21 * t22 + 24.0 * t22 * t22 + 4.0 * poly_z) + 120.0 * u))) / (900.0 * L);
    //kl(5, 6) = AE_L_900 * t2_14 * t3_14;

    //// ----- 第 6 行 (弯曲 t32) -----
    //kl(6, 6) = (E * (3600.0 * Iy + 60.0 * Io * dt1_sq + A * L * (L * (4.0 * poly_y + 9.0 * t31 * t31 - 12.0 * t31 * t32 + 24.0 * t32 * t32) + 120.0 * u))) / (900.0 * L);


    ////利用 Eigen 的底层机制，把上三角的值自动复制到下三角

    //kl.triangularView<Eigen::Lower>() = kl.transpose();
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

    // 变形后坐标
    def_p1 = init_p1 + ug_p1;
    def_p2 = init_p2 + ug_p2;

    Rg_1 = pNode0->m_Rg;
    Rg_2 = pNode1->m_Rg;

    // 截面方向向量
    q0.normalize(); // 确保初始截面方向向量是单位向量
    Vector3d beam_axis = init_p2 - init_p1;
    beam_axis.normalize();
    if (q0.cross(beam_axis).norm() < 1e-6)
    {
        qDebug().noquote() << QStringLiteral("初始截面向量与轴线重合");
        q0 = Vector3d(0, 0, 1); // 如果重合，选择一个默认的垂直向量
    }

    Vector3d ey(0, 1, 0);
    q1 = Rg_1 * R0 * ey;
    q2 = Rg_2 * R0 * ey;
    q = 0.5 * (q1 + q2);
}

Matrix3d ElementBeam_CR::ComputeLocalFrame(const Vector3d& def_p1, const Vector3d& def_p2,
    const Vector3d& q)
{
    Matrix3d Rr;
    Utility::CR::Calculate_Rr(def_p1, def_p2, q, Rr);
    return Rr;
}

VectorXd ElementBeam_CR::ComputeLocalDeformation(const Vector3d& def_p1, const Vector3d& def_p2, const Matrix3d& Rr, const Matrix3d& Rg_1, const Matrix3d& Rg_2)
{
    // 当前长度与轴向伸长
    double L = (def_p2 - def_p1).norm();
    double u_ = L - L0;

    // 局部旋转矩阵
    Matrix3d R1_ = Rr.transpose() * Rg_1 * R0;
    Matrix3d R2_ = Rr.transpose() * Rg_2 * R0;

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
    MatrixXd& P, MatrixXd& G)
{
    // 组装转换矩阵 E (12x12) 和投影矩阵 P (7x12)、自旋矩阵 G (3x12)
    MatrixXd ET;
    Utility::CR::Assemble_Matrix_E(Rr, ET);
    Utility::CR::Assemble_Matrix_PG(def_p1, def_p2, q1, q2, Rr, P, G);

    VectorXd r = VectorXd::Zero(12);
    Vector3d r1 = (def_p2 - def_p1).normalized();
    r.segment<3>(0) = -r1;
    r.segment<3>(6) = r1;
    Eigen::Matrix<double, 7, 12> Bg;
    Bg.setZero();
    Bg.row(0) = r.transpose();
    Bg.block<6, 12>(1, 0) = P * ET;

    fp = Bg.transpose() * fa;

    K_material = Bg.transpose() * ka * Bg;
}