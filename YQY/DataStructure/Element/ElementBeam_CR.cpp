#include "ElementBeam_CR.h"

ElementBeam_CR::ElementBeam_CR()
{
    m_pNode.resize(2);
}

void ElementBeam_CR::Get_ke(MatrixXd& ke)
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
    MatrixXd consistentMass;
    Get_me_Consistent(consistentMass);

    me = MatrixXd::Zero(12, 12);
    for (int i = 0; i < consistentMass.rows(); ++i)
        me(i, i) = consistentMass.row(i).sum();
    m_me = me;
}

void ElementBeam_CR::EvaluateDynamicSystem(MatrixXd* massMatrix,
    VectorXd* inertiaForce, MatrixXd* velocityTangent,
    MatrixXd* configurationTangent)
{
    if (L0 <= 0.0)
        Get_L0();

    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty ? pProperty->m_pMaterial.lock() : nullptr;
    auto pSection = pProperty ? pProperty->m_pSection.lock() : nullptr;
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pMaterial || !pSection || !pNode0 || !pNode1)
        throw std::runtime_error("ElementBeam_CR dynamic data is incomplete");

    // Le et al. (2012), spatial-spin formulation, Eqs. (34) and (53).
    // All four dynamic quantities are integrated in one pass so that the
    // residual and its three tangent parts use exactly the same Gauss-point
    // rotations, velocities and accelerations.
    const double lineMass = pMaterial->m_Density * pSection->m_Area;
    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);

    Matrix3d materialRotaryInertia = Matrix3d::Zero();
    materialRotaryInertia(0, 0) =
        pMaterial->m_Density * (Iy + Iz);
    materialRotaryInertia(1, 1) = pMaterial->m_Density * Iz;
    materialRotaryInertia(2, 2) = pMaterial->m_Density * Iy;
    if ((pSection->m_MassInertiaPerLength.array() > 0.0).all())
        materialRotaryInertia.diagonal() =
            pSection->m_MassInertiaPerLength;

    MatrixXd assembledMass = MatrixXd::Zero(12, 12);
    MatrixXd assembledGyroscopic = MatrixXd::Zero(12, 12);
    MatrixXd assembledCentrifugal = MatrixXd::Zero(12, 12);
    VectorXd assembledInertiaForce = VectorXd::Zero(12);

    const Matrix3d previousSectionRotation0 = pNode0->m_Rg_n * R0;
    const Matrix3d previousSectionRotation1 = pNode1->m_Rg_n * R0;
    Vector3d previousRelativeRotation;
    Utility::CR::Extract_RotationVector(
        previousSectionRotation1
            * previousSectionRotation0.transpose(),
        previousRelativeRotation);

    Vector3d nodalVelocity[2];
    Vector3d nodalAcceleration[2];
    Vector3d nodalAngularVelocity[2];
    Vector3d nodalAngularAcceleration[2];
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto& node = nodeIndex == 0 ? pNode0 : pNode1;
        for (int component = 0; component < 3; ++component)
        {
            nodalVelocity[nodeIndex](component) =
                node->m_Velocity[component];
            nodalAcceleration[nodeIndex](component) =
                node->m_Acceleration[component];
            nodalAngularVelocity[nodeIndex](component) =
                node->m_Velocity[component + 3];
            nodalAngularAcceleration[nodeIndex](component) =
                node->m_Acceleration[component + 3];
        }
    }

    const double gaussPoints[2] = {
        -1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)
    };
    for (double coordinate : gaussPoints)
    {
        const double N[2] = {
            0.5 * (1.0 - coordinate),
            0.5 * (1.0 + coordinate)
        };

        Matrix3d previousRelativeIncrement;
        Utility::CR::Calculate_RotationMatrix(
            N[1] * previousRelativeRotation,
            previousRelativeIncrement);
        const Matrix3d previousGaussRotation =
            previousRelativeIncrement * previousSectionRotation0;

        const Vector3d incrementalGaussRotation =
            N[0] * pNode0->m_StepRotation
            + N[1] * pNode1->m_StepRotation;
        Matrix3d incrementalGaussRotationMatrix;
        Utility::CR::Calculate_RotationMatrix(
            incrementalGaussRotation,
            incrementalGaussRotationMatrix);
        const Matrix3d currentGaussRotation =
            incrementalGaussRotationMatrix * previousGaussRotation;

        Matrix3d inverseSpatialSpin;
        Utility::CR::Calculate_Ts_Inv(
            incrementalGaussRotation, inverseSpatialSpin);

        const Vector3d translationalAcceleration =
            N[0] * nodalAcceleration[0]
            + N[1] * nodalAcceleration[1];
        Vector3d materialAngularVelocity;
        Vector3d materialAngularAcceleration;
        // Simo and Vu-Quoc interpolation discussed after Eqs. (56)-(57):
        // interpolate the current nodal spatial quantities, then map the
        // Gauss-point values to the material frame.  Le et al. report the
        // same response as the stored Gauss-point update, without adding
        // element-history variables.
        const Vector3d spatialAngularVelocity =
            N[0] * nodalAngularVelocity[0]
            + N[1] * nodalAngularVelocity[1];
        const Vector3d spatialAngularAcceleration =
            N[0] * nodalAngularAcceleration[0]
            + N[1] * nodalAngularAcceleration[1];
        materialAngularVelocity =
            currentGaussRotation.transpose()
            * spatialAngularVelocity;
        materialAngularAcceleration =
            currentGaussRotation.transpose()
            * spatialAngularAcceleration;

        const Vector3d materialAngularMomentum =
            materialRotaryInertia * materialAngularVelocity;
        const Vector3d materialDynamicMoment =
            materialRotaryInertia * materialAngularAcceleration
            + materialAngularVelocity.cross(
                materialAngularMomentum);
        const Vector3d spatialDynamicMoment =
            currentGaussRotation * materialDynamicMoment;

        Matrix3d angularVelocitySkew;
        Matrix3d dynamicMomentSkew;
        Utility::CR::SkewSymmetric(
            materialAngularVelocity, angularVelocitySkew);
        Utility::CR::SkewSymmetric(
            materialDynamicMoment, dynamicMomentSkew);

        const Matrix3d rotationalMass =
            currentGaussRotation * materialRotaryInertia
            * previousGaussRotation.transpose()
            * inverseSpatialSpin;
        const Matrix3d rotationalGyroscopic =
            currentGaussRotation
            * (angularVelocitySkew * materialRotaryInertia
                - materialRotaryInertia * angularVelocitySkew)
            * previousGaussRotation.transpose()
            * inverseSpatialSpin;
        const Matrix3d rotationalCentrifugal =
            -currentGaussRotation * dynamicMomentSkew;

        for (int a = 0; a < 2; ++a)
        {
            assembledInertiaForce.segment<3>(6 * a) +=
                N[a] * lineMass * translationalAcceleration;
            assembledInertiaForce.segment<3>(6 * a + 3) +=
                N[a] * spatialDynamicMoment;
            for (int b = 0; b < 2; ++b)
            {
                const double interpolationWeight = N[a] * N[b];
                assembledMass.block<3, 3>(6 * a, 6 * b) +=
                    interpolationWeight * lineMass
                    * Matrix3d::Identity();
                assembledMass.block<3, 3>(
                    6 * a + 3, 6 * b + 3) +=
                    interpolationWeight * rotationalMass;
                assembledGyroscopic.block<3, 3>(
                    6 * a + 3, 6 * b + 3) +=
                    interpolationWeight * rotationalGyroscopic;
                assembledCentrifugal.block<3, 3>(
                    6 * a + 3, 6 * b + 3) +=
                    interpolationWeight * rotationalCentrifugal;
            }
        }
    }

    const double integrationScale = 0.5 * L0;
    if (massMatrix)
        *massMatrix = integrationScale * assembledMass;
    if (inertiaForce)
        *inertiaForce = integrationScale * assembledInertiaForce;
    if (velocityTangent)
        *velocityTangent =
            integrationScale * assembledGyroscopic;
    if (configurationTangent)
        *configurationTangent =
            integrationScale * assembledCentrifugal;

}

void ElementBeam_CR::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
    EvaluateDynamicSystem(&me, nullptr, nullptr);
    m_me = me;
}

void ElementBeam_CR::GetDynamicContributions(
    MatrixXd& massMatrix,
    VectorXd& inertiaForce,
    MatrixXd& gyroscopicMatrix,
    MatrixXd& centrifugalMatrix)
{
    EvaluateDynamicSystem(
        &massMatrix, &inertiaForce, &gyroscopicMatrix, &centrifugalMatrix);
    m_me = massMatrix;
}

void ElementBeam_CR::Get_InertiaForce(VectorXd& inertiaForce)
{
    EvaluateDynamicSystem(nullptr, &inertiaForce, nullptr);
}

void ElementBeam_CR::Get_GyroscopicMatrix(MatrixXd& gyroscopicMatrix)
{
    EvaluateDynamicSystem(nullptr, nullptr, &gyroscopicMatrix);
}

void ElementBeam_CR::Get_CentrifugalMatrix(MatrixXd& centrifugalMatrix)
{
    EvaluateDynamicSystem(
        nullptr, nullptr, nullptr, &centrifugalMatrix);
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

void ElementBeam_CR::Assemble(
    const std::vector<double>& damping,
    _OUT MatrixXd& ce)
{
    // 非线性速度切线属于惯性残量的运动学贡献，不是可选材料阻尼。
    // 保留该旧入口，并与 Newmark 使用的动力切线保持一致。
    (void)damping;
    EvaluateDynamicSystem(nullptr, nullptr, &ce);
}

void ElementBeam_CR::Get_kl(const VectorXd& pl, const double& L, _OUT MatrixXd& kl, _OUT VectorXd& fl)
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

    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);

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
    fl(0) = EA_L * u + m_InitStress * A;

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





    //kl.setZero(7, 7);
    //fl.setZero(7);

    //if (pl.size() != 7)
    //{
    //    qDebug().noquote() << QStringLiteral("ElementBeam_CR::Get_kl: pl 大小不为 7");
    //    return;
    //}

    //if (std::abs(L) < 1e-12)
    //{
    //    qDebug().noquote() << QStringLiteral("ElementBeam_CR::Get_kl: 单元长度过小");
    //    return;
    //}

    //// 提取 Pl 向量的分量
    //const double u = pl(0);
    //const double t11 = pl(1);
    //const double t21 = pl(2);
    //const double t31 = pl(3);
    //const double t12 = pl(4);
    //const double t22 = pl(5);
    //const double t32 = pl(6);

    //auto pProperty = m_pProperty.lock();
    //auto pMaterial = pProperty->m_pMaterial.lock();
    //auto pSection = pProperty->m_pSection.lock();

    //double A = pSection->m_Area;
    //double E = pMaterial->m_Young;
    //double G = E / (2. * (1 + pMaterial->m_Poisson));

    //double Iy = 0.0, Iz = 0.0, J = 0.0;
    //pSection->Calculate_I(Iy, Iz, J);
    //const double Io = pSection->Io;
    //double Irr = pSection->Irr;

    //double L0 = L;
    //double L2 = L0 * L0;
    //double L3 = L2 * L0;

    //kl(0, 0) = (A * E) / L0;

    //kl(1, 1) = (30 * G * J * L2 + 45 * E * Irr * (t11 - t12) * (t11 - t12) +
    //    E * Io * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u)) / (30 * L3);
    //kl(2, 2) = (E * (3600 * Iz + 60 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (24 * t21 * t21 - 12 * t21 * t22 + 9 * t22 * t22 + 8 * t31 * t31 - 4 * t31 * t32 +
    //        8 * t32 * t32) + 120 * u))) / (900 * L0);

    //kl(3, 3) = (E * (3600 * Iy + 60 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (8 * t21 * t21 - 4 * t21 * t22 + 8 * t22 * t22 + 24 * t31 * t31 - 12 * t31 * t32 +
    //        9 * t32 * t32) + 120 * u))) / (900 * L0);

    //kl(4, 4) = kl(1, 1);
    //kl(5, 5) = (E * (3600 * Iz + 60 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (9 * t21 * t21 - 12 * t21 * t22 + 24 * t22 * t22 + 8 * t31 * t31 - 4 * t31 * t32 +
    //        8 * t32 * t32) + 120 * u))) / (900 * L0);

    //kl(6, 6) = (E * (3600 * Iy + 60 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (8 * t21 * t21 - 4 * t21 * t22 + 8 * t22 * t22 + 9 * t31 * t31 - 12 * t31 * t32 +
    //        24 * t32 * t32) + 120 * u))) / (900 * L0);

    //kl(0, 1) = (E * Io * (t11 - t12)) / L2;
    //kl(1, 0) = kl(0, 1);

    //kl(0, 2) = 1. / 30 * A * E * (4 * t21 - t22);
    //kl(2, 0) = kl(0, 2);

    //kl(0, 3) = 1. / 30 * A * E * (4 * t31 - t32);
    //kl(3, 0) = kl(0, 3);

    //kl(0, 4) = (E * Io * (t12 - t11)) / L2;
    //kl(4, 0) = kl(0, 4);
    //kl(0, 5) = -(1. / 30) * A * E * (t21 - 4 * t22);
    //kl(5, 0) = kl(0, 5);

    //kl(0, 6) = -(1. / 30) * A * E * (t31 - 4 * t32);
    //kl(6, 0) = kl(0, 6);
    //kl(1, 2) = (E * Io * (t11 - t12) * (4 * t21 - t22)) / (30 * L0);
    //kl(2, 1) = kl(1, 2);

    //kl(1, 3) = (E * Io * (t11 - t12) * (4 * t31 - t32)) / (30 * L0);
    //kl(3, 1) = kl(1, 3);
    //kl(1, 4) = -((G * J) / L0) - (E * (45 * Irr * (t11 - t12) * (t11 - t12) +
    //    Io * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u))) / (30 * L3);
    //kl(4, 1) = kl(1, 4);

    //kl(1, 5) = -((E * Io * (t11 - t12) * (t21 - 4 * t22)) / (30 * L0));
    //kl(5, 1) = kl(1, 5);

    //kl(1, 6) = -((E * Io * (t11 - t12) * (t31 - 4 * t32)) / (30 * L0));
    //kl(6, 1) = kl(1, 6);
    //kl(2, 3) = 1. / 900 * A * E * L0 * (4 * t21 - t22) * (4 * t31 - t32);
    //kl(3, 2) = kl(2, 3);

    //kl(2, 4) = -((E * Io * (t11 - t12) * (4 * t21 - t22)) / (30 * L0));
    //kl(4, 2) = kl(2, 4);

    //kl(2, 5) = (E * (1800 * Iz - 15 * Io * (t11 - t12) * (t11 - t12) -
    //    A * L0 * (L0 * (6 * t21 * t21 - 18 * t21 * t22 + 6 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u))) / (900 * L0);
    //kl(5, 2) = kl(2, 5);

    //kl(2, 6) = -(1. / 900) * A * E * L0 * (4 * t21 - t22) * (t31 - 4 * t32);
    //kl(6, 2) = kl(2, 6);

    //kl(3, 4) = -((E * Io * (t11 - t12) * (4 * t31 - t32)) / (30 * L0));
    //kl(4, 3) = kl(3, 4);

    //kl(3, 5) = -(1. / 900) * A * E * L0 * (t21 - 4 * t22) * (4 * t31 - t32);
    //kl(5, 3) = kl(3, 5);

    //kl(3, 6) = (E * (1800 * Iy - 15 * Io * (t11 - t12) * (t11 - t12) -
    //    A * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 +
    //        6 * (t31 * t31 - 3 * t31 * t32 + t32 * t32)) + 30 * u))) / (900 * L0);
    //kl(6, 3) = kl(3, 6);

    //kl(4, 5) = (E * Io * (t11 - t12) * (t21 - 4 * t22)) / (30 * L0);
    //kl(5, 4) = kl(4, 5);

    //kl(4, 6) = (E * Io * (t11 - t12) * (t31 - 4 * t32)) / (30 * L0);
    //kl(6, 4) = kl(4, 6);

    //kl(5, 6) = 1. / 900 * A * E * L0 * (t21 - 4 * t22) * (t31 - 4 * t32);
    //kl(6, 5) = kl(5, 6);


    ////内力向量计算:
    //fl(0) = (E * (15 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u))) / (30 * L2);

    //fl(1) = ((t11 - t12) * (30 * G * J * L2 + 15 * E * Irr * (t11 - t12) * (t11 - t12) +
    //    E * Io * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u))) / (30 * L3);

    //fl(2) = (E * (1800 * Iz * (2 * t21 + t22) + (4 * t21 - t22) * (15 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u)))) / (900 * L0);

    //fl(3) = (E * (1800 * Iy * (2 * t31 + t32) + (4 * t31 - t32) * (15 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u)))) / (900 * L0);

    //fl(4) = -fl(1);

    //fl(5) = (E * (1800 * Iz * (t21 + 2 * t22) - (t21 - 4 * t22) * (15 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u)))) / (900 * L0);

    //fl(6) = (E * (1800 * Iy * (t31 + 2 * t32) - (t31 - 4 * t32) * (15 * Io * (t11 - t12) * (t11 - t12) +
    //    A * L0 * (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 +
    //        2 * t32 * t32) + 30 * u)))) / (900 * L0);

    //m_Stress = fl(0) / A;
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
