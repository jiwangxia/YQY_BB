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
    {
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
        return;
    }

    Matrix3d Rg1, Rg2;
    Vector3d q1, q2, q;
    Vector3d currentPoint1, currentPoint2;
    ComputeDeformedState(
        currentPoint1, currentPoint2, Rg1, Rg2, q1, q2, q);
    const Vector3d currentChord = currentPoint2 - currentPoint1;
    const double currentLength = currentChord.norm();
    if (currentLength <= 1.0e-12 || L0 <= 1.0e-12)
        throw std::runtime_error("ElementBeam_CR has zero dynamic integration length");

    const Matrix3d corotationalFrame =
        ComputeLocalFrame(currentPoint1, currentPoint2, q);
    const VectorXd localDeformation = ComputeLocalDeformation(
        currentPoint1, currentPoint2, corotationalFrame, Rg1, Rg2);
    const VectorXd localEndRotation = localDeformation.segment<6>(1);

    MatrixXd deformationProjection;
    MatrixXd spinProjection;
    Utility::CR::Assemble_Matrix_PG(currentPoint1, currentPoint2,
        q1, q2, corotationalFrame, deformationProjection, spinProjection);

    // Q maps global nodal vectors to the current corotational frame.
    MatrixXd Q;
    Utility::CR::Assemble_Matrix_E(corotationalFrame, Q);

    VectorXd globalVelocity = VectorXd::Zero(12);
    VectorXd globalAcceleration = VectorXd::Zero(12);
    for (int component = 0; component < 6; ++component)
    {
        globalVelocity(component) = pNode0->m_Velocity[component];
        globalVelocity(component + 6) = pNode1->m_Velocity[component];
        globalAcceleration(component) = pNode0->m_Acceleration[component];
        globalAcceleration(component + 6) = pNode1->m_Acceleration[component];
    }
    const VectorXd localVelocity = Q * globalVelocity;
    const VectorXd localAcceleration = Q * globalAcceleration;

    const double lineMass = pMaterial->m_Density * pSection->m_Area;
    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);

    Matrix3d materialRotaryInertia = Matrix3d::Zero();
    materialRotaryInertia(0, 0) = pMaterial->m_Density * (Iy + Iz);
    materialRotaryInertia(1, 1) = pMaterial->m_Density * Iz;
    materialRotaryInertia(2, 2) = pMaterial->m_Density * Iy;

    if ((pSection->m_MassInertiaPerLength.array() > 0.0).all())
        materialRotaryInertia.diagonal() =
            pSection->m_MassInertiaPerLength;

    MatrixXd localMass = MatrixXd::Zero(12, 12);
    MatrixXd localVelocityTangent = MatrixXd::Zero(12, 12);
    VectorXd localInertiaForce = VectorXd::Zero(12);

    VectorXd chordGradient = VectorXd::Zero(12);
    const Vector3d chordDirection = currentChord / currentLength;
    chordGradient.segment<3>(0) = -chordDirection;
    chordGradient.segment<3>(6) = chordDirection;

    MatrixXd lengthDerivative1 = MatrixXd::Zero(3, 12);
    lengthDerivative1(1, 1) = -1.0;
    lengthDerivative1(2, 2) = -1.0;
    lengthDerivative1(1, 7) = 1.0;
    lengthDerivative1(2, 8) = 1.0;

    MatrixXd lengthDerivative2 = MatrixXd::Zero(3, 12);
    lengthDerivative2(1, 2) = 1.0;
    lengthDerivative2(2, 1) = -1.0;
    lengthDerivative2(1, 8) = -1.0;
    lengthDerivative2(2, 7) = 1.0;

    MatrixXd velocitySpin = MatrixXd::Zero(12, 3);
    Matrix3d skew;
    for (int block = 0; block < 4; ++block)
    {
        Utility::CR::SkewSymmetric(localVelocity.segment<3>(3 * block), skew);
        velocitySpin.block<3, 3>(3 * block, 0) = skew;
    }

    VectorXd localChordDof = VectorXd::Zero(12);
    localChordDof(0) = -1.0;
    localChordDof(6) = 1.0;

    const double gaussPoints[2] = {
        -1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)
    };
    for (double coordinate : gaussPoints)
    {
        double N[6];
        N[0] = 0.5 * (1.0 - coordinate);
        N[1] = 0.5 * (1.0 + coordinate);
        N[2] = N[0] * N[0] * N[1] * L0;
        N[3] = -N[0] * N[1] * N[1] * L0;
        N[4] = -0.25 - 0.5 * coordinate
            + 0.75 * coordinate * coordinate;
        N[5] = -0.25 + 0.5 * coordinate
            + 0.75 * coordinate * coordinate;

        MatrixXd displacementInterpolation = MatrixXd::Zero(3, 6);
        displacementInterpolation(1, 2) = N[2];
        displacementInterpolation(1, 5) = N[3];
        displacementInterpolation(2, 1) = -N[2];
        displacementInterpolation(2, 4) = -N[3];

        MatrixXd rotationInterpolation = MatrixXd::Zero(3, 6);
        rotationInterpolation(0, 0) = N[0];
        rotationInterpolation(0, 3) = N[1];
        rotationInterpolation(1, 1) = N[4];
        rotationInterpolation(1, 4) = N[5];
        rotationInterpolation(2, 2) = N[4];
        rotationInterpolation(2, 5) = N[5];

        const Vector3d interpolatedDisplacement =
            displacementInterpolation * localEndRotation;
        const Vector3d interpolatedRotation =
            rotationInterpolation * localEndRotation;

        MatrixXd nodalTranslation = MatrixXd::Zero(3, 12);
        nodalTranslation.block<3, 3>(0, 0) = N[0] * Matrix3d::Identity();
        nodalTranslation.block<3, 3>(0, 6) = N[1] * Matrix3d::Identity();

        Matrix3d displacementSkew;
        Utility::CR::SkewSymmetric(interpolatedDisplacement, displacementSkew);

        const MatrixXd H1 = nodalTranslation
            + displacementInterpolation * deformationProjection
            - displacementSkew * spinProjection;
        const MatrixXd H2 = rotationInterpolation * deformationProjection + spinProjection;

        const double inverseCurrentLengthSquared = 1.0 / (currentLength * currentLength);
        const MatrixXd normalizedLengthDerivative1 = (N[2] + N[3]) * inverseCurrentLengthSquared
            * lengthDerivative1;
        const MatrixXd normalizedLengthDerivative2 = (N[4] + N[5] - 1.0) * inverseCurrentLengthSquared
            * lengthDerivative2;

        const Vector3d localDeformationVelocity = displacementInterpolation * deformationProjection
            * localVelocity;
        Matrix3d localDeformationVelocitySkew;
        Utility::CR::SkewSymmetric(localDeformationVelocity, localDeformationVelocitySkew);
        const double axialVelocity = chordGradient.dot(globalVelocity);
        const MatrixXd dH1 = normalizedLengthDerivative1 * axialVelocity
            - localDeformationVelocitySkew * spinProjection;
        const MatrixXd dH2 = normalizedLengthDerivative2 * axialVelocity;

        const Vector3d frameAngularVelocity = spinProjection * localVelocity;
        Matrix3d frameAngularVelocitySkew;
        Utility::CR::SkewSymmetric(frameAngularVelocity, frameAngularVelocitySkew);
        MatrixXd frameSpin = MatrixXd::Zero(12, 12);
        for (int block = 0; block < 4; ++block)
            frameSpin.block<3, 3>(3 * block, 3 * block) = frameAngularVelocitySkew;

        const MatrixXd C1 = frameAngularVelocitySkew * H1 + dH1 - H1 * frameSpin;
        const MatrixXd C2 = frameAngularVelocitySkew * H2 + dH2 - H2 * frameSpin;

        const Vector3d sectionVelocity = H1 * localVelocity;
        const Vector3d sectionAngularVelocity = H2 * localVelocity;
        Matrix3d sectionVelocitySkew;
        Matrix3d sectionAngularVelocitySkew;
        Utility::CR::SkewSymmetric(sectionVelocity, sectionVelocitySkew);
        Utility::CR::SkewSymmetric(sectionAngularVelocity, sectionAngularVelocitySkew);

        const MatrixXd C3 = -sectionVelocitySkew * spinProjection 
            + normalizedLengthDerivative1 * localVelocity * localChordDof.transpose()
            + frameAngularVelocitySkew * displacementInterpolation * deformationProjection
            + H1 * velocitySpin * spinProjection;
        const MatrixXd C4 = -sectionAngularVelocitySkew * spinProjection
            + normalizedLengthDerivative2 * localVelocity * localChordDof.transpose()
            + H2 * velocitySpin * spinProjection;

        Matrix3d interpolatedRotationSkew;
        Utility::CR::SkewSymmetric(interpolatedRotation, interpolatedRotationSkew);

        const Matrix3d sectionDeformationRotation = Matrix3d::Identity() + interpolatedRotationSkew;
        const Matrix3d spatialRotaryInertia = sectionDeformationRotation * materialRotaryInertia
            * sectionDeformationRotation.transpose();

        const Vector3d sectionAcceleration = H1 * localAcceleration + C1 * localVelocity;
        const Vector3d sectionAngularAcceleration = H2 * localAcceleration + C2 * localVelocity;

        localMass += H1.transpose() * lineMass * H1 + H2.transpose() * spatialRotaryInertia * H2;
        localInertiaForce += H1.transpose() * (lineMass * sectionAcceleration)
            + H2.transpose() * (spatialRotaryInertia * sectionAngularAcceleration
                    + sectionAngularVelocity.cross(spatialRotaryInertia * sectionAngularVelocity));
        localVelocityTangent += H1.transpose() * lineMass * (C1 + C3)
            + H2.transpose() * spatialRotaryInertia * (C2 + C4)
            + H2.transpose() * (sectionAngularVelocitySkew * spatialRotaryInertia
            - spatialRotaryInertia * sectionAngularVelocitySkew) * H2;
    }

    const double integrationScale = 0.5 * L0;
    if (massMatrix)
        *massMatrix = integrationScale * Q.transpose() * localMass * Q;
    if (inertiaForce)
        *inertiaForce = integrationScale * Q.transpose() * localInertiaForce;
    if (velocityTangent)
        *velocityTangent = integrationScale * Q.transpose() * localVelocityTangent * Q;
}

void ElementBeam_CR::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
    EvaluateDynamicSystem(&me, nullptr, nullptr);
    m_me = me;
    return;

    if (L0 <= 0.0)
        Get_L0();

    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty ? pProperty->m_pMaterial.lock() : nullptr;
    auto pSection = pProperty ? pProperty->m_pSection.lock() : nullptr;
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pMaterial || !pSection || !pNode0 || !pNode1)
        throw std::runtime_error("ElementBeam_CR mass data is incomplete");

    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);
    const double density = pMaterial->m_Density;
    const double lineDensity = density * pSection->m_Area;
    Matrix3d materialInertia = Matrix3d::Zero();
    materialInertia(0, 0) = density * (Iy + Iz);
    materialInertia(1, 1) = density * Iz;
    materialInertia(2, 2) = density * Iy;
    if ((pSection->m_MassInertiaPerLength.array() > 0.0).all())
        materialInertia.diagonal() = pSection->m_MassInertiaPerLength;

    const Matrix3d sectionRotation0 = pNode0->m_Rg * R0;
    const Matrix3d sectionRotation1 = pNode1->m_Rg * R0;
    Vector3d relativeRotation;
    Utility::CR::Extract_RotationVector(
        sectionRotation1 * sectionRotation0.transpose(), relativeRotation);

    me = MatrixXd::Zero(12, 12);
    const double gaussPoints[2] = {-1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)};
    for (double coordinate : gaussPoints)
    {
        const double N[2] = {0.5 * (1.0 - coordinate), 0.5 * (1.0 + coordinate)};
        Matrix3d relativeIncrement;
        Utility::CR::Calculate_RotationMatrix(N[1] * relativeRotation, relativeIncrement);
        const Matrix3d sectionRotation = relativeIncrement * sectionRotation0;
        const Matrix3d spatialInertia = sectionRotation * materialInertia * sectionRotation.transpose();

        for (int a = 0; a < 2; ++a)
        {
            for (int b = 0; b < 2; ++b)
            {
                const double weight = 0.5 * L0 * N[a] * N[b];
                me.block<3, 3>(6 * a, 6 * b) += weight * lineDensity * Matrix3d::Identity();
                me.block<3, 3>(6 * a + 3, 6 * b + 3) += weight * spatialInertia;
            }
        }
    }
    m_me = me;

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
    return;

    MatrixXd mass;
    Get_me_Consistent(mass);

    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty ? pProperty->m_pMaterial.lock() : nullptr;
    auto pSection = pProperty ? pProperty->m_pSection.lock() : nullptr;
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pMaterial || !pSection || !pNode0 || !pNode1)
        throw std::runtime_error("ElementBeam_CR inertia data is incomplete");

    VectorXd acceleration = VectorXd::Zero(12);
    for (int i = 0; i < 6; ++i)
    {
        acceleration(i) = pNode0->m_Acceleration[i];
        acceleration(i + 6) = pNode1->m_Acceleration[i];
    }
    inertiaForce = mass * acceleration;

    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);
    Matrix3d materialInertia = Matrix3d::Zero();
    materialInertia(0, 0) = pMaterial->m_Density * (Iy + Iz);
    materialInertia(1, 1) = pMaterial->m_Density * Iz;
    materialInertia(2, 2) = pMaterial->m_Density * Iy;
    if ((pSection->m_MassInertiaPerLength.array() > 0.0).all())
        materialInertia.diagonal() = pSection->m_MassInertiaPerLength;

    const Matrix3d sectionRotation0 = pNode0->m_Rg * R0;
    const Matrix3d sectionRotation1 = pNode1->m_Rg * R0;
    Vector3d relativeRotation;
    Utility::CR::Extract_RotationVector(
        sectionRotation1 * sectionRotation0.transpose(), relativeRotation);

    Vector3d angularVelocity[2];
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto node = m_pNode[nodeIndex].lock();
        for (int component = 0; component < 3; ++component)
            angularVelocity[nodeIndex](component) = node->m_Velocity[component + 3];
    }

    const double gaussPoints[2] = {
        -1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)
    };
    for (double coordinate : gaussPoints)
    {
        const double N[2] = {
            0.5 * (1.0 - coordinate), 0.5 * (1.0 + coordinate)
        };
        Matrix3d relativeIncrement;
        Utility::CR::Calculate_RotationMatrix(
            N[1] * relativeRotation, relativeIncrement);
        const Matrix3d sectionRotation =
            relativeIncrement * sectionRotation0;
        const Matrix3d spatialInertia =
            sectionRotation * materialInertia * sectionRotation.transpose();
        const Vector3d omega =
            N[0] * angularVelocity[0] + N[1] * angularVelocity[1];
        const Vector3d gyroscopicMoment =
            omega.cross(spatialInertia * omega);

        for (int a = 0; a < 2; ++a)
            inertiaForce.segment<3>(6 * a + 3) +=
                0.5 * L0 * N[a] * gyroscopicMoment;
    }
}

void ElementBeam_CR::Get_GyroscopicMatrix(MatrixXd& gyroscopicMatrix)
{
    EvaluateDynamicSystem(nullptr, nullptr, &gyroscopicMatrix);
    return;

    if (L0 <= 0.0)
        Get_L0();

    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty ? pProperty->m_pMaterial.lock() : nullptr;
    auto pSection = pProperty ? pProperty->m_pSection.lock() : nullptr;
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pMaterial || !pSection || !pNode0 || !pNode1)
        throw std::runtime_error("ElementBeam_CR gyroscopic data is incomplete");

    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);
    Matrix3d materialInertia = Matrix3d::Zero();
    materialInertia(0, 0) = pMaterial->m_Density * (Iy + Iz);
    materialInertia(1, 1) = pMaterial->m_Density * Iz;
    materialInertia(2, 2) = pMaterial->m_Density * Iy;
    if ((pSection->m_MassInertiaPerLength.array() > 0.0).all())
        materialInertia.diagonal() = pSection->m_MassInertiaPerLength;

    const Matrix3d sectionRotation0 = pNode0->m_Rg * R0;
    const Matrix3d sectionRotation1 = pNode1->m_Rg * R0;
    Vector3d relativeRotation;
    Utility::CR::Extract_RotationVector(
        sectionRotation1 * sectionRotation0.transpose(), relativeRotation);

    Vector3d angularVelocity[2];
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto node = m_pNode[nodeIndex].lock();
        for (int component = 0; component < 3; ++component)
            angularVelocity[nodeIndex](component) = node->m_Velocity[component + 3];
    }

    gyroscopicMatrix = MatrixXd::Zero(12, 12);
    const double gaussPoints[2] = {
        -1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)
    };
    for (double coordinate : gaussPoints)
    {
        const double N[2] = {
            0.5 * (1.0 - coordinate), 0.5 * (1.0 + coordinate)
        };
        Matrix3d relativeIncrement;
        Utility::CR::Calculate_RotationMatrix(
            N[1] * relativeRotation, relativeIncrement);
        const Matrix3d sectionRotation =
            relativeIncrement * sectionRotation0;
        const Matrix3d spatialInertia =
            sectionRotation * materialInertia * sectionRotation.transpose();
        const Vector3d omega =
            N[0] * angularVelocity[0] + N[1] * angularVelocity[1];

        Matrix3d omegaSkew;
        Matrix3d angularMomentumSkew;
        Utility::CR::SkewSymmetric(omega, omegaSkew);
        Utility::CR::SkewSymmetric(
            spatialInertia * omega, angularMomentumSkew);
        const Matrix3d gyroscopicTangent =
            omegaSkew * spatialInertia - angularMomentumSkew;

        for (int a = 0; a < 2; ++a)
        {
            for (int b = 0; b < 2; ++b)
            {
                gyroscopicMatrix.block<3, 3>(
                    6 * a + 3, 6 * b + 3) +=
                    0.5 * L0 * N[a] * N[b] * gyroscopicTangent;
            }
        }
    }
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

void ElementBeam_CR::Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce)
{
    // The nonlinear velocity tangent is a kinematic contribution of the
    // inertial residual, not optional material damping.  Keep this legacy
    // entry point consistent with the matrix assembled by Newmark.
    (void)damping;
    EvaluateDynamicSystem(nullptr, nullptr, &ce);
    return;

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
