#include "ElementTruss.h"

ElementTruss::ElementTruss()
{
    m_pNode.resize(2);
}

void ElementTruss::Get_ke(MatrixXd& ke)
{
    auto pProperty = m_pProperty.lock();
    auto pSection = pProperty->m_pSection.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();

    double E = pMaterial->m_Young;
    double A = pSection->m_Area;

    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();

    if (pNode0 == nullptr || pNode1 == nullptr)
    {
        qDebug().noquote() << QStringLiteral("Error: ElementTruss 节点指针为空");
        return;
    }

    //Get_L0();

    // 计算当前变形后的方向向量 (考虑位移)
    double dx_current = pNode1->m_X + pNode1->m_Displacement[0] - pNode0->m_X - pNode0->m_Displacement[0];
    double dy_current = pNode1->m_Y + pNode1->m_Displacement[1] - pNode0->m_Y - pNode0->m_Displacement[1];
    double dz_current = pNode1->m_Z + pNode1->m_Displacement[2] - pNode0->m_Z - pNode0->m_Displacement[2];

    // 当前长度
    double length_current = sqrt(dx_current * dx_current + dy_current * dy_current + dz_current * dz_current);

    // 当前方向余弦
    double dirCos_x = dx_current / length_current;
    double dirCos_y = dy_current / length_current;
    double dirCos_z = dz_current / length_current;

    // 应变-位移变换矩阵 B = [-l, -m, -n, l, m, n]
    VectorXd B_matrix = VectorXd::Zero(6);
    B_matrix << -dirCos_x, -dirCos_y, -dirCos_z, dirCos_x, dirCos_y, dirCos_z;

    // 选择应变公式: true = 对数应变(体积不变), false = 工程应变
    bool bUseLogStrain = true;

    if (!bUseLogStrain)
    {
        // ===== 工程应变公式 (Engineering Strain) =====
        // ε = (L - L0) / L0
        double materialStiffness = E * A / L0;
        ke = B_matrix * B_matrix.transpose() * materialStiffness;

        double strain = (length_current - L0) / L0;
        m_Stress = E * strain + m_InitStress;
        double axialForce = m_Stress * A;

        m_inforce = B_matrix * axialForce;

        // 几何刚度矩阵
        if (0.0 != m_Stress)
        {
            Matrix3d I = Matrix3d::Identity();
            Vector3d directionVector;
            directionVector << dirCos_x, dirCos_y, dirCos_z;

            double geometricStiffCoeff = A * m_Stress / length_current;
            Matrix3d Kg_block = geometricStiffCoeff * (I - directionVector * directionVector.transpose());

            ke.block<3, 3>(0, 0) += Kg_block;
            ke.block<3, 3>(3, 0) -= Kg_block;
            ke.block<3, 3>(0, 3) -= Kg_block;
            ke.block<3, 3>(3, 3) += Kg_block;
        }
    }
    else
    {
        // ===== 对数应变公式 (True Strain / Logarithmic Strain, 体积不变) =====
        // ε = ln(L / L0), 当前面积 A_current = A * L0 / L (体积守恒)
        // Work-conjugate Hencky axial law: N = EA*ln(L/L0),
        // hence dN/dL = EA/L.
        double materialStiffness = E * A / length_current;
        ke = B_matrix * B_matrix.transpose() * materialStiffness;

        double strain = log(length_current / L0);  // 对数应变
        m_Stress = E * strain + m_InitStress;      // 真应力，保留单元初始应力
        double axialForce = m_Stress * A;

        m_inforce = B_matrix * axialForce;

        // 几何刚度矩阵
        if (0.0 != axialForce)
        {
            Matrix3d I = Matrix3d::Identity();
            Vector3d directionVector;
            directionVector << dirCos_x, dirCos_y, dirCos_z;

            double geometricStiffCoeff = axialForce / length_current;
            Matrix3d Kg_block = geometricStiffCoeff * (I - directionVector * directionVector.transpose());

            ke.block<3, 3>(0, 0) += Kg_block;
            ke.block<3, 3>(3, 0) -= Kg_block;
            ke.block<3, 3>(0, 3) -= Kg_block;
            ke.block<3, 3>(3, 3) += Kg_block;
        }
    }
}

void ElementTruss::Get_me_Lumped(MatrixXd& me)//集中质量矩阵
{
    auto pProperty = m_pProperty.lock();
    auto pSection = pProperty->m_pSection.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();

    double Density = pMaterial->m_Density;
    double A = pSection->m_Area;

    double mass = L0 * A * Density;
    me = MatrixXd::Identity(6, 6) * (mass / 2.0);
}

void ElementTruss::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
    auto pProperty = m_pProperty.lock();
    auto pSection = pProperty->m_pSection.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();

    double Density = pMaterial->m_Density;
    double A = pSection->m_Area;

    double mass = L0 * A * Density;
    me = MatrixXd::Identity(6, 6) * (mass / 3.0);
    me(0, 3) = me(1, 4) = me(2, 5) = mass / 6.0;
    me(3, 0) = me(4, 1) = me(5, 2) = mass / 6.0;
}

void ElementTruss::Get_L0()
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();

    if (pNode0 == nullptr || pNode1 == nullptr)
    {
        qDebug().noquote() << QStringLiteral("Error: ElementTruss 节点指针为空");
        return;
    }

    double dx0 = pNode1->m_X - pNode0->m_X;
    double dy0 = pNode1->m_Y - pNode0->m_Y;
    double dz0 = pNode1->m_Z - pNode0->m_Z;

    L0 = sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
    // 计算初始长度 L0
}

void ElementTruss::Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce)
{
    if (damping.size() < 4)
    {
        // 处理错误：抛出异常或设置空矩阵并返回
        throw std::invalid_argument("damping vector must have at least 4 elements");
    }

    MatrixXd ke;
    MatrixXd me;
    Get_L0();
    Get_ke(ke);
    Get_me_Consistent(me);//一致质量矩阵

    ce = damping[0] * me + damping[1] * ke;
}
