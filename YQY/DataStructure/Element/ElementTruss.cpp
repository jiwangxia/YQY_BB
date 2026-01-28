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

    // 计算单元方向向量分量
    double dx = pNode1->m_X - pNode0->m_X;
    double dy = pNode1->m_Y - pNode0->m_Y;
    double dz = pNode1->m_Z - pNode0->m_Z;

    // 初始长度
    L0 = sqrt(dx * dx + dy * dy + dz * dz);
    double length = L0;

    // 方向余弦 (direction cosines)
    double dirCos_x = dx / length;
    double dirCos_y = dy / length;
    double dirCos_z = dz / length;

    // 应变-位移变换矩阵 B = [-l, -m, -n, l, m, n]
    VectorXd B_matrix = VectorXd::Zero(6);
    B_matrix << -dirCos_x, -dirCos_y, -dirCos_z, dirCos_x, dirCos_y, dirCos_z;

    // 材料刚度系数 EA/L
    double materialStiffness = E * A / length;

    // 单元刚度矩阵 ke = B * B^T * (EA/L)
    ke = B_matrix * B_matrix.transpose() * materialStiffness;
}

void ElementTruss::Get_ke_non(MatrixXd& ke)
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

    //// 计算初始长度 L0
    //double dx0 = pNode1->m_X - pNode0->m_X;
    //double dy0 = pNode1->m_Y - pNode0->m_Y;
    //double dz0 = pNode1->m_Z - pNode0->m_Z;
    //L0 = sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);

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
    bool bUseLogStrain = true;  // TODO: 可改为类成员变量 m_bUseLogStrain

    if (!bUseLogStrain)
    {
        // ===== 工程应变公式 (Engineering Strain) =====
        // ε = (L - L0) / L0
        double materialStiffness = E * A / L0;
        ke = B_matrix * B_matrix.transpose() * materialStiffness;

        double strain = (length_current - L0) / L0;
        m_Stress = E * strain;
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
        double A_current = A * L0 / length_current;
        double materialStiffness = E * A_current / L0;
        ke = B_matrix * B_matrix.transpose() * materialStiffness;

        double strain = log(length_current / L0);  // 对数应变
        m_Stress = E * strain;                     // 真应力
        double axialForce = m_Stress * A_current;

        m_inforce = B_matrix * axialForce;

        // 几何刚度矩阵
        if (0.0 != m_Stress)
        {
            Matrix3d I = Matrix3d::Identity();
            Vector3d directionVector;
            directionVector << dirCos_x, dirCos_y, dirCos_z;

            double geometricStiffCoeff = A_current * m_Stress / length_current;
            Matrix3d Kg_block = geometricStiffCoeff * (I - directionVector * directionVector.transpose());

            ke.block<3, 3>(0, 0) += Kg_block;
            ke.block<3, 3>(3, 0) -= Kg_block;
            ke.block<3, 3>(0, 3) -= Kg_block;
            ke.block<3, 3>(3, 3) += Kg_block;
        }
    }
}

void ElementTruss::Get_L0()
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

    // 计算初始长度 L0
    double dx0 = pNode1->m_X - pNode0->m_X;
    double dy0 = pNode1->m_Y - pNode0->m_Y;
    double dz0 = pNode1->m_Z - pNode0->m_Z;
    L0 = sqrt(dx0 * dx0 + dy0 * dy0 + dz0 * dz0);
}

