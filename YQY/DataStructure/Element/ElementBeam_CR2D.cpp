#include "ElementBeam_CR2D.h"

ElementBeam_CR2D::ElementBeam_CR2D()
{
    m_pNode.resize(2);
}


void ElementBeam_CR2D::Get_ke(MatrixXd& ke)
{
    auto pProperty = m_pProperty.lock();
    auto pSectI0n = pProperty->m_pSection.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();

    double E = pMaterial->m_Young;
    double A = pSectI0n->m_Area;  //I=A/16
    double I = A * A / 4.0 / PI;

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

    // 初始长度
    L0 = sqrt(dx * dx + dy * dy);
    double length = L0;

    // 方向余弦 (directI0n cosines)
    double dirCos_x = dx / length;
    double dirCos_y = dy / length;

    Eigen::Matrix<double, 3, 6> B_matrix;

    B_matrix << -dirCos_x, -dirCos_y, 0, dirCos_x, dirCos_y, 0,
        -dirCos_y / L0, dirCos_x / L0, 1, dirCos_y / L0, -dirCos_x / L0, 0,
        -dirCos_y / L0, dirCos_x / L0, 0, dirCos_y / L0, -dirCos_x / L0, 1;
    std::cout << MatrixXd(B_matrix) << "\n";
    ke.setZero(3, 3);
    ke(0, 0) = E * A / L0; ke(1, 1) = 4 * E * I / L0; ke(2, 2) = 4 * E * I / L0;
    ke(1, 2) = ke(2, 1) = 2 * E * I / L0;
    std::cout << MatrixXd(ke) << "\n";
    ke = B_matrix.transpose() * ke * B_matrix;
    std::cout << MatrixXd(ke) << "\n";
}

void ElementBeam_CR2D::Get_ke_non(MatrixXd& ke)
{
    auto pProperty = m_pProperty.lock();
    auto pSectI0n = pProperty->m_pSection.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();

    double E = pMaterial->m_Young;
    double A = pSectI0n->m_Area;

    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();

    if (pNode0 == nullptr || pNode1 == nullptr)
    {
        qDebug().noquote() << QStringLiteral("Error: ElementTruss 节点指针为空");
        return;
    }

    Get_L0();

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

    Eigen::Matrix<double, 3, 6> B_matrix;

    B_matrix << -dirCos_x, -dirCos_y, 0, dirCos_x, dirCos_y, 0,
        -dirCos_y / L0, dirCos_x / L0, 1, dirCos_y / L0, -dirCos_x / L0, 0,
        -dirCos_y / L0, dirCos_x / L0, 0, dirCos_y / L0, -dirCos_x / L0, 1;

}

void ElementBeam_CR2D::Get_me_Lumped(MatrixXd& me)
{

}

void ElementBeam_CR2D::Get_me_Consistent(MatrixXd& me)
{

}

void ElementBeam_CR2D::Get_L0()
{
    auto pProperty = m_pProperty.lock();
    auto pSectI0n = pProperty->m_pSection.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();

    double E = pMaterial->m_Young;
    double A = pSectI0n->m_Area;

    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();

    if (pNode0 == nullptr || pNode1 == nullptr)
    {
        qDebug().noquote() << QStringLiteral("Error: ElementTruss 节点指针为空");
        return;
    }

    double dx0 = pNode1->m_X - pNode0->m_X;
    double dy0 = pNode1->m_Y - pNode0->m_Y;

    double L = sqrt(dx0 * dx0 + dy0 * dy0);
    // 计算初始长度 L0
    if (m_InitStress == 0)
    {
        L0 = L;
    }
    else
    {
        L0 = E * L / (m_InitStress + E);
    }
}

void ElementBeam_CR2D::Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce)
{
}