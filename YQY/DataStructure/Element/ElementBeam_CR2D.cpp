#include "ElementBeam_CR2D.h"

ElementBeam_CR2D::ElementBeam_CR2D()
{
    m_pNode.resize(2);
}

void ElementBeam_CR2D::Get_ke(MatrixXd& ke)
{
    auto pProperty = m_pProperty.lock();
    auto pSection = pProperty->m_pSection.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();

    double E = pMaterial->m_Young;
    double A = pSection->m_Area;  //I=A/16
    double I = A / 16.0;

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

    // 方向余弦 (direction cosines)
    double dirCos_x = dx / length;
    double dirCos_y = dy / length;

    Eigen::Matrix<double, 3, 6> B_matrix;

    B_matrix << -dirCos_x, -dirCos_y, 0, dirCos_x, dirCos_y, 0,
        -dirCos_y / L0, dirCos_x / L0, 1, dirCos_y / L0, -dirCos_x / L0, 0,
        -dirCos_y / L0, dirCos_x / L0, 0, dirCos_y / L0, -dirCos_x / L0, 1;

    ke.setZero(3, 3);
    ke(0, 0) = E * A / L0; ke(1, 1) = 4 * E * I / L0; ke(2, 2) = 4 * E * I / L0;
    ke(1, 2) = ke(2, 1) = 2 * E * I / L0;
    
    ke = B_matrix.transpose() * ke * B_matrix;
}

void ElementBeam_CR2D::Get_ke_non(MatrixXd& ke)
{

}

void ElementBeam_CR2D::Get_me_Lumped(MatrixXd& me)
{

}

void ElementBeam_CR2D::Get_me_Consistent(MatrixXd& me)
{

}

void ElementBeam_CR2D::Get_L0()
{

}

void ElementBeam_CR2D::Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce)
{
}