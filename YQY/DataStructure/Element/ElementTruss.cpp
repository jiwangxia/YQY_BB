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

    double dx = pNode1->m_X - pNode0->m_X;
    double dy = pNode1->m_Y - pNode0->m_Y;
    double dz = pNode1->m_Z - pNode0->m_Z;

    L0 = sqrt(dx * dx + dy * dy + dz * dz);

    double L = L0;

    double l = dx / L;
    double m = dy / L;
    double n = dz / L;

    VectorXd r1 = VectorXd::Zero(6);
    r1 << -l, -m, -n, l, m, n;

    double EAL = E * A / L;

    ke = r1 * r1.transpose() * EAL;
}

