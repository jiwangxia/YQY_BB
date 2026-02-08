#include "ElementCable.h"

ElementCable::ElementCable()
{
    m_pNode.resize(2);
}
void ElementCable::Get_ke()
{

}

void ElementCable::Get_ke_non()
{

}

void ElementCable::Get_me_Lumped()//集中质量矩阵
{
    auto pMaterial = m_pProperty.lock()->m_pMaterial.lock();
    auto pSection = m_pProperty.lock()->m_pSection.lock();

    double A = pSection->m_Area;
    double Radius = pSection->m_Radius;
    double Density = pMaterial->m_Density;
    Get_L0();  // 获取原长 L

    double Linear_density = A * Density;               //线性密度---单位长度质量
    double I = 0.5 * Linear_density * Radius * Radius; //单位长度转动惯量
    double Sy = 0, Sz = 0.0;

    me.setZero(8, 8);

    me(0, 0) = me(1, 1) = me(2, 2) = me(4, 4) = me(5, 5) = me(6, 6) = Linear_density;
    me(3, 3) = me(7, 7) = I;
    me(1, 3) = me(3, 1) = me(5, 7) = me(7, 5) = -Sy;
    me(2, 3) = me(3, 2) = me(6, 7) = me(7, 6) = Sz;

    me *= (L0 / 2.0);
}

void ElementCable::Get_me_Consistent() //一致质量矩阵
{
    auto pMaterial = m_pProperty.lock()->m_pMaterial.lock();
    auto pSection = m_pProperty.lock()->m_pSection.lock();

    double A = pSection->m_Area;
    double Radius = pSection->m_Radius;
    double Density = pMaterial->m_Density;
    Get_L0();  //原长
    double Linear_density = A * Density;               //线性密度---单位长度质量
    double I = 0.5 * Linear_density * Radius * Radius; //单位长度转动惯量

    double Sy = 0.0, Sz = 0.0;//目前只考虑对称截面

    me.setZero(8, 8);
    me(0, 0) = me(1, 1) = me(2, 2) = me(4, 4) = me(5, 5) = me(6, 6) = 2.0 * Linear_density;
    me(0, 4) = me(1, 5) = me(2, 6) = me(4, 0) = me(5, 1) = me(6, 2) = Linear_density;
    me(3, 3) = me(7, 7) = 2.0 * I;
    me(3, 7) = me(7, 3) = I;
    me(1, 3) = me(3, 1) = me(5, 7) = me(7, 5) = -2.0 * Sy;
    me(1, 7) = me(7, 1) = me(5, 3) = me(3, 5) = -Sy;
    me(2, 3) = me(3, 2) = me(6, 7) = me(7, 6) = 2.0 * Sz;
    me(2, 7) = me(7, 2) = me(6, 3) = me(3, 6) = Sz;
    me *= (L0 / 6.0);
}

void ElementCable::Get_L0()
{

}

void ElementCable::Assemble(double trans_m, double trans_k, double rot_m, double rot_k)
{
    Get_ke_non();
    Get_me_Consistent();//一致质量矩阵

    ce = trans_m * me + trans_k * ke;

    // 平动对角块
    ce.block<3, 3>(0, 0) = trans_m * me.block<3, 3>(0, 0) + trans_k * ke.block<3, 3>(0, 0);
    ce.block<3, 3>(4, 4) = trans_m * me.block<3, 3>(4, 4) + trans_k * ke.block<3, 3>(4, 4);

    // 扭转对角块
    ce(3, 3) = rot_m * me(3, 3) + rot_k * ke(3, 3);
    ce(7, 7) = rot_m * me(7, 7) + rot_k * ke(7, 7);
    ce(3, 7) = rot_m * me(3, 7) + rot_k * ke(3, 7);
    ce(7, 3) = ce(3, 7);
}

