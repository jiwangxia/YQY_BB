#include "ElementBeam.h"

ElementBeam::ElementBeam()
{
    m_pNode.resize(2);
}

void ElementBeam::Get_ke()
{
}

void ElementBeam::Get_ke_non()
{
}

void ElementBeam::Get_me_Lumped()//集中质量矩阵
{
}

void ElementBeam::Get_me_Consistent() //一致质量矩阵
{
}

void ElementBeam::Get_L0()
{
}

void ElementBeam::Assemble(double trans_m, double trans_k, double rot_m, double rot_k)
{
}

