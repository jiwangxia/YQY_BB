#include "ElementBeam.h"

ElementBeam::ElementBeam()
{
    m_pNode.resize(2);
}

void ElementBeam::Get_ke(MatrixXd& ke)
{
}

void ElementBeam::Get_ke_non(MatrixXd& ke)
{
}

void ElementBeam::Get_me_Lumped(MatrixXd& me)//集中质量矩阵
{
}

void ElementBeam::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
}

void ElementBeam::Get_L0()
{
}

void ElementBeam::Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce)
{
}

