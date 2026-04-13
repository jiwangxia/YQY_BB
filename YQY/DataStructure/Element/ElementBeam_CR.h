#pragma once
#include "ElementBase.h"
class ElementBeam_CR : public ElementBase
{
public:
    ElementBeam_CR();

    int Get_NodeDOF() const override { return 12; };

    void Get_ke(MatrixXd& ke);
    void Get_ke_non(MatrixXd& ke);
    void Get_me_Lumped(MatrixXd& me);         //集中质量矩阵
    void Get_me_Consistent(MatrixXd& me);     //一致质量矩阵
    void Get_L0();
    void Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce);
};

