#pragma once
#include "ElementBase.h"
class ElementBeam_CR2D : public ElementBase
{
public:
    /**
     * @brief 构造函数
     */
    ElementBeam_CR2D();
    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 3（平移自由度 X, Y, X轴扭转 RX）
     */
    int Get_NodeDOF() const override { return 3; };

    /**
     * @brief 计算单元刚度矩阵
     * @param [out] ke 单元刚度矩阵（8x8）
     */
    void Get_ke(MatrixXd& ke);
    void Get_ke_non(MatrixXd& ke);
    void Get_me_Lumped(MatrixXd& me);         //集中质量矩阵
    void Get_me_Consistent(MatrixXd& me);     //一致质量矩阵
    void Get_L0();
    void Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce);
};

