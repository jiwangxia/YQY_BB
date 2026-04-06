#pragma once
#include "ElementBase.h"

/**
 * @brief 梁单元类 - 承受弯矩、剪力、轴力的二节点单元
 */
class ElementBeam : public ElementBase
{
public:
    /**
     * @brief 构造函数
     */
    ElementBeam();

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 6（平移 X, Y, Z + 转动 RX, RY, RZ）
     */
    int Get_NodeDOF() const override { return 6; };

    /**
     * @brief 计算单元刚度矩阵
     * @param [out] ke 单元刚度矩阵（12x12）
     */
    void Get_ke(MatrixXd& ke);
    void Get_ke_non(MatrixXd& ke);
    void Get_me_Lumped(MatrixXd& me);         //集中质量矩阵
    void Get_me_Consistent(MatrixXd& me);     //一致质量矩阵
    void Get_L0();
    void Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce);
};

