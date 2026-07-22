#pragma once
#include "ElementBase.h"

/**
 * @brief 索单元类 - 只承受拉力的二节点单元
 */
class ElementCable : public ElementBase
{
private:
    double Sy = 0.0;
    double Sz = 0.0;
public:
    /**
     * @brief 构造函数
     */
    ElementCable();

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 4（平移自由度 X, Y, Z + 扭转 RX）
     */
	int Get_NodeDOF() const override { return 4; };

    /**
     * @brief 计算单元刚度矩阵
     * @param [out] ke 单元刚度矩阵（8x8）
     */
    void Get_ke(MatrixXd& ke);
    void Get_me_Lumped(MatrixXd& me);         //集中质量矩阵
    void Get_me_Consistent(MatrixXd& me);     //一致质量矩阵
    void Get_L0();
    void Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce);
};
