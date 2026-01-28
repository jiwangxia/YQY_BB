#pragma once
#include "ElementBase.h"

/**
 * @brief 桁架单元类 - 只承受轴力的二节点单元
 */
class ElementTruss : public ElementBase
{
public:
    /**
     * @brief 构造函数
     */
    ElementTruss();

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 3（平移自由度 X, Y, Z）
     */
    int Get_NodeDOF() const override { return 3; };

    /**
     * @brief 计算单元刚度矩阵
     * @param [out] ke 单元刚度矩阵（6x6）
     */
    void Get_ke(MatrixXd& ke);
    void Get_ke_non(MatrixXd& ke);
    void Get_L0();
};

