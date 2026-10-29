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

    // FEM 参考程序把初始拉应力编码到无应力长度中，并使用工程应变。
    // 该标记只由带兼容属性的 H5 导入器设置，普通 YQY 模型仍沿用对数应变。
    bool m_UseEngineeringStrain = false;

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 3（平移自由度 X, Y, Z）
     */
    int Get_NodeDOF() const override
    {
        return 3;
    };

    /**
     * @brief 计算单元刚度矩阵
     * @param [out] ke 单元刚度矩阵（6x6）
     */
    void Get_ke(_OUT MatrixXd& ke);
    bool TryGetStrainEnergy(_OUT double& energy) const override;
    bool TryGetKineticEnergy(_OUT double& energy) override;
    void Get_me_Lumped(_OUT MatrixXd& me);     //集中质量矩阵
    void Get_me_Consistent(_OUT MatrixXd& me); //一致质量矩阵
    void Get_L0();
    void Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce);
};
