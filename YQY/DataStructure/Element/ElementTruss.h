#pragma once
#include "ElementBase.h"
#include "DataStructure/Material/Material1D.h"

/**
 * @brief 桁架单元类 - 只承受轴力的二节点单元
 */
class ElementTruss : public ElementBase
{
private:
    Material1DState m_OldState;   ///< 上一个已收敛增量步状态
    Material1DState m_TrialState; ///< 当前 Newton 迭代试算状态
    bool m_StateInitialized = false;

    /**
     * @brief 用单元初始应力建立材料点初始状态，只执行一次
     */
    void InitializeState();

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
    void Get_me_Lumped(MatrixXd& me);         //集中质量矩阵
    void Get_me_Consistent(MatrixXd& me);     //一致质量矩阵
    void Get_L0();
    void Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce);

    /**
     * @brief 提交当前已经收敛的一维材料状态
     */
    void CommitState() override;
};
