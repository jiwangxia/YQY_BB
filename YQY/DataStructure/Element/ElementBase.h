#pragma once
#include "Base/Base.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionBase.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Node/Node.h"
#include "Utility/CR.h"

/**
 * @brief 单元基类 - 所有单元类型的公共基类
 */
class ElementBase : public Base
{
protected:
    MatrixXd m_ke, m_me, m_ce;
public:
    ElementBase();
    QVector<std::weak_ptr<Node>> m_pNode;       //节点指针数组
    std::weak_ptr<Property>      m_pProperty;   //所属属性（材料+截面）
    double L0 = 0.0, L = 0.0;  ///< 单元初始长度、当前长度
    double m_InitStress = 0.0;                  // 初始应力
    double m_Stress = 0.0;                      // 单元应力

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 节点自由度个数
     */
    virtual int Get_NodeDOF() const = 0;

    /**
     * @brief 获取单元所有自由度编号
     * @param [out] DOFs 自由度编号数组
     */
    void GetDOFs(std::vector<int>& DOFs);


    Eigen::VectorXd m_inforce;
    /**
     * @brief 获取单元刚度矩阵
     * @param [out] ke 单元刚度矩阵
     */
    virtual void Get_ke(MatrixXd& _OUT ke) = 0;
    virtual void Get_me_Lumped(MatrixXd& _OUT me) = 0;       //集中质量矩阵
    virtual void Get_me_Consistent(MatrixXd& _OUT me) = 0;   //一致质量矩阵
    virtual void Get_L0() = 0;

    /**
     * @brief 提交当前增量步已经收敛的单元内部状态
     *
     * 没有历史变量的单元使用默认空实现。
     */
    virtual void CommitState() {}

    /**
    * @brief 组装单元阻尼矩阵，用vector保存四个阻尼值double trans_m = 0.0, double trans_k = 0.0, double rot_m = 0.0, double rot_k = 0.0
    * @param [in] Vector[4] damping   //依次上述值
    * @param [out] ce 单元阻尼矩阵
    */
    virtual void Assemble(const std::vector<double>& damping, MatrixXd& _OUT ce) = 0;
};
