#pragma once
#include "Base/Base.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionBase.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Node/Node.h"
#include "Utility/CR.h"

enum class ElementRole
{
    Generic = 0,
    Conductor,
    TensionHardware,
    IntraPhaseSpacer,
    InterPhaseSpacer,
    SuspensionHardware
};

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
    ElementRole m_Role = ElementRole::Generic;  ///< 单元在模型中的业务用途
    int m_WireId = -1;                          ///< 子导线编号；非导线单元为 -1
    int m_AeroBundleCount = 0;                  ///< 所属分裂数；0 表示旧模型未提供
    int m_AeroProfileId = -1;                   ///< 气动参数编号；小于 0 表示不参与气动计算

    bool HasAerodynamicLoad() const { return m_AeroProfileId >= 0; }

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 节点自由度个数
     */
    virtual int Get_NodeDOF() const = 0;

    /**
     * @brief 获取单元所有自由度编号
     * @param [out] DOFs 自由度编号数组
     */
    void GetDOFs(std::vector<int>& DOFs) const;


    Eigen::VectorXd m_inforce;
    /**
     * @brief 获取单元刚度矩阵
     * @param [out] ke 单元刚度矩阵
     */
    virtual void Get_ke(MatrixXd& _OUT ke) = 0;
    virtual void Get_me_Lumped(MatrixXd& _OUT me) = 0;       //集中质量矩阵
    virtual void Get_me_Consistent(MatrixXd& _OUT me) = 0;   //一致质量矩阵
    /**
     * @brief 计算当前试探状态的单元惯性力
     *
     * 默认实现为 M*a。具有有限转动自由度的单元可重写该函数，
     * 以加入陀螺力矩等构型相关项。
     */
    virtual void Get_InertiaForce(VectorXd& _OUT inertiaForce);

    /**
     * @brief 计算惯性力对速度的切线矩阵
     *
     * Newmark 有效切线中的系数为 gamma/(beta*dt)。
     * 不含陀螺项的单元使用默认零矩阵。
     */
    virtual void Get_GyroscopicMatrix(MatrixXd& _OUT gyroscopicMatrix);

    /**
     * @brief 计算惯性力对当前构型的切线矩阵
     *
     * 该项不乘 Newmark 的 a0 或 a1 系数。无有限转动惯性项的单元
     * 使用默认零矩阵。
     */
    virtual void Get_CentrifugalMatrix(MatrixXd& _OUT centrifugalMatrix);

    /**
     * @brief 一次性计算当前试探状态的全部动力学贡献。
     *
     * 默认实现保留旧单元的逐项计算方式；具有共同高斯积分过程的单元可
     * 重写此接口，从而避免重复计算质量、惯性力和动力切线。
     */
    virtual void GetDynamicContributions(
        MatrixXd& massMatrix,
        VectorXd& inertiaForce,
        MatrixXd& gyroscopicMatrix,
        MatrixXd& centrifugalMatrix);
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
