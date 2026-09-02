#pragma once

#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Element/SpringBehavior.h"

// 弹簧单元公共基类，负责本构曲线插值和当前状态记录。
class ElementSpringBase : public ElementBase
{
public:
    std::weak_ptr<SpringBehavior> m_pSpringBehavior; // 弹簧力-位移本构曲线
    double m_CurrentForce = 0.0;                     // 当前相对位移对应的弹簧力
    double m_CurrentRelativeDisplacement = 0.0;      // 当前两端的相对位移

    int Get_NodeDOF() const override
    {
        return 3;
    }

    void Get_me_Lumped(_OUT MatrixXd& me) override;                           // 返回零集中质量矩阵
    void Get_me_Consistent(_OUT MatrixXd& me) override;                       // 返回零一致质量矩阵
    void Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce) override; // 返回零阻尼矩阵
    double GetCurrentTangent() const; // 查询当前位移处的本构切线

protected:
    // 插值计算指定相对位移的弹簧力和切线，并更新当前状态。
    void EvaluateBehavior(double relativeDisplacement, _OUT double& force, _OUT double& tangent);
};
