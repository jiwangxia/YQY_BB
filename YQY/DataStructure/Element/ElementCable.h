#pragma once
#include "ElementBase.h"

#include <array>

/**
 * @brief 索单元类 - 只承受拉力的二节点单元
 */
class ElementCable : public ElementBase
{
private:
    double Sy = 0.0;
    double Sz = 0.0;
    std::array<double, 2> m_CommittedSpatialTwist = {0.0, 0.0};
    std::array<double, 2> m_BackupCommittedSpatialTwist = {0.0, 0.0};

public:
    /**
     * @brief 构造函数
     */
    ElementCable();

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 最小为 4（平移 X/Y/Z + 标量扭转）；与梁共节点时局部布局自动扩展为 6
     */
    int Get_NodeDOF() const override
    {
        return 4;
    };

    void GetNodeLocalDOFCounts(_OUT std::vector<int>& counts) const override;
    double GetNodalTwist(int nodeIndex) const;
    double GetNodalTwistRate(int nodeIndex) const;
    void AddNodalAxialTorque(int nodeIndex, double torque, _OUT VectorXd& elementForce) const;
    void CopyRuntimeState(const ElementCable& source);
    void CommitState() override;
    void BackupState() override;
    void RestoreState() override;

    /**
     * @brief 计算单元刚度矩阵
     * @param [out] ke 单元刚度矩阵（普通节点为 8x8，含共享节点时按 4/6 混合布局扩展）
     */
    void Get_ke(_OUT MatrixXd& ke);
    void Get_me_Lumped(_OUT MatrixXd& me);     //集中质量矩阵
    void Get_me_Consistent(_OUT MatrixXd& me); //一致质量矩阵
    void Get_L0();
    void Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce);
};
