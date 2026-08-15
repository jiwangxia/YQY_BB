#pragma once

#include "Base/Base.h"
#include "Solver/Constraint/NonlinearMPC.h"

#include <QString>
#include <map>
#include <memory>
#include <limits>
#include <vector>

class Node;

class NonlinearMPCConstraint : public Base
{
public:
    virtual ~NonlinearMPCConstraint() = default;

    QString m_Name;
    int m_StepId = 0;
    std::vector<int> m_SlaveDirections;

    virtual bool Evaluate(int fixedDofs, int freeDofs, _OUT SolverNameSpace::NonlinearMPCData& data) const = 0;
    virtual std::shared_ptr<NonlinearMPCConstraint> Clone(const std::map<int, std::shared_ptr<Node>>& nodes) const = 0;
    virtual std::vector<int> GetNodeIds() const = 0;
    virtual void AccumulateReactions(int fixedDofs, const Eigen::VectorXd& multipliers) const = 0;
};

/**
 * 重合节点平移约束：c = u_slave - u_master = 0。
 *
 * 仅耦合 UX、UY、UZ。转动自由度和索扭转自由度保持独立，因此不同节点自由度含义的单元可以连接，
 * 不会在同一节点上错误复用自由度。
 */
class TranslationalTieMPCConstraint final : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pMasterNode;
    std::weak_ptr<Node> m_pSlaveNode;

    bool Evaluate(int fixedDofs, int freeDofs, _OUT SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(int fixedDofs, const Eigen::VectorXd& multipliers) const override;
};

/** 距离约束：c = ||x_a-x_b||-L0 = 0。 */
class DistanceMPCConstraint final : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pNodeA;
    std::weak_ptr<Node> m_pNodeB;
    std::weak_ptr<Node> m_pSlaveNode;
    int m_SlaveDirection = 0;
    double m_Length = -1.0;

    bool Evaluate(int fixedDofs, int freeDofs, _OUT SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(int fixedDofs, const Eigen::VectorXd& multipliers) const override;
};

/**
 * 精确刚性偏置约束：c = x_s - x_m - R_m r_0 = 0。
 * 主节点转动采用与 AnalysisStep::ApplyIncrement 中六自由度梁节点相同的左乘空间增量。
 */
class RigidOffsetMPCConstraint final : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pMasterNode;
    std::weak_ptr<Node> m_pSlaveNode;
    Eigen::Vector3d m_Offset = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

    bool Evaluate(int fixedDofs, int freeDofs, _OUT SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(int fixedDofs, const Eigen::VectorXd& multipliers) const override;
};

/** 将四自由度索节点的扭转自由度约束到六自由度主节点绕导线轴的空间扭转。 */
class AxialTwistTieMPCConstraint final : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pMasterNode;
    std::weak_ptr<Node> m_pSlaveNode;
    Eigen::Vector3d m_Axis = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    int m_SlaveDirection = 3;

    bool Evaluate(int fixedDofs, int freeDofs, _OUT SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(int fixedDofs, const Eigen::VectorXd& multipliers) const override;
};

/**
 * Boungard--Wackerfuss（2024）公式（45）：平面非线性剪切释放。
 *
 * c0 = n(phi_m)^T u_m - n(phi_s)^T u_s = 0
 * c1 = phi_m - phi_s = 0
 *
 * 平面自由度为全局 X、Y 和绕 Z 轴转动。消去从节点 X 与 RZ，对应 SlaveDirection 05。
 */
class PlanarShearReleaseMPCConstraint final : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pMasterNode;
    std::weak_ptr<Node> m_pSlaveNode;

    bool Evaluate(int fixedDofs, int freeDofs, _OUT SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(int fixedDofs, const Eigen::VectorXd& multipliers) const override;
};
