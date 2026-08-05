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

    virtual bool Evaluate(
        int fixedDofs,
        int freeDofs,
        SolverNameSpace::NonlinearMPCData& data) const = 0;
    virtual std::shared_ptr<NonlinearMPCConstraint> Clone(
        const std::map<int, std::shared_ptr<Node>>& nodes) const = 0;
    virtual std::vector<int> GetNodeIds() const = 0;
    virtual void AccumulateReactions(
        int fixedDofs,
        const Eigen::VectorXd& multipliers) const = 0;
};

/**
 * Coincident-node translational tie:
 * c = u_slave - u_master = 0.
 *
 * Only UX/UY/UZ are coupled.  Rotational or cable-twist degrees of freedom
 * remain independent, so elements with different nodal DOF meanings can be
 * connected without aliasing those meanings at one node.
 */
class TranslationalTieMPCConstraint final : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pMasterNode;
    std::weak_ptr<Node> m_pSlaveNode;

    bool Evaluate(
        int fixedDofs,
        int freeDofs,
        SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(
        const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(
        int fixedDofs,
        const Eigen::VectorXd& multipliers) const override;
};

/** c = ||x_a-x_b||-L0 = 0. */
class DistanceMPCConstraint final : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pNodeA;
    std::weak_ptr<Node> m_pNodeB;
    std::weak_ptr<Node> m_pSlaveNode;
    int m_SlaveDirection = 0;
    double m_Length = -1.0;

    bool Evaluate(
        int fixedDofs,
        int freeDofs,
        SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(
        const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(
        int fixedDofs,
        const Eigen::VectorXd& multipliers) const override;
};

/**
 * Exact rigid offset c = x_s - x_m - R_m r_0 = 0.
 *
 * The master rotation uses the same left-multiplicative spatial increment
 * as the six-DOF beam nodes in AnalysisStep::ApplyIncrement.
 */
class RigidOffsetMPCConstraint final : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pMasterNode;
    std::weak_ptr<Node> m_pSlaveNode;
    Eigen::Vector3d m_Offset =
        Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());

    bool Evaluate(
        int fixedDofs,
        int freeDofs,
        SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(
        const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(
        int fixedDofs,
        const Eigen::VectorXd& multipliers) const override;
};

/**
 * Boungard--Wackerfuss (2024), Eq. (45): planar nonlinear shear release.
 *
 * c0 = n(phi_m)^T u_m - n(phi_s)^T u_s = 0
 * c1 = phi_m - phi_s = 0
 *
 * The planar degrees of freedom are global X, Y and rotation about Z.
 * Slave X and slave RZ are eliminated, corresponding to SlaveDirection 05.
 */
class PlanarShearReleaseMPCConstraint final
    : public NonlinearMPCConstraint
{
public:
    std::weak_ptr<Node> m_pMasterNode;
    std::weak_ptr<Node> m_pSlaveNode;

    bool Evaluate(
        int fixedDofs,
        int freeDofs,
        SolverNameSpace::NonlinearMPCData& data) const override;
    std::shared_ptr<NonlinearMPCConstraint> Clone(
        const std::map<int, std::shared_ptr<Node>>& nodes) const override;
    std::vector<int> GetNodeIds() const override;
    void AccumulateReactions(
        int fixedDofs,
        const Eigen::VectorXd& multipliers) const override;
};
