#pragma once

#include "Base/Base.h"
#include "DataStructure/Node/Node.h"

/**
 * @brief 质心节点上的刚体集中惯性
 *
 * 该对象只描述质量特性，不提供刚度，也不参与 MPC 运动学定义。
 * 转动惯量在节点初始随体坐标系中保存，动力组装时转换到当前空间坐标系。
 */
class RigidBodyInertia : public Base
{
public:
    std::weak_ptr<Node> m_pNode;                               ///< 刚体质心参考节点
    double m_Mass = 0.0;                                       ///< 总质量
    Eigen::Matrix3d m_RotaryInertia = Eigen::Matrix3d::Zero(); ///< 初始随体坐标系中的质心转动惯量

    bool IsValid() const;
    void GetDOFs(_OUT std::vector<int>& dofs) const;
    void GetDynamicContributions(_OUT Eigen::MatrixXd& massMatrix, _OUT Eigen::VectorXd& inertiaForce,
                                 _OUT Eigen::MatrixXd& gyroscopicMatrix,
                                 _OUT Eigen::MatrixXd& configurationTangent) const;
};
