#pragma once
#include "Base/Base.h"

/**
 * @brief 节点类 - 存储有限元节点信息
 */
class Node : public Base
{
public:
    Node();

    double m_X, m_Y, m_Z;  ///< 节点坐标
    QVector<int> m_DOF;    ///< 节点自由度编号数组
    std::vector<double>  m_Displacement;
    std::vector<double>  m_Acceleration;
    std::vector<double>  m_Velocity;
    std::vector<double>  m_Force;  ///< 节点力(内力)
    std::vector<double>  m_ReactionForce;  ///< 节点力反力)

    void SetNumDOFs(int num_dofs);

    Eigen::Matrix3d m_Rg = Eigen::Matrix3d::Identity();

    std::vector<double>  m_Displacement_n;                ///< 上一时间步初的位移备份
    Eigen::Matrix3d m_Rg_n = Eigen::Matrix3d::Identity(); ///< 上一时间步初的旋转矩阵备份
};

