#include "CR.h"
#include <cmath>

namespace Utility 
{
    namespace CR 
    {

        // ========================================================================
        // 1. 反对称矩阵
        // ========================================================================
        void SkewSymmetric(const Eigen::Vector3d& v, Eigen::Matrix3d& _OUT  result)
        {
            result << 0.0, -v(2),  v(1),
                     v(2),   0.0, -v(0),
                    -v(1),  v(0),   0.0;
        }

        // ========================================================================
        // 2. 计算 Ts 矩阵
        // ========================================================================
        void Calculate_Ts(const Eigen::Vector3d& vartheta, Eigen::Matrix3d& _OUT  result)
        {
            double theta = vartheta.norm();

            double c1, c2, c3;

            // 泰勒展开防止极小角度下的除零奇异性
            if (theta < 1e-3) 
            {
                double t2 = theta * theta;
                double t4 = t2 * t2;

                c1 = 1.0 - t2 / 6.0 + t4 / 120.0;
                c2 = 0.5 - t2 / 24.0 + t4 / 720.0;
                c3 = 1.0 / 6.0 - t2 / 120.0 + t4 / 5040.0;
            }
            else 
            {
                c1 = std::sin(theta) / theta;
                c2 = (1.0 - std::cos(theta)) / (theta * theta);
                c3 = (theta - std::sin(theta)) / (theta * theta * theta);
            }

            // 调用刚才写的反对称矩阵函数
            Eigen::Matrix3d skew_v;
            SkewSymmetric(vartheta, skew_v);

            // 组装最终结果存入 result
            result = c1 * Eigen::Matrix3d::Identity()
                   + c2 * skew_v
                   + c3 * (vartheta * vartheta.transpose());
        }

        // ========================================================================
        // 3. 计算 Ts 的逆矩阵
        // ========================================================================
        void Calculate_Ts_Inv(const Eigen::Vector3d& vartheta, Eigen::Matrix3d& _OUT  result)
        {
            double theta = vartheta.norm();

            double c1, eta;

            // 泰勒展开防止极小角度下的除零奇异性
            if (theta < 1e-3) 
            {
                double t2 = theta * theta;
                double t4 = t2 * t2;

                c1 = 1.0 - t2 / 12.0 - t4 / 720.0;
                eta = 1.0 / 12.0 + t2 / 720.0 + t4 / 30240.0;
            }
            else 
            {
                double half_theta = 0.5 * theta;
                double cot_half = 1.0 / std::tan(half_theta);

                c1 = half_theta * cot_half;
                eta = (1.0 - c1) / (theta * theta);
            }

            // 调用刚才写的反对称矩阵函数
            Eigen::Matrix3d skew_v;
            SkewSymmetric(vartheta, skew_v);

            // 组装最终结果存入 result
            result = c1 * Eigen::Matrix3d::Identity()
                  - 0.5 * skew_v
                  + eta * (vartheta * vartheta.transpose());
        }

    // ========================================================================
    // 4. 罗德里格斯公式 (向量 -> 矩阵)
    // ========================================================================
        void Calculate_RotationMatrix(const Eigen::Vector3d& vartheta, Eigen::Matrix3d& _OUT result)
        {
            double theta = vartheta.norm();
            Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d skew_v;
            SkewSymmetric(vartheta, skew_v);

            if (theta < 1e-4) 
            {
                // 泰勒展开防奇异
                double t2 = theta * theta;
                double c1 = 1.0 - t2 / 6.0 + (t2 * t2) / 120.0;
                double c2 = 0.5 - t2 / 24.0 + (t2 * t2) / 720.0;
                result = I + c1 * skew_v + c2 * (skew_v * skew_v);
            }
            else 
            {
                // 精确公式
                double c1 = std::sin(theta) / theta;
                double c2 = (1.0 - std::cos(theta)) / (theta * theta);
                result = I + c1 * skew_v + c2 * (skew_v * skew_v);
            }
        }

        // ========================================================================
        // 5. 计算随动坐标系 Rr (局部坐标系架设)
        // ========================================================================
        void Calculate_Rr(const Eigen::Vector3d& p1_def, const Eigen::Vector3d& p2_def, const Eigen::Vector3d& q, Eigen::Matrix3d& _OUT result)
        {
            // 1. 局部 x 轴 (r1)：沿弦线方向
            Eigen::Vector3d d = p2_def - p1_def;
            Eigen::Vector3d r1 = d.normalized();

            // 2. 局部 z 轴 (r3)：垂直于 r1 和 q 构成的平面
            Eigen::Vector3d r3 = r1.cross(q).normalized();

            // 3. 局部 y 轴 (r2)：利用右手定则闭合
            // 两个正交单位向量的叉乘必然是单位向量，无需再次 normalize()
            Eigen::Vector3d r2 = r3.cross(r1);

            // 4. 组装 Rr = [r1, r2, r3]
            result.col(0) = r1;
            result.col(1) = r2;
            result.col(2) = r3;
        }

        // ========================================================================
        // 6. 对数映射 (矩阵 -> 向量)
        // ========================================================================
        void Extract_RotationVector(const Eigen::Matrix3d& R, Eigen::Vector3d& _OUT result)
        {
            // 计算旋转角的余弦值 (利用矩阵的迹 trace)
            double trace = R.trace();
            double cos_theta = 0.5 * (trace - 1.0);

            // 数值保护：防止舍入误差导致 cos_theta 超出 [-1, 1] 范围触发 acos 错误 (NaN)
            if (cos_theta > 1.0) cos_theta = 1.0;
            if (cos_theta < -1.0) cos_theta = -1.0;

            double theta = std::acos(cos_theta);

            // 提取反对称部分： R - R^T = 2 * sin(theta) * \tilde{n}
            Eigen::Matrix3d skew_part = R - R.transpose();
            Eigen::Vector3d spin_vec(skew_part(2, 1), skew_part(0, 2), skew_part(1, 0));

            if (theta < 1e-4) 
            {
                // 当转角极小，sin(theta) 近似为 theta。
                // 此时 R - R^T 近似等于 2 * \tilde{\vartheta}
                result = 0.5 * spin_vec;
            }
            else 
            {
                // 精确计算：乘上修正系数 theta / (2 * sin(theta))
                double coef = theta / (2.0 * std::sin(theta));
                result = coef * spin_vec;
            }
        }

    // 7. 节点姿态增量更新 (乘法更新法则)
    // R_new = exp(delta_theta) * R_old
    // ========================================================================
        void Update_NodalRotation(const Eigen::Vector3d& delta_theta, const Eigen::Matrix3d& R_old, Eigen::Matrix3d& _OUT R_new)
        {
            Eigen::Matrix3d delta_R;
            // 1. 先用罗德里格斯公式把转角增量变成增量旋转矩阵
            Calculate_RotationMatrix(delta_theta, delta_R);

            // 2. 矩阵左乘更新姿态 (基于全局坐标系的增量)
            R_new = delta_R * R_old;

            // (注意：如果你需要覆盖旧的旋转向量，外部调用完这个函数后，
            // 记得调用 Extract_RotationVector(R_new, theta_new) 把向量也更新了)
        }


    // ========================================================================
    // 8. 组装通用坐标系转换矩阵 E
    // ========================================================================
        void Assemble_E_Matrix_Generic(const Eigen::Matrix3d& Rr, int total_dofs, const std::vector<int>& vector_block_indices, Eigen::MatrixXd& _OUT E_result)
        {
            // 1. 初始化为单位矩阵！
            // 这一步极其精妙：它自动保证了那些不需要旋转的标量自由度（如索的滑移）
            // 在转换前后保持不变（对角线系数为 1）。
            E_result = Eigen::MatrixXd::Identity(total_dofs, total_dofs);

            // 2. 准备局部到全局的转置矩阵
            Eigen::Matrix3d Rr_T = Rr.transpose();

            // 3. 在指定的位置覆盖填入 3x3 的旋转块
            for (int start_idx : vector_block_indices) 
            {
                // 安全性检查，防止越界崩溃
                if (start_idx + 2 < total_dofs) 
                {
                    E_result.block<3, 3>(start_idx, start_idx) = Rr_T;
                }
            }
        }

    // ========================================================================
    // 9. 欧拉角转旋转矩阵 (以绕 X-Y-Z 顺序为例，对应 Roll-Pitch-Yaw)
    // ========================================================================
        void EulerAngles_To_RotationMatrix(const Eigen::Vector3d& eulerAngles, Eigen::Matrix3d& _OUT result)
        {
            double cx = std::cos(eulerAngles(0));
            double sx = std::sin(eulerAngles(0));
            double cy = std::cos(eulerAngles(1));
            double sy = std::sin(eulerAngles(1));
            double cz = std::cos(eulerAngles(2));
            double sz = std::sin(eulerAngles(2));

            Eigen::Matrix3d Rx, Ry, Rz;
            Rx << 1.0, 0.0, 0.0,
                  0.0,  cx, -sx,
                  0.0,  sx, cx;

            Ry << cy, 0.0,  sy,
                 0.0, 1.0, 0.0,
                 -sy, 0.0,  cy;

            Rz << cz, -sz, 0.0,
                  sz,  cz, 0.0,
                 0.0, 0.0, 1.0;

            // 连乘得到初始绝对姿态
            result = Rz * Ry * Rx;
        }
    }
}