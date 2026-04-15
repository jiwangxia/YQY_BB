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

        // ========================================================================
        // 组装三维梁专属的 Ba 矩阵 (7x7)
        // 自由度排布: [u, tx1, ty1, tz1, tx2, ty2, tz2]
        // ========================================================================
        void Assemble_Matrix_Ba(const Eigen::Vector3d& vartheta1, const Eigen::Vector3d& vartheta2, Eigen::MatrixXd& _OUT result)
        {
            // 初始化为 7x7 的单位矩阵
            // 这保证了轴向自由度 u 的映射系数为 1.0
            result = Eigen::MatrixXd::Identity(7, 7);

            Eigen::Matrix3d Ts_Inv_1, Ts_Inv_2;

            // 计算两端的 Ts 逆矩阵
            Calculate_Ts_Inv(vartheta1, Ts_Inv_1);
            Calculate_Ts_Inv(vartheta2, Ts_Inv_2);

            // 塞入对应的自由度块 (索引 1 和 4)
            result.block<3, 3>(1, 1) = Ts_Inv_1;
            result.block<3, 3>(4, 4) = Ts_Inv_2;
        }

        // ========================================================================
        // 计算单个节点的 3x3 局部几何刚度块 Khi
        // ========================================================================
        void Calculate_Khi(const Eigen::Vector3d& v, const Eigen::Vector3d& m, Eigen::Matrix3d& _OUT result)
        {
            double theta = v.norm();
            double eta, c2, c3;

            // 同样必须使用泰勒展开防止极小角度下的除零奇异性
            if (theta < 0.1)
            {
                double t2 = theta * theta;
                double t4 = t2 * t2;

                // eta: 1/12 + t2/720 + t4/30240
                eta = 1.0 / 12.0 + t2 / 720.0 + t4 / 30240.0;

                // c2 的三阶泰勒展开
                c2 = -1.0 / 6.0 - t2 / 180.0 - t4 / 5040.0;

                // c3 的三阶泰勒展开
                c3 = 1.0 / 360.0 + t2 / 7560.0 + t4 / 201600.0;
            }
            else
            {
                // 精确公式 (大角度时无相消危险)
                double half_theta = 0.5 * theta;
                double cot_half = 1.0 / std::tan(half_theta);
                double c1 = half_theta * cot_half;

                eta = (1.0 - c1) / (theta * theta);
                c2 = (0.5 * cot_half - 0.25 * theta / (std::sin(half_theta) * std::sin(half_theta))) / theta;
                c3 = -(c2 + 2.0 * eta) / (theta * theta);
            }

            // 内力弯矩的反对称矩阵
            Eigen::Matrix3d skew_m;
            SkewSymmetric(m, skew_m);

            double v_dot_m = v.dot(m);

            // 组装极度复杂的解析微积分解析解 (注意这是非对称阵)
            result = c2 * m * v.transpose()
                - 0.5 * skew_m
                + eta * (v * m.transpose() + v_dot_m * Eigen::Matrix3d::Identity())
                + c3 * v_dot_m * (v * v.transpose());
        }

        // ========================================================================
        // 组装三维梁专属的 Kh 矩阵 (7x7)
        // ========================================================================
        void Assemble_Matrix_Kh(const Eigen::Vector3d& vartheta1, const Eigen::Vector3d& vartheta2, const Eigen::VectorXd& fl, Eigen::MatrixXd& _OUT result)
        {
            result = Eigen::MatrixXd::Zero(7, 7);

            // 从局部内力 fl 中精确提取两端节点的弯矩向量
            // fl(1,2,3) 对应 node 1; fl(4,5,6) 对应 node 2
            Eigen::Vector3d m1 = fl.segment<3>(1);
            Eigen::Vector3d m2 = fl.segment<3>(4);

            Eigen::Matrix3d Kh1, Kh2;
            Calculate_Khi(vartheta1, m1, Kh1);
            Calculate_Khi(vartheta2, m2, Kh2);

            // 填入对角块
            result.block<3, 3>(1, 1) = Kh1;
            result.block<3, 3>(4, 4) = Kh2;
        }

        // ========================================================================
        // 组装三维梁专属的转换矩阵 E (12x12)
        // ========================================================================
        void Assemble_Matrix_E(const Eigen::Matrix3d& Rr, Eigen::MatrixXd& _OUT result)
        {
            // 1. 初始化为 12x12 的单位矩阵 (对角线为 1)
            result = Eigen::MatrixXd::Identity(12, 12);

            // 2. 准备局部到全局的转置矩阵
            Eigen::Matrix3d Rr_T = Rr.transpose();

            // 3. 针对三维梁的 4 个空间向量自由度 (u1, theta1, u2, theta2)
            // 直接在索引 0, 3, 6, 9 处进行块覆盖 (比用 vector 循环速度更快)
            result.block<3, 3>(0, 0) = Rr_T;
            result.block<3, 3>(3, 3) = Rr_T;
            result.block<3, 3>(6, 6) = Rr_T;
            result.block<3, 3>(9, 9) = Rr_T;
        }

        // ========================================================================
        // 组装投影矩阵 P (7x12) 和自旋矩阵 G (3x12)
        // ========================================================================
        void Assemble_Matrix_PG(const Eigen::Vector3d& p1_def, const Eigen::Vector3d& p2_def, const Eigen::Vector3d& q1, const Eigen::Vector3d& q2, const Eigen::Matrix3d& Rr, Eigen::MatrixXd& _OUT P_result, Eigen::MatrixXd& _OUT G_result)
        {
            double Ln = (p2_def - p1_def).norm();

            // 1. 将全局参考向量 q 转换到局部坐标系下
            Eigen::Matrix3d Rr_T = Rr.transpose();
            Eigen::Vector3d q1_l = Rr_T * q1;
            Eigen::Vector3d q2_l = Rr_T * q2;
            Eigen::Vector3d q_l = 0.5 * (q1_l + q2_l);

            double q_x = q_l(0);
            double q_y = q_l(1);
            if (std::abs(q_y) < 1e-12) q_y = (q_y < 0 ? -1e-12 : 1e-12);

            // 2. 计算扭转自旋系数
            double eta = q_x / (Ln * q_y);
            Eigen::Vector3d nu1(q1_l(1) / (2.0 * q_y), -q1_l(0) / (2.0 * q_y), 0.0);
            Eigen::Vector3d nu2(q2_l(1) / (2.0 * q_y), -q2_l(0) / (2.0 * q_y), 0.0);

            // ==========================================
            // 3. 构建局部空间自旋矩阵 G (3x12)
            // ==========================================
            G_result = Eigen::MatrixXd::Zero(3, 12);

            // 极其关键的修正：这里才是绝对正确的空间物理自旋符号！
            // G_u1: 节点1平移带来的自旋
            G_result(0, 2) = eta;       // 对应 u1_z 带来的 X 轴偏转
            G_result(1, 2) = 1.0 / Ln;  // 对应 u1_z 带来的 Y 轴偏转
            G_result(2, 1) = -1.0 / Ln;  // 对应 u1_y 带来的 Z 轴偏转

            // G_u2: 节点2平移带来的自旋 
            G_result.block<3, 3>(0, 6) = -G_result.block<3, 3>(0, 0);

            // G_omega1: 节点1旋转带来的扭转自旋
            G_result(0, 3) = nu1(0);
            G_result(0, 4) = nu1(1);

            // G_omega2: 节点2旋转带来的扭转自旋
            G_result(0, 9) = nu2(0);
            G_result(0, 10) = nu2(1);

            // ==========================================
            // 4. 构建刚体运动滤渣器矩阵 P (7x12)
            // ==========================================
            P_result = Eigen::MatrixXd::Zero(7, 12);

            // 第一行：纯轴向拉伸
            P_result(0, 0) = -1.0;
            P_result(0, 6) = 1.0;

            // 放入单位矩阵提取旋转自由度
            P_result.block<3, 3>(1, 3) = Eigen::Matrix3d::Identity();
            P_result.block<3, 3>(4, 9) = Eigen::Matrix3d::Identity();

            // 减去刚体自旋部分！纯变形 = 总变形 - 物理自旋
            P_result.block<3, 12>(1, 0) -= G_result;
            P_result.block<3, 12>(4, 0) -= G_result;
        }


        void Assemble_stress_k(double L, const Eigen::VectorXd& fa, const Eigen::MatrixXd& G, const Eigen::MatrixXd& P, const Eigen::Matrix3d& Rr, const Eigen::Vector3d& q1_global, const Eigen::Vector3d& q2_global, const Eigen::Vector3d& r1, Eigen::MatrixXd& _OUT result)
        {
            Eigen::Matrix3d Rr_T = Rr.transpose();
            Eigen::Vector3d q1_l = Rr_T * q1_global;
            Eigen::Vector3d q2_l = Rr_T * q2_global;
            Eigen::Vector3d q_l = 0.5 * (q1_l + q2_l);

            double q_x = q_l(0);
            double q_y = q_l(1);
            if (std::abs(q_y) < 1e-12) q_y = (q_y < 0 ? -1e-12 : 1e-12);

            double q1x = q1_l(0), q1y = q1_l(1);
            double q2x = q2_l(0), q2y = q2_l(1);

            const double eta = q_x / (L * q_y);
            const double eta11 = q1x / q_y;
            const double eta12 = q1y / q_y;
            const double eta21 = q2x / q_y;
            const double eta22 = q2y / q_y;

            const double N = fa(0);

            Eigen::MatrixXd E = Eigen::MatrixXd::Zero(12, 12);
            E.block<3, 3>(0, 0) = Rr;  E.block<3, 3>(3, 3) = Rr;
            E.block<3, 3>(6, 6) = Rr;  E.block<3, 3>(9, 9) = Rr;
            Eigen::MatrixXd ET = E.transpose();

            // Km1
            Eigen::MatrixXd Km1 = Eigen::MatrixXd::Zero(12, 12);
            const double N_L = N / L;
            Km1(1, 1) = N_L;  Km1(1, 7) = -N_L;
            Km1(2, 2) = N_L;  Km1(2, 8) = -N_L;
            Km1(7, 1) = -N_L;  Km1(7, 7) = N_L;
            Km1(8, 2) = -N_L;  Km1(8, 8) = N_L;

            // Km2
            Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(12, 3);
            VectorXd m(6);
            m << fa(1), fa(2), fa(3), fa(4), fa(5), fa(6);
            
            Eigen::MatrixXd P_rot = P.bottomRows(6);
            Eigen::VectorXd PTm = P_rot.transpose() * m;   // 12x1 结果

            Vector3d n1 = PTm.block<3, 1>(0, 0);
            Vector3d m1 = PTm.block<3, 1>(3, 0);
            Vector3d n2 = PTm.block<3, 1>(6, 0);
            Vector3d m2 = PTm.block<3, 1>(9, 0);

            Matrix3d skew_n1, skew_m1, skew_n2, skew_m2;
            SkewSymmetric(n1, skew_n1);
            SkewSymmetric(m1, skew_m1);
            SkewSymmetric(n2, skew_n2);
            SkewSymmetric(m2, skew_m2);

            Q.block<3, 3>(0, 0) = skew_n1;
            Q.block<3, 3>(3, 0) = skew_m1;
            Q.block<3, 3>(6, 0) = skew_n2;
            Q.block<3, 3>(9, 0) = skew_m2;

            // G 是 3x12，G.transpose() 是 12x3
            //std::cout << MatrixXd(Q) << std::endl << std::endl;
            //std::cout << MatrixXd(E) << std::endl << std::endl;
            //std::cout << MatrixXd(G) << std::endl << std::endl;

            Eigen::MatrixXd Km2 = E * Q * G * E.transpose();

            // Km3
            Eigen::Vector3d a;
            a(0) = 0.0;
            a(1) = (eta / L) * (fa(1) + fa(4)) - (1.0 / L) * (fa(2) + fa(5));
            a(2) = (1.0 / L) * (fa(3) + fa(6));

            VectorXd r = VectorXd::Zero(12);
            r.block<3, 1>(0, 0) = -r1;
            r.block<3, 1>(6, 0) = r1;

            Eigen::MatrixXd Km3 = E * G.transpose() * a * r.transpose();  // 12x12

            result = Km1 - Km2 + Km3;
        }
    }
}