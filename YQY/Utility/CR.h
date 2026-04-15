#pragma once
#include <Eigen/Dense>
#include <Base/Base.h>

namespace Utility
{
    namespace CR
    {
        /**
         * @brief SkewSymmetric 计算向量的反对称矩阵
         * @param v             输入向量 (3x1)
         * @param[out]          反对称矩阵 (3x3)
         */
        void SkewSymmetric(const Eigen::Vector3d& v, Eigen::Matrix3d& _OUT result);

        /**
        * @brief Calculate_Ts  计算切线自旋张量 Ts
        * @param vartheta      输入旋转向量 (3x1)
        * @param[out] result   输出 Ts 矩阵 (3x3)
        */
        void Calculate_Ts(const Eigen::Vector3d& vartheta, Eigen::Matrix3d& _OUT result);

        /**
        * @brief Calculate_Ts_Inv 计算切线自旋张量 Ts 的逆矩阵
        * @param vartheta         输入旋转向量 (3x1)
        * @param[out] result      输出 Ts 的逆矩阵 (3x3)
        */
        void Calculate_Ts_Inv(const Eigen::Vector3d& vartheta, Eigen::Matrix3d& _OUT result);

        /**
        * @brief Calculate_RotationMatrix 计算旋转矩阵
        * @param vartheta      输入旋转向量 (3x1)
        * @param[out] result   输出旋转矩阵 (3x3)
        */
        void Calculate_RotationMatrix(const Eigen::Vector3d& vartheta, Eigen::Matrix3d& _OUT result);


        /**
        * @brief Calculate_Rr 计算随动坐标系 Rr (局部坐标系架设)
        * @param p1_def        局部坐标系起点
        * @param p2_def        局部坐标系终点
        * @param q             局部坐标系参考向量
        * @param[out] result   输出旋转矩阵 (3x3)
        */
        void Calculate_Rr(const Eigen::Vector3d& p1_def, const Eigen::Vector3d& p2_def, const Eigen::Vector3d& q, Eigen::Matrix3d& _OUT result);

        
        /**
        * @brief Extract_RotationVector 提取旋转向量 (矩阵 -> 向量)
        * @param R             输入旋转矩阵 (3x3)
        * @param[out] result   输出旋转向量 (3x1)
        */
        void Extract_RotationVector(const Eigen::Matrix3d& R, Eigen::Vector3d& _OUT result);

        /**
        * @brief Update_NodalRotation 节点姿态增量更新 (乘法更新法则)
        * @param delta_theta  输入节点微小物理转角 (3x1)
        * @param R_old        输入节点上一步的绝对姿态矩阵 (3x3)
        * @param[out] R_new   输出更新后的节点绝对姿态矩阵 (3x3)
        */
        void Update_NodalRotation(const Eigen::Vector3d& delta_theta, const Eigen::Matrix3d& R_old, Eigen::Matrix3d& _OUT R_new);

        /**
        * @brief EulerAngles_To_RotationMatrix 欧拉角转旋转矩阵X-Y-Z
        * @param eulerAngles 输入欧拉角 (3x1)
        * @param[out] result 输出旋转矩阵 (3x3)
        */
        void EulerAngles_To_RotationMatrix(const Eigen::Vector3d& eulerAngles, Eigen::Matrix3d& _OUT result);

        /**
        * @brief Assemble_Matrix_Ba 组装三维梁单元专属的局部物理/数学转换矩阵 Ba
        * @param vartheta1 节点1的局部纯旋转向量 (3x1)
        * @param vartheta2 节点2的局部纯旋转向量 (3x1)
        * @param[out] result 输出的 Ba 矩阵 (7x7)
        */
        void Assemble_Matrix_Ba(const Eigen::Vector3d& vartheta1, const Eigen::Vector3d& vartheta2, Eigen::MatrixXd& _OUT result);

        /**
        * @brief Assemble_Matrix_Kh 组装三维梁单元专属的局部几何刚度矩阵 Kh
        * @param vartheta1 节点1的局部纯旋转向量 (3x1)
        * @param vartheta2 节点2的局部纯旋转向量 (3x1)
        * @param fl             局部纯内力向量 (7x1)
        * @param[out] result 输出的 Kh 矩阵 (7x7)
        */
        void Assemble_Matrix_Kh(const Eigen::Vector3d& vartheta1, const Eigen::Vector3d& vartheta2, const Eigen::VectorXd& fl, Eigen::MatrixXd& _OUT result);

        /**
        * @brief Assemble_Matrix_E 组装三维梁专属的全局坐标系转换矩阵 E (12x12)
        * @param Rr              当前随动坐标系矩阵 (3x3)
        * @param[out] result     输出的 E 矩阵 (12x12)
        */
        void Assemble_Matrix_E(const Eigen::Matrix3d& Rr, Eigen::MatrixXd& _OUT result);

        /**
        * @brief Assemble_Matrix_PG 组装投影矩阵 P (7x12) 和自旋矩阵 G (3x12)
        * @param p1_def, p2_def  变形后的节点绝对坐标
        * @param q1, q2          变形后的节点截面参考向量 (Rg1*q0, Rg2*q0)
        * @param Rr              当前随动坐标系姿态矩阵
        * @param[out] P_result   输出的滤渣器矩阵 P (7x12)
        * @param[out] G_result   输出的局部自旋矩阵 G (3x12)
        */
        void Assemble_Matrix_PG(const Eigen::Vector3d& p1_def, const Eigen::Vector3d& p2_def, const Eigen::Vector3d& q1, const Eigen::Vector3d& q2, const Eigen::Matrix3d& Rr, Eigen::MatrixXd& _OUT P_result, Eigen::MatrixXd& _OUT G_result);
    
        /**
        * @brief Assemble_stress_k 组装三维梁终极全局切线刚度矩阵 (包含材料与几何非线性)
        * @param L               变形后的当前弦长
        * @param fa              包含 7 个真实物理内力的向量
        * @param G               空间自旋矩阵 (3x12)
        * @param Rr              当前随动坐标系姿态矩阵
        * @param q1_global       全局节点1旋转向量
        * @param q2_global       全局节点2旋转向量
        * @param r1              局部坐标系下的单位轴向向量 r1
        * @param[out] result  输出的终极全局刚度矩阵 ke (12x12)
        */
        void Assemble_stress_k(double L, const Eigen::VectorXd& fa, const Eigen::MatrixXd& G, const Eigen::MatrixXd& P, const Eigen::Matrix3d& Rr, const Eigen::Vector3d& q1_global, const Eigen::Vector3d& q2_global, const Eigen::Vector3d& r1, Eigen::MatrixXd& _OUT result);
    }
}