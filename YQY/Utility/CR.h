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
        * @brief Assemble_E_Matrix_Generic 组装通用坐标系转换矩阵 E
        * @param Rr                        当前随动坐标系矩阵 (3x3)
        * @param total_dofs                单元的总自由度数 (如 12, 6, 8)
        * @param vector_block_indices      存放需要进行 3D 旋转的 3x3 块的起始行/列索引
        * @param[out] E_result             输出转换矩阵 (total_dofs x total_dofs)
        */
        void Assemble_E_Matrix_Generic(const Eigen::Matrix3d& Rr, int total_dofs, const std::vector<int>& vector_block_indices, Eigen::MatrixXd& _OUT E_result);
    
        /**
        * @brief EulerAngles_To_RotationMatrix 欧拉角转旋转矩阵X-Y-Z
        * @param eulerAngles 输入欧拉角 (3x1)
        * @param[out] result 输出旋转矩阵 (3x3)
        */
        void EulerAngles_To_RotationMatrix(const Eigen::Vector3d& eulerAngles, Eigen::Matrix3d& _OUT result);
    }
}