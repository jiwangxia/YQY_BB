/**
 * @file IAnalysisModel.h
 * @brief 分析模型接口 - 求解器通过此接口访问物理数据
 * 
 * 职责划分：
 * - Model 管理物理状态（位移、速度、加速度）
 * - Model 负责组装矩阵和计算力
 * - Solver 只负责算法（迭代、时间积分）
 */
#pragma once
#include <Eigen/Sparse>
#include <functional>

namespace SolverNS
{
    using SpMat = Eigen::SparseMatrix<double>;
    using Vec = Eigen::VectorXd;

    /**
     * @brief 分析模型接口
     * 
     * 任何需要使用求解器的类都应实现此接口。
     * 例如：AnalysisStep 实现此接口后，可以使用 SolverStatic 或 SolverNewmark。
     */
    class IAnalysisModel
    {
    public:
        virtual ~IAnalysisModel() = default;

        // ============ 基本信息 ============
        /** @brief 获取自由自由度数量 */
        virtual int GetFreeDofs() const = 0;

        /** @brief 获取约束自由度数量 */
        virtual int GetFixedDofs() const = 0;

        // ============ 状态管理 ============
        /**
         * @brief 应用位移增量
         * @param[in] dx 位移增量向量 (大小为 FreeDofs)
         * @param[in] phase Trial=迭代中试探, Commit=步末确认
         */
        virtual void ApplyIncrement(const Vec& dx) = 0;

        /**
         * @brief 设置当前试探的速度和加速度（动力学求解器使用）
         * @param[in] v 速度向量
         * @param[in] a 加速度向量
         */
        virtual void SetTrialKinematics(const Vec& v, const Vec& a) = 0;

        /**
         * @brief 获取当前状态
         * @param[out] u 位移向量
         * @param[out] v 速度向量
         * @param[out] a 加速度向量
         */
        virtual void GetState(Vec& u, Vec& v, Vec& a) const = 0;

        // ============ 矩阵组装 ============
        /**
         * @brief 组装系统矩阵
         * @param[out] K 刚度矩阵 (必须)
         * @param[out] M 质量矩阵 (动力学需要，静力可传nullptr)
         * @param[out] C 阻尼矩阵 (动力学需要，静力可传nullptr)
         */
        virtual void AssembleMatrices(SpMat& K, SpMat* M = nullptr, SpMat* C = nullptr) = 0;

        // ============ 力计算 ============
        /**
         * @brief 计算残差向量
         * 
         * 静力学: R = F_ext * loadFactor - F_int
         * 动力学: R = F_ext - F_int - M*a - C*v
         * 
         * @param[in] time 当前时间
         * @param[in] loadFactor 荷载因子（静力增量时使用）
         * @param[out] R 残差向量
         */
        virtual void ComputeResidual(double time, double loadFactor, Vec& R) = 0;

        // ============ 回调 ============
        /**
         * @brief 每步结束后的回调
         * 用于保存结果、输出日志等
         * @param[in] time 当前时间
         */
        virtual void OnStepCompleted(double time) = 0;
    };

    /** @brief 步回调函数类型 */
    using StepCallback = std::function<void(int step, double time, const Vec& u)>;
}
