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
#include <QString>
#include <functional>
#include "Solver/Constraint/NonlinearMPC.h"

namespace SolverNameSpace
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

        bool isDynamic = false;

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

        /** @brief 开始一个 Newmark 动力时间步并建立预测状态 */
        virtual void BeginDynamicStep(double dt, double beta, double gamma) = 0;

        /**
         * @brief 应用一次动力 Newton 修正
         *
         * 平动按普通 Newmark 公式更新；转动由节点在 SO(3) 上更新姿态，
         * 并在材料坐标系内更新角速度和角加速度。
         */
        virtual void ApplyDynamicCorrection(const Vec& dx, double a0, double a1) = 0;

        /** @brief 放弃当前动力时间步并恢复步初状态 */
        virtual void RollbackDynamicStep() = 0;

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
         * @param[out] K 等效刚度矩阵
         * @param[out] isDynamic 是否为动力分析
         */
        virtual void Assemble_Matrix(SpMat& Keff, bool isDynamic) = 0;

        /**
         * @brief 组装当前试探状态的一致质量矩阵、陀螺切线和惯性力
         */
        virtual void AssembleDynamicSystem(
            SpMat& mass,
            SpMat& gyroscopic,
            SpMat& centrifugal)
        {
            mass.resize(GetFreeDofs(), GetFreeDofs());
            gyroscopic.resize(GetFreeDofs(), GetFreeDofs());
            centrifugal.resize(GetFreeDofs(), GetFreeDofs());
            mass.setZero();
            gyroscopic.setZero();
            centrifugal.setZero();
        }

        /** @brief 组装当前试探状态下的非线性主从约束。 */
        virtual bool AssembleNonlinearMPC(
            NonlinearMPCData& constraints)
        {
            constraints.Clear();
            return true;
        }

        /** @brief 保存收敛状态的MPC乘子，用于约束力和反力恢复。 */
        virtual void SetNonlinearMPCMultipliers(
            const Vec& multipliers)
        {
            Q_UNUSED(multipliers);
        }

        // ============ 力计算 ============
        /**
         * @brief 计算外荷载向量
         *
         * @param[in] time 当前时间
         * @param[in] loadFactor 荷载因子（静力增量时使用）
         * @param[out] F1 约束自由度对应的外力向量
         * @param[out] F2 自由自由度对应的外力向量
         */
        virtual void ComputeExternalForce(double time, double loadFactor, Vec& F1, Vec& F2) = 0;

        // ============ 约束位移计算 ============
        /**
         * @brief 计算位移向量
         *
         * @param[out] x1 约束自由度对应的位移向量
         * @param[in] currentTime 当前分析步伪时间
         * @param[in] factor 位移因子（静力增量时使用）
         */
        virtual void Assemble_Constraint(
            Vec& x1,
            double currentTime,
            double factor) = 0;

        /**
         * @brief 计算残差向量
         *
         * 静力学: R = F_ext - F_int
         * 动力学: R = F_ext - F_int - M*a - C*v
         *
         * @param[in] F_ext 外荷载向量（自由自由度）
         * @param[out] R 残差向量
         */
        virtual void ComputeResidual(const Vec& F_ext, Vec& R) = 0;

        /**
         * @brief 计算不含惯性力的静力残差 F_ext-F_int
         *
         * HHT-alpha 方法使用该量在相邻时间步之间进行加权。
         */
        virtual void ComputeStaticResidual(const Vec& F_ext, Vec& R)
        {
            ComputeResidual(F_ext, R);
        }

        /**
         * @brief 计算反力向量
         * @param [out] F1 约束自由度对应的反力向量
         */
        virtual void CalculateReactions(Vec& F1) = 0;

        // ============ 回调 ============
        /**
         * @brief 每步结束后的回调
         * 用于保存结果、输出日志等
         * @param[in] time 当前时间
         */
        virtual void OnStepCompleted(double time) = 0;

        virtual void RecordStepIterations(double time, int iterations)
        {
            Q_UNUSED(time);
            Q_UNUSED(iterations);
        }

        /**
         * @brief 提交当前已经收敛或接受的增量步内部状态
         */
        virtual void CommitState() = 0;

        virtual void BackupStepState() = 0;
        virtual void GetStepIncrement(SolverNameSpace::Vec& dx_step) const = 0;

        /** @brief 查询宿主任务是否请求取消。求解器在增量/时间步边界调用。 */
	virtual bool IsCancellationRequested() const { return false; }

        /** @brief 向宿主任务报告当前分析步进度，progress 取值 0~1。 */
        virtual void ReportProgress(double progress, const QString& message = QString())
        {
            Q_UNUSED(progress);
            Q_UNUSED(message);
        }
    };

    /** @brief 步回调函数类型 */
    using StepCallback = std::function<void(int step, double time, const Vec& u)>;
}
