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
#include <array>
#include <functional>
#include <vector>
#include "Solver/Constraint/NonlinearMPC.h"
#include "Solver/Assembly/SystemEvaluation.h"
#include "Base/EmptyOUT.h"

namespace SolverNameSpace
{
using SpMat = Eigen::SparseMatrix<double>;
using Vec = Eigen::VectorXd;

/** 六自由度节点的真实姿态和空间转动自由度；固定分量的编号为 -1。 */
struct FiniteRotationState
{
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    std::array<int, 3> freeDofs{-1, -1, -1};
};

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

    /// 由模型决定自动模式是否值得尝试 GPU 直接求解。
    virtual bool ShouldPreferGpuLinearSolver() const
    {
        return true;
    }

    /** @brief 返回整体稀疏矩阵非零结构版本，0 表示版本未知。 */
    virtual std::uint64_t GetSparsePatternVersion() const
    {
        return 0;
    }

    /** @brief 当前模型的质量矩阵数值是否会随试探状态变化。 */
    virtual bool HasStateDependentMass() const
    {
        return true;
    }

    virtual QString DescribeFreeDof(int freeDof) const
    {
        return QStringLiteral("free_dof=%1").arg(freeDof);
    }

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

    /** @brief 保存和恢复当前 Newton 试探状态，供全局化算法使用。 */
    virtual bool SaveTrialState()
    {
        return false;
    }
    virtual bool RestoreTrialState()
    {
        return false;
    }

    /**
     * @brief 保存一个可跨越多个试算步的状态检查点。
     *
     * 自适应步长的整步/两半步比较需要恢复到同一物理步初状态，不能复用 Newton
     * 线搜索的单层试探状态。
     */
    virtual bool PushStateCheckpoint()
    {
        return false;
    }

    /** @brief 恢复最近一次状态检查点。 */
    virtual bool RestoreStateCheckpoint()
    {
        return false;
    }

    /** @brief 丢弃最近一次状态检查点而不恢复。 */
    virtual void DiscardStateCheckpoint()
    {
    }

    /**
         * @brief 设置当前试探的速度和加速度（动力学求解器使用）
         * @param[in] v 速度向量
         * @param[in] a 加速度向量
         */
    virtual void SetTrialKinematics(const Vec& v, const Vec& a) = 0;

    /** @brief 普通 TSSBN 设置运动学时保留独立步内转角；普通向量模型沿用通用接口。 */
    virtual void SetTssbnTrialKinematics(const Vec& v, const Vec& a)
    {
        SetTrialKinematics(v, a);
    }

    /**
     * @brief 建立一个 TSSBN 试算阶段的速度和加速度状态。
     *
     * 默认实现适用于平动和普通向量自由度。有限转动模型应重写该函数，在 SO(3) 上积分材料角速度，
     * 并覆盖 velocity 和 acceleration 中的转动分量。
     */
    virtual void SetTssbnStageKinematics(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                         double secondStageDiagonalFraction, _OUT Vec& velocity, _OUT Vec& acceleration,
                                         bool useStoredIncrement = false)
    {
        Q_UNUSED(stageIndex);
        Q_UNUSED(timeStep);
        Q_UNUSED(firstStageTime);
        Q_UNUSED(secondStageTime);
        Q_UNUSED(secondStageDiagonalFraction);
        Q_UNUSED(useStoredIncrement);
        SetTrialKinematics(velocity, acceleration);
    }

    /** @brief 普通隐式阶段在实际预测构型上只初始化一次未投影材料状态。 */
    virtual void BeginTssbnImplicitStage(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                         double secondStageDiagonalFraction)
    {
        Q_UNUSED(stageIndex);
        Q_UNUSED(timeStep);
        Q_UNUSED(firstStageTime);
        Q_UNUSED(secondStageTime);
        Q_UNUSED(secondStageDiagonalFraction);
    }

    /** @brief 普通阶段完整修正；普通向量模型仍由求解器独立累积速度和加速度。 */
    virtual void ApplyTssbnImplicitStageCorrection(const Vec& increment)
    {
        ApplyDynamicCorrection(increment, 0.0, 0.0);
    }

    /** @brief 普通 TSSBN 接受端点保持 SO(3)；非节点模型无需处理，参考路径不调用。 */
    virtual bool NormalizeTssbnAcceptedRotations()
    {
        return true;
    }

    /** @brief 声明有限转动阶段与当前 Node 的材料 C1/C2 映射一致；默认保留通用保守预算。 */
    virtual bool SupportsTssbnRotationPrecisionEstimate() const
    {
        return false;
    }

    /** @brief 将显式末阶段 Y3 的有限转动位移和速度变换为空间坐标；不建立试探状态。 */
    virtual void CorrectTssbnExplicitLastStage(double timeStep, double lastStageFirstCoefficient,
                                               double lastStageSecondCoefficient, _OUT Vec& stepIncrement,
                                               _OUT Vec& velocity)
    {
        Q_UNUSED(timeStep);
        Q_UNUSED(lastStageFirstCoefficient);
        Q_UNUSED(lastStageSecondCoefficient);
        Q_UNUSED(stepIncrement);
        Q_UNUSED(velocity);
    }

    /** @brief 使用满足 SO(3) 一致性的值替换基础与嵌入 TSSBN 状态中的有限转动分量。 */
    virtual void CorrectTssbnStepStates(double timeStep, double firstStageTime, double secondStageTime,
                                        double lastStageTime, double baseFirstWeight, double embeddedFirstWeight,
                                        double embeddedSecondWeight, double embeddedLastWeight,
                                        double lastStageFirstCoefficient, double lastStageSecondCoefficient,
                                        _OUT Vec& baseIncrement, _OUT Vec& baseVelocity, _OUT Vec& embeddedIncrement,
                                        _OUT Vec& embeddedVelocity, _OUT Vec& acceptedAcceleration)
    {
        Q_UNUSED(timeStep);
        Q_UNUSED(firstStageTime);
        Q_UNUSED(secondStageTime);
        Q_UNUSED(lastStageTime);
        Q_UNUSED(baseFirstWeight);
        Q_UNUSED(embeddedFirstWeight);
        Q_UNUSED(embeddedSecondWeight);
        Q_UNUSED(embeddedLastWeight);
        Q_UNUSED(lastStageFirstCoefficient);
        Q_UNUSED(lastStageSecondCoefficient);
        Q_UNUSED(baseIncrement);
        Q_UNUSED(baseVelocity);
        Q_UNUSED(embeddedIncrement);
        Q_UNUSED(embeddedVelocity);
        Q_UNUSED(acceptedAcceleration);
    }

    /** @brief 仅参考线路使用：六维节点的 LTE 位移填材料增量，步初状态填主转角。 */
    virtual bool CorrectReferenceTssbnErrorDisplacements(
        double timeStep, double baseFirstWeight, double embeddedFirstWeight, double embeddedSecondWeight,
        double embeddedLastWeight, double lastStageFirstCoefficient, double lastStageSecondCoefficient,
        _OUT Vec& stepStartDisplacement, _OUT Vec& baseDisplacement, _OUT Vec& embeddedDisplacement) const
    {
        Q_UNUSED(timeStep);
        Q_UNUSED(baseFirstWeight);
        Q_UNUSED(embeddedFirstWeight);
        Q_UNUSED(embeddedSecondWeight);
        Q_UNUSED(embeddedLastWeight);
        Q_UNUSED(lastStageFirstCoefficient);
        Q_UNUSED(lastStageSecondCoefficient);
        Q_UNUSED(stepStartDisplacement);
        Q_UNUSED(baseDisplacement);
        Q_UNUSED(embeddedDisplacement);
        return false;
    }

    /** @brief 用真实显式末阶段加速度修正嵌入速度的有限转动分量；不修改接受加速度。 */
    virtual void CorrectTssbnEmbeddedVelocity(double timeStep, double embeddedFirstWeight,
                                              double embeddedSecondWeight, double embeddedLastWeight,
                                              double lastStageFirstCoefficient, double lastStageSecondCoefficient,
                                              const Vec& explicitLastStageAcceleration, _OUT Vec& embeddedVelocity)
    {
        Q_UNUSED(timeStep);
        Q_UNUSED(embeddedFirstWeight);
        Q_UNUSED(embeddedSecondWeight);
        Q_UNUSED(embeddedLastWeight);
        Q_UNUSED(lastStageFirstCoefficient);
        Q_UNUSED(lastStageSecondCoefficient);
        Q_UNUSED(explicitLastStageAcceleration);
        Q_UNUSED(embeddedVelocity);
    }

    /**
         * @brief 获取当前状态
         * @param[out] u 位移向量
         * @param[out] v 速度向量
         * @param[out] a 加速度向量
         */
    virtual void GetState(_OUT Vec& u, _OUT Vec& v, _OUT Vec& a) const = 0;

    /** 返回全部有限转动节点，供群上的积分及真实姿态误差估计使用。 */
    virtual void GetFiniteRotationStates(_OUT std::vector<FiniteRotationState>& states) const
    {
        states.clear();
    }

    /**
     * @brief 返回当前 CR 二阶辅助姿态，顺序在一次求解期间必须稳定。
     *
     * 自适应 RK4 将非空辅助姿态连同节点按整体二阶控制；默认空保持普通模型的四阶控制。
     * 这里只查询当前试探状态，不提交历史；相应历史必须已纳入模型检查点。
     */
    virtual void GetAuxiliaryFiniteRotationStates(_OUT std::vector<Eigen::Matrix3d>& states) const
    {
        states.clear();
    }

    // ============ 矩阵组装 ============
    /**
         * @brief 组装系统矩阵
         * @param[out] K 等效刚度矩阵
         * @param[out] isDynamic 是否为动力分析
         */
    virtual void Assemble_Matrix(_OUT SpMat& Keff, bool isDynamic) = 0;

    /**
         * @brief 组装当前试探状态的一致质量矩阵、陀螺切线和惯性力
         */
    virtual void AssembleDynamicSystem(_OUT SpMat& mass, _OUT SpMat& gyroscopic, _OUT SpMat& centrifugal)
    {
        mass.resize(GetFreeDofs(), GetFreeDofs());
        gyroscopic.resize(GetFreeDofs(), GetFreeDofs());
        centrifugal.resize(GetFreeDofs(), GetFreeDofs());
        mass.setZero();
        gyroscopic.setZero();
        centrifugal.setZero();
    }

    /**
     * @brief 装配空间加速度的物理质量矩阵，用于显式积分、初始加速度及模态/阻尼标定。
     *
     * 有限转动模型必须覆盖此接口，不能返回隐式增量切线质量；默认实现兼容普通向量模型。
     */
    virtual void AssembleMassMatrix(_OUT SpMat& mass)
    {
        SpMat velocityTangent;
        SpMat configurationTangent;
        AssembleDynamicSystem(mass, velocityTangent, configurationTangent);
    }

    virtual void AssembleEffectiveDynamicSystem(double accelerationDerivative, double velocityDerivative,
                                                _OUT SpMat& effectiveDynamicTangent)
    {
        SpMat mass;
        SpMat velocityTangent;
        SpMat configurationTangent;
        AssembleDynamicSystem(mass, velocityTangent, configurationTangent);
        effectiveDynamicTangent =
            accelerationDerivative * mass + velocityDerivative * velocityTangent + configurationTangent;
    }

    virtual void AssembleEffectiveTangent(double accelerationDerivative, double velocityDerivative,
                                          _OUT SpMat& effectiveTangent)
    {
        SpMat stiffness;
        SpMat dynamicTangent;
        Assemble_Matrix(stiffness, true);
        AssembleEffectiveDynamicSystem(accelerationDerivative, velocityDerivative, dynamicTangent);
        effectiveTangent = stiffness + dynamicTangent;
    }

    /**
     * @brief 刷新当前试探位移对应的单元内力
     *
     * 显式积分器会在一个时间步内建立多个试探状态。模型必须在计算每个阶段的
     * 残差前更新内力；默认实现借助刚度组装兼容已有模型。
     */
    virtual void UpdateInternalForce()
    {
        SpMat unusedStiffness;
        Assemble_Matrix(unusedStiffness, true);
    }

    /** @brief 刷新动力残差所需的内力和惯性力，不要求保留整体切线矩阵。 */
    virtual void RefreshResidualState()
    {
        UpdateInternalForce();
        SpMat mass;
        SpMat velocityTangent;
        SpMat configurationTangent;
        AssembleDynamicSystem(mass, velocityTangent, configurationTangent);
    }

    /** @brief 只刷新惯性力，避免生成未使用的动力矩阵。 */
    virtual void RefreshInertiaForce()
    {
        SpMat mass;
        SpMat velocityTangent;
        SpMat configurationTangent;
        AssembleDynamicSystem(mass, velocityTangent, configurationTangent);
    }

    /** @brief 装配状态相关外载对本次位移修正的导数。 */
    virtual void AssembleExternalLoadTangent(double time, double loadFactor, double velocityDerivative,
                                             _OUT SpMat& externalTangent)
    {
        Q_UNUSED(time);
        Q_UNUSED(loadFactor);
        Q_UNUSED(velocityDerivative);
        externalTangent.resize(GetFreeDofs(), GetFreeDofs());
        externalTangent.setZero();
    }

    /**
     * @brief 按统一请求计算系统贡献，供不同时间积分器复用同一组装入口。
     *
     * 默认实现兼容原有模型接口；高性能模型可重写该函数，将多个贡献合并到一次并行遍历中。
     */
    virtual void EvaluateSystem(const SystemEvaluationRequest& request, _OUT SystemEvaluationResult& result)
    {
        result = {};
        const int freeDofs = GetFreeDofs();
        const bool requiresDynamicTangents = request.Requires(SystemContribution::VelocityTangent) ||
                                             request.Requires(SystemContribution::ConfigurationTangent);
        const bool residualStateRequested = request.Requires(SystemContribution::ResidualState);
        if (request.Requires(SystemContribution::ExternalForce))
            ComputeExternalForce(request.time, request.loadFactor, result.constrainedExternalForce,
                                 result.externalForce);
        if (residualStateRequested)
            RefreshResidualState();
        if (request.Requires(SystemContribution::Stiffness))
            Assemble_Matrix(result.stiffness, true);
        if (request.Requires(SystemContribution::EffectiveDynamicTangent))
            AssembleEffectiveDynamicSystem(request.accelerationDerivative, request.velocityDerivative,
                                           result.effectiveDynamicTangent);
        if (request.Requires(SystemContribution::EffectiveTangent))
            AssembleEffectiveTangent(request.accelerationDerivative, request.velocityDerivative,
                                     result.effectiveTangent);
        if (requiresDynamicTangents)
        {
            AssembleDynamicSystem(result.mass, result.velocityTangent, result.configurationTangent);
        }
        else if (request.Requires(SystemContribution::Mass))
            AssembleMassMatrix(result.mass);
        if (request.Requires(SystemContribution::InertiaForce) && !requiresDynamicTangents &&
            !residualStateRequested)
            RefreshInertiaForce();
        if (request.Requires(SystemContribution::InertiaForce) || residualStateRequested)
            GetDynamicInertiaForce(result.inertiaForce);
        if (request.Requires(SystemContribution::Damping))
        {
            result.damping.resize(freeDofs, freeDofs);
            result.damping.setZero();
        }
        if (request.Requires(SystemContribution::InternalForce))
        {
            Vec staticResidual;
            ComputeStaticResidual(Vec::Zero(freeDofs), staticResidual);
            result.internalForce = -staticResidual;
        }
        if (request.Requires(SystemContribution::ExternalTangent))
            AssembleExternalLoadTangent(request.time, request.loadFactor, request.velocityDerivative,
                                        result.externalTangent);
        result.sparsePatternVersion = GetSparsePatternVersion();
    }

    /** @brief 返回最近一次动力系统组装得到的惯性力。 */
    virtual void GetDynamicInertiaForce(_OUT Vec& inertiaForce) const
    {
        inertiaForce = Vec::Zero(GetFreeDofs());
    }

    /** @brief 组装当前试探状态下的非线性主从约束。 */
    virtual bool AssembleNonlinearMPC(_OUT NonlinearMPCData& constraints)
    {
        constraints.Clear();
        return true;
    }

    /** @brief 当前分析步是否包含可能参与线搜索的非线性 MPC。 */
    virtual bool HasActiveNonlinearMPC() const
    {
        return false;
    }

    /** @brief 保存收敛状态的MPC乘子，用于约束力和反力恢复。 */
    virtual void SetNonlinearMPCMultipliers(const Vec& multipliers)
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
    virtual void ComputeExternalForce(double time, double loadFactor, _OUT Vec& F1, _OUT Vec& F2) = 0;

    // ============ 约束位移计算 ============
    /**
         * @brief 计算位移向量
         *
         * @param[out] x1 约束自由度对应的位移向量
         * @param[in] currentTime 当前分析步伪时间
         * @param[in] factor 位移因子（静力增量时使用）
         */
    virtual void Assemble_Constraint(_OUT Vec& x1, double currentTime, double factor) = 0;

    /**
         * @brief 计算残差向量
         *
         * 静力学: R = F_ext - F_int
         * 动力学: R = F_ext - F_int - M*a - C*v
         *
         * @param[in] F_ext 外荷载向量（自由自由度）
         * @param[out] R 残差向量
         */
    virtual void ComputeResidual(const Vec& F_ext, _OUT Vec& R) = 0;

    // 参考程序的 Newton 停止判据包含约束自由度上的反力变化。
    virtual void GetReferenceConvergenceResidual(const Vec& freeResidual, const Vec& constrainedExternalForce,
                                                  _OUT Vec& residual) const
    {
        Q_UNUSED(constrainedExternalForce);
        residual = freeResidual;
    }

    // 仅适用模型返回参考自由轴排列；空映射保持通用求解顺序。
    virtual bool GetReferenceLinearSolvePermutation(_OUT std::vector<int>& signedSource) const
    {
        signedSource.clear();
        return false;
    }

    virtual double GetReferenceVectorNorm(const Vec& vector) const
    {
        return vector.norm();
    }

    /**
         * @brief 计算不含惯性力的静力残差 F_ext-F_int
         *
         * HHT-alpha 方法使用该量在相邻时间步之间进行加权。
         */
    virtual void ComputeStaticResidual(const Vec& F_ext, _OUT Vec& R)
    {
        ComputeResidual(F_ext, R);
    }

    /**
         * @brief 根据约束自由度的外力计算并写入节点反力
         * @param [in] F1 约束自由度对应的外力向量
         */
    virtual void CalculateReactions(_OUT Vec& F1) = 0;

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
    virtual void GetStepIncrement(_OUT SolverNameSpace::Vec& dx_step) const = 0;

    /** @brief 查询宿主任务是否请求取消。求解器在增量/时间步边界调用。 */
    virtual bool IsCancellationRequested() const
    {
        return false;
    }

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
