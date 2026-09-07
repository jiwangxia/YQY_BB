/**
 * @file TimeStepControl.h
 * @brief 时间步接受判定与步长调整策略。
 */
#pragma once

#include <algorithm>

namespace SolverNameSpace
{
enum class TimeStepMode
{
    Fixed,   ///< 始终采用用户指定的标准步长；仅在非线性求解失败时按求解器自身规则回退。
    Adaptive ///< 根据局部误差接受或拒绝本步，并为下一步计算新的时间步长。
};

/**
 * @brief 基于局部误差的自适应步长参数。
 *
 * 每个自由度的位移误差先按 absoluteTolerance + relativeTolerance * 该自由度状态量级归一化，
 * 再取所有自由度中的最大值；最大归一化误差不大于 1 时接受本步。
 */
struct AdaptiveTimeStepSettings
{
    double minimumTimeStep = 1.0e-6;           ///< 缩步下限；达到该值仍不合格时终止求解。
    double maximumTimeStep = 0.5;              ///< 增步上限；避免平稳阶段无限放大时间步。
    double relativeTolerance = 1.0e-3;         ///< 与当前位移量级相乘的相对容差。
    double absoluteTolerance = 1.0e-6;         ///< 位移接近零时仍允许的绝对误差。
    bool includeVelocityInError = true;        ///< 是否同时控制速度误差；长时振动建议保持启用。
    double velocityRelativeTolerance = 1.0e-3; ///< 与当前速度量级相乘的相对容差。
    double velocityAbsoluteTolerance = 1.0e-6; ///< 速度接近零时仍允许的绝对误差。
    double safetyFactor = 0.9;                 ///< 理论步长乘以该系数，给误差预测留出余量。
    double minimumScaleFactor = 0.2;           ///< 单次误差控制允许的最小步长倍率。
    double maximumGrowthFactor = 2.0;          ///< 单次误差控制允许的最大步长倍率。
    double solveFailureFactor = 0.5;           ///< 积分或非线性求解失败且没有可信误差时的缩步倍率。
    int maximumRejectedAttempts = 12;          ///< 同一物理时刻最多允许的拒绝重试次数。
};

/** @brief 一次步长判定的结果。 */
struct TimeStepDecision
{
    bool accepted = false;     ///< true 时提交当前试算状态；false 时回滚并重试。
    double nextTimeStep = 0.0; ///< 下一次尝试应使用的时间步长。
};

/** @brief 选择本次实际步长，并在接近总时长时直接吸附到终点。 */
double SelectTimeStep(double currentTime, double duration, double proposedTimeStep);

/** @brief 一次求解的时间步统计，用于结果诊断和性能验证。 */
struct TimeStepStatistics
{
    int acceptedSteps = 0;
    int rejectedAttempts = 0;
    double minimumAcceptedTimeStep = 0.0;
    double maximumAcceptedTimeStep = 0.0;

    void RecordAccepted(double timeStep)
    {
        ++acceptedSteps;
        minimumAcceptedTimeStep = acceptedSteps == 1 ? timeStep : std::min(minimumAcceptedTimeStep, timeStep);
        maximumAcceptedTimeStep = std::max(maximumAcceptedTimeStep, timeStep);
    }

    void RecordRejected()
    {
        ++rejectedAttempts;
    }
};

/**
 * @brief 将误差估计转换为“接受/拒绝 + 下一步步长”的控制器。
 *
 * 对 p 阶积分器，步长预测使用 dt_new = safetyFactor * dt * error^(-1/(p+1))。
 * 该类不保存模型状态，也不执行积分或回滚，仅负责纯数值决策。
 */
class TimeStepController final
{
public:
    /** @brief 保存并校验自适应步长参数。 */
    explicit TimeStepController(AdaptiveTimeStepSettings settings);

    /** @brief 检查步长上下限、容差和调整因子是否满足控制器要求；无效时抛出异常。 */
    void Validate() const;

    /**
     * @brief 根据已完成试算的归一化误差判定是否接受本步。
     * @param currentTimeStep 刚刚完成试算的时间步长。
     * @param normalizedError 误差估计器返回的无量纲误差；不大于 1 表示满足容差。
     * @param methodOrder 积分器全局阶数，例如 Newmark 为 2、经典 RK4 为 4。
     * @return 当前误差合格时 accepted 为 true，同时给出受上下限约束后的下一步步长。
     */
    TimeStepDecision Decide(double currentTimeStep, double normalizedError, int methodOrder) const;

    /**
     * @brief 为没有可信误差的积分或非线性求解失败提出缩步重试方案。
     * @param currentTimeStep 被拒绝的时间步长。
     * @return accepted 恒为 false，nextTimeStep 不小于 minimumTimeStep。
     */
    TimeStepDecision DecideSolveFailure(double currentTimeStep) const;

private:
    AdaptiveTimeStepSettings settings_;
};
}
