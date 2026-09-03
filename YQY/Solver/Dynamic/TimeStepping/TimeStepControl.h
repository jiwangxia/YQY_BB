/**
 * @file TimeStepControl.h
 * @brief 时间步接受判定与步长调整策略。
 */
#pragma once

namespace SolverNameSpace
{
enum class TimeStepMode
{
    Fixed,    ///< 始终采用用户指定的标准步长；仅在非线性求解失败时按求解器自身规则回退。
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
    double minimumTimeStep = 1.0e-6;      ///< 缩步下限；达到该值仍不合格时终止求解。
    double maximumTimeStep = 0.5;         ///< 增步上限；避免平稳阶段无限放大时间步。
    double relativeTolerance = 1.0e-3;    ///< 与当前位移量级相乘的相对容差。
    double absoluteTolerance = 1.0e-6;    ///< 位移接近零时仍允许的绝对误差。
    bool includeVelocityInError = false;  ///< 是否将速度纳入误差；速度与位移量纲不同，默认仅控制位移。
    double safetyFactor = 0.9;            ///< 理论步长乘以该系数，给误差预测留出余量。
    double shrinkFactor = 0.5;            ///< 拒绝时允许的最大比例；也限制接受步不会缩得过慢。
    double maximumGrowthFactor = 2.0;     ///< 单次接受后允许的最大增步比例。
    int maximumRejectedAttempts = 12;     ///< 同一物理时刻最多允许的拒绝重试次数。
};

/** @brief 一次步长判定的结果。 */
struct TimeStepDecision
{
    bool accepted = false;    ///< true 时提交当前试算状态；false 时回滚并重试。
    double nextTimeStep = 0.0; ///< 下一次尝试应使用的时间步长。
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
    TimeStepDecision DecideAccepted(double currentTimeStep, double normalizedError, int methodOrder) const;

    /**
     * @brief 为误差超限或试算失败的时间步提出缩步重试方案。
     * @param currentTimeStep 被拒绝的时间步长。
     * @param normalizedError 已知时使用误差阶数预测缩步；非有限值表示求解失败，直接使用 shrinkFactor。
     * @param methodOrder 积分器全局阶数。
     * @return accepted 恒为 false，nextTimeStep 不小于 minimumTimeStep。
     */
    TimeStepDecision DecideRejected(double currentTimeStep, double normalizedError, int methodOrder) const;

private:
    AdaptiveTimeStepSettings settings_;
};
}
