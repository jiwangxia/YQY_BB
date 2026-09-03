/**
 * @file TimeStepError.h
 * @brief 时间积分步长加倍误差估计。
 */
#pragma once

#include "TimeStepControl.h"
#include "Solver/Interface/IAnalysisModel.h"

namespace SolverNameSpace
{
/**
 * @brief 用整步和两半步的结果估计局部误差，并返回归一化后的综合误差。
 *
 * 同一时间区间上，粗解由一个 dt 完成，细解由两个 dt/2 完成。对 p 阶方法，细解误差估计为
 * |fine - coarse| / (2^p - 1)。每个自由度分别按绝对/相对容差归一化，最终取
 * 最大自由度误差；仅在 includeVelocityInError 为 true 时，再在位移和速度误差
 * 之间取最大值。
 *
 * @param coarseDisplacement 一个完整步得到的步末位移。
 * @param coarseVelocity 一个完整步得到的步末速度。
 * @param fineDisplacement 两个半步得到的步末位移。
 * @param fineVelocity 两个半步得到的步末速度。
 * @param settings 绝对容差、相对容差等控制参数。
 * @param methodOrder 积分器全局阶数；Newmark 为 2，RK4 为 4。
 *
 * 相对误差尺度使用 max(|coarse|, |fine|)，与 yA/yB 误差估计公式一致。
 * @return 不大于 1 表示当前细解可接受；非有限值表示输入状态不匹配或阶数无效。
 */
double EstimateStepDoublingError(const Vec& coarseDisplacement, const Vec& coarseVelocity,
                                 const Vec& fineDisplacement, const Vec& fineVelocity,
                                 const AdaptiveTimeStepSettings& settings, int methodOrder);
}
