#pragma once

namespace SolverNameSpace
{
enum class TssbnErrorNormMode
{
    ComponentwiseMax = 0,
    LegacyGlobal = 1,
    DisplacementMaxVelocityRms = 2,
    PaperMixedRms = 3,
    DisplacementGlobalVelocityMax = 4,
    GlobalMax = 5
};

/// 用户可配置的自适应 TSSBN 参数。
/// 默认值集中在此处，避免模型、求解器与界面出现不一致。
struct AdaptiveTssbnSettings
{
    double spectralRadiusInfinity = 0.0;
    double minimumTimeStep = 1.0e-6;
    double maximumTimeStep = 0.5;

    double relativeTolerance = 0.001;
    double absoluteTolerance = 1.0e-6;
    // 速度误差使用速度量纲，不能直接沿用位移的绝对容限。
    double velocityAbsoluteTolerance = 1.0e-4;
    // 普通版位移、速度分别按整体向量缩放，二者取最大值。
    TssbnErrorNormMode errorNormMode = TssbnErrorNormMode::GlobalMax;
    bool useDuan2026Controller = false;
    // 旧兼容配置是否保留外推的接受端点加速度；默认 false，仍恢复物理端点加速度。
    // 普通两阶段路径的嵌入外推由求解器局部策略决定，不由此开关控制。
    bool useReferenceExtrapolatedAcceleration = false;
    // 参考线路可选逐自由度误差尺度和物理端点加速度；两种配置均采用真实 Y3 速度估计。
    bool referenceIndependentErrorControl = false;
    double safetyFactor = 0.9;
    double shrinkFactor = 0.8;
    double maximumGrowthFactor = 3.0;
    // 可选 Duan 控制器使用迭代反馈；默认比例控制器不按迭代量缩放时间步。
    int targetNewtonIterations = 8;
    int maximumTotalNewtonIterations = 32;
    // 可选 Duan 控制器的误差变化项使用负指数，原有模型参数保持兼容。
    double derivativeGain = 0.1;
    double minimumDerivativeFactor = 0.5;
    double maximumDerivativeFactor = 1.5;
    int maximumRejectedAttempts = 24;
};
}
