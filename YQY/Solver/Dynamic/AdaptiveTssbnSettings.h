#pragma once

namespace SolverNameSpace
{
    /// 用户可配置的自适应 TSSBN 参数。
    /// 默认值集中在此处，避免模型、求解器与界面出现不一致。
    struct AdaptiveTssbnSettings
    {
        double spectralRadiusInfinity = 0.0;
        double minimumTimeStep = 1.0e-6;
        double maximumTimeStep = 0.5;
        double relativeTolerance = 2.0e-4;
        double absoluteTolerance = 1.0e-6;
        double safetyFactor = 0.9;
        double shrinkFactor = 0.8;
        double maximumGrowthFactor = 3.0;
        int targetNewtonIterations = 16;
        double derivativeGain = 0.1;
        double minimumDerivativeFactor = 0.5;
        double maximumDerivativeFactor = 1.5;
        int maximumRejectedAttempts = 24;
    };
}
