#pragma once

#include "AeroManager.h"

#include <vector>

// 本程序的力方向约定为 H = Cd + dCl/d(alpha)。H < 0 表示横风向平移模态存在负气动阻尼。
// 该判据仅用于快速气动预筛选，不能替代气动与结构耦合稳定性计算。
struct GallopingInstabilityInterval
{
    int profileId = 0;
    double startAngleDegrees = 0.0;
    double endAngleDegrees = 0.0;
    double minimumDenHartog = 0.0;
};

class GallopingStabilityAnalyzer final
{
public:
    // 使用与 AeroManager 相同的分段线性系数，通过 H 的解析零点确定边界，不引入角度采样误差。
    static std::vector<GallopingInstabilityInterval> FindNegativeDampingIntervals(const std::vector<BladeModel>& models,
                                                                                  double angleStepDegrees = 5.0);
};
