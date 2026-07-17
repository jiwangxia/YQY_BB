#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace SolverNameSpace
{
    struct RayleighCoefficients
    {
        double alphaM = 0.0; ///< 质量比例系数，单位 1/s
        double betaK = 0.0;  ///< 刚度比例系数，单位 s
    };

    /**
     * @brief 参数计算结果；只保存标量，不依赖分析步、单元或动力求解器。
     */
    struct RayleighDampingResult
    {
        RayleighCoefficients coefficients;
        double frequency1Hz = 0.0;
        double frequency2Hz = 0.0;
        double omega1 = 0.0;
        double omega2 = 0.0;
        double dampingRatio1 = 0.0;
        double dampingRatio2 = 0.0;

        double DampingRatioAtFrequency(double frequencyHz) const
        {
            if (!std::isfinite(frequencyHz) || frequencyHz <= 0.0)
                throw std::invalid_argument("frequency must be finite and positive");
            constexpr double twoPi = 6.283185307179586476925286766559;
            const double omega = twoPi * frequencyHz;
            return 0.5 * (coefficients.alphaM / omega + coefficients.betaK * omega);
        }

        std::string ToText() const
        {
            std::ostringstream output;
            output << std::setprecision(12)
                << "Rayleigh damping: C = alphaM*M + betaK*K\n"
                << "  f1 = " << frequency1Hz << " Hz, zeta1 = " << dampingRatio1 << "\n"
                << "  f2 = " << frequency2Hz << " Hz, zeta2 = " << dampingRatio2 << "\n"
                << "  alphaM (y1) = " << coefficients.alphaM << " 1/s\n"
                << "  betaK  (y2) = " << coefficients.betaK << " s";
            return output.str();
        }
    };

    /** 由两个频率点及其目标阻尼比计算 C = alphaM*M + betaK*K 的系数。 */
    inline RayleighCoefficients CalculateRayleighFromFrequencies(
        double frequency1Hz,
        double dampingRatio1,
        double frequency2Hz,
        double dampingRatio2)
    {
        if (!std::isfinite(frequency1Hz) || !std::isfinite(frequency2Hz) ||
            frequency1Hz <= 0.0 || frequency2Hz <= 0.0)
            throw std::invalid_argument("Rayleigh reference frequencies must be finite and positive");

        if (!std::isfinite(dampingRatio1) || !std::isfinite(dampingRatio2) ||
            dampingRatio1 < 0.0 || dampingRatio2 < 0.0)
            throw std::invalid_argument("Rayleigh damping ratios must be finite and non-negative");

        constexpr double twoPi = 6.283185307179586476925286766559;
        const double omega1 = twoPi * frequency1Hz;
        const double omega2 = twoPi * frequency2Hz;
        const double denominator = omega2 * omega2 - omega1 * omega1;
        const double scale = std::max(omega1 * omega1, omega2 * omega2);
        if (std::abs(denominator) <= 1.0e-12 * scale)
            throw std::invalid_argument("Rayleigh reference frequencies must be distinct");

        RayleighCoefficients result;
        result.alphaM = 2.0 * omega1 * omega2 *
            (dampingRatio1 * omega2 - dampingRatio2 * omega1) / denominator;
        result.betaK = 2.0 *
            (dampingRatio2 * omega2 - dampingRatio1 * omega1) / denominator;

        const double coefficientScale = std::max(1.0, std::abs(result.alphaM) + std::abs(result.betaK));
        if (result.alphaM < -1.0e-12 * coefficientScale ||
            result.betaK < -1.0e-12 * coefficientScale)
            throw std::invalid_argument("Rayleigh targets produce a negative damping coefficient");

        result.alphaM = std::max(0.0, result.alphaM);
        result.betaK = std::max(0.0, result.betaK);
        return result;
    }

    /**
     * @brief 面向调用方的独立接口：输入 Hz 和阻尼比，输出系数及校核信息。
     */
    inline RayleighDampingResult SolveRayleighDamping(
        double frequency1Hz,
        double dampingRatio1,
        double frequency2Hz,
        double dampingRatio2)
    {
        constexpr double twoPi = 6.283185307179586476925286766559;
        RayleighDampingResult result;
        result.coefficients = CalculateRayleighFromFrequencies(
            frequency1Hz, dampingRatio1, frequency2Hz, dampingRatio2);
        result.frequency1Hz = frequency1Hz;
        result.frequency2Hz = frequency2Hz;
        result.omega1 = twoPi * frequency1Hz;
        result.omega2 = twoPi * frequency2Hz;
        result.dampingRatio1 = dampingRatio1;
        result.dampingRatio2 = dampingRatio2;
        return result;
    }

    /** 相同目标阻尼比时的便捷重载。 */
    inline RayleighDampingResult SolveRayleighDamping(
        double frequency1Hz,
        double frequency2Hz,
        double dampingRatio)
    {
        return SolveRayleighDamping(
            frequency1Hz, dampingRatio, frequency2Hz, dampingRatio);
    }
}
