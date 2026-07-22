#pragma once
#include "Base/Base.h"
#include <cmath>

/**
 * @brief 时间函数类型枚举
 */
enum class TimeFunctionType
{
    CONSTANT,      // 常数
    SIN,           // 正弦
    COS,           // 余弦
    RAMP,          // 斜坡
    EXPONENTIAL,   // 指数衰减
    TRIANGULAR,    // 三角波
    SQUARE         // 方波
};

/**
 * @brief 荷载基类 - 所有荷载类型的公共基类
 */
class LoadBase : public Base
{
public:
	LoadBase() {};
    QString m_Name;                 // 用户可读名称；为空时界面回退为 Load-ID
    EnumKeyword::LoadType   m_LoadType = EnumKeyword::LoadType::UNKNOWN;  ///< 荷载类型
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN;  ///< 荷载方向

    int m_StepId = 0;               // 作用的分析步ID

    double m_StartTime = 0.0;       // 起始时间（相对于所属分析步）
    double m_EndTime = 1e10;        // 结束时间（默认很大，表示全程作用）

    // 时间函数类型和参数
    TimeFunctionType m_FunctionType = TimeFunctionType::CONSTANT;

    // 通用参数（根据函数类型使用不同的参数）
    double m_Amplitude = 1.0;    // 幅值（SIN, COS）
    double m_Frequency = 1.0;    // 频率 Hz（SIN, COS）
    double m_Phase = 0.0;        // 相位 弧度（SIN, COS）
    double m_Offset = 0.0;       // 偏移（SIN, COS）
    double m_RampT0 = 0.0;       // 斜坡起始时间（RAMP）
    double m_RampT1 = 1.0;       // 斜坡结束时间（RAMP）
    double m_Decay = 1.0;        // 衰减系数（EXPONENTIAL）
    double m_Period = 1.0;       // 周期（TRIANGULAR, SQUARE）
    double m_DutyCycle = 0.5;    // 占空比（SQUARE）

	bool IsActive(double time) const { return (time >= m_StartTime && time <= m_EndTime); }

    /**
     * @brief 计算当前时间的荷载缩放因子
     * @param time 当前时间（绝对时间）
     * @return 缩放因子
     */
    double GetScaleFactor(double time) const
    {
        if (!IsActive(time)) return 0.0;

        // 将时间映射到相对于起始时间的局部时间
        double localTime = time - m_StartTime;

        return EvaluateTimeFunction(localTime);
    }

private:
    /**
     * @brief 计算时间函数值
     * @param t 局部时间
     * @return 函数值
     */
    double EvaluateTimeFunction(double t) const
    {
        switch (m_FunctionType)
        {
        case TimeFunctionType::CONSTANT:
            return 1.0;

        case TimeFunctionType::SIN:
        {
            double omega = 2.0 * M_PI * m_Frequency;
            return m_Offset + m_Amplitude * std::sin(omega * t + m_Phase);
        }

        case TimeFunctionType::COS:
        {
            double omega = 2.0 * M_PI * m_Frequency;
            return m_Offset + m_Amplitude * std::cos(omega * t + m_Phase);
        }

        case TimeFunctionType::RAMP:
        {
            if (t <= m_RampT0) return 0.0;
            if (t >= m_RampT1) return 1.0;
            return (t - m_RampT0) / (m_RampT1 - m_RampT0);
        }

        case TimeFunctionType::EXPONENTIAL:
            return std::exp(-m_Decay * t);

        case TimeFunctionType::TRIANGULAR:
        {
            double phase = std::fmod(t, m_Period) / m_Period;
            if (phase < 0.5)
                return 2.0 * phase;
            else
                return 2.0 * (1.0 - phase);
        }

        case TimeFunctionType::SQUARE:
        {
            double phase = std::fmod(t, m_Period) / m_Period;
            return (phase < m_DutyCycle) ? 1.0 : 0.0;
        }

        default:
            return 1.0;
        }
    }
};

