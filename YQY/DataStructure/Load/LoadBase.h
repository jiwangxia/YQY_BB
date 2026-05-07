#pragma once
#include "Base/Base.h"

/**
 * @brief 荷载基类 - 所有荷载类型的公共基类
 */
class LoadBase : public Base
{
public:
    LoadBase() {};
    EnumKeyword::LoadType   m_LoadType = EnumKeyword::LoadType::UNKNOWN;  ///< 荷载类型
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN;  ///< 荷载方向

    int m_StepId = 0;               // 作用的分析步ID

    double m_StartTime = 0.0;       // 起始时间（相对于所属分析步）
    double m_EndTime = 1e10;        // 结束时间（默认很大，表示全程作用）

    bool IsActive(double time) const 
    {
        return (time >= m_StartTime && time <= m_EndTime);
    }
};

