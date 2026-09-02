#pragma once
#include "LoadBase.h"
// 施加于全部单元质量的重力荷载。
class Force_Gravity : public LoadBase
{
public:
    Force_Gravity()
    {
        m_LoadType = EnumKeyword::LoadType::FORCE_GRAVITY;
    }
    double m_g = -9.80665; // 重力加速度，负号表示全局 Z 负向
};
