#pragma once
#include "LoadBase.h"
class Force_Wind : public LoadBase
{
public:
    Force_Wind() { m_LoadType = EnumKeyword::LoadType::FORCE_WIND; }
    double m_velocity = 0.0;    // 风速
    double m_windDensity = 1.225;   // 风密度，海平面标准空气密度 kg/m³
};

