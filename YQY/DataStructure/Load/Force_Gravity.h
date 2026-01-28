#pragma once
#include "LoadBase.h"
class Force_Gravity : public LoadBase
{
public:
    Force_Gravity() { m_LoadType = EnumKeyword::LoadType::FORCE_GRAVITY;}
    double m_g = -9.80665;
};

