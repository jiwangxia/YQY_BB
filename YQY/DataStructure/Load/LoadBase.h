#pragma once
#include "Base/Base.h"
class LoadBase : public Base
{
public:
    LoadBase() {};
    EnumKeyword::LoadType m_LoadType = EnumKeyword::LoadType::UNKNOWN;
    //virtual ~LoadBase() = 0;
};

