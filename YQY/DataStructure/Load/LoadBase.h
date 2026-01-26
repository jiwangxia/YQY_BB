#pragma once
#include "Base/Base.h"

/**
 * @brief 荷载基类 - 所有荷载类型的公共基类
 */
class LoadBase : public Base
{
public:
    LoadBase() {};
    EnumKeyword::LoadType m_LoadType = EnumKeyword::LoadType::UNKNOWN;  ///< 荷载类型

    int m_StepId = 0;               // 作用的分析步ID
};

