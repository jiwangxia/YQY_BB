#pragma once
#include "LoadBase.h"
class ElementBase;

/**
 * @brief 单元荷载类 - 施加在单元上的荷载
 */
class Force_Element : public LoadBase
{
public:
    Force_Element() { m_LoadType = EnumKeyword::LoadType::FORCE_ELEMENT; }

    std::weak_ptr<ElementBase> m_pElement;  ///< 荷载所在单元
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN;  ///< 荷载方向

    double m_Value = 0.0;  ///< 荷载值
};

