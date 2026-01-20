#pragma once
#include "Base/Base.h"

class Material;
class SectionBase;

/**
 * @brief 属性类 - 关联材料和截面
 */
class Property : public Base
{
public:
    Property();
    std::weak_ptr<Material>    m_pMaterial;   ///< 关联的材料
    std::weak_ptr<SectionBase> m_pSection;    ///< 关联的截面
};

