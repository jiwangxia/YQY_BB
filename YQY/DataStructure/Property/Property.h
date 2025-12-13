#pragma once
#include "Base/Base.h"

class Material;
class SectionBase;
class Property : public Base
{
public:
    Property();
    std::weak_ptr<Material>    m_pMaterial;   //材料
    std::weak_ptr<SectionBase> m_pSection;    //截面
};

