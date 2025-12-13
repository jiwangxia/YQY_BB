#pragma once
#include "Base/Base.h"
class SectionBase : public Base
{
public:
    SectionBase();

    double m_Area = 0.0;   //截面面积

    virtual void Calculate_Area() = 0;
    double Get_AreaValue() const { return m_Area; };
};

