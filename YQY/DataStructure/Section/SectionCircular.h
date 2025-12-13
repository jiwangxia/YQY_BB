#pragma once
#include "SectionBase.h"
class SectionCircular : public SectionBase
{
public:
    double m_Radius = 0.0;   //半径
    void Calculate_Area() override;
};

