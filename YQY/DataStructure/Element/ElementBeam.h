#pragma once
#include "ElementBase.h"
class ElementBeam : public ElementBase
{
public:
    ElementBeam();
    int Get_NodeDOF() const override { return 6; };
};

