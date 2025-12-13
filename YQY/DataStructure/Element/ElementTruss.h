#pragma once
#include "ElementBase.h"
class ElementTruss : public ElementBase
{
public:
    ElementTruss();
    int Get_NodeDOF() const override { return 3; };
};

