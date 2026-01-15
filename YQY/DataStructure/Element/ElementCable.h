#pragma once
#include "ElementBase.h"
class ElementCable : public ElementBase
{
public:
    ElementCable();
    int Get_NodeDOF() const override { return 4; };
};

