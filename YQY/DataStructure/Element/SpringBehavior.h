#pragma once

#include "Base/Base.h"

#include <vector>

// 非线性弹簧的一个力-相对位移数据点，输入顺序为力、位移。
struct SpringForceDisplacementPoint
{
    double force = 0.0;
    double displacement = 0.0;
};

// 可由多个弹簧单元共享的力-位移本构曲线。
class SpringBehavior : public Base
{
public:
    std::vector<SpringForceDisplacementPoint> m_Points; // 按位移升序排列的本构数据点
};
