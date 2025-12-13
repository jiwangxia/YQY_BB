#pragma once
#include "Base/Base.h"
class Material :  public Base
{
public:
    Material();                 //构造函数
    double m_Young     = 0.0;   //弹性模量
    double m_Poisson   = 0.0;   //泊松比
    double m_Density   = 0.0;   //密度
    double m_MaxStress = 0.0;   //极限应力
    double m_Expansion = 0.0;   //热膨胀系数
};

