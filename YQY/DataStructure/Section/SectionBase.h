#pragma once
#include "Base/Base.h"

/**
 * @brief 截面基类 - 所有截面类型的公共基类
 */
class SectionBase : public Base
{
public:
    SectionBase();

    double m_Area = 0.0;  ///< 截面面积

    /**
     * @brief 计算截面面积（纯虚函数）
     */
    virtual void Calculate_Area() = 0;

    /**
     * @brief 获取截面面积
     * @return 截面面积值
     */
    double Get_AreaValue() const { return m_Area; };
};

