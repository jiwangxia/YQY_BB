#pragma once
#include "SectionBase.h"

/**
 * @brief 圆形截面类
 */
class SectionCircular : public SectionBase
{
public:
    double m_Radius = 0.0;  ///< 半径

    /**
     * @brief 根据半径计算截面面积
     */
    void Calculate_Area() override;
};

