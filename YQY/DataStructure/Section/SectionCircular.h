#pragma once
#include "SectionBase.h"

/**
 * @brief 圆形截面类
 */
class SectionCircular : public SectionBase
{
public:

    /**
     * @brief 根据截面面积计算半径
     */
    void Calculate_Radius();

    /**
     * @brief 根据截面面积计算截面惯性矩
     */
    void Calculate_I(double& Iy, double& Iz,double& J) override;
};

