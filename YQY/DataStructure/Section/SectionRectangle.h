#pragma once
#include "SectionBase.h"
class SectionRectangle : public SectionBase
{
public:
    double m_Width = 0.0;  // 矩形宽度
    double m_Height = 0.0; // 矩形高度
    /**
     * @brief 根据宽高计算截面面积
     */
    void Calculate_Area();
    /**
     * @brief 根据宽高计算截面惯性矩
     */
    void Calculate_I(_OUT double& Iy, _OUT double& Iz, _OUT double& J);
};
