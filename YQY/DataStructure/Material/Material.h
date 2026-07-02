#pragma once
#include "Base/Base.h"
#include "Material1D.h"

/**
 * @brief 材料类 - 存储材料属性
 */
class Material : public Base
{
public:
    Material();
    double m_Young       = 0.0; ///< 弹性模量
    double m_Poisson     = 0.0; ///< 泊松比
    double m_Density     = 0.0; ///< 密度
    double m_YieldStress = 0.0; ///< 初始屈服应力
    double m_Expansion   = 0.0; ///< 热膨胀系数
    double m_Hardening   = 0.0; ///< 线性硬化模量，0 表示理想塑性

    MaterialModel m_Model = MaterialModel::Elastic; ///< 默认保持原有线弹性行为

    /**
     * @brief 根据杆单元当前总应变计算一维材料试算响应
     * @param strain 当前总应变
     * @param oldState 上一个已收敛增量步的材料状态
     * @return 当前试算应力、一维切线刚度和试算状态
     */
    Material1DResult Update1D(
        double strain,
        const Material1DState& oldState) const;
};
