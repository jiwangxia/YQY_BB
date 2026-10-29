#pragma once
#include "Base/Base.h"
#include <array>

/**
 * @brief 截面基类 - 所有截面类型的公共基类
 */
class SectionBase : public Base
{
public:
    SectionBase();

    double m_Area = 0.0;   ///< 截面面积
    double m_Radius = 0.0; ///< 半径
    double Io = 0.0;
    double Irr = 0.0;
    double Sy = 0.0;
    double Sz = 0.0;

    bool m_HasExplicitSectionInertia = false;
    double m_ExplicitIy = 0.0;
    double m_ExplicitIz = 0.0;
    double m_ExplicitJ = 0.0;

    Eigen::Vector3d m_MassInertiaPerLength = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_MassFirstMomentPerLength = Eigen::Vector3d::Zero(); // 零扭转时相对于导线轴的质量一次矩 (kg)
    bool m_DiagnosticReferenceFixedInertialCoupling = false; // 旧参考索使用固定全局偏心质量矩阵

    /**
     * @brief 计算截面面积（纯虚函数）
     */
    //virtual void Calculate_Area() = 0;

    /**
     * @brief 获取截面面积
     * @return 截面面积值
     */
    double Get_AreaValue() const
    {
        return m_Area;
    };
    // Iy=∫z²dA、Iz=∫y²dA；J 为 Saint-Venant 扭转常数，与极惯量 Iy+Iz 区分。
    virtual void Calculate_I(_OUT double& Iy, _OUT double& Iz, _OUT double& J) = 0;
    /** @brief 结果采样线程查询截面惯性；按输入参数共享缓存，不改写截面几何和物理字段。 */
    bool TryGetEnergyInertia(_OUT double& Iy, _OUT double& Iz, _OUT double& J) const;

private:
    mutable std::array<double, 6> m_energyInertiaParameters{};
    mutable std::array<double, 3> m_energyInertia{};
    mutable bool m_energyInertiaValid = false;
};
