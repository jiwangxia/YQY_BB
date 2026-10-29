#pragma once

#include <cmath>

// 当前输出帧的结构能量，单位 J；掩码未置位的分量不得作为零值使用。
struct EnergyResult
{
    static constexpr unsigned int Kinetic = 1;
    static constexpr unsigned int Strain = 2;
    static constexpr unsigned int Gravity = 4;
    static constexpr unsigned int Mechanical = 8;
    static constexpr unsigned int All = Kinetic | Strain | Gravity | Mechanical;

    double kinetic = 0.0;
    double strain = 0.0;
    double gravity = 0.0;
    double mechanical = 0.0;
    unsigned int availableMask = 0;

    void FinalizeTotals()
    {
        availableMask &= Kinetic | Strain | Gravity;
        if (!std::isfinite(kinetic))
            availableMask &= ~Kinetic;
        if (!std::isfinite(strain))
            availableMask &= ~Strain;
        if (!std::isfinite(gravity))
            availableMask &= ~Gravity;
        if (!(availableMask & Kinetic))
            kinetic = 0.0;
        if (!(availableMask & Strain))
            strain = 0.0;
        if (!(availableMask & Gravity))
            gravity = 0.0;
        mechanical = 0.0;
        if (availableMask == (Kinetic | Strain | Gravity))
        {
            mechanical = kinetic + strain + gravity;
            if (std::isfinite(mechanical))
                availableMask |= Mechanical;
            else
                mechanical = 0.0;
        }
    }

    // 累加器需先以已知有效的零能量或首个区域初始化。
    void Accumulate(const EnergyResult& other)
    {
        availableMask &= other.availableMask;
        kinetic += other.kinetic;
        strain += other.strain;
        gravity += other.gravity;
        FinalizeTotals();
    }
};
