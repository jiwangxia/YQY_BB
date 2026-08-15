#include "../Solver/Dynamic/RayleighDamping.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
void CheckNear(double actual, double expected, double tolerance, const char* message)
{
    if (std::abs(actual - expected) > tolerance)
        throw std::runtime_error(message);
}
}

int main()
{
    const auto equalRatio = SolverNameSpace::SolveRayleighDamping(0.5, 5.0, 0.02);
    CheckNear(equalRatio.DampingRatioAtFrequency(0.5), 0.02, 1.0e-12, "damping ratio at f1 does not match");
    CheckNear(equalRatio.DampingRatioAtFrequency(5.0), 0.02, 1.0e-12, "damping ratio at f2 does not match");

    const auto unequalRatio = SolverNameSpace::SolveRayleighDamping(0.5, 0.02, 5.0, 0.03);
    CheckNear(unequalRatio.DampingRatioAtFrequency(0.5), 0.02, 1.0e-12, "unequal damping ratio at f1 does not match");
    CheckNear(unequalRatio.DampingRatioAtFrequency(5.0), 0.03, 1.0e-12, "unequal damping ratio at f2 does not match");

    bool rejectedEqualFrequencies = false;
    try
    {
        SolverNameSpace::SolveRayleighDamping(1.0, 1.0, 0.02);
    }
    catch (const std::invalid_argument&)
    {
        rejectedEqualFrequencies = true;
    }
    if (!rejectedEqualFrequencies)
        throw std::runtime_error("equal reference frequencies must be rejected");

    std::cout << equalRatio.ToText() << '\n';
    return 0;
}
