#pragma once

#include "AeroManager.h"

#include <vector>

// For the force convention used by this program, H = Cd + dCl/d(alpha).
// H < 0 is negative aerodynamic damping in the cross-wind translation mode.
// This is a fast aerodynamic pre-screen, not a substitute for a coupled
// structural stability calculation.
struct GallopingInstabilityInterval
{
    int profileId = 0;
    double startAngleDegrees = 0.0;
    double endAngleDegrees = 0.0;
    double minimumDenHartog = 0.0;
};

class GallopingStabilityAnalyzer final
{
public:
    // Uses the same piecewise-linear coefficients as AeroManager. Boundaries
    // are analytical zero crossings of H, without angular sampling error.
    static std::vector<GallopingInstabilityInterval> FindNegativeDampingIntervals(
        const std::vector<BladeModel>& models, double angleStepDegrees = 5.0);
};
