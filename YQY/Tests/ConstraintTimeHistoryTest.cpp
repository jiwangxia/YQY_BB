#include "DataStructure/Constraint/Constraint.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void CheckNear(double actual, double expected, const std::string& message)
{
    if (std::abs(actual - expected) > 1e-12)
    {
        throw std::runtime_error(message);
    }
}

void TestOldConstraintUsesIncrementFactor()
{
    Constraint constraint;
    constraint.m_Value = 5.0;

    CheckNear(
        constraint.GetValue(100.0, 0.4),
        2.0,
        "old constraint must use value times factor");
}

void TestTabularConstraintInterpolatesAndClamps()
{
    Constraint constraint;
    constraint.m_Value = 5.0;

    if (constraint.HasTimePoints())
    {
        throw std::runtime_error(
            "new constraint must not have a time history");
    }

    constraint.SetTimePoints({
        { 0.0, 0.0 },
        { 1.0, 1.0 },
        { 2.0, 0.5 }
    });

    if (!constraint.HasTimePoints())
    {
        throw std::runtime_error(
            "SetTimePoints must attach the time history");
    }

    CheckNear(constraint.GetValue(-1.0, 0.0), 0.0, "left clamp");
    CheckNear(constraint.GetValue(0.5, 0.0), 2.5, "first segment");
    CheckNear(constraint.GetValue(1.5, 0.0), 3.75, "second segment");
    CheckNear(constraint.GetValue(3.0, 0.0), 2.5, "right clamp");
}

void TestTimeMustIncreaseStrictly()
{
    Constraint constraint;
    bool rejected = false;

    try
    {
        constraint.SetTimePoints({
            { 0.0, 0.0 },
            { 0.0, 1.0 }
        });
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }

    if (!rejected)
    {
        throw std::runtime_error("duplicate time must be rejected");
    }
}
}

int main()
{
    try
    {
        TestOldConstraintUsesIncrementFactor();
        TestTabularConstraintInterpolatesAndClamps();
        TestTimeMustIncreaseStrictly();
        std::cout << "Constraint time-history tests passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Constraint time-history test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
