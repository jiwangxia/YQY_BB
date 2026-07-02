#include "DataStructure/Material/Material1D.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
constexpr double kTolerance = 1e-9;

void CheckNear(double actual, double expected, const std::string& message)
{
    const double scale = std::max(1.0, std::abs(expected));
    if (std::abs(actual - expected) > kTolerance * scale)
    {
        throw std::runtime_error(
            message + ": expected " + std::to_string(expected)
            + ", actual " + std::to_string(actual));
    }
}

void Check(bool condition, const std::string& message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void TestElasticResponseKeepsInitialStress()
{
    Material1DState oldState;
    oldState.stress = 50.0;

    const auto result = CalculateMaterial1D(
        MaterialModel::Elastic,
        200000.0,
        0.0,
        0.0,
        0.001,
        oldState);

    CheckNear(result.stress, 250.0, "elastic stress with initial stress");
    CheckNear(result.stiffness, 200000.0, "elastic stiffness");
    CheckNear(result.state.strain, 0.001, "elastic state strain");
    CheckNear(result.state.stress, 250.0, "elastic state stress");
}

void TestPerfectPlasticityReturnsToYieldSurface()
{
    const auto result = CalculateMaterial1D(
        MaterialModel::IdealPlastic1D,
        200000.0,
        200.0,
        0.0,
        0.002,
        {});

    CheckNear(result.stress, 200.0, "perfect-plastic stress");
    CheckNear(result.stiffness, 0.0, "perfect-plastic stiffness");
    CheckNear(result.state.plasticStrain, 0.001, "perfect-plastic strain");
    CheckNear(result.state.accumulatedPlastic, 0.001, "accumulated plastic strain");
}

void TestLinearHardeningUsesConsistentTangent()
{
    constexpr double young = 200000.0;
    constexpr double hardening = 10000.0;

    const auto result = CalculateMaterial1D(
        MaterialModel::HardeningPlastic1D,
        young,
        200.0,
        hardening,
        0.002,
        {});

    const double deltaGamma = 200.0 / (young + hardening);
    CheckNear(result.stress, 400.0 - young * deltaGamma, "hardening stress");
    CheckNear(
        result.stiffness,
        young * hardening / (young + hardening),
        "consistent elastoplastic stiffness");
    CheckNear(result.state.plasticStrain, deltaGamma, "hardening plastic strain");
    CheckNear(result.state.accumulatedPlastic, deltaGamma, "hardening accumulation");
}

void TestUnloadingUsesElasticSlope()
{
    const auto loaded = CalculateMaterial1D(
        MaterialModel::HardeningPlastic1D,
        200000.0,
        200.0,
        10000.0,
        0.002,
        {});

    const auto unloaded = CalculateMaterial1D(
        MaterialModel::HardeningPlastic1D,
        200000.0,
        200.0,
        10000.0,
        0.0015,
        loaded.state);

    CheckNear(
        unloaded.stress,
        loaded.stress - 200000.0 * 0.0005,
        "unloading stress");
    CheckNear(unloaded.stiffness, 200000.0, "unloading stiffness");
    CheckNear(
        unloaded.state.plasticStrain,
        loaded.state.plasticStrain,
        "unloading keeps plastic strain");
}

void TestReverseLoadingUsesAbsoluteYieldFunction()
{
    const auto loaded = CalculateMaterial1D(
        MaterialModel::HardeningPlastic1D,
        200000.0,
        200.0,
        10000.0,
        0.002,
        {});

    const auto reversed = CalculateMaterial1D(
        MaterialModel::HardeningPlastic1D,
        200000.0,
        200.0,
        10000.0,
        -0.001,
        loaded.state);

    Check(reversed.stress < 0.0, "reverse loading must produce compression");
    CheckNear(
        std::abs(reversed.stress),
        200.0 + 10000.0 * reversed.state.accumulatedPlastic,
        "reverse stress lies on expanded yield surface");
}

void TestRepeatedTrialDoesNotAccumulateTwice()
{
    const Material1DState oldState;
    const auto first = CalculateMaterial1D(
        MaterialModel::IdealPlastic1D,
        200000.0,
        200.0,
        0.0,
        0.002,
        oldState);
    const auto repeated = CalculateMaterial1D(
        MaterialModel::IdealPlastic1D,
        200000.0,
        200.0,
        0.0,
        0.002,
        oldState);

    CheckNear(
        repeated.state.plasticStrain,
        first.state.plasticStrain,
        "repeated trial plastic strain");
    CheckNear(
        repeated.state.accumulatedPlastic,
        first.state.accumulatedPlastic,
        "repeated trial accumulation");
}

void TestInvalidInitialStressIsRejected()
{
    Material1DState oldState;
    oldState.stress = 250.0;

    bool rejected = false;
    try
    {
        (void)CalculateMaterial1D(
            MaterialModel::IdealPlastic1D,
            200000.0,
            200.0,
            0.0,
            0.0,
            oldState);
    }
    catch (const std::runtime_error&)
    {
        rejected = true;
    }

    Check(rejected, "initial stress outside yield surface must be rejected");
}

void TestPlasticTypeMustMatchHardeningModulus()
{
    bool idealRejected = false;
    try
    {
        (void)CalculateMaterial1D(
            MaterialModel::IdealPlastic1D,
            200000.0,
            200.0,
            1000.0,
            0.0,
            {});
    }
    catch (const std::runtime_error&)
    {
        idealRejected = true;
    }

    bool hardeningRejected = false;
    try
    {
        (void)CalculateMaterial1D(
            MaterialModel::HardeningPlastic1D,
            200000.0,
            200.0,
            0.0,
            0.0,
            {});
    }
    catch (const std::runtime_error&)
    {
        hardeningRejected = true;
    }

    Check(idealRejected, "ideal plasticity requires H = 0");
    Check(hardeningRejected, "hardening plasticity requires H > 0");
}
}

int main()
{
    try
    {
        TestElasticResponseKeepsInitialStress();
        TestPerfectPlasticityReturnsToYieldSurface();
        TestLinearHardeningUsesConsistentTangent();
        TestUnloadingUsesElasticSlope();
        TestReverseLoadingUsesAbsoluteYieldFunction();
        TestRepeatedTrialDoesNotAccumulateTwice();
        TestInvalidInitialStressIsRejected();
        TestPlasticTypeMustMatchHardeningModulus();
        std::cout << "Material1D tests passed\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Material1D test failed: " << error.what() << '\n';
        return 1;
    }
}
