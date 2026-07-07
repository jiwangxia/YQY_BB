#include "Material1D.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

Material1DResult CalculateMaterial1D(
    MaterialModel model,
    double young,       //弹性模量
    double yieldStress, //初始屈服应力
    double hardening,   //硬化模量
    double strain,      //当前总应变
    const Material1DState& oldState)
{
    const bool valuesAreFinite =
        std::isfinite(young)
        && std::isfinite(yieldStress)
        && std::isfinite(hardening)
        && std::isfinite(strain)
        && std::isfinite(oldState.strain)
        && std::isfinite(oldState.stress)
        && std::isfinite(oldState.plasticStrain)
        && std::isfinite(oldState.accumulatedPlastic);
    if (!valuesAreFinite)
    {
        throw std::runtime_error("Material1D contains a non-finite value");
    }
    if (young <= 0.0)
    {
        throw std::runtime_error("Material1D Young's modulus must be positive");
    }

    Material1DResult result;
    result.state = oldState;

    const double strainIncrement = strain - oldState.strain;
    const double trialStress = oldState.stress + young * strainIncrement;

    if (model == MaterialModel::Elastic)
    {
        result.stress = trialStress;
        result.stiffness = young;
        result.state.strain = strain;
        result.state.stress = result.stress;
        return result;
    }

    if (yieldStress <= 0.0)
    {
        throw std::runtime_error("Material1D yield stress must be positive");
    }
    if (hardening < 0.0)
    {
        throw std::runtime_error("Material1D hardening modulus cannot be negative");
    }
    if (model == MaterialModel::IdealPlastic1D && hardening != 0.0)
    {
        throw std::runtime_error(
            "Material1D ideal plasticity requires zero hardening modulus");
    }
    if (model == MaterialModel::HardeningPlastic1D && hardening <= 0.0)
    {
        throw std::runtime_error(
            "Material1D hardening plasticity requires positive hardening modulus");
    }

    // 初始状态没有塑性历史时，初始应力必须位于初始屈服面内。
    const bool isInitialState =
        oldState.strain == 0.0
        && oldState.plasticStrain == 0.0
        && oldState.accumulatedPlastic == 0.0;
    const double yieldTolerance =
        1e-10 * std::max(1.0, std::abs(yieldStress));
    if (isInitialState
        && std::abs(oldState.stress) > yieldStress + yieldTolerance)
    {
        throw std::runtime_error(
            "Material1D initial stress exceeds the initial yield stress");
    }

    const double currentYieldStress =
        yieldStress + hardening * oldState.accumulatedPlastic;
    const double yieldFunction =
        std::abs(trialStress) - currentYieldStress;

    if (yieldFunction <= yieldTolerance)
    {
        result.stress = trialStress;
        result.stiffness = young;
    }
    else
    {
        const double direction = std::copysign(1.0, trialStress);
        const double plasticIncrement =
            yieldFunction / (young + hardening);

        result.stress =
            trialStress - young * plasticIncrement * direction;
        result.stiffness =
            young * hardening / (young + hardening);
        result.state.plasticStrain += plasticIncrement * direction;
        result.state.accumulatedPlastic += plasticIncrement;
    }

    result.state.strain = strain;
    result.state.stress = result.stress;
    return result;
}
