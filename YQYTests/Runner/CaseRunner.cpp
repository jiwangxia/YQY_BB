#include "Runner/CaseRunner.h"
#include "Dynamics/ConstrainedPendulum/ConstrainedPendulumTest.h"
#include "Dynamics/Oscillators/OscillatorCases.h"
#include "Dynamics/PlanarBeam/SmallAmplitudeTest.h"
#include "Dynamics/PlanarBeam/FiniteAmplitudeTest.h"
#include "Dynamics/SpatialBeam/SpatialRigidMotionTest.h"
#include "Dynamics/SpatialBeam/SpatialFrameTest.h"
#include "Dynamics/TrussVibration/TrussVibrationTest.h"
#include "Dynamics/CableVibration/CableVibrationTest.h"
#include "Dynamics/SingleConductor/SingleConductorTest.h"
#include "Statics/HelicalBeam/HelicalBeamTest.h"
#include "Support/TestPaths.h"

#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QTextStream>
#include <algorithm>
#include <cmath>

static std::optional<int> RunSpatialFrameCase(const QStringList& arguments)
{
    const bool beam = arguments.contains(QStringLiteral("--verify-duan2026-beam"));
    const bool ring = arguments.contains(QStringLiteral("--verify-duan2026-ring"));
    if (!beam && !ring)
        return std::nullopt;
    if (beam && ring)
        return 1;
    const int solverIndex = arguments.indexOf(QStringLiteral("--paper-solver"));
    if (solverIndex < 0 || solverIndex + 1 >= arguments.size() ||
        arguments.count(QStringLiteral("--paper-solver")) != 1)
        return 1;
    const QString solver = arguments[solverIndex + 1].trimmed().toLower();
    if (solver != QStringLiteral("adaptive-newmark") && solver != QStringLiteral("adaptive-tssbn") &&
        solver != QStringLiteral("newmark") && solver != QStringLiteral("reference-tssbn") &&
        solver != QStringLiteral("rk4") && solver != QStringLiteral("adaptive-rk4"))
        return 1;
    const auto numericOption = [&](const QString& option, double fallback)
    {
        const int index = arguments.indexOf(option);
        if (index < 0)
            return fallback;
        if (index + 1 >= arguments.size() || arguments.count(option) != 1)
            return -1.0;
        bool ok = false;
        const double value = arguments[index + 1].toDouble(&ok);
        return ok && std::isfinite(value) && value > 0.0 ? value : -1.0;
    };
    const double duration = numericOption(QStringLiteral("--paper-duration"), ring ? 100.0 : 30.0);
    const double maximumTimeStep = numericOption(QStringLiteral("--paper-max-dt"), 0.5);
    const double tolerance = numericOption(QStringLiteral("--paper-tolerance"), 1.0e-3);
    const double newmarkAbsoluteTolerance = numericOption(QStringLiteral("--paper-newmark-atol"), 1.0e-6);
    const double newmarkVelocityAbsoluteTolerance =
        numericOption(QStringLiteral("--paper-newmark-velocity-atol"), 1.0e-6);
    const double beamElementsValue = numericOption(QStringLiteral("--paper-beam-elements-per-leg"), 10.0);
    const double fullStateStrideValue = numericOption(QStringLiteral("--paper-state-stride"), 1.0);
    PaperTssbnRunOptions runOptions;
    runOptions.useRungeKutta = solver == QStringLiteral("rk4") || solver == QStringLiteral("adaptive-rk4");
    runOptions.initialTimeStep = numericOption(QStringLiteral("--paper-initial-dt"), std::min(0.01, maximumTimeStep));
    runOptions.minimumTimeStep = numericOption(QStringLiteral("--paper-min-dt"), 1.0e-6);
    runOptions.nonlinearTolerance = numericOption(QStringLiteral("--paper-newton-tol"), 1.0e-6);
    runOptions.diagnostics = arguments.contains(QStringLiteral("--paper-diagnostics"));
    runOptions.profile = arguments.contains(QStringLiteral("--paper-profile"));
    runOptions.finalStateOnly = arguments.contains(QStringLiteral("--paper-final-state-only"));
    runOptions.fusedAssembly = arguments.contains(QStringLiteral("--paper-fused-assembly"));
    const QString energyEveryStepOption = QStringLiteral("--paper-energy-every-step");
    if (arguments.count(energyEveryStepOption) > 1)
    {
        QTextStream(stderr) << "Duan2026 every-step energy flag must occur at most once" << Qt::endl;
        return 1;
    }
    runOptions.energyEveryStep = arguments.contains(energyEveryStepOption);
    for (const QString& argument : arguments)
    {
        if (argument.startsWith(QStringLiteral("--paper-tssbn-acceleration-")) ||
            argument.startsWith(QStringLiteral("--paper-tssbn-angular-acceleration-")))
        {
            QTextStream(stderr) << "加速度误差控制已移除，请删除对应参数。" << Qt::endl;
            return 1;
        }
    }
    const int spectralRadiusIndex = arguments.indexOf(QStringLiteral("--paper-spectral-radius"));
    const int errorNormIndex = arguments.indexOf(QStringLiteral("--paper-error-norm"));
    runOptions.overrideAlgorithmControls = spectralRadiusIndex >= 0 || errorNormIndex >= 0;
    if (runOptions.overrideAlgorithmControls && solver != QStringLiteral("adaptive-tssbn"))
        return 1;
    if (spectralRadiusIndex >= 0)
    {
        if (spectralRadiusIndex + 1 >= arguments.size() ||
            arguments.count(QStringLiteral("--paper-spectral-radius")) != 1)
            return 1;
        bool valid = false;
        runOptions.spectralRadiusInfinity = arguments[spectralRadiusIndex + 1].toDouble(&valid);
        if (!valid || !std::isfinite(runOptions.spectralRadiusInfinity) || runOptions.spectralRadiusInfinity < 0.0 ||
            runOptions.spectralRadiusInfinity > 1.0)
            return 1;
    }
    if (errorNormIndex >= 0)
    {
        if (errorNormIndex + 1 >= arguments.size() || arguments.count(QStringLiteral("--paper-error-norm")) != 1)
            return 1;
        const QString errorNorm = arguments[errorNormIndex + 1].trimmed().toLower();
        if (errorNorm == QStringLiteral("max"))
            runOptions.errorNormMode = SolverNameSpace::TssbnErrorNormMode::ComponentwiseMax;
        else if (errorNorm == QStringLiteral("paper-rms"))
            runOptions.errorNormMode = SolverNameSpace::TssbnErrorNormMode::PaperMixedRms;
        else if (errorNorm == QStringLiteral("global"))
            runOptions.errorNormMode = SolverNameSpace::TssbnErrorNormMode::LegacyGlobal;
        else if (errorNorm == QStringLiteral("global-displacement"))
            runOptions.errorNormMode = SolverNameSpace::TssbnErrorNormMode::DisplacementGlobalVelocityMax;
        else if (errorNorm == QStringLiteral("global-max"))
            runOptions.errorNormMode = SolverNameSpace::TssbnErrorNormMode::GlobalMax;
        else
            return 1;
    }
    if (duration <= 0.0 || maximumTimeStep <= 0.0 || tolerance <= 0.0 || newmarkAbsoluteTolerance <= 0.0 ||
        newmarkVelocityAbsoluteTolerance <= 0.0 || beamElementsValue < 1.0 || beamElementsValue > 40.0 ||
        std::floor(beamElementsValue) != beamElementsValue || fullStateStrideValue < 1.0 ||
        fullStateStrideValue > 1000000.0 || std::floor(fullStateStrideValue) != fullStateStrideValue ||
        runOptions.initialTimeStep <= 0.0 || runOptions.minimumTimeStep <= 0.0 || runOptions.nonlinearTolerance <= 0.0)
        return 1;
    if (runOptions.minimumTimeStep > runOptions.initialTimeStep || runOptions.initialTimeStep > maximumTimeStep)
        return 1;
    const int beamElementsPerLeg = static_cast<int>(beamElementsValue);
    const int fullStateStride = static_cast<int>(fullStateStrideValue);
    const QString caseName = ring ? QStringLiteral("ring") : QStringLiteral("beam");
    const int outputIndex = arguments.indexOf(QStringLiteral("--paper-output"));
    if (outputIndex >= 0 &&
        (outputIndex + 1 >= arguments.size() || arguments[outputIndex + 1].startsWith(QStringLiteral("--")) ||
         arguments[outputIndex + 1].trimmed().isEmpty() || arguments.count(QStringLiteral("--paper-output")) != 1))
        return 1;
    const QString output =
        outputIndex >= 0 ? arguments[outputIndex + 1]
                         : TestPaths::VerificationOutputDirectory(
                               QStringLiteral("duan2026_%1_%2_e%3").arg(caseName, solver).arg(beamElementsPerLeg));
    return PaperBeamDynamicsVerification::RunDuan2026(
        ring, solver == QStringLiteral("adaptive-tssbn"), output, duration, maximumTimeStep, tolerance,
        beamElementsPerLeg, solver == QStringLiteral("newmark") || solver == QStringLiteral("rk4"),
        newmarkAbsoluteTolerance, newmarkVelocityAbsoluteTolerance, solver == QStringLiteral("reference-tssbn"),
        fullStateStride, &runOptions);
}

std::optional<int> RunNumericalCase(const QStringList& arguments)
{
    if (arguments.contains(QStringLiteral("--list-cases")))
    {
        QFile file(QDir(TestPaths::RepositoryRoot()).filePath(QStringLiteral("YQYTests/cases.json")));
        if (!file.open(QIODevice::ReadOnly))
        {
            QTextStream(stderr) << "无法读取算例登记清单。" << Qt::endl;
            return 2;
        }
        const QByteArray bytes = file.readAll();
        if (QJsonDocument::fromJson(bytes).isNull())
            return 2;
        QTextStream(stdout) << QString::fromUtf8(bytes) << Qt::endl;
        return 0;
    }
    if (const auto result = VerifySingleConductorNewmark(arguments))
        return result;
    if (const auto result = VerifyBetschConstraintDynamics(arguments))
        return result;
    if (const auto result = VerifyRungeKuttaSpatial(arguments))
        return result;
    if (const auto result = VerifyTssbnSpatial(arguments))
        return result;
    if (const auto result = VerifySpatialBeamDynamics(arguments))
        return result;
    if (const auto result = VerifyNewmarkTruss(arguments))
        return result;
    if (const auto result = VerifyNewmarkCable(arguments))
        return result;
    if (const auto result = VerifyNewmarkBeamBending(arguments))
        return result;
    if (const auto result = VerifyNewmarkBeamNonlinear(arguments))
        return result;
    if (const auto result = VerifyNewmarkLiterature(arguments))
        return result;
    if (const auto result = VerificationCases::verifyBattiniHelicalBeam(arguments))
        return result;
    return RunSpatialFrameCase(arguments);
}
