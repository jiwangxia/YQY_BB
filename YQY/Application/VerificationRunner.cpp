#include "Application/VerificationRunner.h"
#include "Application/PaperBeamDynamicsVerification.h"

#include "Conductor/ConductorModelBuilder.h"
#include "Conductor/PropertyLibrary.h"
#include "DataStructure/Aerodynamics/BundleAeroMapper.h"
#include "DataStructure/Aerodynamics/GallopingStabilityAnalyzer.h"
#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Load/Force_Wind.h"
#include "DataStructure/Load/LoadAssembler.h"
#include "DataStructure/Load/AerodynamicLoadCalculator.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include "GUI/Controllers/ModelController.h"
#include "GUI/Controllers/SolveTaskController.h"
#include "GUI/Widgets/ConductorModule.h"
#include "Import/Input_Model.h"
#include "Solver/AnalysisSolve.h"
#include "Solver/Dynamic/SolverAdaptiveTSSBN.h"
#include "Solver/SolverFactory.h"

#include <QComboBox>
#include <QTemporaryDir>
#include <cmath>

namespace
{
std::optional<int> verifyGallopingStability(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-galloping-stability"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    StructureData structure;
    Hdf5ModelIO hdf5;
    const QString modelPath = arguments.at(index + 1);
    if (!hdf5.ImportHdf5(modelPath, &structure))
    {
        QTextStream(stderr) << "cannot import galloping stability model="
                            << modelPath << Qt::endl;
        return 2;
    }

    const auto& cases = structure.m_AeroManager.getCaseModels();
    if (cases.empty())
    {
        QTextStream(stderr) << "model has no loaded aerodynamic cases" << Qt::endl;
        return 3;
    }

    int intervalCount = 0;
    QTextStream stream(stdout);
    stream << "galloping stability model=" << modelPath << Qt::endl;
    for (const auto& [key, models] : cases)
    {
        const auto intervals =
            GallopingStabilityAnalyzer::FindNegativeDampingIntervals(models);
        stream << "case bundle=" << key.bundleCount
               << " wind=" << key.windSpeed
               << " ice=" << key.iceThickness
               << " profiles=" << models.size()
               << " negative_intervals=" << intervals.size() << Qt::endl;
        for (const GallopingInstabilityInterval& interval : intervals)
        {
            stream << "  profile=" << interval.profileId
                   << " alpha_start=" << interval.startAngleDegrees
                   << " alpha_end=" << interval.endAngleDegrees
                   << " min_H=" << interval.minimumDenHartog << Qt::endl;
        }
        intervalCount += static_cast<int>(intervals.size());
    }
    return intervalCount > 0 ? 0 : 4;
}

class LinearOscillatorModel final : public SolverNameSpace::IAnalysisModel
{
public:
    LinearOscillatorModel(double mass, double stiffness)
        : mass_(mass)
        , stiffness_(stiffness)
    {
        displacement_[0] = 1.0;
        velocity_[0] = 0.0;
        acceleration_[0] = -stiffness_ / mass_;
        BackupStepState();
    }

    int GetFreeDofs() const override { return 1; }
    int GetFixedDofs() const override { return 0; }

    void ApplyIncrement(const SolverNameSpace::Vec& increment) override
    {
        displacement_ += increment;
    }

    void BeginDynamicStep(double, double, double) override {}

    void ApplyDynamicCorrection(
        const SolverNameSpace::Vec& increment, double a0, double a1) override
    {
        displacement_ += increment;
        velocity_ += a1 * increment;
        acceleration_ += a0 * increment;
    }

    void RollbackDynamicStep() override
    {
        displacement_ = savedDisplacement_;
        velocity_ = savedVelocity_;
        acceleration_ = savedAcceleration_;
    }

    void SetTrialKinematics(
        const SolverNameSpace::Vec& velocity,
        const SolverNameSpace::Vec& acceleration) override
    {
        velocity_ = velocity;
        acceleration_ = acceleration;
    }

    void GetState(
        SolverNameSpace::Vec& displacement,
        SolverNameSpace::Vec& velocity,
        SolverNameSpace::Vec& acceleration) const override
    {
        displacement = displacement_;
        velocity = velocity_;
        acceleration = acceleration_;
    }

    void Assemble_Matrix(SolverNameSpace::SpMat& stiffness, bool) override
    {
        stiffness.resize(1, 1);
        stiffness.setZero();
        stiffness.insert(0, 0) = stiffness_;
        stiffness.makeCompressed();
    }

    void AssembleDynamicSystem(
        SolverNameSpace::SpMat& mass,
        SolverNameSpace::SpMat& gyroscopic,
        SolverNameSpace::SpMat& centrifugal) override
    {
        mass.resize(1, 1);
        mass.setZero();
        mass.insert(0, 0) = mass_;
        mass.makeCompressed();
        gyroscopic.resize(1, 1);
        gyroscopic.setZero();
        centrifugal.resize(1, 1);
        centrifugal.setZero();
    }

    void ComputeExternalForce(
        double,
        double,
        SolverNameSpace::Vec& constrained,
        SolverNameSpace::Vec& free) override
    {
        constrained.resize(0);
        free = SolverNameSpace::Vec::Zero(1);
    }

    void Assemble_Constraint(
        SolverNameSpace::Vec& constrained,
        double,
        double) override
    {
        constrained.resize(0);
    }

    void ComputeResidual(
        const SolverNameSpace::Vec& external,
        SolverNameSpace::Vec& residual) override
    {
        residual = external
            - stiffness_ * displacement_
            - mass_ * acceleration_;
    }

    void CalculateReactions(SolverNameSpace::Vec& reactions) override
    {
        reactions.resize(0);
    }

    void OnStepCompleted(double) override { ++completedSteps_; }
    void CommitState() override {}

    void BackupStepState() override
    {
        savedDisplacement_ = displacement_;
        savedVelocity_ = velocity_;
        savedAcceleration_ = acceleration_;
    }

    void GetStepIncrement(SolverNameSpace::Vec& increment) const override
    {
        increment = displacement_ - savedDisplacement_;
    }

    int CompletedSteps() const { return completedSteps_; }

private:
    double mass_ = 1.0;
    double stiffness_ = 1.0;
    SolverNameSpace::Vec displacement_ =
        SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec velocity_ =
        SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec acceleration_ =
        SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec savedDisplacement_ =
        SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec savedVelocity_ =
        SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec savedAcceleration_ =
        SolverNameSpace::Vec::Zero(1);
    int completedSteps_ = 0;
};

std::optional<int> verifyAdaptiveTssbn(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-adaptive-tssbn")))
        return std::nullopt;

    AnalysisStep factoryStep;
    factoryStep.m_Type = EnumKeyword::StepType::DYNAMIC;
    factoryStep.m_DynamicSolverType =
        SolverNameSpace::SolverType::AdaptiveTSSBN;
    factoryStep.m_StepSize = 0.1;
    factoryStep.m_Tolerance = 1.0e-5;
    factoryStep.m_MaxIterations = 20;
    factoryStep.m_AdaptiveTssbn.spectralRadiusInfinity = 0.77;
    factoryStep.m_AdaptiveTssbn.minimumTimeStep = 2.0e-6;
    factoryStep.m_AdaptiveTssbn.maximumTimeStep = 0.42;
    factoryStep.m_AdaptiveTssbn.relativeTolerance = 8.0e-4;
    factoryStep.m_AdaptiveTssbn.absoluteTolerance = 4.0e-7;
    factoryStep.m_AdaptiveTssbn.shrinkFactor = 0.74;
    factoryStep.m_AdaptiveTssbn.targetNewtonIterations = 9;
    factoryStep.m_AdaptiveTssbn.derivativeGain = 0.12;
    factoryStep.m_AdaptiveTssbn.minimumDerivativeFactor = 0.43;
    factoryStep.m_AdaptiveTssbn.maximumDerivativeFactor = 1.61;
    factoryStep.m_AdaptiveTssbn.maximumRejectedAttempts = 17;
    const auto factorySolver =
        SolverNameSpace::SolverFactory::Create_StepForSlover(factoryStep);
    const auto* adaptiveFactorySolver =
        dynamic_cast<const SolverNameSpace::SolverAdaptiveTSSBN*>(
            factorySolver.get());
    const bool factoryConnected = adaptiveFactorySolver
        && adaptiveFactorySolver->GetType()
            == SolverNameSpace::SolverType::AdaptiveTSSBN
        && std::abs(
            adaptiveFactorySolver->GetParams().spectralRadiusInfinity
                - 0.77) < 1.0e-12
        && std::abs(
            adaptiveFactorySolver->GetParams().minimumTimeStep
                - 2.0e-6) < 1.0e-15
        && std::abs(
            adaptiveFactorySolver->GetParams().maximumTimeStep
                - 0.42) < 1.0e-12
        && std::abs(
            adaptiveFactorySolver->GetParams().relativeTolerance
                - 8.0e-4) < 1.0e-15
        && std::abs(
            adaptiveFactorySolver->GetParams().absoluteTolerance
                - 4.0e-7) < 1.0e-15
        && std::abs(
            adaptiveFactorySolver->GetParams().shrinkFactor
                - 0.74) < 1.0e-12
        && adaptiveFactorySolver->GetParams().targetNewtonIterations
            == 9
        && std::abs(
            adaptiveFactorySolver->GetParams().derivativeGain
                - 0.12) < 1.0e-12
        && std::abs(
            adaptiveFactorySolver->GetParams().minimumDerivativeFactor
                - 0.43) < 1.0e-12
        && std::abs(
            adaptiveFactorySolver->GetParams().maximumDerivativeFactor
                - 1.61) < 1.0e-12
        && adaptiveFactorySolver->GetParams().maximumRejectedAttempts
            == 17;

    constexpr double mass = 2.0;
    constexpr double stiffness = 8.0;
    constexpr double angularFrequency = 2.0;
    const double duration = 2.0 * std::acos(-1.0);
    LinearOscillatorModel model(mass, stiffness);

    SolverNameSpace::SolverAdaptiveTSSBN::Params parameters;
    parameters.initialTimeStep = 0.15;
    parameters.minimumTimeStep = 1.0e-5;
    parameters.maximumTimeStep = 0.3;
    parameters.relativeTolerance = 2.0e-6;
    parameters.absoluteTolerance = 1.0e-9;
    parameters.nonlinearTolerance = 1.0e-9;
    parameters.maximumNewtonIterations = 8;
    SolverNameSpace::SolverAdaptiveTSSBN solver(parameters);
    const bool solved = solver.Solve(model, duration);

    SolverNameSpace::Vec displacement;
    SolverNameSpace::Vec velocity;
    SolverNameSpace::Vec acceleration;
    model.GetState(displacement, velocity, acceleration);
    const double exactDisplacement =
        std::cos(angularFrequency * duration);
    const double exactVelocity =
        -angularFrequency * std::sin(angularFrequency * duration);
    const double exactAcceleration =
        -angularFrequency * angularFrequency * exactDisplacement;
    const double displacementError =
        std::abs(displacement[0] - exactDisplacement);
    const double velocityError =
        std::abs(velocity[0] - exactVelocity);
    const double accelerationError =
        std::abs(acceleration[0] - exactAcceleration);
    const bool passed =
        factoryConnected
        && solved
        && solver.GetAcceptedStepCount() == model.CompletedSteps()
        && solver.GetAcceptedStepCount() > 0
        && displacementError < 2.0e-3
        && velocityError < 2.0e-3
        && accelerationError < 2.0e-3;

    QTextStream(stdout)
        << "adaptive_tssbn factory=" << factoryConnected
        << " solved=" << solved
        << " accepted=" << solver.GetAcceptedStepCount()
        << " rejected=" << solver.GetRejectedStepCount()
        << " displacement_error=" << displacementError
        << " velocity_error=" << velocityError
        << " acceleration_error=" << accelerationError
        << Qt::endl;
    return passed ? 0 : 1;
}

std::optional<int> verifyLe2012Example1(const QStringList& arguments)
{
    if (arguments.contains(
            QStringLiteral("--verify-le2012-example1-tssbn")))
    {
        return PaperBeamDynamicsVerification::RunExample1Adaptive(
            QStringLiteral(
                "output/verification/le2012_example1_tssbn"));
    }
    if (!arguments.contains(QStringLiteral("--verify-le2012-example1")))
        return std::nullopt;
    return PaperBeamDynamicsVerification::RunExample1(
        QStringLiteral("output/verification/le2012_example1"));
}

std::optional<int> verifyLe2012Example4(const QStringList& arguments)
{
    if (arguments.contains(
            QStringLiteral("--verify-le2012-example4-tssbn")))
    {
        return PaperBeamDynamicsVerification::RunAdaptive(
            QStringLiteral(
                "output/verification/le2012_example4_tssbn"));
    }
    if (!arguments.contains(QStringLiteral("--verify-le2012-example4")))
        return std::nullopt;
    return PaperBeamDynamicsVerification::Run(
        QStringLiteral("output/verification/le2012_example4"));
}

std::optional<int> verifyBeamDynamics(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-beam-dynamics")))
        return std::nullopt;

    constexpr double length = 2.0;
    constexpr double area = 0.01;
    constexpr double young = 210.0e9;
    constexpr double density = 7850.0;

    auto node0 = std::make_shared<Node>();
    auto node1 = std::make_shared<Node>();
    node0->SetNumDOFs(6);
    node1->SetNumDOFs(6);
    node1->m_X = length;

    auto material = std::make_shared<Material>();
    material->m_Young = young;
    material->m_Poisson = 0.3;
    material->m_Density = density;
    auto section = std::make_shared<SectionCircular>();
    section->m_Area = area;
    auto property = std::make_shared<Property>();
    property->m_pMaterial = material;
    property->m_pSection = section;

    ElementBeam_CR beam;
    beam.m_pNode[0] = node0;
    beam.m_pNode[1] = node1;
    beam.m_pProperty = property;
    beam.q0 = Eigen::Vector3d::UnitY();
    beam.Get_L0();

    Eigen::MatrixXd mass;
    beam.Get_me_Consistent(mass);
    const double expectedMass = density * area * length;
    double assembledMass = 0.0;
    for (int a = 0; a < 2; ++a)
        for (int b = 0; b < 2; ++b)
            assembledMass += mass(6 * a, 6 * b);
    const double symmetryError =
        (mass - mass.transpose()).cwiseAbs().maxCoeff();
    const double massError =
        std::abs(assembledMass - expectedMass) / expectedMass;

    node0->m_Velocity[3] = 1.0;
    node0->m_Velocity[4] = 1.0;
    node1->m_Velocity[3] = 1.0;
    node1->m_Velocity[4] = 1.0;
    Eigen::VectorXd inertiaForce;
    beam.Get_InertiaForce(inertiaForce);
    double Iy = 0.0;
    double Iz = 0.0;
    double polar = 0.0;
    section->Calculate_I(Iy, Iz, polar);
    const double expectedNodalGyroscopicMoment =
        -0.5 * length * density * Iy;
    const double gyroscopicError = std::abs(
        inertiaForce(5) - expectedNodalGyroscopicMoment)
        / std::abs(expectedNodalGyroscopicMoment);

    const double stiffness = young * area / length;
    const double effectiveMass = mass(6, 6);
    const double omega = std::sqrt(stiffness / effectiveMass);
    const double period = 2.0 * std::acos(-1.0) / omega;
    const int steps = 200;
    const double dt = period / steps;
    constexpr double beta = 0.25;
    constexpr double gamma = 0.5;
    constexpr double initialDisplacement = 1.0e-3;
    double displacement = initialDisplacement;
    double velocity = 0.0;
    double acceleration = -stiffness * displacement / effectiveMass;
    for (int step = 0; step < steps; ++step)
    {
        const double predictedDisplacement = displacement
            + dt * velocity + dt * dt * (0.5 - beta) * acceleration;
        const double predictedVelocity =
            velocity + dt * (1.0 - gamma) * acceleration;
        const double newAcceleration =
            -stiffness * predictedDisplacement
            / (effectiveMass + beta * dt * dt * stiffness);
        displacement =
            predictedDisplacement + beta * dt * dt * newAcceleration;
        velocity = predictedVelocity + gamma * dt * newAcceleration;
        acceleration = newAcceleration;
    }
    const double responseError =
        std::abs(displacement - initialDisplacement)
        / initialDisplacement;

    QTextStream(stdout)
        << "beam dynamics mass_error=" << massError
        << " symmetry_error=" << symmetryError
        << " gyroscopic_error=" << gyroscopicError
        << " period=" << period
        << " one_period_response_error=" << responseError
        << Qt::endl;
    return massError <= 1.0e-12
        && symmetryError <= 1.0e-12
        && gyroscopicError <= 1.0e-12
        && responseError <= 2.0e-3 ? 0 : 1;
}

std::optional<int> verifyExactAerodynamicAngle(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-exact-aero-angle")))
        return std::nullopt;

    AerodynamicSectionState state;
    state.firstPosition = Eigen::Vector3d(0.0, 0.0, 0.0);
    state.secondPosition = Eigen::Vector3d(5.0, 0.0, 0.0);
    state.firstVelocity = state.secondVelocity = Eigen::Vector3d(0.0, 3.0, 1.0);
    state.windVelocity = Eigen::Vector3d(0.0, 0.0, 10.0);
    state.firstTwist = state.secondTwist = 0.2;
    state.firstTwistRate = state.secondTwistRate = 2.0;
    state.radius = 0.5;
    state.initialAttack = 0.4;

    auto result = AerodynamicLoadCalculator::ComputeKinematics(state);
    const double expectedSpeed = std::hypot(4.0, 9.0);
    const double expectedFlowAngle = std::atan2(4.0, 9.0);
    const double expectedAttack = 0.4 + 0.2 - expectedFlowAngle;
    const double speedError = std::abs(result.relativeSpeed - expectedSpeed);
    const double flowError = std::abs(result.flowAngle - expectedFlowAngle);
    const double attackError = std::abs(result.attackAngle - expectedAttack);

    AerodynamicLoadCalculator::ComputeLineLoad(result, 1.225, 1.0, 1.0, 2.0, 0.1);
    const bool finiteLoads = result.lineForce.allFinite() && result.lineMoment.allFinite();
    QTextStream(stdout)
        << "exact aero relative_speed=" << result.relativeSpeed
        << " flow_angle=" << result.flowAngle
        << " attack_angle=" << result.attackAngle
        << " speed_error=" << speedError
        << " flow_error=" << flowError
        << " attack_error=" << attackError << Qt::endl;
    return speedError <= 1.0e-12 && flowError <= 1.0e-12
        && attackError <= 1.0e-12 && finiteLoads ? 0 : 1;
}

std::optional<int> verifyGallopingCaseSelection(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-galloping-case")))
        return std::nullopt;

    StructureData structure;
    AnalysisStepConfig config;
    config.id = 1;
    config.type = EnumKeyword::StepType::DYNAMIC;
    config.enableGalloping = true;
    config.gallopingIceThickness = 25;
    structure.AddAnalysisStep(config);
    const auto step = structure.m_AnalysisStep.at(1);
    Force_Wind wind;
    wind.m_velocity = 14;
    const AeroCaseKey key = step->GetGallopingAeroCase(4, wind);
    const bool supported = step->ShouldAssembleGalloping(4, wind);
    const bool unsupportedBundleRejected = !step->ShouldAssembleGalloping(2, wind);

    config.id = 2;
    config.type = EnumKeyword::StepType::STATIC;
    structure.AddAnalysisStep(config);
    const bool staticRejected = !structure.m_AnalysisStep.at(2)->ShouldAssembleGalloping(4, wind);
    AeroManager manager;
    const bool loaded = manager.loadCase(
        std::filesystem::path("YQY/Import/Aero_Data/Input_Data"), key);
    AeroManager completeCatalog;
    const bool allCasesLoaded = completeCatalog.loadAllCases(
        std::filesystem::path("YQY/Import/Aero_Data/Input_Data"));
    bool catalogShapeValid =
        completeCatalog.getLoadedCaseCount() == 40;
    double maximumEndpointGap = 0.0;
    double maximumSymmetryError = 0.0;
    for (const auto& [caseKey, models] : completeCatalog.getCaseModels())
    {
        catalogShapeValid = catalogShapeValid
            && static_cast<int>(models.size()) == caseKey.bundleCount;
        for (const BladeModel& model : models)
        {
            catalogShapeValid = catalogShapeValid
                && model.lift.size() == 73
                && model.drag.size() == 73
                && model.moment.size() == 73;
            if (model.lift.size() == 73)
            {
                maximumEndpointGap = std::max(maximumEndpointGap,
                    std::abs(model.lift.front() - model.lift.back()));
                maximumEndpointGap = std::max(maximumEndpointGap,
                    std::abs(model.drag.front() - model.drag.back()));
                maximumEndpointGap = std::max(maximumEndpointGap,
                    std::abs(model.moment.front() - model.moment.back()));
            }
        }
        if (models.size() == 1 || models.size() == 4)
        {
            const std::vector<int> reflectedProfile =
                models.size() == 1 ? std::vector<int>{ 0 }
                                   : std::vector<int>{ 0, 3, 2, 1 };
            for (int sourceProfile = 0;
                sourceProfile < static_cast<int>(models.size());
                ++sourceProfile)
            {
                const int targetProfile =
                    reflectedProfile[static_cast<size_t>(sourceProfile)];
                for (int sourceIndex = 1; sourceIndex < 36; ++sourceIndex)
                {
                    const int targetIndex = 72 - sourceIndex;
                    maximumSymmetryError = std::max(
                        maximumSymmetryError,
                        std::abs(models[sourceProfile].lift[sourceIndex]
                            + models[targetProfile].lift[targetIndex]));
                    maximumSymmetryError = std::max(
                        maximumSymmetryError,
                        std::abs(models[sourceProfile].drag[sourceIndex]
                            - models[targetProfile].drag[targetIndex]));
                    maximumSymmetryError = std::max(
                        maximumSymmetryError,
                        std::abs(models[sourceProfile].moment[sourceIndex]
                            + models[targetProfile].moment[targetIndex]));
                }
            }
        }
    }
    const auto* boundCase = manager.findCaseModels(key);
    const AeroCoefficients coefficients = boundCase
        ? manager.getCoefficients(*boundCase, 0, 17.5) : AeroCoefficients{};
    const bool combinedLookupMatches = boundCase
        && std::abs(coefficients.lift - manager.getData(key, 0, LIFT, 17.5)) <= 1.0e-12
        && std::abs(coefficients.drag - manager.getData(key, 0, DRAG, 17.5)) <= 1.0e-12
        && std::abs(coefficients.moment - manager.getData(key, 0, MOMENT, 17.5)) <= 1.0e-12;
    const double expected357 = boundCase && !boundCase->empty()
        ? boundCase->front().lift[71]
            + 0.4 * (boundCase->front().lift[72] - boundCase->front().lift[71])
        : 0.0;
    const bool periodicAngles =
        std::abs(AeroManager::normalizeAngleDegrees(357.0) - 357.0) <= 1.0e-12
        && std::abs(AeroManager::normalizeAngleDegrees(1000.0) - 280.0) <= 1.0e-12
        && std::abs(manager.getData(key, 0, LIFT, 357.0) - expected357) <= 1.0e-12
        && std::abs(manager.getData(key, 0, LIFT, 1000.0)
            - manager.getData(key, 0, LIFT, 280.0)) <= 1.0e-12;
    const Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    const int referenceProfile = BundleAeroMapper::ResolveProfile(
        4, 0, axis, up, -Eigen::Vector3d::UnitY());
    const int reversedProfile = BundleAeroMapper::ResolveProfile(
        4, 0, axis, up, Eigen::Vector3d::UnitY());
    const bool profileMapping = referenceProfile == 0 && reversedProfile == 2;

    auto firstNode = std::make_shared<Node>();
    auto secondNode = std::make_shared<Node>();
    firstNode->SetNumDOFs(4);
    secondNode->SetNumDOFs(4);
    secondNode->m_X = 2.0;
    for (int dof = 0; dof < 4; ++dof)
    {
        firstNode->m_DOF[dof] = dof;
        secondNode->m_DOF[dof] = dof + 4;
    }
    auto section = std::make_shared<SectionCircular>();
    section->m_Radius = 0.015;
    section->m_Area = std::acos(-1.0) * section->m_Radius * section->m_Radius;
    auto property = std::make_shared<Property>();
    property->m_pSection = section;
    auto element = std::make_shared<ElementCable>();
    element->m_pNode[0] = firstNode;
    element->m_pNode[1] = secondNode;
    element->m_pProperty = property;
    element->m_WireId = 0;
    element->m_AeroBundleCount = 4;
    element->m_AeroProfileId = 0;
    structure.m_Elements[1] = element;
    structure.m_AeroManager = manager;
    wind.m_direction = -Eigen::Vector3d::UnitY();
    Eigen::VectorXd fixed = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd free = Eigen::VectorXd::Zero(4);
    LoadAssembler::AssembleGalloping(
        wind, structure, 25, 45.0, up, 4, 1.0, fixed, free);
    const bool aerodynamicAssembly =
        fixed.allFinite() && free.allFinite()
        && fixed.norm() > 0.0 && free.norm() > 0.0;
    Eigen::VectorXd angle1000Fixed = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd angle1000Free = Eigen::VectorXd::Zero(4);
    LoadAssembler::AssembleGalloping(
        wind, structure, 25, 1000.0, up, 4, 1.0,
        angle1000Fixed, angle1000Free);
    Eigen::VectorXd angle280Fixed = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd angle280Free = Eigen::VectorXd::Zero(4);
    LoadAssembler::AssembleGalloping(
        wind, structure, 25, 280.0, up, 4, 1.0,
        angle280Fixed, angle280Free);
    const double periodicAssemblyError = std::max(
        (angle1000Fixed - angle280Fixed).norm(),
        (angle1000Free - angle280Free).norm());

    QTextStream(stdout)
        << "galloping case bundle=" << key.bundleCount
        << " wind=" << key.windSpeed
        << " ice=" << key.iceThickness
        << " supported=" << supported
        << " unsupported_bundle_rejected=" << unsupportedBundleRejected
        << " static_rejected=" << staticRejected
        << " case_loaded=" << loaded
        << " all_cases_loaded=" << allCasesLoaded
        << " catalog_cases=" << completeCatalog.getLoadedCaseCount()
        << " max_0_360_gap=" << maximumEndpointGap
        << " max_symmetry_error=" << maximumSymmetryError
        << " combined_lookup_matches=" << combinedLookupMatches
        << " angle_357_cl=" << manager.getData(key, 0, LIFT, 357.0)
        << " angle_1000=" << AeroManager::normalizeAngleDegrees(1000.0)
        << " reference_profile=" << referenceProfile
        << " reversed_profile=" << reversedProfile
        << " aero_force_norm=" << free.norm()
        << " periodic_force_error=" << periodicAssemblyError << Qt::endl;
    return supported && unsupportedBundleRejected && staticRejected
        && loaded && allCasesLoaded && catalogShapeValid
        && maximumEndpointGap <= 1.0e-12
        && maximumSymmetryError <= 1.0e-12
        && combinedLookupMatches && periodicAngles && profileMapping
        && aerodynamicAssembly && periodicAssemblyError <= 1.0e-12
        && key == AeroCaseKey{ 4, 14, 25 } ? 0 : 1;
}

std::optional<int> verifyCableTorsion(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-cable-torsion")))
        return std::nullopt;

    // Uniform circular cable: fixed at x=0, torque applied at x=L.
    // Saint-Venant solution: T = GJ*theta/L, and the fixed-free first
    // torsional frequency is omega_1 = pi/(2L)*sqrt(G/rho).
    constexpr double length = 10.0;
    constexpr double radius = 0.02;
    constexpr double young = 200.0e9;
    constexpr double poisson = 0.30;
    constexpr double density = 7850.0;
    constexpr double appliedTwist = 0.01;
    constexpr int elementCount = 32;

    const double area = std::acos(-1.0) * radius * radius;
    const double polarMoment = 0.5 * area * radius * radius;
    const double shearModulus = young / (2.0 * (1.0 + poisson));

    auto material = std::make_shared<Material>();
    material->m_Young = young;
    material->m_Poisson = poisson;
    material->m_Density = density;
    auto section = std::make_shared<SectionCircular>();
    section->m_Area = area;
    auto property = std::make_shared<Property>();
    property->m_pMaterial = material;
    property->m_pSection = section;

    auto node0 = std::make_shared<Node>();
    auto node1 = std::make_shared<Node>();
    node0->SetNumDOFs(4);
    node1->SetNumDOFs(4);
    node1->m_X = length;
    node1->m_Displacement[3] = appliedTwist;
    ElementCable staticCable;
    staticCable.m_pNode[0] = node0;
    staticCable.m_pNode[1] = node1;
    staticCable.m_pProperty = property;
    staticCable.m_InitStress = 100.0e6;
    Eigen::MatrixXd staticStiffness;
    staticCable.Get_ke(staticStiffness);
    const double expectedTorque = shearModulus * polarMoment / length * appliedTwist;
    const double staticError = std::abs(staticCable.m_inforce(7) - expectedTorque)
        / expectedTorque;
    const double torsionalStiffnessError = std::abs(
        staticStiffness(7, 7) - shearModulus * polarMoment / length)
        / (shearModulus * polarMoment / length);

    // A compressed, initially unstressed cable must go slack: axial terms
    // disappear, whereas the explicitly modelled torsional DOF remains valid.
    node1->m_Displacement[0] = -1.0e-3;
    node1->m_Displacement[3] = 0.0;
    staticCable.m_InitStress = 0.0;
    Eigen::MatrixXd slackStiffness;
    staticCable.Get_ke(slackStiffness);
    const double slackAxialForce = staticCable.m_inforce.head<3>().norm()
        + staticCable.m_inforce.segment<3>(4).norm();
    const double slackTorsionalStiffness = slackStiffness(7, 7);

    std::vector<std::shared_ptr<Node>> nodes(elementCount + 1);
    for (int index = 0; index <= elementCount; ++index)
    {
        nodes[index] = std::make_shared<Node>();
        nodes[index]->SetNumDOFs(4);
        nodes[index]->m_X = length * index / elementCount;
    }
    Eigen::MatrixXd torsionalK = Eigen::MatrixXd::Zero(elementCount, elementCount);
    Eigen::MatrixXd torsionalM = Eigen::MatrixXd::Zero(elementCount, elementCount);
    for (int index = 0; index < elementCount; ++index)
    {
        ElementCable cable;
        cable.m_pNode[0] = nodes[index];
        cable.m_pNode[1] = nodes[index + 1];
        cable.m_pProperty = property;
        cable.m_InitStress = 100.0e6;
        Eigen::MatrixXd stiffness;
        Eigen::MatrixXd mass;
        cable.Get_ke(stiffness);
        cable.Get_me_Consistent(mass);
        const int localNode[2] = {index - 1, index};
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b)
                if (localNode[a] >= 0 && localNode[b] >= 0)
                {
                    torsionalK(localNode[a], localNode[b]) += stiffness(4 * a + 3, 4 * b + 3);
                    torsionalM(localNode[a], localNode[b]) += mass(4 * a + 3, 4 * b + 3);
                }
    }
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> modes(torsionalK, torsionalM);
    if (modes.info() != Eigen::Success)
        return 2;
    const double computedOmega = std::sqrt(modes.eigenvalues()(0));
    const double expectedOmega = std::acos(-1.0) / (2.0 * length)
        * std::sqrt(shearModulus / density);
    const double frequencyError = std::abs(computedOmega - expectedOmega) / expectedOmega;

    // Newmark average-acceleration integration of that FEM first torsional
    // mode. One calculated period must recover its initial amplitude.
    const double period = 2.0 * std::acos(-1.0) / computedOmega;
    constexpr int steps = 400;
    const double dt = period / steps;
    double displacement = appliedTwist;
    double velocity = 0.0;
    double acceleration = -computedOmega * computedOmega * displacement;
    for (int step = 0; step < steps; ++step)
    {
        const double predictedDisplacement = displacement + dt * velocity
            + 0.25 * dt * dt * acceleration;
        const double predictedVelocity = velocity + 0.5 * dt * acceleration;
        const double newAcceleration = -computedOmega * computedOmega
            * predictedDisplacement / (1.0 + 0.25 * dt * dt * computedOmega * computedOmega);
        displacement = predictedDisplacement + 0.25 * dt * dt * newAcceleration;
        velocity = predictedVelocity + 0.5 * dt * newAcceleration;
        acceleration = newAcceleration;
    }
    const double transientError = std::abs(displacement - appliedTwist) / appliedTwist;

    QTextStream(stdout)
        << "cable torsion static_torque_error=" << staticError
        << " static_stiffness_error=" << torsionalStiffnessError
        << " slack_axial_force=" << slackAxialForce
        << " slack_torsional_stiffness=" << slackTorsionalStiffness
        << " omega=" << computedOmega
        << " omega_reference=" << expectedOmega
        << " frequency_error=" << frequencyError
        << " one_period_response_error=" << transientError << Qt::endl;
    return staticError <= 1.0e-12
        && torsionalStiffnessError <= 1.0e-12
        && slackAxialForce <= 1.0e-8
        && slackTorsionalStiffness > 0.0
        && frequencyError <= 2.0e-3
        && transientError <= 2.0e-3 ? 0 : 3;
}

std::optional<int> verifySpatialWindLoad(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-spatial-wind-load")))
        return std::nullopt;

    constexpr double radius = 0.015;
    constexpr double initialLength = 5.0;
    constexpr double density = 1.225;
    constexpr double speed = 10.0;
    const double q = 0.5 * density * speed * speed * (2.0 * radius);

    StructureData structure;
    auto node0 = std::make_shared<Node>();
    auto node1 = std::make_shared<Node>();
    node0->SetNumDOFs(4);
    node1->SetNumDOFs(4);
    node1->m_X = initialLength;
    for (int i = 0; i < 4; ++i)
    {
        node0->m_DOF[i] = i;
        node1->m_DOF[i] = i + 4;
    }

    auto material = std::make_shared<Material>();
    material->m_Density = 7850.0;
    auto section = std::make_shared<SectionCircular>();
    section->m_Radius = radius;
    section->m_Area = std::acos(-1.0) * radius * radius;
    auto property = std::make_shared<Property>();
    property->m_pMaterial = material;
    property->m_pSection = section;
    auto cable = std::make_shared<ElementCable>();
    cable->m_pNode[0] = node0;
    cable->m_pNode[1] = node1;
    cable->m_pProperty = property;
    structure.m_Elements.emplace(1, cable);

    Force_Wind wind;
    wind.m_Direction = EnumKeyword::Direction::Y;
    wind.m_velocity = speed;
    wind.m_windDensity = density;

    Eigen::VectorXd fixed = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd free = Eigen::VectorXd::Zero(4);
    LoadAssembler::Assemble(wind, structure, 4, 1.0, fixed, free);
    const double forceError = std::max(
        std::abs(fixed[1] - q * initialLength / 2.0),
        std::abs(free[1] - q * initialLength / 2.0));

    // Rotate the current cable chord to prove that the spatial wind direction
    // stays global while equivalent nodal loads use the current configuration.
    node1->m_Displacement[0] = -2.0;
    node1->m_Displacement[1] = 4.0;
    fixed.setZero();
    free.setZero();
    LoadAssembler::Assemble(wind, structure, 4, 1.0, fixed, free);
    const double rotatedForceError = std::max(
        std::abs(fixed[1] - q * 5.0 / 2.0),
        std::abs(free[1] - q * 5.0 / 2.0));

    // An arbitrary global direction must be assembled component-by-component
    // without reducing it to one of the coordinate axes.
    std::fill(
        node1->m_Displacement.begin(),
        node1->m_Displacement.end(),
        0.0);
    wind.m_direction = Eigen::Vector3d(1.0, 2.0, -2.0);
    fixed.setZero();
    free.setZero();
    LoadAssembler::Assemble(wind, structure, 4, 1.0, fixed, free);
    const Eigen::Vector3d expectedVector =
        q * initialLength / 2.0 * wind.m_direction.normalized();
    const double vectorDirectionError = std::max(
        (fixed.head<3>() - expectedVector).norm(),
        (free.head<3>() - expectedVector).norm());

    QTextStream(stdout)
        << "spatial wind q=" << q
        << " initial_node_force=" << q * initialLength / 2.0
        << " rotated_node_force=" << q * 5.0 / 2.0
        << " error=" << forceError
        << " rotated_error=" << rotatedForceError
        << " vector_error=" << vectorDirectionError << Qt::endl;
    return forceError <= 1.0e-12 && rotatedForceError <= 1.0e-12
        && vectorDirectionError <= 1.0e-12 ? 0 : 1;
}

std::optional<int> verifyNodeExport(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-node-export"));
    if (index < 0)
        return std::nullopt;
    if (index + 2 >= arguments.size())
        return 1;

    const QString source = arguments.at(index + 1);
    const QString target = arguments.at(index + 2);
    Hdf5ModelIO exporter;
    const bool exported =
        exporter.ExportBdfResultFromHdf5(source, target, {1},
                                         {EnumKeyword::NodeResultType::U1, EnumKeyword::NodeResultType::U2,
                                          EnumKeyword::NodeResultType::U3, EnumKeyword::NodeResultType::MagnitudeU},
                                         {}, {});
    QTextStream(stdout) << "node export=" << (exported ? "success" : "failed") << " file=" << target << Qt::endl;
    return exported ? 0 : 2;
}

std::optional<int> verifyResultRead(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-result-read"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    Hdf5ModelIO reader;
    std::vector<Hdf5ResultFrameInfo> frames;
    if (!reader.OpenResultFile(arguments.at(index + 1), frames) || frames.empty())
        return 2;
    Hdf5ResultFrame first;
    Hdf5ResultFrame last;
    if (!reader.ReadResultFrame(0, first) || !reader.ReadResultFrame(static_cast<int>(frames.size()) - 1, last))
    {
        return 3;
    }
    QTextStream(stdout) << "result frames=" << frames.size() << " first_step=" << first.info.stepId
                        << " first_nodes=" << first.nodes.size() << " first_elements=" << first.elements.size()
                        << " last_time=" << last.info.time << Qt::endl;
    return first.nodes.empty() || first.elements.empty() ? 4 : 0;
}

std::optional<int> verifyShearReleaseResult(
    const QStringList& arguments)
{
    const int index = arguments.indexOf(
        QStringLiteral("--verify-shear-release-result"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    Hdf5ModelIO reader;
    std::vector<Hdf5ResultFrameInfo> frames;
    if (!reader.OpenResultFile(arguments.at(index + 1), frames)
        || frames.empty())
        return 2;

    double maximumConstraintGap = 0.0;
    double maximumNormalizedError = 0.0;
    double finalValues[4] = {};
    for (int frameIndex = 0;
         frameIndex < static_cast<int>(frames.size());
         ++frameIndex)
    {
        Hdf5ResultFrame frame;
        if (!reader.ReadResultFrame(frameIndex, frame))
            return 3;
        const auto findNode = [&frame](int id)
            -> const Hdf5NodalResult*
        {
            const auto found = std::find_if(
                frame.nodes.cbegin(), frame.nodes.cend(),
                [id](const Hdf5NodalResult& node)
                {
                    return node.id == id;
                });
            return found == frame.nodes.cend() ? nullptr : &*found;
        };
        const Hdf5NodalResult* node2 = findNode(2);
        const Hdf5NodalResult* node3 = findNode(3);
        if (!node2 || !node3)
            return 4;

        const double phi =
            2.0 * std::acos(-1.0) * frame.info.time;
        if (std::abs(phi) < 1.0e-12)
            continue;
        const double sinPhi = std::sin(phi);
        const double cosPhi = std::cos(phi);
        const double u2Expected = sinPhi / phi - 1.0;
        const double w2Expected = (1.0 - cosPhi) / phi;
        const double u3Expected =
            u2Expected
            + 2.0 * std::tan(phi) * (1.0 - cosPhi) / phi;
        const double w3Expected = -w2Expected;
        const double calculated[4] = {
            node2->displacement[0], node2->displacement[1],
            node3->displacement[0], node3->displacement[1]
        };
        const double expected[4] = {
            u2Expected, w2Expected, u3Expected, w3Expected
        };
        for (int component = 0; component < 4; ++component)
        {
            if (!std::isfinite(calculated[component]))
                return 5;
            // The paper also reports visibly larger discretization errors in
            // the load steps close to M=pi and 3*pi, where u3 is singular.
            if (std::abs(cosPhi) >= 0.10)
            {
                maximumNormalizedError = std::max(
                    maximumNormalizedError,
                    std::abs(calculated[component] - expected[component])
                        / std::max(1.0, std::abs(expected[component])));
            }
            if (frameIndex + 1 == static_cast<int>(frames.size()))
                finalValues[component] = calculated[component];
        }

        const double constraintGap =
            cosPhi * (calculated[0] - calculated[2])
            + sinPhi * (calculated[1] - calculated[3]);
        maximumConstraintGap =
            std::max(maximumConstraintGap, std::abs(constraintGap));
    }
    reader.CloseResultFile();

    QTextStream(stdout)
        << "shear-release frames=" << frames.size()
        << " max_constraint_gap=" << maximumConstraintGap
        << " max_normalized_analytical_error="
        << maximumNormalizedError
        << " final_u2=" << finalValues[0]
        << " final_w2=" << finalValues[1]
        << " final_u3=" << finalValues[2]
        << " final_w3=" << finalValues[3] << Qt::endl;
    return maximumConstraintGap <= 1.0e-8
        && maximumNormalizedError <= 0.25 ? 0 : 6;
}

std::optional<int> verifyShearReleaseModel(
    const QStringList& arguments)
{
    const int index = arguments.indexOf(
        QStringLiteral("--verify-shear-release-model"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), structure))
        return 2;
    structure->m_OutputControl.m_StreamResult = false;
    AnalysisRunner runner;
    runner.SetStructure(structure);
    runner.SetRuntimeCallbacks(
        [](double progress, const QString& message)
        {
            if (message.contains(QStringLiteral("失败"))
                || message.contains(QStringLiteral("未收敛"))
                || message.contains(QStringLiteral("奇异")))
            {
                QTextStream(stdout)
                    << "progress=" << progress
                    << " message=" << message << Qt::endl;
            }
        },
        []() { return false; });
    if (!runner.RunAll())
        return 3;

    const auto& frames = structure->GetOutputter().GetFrames();
    if (frames.empty())
        return 4;
    double maximumConstraintGap = 0.0;
    double maximumRotationGap = 0.0;
    double maximumNormalizedError = 0.0;
    double worstPhi = 0.0;
    double worstCalculated = 0.0;
    double worstExpected = 0.0;
    int worstComponent = -1;
    for (const DataFrame& frame : frames)
    {
        const double u2 = frame.GetNodeData(
            2, EnumKeyword::NodeResultType::U1);
        const double w2 = frame.GetNodeData(
            2, EnumKeyword::NodeResultType::U2);
        const double u3 = frame.GetNodeData(
            3, EnumKeyword::NodeResultType::U1);
        const double w3 = frame.GetNodeData(
            3, EnumKeyword::NodeResultType::U2);
        const double phi2Output = frame.GetNodeData(
            2, EnumKeyword::NodeResultType::UR3);
        const double phi3 = frame.GetNodeData(
            3, EnumKeyword::NodeResultType::UR3);
        const double phi2 =
            2.0 * std::acos(-1.0) * frame.GetTime();
        const double sinPhi = std::sin(phi2);
        const double cosPhi = std::cos(phi2);
        const double expected[4] = {
            sinPhi / phi2 - 1.0,
            (1.0 - cosPhi) / phi2,
            sinPhi / phi2 - 1.0
                + 2.0 * std::tan(phi2)
                    * (1.0 - cosPhi) / phi2,
            -(1.0 - cosPhi) / phi2
        };
        const double calculated[4] = {u2, w2, u3, w3};
        for (int component = 0; component < 4; ++component)
        {
            if (!std::isfinite(calculated[component]))
                return 5;
            if (std::abs(cosPhi) >= 0.10)
            {
                const double error =
                    std::abs(calculated[component] - expected[component])
                    / std::max(1.0, std::abs(expected[component]));
                if (error > maximumNormalizedError)
                {
                    maximumNormalizedError = error;
                    worstPhi = phi2;
                    worstCalculated = calculated[component];
                    worstExpected = expected[component];
                    worstComponent = component;
                }
            }
        }
        maximumConstraintGap = std::max(
            maximumConstraintGap,
            std::abs(cosPhi * (u2 - u3)
                + sinPhi * (w2 - w3)));
        maximumRotationGap = std::max(
            maximumRotationGap,
            std::abs(std::remainder(
                phi2Output - phi3, 2.0 * std::acos(-1.0))));
    }

    const DataFrame& last = frames.back();
    QTextStream(stdout)
        << "shear-release direct frames=" << frames.size()
        << " max_constraint_gap=" << maximumConstraintGap
        << " max_rotation_gap=" << maximumRotationGap
        << " max_normalized_analytical_error="
        << maximumNormalizedError
        << " worst_phi=" << worstPhi
        << " worst_component=" << worstComponent
        << " worst_calculated=" << worstCalculated
        << " worst_expected=" << worstExpected
        << " final_u2=" << last.GetNodeData(
            2, EnumKeyword::NodeResultType::U1)
        << " final_w2=" << last.GetNodeData(
            2, EnumKeyword::NodeResultType::U2)
        << " final_u3=" << last.GetNodeData(
            3, EnumKeyword::NodeResultType::U1)
        << " final_w3=" << last.GetNodeData(
            3, EnumKeyword::NodeResultType::U2)
        << Qt::endl;
    return maximumConstraintGap <= 1.0e-8
        && maximumRotationGap <= 1.0e-8
        && maximumNormalizedError <= 0.25 ? 0 : 6;
}

std::optional<int> verifyHdf5Model(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-hdf5-model"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Hdf5ModelIO reader;
    if (!reader.ImportHdf5(arguments.at(index + 1), structure.get()))
        return 2;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        if (!element || element->m_pNode.size() != 2 || element->m_pNode[0].expired()
            || element->m_pNode[1].expired())
        {
            QTextStream(stderr) << "invalid element=" << elementId << Qt::endl;
            return 3;
        }
    }
    QTextStream(stdout) << "hdf5 model nodes=" << structure->m_Nodes.size()
                        << " elements=" << structure->m_Elements.size()
                        << " materials=" << structure->m_Material.size()
                        << " sections=" << structure->m_Section.size() << Qt::endl;
    return structure->m_Nodes.empty() || structure->m_Elements.empty() ? 4 : 0;
}

std::optional<int> verifyHdf5ModelContract(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-hdf5-model-contract"));
    if (index < 0)
        return std::nullopt;
    if (index + 2 >= arguments.size())
        return 1;

    auto source = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), source))
        return 2;
    source->m_ModelSets.clear();
    source->m_ComputeRegions.clear();
    QString error;
    const int secondElementId = source->m_Elements.size() > 2
        ? source->m_Elements.crbegin()->first : 2;
    const int firstSetId = source->AddModelSet(QStringLiteral("区域A单元"), ModelSetType::Element, {1}, &error);
    const int secondSetId = source->AddModelSet(
        QStringLiteral("区域B单元"), ModelSetType::Element,
        {secondElementId}, &error);
    const int firstRegionId = source->AddComputeRegionFromSets(
        QStringLiteral("区域A"), {firstSetId}, true, &error);
    const int secondRegionId = source->AddComputeRegionFromSets(
        QStringLiteral("区域B"), {secondSetId}, true, &error);
    if (firstSetId <= 0 || secondSetId <= 0 || firstRegionId <= 0 || secondRegionId <= 0)
        return 3;

    source->EnsureDefaultAnalysisConfiguration();
    if (source->m_AnalysisStep.empty() || !source->m_AnalysisStep.begin()->second)
        return 4;
    const int equilibriumStepId = source->m_AnalysisStep.begin()->first;
    auto equilibriumStep = source->m_AnalysisStep.begin()->second;
    equilibriumStep->m_Name = QStringLiteral("静力平衡");
    equilibriumStep->m_Type = EnumKeyword::StepType::STATIC;
    equilibriumStep->isDynamic = false;
    equilibriumStep->m_InitialStaticStepId = 0;
    AnalysisStepConfig dynamicConfig;
    dynamicConfig.id = source->m_AnalysisStep.crbegin()->first + 1;
    dynamicConfig.name = QStringLiteral("导线区域分析");
    dynamicConfig.type = EnumKeyword::StepType::DYNAMIC;
    dynamicConfig.totalTime = 1.0;
    dynamicConfig.stepSize = 0.1;
    dynamicConfig.tolerance = 1.0e-5;
    dynamicConfig.maxIterations = 20;
    dynamicConfig.initialStaticStepId = equilibriumStepId;
    source->AddAnalysisStep(dynamicConfig);
    auto sourceStep = source->m_AnalysisStep.at(dynamicConfig.id);
    sourceStep->m_DynamicSolverType =
        SolverNameSpace::SolverType::AdaptiveTSSBN;
    sourceStep->m_AdaptiveTssbn.spectralRadiusInfinity = 0.73;
    sourceStep->m_AdaptiveTssbn.minimumTimeStep = 2.5e-6;
    sourceStep->m_AdaptiveTssbn.maximumTimeStep = 0.37;
    sourceStep->m_AdaptiveTssbn.relativeTolerance = 7.5e-4;
    sourceStep->m_AdaptiveTssbn.absoluteTolerance = 3.5e-7;
    sourceStep->m_AdaptiveTssbn.safetyFactor = 0.82;
    sourceStep->m_AdaptiveTssbn.shrinkFactor = 0.76;
    sourceStep->m_AdaptiveTssbn.maximumGrowthFactor = 2.4;
    sourceStep->m_AdaptiveTssbn.targetNewtonIterations = 11;
    sourceStep->m_AdaptiveTssbn.derivativeGain = 0.13;
    sourceStep->m_AdaptiveTssbn.minimumDerivativeFactor = 0.44;
    sourceStep->m_AdaptiveTssbn.maximumDerivativeFactor = 1.72;
    sourceStep->m_AdaptiveTssbn.maximumRejectedAttempts = 19;
    sourceStep->m_EnableGalloping = true;
    sourceStep->m_GallopingIceThickness = 28;
    sourceStep->m_GallopingInitialAttackDegrees = 357.0;
    sourceStep->m_RegionScope = AnalysisRegionScope::SelectedRegions;
    sourceStep->m_ComputeRegionIds = {firstRegionId, secondRegionId};
    const AeroCaseKey sourceAeroKey{ 1, 18, 28 };
    if (!source->m_AeroManager.loadCase(
        std::filesystem::path("YQY/Import/Aero_Data/Input_Data"), sourceAeroKey))
        return 5;
    const int windLoadId = source->m_Load.empty()
        ? 1 : source->m_Load.crbegin()->first + 1;
    auto sourceWind = std::make_shared<Force_Wind>();
    sourceWind->m_Id = windLoadId;
    sourceWind->m_Name = QStringLiteral("任意方向风");
    sourceWind->m_StepId = sourceStep->m_Id;
    sourceWind->m_velocity = 18.0;
    sourceWind->m_direction = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
    source->m_Load[windLoadId] = sourceWind;

    Hdf5ModelIO writer;
    if (!writer.ExportModelHdf5(arguments.at(index + 2), source.get(), arguments.at(index + 1)))
        return 5;
    Hdf5ResultSummary summary;
    if (!writer.InspectHdf5(arguments.at(index + 2), summary) || !summary.hasModel || summary.hasResult)
        return 6;

    auto restored = std::make_shared<StructureData>();
    if (!writer.ImportHdf5(arguments.at(index + 2), restored.get()))
        return 7;
    const auto restoredStep = restored->m_AnalysisStep.find(sourceStep->m_Id);
    const auto restoredWindEntry = restored->m_Load.find(windLoadId);
    const auto restoredWind = restoredWindEntry != restored->m_Load.cend()
        ? std::dynamic_pointer_cast<Force_Wind>(restoredWindEntry->second) : nullptr;
    const double windDirectionError = restoredWind
        ? (restoredWind->m_direction - sourceWind->m_direction).norm()
        : std::numeric_limits<double>::infinity();
    if (restored->m_ModelSets.size() != 2 || restored->m_ComputeRegions.size() != 2
        || restored->m_MPCConstraints.size() != source->m_MPCConstraints.size()
        || restoredStep == restored->m_AnalysisStep.cend() || !restoredStep->second
        || restoredStep->second->m_Name != sourceStep->m_Name
        || restoredStep->second->m_InitialStaticStepId != equilibriumStepId
        || !restoredStep->second->m_EnableGalloping
        || restoredStep->second->m_GallopingIceThickness != 28
        || restoredStep->second->m_DynamicSolverType
            != SolverNameSpace::SolverType::AdaptiveTSSBN
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.spectralRadiusInfinity
                - 0.73) > 1.0e-12
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.minimumTimeStep
                - 2.5e-6) > 1.0e-15
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.maximumTimeStep
                - 0.37) > 1.0e-12
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.relativeTolerance
                - 7.5e-4) > 1.0e-15
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.absoluteTolerance
                - 3.5e-7) > 1.0e-15
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.safetyFactor
                - 0.82) > 1.0e-12
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.shrinkFactor
                - 0.76) > 1.0e-12
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.maximumGrowthFactor
                - 2.4) > 1.0e-12
        || restoredStep->second->m_AdaptiveTssbn.targetNewtonIterations
            != 11
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.derivativeGain
                - 0.13) > 1.0e-12
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.minimumDerivativeFactor
                - 0.44) > 1.0e-12
        || std::abs(
            restoredStep->second->m_AdaptiveTssbn.maximumDerivativeFactor
                - 1.72) > 1.0e-12
        || restoredStep->second->m_AdaptiveTssbn.maximumRejectedAttempts
            != 19
        || std::abs(
            restoredStep->second->m_GallopingInitialAttackDegrees - 357.0)
            > 1.0e-12
        || !restored->m_AeroManager.hasCase(sourceAeroKey)
        || std::abs(restored->m_AeroManager.getData(sourceAeroKey, 0, LIFT, 17.5)
            - source->m_AeroManager.getData(sourceAeroKey, 0, LIFT, 17.5)) > 1.0e-12
        || restoredStep->second->m_RegionScope != AnalysisRegionScope::SelectedRegions
        || restoredStep->second->m_ComputeRegionIds.size() != 2
        || !restoredWind
        || std::abs(restoredWind->m_velocity - 18.0) > 1.0e-12
        || windDirectionError > 1.0e-12)
    {
        return 8;
    }
    QTextStream(stdout) << "hdf5 contract sets=" << restored->m_ModelSets.size()
        << " regions=" << restored->m_ComputeRegions.size()
        << " mpcs=" << restored->m_MPCConstraints.size()
        << " steps=" << restored->m_AnalysisStep.size()
        << " galloping=" << (restoredStep->second->m_EnableGalloping ? 1 : 0)
        << " ice=" << restoredStep->second->m_GallopingIceThickness
        << " initial_attack="
        << restoredStep->second->m_GallopingInitialAttackDegrees
        << " tssbn_rel_tol="
        << restoredStep->second->m_AdaptiveTssbn.relativeTolerance
        << " tssbn_max_reject="
        << restoredStep->second->m_AdaptiveTssbn.maximumRejectedAttempts
        << " aero_cases=" << restored->m_AeroManager.getLoadedCaseCount()
        << " wind_direction_error=" << windDirectionError
        << " result=" << (summary.hasResult ? 1 : 0) << Qt::endl;
    return 0;
}

std::optional<int> verifyComputeRegions(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-compute-regions"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), structure))
        return 2;
    structure->m_ComputeRegions.clear();
    QString error;
    const int firstRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 A"), {}, {1}, {}, true, &error);
    const int secondRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 B"), {}, {2}, {}, true, &error);
    if (firstRegionId <= 0 || secondRegionId <= 0 || structure->m_ComputeRegions.size() != 2)
    {
        QTextStream(stderr) << "region setup failed: " << error << Qt::endl;
        return 3;
    }
    const int mergedRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 A 重叠候选"), {}, {1}, {}, true, &error);
    if (mergedRegionId != firstRegionId || structure->m_ComputeRegions.size() != 2)
    {
        QTextStream(stderr) << "overlap merge failed: " << error << Qt::endl;
        return 4;
    }

    structure->m_OutputControl.m_StreamResult = false;
    AnalysisRunner runner;
    runner.SetStructure(structure);
    if (!runner.RunAll() || structure->GetOutputter().GetDataSet().empty())
        return 5;
    const auto& frame = structure->GetOutputter().GetDataSet().back();
    QTextStream(stdout) << "compute regions=" << structure->m_ComputeRegions.size()
        << " merged_id=" << mergedRegionId
        << " result_nodes=" << frame.GetNodeDatas().size()
        << " result_elements=" << frame.GetElementDatas().size() << Qt::endl;
    return frame.GetNodeDatas().size() == 4 && frame.GetElementDatas().size() == 2 ? 0 : 6;
}

std::optional<int> verifyMPCRegions(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-mpc-regions"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), structure)
        || structure->m_MPCConstraints.empty())
    {
        return 2;
    }
    const auto sourceMPC = structure->m_MPCConstraints.cbegin()->second;
    const std::vector<int> nodeIds = sourceMPC ? sourceMPC->GetNodeIds()
                                               : std::vector<int>();
    if (nodeIds.size() != 2)
        return 3;
    const int masterNodeId = nodeIds[0];
    const int slaveNodeId = nodeIds[1];

    int detachedElementId = -1;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        bool containsSlave = false;
        if (element)
        {
            for (const auto& nodeRef : element->m_pNode)
            {
                const auto node = nodeRef.lock();
                containsSlave = containsSlave
                    || (node && node->m_Id == slaveNodeId);
            }
        }
        if (element && !containsSlave)
        {
            detachedElementId = elementId;
            break;
        }
    }
    if (detachedElementId < 0)
        return 4;

    structure->m_ComputeRegions.clear();
    QString error;
    const int regionId = structure->AddComputeRegion(
        QStringLiteral("MPC 主节点区域"), {masterNodeId},
        {detachedElementId}, {}, true, &error);
    const auto region = structure->m_ComputeRegions.find(regionId);
    if (regionId <= 0 || region == structure->m_ComputeRegions.cend()
        || !region->second || !region->second->ContainsNode(masterNodeId)
        || !region->second->ContainsNode(slaveNodeId)
        || structure->m_MPCConstraints.size() != 1)
    {
        QTextStream(stderr) << "MPC region expansion failed: "
                            << error << Qt::endl;
        return 5;
    }

    const int stepId = structure->m_AnalysisStep.empty()
        ? 0 : structure->m_AnalysisStep.cbegin()->first;
    if (stepId <= 0)
        return 6;
    auto clone = structure->CloneRegionForAnalysis(regionId, stepId, &error);
    if (!clone || clone->m_MPCConstraints.size() != 1
        || clone->m_Nodes.find(masterNodeId) == clone->m_Nodes.cend()
        || clone->m_Nodes.find(slaveNodeId) == clone->m_Nodes.cend()
        || structure->m_MPCConstraints.size() != 1)
    {
        QTextStream(stderr) << "MPC regional clone failed: "
                            << error << Qt::endl;
        return 7;
    }

    QTextStream(stdout)
        << "mpc regions=" << structure->m_ComputeRegions.size()
        << " master=" << masterNodeId
        << " slave=" << slaveNodeId
        << " source_mpcs=" << structure->m_MPCConstraints.size()
        << " clone_mpcs=" << clone->m_MPCConstraints.size()
        << Qt::endl;
    return 0;
}

std::optional<int> verifyConductorBundle(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-conductor-bundle")))
        return std::nullopt;

    auto structure = std::make_shared<StructureData>();
    Conductor::PropertyLibrary library;
    QString propertyError;
    if (!library.load(propertyError) || !library.isReady())
    {
        QTextStream(stderr) << "conductor property library failed: " << propertyError << Qt::endl;
        return 1;
    }

    auto property = library.instantiateProperty(0, 0, *structure, propertyError);
    if (!property)
    {
        QTextStream(stderr) << "conductor property failed: " << propertyError << Qt::endl;
        return 2;
    }

    Conductor::LineBuildConfig config;
    config.start = Vector3d(0.0, 0.0, 20.0);
    config.end = Vector3d(300.0, 0.0, 20.0);
    config.property = property;
    config.elementType = EnumKeyword::ElementType::T3D2;
    config.conductor.nBundle = 4;
    config.conductor.spacing = 0.45;
    config.conductor.segments = 10;
    config.conductor.stress0 = 50.0e6;
    config.conductor.connecttype = Conductor::ConnectionMode::Parallel;
    config.convergeBundleEnds = true;
    config.setNamePrefix = QStringLiteral("验证单档");

    Conductor::ConductorModelBuilder builder(structure);
    Conductor::LineBuildResult result;
    std::string buildError;
    Conductor::SpanConductorBuildConfig spanConfig;
    spanConfig.line = config;
    spanConfig.useInnerSpacerLayout = true;
    spanConfig.innerSpacerLayout.useEqualSpacing = false;
    spanConfig.innerSpacerLayout.spacer.elementType = EnumKeyword::ElementType::CR3D;
    spanConfig.innerSpacerLayout.spacer.property = property;
    if (!builder.BuildSpanConductor(spanConfig, result, buildError))
    {
        QTextStream(stderr) << "conductor build failed: " << QString::fromStdString(buildError) << Qt::endl;
        return 3;
    }

    if (result.subConductors.size() != 4
        || result.leftSupportNodeId <= 0
        || result.rightSupportNodeId <= 0
        || result.leftTensionEnd.groupNodeIds.size() != 2
        || result.rightTensionEnd.groupNodeIds.size() != 2
        || result.leftTensionEnd.yokeElementIds.size() != 2
        || result.rightTensionEnd.yokeElementIds.size() != 2
        || result.leftTensionEnd.stabilizerElementId <= 0
        || result.rightTensionEnd.stabilizerElementId <= 0
        || result.innerSpacers.empty()
        || structure->m_ModelSets.size() != 8)
    {
        return 4;
    }

    for (const auto& [wireId, sub] : result.subConductors)
    {
        if (sub.nodeIds.empty()
            || sub.nodeSetId <= 0
            || sub.elementSetId <= 0)
        {
            return 5;
        }
    }
    if (result.subConductors.at(0).nodeIds.front() != result.subConductors.at(1).nodeIds.front()
        || result.subConductors.at(2).nodeIds.front() != result.subConductors.at(3).nodeIds.front()
        || result.subConductors.at(0).nodeIds.front() == result.subConductors.at(2).nodeIds.front())
        return 10;

    int expectedConductorNodeId = 1;
    for (std::size_t groupIndex = 0; groupIndex < result.leftTensionEnd.groupNodeIds.size(); ++groupIndex)
    {
        const int leftGroupId = result.leftTensionEnd.groupNodeIds[groupIndex];
        const int rightGroupId = result.rightTensionEnd.groupNodeIds[groupIndex];
        if (leftGroupId != expectedConductorNodeId++)
            return 12;
        for (const auto& [wireId, sub] : result.subConductors)
        {
            if (sub.nodeIds.front() != leftGroupId)
                continue;
            for (std::size_t index = 1; index + 1 < sub.nodeIds.size(); ++index)
            {
                if (sub.nodeIds[index] != expectedConductorNodeId++)
                    return 13;
            }
            if (sub.nodeIds.back() != rightGroupId)
                return 14;
        }
        if (rightGroupId != expectedConductorNodeId++)
            return 15;
    }
    if (result.leftSupportNodeId != expectedConductorNodeId++
        || result.rightSupportNodeId != expectedConductorNodeId++)
        return 16;

    int expectedId = 1;
    for (const auto& [nodeId, node] : structure->m_Nodes)
    {
        if (!node || nodeId != expectedId++)
            return 6;
    }
    expectedId = 1;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        if (!element || elementId != expectedId++)
            return 7;
    }

    int expectedConductorElementId = 1;
    for (const auto& [wireId, sub] : result.subConductors)
    {
        for (int elementId : sub.elementIds)
        {
            const auto element = structure->FindElement(elementId);
            if (elementId != expectedConductorElementId++
                || !element
                || element->m_Role != ElementRole::Conductor
                || element->m_WireId != wireId
                || element->m_AeroBundleCount != 4
                || element->m_AeroProfileId != wireId
                || !element->HasAerodynamicLoad())
                return 17;
        }
    }
    for (const auto* end : {&result.leftTensionEnd, &result.rightTensionEnd})
    {
        std::vector<int> hardwareIds = end->yokeElementIds;
        if (end->stabilizerElementId > 0)
            hardwareIds.push_back(end->stabilizerElementId);
        for (int elementId : hardwareIds)
        {
            const auto element = structure->FindElement(elementId);
            if (!element
                || element->m_Role != ElementRole::TensionHardware
                || std::abs(element->m_InitStress - config.conductor.stress0) > 1.0e-6
                || element->HasAerodynamicLoad())
                return 18;
        }
    }
    for (const auto& spacer : result.innerSpacers)
    {
        for (int elementId : spacer.elementIds)
        {
            const auto element = structure->FindElement(elementId);
            if (!element
                || element->m_Role != ElementRole::IntraPhaseSpacer
                || std::abs(element->m_InitStress) > 1.0e-12
                || element->HasAerodynamicLoad())
                return 19;
        }
    }

    const auto firstWire = result.subConductors.find(0);
    if (firstWire == result.subConductors.cend() || firstWire->second.nodeIds.size() < 3)
        return 8;
    const auto rightBottomNode = structure->FindNode(firstWire->second.nodeIds[1]);
    if (!rightBottomNode || rightBottomNode->m_Y >= 0.0 || rightBottomNode->m_Z >= config.start.z())
        return 9;
    const double transitionDx = rightBottomNode->m_X - config.start.x();
    const double transitionDy = rightBottomNode->m_Y - config.start.y();
    if (std::hypot(transitionDx, transitionDy) < 0.5)
        return 11;

    // 文本导线模型的简化端部：四根子导线均直接锚固在端部截面，
    // 不应生成汇集节点、耐张联板或稳定梁。
    auto directStructure = std::make_shared<StructureData>();
    auto directProperty = library.instantiateProperty(0, 0, *directStructure, propertyError);
    if (!directProperty)
        return 22;
    auto directConfig = config;
    directConfig.property = directProperty;
    directConfig.endTopology = Conductor::BundleEndTopology::DirectWireSupports;
    Conductor::ConductorModelBuilder directBuilder(directStructure);
    Conductor::LineBuildResult directResult;
    if (!directBuilder.BuildLine(directConfig, directResult, buildError))
        return 23;
    if (directResult.leftTensionEnd.supportNodeIds.size() != 4
        || directResult.rightTensionEnd.supportNodeIds.size() != 4
        || !directResult.leftTensionEnd.groupNodeIds.empty()
        || !directResult.rightTensionEnd.groupNodeIds.empty()
        || !directResult.leftTensionEnd.yokeElementIds.empty()
        || !directResult.rightTensionEnd.yokeElementIds.empty()
        || directResult.leftTensionEnd.stabilizerElementId > 0
        || directResult.rightTensionEnd.stabilizerElementId > 0)
        return 24;
    for (const auto& [wireId, sub] : directResult.subConductors)
    {
        if (sub.nodeIds.empty()
            || sub.nodeIds.front() != directResult.leftTensionEnd.supportNodeIds[wireId]
            || sub.nodeIds.back() != directResult.rightTensionEnd.supportNodeIds[wireId])
            return 25;
        for (int elementId : sub.elementIds)
        {
            const auto element = directStructure->FindElement(elementId);
            if (!element || element->m_Role != ElementRole::Conductor || !element->HasAerodynamicLoad())
                return 26;
        }
    }

    QTemporaryDir temporaryDirectory;
    const QString h5Path = temporaryDirectory.filePath(QStringLiteral("conductor_tags.h5"));
    Hdf5ModelIO hdf5;
    StructureData restored;
    if (!temporaryDirectory.isValid()
        || !hdf5.ExportModelHdf5(h5Path, structure.get(), QStringLiteral("verification"))
        || !hdf5.ImportHdf5(h5Path, &restored)
        || restored.m_Elements.size() != structure->m_Elements.size())
        return 20;
    for (const auto& [elementId, source] : structure->m_Elements)
    {
        const auto target = restored.FindElement(elementId);
        if (!source || !target
            || target->m_Role != source->m_Role
            || target->m_WireId != source->m_WireId
            || target->m_AeroBundleCount != source->m_AeroBundleCount
            || target->m_AeroProfileId != source->m_AeroProfileId)
            return 21;
    }

    QTextStream(stdout)
        << "conductor bundle nodes=" << structure->m_Nodes.size()
        << " elements=" << structure->m_Elements.size()
        << " sets=" << structure->m_ModelSets.size()
        << " supports=" << result.leftSupportNodeId << "," << result.rightSupportNodeId
        << Qt::endl;
    return 0;
}

std::optional<int> verifyConductorMultiSpan(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-conductor-multispan")))
        return std::nullopt;

    auto structure = std::make_shared<StructureData>();
    Conductor::PropertyLibrary library;
    QString propertyError;
    if (!library.load(propertyError) || !library.isReady())
        return 1;
    auto property = library.instantiateProperty(0, 0, *structure, propertyError);
    if (!property)
        return 2;

    Conductor::LineBuildConfig line;
    line.property = property;
    line.elementType = EnumKeyword::ElementType::CR3D;
    line.conductor.nBundle = 4;
    line.conductor.spacing = 0.45;
    line.conductor.segments = 6;
    line.conductor.stress0 = 50.0e6;
    line.conductor.connecttype = Conductor::ConnectionMode::Parallel;
    line.convergeBundleEnds = true;
    line.bundleEndTransitionLength = 1.0;
    line.setNamePrefix = QStringLiteral("验证多档");

    Conductor::MultiSpanConductorBuildConfig config;
    config.span.line = line;
    config.stationCenters = {
        Vector3d(0.0, 0.0, 0.0),
        Vector3d(100.0, 0.0, 0.0),
        Vector3d(220.0, 0.0, 0.0)
    };
    config.suspensionStringLength = 1.0;
    config.suspensionElementType = EnumKeyword::ElementType::T3D2;
    config.suspensionProperty = property;
    config.span.useInnerSpacerLayout = true;
    config.span.innerSpacerLayout.useEqualSpacing = true;
    config.span.innerSpacerLayout.count = 1;
    config.span.innerSpacerLayout.spacer.elementType = EnumKeyword::ElementType::CR3D;
    config.span.innerSpacerLayout.spacer.property = property;
    config.span.innerSpacerLayout.spacer.style = Conductor::InnerSpacerStyle::OuterPolygon;

    Conductor::ConductorModelBuilder builder(structure);
    Conductor::LineBuildResult result;
    std::string buildError;
    if (!builder.BuildMultiSpanConductor(config, result, buildError))
    {
        QTextStream(stderr) << "multispan conductor build failed: "
                            << QString::fromStdString(buildError) << Qt::endl;
        return 3;
    }
    if (result.spanCount != 2 ||
        result.subConductors.size() != 4 ||
        result.suspensionPoints.size() != 1 ||
        result.leftTensionEnd.groupNodeIds.size() != 2 ||
        result.rightTensionEnd.groupNodeIds.size() != 2)
        return 4;

    const auto& suspension = result.suspensionPoints.front();
    if (suspension.wireNodeIds.size() != 4 ||
        suspension.yokeElementIds.size() != 2 ||
        suspension.spacerElementIds.size() != 4 ||
        suspension.stringElementId <= 0 ||
        suspension.supportNodeId <= 0 ||
        suspension.junctionNodeId <= 0)
        return 5;

    const auto junctionIt = structure->m_Nodes.find(suspension.junctionNodeId);
    const auto supportIt = structure->m_Nodes.find(suspension.supportNodeId);
    if (junctionIt == structure->m_Nodes.end() ||
        supportIt == structure->m_Nodes.end() ||
        std::abs(junctionIt->second->m_X - 100.0) > 1.0e-9 ||
        std::abs(junctionIt->second->m_Z - 0.45) > 1.0e-9 ||
        std::abs(supportIt->second->m_Z - 1.45) > 1.0e-9)
        return 6;

    for (int wireId = 0; wireId < 4; ++wireId)
    {
        const auto& nodes = result.subConductors.at(wireId).nodeIds;
        if (std::count(nodes.begin(), nodes.end(), suspension.wireNodeIds[wireId]) != 1)
            return 7;
    }
    for (int elementId : suspension.yokeElementIds)
    {
        const auto found = structure->m_Elements.find(elementId);
        if (found == structure->m_Elements.end() ||
            found->second->m_Role != ElementRole::SuspensionHardware ||
            !std::dynamic_pointer_cast<ElementTruss>(found->second))
            return 8;
    }
    for (int elementId : suspension.spacerElementIds)
    {
        const auto found = structure->m_Elements.find(elementId);
        if (found == structure->m_Elements.end() ||
            found->second->m_Role != ElementRole::IntraPhaseSpacer ||
            found->second->HasAerodynamicLoad())
            return 8;
    }
    const auto stringIt = structure->m_Elements.find(suspension.stringElementId);
    if (stringIt == structure->m_Elements.end() ||
        stringIt->second->m_Role != ElementRole::SuspensionHardware ||
        stringIt->second->HasAerodynamicLoad())
        return 9;

    // 多档也必须支持简化直接端部：只有首末站端部不同，中间 H 型悬垂串、
    // 线身与相内间隔棒均保持同一套逻辑。
    auto directStructure = std::make_shared<StructureData>();
    auto directProperty = library.instantiateProperty(0, 0, *directStructure, propertyError);
    if (!directProperty)
        return 14;
    auto directConfig = config;
    directConfig.span.line.property = directProperty;
    directConfig.span.line.endTopology = Conductor::BundleEndTopology::DirectWireSupports;
    directConfig.suspensionProperty = directProperty;
    directConfig.span.innerSpacerLayout.spacer.property = directProperty;
    Conductor::ConductorModelBuilder directBuilder(directStructure);
    Conductor::LineBuildResult directResult;
    if (!directBuilder.BuildMultiSpanConductor(directConfig, directResult, buildError))
        return 15;
    if (directResult.suspensionPoints.size() != 1 ||
        directResult.leftTensionEnd.supportNodeIds.size() != 4 ||
        directResult.rightTensionEnd.supportNodeIds.size() != 4 ||
        !directResult.leftTensionEnd.groupNodeIds.empty() ||
        !directResult.rightTensionEnd.groupNodeIds.empty() ||
        !directResult.leftTensionEnd.yokeElementIds.empty() ||
        !directResult.rightTensionEnd.yokeElementIds.empty())
        return 16;
    for (int wireId = 0; wireId < 4; ++wireId)
    {
        const auto& nodes = directResult.subConductors.at(wireId).nodeIds;
        if (nodes.empty() ||
            nodes.front() != directResult.leftTensionEnd.supportNodeIds[wireId] ||
            nodes.back() != directResult.rightTensionEnd.supportNodeIds[wireId])
            return 17;
    }

    QTemporaryDir temporaryDirectory;
    const QString h5Path = temporaryDirectory.filePath(QStringLiteral("conductor_multispan.h5"));
    Hdf5ModelIO hdf5;
    StructureData restored;
    if (!temporaryDirectory.isValid() ||
        !hdf5.ExportModelHdf5(h5Path, structure.get(), QStringLiteral("multispan verification")) ||
        !hdf5.ImportHdf5(h5Path, &restored))
        return 10;
    const auto restoredString = restored.FindElement(suspension.stringElementId);
    if (!restoredString ||
        restoredString->m_Role != ElementRole::SuspensionHardware ||
        restoredString->HasAerodynamicLoad())
        return 11;

    QTemporaryDir uiOutputDirectory;
    ConductorModule module;
    module.setPropertyLibrary(&library);
    auto* modeCombo = module.findChild<QComboBox*>(QStringLiteral("modelModeCombo"));
    if (!uiOutputDirectory.isValid() || !modeCombo)
        return 12;
    modeCombo->setCurrentIndex(1);
    const auto uiBuild = module.buildModel(library, uiOutputDirectory.path());
    if (!uiBuild.succeeded() ||
        uiBuild.structure->m_Constraint.size() != 36)
    {
        QTextStream(stderr) << "multispan conductor UI build failed: "
                            << uiBuild.error
                            << " constraints="
                            << (uiBuild.structure ? uiBuild.structure->m_Constraint.size() : 0)
                            << Qt::endl;
        return 13;
    }

    QTextStream(stdout)
        << "conductor multispan spans=" << result.spanCount
        << " nodes=" << result.NodeCount()
        << " elements=" << result.ElementCount()
        << " suspension_center_z=" << suspension.center.z()
        << " junction_z=" << junctionIt->second->m_Z
        << " support_z=" << supportIt->second->m_Z
        << Qt::endl;
    return 0;
}

std::optional<int> verifyConductorStatic(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-conductor-static")))
        return std::nullopt;

    Conductor::PropertyLibrary library;
    QString libraryError;
    if (!library.load(libraryError) || !library.isReady())
    {
        QTextStream(stderr) << "conductor property library failed: "
                            << libraryError << Qt::endl;
        return 1;
    }

    const EnumKeyword::ElementType elementTypes[] = {
        EnumKeyword::ElementType::T3D2,
        EnumKeyword::ElementType::CABLE,
        EnumKeyword::ElementType::CR3D
    };
    for (EnumKeyword::ElementType elementType : elementTypes)
    {
        QTemporaryDir outputDirectory;
        ConductorModule module;
        module.setPropertyLibrary(&library);
        const int elementIndex = module.elementCombo()->findData(
            static_cast<int>(elementType));
        const int bundleIndex = module.bundleCombo()->findData(4);
        const int beamSpacerIndex = module.spacerElementCombo()->findData(
            static_cast<int>(EnumKeyword::ElementType::CR3D));
        if (!outputDirectory.isValid() || elementIndex < 0 ||
            bundleIndex < 0 || beamSpacerIndex < 0)
        {
            return 2;
        }

        module.elementCombo()->setCurrentIndex(elementIndex);
        module.bundleCombo()->setCurrentIndex(bundleIndex);
        // Regression case for the default UI combination that previously
        // exposed a singular static tangent: truss conductors with four
        // beam-type spacers and the full 50 subdivisions.
        module.segmentsSpin()->setValue(
            elementType == EnumKeyword::ElementType::T3D2 ? 50 : 10);
        module.innerSpacerCheck()->setChecked(true);
        module.spacerCountSpin()->setValue(
            elementType == EnumKeyword::ElementType::T3D2 ? 4 : 2);
        module.spacerElementCombo()->setCurrentIndex(beamSpacerIndex);
        module.analysisCheck()->setChecked(true);

        const auto build = module.buildModel(library, outputDirectory.path());
        if (!build.succeeded() || build.structure->m_AnalysisStep.empty())
        {
            QTextStream(stderr)
                << "conductor static build failed type="
                << static_cast<int>(elementType)
                << " error=" << build.error << Qt::endl;
            return 3;
        }

        int spacerElementCount = 0;
        int beamSpacerElementCount = 0;
        for (const auto& [elementId, element] :
             build.structure->m_Elements)
        {
            Q_UNUSED(elementId);
            if (!element
                || element->m_Role != ElementRole::IntraPhaseSpacer)
                continue;
            ++spacerElementCount;
            if (std::dynamic_pointer_cast<ElementBeam_CR>(element))
                ++beamSpacerElementCount;
        }
        if (spacerElementCount == 0
            || beamSpacerElementCount != spacerElementCount
            || !build.structure->m_MPCConstraints.empty())
        {
            QTextStream(stderr)
                << "conductor mixed topology invalid type="
                << static_cast<int>(elementType)
                << " spacers=" << spacerElementCount
                << " beam_spacers=" << beamSpacerElementCount
                << " mpcs=" << build.structure->m_MPCConstraints.size()
                << Qt::endl;
            return 5;
        }

        std::shared_ptr<StructureData> solveStructure = build.structure;
        const auto step =
            solveStructure->m_AnalysisStep.begin()->second;
        step->SetStructure(solveStructure);
        if (!step->Solve(false))
        {
            QTextStream(stderr)
                << "conductor static solve failed type="
                << static_cast<int>(elementType)
                << " nodes=" << solveStructure->m_Nodes.size()
                << " elements=" << solveStructure->m_Elements.size()
                << " constraints=" << solveStructure->m_Constraint.size()
                << Qt::endl;
            return 4;
        }
        QTextStream(stdout)
            << "conductor static solved type="
            << static_cast<int>(elementType)
            << " nodes=" << solveStructure->m_Nodes.size()
            << " elements=" << solveStructure->m_Elements.size()
            << " constraints=" << solveStructure->m_Constraint.size()
            << " mpcs=" << solveStructure->m_MPCConstraints.size()
            << Qt::endl;
    }
    return 0;
}

std::optional<int> verifyConductorGallopingDynamics(
    const QStringList& arguments)
{
    const int exportIndex = arguments.indexOf(QStringLiteral("--export-conductor-galloping"));
    const bool verifyRequested = arguments.contains(QStringLiteral("--verify-conductor-galloping"));
    if (!verifyRequested && exportIndex < 0)
        return std::nullopt;
    if (exportIndex >= 0 && exportIndex + 1 >= arguments.size())
        return 1;

    const QString exportPath = exportIndex >= 0 ? arguments.at(exportIndex + 1) : QString();

    // A dependent dynamic branch inherits its static source and its own
    // definitions, but never an earlier sibling dynamic merely because that
    // sibling has a smaller ID.
    AnalysisStep branchScopeProbe;
    branchScopeProbe.m_Id = 3;
    branchScopeProbe.m_Type = EnumKeyword::StepType::DYNAMIC;
    branchScopeProbe.m_InitialStaticStepId = 1;
    if (!branchScopeProbe.IsStepScopedDataActive(0)
        || !branchScopeProbe.IsStepScopedDataActive(1)
        || branchScopeProbe.IsStepScopedDataActive(2)
        || !branchScopeProbe.IsStepScopedDataActive(3)
        || branchScopeProbe.IsStepScopedDataActive(4))
    {
        QTextStream(stderr) << "dynamic branch step scope isolation failed" << Qt::endl;
        return 2;
    }

    Conductor::PropertyLibrary library;
    QString libraryError;
    if (!library.load(libraryError) || !library.isReady())
    {
        QTextStream(stderr) << "conductor property library failed: "
                            << libraryError << Qt::endl;
        return 1;
    }

    const auto solve = [&library, &exportPath](SolverNameSpace::SolverType solverType,
                                                double& maximumDisplacement,
                                                double& inheritanceGap)
    {
        QTemporaryDir outputDirectory;
        ConductorModule module;
        module.setPropertyLibrary(&library);
        const int cableIndex = module.elementCombo()->findData(
            static_cast<int>(EnumKeyword::ElementType::CABLE));
        const int bundleIndex = module.bundleCombo()->findData(4);
        if (!outputDirectory.isValid() || cableIndex < 0 || bundleIndex < 0)
            return false;

        module.elementCombo()->setCurrentIndex(cableIndex);
        module.bundleCombo()->setCurrentIndex(bundleIndex);
        module.segmentsSpin()->setValue(10);
        module.analysisCheck()->setChecked(true);
        const auto build = module.buildModel(library, outputDirectory.path());
        if (!build.succeeded() || build.structure->m_AnalysisStep.empty())
            return false;

        const auto structure = build.structure;
        const int staticStepId = structure->m_AnalysisStep.begin()->first;
        structure->m_AnalysisStep.at(staticStepId)->m_StepSize = 0.05;
        AnalysisStepConfig dynamicConfig;
        dynamicConfig.id = structure->m_AnalysisStep.rbegin()->first + 1;
        dynamicConfig.name = QStringLiteral("verification dynamics");
        dynamicConfig.type = EnumKeyword::StepType::DYNAMIC;
        dynamicConfig.totalTime = 0.02;
        dynamicConfig.stepSize = 0.005;
        dynamicConfig.tolerance = 1.0e-5;
        dynamicConfig.maxIterations = 48;
        dynamicConfig.dynamicSolverType = solverType;
        dynamicConfig.initialStaticStepId = staticStepId;
        dynamicConfig.enableGalloping = true;
        dynamicConfig.gallopingIceThickness = 25;
        dynamicConfig.gallopingInitialAttackDegrees = 45.0;
        dynamicConfig.adaptiveTssbn.minimumTimeStep = 1.0e-5;
        dynamicConfig.adaptiveTssbn.maximumTimeStep = 0.005;
        dynamicConfig.adaptiveTssbn.relativeTolerance = 1.0e-4;
        dynamicConfig.adaptiveTssbn.absoluteTolerance = 1.0e-6;
        dynamicConfig.adaptiveTssbn.targetNewtonIterations = 16;
        structure->AddAnalysisStep(dynamicConfig);

        auto wind = std::make_shared<Force_Wind>();
        wind->m_Id = structure->m_Load.empty()
            ? 1 : structure->m_Load.rbegin()->first + 1;
        wind->m_Name = QStringLiteral("verification galloping wind");
        wind->m_StepId = dynamicConfig.id;
        wind->m_velocity = 14.0;
        wind->m_direction = Eigen::Vector3d::UnitY();
        wind->m_windDensity = 1.225;
        structure->m_Load.emplace(wind->m_Id, wind);

        // BDF does not have fields for conductor aerodynamic tags or adaptive
        // TSSBN settings.  Export the configured model in HDF5 before solving
        // so it can be re-opened without losing the galloping definition.
        if (!exportPath.isEmpty() && solverType == SolverNameSpace::SolverType::AdaptiveTSSBN)
        {
            Hdf5ModelIO hdf5;
            if (!hdf5.ExportModelHdf5(exportPath, structure.get(),
                                      QStringLiteral("ice galloping conductor - adaptive TSSBN")))
                return false;
        }

        QString dynamicResultPath = outputDirectory.filePath(
            QStringLiteral("verification_chain.h5"));
        if (!exportPath.isEmpty()
            && solverType == SolverNameSpace::SolverType::AdaptiveTSSBN)
        {
            const QFileInfo modelFile(exportPath);
            dynamicResultPath = modelFile.dir().filePath(
                modelFile.completeBaseName() + QStringLiteral("_result.h5"));
        }
        structure->m_OutputControl.m_Hdf5FileName = dynamicResultPath;
        structure->m_OutputControl.m_StreamResult = false;

        // Solve an independent reference copy of the equilibrium step.  The
        // actual dynamic task below must reproduce this accepted state in its
        // t=0 frame without publishing the static frames into the dynamic H5.
        QString referenceCloneError;
        auto equilibriumReference = structure->CloneForAnalysis(&referenceCloneError);
        if (!equilibriumReference)
            return false;
        equilibriumReference->m_OutputControl.m_Hdf5FileName = outputDirectory.filePath(
            QStringLiteral("verification_static_reference.h5"));
        AnalysisRunner equilibriumRunner;
        equilibriumRunner.SetStructure(equilibriumReference);
        equilibriumRunner.SetRuntimeCallbacks({}, []() { return false; });
        if (!equilibriumRunner.RunStep(staticStepId))
            return false;
        const DataFrame* finalStaticFrame = nullptr;
        for (const DataFrame& frame : equilibriumReference->GetOutputter().GetFrames())
        {
            if (frame.GetStepId() == staticStepId
                && (!finalStaticFrame || frame.GetTime() > finalStaticFrame->GetTime()))
            {
                finalStaticFrame = &frame;
            }
        }
        if (!finalStaticFrame)
            return false;

        AnalysisRunner runner;
        runner.SetStructure(structure);
        runner.SetRuntimeCallbacks({}, []() { return false; });
        if (!runner.RunStep(staticStepId))
        {
            QTextStream(stderr) << "conductor standalone equilibrium task failed"
                << " solver=" << static_cast<int>(solverType) << Qt::endl;
            return false;
        }
        QString inheritedCloneError;
        auto dynamicStructure = structure->CloneForAnalysis(&inheritedCloneError);
        if (!dynamicStructure)
            return false;
        dynamicStructure->GetOutputter().Clear();
        dynamicStructure->m_OutputControl.m_Hdf5FileName = dynamicResultPath;
        AnalysisRunner dynamicRunner;
        dynamicRunner.SetStructure(dynamicStructure);
        dynamicRunner.SetRuntimeCallbacks({}, []() { return false; });
        if (!dynamicRunner.RunStepFromCurrentState(dynamicConfig.id))
        {
            QTextStream(stderr) << "conductor dynamic task failed after equilibrium inheritance"
                << " solver=" << static_cast<int>(solverType)
                << " regions=" << dynamicStructure->m_ComputeRegions.size()
                << " nodes=" << dynamicStructure->m_Nodes.size()
                << " elements=" << dynamicStructure->m_Elements.size() << Qt::endl;
            return false;
        }
        maximumDisplacement = 0.0;
        inheritanceGap = std::numeric_limits<double>::infinity();
        const DataFrame* initialDynamicFrame = nullptr;
        bool publishedStaticFrame = false;
        for (const DataFrame& frame : dynamicStructure->GetOutputter().GetFrames())
        {
            publishedStaticFrame = publishedStaticFrame
                || frame.GetStepId() == staticStepId;
            if (frame.GetStepId() != dynamicConfig.id)
                continue;
            if (!initialDynamicFrame || frame.GetTime() < initialDynamicFrame->GetTime())
                initialDynamicFrame = &frame;
            for (const auto& [nodeId, nodeData] : frame.GetNodeDatas())
            {
                Q_UNUSED(nodeData);
                const double u1 = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U1);
                const double u2 = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U2);
                const double u3 = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U3);
                maximumDisplacement = std::max(
                    maximumDisplacement, std::sqrt(u1 * u1 + u2 * u2 + u3 * u3));
            }
        }
        if (!publishedStaticFrame && initialDynamicFrame)
        {
            inheritanceGap = 0.0;
            for (const auto& [nodeId, nodeData] : finalStaticFrame->GetNodeDatas())
            {
                Q_UNUSED(nodeData);
                if (initialDynamicFrame->GetNodeDatas().find(nodeId)
                    == initialDynamicFrame->GetNodeDatas().cend())
                    continue;
                for (const auto component : {EnumKeyword::NodeResultType::U1,
                                             EnumKeyword::NodeResultType::U2,
                                             EnumKeyword::NodeResultType::U3})
                {
                    inheritanceGap = std::max(inheritanceGap, std::abs(
                        finalStaticFrame->GetNodeData(nodeId, component)
                        - initialDynamicFrame->GetNodeData(nodeId, component)));
                }
            }
        }
        return std::isfinite(maximumDisplacement)
            && std::isfinite(inheritanceGap) && inheritanceGap <= 1.0e-10;
    };

    double newmarkMaximumDisplacement = 0.0;
    double tssbnMaximumDisplacement = 0.0;
    double newmarkInheritanceGap = 0.0;
    double tssbnInheritanceGap = 0.0;
    const bool newmarkSolved = solve(
        SolverNameSpace::SolverType::Newmark, newmarkMaximumDisplacement,
        newmarkInheritanceGap);
    const bool tssbnSolved = solve(
        SolverNameSpace::SolverType::AdaptiveTSSBN,
        tssbnMaximumDisplacement, tssbnInheritanceGap);
    const double relativeDifference =
        std::abs(tssbnMaximumDisplacement - newmarkMaximumDisplacement)
        / std::max(1.0e-12, newmarkMaximumDisplacement);
    QTextStream(stdout)
        << "conductor galloping newmark_solved=" << newmarkSolved
        << " tssbn_solved=" << tssbnSolved
        << " newmark_max_displacement=" << newmarkMaximumDisplacement
        << " tssbn_max_displacement=" << tssbnMaximumDisplacement
        << " newmark_inheritance_gap=" << newmarkInheritanceGap
        << " tssbn_inheritance_gap=" << tssbnInheritanceGap
        << " relative_difference=" << relativeDifference << Qt::endl;

    return newmarkSolved && tssbnSolved
        && newmarkMaximumDisplacement > 0.0
        && tssbnMaximumDisplacement > 0.0
        && relativeDifference <= 0.05 ? 0 : 2;
}

std::optional<int> verifySolveTask(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-solve-task"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    const QString filePath = arguments.at(index + 1);
    ModelController modelController;
    SolveTaskController taskController;
    int taskId = -1;
    QObject::connect(
        &modelController, &ModelController::loadSucceeded,
        [&modelController, &taskController, &taskId, &application](int modelId, const QString& loadedFile, qint64)
        {
            taskId = taskController.prepare(modelController.model(modelId), loadedFile, 0);
            if (taskId < 0 || !taskController.start(taskId))
                application.exit(2);
        });
    QObject::connect(&modelController, &ModelController::loadFailed,
                     [&application](const QString&, const QString&) { application.exit(3); });
    QObject::connect(&taskController, &SolveTaskController::taskUpdated,
                     [&taskController, &taskId, &application](int updatedTaskId)
                     {
                         if (updatedTaskId != taskId)
                             return;
                         const auto info = taskController.taskInfo(updatedTaskId);
                         if (info.status != SolveTaskController::Status::Completed &&
                             info.status != SolveTaskController::Status::Failed &&
                             info.status != SolveTaskController::Status::Cancelled)
                         {
                             return;
                         }
                         QTextStream(stdout)
                             << "solve status=" << SolveTaskController::statusText(info.status)
                             << " elapsed_ms=" << info.elapsedMs << " message=" << info.message << Qt::endl;
                         application.exit(info.status == SolveTaskController::Status::Completed ? 0 : 4);
                     });
    QTimer::singleShot(120000, &application, [&application]() { application.exit(5); });
    if (!modelController.loadModel(filePath))
        return 1;
    return application.exec();
}

std::optional<int> verifyStaticDynamicTaskChain(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-static-dynamic-task-chain"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    const QString filePath = arguments.at(index + 1);
    auto model = std::make_shared<StructureData>();
    Hdf5ModelIO hdf5;
    if (!hdf5.ImportHdf5(filePath, model.get()))
        return 2;

    int staticStepId = 0;
    int dynamicStepId = 0;
    for (const auto& [stepId, step] : model->m_AnalysisStep)
    {
        if (!step)
            continue;
        if (step->m_Type == EnumKeyword::StepType::STATIC)
            staticStepId = stepId;
        if (step->m_Type == EnumKeyword::StepType::DYNAMIC && step->m_InitialStaticStepId > 0)
        {
            dynamicStepId = stepId;
            staticStepId = step->m_InitialStaticStepId;
            break;
        }
    }
    if (staticStepId <= 0 || dynamicStepId <= 0)
        return 3;

    SolveTaskController taskController;
    const int staticTaskId = taskController.prepare(model, filePath, staticStepId);
    const int dynamicTaskId = taskController.prepare(model, filePath, dynamicStepId);
    if (staticTaskId < 0 || dynamicTaskId < 0)
        return 4;

    bool staticCompleted = false;
    bool dynamicStartedBeforeStatic = false;
    QObject::connect(&taskController, &SolveTaskController::taskUpdated,
        [&taskController, staticTaskId, dynamicTaskId, &staticCompleted,
            &dynamicStartedBeforeStatic, &application](int taskId)
        {
            const auto info = taskController.taskInfo(taskId);
            if (taskId == staticTaskId && info.status == SolveTaskController::Status::Completed)
                staticCompleted = true;
            if (taskId == dynamicTaskId && info.status == SolveTaskController::Status::Running
                && !staticCompleted)
                dynamicStartedBeforeStatic = true;
            if (taskId != dynamicTaskId || (info.status != SolveTaskController::Status::Completed
                && info.status != SolveTaskController::Status::Failed
                && info.status != SolveTaskController::Status::Cancelled))
                return;

            const auto staticInfo = taskController.taskInfo(staticTaskId);
            const bool passed = info.status == SolveTaskController::Status::Completed
                && staticInfo.status == SolveTaskController::Status::Completed
                && !dynamicStartedBeforeStatic;
            QTextStream(stdout) << "task chain static="
                                << SolveTaskController::statusText(staticInfo.status)
                                << " dynamic=" << SolveTaskController::statusText(info.status)
                                << " dynamic_started_before_static="
                                << (dynamicStartedBeforeStatic ? "true" : "false") << Qt::endl;
            application.exit(passed ? 0 : 5);
        });
    QTimer::singleShot(120000, &application, [&application]() { application.exit(6); });
    if (!taskController.start(dynamicTaskId))
        return 7;
    return application.exec();
}

std::optional<int> verifyPartialResult(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-partial-result"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    const QString filePath = arguments.at(index + 1);
    ModelController modelController;
    SolveTaskController taskController;
    int taskId = -1;
    bool cancellationRequested = false;
    QObject::connect(
        &modelController, &ModelController::loadSucceeded,
        [&modelController, &taskController, &taskId, &application](int modelId, const QString& loadedFile, qint64)
        {
            taskId = taskController.prepare(modelController.model(modelId), loadedFile, 0);
            if (taskId < 0 || !taskController.start(taskId))
                application.exit(2);
        });
    QObject::connect(&modelController, &ModelController::loadFailed,
                     [&application](const QString&, const QString&) { application.exit(3); });
    QObject::connect(
        &taskController, &SolveTaskController::taskUpdated,
        [&taskController, &taskId, &cancellationRequested, &application](int updatedTaskId)
        {
            if (updatedTaskId != taskId)
                return;
            const auto info = taskController.taskInfo(updatedTaskId);
            if (!cancellationRequested && info.status == SolveTaskController::Status::Running && info.progress >= 0.05)
            {
                cancellationRequested = taskController.cancel(updatedTaskId);
                return;
            }
            if (info.status != SolveTaskController::Status::Completed &&
                info.status != SolveTaskController::Status::Failed &&
                info.status != SolveTaskController::Status::Cancelled)
            {
                return;
            }
            QTextStream(stdout) << "partial status=" << SolveTaskController::statusText(info.status)
                                << " available=" << (info.hasUsableResult ? "true" : "false")
                                << " partial=" << (info.partialResult ? "true" : "false")
                                << " frames=" << info.resultFrameCount << " end_time=" << info.resultEndTime
                                << " file=" << info.outputFile << Qt::endl;
            application.exit(info.hasUsableResult && info.partialResult && info.resultFrameCount > 0 ? 0 : 4);
        });
    QTimer::singleShot(120000, &application, [&application]() { application.exit(5); });
    if (!modelController.loadModel(filePath))
        return 1;
    return application.exec();
}

std::optional<int> verifyModelImport(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-model-import"));
    if (index < 0)
        return std::nullopt;

    const QStringList files = arguments.mid(index + 1);
    ModelController controller;
    int loadedCount = 0;
    int failedCount = 0;
    int cloneFailedCount = 0;
    QObject::connect(
        &controller, &ModelController::loadSucceeded,
        [&controller, &loadedCount, &cloneFailedCount](int modelId, const QString& filePath, qint64 elapsedMs)
        {
            ++loadedCount;
            QString cloneError;
            const auto firstClone = controller.model(modelId)->CloneForAnalysis(&cloneError);
            const auto secondClone = firstClone ? firstClone->CloneForAnalysis(&cloneError) : nullptr;
            if (!secondClone)
            {
                ++cloneFailedCount;
                QTextStream(stderr) << "clone failed model=" << modelId << " error=" << cloneError << Qt::endl;
            }
            QTextStream(stdout) << "loaded model=" << modelId << " file=" << filePath << " elapsed_ms=" << elapsedMs
                                << Qt::endl;
        });
    QObject::connect(&controller, &ModelController::loadFailed,
                     [&failedCount](const QString& filePath, const QString& errorMessage)
                     {
                         ++failedCount;
                         QTextStream(stderr) << "failed file=" << filePath << " error=" << errorMessage << Qt::endl;
                     });
    QObject::connect(
        &controller, &ModelController::busyChanged,
        [&controller, &application, &loadedCount, &failedCount, &cloneFailedCount, expected = files.size()](bool busy)
        {
            if (busy)
                return;
            const QList<int> ids = controller.modelIds();
            bool isolated = ids.size() == loadedCount;
            if (ids.size() >= 2)
            {
                const auto first = controller.model(ids.at(0));
                const auto second = controller.model(ids.at(1));
                if (first && second)
                {
                    const auto firstNode = first->FindNode(1);
                    const auto secondNode = second->FindNode(1);
                    if (firstNode && secondNode)
                        isolated = isolated && firstNode.get() != secondNode.get();
                }
            }
            QTextStream(stdout) << "summary loaded=" << loadedCount << " failed=" << failedCount
                                << " clone_failed=" << cloneFailedCount << " documents=" << controller.modelCount()
                                << " isolated=" << (isolated ? "true" : "false") << Qt::endl;
            application.exit(loadedCount == expected && failedCount == 0 && cloneFailedCount == 0 && isolated ? 0 : 1);
        });
    QTimer::singleShot(30000, &application, [&application]() { application.exit(2); });
    if (files.isEmpty() || controller.loadModels(files) != files.size())
        return 1;
    return application.exec();
}
} // namespace

std::optional<int> VerificationRunner::runHeadless(
    const QStringList& arguments)
{
    if (const auto result = verifyAdaptiveTssbn(arguments))
        return result;
    if (const auto result = verifyLe2012Example1(arguments))
        return result;
    if (const auto result = verifyLe2012Example4(arguments))
        return result;
    if (const auto result = verifyCableTorsion(arguments))
        return result;
    if (const auto result = verifySpatialWindLoad(arguments))
        return result;
    if (const auto result = verifyExactAerodynamicAngle(arguments))
        return result;
    if (const auto result = verifyGallopingCaseSelection(arguments))
        return result;
    if (const auto result = verifyGallopingStability(arguments))
        return result;
    return verifyBeamDynamics(arguments);
}

std::optional<int> VerificationRunner::run(QApplication& application, const QStringList& arguments)
{
    if (const auto result = runHeadless(arguments))
        return result;
    if (const auto result = verifyNodeExport(arguments))
        return result;
    if (const auto result = verifyResultRead(arguments))
        return result;
    if (const auto result = verifyShearReleaseResult(arguments))
        return result;
    if (const auto result = verifyShearReleaseModel(arguments))
        return result;
    if (const auto result = verifyHdf5Model(arguments))
        return result;
    if (const auto result = verifyHdf5ModelContract(arguments))
        return result;
    if (const auto result = verifyComputeRegions(arguments))
        return result;
    if (const auto result = verifyMPCRegions(arguments))
        return result;
    if (const auto result = verifyConductorBundle(arguments))
        return result;
    if (const auto result = verifyConductorMultiSpan(arguments))
        return result;
    if (const auto result = verifyConductorStatic(arguments))
        return result;
    if (const auto result = verifyConductorGallopingDynamics(arguments))
        return result;
    if (const auto result = verifyStaticDynamicTaskChain(application, arguments))
        return result;
    if (const auto result = verifySolveTask(application, arguments))
        return result;
    if (const auto result = verifyPartialResult(application, arguments))
        return result;
    return verifyModelImport(application, arguments);
}
