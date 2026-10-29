#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Node/Node.h"
#include "Solver/Dynamic/Damping/StructuralDamping.h"
#include "Solver/Dynamic/Integrators/SolverAdaptiveTSSBN.h"
#include "Solver/Factory/SolverFactory.h"

#include <gtest/gtest.h>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QTemporaryDir>
#include <QTextStream>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using SolverNameSpace::SpMat;
using SolverNameSpace::Vec;

constexpr double benchmarkYoung = 1.0e7;
constexpr double benchmarkArea = 1.0;
constexpr double benchmarkLength = 10.0;
constexpr double benchmarkDensity = 317.0;
constexpr double benchmarkMass = benchmarkDensity * benchmarkArea * benchmarkLength / 2.0;
constexpr double benchmarkStiffness = benchmarkYoung * benchmarkArea / benchmarkLength;
constexpr double benchmarkDuration = 2.5;

struct DampingCase
{
    const char* name;
    double alpha;
    double beta;
};

// 解析参考仅依赖官方INP常数，不读取任一路径的生产阻尼矩阵或标定报告。
class IndependentDecayReference
{
public:
    explicit IndependentDecayReference(const DampingCase& input)
        : damping(input.alpha * benchmarkMass + input.beta * benchmarkStiffness)
        , omega(std::sqrt(benchmarkStiffness / benchmarkMass))
        , sigma(damping / (2.0 * benchmarkMass))
        , dampedOmega(std::sqrt(omega * omega - sigma * sigma))
    {
    }

    Eigen::Vector3d State(double time) const
    {
        const double sine = std::sin(dampedOmega * time);
        const double cosine = std::cos(dampedOmega * time);
        const double decay = std::exp(-sigma * time);
        return {decay * (cosine + sigma / dampedOmega * sine),
                -omega * omega / dampedOmega * decay * sine,
                omega * omega * decay * (sigma / dampedOmega * sine - cosine)};
    }

    double damping;
    double omega;
    double sigma;
    double dampedOmega;
};

class DampedBenchmarkModel final : public SolverNameSpace::IAnalysisModel
{
public:
    DampedBenchmarkModel(double damping, const IndependentDecayReference& reference, const QString& csvPath)
        : damping_(damping)
        , reference_(reference)
        , file_(csvPath)
        , stream_(&file_)
    {
        if (!file_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::NewOnly))
        {
            throw std::runtime_error("无法创建独立的阻尼基准CSV");
        }
        isDynamic = true;
        stream_.setLocale(QLocale::c());
        stream_.setRealNumberPrecision(17);
        stream_ << "time,u,v,a,exact_u,exact_v,exact_a,error_u,error_v,error_a,"
                   "normalized_error_u,normalized_error_v,normalized_error_a,energy,exact_energy\n";
        state_ << 1.0, 0.0, -benchmarkStiffness / benchmarkMass;
        BackupStepState();
        OnStepCompleted(0.0);
    }

    int GetFreeDofs() const override
    {
        return 1;
    }
    int GetFixedDofs() const override
    {
        return 0;
    }
    bool HasStateDependentMass() const override
    {
        return false;
    }
    bool ShouldPreferGpuLinearSolver() const override
    {
        return false;
    }
    std::uint64_t GetSparsePatternVersion() const override
    {
        return 1;
    }

    void ApplyIncrement(const Vec& increment) override
    {
        state_[0] += increment[0];
    }

    void BeginDynamicStep(double step, double beta, double gamma) override
    {
        BackupStepState();
        state_[0] = saved_[0] + step * saved_[1] + step * step * (0.5 - beta) * saved_[2];
        state_[1] = saved_[1] + step * (1.0 - gamma) * saved_[2];
        state_[2] = 0.0;
    }

    void ApplyDynamicCorrection(const Vec& increment, double accelerationDerivative, double velocityDerivative) override
    {
        state_[0] += increment[0];
        state_[1] += velocityDerivative * increment[0];
        state_[2] += accelerationDerivative * increment[0];
    }

    void RollbackDynamicStep() override
    {
        state_ = saved_;
    }
    bool SaveTrialState() override
    {
        trial_ = state_;
        return true;
    }
    bool RestoreTrialState() override
    {
        state_ = trial_;
        return true;
    }
    bool PushStateCheckpoint() override
    {
        checkpoints_.push_back({state_, saved_, trial_});
        return true;
    }
    bool RestoreStateCheckpoint() override
    {
        if (checkpoints_.empty())
        {
            return false;
        }
        state_ = checkpoints_.back()[0];
        saved_ = checkpoints_.back()[1];
        trial_ = checkpoints_.back()[2];
        return true;
    }
    void DiscardStateCheckpoint() override
    {
        checkpoints_.pop_back();
    }

    void SetTrialKinematics(const Vec& velocity, const Vec& acceleration) override
    {
        state_[1] = velocity[0];
        state_[2] = acceleration[0];
    }
    void GetState(_OUT Vec& displacement, _OUT Vec& velocity, _OUT Vec& acceleration) const override
    {
        displacement = Vec::Constant(1, state_[0]);
        velocity = Vec::Constant(1, state_[1]);
        acceleration = Vec::Constant(1, state_[2]);
    }

    void Assemble_Matrix(_OUT SpMat& stiffness, bool) override
    {
        stiffness.resize(1, 1);
        stiffness.setIdentity();
        stiffness *= benchmarkStiffness;
    }
    void AssembleDynamicSystem(_OUT SpMat& mass, _OUT SpMat& velocityTangent,
                               _OUT SpMat& configurationTangent) override
    {
        mass.resize(1, 1);
        mass.setIdentity();
        mass *= benchmarkMass;
        velocityTangent.resize(1, 1);
        velocityTangent.setIdentity();
        velocityTangent *= damping_;
        configurationTangent.resize(1, 1);
        configurationTangent.setZero();
    }
    // C 已计入速度切线。
    void EvaluateSystem(const SolverNameSpace::SystemEvaluationRequest& request,
                        _OUT SolverNameSpace::SystemEvaluationResult& result) override
    {
        IAnalysisModel::EvaluateSystem(request, result);
        if (request.Requires(SolverNameSpace::SystemContribution::Damping))
        {
            result.damping.resize(1, 1);
            result.damping.setIdentity();
            result.damping *= damping_;
        }
    }
    void GetDynamicInertiaForce(_OUT Vec& force) const override
    {
        force = Vec::Constant(1, benchmarkMass * state_[2]);
    }
    void ComputeExternalForce(double, double, _OUT Vec& constrained, _OUT Vec& free) override
    {
        constrained.resize(0);
        free = Vec::Zero(1);
    }
    void Assemble_Constraint(_OUT Vec& constrained, double, double) override
    {
        constrained.resize(0);
    }
    void ComputeResidual(const Vec& external, _OUT Vec& residual) override
    {
        residual = external - Vec::Constant(1, benchmarkStiffness * state_[0] +
                                             damping_ * state_[1] + benchmarkMass * state_[2]);
    }
    void ComputeStaticResidual(const Vec& external, _OUT Vec& residual) override
    {
        residual = external - Vec::Constant(1, benchmarkStiffness * state_[0]);
    }
    void CalculateReactions(_OUT Vec& reactions) override
    {
        reactions.resize(0);
    }
    void CommitState() override
    {
    }
    void BackupStepState() override
    {
        saved_ = state_;
    }
    void GetStepIncrement(_OUT Vec& increment) const override
    {
        increment = Vec::Constant(1, state_[0] - saved_[0]);
    }

    void OnStepCompleted(double time) override
    {
        if (!std::isfinite(time) || !state_.allFinite() || (!times.empty() && time <= times.back()))
        {
            throw std::runtime_error("阻尼基准时刻不严格递增或状态非有限");
        }
        const Eigen::Vector3d exact = reference_.State(time);
        const Eigen::Vector3d error = state_ - exact;
        const Eigen::Vector3d scales(1.0, reference_.omega, reference_.omega * reference_.omega);
        const Eigen::Vector3d normalized = error.cwiseQuotient(scales);
        maximumNormalized = maximumNormalized.cwiseMax(normalized.cwiseAbs());
        squaredNormalized += normalized.array().square().matrix();
        const double energy = 0.5 * benchmarkMass * state_[1] * state_[1] +
                              0.5 * benchmarkStiffness * state_[0] * state_[0];
        const double exactEnergy = 0.5 * benchmarkMass * exact[1] * exact[1] +
                                   0.5 * benchmarkStiffness * exact[0] * exact[0];
        stream_ << time;
        for (const double value : state_)
        {
            stream_ << ',' << value;
        }
        for (const double value : exact)
        {
            stream_ << ',' << value;
        }
        for (const double value : error)
        {
            stream_ << ',' << value;
        }
        for (const double value : normalized)
        {
            stream_ << ',' << value;
        }
        stream_ << ',' << energy << ',' << exactEnergy << '\n';
        times.push_back(time);
    }

    void Finish()
    {
        stream_.flush();
        if (stream_.status() != QTextStream::Ok || file_.error() != QFileDevice::NoError)
        {
            throw std::runtime_error("阻尼基准CSV写入失败");
        }
    }

    std::vector<double> times;
    Eigen::Vector3d maximumNormalized = Eigen::Vector3d::Zero();
    Eigen::Vector3d squaredNormalized = Eigen::Vector3d::Zero();

private:
    double damping_;
    IndependentDecayReference reference_;
    QFile file_;
    QTextStream stream_;
    Eigen::Vector3d state_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d saved_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d trial_ = Eigen::Vector3d::Zero();
    std::vector<std::array<Eigen::Vector3d, 3>> checkpoints_;
};

double BuildLocalDamping(const std::array<double, 4>& coefficients)
{
    auto first = std::make_shared<Node>();
    auto second = std::make_shared<Node>();
    first->SetNumDOFs(4);
    second->SetNumDOFs(4);
    second->m_X = benchmarkLength;
    ElementCable cable;
    cable.m_pNode[0] = first;
    cable.m_pNode[1] = second;
    cable.SetReferenceTssbnConstitutiveMode(true, false);
    Eigen::MatrixXd stiffness = Eigen::MatrixXd::Zero(8, 8);
    Eigen::MatrixXd mass = Eigen::MatrixXd::Zero(8, 8);
    stiffness(0, 0) = stiffness(4, 4) = benchmarkStiffness;
    stiffness(0, 4) = stiffness(4, 0) = -benchmarkStiffness;
    mass(0, 0) = mass(4, 4) = benchmarkMass;
    Eigen::MatrixXd damping;
    cable.GetRayleighDamping(coefficients, stiffness, mass, damping);
    if (!damping.allFinite() || damping.rows() != 8 || damping.cols() != 8)
    {
        throw std::runtime_error("生产局部阻尼构造器没有返回有限8×8矩阵");
    }
    return damping(4, 4);
}

double BuildCalibratedLocalDamping(const IndependentDecayReference& reference, _OUT QJsonObject& metadata)
{
    SpMat mass(1, 1), stiffness(1, 1), identity(1, 1), damping(1, 1);
    mass.setIdentity();
    mass *= benchmarkMass;
    stiffness.setIdentity();
    stiffness *= benchmarkStiffness;
    identity.setIdentity();
    SolverNameSpace::StructuralDampingModel model;
    model.settings.enabled = true;
    model.settings.translationDampingRatio = reference.sigma / reference.omega;
    std::array<SpMat, 4> localDampingBasis;
    for (int component = 0; component < 4; ++component)
    {
        std::array<double, 4> unitCoefficients{};
        unitCoefficients[component] = 1.0;
        localDampingBasis[component].resize(1, 1);
        localDampingBasis[component].insert(0, 0) = BuildLocalDamping(unitCoefficients);
    }
    QString error;
    if (!model.Calculate(stiffness, mass, localDampingBasis, {false}, identity, error))
        throw std::runtime_error(error.toStdString());
    const auto& report = model.report;
    damping.insert(0, 0) = BuildLocalDamping({report.translationMassCoefficient, report.translationStiffnessCoefficient,
                                            report.torsionMassCoefficient, report.torsionStiffnessCoefficient});
    if (!model.ValidateRayleighMatrix(mass, damping, identity, error))
    {
        throw std::runtime_error(error.toStdString());
    }
    metadata = {{"single_mode_mass_coefficient", model.report.translationMassCoefficient},
                {"single_mode_stiffness_coefficient", model.report.translationStiffnessCoefficient},
                {"reference_frequency_hz", model.report.translationReferenceFrequency1Hz},
                {"matrix_validation_passed", model.report.matrixValidationPassed},
                {"maximum_reference_projection_error", model.report.maximumReferenceProjectionError},
                {"projected_damping_ratio", model.report.modeSpectrum.at(0).projectedDampingRatio},
                {"scope", "单参考模态的局部四基自动标定，检验实际标量C，不要求唯一复原官方alpha/beta分解"}};
    return damping.coeff(0, 0);
}

QJsonObject ParametersJson(const SolverNameSpace::SolverAdaptiveTSSBN::Params& p)
{
    return {{"initialTimeStep", p.initialTimeStep}, {"nonlinearTolerance", p.nonlinearTolerance},
            {"maximumNewtonIterations", p.maximumNewtonIterations},
            {"aerodynamicTangentMode", static_cast<int>(p.aerodynamicTangentMode)},
            {"diagnosticCacheExternalForcePerStage", p.diagnosticCacheExternalForcePerStage},
            {"diagnosticErrorTrace", p.diagnosticErrorTrace}, {"referenceGalloping", p.referenceGalloping},
            {"spectralRadiusInfinity", p.spectralRadiusInfinity}, {"minimumTimeStep", p.minimumTimeStep},
            {"maximumTimeStep", p.maximumTimeStep}, {"relativeTolerance", p.relativeTolerance},
            {"absoluteTolerance", p.absoluteTolerance}, {"velocityAbsoluteTolerance", p.velocityAbsoluteTolerance},
            {"errorNormMode", static_cast<int>(p.errorNormMode)}, {"useDuan2026Controller", p.useDuan2026Controller},
            {"useReferenceExtrapolatedAcceleration", p.useReferenceExtrapolatedAcceleration},
            {"referenceIndependentErrorControl", p.referenceIndependentErrorControl},
            {"safetyFactor", p.safetyFactor}, {"shrinkFactor", p.shrinkFactor},
            {"maximumGrowthFactor", p.maximumGrowthFactor}, {"targetNewtonIterations", p.targetNewtonIterations},
            {"maximumTotalNewtonIterations", p.maximumTotalNewtonIterations}, {"derivativeGain", p.derivativeGain},
            {"minimumDerivativeFactor", p.minimumDerivativeFactor},
            {"maximumDerivativeFactor", p.maximumDerivativeFactor},
            {"maximumRejectedAttempts", p.maximumRejectedAttempts}};
}

void VerifyDampingInterface(DampedBenchmarkModel& model, double damping)
{
    SolverNameSpace::SystemEvaluationRequest request;
    request.Include(SolverNameSpace::SystemContribution::Damping);
    SolverNameSpace::SystemEvaluationResult result;
    model.EvaluateSystem(request, result);
    ASSERT_EQ(result.damping.rows(), 1);
    EXPECT_NEAR(result.damping.coeff(0, 0) / damping, 1.0, 1.0e-12);
    ASSERT_TRUE(model.PushStateCheckpoint());
    model.SetTrialKinematics(Vec::Constant(1, 0.25), Vec::Zero(1));
    Vec residual;
    model.ComputeResidual(Vec::Zero(1), residual);
    EXPECT_NEAR(residual[0], -benchmarkStiffness - 0.25 * damping, 1.0e-9);
    SpMat tangent;
    model.AssembleEffectiveTangent(2.0, 3.0, tangent);
    EXPECT_NEAR(tangent.coeff(0, 0), benchmarkStiffness + 2.0 * benchmarkMass + 3.0 * damping, 1.0e-8);
    ASSERT_TRUE(model.RestoreStateCheckpoint());
    model.DiscardStateCheckpoint();
    Vec displacement, velocity, acceleration;
    model.GetState(displacement, velocity, acceleration);
    EXPECT_DOUBLE_EQ(displacement[0], 1.0);
    EXPECT_DOUBLE_EQ(velocity[0], 0.0);
    EXPECT_DOUBLE_EQ(acceleration[0], -benchmarkStiffness / benchmarkMass);
}

void RunDampedCase(const DampingCase& input, const QString& directory, _OUT QJsonArray& runs,
                   _OUT QJsonArray& cases)
{
    SCOPED_TRACE(input.name);
    const IndependentDecayReference reference(input);
    const double explicitDamping = BuildLocalDamping({input.alpha, input.beta, 0.0, 0.0});
    QJsonObject calibrationMetadata;
    const double calibratedDamping = BuildCalibratedLocalDamping(reference, calibrationMetadata);
    ASSERT_TRUE(std::isfinite(explicitDamping));
    ASSERT_TRUE(std::isfinite(calibratedDamping));
    EXPECT_LT(std::abs(explicitDamping - reference.damping) / reference.damping, 1.0e-12);
    EXPECT_LT(std::abs(calibratedDamping - reference.damping) / reference.damping, 1.0e-12);
    cases.append(QJsonObject{{"case", input.name}, {"official_alpha", input.alpha}, {"official_beta", input.beta},
        {"M", benchmarkMass}, {"K", benchmarkStiffness}, {"independent_C", reference.damping},
        {"local_explicit_C", explicitDamping}, {"local_calibrated_C", calibratedDamping},
        {"independent_omega", reference.omega}, {"independent_damping_ratio", reference.sigma / reference.omega},
        {"calibration_metadata", calibrationMetadata}});
    const std::array<double, 4> caps{0.01, 0.005, 0.0025, 0.00125};
    for (const bool calibratedPath : {false, true})
    {
        const char* pathName = calibratedPath ? "local-calibrated" : "local-explicit";
        const double actualDamping = calibratedPath ? calibratedDamping : explicitDamping;
        SCOPED_TRACE(pathName);
        Eigen::Vector3d previousError = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
        for (int level = 0; level < caps.size(); ++level)
        {
            const double cap = caps[level];
            SCOPED_TRACE(cap);
            const QString name = QStringLiteral("%1_%2_h%3.csv").arg(input.name, pathName)
                                     .arg(cap, 0, 'g', 8);
            DampedBenchmarkModel model(actualDamping, reference, QDir(directory).filePath(name));
            VerifyDampingInterface(model, actualDamping);
            AnalysisStep configuration;
            configuration.m_Type = EnumKeyword::StepType::DYNAMIC;
            configuration.m_DynamicSolverType = SolverNameSpace::SolverType::ReferenceTSSBN;
            configuration.m_Time = benchmarkDuration;
            configuration.m_StepSize = cap;
            configuration.m_Tolerance = 1.0e-10;
            configuration.m_MaxIterations = 20;
            configuration.m_AdaptiveTssbn.spectralRadiusInfinity = 1.0;
            configuration.m_AdaptiveTssbn.minimumTimeStep = cap / 1024.0;
            configuration.m_AdaptiveTssbn.maximumTimeStep = cap;
            configuration.m_AdaptiveTssbn.relativeTolerance = 1.0;
            configuration.m_AdaptiveTssbn.absoluteTolerance = 1.0;
            configuration.m_AdaptiveTssbn.velocityAbsoluteTolerance = 1.0;
            configuration.m_AdaptiveTssbn.referenceIndependentErrorControl = false;
            auto solver = SolverNameSpace::SolverFactory::Create(configuration);
            ASSERT_TRUE(solver);
            ASSERT_EQ(solver->GetType(), SolverNameSpace::SolverType::ReferenceTSSBN);
            auto* referenceSolver = dynamic_cast<SolverNameSpace::SolverAdaptiveTSSBN*>(solver.get());
            ASSERT_NE(referenceSolver, nullptr);
            const auto& parameters = referenceSolver->GetParams();
            ASSERT_TRUE(parameters.referenceGalloping);
            ASSERT_TRUE(parameters.useDuan2026Controller);
            ASSERT_TRUE(parameters.useReferenceExtrapolatedAcceleration);
            ASSERT_FALSE(parameters.referenceIndependentErrorControl);
            ASSERT_EQ(parameters.errorNormMode, SolverNameSpace::TssbnErrorNormMode::LegacyGlobal);
            ASSERT_EQ(parameters.aerodynamicTangentMode, SolverNameSpace::AerodynamicTangentMode::Disabled);
            ASSERT_FALSE(parameters.diagnosticCacheExternalForcePerStage);
            ASSERT_DOUBLE_EQ(parameters.velocityAbsoluteTolerance, parameters.absoluteTolerance);
            ASSERT_DOUBLE_EQ(parameters.spectralRadiusInfinity, 1.0);
            QElapsedTimer timer;
            timer.start();
            const bool solved = solver->Solve(model, benchmarkDuration);
            const double wallSeconds = timer.nsecsElapsed() / 1.0e9;
            model.Finish();
            const double end = model.times.back();
            double minimumStep = std::numeric_limits<double>::infinity();
            double maximumStep = 0.0;
            for (std::size_t index = 1; index < model.times.size(); ++index)
            {
                const double step = model.times[index] - model.times[index - 1];
                minimumStep = std::min(minimumStep, step);
                maximumStep = std::max(maximumStep, step);
            }
            const Eigen::Vector3d rms = (model.squaredNormalized / model.times.size()).array().sqrt().matrix();
            QJsonObject row{{"case", input.name}, {"path", pathName}, {"csv", name},
                {"M", benchmarkMass}, {"K", benchmarkStiffness}, {"C", actualDamping},
                {"reference_damping_ratio", reference.sigma / reference.omega},
                {"assembled_modal_ratio", actualDamping / (2.0 * benchmarkMass * reference.omega)},
                {"solver_name", solver->GetName()}, {"solver_type", static_cast<int>(solver->GetType())},
                {"solver_params", ParametersJson(parameters)}, {"controller_enabled", true},
                {"requested_maximum_step", cap}, {"solved", solved}, {"actual_end_time", end},
                {"accepted_steps", referenceSolver->GetAcceptedStepCount()},
                {"rejected_attempts", referenceSolver->GetRejectedStepCount()},
                {"precision_limited_stages", static_cast<double>(referenceSolver->GetPrecisionLimitedStageCount())},
                {"precision_candidate_solves", static_cast<double>(referenceSolver->GetPrecisionCandidateSolveCount())},
                {"average_accepted_step_control_newton_count",
                 referenceSolver->GetAverageStageNewtonIterations()},
                {"maximum_accepted_step_control_newton_count",
                 referenceSolver->GetMaximumStageNewtonIterations()},
                {"iteration_count_definition", "每接受步max(1,C1.newtonIterations+C2.newtonIterations-2)"},
                {"minimum_accepted_step", std::isfinite(minimumStep) ? QJsonValue(minimumStep) : QJsonValue()},
                {"maximum_accepted_step", maximumStep},
                {"average_accepted_step", referenceSolver->GetAverageTimeStep()},
                {"wall_seconds", wallSeconds}, {"maximum_normalized_u_error", model.maximumNormalized[0]},
                {"maximum_normalized_v_error", model.maximumNormalized[1]},
                {"maximum_normalized_a_error", model.maximumNormalized[2]},
                {"rms_normalized_u_error", rms[0]}, {"rms_normalized_v_error", rms[1]},
                {"rms_normalized_a_error", rms[2]}, {"raw_frame_count", static_cast<int>(model.times.size())},
                {"finite_and_strictly_increasing_history", true},
                {"reached_duration", std::abs(end - benchmarkDuration) < 1.0e-10}};
            runs.append(row);
            EXPECT_TRUE(solved);
            EXPECT_NEAR(end, benchmarkDuration, 1.0e-10);
            EXPECT_EQ(model.times.size(), static_cast<std::size_t>(referenceSolver->GetAcceptedStepCount() + 1));
            EXPECT_LE(maximumStep, cap * (1.0 + 1.0e-8));
            EXPECT_TRUE(model.maximumNormalized.allFinite());
            if (level > 0 && solved)
            {
                for (int component = 0; component < 3; ++component)
                {
                    EXPECT_LT(model.maximumNormalized[component], 0.4 * previousError[component]);
                }
            }
            if (level == caps.size() - 1)
            {
                EXPECT_LT(model.maximumNormalized.maxCoeff(), 0.01);
            }
            previousError = model.maximumNormalized;
        }
    }
}

/** @brief Abaqus官方Rayleigh自由衰减SDOF的局部系数输入、自动标定及ReferenceTSSBN解析时程验证
 * @details 来源：Abaqus 2025 simabmk-c-rayleighdamping.htm及其direct_alpha/direct_beta/direct.inp。
 *          原始(in,lb,s)一致单位：E=1e7、A=1、L=10、rho=317，端点集中质量1585，k=1e6，u0=1、v0=0。
 *          三工况(alpha,beta)=(1.00472,0)、(0,1.59248e-3)、(1.00472,1.59248e-3)，独立解析c=alpha*m+beta*k。
 *          两入口均调用生产ElementCable的8×8线性杆K/M局部阻尼构造器；自动入口由实际单位系数基标定。
 *          只验证实际标量C，单参考模态不要求唯一复原原始alpha/beta分解。
 *          经生产SolverFactory创建ReferenceTSSBN；rho_inf=1，控制器启用，四档最大步长0.01/0.005/0.0025/0.00125。
 *          判据预设：两C相对误差<1e-12，达到2.5s且状态有限，细档归一化u/v/a最大误差<0.01；
 *          每次步长上限减半，各物理量最大误差须降至前档0.4倍以下。实际接受步长与误差逐帧输出。
 *          范围：官方离散SDOF、阻尼构造器及生产工厂积分器级验证，不是完整非线性索/舞动任务链，
 *          解析解不读取生产C，不据此推断多模态非线性导线的阻尼比在整个动力过程中恒定。
 * @author   yangqiuyou
 * @date     2026.10.26
 */
TEST(RayleighDecay, ReferenceTssbnAdaptive)
{
    QTemporaryDir temporary;
    const QString requested = qEnvironmentVariable("YQY_DAMPED_BENCHMARK_OUTPUT");
    const QString directory = requested.isEmpty() ? temporary.path() : QDir(requested).absolutePath();
    ASSERT_FALSE(directory.isEmpty());
    ASSERT_TRUE(QDir().mkpath(directory));
    QJsonArray runs, cases;
    for (const DampingCase& input : std::array<DampingCase, 3>{{
             {"mass", 1.00472, 0.0}, {"stiffness", 0.0, 1.59248e-3}, {"combined", 1.00472, 1.59248e-3}}})
    {
        ASSERT_NO_THROW(RunDampedCase(input, directory, runs, cases));
    }
    EXPECT_EQ(runs.size(), 24);
    QJsonObject summary{{"source", "https://docs.software.vt.edu/abaqusv2025/English/SIMACAEBMKRefMap/"
                                  "simabmk-c-rayleighdamping.htm"},
        {"input_files", QJsonArray{"rayleighdamping_direct_alpha.inp", "rayleighdamping_direct_beta.inp",
                                   "rayleighdamping_direct.inp"}},
        {"units", "原始一致英制(in,lb,s)，质量单位lb*s^2/in；未混用SI数值"},
        {"scope", "官方线性离散SDOF+生产阻尼构造器+生产工厂ReferenceTSSBN，不是完整AnalysisStep任务链"},
        {"reference", "独立原始alpha/beta,m,k闭式解，在各原始接受时刻评价；无插值、无相位对齐"},
        {"normalization", "u除初幅1，v除初幅*omega，a除初幅*omega^2；RMS为原始接受帧等权RMS"},
        {"duration", benchmarkDuration}, {"young", benchmarkYoung}, {"area", benchmarkArea},
        {"length", benchmarkLength}, {"density", benchmarkDensity}, {"cases", cases}, {"runs", runs},
        {"all_expectations_passed", !::testing::Test::HasFailure()}};
    QFile file(QDir(directory).filePath(QStringLiteral("summary.json")));
    ASSERT_TRUE(file.open(QIODevice::WriteOnly | QIODevice::NewOnly));
    const QByteArray bytes = QJsonDocument(summary).toJson(QJsonDocument::Indented);
    ASSERT_EQ(file.write(bytes), bytes.size());
    ASSERT_TRUE(file.flush());
    ::testing::Test::RecordProperty("output_directory", directory.toStdString());
}
/** @brief 四种自适应积分器仅收紧速度容差时的阻尼自由振动精度验证
 * @details 使用本文件相同的原始 Rayleigh 离散单自由度模型，alpha=1.00472、beta=0，初位移1、初速度0。
 *          参考为 IndependentDecayReference 的独立闭式解；仅验证该物理方程与生产积分器，不外推完整有限元任务链。
 *          终时刻0.5、初始步长0.02、最大步长0.05、最小步长1e-7、Newton容差1e-10。
 *          位移绝对容差固定1000，相对容差1e-10；速度绝对容差从0.1收紧到0.001。
 *          Reference采用逐自由度模式，以允许单独设置速度容差。旧速度关闭字段故意设false，验证其不再取消速度控制。
 *          判据：两档均完成至0.5(1e-12)，细档接受步数增加，最大归一化速度误差低于粗档0.8倍且低于0.01。
 * @author   yangqiuyou
 * @date     2026.10.28
 */
TEST(RayleighDecay, AdaptiveVelocityTolerance)
{
    QTemporaryDir temporary;
    ASSERT_TRUE(temporary.isValid());
    const DampingCase input{"mass", 1.00472, 0.0};
    const IndependentDecayReference reference(input);
    const std::array<SolverNameSpace::SolverType, 4> algorithms{
        SolverNameSpace::SolverType::AdaptiveNewmark, SolverNameSpace::SolverType::AdaptiveRungeKutta4,
        SolverNameSpace::SolverType::AdaptiveTSSBN, SolverNameSpace::SolverType::ReferenceTSSBN};
    for (int algorithm = 0; algorithm < static_cast<int>(algorithms.size()); ++algorithm)
    {
        SCOPED_TRACE(algorithm);
        std::array<double, 2> errors{};
        std::array<std::size_t, 2> counts{};
        for (int level = 0; level < 2; ++level)
        {
            SCOPED_TRACE(level);
            const double velocityTolerance = level == 0 ? 0.1 : 0.001;
            DampedBenchmarkModel model(
                reference.damping, reference,
                QDir(temporary.path()).filePath(QStringLiteral("velocity_%1_%2.csv").arg(algorithm).arg(level)));
            AnalysisStep configuration;
            configuration.m_Type = EnumKeyword::StepType::DYNAMIC;
            configuration.m_DynamicSolverType = algorithms[algorithm];
            configuration.m_StepSize = 0.02;
            configuration.m_Tolerance = 1.0e-10;
            configuration.m_MaxIterations = 20;
            auto& adaptive = configuration.m_AdaptiveTimeStep;
            adaptive.minimumTimeStep = 1.0e-7;
            adaptive.maximumTimeStep = 0.05;
            adaptive.absoluteTolerance = 1000.0;
            adaptive.relativeTolerance = 1.0e-10;
            adaptive.velocityAbsoluteTolerance = velocityTolerance;
            adaptive.velocityRelativeTolerance = 1.0e-10;
            adaptive.includeVelocityInError = false;
            auto& tssbn = configuration.m_AdaptiveTssbn;
            tssbn.minimumTimeStep = 1.0e-7;
            tssbn.maximumTimeStep = 0.05;
            tssbn.absoluteTolerance = 1000.0;
            tssbn.relativeTolerance = 1.0e-10;
            tssbn.velocityAbsoluteTolerance = velocityTolerance;
            tssbn.spectralRadiusInfinity = 1.0;
            tssbn.referenceIndependentErrorControl = true;
            auto solver = SolverNameSpace::SolverFactory::Create(configuration);
            ASSERT_TRUE(solver);
            ASSERT_TRUE(solver->Solve(model, 0.5));
            ASSERT_FALSE(model.times.empty());
            EXPECT_NEAR(model.times.back(), 0.5, 1.0e-12);
            model.Finish();
            errors[level] = model.maximumNormalized[1];
            counts[level] = model.times.size() - 1;
            const QString key = QStringLiteral("algorithm_%1_level_%2_").arg(algorithm).arg(level);
            ::testing::Test::RecordProperty((key + QStringLiteral("steps")).toStdString(),
                                            static_cast<int>(counts[level]));
            ::testing::Test::RecordProperty((key + QStringLiteral("velocity_error")).toStdString(),
                                            QString::number(errors[level], 'g', 17).toStdString());
        }
        EXPECT_GT(counts[1], counts[0]);
        EXPECT_LT(errors[1], 0.8 * errors[0]);
        EXPECT_LT(errors[1], 0.01);
    }
}

/** @brief 四种自适应积分器的三档位移精度与容差细化验证
 * @details 使用原 Rayleigh 离散单自由度模型，alpha=1.00472、beta=0，初位移1、初速度0，终时刻2.5。
 *          独立参考为本文件闭式解；仅验证该物理方程及生产工厂积分器，不外推完整空间有限元任务链。
 *          相对容差1e-4/1e-6/1e-8，位移绝对容差为其0.01倍，速度绝对容差为其1倍；
 *          Reference保留默认范数及速度绝对容差绑定策略。Newton容差1e-10，初始步长0.01，上限0.05。
 *          预设判据：各档完成且状态有限，位移最大误差逐档降至前档0.5倍以下，细档低于初幅0.1%。
 *          YQY_DISPLACEMENT_BENCHMARK_OUTPUT可指定全时程证据目录；能量不参与接受或拒绝时间步。
 * @author   yangqiuyou
 * @date     2026.10.28
 */
TEST(RayleighDecay, AdaptiveDisplacementTolerance)
{
    QTemporaryDir temporary;
    const QString requested = qEnvironmentVariable("YQY_DISPLACEMENT_BENCHMARK_OUTPUT");
    const QString directory = requested.isEmpty() ? temporary.path() : QDir(requested).absolutePath();
    ASSERT_FALSE(directory.isEmpty());
    ASSERT_TRUE(QDir().mkpath(directory));
    const DampingCase input{"mass", 1.00472, 0.0};
    const IndependentDecayReference reference(input);
    const std::array<SolverNameSpace::SolverType, 4> algorithms{
        SolverNameSpace::SolverType::AdaptiveNewmark, SolverNameSpace::SolverType::AdaptiveRungeKutta4,
        SolverNameSpace::SolverType::AdaptiveTSSBN, SolverNameSpace::SolverType::ReferenceTSSBN};
    const std::array<double, 3> tolerances{1.0e-4, 1.0e-6, 1.0e-8};
    for (int algorithm = 0; algorithm < static_cast<int>(algorithms.size()); ++algorithm)
    {
        SCOPED_TRACE(algorithm);
        double previousError = std::numeric_limits<double>::infinity();
        for (int level = 0; level < static_cast<int>(tolerances.size()); ++level)
        {
            SCOPED_TRACE(level);
            const double tolerance = tolerances[level];
            DampedBenchmarkModel model(
                reference.damping, reference,
                QDir(directory).filePath(QStringLiteral("displacement_%1_%2.csv").arg(algorithm).arg(level)));
            AnalysisStep configuration;
            configuration.m_Type = EnumKeyword::StepType::DYNAMIC;
            configuration.m_DynamicSolverType = algorithms[algorithm];
            configuration.m_StepSize = 0.01;
            configuration.m_Tolerance = 1.0e-10;
            configuration.m_MaxIterations = 48;
            auto& adaptive = configuration.m_AdaptiveTimeStep;
            adaptive.minimumTimeStep = 1.0e-7;
            adaptive.maximumTimeStep = 0.05;
            adaptive.absoluteTolerance = 0.01 * tolerance;
            adaptive.relativeTolerance = tolerance;
            adaptive.velocityAbsoluteTolerance = tolerance;
            adaptive.velocityRelativeTolerance = tolerance;
            auto& tssbn = configuration.m_AdaptiveTssbn;
            tssbn.minimumTimeStep = 1.0e-7;
            tssbn.maximumTimeStep = 0.05;
            tssbn.absoluteTolerance = 0.01 * tolerance;
            tssbn.relativeTolerance = tolerance;
            tssbn.velocityAbsoluteTolerance = tolerance;
            tssbn.spectralRadiusInfinity = 1.0;
            auto solver = SolverNameSpace::SolverFactory::Create(configuration);
            ASSERT_TRUE(solver);
            QElapsedTimer timer;
            timer.start();
            ASSERT_TRUE(solver->Solve(model, benchmarkDuration));
            const double seconds = timer.nsecsElapsed() / 1.0e9;
            model.Finish();
            ASSERT_FALSE(model.times.empty());
            EXPECT_NEAR(model.times.back(), benchmarkDuration, 1.0e-12);
            EXPECT_TRUE(model.maximumNormalized.allFinite());
            const double error = model.maximumNormalized[0];
            const QString key = QStringLiteral("algorithm_%1_level_%2_").arg(algorithm).arg(level);
            ::testing::Test::RecordProperty((key + QStringLiteral("steps")).toStdString(),
                                            static_cast<int>(model.times.size() - 1));
            ::testing::Test::RecordProperty((key + QStringLiteral("displacement_error")).toStdString(),
                                            QString::number(error, 'g', 17).toStdString());
            ::testing::Test::RecordProperty((key + QStringLiteral("wall_seconds")).toStdString(),
                                            QString::number(seconds, 'g', 17).toStdString());
            if (level > 0)
                EXPECT_LT(error, 0.5 * previousError);
            if (level == static_cast<int>(tolerances.size()) - 1)
                EXPECT_LT(error, 0.001);
            previousError = error;
        }
    }
    ::testing::Test::RecordProperty("output_directory", directory.toStdString());
}

}
