#include "Dynamics/Oscillators/OscillatorCases.h"

#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "Solver/Dynamic/Integrators/SolverNewmark.h"
#include "Solver/Factory/SolverFactory.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using SolverNameSpace::SpMat;
using SolverNameSpace::Vec;

constexpr double chopraMass = 0.2533;
constexpr double chopraStiffness = 10.0;
constexpr double chopraDampingRatio = 0.05;
constexpr double chopraYieldForce = 7.5;

struct LiteratureState
{
    double q = 0.0;
    double v = 0.0;
    double a = 0.0;
    double increment = 0.0;
    double plasticDisplacement = 0.0;
};

struct LiteratureCheckpoint
{
    LiteratureState state;
    LiteratureState step;
    LiteratureState trial;
    LiteratureState trialStep;
};

struct ReturnMappingResult
{
    double force = 0.0;
    double tangent = chopraStiffness;
    double plasticDisplacement = 0.0;
};

static double ChopraDamping()
{
    return 2.0 * chopraDampingRatio * std::sqrt(chopraStiffness * chopraMass);
}

static double ChopraLoad(double time)
{
    return time >= 0.0 && time <= 0.6 ? 10.0 * std::sin(2.0 * std::acos(-1.0) * time / 1.2) : 0.0;
}

static bool WriteLiteratureJson(const QString& path, const QJsonObject& object)
{
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate) &&
           file.write(QJsonDocument(object).toJson(QJsonDocument::Indented)) > 0;
}

// 这里只建立文献的一维物理模型；时间公式、Newton、线搜索和失败缩步均来自生产工厂。
class NewmarkLiteratureModel final : public SolverNameSpace::IAnalysisModel
{
public:
    NewmarkLiteratureModel(bool elastoplastic, double pendulumAmplitude, const QString& outputPath)
        : elastoplastic_(elastoplastic)
        , pendulumAmplitude_(pendulumAmplitude)
        , file_(outputPath)
        , stream_(&file_)
    {
        if (!file_.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
            throw std::runtime_error("Cannot write Newmark literature response");
        state_.q = pendulumAmplitude;
        state_.a = -std::sin(pendulumAmplitude);
        stream_.setRealNumberPrecision(17);
        stream_ << "time,q,v,a,accepted_dt,external_force,restoring_force,damping_force,inertia_force,"
                   "equilibrium_residual,committed_plastic_displacement,step_increment,newton_evaluations\n";
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
        state_.increment += increment[0];
        state_.q = step_.q + state_.increment;
    }
    void BeginDynamicStep(double timeStep, double beta, double gamma) override
    {
        BackupStepState();
        state_.increment = timeStep * step_.v + timeStep * timeStep * (0.5 - beta) * step_.a;
        state_.q = step_.q + state_.increment;
        state_.v = step_.v + timeStep * (1.0 - gamma) * step_.a;
        state_.a = 0.0;
    }
    void ApplyDynamicCorrection(const Vec& increment, double a0, double a1) override
    {
        ApplyIncrement(increment);
        state_.v += a1 * increment[0];
        state_.a += a0 * increment[0];
    }
    void RollbackDynamicStep() override
    {
        state_ = step_;
        state_.increment = 0.0;
    }
    bool SaveTrialState() override
    {
        trial_ = state_;
        trialStep_ = step_;
        return true;
    }
    bool RestoreTrialState() override
    {
        state_ = trial_;
        step_ = trialStep_;
        return true;
    }
    bool PushStateCheckpoint() override
    {
        checkpoints_.push_back({state_, step_, trial_, trialStep_});
        return true;
    }
    bool RestoreStateCheckpoint() override
    {
        if (checkpoints_.empty())
            return false;
        state_ = checkpoints_.back().state;
        step_ = checkpoints_.back().step;
        trial_ = checkpoints_.back().trial;
        trialStep_ = checkpoints_.back().trialStep;
        return true;
    }
    void DiscardStateCheckpoint() override
    {
        if (!checkpoints_.empty())
            checkpoints_.pop_back();
    }
    void SetTrialKinematics(const Vec& velocity, const Vec& acceleration) override
    {
        state_.v = velocity[0];
        state_.a = acceleration[0];
    }
    void GetState(_OUT Vec& displacement, _OUT Vec& velocity, _OUT Vec& acceleration) const override
    {
        displacement = Vec::Constant(1, state_.q);
        velocity = Vec::Constant(1, state_.v);
        acceleration = Vec::Constant(1, state_.a);
    }
    void Assemble_Matrix(_OUT SpMat& stiffness, bool) override
    {
        stiffness.resize(1, 1);
        stiffness.setZero();
        stiffness.insert(0, 0) = ReturnMapping().tangent;
        stiffness.makeCompressed();
    }
    void AssembleDynamicSystem(_OUT SpMat& mass, _OUT SpMat& velocity, _OUT SpMat& configuration) override
    {
        mass.resize(1, 1);
        mass.setZero();
        mass.insert(0, 0) = Mass();
        mass.makeCompressed();
        velocity.resize(1, 1);
        configuration.resize(1, 1);
        velocity.setZero();
        velocity.insert(0, 0) = Damping();
        configuration.setZero();
    }
    void EvaluateSystem(const SolverNameSpace::SystemEvaluationRequest& request,
                        _OUT SolverNameSpace::SystemEvaluationResult& result) override
    {
        IAnalysisModel::EvaluateSystem(request, result);
        if (request.Requires(SolverNameSpace::SystemContribution::Damping))
            result.damping.coeffRef(0, 0) = Damping();
    }
    void GetDynamicInertiaForce(_OUT Vec& force) const override
    {
        force = Vec::Constant(1, Mass() * state_.a);
    }
    void ComputeExternalForce(double time, double factor, _OUT Vec& constrained, _OUT Vec& force) override
    {
        constrained.resize(0);
        force = Vec::Constant(1, factor * Load(time));
    }
    void Assemble_Constraint(_OUT Vec& constrained, double, double) override
    {
        constrained.resize(0);
    }
    void ComputeStaticResidual(const Vec& force, _OUT Vec& residual) override
    {
        residual = force - Vec::Constant(1, ReturnMapping().force);
    }
    void ComputeResidual(const Vec& force, _OUT Vec& residual) override
    {
        residual = force - Vec::Constant(1, ReturnMapping().force + Damping() * state_.v + Mass() * state_.a);
    }
    void CalculateReactions(_OUT Vec& force) override
    {
        force.resize(0);
    }
    void CommitState() override
    {
        // Newton 及线搜索只读取已提交塑性位移；仅接受步更新内部变量。
        state_.plasticDisplacement = ReturnMapping().plasticDisplacement;
        ++commits;
    }
    void BackupStepState() override
    {
        step_ = state_;
        state_.increment = 0.0;
    }
    void GetStepIncrement(_OUT Vec& increment) const override
    {
        increment = Vec::Constant(1, state_.increment);
    }
    void RecordStepIterations(double, int iterations) override
    {
        lastIterations_ = iterations;
        maximumIterations = std::max(maximumIterations, iterations);
        totalIterations += iterations;
    }
    void ReportProgress(double, const QString& message) override
    {
        lastMessage = message;
    }
    void OnStepCompleted(double time) override
    {
        if (!std::isfinite(time) || !std::isfinite(state_.q) || !std::isfinite(state_.v) || !std::isfinite(state_.a) ||
            !std::isfinite(state_.plasticDisplacement))
            throw std::runtime_error("Nonfinite Newmark literature state");
        const double timeStep = time - endTime;
        if (sampleCount > 0)
        {
            increasing = increasing && timeStep > 0.0;
            steps.push_back(timeStep);
        }
        const double load = Load(time);
        const double force = ReturnMapping().force;
        const double damping = Damping() * state_.v;
        const double inertia = Mass() * state_.a;
        const double residual = load - force - damping - inertia;
        maximumAcceptedResidual = std::max(maximumAcceptedResidual, std::abs(residual));
        stream_ << time << ',' << state_.q << ',' << state_.v << ',' << state_.a << ',' << timeStep << ',' << load
                << ',' << force << ',' << damping << ',' << inertia << ',' << residual << ','
                << state_.plasticDisplacement << ',' << state_.increment << ',' << lastIterations_ << '\n';
        endTime = time;
        ++sampleCount;
    }

    std::vector<double> steps;
    double endTime = 0.0;
    double maximumAcceptedResidual = 0.0;
    int sampleCount = 0;
    int commits = 0;
    int maximumIterations = 0;
    long long totalIterations = 0;
    bool increasing = true;
    QString lastMessage;

private:
    double Mass() const
    {
        return pendulumAmplitude_ > 0.0 ? 1.0 : chopraMass;
    }
    double Damping() const
    {
        return pendulumAmplitude_ > 0.0 ? 0.0 : ChopraDamping();
    }
    double Load(double time) const
    {
        return pendulumAmplitude_ > 0.0 ? 0.0 : ChopraLoad(time);
    }

    ReturnMappingResult ReturnMapping() const
    {
        ReturnMappingResult result;
        if (pendulumAmplitude_ > 0.0)
        {
            result.force = std::sin(state_.q);
            result.tangent = std::cos(state_.q);
            return result;
        }
        result.plasticDisplacement = state_.plasticDisplacement;
        const double elasticTrial = chopraStiffness * (state_.q - state_.plasticDisplacement);
        // 与官方 ElasticPP 的屈服边界切线约定相同；迭代中不提交塑性位移。
        if (!elastoplastic_ ||
            std::abs(elasticTrial) - chopraYieldForce <= -chopraStiffness * std::numeric_limits<double>::epsilon())
        {
            result.force = elastoplastic_ ? elasticTrial : chopraStiffness * state_.q;
            return result;
        }
        result.force = std::copysign(chopraYieldForce, elasticTrial);
        result.tangent = 0.0;
        result.plasticDisplacement = state_.q - result.force / chopraStiffness;
        return result;
    }

    bool elastoplastic_;
    double pendulumAmplitude_;
    LiteratureState state_;
    LiteratureState step_;
    LiteratureState trial_;
    LiteratureState trialStep_;
    std::vector<LiteratureCheckpoint> checkpoints_;
    QFile file_;
    QTextStream stream_;
    int lastIterations_ = 0;
};

static QString LiteratureOption(const QStringList& arguments, const QString& option)
{
    const int index = arguments.indexOf(option);
    if (index < 0)
        return {};
    if (index + 1 >= arguments.size() || arguments[index + 1].startsWith(QStringLiteral("--")))
        throw std::runtime_error((option + QStringLiteral(" requires a value")).toStdString());
    return arguments[index + 1];
}

static double PositiveLiteratureOption(const QStringList& arguments, const QString& option, double defaultValue)
{
    const QString value = LiteratureOption(arguments, option);
    if (value.isEmpty())
        return defaultValue;
    bool valid = false;
    const double result = value.toDouble(&valid);
    if (!valid || !std::isfinite(result) || result <= 0.0)
        throw std::runtime_error((option + QStringLiteral(" must be positive and finite")).toStdString());
    return result;
}

static QJsonObject RunNewmarkLiteratureCase(const QString& name, const QString& directory, double timeStep,
                                            double duration, double tolerance)
{
    const bool elastoplastic = name == QStringLiteral("chopra-elastoplastic");
    const bool pendulum = name.startsWith(QStringLiteral("liang-pendulum-"));
    const double amplitude = pendulum ? (name.endsWith(QStringLiteral("small")) ? 0.1 : 0.5) * std::acos(-1.0) : 0.0;
    const QString configurationPath = QDir(directory).filePath(name + QStringLiteral(".configuration.json"));
    const QString resultPath = QDir(directory).filePath(name + QStringLiteral(".result.json"));
    const QString responsePath = QDir(directory).filePath(name + QStringLiteral(".csv"));
    for (const QString& path : {configurationPath, resultPath, responsePath})
        if (QFileInfo::exists(path))
            throw std::runtime_error("Newmark literature output exists; use a fresh directory to preserve records");
    AnalysisStep step;
    step.m_Type = EnumKeyword::StepType::DYNAMIC;
    step.m_DynamicSolverType = SolverNameSpace::SolverType::Newmark;
    step.m_StepSize = timeStep;
    step.m_Tolerance = tolerance;
    step.m_MaxIterations = 32;
    auto solver = SolverNameSpace::SolverFactory::Create(step);
    auto* newmark = dynamic_cast<SolverNameSpace::SolverNewmark*>(solver.get());
    if (!newmark || newmark->GetType() != SolverNameSpace::SolverType::Newmark ||
        newmark->GetParams().timeStepMode != SolverNameSpace::TimeStepMode::Fixed ||
        newmark->GetParams().beta != 0.25 || newmark->GetParams().gamma != 0.5)
        throw std::runtime_error("Factory did not preserve fixed average-acceleration Newmark identity");
    const auto& parameters = newmark->GetParams();
    QJsonObject configuration{
        {"schema", "newmark-literature-case-v1"},
        {"date", "2026-10-19"},
        {"case", name},
        {"factory_solver_type", static_cast<int>(solver->GetType())},
        {"factory_solver_name", solver->GetName()},
        {"beta", parameters.beta},
        {"gamma", parameters.gamma},
        {"mode", "fixed_with_production_failure_cutback"},
        {"requested_step", timeStep},
        {"requested_duration", duration},
        {"nonlinear_tolerance", parameters.tol},
        {"normal_residual_definition", "norm(R) <= tol*max(1,norm(Fext))"},
        {"maximum_newton_evaluations", parameters.maxIter},
        {"minimum_step", parameters.minimumTimeStep},
        {"cutback_factor", parameters.cutbackFactor},
        {"recovery_factor", parameters.recoveryFactor},
        {"maximum_cutbacks", parameters.maximumCutbacks},
        {"mass", pendulum ? 1.0 : chopraMass},
        {"stiffness", pendulum ? QJsonValue() : QJsonValue(chopraStiffness)},
        {"damping_ratio", pendulum ? 0.0 : chopraDampingRatio},
        {"damping_coefficient", pendulum ? 0.0 : ChopraDamping()},
        {"yield_force", elastoplastic ? QJsonValue(chopraYieldForce) : QJsonValue()},
        {"yield_displacement", elastoplastic ? QJsonValue(0.75) : QJsonValue()},
        {"initial_q_v_a", QJsonArray{amplitude, 0.0, -std::sin(amplitude)}},
        {"load", "10*sin(2*pi*t/1.2), 0<=t<=0.6; zero afterwards"},
        {"source_url", "https://raw.githubusercontent.com/OpenSees/OpenSees/ddf7b364e0a089a8e05c75a0576e759f3a5eba91/"
                       "EXAMPLES/verification/NewmarkIntegrator.tcl"},
        {"source_commit", "ddf7b364e0a089a8e05c75a0576e759f3a5eba91"},
        {"source_sha256", "0a4a745d27c368814f061ab1ecf2e36f5d98a0117aa8e2b758448abe65628591"},
        {"source_reference", "Chopra, Dynamics of Structures, 4th ed., 2012; linear ex5.3/5.4, nonlinear ex5.5/5.6"},
        {"source_notes", "Official script additionally labels nonlinear Section5.7; nonlinear loop checks only t=.1.. "
                         ".9 despite ten table rows"},
        {"official_nonlinear_solver",
         "OpenSees Newton, NormDispIncr1e-4, max6; candidate uses production residual Newton with Armijo"},
        {"material_scope", elastoplastic
                               ? "test-only 1D ideal plastic return mapping; production material is not certified"
                               : "test-only linear SDOF restoring force"},
        {"state_policy", "independently accumulated step increment; full trial/checkpoint/plastic-state restore"},
        {"response_csv", responsePath},
        {"numerical_pass", QJsonValue()},
        {"source_units", "consistent source numerical units; official Chopra script does not specify physical units"},
        {"comparison_policy",
         "COMPLETE_UNASSESSED; independent Python uses official u assertion and independent full-precision q/v/a"}};
    if (pendulum)
    {
        configuration["source_url"] = QStringLiteral("https://doi.org/10.1061/(ASCE)EM.1943-7889.0001073");
        configuration["source_reference"] =
            QStringLiteral("Liang and Mosalam (2016), Lyapunov Stability and Accuracy of Direct Integration Algorithms "
                           "Applied to Nonlinear Dynamic Problems, JEM142(5)04016022");
        configuration["source_commit"] = QJsonValue();
        configuration["source_sha256"] = QJsonValue();
        configuration["source_notes"] =
            QStringLiteral("Implicit Newmark beta=.25 gamma=.5; paper step ratios h/T=.04,.08,.10; exact period and "
                           "comparison windows frozen by external runner");
        configuration["source_units"] =
            QStringLiteral("normalized time tau=sqrt(g/L)*t; no physical g or L is invented");
        configuration["equation"] = QStringLiteral("d2theta/dtau2+sin(theta)=0; M=1 C=0 tangent=cos(theta)");
        configuration["load"] = QStringLiteral("zero external load");
        configuration["material_scope"] = QStringLiteral(
            "test-only exact nonlinear pendulum restoring force; ordinary scalar angle, no FE rotation model");
        configuration["official_nonlinear_solver"] =
            QStringLiteral("paper implicit Newmark; candidate uses production residual Newton with Armijo");
        configuration["comparison_policy"] = QStringLiteral(
            "COMPLETE_UNASSESSED; independent elliptic exact reference and paper period/amplitude checks required");
        configuration["pendulum_parameter_m"] = std::pow(std::sin(0.5 * amplitude), 2);
        configuration["period_formula"] =
            QStringLiteral("T_tau=4*K(m), m=sin(theta0/2)^2; external runner provides exact T");
    }
    if (!WriteLiteratureJson(configurationPath, configuration))
        throw std::runtime_error("Cannot freeze Newmark literature configuration");
    NewmarkLiteratureModel model(elastoplastic, amplitude, responsePath);
    bool solved = false;
    QString exception;
    try
    {
        solved = solver->Solve(model, duration);
    }
    catch (const std::exception& error)
    {
        exception = QString::fromLocal8Bit(error.what());
    }
    const bool reached = std::abs(model.endTime - duration) <= 1.0e-10 * std::max(1.0, duration);
    const auto& statistics = newmark->GetTimeStepStatistics();
    QJsonArray acceptedSteps;
    bool nominalSequence = !model.steps.empty();
    const double stepComparisonLimit =
        128.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, duration) + 1.0e-8 * timeStep;
    double maximumStepDeviation = 0.0;
    for (std::size_t index = 0; index < model.steps.size(); ++index)
    {
        const double actual = model.steps[index];
        acceptedSteps.append(actual);
        const bool last = index + 1 == model.steps.size();
        maximumStepDeviation = std::max(maximumStepDeviation, std::abs(actual - timeStep));
        nominalSequence = nominalSequence && actual > 0.0 &&
                          (std::abs(actual - timeStep) <= stepComparisonLimit ||
                           (last && actual < timeStep && duration / timeStep != std::floor(duration / timeStep)));
    }
    const bool complete = solved && reached && model.increasing && exception.isEmpty();
    configuration["status"] = complete ? QStringLiteral("COMPLETE_UNASSESSED") : QStringLiteral("INCOMPLETE");
    configuration["solved"] = solved;
    configuration["actual_end_time"] = model.endTime;
    configuration["reached_duration"] = reached;
    configuration["strictly_increasing_history"] = model.increasing;
    configuration["accepted_steps"] = statistics.acceptedSteps;
    configuration["rejected_attempts"] = statistics.rejectedAttempts;
    configuration["accepted_time_steps"] = acceptedSteps;
    configuration["minimum_accepted_step"] = statistics.minimumAcceptedTimeStep;
    configuration["maximum_accepted_step"] = statistics.maximumAcceptedTimeStep;
    configuration["maximum_step_deviation"] = maximumStepDeviation;
    configuration["fixed_step_comparison_absolute_limit"] = stepComparisonLimit;
    configuration["fixed_step_sequence_matches_requested"] = nominalSequence;
    configuration["precision_limited_solves"] = static_cast<double>(newmark->GetPrecisionLimitedSolveCount());
    configuration["normal_newton_accepted_steps"] =
        static_cast<double>(statistics.acceptedSteps - newmark->GetPrecisionLimitedSolveCount());
    configuration["maximum_accepted_equilibrium_residual"] = model.maximumAcceptedResidual;
    configuration["maximum_newton_evaluations_observed"] = model.maximumIterations;
    configuration["total_newton_evaluations_accepted_steps"] = static_cast<double>(model.totalIterations);
    configuration["commits"] = model.commits;
    configuration["fixed_step_reference_comparison_eligible"] = complete && nominalSequence &&
                                                                statistics.rejectedAttempts == 0 &&
                                                                newmark->GetPrecisionLimitedSolveCount() == 0;
    configuration["exception"] = exception;
    configuration["last_solver_message"] = model.lastMessage;
    if (!WriteLiteratureJson(resultPath, configuration))
        throw std::runtime_error("Cannot write Newmark literature result");
    QTextStream(stdout) << "newmark_literature case=" << name << " status=" << configuration["status"].toString()
                        << " end=" << model.endTime << " accepted=" << statistics.acceptedSteps
                        << " rejected=" << statistics.rejectedAttempts << Qt::endl;
    return configuration;
}
}

QJsonObject NewmarkLiterature::RunCase(const QString& name, const QString& outputDirectory, double timeStep,
                                       double duration, double tolerance)
{
    if (name != QStringLiteral("chopra-linear") && name != QStringLiteral("chopra-elastoplastic") &&
        name != QStringLiteral("liang-pendulum-small") && name != QStringLiteral("liang-pendulum-large"))
        throw std::runtime_error("Unsupported Newmark literature case");
    if (!std::isfinite(timeStep) || timeStep <= 0.0 || !std::isfinite(duration) || duration <= 0.0 ||
        !std::isfinite(tolerance) || tolerance <= 0.0)
        throw std::runtime_error("Newmark literature step, duration and tolerance must be positive and finite");
    if (!QDir().mkpath(outputDirectory))
        throw std::runtime_error("Cannot create Newmark literature output directory");
    return RunNewmarkLiteratureCase(name, outputDirectory, timeStep, duration, tolerance);
}

/** @brief 用 Newmark 文献的原始物理输入输出逐项建立专属复现记录。
 * @details 输入 Chopra 线性、理想弹塑性及 Liang/Mosalam 两幅值非线性单摆原题。
 *          单摆使用无量纲时间，必须显式指定已核定的步长与时长；工厂固定平均加速度 Newmark。
 *          完整输出 q/v/a、塑性内部状态、平衡残差和真实步列；最终数值判定由独立原表比较执行。
 *          测试专用理想塑性模型仅用于积分器验收，不认证生产材料、梁索装配或有限转动。
 * @author   yangqiuyou
 * @date     2026.10.19
 */
std::optional<int> VerifyNewmarkLiterature(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-newmark-literature")))
        return std::nullopt;
    try
    {
        const QString directory =
            QFileInfo(LiteratureOption(arguments, QStringLiteral("--verify-newmark-literature"))).absoluteFilePath();
        const QString selected = LiteratureOption(arguments, QStringLiteral("--newmark-case")).toLower();
        if (!selected.isEmpty() && selected != "chopra-linear" && selected != "chopra-elastoplastic" &&
            selected != "liang-pendulum-small" && selected != "liang-pendulum-large")
            throw std::runtime_error("Unsupported Newmark literature case");
        if (!QDir().mkpath(directory))
            throw std::runtime_error("Cannot create Newmark literature output directory");
        const QString summaryPath = QDir(directory).filePath(QStringLiteral("summary.json"));
        if (QFileInfo::exists(summaryPath))
            throw std::runtime_error("Newmark literature summary exists; use a fresh output directory");
        QJsonArray results;
        bool completed = true;
        for (const QString& name : {QStringLiteral("chopra-linear"), QStringLiteral("chopra-elastoplastic"),
                                    QStringLiteral("liang-pendulum-small"), QStringLiteral("liang-pendulum-large")})
        {
            if (selected.isEmpty() && name.startsWith(QStringLiteral("liang-pendulum-")))
                continue;
            if (!selected.isEmpty() && selected != name)
                continue;
            const bool pendulum = name.startsWith(QStringLiteral("liang-pendulum-"));
            if (pendulum && (LiteratureOption(arguments, QStringLiteral("--newmark-step")).isEmpty() ||
                             LiteratureOption(arguments, QStringLiteral("--newmark-duration")).isEmpty()))
                throw std::runtime_error(
                    "Liang pendulum requires explicit normalized --newmark-step and --newmark-duration");
            const double duration = PositiveLiteratureOption(arguments, QStringLiteral("--newmark-duration"),
                                                             name == "chopra-elastoplastic" ? 0.9 : 1.0);
            const double timeStep = PositiveLiteratureOption(arguments, QStringLiteral("--newmark-step"), 0.1);
            const double tolerance =
                PositiveLiteratureOption(arguments, QStringLiteral("--newmark-tolerance"), 1.0e-12);
            const QJsonObject result = NewmarkLiterature::RunCase(name, directory, timeStep, duration, tolerance);
            completed = completed && result["status"].toString() == QStringLiteral("COMPLETE_UNASSESSED");
            results.append(result);
        }
        const QJsonObject summary{{"schema", "newmark-literature-v1"},
                                  {"date", "2026-10-19"},
                                  {"algorithm", "factory fixed Newmark beta=.25 gamma=.5 only"},
                                  {"status", completed ? "COMPLETE_UNASSESSED" : "INCOMPLETE"},
                                  {"numerical_pass", QJsonValue()},
                                  {"results", results},
                                  {"comparison_policy", "independent frozen public-source q/v/a table required"}};
        if (!WriteLiteratureJson(summaryPath, summary))
            throw std::runtime_error("Cannot write Newmark literature summary");
        return completed ? 0 : 4;
    }
    catch (const std::exception& error)
    {
        QTextStream(stderr) << "newmark_literature error=" << error.what() << Qt::endl;
        return 2;
    }
}
