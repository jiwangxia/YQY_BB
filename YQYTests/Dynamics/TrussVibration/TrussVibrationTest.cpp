#include "Dynamics/TrussVibration/TrussVibrationTest.h"

#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementTruss.h"
#include "Solver/Dynamic/Integrators/SolverNewmark.h"
#include "Solver/Dynamic/Integrators/SolverAdaptiveTSSBN.h"
#include "Solver/Dynamic/Integrators/SolverRungeKutta4.h"
#include "Solver/Factory/SolverFactory.h"
#include "Support/SolveModel.h"
#include "Support/TestPaths.h"
#include <gtest/gtest.h>
#include <Eigen/Cholesky>
#include <QCryptographicHash>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTextStream>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace
{
using SolverNameSpace::SpMat;
using SolverNameSpace::Vec;
const char* trussPlanHash = "f5d5d5c7e8a493dbe9afd4c8fe7c1e96e8422c0f77a3a279625ae1464a8a38b4";
const char* trussReferenceHash = "9379c970341ab18e39c540de739e7e2b5dc074b019b796f0927212faf7b7276d";
const char* adaptiveTrussPlanHash = "7fc6e2aa65d67611558f53769572c35223971b1595703233874d4da7b4cd3811";
const char* rkTrussPlanHash = "270a55aa339a8382fbb1c1b2932af2e8ee9a471120f5951639209b7c0418ba09";
const char* tssbnTrussPlanHash = "faa142836562e96d64c6cadf656594b7bac35a734becd4b324ab0255213f5d70";

struct TrussSample
{
    double time = 0.0;
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d a = Eigen::Vector3d::Zero();
    Eigen::Vector3d axial = Eigen::Vector3d::Zero();
    double reaction = 0.0;
    double constrainedMaximum = 0.0;
};

static QString TrussInputDirectory()
{
    return QDir(TestPaths::TestRoot()).filePath(QStringLiteral("TestModels/Dynamics/TrussVibration"));
}

static QByteArray TrussFileHash(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot read truss benchmark input for SHA256");
    return QCryptographicHash::hash(file.readAll(), QCryptographicHash::Sha256).toHex();
}

static QJsonObject ReadTrussJson(const QString& path, const QByteArray& expectedHash)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot read persistent real-truss verification data");
    const QByteArray bytes = file.readAll();
    if (QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex() != expectedHash)
        throw std::runtime_error("Real-truss frozen input/reference SHA256 differs");
    const QJsonObject object = QJsonDocument::fromJson(bytes).object();
    if (object.isEmpty())
        throw std::runtime_error("Invalid real-truss verification JSON");
    return object;
}

static bool WriteTrussJson(const QString& path, const QJsonObject& object)
{
    QFile file(path);
    return file.open(QIODevice::WriteOnly | QIODevice::Truncate) &&
           file.write(QJsonDocument(object).toJson(QJsonDocument::Indented)) > 0;
}

static Eigen::Vector3d TrussVector(const QJsonValue& value)
{
    const QJsonArray values = value.toArray();
    if (values.size() != 3)
        throw std::runtime_error("Real-truss reference vector must have three components");
    Eigen::Vector3d vector;
    for (int index = 0; index < 3; ++index)
    {
        if (!values[index].isDouble() || !std::isfinite(values[index].toDouble()))
            throw std::runtime_error("Nonfinite real-truss reference vector");
        vector[index] = values[index].toDouble();
    }
    return vector;
}

static QJsonArray TrussArray(const Eigen::Vector3d& value)
{
    return {value[0], value[1], value[2]};
}

static Eigen::Matrix3d TrussMatrix(const QJsonValue& value)
{
    const QJsonArray rows = value.toArray();
    if (rows.size() != 3)
        throw std::runtime_error("Real-truss reference matrix must have three rows");
    Eigen::Matrix3d matrix;
    for (int row = 0; row < 3; ++row)
        matrix.row(row) = TrussVector(rows[row]).transpose();
    return matrix;
}

// 从真实生产结果帧读取全部三个自由节点；不以测试ODE替代节点、单元或装配。
static std::vector<TrussSample> SampleTrussHistory(const StructureData& model)
{
    std::vector<TrussSample> samples;
    for (const auto& frame : model.GetOutputter().GetDataSet())
    {
        if (frame.GetNodeDatas().size() != 4 || frame.GetElementDatas().size() != 3)
            throw std::runtime_error("Actual truss result frame has missing nodes or elements");
        TrussSample sample;
        sample.time = frame.GetTime();
        for (int index = 0; index < 3; ++index)
        {
            sample.q[index] = frame.GetNodeData(index + 2, EnumKeyword::NodeResultType::U1);
            sample.v[index] = frame.GetNodeData(index + 2, EnumKeyword::NodeResultType::V1);
            sample.a[index] = frame.GetNodeData(index + 2, EnumKeyword::NodeResultType::A1);
            sample.axial[index] = frame.GetElementData(index + 1, EnumKeyword::ElementResultType::AxialForce);
        }
        sample.reaction = frame.GetNodeData(1, EnumKeyword::NodeResultType::R1);
        for (const auto& [id, node] : frame.GetNodeDatas())
        {
            for (int component = 0; component < 3; ++component)
            {
                if (id != 1 && component == 0)
                    continue;
                const auto displacementType = component == 0   ? EnumKeyword::NodeResultType::U1
                                              : component == 1 ? EnumKeyword::NodeResultType::U2
                                                               : EnumKeyword::NodeResultType::U3;
                if (!std::isfinite(node.GetValue(displacementType)) ||
                    !std::isfinite(node.GetVelocityComponent(component)) ||
                    !std::isfinite(node.GetAccelerationComponent(component)))
                    sample.constrainedMaximum = std::numeric_limits<double>::infinity();
                sample.constrainedMaximum =
                    std::max({sample.constrainedMaximum, std::abs(node.GetValue(displacementType)),
                              std::abs(node.GetVelocityComponent(component)),
                              std::abs(node.GetAccelerationComponent(component))});
            }
        }
        samples.push_back(sample);
    }
    return samples;
}

static void WriteTrussHistory(const QString& path, const std::vector<TrussSample>& samples)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        throw std::runtime_error("Cannot write actual truss history");
    QTextStream stream(&file);
    stream.setRealNumberPrecision(17);
    stream << "time,q2,q3,q4,v2,v3,v4,a2,a3,a4,N1,N2,N3,root_reaction,max_constrained_q_v_a,actual_dt\n";
    double previous = 0.0;
    for (const TrussSample& sample : samples)
    {
        stream << sample.time;
        for (const auto* vector : {&sample.q, &sample.v, &sample.a, &sample.axial})
            for (int index = 0; index < 3; ++index)
                stream << ',' << (*vector)[index];
        stream << ',' << sample.reaction << ',' << sample.constrainedMaximum << ',' << sample.time - previous << '\n';
        previous = sample.time;
    }
}

static void WriteAdaptiveTrussHistory(const QString& path, const StructureData& model)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        throw std::runtime_error("Cannot write adaptive real-truss nodal history");
    QTextStream stream(&file);
    stream.setRealNumberPrecision(17);
    stream << "time,node_id,ux,uy,uz,vx,vy,vz,ax,ay,az,accepted_dt\n";
    double previous = 0.0;
    for (const auto& frame : model.GetOutputter().GetDataSet())
    {
        for (const auto& [id, node] : frame.GetNodeDatas())
        {
            stream << frame.GetTime() << ',' << id;
            for (int component = 0; component < 3; ++component)
                stream << ','
                       << node.GetValue(component == 0   ? EnumKeyword::NodeResultType::U1
                                        : component == 1 ? EnumKeyword::NodeResultType::U2
                                                         : EnumKeyword::NodeResultType::U3);
            for (int component = 0; component < 3; ++component)
                stream << ',' << node.GetVelocityComponent(component);
            for (int component = 0; component < 3; ++component)
                stream << ',' << node.GetAccelerationComponent(component);
            stream << ',' << frame.GetTime() - previous << '\n';
        }
        previous = frame.GetTime();
    }
}

static void SetAdaptiveTrussControls(AnalysisStep& step, const QJsonObject& run)
{
    step.m_DynamicSolverType = SolverNameSpace::SolverType::AdaptiveNewmark;
    step.m_StepSize = run.value("initial_time_step").toDouble();
    auto& settings = step.m_AdaptiveTimeStep;
    settings.minimumTimeStep = run.value("minimum_time_step").toDouble();
    settings.maximumTimeStep = run.value("maximum_time_step").toDouble();
    settings.relativeTolerance = run.value("relative_tolerance").toDouble();
    settings.absoluteTolerance = run.value("absolute_tolerance").toDouble();
    settings.velocityRelativeTolerance = run.value("velocity_relative_tolerance").toDouble();
    settings.velocityAbsoluteTolerance = run.value("velocity_absolute_tolerance").toDouble();
    settings.includeVelocityInError = true;
    settings.safetyFactor = .9;
    settings.minimumScaleFactor = .2;
    settings.maximumGrowthFactor = 2.0;
    settings.solveFailureFactor = .5;
    settings.maximumRejectedAttempts = 32;
}

// 只覆盖冻结计划中的步长和误差尺度，其余控制器设置保留生产默认值。
static void SetTssbnTrussControls(AnalysisStep& step, const QJsonObject& run, SolverNameSpace::SolverType type)
{
    step.m_DynamicSolverType = type;
    step.m_StepSize = run.value("initial_time_step").toDouble();
    auto& settings = step.m_AdaptiveTssbn;
    settings.spectralRadiusInfinity = 1.0;
    settings.minimumTimeStep = run.value("minimum_time_step").toDouble();
    settings.maximumTimeStep = run.value("maximum_time_step").toDouble();
    settings.relativeTolerance = run.value("relative_tolerance").toDouble();
    settings.absoluteTolerance = run.value("absolute_tolerance").toDouble();
    settings.velocityAbsoluteTolerance = run.value("velocity_absolute_tolerance").toDouble();
    if (!(step.m_StepSize > 0.0 && settings.minimumTimeStep > 0.0 &&
          settings.maximumTimeStep >= settings.minimumTimeStep && settings.relativeTolerance > 0.0 &&
          settings.absoluteTolerance > 0.0 && settings.velocityAbsoluteTolerance > 0.0))
        throw std::runtime_error("Invalid frozen TSSBN time-step/error scales");
}

static QJsonObject TssbnTrussParameters(const SolverNameSpace::SolverAdaptiveTSSBN::Params& p)
{
    return {{"spectral_radius_infinity", p.spectralRadiusInfinity},
            {"initial_time_step", p.initialTimeStep},
            {"minimum_time_step", p.minimumTimeStep},
            {"maximum_time_step", p.maximumTimeStep},
            {"relative_tolerance", p.relativeTolerance},
            {"absolute_tolerance", p.absoluteTolerance},
            {"velocity_absolute_tolerance", p.velocityAbsoluteTolerance},
            {"error_norm_mode", static_cast<int>(p.errorNormMode)},
            {"error_norm_name", p.errorNormMode == SolverNameSpace::TssbnErrorNormMode::LegacyGlobal
                                    ? "LegacyGlobal"
                                    : "ComponentwiseMax"},
            {"use_duan2026_controller", p.useDuan2026Controller},
            {"controller", p.useDuan2026Controller ? "Duan2026" : "Mahnken"},
            {"use_reference_extrapolated_acceleration", p.useReferenceExtrapolatedAcceleration},
            {"reference_independent_error_control", p.referenceIndependentErrorControl},
            {"safety_factor", p.safetyFactor},
            {"shrink_factor", p.shrinkFactor},
            {"maximum_growth_factor", p.maximumGrowthFactor},
            {"target_newton_iterations", p.targetNewtonIterations},
            {"maximum_total_newton_iterations", p.maximumTotalNewtonIterations},
            {"derivative_gain", p.derivativeGain},
            {"minimum_derivative_factor", p.minimumDerivativeFactor},
            {"maximum_derivative_factor", p.maximumDerivativeFactor},
            {"maximum_rejected_attempts", p.maximumRejectedAttempts},
            {"nonlinear_tolerance", p.nonlinearTolerance},
            {"maximum_newton_iterations", p.maximumNewtonIterations},
            {"aerodynamic_tangent_mode", static_cast<int>(p.aerodynamicTangentMode)},
            {"diagnostic_cache_external_force_per_stage", p.diagnosticCacheExternalForcePerStage},
            {"diagnostic_error_trace", p.diagnosticErrorTrace},
            {"reference_galloping", p.referenceGalloping}};
}

static QJsonObject RunActualTruss(const QJsonObject& run, const QJsonObject& plan, const QString& directory,
                                  const QJsonObject& adaptiveRun = {},
                                  SolverNameSpace::SolverType requestedSolver = SolverNameSpace::SolverType::Newmark)
{
    const bool rk = requestedSolver == SolverNameSpace::SolverType::RungeKutta4 ||
                    requestedSolver == SolverNameSpace::SolverType::AdaptiveRungeKutta4;
    const bool tssbn = requestedSolver == SolverNameSpace::SolverType::AdaptiveTSSBN ||
                       requestedSolver == SolverNameSpace::SolverType::ReferenceTSSBN;
    const bool referenceTssbn = requestedSolver == SolverNameSpace::SolverType::ReferenceTSSBN;
    const bool adaptive =
        rk ? requestedSolver == SolverNameSpace::SolverType::AdaptiveRungeKutta4 : !adaptiveRun.isEmpty();
    const bool independentExport = adaptive || rk;
    if (!QDir().mkpath(directory) || QFileInfo::exists(QDir(directory).filePath(QStringLiteral("result.json"))))
        throw std::runtime_error("Use a fresh actual-truss output directory");
    const QString name = run.value("case").toString();
    const double force = run.value("force").toDouble();
    const double stepSize = run.value("step").toDouble();
    const double duration = run.value("duration").toDouble();
    const Eigen::Matrix3d referenceMass = TrussMatrix(plan.value("mass"));
    const Eigen::Matrix3d referenceStiffness = TrussMatrix(plan.value("linear_stiffness"));
    const Eigen::Vector3d scales = TrussVector(run.value("scales_q_v_a"));
    const QJsonObject criteria = plan.value("criteria").toObject();
    QString error;
    auto model = SolveModel::Read(QStringLiteral("Dynamics/TrussVibration/") + name + ".bdf", error);
    if (!model || model->m_Elements.size() != 3 || model->m_Nodes.size() != 4 || model->m_AnalysisStep.size() != 1)
        throw std::runtime_error((QStringLiteral("Cannot read actual three-truss model: ") + error).toStdString());
    for (const auto& [id, element] : model->m_Elements)
    {
        Q_UNUSED(id);
        const auto truss = std::dynamic_pointer_cast<ElementTruss>(element);
        if (!truss || truss->m_UseEngineeringStrain)
            throw std::runtime_error("Actual truss benchmark requires three native logarithmic ElementTruss");
        truss->Get_L0();
    }
    auto step = model->m_AnalysisStep.at(1);
    step->m_Type = EnumKeyword::StepType::DYNAMIC;
    step->isDynamic = true;
    step->SolverNameSpace::IAnalysisModel::isDynamic = true;
    step->m_DynamicSolverType = SolverNameSpace::SolverType::Newmark;
    step->m_Time = duration;
    step->m_StepSize = stepSize;
    if (tssbn)
        SetTssbnTrussControls(*step, adaptiveRun, requestedSolver);
    else if (adaptive)
        SetAdaptiveTrussControls(*step, adaptiveRun);
    if (rk)
        step->m_DynamicSolverType = requestedSolver;
    step->m_Tolerance = plan.value("newmark").toObject().value("tolerance").toDouble();
    step->m_MaxIterations = plan.value("newmark").toObject().value("max_iterations").toInt();
    step->SetAssemblyThreadCountOverride(1);
    step->SetPardisoThreadCountOverride(1);
    model->m_OutputControl.m_StreamResult = false;
    model->GetOutputter().SetKeepFramesInMemory(true);
    step->SetStructure(model);
    step->Init();
    if (step->GetFreeDofs() != 3)
        throw std::runtime_error("Real-truss model must have exactly three axial free DOFs");
    SpMat mass, stiffness;
    step->AssembleMassMatrix(mass);
    step->Assemble_Matrix(stiffness, false);
    const double massError = (Eigen::MatrixXd(mass) - referenceMass).cwiseAbs().maxCoeff();
    const double stiffnessError = (Eigen::MatrixXd(stiffness) - referenceStiffness).cwiseAbs().maxCoeff();
    SolverNameSpace::SystemEvaluationRequest request;
    request.Include(SolverNameSpace::SystemContribution::ExternalForce);
    request.Include(SolverNameSpace::SystemContribution::InternalForce);
    SolverNameSpace::SystemEvaluationResult initial;
    step->EvaluateSystem(request, initial);
    if (initial.externalForce.size() != 3 || initial.internalForce.size() != 3)
        throw std::runtime_error("Real-truss initial production force dimension differs");
    const Vec initialAcceleration = Eigen::MatrixXd(mass).ldlt().solve(initial.externalForce - initial.internalForce);
    if (!initialAcceleration.allFinite())
        throw std::runtime_error("Nonfinite real-truss initial acceleration");
    step->SetTrialKinematics(Vec::Zero(3), initialAcceleration);
    step->CommitState();
    step->OnStepCompleted(0.0);
    const auto solver = SolverNameSpace::SolverFactory::Create(*step);
    auto* newmark = dynamic_cast<SolverNameSpace::SolverNewmark*>(solver.get());
    auto* rungeKutta = dynamic_cast<SolverNameSpace::SolverRungeKutta4*>(solver.get());
    const auto* tssbnSolver = dynamic_cast<const SolverNameSpace::SolverAdaptiveTSSBN*>(solver.get());
    if (tssbn && (!tssbnSolver || solver->GetType() != requestedSolver ||
                  tssbnSolver->GetParams().spectralRadiusInfinity != 1.0 ||
                  tssbnSolver->GetParams().referenceGalloping != referenceTssbn ||
                  tssbnSolver->GetParams().referenceIndependentErrorControl ||
                  tssbnSolver->GetParams().useDuan2026Controller != referenceTssbn ||
                  tssbnSolver->GetParams().useReferenceExtrapolatedAcceleration != referenceTssbn ||
                  tssbnSolver->GetParams().errorNormMode !=
                      (referenceTssbn ? SolverNameSpace::TssbnErrorNormMode::LegacyGlobal
                                      : SolverNameSpace::TssbnErrorNormMode::ComponentwiseMax)))
        throw std::runtime_error("Factory TSSBN identity/default numerical policy differs");
    if (rk && (!rungeKutta || solver->GetType() != requestedSolver))
        throw std::runtime_error("Factory did not create requested real-truss RK4 algorithm");
    if (!rk && !tssbn &&
        (!newmark ||
         solver->GetType() !=
             (adaptive ? SolverNameSpace::SolverType::AdaptiveNewmark : SolverNameSpace::SolverType::Newmark) ||
         newmark->GetParams().timeStepMode !=
             (adaptive ? SolverNameSpace::TimeStepMode::Adaptive : SolverNameSpace::TimeStepMode::Fixed) ||
         newmark->GetParams().beta != 0.25 || newmark->GetParams().gamma != 0.5))
        throw std::runtime_error("Factory did not create fixed average-acceleration Newmark");
    QElapsedTimer timer;
    timer.start();
    step->SetRuntimeCallbacks({},
                              [&timer]()
                              {
                                  return timer.elapsed() > 120000;
                              });
    bool solved = false;
    QString exception;
    QElapsedTimer solverTimer;
    solverTimer.start();
    try
    {
        solved = solver->Solve(*step, duration);
    }
    catch (const std::exception& caught)
    {
        exception = QString::fromLocal8Bit(caught.what());
    }
    const double solverWallSeconds = solverTimer.nsecsElapsed() * 1.0e-9;
    step->ClearRuntimeCallbacks();
    const auto samples = SampleTrussHistory(*model);
    const QString historyPath = QDir(directory).filePath(QStringLiteral("response.csv"));
    WriteTrussHistory(independentExport ? QDir(directory).filePath("axial-response.csv") : historyPath, samples);
    if (independentExport)
        WriteAdaptiveTrussHistory(historyPath, *model);
    SolverNameSpace::TimeStepStatistics statistics;
    if (tssbn)
    {
        for (std::size_t index = 1; index < samples.size(); ++index)
            statistics.RecordAccepted(samples[index].time - samples[index - 1].time);
        statistics.acceptedSteps = tssbnSolver->GetAcceptedStepCount();
        statistics.rejectedAttempts = tssbnSolver->GetRejectedStepCount();
    }
    else
        statistics = rk ? rungeKutta->GetTimeStepStatistics() : newmark->GetTimeStepStatistics();
    const QJsonArray exact = run.value("rows").toArray();
    bool finite = !samples.empty();
    bool increasing = !samples.empty();
    bool fixed = !samples.empty();
    Eigen::Vector3d errors = Eigen::Vector3d::Zero();
    double maximumEquilibrium = 0.0, maximumForceLaw = 0.0, maximumReaction = 0.0;
    double maximumQUpdate = 0.0, maximumVUpdate = 0.0, maximumConstraint = 0.0, maximumStrain = 0.0;
    double maximumTimeDifference = 0.0;
    QJsonArray actualSteps;
    const double timeLimit = criteria.value("frame_time_absolute_limit").toDouble();
    const double forceLimit = std::max(1.0e-12, 1.0e-8 * force);
    const double qUpdateLimit = std::max(1.0e-12, 1.0e-9 * scales[0]);
    const double vUpdateLimit = std::max(1.0e-12, 1.0e-9 * scales[1]);
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
        const TrussSample& sample = samples[index];
        finite = finite && std::isfinite(sample.time) && sample.q.allFinite() && sample.v.allFinite() &&
                 sample.a.allFinite() && sample.axial.allFinite() && std::isfinite(sample.reaction) &&
                 std::isfinite(sample.constrainedMaximum);
        maximumConstraint = std::max(maximumConstraint, sample.constrainedMaximum);
        if (!independentExport && index < static_cast<std::size_t>(exact.size()))
        {
            const QJsonObject truth = exact[static_cast<int>(index)].toObject();
            maximumTimeDifference =
                std::max(maximumTimeDifference, std::abs(sample.time - truth.value("time").toDouble()));
            errors[0] =
                std::max(errors[0], (sample.q - TrussVector(truth.value("q"))).cwiseAbs().maxCoeff() / scales[0]);
            errors[1] =
                std::max(errors[1], (sample.v - TrussVector(truth.value("v"))).cwiseAbs().maxCoeff() / scales[1]);
            errors[2] =
                std::max(errors[2], (sample.a - TrussVector(truth.value("a"))).cwiseAbs().maxCoeff() / scales[2]);
        }
        Eigen::Vector3d axial;
        double previousQ = 0.0;
        for (int element = 0; element < 3; ++element)
        {
            const double extension = (sample.q[element] - previousQ) / 10.0;
            finite = finite && extension > -1.0;
            axial[element] = 10.0 * std::log1p(extension) / (1.0 + extension);
            maximumStrain = std::max(maximumStrain, std::abs(extension));
            previousQ = sample.q[element];
        }
        Eigen::Vector3d restoring(axial[0] - axial[1], axial[1] - axial[2], axial[2]);
        const Eigen::Vector3d load(0.0, 0.0, force);
        maximumEquilibrium =
            std::max(maximumEquilibrium, (referenceMass * sample.a + restoring - load).cwiseAbs().maxCoeff());
        maximumForceLaw = std::max(maximumForceLaw, (sample.axial - axial).cwiseAbs().maxCoeff());
        maximumReaction = std::max(maximumReaction, std::abs(sample.reaction - (sample.a[0] / 27.0 - axial[0])));
        if (index > 0)
        {
            const auto& previous = samples[index - 1];
            const double h = sample.time - previous.time;
            actualSteps.append(h);
            increasing = increasing && h > 0.0;
            fixed = fixed && std::abs(h - stepSize) <= timeLimit;
            if (!independentExport)
            {
                maximumQUpdate = std::max(
                    maximumQUpdate, (sample.q - previous.q - h * previous.v - 0.25 * h * h * (previous.a + sample.a))
                                        .cwiseAbs()
                                        .maxCoeff());
                maximumVUpdate = std::max(
                    maximumVUpdate, (sample.v - previous.v - 0.5 * h * (previous.a + sample.a)).cwiseAbs().maxCoeff());
            }
        }
    }
    const bool complete = solved && exception.isEmpty() && !samples.empty() &&
                          std::abs(samples.back().time - duration) <= timeLimit &&
                          (independentExport || samples.size() == static_cast<std::size_t>(exact.size()));
    const Eigen::Vector3d expectedInitialA = referenceMass.ldlt().solve(Eigen::Vector3d(0.0, 0.0, force));
    const bool initialMatches =
        !samples.empty() && samples.front().time == 0.0 && samples.front().q.isZero(1.0e-14) &&
        samples.front().v.isZero(1.0e-14) &&
        (samples.front().a - expectedInitialA).cwiseAbs().maxCoeff() <= std::max(1.0e-12, 1.0e-10 * scales[2]);
    const double errorLimit = 0.002 * std::pow(4.0, 2 - run.value("level").toInt());
    QJsonObject gates{
        {"complete_finite", complete && finite && increasing},
        {"actual_fixed_step_sequence", fixed && maximumTimeDifference <= timeLimit},
        {"zero_rejection_and_precision_limited",
         statistics.rejectedAttempts == 0 && (rk || tssbn || newmark->GetPrecisionLimitedSolveCount() == 0)},
        {"accepted_count_matches_history", statistics.acceptedSteps + 1 == static_cast<int>(samples.size())},
        {"native_truss_mass_and_initial_stiffness", massError <= 1.0e-12 && stiffnessError <= 1.0e-12},
        {"physical_initial_acceleration",
         initialMatches && (initial.externalForce - Eigen::Vector3d(0, 0, force)).norm() <= forceLimit},
        {"full_state_q_v_a", errors.allFinite() && errors.maxCoeff() <= errorLimit},
        {"independent_equilibrium", maximumEquilibrium <= forceLimit},
        {"native_axial_force_law", maximumForceLaw <= forceLimit},
        {"consistent_mass_support_reaction", maximumReaction <= forceLimit},
        {"Newmark_q_update", maximumQUpdate <= qUpdateLimit},
        {"Newmark_v_update", maximumVUpdate <= vUpdateLimit},
        {"constrained_q_v_a", maximumConstraint <= 1.0e-13},
        {"nonlinear_strain_exercised", name != "nonlinear" || maximumStrain >= 0.05}};
    if (independentExport)
    {
        for (const QString& gate :
             {QStringLiteral("actual_fixed_step_sequence"), QStringLiteral("zero_rejection_and_precision_limited"),
              QStringLiteral("full_state_q_v_a"), QStringLiteral("Newmark_q_update"),
              QStringLiteral("Newmark_v_update")})
            gates.remove(gate);
        bool bounded = true;
        const double boundRoundoff = 64 * std::numeric_limits<double>::epsilon() * std::max(1.0, duration);
        for (std::size_t index = 1; index < samples.size(); ++index)
        {
            const double h = samples[index].time - samples[index - 1].time;
            const bool finalRemainder =
                index + 1 == samples.size() && std::abs(samples[index].time - duration) <= timeLimit;
            bounded = bounded && h > 0.0 &&
                      (finalRemainder || h + boundRoundoff >= adaptiveRun.value("minimum_time_step").toDouble()) &&
                      h <= adaptiveRun.value("maximum_time_step").toDouble() * (1.0 + 1.0e-6) + boundRoundoff;
        }
        gates.insert(adaptive ? "adaptive_step_bounds" : "fixed_rk_step_bounds", bounded);
        if (rk && !adaptive)
            gates.insert("fixed_rk_step_sequence",
                         fixed && statistics.rejectedAttempts == 0 &&
                             statistics.acceptedSteps == adaptiveRun.value("expected_accepted_steps").toInt());
    }
    const bool endpointEquilibriumDiagnostic = gates.value("independent_equilibrium").toBool();
    if (referenceTssbn)
        gates.remove("independent_equilibrium");
    if (tssbn)
        gates.insert("zero_precision_limited_stages", tssbnSolver->GetPrecisionLimitedStageCount() == 0);
    bool passed = true;
    for (const QJsonValue& gate : gates)
        passed = passed && gate.toBool();
    QJsonObject result{
        {"case", name},
        {"level", run.value("level")},
        {"requested_step", stepSize},
        {"requested_duration", duration},
        {"force", force},
        {"actual_element_type", "ElementTruss"},
        {"native_logarithmic_strain", true},
        {"mass_type", "consistent"},
        {"solver", "factory fixed Newmark"},
        {"beta", (rk || tssbn) ? QJsonValue() : QJsonValue(newmark->GetParams().beta)},
        {"gamma", (rk || tssbn) ? QJsonValue() : QJsonValue(newmark->GetParams().gamma)},
        {"newton_tolerance",
         rk ? QJsonValue()
            : QJsonValue(tssbn ? tssbnSolver->GetParams().nonlinearTolerance : newmark->GetParams().tol)},
        {"solver_return", solved},
        {"exception", exception},
        {"completed", complete},
        {"finite", finite},
        {"accepted_steps", statistics.acceptedSteps},
        {"rejected_attempts", statistics.rejectedAttempts},
        {"precision_limited_solves",
         (rk || tssbn) ? QJsonValue() : QJsonValue(static_cast<double>(newmark->GetPrecisionLimitedSolveCount()))},
        {"actual_time_steps", actualSteps},
        {"last_time", samples.empty() ? 0.0 : samples.back().time},
        {"maximum_time_grid_difference", maximumTimeDifference},
        {"maximum_normalized_q_v_a_errors", TrussArray(errors)},
        {"level_normalized_q_v_a_limit", errorLimit},
        {"mass_max_absolute_error", massError},
        {"initial_stiffness_max_absolute_error", stiffnessError},
        {"physical_initial_acceleration", TrussArray(expectedInitialA)},
        {"maximum_absolute_equilibrium", maximumEquilibrium},
        {"maximum_axial_force_law_error", maximumForceLaw},
        {"maximum_reaction_error", maximumReaction},
        {"force_equilibrium_absolute_limit", forceLimit},
        {"maximum_Newmark_q_update_defect", maximumQUpdate},
        {"maximum_Newmark_v_update_defect", maximumVUpdate},
        {"q_update_absolute_limit", qUpdateLimit},
        {"v_update_absolute_limit", vUpdateLimit},
        {"maximum_constrained_q_v_a", maximumConstraint},
        {"maximum_abs_engineering_strain", maximumStrain},
        {"response_csv", historyPath},
        {"response_sha256", QString::fromLatin1(TrussFileHash(historyPath))},
        {"wall_seconds", timer.nsecsElapsed() * 1.0e-9},
        {"solver_wall_seconds", solverWallSeconds},
        {"gates", gates},
        {"numerical_pass", passed},
        {"status", passed     ? "PASS_SINGLE_LEVEL"
                   : complete ? "FAIL_FROZEN_GATE"
                              : "INCOMPLETE"},
        {"scope", "Derived public three-rod geometry/material with matched native logarithmic law and "
                  "consistent mass; not original Abaqus linear/lumped golden."}};
    if (independentExport)
    {
        for (const QString& field :
             {QStringLiteral("numerical_pass"), QStringLiteral("maximum_time_grid_difference"),
              QStringLiteral("maximum_normalized_q_v_a_errors"), QStringLiteral("level_normalized_q_v_a_limit"),
              QStringLiteral("maximum_Newmark_q_update_defect"), QStringLiteral("maximum_Newmark_v_update_defect"),
              QStringLiteral("q_update_absolute_limit"), QStringLiteral("v_update_absolute_limit")})
            result.remove(field);
        QJsonArray times;
        for (const auto& sample : samples)
            times.append(sample.time);
        result.insert("name", name);
        result.insert("solver", "factory AdaptiveNewmark / SolverNewmark step doubling");
        result.insert("factory_solver_name", solver->GetName());
        result.insert("factory_solver_type", "AdaptiveNewmark");
        result.insert("time_step_mode", "Adaptive");
        result.insert("adaptive_plan_sha256", adaptiveTrussPlanHash);
        result.insert("adaptive_settings", adaptiveRun);
        result.insert("rtol", adaptiveRun.value("relative_tolerance"));
        result.insert("q_atol", adaptiveRun.value("absolute_tolerance"));
        result.insert("v_atol", adaptiveRun.value("velocity_absolute_tolerance"));
        result.insert("initial_time_step", step->m_StepSize);
        result.insert("requested_step", step->m_StepSize);
        result.insert("minimum_time_step", adaptiveRun.value("minimum_time_step"));
        result.insert("maximum_time_step", adaptiveRun.value("maximum_time_step"));
        result.insert("minimum_accepted_step", statistics.minimumAcceptedTimeStep);
        result.insert("maximum_accepted_step", statistics.maximumAcceptedTimeStep);
        result.insert("duration", duration);
        result.insert("actual_times", times);
        result.insert("frame_count", static_cast<int>(samples.size()));
        result.insert("normalization_q_v_a", TrussArray(scales));
        result.insert("runtime_pass", passed);
        result.insert("numerical_acceptance_pending", true);
        result.insert("status", passed ? "COMPLETE_UNASSESSED" : "FAIL_RUNTIME_GATE");
        result.insert("reference_evaluation", "independent dense solution at actual accepted times required");
        if (rk)
        {
            result.remove("adaptive_plan_sha256");
            result.remove("beta");
            result.remove("gamma");
            result.remove("newton_tolerance");
            result.insert("rk_plan_sha256", rkTrussPlanHash);
            result.insert("solver", "factory RK4 / SolverRungeKutta4");
            result.insert("factory_solver_type", adaptive ? "AdaptiveRungeKutta4" : "RungeKutta4");
            result.insert("time_step_mode", adaptive ? "Adaptive" : "Fixed");
            result.insert("precision_counter_applicable", false);
            result.insert("mass_factorization_count", rungeKutta->GetMassFactorizationCount());
            result.insert("mass_assembly_count", rungeKutta->GetMassAssemblyCount());
            if (!adaptive)
            {
                result.remove("rtol");
                result.remove("q_atol");
                result.remove("v_atol");
                result.remove("adaptive_settings");
                result.insert("fixed_settings", adaptiveRun);
            }
        }
    }
    if (tssbn)
    {
        result.remove("adaptive_plan_sha256");
        result.remove("adaptive_settings");
        result.remove("beta");
        result.remove("gamma");
        const auto& parameters = tssbnSolver->GetParams();
        const auto policy = SolverNameSpace::SolverFactory::GetDynamicExecutionPolicy(requestedSolver);
        result.insert("solver", "factory SolverAdaptiveTSSBN; production default numerical policy");
        result.insert("factory_solver_name", solver->GetName());
        result.insert("factory_solver_type", referenceTssbn ? "ReferenceTSSBN" : "AdaptiveTSSBN");
        result.insert("factory_identity_asserted", true);
        result.insert("factory_solver_type_id", static_cast<int>(solver->GetType()));
        result.insert("tssbn_plan_sha256", tssbnTrussPlanHash);
        result.insert("effective_tssbn_parameters", TssbnTrussParameters(parameters));
        result.insert("requested_tssbn_settings", adaptiveRun);
        result.insert("tssbn_settings", adaptiveRun);
        result.insert("rtol", parameters.relativeTolerance);
        result.insert("q_atol", parameters.absoluteTolerance);
        result.insert("v_atol", parameters.velocityAbsoluteTolerance);
        result.insert("requested_v_atol", adaptiveRun.value("velocity_absolute_tolerance"));
        result.insert("endpoint_equilibrium_strict_diagnostic_pass", endpointEquilibriumDiagnostic);
        result.insert("endpoint_equilibrium_is_acceptance_gate", !referenceTssbn);
        result.insert("endpoint_acceleration_policy",
                      referenceTssbn ? "reference extrapolated acceleration" : "physical endpoint acceleration solve");
        result.insert("precision_counter_applicable", true);
        result.insert("precision_limited_stages", static_cast<double>(tssbnSolver->GetPrecisionLimitedStageCount()));
        result.insert("precision_candidate_solves",
                      static_cast<double>(tssbnSolver->GetPrecisionCandidateSolveCount()));
        result.insert("local_residual_delay_count", static_cast<double>(tssbnSolver->GetLocalResidualDelayCount()));
        result.insert("average_time_step", tssbnSolver->GetAverageTimeStep());
        result.insert("average_stage_newton_iterations", tssbnSolver->GetAverageStageNewtonIterations());
        result.insert("stage_iteration_counter_scope", "Production getter: total stage Newton iterations divided "
                                                       "by accepted outer steps; maximum is per stage.");
        result.insert("maximum_stage_newton_iterations", tssbnSolver->GetMaximumStageNewtonIterations());
        result.insert("factory_execution_policy",
                      QJsonObject{{"implemented", policy.implemented},
                                  {"requires_static_state", policy.requiresStaticState},
                                  {"requires_galloping", policy.requiresGalloping},
                                  {"initial_static_mode",
                                   policy.initialStaticMode == SolverNameSpace::InitialStaticMode::ReferenceWindless
                                       ? "ReferenceWindless"
                                       : "Configured"},
                                  {"publish_initial_static_results", policy.publishInitialStaticResults},
                                  {"supports_nonlinear_mpc", policy.supportsNonlinearMpc},
                                  {"supports_prescribed_boundary_motion", policy.supportsPrescribedBoundaryMotion}});
        result.insert("workflow_scope", "Direct production AnalysisStep::Init and SolverFactory::Create/Solve; "
                                        "not the galloping task orchestration required by ReferenceTSSBN policy.");
        if (!samples.empty())
            result.insert("actual_initial_free_q_v_a", QJsonObject{{"q", TrussArray(samples.front().q)},
                                                                   {"v", TrussArray(samples.front().v)},
                                                                   {"a", TrussArray(samples.front().a)}});
        result.insert("reference_constitutive_mode", false);
        result.insert("constitutive_scope", "Native logarithmic ElementTruss retained for both factory types.");
    }
    if (!WriteTrussJson(QDir(directory).filePath(QStringLiteral("result.json")), result))
        throw std::runtime_error("Cannot save actual-truss comparison result");
    QTextStream(stdout) << "newmark_truss case=" << name << " level=" << run.value("level").toInt()
                        << " status=" << result.value("status").toString() << Qt::endl;
    return result;
}

static QJsonObject RunTrussSuite(const QString& directory, const QString& selectedCase, int selectedLevel)
{
    const QDir inputs(TrussInputDirectory());
    const QJsonObject plan = ReadTrussJson(inputs.filePath("validation-plan.json"), trussPlanHash);
    const QJsonObject reference = ReadTrussJson(inputs.filePath("reference.json"), trussReferenceHash);
    if (plan.value("schema").toString() != "newmark-real-truss-plan-v1" ||
        reference.value("schema").toString() != "newmark-real-truss-reference-v1" ||
        reference.value("plan_sha256").toString() != QLatin1String(trussPlanHash) ||
        reference.value("cases").toArray().size() != 6)
        throw std::runtime_error("Real-truss frozen schema/plan/reference mismatch");
    for (const QString& name : {QStringLiteral("small"), QStringLiteral("nonlinear")})
        if (QString::fromLatin1(TrussFileHash(inputs.filePath(name + ".bdf"))) !=
            plan.value("input_sha256").toObject().value(name).toString())
            throw std::runtime_error("Real-truss model input SHA256 differs from the frozen plan");
    if (!QDir().mkpath(directory) || QFileInfo::exists(QDir(directory).filePath("summary.json")))
        throw std::runtime_error("Use a fresh real-truss suite output directory");
    QJsonArray results;
    bool passed = true;
    std::map<QString, std::vector<Eigen::Vector3d>> errors;
    for (const QJsonValue& value : reference.value("cases").toArray())
    {
        const QJsonObject run = value.toObject();
        const QString name = run.value("case").toString();
        const int level = run.value("level").toInt();
        if ((!selectedCase.isEmpty() && selectedCase != name) || (selectedLevel >= 0 && selectedLevel != level))
            continue;
        QJsonObject result;
        try
        {
            result = RunActualTruss(run, plan, QDir(directory).filePath(name + QStringLiteral("-level%1").arg(level)));
            errors[name].push_back(TrussVector(result.value("maximum_normalized_q_v_a_errors")));
        }
        catch (const std::exception& exception)
        {
            result = {{"case", name},
                      {"level", level},
                      {"status", "EXCEPTION"},
                      {"numerical_pass", false},
                      {"exception", QString::fromLocal8Bit(exception.what())}};
        }
        passed = passed && result.value("numerical_pass").toBool();
        results.append(result);
    }
    QJsonArray orders;
    if (selectedLevel < 0)
    {
        for (const auto& [name, values] : errors)
        {
            if (values.size() != 3)
            {
                passed = false;
                continue;
            }
            for (int level = 1; level < 3; ++level)
            {
                Eigen::Vector3d order;
                for (int component = 0; component < 3; ++component)
                    order[component] = std::log2(values[level - 1][component] / values[level][component]);
                const bool orderPass = order.allFinite() && order.minCoeff() >= 1.8 && order.maxCoeff() <= 2.2;
                passed = passed && orderPass;
                orders.append(QJsonObject{{"case", name},
                                          {"coarse_level", level - 1},
                                          {"fine_level", level},
                                          {"q_v_a_observed_orders", TrussArray(order)},
                                          {"pass", orderPass}});
            }
        }
    }
    const int expected = (selectedCase.isEmpty() ? 2 : 1) * (selectedLevel < 0 ? 3 : 1);
    passed = passed && results.size() == expected && (selectedLevel >= 0 || orders.size() == expected / 3 * 2);
    const QJsonObject summary{
        {"schema", "newmark-real-truss-comparison-v1"},
        {"plan_sha256", trussPlanHash},
        {"reference_sha256", trussReferenceHash},
        {"results", results},
        {"refinement_orders", orders},
        {"all_cases_numerical_pass", passed},
        {"full_six_case_acceptance", passed && results.size() == 6 && orders.size() == 4},
        {"status", passed ? "PASS_FROZEN_GATES" : "FAIL_FROZEN_GATES"},
        {"scope", "Actual production Node/Constraint/ElementTruss/AnalysisStep/Factory/Newmark assembly and full q/v/a "
                  "history; no beam, cable, finite rotation or aero certification."}};
    if (!WriteTrussJson(QDir(directory).filePath(QStringLiteral("summary.json")), summary))
        throw std::runtime_error("Cannot write real-truss suite summary");
    return summary;
}

static QString TrussOption(const QStringList& arguments, const QString& option)
{
    const int position = arguments.indexOf(option);
    if (position < 0)
        return {};
    if (position + 1 >= arguments.size() || arguments[position + 1].startsWith("--"))
        throw std::runtime_error("Real-truss option requires a value");
    return arguments[position + 1];
}

static QJsonObject RunAlternateTrussSuite(const QString& directory, const QString& selectedCase, int selectedLevel,
                                          bool rk = false, bool adaptiveRk = true,
                                          SolverNameSpace::SolverType tssbnType = SolverNameSpace::SolverType::Newmark)
{
    const bool tssbn = tssbnType == SolverNameSpace::SolverType::AdaptiveTSSBN ||
                       tssbnType == SolverNameSpace::SolverType::ReferenceTSSBN;
    const QString adaptivePath =
        QDir(TestPaths::TestRoot())
            .filePath(tssbn ? QStringLiteral("TestModels/Support/AlgorithmPlans/TssbnDynamics/truss-cable-plan.json")
                      : rk  ? QStringLiteral("TestModels/Support/AlgorithmPlans/RungeKuttaDynamics/truss-cable-plan.json")
                            : QStringLiteral("TestModels/Support/AlgorithmPlans/AdaptiveNewmark/truss-cable-plan.json"));
    const QJsonObject adaptivePlan = ReadTrussJson(adaptivePath, tssbn ? tssbnTrussPlanHash
                                                                 : rk  ? rkTrussPlanHash
                                                                       : adaptiveTrussPlanHash);
    if (adaptivePlan.value("schema").toString() != (tssbn ? "tssbn-real-elements-plan-v1"
                                                    : rk  ? "runge-kutta-truss-cable-plan-v1"
                                                          : "adaptive-newmark-truss-cable-plan-v1") ||
        !adaptivePlan.value("frozen_before_candidate_execution").toBool() ||
        adaptivePlan.value("families").toArray().size() != 4)
        throw std::runtime_error("Invalid frozen adaptive real-element plan");
    const QDir inputs(TrussInputDirectory());
    const QJsonObject plan = ReadTrussJson(inputs.filePath("validation-plan.json"), trussPlanHash);
    const QJsonObject reference = ReadTrussJson(inputs.filePath("reference.json"), trussReferenceHash);
    if (!QDir().mkpath(directory) || QFileInfo::exists(QDir(directory).filePath("summary.json")) ||
        !QFile::copy(adaptivePath, QDir(directory).filePath("truss-cable-plan.json")))
        throw std::runtime_error("Use a fresh adaptive real-truss output directory");
    QJsonArray results;
    bool runtimePassed = true;
    for (const auto familyValue : adaptivePlan.value("families").toArray())
    {
        const auto family = familyValue.toObject();
        const QString name = family.value("name").toString();
        if (family.value("element").toString() != "truss" || (!selectedCase.isEmpty() && name != selectedCase))
            continue;
        if (TrussFileHash(inputs.filePath(name + ".bdf")) != family.value("input_sha256").toString().toLatin1())
            throw std::runtime_error("Adaptive real-truss BDF SHA256 differs from frozen input");
        QJsonObject source;
        for (const auto referenceValue : reference.value("cases").toArray())
            if (referenceValue.toObject().value("case").toString() == name)
            {
                source = referenceValue.toObject();
                break;
            }
        const QString runKey = rk ? (adaptiveRk ? "adaptive_runs" : "fixed_runs") : "runs";
        if (source.isEmpty() || family.value(runKey).toArray().size() != 3)
            throw std::runtime_error("Missing adaptive real-truss family/source parameters");
        if (tssbn && family.value("duration").toDouble() != source.value("duration").toDouble())
            throw std::runtime_error("Frozen TSSBN and physical source durations differ");
        source.remove("rows");
        for (const auto adaptiveValue : family.value(runKey).toArray())
        {
            const auto adaptiveRun = adaptiveValue.toObject();
            const int level = adaptiveRun.value("level").toInt();
            if (selectedLevel >= 0 && selectedLevel != level)
                continue;
            source.insert("level", level);
            source.insert("step", adaptiveRun.value("initial_time_step"));
            QJsonObject result;
            try
            {
                result = RunActualTruss(source, plan,
                                        QDir(directory).filePath(QStringLiteral("%1-level%2").arg(name).arg(level)),
                                        adaptiveRun,
                                        tssbn ? tssbnType
                                        : rk  ? (adaptiveRk ? SolverNameSpace::SolverType::AdaptiveRungeKutta4
                                                            : SolverNameSpace::SolverType::RungeKutta4)
                                              : SolverNameSpace::SolverType::Newmark);
            }
            catch (const std::exception& exception)
            {
                result = {{"case", name},
                          {"name", name},
                          {"level", level},
                          {"runtime_pass", false},
                          {"status", "SETUP_OR_RUNTIME_EXCEPTION"},
                          {"exception", QString::fromLocal8Bit(exception.what())}};
            }
            runtimePassed = runtimePassed && result.value("runtime_pass").toBool();
            results.append(result);
        }
    }
    const int expected = (selectedCase.isEmpty() ? 2 : 1) * (selectedLevel < 0 ? 3 : 1);
    runtimePassed = runtimePassed && results.size() == expected;
    QJsonObject summary{{"schema", "adaptive-newmark-real-truss-export-v1"},
                        {"adaptive_plan_sha256", adaptiveTrussPlanHash},
                        {"source_plan_sha256", trussPlanHash},
                        {"source_reference_sha256", trussReferenceHash},
                        {"results", results},
                        {"all_runtime_pass", runtimePassed},
                        {"numerical_acceptance_pending", true},
                        {"status", runtimePassed ? "COMPLETE_UNASSESSED" : "FAIL_RUNTIME_GATE"},
                        {"comparison_criteria", adaptivePlan.value("comparison_criteria")},
                        {"scope", "Actual production logarithmic ElementTruss and factory AdaptiveNewmark; "
                                  "independent dense all-time q/v/a comparison remains required."}};
    if (rk)
    {
        summary.remove("adaptive_plan_sha256");
        summary.insert("rk_plan_sha256", rkTrussPlanHash);
        summary.insert("schema", "runge-kutta-real-truss-export-v1");
        summary.insert("factory_solver_type", adaptiveRk ? "AdaptiveRungeKutta4" : "RungeKutta4");
        summary.insert("scope", "Actual production logarithmic ElementTruss and factory RK4; independent dense "
                                "all-time q/v/a comparison and declared refinement gates remain required.");
    }
    if (tssbn)
    {
        summary.remove("adaptive_plan_sha256");
        summary.insert("tssbn_plan_sha256", tssbnTrussPlanHash);
        summary.insert("schema", "tssbn-real-truss-export-v1");
        summary.insert("factory_solver_type",
                       tssbnType == SolverNameSpace::SolverType::ReferenceTSSBN ? "ReferenceTSSBN" : "AdaptiveTSSBN");
        summary.insert("algorithm_policies", adaptivePlan.value("algorithm_policies"));
        summary.insert("scope", "Actual production elements and factory TSSBN default numerical policy; direct "
                                "Init/Solve diagnostic, not complete galloping workflow. Independent dense "
                                "q/v/a and tolerance refinement remain required; completion is not accuracy PASS.");
    }
    if (!WriteTrussJson(QDir(directory).filePath("summary.json"), summary))
        throw std::runtime_error("Cannot save adaptive real-truss summary");
    return summary;
}

/** @brief 真实三杆 Newmark 微幅及明显非线性动力的独立全时程与二阶收敛验收。
 * @details 输入公开Abaqus三T3D2杆几何/材料：3x10、A2、E5、rho1/90、根固定、横向约束。
 *          本题显式派生为生产默认对数应变/体积不变及一致质量；阶跃末端力1e-6与1，t0至10。
 *          实际读取BDF并运行Node/Constraint/ElementTruss/AnalysisStep及工厂固定Newmark，h=.01/.005/.0025。
 *          参考为同本构三自由度DOP853双容差独立解；微幅另核线性模态闭式，非线性参考最大应变约32%。
 *          每帧核全部q/v/a、初加速度、真实步列、零拒步/precision、轴力/反力、平衡及Newmark更新。
 *          最细归一q/v/a各<=.002，两次减半阶次各1.8至2.2；门与参考SHA在候选运行前冻结。
 *          此为公开结构的本构/质量匹配派生题，不宣称原线性集中质量算例逐数值复现，也不认证索梁或气动力。
 * @author   yangqiuyou
 * @date     2026.10.19
 */
TEST(TrussVibration, NewmarkFixed)
{
    QTemporaryDir output;
    ASSERT_TRUE(output.isValid());
    output.setAutoRemove(false);
    RecordProperty("temporary_output_directory", output.path().toStdString());
    RecordProperty("plan_sha256", trussPlanHash);
    RecordProperty("reference_sha256", trussReferenceHash);
    QJsonObject summary;
    ASSERT_NO_THROW(summary = RunTrussSuite(output.path(), {}, -1));
    EXPECT_TRUE(summary.value("full_six_case_acceptance").toBool())
        << QJsonDocument(summary).toJson(QJsonDocument::Compact).toStdString();
    output.setAutoRemove(!HasFailure());
}
}

std::optional<int> VerifyNewmarkTruss(const QStringList& arguments)
{
    const bool referenceTssbn = arguments.contains(QStringLiteral("--verify-reference-tssbn-truss"));
    const bool tssbn = referenceTssbn || arguments.contains(QStringLiteral("--verify-tssbn-truss"));
    const auto tssbnType =
        referenceTssbn ? SolverNameSpace::SolverType::ReferenceTSSBN : SolverNameSpace::SolverType::AdaptiveTSSBN;
    const bool adaptiveRk = arguments.contains(QStringLiteral("--verify-adaptive-rk4-truss"));
    const bool rk = adaptiveRk || arguments.contains(QStringLiteral("--verify-rk4-truss"));
    const bool adaptive = adaptiveRk || arguments.contains(QStringLiteral("--verify-adaptive-newmark-truss"));
    if (!tssbn && !rk && !adaptive && !arguments.contains(QStringLiteral("--verify-newmark-truss")))
        return std::nullopt;
    try
    {
        int entries = 0;
        for (const QString& flag :
             {QStringLiteral("--verify-newmark-truss"), QStringLiteral("--verify-adaptive-newmark-truss"),
              QStringLiteral("--verify-rk4-truss"), QStringLiteral("--verify-adaptive-rk4-truss"),
              QStringLiteral("--verify-tssbn-truss"), QStringLiteral("--verify-reference-tssbn-truss")})
            entries += arguments.contains(flag) ? 1 : 0;
        if (entries != 1)
            throw std::runtime_error("Choose exactly one real-truss dynamics entry");
        const QString flag = tssbn ? (referenceTssbn ? "--verify-reference-tssbn-truss" : "--verify-tssbn-truss")
                             : rk  ? (adaptiveRk ? "--verify-adaptive-rk4-truss" : "--verify-rk4-truss")
                                   : (adaptive ? "--verify-adaptive-newmark-truss" : "--verify-newmark-truss");
        const QString directory = QFileInfo(TrussOption(arguments, flag)).absoluteFilePath();
        const QString name = TrussOption(arguments, "--truss-case");
        if (!name.isEmpty() && name != "small" && name != "nonlinear")
            throw std::runtime_error("Real-truss case must be small or nonlinear");
        const QString levelText = TrussOption(arguments, "--truss-level");
        bool valid = levelText.isEmpty();
        const int level = levelText.isEmpty() ? -1 : levelText.toInt(&valid);
        if (!valid || level < -1 || level > 2)
            throw std::runtime_error("Real-truss level must be0,1,2");
        const QJsonObject summary =
            (tssbn || adaptive || rk) ? RunAlternateTrussSuite(directory, name, level, rk, adaptiveRk,
                                                               tssbn ? tssbnType : SolverNameSpace::SolverType::Newmark)
                                      : RunTrussSuite(directory, name, level);
        return summary.value((tssbn || adaptive || rk) ? "all_runtime_pass" : "all_cases_numerical_pass").toBool() ? 0
                                                                                                                   : 4;
    }
    catch (const std::exception& exception)
    {
        QTextStream(stderr) << "newmark_truss error=" << exception.what() << Qt::endl;
        return 2;
    }
}
