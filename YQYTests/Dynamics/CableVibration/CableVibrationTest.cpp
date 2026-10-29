#include "Dynamics/CableVibration/CableVibrationTest.h"

#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "DataStructure/Structure/StructureData.h"
#include "Solver/Dynamic/Integrators/SolverNewmark.h"
#include "Solver/Dynamic/Integrators/SolverAdaptiveTSSBN.h"
#include "Solver/Dynamic/Integrators/SolverRungeKutta4.h"
#include "Solver/Factory/SolverFactory.h"
#include "Support/SolveModel.h"
#include "Support/TestPaths.h"

#include <Eigen/Cholesky>
#include <QCryptographicHash>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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
using NodeState = std::array<double, 9>;

const char* frozenPlanHash = "80e8d48ddf68fed6585b755d7d44e94233eeb93c88e010abe028b823e5955b83";
const char* adaptiveCablePlanHash = "7fc6e2aa65d67611558f53769572c35223971b1595703233874d4da7b4cd3811";
const char* rkCablePlanHash = "270a55aa339a8382fbb1c1b2932af2e8ee9a471120f5951639209b7c0418ba09";
const char* tssbnCablePlanHash = "faa142836562e96d64c6cadf656594b7bac35a734becd4b324ab0255213f5d70";

struct CableFrame
{
    double time = 0.0;
    std::vector<NodeState> nodes;
};

static double Number(const QJsonObject& object, const char* key)
{
    const QJsonValue value = object.value(QLatin1String(key));
    if (!value.isDouble() || !std::isfinite(value.toDouble()))
        throw std::runtime_error(std::string("索参考缺少有限数值: ") + key);
    return value.toDouble();
}

static QString InputDirectory()
{
    return QDir(TestPaths::TestRoot()).filePath(QStringLiteral("TestModels/Dynamics/CableVibration"));
}

static QByteArray HashFile(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error((QStringLiteral("无法读取索验证文件: ") + path).toStdString());
    return QCryptographicHash::hash(file.readAll(), QCryptographicHash::Sha256).toHex();
}

static void WriteJson(const QString& path, const QJsonObject& object)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate) ||
        file.write(QJsonDocument(object).toJson(QJsonDocument::Indented)) <= 0)
        throw std::runtime_error("无法写入索验证结果");
}

static QJsonObject ReadPlan()
{
    const QString path = QDir(InputDirectory()).filePath(QStringLiteral("validation-plan.json"));
    if (HashFile(path) != frozenPlanHash)
        throw std::runtime_error("索验证计划 SHA256 与运行前冻结版本不同");
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("缺少索验证计划");
    const QJsonObject plan = QJsonDocument::fromJson(file.readAll()).object();
    if (plan.value("schema").toString() != "newmark-real-cable-validation-v1" ||
        !plan.value("frozen_before_candidate_execution").toBool() || plan.value("cases").toArray().size() != 2)
        throw std::runtime_error("索验证计划内容不完整");
    return plan;
}

static std::vector<CableFrame> ReadReference(const QJsonObject& specification)
{
    const QString path = QDir(InputDirectory()).filePath(specification.value("reference_csv").toString());
    if (HashFile(path) != specification.value("reference_sha256").toString().toLatin1())
        throw std::runtime_error("独立索参考 CSV SHA256 不一致");
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text) ||
        file.readLine().trimmed() != "time,node_id,ux,uy,uz,vx,vy,vz,ax,ay,az")
        throw std::runtime_error("独立索参考 CSV 列不完整");
    const int nodeCount = specification.value("element_count").toInt() + 1;
    std::vector<CableFrame> frames;
    while (!file.atEnd())
    {
        const auto fields = file.readLine().trimmed().split(',');
        if (fields.size() != 11)
            throw std::runtime_error("独立索参考 CSV 行不完整");
        bool valid = false;
        const double time = fields[0].toDouble(&valid);
        if (!valid || !std::isfinite(time))
            throw std::runtime_error("独立索参考时间无效");
        const int nodeId = fields[1].toInt(&valid);
        if (!valid || nodeId < 1 || nodeId > nodeCount)
            throw std::runtime_error("独立索参考节点编号无效");
        if (frames.empty() || time != frames.back().time)
        {
            if (!frames.empty() && (time <= frames.back().time || frames.back().nodes.size() != nodeCount))
                throw std::runtime_error("独立索参考时间或帧节点不完整");
            frames.push_back({time, {}});
        }
        if (nodeId != static_cast<int>(frames.back().nodes.size()) + 1)
            throw std::runtime_error("独立索参考节点顺序不一致");
        NodeState state{};
        for (int i = 0; i < 9; ++i)
        {
            state[i] = fields[i + 2].toDouble(&valid);
            if (!valid || !std::isfinite(state[i]))
                throw std::runtime_error("独立索参考状态非有限");
        }
        frames.back().nodes.push_back(state);
    }
    if (frames.size() < 2 || frames.front().time != 0.0 || frames.back().nodes.size() != nodeCount ||
        std::abs(frames.back().time - Number(specification, "duration")) > 1.0e-12)
        throw std::runtime_error("独立索参考未覆盖完整时程");
    return frames;
}

static bool AllGates(const QJsonObject& gates)
{
    for (const auto value : gates)
        if (!value.isBool() || !value.toBool())
            return false;
    return !gates.isEmpty();
}

class CableStaticStep final : public AnalysisStep
{
public:
    void OnStepCompleted(double time) override
    {
        lastTime = time;
    }
    bool ShouldPreferGpuLinearSolver() const override
    {
        return false;
    }
    double lastTime = 0.0;
};

class CableProbeStep final : public AnalysisStep
{
public:
    CableProbeStep(const QString& output, const std::vector<CableFrame>& reference, double timeout,
                   bool adaptive = false)
        : file_(output)
        , stream_(&file_)
        , reference_(reference)
        , timeout_(timeout)
        , adaptive_(adaptive)
    {
        if (!file_.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
            throw std::runtime_error("无法写入真实索节点时程");
        stream_.setRealNumberPrecision(17);
        stream_ << "time,node_id,ux,uy,uz,vx,vy,vz,ax,ay,az,accepted_dt,equilibrium_norm,min_tension\n";
        timer_.start();
    }

    bool ShouldPreferGpuLinearSolver() const override
    {
        return false;
    }
    bool IsCancellationRequested() const override
    {
        return timer_.elapsed() > timeout_ * 1000.0;
    }

    void ReportProgress(double progress, const QString& message) override
    {
        Q_UNUSED(progress);
        lastMessage = message;
    }

    void RecordStepIterations(double time, int iterations) override
    {
        Q_UNUSED(time);
        maximumIterations = std::max(maximumIterations, iterations);
        totalIterations += iterations;
    }

    void OnStepCompleted(double time) override
    {
        // 只替代结果写出；节点、单元、组装、Newton 和积分均使用生产实现。
        SolverNameSpace::SystemEvaluationResult evaluation;
        EvaluateSystem(SolverNameSpace::SystemEvaluationRequest::TrialResidual(time), evaluation);
        Vec residual;
        ComputeResidual(evaluation.externalForce, residual);
        const double residualNorm = residual.norm();
        diagnosticFinite = diagnosticFinite && std::isfinite(time) && residual.allFinite() &&
                           evaluation.externalForce.allFinite() && std::isfinite(residualNorm);
        maximumResidual = std::max(maximumResidual, residualNorm);
        double frameMinimumTension = std::numeric_limits<double>::infinity();
        for (const auto& [id, element] : GetStructure()->m_Elements)
        {
            Q_UNUSED(id);
            const auto property = element->m_pProperty.lock();
            const double tension = element->m_Stress * property->m_pSection.lock()->m_Area;
            diagnosticFinite = diagnosticFinite && std::isfinite(tension);
            frameMinimumTension = std::min(frameMinimumTension, tension);
        }
        diagnosticFinite = diagnosticFinite && std::isfinite(frameMinimumTension);
        minimumTension = std::min(minimumTension, frameMinimumTension);
        CableFrame frame{time, {}};
        for (const auto& [id, node] : GetStructure()->m_Nodes)
        {
            NodeState state{};
            for (int d = 0; d < 3; ++d)
            {
                state[d] = node->m_Displacement[d];
                state[d + 3] = node->m_Velocity[d];
                state[d + 6] = node->m_Acceleration[d];
            }
            for (double value : state)
                finite = finite && std::isfinite(value);
            diagnosticFinite = diagnosticFinite && std::isfinite(node->m_Displacement[3]) &&
                               std::isfinite(node->m_Velocity[3]) && std::isfinite(node->m_Acceleration[3]);
            maximumLockedTwist = std::max({maximumLockedTwist, std::abs(node->m_Displacement[3]),
                                           std::abs(node->m_Velocity[3]), std::abs(node->m_Acceleration[3])});
            frame.nodes.push_back(state);
            stream_ << time << ',' << id;
            for (double value : state)
                stream_ << ',' << value;
            stream_ << ',' << (history.empty() ? 0.0 : time - history.back().time) << ',' << residualNorm << ','
                    << frameMinimumTension << '\n';
        }
        if (!adaptive_)
        {
            const auto near = std::lower_bound(reference_.cbegin(), reference_.cend(), time,
                                               [](const CableFrame& f, double t)
                                               {
                                                   return f.time < t;
                                               });
            auto selected = near;
            if (near == reference_.cend() ||
                (near != reference_.cbegin() && std::abs((near - 1)->time - time) < std::abs(near->time - time)))
                selected = near - 1;
            if (selected == reference_.cend() || std::abs(selected->time - time) > 1.0e-11 ||
                selected->nodes.size() != frame.nodes.size())
                ++unmatchedReferenceFrames;
            else
                for (std::size_t n = 0; n < frame.nodes.size(); ++n)
                    for (int d = 0; d < 9; ++d)
                        maximumErrors[d / 3] =
                            std::max(maximumErrors[d / 3], std::abs(frame.nodes[n][d] - selected->nodes[n][d]));
        }
        if (!history.empty())
        {
            const CableFrame& previous = history.back();
            const double h = time - previous.time;
            increasing = increasing && h > 0.0;
            maximumStepDeviation = std::max(maximumStepDeviation, std::abs(h - m_StepSize));
            if (!adaptive_)
                for (std::size_t n = 0; n < frame.nodes.size(); ++n)
                    for (int d = 0; d < 3; ++d)
                    {
                        const auto& a = previous.nodes[n];
                        const auto& b = frame.nodes[n];
                        maximumDisplacementIdentity =
                            std::max(maximumDisplacementIdentity,
                                     std::abs(b[d] - a[d] - h * a[d + 3] - .25 * h * h * (a[d + 6] + b[d + 6])));
                        maximumVelocityIdentity = std::max(
                            maximumVelocityIdentity, std::abs(b[d + 3] - a[d + 3] - .5 * h * (a[d + 6] + b[d + 6])));
                    }
        }
        history.push_back(std::move(frame));
    }

    std::vector<CableFrame> history;
    std::array<double, 3> maximumErrors{0.0, 0.0, 0.0};
    double maximumResidual = 0.0;
    double minimumTension = std::numeric_limits<double>::infinity();
    double maximumDisplacementIdentity = 0.0;
    double maximumVelocityIdentity = 0.0;
    double maximumStepDeviation = 0.0;
    double maximumLockedTwist = 0.0;
    int unmatchedReferenceFrames = 0;
    int maximumIterations = 0;
    long long totalIterations = 0;
    bool finite = true;
    bool diagnosticFinite = true;
    bool increasing = true;
    QString lastMessage;

private:
    QFile file_;
    QTextStream stream_;
    const std::vector<CableFrame>& reference_;
    QElapsedTimer timer_;
    double timeout_;
    bool adaptive_;
};

static void AddConstraint(StructureData& model, int id, int nodeId, int direction, double value)
{
    auto constraint = std::make_shared<Constraint>();
    constraint->m_Id = id;
    constraint->m_pNode = model.m_Nodes.at(nodeId);
    constraint->m_Direction = static_cast<EnumKeyword::Direction>(direction);
    constraint->m_Value = value;
    model.m_Constraint.emplace(id, std::move(constraint));
}

static double MatrixError(const SpMat& actual, const QJsonArray& expected, const QJsonArray& dofs,
                          const AnalysisStep& step, const StructureData& model)
{
    if (actual.rows() != dofs.size() || expected.size() != dofs.size())
        throw std::runtime_error("真实索自由度与独立矩阵维数不一致");
    std::vector<int> order;
    for (const auto entry : dofs)
    {
        const QJsonArray descriptor = entry.toArray();
        const int index = model.m_Nodes.at(descriptor[0].toInt())->m_DOF[descriptor[1].toInt()] - step.m_nFixed;
        if (index < 0 || index >= step.m_nFree)
            throw std::runtime_error("真实索自由度映射无效");
        order.push_back(index);
    }
    Eigen::MatrixXd reordered(actual.rows(), actual.cols());
    Eigen::MatrixXd oracle(actual.rows(), actual.cols());
    for (int i = 0; i < actual.rows(); ++i)
    {
        const QJsonArray row = expected[i].toArray();
        if (row.size() != actual.cols())
            throw std::runtime_error("独立索矩阵行长度无效");
        for (int j = 0; j < actual.cols(); ++j)
        {
            oracle(i, j) = row[j].toDouble();
            reordered(i, j) = actual.coeff(order[i], order[j]);
        }
    }
    return (reordered - oracle).norm() / oracle.norm();
}

static void SetAdaptiveCableControls(AnalysisStep& step, const QJsonObject& run)
{
    step.m_DynamicSolverType = SolverNameSpace::SolverType::AdaptiveNewmark;
    step.m_StepSize = Number(run, "initial_time_step");
    auto& settings = step.m_AdaptiveTimeStep;
    settings.minimumTimeStep = Number(run, "minimum_time_step");
    settings.maximumTimeStep = Number(run, "maximum_time_step");
    settings.relativeTolerance = Number(run, "relative_tolerance");
    settings.absoluteTolerance = Number(run, "absolute_tolerance");
    settings.velocityRelativeTolerance = Number(run, "velocity_relative_tolerance");
    settings.velocityAbsoluteTolerance = Number(run, "velocity_absolute_tolerance");
    settings.includeVelocityInError = true;
    settings.safetyFactor = .9;
    settings.minimumScaleFactor = .2;
    settings.maximumGrowthFactor = 2.0;
    settings.solveFailureFactor = .5;
    settings.maximumRejectedAttempts = 32;
}

// 只覆盖冻结计划中的步长和误差尺度，其余控制器设置保留生产默认值。
static void SetTssbnCableControls(AnalysisStep& step, const QJsonObject& run, SolverNameSpace::SolverType type)
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

static QJsonObject TssbnCableParameters(const SolverNameSpace::SolverAdaptiveTSSBN::Params& p)
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

static QJsonObject RunCableLevel(const QJsonObject& plan, const QJsonObject& specification, int level,
                                 const QString& directory, double timeout, const QJsonObject& adaptiveRun = {},
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
    const QString name = specification.value("name").toString();
    const bool nonlinear = name == "nonlinear";
    const QJsonObject criteria = plan.value("criteria").toObject();
    const QJsonArray dofs = specification.value("free_dofs").toArray();
    const auto reference = ReadReference(specification);
    const QString inputName = specification.value("input_file").toString();
    if (HashFile(QDir(InputDirectory()).filePath(inputName)) !=
        specification.value("input_sha256").toString().toLatin1())
        throw std::runtime_error("真实索 BDF 输入 SHA256 不一致");
    QString error;
    auto model = SolveModel::Read(QStringLiteral("Dynamics/CableVibration/") + inputName, error);
    if (!model)
        throw std::runtime_error(error.toStdString());
    const int elements = specification.value("element_count").toInt();
    if (model->m_Elements.size() != elements)
        throw std::runtime_error("真实索 BDF 单元数不一致");
    for (const auto& [id, element] : model->m_Elements)
    {
        Q_UNUSED(id);
        if (!std::dynamic_pointer_cast<ElementCable>(element))
            throw std::runtime_error("真实索测试出现非 ElementCable 单元");
    }
    if (!nonlinear)
        for (int nodeId = 2; nodeId <= elements; ++nodeId)
            AddConstraint(*model, 1000 + nodeId, nodeId, 1, 0.0);
    CableStaticStep initial;
    initial.m_Id = 1;
    initial.m_Type = EnumKeyword::StepType::STATIC;
    initial.m_Time = 1.0;
    initial.m_StepSize = .1;
    initial.m_Tolerance = 1.0e-11;
    initial.m_MaxIterations = 32;
    initial.SetAssemblyThreadCountOverride(1);
    initial.SetPardisoThreadCountOverride(1);
    initial.SetStructure(model);
    initial.Init();
    for (const auto& [id, element] : model->m_Elements)
    {
        Q_UNUSED(id);
        element->Get_L0();
    }
    // 仅受拉索在零应变处没有轴向切线；给静力 Newton 一个正应变初猜，目标荷载和最终平衡不变。
    if (!nonlinear)
        for (const auto& [id, node] : model->m_Nodes)
        {
            Q_UNUSED(id);
            node->SetAdditiveDisplacement(0, 1.0e-8 * node->m_X);
        }
    auto staticSolver = SolverNameSpace::SolverFactory::Create(initial);
    const bool staticSolved = staticSolver && staticSolver->Solve(initial, initial.m_Time);
    double staticError = 0.0;
    const QJsonArray equilibriumX = specification.value("axial_equilibrium_displacements").toArray();
    for (int i = 0; i <= elements; ++i)
        staticError =
            std::max(staticError, std::abs(model->m_Nodes.at(i + 1)->m_Displacement[0] - equilibriumX[i].toDouble()));
    if (!staticSolved || std::abs(initial.lastTime - 1.0) > 1.0e-12 ||
        staticError > Number(criteria, "static_displacement_absolute_limit_in"))
        throw std::runtime_error("真实索前置静力未完成或预伸长不符");
    if (!nonlinear)
    {
        for (int nodeId = 2; nodeId <= elements; ++nodeId)
            model->m_Constraint.erase(1000 + nodeId);
        for (int nodeId = 2; nodeId <= elements + 1; ++nodeId)
            AddConstraint(*model, 2000 + nodeId, nodeId, 0, model->m_Nodes.at(nodeId)->m_Displacement[0]);
    }
    const QString runDirectory = QDir(directory).filePath(QStringLiteral("%1-level%2").arg(name).arg(level));
    if (!QDir().mkpath(runDirectory))
        throw std::runtime_error("无法创建真实索独立运行目录");
    CableProbeStep step(QDir(runDirectory).filePath("response.csv"), reference, timeout, independentExport);
    step.m_Id = 1;
    step.m_Type = EnumKeyword::StepType::DYNAMIC;
    step.m_DynamicSolverType = SolverNameSpace::SolverType::Newmark;
    step.isDynamic = true;
    step.SolverNameSpace::IAnalysisModel::isDynamic = true;
    step.m_Time = Number(specification, "duration");
    const int stepsPerPeriod = specification.value("levels_steps_per_period").toArray()[level].toInt();
    step.m_StepSize = Number(specification, "period") / stepsPerPeriod;
    if (tssbn)
        SetTssbnCableControls(step, adaptiveRun, requestedSolver);
    else if (adaptive)
        SetAdaptiveCableControls(step, adaptiveRun);
    if (rk)
    {
        step.m_DynamicSolverType = requestedSolver;
        step.m_StepSize = Number(adaptiveRun, "initial_time_step");
    }
    step.m_Tolerance = Number(plan, "candidate_newton_tolerance");
    step.m_MaxIterations = plan.value("candidate_maximum_newton_evaluations").toInt();
    step.SetAssemblyThreadCountOverride(1);
    step.SetPardisoThreadCountOverride(1);
    step.SetStructure(model);
    step.SetInitializeFromCurrentState(true);
    step.Init();
    if (tssbn)
        for (const auto& [id, element] : model->m_Elements)
        {
            Q_UNUSED(id);
            if (std::dynamic_pointer_cast<ElementCable>(element)->IsReferenceTssbnConstitutiveMode() != referenceTssbn)
                throw std::runtime_error("Production Cable Init did not preserve factory constitutive policy");
        }
    SpMat stiffness, mass;
    step.Assemble_Matrix(stiffness, false);
    step.AssembleMassMatrix(mass);
    const double massError = MatrixError(mass, specification.value("mass_matrix").toArray(), dofs, step, *model);
    const double equilibriumStiffnessError =
        MatrixError(stiffness, specification.value("equilibrium_stiffness_matrix").toArray(), dofs, step, *model);
    const QJsonArray initialQ = specification.value("initial_q").toArray();
    for (int i = 0; i < dofs.size(); ++i)
    {
        const auto descriptor = dofs[i].toArray();
        model->m_Nodes.at(descriptor[0].toInt())
            ->SetAdditiveDisplacement(descriptor[1].toInt(), initialQ[i].toDouble());
    }
    step.Assemble_Matrix(stiffness, false);
    const double initialStiffnessError =
        MatrixError(stiffness, specification.value("initial_stiffness_matrix").toArray(), dofs, step, *model);
    SolverNameSpace::SystemEvaluationResult evaluation;
    step.EvaluateSystem(SolverNameSpace::SystemEvaluationRequest::ExplicitDynamics(0.0), evaluation);
    Vec residual;
    step.ComputeResidual(evaluation.externalForce, residual);
    const Vec acceleration = Eigen::MatrixXd(evaluation.mass).ldlt().solve(residual);
    if (!acceleration.allFinite())
        throw std::runtime_error("真实索初始质量方程无有限解");
    step.SetTrialKinematics(Vec::Zero(step.m_nFree), acceleration);
    step.CommitState();
    const QJsonArray initialA = specification.value("initial_a").toArray();
    const QJsonArray scales = specification.value("normalization_q_v_a").toArray();
    double initialAccelerationError = 0.0;
    for (int i = 0; i < dofs.size(); ++i)
    {
        const auto descriptor = dofs[i].toArray();
        const double actual = model->m_Nodes.at(descriptor[0].toInt())->m_Acceleration[descriptor[1].toInt()];
        initialAccelerationError =
            std::max(initialAccelerationError, std::abs(actual - initialA[i].toDouble()) / scales[2].toDouble());
    }
    step.OnStepCompleted(0.0);
    const auto solver = SolverNameSpace::SolverFactory::Create(step);
    const auto* newmark = dynamic_cast<const SolverNameSpace::SolverNewmark*>(solver.get());
    const auto* rungeKutta = dynamic_cast<const SolverNameSpace::SolverRungeKutta4*>(solver.get());
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
        throw std::runtime_error("Factory did not create requested real-cable RK4 algorithm");
    if (!rk && !tssbn &&
        (!newmark ||
         solver->GetType() !=
             (adaptive ? SolverNameSpace::SolverType::AdaptiveNewmark : SolverNameSpace::SolverType::Newmark) ||
         newmark->GetParams().timeStepMode !=
             (adaptive ? SolverNameSpace::TimeStepMode::Adaptive : SolverNameSpace::TimeStepMode::Fixed) ||
         newmark->GetParams().beta != .25 || newmark->GetParams().gamma != .5))
        throw std::runtime_error("真实索工厂没有返回固定 Newmark");
    QElapsedTimer timer;
    timer.start();
    bool solved = false;
    QString exception;
    QElapsedTimer solverTimer;
    solverTimer.start();
    try
    {
        solved = solver->Solve(step, step.m_Time);
    }
    catch (const std::exception& e)
    {
        exception = QString::fromUtf8(e.what());
    }
    const double solverWallSeconds = solverTimer.nsecsElapsed() * 1.0e-9;
    SolverNameSpace::TimeStepStatistics stats;
    if (tssbn)
    {
        for (std::size_t index = 1; index < step.history.size(); ++index)
            stats.RecordAccepted(step.history[index].time - step.history[index - 1].time);
        stats.acceptedSteps = tssbnSolver->GetAcceptedStepCount();
        stats.rejectedAttempts = tssbnSolver->GetRejectedStepCount();
    }
    else
        stats = rk ? rungeKutta->GetTimeStepStatistics() : newmark->GetTimeStepStatistics();
    const bool complete =
        solved && !step.history.empty() && std::abs(step.history.back().time - step.m_Time) <= 1.0e-11;
    const double matrixLimit = Number(criteria, "matrix_relative_frobenius_limit");
    const double identityLimit = Number(criteria, "newmark_kinematics_absolute_limit");
    QJsonObject gates{
        {"independent_reference_budget", AllGates(specification.value("reference_gates").toObject())},
        {"static_equilibrium", staticSolved},
        {"complete_finite_increasing", complete && step.finite && step.increasing},
        {"finite_residual_tension_locked_twist", step.diagnosticFinite},
        {"ordinary_cable_consistent_mass", massError <= matrixLimit},
        {"prestress_geometric_stiffness", equilibriumStiffnessError <= matrixLimit},
        {"nonlinear_initial_tangent", initialStiffnessError <= matrixLimit},
        {"physical_initial_acceleration",
         initialAccelerationError <= Number(criteria, "initial_acceleration_relative_limit")},
        {"no_rejection_or_precision_limited",
         stats.rejectedAttempts == 0 && (rk || tssbn || newmark->GetPrecisionLimitedSolveCount() == 0)},
        {"nominal_steps", stats.acceptedSteps == 5 * stepsPerPeriod &&
                              step.maximumStepDeviation <= 64 * std::numeric_limits<double>::epsilon()},
        {"all_reference_frames_available", step.unmatchedReferenceFrames == 0},
        {"accepted_equilibrium", step.maximumResidual <= Number(criteria, "accepted_equilibrium_absolute_limit_lbf")},
        {"newmark_displacement_identity", step.maximumDisplacementIdentity <= identityLimit},
        {"newmark_velocity_identity", step.maximumVelocityIdentity <= identityLimit},
        {"positive_tension", step.minimumTension > Number(criteria, "minimum_tension_lbf")},
        {"twist_locked", step.maximumLockedTwist == 0.0}};
    if (independentExport)
    {
        for (const QString& gate :
             {QStringLiteral("no_rejection_or_precision_limited"), QStringLiteral("nominal_steps"),
              QStringLiteral("all_reference_frames_available"), QStringLiteral("newmark_displacement_identity"),
              QStringLiteral("newmark_velocity_identity")})
            gates.remove(gate);
        bool bounded = true;
        const double timeLimit = 1.0e-9;
        const double boundRoundoff = 64 * std::numeric_limits<double>::epsilon() * std::max(1.0, step.m_Time);
        for (std::size_t index = 1; index < step.history.size(); ++index)
        {
            const double h = step.history[index].time - step.history[index - 1].time;
            const bool finalRemainder =
                index + 1 == step.history.size() && std::abs(step.history[index].time - step.m_Time) <= timeLimit;
            bounded = bounded && h > 0.0 &&
                      (finalRemainder || h + boundRoundoff >= Number(adaptiveRun, "minimum_time_step")) &&
                      h <= Number(adaptiveRun, "maximum_time_step") * (1.0 + 1.0e-6) + boundRoundoff;
        }
        gates.insert(adaptive ? "adaptive_step_bounds" : "fixed_rk_step_bounds", bounded);
        if (rk && !adaptive)
            gates.insert("fixed_rk_step_sequence",
                         stats.rejectedAttempts == 0 &&
                             stats.acceptedSteps == adaptiveRun.value("expected_accepted_steps").toInt() &&
                             step.maximumStepDeviation <= timeLimit);
        gates.insert("initial_frame_present", !step.history.empty() && step.history.front().time == 0.0);
        gates.insert("accepted_count_matches_frames", stats.acceptedSteps + 1 == static_cast<int>(step.history.size()));
    }
    QJsonArray errors, normalized;
    for (int i = 0; i < 3; ++i)
    {
        errors.append(step.maximumErrors[i]);
        normalized.append(step.maximumErrors[i] / scales[i].toDouble());
        if (!independentExport && level == 2)
            gates.insert(QStringLiteral("finest_state_%1").arg(i),
                         normalized[i].toDouble() <=
                             criteria.value("maximum_normalized_q_v_a_at_finest").toArray()[i].toDouble());
    }
    if (nonlinear)
        gates.insert("substantial_nonlinearity",
                     Number(specification, "initial_force_departure_from_linear") >=
                         Number(criteria, "nonlinear_initial_force_departure_from_linear_minimum"));
    const bool endpointEquilibriumDiagnostic = gates.value("accepted_equilibrium").toBool();
    if (referenceTssbn)
        gates.remove("accepted_equilibrium");
    if (tssbn)
        gates.insert("zero_precision_limited_stages", tssbnSolver->GetPrecisionLimitedStageCount() == 0);
    const bool passed = AllGates(gates);
    QJsonObject result{
        {"case", name},
        {"level", level},
        {"element_count", elements},
        {"production_element", "ElementCable"},
        {"production_model", "AnalysisStep/Node"},
        {"factory_solver", solver->GetName()},
        {"beta", (rk || tssbn) ? QJsonValue() : QJsonValue(newmark->GetParams().beta)},
        {"gamma", (rk || tssbn) ? QJsonValue() : QJsonValue(newmark->GetParams().gamma)},
        {"newton_tolerance",
         rk ? QJsonValue()
            : QJsonValue(tssbn ? tssbnSolver->GetParams().nonlinearTolerance : newmark->GetParams().tol)},
        {"solver_return", solved},
        {"finite_diagnostic_quantities", step.diagnosticFinite},
        {"completed", complete},
        {"gates", gates},
        {"numerical_pass", passed},
        {"status", passed ? "PASS_SELECTED_LEVEL" : "FAIL_FROZEN_GATE"},
        {"period", Number(specification, "period")},
        {"duration", step.m_Time},
        {"last_time", step.history.empty() ? 0.0 : step.history.back().time},
        {"requested_step", step.m_StepSize},
        {"accepted_steps", stats.acceptedSteps},
        {"minimum_accepted_step", stats.minimumAcceptedTimeStep},
        {"maximum_accepted_step", stats.maximumAcceptedTimeStep},
        {"maximum_step_deviation", step.maximumStepDeviation},
        {"rejected_attempts", stats.rejectedAttempts},
        {"precision_limited_solves",
         (rk || tssbn) ? QJsonValue() : QJsonValue(static_cast<double>(newmark->GetPrecisionLimitedSolveCount()))},
        {"maximum_newton_evaluations", step.maximumIterations},
        {"total_newton_evaluations", static_cast<double>(step.totalIterations)},
        {"maximum_q_v_a_absolute_errors", errors},
        {"maximum_q_v_a_normalized_errors", normalized},
        {"maximum_equilibrium_residual", step.maximumResidual},
        {"maximum_displacement_identity_defect", step.maximumDisplacementIdentity},
        {"maximum_velocity_identity_defect", step.maximumVelocityIdentity},
        {"mass_matrix_relative_error", massError},
        {"equilibrium_stiffness_relative_error", equilibriumStiffnessError},
        {"initial_stiffness_relative_error", initialStiffnessError},
        {"static_displacement_max_error", staticError},
        {"static_positive_strain_initial_guess", nonlinear ? 0.0 : 1.0e-8},
        {"initial_acceleration_normalized_error", initialAccelerationError},
        {"minimum_tension", step.minimumTension},
        {"seconds", timer.elapsed() / 1000.0},
        {"solver_wall_seconds", solverWallSeconds},
        {"last_message", step.lastMessage},
        {"exception", exception},
        {"reference_sha256", specification.value("reference_sha256")},
        {"input_sha256", specification.value("input_sha256")},
        {"source_boundary", specification.value("source_boundary")},
        {"normalization_q_v_a", scales},
        {"response_csv", QDir(runDirectory).filePath("response.csv")}};
    if (independentExport)
    {
        for (const QString& field :
             {QStringLiteral("numerical_pass"), QStringLiteral("maximum_q_v_a_absolute_errors"),
              QStringLiteral("maximum_q_v_a_normalized_errors"), QStringLiteral("maximum_displacement_identity_defect"),
              QStringLiteral("maximum_velocity_identity_defect"), QStringLiteral("maximum_step_deviation")})
            result.remove(field);
        QJsonArray times, steps;
        for (std::size_t index = 0; index < step.history.size(); ++index)
        {
            times.append(step.history[index].time);
            if (index > 0)
                steps.append(step.history[index].time - step.history[index - 1].time);
        }
        result.insert("name", name);
        result.insert("factory_solver_type", "AdaptiveNewmark");
        result.insert("time_step_mode", "Adaptive");
        result.insert("adaptive_plan_sha256", adaptiveCablePlanHash);
        result.insert("adaptive_settings", adaptiveRun);
        result.insert("rtol", adaptiveRun.value("relative_tolerance"));
        result.insert("q_atol", adaptiveRun.value("absolute_tolerance"));
        result.insert("v_atol", adaptiveRun.value("velocity_absolute_tolerance"));
        result.insert("initial_time_step", step.m_StepSize);
        result.insert("minimum_time_step", adaptiveRun.value("minimum_time_step"));
        result.insert("maximum_time_step", adaptiveRun.value("maximum_time_step"));
        result.insert("actual_times", times);
        result.insert("actual_time_steps", steps);
        result.insert("frame_count", static_cast<int>(step.history.size()));
        result.insert("runtime_pass", passed);
        result.insert("numerical_acceptance_pending", true);
        result.insert("status", passed ? "COMPLETE_UNASSESSED" : "FAIL_RUNTIME_GATE");
        result.insert("reference_evaluation", "independent dense solution at actual accepted times required");
        if (rk)
        {
            result.remove("adaptive_plan_sha256");
            for (const QString& field :
                 {QStringLiteral("beta"), QStringLiteral("gamma"), QStringLiteral("newton_tolerance"),
                  QStringLiteral("maximum_newton_evaluations"), QStringLiteral("total_newton_evaluations")})
                result.remove(field);
            result.insert("rk_plan_sha256", rkCablePlanHash);
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
        result.insert("tssbn_plan_sha256", tssbnCablePlanHash);
        result.insert("effective_tssbn_parameters", TssbnCableParameters(parameters));
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
        QJsonArray initialNodes;
        if (!step.history.empty())
            for (std::size_t index = 0; index < step.history.front().nodes.size(); ++index)
            {
                QJsonArray values;
                for (double value : step.history.front().nodes[index])
                    values.append(value);
                initialNodes.append(QJsonObject{{"node_id", static_cast<int>(index + 1)}, {"q_v_a", values}});
            }
        result.insert("actual_initial_nodal_q_v_a", initialNodes);
        bool actualReferenceMode = true;
        for (const auto& [id, element] : model->m_Elements)
        {
            Q_UNUSED(id);
            actualReferenceMode =
                actualReferenceMode &&
                std::dynamic_pointer_cast<ElementCable>(element)->IsReferenceTssbnConstitutiveMode() == referenceTssbn;
        }
        result.insert("reference_constitutive_mode", referenceTssbn);
        result.insert("constitutive_mode_asserted", actualReferenceMode);
        result.insert("preparation_scope", "Ordinary production static prestretch, then Init from current state; "
                                           "ReferenceWindless galloping workflow not executed.");
        result.insert("constitutive_scope",
                      "Production Init selects the cable mode recorded above. Same force/mass "
                      "is conditional on positive tension, matched initial chord/L0, locked twist and zero first mass "
                      "moment; matrix/initial acceleration gates and endpoint equilibrium diagnostics test this case.");
    }
    WriteJson(QDir(runDirectory).filePath("result.json"), result);
    return result;
}

static bool RunCableSuite(const QString& directory, const QString& selectedCase, int selectedLevel, double timeout)
{
    if (!QDir().mkpath(directory))
        return false;
    const auto plan = ReadPlan();
    QJsonArray cases, orders;
    bool allPassed = true;
    for (const auto entry : plan.value("cases").toArray())
    {
        const auto specification = entry.toObject();
        const auto name = specification.value("name").toString();
        if (selectedCase != "all" && selectedCase != name)
            continue;
        std::vector<QJsonObject> runs;
        for (int level = 0; level < 3; ++level)
        {
            if (selectedLevel >= 0 && selectedLevel != level)
                continue;
            try
            {
                runs.push_back(RunCableLevel(plan, specification, level, directory, timeout));
            }
            catch (const std::exception& e)
            {
                runs.push_back({{"case", name},
                                {"level", level},
                                {"numerical_pass", false},
                                {"status", "SETUP_OR_REFERENCE_FAILED"},
                                {"exception", QString::fromUtf8(e.what())}});
            }
            cases.append(runs.back());
            allPassed = allPassed && runs.back().value("numerical_pass").toBool();
        }
        if (runs.size() != 3)
            continue;
        const QJsonArray range = plan.value("criteria").toObject().value("observed_order_range").toArray();
        for (int pair = 0; pair < 2; ++pair)
        {
            QJsonArray values, passed;
            const auto coarse = runs[pair].value("maximum_q_v_a_normalized_errors").toArray();
            const auto fine = runs[pair + 1].value("maximum_q_v_a_normalized_errors").toArray();
            bool orderPassed = coarse.size() == 3 && fine.size() == 3;
            if (orderPassed)
                for (int component = 0; component < 3; ++component)
                {
                    const double p =
                        std::log(coarse[component].toDouble() / fine[component].toDouble()) / std::log(2.0);
                    const bool ok = std::isfinite(p) && p >= range[0].toDouble() && p <= range[1].toDouble();
                    values.append(std::isfinite(p) ? QJsonValue(p) : QJsonValue());
                    passed.append(ok);
                    orderPassed = orderPassed && ok;
                }
            orders.append(QJsonObject{{"case", name},
                                      {"coarse_level", pair},
                                      {"fine_level", pair + 1},
                                      {"q_v_a_orders", values},
                                      {"q_v_a_pass", passed},
                                      {"pass", orderPassed}});
            allPassed = allPassed && orderPassed;
        }
    }
    QJsonObject summary{{"schema", "newmark-real-cable-result-v1"},
                        {"numerical_pass", allPassed && !cases.isEmpty()},
                        {"status", allPassed && !cases.isEmpty() ? "PASS_FROZEN_SCOPE" : "FAIL_FROZEN_GATE"},
                        {"selection", selectedCase},
                        {"selected_level", selectedLevel},
                        {"full_three_level_validation", selectedLevel < 0},
                        {"frozen_plan_sha256", frozenPlanHash},
                        {"criteria", plan.value("criteria")},
                        {"cases", cases},
                        {"orders", orders},
                        {"scope", "real imported ElementCable/Node/AnalysisStep; independent same-constitutive "
                                  "consistent-mass q/v/a reference; public benchmark transient and guide "
                                  "boundaries explicitly derived"}};
    WriteJson(QDir(directory).filePath("summary.json"), summary);
    return allPassed && !cases.isEmpty();
}

static bool RunAlternateCableSuite(const QString& directory, const QString& selectedCase, int selectedLevel,
                                   double timeout, bool rk = false, bool adaptiveRk = true,
                                   SolverNameSpace::SolverType tssbnType = SolverNameSpace::SolverType::Newmark)
{
    const bool tssbn = tssbnType == SolverNameSpace::SolverType::AdaptiveTSSBN ||
                       tssbnType == SolverNameSpace::SolverType::ReferenceTSSBN;
    const QString adaptivePath =
        QDir(TestPaths::TestRoot())
            .filePath(tssbn ? QStringLiteral("TestModels/Support/AlgorithmPlans/TssbnDynamics/truss-cable-plan.json")
                      : rk  ? QStringLiteral("TestModels/Support/AlgorithmPlans/RungeKuttaDynamics/truss-cable-plan.json")
                            : QStringLiteral("TestModels/Support/AlgorithmPlans/AdaptiveNewmark/truss-cable-plan.json"));
    if (HashFile(adaptivePath) != (tssbn ? tssbnCablePlanHash : rk ? rkCablePlanHash : adaptiveCablePlanHash))
        throw std::runtime_error("Adaptive real-cable frozen plan SHA256 differs");
    QFile file(adaptivePath);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Cannot read adaptive real-cable plan");
    const auto adaptivePlan = QJsonDocument::fromJson(file.readAll()).object();
    if (adaptivePlan.value("schema").toString() != (tssbn ? "tssbn-real-elements-plan-v1"
                                                    : rk  ? "runge-kutta-truss-cable-plan-v1"
                                                          : "adaptive-newmark-truss-cable-plan-v1") ||
        !adaptivePlan.value("frozen_before_candidate_execution").toBool() ||
        adaptivePlan.value("families").toArray().size() != 4)
        throw std::runtime_error("Invalid adaptive real-element frozen schema");
    if (!QDir().mkpath(directory) || QFileInfo::exists(QDir(directory).filePath("summary.json")) ||
        !QFile::copy(adaptivePath, QDir(directory).filePath("truss-cable-plan.json")))
        throw std::runtime_error("Use a fresh adaptive real-cable output directory");
    const auto plan = ReadPlan();
    QJsonArray results;
    bool runtimePassed = true;
    for (const auto familyValue : adaptivePlan.value("families").toArray())
    {
        const auto family = familyValue.toObject();
        const QString name = family.value("name").toString();
        if (family.value("element").toString() != "cable" || (selectedCase != "all" && name != selectedCase))
            continue;
        QJsonObject specification;
        for (const auto sourceValue : plan.value("cases").toArray())
            if (sourceValue.toObject().value("name").toString() == name)
            {
                specification = sourceValue.toObject();
                break;
            }
        const QString runKey = rk ? (adaptiveRk ? "adaptive_runs" : "fixed_runs") : "runs";
        if (specification.isEmpty() || family.value(runKey).toArray().size() != 3 ||
            specification.value("input_sha256") != family.value("input_sha256"))
            throw std::runtime_error("Adaptive real-cable source family/input differs");
        if (tssbn && family.value("duration").toDouble() != specification.value("duration").toDouble())
            throw std::runtime_error("Frozen TSSBN and physical source durations differ");
        for (const auto runValue : family.value(runKey).toArray())
        {
            const auto adaptiveRun = runValue.toObject();
            const int level = adaptiveRun.value("level").toInt();
            if (selectedLevel >= 0 && selectedLevel != level)
                continue;
            QJsonObject result;
            try
            {
                result = RunCableLevel(plan, specification, level, directory, timeout, adaptiveRun,
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
    const int expected = (selectedCase == "all" ? 2 : 1) * (selectedLevel < 0 ? 3 : 1);
    runtimePassed = runtimePassed && results.size() == expected;
    QJsonObject summary{{"schema", "adaptive-newmark-real-cable-export-v1"},
                        {"adaptive_plan_sha256", adaptiveCablePlanHash},
                        {"source_plan_sha256", frozenPlanHash},
                        {"results", results},
                        {"all_runtime_pass", runtimePassed},
                        {"numerical_acceptance_pending", true},
                        {"status", runtimePassed ? "COMPLETE_UNASSESSED" : "FAIL_RUNTIME_GATE"},
                        {"comparison_criteria", adaptivePlan.value("comparison_criteria")},
                        {"scope", "Actual prestressed production ElementCable and factory AdaptiveNewmark; "
                                  "independent dense all-time q/v/a comparison remains required."}};
    if (rk)
    {
        summary.remove("adaptive_plan_sha256");
        summary.insert("rk_plan_sha256", rkCablePlanHash);
        summary.insert("schema", "runge-kutta-real-cable-export-v1");
        summary.insert("factory_solver_type", adaptiveRk ? "AdaptiveRungeKutta4" : "RungeKutta4");
        summary.insert("scope", "Actual prestressed production ElementCable and factory RK4; independent dense "
                                "all-time q/v/a comparison and declared refinement gates remain required.");
    }
    if (tssbn)
    {
        summary.remove("adaptive_plan_sha256");
        summary.insert("tssbn_plan_sha256", tssbnCablePlanHash);
        summary.insert("schema", "tssbn-real-cable-export-v1");
        summary.insert("factory_solver_type",
                       tssbnType == SolverNameSpace::SolverType::ReferenceTSSBN ? "ReferenceTSSBN" : "AdaptiveTSSBN");
        summary.insert("algorithm_policies", adaptivePlan.value("algorithm_policies"));
        summary.insert("scope", "Actual production elements and factory TSSBN default numerical policy; direct "
                                "Init/Solve diagnostic, not complete galloping workflow. Independent dense "
                                "q/v/a and tolerance refinement remain required; completion is not accuracy PASS.");
    }
    WriteJson(QDir(directory).filePath("summary.json"), summary);
    return runtimePassed;
}
}

/** @brief 分派真实预张紧索与大幅单索的固定验收或自适应导出。
 * @details 从冻结 BDF 读取真实 Node/ElementCable，经生产工厂静力和动力求解；独立参考按同工程应变、
 *          受拉分支与一致质量组装。最细 q/v/a 误差各不超过 0.1%，两级收敛阶均在 1.8–2.2；
 *          另查矩阵、物理初加速度、完整终点、运动学和平衡，不允许拒步或精度受限。
 *          Abaqus 原题只给模态，当前导向边界和释放时程是明确派生；大幅题不冒称公开原题。
 *          自适应入口复用原模型，导出实际接受时刻全节点状态；不套固定步行号或整步更新式，
 *          允许LTE拒步，数值精度与容差收紧门交由独立dense参考复核，完成不计通过。
 *          TSSBN 两入口保留各自工厂控制器；Reference 端点外推加速度的平衡残差单列诊断。
 * @author   yangqiuyou
 * @date     2026.10.19
 */
std::optional<int> VerifyNewmarkCable(const QStringList& arguments)
{
    const bool referenceTssbn = arguments.contains(QStringLiteral("--verify-reference-tssbn-cable"));
    const bool tssbn = referenceTssbn || arguments.contains(QStringLiteral("--verify-tssbn-cable"));
    const auto tssbnType =
        referenceTssbn ? SolverNameSpace::SolverType::ReferenceTSSBN : SolverNameSpace::SolverType::AdaptiveTSSBN;
    const bool adaptiveRk = arguments.contains(QStringLiteral("--verify-adaptive-rk4-cable"));
    const bool rk = adaptiveRk || arguments.contains(QStringLiteral("--verify-rk4-cable"));
    const bool adaptive = adaptiveRk || arguments.contains(QStringLiteral("--verify-adaptive-newmark-cable"));
    const QString flag = tssbn ? (referenceTssbn ? "--verify-reference-tssbn-cable" : "--verify-tssbn-cable")
                         : rk  ? (adaptiveRk ? "--verify-adaptive-rk4-cable" : "--verify-rk4-cable")
                               : (adaptive ? "--verify-adaptive-newmark-cable" : "--verify-newmark-cable");
    const int option = arguments.indexOf(flag);
    if (option < 0)
        return std::nullopt;
    int entries = 0;
    for (const QString& entry :
         {QStringLiteral("--verify-newmark-cable"), QStringLiteral("--verify-adaptive-newmark-cable"),
          QStringLiteral("--verify-rk4-cable"), QStringLiteral("--verify-adaptive-rk4-cable"),
          QStringLiteral("--verify-tssbn-cable"), QStringLiteral("--verify-reference-tssbn-cable")})
        entries += arguments.contains(entry) ? 1 : 0;
    if (option + 1 >= arguments.size() || arguments[option + 1].startsWith("--") || entries != 1)
        return 1;
    const auto value = [&arguments](const QString& key, const QString& fallback)
    {
        const int index = arguments.indexOf(key);
        return index >= 0 && index + 1 < arguments.size() ? arguments[index + 1] : fallback;
    };
    const QString selectedCase = value("--newmark-cable-case", "all");
    const QString levelText = value("--newmark-cable-level", "all");
    bool valid = false;
    const double timeout = value("--newmark-cable-timeout", "60").toDouble(&valid);
    if (!valid || timeout <= 0 || !std::isfinite(timeout) ||
        (selectedCase != "all" && selectedCase != "taut-string" && selectedCase != "nonlinear") ||
        (levelText != "all" && levelText != "0" && levelText != "1" && levelText != "2"))
        return 1;
    try
    {
        const QString directory = QDir(arguments[option + 1]).absolutePath();
        const int level = levelText == "all" ? -1 : levelText.toInt();
        return ((tssbn || adaptive || rk)
                    ? RunAlternateCableSuite(directory, selectedCase, level, timeout, rk, adaptiveRk,
                                             tssbn ? tssbnType : SolverNameSpace::SolverType::Newmark)
                    : RunCableSuite(directory, selectedCase, level, timeout))
                   ? 0
                   : 1;
    }
    catch (const std::exception& e)
    {
        QTextStream(stderr) << "Newmark cable validation failed: " << e.what() << Qt::endl;
        return 1;
    }
}

