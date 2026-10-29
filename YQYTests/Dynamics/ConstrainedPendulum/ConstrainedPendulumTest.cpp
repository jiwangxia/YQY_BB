#include "Dynamics/ConstrainedPendulum/ConstrainedPendulumTest.h"

#include "DataStructure/Structure/StructureData.h"
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

namespace
{
using Eigen::Vector3d;
using SolverNameSpace::Vec;
constexpr std::array<double, 2> masses{10.0, 5.0};
constexpr double gravity = 10.0;

static QJsonArray BetschArray(const Vector3d& value)
{
    return {value[0], value[1], value[2]};
}

static Vector3d BetschNodePosition(const Node& node)
{
    return Vector3d(node.m_X + node.m_Displacement[0], node.m_Y + node.m_Displacement[1],
                    node.m_Z + node.m_Displacement[2]);
}

static Vector3d BetschNodeVector(const std::vector<double>& values)
{
    return Vector3d(values[0], values[1], values[2]);
}

// 仅记录真实 AnalysisStep 的状态与乘子；不覆盖约束、质量、荷载或动力残量装配。
class BetschPendulumProbe final : public AnalysisStep
{
public:
    explicit BetschPendulumProbe(QTextStream& stream, double nominalStep, bool strictNewmarkStep)
        : stream_(stream)
        , nominalStep_(nominalStep)
        , strictNewmarkStep_(strictNewmarkStep)
    {
    }

    bool ShouldPreferGpuLinearSolver() const override
    {
        return false;
    }

    void BeginDynamicStep(double dt, double beta, double gamma) override
    {
        // 固定步长试验在生产 Newmark 尝试回退时停止，不能把缩步完成误报为原步长完成。
        if (strictNewmarkStep_ && std::abs(dt - nominalStep_) > 1.0e-10 * std::max(1.0, nominalStep_))
        {
            rejectedTrialTimeStep = dt;
            throw std::runtime_error("Fixed-step benchmark stopped before Newmark cutback; nominal step failed");
        }
        AnalysisStep::BeginDynamicStep(dt, beta, gamma);
    }

    void SetNonlinearMPCMultipliers(const Vec& multipliers) override
    {
        AnalysisStep::SetNonlinearMPCMultipliers(multipliers);
        multipliers_ = multipliers;
    }

    void OnStepCompleted(double time) override
    {
        AnalysisStep::OnStepCompleted(time);
        Record(time);
    }

    void WriteHeader()
    {
        stream_ << "time,p1_x,p1_y,p1_z,p2_x,p2_y,p2_z,v1_x,v1_y,v1_z,v2_x,v2_y,v2_z,"
                   "a1_x,a1_y,a1_z,a2_x,a2_y,a2_z,lambda1,lambda2,kinetic,potential,energy,Lx,Ly,Lz,"
                   "c1,c2,Gv1,Gv2,Ga_h1,Ga_h2,lambda_distance1,lambda_distance2,slave_dof1,slave_dof2,"
                   "equilibrium_residual,max_mass_error,actual_dt\n";
    }

    int samples = 0;
    int slaveChanges = 0;
    bool finite = true;
    bool increasingTimes = true;
    double lastTime = 0.0;
    double rejectedTrialTimeStep = 0.0;
    double maximumPositionConstraint = 0.0;
    double maximumVelocityConstraint = 0.0;
    double maximumAccelerationConstraint = 0.0;
    double maximumEnergyDrift = 0.0;
    double maximumLzDrift = 0.0;
    double maximumEquilibriumResidual = 0.0;
    double maximumMassError = 0.0;
    double maximumStepDifference = 0.0;
    double initialStateError = std::numeric_limits<double>::infinity();
    QJsonObject initial;

private:
    void Record(double time)
    {
        Vec displacement, velocity, acceleration;
        GetState(displacement, velocity, acceleration);
        SolverNameSpace::NonlinearMPCData constraints;
        if (!AssembleNonlinearMPC(constraints) || constraints.value.size() != 2 || constraints.jacobian.cols() != 6 ||
            multipliers_.size() != 2 || velocity.size() != 6 || acceleration.size() != 6)
            throw std::runtime_error("Betsch production state or constraint dimensions differ");

        const Vec velocityGap = constraints.jacobian * velocity;
        Vec accelerationGap = constraints.jacobian * acceleration;
        for (int row = 0; row < 2; ++row)
        {
            if (constraints.hessians.size() == 2 && constraints.hessians[row].rows() == 6)
                accelerationGap[row] += velocity.dot(constraints.hessians[row] * velocity);
            if (constraints.hessianEntries.size() == 2)
                for (const auto& entry : constraints.hessianEntries[row])
                    accelerationGap[row] += velocity[entry.row()] * entry.value() * velocity[entry.col()];
        }

        SolverNameSpace::SpMat mass;
        AssembleMassMatrix(mass);
        Eigen::MatrixXd expectedMass = Eigen::MatrixXd::Zero(6, 6);
        Vec externalForce = Vec::Zero(6);
        std::array<Vector3d, 2> positions, velocities, accelerations;
        double kinetic = 0.0, potential = 0.0;
        Vector3d angularMomentum = Vector3d::Zero();
        for (int index = 0; index < 2; ++index)
        {
            const auto& node = *GetStructure()->m_Nodes.at(index + 2);
            positions[index] = BetschNodePosition(node);
            velocities[index] = BetschNodeVector(node.m_Velocity);
            accelerations[index] = BetschNodeVector(node.m_Acceleration);
            kinetic += 0.5 * masses[index] * velocities[index].squaredNorm();
            potential += masses[index] * gravity * positions[index].z();
            angularMomentum += masses[index] * positions[index].cross(velocities[index]);
            for (int axis = 0; axis < 3; ++axis)
            {
                const int dof = node.m_DOF[axis] - GetFixedDofs();
                expectedMass(dof, dof) = masses[index];
                if (axis == 2)
                    externalForce[dof] = -masses[index] * gravity;
            }
        }
        const double massError = (Eigen::MatrixXd(mass) - expectedMass).cwiseAbs().maxCoeff();
        const double equilibrium =
            (externalForce - mass * acceleration + constraints.jacobian.transpose() * multipliers_).norm();
        const double energy = kinetic + potential;
        const double actualStep = samples == 0 ? 0.0 : time - lastTime;
        finite = finite && std::isfinite(time) && displacement.allFinite() && velocity.allFinite() &&
                 acceleration.allFinite() && constraints.value.allFinite() && velocityGap.allFinite() &&
                 accelerationGap.allFinite() && multipliers_.allFinite() && std::isfinite(energy) &&
                 angularMomentum.allFinite() && std::isfinite(equilibrium) && std::isfinite(massError);
        if (samples > 0)
        {
            increasingTimes = increasingTimes && actualStep > 0.0;
            maximumStepDifference = std::max(maximumStepDifference, std::abs(actualStep - nominalStep_));
            slaveChanges += constraints.slaveDofs != previousSlaves_;
        }
        else
        {
            initialStateError = std::max(
                {(positions[0] - Vector3d(0.0, 1.0, 0.0)).norm(), (positions[1] - Vector3d(0.0, 2.0, 0.0)).norm(),
                 (velocities[0] - Vector3d(1.0, 0.0, 0.0)).norm(), (velocities[1] - Vector3d(2.0, 0.0, 0.0)).norm(),
                 (accelerations[0] - Vector3d(0.0, -1.0, -10.0)).norm(),
                 (accelerations[1] - Vector3d(0.0, -2.0, -10.0)).norm(), std::abs(multipliers_[0] + 20.0),
                 std::abs(multipliers_[1] + 10.0), std::abs(energy - 15.0), std::abs(angularMomentum.z() + 30.0)});
            initial = {{"p1", BetschArray(positions[0])},
                       {"p2", BetschArray(positions[1])},
                       {"v1", BetschArray(velocities[0])},
                       {"v2", BetschArray(velocities[1])},
                       {"a1", BetschArray(accelerations[0])},
                       {"a2", BetschArray(accelerations[1])},
                       {"lambda_paper", QJsonArray{-multipliers_[0], -multipliers_[1]}},
                       {"energy", energy},
                       {"Lz", angularMomentum.z()},
                       {"position_constraint_norm", constraints.value.norm()},
                       {"velocity_constraint_norm", velocityGap.norm()},
                       {"acceleration_constraint_norm", accelerationGap.norm()}};
        }
        maximumPositionConstraint = std::max(maximumPositionConstraint, constraints.value.cwiseAbs().maxCoeff());
        maximumVelocityConstraint = std::max(maximumVelocityConstraint, velocityGap.cwiseAbs().maxCoeff());
        maximumAccelerationConstraint = std::max(maximumAccelerationConstraint, accelerationGap.cwiseAbs().maxCoeff());
        maximumEnergyDrift = std::max(maximumEnergyDrift, std::abs(energy - 15.0));
        maximumLzDrift = std::max(maximumLzDrift, std::abs(angularMomentum.z() + 30.0));
        maximumEquilibriumResidual = std::max(maximumEquilibriumResidual, equilibrium);
        maximumMassError = std::max(maximumMassError, massError);

        stream_ << time;
        for (const auto* vectors : {&positions, &velocities, &accelerations})
            for (const Vector3d& vector : *vectors)
                for (int axis = 0; axis < 3; ++axis)
                    stream_ << ',' << vector[axis];
        // 论文采用平方长度约束及 Ma=F-G^T*lambda；本例 L=1，故 lambda_paper=-lambda_distance。
        stream_ << ',' << -multipliers_[0] << ',' << -multipliers_[1] << ',' << kinetic << ',' << potential << ','
                << energy << ',' << angularMomentum.x() << ',' << angularMomentum.y() << ',' << angularMomentum.z();
        const std::array<const Vec*, 4> constraintValues{&constraints.value, &velocityGap, &accelerationGap,
                                                         &multipliers_};
        for (const Vec* values : constraintValues)
            for (int row = 0; row < 2; ++row)
                stream_ << ',' << (*values)[row];
        stream_ << ',' << constraints.slaveDofs[0] << ',' << constraints.slaveDofs[1] << ',' << equilibrium << ','
                << massError << ',' << actualStep << '\n';
        previousSlaves_ = constraints.slaveDofs;
        lastTime = time;
        ++samples;
    }

    QTextStream& stream_;
    double nominalStep_;
    bool strictNewmarkStep_;
    Vec multipliers_;
    std::vector<int> previousSlaves_;
};

static std::shared_ptr<StructureData> BuildBetschPendulum(const std::shared_ptr<BetschPendulumProbe>& step,
                                                          bool automaticSlave)
{
    auto model = std::make_shared<StructureData>();
    for (int index = 0; index < 3; ++index)
    {
        auto node = std::make_shared<Node>();
        node->m_Id = index + 1;
        node->m_X = 0.0;
        node->m_Y = static_cast<double>(index);
        node->m_Z = 0.0;
        node->SetExactNumDOFs(6);
        model->m_Nodes[node->m_Id] = node;
        for (int axis = index == 0 ? 0 : 3; axis < 6; ++axis)
        {
            auto constraint = std::make_shared<Constraint>();
            constraint->m_Id = static_cast<int>(model->m_Constraint.size()) + 1;
            constraint->m_pNode = node;
            constraint->m_Direction = static_cast<EnumKeyword::Direction>(axis);
            model->m_Constraint[constraint->m_Id] = constraint;
        }
        if (index == 0)
            continue;
        auto inertia = std::make_shared<RigidBodyInertia>();
        inertia->m_Id = index;
        inertia->m_pNode = node;
        inertia->m_Mass = masses[index - 1];
        // 三个转动自由度已全部固定；正定转动惯量不进入自由平动方程。
        inertia->m_RotaryInertia = Eigen::Matrix3d::Identity();
        model->m_RigidBodyInertias[index] = inertia;
        auto load = std::make_shared<Force_Node>();
        load->m_Id = index;
        load->m_pNode = node;
        load->m_Direction = static_cast<EnumKeyword::Direction>(2);
        load->m_Value = -gravity * masses[index - 1];
        model->m_Load[index] = load;
        auto distance = std::make_shared<DistanceMPCConstraint>();
        distance->m_Id = index;
        distance->m_pNodeA = model->m_Nodes.at(index);
        distance->m_pNodeB = node;
        distance->m_pSlaveNode = node;
        distance->m_SlaveDirection = automaticSlave ? -1 : 1;
        distance->m_Length = 1.0;
        model->m_MPCConstraints[index] = distance;
    }
    model->m_OutputControl.m_StreamResult = false;
    model->m_AnalysisStep[1] = step;
    step->SetStructure(model);
    step->SetAssemblyThreadCountOverride(1);
    step->SetPardisoThreadCountOverride(1);
    step->SetInitialDynamicStateCallback(
        [model](QString& error)
        {
            error.clear();
            // 论文 q^1,q^2 为杆相对向量；粒子速度分别为 qdot^1 和 qdot^1+qdot^2。
            model->m_Nodes.at(2)->m_Velocity[0] = 1.0;
            model->m_Nodes.at(3)->m_Velocity[0] = 2.0;
            return true;
        });
    return model;
}

static QString BetschOption(const QStringList& arguments, const QString& name, const QString& fallback)
{
    const int index = arguments.indexOf(name);
    if (index < 0)
        return fallback;
    if (index + 1 >= arguments.size() || arguments[index + 1].startsWith(QStringLiteral("--")))
        throw std::runtime_error("Betsch benchmark option requires a value");
    return arguments[index + 1];
}

static double BetschPositiveOption(const QStringList& arguments, const QString& name, double fallback)
{
    bool converted = false;
    const double value = BetschOption(arguments, name, QString::number(fallback, 'g', 17)).toDouble(&converted);
    if (!converted || !std::isfinite(value) || value <= 0.0)
        throw std::runtime_error("Betsch benchmark option requires a finite positive value");
    return value;
}

static bool RunBetschPendulum(const QStringList& arguments, const QString& directory)
{
    const QString solver = BetschOption(arguments, QStringLiteral("--betsch-solver"), QStringLiteral("newmark"));
    if (solver != QStringLiteral("newmark") && solver != QStringLiteral("tssbn"))
        throw std::runtime_error("Betsch solver must be newmark or tssbn");
    const double stepSize = BetschPositiveOption(arguments, QStringLiteral("--betsch-step"), 0.01);
    const double duration = BetschPositiveOption(arguments, QStringLiteral("--betsch-duration"), 10.0);
    const bool automaticSlave = arguments.contains(QStringLiteral("--betsch-auto-slave"));
    const bool fixedStep = arguments.contains(QStringLiteral("--betsch-fixed-step"));
    const double stepCount = duration / stepSize;
    if (fixedStep && std::abs(stepCount - std::round(stepCount)) > 1.0e-9)
        throw std::runtime_error("Fixed-step benchmark duration must be an integer multiple of the step");
    if (!QDir().mkpath(directory))
        throw std::runtime_error("Cannot create Betsch benchmark output directory");
    const QString historyPath = QDir(directory).filePath(QStringLiteral("response.csv"));
    const QString summaryPath = QDir(directory).filePath(QStringLiteral("summary.json"));
    if (QFileInfo::exists(historyPath) || QFileInfo::exists(summaryPath))
        throw std::runtime_error("Betsch benchmark output exists; use a fresh directory");
    QFile history(historyPath);
    if (!history.open(QIODevice::WriteOnly | QIODevice::Text))
        throw std::runtime_error("Cannot open Betsch benchmark history");
    QTextStream stream(&history);
    stream.setRealNumberPrecision(17);
    auto step = std::make_shared<BetschPendulumProbe>(stream, stepSize,
                                                   fixedStep && solver == QStringLiteral("newmark"));
    step->m_Id = 1;
    step->m_Name = QStringLiteral("Betsch 2005 双球面摆");
    step->m_Type = EnumKeyword::StepType::DYNAMIC;
    step->isDynamic = true;
    step->SolverNameSpace::IAnalysisModel::isDynamic = true;
    step->m_DynamicSolverType = solver == QStringLiteral("newmark") ? SolverNameSpace::SolverType::Newmark
                                                                    : SolverNameSpace::SolverType::AdaptiveTSSBN;
    step->m_Time = duration;
    step->m_StepSize = stepSize;
    step->m_Tolerance = 1.0e-10;
    step->m_MaxIterations = 40;
    auto& tssbn = step->m_AdaptiveTssbn;
    tssbn.spectralRadiusInfinity = 1.0;
    tssbn.minimumTimeStep = stepSize;
    tssbn.maximumTimeStep = stepSize;
    tssbn.relativeTolerance = 1.0;
    tssbn.absoluteTolerance = 1.0;
    tssbn.velocityAbsoluteTolerance = 1.0;
    const auto model = BuildBetschPendulum(step, automaticSlave);
    QElapsedTimer timer;
    timer.start();
    step->SetRuntimeCallbacks({},
                              [&timer]()
                              {
                                  return timer.elapsed() > 180000;
                              });
    step->WriteHeader();
    bool solved = false;
    QString exception;
    try
    {
        solved = step->Solve(false);
    }
    catch (const std::exception& error)
    {
        exception = QString::fromUtf8(error.what());
    }
    step->ClearRuntimeCallbacks();
    stream.flush();
    history.close();
    const bool completed = solved && std::abs(step->lastTime - duration) <= 1.0e-10 * std::max(1.0, duration);
    const bool fixedGrid = step->maximumStepDifference < 1.0e-9 &&
                           std::abs(step->samples - (std::round(stepCount) + 1.0)) < 0.5;
    const bool passed = completed && (!fixedStep || fixedGrid) && step->samples > 1 && step->finite &&
                        step->increasingTimes && step->initialStateError < 1.0e-10 &&
                        step->maximumMassError < 1.0e-12 && step->maximumPositionConstraint < 1.0e-8 &&
                        step->maximumVelocityConstraint < 1.0e-7 &&
                        step->maximumAccelerationConstraint < 1.0e-6;
    const QJsonObject summary{
        {"source", "Betsch (2005), The discrete null space method, Part I: Holonomic constraints, Section 4.2.5"},
        {"doi", "10.1016/j.cma.2005.01.004"},
        {"solver", solver},
        {"paper_time_integrator_reproduced", false},
        {"model", "production AnalysisStep + Node + RigidBodyInertia + DistanceMPCConstraint + Force_Node"},
        {"slave_selection", automaticSlave ? "automatic (-1)" : "fixed Y (1)"},
        {"mass", QJsonArray{masses[0], masses[1]}},
        {"rod_lengths", QJsonArray{1.0, 1.0}},
        {"gravity", QJsonArray{0.0, 0.0, -gravity}},
        {"q1_relative_initial", QJsonArray{0.0, 1.0, 0.0}},
        {"q2_relative_initial", QJsonArray{0.0, 1.0, 0.0}},
        {"qdot1_relative_initial", QJsonArray{1.0, 0.0, 0.0}},
        {"qdot2_relative_initial", QJsonArray{1.0, 0.0, 0.0}},
        {"rotation_dofs", "all fixed; positive unit rotary inertias do not enter the six free translations"},
        {"constraint_definition", "c=norm(xB-xA)-1; G and H from production DistanceMPCConstraint"},
        {"lambda_definition", "paper lambda=-production distance lambda for L=1; M*a=F+G^T*lambda_distance"},
        {"nominal_time_step", stepSize},
        {"fixed_step_requested", fixedStep},
        {"newmark_cutback_prohibited", fixedStep && solver == QStringLiteral("newmark")},
        {"rejected_trial_time_step", step->rejectedTrialTimeStep},
        {"fixed_grid_completed", completed && fixedGrid},
        {"tssbn_accepted_solution",
         solver == QStringLiteral("tssbn") ? "two-half-step fine solution" : "not applicable"},
        {"tssbn_nominal_internal_fine_step", solver == QStringLiteral("tssbn") ? 0.5 * stepSize : 0.0},
        {"tssbn_coarse_step_also_evaluated", solver == QStringLiteral("tssbn")},
        {"requested_end_time", duration},
        {"actual_end_time", step->lastTime},
        {"samples", step->samples},
        {"free_dofs", step->GetFreeDofs()},
        {"fixed_dofs", step->GetFixedDofs()},
        {"solved", solved},
        {"completed", completed},
        {"finite", step->finite},
        {"increasing_times", step->increasingTimes},
        {"internal_checks_passed", passed},
        {"external_reference_checks", "must be evaluated independently against the ODE reference and paper figures"},
        {"max_position_constraint", step->maximumPositionConstraint},
        {"max_velocity_constraint", step->maximumVelocityConstraint},
        {"max_acceleration_constraint", step->maximumAccelerationConstraint},
        {"max_energy_drift", step->maximumEnergyDrift},
        {"max_Lz_drift", step->maximumLzDrift},
        {"max_equilibrium_residual", step->maximumEquilibriumResidual},
        {"max_mass_matrix_error", step->maximumMassError},
        {"max_time_step_difference", step->maximumStepDifference},
        {"slave_changes", step->slaveChanges},
        {"initial_state", step->initial},
        {"initial_state_max_error", step->initialStateError},
        {"failure_reason", step->LastFailureReason()},
        {"exception", exception},
        {"elapsed_seconds", timer.elapsed() * 0.001},
        {"history", QFileInfo(historyPath).absoluteFilePath()}};
    QFile output(summaryPath);
    if (!output.open(QIODevice::WriteOnly | QIODevice::Truncate) ||
        output.write(QJsonDocument(summary).toJson(QJsonDocument::Indented)) <= 0)
        throw std::runtime_error("Cannot write Betsch benchmark summary");
    QTextStream(stdout) << "betsch_double_pendulum solver=" << solver << " dt=" << stepSize
                        << " automatic_slave=" << automaticSlave << " solved=" << solved
                        << " end_time=" << step->lastTime << " samples=" << step->samples
                        << " max_c=" << step->maximumPositionConstraint << " max_Gv=" << step->maximumVelocityConstraint
                        << " max_Ga_h=" << step->maximumAccelerationConstraint
                        << " max_energy_drift=" << step->maximumEnergyDrift << " max_Lz_drift=" << step->maximumLzDrift
                        << " passed=" << passed << " summary=" << QFileInfo(summaryPath).absoluteFilePath() << Qt::endl;
    return passed;
}
}

/** @brief Betsch（2005）双球面摆的真实 MPC 动力回归入口
 * @details 输入：论文第 4.2.5 节的两个质点和两个单位长度无质量杆，m=10/5、重力 -10e3，
 *          杆相对初位置均 e2、相对初速度均 e1；真实粒子初位置为 e2/2e2，初速度为 e1/2e1。
 *          通过 AnalysisStep::Solve 调用生产 Newmark 或 TSSBN，包含生产初始加速度一致性恢复。
 *          导出接受态、生产三层约束、乘子、能量和竖向角动量；返回值仅验完成和内部一致性。
 *          论文的离散零空间能量动量积分器未在此实现，轨迹精度须与独立参考另行核对。
 *          仅通过显式参数 --verify-betsch-double-pendulum <新输出目录> 运行，不进入默认测试套件。
 * @author   yangqiuyou
 * @date     2026.10.20
 */
std::optional<int> VerifyBetschConstraintDynamics(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-betsch-double-pendulum"));
    if (index < 0)
        return std::nullopt;
    try
    {
        if (index + 1 >= arguments.size() || arguments[index + 1].startsWith(QStringLiteral("--")))
            throw std::runtime_error("Betsch benchmark requires an explicit fresh output directory");
        return RunBetschPendulum(arguments, arguments[index + 1]) ? 0 : 2;
    }
    catch (const std::exception& error)
    {
        QTextStream(stderr) << "Betsch benchmark failed: " << error.what() << Qt::endl;
        return 2;
    }
}
