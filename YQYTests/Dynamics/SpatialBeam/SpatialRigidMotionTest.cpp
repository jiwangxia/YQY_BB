#include "Dynamics/SpatialBeam/SpatialRigidMotionTest.h"

#include "DataStructure/Structure/StructureData.h"
#include "Solver/Dynamic/Integrators/SolverNewmark.h"
#include "Solver/Factory/SolverFactory.h"
#include "Support/TestPaths.h"
#include <Eigen/Geometry>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <QTextStream>
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace
{
using Eigen::Matrix3d;
using Eigen::Vector3d;
using SolverNameSpace::Vec;

QByteArray ReadFrozenSpatialFile(const QString& name, const QByteArray& expected)
{
    QFile file(QDir(TestPaths::TestRoot()).filePath("TestModels/Dynamics/SpatialBeam/" + name));
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Missing frozen spatial dynamics input");
    const auto bytes = file.readAll();
    if (QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex() != expected)
        throw std::runtime_error("Spatial dynamics input/reference SHA differs");
    return bytes;
}

struct SpatialExactState
{
    Matrix3d rotation;
    Vector3d omega;
    Vector3d alpha;
    std::array<Vector3d, 2> position;
    std::array<Vector3d, 2> velocity;
    std::array<Vector3d, 2> acceleration;
};

SpatialExactState ExactSpatialMotion(double time)
{
    const Matrix3d rz = Eigen::AngleAxisd(.7 * time, Vector3d::UnitZ()).toRotationMatrix();
    const Matrix3d ry = Eigen::AngleAxisd(.4 * time, Vector3d::UnitY()).toRotationMatrix();
    const Matrix3d rx = Eigen::AngleAxisd(.9 * time, Vector3d::UnitX()).toRotationMatrix();
    SpatialExactState state;
    state.rotation = rz * ry * rx;
    const Vector3d first = .7 * Vector3d::UnitZ();
    const Vector3d second = .4 * rz * Vector3d::UnitY();
    const Vector3d third = .9 * rz * ry * Vector3d::UnitX();
    state.omega = first + second + third;
    state.alpha = first.cross(second) + (first + second).cross(third);
    const Vector3d center(.5 + .1 * std::sin(time), .08 * (std::cos(.8 * time) - 1.0), .03 * time * time);
    const Vector3d velocity(.1 * std::cos(time), -.064 * std::sin(.8 * time), .06 * time);
    const Vector3d acceleration(-.1 * std::sin(time), -.0512 * std::cos(.8 * time), .06);
    for (int node = 0; node < 2; ++node)
    {
        const Vector3d arm = state.rotation * Vector3d(node == 0 ? -.5 : .5, 0., 0.);
        state.position[node] = center + arm;
        state.velocity[node] = velocity + state.omega.cross(arm);
        state.acceleration[node] = acceleration + state.alpha.cross(arm) + state.omega.cross(state.omega.cross(arm));
    }
    return state;
}

class SpatialMotionProbe final : public AnalysisStep
{
public:
    explicit SpatialMotionProbe(QTextStream& output)
        : output_(output)
    {
    }

    bool ShouldPreferGpuLinearSolver() const override
    {
        return false;
    }

    void ComputeExternalForce(double time, double, _OUT Vec& fixed, _OUT Vec& free) override
    {
        const auto exact = ExactSpatialMotion(time);
        const Matrix3d spatialInertia = exact.rotation * rotaryInertia.asDiagonal() * exact.rotation.transpose();
        const Vector3d moment = spatialInertia * exact.alpha + exact.omega.cross(spatialInertia * exact.omega);
        fixed = Vec::Zero(GetFixedDofs());
        free = Vec::Zero(GetFreeDofs());
        for (int node = 0; node < 2; ++node)
        {
            const auto& actual = GetStructure()->m_Nodes.at(node + 1);
            const Vector3d force = lineMass / 6.0 * (2.0 * exact.acceleration[node] + exact.acceleration[1 - node]);
            for (int axis = 0; axis < 3; ++axis)
            {
                free[actual->m_DOF[axis] - GetFixedDofs()] = force[axis];
                free[actual->m_DOF[axis + 3] - GetFixedDofs()] = .5 * moment[axis];
            }
        }
    }

    void OnStepCompleted(double time) override
    {
        AnalysisStep::OnStepCompleted(time);
        Record(time);
        ++steps;
        lastTime = time;
    }

    void Record(double time)
    {
        Vec fixed, external, residual;
        ComputeExternalForce(time, 1.0, fixed, external);
        RefreshResidualState();
        RefreshInertiaForce();
        ComputeResidual(external, residual);
        maximumResidual = std::max(maximumResidual, residual.norm());
        normalResiduals = normalResiduals && residual.allFinite() && residual.norm() <= m_Tolerance;
        for (int node = 0; node < 2; ++node)
        {
            const auto& actual = GetStructure()->m_Nodes.at(node + 1);
            output_ << time << ',' << node + 1;
            for (int axis = 0; axis < 3; ++axis)
                output_ << ',' << (axis == 0 ? static_cast<double>(node) : 0.) + actual->m_Displacement[axis];
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    output_ << ',' << actual->m_Rg(row, column);
            for (const auto* values : {&actual->m_Velocity, &actual->m_Acceleration})
                for (int axis = 0; axis < 6; ++axis)
                    output_ << ',' << (*values)[axis];
            output_ << '\n';
        }
    }

    double lineMass = .04;
    Vector3d rotaryInertia{2.0 * (.2 * std::pow(.1, 3) / 12.0 + .1 * std::pow(.2, 3) / 12.0),
                           2.0 * .2 * std::pow(.1, 3) / 12.0, 2.0 * .1 * std::pow(.2, 3) / 12.0};
    int steps = 0;
    double lastTime = 0.0;
    double maximumResidual = 0.0;
    bool normalResiduals = true;

private:
    QTextStream& output_;
};

std::shared_ptr<StructureData> BuildSpatialMotionModel(const std::shared_ptr<SpatialMotionProbe>& step)
{
    auto structure = std::make_shared<StructureData>();
    auto material = std::make_shared<Material>();
    material->m_Id = 1;
    material->m_Young = 100.;
    material->m_Density = 2.;
    material->m_Poisson = .3;
    auto section = std::make_shared<SectionRectangle>();
    section->m_Id = 1;
    section->m_Width = .2;
    section->m_Height = .1;
    section->Calculate_Area();
    auto property = std::make_shared<Property>();
    property->m_Id = 1;
    property->m_pMaterial = material;
    property->m_pSection = section;
    structure->m_Material[1] = material;
    structure->m_Section[1] = section;
    structure->m_Property[1] = property;
    for (int index = 0; index < 2; ++index)
    {
        auto node = std::make_shared<Node>();
        node->m_Id = index + 1;
        node->SetExactNumDOFs(6);
        node->m_X = static_cast<double>(index);
        structure->m_Nodes[node->m_Id] = node;
    }
    auto beam = std::make_shared<ElementBeam_CR>();
    beam->m_Id = 1;
    beam->m_pNode[0] = structure->m_Nodes.at(1);
    beam->m_pNode[1] = structure->m_Nodes.at(2);
    beam->m_pProperty = property;
    beam->q0 = Vector3d::UnitY();
    beam->Get_L0();
    structure->m_Elements[1] = beam;
    structure->m_OutputControl.m_StreamResult = false;
    structure->m_AnalysisStep[1] = step;
    step->SetAssemblyThreadCountOverride(1);
    step->SetPardisoThreadCountOverride(1);
    step->SetStructure(structure);
    step->Init();
    return structure;
}

bool RunSpatialDynamicsSuite(const QString& directory)
{
    const auto input =
        ReadFrozenSpatialFile("input.json", "726692e5c7aa6350eb9e67ad47244e4a5ec1f0465038455bb8a518bdd058bf3e");
    const auto planBytes = ReadFrozenSpatialFile("validation-plan.json",
                                                 "b68913bf00237bbacc0191c004d5e88d6fe1c27990defed33cb74ee1fe719bc6");
    const auto plan = QJsonDocument::fromJson(planBytes).object();
    const auto configuration = QJsonDocument::fromJson(input).object();
    const std::array<QByteArray, 3> referenceHashes{"bbc84681085588fc18cbeb9ecf938b3e8fd973327981730e7b801abd64a7344a",
                                                    "dcde822220645b6a585a5716e5d21940a839c9275d83482733d91e770c8cc3c6",
                                                    "ac3f7f1dd434fc64014c39b23c305918b7361581650c188c6f40ed17f4f038ef"};
    if (!QDir().mkpath(directory))
        throw std::runtime_error("Cannot create spatial dynamics output directory");
    QJsonArray results;
    bool complete = true;
    for (int level = 0; level < 3; ++level)
    {
        const auto reference =
            ReadFrozenSpatialFile(QString("level%1.reference.csv").arg(level), referenceHashes[level]);
        QFile referenceFile(QDir(directory).filePath(QString("level%1.reference.csv").arg(level)));
        if (!referenceFile.open(QIODevice::WriteOnly))
            throw std::runtime_error("Cannot copy spatial reference");
        referenceFile.write(reference);
        QFile file(QDir(directory).filePath(QString("level%1.csv").arg(level)));
        if (file.exists() || !file.open(QIODevice::WriteOnly | QIODevice::Text))
            throw std::runtime_error("Spatial dynamics output exists; use a fresh directory");
        QTextStream output(&file);
        output.setRealNumberPrecision(17);
        output << "time,node,px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22,"
                  "vx,vy,vz,wx,wy,wz,ax,ay,az,alphax,alphay,alphaz\n";
        auto step = std::make_shared<SpatialMotionProbe>(output);
        step->m_Id = 1;
        step->m_Type = EnumKeyword::StepType::DYNAMIC;
        step->m_DynamicSolverType = SolverNameSpace::SolverType::Newmark;
        step->isDynamic = true;
        step->SolverNameSpace::IAnalysisModel::isDynamic = true;
        step->m_Time = configuration.value("duration").toDouble();
        step->m_StepSize = configuration.value("time_steps").toArray()[level].toDouble();
        step->m_Tolerance = plan.value("newton_tolerance").toDouble();
        step->m_MaxIterations = plan.value("maximum_iterations").toInt();
        auto structure = BuildSpatialMotionModel(step);
        if (step->GetFreeDofs() != 12 || step->GetFixedDofs() != 0)
            throw std::runtime_error("Spatial motion must have all 12 free physical DOFs");
        const auto initial = ExactSpatialMotion(0.);
        Vec velocity = Vec::Zero(12), acceleration = Vec::Zero(12);
        for (int node = 0; node < 2; ++node)
            for (int axis = 0; axis < 3; ++axis)
            {
                const auto& actual = structure->m_Nodes.at(node + 1);
                velocity[actual->m_DOF[axis]] = initial.velocity[node][axis];
                velocity[actual->m_DOF[axis + 3]] = initial.omega[axis];
                acceleration[actual->m_DOF[axis]] = initial.acceleration[node][axis];
                acceleration[actual->m_DOF[axis + 3]] = initial.alpha[axis];
            }
        step->SetTrialKinematics(velocity, acceleration);
        step->CommitState();
        SolverNameSpace::SpMat actualMass;
        step->AssembleMassMatrix(actualMass);
        Eigen::MatrixXd expectedMass = Eigen::MatrixXd::Zero(12, 12);
        for (int rowNode = 0; rowNode < 2; ++rowNode)
            for (int columnNode = 0; columnNode < 2; ++columnNode)
                for (int axis = 0; axis < 3; ++axis)
                {
                    const double factor = (rowNode == columnNode ? 2.0 : 1.0) / 6.0;
                    const auto& first = structure->m_Nodes.at(rowNode + 1);
                    const auto& second = structure->m_Nodes.at(columnNode + 1);
                    expectedMass(first->m_DOF[axis], second->m_DOF[axis]) = factor * step->lineMass;
                    expectedMass(first->m_DOF[axis + 3], second->m_DOF[axis + 3]) = factor * step->rotaryInertia[axis];
                }
        const double initialMassError = (Eigen::MatrixXd(actualMass) - expectedMass).norm() / expectedMass.norm();
        step->Record(0.);
        const double initialResidual = step->maximumResidual;
        auto solver = SolverNameSpace::SolverFactory::Create(*step);
        auto* newmark = dynamic_cast<SolverNameSpace::SolverNewmark*>(solver.get());
        if (!newmark || newmark->GetParams().beta != .25 || newmark->GetParams().gamma != .5)
            throw std::runtime_error("Spatial motion factory is not the declared Newmark algorithm");
        const bool solved = solver->Solve(*step, step->m_Time);
        const bool reached = std::abs(step->lastTime - step->m_Time) < 1e-12;
        const bool fixed = step->steps == static_cast<int>(std::llround(step->m_Time / step->m_StepSize));
        const auto rejected = static_cast<double>(newmark->GetTimeStepStatistics().rejectedAttempts);
        const auto precision = static_cast<double>(newmark->GetPrecisionLimitedSolveCount());
        const bool execution = solved && reached && fixed && rejected == 0 && precision == 0 && step->normalResiduals &&
                               initialMassError <= plan.value("initial_mass_relative_limit").toDouble() &&
                               initialResidual <= plan.value("initial_residual_limit").toDouble();
        complete = complete && execution;
        results.append(QJsonObject{{"level", level},
                                   {"step", step->m_StepSize},
                                   {"solved", solved},
                                   {"reached_end", reached},
                                   {"accepted_steps", step->steps},
                                   {"last_time", step->lastTime},
                                   {"fixed_step_count", fixed},
                                   {"rejected_steps", rejected},
                                   {"precision_limited_steps", precision},
                                   {"normal_Newton_residuals", step->normalResiduals},
                                   {"maximum_residual", step->maximumResidual},
                                   {"initial_residual", initialResidual},
                                   {"initial_mass_relative_error", initialMassError},
                                   {"execution_gates_passed", execution}});
        output.flush();
    }
    QFile file(QDir(directory).filePath("execution.json"));
    if (!file.open(QIODevice::WriteOnly))
        throw std::runtime_error("Cannot write spatial dynamics execution record");
    file.write(
        QJsonDocument(QJsonObject{{"input", configuration},
                                  {"plan", plan},
                                  {"runs", results},
                                  {"execution_gates_passed", complete},
                                  {"numerical_acceptance", "Independent Python full-history comparison required"}})
            .toJson());
    return complete;
}
}

std::optional<int> VerifySpatialBeamDynamics(const QStringList& arguments)
{
    const int index = arguments.indexOf("--verify-spatial-beam-dynamics");
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 2;
    try
    {
        return RunSpatialDynamicsSuite(arguments[index + 1]) ? 0 : 4;
    }
    catch (const std::exception& error)
    {
        QTextStream(stderr) << error.what() << Qt::endl;
        return 2;
    }
}
