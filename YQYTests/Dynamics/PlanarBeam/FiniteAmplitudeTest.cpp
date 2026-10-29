#include "Dynamics/PlanarBeam/FiniteAmplitudeTest.h"

#include "DataStructure/Structure/StructureData.h"
#include "Solver/Dynamic/Integrators/SolverNewmark.h"
#include "Solver/Dynamic/Integrators/SolverRungeKutta4.h"
#include "Solver/Factory/SolverFactory.h"
#include "Support/TestPaths.h"
#include <QCryptographicHash>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <QTextStream>
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace
{
using PlanarBeamVector = Eigen::Vector2d;
using PlanarBeamMatrix = Eigen::Matrix2d;
using SolverNameSpace::SpMat;
using SolverNameSpace::Vec;

static QJsonObject ReadPlanarJson(const QString& name, const QByteArray& expectedHash)
{
    const QString directory = QDir(TestPaths::TestRoot()).filePath("TestModels/Dynamics/PlanarBeam/FiniteAmplitude");
    QFile file(QDir(directory).filePath(name));
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error("Missing frozen nonlinear beam input or independent reference");
    const QByteArray bytes = file.readAll();
    if (QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex() != expectedHash)
        throw std::runtime_error("Frozen nonlinear beam input/reference SHA256 differs");
    QJsonParseError error;
    const auto document = QJsonDocument::fromJson(bytes, &error);
    if (error.error != QJsonParseError::NoError || !document.isObject())
        throw std::runtime_error("Invalid frozen nonlinear beam JSON");
    return document.object();
}

static double PlanarNumber(const QJsonObject& object, const char* name)
{
    const QJsonValue value = object.value(QLatin1String(name));
    if (!value.isDouble() || !std::isfinite(value.toDouble()))
        throw std::runtime_error("Missing finite nonlinear beam reference value");
    return value.toDouble();
}

static void WritePlanarJson(const QString& path, const QJsonObject& object)
{
    QFile file(path);
    const QByteArray bytes = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size())
        throw std::runtime_error("Cannot write nonlinear beam result");
}

struct PlanarPhysics
{
    double length;
    double axial;
    double bending;
    PlanarBeamVector mass;

    explicit PlanarPhysics(const QJsonObject& input)
    {
        length = PlanarNumber(input, "length");
        const double width = PlanarNumber(input, "width");
        const double height = PlanarNumber(input, "height");
        const double area = width * height;
        const double inertia = height * std::pow(width, 3) / 12.0;
        const double young = PlanarNumber(input, "young");
        const double density = PlanarNumber(input, "density");
        axial = young * area;
        bending = young * inertia;
        mass << density * area * length / 3.0, density * inertia * length / 3.0;
    }

    PlanarBeamVector Force(const PlanarBeamVector& q) const
    {
        const double ell = std::hypot(length, q[0]);
        const double extension = q[0] * q[0] / (ell + length);
        const double phi = std::atan2(q[0], length);
        return {axial / length * extension * q[0] / ell +
                    2.0 * bending / length * (6.0 * phi - 3.0 * q[1]) * length / (ell * ell),
                2.0 * bending / length * (-3.0 * phi + 2.0 * q[1])};
    }

    double Potential(const PlanarBeamVector& q) const
    {
        const double extension = q[0] * q[0] / (std::hypot(length, q[0]) + length);
        const double phi = std::atan2(q[0], length);
        return axial / (2.0 * length) * extension * extension +
               2.0 * bending / length * (3.0 * phi * phi - 3.0 * phi * q[1] + q[1] * q[1]);
    }
};

class PlanarReferenceHistory
{
public:
    explicit PlanarReferenceHistory(const QJsonArray& rows)
    {
        if (rows.size() < 2)
            throw std::runtime_error("Nonlinear beam interpolation requires at least two reference rows");
        for (const auto& value : rows)
        {
            const auto object = value.toObject();
            ReferenceRow row;
            row.time = PlanarNumber(object, "time");
            for (int component = 0; component < 3; ++component)
            {
                const char* key = component == 0 ? "q" : component == 1 ? "v" : "a";
                const auto entries = object.value(QLatin1String(key)).toArray();
                if (entries.size() != 2)
                    throw std::runtime_error("Nonlinear beam reference state must have two components");
                for (int dof = 0; dof < 2; ++dof)
                {
                    if (!entries[dof].isDouble() || !std::isfinite(entries[dof].toDouble()))
                        throw std::runtime_error("Nonlinear beam reference state must be finite");
                    row.state[component][dof] = entries[dof].toDouble();
                }
            }
            if (!rows_.empty() && row.time <= rows_.back().time)
                throw std::runtime_error("Nonlinear beam reference times must be strictly increasing");
            rows_.push_back(row);
        }
    }

    std::array<PlanarBeamVector, 3> State(double time, const PlanarPhysics& physics) const
    {
        constexpr double endpointTolerance = 1.0e-12;
        if (!std::isfinite(time) || time < rows_.front().time - endpointTolerance ||
            time > rows_.back().time + endpointTolerance)
            throw std::runtime_error("Nonlinear beam sample time is outside the frozen reference");
        time = std::clamp(time, rows_.front().time, rows_.back().time);
        const auto upper = std::lower_bound(rows_.begin(), rows_.end(), time,
                                            [](const ReferenceRow& row, double requested)
                                            {
                                                return row.time < requested;
                                            });
        if (upper == rows_.end())
            throw std::runtime_error("Nonlinear beam reference time lookup failed");
        if (upper->time == time)
        {
            auto result = upper->state;
            result[2] = -physics.Force(result[0]).cwiseQuotient(physics.mass);
            return result;
        }
        const auto& first = *(upper - 1);
        const auto& second = *upper;
        const double h = second.time - first.time;
        const double s = (time - first.time) / h;
        const PlanarBeamVector positionRemainder =
            second.state[0] - first.state[0] - h * first.state[1] - 0.5 * h * h * first.state[2];
        const PlanarBeamVector velocityRemainder = h * (second.state[1] - first.state[1]) - h * h * first.state[2];
        const PlanarBeamVector accelerationRemainder = h * h * (second.state[2] - first.state[2]);
        const PlanarBeamVector cubic = 10.0 * positionRemainder - 4.0 * velocityRemainder + 0.5 * accelerationRemainder;
        const PlanarBeamVector quartic = -15.0 * positionRemainder + 7.0 * velocityRemainder - accelerationRemainder;
        const PlanarBeamVector quintic =
            6.0 * positionRemainder - 3.0 * velocityRemainder + 0.5 * accelerationRemainder;
        // 冻结端点的 q/v/a 定义五次 Hermite；加速度由独立势能梯度计算，避免插值导数放大。
        std::array<PlanarBeamVector, 3> result;
        result[0] =
            first.state[0] +
            s * (h * first.state[1] + s * (0.5 * h * h * first.state[2] + s * (cubic + s * (quartic + s * quintic))));
        result[1] =
            first.state[1] + s * (h * first.state[2] + s * (3.0 * cubic + s * (4.0 * quartic + s * 5.0 * quintic)) / h);
        result[2] = -physics.Force(result[0]).cwiseQuotient(physics.mass);
        return result;
    }

private:
    struct ReferenceRow
    {
        double time;
        std::array<PlanarBeamVector, 3> state;
    };
    std::vector<ReferenceRow> rows_;
};

static double CheckPlanarReferenceInterpolation(const QJsonArray& levels, const QJsonArray& scales,
                                                const PlanarPhysics& physics, double limit)
{
    const PlanarReferenceHistory coarse(levels[1].toObject().value("rows").toArray());
    const auto fine = levels[2].toObject().value("rows").toArray();
    double maximum = 0.0;
    // 用两倍网格间距的插值对比独立最细参考的全部中点，检查误差远低于生产响应门。
    for (int index = 1; index < fine.size(); index += 2)
    {
        const auto row = fine[index].toObject();
        const auto actual = coarse.State(PlanarNumber(row, "time"), physics);
        for (int component = 0; component < 3; ++component)
        {
            const char* key = component == 0 ? "q" : component == 1 ? "v" : "a";
            const auto expected = row.value(QLatin1String(key)).toArray();
            for (int dof = 0; dof < 2; ++dof)
            {
                const double scale = scales[2 * component + dof].toDouble();
                if (!std::isfinite(scale) || scale <= 0.0)
                    throw std::runtime_error("Nonlinear beam reference scale must be positive and finite");
                maximum = std::max(maximum, std::abs(actual[component][dof] - expected[dof].toDouble()) / scale);
            }
        }
    }
    if (!std::isfinite(maximum) || maximum > limit)
        throw std::runtime_error("Nonlinear beam reference interpolation failed frozen consistency gate");
    return maximum;
}

struct PlanarSample
{
    double time;
    std::array<PlanarBeamVector, 3> state;
    double residual;
    double energy;
    double forceDifference = 0.0;
    double massDifference = 0.0;
    double potentialDifference = 0.0;
};

class NonlinearBeamProbe final : public AnalysisStep
{
public:
    explicit NonlinearBeamProbe(const PlanarPhysics& physics)
        : physics_(physics)
    {
    }

    bool ShouldPreferGpuLinearSolver() const override
    {
        return false;
    }

    void OnStepCompleted(double time) override
    {
        AnalysisStep::OnStepCompleted(time);
        const auto& tip = GetStructure()->m_Nodes.at(2);
        const int indices[2] = {tip->m_DOF[1] - GetFixedDofs(), tip->m_DOF[5] - GetFixedDofs()};
        if (GetFreeDofs() != 2 || indices[0] < 0 || indices[1] < 0 || indices[0] == indices[1])
            throw std::runtime_error("Nonlinear beam does not have the declared physical Y/RZ free DOFs");
        PlanarSample sample{time,
                            {PlanarBeamVector(tip->m_Displacement[1], std::atan2(tip->m_Rg(1, 0), tip->m_Rg(0, 0))),
                             PlanarBeamVector(tip->m_Velocity[1], tip->m_Velocity[5]),
                             PlanarBeamVector(tip->m_Acceleration[1], tip->m_Acceleration[5])},
                            0.0,
                            0.0};
        SolverNameSpace::SystemEvaluationRequest request{
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ResidualState) |
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ExternalForce) |
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Mass) |
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InternalForce)};
        request.time = time;
        SolverNameSpace::SystemEvaluationResult result;
        EvaluateSystem(request, result);
        Vec residual;
        ComputeResidual(result.externalForce, residual);
        sample.residual = residual.norm();
        PlanarBeamMatrix actualMass;
        PlanarBeamVector internal;
        for (int i = 0; i < 2; ++i)
        {
            internal[i] = result.internalForce[indices[i]];
            for (int j = 0; j < 2; ++j)
                actualMass(i, j) = result.mass.coeff(indices[i], indices[j]);
        }
        const PlanarBeamVector independentForce = physics_.Force(sample.state[0]);
        sample.forceDifference = (internal - independentForce).norm() / std::max(1.0, independentForce.norm());
        maximumForceDifference = std::max(maximumForceDifference, (internal - independentForce).norm() /
                                                                      std::max(1.0, independentForce.norm()));
        const PlanarBeamMatrix independentMass = physics_.mass.asDiagonal();
        sample.massDifference = (actualMass - independentMass).norm() / independentMass.norm();
        maximumMassDifference =
            std::max(maximumMassDifference, (actualMass - independentMass).norm() / independentMass.norm());
        const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(GetStructure()->m_Elements.at(1));
        const double potential = beam->GetStrainEnergy();
        const double independentPotential = physics_.Potential(sample.state[0]);
        const double kinetic = 0.5 * sample.state[1].dot(actualMass * sample.state[1]);
        const std::array<Node, 2> nodesBefore{*beam->m_pNode[0].lock(), *beam->m_pNode[1].lock()};
        const double stressBefore = beam->m_Stress;
        const Vec internalForceBefore = beam->m_inforce;
        double exportedKinetic = 0.0, exportedStrain = 0.0;
        if (!beam->TryGetKineticEnergy(exportedKinetic) || !beam->TryGetStrainEnergy(exportedStrain) ||
            !std::isfinite(exportedKinetic) || !std::isfinite(exportedStrain) ||
            std::abs(exportedKinetic - kinetic) > 5.0e-12 * std::max(1.0, std::abs(kinetic)) ||
            std::abs(exportedStrain - potential) > 5.0e-12 * std::max(1.0, std::abs(potential)))
            throw std::runtime_error("Nonlinear beam energy query differs from physical mass or elastic potential");
        // 能量查询只观察接受态，不能重写节点、内力或应力缓存。
        if (beam->m_Stress != stressBefore || !beam->m_inforce.isApprox(internalForceBefore, 0.0))
            throw std::runtime_error("Nonlinear beam energy query changed force or stress");
        for (int index = 0; index < 2; ++index)
        {
            const auto node = beam->m_pNode[index].lock();
            const auto& before = nodesBefore[index];
            if (node->m_Displacement != before.m_Displacement || node->m_Velocity != before.m_Velocity ||
                node->m_Acceleration != before.m_Acceleration || node->m_Force != before.m_Force ||
                node->m_ReactionForce != before.m_ReactionForce || !node->m_Rg.isApprox(before.m_Rg, 0.0) ||
                !node->m_Rg_n.isApprox(before.m_Rg_n, 0.0) ||
                !node->m_StepRotation.isApprox(before.m_StepRotation, 0.0) ||
                !node->m_OmegaMaterial.isApprox(before.m_OmegaMaterial, 0.0) ||
                !node->m_AlphaMaterial.isApprox(before.m_AlphaMaterial, 0.0))
                throw std::runtime_error("Nonlinear beam energy query changed accepted node state");
        }
        sample.potentialDifference =
            std::max(std::abs(potential - independentPotential), std::abs(exportedStrain - independentPotential)) /
            std::max(1.0, independentPotential);
        if (requireFiniteFormulaDiagnostics &&
            (!actualMass.allFinite() || !internal.allFinite() || !independentForce.allFinite() ||
             !std::isfinite(sample.forceDifference) || !std::isfinite(sample.massDifference) ||
             !std::isfinite(sample.potentialDifference)))
            throw std::runtime_error("Adaptive nonlinear beam mass/force/potential diagnostics are nonfinite");
        maximumPotentialDifference = std::max(maximumPotentialDifference, sample.potentialDifference);
        sample.energy = potential + kinetic;
        normalResiduals = normalResiduals && sample.residual <= m_Tolerance;
        maximumResidual = std::max(maximumResidual, sample.residual);
        const double phi = std::atan2(sample.state[0][0], physics_.length);
        principalFrame = principalFrame && std::abs(phi) < 0.5 && std::abs(sample.state[0][1] - phi) < 0.5;
        if (!std::isfinite(time) || !std::isfinite(sample.residual) || !std::isfinite(sample.energy) ||
            !sample.state[0].allFinite() || !sample.state[1].allFinite() || !sample.state[2].allFinite() ||
            (!history.empty() && time <= history.back().time))
            throw std::runtime_error("Nonlinear production beam state/time is invalid");
        history.push_back(sample);
    }

    std::vector<PlanarSample> history;
    double maximumForceDifference = 0.0;
    double maximumMassDifference = 0.0;
    double maximumPotentialDifference = 0.0;
    double maximumResidual = 0.0;
    bool normalResiduals = true;
    bool principalFrame = true;
    bool requireFiniteFormulaDiagnostics = false;

private:
    PlanarPhysics physics_;
};

static std::shared_ptr<StructureData> BuildPlanarModel(const QJsonObject& input,
                                                       const std::shared_ptr<NonlinearBeamProbe>& step)
{
    auto structure = std::make_shared<StructureData>();
    auto material = std::make_shared<Material>();
    material->m_Id = 1;
    material->m_Young = PlanarNumber(input, "young");
    material->m_Density = PlanarNumber(input, "density");
    material->m_Poisson = PlanarNumber(input, "poisson");
    auto section = std::make_shared<SectionRectangle>();
    section->m_Id = 1;
    section->m_Width = PlanarNumber(input, "width");
    section->m_Height = PlanarNumber(input, "height");
    section->Calculate_Area();
    auto property = std::make_shared<Property>();
    property->m_Id = 1;
    property->m_pMaterial = material;
    property->m_pSection = section;
    structure->m_Material[1] = material;
    structure->m_Section[1] = section;
    structure->m_Property[1] = property;
    for (int i = 0; i < 2; ++i)
    {
        auto node = std::make_shared<Node>();
        node->m_Id = i + 1;
        node->SetExactNumDOFs(6);
        node->m_X = PlanarNumber(input, "length") * i;
        structure->m_Nodes[node->m_Id] = node;
    }
    auto beam = std::make_shared<ElementBeam_CR>();
    beam->m_Id = 1;
    beam->m_pNode[0] = structure->m_Nodes.at(1);
    beam->m_pNode[1] = structure->m_Nodes.at(2);
    beam->m_pProperty = property;
    beam->q0 = Eigen::Vector3d::UnitY();
    beam->Get_L0();
    structure->m_Elements[1] = beam;
    for (int node = 1; node <= 2; ++node)
        for (int direction = 0; direction < 6; ++direction)
        {
            if (node == 2 && (direction == 1 || direction == 5))
                continue;
            auto constraint = std::make_shared<Constraint>();
            constraint->m_Id = static_cast<int>(structure->m_Constraint.size()) + 1;
            constraint->m_pNode = structure->m_Nodes.at(node);
            constraint->m_Direction = static_cast<EnumKeyword::Direction>(direction);
            structure->m_Constraint[constraint->m_Id] = constraint;
        }
    structure->m_OutputControl.m_StreamResult = false;
    structure->m_AnalysisStep[1] = step;
    step->SetAssemblyThreadCountOverride(1);
    step->SetPardisoThreadCountOverride(1);
    step->SetStructure(structure);
    step->Init();
    return structure;
}
}

QJsonObject NewmarkBeamNonlinear::RunCase(const QString& directory, int level,
                                          std::optional<double> diagnosticTolerance, bool adaptive, bool rk4)
{
    if (level < 0 || level > 2 || !QDir().mkpath(directory))
        throw std::runtime_error("Invalid nonlinear beam level or output directory");
    const auto input = ReadPlanarJson("input.json", "dda1c8559c2d5eca507cc71aa714b4ae3cd15c835aa9de25b6ae4d8e096e35e6");
    const auto plan =
        ReadPlanarJson("validation-plan.json", "280af7ca6d0f6fc9a1955a736715e1d54c5b0fe0bdad5caeaae14ac0a77a233b");
    const auto reference =
        ReadPlanarJson("reference.json", "98a5e7c54590ce6a9393cd4b5fcaa3481aeb408d9c5a8aa3af39be530c81b97b");
    QJsonObject adaptivePlan;
    if (adaptive || rk4)
    {
        QFile file(QDir(TestPaths::TestRoot())
                       .filePath(rk4 ? "TestModels/Support/AlgorithmPlans/RungeKuttaDynamics/beam-plan.json"
                                     : "TestModels/Support/AlgorithmPlans/AdaptiveNewmark/beam-plan.json"));
        if (!file.open(QIODevice::ReadOnly))
            throw std::runtime_error("Missing frozen adaptive beam plan");
        const QByteArray bytes = file.readAll();
        if (QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex() !=
            (rk4 ? QByteArray("eb98c37607ca54f549b386b76f4f0e929791aa86a2f4cd5bde626030ef8c1556")
                 : QByteArray("c684beb8417d760b984341fa1fc7d7f0b66e753f1be08807c55ffdbfe078e034")))
            throw std::runtime_error("Frozen adaptive beam plan SHA differs");
        adaptivePlan = QJsonDocument::fromJson(bytes).object();
    }
    if (!reference.value("selfcheck_passed").toBool())
        throw std::runtime_error("Independent nonlinear beam reference has not passed self checks");
    const auto levels = reference.value("levels").toArray();
    if (levels.size() != 3)
        throw std::runtime_error("Independent nonlinear beam reference does not contain three levels");
    const auto run = levels[level].toObject();
    const auto rows = run.value("rows").toArray();
    const auto scales = reference.value("normalization_scales_q_v_a_y_rz").toArray();
    if (rows.size() < 2 || scales.size() != 6)
        throw std::runtime_error("Independent nonlinear beam reference is incomplete");
    const QString prefix = QDir(directory).filePath(QStringLiteral("nonlinear-level%1").arg(level));
    for (const QString& suffix :
         {QStringLiteral(".csv"), QStringLiteral(".configuration.json"), QStringLiteral(".result.json")})
        if (QFileInfo::exists(prefix + suffix))
            throw std::runtime_error("Nonlinear beam record exists; use a fresh directory");
    const PlanarPhysics physics(input);
    const PlanarReferenceHistory referenceHistory(levels[2].toObject().value("rows").toArray());
    const double interpolationDifference = CheckPlanarReferenceInterpolation(
        levels, scales, physics, PlanarNumber(plan, "reference_dual_normalized_difference_limit"));
    auto step = std::make_shared<NonlinearBeamProbe>(physics);
    step->requireFiniteFormulaDiagnostics = adaptive || rk4;
    step->m_Id = 1;
    step->m_Type = EnumKeyword::StepType::DYNAMIC;
    step->m_DynamicSolverType =
        rk4 ? (adaptive ? SolverNameSpace::SolverType::AdaptiveRungeKutta4 : SolverNameSpace::SolverType::RungeKutta4)
            : (adaptive ? SolverNameSpace::SolverType::AdaptiveNewmark : SolverNameSpace::SolverType::Newmark);
    step->isDynamic = true;
    step->SolverNameSpace::IAnalysisModel::isDynamic = true;
    step->m_Time = PlanarNumber(run, "duration");
    step->m_StepSize = PlanarNumber(run, "step");
    if (rk4 && !adaptive)
        step->m_StepSize =
            step->m_Time /
            (2.0 *
             adaptivePlan.value("nonlinear").toObject().value("fixed_steps_per_period").toArray()[level].toDouble());
    if (diagnosticTolerance && (!std::isfinite(*diagnosticTolerance) || *diagnosticTolerance <= 0.0))
        throw std::runtime_error("Nonlinear beam diagnostic tolerance must be finite and positive");
    step->m_Tolerance = diagnosticTolerance.value_or(
        rk4 ? PlanarNumber(adaptivePlan, "physical_residual_absolute_limit") : PlanarNumber(plan, "newton_tolerance"));
    step->m_MaxIterations = 40;
    if (adaptive)
    {
        const auto settings = adaptivePlan.value("nonlinear").toObject();
        auto& control = step->m_AdaptiveTimeStep;
        control.relativeTolerance = adaptivePlan.value("relative_tolerances").toArray()[level].toDouble();
        control.velocityRelativeTolerance = control.relativeTolerance;
        const double fraction = PlanarNumber(adaptivePlan, "absolute_tolerance_scale_fraction");
        control.absoluteTolerance =
            control.relativeTolerance * fraction * std::min(scales[0].toDouble(), scales[1].toDouble());
        control.velocityAbsoluteTolerance =
            control.relativeTolerance * fraction * std::min(scales[2].toDouble(), scales[3].toDouble());
        control.minimumTimeStep = step->m_Time * PlanarNumber(settings, "minimum_step_over_duration");
        control.maximumTimeStep = step->m_Time * PlanarNumber(settings, "maximum_step_over_duration");
        control.includeVelocityInError = true;
        control.safetyFactor = PlanarNumber(adaptivePlan, "safety_factor");
        control.minimumScaleFactor = PlanarNumber(adaptivePlan, "minimum_scale_factor");
        control.maximumGrowthFactor = PlanarNumber(adaptivePlan, "maximum_growth_factor");
        control.solveFailureFactor = PlanarNumber(adaptivePlan, "solve_failure_factor");
        control.maximumRejectedAttempts =
            static_cast<int>(PlanarNumber(adaptivePlan, "maximum_rejected_attempts_per_time"));
        step->m_StepSize = step->m_Time * PlanarNumber(settings, "initial_step_over_duration");
    }
    auto structure = BuildPlanarModel(input, step);
    const auto& tip = structure->m_Nodes.at(2);
    const int indices[2] = {tip->m_DOF[1] - step->GetFixedDofs(), tip->m_DOF[5] - step->GetFixedDofs()};
    if (step->GetFreeDofs() != 2 || indices[0] < 0 || indices[1] < 0 || indices[0] == indices[1])
        throw std::runtime_error("Nonlinear beam initial free DOFs differ");
    const auto q0 = input.value("initial_q_y_rz").toArray();
    const auto v0 = input.value("initial_v_y_rz").toArray();
    const PlanarBeamVector position(q0[0].toDouble(), q0[1].toDouble());
    const PlanarBeamVector initialAcceleration = -physics.Force(position).cwiseQuotient(physics.mass);
    Vec increment = Vec::Zero(2), velocity = Vec::Zero(2), acceleration = Vec::Zero(2);
    for (int i = 0; i < 2; ++i)
    {
        increment[indices[i]] = position[i];
        velocity[indices[i]] = v0[i].toDouble();
        acceleration[indices[i]] = initialAcceleration[i];
    }
    // 初态经实际节点位移/姿态入口建立，初加速度来自独立同方程平衡，随后另核真实残差。
    step->ApplyIncrement(increment);
    step->SetTrialKinematics(velocity, acceleration);
    step->CommitState();
    auto solver = SolverNameSpace::SolverFactory::Create(*step);
    auto* newmark = dynamic_cast<SolverNameSpace::SolverNewmark*>(solver.get());
    auto* rungeKutta = dynamic_cast<SolverNameSpace::SolverRungeKutta4*>(solver.get());
    if (rk4 ? (!rungeKutta || rungeKutta->GetType() != step->m_DynamicSolverType)
            : (!newmark || newmark->GetType() != step->m_DynamicSolverType ||
               newmark->GetParams().timeStepMode !=
                   (adaptive ? SolverNameSpace::TimeStepMode::Adaptive : SolverNameSpace::TimeStepMode::Fixed) ||
               newmark->GetParams().beta != 0.25 || newmark->GetParams().gamma != 0.5))
        throw std::runtime_error("Factory nonlinear beam Newmark method differs");
    const double originalMinimum = newmark ? newmark->GetParams().minimumTimeStep : 0.0;
    if (!adaptive && !rk4)
        newmark->GetParams().minimumTimeStep =
            PlanarNumber(plan.value("factory_parameter_override").toObject(), "minimumTimeStep");
    QJsonObject configuration{
        {"input", input},
        {"plan", plan},
        {"reference_sha256", "98a5e7c54590ce6a9393cd4b5fcaa3481aeb408d9c5a8aa3af39be530c81b97b"},
        {"level", level},
        {"step", step->m_StepSize},
        {"duration", step->m_Time},
        {"default_minimum_step", originalMinimum},
        {"actual_minimum_step", newmark ? QJsonValue(newmark->GetParams().minimumTimeStep) : QJsonValue()},
        {"execution", "Factory-held production fixed Newmark with actual initialized AnalysisStep, Node and CR beam; "
                      "not AnalysisStep::Solve wrapper"},
        {"initial_acceleration_policy",
         "independent exact planar M^-1(-grad U); actual production initial residual separately gated"},
        {"normalization_scales_q_v_a_y_rz", scales}};
    configuration["reference_time_evaluation"] =
        "Actual accepted time; frozen finest-grid quintic Hermite q/v with independent -grad(U)/M acceleration";
    configuration["reference_interpolation_coarser_midpoint_scaled_difference"] = interpolationDifference;
    if (adaptive)
    {
        const auto& control = step->m_AdaptiveTimeStep;
        configuration["schema"] = "adaptive-newmark-production-planar-cr-beam-nonlinear-v1";
        configuration["case"] = "finite-amplitude-planar";
        configuration["adaptive_plan"] = adaptivePlan;
        configuration["factory_type"] = "AdaptiveNewmark";
        configuration["factory_time_step_mode"] = "Adaptive";
        configuration["factory_identity_checked"] = true;
        configuration["rtol"] = control.relativeTolerance;
        configuration["atol_q"] = control.absoluteTolerance;
        configuration["atol_v"] = control.velocityAbsoluteTolerance;
        configuration["h0"] = step->m_StepSize;
        configuration["minimum_step"] = control.minimumTimeStep;
        configuration["maximum_step"] = control.maximumTimeStep;
        configuration["absolute_tolerance_unit_qualification"] = adaptivePlan.value("absolute_tolerance_policy");
        configuration["execution"] = "Factory-held production AdaptiveNewmark on actual initialized "
                                     "AnalysisStep/Node/CR; two-half-step accepted state";
    }
    if (diagnosticTolerance)
    {
        configuration["diagnostic_only"] = true;
        configuration["original_frozen_Newton_tolerance"] = PlanarNumber(plan, "newton_tolerance");
        configuration["diagnostic_Newton_tolerance"] = step->m_Tolerance;
        configuration["original_frozen_acceptance_result_revised"] = false;
    }
    if (rk4)
    {
        const double omegaHigh = 4.0 * std::acos(-1.0) / step->m_Time;
        const double maximumStep = adaptive ? step->m_AdaptiveTimeStep.maximumTimeStep : step->m_StepSize;
        if (omegaHigh * maximumStep >= 2.0 * std::sqrt(2.0))
            throw std::runtime_error("RK4 nonlinear beam step exceeds initial highest-frequency stability bound");
        configuration["schema"] = adaptive ? "adaptive-rk4-production-planar-cr-beam-nonlinear-v1"
                                           : "rk4-production-planar-cr-beam-nonlinear-v1";
        configuration["rk_plan"] = adaptivePlan;
        configuration["factory_type"] = adaptive ? "AdaptiveRungeKutta4" : "RungeKutta4";
        configuration["factory_time_step_mode"] = adaptive ? "Adaptive" : "Fixed";
        configuration["factory_identity_checked"] = true;
        configuration["case"] = "finite-amplitude-planar";
        configuration["initial_highest_frequency_rad_s"] = omegaHigh;
        configuration["omega_high_times_maximum_step"] = omegaHigh * maximumStep;
        configuration["linear_explicit_stability_gate"] = true;
        configuration["expected_steps"] = adaptive ? QJsonValue()
                                                   : QJsonValue(2 * adaptivePlan.value("nonlinear")
                                                                        .toObject()
                                                                        .value("fixed_steps_per_period")
                                                                        .toArray()[level]
                                                                        .toInt());
        configuration["h0"] = step->m_StepSize;
        if (!adaptive)
        {
            configuration["rtol"] = QJsonValue();
            configuration["atol_q"] = QJsonValue();
            configuration["atol_v"] = QJsonValue();
        }
        configuration["newton_tolerance"] = QJsonValue();
        configuration["physical_residual_absolute_limit"] = step->m_Tolerance;
        configuration["default_minimum_step"] = QJsonValue();
        configuration["execution"] = "Factory-held production RK4 on actual initialized AnalysisStep/Node/CR, "
                                     "independent physical mass-solve balance and force/potential checks";
        configuration["state_update_scope"] =
            adaptive
                ? "Accepted RK4 two-half-step fine state; augmented CR controller usesp2, no fixed-h order assertion"
                : "Classical RK4 accepted full step; coaxial planar inertia permits q/v/a order4";
    }
    WritePlanarJson(prefix + ".configuration.json", configuration);
    step->OnStepCompleted(0.0);
    QElapsedTimer timer;
    timer.start();
    step->SetRuntimeCallbacks({},
                              [&timer]
                              {
                                  return timer.elapsed() > 120000;
                              });
    bool solved = false;
    QString exception;
    try
    {
        solved = solver->Solve(*step, step->m_Time);
    }
    catch (const std::exception& error)
    {
        exception = QString::fromUtf8(error.what());
    }
    step->ClearRuntimeCallbacks();
    QFile csv(prefix + ".csv");
    if (!csv.open(QIODevice::WriteOnly | QIODevice::Text))
        throw std::runtime_error("Cannot write nonlinear beam CSV");
    QTextStream output(&csv);
    output.setRealNumberPrecision(17);
    output << "time,q_y,q_rz,v_y,v_rz,a_y,a_rz,reference_q_y,reference_q_rz,reference_v_y,reference_v_rz,reference_a_y,"
              "reference_a_rz,residual_norm,mechanical_energy_J";
    if (adaptive || rk4)
        output
            << ",force_formula_scaled_difference,physical_mass_relative_difference,potential_formula_scaled_difference";
    output << '\n';
    std::array<double, 6> errors{};
    double initialError = 0.0, maximumEnergyError = 0.0;
    const double energy0 = PlanarNumber(reference, "initial_energy");
    bool fixed = true;
    bool adaptiveTimeBounds = true;
    QJsonArray actualSteps;
    double previous = 0.0;
    const bool countMatches =
        rk4 ? step->history.size() == static_cast<std::size_t>(2 * adaptivePlan.value("nonlinear")
                                                                       .toObject()
                                                                       .value("fixed_steps_per_period")
                                                                       .toArray()[level]
                                                                       .toInt() +
                                                               1)
            : step->history.size() == static_cast<std::size_t>(rows.size());
    for (std::size_t index = 0; index < step->history.size(); ++index)
    {
        const auto& sample = step->history[index];
        const auto planned =
            index < static_cast<std::size_t>(rows.size()) ? rows[static_cast<int>(index)].toObject() : QJsonObject();
        const auto exact = referenceHistory.State(sample.time, physics);
        output << sample.time;
        for (const auto& pair : sample.state)
            output << ',' << pair[0] << ',' << pair[1];
        for (int component = 0; component < 3; ++component)
        {
            for (int dof = 0; dof < 2; ++dof)
            {
                const double value = exact[component][dof];
                output << ',' << value;
                const int channel = 2 * component + dof;
                const double difference = std::abs(sample.state[component][dof] - value) / scales[channel].toDouble();
                errors[channel] = std::max(errors[channel], difference);
                if (index == 0)
                    initialError = std::max(initialError, difference);
            }
        }
        output << ',' << sample.residual << ',' << sample.energy;
        if (adaptive || rk4)
            output << ',' << sample.forceDifference << ',' << sample.massDifference << ','
                   << sample.potentialDifference;
        output << '\n';
        maximumEnergyError = std::max(maximumEnergyError, std::abs(sample.energy - energy0) / energy0);
        if (!rk4)
            fixed = fixed && !planned.isEmpty() && std::abs(sample.time - PlanarNumber(planned, "time")) <= 1.0e-12;
        if (index > 0)
        {
            const double h = sample.time - previous;
            actualSteps.append(h);
            fixed = fixed && std::abs(h - step->m_StepSize) <= 1.0e-13;
            if (adaptive)
                adaptiveTimeBounds = adaptiveTimeBounds && h > 0.0 &&
                                     h <= step->m_AdaptiveTimeStep.maximumTimeStep + 1e-13 &&
                                     (h >= step->m_AdaptiveTimeStep.minimumTimeStep - 1e-13 ||
                                      std::abs(sample.time - step->m_Time) <= 1e-12);
        }
        previous = sample.time;
    }
    output.flush();
    if (output.status() != QTextStream::Ok)
        throw std::runtime_error("Nonlinear beam CSV stream failed");
    const auto& statistics = rk4 ? rungeKutta->GetTimeStepStatistics() : newmark->GetTimeStepStatistics();
    const bool complete = solved && exception.isEmpty() && (adaptive ? step->history.size() > 1 : countMatches) &&
                          std::abs(previous - step->m_Time) <= 1.0e-12;
    const bool formulas = step->maximumForceDifference <= PlanarNumber(plan, "force_formula_scaled_limit") &&
                          step->maximumMassDifference <= PlanarNumber(plan, "physical_mass_relative_limit") &&
                          step->maximumPotentialDifference <= PlanarNumber(plan, "production_potential_scaled_limit");
    const bool finestAccuracy =
        *std::max_element(errors.begin(), errors.end()) <= PlanarNumber(plan, "finest_max_normalized_state_error");
    const bool adaptiveSequence = adaptiveTimeBounds && !step->history.empty() && step->history.front().time == 0.0 &&
                                  statistics.acceptedSteps == actualSteps.size();
    const bool eligible = complete && (adaptive ? adaptiveSequence : fixed) && formulas && step->normalResiduals &&
                          step->principalFrame && (adaptive || statistics.rejectedAttempts == 0) &&
                          (rk4 || newmark->GetPrecisionLimitedSolveCount() == 0) &&
                          initialError <= PlanarNumber(plan, "initial_state_normalized_limit") &&
                          maximumEnergyError <= PlanarNumber(plan, "relative_mechanical_energy_limit");
    QJsonArray errorArray;
    for (double value : errors)
        errorArray.append(value);
    configuration["solved"] = solved;
    configuration["complete"] = complete;
    configuration["exception"] = exception;
    configuration["fixed_step_sequence"] = fixed;
    configuration["accepted_time_steps"] = actualSteps;
    configuration["accepted_steps"] = statistics.acceptedSteps;
    configuration["rejected_attempts"] = statistics.rejectedAttempts;
    configuration["precision_limited_solves"] =
        rk4 ? QJsonValue() : QJsonValue(static_cast<double>(newmark->GetPrecisionLimitedSolveCount()));
    configuration["normal_residual_gate_passed"] = step->normalResiduals;
    configuration["maximum_residual_norm"] = step->maximumResidual;
    configuration["force_formula_scaled_difference"] = step->maximumForceDifference;
    configuration["physical_mass_relative_difference"] = step->maximumMassDifference;
    configuration["potential_formula_scaled_difference"] = step->maximumPotentialDifference;
    configuration["reference_formula_gates_passed"] = formulas;
    configuration["principal_frame_scope_passed"] = step->principalFrame;
    configuration["initial_max_normalized_error"] = initialError;
    configuration["maximum_relative_energy_error"] = maximumEnergyError;
    configuration["max_normalized_errors_q_v_a_y_rz"] = errorArray;
    configuration["finest_accuracy_passed"] = level == 2 ? QJsonValue(finestAccuracy) : QJsonValue();
    configuration["single_run_passed"] = eligible && (level != 2 || finestAccuracy);
    configuration["status"] = eligible && (level != 2 || finestAccuracy) ? "PASS_RUN_REQUIRES_SUITE_ORDER" : "FAIL";
    configuration["response_csv"] = prefix + ".csv";
    configuration["wall_seconds"] = timer.elapsed() / 1000.0;
    if (diagnosticTolerance)
        configuration["status"] = eligible && (level != 2 || finestAccuracy) ? "DIAGNOSTIC_ONLY_NUMERIC_GATES_PASSED"
                                                                             : "DIAGNOSTIC_ONLY_NUMERIC_GATES_FAILED";
    if (adaptive)
    {
        configuration["fixed_step_sequence"] = QJsonValue();
        configuration["actual_end_time"] = previous;
        configuration["accepted_steps_within_declared_bounds"] = adaptiveTimeBounds;
        configuration["minimum_accepted_time_step"] = statistics.minimumAcceptedTimeStep;
        configuration["maximum_accepted_time_step"] = statistics.maximumAcceptedTimeStep;
        configuration["all_frame_formula_diagnostics_finite"] = true;
        configuration["initial_time_exported"] = !step->history.empty() && step->history.front().time == 0.0;
        configuration["status"] =
            eligible && (level != 2 || finestAccuracy) ? "PASS_RUN_REQUIRES_TOLERANCE_TREND" : "FAIL";
    }
    if (rk4)
    {
        configuration["normal_residual_is_Newton_gate"] = false;
        configuration["all_accepted_physical_mass_solve_balance_passed"] = step->normalResiduals;
        configuration["maximum_physical_force_balance_norm"] = step->maximumResidual;
        configuration["mass_pattern_analysis_count"] = rungeKutta->GetMassPatternAnalysisCount();
        configuration["mass_factorization_count"] = rungeKutta->GetMassFactorizationCount();
        configuration["mass_factorization_reuse_count"] = rungeKutta->GetMassFactorizationReuseCount();
        configuration["mass_assembly_count"] = rungeKutta->GetMassAssemblyCount();
        configuration["mass_assembly_reuse_count"] = rungeKutta->GetMassAssemblyReuseCount();
        configuration["precision_limited_solves"] = QJsonValue();
        configuration["precision_counter_applicable"] = false;
        configuration["initial_time_exported"] = !step->history.empty() && step->history.front().time == 0.0;
        configuration["status"] = eligible && (level != 2 || finestAccuracy)
                                      ? (adaptive ? "PASS_RUN_REQUIRES_TOLERANCE_TREND" : "PASS_RUN_REQUIRES_RK4_ORDER")
                                      : "FAIL";
    }
    WritePlanarJson(prefix + ".result.json", configuration);
    return configuration;
}

QJsonObject NewmarkBeamNonlinear::RunSuite(const QString& directory, bool rk4)
{
    const auto plan =
        ReadPlanarJson("validation-plan.json", "280af7ca6d0f6fc9a1955a736715e1d54c5b0fe0bdad5caeaae14ac0a77a233b");
    std::array<QJsonObject, 3> runs;
    QJsonArray cases, orders;
    bool passed = true;
    for (int level = 0; level < 3; ++level)
    {
        runs[level] = RunCase(directory, level, std::nullopt, false, rk4);
        cases.append(runs[level]);
        passed = passed && runs[level].value("single_run_passed").toBool();
    }
    for (int level = 1; level < 3; ++level)
    {
        const auto coarse = runs[level - 1].value("max_normalized_errors_q_v_a_y_rz").toArray();
        const auto fine = runs[level].value("max_normalized_errors_q_v_a_y_rz").toArray();
        QJsonArray values;
        for (int channel = 0; channel < 6; ++channel)
        {
            const double first = coarse[channel].toDouble(), second = fine[channel].toDouble();
            const double order = first > 0.0 && second > 0.0 ? std::log2(first / second) : 0.0;
            values.append(order);
            passed = passed && std::isfinite(order) && order >= (rk4 ? 3.5 : PlanarNumber(plan, "order_lower")) &&
                     order <= (rk4 ? 4.5 : PlanarNumber(plan, "order_upper"));
        }
        orders.append(QJsonObject{{"fine_level", level}, {"orders_q_v_a_y_rz", values}});
    }
    QJsonObject summary{{"schema", "newmark-production-planar-cr-beam-nonlinear-suite-v1"},
                        {"passed", passed},
                        {"cases", cases},
                        {"orders", orders},
                        {"status", passed ? "PASS_FINITE_AMPLITUDE_PLANAR_SHORT_WINDOW" : "FAIL"},
                        {"general_spatial_large_rotation_certified", false},
                        {"full_analysis_step_solve_wrapper_used", false},
                        {"gates", QJsonObject{{"independent_reference_selfchecked", true},
                                              {"actual_nonlinear_force_potential_and_mass_checked", true},
                                              {"complete_fixed_steps_zero_reject_precision_normal_residuals", true},
                                              {"all_six_state_channels_finest_and_two_order_checks", true},
                                              {"all_passed", passed}}}};
    if (rk4)
    {
        summary["schema"] = "rk4-production-planar-cr-beam-nonlinear-suite-v1";
        summary["status"] = passed ? "PASS_FINITE_AMPLITUDE_PLANAR_RK4_SCOPE" : "FAIL";
        summary["fixed_order_bounds"] = QJsonArray{3.5, 4.5};
    }
    WritePlanarJson(QDir(directory).filePath("summary.json"), summary);
    return summary;
}

static QJsonObject RunAdaptiveBeamNonlinearSuite(const QString& directory, bool rk4 = false)
{
    std::array<QJsonObject, 3> runs;
    QJsonArray cases, trends;
    bool passed = true;
    for (int level = 0; level < 3; ++level)
    {
        runs[level] = NewmarkBeamNonlinear::RunCase(directory, level, std::nullopt, true, rk4);
        cases.append(runs[level]);
        passed = passed && runs[level].value("single_run_passed").toBool();
    }
    for (int level = 1; level < 3; ++level)
    {
        const auto coarse = runs[level - 1].value("max_normalized_errors_q_v_a_y_rz").toArray();
        const auto fine = runs[level].value("max_normalized_errors_q_v_a_y_rz").toArray();
        QJsonArray ratios;
        for (int group = 0; group < 3; ++group)
        {
            const double first = std::max(coarse[2 * group].toDouble(), coarse[2 * group + 1].toDouble());
            const double second = std::max(fine[2 * group].toDouble(), fine[2 * group + 1].toDouble());
            const double ratio = first > 0.0     ? second / first
                                 : second == 0.0 ? 0.0
                                                 : std::numeric_limits<double>::infinity();
            ratios.append(std::isfinite(ratio) ? QJsonValue(ratio) : QJsonValue());
            passed = passed && std::isfinite(ratio) && ratio <= 0.9;
        }
        trends.append(QJsonObject{{"fine_level", level}, {"error_ratios_q_v_a_envelopes", ratios}});
    }
    QJsonObject summary{{"schema", "adaptive-newmark-production-planar-cr-beam-nonlinear-suite-v1"},
                        {"cases", cases},
                        {"tolerance_trends", trends},
                        {"passed", passed},
                        {"status", passed ? "PASS_FINITE_AMPLITUDE_PLANAR_ADAPTIVE_SCOPE" : "FAIL"},
                        {"fixed_step_order_assessed", false},
                        {"local_error_rejections_allowed", true},
                        {"general_spatial_large_rotation_certified", false}};
    if (rk4)
    {
        summary["schema"] = "adaptive-rk4-production-planar-cr-beam-nonlinear-suite-v1";
        summary["status"] = passed ? "PASS_FINITE_AMPLITUDE_PLANAR_ADAPTIVE_RK4_SCOPE" : "FAIL";
    }
    WritePlanarJson(QDir(directory).filePath("summary.json"), summary);
    return summary;
}

std::optional<int> VerifyNewmarkBeamNonlinear(const QStringList& arguments)
{
    const int rkAdaptiveOption = arguments.indexOf("--verify-adaptive-rk4-beam-nonlinear");
    const int rkOption = arguments.indexOf("--verify-rk4-beam-nonlinear");
    const bool rk4 = rkAdaptiveOption >= 0 || rkOption >= 0;
    const int adaptiveOption = arguments.indexOf("--verify-adaptive-newmark-beam-nonlinear");
    const bool adaptive = rk4 ? rkAdaptiveOption >= 0 : adaptiveOption >= 0;
    const int option = rk4 ? (adaptive ? rkAdaptiveOption : rkOption)
                           : (adaptive ? adaptiveOption : arguments.indexOf("--verify-newmark-beam-nonlinear"));
    if (option < 0)
        return std::nullopt;
    try
    {
        if (option + 1 >= arguments.size() || arguments[option + 1].startsWith("--"))
            throw std::runtime_error("Nonlinear beam verification needs a fresh output directory");
        const QString directory = QDir(arguments[option + 1]).absolutePath();
        const int selected = arguments.indexOf("--beam-nonlinear-level");
        if (selected >= 0)
        {
            bool valid = false;
            const int level = selected + 1 < arguments.size() ? arguments[selected + 1].toInt(&valid) : -1;
            if (!valid)
                throw std::runtime_error("Invalid --beam-nonlinear-level");
            return NewmarkBeamNonlinear::RunCase(directory, level, std::nullopt, adaptive, rk4)
                           .value("single_run_passed")
                           .toBool()
                       ? 0
                       : 2;
        }
        if (adaptive)
            return RunAdaptiveBeamNonlinearSuite(directory, rk4).value("passed").toBool() ? 0 : 2;
        return NewmarkBeamNonlinear::RunSuite(directory, rk4).value("passed").toBool() ? 0 : 2;
    }
    catch (const std::exception& error)
    {
        QTextStream(stderr) << error.what() << Qt::endl;
        return 2;
    }
}

namespace
{
/** @brief 真实 CR 梁有限幅值平面弯曲的独立非线性半离散参考及 Newmark 二阶验收。
 * @details y0=.2m、theta0=0、无外力阻尼，实际Node/AnalysisStep/CR单元；独立能量方程DOP853双精度参考。
 *          预先冻结2个最高频周期短窗及h=Th/128,/256,/512，工厂最小步显式改为1e-8并记录默认差异。
 *          各接受态核对独立势能梯度、物理质量、势能、正常残差和能量；全q/v/a六分量最细门.0015、阶1.8–2.2。
 *          必须完整且零拒步与precision接受，不认证一般空间大转动或连续体空间离散精度。
 * @author   yangqiuyou
 * @date     2026.10.19
 */
TEST(PlanarBeamFiniteAmplitude, NewmarkFixed)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    directory.setAutoRemove(false);
    RecordProperty("temporary_output_directory", directory.path().toStdString());
    QJsonObject result;
    ASSERT_NO_THROW(result = NewmarkBeamNonlinear::RunSuite(directory.path()));
    EXPECT_TRUE(result.value("passed").toBool())
        << "Full nonlinear history/fixed-step/Newton/order gates failed; diagnostics: "
        << directory.path().toStdString() << "/summary.json";
    directory.setAutoRemove(!HasFailure());
}
}
