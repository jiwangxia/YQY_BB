#include "Dynamics/PlanarBeam/SmallAmplitudeTest.h"

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
using SolverNameSpace::SpMat;
using SolverNameSpace::Vec;
using BeamMatrix2 = Eigen::Matrix2d;
using BeamVector2 = Eigen::Vector2d;

static double Number(const QJsonObject& object, const char* key)
{
    const QJsonValue value = object.value(QLatin1String(key));
    if (!value.isDouble() || !std::isfinite(value.toDouble()))
        throw std::runtime_error(std::string("Missing finite beam input: ") + key);
    return value.toDouble();
}

static QJsonObject ReadJson(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error((QStringLiteral("Missing beam reference/input: ") + path).toStdString());
    const QByteArray bytes = file.readAll();
    const QByteArray expectedHash =
        QFileInfo(path).fileName() == QStringLiteral("input.json")
            ? QByteArray("c06c2e667ae7febfa28703ce7b85a290638b956a47a7de5b7513b64be76e3ffd")
            : QByteArray("59aba9f77666872aa88e022aa2990e3f7312b504f3a58f8176971124a971981d");
    if (QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex() != expectedHash)
        throw std::runtime_error("Frozen beam input or validation plan SHA differs");
    QJsonParseError error;
    const QJsonDocument document = QJsonDocument::fromJson(bytes, &error);
    if (error.error != QJsonParseError::NoError || !document.isObject())
        throw std::runtime_error("Invalid beam input JSON");
    return document.object();
}

static void WriteJson(const QString& path, const QJsonObject& object)
{
    QFile file(path);
    const QByteArray data = QJsonDocument(object).toJson(QJsonDocument::Indented);
    if (!file.open(QIODevice::WriteOnly) || file.write(data) != data.size())
        throw std::runtime_error("Cannot write beam verification JSON");
}

static QString InputDirectory()
{
    return QDir(TestPaths::TestRoot()).filePath(QStringLiteral("TestModels/Dynamics/PlanarBeam/SmallAmplitude"));
}

struct Reference
{
    double length;
    double young;
    double density;
    double width;
    double height;
    double amplitude;
    double ratio;
    double omega;
    double lowOmega;
    double highOmega;
    double period;
    double duration;
    double zeta;
    double rayleighMass;
    double rayleighStiffness;
    BeamMatrix2 mass;
    BeamMatrix2 stiffness;
    BeamMatrix2 damping;

    Reference(const QJsonObject& input, bool damped, double highModeAngularAmplitude = 0.0)
        : length(Number(input, "length"))
        , young(Number(input, "young"))
        , density(Number(input, "density"))
        , width(Number(input, "width"))
        , height(Number(input, "height"))
        , amplitude(Number(input, "transverse_amplitude_scale"))
        , zeta(damped ? Number(input, "damped_modal_ratio") : 0.0)
    {
        const double area = width * height;
        const double iz = height * std::pow(width, 3) / 12.0;
        const double rigidity = young * iz;
        mass = BeamMatrix2::Zero();
        mass(0, 0) = density * area * length / 3.0;
        mass(1, 1) = density * iz * length / 3.0;
        stiffness << 12.0 * rigidity / std::pow(length, 3), -6.0 * rigidity / (length * length),
            -6.0 * rigidity / (length * length), 4.0 * rigidity / length;
        const double a = stiffness(0, 0) / mass(0, 0);
        const double d = stiffness(1, 1) / mass(1, 1);
        const double b = stiffness(0, 1) / std::sqrt(mass(0, 0) * mass(1, 1));
        const double discriminant = std::hypot(a - d, 2.0 * b);
        // 用行列式求小根，避免高低弯曲频率分离时相减丢失有效位。
        const double high = 0.5 * (a + d + discriminant);
        const double low = (a * d - b * b) / high;
        lowOmega = std::sqrt(low);
        highOmega = std::sqrt(high);
        const bool highMode = highModeAngularAmplitude > 0.0;
        omega = highMode ? highOmega : lowOmega;
        ratio = -(stiffness(0, 0) - (highMode ? high : low) * mass(0, 0)) / stiffness(0, 1);
        if (highMode)
            amplitude = highModeAngularAmplitude / std::abs(ratio);
        period = 2.0 * std::acos(-1.0) / omega;
        duration = Number(input, "duration_periods") * period;
        rayleighMass = 2.0 * zeta * lowOmega * highOmega / (lowOmega + highOmega);
        rayleighStiffness = 2.0 * zeta / (lowOmega + highOmega);
        damping = rayleighMass * mass + rayleighStiffness * stiffness;
    }

    std::array<BeamVector2, 3> State(double time) const
    {
        const double sigma = zeta * omega;
        const double wd = omega * std::sqrt(1.0 - zeta * zeta);
        const double envelope = amplitude * omega * std::exp(-sigma * time);
        const double q = envelope * std::sin(wd * time) / wd;
        const double v = envelope * (std::cos(wd * time) - sigma * std::sin(wd * time) / wd);
        const double a = -2.0 * sigma * v - omega * omega * q;
        const BeamVector2 shape(1.0, ratio);
        return {shape * q, shape * v, shape * a};
    }

    std::array<BeamVector2, 3> Scales() const
    {
        const BeamVector2 q(amplitude, amplitude * std::abs(ratio));
        return {q, q * omega, q * omega * omega};
    }
};

struct Sample
{
    double time;
    std::array<BeamVector2, 3> state;
    double residual;
    double energy;
    double forceDifference = 0.0;
    double massDifference = 0.0;
    double potentialDifference = 0.0;
};

class BeamProbeStep final : public AnalysisStep
{
public:
    bool PushStateCheckpoint() override
    {
        const bool pushed = AnalysisStep::PushStateCheckpoint();
        if (pushed)
            ++checkpointPushes;
        return pushed;
    }

    void RollbackDynamicStep() override
    {
        ++rollbacks;
        AnalysisStep::RollbackDynamicStep();
    }

    void RecordStepIterations(double time, int iterations) override
    {
        maximumIterations = std::max(maximumIterations, iterations);
        totalIterations += iterations;
        AnalysisStep::RecordStepIterations(time, iterations);
    }

    bool ShouldPreferGpuLinearSolver() const override
    {
        return false;
    }

    void OnStepCompleted(double time) override
    {
        // 反力及正式输出仍走生产路径，随后只读取已接受节点并复核系统贡献。
        AnalysisStep::OnStepCompleted(time);
        const auto& tip = GetStructure()->m_Nodes.at(2);
        const int y = tip->m_DOF[1] - GetFixedDofs();
        const int rz = tip->m_DOF[5] - GetFixedDofs();
        if (GetFreeDofs() != 2 || y < 0 || rz < 0 || y == rz)
            throw std::runtime_error("Beam bending free DOF mapping differs from the declared Y/RZ model");
        SolverNameSpace::SystemEvaluationRequest request{
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ResidualState) |
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ExternalForce)};
        request.time = time;
        if (adaptiveReference)
        {
            request.Include(SolverNameSpace::SystemContribution::Mass);
            request.Include(SolverNameSpace::SystemContribution::InternalForce);
        }
        if (history.empty())
        {
            request.Include(SolverNameSpace::SystemContribution::Stiffness);
            request.Include(SolverNameSpace::SystemContribution::Mass);
            request.Include(SolverNameSpace::SystemContribution::Damping);
        }
        SolverNameSpace::SystemEvaluationResult system;
        EvaluateSystem(request, system);
        Vec residual;
        ComputeResidual(system.externalForce, residual);
        if (history.empty())
        {
            const int indices[2] = {y, rz};
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                {
                    initialMass(i, j) = system.mass.coeff(indices[i], indices[j]);
                    initialStiffness(i, j) = system.stiffness.coeff(indices[i], indices[j]);
                    initialDamping(i, j) = system.damping.coeff(indices[i], indices[j]);
                }
        }
        const double angle = std::atan2(tip->m_Rg(1, 0), tip->m_Rg(0, 0));
        Sample sample{time,
                      {BeamVector2(tip->m_Displacement[1], angle), BeamVector2(tip->m_Velocity[1], tip->m_Velocity[5]),
                       BeamVector2(tip->m_Acceleration[1], tip->m_Acceleration[5])},
                      residual.norm(),
                      0.0};
        SpMat physicalMass;
        AssembleMassMatrix(physicalMass);
        Vec q, v, a;
        GetState(q, v, a);
        const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(GetStructure()->m_Elements.at(1));
        const double kinetic = 0.5 * v.dot(physicalMass * v);
        const double strain = beam->GetStrainEnergy();
        const std::array<Node, 2> nodesBefore{*beam->m_pNode[0].lock(), *beam->m_pNode[1].lock()};
        const double stressBefore = beam->m_Stress;
        const Vec internalForceBefore = beam->m_inforce;
        double exportedKinetic = 0.0, exportedStrain = 0.0;
        if (!beam->TryGetKineticEnergy(exportedKinetic) || !beam->TryGetStrainEnergy(exportedStrain) ||
            !std::isfinite(exportedKinetic) || !std::isfinite(exportedStrain) ||
            std::abs(exportedKinetic - kinetic) > 5.0e-12 * std::max(1.0, std::abs(kinetic)) ||
            std::abs(exportedStrain - strain) > 5.0e-12 * std::max(1.0, std::abs(strain)))
            throw std::runtime_error("Beam energy query differs from physical mass or elastic potential");
        // 在有转动速度的接受态检查接口，并严格保持节点、内力和应力缓存。
        if (beam->m_Stress != stressBefore || !beam->m_inforce.isApprox(internalForceBefore, 0.0))
            throw std::runtime_error("Beam energy query changed force or stress");
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
                throw std::runtime_error("Beam energy query changed accepted node state");
        }
        sample.energy = kinetic + strain;
        if (adaptiveReference)
        {
            const auto& ref = *adaptiveReference;
            const double area = ref.width * ref.height;
            const double inertia = ref.height * std::pow(ref.width, 3) / 12.0;
            const double ell = std::hypot(ref.length, sample.state[0][0]);
            const double extension = sample.state[0][0] * sample.state[0][0] / (ell + ref.length);
            const double phi = std::atan2(sample.state[0][0], ref.length);
            const double theta = sample.state[0][1];
            const double bending = ref.young * inertia;
            const BeamVector2 independentForce(ref.young * area / ref.length * extension * sample.state[0][0] / ell +
                                                   2.0 * bending / ref.length * (6.0 * phi - 3.0 * theta) * ref.length /
                                                       (ell * ell),
                                               2.0 * bending / ref.length * (-3.0 * phi + 2.0 * theta));
            const double potential = ref.young * area / (2.0 * ref.length) * extension * extension +
                                     2.0 * bending / ref.length * (3.0 * phi * phi - 3.0 * phi * theta + theta * theta);
            const BeamVector2 internal(system.internalForce[y], system.internalForce[rz]);
            BeamMatrix2 currentMass;
            const int indices[2] = {y, rz};
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                    currentMass(i, j) = physicalMass.coeff(indices[i], indices[j]);
            const double actualPotential =
                std::dynamic_pointer_cast<ElementBeam_CR>(GetStructure()->m_Elements.at(1))->GetStrainEnergy();
            sample.forceDifference = (internal - independentForce).norm() / std::max(1.0, independentForce.norm());
            sample.massDifference = (currentMass - ref.mass).norm() / ref.mass.norm();
            sample.potentialDifference =
                std::max(std::abs(actualPotential - potential), std::abs(exportedStrain - potential)) /
                std::max(1.0, std::abs(potential));
            if (!currentMass.allFinite() || !internal.allFinite() || !independentForce.allFinite() ||
                !std::isfinite(sample.forceDifference) || !std::isfinite(sample.massDifference) ||
                !std::isfinite(sample.potentialDifference))
                throw std::runtime_error("Adaptive beam mass/force/potential diagnostics are nonfinite");
            maximumForceDifference = std::max(maximumForceDifference, sample.forceDifference);
            maximumMassDifference = std::max(maximumMassDifference, sample.massDifference);
            maximumPotentialDifference = std::max(maximumPotentialDifference, sample.potentialDifference);
        }
        if (!std::isfinite(time) || !std::isfinite(sample.residual) || !std::isfinite(sample.energy) ||
            !sample.state[0].allFinite() || !sample.state[1].allFinite() || !sample.state[2].allFinite() ||
            (!history.empty() && time <= history.back().time))
            throw std::runtime_error("Beam bending history is nonfinite or time is not increasing");
        maximumResidual = std::max(maximumResidual, sample.residual);
        normalResiduals =
            normalResiduals && sample.residual <= m_Tolerance * std::max(1.0, system.externalForce.norm());
        maximumRotationMismatch = std::max(maximumRotationMismatch, std::abs(angle - tip->m_Displacement[5]));
        history.push_back(sample);
    }

    BeamMatrix2 initialMass = BeamMatrix2::Zero();
    BeamMatrix2 initialStiffness = BeamMatrix2::Zero();
    BeamMatrix2 initialDamping = BeamMatrix2::Zero();
    std::vector<Sample> history;
    int rollbacks = 0;
    int maximumIterations = 0;
    long long totalIterations = 0;
    double maximumResidual = 0.0;
    double maximumRotationMismatch = 0.0;
    bool normalResiduals = true;
    int checkpointPushes = 0;
    std::optional<Reference> adaptiveReference;
    double maximumForceDifference = 0.0;
    double maximumMassDifference = 0.0;
    double maximumPotentialDifference = 0.0;
};

static void Constrain(StructureData& model, const std::shared_ptr<Node>& node, int direction)
{
    auto constraint = std::make_shared<Constraint>();
    constraint->m_Id = static_cast<int>(model.m_Constraint.size()) + 1;
    constraint->m_pNode = node;
    constraint->m_Direction = static_cast<EnumKeyword::Direction>(direction);
    model.m_Constraint.emplace(constraint->m_Id, constraint);
}

static std::shared_ptr<StructureData> BuildModel(const Reference& reference, bool damped,
                                                 const std::shared_ptr<BeamProbeStep>& step)
{
    auto model = std::make_shared<StructureData>();
    auto material = std::make_shared<Material>();
    material->m_Id = 1;
    material->m_Young = reference.young;
    material->m_Poisson = 0.3;
    material->m_Density = reference.density;
    auto section = std::make_shared<SectionRectangle>();
    section->m_Id = 1;
    section->m_Width = reference.width;
    section->m_Height = reference.height;
    section->Calculate_Area();
    auto property = std::make_shared<Property>();
    property->m_Id = 1;
    property->m_pMaterial = material;
    property->m_pSection = section;
    model->m_Material[1] = material;
    model->m_Section[1] = section;
    model->m_Property[1] = property;
    for (int i = 0; i < 2; ++i)
    {
        auto node = std::make_shared<Node>();
        node->m_Id = i + 1;
        node->SetExactNumDOFs(6);
        node->m_X = reference.length * i;
        model->m_Nodes[node->m_Id] = node;
    }
    auto beam = std::make_shared<ElementBeam_CR>();
    beam->m_Id = 1;
    beam->m_pNode[0] = model->m_Nodes.at(1);
    beam->m_pNode[1] = model->m_Nodes.at(2);
    beam->m_pProperty = property;
    beam->q0 = Eigen::Vector3d::UnitY();
    model->m_Elements[1] = beam;
    for (int direction = 0; direction < 6; ++direction)
        Constrain(*model, model->m_Nodes.at(1), direction);
    for (int direction : {0, 2, 3, 4})
        Constrain(*model, model->m_Nodes.at(2), direction);
    step->SetAssemblyThreadCountOverride(1);
    step->SetPardisoThreadCountOverride(1);
    step->m_StructuralDamping.settings.enabled = damped;
    step->m_StructuralDamping.settings.translationDampingRatio = reference.zeta;
    step->m_StructuralDamping.settings.torsionDampingRatio = reference.zeta;
    step->m_StructuralDamping.settings.translationReferenceModes = {};
    step->m_StructuralDamping.settings.torsionReferenceModes = {};
    step->SetStructure(model);
    const std::weak_ptr<StructureData> weak = model;
    step->SetInitialDynamicStateCallback(
        [weak, reference](_OUT QString& error)
        {
            const auto structure = weak.lock();
            if (!structure)
            {
                error = QStringLiteral("梁验证模型已失效");
                return false;
            }
            const auto& node = structure->m_Nodes.at(2);
            // 初始位移与姿态均为零；初始加速度由正式动力初始化恢复，不能手填零。
            node->m_Velocity[1] = reference.amplitude * reference.omega;
            node->m_Velocity[5] = reference.ratio * reference.amplitude * reference.omega;
            node->m_OmegaMaterial[2] = node->m_Velocity[5];
            node->m_Velocity_n = node->m_Velocity;
            node->m_OmegaMaterial_n = node->m_OmegaMaterial;
            return true;
        });
    model->m_AnalysisStep[1] = step;
    model->m_OutputControl.m_StreamResult = false;
    return model;
}

static QJsonArray MatrixJson(const BeamMatrix2& matrix)
{
    return QJsonArray{QJsonArray{matrix(0, 0), matrix(0, 1)}, QJsonArray{matrix(1, 0), matrix(1, 1)}};
}

static QJsonArray ErrorJson(const std::array<BeamVector2, 3>& errors)
{
    QJsonArray array;
    for (const auto& pair : errors)
        array.append(QJsonArray{pair[0], pair[1]});
    return array;
}
}

QJsonObject NewmarkBeamBending::RunCase(const QString& directory, bool damped, int level,
                                        std::optional<double> diagnosticTolerance, bool adaptive, bool rk4)
{
    if (level < 0 || level > 2 || !QDir().mkpath(directory))
        throw std::runtime_error("Invalid beam verification level or output directory");
    const QJsonObject input = ReadJson(QDir(InputDirectory()).filePath(QStringLiteral("input.json")));
    const QJsonObject plan = ReadJson(QDir(InputDirectory()).filePath(QStringLiteral("validation-plan.json")));
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
    const Reference reference(input, damped,
                              rk4 ? Number(adaptivePlan.value("bending").toObject(), "angular_amplitude_scale") : 0.0);
    const QString stem = QStringLiteral("%1-level%2").arg(damped ? "modal-damped" : "undamped").arg(level);
    const QString prefix = QDir(directory).filePath(stem);
    for (const QString& suffix :
         {QStringLiteral(".csv"), QStringLiteral(".result.json"), QStringLiteral(".configuration.json")})
        if (QFileInfo::exists(prefix + suffix))
            throw std::runtime_error("Beam records already exist; use a fresh directory");
    const auto methodSettings = adaptivePlan.value("bending").toObject();
    const double stepSize =
        adaptive ? reference.period * Number(methodSettings, rk4 ? "initial_step_over_reference_period"
                                                                 : "initial_step_over_low_period")
        : rk4    ? reference.period / methodSettings.value("fixed_steps_per_period").toArray()[level].toDouble()
                 : reference.period / (80.0 * std::pow(2.0, level));
    auto step = std::make_shared<BeamProbeStep>();
    step->m_Id = 1;
    step->m_Type = EnumKeyword::StepType::DYNAMIC;
    step->m_DynamicSolverType =
        rk4 ? (adaptive ? SolverNameSpace::SolverType::AdaptiveRungeKutta4 : SolverNameSpace::SolverType::RungeKutta4)
            : (adaptive ? SolverNameSpace::SolverType::AdaptiveNewmark : SolverNameSpace::SolverType::Newmark);
    step->isDynamic = true;
    step->SolverNameSpace::IAnalysisModel::isDynamic = true;
    step->m_Time = reference.duration;
    step->m_StepSize = stepSize;
    if (diagnosticTolerance && (!std::isfinite(*diagnosticTolerance) || *diagnosticTolerance <= 0.0))
        throw std::runtime_error("Beam diagnostic tolerance must be finite and positive");
    step->m_Tolerance = diagnosticTolerance.value_or(rk4 ? Number(adaptivePlan, "physical_residual_absolute_limit")
                                                         : Number(plan, "newton_residual_tolerance"));
    step->m_MaxIterations = 40;
    if (adaptive)
    {
        const auto settings = adaptivePlan.value("bending").toObject();
        const auto scales = reference.Scales();
        auto& control = step->m_AdaptiveTimeStep;
        control.relativeTolerance = adaptivePlan.value("relative_tolerances").toArray()[level].toDouble();
        control.velocityRelativeTolerance = control.relativeTolerance;
        const double fraction = Number(adaptivePlan, "absolute_tolerance_scale_fraction");
        control.absoluteTolerance = control.relativeTolerance * fraction * scales[0].minCoeff();
        control.velocityAbsoluteTolerance = control.relativeTolerance * fraction * scales[1].minCoeff();
        control.minimumTimeStep = reference.period * Number(settings, rk4 ? "minimum_step_over_reference_period"
                                                                          : "minimum_step_over_low_period");
        control.maximumTimeStep = reference.period * Number(settings, rk4 ? "maximum_step_over_reference_period"
                                                                          : "maximum_step_over_low_period");
        control.includeVelocityInError = true;
        control.safetyFactor = Number(adaptivePlan, "safety_factor");
        control.minimumScaleFactor = Number(adaptivePlan, "minimum_scale_factor");
        control.maximumGrowthFactor = Number(adaptivePlan, "maximum_growth_factor");
        control.solveFailureFactor = Number(adaptivePlan, "solve_failure_factor");
        control.maximumRejectedAttempts = static_cast<int>(Number(adaptivePlan, "maximum_rejected_attempts_per_time"));
        step->adaptiveReference = reference;
    }
    if (rk4)
        step->adaptiveReference = reference;
    const auto identity = SolverNameSpace::SolverFactory::Create(*step);
    const auto* newmark = dynamic_cast<const SolverNameSpace::SolverNewmark*>(identity.get());
    const auto* rungeKutta = dynamic_cast<const SolverNameSpace::SolverRungeKutta4*>(identity.get());
    if (rk4 ? (!rungeKutta || rungeKutta->GetType() != step->m_DynamicSolverType)
            : (!newmark || newmark->GetType() != step->m_DynamicSolverType ||
               newmark->GetParams().timeStepMode !=
                   (adaptive ? SolverNameSpace::TimeStepMode::Adaptive : SolverNameSpace::TimeStepMode::Fixed) ||
               newmark->GetParams().beta != 0.25 || newmark->GetParams().gamma != 0.5))
        throw std::runtime_error("Factory fixed Newmark identity differs from the frozen method");
    QJsonObject configuration{
        {"schema", "newmark-production-beam-bending-v1"},
        {"input", input},
        {"plan", plan},
        {"damped", damped},
        {"level", level},
        {"step", stepSize},
        {"duration", reference.duration},
        {"omega_low", reference.lowOmega},
        {"omega_high", reference.highOmega},
        {"mode_tip_rotation_over_translation", reference.ratio},
        {"independent_M", MatrixJson(reference.mass)},
        {"independent_K", MatrixJson(reference.stiffness)},
        {"independent_C", MatrixJson(reference.damping)},
        {"execution", "actual AnalysisStep::Solve(false), production CR beam and Node"},
        {"reference_scope",
         "derived small-amplitude two-DOF semidiscrete bending; not Aster Hermite M or large-rotation certification"},
        {"precision_limited_count", QJsonValue()},
        {"precision_observation", "internal count unavailable from Solve; retain process stdout and require no "
                                  "newmark_precision_limited event; every accepted residual independently gated"}};
    if (diagnosticTolerance)
    {
        configuration["diagnostic_only"] = true;
        configuration["original_frozen_Newton_tolerance"] = Number(plan, "newton_residual_tolerance");
        configuration["diagnostic_Newton_tolerance"] = step->m_Tolerance;
        configuration["original_frozen_acceptance_result_revised"] = false;
    }
    if (adaptive)
    {
        const auto& control = step->m_AdaptiveTimeStep;
        configuration["schema"] = "adaptive-newmark-production-beam-bending-v1";
        configuration["case"] = damped ? "modal-damped" : "undamped";
        configuration["adaptive_plan"] = adaptivePlan;
        configuration["factory_type"] = "AdaptiveNewmark";
        configuration["factory_time_step_mode"] = "Adaptive";
        configuration["factory_identity_checked"] = true;
        configuration["rtol"] = control.relativeTolerance;
        configuration["atol_q"] = control.absoluteTolerance;
        configuration["atol_v"] = control.velocityAbsoluteTolerance;
        configuration["h0"] = stepSize;
        configuration["minimum_step"] = control.minimumTimeStep;
        configuration["maximum_step"] = control.maximumTimeStep;
        configuration["absolute_tolerance_unit_qualification"] = adaptivePlan.value("absolute_tolerance_policy");
        configuration["precision_limited_solves"] = QJsonValue();
        configuration["process_precision_review_pending"] = true;
        configuration["actual_factory_solver_statistics_visible"] = false;
        configuration["state_update_scope"] =
            "Accepted two-half-step state; no full-step Newmark update identity or fixed-grid gate";
    }
    if (rk4)
    {
        const double maximumStep = adaptive ? step->m_AdaptiveTimeStep.maximumTimeStep : stepSize;
        const bool stable = reference.highOmega * maximumStep < 2.0 * std::sqrt(2.0);
        if (!stable)
            throw std::runtime_error("RK4 beam step exceeds the declared highest-frequency stability bound");
        configuration["schema"] =
            adaptive ? "adaptive-rk4-production-beam-bending-v1" : "rk4-production-beam-bending-v1";
        configuration["rk_plan"] = adaptivePlan;
        configuration["factory_type"] = adaptive ? "AdaptiveRungeKutta4" : "RungeKutta4";
        configuration["factory_time_step_mode"] = adaptive ? "Adaptive" : "Fixed";
        configuration["factory_identity_checked"] = true;
        configuration["case"] = damped ? "modal-damped" : "undamped";
        configuration["reference_mode"] = "highest bending eigenpair";
        configuration["reference_omega"] = reference.omega;
        configuration["reference_transverse_scale"] = reference.amplitude;
        configuration["reference_angular_scale"] = std::abs(reference.ratio) * reference.amplitude;
        configuration["omega_high_times_maximum_step"] = reference.highOmega * maximumStep;
        configuration["linear_explicit_stability_gate"] = stable;
        configuration["expected_steps"] =
            adaptive ? QJsonValue()
                     : QJsonValue(3 * methodSettings.value("fixed_steps_per_period").toArray()[level].toInt());
        configuration["h0"] = stepSize;
        if (!adaptive)
        {
            configuration["rtol"] = QJsonValue();
            configuration["atol_q"] = QJsonValue();
            configuration["atol_v"] = QJsonValue();
        }
        configuration["physical_residual_absolute_limit"] = step->m_Tolerance;
        configuration["newton_tolerance"] = QJsonValue();
        configuration["precision_limited_count"] = QJsonValue();
        configuration["precision_limited_solves"] = QJsonValue();
        configuration["precision_observation"] = "Not applicable: explicit RK has no Newton precision-limited counter";
        configuration["process_precision_review_pending"] = false;
        configuration["state_update_scope"] =
            adaptive
                ? "RK4 accepted two-half-step fine state; augmented CR controller order2, no fixed-h order assertion"
                : "Classical RK4 full accepted step; planar coaxial inertia leaves q/v/a fourth-order expectation";
    }
    WriteJson(prefix + QStringLiteral(".configuration.json"), configuration);
    auto model = BuildModel(reference, damped, step);
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
        solved = step->Solve(false);
    }
    catch (const std::exception& error)
    {
        exception = QString::fromUtf8(error.what());
    }
    step->ClearRuntimeCallbacks();
    QFile csv(prefix + QStringLiteral(".csv"));
    if (!csv.open(QIODevice::WriteOnly | QIODevice::Text))
        throw std::runtime_error("Cannot write beam bending time history");
    QTextStream output(&csv);
    output.setRealNumberPrecision(17);
    output << "time,q_y,q_rz,v_y,v_rz,a_y,a_rz,reference_q_y,reference_q_rz,reference_v_y,reference_v_rz,reference_a_y,"
              "reference_a_rz,residual_norm,mechanical_energy_J";
    if (adaptive || rk4)
        output
            << ",force_formula_scaled_difference,physical_mass_relative_difference,potential_formula_scaled_difference";
    output << '\n';
    std::array<BeamVector2, 3> errors{BeamVector2::Zero(), BeamVector2::Zero(), BeamVector2::Zero()};
    const auto scales = reference.Scales();
    QJsonArray actualSteps;
    bool fixed = true;
    bool envelope = true;
    bool adaptiveTimeBounds = true;
    double previous = 0.0;
    double initialError = 0.0;
    for (std::size_t i = 0; i < step->history.size(); ++i)
    {
        const Sample& sample = step->history[i];
        const auto exact = reference.State(sample.time);
        output << sample.time;
        for (const auto& pair : sample.state)
            output << ',' << pair[0] << ',' << pair[1];
        for (const auto& pair : exact)
            output << ',' << pair[0] << ',' << pair[1];
        output << ',' << sample.residual << ',' << sample.energy;
        if (adaptive || rk4)
            output << ',' << sample.forceDifference << ',' << sample.massDifference << ','
                   << sample.potentialDifference;
        output << '\n';
        for (int component = 0; component < 3; ++component)
        {
            const BeamVector2 normalized =
                (sample.state[component] - exact[component]).cwiseQuotient(scales[component]).cwiseAbs();
            errors[component] = errors[component].cwiseMax(normalized);
            if (i == 0)
                initialError = std::max(initialError, normalized.maxCoeff());
        }
        envelope = envelope && (sample.state[0].cwiseAbs().array() <= (2.0 * scales[0]).array()).all();
        if (i > 0)
        {
            const double h = sample.time - previous;
            actualSteps.append(h);
            fixed = fixed && std::abs(h - stepSize) <= 1.0e-10 * stepSize + 1.0e-13;
            if (adaptive)
                adaptiveTimeBounds = adaptiveTimeBounds && h > 0.0 &&
                                     h <= step->m_AdaptiveTimeStep.maximumTimeStep + 1e-13 &&
                                     (h >= step->m_AdaptiveTimeStep.minimumTimeStep - 1e-13 ||
                                      std::abs(sample.time - reference.duration) <= 1e-10);
        }
        previous = sample.time;
    }
    output.flush();
    if (output.status() != QTextStream::Ok)
        throw std::runtime_error("Beam history stream failed");
    const double massError = (step->initialMass - reference.mass).norm() / reference.mass.norm();
    const double stiffnessError = (step->initialStiffness - reference.stiffness).norm() / reference.stiffness.norm();
    const double dampingError =
        (step->initialDamping - reference.damping).norm() / std::max(1.0, reference.damping.norm());
    const BeamVector2 shape(1.0, reference.ratio);
    const double projectedDamping =
        shape.dot(step->initialDamping * shape) / (2.0 * reference.omega * shape.dot(reference.mass * shape));
    const auto& report = step->m_StructuralDamping.report;
    const bool dampingProjectionPassed =
        std::abs(projectedDamping - reference.zeta) <= Number(plan, "initial_matrix_relative_limit") &&
        (!damped || report.matrixValidationPassed);
    const bool complete =
        solved && exception.isEmpty() && step->history.size() > 1 &&
        std::abs(previous - reference.duration) <= 1.0e-10 &&
        (adaptive ||
         actualSteps.size() ==
             (rk4 ? 3 * methodSettings.value("fixed_steps_per_period").toArray()[level].toInt() : 240 * (1 << level)));
    const double matrixLimit = Number(plan, "initial_matrix_relative_limit");
    const bool matrices = massError <= matrixLimit && stiffnessError <= matrixLimit && dampingError <= matrixLimit &&
                          dampingProjectionPassed;
    bool accuracy = true;
    for (const auto& pair : errors)
        accuracy = accuracy && pair.maxCoeff() <= Number(plan, "finest_max_normalized_state_error");
    const auto adaptivePhysics = adaptivePlan.value("bending").toObject();
    const bool physicalFormulas =
        !(adaptive || rk4) ||
        (step->maximumForceDifference <= Number(adaptivePhysics, "force_formula_scaled_limit") &&
         step->maximumMassDifference <= Number(adaptivePhysics, "physical_mass_relative_limit") &&
         step->maximumPotentialDifference <= Number(adaptivePhysics, "potential_formula_scaled_limit"));
    const bool adaptiveSequence = adaptiveTimeBounds && !step->history.empty() && step->history.front().time == 0.0 &&
                                  step->checkpointPushes >= actualSteps.size();
    const bool eligible = complete && (adaptive ? adaptiveSequence : fixed && (rk4 || step->rollbacks == 0)) &&
                          step->normalResiduals && matrices && envelope && physicalFormulas &&
                          initialError <= Number(plan, "initial_state_normalized_limit");
    configuration["solved"] = solved;
    configuration["complete"] = complete;
    configuration["exception"] = exception;
    configuration["failure_reason"] = step->LastFailureReason();
    configuration["actual_end_time"] = previous;
    configuration["accepted_steps"] = actualSteps.size();
    configuration["accepted_time_steps"] = actualSteps;
    configuration["fixed_step_sequence"] = fixed;
    configuration["rollback_count"] = step->rollbacks;
    configuration["maximum_iterations"] = step->maximumIterations;
    configuration["total_iterations"] = static_cast<double>(step->totalIterations);
    configuration["all_accepted_residuals_meet_production_normal_gate"] = step->normalResiduals;
    configuration["maximum_accepted_residual_norm"] = step->maximumResidual;
    configuration["initial_mass_relative_error"] = massError;
    configuration["initial_stiffness_relative_error"] = stiffnessError;
    configuration["initial_damping_scaled_error"] = dampingError;
    configuration["actual_initial_M"] = MatrixJson(step->initialMass);
    configuration["actual_initial_K"] = MatrixJson(step->initialStiffness);
    configuration["actual_initial_C"] = MatrixJson(step->initialDamping);
    configuration["initial_state_max_normalized_error"] = initialError;
    configuration["rotation_vector_matrix_angle_max_difference"] = step->maximumRotationMismatch;
    configuration["small_amplitude_envelope_passed"] = envelope;
    configuration["max_normalized_errors_q_v_a_by_y_rz"] = ErrorJson(errors);
    configuration["reference_comparison_eligible"] = eligible;
    configuration["finest_accuracy_passed"] = level == 2 ? QJsonValue(accuracy) : QJsonValue();
    configuration["single_run_passed"] = eligible && (level != 2 || accuracy);
    configuration["status"] = eligible && (level != 2 || accuracy) ? "PASS_RUN_REQUIRES_SUITE_ORDER" : "FAIL";
    configuration["wall_seconds"] = timer.elapsed() / 1000.0;
    configuration["response_csv"] = prefix + QStringLiteral(".csv");
    configuration["actual_rayleigh_mass_coefficient"] = report.translationMassCoefficient;
    configuration["actual_rayleigh_stiffness_coefficient"] = report.translationStiffnessCoefficient;
    configuration["actual_low_mode_projected_damping_ratio"] = projectedDamping;
    configuration["damping_projection_passed"] = dampingProjectionPassed;
    configuration["actual_modal_damping_matrix_validated"] =
        damped ? QJsonValue(report.matrixValidationPassed) : QJsonValue();
    if (diagnosticTolerance)
        configuration["status"] = eligible && (level != 2 || accuracy) ? "DIAGNOSTIC_ONLY_NUMERIC_GATES_PASSED"
                                                                       : "DIAGNOSTIC_ONLY_NUMERIC_GATES_FAILED";
    if (adaptive)
    {
        configuration["fixed_step_sequence"] = QJsonValue();
        configuration["checkpoint_attempts"] = step->checkpointPushes;
        configuration["rejected_attempt_estimate"] = step->checkpointPushes - actualSteps.size();
        configuration["rejected_count_source"] = "Successful production PushStateCheckpoint calls minus accepted "
                                                 "frames; estimate exact only for completed adaptive runs";
        configuration["accepted_steps_within_declared_bounds"] = adaptiveTimeBounds;
        double minimumAccepted = std::numeric_limits<double>::infinity(), maximumAccepted = 0.0;
        for (const auto& value : actualSteps)
        {
            minimumAccepted = std::min(minimumAccepted, value.toDouble());
            maximumAccepted = std::max(maximumAccepted, value.toDouble());
        }
        configuration["minimum_accepted_time_step"] =
            std::isfinite(minimumAccepted) ? QJsonValue(minimumAccepted) : QJsonValue();
        configuration["maximum_accepted_time_step"] = maximumAccepted;
        configuration["initial_time_exported"] = !step->history.empty() && step->history.front().time == 0.0;
        configuration["all_frame_mass_relative_difference"] = step->maximumMassDifference;
        configuration["all_frame_force_formula_scaled_difference"] = step->maximumForceDifference;
        configuration["all_frame_potential_formula_scaled_difference"] = step->maximumPotentialDifference;
        configuration["all_frame_formula_gates_passed"] = physicalFormulas;
        configuration["finest_accuracy_passed"] = level == 2 ? QJsonValue(accuracy) : QJsonValue();
        configuration["numeric_gates_passed"] = eligible && (level != 2 || accuracy);
        configuration["single_run_passed"] = QJsonValue();
        configuration["status"] =
            eligible && (level != 2 || accuracy) ? "NUMERIC_PASS_REQUIRES_PROCESS_PRECISION_REVIEW" : "FAIL";
    }
    if (rk4)
    {
        configuration["normal_residual_is_Newton_gate"] = false;
        configuration["stage_state_restore_calls"] = step->rollbacks;
        configuration["rollback_count"] = QJsonValue();
        configuration["fixed_RK_has_no_LTE_rejection_path"] = !adaptive;
        configuration["time_interval_rejection_statistics_visible"] = false;
        configuration["all_accepted_physical_mass_solve_balance_passed"] = step->normalResiduals;
        configuration["maximum_physical_force_balance_norm"] = step->maximumResidual;
        configuration["all_frame_mass_relative_difference"] = step->maximumMassDifference;
        configuration["all_frame_force_formula_scaled_difference"] = step->maximumForceDifference;
        configuration["all_frame_potential_formula_scaled_difference"] = step->maximumPotentialDifference;
        configuration["all_frame_formula_gates_passed"] = physicalFormulas;
        configuration["maximum_iterations"] = QJsonValue();
        configuration["total_iterations"] = QJsonValue();
        configuration["rejected_attempt_estimate"] =
            adaptive ? QJsonValue(step->checkpointPushes - actualSteps.size()) : QJsonValue();
        configuration["numeric_gates_passed"] = eligible && (level != 2 || accuracy);
        configuration["single_run_passed"] = eligible && (level != 2 || accuracy);
        configuration["status"] = eligible && (level != 2 || accuracy)
                                      ? (adaptive ? "PASS_RUN_REQUIRES_TOLERANCE_TREND" : "PASS_RUN_REQUIRES_RK4_ORDER")
                                      : "FAIL";
    }
    WriteJson(prefix + QStringLiteral(".result.json"), configuration);
    return configuration;
}

QJsonObject NewmarkBeamBending::RunSuite(const QString& directory, bool rk4)
{
    const QJsonObject plan = ReadJson(QDir(InputDirectory()).filePath(QStringLiteral("validation-plan.json")));
    QJsonArray results;
    QJsonArray orders;
    bool passed = true;
    for (bool damped : {false, true})
    {
        std::array<QJsonObject, 3> rows;
        for (int level = 0; level < 3; ++level)
        {
            rows[level] = RunCase(directory, damped, level, std::nullopt, false, rk4);
            results.append(rows[level]);
            passed = passed && rows[level].value("single_run_passed").toBool();
        }
        for (int level = 1; level < 3; ++level)
        {
            const auto coarse = rows[level - 1].value("max_normalized_errors_q_v_a_by_y_rz").toArray();
            const auto fine = rows[level].value("max_normalized_errors_q_v_a_by_y_rz").toArray();
            QJsonArray componentOrders;
            for (int component = 0; component < 3; ++component)
            {
                QJsonArray pair;
                for (int dof = 0; dof < 2; ++dof)
                {
                    const double e0 = coarse[component].toArray()[dof].toDouble();
                    const double e1 = fine[component].toArray()[dof].toDouble();
                    const double order = e0 > 0.0 && e1 > 0.0 ? std::log2(e0 / e1) : 0.0;
                    pair.append(order);
                    passed = passed && std::isfinite(order) && order >= (rk4 ? 3.5 : Number(plan, "order_lower")) &&
                             order <= (rk4 ? 4.5 : Number(plan, "order_upper"));
                }
                componentOrders.append(pair);
            }
            orders.append(
                QJsonObject{{"damped", damped}, {"fine_level", level}, {"orders_q_v_a_by_y_rz", componentOrders}});
        }
    }
    QJsonObject summary{{"schema", "newmark-production-beam-bending-suite-v1"},
                        {"passed", passed},
                        {"status", passed ? "PASS_SMALL_AMPLITUDE_SEMIDISCRETE_SCOPE" : "FAIL"},
                        {"cases", results},
                        {"orders", orders},
                        {"large_rotation_certified", false},
                        {"precision_stdout_review_required", true}};
    summary["gates"] = QJsonObject{{"six_actual_production_runs_required", true},
                                   {"complete_fixed_steps_normal_equilibrium_and_no_rollback_per_run", true},
                                   {"independent_initial_M_K_C_and_modal_projection_per_run", true},
                                   {"finest_all_six_state_channels_accuracy", true},
                                   {"two_halvings_all_six_state_channels_order", true},
                                   {"numeric_gates_passed", passed},
                                   {"process_stdout_precision_review_pending", true}};
    if (rk4)
    {
        summary["schema"] = "rk4-production-beam-bending-suite-v1";
        summary["status"] = passed ? "PASS_HIGH_MODE_RK4_SCOPE" : "FAIL";
        summary["precision_stdout_review_required"] = false;
        summary["gates"] =
            QJsonObject{{"high_mode_fixed_order_bounds", QJsonArray{3.5, 4.5}}, {"numeric_gates_passed", passed}};
    }
    WriteJson(QDir(directory).filePath(QStringLiteral("summary.json")), summary);
    return summary;
}

static QJsonObject RunAdaptiveBeamBendingSuite(const QString& directory, bool rk4 = false)
{
    QJsonArray cases, trends;
    bool numeric = true;
    for (bool damped : {false, true})
    {
        std::array<QJsonObject, 3> runs;
        for (int level = 0; level < 3; ++level)
        {
            runs[level] = NewmarkBeamBending::RunCase(directory, damped, level, std::nullopt, true, rk4);
            cases.append(runs[level]);
            numeric = numeric && runs[level].value("numeric_gates_passed").toBool();
        }
        for (int level = 1; level < 3; ++level)
        {
            const auto coarse = runs[level - 1].value("max_normalized_errors_q_v_a_by_y_rz").toArray();
            const auto fine = runs[level].value("max_normalized_errors_q_v_a_by_y_rz").toArray();
            QJsonArray ratios;
            for (int group = 0; group < 3; ++group)
            {
                const auto first = coarse[group].toArray(), second = fine[group].toArray();
                const double e0 = std::max(first[0].toDouble(), first[1].toDouble());
                const double e1 = std::max(second[0].toDouble(), second[1].toDouble());
                const double ratio = e0 > 0.0 ? e1 / e0 : e1 == 0.0 ? 0.0 : std::numeric_limits<double>::infinity();
                ratios.append(std::isfinite(ratio) ? QJsonValue(ratio) : QJsonValue());
                numeric = numeric && std::isfinite(ratio) && ratio <= 0.9;
            }
            trends.append(
                QJsonObject{{"damped", damped}, {"fine_level", level}, {"error_ratios_q_v_a_envelopes", ratios}});
        }
    }
    QJsonObject summary{{"schema", "adaptive-newmark-production-beam-bending-suite-v1"},
                        {"cases", cases},
                        {"tolerance_trends", trends},
                        {"numeric_gates_passed", numeric},
                        {"passed", QJsonValue()},
                        {"process_precision_review_pending", true},
                        {"status", numeric ? "NUMERIC_PASS_REQUIRES_PROCESS_PRECISION_REVIEW" : "FAIL"},
                        {"fixed_step_order_assessed", false},
                        {"precision_review_gate",
                         "Complete captured process stdout/stderr must contain zero newmark_precision_limited events"}};
    if (rk4)
    {
        summary["schema"] = "adaptive-rk4-production-beam-bending-suite-v1";
        summary["passed"] = numeric;
        summary["process_precision_review_pending"] = false;
        summary["status"] = numeric ? "PASS_HIGH_MODE_ADAPTIVE_RK4_SCOPE" : "FAIL";
        summary["precision_review_gate"] = "Not applicable to RK";
    }
    WriteJson(QDir(directory).filePath("summary.json"), summary);
    return summary;
}

std::optional<int> VerifyNewmarkBeamBending(const QStringList& arguments)
{
    const int rkAdaptiveOption = arguments.indexOf("--verify-adaptive-rk4-beam-bending");
    const int rkOption = arguments.indexOf("--verify-rk4-beam-bending");
    const bool rk4 = rkAdaptiveOption >= 0 || rkOption >= 0;
    const int adaptiveOption = arguments.indexOf(QStringLiteral("--verify-adaptive-newmark-beam-bending"));
    const bool adaptive = rk4 ? rkAdaptiveOption >= 0 : adaptiveOption >= 0;
    const int option =
        rk4 ? (adaptive ? rkAdaptiveOption : rkOption)
            : (adaptive ? adaptiveOption : arguments.indexOf(QStringLiteral("--verify-newmark-beam-bending")));
    if (option < 0)
        return std::nullopt;
    try
    {
        if (option + 1 >= arguments.size() || arguments[option + 1].startsWith(QStringLiteral("--")))
            throw std::runtime_error("Beam verification requires a fresh output directory");
        const QString directory = QDir(arguments[option + 1]).absolutePath();
        const int levelOption = arguments.indexOf(QStringLiteral("--beam-level"));
        if (levelOption >= 0)
        {
            bool valid = false;
            const int level = levelOption + 1 < arguments.size() ? arguments[levelOption + 1].toInt(&valid) : -1;
            if (!valid)
                throw std::runtime_error("--beam-level must be 0, 1 or 2");
            const bool damped = arguments.contains(QStringLiteral("--beam-damped"));
            const auto result = NewmarkBeamBending::RunCase(directory, damped, level, std::nullopt, adaptive, rk4);
            return result.value(adaptive ? "numeric_gates_passed" : "single_run_passed").toBool() ? 0 : 2;
        }
        if (adaptive)
            return RunAdaptiveBeamBendingSuite(directory, rk4).value("numeric_gates_passed").toBool() ? 0 : 2;
        return NewmarkBeamBending::RunSuite(directory, rk4).value("passed").toBool() ? 0 : 2;
    }
    catch (const std::exception& error)
    {
        QTextStream(stderr) << error.what() << Qt::endl;
        return 2;
    }
}

namespace
{
/** @brief 真实 CR 梁的两自由度微幅弯曲及正式模态阻尼动力验收。
 * @details 公开直梁几何与Euler弯曲理论派生输入，生产AnalysisStep::Solve(false)、Node有限转动与CR惯性；
 *          独立两自由度线性N质量和Euler刚度闭式解，包含Y/RZ的全程q/v/a及三步长二阶门。
 *          初态矩阵/阻尼投影/正常平衡/完整固定步另验；不认证大转动、空间网格精度或Aster Hermite质量。
 *          门槛与微幅模型误差预算在生产候选运行前冻结于输入目录，precision事件另保留stdout供根复核。
 * @author   yangqiuyou
 * @date     2026.10.19
 */
TEST(PlanarBeamSmallAmplitude, NewmarkFixed)
{
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    directory.setAutoRemove(false);
    RecordProperty("temporary_output_directory", directory.path().toStdString());
    QJsonObject result;
    ASSERT_NO_THROW(result = NewmarkBeamBending::RunSuite(directory.path()));
    EXPECT_TRUE(result.value("passed").toBool())
        << "Full-history/fixed-step/Newton/order gates failed; diagnostics: " << directory.path().toStdString()
        << "/summary.json";
    directory.setAutoRemove(!HasFailure());
}
}
