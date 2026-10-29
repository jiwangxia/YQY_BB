#include "Dynamics/SingleConductor/SingleConductorModel.h"

#include "Conductor/ConductorModelBuilder.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelSerializer.h"

#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <set>

namespace SingleConductorCase
{
std::shared_ptr<StructureData> BuildModel(const ModelOptions& options, _OUT QJsonObject& metadata, _OUT QString& error)
{
    error.clear();
    metadata = {};
    if (!std::isfinite(options.duration) || options.duration <= 0.0 || !std::isfinite(options.attackDegrees) ||
        !std::isfinite(options.initialStress) || options.initialStress <= 0.0 ||
        !std::isfinite(options.windSpeed) || options.windSpeed != 10.0 ||
        !std::isfinite(options.airDensity) || options.airDensity <= 0.0 || options.segments < 2)
    {
        error = QStringLiteral("单导线参数无效；当前气动试验数据限定10 m/s、12 mm覆冰。");
        return nullptr;
    }

    constexpr double span = 200.0;
    constexpr double area = 0.00045155;
    constexpr double young = 70.0e9;
    constexpr double poisson = 0.3;
    constexpr double lineMass = 1.732;
    constexpr double torsionalRigidity = 101.0;
    constexpr double axialRotaryInertia = 0.000268;
    constexpr double localFirstMomentY = -0.000459;
    constexpr double diameter = 0.0276;
    constexpr double transitionLength = 1.0;
    const double bendingInertia = area * area / (4.0 * std::numbers::pi);
    const double density = lineMass / area;
    const double shearModulus = young / (2.0 * (1.0 + poisson));

    auto model = std::make_shared<StructureData>();
    auto material = std::make_shared<Material>();
    material->m_Id = 1;
    material->m_Young = young;
    material->m_Poisson = poisson;
    material->m_Density = density;
    model->m_Material.emplace(material->m_Id, material);
    auto section = std::make_shared<SectionCircular>();
    section->m_Id = 1;
    section->m_Area = area;
    section->m_Radius = diameter / 2.0;
    section->m_HasExplicitSectionInertia = true;
    section->m_ExplicitIy = bendingInertia;
    section->m_ExplicitIz = bendingInertia;
    section->m_ExplicitJ = torsionalRigidity / shearModulus;
    section->m_MassInertiaPerLength =
        Vector3d(axialRotaryInertia, density * bendingInertia, density * bendingInertia);
    model->m_Section.emplace(section->m_Id, section);
    auto property = std::make_shared<Property>();
    property->m_Id = 1;
    property->m_pMaterial = material;
    property->m_pSection = section;
    model->m_Property.emplace(property->m_Id, property);

    Conductor::LineBuildConfig line;
    line.start = Vector3d(0.0, 0.0, 30.0);
    line.end = Vector3d(span, 0.0, 30.0);
    line.elementType = options.beam ? EnumKeyword::ElementType::CR3D : EnumKeyword::ElementType::CABLE;
    line.property = property;
    line.endTopology = Conductor::BundleEndTopology::DirectWireSupports;
    line.bundleEndTransitionLength = transitionLength;
    line.conductor.nBundle = 1;
    line.conductor.segments = options.segments;
    line.conductor.initialStress = options.initialStress;
    line.conductor.spacing = 0.0;
    line.setNamePrefix = QStringLiteral("Duan单导线对照");
    // 当前CR梁没有质量一次矩惯性及偏心重力项，梁对照明确保留其现有同轴实现。
    if (!options.beam)
        line.localMassFirstMomentPerLength = Vector3d(0.0, localFirstMomentY, 0.0);
    Conductor::ConductorModelBuilder builder(model);
    Conductor::LineBuildResult built;
    std::string buildError;
    if (!builder.BuildLine(line, built, buildError) || built.subConductors.size() != 1)
    {
        error = QStringLiteral("单导线生成失败：%1").arg(QString::fromStdString(buildError));
        return nullptr;
    }
    const auto& wire = built.subConductors.begin()->second;
    if (wire.nodeIds.size() < 3 || wire.elementIds.empty())
    {
        error = QStringLiteral("单导线生成结果没有完整节点与单元。");
        return nullptr;
    }
    const std::vector<int> endpoints{wire.nodeIds.front(), wire.nodeIds.back()};
    // 索端固定三向平动和标量扭转；梁端固定三向平动和全局X转角，Y/Z弯曲转角释放。
    if (model->Add_Constraint(endpoints, {0, 1, 2, 3}, {0.0, 0.0, 0.0, 0.0}) != 8)
    {
        error = QStringLiteral("单导线端部约束创建失败。");
        return nullptr;
    }
    std::set<int> nodeIds(wire.nodeIds.begin(), wire.nodeIds.end());
    std::set<int> elementIds(wire.elementIds.begin(), wire.elementIds.end());
    if (model->AddComputeRegion(QStringLiteral("单导线"), nodeIds, elementIds, {}, true, &error) <= 0)
        return nullptr;

    AnalysisStepConfig gravity;
    gravity.id = 1;
    gravity.name = QStringLiteral("仅重力静力平衡");
    gravity.type = EnumKeyword::StepType::STATIC;
    gravity.totalTime = 1.0;
    gravity.stepSize = 0.01;
    gravity.tolerance = 1.0e-6;
    gravity.maxIterations = 80;
    model->AddAnalysisStep(gravity);
    model->Add_Gravity(static_cast<int>(EnumKeyword::Direction::Z), gravity.id);
    const auto gravityLoad = std::dynamic_pointer_cast<Force_Gravity>(model->m_Load.rbegin()->second);
    gravityLoad->m_g = -9.81;
    gravityLoad->m_Name = QStringLiteral("全结构重力");

    AnalysisStepConfig dynamic;
    dynamic.id = 2;
    dynamic.name = QStringLiteral("单导线舞动300秒对照");
    dynamic.type = EnumKeyword::StepType::DYNAMIC;
    dynamic.totalTime = options.duration;
    dynamic.stepSize = 0.005;
    dynamic.tolerance = 1.0e-6;
    dynamic.maxIterations = 48;
    dynamic.initialStaticStepId = gravity.id;
    dynamic.enableGalloping = true;
    dynamic.gallopingIceThickness = 12;
    dynamic.gallopingInitialAttackDegrees = options.attackDegrees;
    dynamic.gallopingAerodynamicTangentMode = SolverNameSpace::AerodynamicTangentMode::EveryNewtonIteration;
    dynamic.structuralDamping.enabled = true;
    dynamic.structuralDamping.formulation = SolverNameSpace::StructuralDampingFormulation::LocalRayleigh;
    dynamic.structuralDamping.translationDampingRatio = 0.0005;
    dynamic.structuralDamping.torsionDampingRatio = 0.0005;
    dynamic.structuralDamping.useExplicitRayleighCoefficients = true;
    dynamic.structuralDamping.translationMassCoefficient = 0.0013097343646597613;
    dynamic.structuralDamping.translationStiffnessCoefficient = 0.0001696696889219597;
    dynamic.structuralDamping.torsionMassCoefficient = 0.006428681471391675;
    dynamic.structuralDamping.torsionStiffnessCoefficient = 0.000034567309519243565;
    dynamic.adaptiveTimeStep.minimumTimeStep = 1.0e-5;
    dynamic.adaptiveTimeStep.maximumTimeStep = 0.02;
    dynamic.adaptiveTimeStep.relativeTolerance = 5.0e-4;
    dynamic.adaptiveTimeStep.absoluteTolerance = 1.0e-5;
    dynamic.adaptiveTimeStep.includeVelocityInError = true;
    dynamic.adaptiveTimeStep.velocityRelativeTolerance = 5.0e-4;
    dynamic.adaptiveTimeStep.velocityAbsoluteTolerance = 1.0e-4;
    model->AddAnalysisStep(dynamic);
    auto wind = std::make_shared<Force_Wind>();
    wind->m_Id = model->m_Load.rbegin()->first + 1;
    wind->m_Name = QStringLiteral("动力步恒风10m每秒");
    wind->m_StepId = dynamic.id;
    wind->m_velocity = options.windSpeed;
    wind->m_direction = -Vector3d::UnitY();
    wind->m_windDensity = options.airDensity;
    model->m_Load.emplace(wind->m_Id, wind);
    model->m_AeroManager.setShapePreservingInterpolation(false);
    if (!model->m_AnalysisStep.at(dynamic.id)->PreloadGallopingDataForSnapshot(model, &error))
        return nullptr;

    QJsonArray sampleNodes;
    for (const double fraction : {0.25, 0.5, 0.75})
    {
        int selectedId = -1;
        double closest = std::numeric_limits<double>::infinity();
        for (const int nodeId : wire.nodeIds)
        {
            const auto node = model->FindNode(nodeId);
            const double distance = std::abs(node->m_X - span * fraction);
            if (distance < closest)
            {
                closest = distance;
                selectedId = nodeId;
            }
        }
        sampleNodes.append(QJsonObject{{"span_fraction", fraction}, {"node_id", selectedId}});
    }
    metadata = QJsonObject{
        {"element", options.beam ? "CR3D" : "CABLE"}, {"span_m", span},
        {"nodes", static_cast<int>(model->m_Nodes.size())},
        {"elements", static_cast<int>(model->m_Elements.size())}, {"requested_segments", options.segments},
        {"end_transition_length_m", transitionLength}, {"sample_nodes", sampleNodes},
        {"left_node", endpoints.front()}, {"right_node", endpoints.back()}, {"area_m2", area},
        {"young_pa", young}, {"poisson", poisson}, {"line_mass_kg_m", lineMass},
        {"density_kg_m3", density}, {"torsional_rigidity_nm2", torsionalRigidity},
        {"bending_inertia_m4", bendingInertia}, {"bending_rigidity_nm2", young * bendingInertia},
        {"axial_rotary_inertia_kg_m", axialRotaryInertia},
        {"bending_rotary_inertia_kg_m", density * bendingInertia},
        {"requested_local_first_moment_y_kg", localFirstMomentY},
        {"active_local_first_moment_y_kg", options.beam ? 0.0 : localFirstMomentY},
        {"eccentric_mass_and_gravity_supported", !options.beam}, {"diameter_m", diameter},
        {"initial_horizontal_stress_pa", options.initialStress},
        {"initial_horizontal_tension_n", options.initialStress * area}, {"gravity_m_s2", 9.81},
        {"duration_s", options.duration}, {"wind_speed_m_s", options.windSpeed},
        {"air_density_kg_m3", options.airDensity}, {"initial_attack_degrees", options.attackDegrees},
        {"ice_mm", 12}, {"aero_interpolation", "linear CSV"},
        {"aero_force_model", "production relative wind and current force directions"},
        {"aero_source", QString::fromStdWString(model->m_AeroManager.getCaseSourceFile({1, 10, 12}).wstring())},
        {"initial_perturbation_m", 0.0}, {"static_wind", false}, {"static_step_id", 1},
        {"dynamic_step_id", 2}, {"rayleigh_y1", dynamic.structuralDamping.translationMassCoefficient},
        {"rayleigh_y2", dynamic.structuralDamping.translationStiffnessCoefficient},
        {"rayleigh_c1", dynamic.structuralDamping.torsionMassCoefficient},
        {"rayleigh_c2", dynamic.structuralDamping.torsionStiffnessCoefficient},
        {"damping_scope", options.beam ? "CR3D all channels use y1*M+y2*K; c1/c2 unused"
                                      : "cable translation y1/y2 and axial torsion c1/c2"},
        {"boundary_scope", options.beam ? "translations and global rotation X fixed; rotation Y/Z free"
                                        : "translations and scalar axial twist fixed"},
        {"qualification", "Duan Table 7 based extension; not a strict reproduction of the paper"},
        {"assumptions", QJsonArray{"Table 7 1.732 interpreted as total line mass, not additional ice mass",
                                    "area, Poisson ratio, initial tension, air density, mesh and end details assumed",
                                    "bending inertia uses equivalent solid area; physical diameter is aerodynamic",
                                    "first mass moment sign follows historical common model",
                                    "Rayleigh coefficients supplied from taut-string estimates, not paper matrix"}}};
    return model;
}

bool ExportModel(const ModelOptions& options, const QString& filePath, _OUT QJsonObject& metadata, _OUT QString& error)
{
    if (QFileInfo::exists(filePath))
    {
        metadata = {};
        error = QStringLiteral("目标输入已存在，拒绝覆盖冻结模型：%1").arg(filePath);
        return false;
    }
    const auto model = BuildModel(options, metadata, error);
    if (!model)
        return false;
    if (!QDir().mkpath(QFileInfo(filePath).absolutePath()))
    {
        error = QStringLiteral("无法建立单导线输入目录。");
        return false;
    }
    Hdf5ModelSerializer serializer;
    if (!serializer.exportModel(filePath, model.get(), QStringLiteral("Duan Table 7 single conductor extension")))
    {
        error = QStringLiteral("单导线HDF5输入导出失败。");
        return false;
    }
    metadata.insert(QStringLiteral("input_file"), QFileInfo(filePath).absoluteFilePath());
    return true;
}
}
