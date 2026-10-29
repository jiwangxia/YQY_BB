#include "Dynamics/SingleConductor/SingleConductorTest.h"
#include "Dynamics/SingleConductor/SingleConductorModel.h"

#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include "Export/Hdf5ResultReader.h"
#include "Solver/Workflow/AnalysisSolve.h"
#include "Utility/CR.h"

#include <QCryptographicHash>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLocale>
#include <QTextStream>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <set>

static bool WriteSingleConductorJson(const QString& path, const QJsonObject& document)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly))
        return false;
    return file.write(QJsonDocument(document).toJson(QJsonDocument::Indented)) >= 0;
}

static double SingleConductorComponent(const std::vector<double>& values, int component)
{
    return component < static_cast<int>(values.size()) ? values[component] : 0.0;
}

static QJsonArray SingleConductorVector(const Vector3d& vector)
{
    return {vector.x(), vector.y(), vector.z()};
}

static QString SingleConductorHistoryHeader()
{
    return QStringLiteral("time,dt,node_id,ux,uy,uz,rx,ry,rz,vx,vy,vz,wx,wy,wz,ax,ay,az,alphax,alphay,alphaz,twist\n");
}

static double SingleConductorTwist(const Node& node, bool cable)
{
    return cable ? SingleConductorComponent(node.m_Displacement, 3)
                 : Utility::CR::ExtractAxialTwist(node.m_Rg, Vector3d::UnitX());
}

static bool WriteSingleConductorInitialFrame(const QString& path, const Hdf5ResultFrame& frame,
                                               const std::map<int, double>& twists)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    QTextStream output(&file);
    output.setLocale(QLocale::c());
    output.setRealNumberPrecision(17);
    output << SingleConductorHistoryHeader();
    for (const auto& node : frame.nodes)
    {
        output << frame.info.time << ",0," << node.id;
        for (double value : node.displacement)
            output << ',' << value;
        for (double value : node.velocity)
            output << ',' << value;
        for (double value : node.acceleration)
            output << ',' << value;
        output << ',' << twists.at(node.id) << '\n';
    }
    output.flush();
    return file.error() == QFile::NoError;
}

static bool ValidateSingleConductorDamping(_OUT AnalysisStep& step, _OUT QJsonObject& evidence)
{
    const auto& settings = step.m_StructuralDamping.settings;
    const auto& report = step.m_StructuralDamping.report;
    if (!settings.enabled)
        return true;
    if (!settings.useExplicitRayleighCoefficients)
        return step.m_StructuralDamping.IsComputed() && report.matrixValidationPassed;

    // 显式四系数不经过模态标定，IsComputed 不是该路径的就绪标志；直接检查生产装配的实际矩阵。
    const std::array<double, 4> configured{settings.translationMassCoefficient,
                                          settings.translationStiffnessCoefficient,
                                          settings.torsionMassCoefficient, settings.torsionStiffnessCoefficient};
    const std::array<double, 4> actual{report.translationMassCoefficient, report.translationStiffnessCoefficient,
                                      report.torsionMassCoefficient, report.torsionStiffnessCoefficient};
    const bool coefficientsMatch = configured == actual;
    SolverNameSpace::SystemEvaluationRequest request;
    request.contributions = SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Damping);
    request.time = step.m_Time;
    SolverNameSpace::SystemEvaluationResult result;
    step.EvaluateSystem(request, result);
    const Eigen::MatrixXd damping(result.damping);
    if (damping.rows() != step.GetFreeDofs() || damping.cols() != step.GetFreeDofs() ||
        damping.rows() == 0 || !damping.allFinite())
        return false;
    const double norm = damping.norm();
    const double relativeAsymmetry = (damping - damping.transpose()).norm() / std::max(1.0, norm);
    const Eigen::MatrixXd symmetric = 0.5 * (damping + damping.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(symmetric, Eigen::EigenvaluesOnly);
    if (spectrum.info() != Eigen::Success || !spectrum.eigenvalues().allFinite())
        return false;
    const double minimumEigenvalue = spectrum.eigenvalues().minCoeff();
    const double eigenvalueTolerance = 1.0e-10 * std::max(1.0, spectrum.eigenvalues().cwiseAbs().maxCoeff());
    const bool anyCoefficient = std::any_of(configured.begin(), configured.end(),
                                           [](double value) { return value > 0.0; });
    const double productionPower = step.SampleStructuralDampingPower();
    SolverNameSpace::Vec displacement, velocity, acceleration;
    step.GetState(displacement, velocity, acceleration);
    const double matrixPower = velocity.dot(damping * velocity);
    const double powerTolerance = 1.0e-9 * std::max({1.0, std::abs(matrixPower), std::abs(productionPower)});
    const bool valid = coefficientsMatch && (!anyCoefficient || norm > 0.0) && relativeAsymmetry <= 1.0e-9 &&
                       minimumEigenvalue >= -eigenvalueTolerance && std::isfinite(productionPower) &&
                       matrixPower >= -powerTolerance && std::abs(matrixPower - productionPower) <= powerTolerance;
    evidence = QJsonObject{{"route", "explicit coefficients; actual final-state production damping matrix"},
                           {"coefficients_match", coefficientsMatch}, {"matrix_norm", norm},
                           {"relative_asymmetry", relativeAsymmetry}, {"minimum_eigenvalue", minimumEigenvalue},
                           {"eigenvalue_tolerance", eigenvalueTolerance}, {"matrix_power", matrixPower},
                           {"production_power", productionPower}, {"passed", valid}};
    return valid;
}

std::optional<int> VerifySingleConductorNewmark(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-single-conductor-newmark")))
        return std::nullopt;
    const auto textOption = [&arguments](const QString& key)
    {
        const int index = arguments.indexOf(key);
        return arguments.count(key) == 1 && index + 1 < arguments.size() ? arguments[index + 1] : QString();
    };
    const auto numberOption = [&arguments, &textOption](const QString& key, double fallback)
    {
        if (!arguments.contains(key))
            return fallback;
        bool valid = false;
        const double value = textOption(key).toDouble(&valid);
        return valid && std::isfinite(value) && value > 0.0 ? value : -1.0;
    };
    const QString input = QFileInfo(textOption(QStringLiteral("--input"))).absoluteFilePath();
    const QString outputPath = textOption(QStringLiteral("--output"));
    const QString solverName = textOption(QStringLiteral("--solver"));
    if (arguments.contains(QStringLiteral("--build-only")))
    {
        const QString element = textOption(QStringLiteral("--element"));
        const QDir directory(QFileInfo(outputPath).absoluteFilePath());
        if (outputPath.isEmpty() || (element != QStringLiteral("cable") && element != QStringLiteral("beam")) ||
            ((!directory.exists() && !QDir().mkpath(directory.absolutePath())) ||
             !directory.entryList(QDir::AllEntries | QDir::NoDotAndDotDot).isEmpty()))
            return 2;
        SingleConductorCase::ModelOptions options;
        options.beam = element == QStringLiteral("beam");
        options.duration = numberOption(QStringLiteral("--duration"), 300.0);
        QJsonObject metadata;
        QString error;
        const bool built = SingleConductorCase::ExportModel(
            options, directory.filePath(QStringLiteral("model.h5")), metadata, error);
        metadata["build_succeeded"] = built;
        metadata["failure_reason"] = error;
        if (!WriteSingleConductorJson(directory.filePath(QStringLiteral("model.json")), metadata))
            return 3;
        QTextStream(stdout) << "single_conductor_model built=" << built << " reason=" << error << Qt::endl;
        return built ? 0 : 4;
    }
    const double duration = numberOption(QStringLiteral("--duration"), 300.0);
    const double timeStep = numberOption(QStringLiteral("--dt"), 0.005);
    const double maximumStep = numberOption(QStringLiteral("--max-dt"), 0.02);
    const double minimumStep = numberOption(QStringLiteral("--min-dt"), 1.0e-6);
    const double relativeTolerance = numberOption(QStringLiteral("--rtol"), 1.0e-4);
    const double absoluteTolerance = numberOption(QStringLiteral("--atol"), 1.0e-6);
    const double velocityTolerance = numberOption(QStringLiteral("--velocity-atol"), 1.0e-5);
    const bool noAero = arguments.contains(QStringLiteral("--no-aero"));
    if (textOption(QStringLiteral("--input")).isEmpty() || outputPath.isEmpty() ||
        (solverName != QStringLiteral("fixed") && solverName != QStringLiteral("adaptive")) ||
        duration <= 0.0 || timeStep <= 0.0 || maximumStep < timeStep || minimumStep <= 0.0 ||
        minimumStep > timeStep || relativeTolerance <= 0.0 || absoluteTolerance <= 0.0 || velocityTolerance <= 0.0)
        return 2;
    const QDir output(QFileInfo(outputPath).absoluteFilePath());
    if ((!output.exists() && !QDir().mkpath(output.absolutePath())) ||
        !output.entryList(QDir::AllEntries | QDir::NoDotAndDotDot).isEmpty())
        return 2;

    QJsonObject summary{{"completed", false}, {"static_solved", false}, {"dynamic_solved", false},
                        {"solver", solverName}, {"input", input},
                        {"duration", duration}, {"nominal_dt", timeStep}, {"maximum_dt", maximumStep},
                        {"minimum_dt", minimumStep}, {"rtol", relativeTolerance}, {"atol", absoluteTolerance},
                        {"velocity_atol", velocityTolerance}, {"include_velocity_error", true},
                        {"no_aero", noAero}, {"rejected_attempts", QJsonValue::Null},
                        {"rejected_attempts_note", "AnalysisStep does not retain Newmark rejected-attempt counts"},
                        {"output_policy", "full HDF5 at each accepted step, identical for both algorithms"},
                        {"scope", "completion and response evidence; numerical and galloping acceptance is external"}};
    const auto finish = [&summary, &output](int code, const QString& reason)
    {
        summary["exit_code"] = code;
        summary["failure_reason"] = reason;
        const bool saved = WriteSingleConductorJson(output.filePath(QStringLiteral("summary.json")), summary);
        QTextStream(stdout) << "single_conductor completed=" << (code == 0) << " reason=" << reason << Qt::endl;
        return saved ? code : 3;
    };
    try
    {
        QFile inputFile(input);
        if (!inputFile.open(QIODevice::ReadOnly))
            return finish(4, QStringLiteral("Cannot read input"));
        summary["input_sha256"] = QString::fromLatin1(QCryptographicHash::hash(inputFile.readAll(),
                                                                              QCryptographicHash::Sha256).toHex());
        inputFile.close();
        Hdf5ModelIO io;
        auto model = std::make_shared<StructureData>();
        if (!io.ImportHdf5(input, model.get()))
            return finish(4, QStringLiteral("HDF5 model import failed"));
        if (model->m_AnalysisStep.size() != 2 || !model->m_MPCConstraints.empty() || model->m_Nodes.size() < 5)
            return finish(4, QStringLiteral("Expected one static and one dynamic step "
                                            "and a single conductor without MPC"));
        std::shared_ptr<AnalysisStep> staticStep;
        std::shared_ptr<AnalysisStep> dynamicStep;
        for (const auto& [id, step] : model->m_AnalysisStep)
        {
            if (!step)
                return finish(4, QStringLiteral("Null analysis step"));
            if (step->m_Type == EnumKeyword::StepType::STATIC)
                staticStep = step;
            else if (step->m_Type == EnumKeyword::StepType::DYNAMIC)
                dynamicStep = step;
        }
        if (!staticStep || !dynamicStep || dynamicStep->m_InitialStaticStepId != staticStep->m_Id ||
            !dynamicStep->m_EnableGalloping)
            return finish(4, QStringLiteral("Input must define a galloping dynamic step inheriting the static step"));
        const int staticId = staticStep->m_Id;
        const int dynamicId = dynamicStep->m_Id;
        model->EnsureDefaultAnalysisConfiguration();
        for (const auto& step : {staticStep, dynamicStep})
        {
            const auto regions = model->ResolveAnalysisStepRegionIds(*step);
            if (regions.size() != 1)
                return finish(4, QStringLiteral("Expected one complete analysis region"));
            const auto region = model->m_ComputeRegions.at(regions.front());
            if (!region || region->m_NodeIds.size() != model->m_Nodes.size() ||
                region->m_ElementIds.size() != model->m_Elements.size())
                return finish(4, QStringLiteral("Analysis region must include the complete model"));
        }
        std::set<int> wireIds;
        QJsonArray elements;
        std::set<int> elementTypes;
        for (const auto& [id, element] : model->m_Elements)
        {
            if (!element || element->m_Role != ElementRole::Conductor || element->m_AeroBundleCount != 1 ||
                !element->HasAerodynamicLoad())
                return finish(4, QStringLiteral("Every element must belong to the single aerodynamic conductor"));
            const bool cable = std::dynamic_pointer_cast<ElementCable>(element) != nullptr;
            const bool beam = std::dynamic_pointer_cast<ElementBeam_CR>(element) != nullptr;
            if (!cable && !beam)
                return finish(4, QStringLiteral("Only cable or spatial CR beam elements are supported by this case"));
            elementTypes.insert(cable ? 0 : 1);
            wireIds.insert(element->m_WireId);
            const auto property = element->m_pProperty.lock();
            const auto material = property ? property->m_pMaterial.lock() : nullptr;
            const auto section = property ? property->m_pSection.lock() : nullptr;
            if (!material || !section)
                return finish(4, QStringLiteral("Element property is incomplete"));
            double iy = 0.0, iz = 0.0, polar = 0.0;
            section->Calculate_I(iy, iz, polar);
            elements.append(QJsonObject{{"id", id}, {"kind", cable ? "cable" : "spatial_CR_beam"},
                                       {"wire_id", element->m_WireId}, {"aero_profile_id", element->m_AeroProfileId},
                                       {"initial_stress", element->m_InitStress}, {"area", section->m_Area},
                                       {"young", material->m_Young}, {"density", material->m_Density},
                                       {"line_mass", material->m_Density * section->m_Area},
                                       {"iy", iy}, {"iz", iz}, {"j", polar},
                                       {"mass_inertia_per_length",
                                        SingleConductorVector(section->m_MassInertiaPerLength)},
                                       {"mass_first_moment_per_length",
                                        SingleConductorVector(section->m_MassFirstMomentPerLength)}});
        }
        if (wireIds.size() != 1 || elementTypes.size() != 1)
            return finish(4, QStringLiteral("Expected one wire and one homogeneous element type"));
        const bool cableModel = *elementTypes.begin() == 0;
        summary["element_type"] = cableModel ? "cable" : "spatial_CR_beam";
        summary["twist_definition"] = cableModel ? "scalar cable axial twist"
                                                  : "spatial orientation swing-twist angle about global span X";
        summary["node_count"] = static_cast<int>(model->m_Nodes.size());
        summary["element_count"] = static_cast<int>(model->m_Elements.size());
        summary["elements"] = elements;
        summary["static_step_id"] = staticId;
        summary["dynamic_step_id"] = dynamicId;
        summary["initial_attack_degrees"] = dynamicStep->m_GallopingInitialAttackDegrees;
        summary["ice_thickness_mm"] = dynamicStep->m_GallopingIceThickness;
        summary["initial_vertical_perturbation"] = dynamicStep->m_GallopingInitialVerticalPerturbationMeters;
        QJsonArray winds;
        for (const auto& [id, load] : model->m_Load)
        {
            const auto wind = std::dynamic_pointer_cast<Force_Wind>(load);
            if (!wind)
                continue;
            winds.append(QJsonObject{{"id", id}, {"step_id", wind->m_StepId}, {"speed", wind->m_velocity},
                                    {"direction", SingleConductorVector(wind->m_direction)},
                                    {"air_density", wind->m_windDensity}, {"moment", wind->m_EnableAerodynamicMoment},
                                    {"fixed_force_directions", wind->m_DiagnosticReferenceFixedForceDirections},
                                    {"fixed_wind_speed", wind->m_DiagnosticReferenceFixedWindSpeed},
                                    {"exact_fitted_aero", wind->m_DiagnosticReferenceExactFittedAero},
                                    {"bounded_fitted_aero", wind->m_DiagnosticPaperBoundedFittedAero}});
        }
        if (winds.isEmpty())
            return finish(4, QStringLiteral("Input has no wind load"));
        summary["source_winds"] = winds;
        if (noAero)
        {
            // 本对照只移除动力步引入的风，禁止改变静力载荷或隐式删去既定初扰。
            if (dynamicStep->m_GallopingInitialVerticalPerturbationMeters != 0.0 ||
                std::any_of(winds.begin(), winds.end(), [dynamicId](const QJsonValue& value)
                            { return value.toObject()["step_id"].toInt() != dynamicId; }))
                return finish(4, QStringLiteral("No-aero comparison requires dynamic-only wind "
                                                "and zero initial perturbation"));
            dynamicStep->m_EnableGalloping = false;
            for (auto it = model->m_Load.begin(); it != model->m_Load.end();)
            {
                if (std::dynamic_pointer_cast<Force_Wind>(it->second))
                    it = model->m_Load.erase(it);
                else
                    ++it;
            }
        }
        const auto& damping = dynamicStep->m_StructuralDamping.settings;
        summary["damping"] = QJsonObject{{"enabled", damping.enabled},
                                         {"translation_ratio", damping.translationDampingRatio},
                                         {"torsion_ratio", damping.torsionDampingRatio},
                                         {"explicit_coefficients", damping.useExplicitRayleighCoefficients},
                                         {"translation_mass", damping.translationMassCoefficient},
                                         {"translation_stiffness", damping.translationStiffnessCoefficient},
                                         {"torsion_mass", damping.torsionMassCoefficient},
                                         {"torsion_stiffness", damping.torsionStiffnessCoefficient}};
        dynamicStep->m_DynamicSolverType = solverName == QStringLiteral("fixed")
                                             ? SolverNameSpace::SolverType::Newmark
                                             : SolverNameSpace::SolverType::AdaptiveNewmark;
        dynamicStep->m_Time = duration;
        dynamicStep->m_StepSize = timeStep;
        dynamicStep->m_AdaptiveTimeStep.minimumTimeStep = minimumStep;
        dynamicStep->m_AdaptiveTimeStep.maximumTimeStep = maximumStep;
        dynamicStep->m_AdaptiveTimeStep.relativeTolerance = relativeTolerance;
        dynamicStep->m_AdaptiveTimeStep.absoluteTolerance = absoluteTolerance;
        dynamicStep->m_AdaptiveTimeStep.includeVelocityInError = true;
        dynamicStep->m_AdaptiveTimeStep.velocityRelativeTolerance = relativeTolerance;
        dynamicStep->m_AdaptiveTimeStep.velocityAbsoluteTolerance = velocityTolerance;
        summary["newton_tolerance"] = dynamicStep->m_Tolerance;
        summary["maximum_newton_iterations"] = dynamicStep->m_MaxIterations;
        model->m_OutputControl.m_Hdf5FileName = output.filePath(QStringLiteral("static.h5"));
        model->m_OutputControl.m_SourceModelName = QFileInfo(input).fileName();
        model->m_OutputControl.m_StreamResult = true;
        if (!io.ExportModelHdf5(output.filePath(QStringLiteral("configured.h5")), model.get(), input))
            return finish(4, QStringLiteral("Cannot save configured model"));
        WriteSingleConductorJson(output.filePath(QStringLiteral("summary.json")), summary);

        AnalysisRunner runner;
        runner.SetStructure(model);
        runner.SetMaximumRegionThreads(1);
        runner.SetExecutionThreadBudget(1);
        QElapsedTimer timer;
        timer.start();
        const bool staticSolved = runner.RunStep(staticId);
        summary["static_wall_seconds"] = timer.nsecsElapsed() / 1.0e9;
        if (!staticSolved)
            return finish(5, QStringLiteral("Static equilibrium failed: ") + staticStep->LastFailureReason());
        summary["static_solved"] = true;
        std::map<int, double> initialTwists;
        for (const auto& [id, node] : model->m_Nodes)
            initialTwists.emplace(id, SingleConductorTwist(*node, cableModel));
        model->GetOutputter().Clear();
        const QString resultPath = output.filePath(QStringLiteral("dynamic.h5"));
        model->m_OutputControl.m_Hdf5FileName = resultPath;
        QJsonArray probes;
        std::vector<int> probeIds;
        double minimumX = std::numeric_limits<double>::infinity();
        double maximumX = -std::numeric_limits<double>::infinity();
        for (const auto& [id, node] : model->m_Nodes)
        {
            if (!node)
                return finish(4, QStringLiteral("Null node"));
            minimumX = std::min(minimumX, node->m_X);
            maximumX = std::max(maximumX, node->m_X);
        }
        if (maximumX - minimumX <= 1.0e-9)
            return finish(4, QStringLiteral("Single-conductor case expects a span along global X"));
        for (double fraction : {0.25, 0.5, 0.75})
        {
            int selected = -1;
            double gap = std::numeric_limits<double>::infinity();
            for (const auto& [id, node] : model->m_Nodes)
            {
                const double distance = std::abs(node->m_X - minimumX - fraction * (maximumX - minimumX));
                if (distance < gap)
                {
                    gap = distance;
                    selected = id;
                }
            }
            if (selected < 0 || gap > 0.06 * (maximumX - minimumX))
                return finish(4, QStringLiteral("Missing quarter-span or midspan observation node"));
            probeIds.push_back(selected);
            probes.append(QJsonObject{{"node_id", selected}, {"target_fraction", fraction},
                                     {"x", model->m_Nodes.at(selected)->m_X}});
        }
        summary["probes"] = probes;
        QFile history(output.filePath(QStringLiteral("accepted.csv")));
        if (!history.open(QIODevice::WriteOnly | QIODevice::Text))
            return finish(3, QStringLiteral("Cannot create response history"));
        QTextStream historyStream(&history);
        historyStream.setLocale(QLocale::c());
        historyStream.setRealNumberPrecision(17);
        historyStream << SingleConductorHistoryHeader();
        double lastTime = 0.0;
        double minimumAcceptedStep = std::numeric_limits<double>::infinity();
        double maximumAcceptedStep = 0.0;
        double csvMilliseconds = 0.0;
        int accepted = 0;
        int nonNominalSteps = 0;
        bool finiteStates = true;
        double nextProgress = 10.0;
        runner.SetRuntimeCallbacks(
            [&](double progress, const QString& message)
            {
                const double time = progress * duration;
                if (time <= lastTime + 1.0e-12 || !message.contains(QStringLiteral("Newmark 第")))
                    return;
                QElapsedTimer sampleTimer;
                sampleTimer.start();
                const double dt = time - lastTime;
                minimumAcceptedStep = std::min(minimumAcceptedStep, dt);
                maximumAcceptedStep = std::max(maximumAcceptedStep, dt);
                if (time < duration - 1.0e-8 && std::abs(dt - timeStep) > 1.0e-9 * std::max(1.0, timeStep))
                    ++nonNominalSteps;
                ++accepted;
                for (int id : probeIds)
                {
                    const auto node = model->m_Nodes.at(id);
                    historyStream << time << ',' << dt << ',' << id;
                    for (const auto* values : {&node->m_Displacement, &node->m_Velocity, &node->m_Acceleration})
                    {
                        for (int component = 0; component < 6; ++component)
                        {
                            const double value = SingleConductorComponent(*values, component);
                            finiteStates = finiteStates && std::isfinite(value);
                            historyStream << ',' << value;
                        }
                    }
                    const double twist = SingleConductorTwist(*node, cableModel);
                    finiteStates = finiteStates && std::isfinite(twist);
                    historyStream << ',' << twist << '\n';
                }
                lastTime = time;
                if (time + 1.0e-9 >= nextProgress)
                {
                    historyStream.flush();
                    QTextStream(stdout) << "single_conductor time=" << time << " accepted=" << accepted << Qt::endl;
                    nextProgress += 10.0;
                }
                csvMilliseconds += sampleTimer.nsecsElapsed() / 1.0e6;
            },
            [&finiteStates]() { return !finiteStates; });
        timer.restart();
        const bool dynamicSolved = runner.RunStepFromCurrentState(dynamicId);
        summary["dynamic_wall_seconds"] = timer.nsecsElapsed() / 1.0e9;
        summary["dynamic_solved"] = dynamicSolved;
        const auto& dampingReport = dynamicStep->m_StructuralDamping.report;
        summary["actual_damping"] = QJsonObject{
            {"computed", dynamicStep->m_StructuralDamping.IsComputed()},
            {"matrix_validation_passed", dampingReport.matrixValidationPassed},
            {"translation_mass", dampingReport.translationMassCoefficient},
            {"translation_stiffness", dampingReport.translationStiffnessCoefficient},
            {"torsion_mass", dampingReport.torsionMassCoefficient},
            {"torsion_stiffness", dampingReport.torsionStiffnessCoefficient},
            {"translation_frequency_1_hz", dampingReport.translationReferenceFrequency1Hz},
            {"translation_frequency_2_hz", dampingReport.translationReferenceFrequency2Hz},
            {"torsion_frequency_1_hz", dampingReport.torsionReferenceFrequency1Hz},
            {"torsion_frequency_2_hz", dampingReport.torsionReferenceFrequency2Hz},
            {"maximum_backcheck_error", dampingReport.maximumBackCheckError},
            {"summary", dampingReport.summary}};
        historyStream.flush();
        history.close();
        runner.SetRuntimeCallbacks({}, {});
        summary["accepted_steps"] = accepted;
        summary["minimum_accepted_dt"] = accepted ? minimumAcceptedStep : 0.0;
        summary["maximum_accepted_dt"] = maximumAcceptedStep;
        summary["non_nominal_steps_excluding_terminal"] = nonNominalSteps;
        summary["last_accepted_time"] = lastTime;
        summary["finite_probe_states"] = finiteStates;
        summary["csv_sampling_ms"] = csvMilliseconds;
        const auto& performance = dynamicStep->GetPerformanceStatistics();
        summary["performance"] = QJsonObject{{"total_ms", performance.totalMilliseconds},
                                             {"output_ms", performance.outputMilliseconds},
                                             {"system_evaluation_ms", performance.systemEvaluationMilliseconds},
                                             {"system_evaluation_calls", performance.systemEvaluationCount},
                                             {"effective_tangent_calls", performance.effectiveTangentCount},
                                             {"dynamic_system_calls", performance.dynamicSystemCount},
                                             {"external_tangent_calls", performance.externalTangentCount},
                                             {"residual_calls", performance.residualCount}};
        QFile iterations(output.filePath(QStringLiteral("iterations.csv")));
        if (!iterations.open(QIODevice::WriteOnly | QIODevice::Text))
            return finish(3, QStringLiteral("Cannot save iteration records"));
        QTextStream iterationStream(&iterations);
        iterationStream.setRealNumberPrecision(17);
        iterationStream << "time,iterations\n";
        int recordedSteps = 0;
        for (const auto& record : model->GetOutputter().GetSolverIterationRecords())
        {
            if (record.stepId == dynamicId)
            {
                iterationStream << record.time << ',' << record.iterations << '\n';
                ++recordedSteps;
            }
        }
        iterationStream.flush();
        summary["iteration_records"] = recordedSteps;
        Hdf5ResultReader reader;
        std::vector<Hdf5ResultFrameInfo> frames;
        Hdf5ResultSummary resultSummary;
        Hdf5ResultFrame initialFrame;
        const bool resultReadable = reader.inspect(resultPath, resultSummary) && reader.open(resultPath, frames);
        const bool initialReadable = resultReadable && !frames.empty() && frames.front().stepId == dynamicId &&
                                     std::abs(frames.front().time) < 1.0e-12 && reader.readFrame(0, initialFrame);
        const bool initialSaved = initialReadable &&
                                  WriteSingleConductorInitialFrame(
                                      output.filePath(QStringLiteral("initial_state.csv")), initialFrame,
                                      initialTwists);
        summary["initial_state_saved"] = initialSaved;
        summary["result_frame_count"] = static_cast<double>(resultSummary.frameCount);
        summary["result_complete"] = resultReadable && !resultSummary.partialResult;
        summary["result_end_time"] = frames.empty() ? 0.0 : frames.back().time;
        QJsonObject dampingEvidence;
        QElapsedTimer dampingCheckTimer;
        dampingCheckTimer.start();
        // 这是求解结束后的证据检查，不放入前面的动力耗时及装配统计。
        const bool dampingValid = dynamicSolved && ValidateSingleConductorDamping(*dynamicStep, dampingEvidence);
        summary["damping_check_wall_seconds"] = dampingCheckTimer.nsecsElapsed() / 1.0e9;
        summary["damping_evidence"] = dampingEvidence;
        summary["damping_valid"] = dampingValid;
        const bool complete = dynamicSolved && dampingValid && finiteStates && initialSaved &&
                              recordedSteps == accepted &&
                              resultReadable && !resultSummary.partialResult && !frames.empty() &&
                              std::abs(lastTime - duration) < 1.0e-8 &&
                              std::abs(frames.back().time - duration) < 1.0e-8 &&
                              history.error() == QFile::NoError && iterations.error() == QFile::NoError;
        summary["completed"] = complete;
        const QString failure = !dynamicStep->LastFailureReason().isEmpty()
                                    ? QStringLiteral("Dynamic solve failed: ") + dynamicStep->LastFailureReason()
                                    : QStringLiteral("Dynamic history, endpoint, finite-state "
                                                     "or result-file check failed");
        return finish(complete ? 0 : 6, complete ? QString() : failure);
    }
    catch (const std::exception& error)
    {
        return finish(7, QStringLiteral("Unhandled case exception: ") + QString::fromUtf8(error.what()));
    }
    catch (...)
    {
        return finish(7, QStringLiteral("Unhandled case exception"));
    }
}
