#include "Application/VerificationRunner.h"
#include "Application/PaperBeamDynamicsVerification.h"

#include "Conductor/ConductorModelBuilder.h"
#include "Conductor/PropertyLibrary.h"
#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include "GUI/Controllers/ModelController.h"
#include "GUI/Controllers/SolveTaskController.h"
#include "GUI/Widgets/ConductorModule.h"
#include "Import/Input_Model.h"
#include "Solver/AnalysisSolve.h"

#include <QComboBox>
#include <QTemporaryDir>

namespace
{
std::optional<int> verifyLe2012Example1(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-le2012-example1")))
        return std::nullopt;
    return PaperBeamDynamicsVerification::RunExample1(
        QStringLiteral("output/verification/le2012_example1"));
}

std::optional<int> verifyLe2012Example4(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-le2012-example4")))
        return std::nullopt;
    return PaperBeamDynamicsVerification::Run(
        QStringLiteral("output/verification/le2012_example4"));
}

std::optional<int> verifyBeamDynamics(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-beam-dynamics")))
        return std::nullopt;

    constexpr double length = 2.0;
    constexpr double area = 0.01;
    constexpr double young = 210.0e9;
    constexpr double density = 7850.0;

    auto node0 = std::make_shared<Node>();
    auto node1 = std::make_shared<Node>();
    node0->SetNumDOFs(6);
    node1->SetNumDOFs(6);
    node1->m_X = length;

    auto material = std::make_shared<Material>();
    material->m_Young = young;
    material->m_Poisson = 0.3;
    material->m_Density = density;
    auto section = std::make_shared<SectionCircular>();
    section->m_Area = area;
    auto property = std::make_shared<Property>();
    property->m_pMaterial = material;
    property->m_pSection = section;

    ElementBeam_CR beam;
    beam.m_pNode[0] = node0;
    beam.m_pNode[1] = node1;
    beam.m_pProperty = property;
    beam.q0 = Eigen::Vector3d::UnitY();
    beam.Get_L0();

    Eigen::MatrixXd mass;
    beam.Get_me_Consistent(mass);
    const double expectedMass = density * area * length;
    double assembledMass = 0.0;
    for (int a = 0; a < 2; ++a)
        for (int b = 0; b < 2; ++b)
            assembledMass += mass(6 * a, 6 * b);
    const double symmetryError =
        (mass - mass.transpose()).cwiseAbs().maxCoeff();
    const double massError =
        std::abs(assembledMass - expectedMass) / expectedMass;

    node0->m_Velocity[3] = 1.0;
    node0->m_Velocity[4] = 1.0;
    node1->m_Velocity[3] = 1.0;
    node1->m_Velocity[4] = 1.0;
    Eigen::VectorXd inertiaForce;
    beam.Get_InertiaForce(inertiaForce);
    double Iy = 0.0;
    double Iz = 0.0;
    double polar = 0.0;
    section->Calculate_I(Iy, Iz, polar);
    const double expectedNodalGyroscopicMoment =
        -0.5 * length * density * Iy;
    const double gyroscopicError = std::abs(
        inertiaForce(5) - expectedNodalGyroscopicMoment)
        / std::abs(expectedNodalGyroscopicMoment);

    const double stiffness = young * area / length;
    const double effectiveMass = mass(6, 6);
    const double omega = std::sqrt(stiffness / effectiveMass);
    const double period = 2.0 * std::acos(-1.0) / omega;
    const int steps = 200;
    const double dt = period / steps;
    constexpr double beta = 0.25;
    constexpr double gamma = 0.5;
    constexpr double initialDisplacement = 1.0e-3;
    double displacement = initialDisplacement;
    double velocity = 0.0;
    double acceleration = -stiffness * displacement / effectiveMass;
    for (int step = 0; step < steps; ++step)
    {
        const double predictedDisplacement = displacement
            + dt * velocity + dt * dt * (0.5 - beta) * acceleration;
        const double predictedVelocity =
            velocity + dt * (1.0 - gamma) * acceleration;
        const double newAcceleration =
            -stiffness * predictedDisplacement
            / (effectiveMass + beta * dt * dt * stiffness);
        displacement =
            predictedDisplacement + beta * dt * dt * newAcceleration;
        velocity = predictedVelocity + gamma * dt * newAcceleration;
        acceleration = newAcceleration;
    }
    const double responseError =
        std::abs(displacement - initialDisplacement)
        / initialDisplacement;

    QTextStream(stdout)
        << "beam dynamics mass_error=" << massError
        << " symmetry_error=" << symmetryError
        << " gyroscopic_error=" << gyroscopicError
        << " period=" << period
        << " one_period_response_error=" << responseError
        << Qt::endl;
    return massError <= 1.0e-12
        && symmetryError <= 1.0e-12
        && gyroscopicError <= 1.0e-12
        && responseError <= 2.0e-3 ? 0 : 1;
}

std::optional<int> verifyCableTorsion(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-cable-torsion")))
        return std::nullopt;

    // Uniform circular cable: fixed at x=0, torque applied at x=L.
    // Saint-Venant solution: T = GJ*theta/L, and the fixed-free first
    // torsional frequency is omega_1 = pi/(2L)*sqrt(G/rho).
    constexpr double length = 10.0;
    constexpr double radius = 0.02;
    constexpr double young = 200.0e9;
    constexpr double poisson = 0.30;
    constexpr double density = 7850.0;
    constexpr double appliedTwist = 0.01;
    constexpr int elementCount = 32;

    const double area = std::acos(-1.0) * radius * radius;
    const double polarMoment = 0.5 * area * radius * radius;
    const double shearModulus = young / (2.0 * (1.0 + poisson));

    auto material = std::make_shared<Material>();
    material->m_Young = young;
    material->m_Poisson = poisson;
    material->m_Density = density;
    auto section = std::make_shared<SectionCircular>();
    section->m_Area = area;
    auto property = std::make_shared<Property>();
    property->m_pMaterial = material;
    property->m_pSection = section;

    auto node0 = std::make_shared<Node>();
    auto node1 = std::make_shared<Node>();
    node0->SetNumDOFs(4);
    node1->SetNumDOFs(4);
    node1->m_X = length;
    node1->m_Displacement[3] = appliedTwist;
    ElementCable staticCable;
    staticCable.m_pNode[0] = node0;
    staticCable.m_pNode[1] = node1;
    staticCable.m_pProperty = property;
    staticCable.m_InitStress = 100.0e6;
    Eigen::MatrixXd staticStiffness;
    staticCable.Get_ke(staticStiffness);
    const double expectedTorque = shearModulus * polarMoment / length * appliedTwist;
    const double staticError = std::abs(staticCable.m_inforce(7) - expectedTorque)
        / expectedTorque;
    const double torsionalStiffnessError = std::abs(
        staticStiffness(7, 7) - shearModulus * polarMoment / length)
        / (shearModulus * polarMoment / length);

    // A compressed, initially unstressed cable must go slack: axial terms
    // disappear, whereas the explicitly modelled torsional DOF remains valid.
    node1->m_Displacement[0] = -1.0e-3;
    node1->m_Displacement[3] = 0.0;
    staticCable.m_InitStress = 0.0;
    Eigen::MatrixXd slackStiffness;
    staticCable.Get_ke(slackStiffness);
    const double slackAxialForce = staticCable.m_inforce.head<3>().norm()
        + staticCable.m_inforce.segment<3>(4).norm();
    const double slackTorsionalStiffness = slackStiffness(7, 7);

    std::vector<std::shared_ptr<Node>> nodes(elementCount + 1);
    for (int index = 0; index <= elementCount; ++index)
    {
        nodes[index] = std::make_shared<Node>();
        nodes[index]->SetNumDOFs(4);
        nodes[index]->m_X = length * index / elementCount;
    }
    Eigen::MatrixXd torsionalK = Eigen::MatrixXd::Zero(elementCount, elementCount);
    Eigen::MatrixXd torsionalM = Eigen::MatrixXd::Zero(elementCount, elementCount);
    for (int index = 0; index < elementCount; ++index)
    {
        ElementCable cable;
        cable.m_pNode[0] = nodes[index];
        cable.m_pNode[1] = nodes[index + 1];
        cable.m_pProperty = property;
        cable.m_InitStress = 100.0e6;
        Eigen::MatrixXd stiffness;
        Eigen::MatrixXd mass;
        cable.Get_ke(stiffness);
        cable.Get_me_Consistent(mass);
        const int localNode[2] = {index - 1, index};
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b)
                if (localNode[a] >= 0 && localNode[b] >= 0)
                {
                    torsionalK(localNode[a], localNode[b]) += stiffness(4 * a + 3, 4 * b + 3);
                    torsionalM(localNode[a], localNode[b]) += mass(4 * a + 3, 4 * b + 3);
                }
    }
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> modes(torsionalK, torsionalM);
    if (modes.info() != Eigen::Success)
        return 2;
    const double computedOmega = std::sqrt(modes.eigenvalues()(0));
    const double expectedOmega = std::acos(-1.0) / (2.0 * length)
        * std::sqrt(shearModulus / density);
    const double frequencyError = std::abs(computedOmega - expectedOmega) / expectedOmega;

    // Newmark average-acceleration integration of that FEM first torsional
    // mode. One calculated period must recover its initial amplitude.
    const double period = 2.0 * std::acos(-1.0) / computedOmega;
    constexpr int steps = 400;
    const double dt = period / steps;
    double displacement = appliedTwist;
    double velocity = 0.0;
    double acceleration = -computedOmega * computedOmega * displacement;
    for (int step = 0; step < steps; ++step)
    {
        const double predictedDisplacement = displacement + dt * velocity
            + 0.25 * dt * dt * acceleration;
        const double predictedVelocity = velocity + 0.5 * dt * acceleration;
        const double newAcceleration = -computedOmega * computedOmega
            * predictedDisplacement / (1.0 + 0.25 * dt * dt * computedOmega * computedOmega);
        displacement = predictedDisplacement + 0.25 * dt * dt * newAcceleration;
        velocity = predictedVelocity + 0.5 * dt * newAcceleration;
        acceleration = newAcceleration;
    }
    const double transientError = std::abs(displacement - appliedTwist) / appliedTwist;

    QTextStream(stdout)
        << "cable torsion static_torque_error=" << staticError
        << " static_stiffness_error=" << torsionalStiffnessError
        << " slack_axial_force=" << slackAxialForce
        << " slack_torsional_stiffness=" << slackTorsionalStiffness
        << " omega=" << computedOmega
        << " omega_reference=" << expectedOmega
        << " frequency_error=" << frequencyError
        << " one_period_response_error=" << transientError << Qt::endl;
    return staticError <= 1.0e-12
        && torsionalStiffnessError <= 1.0e-12
        && slackAxialForce <= 1.0e-8
        && slackTorsionalStiffness > 0.0
        && frequencyError <= 2.0e-3
        && transientError <= 2.0e-3 ? 0 : 3;
}

std::optional<int> verifyNodeExport(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-node-export"));
    if (index < 0)
        return std::nullopt;
    if (index + 2 >= arguments.size())
        return 1;

    const QString source = arguments.at(index + 1);
    const QString target = arguments.at(index + 2);
    Hdf5ModelIO exporter;
    const bool exported =
        exporter.ExportBdfResultFromHdf5(source, target, {1},
                                         {EnumKeyword::NodeResultType::U1, EnumKeyword::NodeResultType::U2,
                                          EnumKeyword::NodeResultType::U3, EnumKeyword::NodeResultType::MagnitudeU},
                                         {}, {});
    QTextStream(stdout) << "node export=" << (exported ? "success" : "failed") << " file=" << target << Qt::endl;
    return exported ? 0 : 2;
}

std::optional<int> verifyResultRead(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-result-read"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    Hdf5ModelIO reader;
    std::vector<Hdf5ResultFrameInfo> frames;
    if (!reader.OpenResultFile(arguments.at(index + 1), frames) || frames.empty())
        return 2;
    Hdf5ResultFrame first;
    Hdf5ResultFrame last;
    if (!reader.ReadResultFrame(0, first) || !reader.ReadResultFrame(static_cast<int>(frames.size()) - 1, last))
    {
        return 3;
    }
    QTextStream(stdout) << "result frames=" << frames.size() << " first_step=" << first.info.stepId
                        << " first_nodes=" << first.nodes.size() << " first_elements=" << first.elements.size()
                        << " last_time=" << last.info.time << Qt::endl;
    return first.nodes.empty() || first.elements.empty() ? 4 : 0;
}

std::optional<int> verifyShearReleaseResult(
    const QStringList& arguments)
{
    const int index = arguments.indexOf(
        QStringLiteral("--verify-shear-release-result"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    Hdf5ModelIO reader;
    std::vector<Hdf5ResultFrameInfo> frames;
    if (!reader.OpenResultFile(arguments.at(index + 1), frames)
        || frames.empty())
        return 2;

    double maximumConstraintGap = 0.0;
    double maximumNormalizedError = 0.0;
    double finalValues[4] = {};
    for (int frameIndex = 0;
         frameIndex < static_cast<int>(frames.size());
         ++frameIndex)
    {
        Hdf5ResultFrame frame;
        if (!reader.ReadResultFrame(frameIndex, frame))
            return 3;
        const auto findNode = [&frame](int id)
            -> const Hdf5NodalResult*
        {
            const auto found = std::find_if(
                frame.nodes.cbegin(), frame.nodes.cend(),
                [id](const Hdf5NodalResult& node)
                {
                    return node.id == id;
                });
            return found == frame.nodes.cend() ? nullptr : &*found;
        };
        const Hdf5NodalResult* node2 = findNode(2);
        const Hdf5NodalResult* node3 = findNode(3);
        if (!node2 || !node3)
            return 4;

        const double phi =
            2.0 * std::acos(-1.0) * frame.info.time;
        if (std::abs(phi) < 1.0e-12)
            continue;
        const double sinPhi = std::sin(phi);
        const double cosPhi = std::cos(phi);
        const double u2Expected = sinPhi / phi - 1.0;
        const double w2Expected = (1.0 - cosPhi) / phi;
        const double u3Expected =
            u2Expected
            + 2.0 * std::tan(phi) * (1.0 - cosPhi) / phi;
        const double w3Expected = -w2Expected;
        const double calculated[4] = {
            node2->displacement[0], node2->displacement[1],
            node3->displacement[0], node3->displacement[1]
        };
        const double expected[4] = {
            u2Expected, w2Expected, u3Expected, w3Expected
        };
        for (int component = 0; component < 4; ++component)
        {
            if (!std::isfinite(calculated[component]))
                return 5;
            // The paper also reports visibly larger discretization errors in
            // the load steps close to M=pi and 3*pi, where u3 is singular.
            if (std::abs(cosPhi) >= 0.10)
            {
                maximumNormalizedError = std::max(
                    maximumNormalizedError,
                    std::abs(calculated[component] - expected[component])
                        / std::max(1.0, std::abs(expected[component])));
            }
            if (frameIndex + 1 == static_cast<int>(frames.size()))
                finalValues[component] = calculated[component];
        }

        const double constraintGap =
            cosPhi * (calculated[0] - calculated[2])
            + sinPhi * (calculated[1] - calculated[3]);
        maximumConstraintGap =
            std::max(maximumConstraintGap, std::abs(constraintGap));
    }
    reader.CloseResultFile();

    QTextStream(stdout)
        << "shear-release frames=" << frames.size()
        << " max_constraint_gap=" << maximumConstraintGap
        << " max_normalized_analytical_error="
        << maximumNormalizedError
        << " final_u2=" << finalValues[0]
        << " final_w2=" << finalValues[1]
        << " final_u3=" << finalValues[2]
        << " final_w3=" << finalValues[3] << Qt::endl;
    return maximumConstraintGap <= 1.0e-8
        && maximumNormalizedError <= 0.25 ? 0 : 6;
}

std::optional<int> verifyShearReleaseModel(
    const QStringList& arguments)
{
    const int index = arguments.indexOf(
        QStringLiteral("--verify-shear-release-model"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), structure))
        return 2;
    structure->m_OutputControl.m_StreamResult = false;
    AnalysisRunner runner;
    runner.SetStructure(structure);
    runner.SetRuntimeCallbacks(
        [](double progress, const QString& message)
        {
            if (message.contains(QStringLiteral("失败"))
                || message.contains(QStringLiteral("未收敛"))
                || message.contains(QStringLiteral("奇异")))
            {
                QTextStream(stdout)
                    << "progress=" << progress
                    << " message=" << message << Qt::endl;
            }
        },
        []() { return false; });
    if (!runner.RunAll())
        return 3;

    const auto& frames = structure->GetOutputter().GetFrames();
    if (frames.empty())
        return 4;
    double maximumConstraintGap = 0.0;
    double maximumRotationGap = 0.0;
    double maximumNormalizedError = 0.0;
    double worstPhi = 0.0;
    double worstCalculated = 0.0;
    double worstExpected = 0.0;
    int worstComponent = -1;
    for (const DataFrame& frame : frames)
    {
        const double u2 = frame.GetNodeData(
            2, EnumKeyword::NodeResultType::U1);
        const double w2 = frame.GetNodeData(
            2, EnumKeyword::NodeResultType::U2);
        const double u3 = frame.GetNodeData(
            3, EnumKeyword::NodeResultType::U1);
        const double w3 = frame.GetNodeData(
            3, EnumKeyword::NodeResultType::U2);
        const double phi2Output = frame.GetNodeData(
            2, EnumKeyword::NodeResultType::UR3);
        const double phi3 = frame.GetNodeData(
            3, EnumKeyword::NodeResultType::UR3);
        const double phi2 =
            2.0 * std::acos(-1.0) * frame.GetTime();
        const double sinPhi = std::sin(phi2);
        const double cosPhi = std::cos(phi2);
        const double expected[4] = {
            sinPhi / phi2 - 1.0,
            (1.0 - cosPhi) / phi2,
            sinPhi / phi2 - 1.0
                + 2.0 * std::tan(phi2)
                    * (1.0 - cosPhi) / phi2,
            -(1.0 - cosPhi) / phi2
        };
        const double calculated[4] = {u2, w2, u3, w3};
        for (int component = 0; component < 4; ++component)
        {
            if (!std::isfinite(calculated[component]))
                return 5;
            if (std::abs(cosPhi) >= 0.10)
            {
                const double error =
                    std::abs(calculated[component] - expected[component])
                    / std::max(1.0, std::abs(expected[component]));
                if (error > maximumNormalizedError)
                {
                    maximumNormalizedError = error;
                    worstPhi = phi2;
                    worstCalculated = calculated[component];
                    worstExpected = expected[component];
                    worstComponent = component;
                }
            }
        }
        maximumConstraintGap = std::max(
            maximumConstraintGap,
            std::abs(cosPhi * (u2 - u3)
                + sinPhi * (w2 - w3)));
        maximumRotationGap = std::max(
            maximumRotationGap,
            std::abs(std::remainder(
                phi2Output - phi3, 2.0 * std::acos(-1.0))));
    }

    const DataFrame& last = frames.back();
    QTextStream(stdout)
        << "shear-release direct frames=" << frames.size()
        << " max_constraint_gap=" << maximumConstraintGap
        << " max_rotation_gap=" << maximumRotationGap
        << " max_normalized_analytical_error="
        << maximumNormalizedError
        << " worst_phi=" << worstPhi
        << " worst_component=" << worstComponent
        << " worst_calculated=" << worstCalculated
        << " worst_expected=" << worstExpected
        << " final_u2=" << last.GetNodeData(
            2, EnumKeyword::NodeResultType::U1)
        << " final_w2=" << last.GetNodeData(
            2, EnumKeyword::NodeResultType::U2)
        << " final_u3=" << last.GetNodeData(
            3, EnumKeyword::NodeResultType::U1)
        << " final_w3=" << last.GetNodeData(
            3, EnumKeyword::NodeResultType::U2)
        << Qt::endl;
    return maximumConstraintGap <= 1.0e-8
        && maximumRotationGap <= 1.0e-8
        && maximumNormalizedError <= 0.25 ? 0 : 6;
}

std::optional<int> verifyHdf5Model(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-hdf5-model"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Hdf5ModelIO reader;
    if (!reader.ImportHdf5(arguments.at(index + 1), structure.get()))
        return 2;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        if (!element || element->m_pNode.size() != 2 || element->m_pNode[0].expired()
            || element->m_pNode[1].expired())
        {
            QTextStream(stderr) << "invalid element=" << elementId << Qt::endl;
            return 3;
        }
    }
    QTextStream(stdout) << "hdf5 model nodes=" << structure->m_Nodes.size()
                        << " elements=" << structure->m_Elements.size()
                        << " materials=" << structure->m_Material.size()
                        << " sections=" << structure->m_Section.size() << Qt::endl;
    return structure->m_Nodes.empty() || structure->m_Elements.empty() ? 4 : 0;
}

std::optional<int> verifyHdf5ModelContract(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-hdf5-model-contract"));
    if (index < 0)
        return std::nullopt;
    if (index + 2 >= arguments.size())
        return 1;

    auto source = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), source))
        return 2;
    source->m_ModelSets.clear();
    source->m_ComputeRegions.clear();
    QString error;
    const int secondElementId = source->m_Elements.size() > 2
        ? source->m_Elements.crbegin()->first : 2;
    const int firstSetId = source->AddModelSet(QStringLiteral("区域A单元"), ModelSetType::Element, {1}, &error);
    const int secondSetId = source->AddModelSet(
        QStringLiteral("区域B单元"), ModelSetType::Element,
        {secondElementId}, &error);
    const int firstRegionId = source->AddComputeRegionFromSets(
        QStringLiteral("区域A"), {firstSetId}, true, &error);
    const int secondRegionId = source->AddComputeRegionFromSets(
        QStringLiteral("区域B"), {secondSetId}, true, &error);
    if (firstSetId <= 0 || secondSetId <= 0 || firstRegionId <= 0 || secondRegionId <= 0)
        return 3;

    source->EnsureDefaultAnalysisConfiguration();
    if (source->m_AnalysisStep.empty() || !source->m_AnalysisStep.begin()->second)
        return 4;
    auto sourceStep = source->m_AnalysisStep.begin()->second;
    sourceStep->m_Name = QStringLiteral("导线区域分析");
    sourceStep->m_RegionScope = AnalysisRegionScope::SelectedRegions;
    sourceStep->m_ComputeRegionIds = {firstRegionId, secondRegionId};

    Hdf5ModelIO writer;
    if (!writer.ExportModelHdf5(arguments.at(index + 2), source.get(), arguments.at(index + 1)))
        return 5;
    Hdf5ResultSummary summary;
    if (!writer.InspectHdf5(arguments.at(index + 2), summary) || !summary.hasModel || summary.hasResult)
        return 6;

    auto restored = std::make_shared<StructureData>();
    if (!writer.ImportHdf5(arguments.at(index + 2), restored.get()))
        return 7;
    const auto restoredStep = restored->m_AnalysisStep.find(sourceStep->m_Id);
    if (restored->m_ModelSets.size() != 2 || restored->m_ComputeRegions.size() != 2
        || restored->m_MPCConstraints.size() != source->m_MPCConstraints.size()
        || restoredStep == restored->m_AnalysisStep.cend() || !restoredStep->second
        || restoredStep->second->m_Name != sourceStep->m_Name
        || restoredStep->second->m_RegionScope != AnalysisRegionScope::SelectedRegions
        || restoredStep->second->m_ComputeRegionIds.size() != 2)
    {
        return 8;
    }
    QTextStream(stdout) << "hdf5 contract sets=" << restored->m_ModelSets.size()
        << " regions=" << restored->m_ComputeRegions.size()
        << " mpcs=" << restored->m_MPCConstraints.size()
        << " steps=" << restored->m_AnalysisStep.size()
        << " result=" << (summary.hasResult ? 1 : 0) << Qt::endl;
    return 0;
}

std::optional<int> verifyComputeRegions(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-compute-regions"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), structure))
        return 2;
    structure->m_ComputeRegions.clear();
    QString error;
    const int firstRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 A"), {}, {1}, {}, true, &error);
    const int secondRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 B"), {}, {2}, {}, true, &error);
    if (firstRegionId <= 0 || secondRegionId <= 0 || structure->m_ComputeRegions.size() != 2)
    {
        QTextStream(stderr) << "region setup failed: " << error << Qt::endl;
        return 3;
    }
    const int mergedRegionId = structure->AddComputeRegion(
        QStringLiteral("区域 A 重叠候选"), {}, {1}, {}, true, &error);
    if (mergedRegionId != firstRegionId || structure->m_ComputeRegions.size() != 2)
    {
        QTextStream(stderr) << "overlap merge failed: " << error << Qt::endl;
        return 4;
    }

    structure->m_OutputControl.m_StreamResult = false;
    AnalysisRunner runner;
    runner.SetStructure(structure);
    if (!runner.RunAll() || structure->GetOutputter().GetDataSet().empty())
        return 5;
    const auto& frame = structure->GetOutputter().GetDataSet().back();
    QTextStream(stdout) << "compute regions=" << structure->m_ComputeRegions.size()
        << " merged_id=" << mergedRegionId
        << " result_nodes=" << frame.GetNodeDatas().size()
        << " result_elements=" << frame.GetElementDatas().size() << Qt::endl;
    return frame.GetNodeDatas().size() == 4 && frame.GetElementDatas().size() == 2 ? 0 : 6;
}

std::optional<int> verifyMPCRegions(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-mpc-regions"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), structure)
        || structure->m_MPCConstraints.empty())
    {
        return 2;
    }
    const auto sourceMPC = structure->m_MPCConstraints.cbegin()->second;
    const std::vector<int> nodeIds = sourceMPC ? sourceMPC->GetNodeIds()
                                               : std::vector<int>();
    if (nodeIds.size() != 2)
        return 3;
    const int masterNodeId = nodeIds[0];
    const int slaveNodeId = nodeIds[1];

    int detachedElementId = -1;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        bool containsSlave = false;
        if (element)
        {
            for (const auto& nodeRef : element->m_pNode)
            {
                const auto node = nodeRef.lock();
                containsSlave = containsSlave
                    || (node && node->m_Id == slaveNodeId);
            }
        }
        if (element && !containsSlave)
        {
            detachedElementId = elementId;
            break;
        }
    }
    if (detachedElementId < 0)
        return 4;

    structure->m_ComputeRegions.clear();
    QString error;
    const int regionId = structure->AddComputeRegion(
        QStringLiteral("MPC 主节点区域"), {masterNodeId},
        {detachedElementId}, {}, true, &error);
    const auto region = structure->m_ComputeRegions.find(regionId);
    if (regionId <= 0 || region == structure->m_ComputeRegions.cend()
        || !region->second || !region->second->ContainsNode(masterNodeId)
        || !region->second->ContainsNode(slaveNodeId)
        || structure->m_MPCConstraints.size() != 1)
    {
        QTextStream(stderr) << "MPC region expansion failed: "
                            << error << Qt::endl;
        return 5;
    }

    const int stepId = structure->m_AnalysisStep.empty()
        ? 0 : structure->m_AnalysisStep.cbegin()->first;
    if (stepId <= 0)
        return 6;
    auto clone = structure->CloneRegionForAnalysis(regionId, stepId, &error);
    if (!clone || clone->m_MPCConstraints.size() != 1
        || clone->m_Nodes.find(masterNodeId) == clone->m_Nodes.cend()
        || clone->m_Nodes.find(slaveNodeId) == clone->m_Nodes.cend()
        || structure->m_MPCConstraints.size() != 1)
    {
        QTextStream(stderr) << "MPC regional clone failed: "
                            << error << Qt::endl;
        return 7;
    }

    QTextStream(stdout)
        << "mpc regions=" << structure->m_ComputeRegions.size()
        << " master=" << masterNodeId
        << " slave=" << slaveNodeId
        << " source_mpcs=" << structure->m_MPCConstraints.size()
        << " clone_mpcs=" << clone->m_MPCConstraints.size()
        << Qt::endl;
    return 0;
}

std::optional<int> verifyConductorBundle(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-conductor-bundle")))
        return std::nullopt;

    auto structure = std::make_shared<StructureData>();
    Conductor::PropertyLibrary library;
    QString propertyError;
    if (!library.load(propertyError) || !library.isReady())
    {
        QTextStream(stderr) << "conductor property library failed: " << propertyError << Qt::endl;
        return 1;
    }

    auto property = library.instantiateProperty(0, 0, *structure, propertyError);
    if (!property)
    {
        QTextStream(stderr) << "conductor property failed: " << propertyError << Qt::endl;
        return 2;
    }

    Conductor::LineBuildConfig config;
    config.start = Vector3d(0.0, 0.0, 20.0);
    config.end = Vector3d(300.0, 0.0, 20.0);
    config.property = property;
    config.elementType = EnumKeyword::ElementType::T3D2;
    config.conductor.nBundle = 4;
    config.conductor.spacing = 0.45;
    config.conductor.segments = 10;
    config.conductor.stress0 = 50.0e6;
    config.conductor.connecttype = Conductor::ConnectionMode::Parallel;
    config.convergeBundleEnds = true;
    config.setNamePrefix = QStringLiteral("验证单档");

    Conductor::ConductorModelBuilder builder(structure);
    Conductor::LineBuildResult result;
    std::string buildError;
    Conductor::SpanConductorBuildConfig spanConfig;
    spanConfig.line = config;
    spanConfig.useInnerSpacerLayout = true;
    spanConfig.innerSpacerLayout.useEqualSpacing = false;
    spanConfig.innerSpacerLayout.spacer.elementType = EnumKeyword::ElementType::CR3D;
    spanConfig.innerSpacerLayout.spacer.property = property;
    if (!builder.BuildSpanConductor(spanConfig, result, buildError))
    {
        QTextStream(stderr) << "conductor build failed: " << QString::fromStdString(buildError) << Qt::endl;
        return 3;
    }

    if (result.subConductors.size() != 4
        || result.leftSupportNodeId <= 0
        || result.rightSupportNodeId <= 0
        || result.leftTensionEnd.groupNodeIds.size() != 2
        || result.rightTensionEnd.groupNodeIds.size() != 2
        || result.leftTensionEnd.yokeElementIds.size() != 2
        || result.rightTensionEnd.yokeElementIds.size() != 2
        || result.leftTensionEnd.stabilizerElementId <= 0
        || result.rightTensionEnd.stabilizerElementId <= 0
        || result.innerSpacers.empty()
        || structure->m_ModelSets.size() != 8)
    {
        return 4;
    }

    for (const auto& [wireId, sub] : result.subConductors)
    {
        if (sub.nodeIds.empty()
            || sub.nodeSetId <= 0
            || sub.elementSetId <= 0)
        {
            return 5;
        }
    }
    if (result.subConductors.at(0).nodeIds.front() != result.subConductors.at(1).nodeIds.front()
        || result.subConductors.at(2).nodeIds.front() != result.subConductors.at(3).nodeIds.front()
        || result.subConductors.at(0).nodeIds.front() == result.subConductors.at(2).nodeIds.front())
        return 10;

    int expectedConductorNodeId = 1;
    for (std::size_t groupIndex = 0; groupIndex < result.leftTensionEnd.groupNodeIds.size(); ++groupIndex)
    {
        const int leftGroupId = result.leftTensionEnd.groupNodeIds[groupIndex];
        const int rightGroupId = result.rightTensionEnd.groupNodeIds[groupIndex];
        if (leftGroupId != expectedConductorNodeId++)
            return 12;
        for (const auto& [wireId, sub] : result.subConductors)
        {
            if (sub.nodeIds.front() != leftGroupId)
                continue;
            for (std::size_t index = 1; index + 1 < sub.nodeIds.size(); ++index)
            {
                if (sub.nodeIds[index] != expectedConductorNodeId++)
                    return 13;
            }
            if (sub.nodeIds.back() != rightGroupId)
                return 14;
        }
        if (rightGroupId != expectedConductorNodeId++)
            return 15;
    }
    if (result.leftSupportNodeId != expectedConductorNodeId++
        || result.rightSupportNodeId != expectedConductorNodeId++)
        return 16;

    int expectedId = 1;
    for (const auto& [nodeId, node] : structure->m_Nodes)
    {
        if (!node || nodeId != expectedId++)
            return 6;
    }
    expectedId = 1;
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        if (!element || elementId != expectedId++)
            return 7;
    }

    int expectedConductorElementId = 1;
    for (const auto& [wireId, sub] : result.subConductors)
    {
        for (int elementId : sub.elementIds)
        {
            const auto element = structure->FindElement(elementId);
            if (elementId != expectedConductorElementId++
                || !element
                || element->m_Role != ElementRole::Conductor
                || element->m_WireId != wireId
                || element->m_AeroProfileId != wireId
                || !element->HasAerodynamicLoad())
                return 17;
        }
    }
    for (const auto* end : {&result.leftTensionEnd, &result.rightTensionEnd})
    {
        std::vector<int> hardwareIds = end->yokeElementIds;
        if (end->stabilizerElementId > 0)
            hardwareIds.push_back(end->stabilizerElementId);
        for (int elementId : hardwareIds)
        {
            const auto element = structure->FindElement(elementId);
            if (!element
                || element->m_Role != ElementRole::TensionHardware
                || element->HasAerodynamicLoad())
                return 18;
        }
    }
    for (const auto& spacer : result.innerSpacers)
    {
        for (int elementId : spacer.elementIds)
        {
            const auto element = structure->FindElement(elementId);
            if (!element
                || element->m_Role != ElementRole::IntraPhaseSpacer
                || element->HasAerodynamicLoad())
                return 19;
        }
    }

    const auto firstWire = result.subConductors.find(0);
    if (firstWire == result.subConductors.cend() || firstWire->second.nodeIds.size() < 3)
        return 8;
    const auto rightBottomNode = structure->FindNode(firstWire->second.nodeIds[1]);
    if (!rightBottomNode || rightBottomNode->m_Y >= 0.0 || rightBottomNode->m_Z >= config.start.z())
        return 9;
    const double transitionDx = rightBottomNode->m_X - config.start.x();
    const double transitionDy = rightBottomNode->m_Y - config.start.y();
    if (std::hypot(transitionDx, transitionDy) < 0.5)
        return 11;

    QTemporaryDir temporaryDirectory;
    const QString h5Path = temporaryDirectory.filePath(QStringLiteral("conductor_tags.h5"));
    Hdf5ModelIO hdf5;
    StructureData restored;
    if (!temporaryDirectory.isValid()
        || !hdf5.ExportModelHdf5(h5Path, structure.get(), QStringLiteral("verification"))
        || !hdf5.ImportHdf5(h5Path, &restored)
        || restored.m_Elements.size() != structure->m_Elements.size())
        return 20;
    for (const auto& [elementId, source] : structure->m_Elements)
    {
        const auto target = restored.FindElement(elementId);
        if (!source || !target
            || target->m_Role != source->m_Role
            || target->m_WireId != source->m_WireId
            || target->m_AeroProfileId != source->m_AeroProfileId)
            return 21;
    }

    QTextStream(stdout)
        << "conductor bundle nodes=" << structure->m_Nodes.size()
        << " elements=" << structure->m_Elements.size()
        << " sets=" << structure->m_ModelSets.size()
        << " supports=" << result.leftSupportNodeId << "," << result.rightSupportNodeId
        << Qt::endl;
    return 0;
}

std::optional<int> verifyConductorMultiSpan(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-conductor-multispan")))
        return std::nullopt;

    auto structure = std::make_shared<StructureData>();
    Conductor::PropertyLibrary library;
    QString propertyError;
    if (!library.load(propertyError) || !library.isReady())
        return 1;
    auto property = library.instantiateProperty(0, 0, *structure, propertyError);
    if (!property)
        return 2;

    Conductor::LineBuildConfig line;
    line.property = property;
    line.elementType = EnumKeyword::ElementType::CR3D;
    line.conductor.nBundle = 4;
    line.conductor.spacing = 0.45;
    line.conductor.segments = 6;
    line.conductor.stress0 = 50.0e6;
    line.conductor.connecttype = Conductor::ConnectionMode::Parallel;
    line.convergeBundleEnds = true;
    line.bundleEndTransitionLength = 1.0;
    line.setNamePrefix = QStringLiteral("验证多档");

    Conductor::MultiSpanConductorBuildConfig config;
    config.span.line = line;
    config.stationCenters = {
        Vector3d(0.0, 0.0, 0.0),
        Vector3d(100.0, 0.0, 0.0),
        Vector3d(220.0, 0.0, 0.0)
    };
    config.suspensionStringLength = 1.0;
    config.suspensionElementType = EnumKeyword::ElementType::T3D2;
    config.suspensionProperty = property;
    config.span.useInnerSpacerLayout = true;
    config.span.innerSpacerLayout.useEqualSpacing = true;
    config.span.innerSpacerLayout.count = 1;
    config.span.innerSpacerLayout.spacer.elementType = EnumKeyword::ElementType::CR3D;
    config.span.innerSpacerLayout.spacer.property = property;
    config.span.innerSpacerLayout.spacer.style = Conductor::InnerSpacerStyle::OuterPolygon;

    Conductor::ConductorModelBuilder builder(structure);
    Conductor::LineBuildResult result;
    std::string buildError;
    if (!builder.BuildMultiSpanConductor(config, result, buildError))
    {
        QTextStream(stderr) << "multispan conductor build failed: "
                            << QString::fromStdString(buildError) << Qt::endl;
        return 3;
    }
    if (result.spanCount != 2 ||
        result.subConductors.size() != 4 ||
        result.suspensionPoints.size() != 1 ||
        result.leftTensionEnd.groupNodeIds.size() != 2 ||
        result.rightTensionEnd.groupNodeIds.size() != 2)
        return 4;

    const auto& suspension = result.suspensionPoints.front();
    if (suspension.wireNodeIds.size() != 4 ||
        suspension.yokeElementIds.size() != 2 ||
        suspension.spacerElementIds.size() != 4 ||
        suspension.stringElementId <= 0 ||
        suspension.supportNodeId <= 0 ||
        suspension.junctionNodeId <= 0)
        return 5;

    const auto junctionIt = structure->m_Nodes.find(suspension.junctionNodeId);
    const auto supportIt = structure->m_Nodes.find(suspension.supportNodeId);
    if (junctionIt == structure->m_Nodes.end() ||
        supportIt == structure->m_Nodes.end() ||
        std::abs(junctionIt->second->m_X - 100.0) > 1.0e-9 ||
        std::abs(junctionIt->second->m_Z - 0.45) > 1.0e-9 ||
        std::abs(supportIt->second->m_Z - 1.45) > 1.0e-9)
        return 6;

    for (int wireId = 0; wireId < 4; ++wireId)
    {
        const auto& nodes = result.subConductors.at(wireId).nodeIds;
        if (std::count(nodes.begin(), nodes.end(), suspension.wireNodeIds[wireId]) != 1)
            return 7;
    }
    for (int elementId : suspension.yokeElementIds)
    {
        const auto found = structure->m_Elements.find(elementId);
        if (found == structure->m_Elements.end() ||
            found->second->m_Role != ElementRole::SuspensionHardware ||
            !std::dynamic_pointer_cast<ElementTruss>(found->second))
            return 8;
    }
    for (int elementId : suspension.spacerElementIds)
    {
        const auto found = structure->m_Elements.find(elementId);
        if (found == structure->m_Elements.end() ||
            found->second->m_Role != ElementRole::IntraPhaseSpacer ||
            found->second->HasAerodynamicLoad())
            return 8;
    }
    const auto stringIt = structure->m_Elements.find(suspension.stringElementId);
    if (stringIt == structure->m_Elements.end() ||
        stringIt->second->m_Role != ElementRole::SuspensionHardware ||
        stringIt->second->HasAerodynamicLoad())
        return 9;

    QTemporaryDir temporaryDirectory;
    const QString h5Path = temporaryDirectory.filePath(QStringLiteral("conductor_multispan.h5"));
    Hdf5ModelIO hdf5;
    StructureData restored;
    if (!temporaryDirectory.isValid() ||
        !hdf5.ExportModelHdf5(h5Path, structure.get(), QStringLiteral("multispan verification")) ||
        !hdf5.ImportHdf5(h5Path, &restored))
        return 10;
    const auto restoredString = restored.FindElement(suspension.stringElementId);
    if (!restoredString ||
        restoredString->m_Role != ElementRole::SuspensionHardware ||
        restoredString->HasAerodynamicLoad())
        return 11;

    QTemporaryDir uiOutputDirectory;
    ConductorModule module;
    module.setPropertyLibrary(&library);
    auto* modeCombo = module.findChild<QComboBox*>(QStringLiteral("modelModeCombo"));
    if (!uiOutputDirectory.isValid() || !modeCombo)
        return 12;
    modeCombo->setCurrentIndex(1);
    const auto uiBuild = module.buildModel(library, uiOutputDirectory.path());
    if (!uiBuild.succeeded() ||
        uiBuild.structure->m_Constraint.size() != 9)
    {
        QTextStream(stderr) << "multispan conductor UI build failed: "
                            << uiBuild.error
                            << " constraints="
                            << (uiBuild.structure ? uiBuild.structure->m_Constraint.size() : 0)
                            << Qt::endl;
        return 13;
    }

    QTextStream(stdout)
        << "conductor multispan spans=" << result.spanCount
        << " nodes=" << result.NodeCount()
        << " elements=" << result.ElementCount()
        << " suspension_center_z=" << suspension.center.z()
        << " junction_z=" << junctionIt->second->m_Z
        << " support_z=" << supportIt->second->m_Z
        << Qt::endl;
    return 0;
}

std::optional<int> verifySolveTask(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-solve-task"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    const QString filePath = arguments.at(index + 1);
    ModelController modelController;
    SolveTaskController taskController;
    int taskId = -1;
    QObject::connect(
        &modelController, &ModelController::loadSucceeded,
        [&modelController, &taskController, &taskId, &application](int modelId, const QString& loadedFile, qint64)
        {
            taskId = taskController.prepare(modelController.model(modelId), loadedFile, 0);
            if (taskId < 0 || !taskController.start(taskId))
                application.exit(2);
        });
    QObject::connect(&modelController, &ModelController::loadFailed,
                     [&application](const QString&, const QString&) { application.exit(3); });
    QObject::connect(&taskController, &SolveTaskController::taskUpdated,
                     [&taskController, &taskId, &application](int updatedTaskId)
                     {
                         if (updatedTaskId != taskId)
                             return;
                         const auto info = taskController.taskInfo(updatedTaskId);
                         if (info.status != SolveTaskController::Status::Completed &&
                             info.status != SolveTaskController::Status::Failed &&
                             info.status != SolveTaskController::Status::Cancelled)
                         {
                             return;
                         }
                         QTextStream(stdout)
                             << "solve status=" << SolveTaskController::statusText(info.status)
                             << " elapsed_ms=" << info.elapsedMs << " message=" << info.message << Qt::endl;
                         application.exit(info.status == SolveTaskController::Status::Completed ? 0 : 4);
                     });
    QTimer::singleShot(120000, &application, [&application]() { application.exit(5); });
    if (!modelController.loadModel(filePath))
        return 1;
    return application.exec();
}

std::optional<int> verifyPartialResult(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-partial-result"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    const QString filePath = arguments.at(index + 1);
    ModelController modelController;
    SolveTaskController taskController;
    int taskId = -1;
    bool cancellationRequested = false;
    QObject::connect(
        &modelController, &ModelController::loadSucceeded,
        [&modelController, &taskController, &taskId, &application](int modelId, const QString& loadedFile, qint64)
        {
            taskId = taskController.prepare(modelController.model(modelId), loadedFile, 0);
            if (taskId < 0 || !taskController.start(taskId))
                application.exit(2);
        });
    QObject::connect(&modelController, &ModelController::loadFailed,
                     [&application](const QString&, const QString&) { application.exit(3); });
    QObject::connect(
        &taskController, &SolveTaskController::taskUpdated,
        [&taskController, &taskId, &cancellationRequested, &application](int updatedTaskId)
        {
            if (updatedTaskId != taskId)
                return;
            const auto info = taskController.taskInfo(updatedTaskId);
            if (!cancellationRequested && info.status == SolveTaskController::Status::Running && info.progress >= 0.05)
            {
                cancellationRequested = taskController.cancel(updatedTaskId);
                return;
            }
            if (info.status != SolveTaskController::Status::Completed &&
                info.status != SolveTaskController::Status::Failed &&
                info.status != SolveTaskController::Status::Cancelled)
            {
                return;
            }
            QTextStream(stdout) << "partial status=" << SolveTaskController::statusText(info.status)
                                << " available=" << (info.hasUsableResult ? "true" : "false")
                                << " partial=" << (info.partialResult ? "true" : "false")
                                << " frames=" << info.resultFrameCount << " end_time=" << info.resultEndTime
                                << " file=" << info.outputFile << Qt::endl;
            application.exit(info.hasUsableResult && info.partialResult && info.resultFrameCount > 0 ? 0 : 4);
        });
    QTimer::singleShot(120000, &application, [&application]() { application.exit(5); });
    if (!modelController.loadModel(filePath))
        return 1;
    return application.exec();
}

std::optional<int> verifyModelImport(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-model-import"));
    if (index < 0)
        return std::nullopt;

    const QStringList files = arguments.mid(index + 1);
    ModelController controller;
    int loadedCount = 0;
    int failedCount = 0;
    int cloneFailedCount = 0;
    QObject::connect(
        &controller, &ModelController::loadSucceeded,
        [&controller, &loadedCount, &cloneFailedCount](int modelId, const QString& filePath, qint64 elapsedMs)
        {
            ++loadedCount;
            QString cloneError;
            const auto firstClone = controller.model(modelId)->CloneForAnalysis(&cloneError);
            const auto secondClone = firstClone ? firstClone->CloneForAnalysis(&cloneError) : nullptr;
            if (!secondClone)
            {
                ++cloneFailedCount;
                QTextStream(stderr) << "clone failed model=" << modelId << " error=" << cloneError << Qt::endl;
            }
            QTextStream(stdout) << "loaded model=" << modelId << " file=" << filePath << " elapsed_ms=" << elapsedMs
                                << Qt::endl;
        });
    QObject::connect(&controller, &ModelController::loadFailed,
                     [&failedCount](const QString& filePath, const QString& errorMessage)
                     {
                         ++failedCount;
                         QTextStream(stderr) << "failed file=" << filePath << " error=" << errorMessage << Qt::endl;
                     });
    QObject::connect(
        &controller, &ModelController::busyChanged,
        [&controller, &application, &loadedCount, &failedCount, &cloneFailedCount, expected = files.size()](bool busy)
        {
            if (busy)
                return;
            const QList<int> ids = controller.modelIds();
            bool isolated = ids.size() == loadedCount;
            if (ids.size() >= 2)
            {
                const auto first = controller.model(ids.at(0));
                const auto second = controller.model(ids.at(1));
                if (first && second)
                {
                    const auto firstNode = first->FindNode(1);
                    const auto secondNode = second->FindNode(1);
                    if (firstNode && secondNode)
                        isolated = isolated && firstNode.get() != secondNode.get();
                }
            }
            QTextStream(stdout) << "summary loaded=" << loadedCount << " failed=" << failedCount
                                << " clone_failed=" << cloneFailedCount << " documents=" << controller.modelCount()
                                << " isolated=" << (isolated ? "true" : "false") << Qt::endl;
            application.exit(loadedCount == expected && failedCount == 0 && cloneFailedCount == 0 && isolated ? 0 : 1);
        });
    QTimer::singleShot(30000, &application, [&application]() { application.exit(2); });
    if (files.isEmpty() || controller.loadModels(files) != files.size())
        return 1;
    return application.exec();
}
} // namespace

std::optional<int> VerificationRunner::runHeadless(
    const QStringList& arguments)
{
    if (const auto result = verifyLe2012Example1(arguments))
        return result;
    if (const auto result = verifyLe2012Example4(arguments))
        return result;
    if (const auto result = verifyCableTorsion(arguments))
        return result;
    return verifyBeamDynamics(arguments);
}

std::optional<int> VerificationRunner::run(QApplication& application, const QStringList& arguments)
{
    if (const auto result = runHeadless(arguments))
        return result;
    if (const auto result = verifyNodeExport(arguments))
        return result;
    if (const auto result = verifyResultRead(arguments))
        return result;
    if (const auto result = verifyShearReleaseResult(arguments))
        return result;
    if (const auto result = verifyShearReleaseModel(arguments))
        return result;
    if (const auto result = verifyHdf5Model(arguments))
        return result;
    if (const auto result = verifyHdf5ModelContract(arguments))
        return result;
    if (const auto result = verifyComputeRegions(arguments))
        return result;
    if (const auto result = verifyMPCRegions(arguments))
        return result;
    if (const auto result = verifyConductorBundle(arguments))
        return result;
    if (const auto result = verifyConductorMultiSpan(arguments))
        return result;
    if (const auto result = verifySolveTask(application, arguments))
        return result;
    if (const auto result = verifyPartialResult(application, arguments))
        return result;
    return verifyModelImport(application, arguments);
}
