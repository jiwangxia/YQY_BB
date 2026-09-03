#include "Application/VerificationRunner.h"
#include "Application/ApplicationPaths.h"
#include "Application/PaperBeamDynamicsVerification.h"

#include "Conductor/ConductorModelBuilder.h"
#include "Conductor/PropertyLibrary.h"
#include "DataStructure/Aerodynamics/BundleAeroMapper.h"
#include "DataStructure/Aerodynamics/GallopingStabilityAnalyzer.h"
#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Element/ElementSpring1.h"
#include "DataStructure/Element/ElementSpring2.h"
#include "DataStructure/Element/ElementSpringA.h"
#include "DataStructure/Inertia/RigidBodyInertia.h"
#include "DataStructure/Load/Force_Wind.h"
#include "DataStructure/Load/LoadAssembler.h"
#include "DataStructure/Load/AerodynamicLoadCalculator.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include "GUI/Controllers/ModelController.h"
#include "GUI/Controllers/SolveTaskController.h"
#include "GUI/Widgets/ConductorModule.h"
#include "Import/Input_Model.h"
#include "Solver/Workflow/AnalysisSolve.h"
#include "Solver/Linear/Gpu/CudssSolver.h"
#include "Solver/Linear/Gpu/CudaSparseSolver.h"
#include "Solver/Dynamic/Integrators/SolverAdaptiveTSSBN.h"
#include "Solver/Dynamic/Integrators/SolverNewmark.h"
#include "Solver/Dynamic/Integrators/SolverRungeKutta4.h"
#include "Solver/Dynamic/Damping/StructuralDamping.h"
#include "Solver/Linear/LinearSystemSolver.h"
#include "Solver/Linear/Direct/PardisoSolver.h"
#include "Solver/Factory/SolverFactory.h"
#include "Utility/CR.h"

#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QFile>
#include <QTemporaryDir>
#include <array>
#include <cmath>
#include <numbers>

namespace
{
static QDir FindVerificationSourceRoot()
{
    QDir sourceRoot(QCoreApplication::applicationDirPath());
    while (!sourceRoot.exists(QStringLiteral("YQY/Import/ImportFile")) && sourceRoot.cdUp())
    {
    }
    return sourceRoot;
}

std::optional<int> verifySpringElements(const QStringList& arguments)
{
    const bool verifyBasicSpringElements = arguments.contains(QStringLiteral("--verify-spring-elements"));
    const bool verifyComplexSpringElements = arguments.contains(QStringLiteral("--verify-spring-complex"));
    const bool verifyDirectionContrast = arguments.contains(QStringLiteral("--verify-spring-direction-contrast"));
    if (!verifyBasicSpringElements && !verifyComplexSpringElements && !verifyDirectionContrast)
        return std::nullopt;

    if (verifyDirectionContrast)
    {
        constexpr double expectedNode2U1 = -1.0;
        constexpr double expectedNode2U2 = 1.0;
        constexpr double expectedNode4U1 = -1.0;
        constexpr double expectedNode4U2 = 1.0;
        // 路径经过 q=-0.5 拐点，终点 q=-1 的力为-3。
        constexpr double expectedElement1Force = -3.0;
        constexpr double expectedElement2Force = 0.0;

        auto structure = std::make_shared<StructureData>();
        Input_Model importer;
        const QDir sourceRoot = FindVerificationSourceRoot();
        const QString relativePath =
            QStringLiteral("YQY/Import/ImportFile/弹簧方向对比算例/01_spring2_springa_rotation_nonlinear.bdf");
        const QString path = sourceRoot.filePath(relativePath);
        const QFileInfo modelFile(path);
        const bool imported = importer.InputData(path, structure);

        bool solved = false;
        const DataFrame* finalFrame = nullptr;
        if (imported)
        {
            structure->m_OutputControl.m_StreamResult = false;
            AnalysisRunner runner;
            runner.SetStructure(structure);
            solved = runner.RunAll();
            const auto& dataSet = structure->GetOutputter().GetDataSet();
            if (solved && !dataSet.empty())
                finalFrame = &dataSet.back();
        }

        double node2U1 = 0.0;
        double node2U2 = 0.0;
        double node2U3 = 0.0;
        double node4U1 = 0.0;
        double node4U2 = 0.0;
        double node4U3 = 0.0;
        double element1Force = 0.0;
        double element2Force = 0.0;
        bool hasNode2 = false;
        bool hasNode4 = false;
        bool hasElement1 = false;
        bool hasElement2 = false;
        if (finalFrame)
        {
            hasNode2 = finalFrame->GetNodeDatas().find(2) != finalFrame->GetNodeDatas().cend();
            hasNode4 = finalFrame->GetNodeDatas().find(4) != finalFrame->GetNodeDatas().cend();
            hasElement1 = finalFrame->GetElementDatas().find(1) != finalFrame->GetElementDatas().cend();
            hasElement2 = finalFrame->GetElementDatas().find(2) != finalFrame->GetElementDatas().cend();
            if (hasNode2)
            {
                node2U1 = finalFrame->GetNodeData(2, EnumKeyword::NodeResultType::U1);
                node2U2 = finalFrame->GetNodeData(2, EnumKeyword::NodeResultType::U2);
                node2U3 = finalFrame->GetNodeData(2, EnumKeyword::NodeResultType::U3);
            }
            if (hasNode4)
            {
                node4U1 = finalFrame->GetNodeData(4, EnumKeyword::NodeResultType::U1);
                node4U2 = finalFrame->GetNodeData(4, EnumKeyword::NodeResultType::U2);
                node4U3 = finalFrame->GetNodeData(4, EnumKeyword::NodeResultType::U3);
            }
            if (hasElement1)
                element1Force = finalFrame->GetElementData(1, EnumKeyword::ElementResultType::AxialForce);
            if (hasElement2)
                element2Force = finalFrame->GetElementData(2, EnumKeyword::ElementResultType::AxialForce);
        }

        const bool continuousElementIds = structure->m_Elements.size() == 2 &&
                                          structure->m_Elements.find(1) != structure->m_Elements.cend() &&
                                          structure->m_Elements.find(2) != structure->m_Elements.cend();
        const QString resultPrefix =
            modelFile.dir().filePath(modelFile.completeBaseName() + QStringLiteral("_YQY"));
        const QString nodeResultPath = resultPrefix + QStringLiteral("_node_result.bdf");
        const QString springResultPath = resultPrefix + QStringLiteral("_spring_result.bdf");
        if (finalFrame)
        {
            const std::vector<EnumKeyword::NodeResultType> nodeTypes = {
                EnumKeyword::NodeResultType::U1,
                EnumKeyword::NodeResultType::U2,
                EnumKeyword::NodeResultType::U3};
            const std::vector<EnumKeyword::ElementResultType> elementTypes = {
                EnumKeyword::ElementResultType::AxialForce,
                EnumKeyword::ElementResultType::RelativeDisplacement};
            structure->GetOutputter().ExportNodes(nodeResultPath, {2, 4}, nodeTypes);
            structure->GetOutputter().ExportElements(springResultPath, {1, 2}, elementTypes);
        }

        const bool resultFilesValid = QFileInfo(nodeResultPath).isFile() && QFileInfo(nodeResultPath).size() > 0 &&
                                       QFileInfo(springResultPath).isFile() && QFileInfo(springResultPath).size() > 0;
        const bool displacementValid = hasNode2 && hasNode4 && std::isfinite(node2U1) && std::isfinite(node2U2) &&
                                       std::isfinite(node2U3) && std::isfinite(node4U1) && std::isfinite(node4U2) &&
                                       std::isfinite(node4U3) && std::abs(node2U1 - expectedNode2U1) <= 1.0e-6 &&
                                       std::abs(node2U2 - expectedNode2U2) <= 1.0e-6 &&
                                       std::abs(node4U1 - expectedNode4U1) <= 1.0e-6 &&
                                       std::abs(node4U2 - expectedNode4U2) <= 1.0e-6;
        const bool forceValid = hasElement1 && hasElement2 && std::isfinite(element1Force) &&
                                std::isfinite(element2Force) &&
                                std::abs(element1Force - expectedElement1Force) <= 1.0e-6 &&
                                std::abs(element2Force - expectedElement2Force) <= 1.0e-6;
        const bool valid = imported && solved && finalFrame != nullptr && continuousElementIds && displacementValid &&
                           forceValid && resultFilesValid;

        QTextStream errorStream(stderr);
        if (!imported)
            errorStream << "spring direction contrast import failed: " << path << " error=" << importer.LastError()
                        << Qt::endl;
        if (imported && !solved)
            errorStream << "spring direction contrast solve failed: " << path << Qt::endl;
        errorStream << "N2 U1=" << node2U1 << " expected=" << expectedNode2U1 << " U2=" << node2U2
                    << " expected=" << expectedNode2U2 << Qt::endl;
        errorStream << "N4 U1=" << node4U1 << " expected=" << expectedNode4U1 << " U2=" << node4U2
                    << " expected=" << expectedNode4U2 << Qt::endl;
        errorStream << "E1 AxialForce=" << element1Force << " expected=" << expectedElement1Force << Qt::endl;
        errorStream << "E2 AxialForce=" << element2Force << " expected=" << expectedElement2Force << Qt::endl;
        if (!continuousElementIds)
            errorStream << "spring direction contrast element numbering is not continuous after CleanupModel"
                        << Qt::endl;
        if (!resultFilesValid)
            errorStream << "spring direction contrast result export failed node=" << nodeResultPath
                        << " spring=" << springResultPath << Qt::endl;
        QTextStream(stdout) << "spring_direction_contrast=" << (valid ? 1 : 0) << Qt::endl;
        return valid ? 0 : 1;
    }

    bool complexSerialParallelValid = true;
    bool complexVSpringAValid = true;
    bool complexMixedSupportValid = true;
    if (verifyComplexSpringElements)
    {
        const auto runComplexCase = [](const QString& relativePath, int resultNodeId, double expectedU1,
                                       double expectedU2, const std::vector<std::pair<int, double>>& expectedForces)
        {
            auto structure = std::make_shared<StructureData>();
            Input_Model importer;
            const QDir sourceRoot = FindVerificationSourceRoot();
            const QString path = sourceRoot.filePath(relativePath);
            const QFileInfo modelFile(path);
            if (!importer.InputData(path, structure))
            {
                QTextStream(stderr) << "complex spring import failed: " << path << " error=" << importer.LastError()
                                    << Qt::endl;
                return false;
            }

            structure->m_OutputControl.m_StreamResult = false;
            AnalysisRunner runner;
            runner.SetStructure(structure);
            if (!runner.RunAll())
            {
                QTextStream(stderr) << "complex spring solve failed: " << path << Qt::endl;
                return false;
            }

            const auto& dataSet = structure->GetOutputter().GetDataSet();
            if (dataSet.empty())
            {
                QTextStream(stderr) << "complex spring result missing: " << path << Qt::endl;
                return false;
            }

            const auto& frame = dataSet.back();
            const auto nodeIt = frame.GetNodeDatas().find(resultNodeId);
            const bool hasNodeResult = nodeIt != frame.GetNodeDatas().cend();
            const double displacementU1 = hasNodeResult
                                              ? frame.GetNodeData(resultNodeId, EnumKeyword::NodeResultType::U1)
                                              : 0.0;
            const double displacementU2 = hasNodeResult
                                              ? frame.GetNodeData(resultNodeId, EnumKeyword::NodeResultType::U2)
                                              : 0.0;
            const double displacementU3 = hasNodeResult
                                              ? frame.GetNodeData(resultNodeId, EnumKeyword::NodeResultType::U3)
                                              : 0.0;

            const QString resultPrefix =
                modelFile.dir().filePath(modelFile.completeBaseName() + QStringLiteral("_YQY"));
            const QString nodeResultPath = resultPrefix + QStringLiteral("_node_result.bdf");
            const QString springResultPath = resultPrefix + QStringLiteral("_spring_result.bdf");
            const std::vector<EnumKeyword::NodeResultType> nodeTypes = {
                EnumKeyword::NodeResultType::U1,
                EnumKeyword::NodeResultType::U2,
                EnumKeyword::NodeResultType::U3};
            const std::vector<EnumKeyword::ElementResultType> elementTypes = {
                EnumKeyword::ElementResultType::AxialForce,
                EnumKeyword::ElementResultType::RelativeDisplacement};
            std::vector<int> elementIds;
            elementIds.reserve(expectedForces.size());
            for (const auto& [elementId, expectedForce] : expectedForces)
            {
                Q_UNUSED(expectedForce);
                elementIds.push_back(elementId);
            }

            // 求解成功后保留目标节点三向位移和全部目标弹簧轴力，文件名与原 BDF 完整基名对应。
            structure->GetOutputter().ExportNodes(nodeResultPath, {resultNodeId}, nodeTypes);
            structure->GetOutputter().ExportElements(springResultPath, elementIds, elementTypes);
            const bool exported = QFileInfo(nodeResultPath).isFile() && QFileInfo(nodeResultPath).size() > 0 &&
                                  QFileInfo(springResultPath).isFile() && QFileInfo(springResultPath).size() > 0;

            bool valid = hasNodeResult && std::isfinite(displacementU1) && std::isfinite(displacementU2) &&
                         std::isfinite(displacementU3) && std::abs(displacementU1 - expectedU1) <= 1.0e-6 &&
                         std::abs(displacementU2 - expectedU2) <= 1.0e-6 && exported;
            QTextStream errorStream(stderr);
            errorStream << "complex spring case=" << modelFile.fileName() << " U1=" << displacementU1
                        << " U2=" << displacementU2 << Qt::endl;
            for (const auto& [elementId, expectedForce] : expectedForces)
            {
                const auto elementIt = frame.GetElementDatas().find(elementId);
                const bool hasElementResult = elementIt != frame.GetElementDatas().cend();
                const double actualForce = hasElementResult
                                               ? frame.GetElementData(elementId,
                                                                      EnumKeyword::ElementResultType::AxialForce)
                                               : 0.0;
                errorStream << "  E" << elementId << " AxialForce=" << actualForce
                            << " expected=" << expectedForce << Qt::endl;
                valid = valid && hasElementResult && std::isfinite(actualForce) &&
                        std::abs(actualForce - expectedForce) <= 1.0e-6;
            }
            if (!exported)
                errorStream << "  result export failed node=" << nodeResultPath << " spring=" << springResultPath
                            << Qt::endl;
            return valid;
        };

        complexSerialParallelValid = runComplexCase(
            QStringLiteral("YQY/Import/ImportFile/弹簧算例3/01_串并联网络/01_serial_parallel_static_nonlinear.bdf"), 3,
            0.225, 0.0, {{1, 1.5}, {2, 1.5}, {3, 1.5}, {4, 3.0}});
        complexVSpringAValid = runComplexCase(
            QStringLiteral("YQY/Import/ImportFile/弹簧算例3/02_V形SPRINGA/02_v_springa_geometric_nonlinear.bdf"), 3,
            0.0, -0.20, {{1, -0.493355572224}, {2, -0.493355572224}});
        complexMixedSupportValid = runComplexCase(
            QStringLiteral("YQY/Import/ImportFile/弹簧算例3/03_混合支承/03_mixed_support_static_nonlinear.bdf"), 3,
            0.030987372622, 0.006440055557,
            // 导入 CleanupModel 后，弹簧单元按导入顺序连续编号。
            {{1, 0.619747459888}, {2, 0.128801107407}, {3, 0.531394958496}});
    }

    if (!verifyBasicSpringElements)
    {
        QTextStream(stdout) << "complex_serial_parallel=" << (complexSerialParallelValid ? 1 : 0)
                            << " complex_v_springa=" << (complexVSpringAValid ? 1 : 0)
                            << " complex_mixed_support=" << (complexMixedSupportValid ? 1 : 0) << Qt::endl;
        return complexSerialParallelValid && complexVSpringAValid && complexMixedSupportValid ? 0 : 1;
    }

    auto behavior = std::make_shared<SpringBehavior>();
    behavior->m_Points = {{-1000.0, -0.01}, {0.0, 0.0}, {1200.0, 0.01}};

    auto spring1Node = std::make_shared<Node>();
    spring1Node->m_Displacement[0] = 0.005;
    ElementSpring1 spring1;
    spring1.m_DOF = 0;
    spring1.m_pNode[0] = spring1Node;
    spring1.m_pSpringBehavior = behavior;
    MatrixXd spring1Stiffness;
    spring1.Get_ke(spring1Stiffness);
    const bool spring1Valid = std::abs(spring1.m_inforce[0] - 600.0) < 1.0e-12 &&
                              std::abs(spring1Stiffness(0, 0) - 120000.0) < 1.0e-12;

    auto spring2FirstNode = std::make_shared<Node>();
    auto spring2SecondNode = std::make_shared<Node>();
    spring2FirstNode->m_Displacement[0] = 0.005;
    spring2SecondNode->m_Displacement[1] = -0.003;
    ElementSpring2 spring2;
    spring2.m_FirstDOF = 0;
    spring2.m_SecondDOF = 1;
    spring2.m_pNode[0] = spring2FirstNode;
    spring2.m_pNode[1] = spring2SecondNode;
    spring2.m_pSpringBehavior = behavior;
    MatrixXd spring2Stiffness;
    spring2.Get_ke(spring2Stiffness);
    const bool spring2Valid = std::abs(spring2.m_inforce[0] - 960.0) < 1.0e-12 &&
                              std::abs(spring2.m_inforce[4] + 960.0) < 1.0e-12 &&
                              std::abs(spring2Stiffness(0, 4) + 120000.0) < 1.0e-12;

    auto springAFirstNode = std::make_shared<Node>();
    auto springASecondNode = std::make_shared<Node>();
    springASecondNode->m_X = 1.0;
    springASecondNode->m_Displacement[0] = 0.001;
    springASecondNode->m_Displacement[1] = 0.05;
    ElementSpringA springA;
    springA.m_pNode[0] = springAFirstNode;
    springA.m_pNode[1] = springASecondNode;
    springA.m_pSpringBehavior = behavior;
    MatrixXd springAStiffness;
    springA.Get_ke(springAStiffness);
    const double length = std::sqrt(1.001 * 1.001 + 0.05 * 0.05);
    const double force = 120000.0 * (length - 1.0);
    const bool springAValid = std::abs(springA.m_inforce.segment<3>(0).norm() - force) < 1.0e-10 &&
                              (springAStiffness - springAStiffness.transpose()).norm() < 1.0e-12 &&
                              springAStiffness(1, 1) > 0.0;

    const auto runStaticCase = [](const QString& fileName, int resultNodeId, int resultElementId, double expectedU1,
                                  double expectedU2, double expectedSpringForce)
    {
        auto structure = std::make_shared<StructureData>();
        Input_Model importer;
        const QDir sourceRoot = FindVerificationSourceRoot();
        const QString relativeDirectory = QStringLiteral("YQY/Import/ImportFile/SpringStaticNonlinear/");
        const QString path = sourceRoot.filePath(relativeDirectory + fileName);
        if (!importer.InputData(path, structure))
        {
            QTextStream(stderr) << "spring static import failed: " << importer.LastError() << Qt::endl;
            return false;
        }
        structure->m_OutputControl.m_StreamResult = false;
        AnalysisRunner runner;
        runner.SetStructure(structure);
        if (!runner.RunAll())
        {
            QTextStream(stderr) << "spring static solve failed: " << fileName << Qt::endl;
            return false;
        }
        const auto& dataSet = structure->GetOutputter().GetDataSet();
        const bool hasResult = !dataSet.empty();
        if (hasResult)
        {
            const QFileInfo modelFile(path);
            const QString resultName = modelFile.completeBaseName() + QStringLiteral("_YQY");
            const QString resultPrefix = modelFile.dir().filePath(resultName);
            const std::vector<EnumKeyword::NodeResultType> nodeTypes = {
                EnumKeyword::NodeResultType::U1,
                EnumKeyword::NodeResultType::U2,
                EnumKeyword::NodeResultType::U3};
            const std::vector<EnumKeyword::ElementResultType> elementTypes = {
                EnumKeyword::ElementResultType::AxialForce,
                EnumKeyword::ElementResultType::RelativeDisplacement};

            // 结果文件与模型并列保存，供曲线工具直接读取；输出节点位移、弹簧力和相对位移。
            structure->GetOutputter().ExportNodes(resultPrefix + QStringLiteral("_node_result.bdf"), {resultNodeId},
                                                   nodeTypes);
            structure->GetOutputter().ExportElements(resultPrefix + QStringLiteral("_spring_result.bdf"),
                                                      {resultElementId}, elementTypes);
        }
        const double displacementU1 = hasResult
                                          ? dataSet.back().GetNodeData(resultNodeId, EnumKeyword::NodeResultType::U1)
                                          : 0.0;
        const double displacementU2 = hasResult
                                          ? dataSet.back().GetNodeData(resultNodeId, EnumKeyword::NodeResultType::U2)
                                          : 0.0;
        const double springForce = hasResult
                                       ? dataSet.back().GetElementData(resultElementId,
                                                                      EnumKeyword::ElementResultType::AxialForce)
                                       : 0.0;
        const bool valid = hasResult && std::abs(displacementU1 - expectedU1) < 1.0e-8 &&
                           std::abs(displacementU2 - expectedU2) < 1.0e-8 &&
                           std::abs(springForce - expectedSpringForce) < 1.0e-8;
        if (!valid)
        {
            QTextStream(stderr) << "spring static displacement mismatch: " << fileName << " U1="
                                << displacementU1 << " U2=" << displacementU2 << " force=" << springForce
                                << Qt::endl;
        }
        return valid;
    };
    const bool spring1LinearStaticValid =
        runStaticCase(QStringLiteral("01_SPRING1_linear_static.bdf"), 1, 1, 0.5, 0.0, 50.0);
    const bool spring2LinearStaticValid =
        runStaticCase(QStringLiteral("02_SPRING2_linear_static.bdf"), 1, 1, 0.25, 0.0, 50.0);
    const bool springALinearStaticValid =
        runStaticCase(QStringLiteral("03_SPRINGA_linear_static.bdf"), 2, 1, 0.2, 0.0, 60.0);
    const bool spring1NonlinearStaticValid =
        runStaticCase(QStringLiteral("04_SPRING1_nonlinear_static.bdf"), 1, 1, 0.15, 0.0, 2.5);
    const bool spring2NonlinearStaticValid =
        runStaticCase(QStringLiteral("05_SPRING2_nonlinear_static.bdf"), 1, 1, 0.15, 0.0, 2.5);
    const bool springANonlinearStaticValid =
        runStaticCase(QStringLiteral("06_SPRINGA_nonlinear_static.bdf"), 2, 1, 0.275, 0.0, 2.5);
    const bool springAGeometricNonlinearStaticValid = runStaticCase(
        QStringLiteral("07_SPRINGA_geom_nonlinear_static.bdf"), 2, 1, 0.0, 0.464282623650, 180.725732102450);

    QTextStream(stdout) << "spring1=" << spring1Valid << " spring2=" << spring2Valid << " springa="
                        << springAValid << " linear_spring1=" << spring1LinearStaticValid << " linear_spring2="
                        << spring2LinearStaticValid << " linear_springa=" << springALinearStaticValid
                        << " nonlinear_spring1=" << spring1NonlinearStaticValid << " nonlinear_spring2="
                        << spring2NonlinearStaticValid << " nonlinear_springa=" << springANonlinearStaticValid
                        << " geometric_springa=" << springAGeometricNonlinearStaticValid << Qt::endl;
    if (verifyComplexSpringElements)
    {
        QTextStream(stdout) << "complex_serial_parallel=" << (complexSerialParallelValid ? 1 : 0)
                            << " complex_v_springa=" << (complexVSpringAValid ? 1 : 0)
                            << " complex_mixed_support=" << (complexMixedSupportValid ? 1 : 0) << Qt::endl;
    }
    return spring1Valid && spring2Valid && springAValid && spring1LinearStaticValid && spring2LinearStaticValid &&
                   springALinearStaticValid && spring1NonlinearStaticValid && spring2NonlinearStaticValid &&
                   springANonlinearStaticValid && springAGeometricNonlinearStaticValid &&
                   complexSerialParallelValid && complexVSpringAValid && complexMixedSupportValid
               ? 0
               : 1;
}

std::optional<int> verifyLowRankDampingSolver(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-low-rank-damping")))
        return std::nullopt;

    constexpr int dimension = 256;
    constexpr int rank = 4;
    constexpr double scale = 17.0;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(dimension * 3);
    for (int row = 0; row < dimension; ++row)
    {
        triplets.emplace_back(row, row, 6.0);
        if (row > 0)
            triplets.emplace_back(row, row - 1, -1.0);
        if (row + 1 < dimension)
            triplets.emplace_back(row, row + 1, -1.0);
    }
    SolverNameSpace::SpMat base(dimension, dimension);
    base.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::MatrixXd factor(dimension, rank);
    for (int row = 0; row < dimension; ++row)
        for (int column = 0; column < rank; ++column)
            factor(row, column) = 0.01 * std::sin((row + 1.0) * (column + 1.0) * 0.037);
    const Eigen::VectorXd rhs = Eigen::VectorXd::LinSpaced(dimension, -1.0, 2.0);
    const Eigen::MatrixXd explicitMatrix = Eigen::MatrixXd(base) + scale * factor * factor.transpose();
    const Eigen::VectorXd reference = explicitMatrix.partialPivLu().solve(rhs);
    if (!reference.allFinite())
        return 1;

    const std::array<std::pair<SolverNameSpace::LinearSolverMode, const char*>, 6> modes = {
        std::pair{SolverNameSpace::LinearSolverMode::Ldlt, "ldlt"},
        std::pair{SolverNameSpace::LinearSolverMode::Lu, "lu"},
        std::pair{SolverNameSpace::LinearSolverMode::Pardiso, "pardiso"},
        std::pair{SolverNameSpace::LinearSolverMode::CudaIterative, "cuda_iterative"},
        std::pair{SolverNameSpace::LinearSolverMode::Cudss, "cudss"},
        std::pair{SolverNameSpace::LinearSolverMode::Automatic, "automatic"}};
    for (const auto& [mode, name] : modes)
    {
        SolverNameSpace::LinearSolverSettings::SetMode(mode);
        SolverNameSpace::LinearSystemSolver solver;
        Eigen::VectorXd solution;
        if (!solver.SolveLowRank(base, factor, scale, rhs, solution))
        {
            SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::Automatic);
            QTextStream(stderr) << "low_rank_damping backend=" << name << " failed" << Qt::endl;
            return 2;
        }
        const double relativeError = (solution - reference).norm() / std::max(1.0, reference.norm());
        QTextStream(stdout) << "low_rank_damping backend=" << name << " relative_error=" << relativeError
                            << Qt::endl;
        if (!std::isfinite(relativeError) || relativeError > 1.0e-8)
        {
            SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::Automatic);
            return 3;
        }
    }
    SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::Automatic);
    return 0;
}

std::optional<int> verifyStructuralDamping(const QStringList& arguments)
{
    const int hdf5Index = arguments.indexOf(QStringLiteral("--verify-structural-damping-hdf5"));
    if (hdf5Index >= 0)
    {
        if (hdf5Index + 1 >= arguments.size())
            return 1;
        StructureData restored;
        Hdf5ModelIO hdf5;
        if (!hdf5.ImportHdf5(arguments.at(hdf5Index + 1), &restored))
            return 2;
        for (const auto& [stepId, step] : restored.m_AnalysisStep)
        {
            if (!step || !step->isDynamic || !step->m_StructuralDamping.settings.enabled)
                continue;
            const auto& settings = step->m_StructuralDamping.settings;
            QTextStream(stdout) << "structural_damping_hdf5 step=" << stepId
                                << " translation_ratio=" << settings.translationDampingRatio
                                << " torsion_ratio=" << settings.torsionDampingRatio
                                << " maximum_frequency_hz=" << settings.maximumFrequencyHz << Qt::endl;
            return std::abs(settings.translationDampingRatio - 0.005) <= 1.0e-14 &&
                           std::abs(settings.torsionDampingRatio - 0.038) <= 1.0e-14 &&
                           std::abs(settings.maximumFrequencyHz - 3.0) <= 1.0e-14
                       ? 0
                       : 3;
        }
        return 4;
    }
    if (!arguments.contains(QStringLiteral("--verify-structural-damping")))
        return std::nullopt;

    constexpr int dofCount = 4;
    constexpr double translationRatio = 0.005;
    constexpr double torsionRatio = 0.038;
    const Eigen::Vector4d modalMasses(2.0, 3.0, 5.0, 7.0);
    const Eigen::Vector4d frequenciesHz(1.0, 1.5, 2.0, 2.5);
    Eigen::Vector4d stiffnessDiagonal;
    for (int dof = 0; dof < dofCount; ++dof)
    {
        const double omega = 2.0 * std::numbers::pi * frequenciesHz[dof];
        stiffnessDiagonal[dof] = modalMasses[dof] * omega * omega;
    }
    const Eigen::SparseMatrix<double> mass(modalMasses.asDiagonal());
    const Eigen::SparseMatrix<double> stiffness(stiffnessDiagonal.asDiagonal());
    SolverNameSpace::StructuralDampingModel damping;
    damping.settings.enabled = true;
    damping.settings.translationDampingRatio = translationRatio;
    damping.settings.torsionDampingRatio = torsionRatio;
    damping.settings.maximumFrequencyHz = 3.0;
    QString error;
    Eigen::SparseMatrix<double> identity(dofCount, dofCount);
    identity.setIdentity();
    if (!damping.Calculate(stiffness, mass, {false, false, true, true}, identity, error))
    {
        QTextStream(stderr) << "structural damping calculation failed: " << error << Qt::endl;
        return 1;
    }
    const Eigen::MatrixXd actual = damping.Factor() * damping.Factor().transpose();
    Eigen::Vector4d expectedDiagonal;
    for (int dof = 0; dof < dofCount; ++dof)
    {
        const double ratio = dof < 2 ? translationRatio : torsionRatio;
        expectedDiagonal[dof] = 2.0 * ratio * 2.0 * std::numbers::pi * frequenciesHz[dof] * modalMasses[dof];
    }
    const double diagonalError = (actual.diagonal() - expectedDiagonal).cwiseAbs().maxCoeff();
    const Eigen::MatrixXd diagonalMatrix = actual.diagonal().asDiagonal();
    const double offDiagonalError = (actual - diagonalMatrix).cwiseAbs().maxCoeff();
    QTextStream(stdout) << "structural_damping selected_modes=" << damping.report.selectedModeCount
                        << " maximum_backcheck_error=" << damping.report.maximumBackCheckError
                        << " diagonal_error=" << diagonalError << " off_diagonal_error=" << offDiagonalError
                        << Qt::endl;
    return diagonalError <= 1.0e-10 && offDiagonalError <= 1.0e-10 &&
                   damping.report.maximumBackCheckError <= 1.0e-10
               ? 0
               : 2;
}

std::optional<int> verifyGallopingStability(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-galloping-stability"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    StructureData structure;
    Hdf5ModelIO hdf5;
    const QString modelPath = arguments.at(index + 1);
    if (!hdf5.ImportHdf5(modelPath, &structure))
    {
        QTextStream(stderr) << "cannot import galloping stability model=" << modelPath << Qt::endl;
        return 2;
    }

    const auto& cases = structure.m_AeroManager.getCaseModels();
    if (cases.empty())
    {
        QTextStream(stderr) << "model has no loaded aerodynamic cases" << Qt::endl;
        return 3;
    }

    int intervalCount = 0;
    QTextStream stream(stdout);
    stream << "galloping stability model=" << modelPath << Qt::endl;
    for (const auto& [key, models] : cases)
    {
        const auto intervals = GallopingStabilityAnalyzer::FindNegativeDampingIntervals(models);
        stream << "case bundle=" << key.bundleCount << " wind=" << key.windSpeed << " ice=" << key.iceThickness
               << " profiles=" << models.size() << " negative_intervals=" << intervals.size() << Qt::endl;
        for (const GallopingInstabilityInterval& interval : intervals)
        {
            stream << "  profile=" << interval.profileId << " alpha_start=" << interval.startAngleDegrees
                   << " alpha_end=" << interval.endAngleDegrees << " min_H=" << interval.minimumDenHartog << Qt::endl;
        }
        intervalCount += static_cast<int>(intervals.size());
    }
    return intervalCount > 0 ? 0 : 4;
}

class LinearOscillatorModel final : public SolverNameSpace::IAnalysisModel
{
public:
    LinearOscillatorModel(double mass, double stiffness)
        : mass_(mass)
        , stiffness_(stiffness)
    {
        displacement_[0] = 1.0;
        velocity_[0] = 0.0;
        acceleration_[0] = -stiffness_ / mass_;
        BackupStepState();
    }

    int GetFreeDofs() const override
    {
        return 1;
    }
    int GetFixedDofs() const override
    {
        return 0;
    }

    void ApplyIncrement(const SolverNameSpace::Vec& increment) override
    {
        displacement_ += increment;
    }

    void BeginDynamicStep(double, double, double) override
    {
    }

    void ApplyDynamicCorrection(const SolverNameSpace::Vec& increment, double a0, double a1) override
    {
        displacement_ += increment;
        velocity_ += a1 * increment;
        acceleration_ += a0 * increment;
    }

    void RollbackDynamicStep() override
    {
        displacement_ = savedDisplacement_;
        velocity_ = savedVelocity_;
        acceleration_ = savedAcceleration_;
    }

    void SetTrialKinematics(const SolverNameSpace::Vec& velocity, const SolverNameSpace::Vec& acceleration) override
    {
        velocity_ = velocity;
        acceleration_ = acceleration;
    }

    void GetState(SolverNameSpace::Vec& displacement, SolverNameSpace::Vec& velocity,
                  SolverNameSpace::Vec& acceleration) const override
    {
        displacement = displacement_;
        velocity = velocity_;
        acceleration = acceleration_;
    }

    void Assemble_Matrix(SolverNameSpace::SpMat& stiffness, bool) override
    {
        stiffness.resize(1, 1);
        stiffness.setZero();
        stiffness.insert(0, 0) = stiffness_;
        stiffness.makeCompressed();
    }

    void AssembleDynamicSystem(SolverNameSpace::SpMat& mass, SolverNameSpace::SpMat& gyroscopic,
                               SolverNameSpace::SpMat& centrifugal) override
    {
        mass.resize(1, 1);
        mass.setZero();
        mass.insert(0, 0) = mass_;
        mass.makeCompressed();
        gyroscopic.resize(1, 1);
        gyroscopic.setZero();
        centrifugal.resize(1, 1);
        centrifugal.setZero();
    }

    void ComputeExternalForce(double, double, SolverNameSpace::Vec& constrained, SolverNameSpace::Vec& free) override
    {
        constrained.resize(0);
        free = SolverNameSpace::Vec::Zero(1);
    }

    void Assemble_Constraint(SolverNameSpace::Vec& constrained, double, double) override
    {
        constrained.resize(0);
    }

    void ComputeResidual(const SolverNameSpace::Vec& external, SolverNameSpace::Vec& residual) override
    {
        residual = external - stiffness_ * displacement_ - mass_ * acceleration_;
    }

    void CalculateReactions(_OUT SolverNameSpace::Vec& reactions) override
    {
        reactions.resize(0);
    }

    void OnStepCompleted(double) override
    {
        ++completedSteps_;
    }
    void CommitState() override
    {
    }

    void BackupStepState() override
    {
        savedDisplacement_ = displacement_;
        savedVelocity_ = velocity_;
        savedAcceleration_ = acceleration_;
    }

    bool PushStateCheckpoint() override
    {
        checkpointDisplacement_ = displacement_;
        checkpointVelocity_ = velocity_;
        checkpointAcceleration_ = acceleration_;
        hasCheckpoint_ = true;
        return true;
    }

    bool RestoreStateCheckpoint() override
    {
        if (!hasCheckpoint_)
            return false;
        displacement_ = checkpointDisplacement_;
        velocity_ = checkpointVelocity_;
        acceleration_ = checkpointAcceleration_;
        return true;
    }

    void DiscardStateCheckpoint() override
    {
        hasCheckpoint_ = false;
    }

    void GetStepIncrement(SolverNameSpace::Vec& increment) const override
    {
        increment = displacement_ - savedDisplacement_;
    }

    int CompletedSteps() const
    {
        return completedSteps_;
    }

private:
    double mass_ = 1.0;
    double stiffness_ = 1.0;
    SolverNameSpace::Vec displacement_ = SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec velocity_ = SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec acceleration_ = SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec savedDisplacement_ = SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec savedVelocity_ = SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec savedAcceleration_ = SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec checkpointDisplacement_ = SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec checkpointVelocity_ = SolverNameSpace::Vec::Zero(1);
    SolverNameSpace::Vec checkpointAcceleration_ = SolverNameSpace::Vec::Zero(1);
    bool hasCheckpoint_ = false;
    int completedSteps_ = 0;
};

class RigidOffsetOscillatorModel final : public SolverNameSpace::IAnalysisModel
{
public:
    RigidOffsetOscillatorModel(double pointMass, double rotaryInertia, double stiffness, double offset,
                               double initialAngle)
        : pointMass_(pointMass)
        , rotaryInertia_(rotaryInertia)
        , stiffness_(stiffness)
        , offset_(offset)
    {
        state_ << initialAngle, offset_ * std::cos(initialAngle), offset_ * std::sin(initialAngle);
        velocity_.setZero();
        const double angularAcceleration =
            -stiffness_ * initialAngle / (rotaryInertia_ + pointMass_ * offset_ * offset_);
        acceleration_ << angularAcceleration, -offset_ * std::sin(initialAngle) * angularAcceleration,
            offset_ * std::cos(initialAngle) * angularAcceleration;
        savedState_ = state_;
        savedVelocity_ = velocity_;
        savedAcceleration_ = acceleration_;
    }

    int GetFreeDofs() const override
    {
        return 3;
    }
    int GetFixedDofs() const override
    {
        return 0;
    }
    void ApplyIncrement(const SolverNameSpace::Vec& increment) override
    {
        state_ += increment;
    }
    void BeginDynamicStep(double dt, double beta, double gamma) override
    {
        savedState_ = state_;
        savedVelocity_ = velocity_;
        savedAcceleration_ = acceleration_;
        state_ = savedState_ + dt * savedVelocity_ + dt * dt * (0.5 - beta) * savedAcceleration_;
        velocity_ = savedVelocity_ + dt * (1.0 - gamma) * savedAcceleration_;
        acceleration_.setZero();
    }
    void ApplyDynamicCorrection(const SolverNameSpace::Vec& correction, double accelerationDerivative,
                                double velocityDerivative) override
    {
        state_ += correction;
        velocity_ += velocityDerivative * correction;
        acceleration_ += accelerationDerivative * correction;
    }
    void RollbackDynamicStep() override
    {
        state_ = savedState_;
        velocity_ = savedVelocity_;
        acceleration_ = savedAcceleration_;
    }
    void SetTrialKinematics(const SolverNameSpace::Vec& velocity, const SolverNameSpace::Vec& acceleration) override
    {
        velocity_ = velocity;
        acceleration_ = acceleration;
    }
    void GetState(SolverNameSpace::Vec& displacement, SolverNameSpace::Vec& velocity,
                  SolverNameSpace::Vec& acceleration) const override
    {
        displacement = state_;
        velocity = velocity_;
        acceleration = acceleration_;
    }
    void Assemble_Matrix(SolverNameSpace::SpMat& tangent, bool) override
    {
        tangent.resize(3, 3);
        tangent.setZero();
        tangent.insert(0, 0) = stiffness_;
        tangent.makeCompressed();
    }
    void AssembleDynamicSystem(SolverNameSpace::SpMat& mass, SolverNameSpace::SpMat& velocityTangent,
                               SolverNameSpace::SpMat& configurationTangent) override
    {
        mass.resize(3, 3);
        mass.setZero();
        mass.insert(0, 0) = rotaryInertia_;
        mass.insert(1, 1) = pointMass_;
        mass.insert(2, 2) = pointMass_;
        mass.makeCompressed();
        velocityTangent.resize(3, 3);
        velocityTangent.setZero();
        configurationTangent.resize(3, 3);
        configurationTangent.setZero();
    }
    bool AssembleNonlinearMPC(SolverNameSpace::NonlinearMPCData& constraints) override
    {
        ++mpcAssemblyCount_;
        const double angle = state_[0];
        constraints.Clear();
        constraints.value.resize(2);
        constraints.value << state_[1] - offset_ * std::cos(angle), state_[2] - offset_ * std::sin(angle);
        constraints.jacobian = Eigen::MatrixXd::Zero(2, 3);
        constraints.jacobian(0, 0) = offset_ * std::sin(angle);
        constraints.jacobian(0, 1) = 1.0;
        constraints.jacobian(1, 0) = -offset_ * std::cos(angle);
        constraints.jacobian(1, 2) = 1.0;
        constraints.hessians.resize(2);
        for (auto& hessian : constraints.hessians)
            hessian.resize(3, 3);
        constraints.hessians[0].insert(0, 0) = offset_ * std::cos(angle);
        constraints.hessians[1].insert(0, 0) = offset_ * std::sin(angle);
        constraints.slaveDofs = {1, 2};
        return constraints.IsValid(3);
    }
    void SetNonlinearMPCMultipliers(const SolverNameSpace::Vec& multipliers) override
    {
        multipliers_ = multipliers;
    }
    void ComputeExternalForce(double, double, SolverNameSpace::Vec& constrained, SolverNameSpace::Vec& free) override
    {
        constrained.resize(0);
        free = SolverNameSpace::Vec::Zero(3);
    }
    void Assemble_Constraint(SolverNameSpace::Vec& constrained, double, double) override
    {
        constrained.resize(0);
    }
    void ComputeResidual(const SolverNameSpace::Vec& external, SolverNameSpace::Vec& residual) override
    {
        SolverNameSpace::Vec internal = SolverNameSpace::Vec::Zero(3);
        internal[0] = stiffness_ * state_[0];
        SolverNameSpace::Vec inertia(3);
        inertia << rotaryInertia_ * acceleration_[0], pointMass_ * acceleration_[1], pointMass_ * acceleration_[2];
        residual = external - internal - inertia;
    }
    void CalculateReactions(_OUT SolverNameSpace::Vec& reactions) override
    {
        reactions.resize(0);
    }
    void OnStepCompleted(double) override
    {
        const double angle = state_[0];
        Eigen::Matrix<double, 2, 3> jacobian;
        jacobian << offset_ * std::sin(angle), 1.0, 0.0, -offset_ * std::cos(angle), 0.0, 1.0;
        Eigen::Vector2d constraint;
        constraint << state_[1] - offset_ * std::cos(angle), state_[2] - offset_ * std::sin(angle);
        const Eigen::Vector2d velocityResidual = jacobian * velocity_;
        Eigen::Vector2d accelerationResidual = jacobian * acceleration_;
        accelerationResidual[0] += offset_ * std::cos(angle) * velocity_[0] * velocity_[0];
        accelerationResidual[1] += offset_ * std::sin(angle) * velocity_[0] * velocity_[0];
        maxConstraintError_ = std::max(maxConstraintError_, constraint.norm());
        maxVelocityConstraintError_ = std::max(maxVelocityConstraintError_, velocityResidual.norm());
        maxAccelerationConstraintError_ = std::max(maxAccelerationConstraintError_, accelerationResidual.norm());
    }
    void CommitState() override
    {
    }
    void BackupStepState() override
    {
        savedState_ = state_;
        savedVelocity_ = velocity_;
        savedAcceleration_ = acceleration_;
    }
    void GetStepIncrement(SolverNameSpace::Vec& increment) const override
    {
        increment = state_ - savedState_;
    }

    int MpcAssemblyCount() const
    {
        return mpcAssemblyCount_;
    }
    int MultiplierCount() const
    {
        return static_cast<int>(multipliers_.size());
    }
    double MaxConstraintError() const
    {
        return maxConstraintError_;
    }
    double MaxVelocityConstraintError() const
    {
        return maxVelocityConstraintError_;
    }
    double MaxAccelerationConstraintError() const
    {
        return maxAccelerationConstraintError_;
    }

private:
    double pointMass_ = 0.0;
    double rotaryInertia_ = 0.0;
    double stiffness_ = 0.0;
    double offset_ = 0.0;
    Eigen::Vector3d state_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d savedState_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d savedVelocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d savedAcceleration_ = Eigen::Vector3d::Zero();
    SolverNameSpace::Vec multipliers_;
    int mpcAssemblyCount_ = 0;
    double maxConstraintError_ = 0.0;
    double maxVelocityConstraintError_ = 0.0;
    double maxAccelerationConstraintError_ = 0.0;
};

class FixedLengthPendulumModel final : public SolverNameSpace::IAnalysisModel
{
public:
    FixedLengthPendulumModel(double mass, double length, double gravity, double initialAngle)
        : mass_(mass)
        , length_(length)
        , gravity_(gravity)
    {
        state_ << length_ * std::sin(initialAngle), -length_ * std::cos(initialAngle);
        velocity_.setZero();
        const double angularAcceleration = -gravity_ / length_ * std::sin(initialAngle);
        acceleration_ << length_ * std::cos(initialAngle) * angularAcceleration,
            length_ * std::sin(initialAngle) * angularAcceleration;
        savedState_ = state_;
        savedVelocity_ = velocity_;
        savedAcceleration_ = acceleration_;
        initialEnergy_ = Energy();
    }

    int GetFreeDofs() const override
    {
        return 2;
    }
    int GetFixedDofs() const override
    {
        return 0;
    }
    void ApplyIncrement(const SolverNameSpace::Vec& increment) override
    {
        state_ += increment;
    }
    void BeginDynamicStep(double dt, double beta, double gamma) override
    {
        savedState_ = state_;
        savedVelocity_ = velocity_;
        savedAcceleration_ = acceleration_;
        state_ = savedState_ + dt * savedVelocity_ + dt * dt * (0.5 - beta) * savedAcceleration_;
        velocity_ = savedVelocity_ + dt * (1.0 - gamma) * savedAcceleration_;
        acceleration_.setZero();
    }
    void ApplyDynamicCorrection(const SolverNameSpace::Vec& correction, double accelerationDerivative,
                                double velocityDerivative) override
    {
        state_ += correction;
        velocity_ += velocityDerivative * correction;
        acceleration_ += accelerationDerivative * correction;
    }
    void RollbackDynamicStep() override
    {
        state_ = savedState_;
        velocity_ = savedVelocity_;
        acceleration_ = savedAcceleration_;
    }
    void SetTrialKinematics(const SolverNameSpace::Vec& velocity, const SolverNameSpace::Vec& acceleration) override
    {
        velocity_ = velocity;
        acceleration_ = acceleration;
    }
    void GetState(SolverNameSpace::Vec& displacement, SolverNameSpace::Vec& velocity,
                  SolverNameSpace::Vec& acceleration) const override
    {
        displacement = state_;
        velocity = velocity_;
        acceleration = acceleration_;
    }
    void Assemble_Matrix(SolverNameSpace::SpMat& tangent, bool) override
    {
        tangent.resize(2, 2);
        tangent.setZero();
    }
    void AssembleDynamicSystem(SolverNameSpace::SpMat& mass, SolverNameSpace::SpMat& velocityTangent,
                               SolverNameSpace::SpMat& configurationTangent) override
    {
        mass.resize(2, 2);
        mass.setZero();
        mass.insert(0, 0) = mass_;
        mass.insert(1, 1) = mass_;
        mass.makeCompressed();
        velocityTangent.resize(2, 2);
        velocityTangent.setZero();
        configurationTangent.resize(2, 2);
        configurationTangent.setZero();
    }
    bool AssembleNonlinearMPC(SolverNameSpace::NonlinearMPCData& constraints) override
    {
        constraints.Clear();
        constraints.value.resize(1);
        constraints.value[0] = 0.5 * (state_.squaredNorm() - length_ * length_);
        constraints.jacobian.resize(1, 2);
        constraints.jacobian = state_.transpose();
        constraints.hessians.resize(1);
        constraints.hessians[0].resize(2, 2);
        constraints.hessians[0].setIdentity();
        constraints.slaveDofs = {1};
        return constraints.IsValid(2);
    }
    void SetNonlinearMPCMultipliers(const SolverNameSpace::Vec& multipliers) override
    {
        multipliers_ = multipliers;
    }
    void ComputeExternalForce(double, double, SolverNameSpace::Vec& constrained, SolverNameSpace::Vec& free) override
    {
        constrained.resize(0);
        free = SolverNameSpace::Vec::Zero(2);
    }
    void Assemble_Constraint(SolverNameSpace::Vec& constrained, double, double) override
    {
        constrained.resize(0);
    }
    void ComputeResidual(const SolverNameSpace::Vec& external, SolverNameSpace::Vec& residual) override
    {
        Eigen::Vector2d force;
        force << mass_ * acceleration_[0], mass_ * acceleration_[1] + mass_ * gravity_;
        residual = external - force;
    }
    void CalculateReactions(_OUT SolverNameSpace::Vec& reactions) override
    {
        reactions.resize(0);
    }
    void OnStepCompleted(double) override
    {
        maxConstraintError_ = std::max(maxConstraintError_, std::abs(state_.squaredNorm() - length_ * length_));
        maxVelocityConstraintError_ = std::max(maxVelocityConstraintError_, std::abs(state_.dot(velocity_)));
        const double accelerationGap = state_.dot(acceleration_) + velocity_.squaredNorm();
        maxAccelerationConstraintError_ = std::max(maxAccelerationConstraintError_, std::abs(accelerationGap));
        maxRelativeEnergyError_ = std::max(maxRelativeEnergyError_, std::abs(Energy() - initialEnergy_) /
                                                                        std::max(1.0, std::abs(initialEnergy_)));
    }
    void CommitState() override
    {
    }
    void BackupStepState() override
    {
        savedState_ = state_;
        savedVelocity_ = velocity_;
        savedAcceleration_ = acceleration_;
    }
    void GetStepIncrement(SolverNameSpace::Vec& increment) const override
    {
        increment = state_ - savedState_;
    }

    int MultiplierCount() const
    {
        return static_cast<int>(multipliers_.size());
    }
    double MaxConstraintError() const
    {
        return maxConstraintError_;
    }
    double MaxVelocityConstraintError() const
    {
        return maxVelocityConstraintError_;
    }
    double MaxAccelerationConstraintError() const
    {
        return maxAccelerationConstraintError_;
    }
    double MaxRelativeEnergyError() const
    {
        return maxRelativeEnergyError_;
    }

private:
    double Energy() const
    {
        return 0.5 * mass_ * velocity_.squaredNorm() + mass_ * gravity_ * (state_[1] + length_);
    }

    double mass_ = 0.0;
    double length_ = 0.0;
    double gravity_ = 0.0;
    Eigen::Vector2d state_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d acceleration_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d savedState_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d savedVelocity_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d savedAcceleration_ = Eigen::Vector2d::Zero();
    SolverNameSpace::Vec multipliers_;
    double initialEnergy_ = 0.0;
    double maxConstraintError_ = 0.0;
    double maxVelocityConstraintError_ = 0.0;
    double maxAccelerationConstraintError_ = 0.0;
    double maxRelativeEnergyError_ = 0.0;
};

std::optional<int> verifyTimeStepIntegrators(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-time-step-integrators")))
        return std::nullopt;

    constexpr double mass = 1.0;
    constexpr double stiffness = 25.0;
    const double period = 2.0 * std::numbers::pi / std::sqrt(stiffness / mass);

    SolverNameSpace::SolverNewmark::Params newmarkParameters;
    newmarkParameters.dt = period / 5.0;
    newmarkParameters.maxIter = 12;
    newmarkParameters.tol = 1.0e-11;
    newmarkParameters.timeStepMode = SolverNameSpace::TimeStepMode::Adaptive;
    newmarkParameters.adaptiveTimeStep.minimumTimeStep = period / 10000.0;
    newmarkParameters.adaptiveTimeStep.maximumTimeStep = period / 4.0;
    newmarkParameters.adaptiveTimeStep.relativeTolerance = 2.0e-4;
    newmarkParameters.adaptiveTimeStep.absoluteTolerance = 1.0e-7;
    LinearOscillatorModel newmarkModel(mass, stiffness);
    SolverNameSpace::SolverNewmark newmarkSolver(newmarkParameters);
    const bool newmarkSolved = newmarkSolver.Solve(newmarkModel, period);
    SolverNameSpace::Vec newmarkDisplacement;
    SolverNameSpace::Vec newmarkVelocity;
    SolverNameSpace::Vec newmarkAcceleration;
    newmarkModel.GetState(newmarkDisplacement, newmarkVelocity, newmarkAcceleration);
    const double newmarkError = std::abs(newmarkDisplacement[0] - 1.0) +
                                std::abs(newmarkVelocity[0]) / std::sqrt(stiffness / mass);

    SolverNameSpace::SolverRungeKutta4::Params rkParameters;
    rkParameters.timeStep = period / 5.0;
    rkParameters.timeStepMode = SolverNameSpace::TimeStepMode::Adaptive;
    rkParameters.adaptiveTimeStep.minimumTimeStep = period / 10000.0;
    rkParameters.adaptiveTimeStep.maximumTimeStep = period / 4.0;
    rkParameters.adaptiveTimeStep.relativeTolerance = 2.0e-5;
    rkParameters.adaptiveTimeStep.absoluteTolerance = 1.0e-8;
    LinearOscillatorModel rkModel(mass, stiffness);
    SolverNameSpace::SolverRungeKutta4 rkSolver(rkParameters);
    const bool rkSolved = rkSolver.Solve(rkModel, period);
    SolverNameSpace::Vec rkDisplacement;
    SolverNameSpace::Vec rkVelocity;
    SolverNameSpace::Vec rkAcceleration;
    rkModel.GetState(rkDisplacement, rkVelocity, rkAcceleration);
    const double rkError =
        std::abs(rkDisplacement[0] - 1.0) + std::abs(rkVelocity[0]) / std::sqrt(stiffness / mass);

    QTextStream(stdout) << "time_step_integrators newmark_solved=" << newmarkSolved
                        << " newmark_error=" << newmarkError << " rk4_solved=" << rkSolved
                        << " rk4_error=" << rkError << Qt::endl;
    return newmarkSolved && rkSolved && newmarkError < 2.0e-2 && rkError < 5.0e-4 ? 0 : 1;
}

std::optional<int> verifyDynamicMpc(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-dynamic-mpc")))
        return std::nullopt;

    constexpr double pointMass = 2.0;
    constexpr double rotaryInertia = 0.7;
    constexpr double stiffness = 9.0;
    constexpr double offset = 1.3;
    constexpr double initialAngle = 0.35;
    const double effectiveInertia = rotaryInertia + pointMass * offset * offset;
    const double angularFrequency = std::sqrt(stiffness / effectiveInertia);
    const double period = 2.0 * std::acos(-1.0) / angularFrequency;

    RigidOffsetOscillatorModel model(pointMass, rotaryInertia, stiffness, offset, initialAngle);
    SolverNameSpace::SolverNewmark::Params parameters;
    parameters.dt = period / 400.0;
    parameters.beta = 0.25;
    parameters.gamma = 0.5;
    parameters.maxIter = 20;
    parameters.tol = 1.0e-10;
    parameters.constraintTolerance = 1.0e-11;
    SolverNameSpace::SolverNewmark solver(parameters);
    const bool solved = solver.Solve(model, period);

    SolverNameSpace::Vec displacement;
    SolverNameSpace::Vec velocity;
    SolverNameSpace::Vec acceleration;
    model.GetState(displacement, velocity, acceleration);
    const double angleError = std::abs(displacement[0] - initialAngle) / initialAngle;
    const double angularVelocityError = std::abs(velocity[0]) / (initialAngle * angularFrequency);
    const bool rigidOffsetPassed = solved && model.MpcAssemblyCount() > 0 && model.MultiplierCount() == 2 &&
                                   model.MaxConstraintError() < 1.0e-10 && angleError < 5.0e-4 &&
                                   angularVelocityError < 5.0e-4;

    QTextStream(stdout) << "dynamic_mpc solved=" << solved << " assemblies=" << model.MpcAssemblyCount()
                        << " multiplier_count=" << model.MultiplierCount()
                        << " max_position_constraint=" << model.MaxConstraintError()
                        << " max_velocity_constraint=" << model.MaxVelocityConstraintError()
                        << " max_acceleration_constraint=" << model.MaxAccelerationConstraintError()
                        << " angle_error=" << angleError << " angular_velocity_error=" << angularVelocityError
                        << Qt::endl;

    constexpr double pendulumMass = 1.7;
    constexpr double pendulumLength = 1.25;
    constexpr double gravity = 9.81;
    constexpr double pendulumAngle = 0.8;
    const double ellipticParameter = std::sin(0.5 * pendulumAngle);
    const double pendulumPeriod = 4.0 * std::sqrt(pendulumLength / gravity) * std::comp_ellint_1(ellipticParameter);
    FixedLengthPendulumModel pendulum(pendulumMass, pendulumLength, gravity, pendulumAngle);
    SolverNameSpace::SolverNewmark::Params pendulumParameters;
    pendulumParameters.dt = pendulumPeriod / 800.0;
    pendulumParameters.maxIter = 20;
    pendulumParameters.tol = 1.0e-9;
    pendulumParameters.constraintTolerance = 1.0e-11;
    SolverNameSpace::SolverNewmark pendulumSolver(pendulumParameters);
    const bool pendulumSolved = pendulumSolver.Solve(pendulum, pendulumPeriod);
    SolverNameSpace::Vec pendulumPosition;
    SolverNameSpace::Vec pendulumVelocity;
    SolverNameSpace::Vec pendulumAcceleration;
    pendulum.GetState(pendulumPosition, pendulumVelocity, pendulumAcceleration);
    Eigen::Vector2d exactPosition;
    exactPosition << pendulumLength * std::sin(pendulumAngle), -pendulumLength * std::cos(pendulumAngle);
    const double pendulumPositionError = (pendulumPosition - exactPosition).norm() / pendulumLength;
    const double pendulumVelocityError = pendulumVelocity.norm() / std::sqrt(gravity * pendulumLength);
    const bool pendulumPassed = pendulumSolved && pendulum.MultiplierCount() == 1 &&
                                pendulum.MaxConstraintError() < 1.0e-10 && pendulumPositionError < 2.0e-4 &&
                                pendulumVelocityError < 2.0e-4 && pendulum.MaxRelativeEnergyError() < 2.0e-4;
    QTextStream(stdout) << "dynamic_mpc_pendulum solved=" << pendulumSolved
                        << " multiplier_count=" << pendulum.MultiplierCount()
                        << " max_position_constraint=" << pendulum.MaxConstraintError()
                        << " max_velocity_constraint=" << pendulum.MaxVelocityConstraintError()
                        << " max_acceleration_constraint=" << pendulum.MaxAccelerationConstraintError()
                        << " max_relative_energy_error=" << pendulum.MaxRelativeEnergyError()
                        << " period_position_error=" << pendulumPositionError
                        << " period_velocity_error=" << pendulumVelocityError << Qt::endl;
    return rigidOffsetPassed && pendulumPassed ? 0 : 1;
}

std::optional<int> verifyDynamicMpcModel(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-dynamic-mpc-model"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    auto structure = std::make_shared<StructureData>();
    Input_Model importer;
    if (!importer.InputData(arguments.at(index + 1), structure))
    {
        QTextStream(stderr) << "dynamic MPC model import failed: " << importer.LastError() << Qt::endl;
        return 2;
    }
    structure->m_OutputControl.m_StreamResult = false;
    AnalysisRunner runner;
    runner.SetStructure(structure);
    runner.SetRuntimeCallbacks(
        [](double, const QString&)
        {
        },
        []()
        {
            return false;
        });
    if (!runner.RunAll())
        return 3;

    const auto master = structure->FindNode(2);
    const auto slave = structure->FindNode(3);
    const auto tip = structure->FindNode(4);
    if (!master || !slave || !tip)
        return 4;
    const Eigen::Vector3d referenceOffset(0.0, 0.2, 0.0);
    const Eigen::Vector3d masterPosition(master->m_X + master->m_Displacement[0],
                                         master->m_Y + master->m_Displacement[1],
                                         master->m_Z + master->m_Displacement[2]);
    const Eigen::Vector3d slavePosition(slave->m_X + slave->m_Displacement[0], slave->m_Y + slave->m_Displacement[1],
                                        slave->m_Z + slave->m_Displacement[2]);
    const double constraintGap = (slavePosition - masterPosition - master->m_Rg * referenceOffset).norm();
    const double tipMotion = std::hypot(tip->m_Displacement[0], tip->m_Displacement[1]);
    const bool passed = constraintGap < 1.0e-8 && std::isfinite(tipMotion) && tipMotion > 1.0e-8;
    QTextStream(stdout) << "dynamic_mpc_model constraint_gap=" << constraintGap << " tip_motion=" << tipMotion
                        << " nodes=" << structure->m_Nodes.size() << " elements=" << structure->m_Elements.size()
                        << " mpcs=" << structure->m_MPCConstraints.size() << Qt::endl;
    return passed ? 0 : 5;
}

std::optional<int> verifyAdaptiveTssbn(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-adaptive-tssbn")))
        return std::nullopt;

    AnalysisStep factoryStep;
    factoryStep.m_Type = EnumKeyword::StepType::DYNAMIC;
    factoryStep.m_DynamicSolverType = SolverNameSpace::SolverType::AdaptiveTSSBN;
    factoryStep.m_StepSize = 0.1;
    factoryStep.m_Tolerance = 1.0e-5;
    factoryStep.m_MaxIterations = 20;
    factoryStep.m_AdaptiveTssbn.spectralRadiusInfinity = 0.77;
    factoryStep.m_AdaptiveTssbn.minimumTimeStep = 2.0e-6;
    factoryStep.m_AdaptiveTssbn.maximumTimeStep = 0.42;
    factoryStep.m_AdaptiveTssbn.relativeTolerance = 8.0e-4;
    factoryStep.m_AdaptiveTssbn.absoluteTolerance = 4.0e-7;
    factoryStep.m_AdaptiveTssbn.shrinkFactor = 0.74;
    factoryStep.m_EnableGalloping = true;
    factoryStep.m_GallopingAerodynamicTangentMode =
        SolverNameSpace::AerodynamicTangentMode::OncePerTimeStepOrStage;
    factoryStep.m_AdaptiveTssbn.targetNewtonIterations = 9;
    factoryStep.m_AdaptiveTssbn.maximumTotalNewtonIterations = 31;
    factoryStep.m_AdaptiveTssbn.derivativeGain = 0.12;
    factoryStep.m_AdaptiveTssbn.minimumDerivativeFactor = 0.43;
    factoryStep.m_AdaptiveTssbn.maximumDerivativeFactor = 1.61;
    factoryStep.m_AdaptiveTssbn.maximumRejectedAttempts = 17;
    const auto factorySolver = SolverNameSpace::SolverFactory::Create_StepForSlover(factoryStep);
    const auto* adaptiveFactorySolver = dynamic_cast<const SolverNameSpace::SolverAdaptiveTSSBN*>(factorySolver.get());
    AnalysisStep newmarkFactoryStep;
    newmarkFactoryStep.m_Type = EnumKeyword::StepType::DYNAMIC;
    newmarkFactoryStep.m_DynamicSolverType = SolverNameSpace::SolverType::Newmark;
    newmarkFactoryStep.m_StepSize = 0.01;
    newmarkFactoryStep.m_EnableGalloping = true;
    newmarkFactoryStep.m_GallopingAerodynamicTangentMode = SolverNameSpace::AerodynamicTangentMode::Disabled;
    const auto newmarkFactoryBase = SolverNameSpace::SolverFactory::Create_StepForSlover(newmarkFactoryStep);
    const auto* newmarkFactorySolver =
        dynamic_cast<const SolverNameSpace::SolverNewmark*>(newmarkFactoryBase.get());
    const bool factoryConnected =
        adaptiveFactorySolver && adaptiveFactorySolver->GetType() == SolverNameSpace::SolverType::AdaptiveTSSBN &&
        std::abs(adaptiveFactorySolver->GetParams().spectralRadiusInfinity - 0.77) < 1.0e-12 &&
        std::abs(adaptiveFactorySolver->GetParams().minimumTimeStep - 2.0e-6) < 1.0e-15 &&
        std::abs(adaptiveFactorySolver->GetParams().maximumTimeStep - 0.42) < 1.0e-12 &&
        std::abs(adaptiveFactorySolver->GetParams().relativeTolerance - 8.0e-4) < 1.0e-15 &&
        std::abs(adaptiveFactorySolver->GetParams().absoluteTolerance - 4.0e-7) < 1.0e-15 &&
        std::abs(adaptiveFactorySolver->GetParams().shrinkFactor - 0.74) < 1.0e-12 &&
        adaptiveFactorySolver->GetParams().aerodynamicTangentMode ==
            SolverNameSpace::AerodynamicTangentMode::OncePerTimeStepOrStage &&
        adaptiveFactorySolver->GetParams().targetNewtonIterations == 9 &&
        adaptiveFactorySolver->GetParams().maximumTotalNewtonIterations == 31 &&
        std::abs(adaptiveFactorySolver->GetParams().derivativeGain - 0.12) < 1.0e-12 &&
        std::abs(adaptiveFactorySolver->GetParams().minimumDerivativeFactor - 0.43) < 1.0e-12 &&
        std::abs(adaptiveFactorySolver->GetParams().maximumDerivativeFactor - 1.61) < 1.0e-12 &&
        adaptiveFactorySolver->GetParams().maximumRejectedAttempts == 17 && newmarkFactorySolver &&
        newmarkFactorySolver->GetParams().aerodynamicTangentMode ==
            SolverNameSpace::AerodynamicTangentMode::Disabled;

    constexpr double mass = 2.0;
    constexpr double stiffness = 8.0;
    constexpr double angularFrequency = 2.0;
    const double duration = 2.0 * std::acos(-1.0);
    LinearOscillatorModel model(mass, stiffness);

    SolverNameSpace::SolverAdaptiveTSSBN::Params parameters;
    parameters.initialTimeStep = 0.15;
    parameters.minimumTimeStep = 1.0e-5;
    parameters.maximumTimeStep = 0.3;
    parameters.relativeTolerance = 2.0e-6;
    parameters.absoluteTolerance = 1.0e-9;
    // 末周期附近稀疏求解的舍入误差已超过 1e-9；1e-5 相对振子力尺度仍约为 1e-6，
    // 可让时间积分误差估计器控制自适应步长，避免代数噪声主导。
    parameters.nonlinearTolerance = 1.0e-5;
    parameters.maximumNewtonIterations = 8;
    SolverNameSpace::SolverAdaptiveTSSBN solver(parameters);
    const bool solved = solver.Solve(model, duration);

    SolverNameSpace::Vec displacement;
    SolverNameSpace::Vec velocity;
    SolverNameSpace::Vec acceleration;
    model.GetState(displacement, velocity, acceleration);
    const double exactDisplacement = std::cos(angularFrequency * duration);
    const double exactVelocity = -angularFrequency * std::sin(angularFrequency * duration);
    const double exactAcceleration = -angularFrequency * angularFrequency * exactDisplacement;
    const double displacementError = std::abs(displacement[0] - exactDisplacement);
    const double velocityError = std::abs(velocity[0] - exactVelocity);
    const double accelerationError = std::abs(acceleration[0] - exactAcceleration);
    const bool passed = factoryConnected && solved && solver.GetAcceptedStepCount() == model.CompletedSteps() &&
                        solver.GetAcceptedStepCount() > 0 && displacementError < 2.0e-3 && velocityError < 2.0e-3 &&
                        accelerationError < 2.0e-3;

    QTextStream(stdout) << "adaptive_tssbn factory=" << factoryConnected << " solved=" << solved
                        << " accepted=" << solver.GetAcceptedStepCount()
                        << " rejected=" << solver.GetRejectedStepCount() << " displacement_error=" << displacementError
                        << " velocity_error=" << velocityError << " acceleration_error=" << accelerationError
                        << Qt::endl;
    return passed ? 0 : 1;
}

std::optional<int> verifyLe2012Example1(const QStringList& arguments)
{
    if (arguments.contains(QStringLiteral("--verify-le2012-example1-tssbn")))
    {
        return PaperBeamDynamicsVerification::RunExample1Adaptive(
            ApplicationPaths::verificationOutputDirectory(QStringLiteral("le2012_example1_tssbn")));
    }
    if (!arguments.contains(QStringLiteral("--verify-le2012-example1")))
        return std::nullopt;
    return PaperBeamDynamicsVerification::RunExample1(
        ApplicationPaths::verificationOutputDirectory(QStringLiteral("le2012_example1")));
}

std::optional<int> verifyLe2012Example4(const QStringList& arguments)
{
    if (arguments.contains(QStringLiteral("--verify-le2012-example4-tssbn")))
    {
        return PaperBeamDynamicsVerification::RunAdaptive(
            ApplicationPaths::verificationOutputDirectory(QStringLiteral("le2012_example4_tssbn")));
    }
    if (!arguments.contains(QStringLiteral("--verify-le2012-example4")))
        return std::nullopt;
    return PaperBeamDynamicsVerification::Run(
        ApplicationPaths::verificationOutputDirectory(QStringLiteral("le2012_example4")));
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
    const double symmetryError = (mass - mass.transpose()).cwiseAbs().maxCoeff();
    const double massError = std::abs(assembledMass - expectedMass) / expectedMass;

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
    const double expectedNodalGyroscopicMoment = -0.5 * length * density * Iy;
    const double gyroscopicError =
        std::abs(inertiaForce(5) - expectedNodalGyroscopicMoment) / std::abs(expectedNodalGyroscopicMoment);

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
        const double predictedDisplacement = displacement + dt * velocity + dt * dt * (0.5 - beta) * acceleration;
        const double predictedVelocity = velocity + dt * (1.0 - gamma) * acceleration;
        const double newAcceleration =
            -stiffness * predictedDisplacement / (effectiveMass + beta * dt * dt * stiffness);
        displacement = predictedDisplacement + beta * dt * dt * newAcceleration;
        velocity = predictedVelocity + gamma * dt * newAcceleration;
        acceleration = newAcceleration;
    }
    const double responseError = std::abs(displacement - initialDisplacement) / initialDisplacement;

    QTextStream(stdout) << "beam dynamics mass_error=" << massError << " symmetry_error=" << symmetryError
                        << " gyroscopic_error=" << gyroscopicError << " period=" << period
                        << " one_period_response_error=" << responseError << Qt::endl;
    return massError <= 1.0e-12 && symmetryError <= 1.0e-12 && gyroscopicError <= 1.0e-12 && responseError <= 2.0e-3
               ? 0
               : 1;
}

std::optional<int> verifyExactAerodynamicAngle(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-exact-aero-angle")))
        return std::nullopt;

    AerodynamicSectionState state;
    state.firstPosition = Eigen::Vector3d(0.0, 0.0, 0.0);
    state.secondPosition = Eigen::Vector3d(5.0, 0.0, 0.0);
    state.firstVelocity = state.secondVelocity = Eigen::Vector3d(0.0, 3.0, 1.0);
    state.windVelocity = Eigen::Vector3d(0.0, 0.0, 10.0);
    state.firstTwist = state.secondTwist = 0.2;
    state.firstTwistRate = state.secondTwistRate = 2.0;
    state.radius = 0.5;
    state.initialAttack = 0.4;

    auto result = AerodynamicLoadCalculator::ComputeKinematics(state);
    const double expectedSpeed = std::hypot(4.0, 9.0);
    const double expectedFlowAngle = std::atan2(4.0, 9.0);
    const double expectedAttack = 0.4 + 0.2 - expectedFlowAngle;
    const double speedError = std::abs(result.relativeSpeed - expectedSpeed);
    const double flowError = std::abs(result.flowAngle - expectedFlowAngle);
    const double attackError = std::abs(result.attackAngle - expectedAttack);

    AerodynamicLoadCalculator::ComputeLineLoad(result, 1.225, 1.0, 1.0, 2.0, 0.1);
    const bool finiteLoads = result.lineForce.allFinite() && result.lineMoment.allFinite();
    QTextStream(stdout) << "exact aero relative_speed=" << result.relativeSpeed << " flow_angle=" << result.flowAngle
                        << " attack_angle=" << result.attackAngle << " speed_error=" << speedError
                        << " flow_error=" << flowError << " attack_error=" << attackError << Qt::endl;
    return speedError <= 1.0e-12 && flowError <= 1.0e-12 && attackError <= 1.0e-12 && finiteLoads ? 0 : 1;
}

std::optional<int> verifyGallopingCaseSelection(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-galloping-case")))
        return std::nullopt;

    StructureData structure;
    AnalysisStepConfig config;
    config.id = 1;
    config.type = EnumKeyword::StepType::DYNAMIC;
    config.enableGalloping = true;
    config.gallopingIceThickness = 25;
    structure.AddAnalysisStep(config);
    const auto step = structure.m_AnalysisStep.at(1);
    Force_Wind wind;
    wind.m_velocity = 14;
    const AeroCaseKey key = step->GetGallopingAeroCase(4, wind);
    const bool supported = step->ShouldAssembleGalloping(4, wind);
    const bool unsupportedBundleRejected = !step->ShouldAssembleGalloping(2, wind);

    config.id = 2;
    config.type = EnumKeyword::StepType::STATIC;
    structure.AddAnalysisStep(config);
    const bool staticRejected = !structure.m_AnalysisStep.at(2)->ShouldAssembleGalloping(4, wind);
    AeroManager manager;
    const std::filesystem::path aerodynamicDataDirectory =
        std::filesystem::path(ApplicationPaths::aerodynamicDataDirectory().toStdWString());
    const bool loaded = manager.loadCase(aerodynamicDataDirectory, key);
    AeroManager completeCatalog;
    const bool allCasesLoaded = completeCatalog.loadAllCases(aerodynamicDataDirectory);
    bool catalogShapeValid = completeCatalog.getLoadedCaseCount() == 40;
    double maximumEndpointGap = 0.0;
    double maximumSymmetryError = 0.0;
    for (const auto& [caseKey, models] : completeCatalog.getCaseModels())
    {
        catalogShapeValid = catalogShapeValid && static_cast<int>(models.size()) == caseKey.bundleCount;
        for (const BladeModel& model : models)
        {
            catalogShapeValid =
                catalogShapeValid && model.lift.size() == 73 && model.drag.size() == 73 && model.moment.size() == 73;
            if (model.lift.size() == 73)
            {
                maximumEndpointGap = std::max(maximumEndpointGap, std::abs(model.lift.front() - model.lift.back()));
                maximumEndpointGap = std::max(maximumEndpointGap, std::abs(model.drag.front() - model.drag.back()));
                maximumEndpointGap = std::max(maximumEndpointGap, std::abs(model.moment.front() - model.moment.back()));
            }
        }
        if (models.size() == 1 || models.size() == 4)
        {
            const std::vector<int> reflectedProfile =
                models.size() == 1 ? std::vector<int>{0} : std::vector<int>{0, 3, 2, 1};
            for (int sourceProfile = 0; sourceProfile < static_cast<int>(models.size()); ++sourceProfile)
            {
                const int targetProfile = reflectedProfile[static_cast<size_t>(sourceProfile)];
                for (int sourceIndex = 1; sourceIndex < 36; ++sourceIndex)
                {
                    const int targetIndex = 72 - sourceIndex;
                    maximumSymmetryError =
                        std::max(maximumSymmetryError, std::abs(models[sourceProfile].lift[sourceIndex] +
                                                                models[targetProfile].lift[targetIndex]));
                    maximumSymmetryError =
                        std::max(maximumSymmetryError, std::abs(models[sourceProfile].drag[sourceIndex] -
                                                                models[targetProfile].drag[targetIndex]));
                    maximumSymmetryError =
                        std::max(maximumSymmetryError, std::abs(models[sourceProfile].moment[sourceIndex] +
                                                                models[targetProfile].moment[targetIndex]));
                }
            }
        }
    }
    const auto* boundCase = manager.findCaseModels(key);
    const AeroCoefficients coefficients = boundCase ? manager.getCoefficients(*boundCase, 0, 17.5) : AeroCoefficients{};
    const bool combinedLookupMatches = boundCase &&
                                       std::abs(coefficients.lift - manager.getData(key, 0, LIFT, 17.5)) <= 1.0e-12 &&
                                       std::abs(coefficients.drag - manager.getData(key, 0, DRAG, 17.5)) <= 1.0e-12 &&
                                       std::abs(coefficients.moment - manager.getData(key, 0, MOMENT, 17.5)) <= 1.0e-12;
    const double expected357 =
        boundCase && !boundCase->empty()
            ? boundCase->front().lift[71] + 0.4 * (boundCase->front().lift[72] - boundCase->front().lift[71])
            : 0.0;
    const bool periodicAngles =
        std::abs(AeroManager::normalizeAngleDegrees(357.0) - 357.0) <= 1.0e-12 &&
        std::abs(AeroManager::normalizeAngleDegrees(1000.0) - 280.0) <= 1.0e-12 &&
        std::abs(manager.getData(key, 0, LIFT, 357.0) - expected357) <= 1.0e-12 &&
        std::abs(manager.getData(key, 0, LIFT, 1000.0) - manager.getData(key, 0, LIFT, 280.0)) <= 1.0e-12;
    const Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    const int referenceProfile = BundleAeroMapper::ResolveProfile(4, 0, axis, up, -Eigen::Vector3d::UnitY());
    const int reversedProfile = BundleAeroMapper::ResolveProfile(4, 0, axis, up, Eigen::Vector3d::UnitY());
    const bool profileMapping = referenceProfile == 0 && reversedProfile == 2;

    auto firstNode = std::make_shared<Node>();
    auto secondNode = std::make_shared<Node>();
    firstNode->SetNumDOFs(4);
    secondNode->SetNumDOFs(4);
    secondNode->m_X = 2.0;
    for (int dof = 0; dof < 4; ++dof)
    {
        firstNode->m_DOF[dof] = dof;
        secondNode->m_DOF[dof] = dof + 4;
    }
    auto section = std::make_shared<SectionCircular>();
    section->m_Radius = 0.015;
    section->m_Area = std::acos(-1.0) * section->m_Radius * section->m_Radius;
    auto property = std::make_shared<Property>();
    property->m_pSection = section;
    auto element = std::make_shared<ElementCable>();
    element->m_pNode[0] = firstNode;
    element->m_pNode[1] = secondNode;
    element->m_pProperty = property;
    element->m_WireId = 0;
    element->m_AeroBundleCount = 4;
    element->m_AeroProfileId = 0;
    structure.m_Elements[1] = element;
    structure.m_AeroManager = manager;
    wind.m_direction = -Eigen::Vector3d::UnitY();
    Eigen::VectorXd fixed = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd free = Eigen::VectorXd::Zero(4);
    const std::map<int, int> profileBindings{{1, referenceProfile}};
    LoadAssembler::AssembleGalloping(wind, structure, profileBindings, 25, 45.0, up, 4, 1.0, fixed, free);
    const bool aerodynamicAssembly = fixed.allFinite() && free.allFinite() && fixed.norm() > 0.0 && free.norm() > 0.0;
    Eigen::VectorXd angle1000Fixed = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd angle1000Free = Eigen::VectorXd::Zero(4);
    LoadAssembler::AssembleGalloping(wind, structure, profileBindings, 25, 1000.0, up, 4, 1.0, angle1000Fixed,
                                     angle1000Free);
    Eigen::VectorXd angle280Fixed = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd angle280Free = Eigen::VectorXd::Zero(4);
    LoadAssembler::AssembleGalloping(wind, structure, profileBindings, 25, 280.0, up, 4, 1.0, angle280Fixed,
                                     angle280Free);
    const double periodicAssemblyError =
        std::max((angle1000Fixed - angle280Fixed).norm(), (angle1000Free - angle280Free).norm());

    QTextStream(stdout) << "galloping case bundle=" << key.bundleCount << " wind=" << key.windSpeed
                        << " ice=" << key.iceThickness << " supported=" << supported
                        << " unsupported_bundle_rejected=" << unsupportedBundleRejected
                        << " static_rejected=" << staticRejected << " case_loaded=" << loaded
                        << " all_cases_loaded=" << allCasesLoaded
                        << " catalog_cases=" << completeCatalog.getLoadedCaseCount()
                        << " max_0_360_gap=" << maximumEndpointGap << " max_symmetry_error=" << maximumSymmetryError
                        << " combined_lookup_matches=" << combinedLookupMatches
                        << " angle_357_cl=" << manager.getData(key, 0, LIFT, 357.0)
                        << " angle_1000=" << AeroManager::normalizeAngleDegrees(1000.0)
                        << " reference_profile=" << referenceProfile << " reversed_profile=" << reversedProfile
                        << " aero_force_norm=" << free.norm() << " periodic_force_error=" << periodicAssemblyError
                        << Qt::endl;
    return supported && unsupportedBundleRejected && staticRejected && loaded && allCasesLoaded && catalogShapeValid &&
                   maximumEndpointGap <= 1.0e-12 && maximumSymmetryError <= 1.0e-12 && combinedLookupMatches &&
                   periodicAngles && profileMapping && aerodynamicAssembly && periodicAssemblyError <= 1.0e-12 &&
                   key == AeroCaseKey{4, 14, 25}
               ? 0
               : 1;
}

std::optional<int> verifyCableTorsion(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-cable-torsion")))
        return std::nullopt;

    // 均匀圆截面索在 x=0 固定、x=L 施加扭矩。Saint-Venant 解为 T = GJ*theta/L，
    // 固支—自由边界的一阶扭转频率为 omega_1 = pi/(2L)*sqrt(G/rho)。
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
    const double staticError = std::abs(staticCable.m_inforce(7) - expectedTorque) / expectedTorque;
    const double torsionalStiffnessError =
        std::abs(staticStiffness(7, 7) - shearModulus * polarMoment / length) / (shearModulus * polarMoment / length);

    // 初始无应力的受压索必须松弛：轴向项消失，但显式建模的扭转自由度仍然有效。
    node1->m_Displacement[0] = -1.0e-3;
    node1->m_Displacement[3] = 0.0;
    staticCable.m_InitStress = 0.0;
    Eigen::MatrixXd slackStiffness;
    staticCable.Get_ke(slackStiffness);
    const double slackAxialForce = staticCable.m_inforce.head<3>().norm() + staticCable.m_inforce.segment<3>(4).norm();
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
    const double expectedOmega = std::acos(-1.0) / (2.0 * length) * std::sqrt(shearModulus / density);
    const double frequencyError = std::abs(computedOmega - expectedOmega) / expectedOmega;

    // 使用 Newmark 平均加速度法积分有限元一阶扭转模态，一个计算周期后应恢复初始振幅。
    const double period = 2.0 * std::acos(-1.0) / computedOmega;
    constexpr int steps = 400;
    const double dt = period / steps;
    double displacement = appliedTwist;
    double velocity = 0.0;
    double acceleration = -computedOmega * computedOmega * displacement;
    for (int step = 0; step < steps; ++step)
    {
        const double predictedDisplacement = displacement + dt * velocity + 0.25 * dt * dt * acceleration;
        const double predictedVelocity = velocity + 0.5 * dt * acceleration;
        const double newAcceleration = -computedOmega * computedOmega * predictedDisplacement /
                                       (1.0 + 0.25 * dt * dt * computedOmega * computedOmega);
        displacement = predictedDisplacement + 0.25 * dt * dt * newAcceleration;
        velocity = predictedVelocity + 0.5 * dt * newAcceleration;
        acceleration = newAcceleration;
    }
    const double transientError = std::abs(displacement - appliedTwist) / appliedTwist;

    QTextStream(stdout) << "cable torsion static_torque_error=" << staticError
                        << " static_stiffness_error=" << torsionalStiffnessError
                        << " slack_axial_force=" << slackAxialForce
                        << " slack_torsional_stiffness=" << slackTorsionalStiffness << " omega=" << computedOmega
                        << " omega_reference=" << expectedOmega << " frequency_error=" << frequencyError
                        << " one_period_response_error=" << transientError << Qt::endl;
    return staticError <= 1.0e-12 && torsionalStiffnessError <= 1.0e-12 && slackAxialForce <= 1.0e-8 &&
                   slackTorsionalStiffness > 0.0 && frequencyError <= 2.0e-3 && transientError <= 2.0e-3
               ? 0
               : 3;
}

std::optional<int> verifyCableReference(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-cable-reference")))
        return std::nullopt;

    // 参考公式位于 D:\VS\TSSBN\Wind_method\Element_Cable_CR.cpp 的
    // Element_Cable_CR::Calculate_ke_TSSBN 与 Calculate_me。
    constexpr double initialLength = 4.0;
    constexpr double radius = 0.012;
    constexpr double young = 70.0e9;
    constexpr double poisson = 0.30;
    constexpr double density = 2700.0;
    constexpr double initialStress = 35.0e6;
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
    auto first = std::make_shared<Node>();
    auto second = std::make_shared<Node>();
    first->SetNumDOFs(4);
    second->SetNumDOFs(4);
    second->m_X = initialLength;
    first->m_Displacement[0] = 0.03;
    first->m_Displacement[1] = -0.02;
    first->m_Displacement[2] = 0.01;
    first->m_Displacement[3] = -0.004;
    second->m_Displacement[0] = 0.11;
    second->m_Displacement[1] = 0.17;
    second->m_Displacement[2] = -0.08;
    second->m_Displacement[3] = 0.013;

    ElementCable cable;
    cable.m_pNode[0] = first;
    cable.m_pNode[1] = second;
    cable.m_pProperty = property;
    cable.m_InitStress = initialStress;

    Eigen::MatrixXd actualStiffness;
    Eigen::MatrixXd actualMass;
    cable.Get_ke(actualStiffness);
    const Eigen::VectorXd actualForce = cable.m_inforce;
    cable.Get_me_Consistent(actualMass);

    const Eigen::Vector3d firstPosition(first->m_X + first->m_Displacement[0], first->m_Y + first->m_Displacement[1],
                                        first->m_Z + first->m_Displacement[2]);
    const Eigen::Vector3d secondPosition(second->m_X + second->m_Displacement[0],
                                         second->m_Y + second->m_Displacement[1],
                                         second->m_Z + second->m_Displacement[2]);
    const Eigen::Vector3d chord = secondPosition - firstPosition;
    const double currentLength = chord.norm();
    const Eigen::Vector3d axis = chord / currentLength;
    const double axialStiffness = young * area / initialLength;
    const double torsionalStiffness = shearModulus * polarMoment / initialLength;
    Eigen::Vector2d generalizedForce;
    generalizedForce[0] = initialStress * area + axialStiffness * (currentLength - initialLength);
    generalizedForce[1] = torsionalStiffness * (second->m_Displacement[3] - first->m_Displacement[3]);
    Eigen::Matrix<double, 2, 8> strainDisplacement = Eigen::Matrix<double, 2, 8>::Zero();
    strainDisplacement.block<1, 3>(0, 0) = -axis.transpose();
    strainDisplacement.block<1, 3>(0, 4) = axis.transpose();
    strainDisplacement(1, 3) = -1.0;
    strainDisplacement(1, 7) = 1.0;
    Eigen::Matrix2d materialTangent = Eigen::Matrix2d::Zero();
    materialTangent(0, 0) = axialStiffness;
    materialTangent(1, 1) = torsionalStiffness;
    Eigen::MatrixXd referenceStiffness = strainDisplacement.transpose() * materialTangent * strainDisplacement;
    const Eigen::Matrix3d geometricBlock =
        generalizedForce[0] / currentLength * (Eigen::Matrix3d::Identity() - axis * axis.transpose());
    referenceStiffness.block<3, 3>(0, 0) += geometricBlock;
    referenceStiffness.block<3, 3>(4, 4) += geometricBlock;
    referenceStiffness.block<3, 3>(0, 4) -= geometricBlock;
    referenceStiffness.block<3, 3>(4, 0) -= geometricBlock;
    const Eigen::VectorXd referenceForce = strainDisplacement.transpose() * generalizedForce;

    Eigen::Matrix4d sectionMass = Eigen::Matrix4d::Zero();
    sectionMass(0, 0) = sectionMass(1, 1) = sectionMass(2, 2) = density * area;
    sectionMass(3, 3) = density * polarMoment;
    Eigen::MatrixXd referenceMass = Eigen::MatrixXd::Zero(8, 8);
    referenceMass.block<4, 4>(0, 0) = 2.0 * sectionMass;
    referenceMass.block<4, 4>(0, 4) = sectionMass;
    referenceMass.block<4, 4>(4, 0) = sectionMass;
    referenceMass.block<4, 4>(4, 4) = 2.0 * sectionMass;
    referenceMass *= initialLength / 6.0;

    const double stiffnessReferenceError =
        (actualStiffness - referenceStiffness).norm() / std::max(1.0, referenceStiffness.norm());
    const double forceReferenceError = (actualForce - referenceForce).norm() / std::max(1.0, referenceForce.norm());
    const double massReferenceError = (actualMass - referenceMass).norm() / std::max(1.0, referenceMass.norm());

    // 数值微分用于证明返回矩阵是内力的一致导数，而不是仅能复现 TSSBN 源代码结果的矩阵。
    Eigen::MatrixXd numericalTangent = Eigen::MatrixXd::Zero(8, 8);
    constexpr double perturbation = 1.0e-7;
    const auto displacementEntry = [&](int localDof) -> double&
    {
        const int nodeIndex = localDof / 4;
        const int component = localDof % 4;
        return (nodeIndex == 0 ? first : second)->m_Displacement[component];
    };
    for (int column = 0; column < 8; ++column)
    {
        double& value = displacementEntry(column);
        const double original = value;
        value = original + perturbation;
        Eigen::MatrixXd ignored;
        cable.Get_ke(ignored);
        const Eigen::VectorXd positiveForce = cable.m_inforce;
        value = original - perturbation;
        cable.Get_ke(ignored);
        const Eigen::VectorXd negativeForce = cable.m_inforce;
        value = original;
        numericalTangent.col(column) = (positiveForce - negativeForce) / (2.0 * perturbation);
    }
    cable.Get_ke(actualStiffness);
    const double tangentConsistencyError =
        (actualStiffness - numericalTangent).norm() / std::max(1.0, numericalTangent.norm());

    // 使用正式单元和 Newton 方法求解静力轴向平衡。
    std::fill(first->m_Displacement.begin(), first->m_Displacement.end(), 0.0);
    std::fill(second->m_Displacement.begin(), second->m_Displacement.end(), 0.0);
    const double initialForce = initialStress * area;
    const double appliedForce = initialForce + 0.0025 * young * area;
    for (int iteration = 0; iteration < 8; ++iteration)
    {
        cable.Get_ke(actualStiffness);
        const double residual = appliedForce - cable.m_inforce[4];
        if (std::abs(residual) <= 1.0e-10 * appliedForce)
            break;
        second->m_Displacement[0] += residual / actualStiffness(4, 4);
    }
    cable.Get_ke(actualStiffness);
    const double exactStaticDisplacement = (appliedForce - initialForce) / axialStiffness;
    const double staticDisplacementError =
        std::abs(second->m_Displacement[0] - exactStaticDisplacement) / exactStaticDisplacement;
    const double staticResidualError = std::abs(appliedForce - cable.m_inforce[4]) / appliedForce;

    // 张紧固支索的横向振动，K 与 M 完全由 ElementCable 组装；连续体参考解为
    // omega_1 = pi/L*sqrt(N0/(rho*A))。
    constexpr int elementCount = 32;
    constexpr double span = 30.0;
    constexpr double pretension = 60.0e3;
    std::vector<std::shared_ptr<Node>> nodes(elementCount + 1);
    for (int index = 0; index <= elementCount; ++index)
    {
        nodes[index] = std::make_shared<Node>();
        nodes[index]->SetNumDOFs(4);
        nodes[index]->m_X = span * index / elementCount;
    }
    const int freeCount = elementCount - 1;
    Eigen::MatrixXd transverseStiffness = Eigen::MatrixXd::Zero(freeCount, freeCount);
    Eigen::MatrixXd transverseMass = Eigen::MatrixXd::Zero(freeCount, freeCount);
    for (int index = 0; index < elementCount; ++index)
    {
        ElementCable segment;
        segment.m_pNode[0] = nodes[index];
        segment.m_pNode[1] = nodes[index + 1];
        segment.m_pProperty = property;
        segment.m_InitStress = pretension / area;
        Eigen::MatrixXd elementStiffness;
        Eigen::MatrixXd elementMass;
        segment.Get_ke(elementStiffness);
        segment.Get_me_Consistent(elementMass);
        const int equation[2] = {index - 1, index};
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b)
                if (equation[a] >= 0 && equation[a] < freeCount && equation[b] >= 0 && equation[b] < freeCount)
                {
                    transverseStiffness(equation[a], equation[b]) += elementStiffness(4 * a + 1, 4 * b + 1);
                    transverseMass(equation[a], equation[b]) += elementMass(4 * a + 1, 4 * b + 1);
                }
    }

    // 经典静力基准：固支张紧索在小均布横向线荷载下的跨中抛物线垂度为 f = q*L^2/(8*H)。
    // 采用较小垂跨比，使恒定水平张力解仍可作为连续体极限，同时正式单元仍求解完整非线性几何。
    constexpr double transverseLineLoad = 1.6;
    const double elementLength = span / elementCount;
    Eigen::VectorXd staticLoad = Eigen::VectorXd::Constant(freeCount, transverseLineLoad * elementLength);
    Eigen::VectorXd staticDisplacement = Eigen::VectorXd::Zero(freeCount);
    double staticCableResidual = std::numeric_limits<double>::infinity();
    int staticCableIterations = 0;
    for (int iteration = 0; iteration < 20; ++iteration)
    {
        staticCableIterations = iteration + 1;
        for (int nodeIndex = 1; nodeIndex < elementCount; ++nodeIndex)
            nodes[nodeIndex]->m_Displacement[1] = staticDisplacement[nodeIndex - 1];

        Eigen::MatrixXd tangent = Eigen::MatrixXd::Zero(freeCount, freeCount);
        Eigen::VectorXd internalForce = Eigen::VectorXd::Zero(freeCount);
        for (int index = 0; index < elementCount; ++index)
        {
            ElementCable segment;
            segment.m_pNode[0] = nodes[index];
            segment.m_pNode[1] = nodes[index + 1];
            segment.m_pProperty = property;
            segment.m_InitStress = pretension / area;
            Eigen::MatrixXd elementStiffness;
            segment.Get_ke(elementStiffness);
            const int equation[2] = {index - 1, index};
            for (int a = 0; a < 2; ++a)
            {
                if (equation[a] < 0 || equation[a] >= freeCount)
                    continue;
                internalForce[equation[a]] += segment.m_inforce[4 * a + 1];
                for (int b = 0; b < 2; ++b)
                {
                    if (equation[b] < 0 || equation[b] >= freeCount)
                        continue;
                    tangent(equation[a], equation[b]) += elementStiffness(4 * a + 1, 4 * b + 1);
                }
            }
        }
        const Eigen::VectorXd residual = staticLoad - internalForce;
        staticCableResidual = residual.norm() / std::max(1.0, staticLoad.norm());
        if (staticCableResidual <= 1.0e-11)
            break;
        const Eigen::VectorXd correction = tangent.ldlt().solve(residual);
        if (!correction.allFinite())
            return 4;
        staticDisplacement += correction;
    }
    const double computedMidspanSag = staticDisplacement[elementCount / 2 - 1];
    const double referenceMidspanSag = transverseLineLoad * span * span / (8.0 * pretension);
    const double midspanSagError = std::abs(computedMidspanSag - referenceMidspanSag) / referenceMidspanSag;

    // 模态计算前恢复直线平衡状态。
    for (int nodeIndex = 1; nodeIndex < elementCount; ++nodeIndex)
        nodes[nodeIndex]->m_Displacement[1] = 0.0;

    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> modes(transverseStiffness, transverseMass);
    if (modes.info() != Eigen::Success)
        return 2;
    const double computedOmega = std::sqrt(modes.eigenvalues()[0]);
    const double referenceOmega = std::acos(-1.0) / span * std::sqrt(pretension / (density * area));
    const double frequencyError = std::abs(computedOmega - referenceOmega) / referenceOmega;

    // 将索有限元一阶模态输入正式自适应 TSSBN 积分器，与五个周期的无阻尼精确解比较。
    const double period = 2.0 * std::acos(-1.0) / computedOmega;
    LinearOscillatorModel oscillator(1.0, computedOmega * computedOmega);
    SolverNameSpace::SolverAdaptiveTSSBN::Params parameters;
    parameters.initialTimeStep = period / 80.0;
    parameters.minimumTimeStep = period / 10000.0;
    parameters.maximumTimeStep = period / 40.0;
    parameters.relativeTolerance = 1.0e-7;
    parameters.absoluteTolerance = 1.0e-10;
    parameters.nonlinearTolerance = 1.0e-6;
    parameters.maximumNewtonIterations = 12;
    SolverNameSpace::SolverAdaptiveTSSBN tssbn(parameters);
    const bool dynamicSolved = tssbn.Solve(oscillator, 5.0 * period);
    SolverNameSpace::Vec displacement;
    SolverNameSpace::Vec velocity;
    SolverNameSpace::Vec acceleration;
    oscillator.GetState(displacement, velocity, acceleration);
    const double dynamicDisplacementError = std::abs(displacement[0] - 1.0);
    const double dynamicVelocityError = std::abs(velocity[0]) / computedOmega;

    QTextStream(stdout) << "cable_reference stiffness_error=" << stiffnessReferenceError
                        << " force_error=" << forceReferenceError << " mass_error=" << massReferenceError
                        << " tangent_error=" << tangentConsistencyError
                        << " static_displacement_error=" << staticDisplacementError
                        << " static_residual_error=" << staticResidualError << " midspan_sag=" << computedMidspanSag
                        << " midspan_sag_reference=" << referenceMidspanSag << " midspan_sag_error=" << midspanSagError
                        << " static_cable_iterations=" << staticCableIterations
                        << " static_cable_residual=" << staticCableResidual << " omega=" << computedOmega
                        << " omega_reference=" << referenceOmega << " frequency_error=" << frequencyError
                        << " tssbn_solved=" << dynamicSolved << " tssbn_displacement_error=" << dynamicDisplacementError
                        << " tssbn_velocity_error=" << dynamicVelocityError
                        << " accepted=" << tssbn.GetAcceptedStepCount() << " rejected=" << tssbn.GetRejectedStepCount()
                        << Qt::endl;

    const bool passed = stiffnessReferenceError <= 1.0e-13 && forceReferenceError <= 1.0e-13 &&
                        massReferenceError <= 1.0e-13 && tangentConsistencyError <= 2.0e-8 &&
                        staticDisplacementError <= 1.0e-12 && staticResidualError <= 1.0e-12 &&
                        midspanSagError <= 5.0e-4 && staticCableResidual <= 1.0e-11 && frequencyError <= 5.0e-4 &&
                        dynamicSolved && dynamicDisplacementError <= 2.0e-4 && dynamicVelocityError <= 5.0e-4;
    return passed ? 0 : 3;
}

std::optional<int> verifySpatialWindLoad(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-spatial-wind-load")))
        return std::nullopt;

    constexpr double radius = 0.015;
    constexpr double initialLength = 5.0;
    constexpr double density = 1.225;
    constexpr double speed = 10.0;
    const double q = 0.5 * density * speed * speed * (2.0 * radius);

    StructureData structure;
    auto node0 = std::make_shared<Node>();
    auto node1 = std::make_shared<Node>();
    node0->SetNumDOFs(4);
    node1->SetNumDOFs(4);
    node1->m_X = initialLength;
    for (int i = 0; i < 4; ++i)
    {
        node0->m_DOF[i] = i;
        node1->m_DOF[i] = i + 4;
    }

    auto material = std::make_shared<Material>();
    material->m_Density = 7850.0;
    auto section = std::make_shared<SectionCircular>();
    section->m_Radius = radius;
    section->m_Area = std::acos(-1.0) * radius * radius;
    auto property = std::make_shared<Property>();
    property->m_pMaterial = material;
    property->m_pSection = section;
    auto cable = std::make_shared<ElementCable>();
    cable->m_pNode[0] = node0;
    cable->m_pNode[1] = node1;
    cable->m_pProperty = property;
    structure.m_Elements.emplace(1, cable);

    Force_Wind wind;
    wind.m_Direction = EnumKeyword::Direction::Y;
    wind.m_direction = Eigen::Vector3d::UnitY();
    wind.m_velocity = speed;
    wind.m_windDensity = density;

    Eigen::VectorXd fixed = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd free = Eigen::VectorXd::Zero(4);
    LoadAssembler::Assemble(wind, structure, 4, 1.0, fixed, free);
    const double forceError =
        std::max(std::abs(fixed[1] - q * initialLength / 2.0), std::abs(free[1] - q * initialLength / 2.0));

    // 旋转当前索弦，验证空间风向保持全局不变，而等效节点荷载采用当前构形。
    node1->m_Displacement[0] = -2.0;
    node1->m_Displacement[1] = 4.0;
    fixed.setZero();
    free.setZero();
    LoadAssembler::Assemble(wind, structure, 4, 1.0, fixed, free);
    const double rotatedForceError = std::max(std::abs(fixed[1] - q * 5.0 / 2.0), std::abs(free[1] - q * 5.0 / 2.0));

    // 任意全局方向必须逐分量组装，不能简化为某个坐标轴方向。
    std::fill(node1->m_Displacement.begin(), node1->m_Displacement.end(), 0.0);
    wind.m_direction = Eigen::Vector3d(1.0, 2.0, -2.0);
    fixed.setZero();
    free.setZero();
    LoadAssembler::Assemble(wind, structure, 4, 1.0, fixed, free);
    const Eigen::Vector3d expectedVector = q * initialLength / 2.0 * wind.m_direction.normalized();
    const double vectorDirectionError =
        std::max((fixed.head<3>() - expectedVector).norm(), (free.head<3>() - expectedVector).norm());

    QTextStream(stdout) << "spatial wind q=" << q << " initial_node_force=" << q * initialLength / 2.0
                        << " rotated_node_force=" << q * 5.0 / 2.0 << " error=" << forceError
                        << " rotated_error=" << rotatedForceError << " vector_error=" << vectorDirectionError
                        << Qt::endl;
    return forceError <= 1.0e-12 && rotatedForceError <= 1.0e-12 && vectorDirectionError <= 1.0e-12 ? 0 : 1;
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
    if (arguments.contains(QStringLiteral("--verify-result-read-all")))
    {
        Hdf5ResultFrame scanned;
        for (int frameIndex = 0; frameIndex < static_cast<int>(frames.size()); ++frameIndex)
        {
            if (!reader.ReadResultFrame(frameIndex, scanned))
            {
                QTextStream(stderr) << "result frame read failed index=" << frameIndex
                                    << " domain=" << frames[static_cast<size_t>(frameIndex)].domainId << Qt::endl;
                return 5;
            }
        }
    }
    QTextStream(stdout) << "result frames=" << frames.size() << " first_step=" << first.info.stepId
                        << " first_nodes=" << first.nodes.size() << " first_elements=" << first.elements.size()
                        << " last_time=" << last.info.time << Qt::endl;
    return first.nodes.empty() || first.elements.empty() ? 4 : 0;
}

std::optional<int> verifyShearReleaseResult(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-shear-release-result"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    Hdf5ModelIO reader;
    std::vector<Hdf5ResultFrameInfo> frames;
    if (!reader.OpenResultFile(arguments.at(index + 1), frames) || frames.empty())
        return 2;

    double maximumConstraintGap = 0.0;
    double maximumNormalizedError = 0.0;
    double finalValues[4] = {};
    for (int frameIndex = 0; frameIndex < static_cast<int>(frames.size()); ++frameIndex)
    {
        Hdf5ResultFrame frame;
        if (!reader.ReadResultFrame(frameIndex, frame))
            return 3;
        const auto findNode = [&frame](int id) -> const Hdf5NodalResult*
        {
            const auto found = std::find_if(frame.nodes.cbegin(), frame.nodes.cend(),
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

        const double phi = 2.0 * std::acos(-1.0) * frame.info.time;
        if (std::abs(phi) < 1.0e-12)
            continue;
        const double sinPhi = std::sin(phi);
        const double cosPhi = std::cos(phi);
        const double u2Expected = sinPhi / phi - 1.0;
        const double w2Expected = (1.0 - cosPhi) / phi;
        const double u3Expected = u2Expected + 2.0 * std::tan(phi) * (1.0 - cosPhi) / phi;
        const double w3Expected = -w2Expected;
        const double calculated[4] = {node2->displacement[0], node2->displacement[1], node3->displacement[0],
                                      node3->displacement[1]};
        const double expected[4] = {u2Expected, w2Expected, u3Expected, w3Expected};
        for (int component = 0; component < 4; ++component)
        {
            if (!std::isfinite(calculated[component]))
                return 5;
            // 论文指出 M 接近 pi 和 3*pi 时 u3 存在奇异性，离散误差会明显增大。
            if (std::abs(cosPhi) >= 0.10)
            {
                maximumNormalizedError =
                    std::max(maximumNormalizedError, std::abs(calculated[component] - expected[component]) /
                                                         std::max(1.0, std::abs(expected[component])));
            }
            if (frameIndex + 1 == static_cast<int>(frames.size()))
                finalValues[component] = calculated[component];
        }

        const double constraintGap =
            cosPhi * (calculated[0] - calculated[2]) + sinPhi * (calculated[1] - calculated[3]);
        maximumConstraintGap = std::max(maximumConstraintGap, std::abs(constraintGap));
    }
    reader.CloseResultFile();

    QTextStream(stdout) << "shear-release frames=" << frames.size() << " max_constraint_gap=" << maximumConstraintGap
                        << " max_normalized_analytical_error=" << maximumNormalizedError
                        << " final_u2=" << finalValues[0] << " final_w2=" << finalValues[1]
                        << " final_u3=" << finalValues[2] << " final_w3=" << finalValues[3] << Qt::endl;
    return maximumConstraintGap <= 1.0e-8 && maximumNormalizedError <= 0.25 ? 0 : 6;
}

std::optional<int> verifyShearReleaseModel(const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-shear-release-model"));
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
            if (message.contains(QStringLiteral("失败")) || message.contains(QStringLiteral("未收敛")) ||
                message.contains(QStringLiteral("奇异")))
            {
                QTextStream(stdout) << "progress=" << progress << " message=" << message << Qt::endl;
            }
        },
        []()
        {
            return false;
        });
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
        const double u2 = frame.GetNodeData(2, EnumKeyword::NodeResultType::U1);
        const double w2 = frame.GetNodeData(2, EnumKeyword::NodeResultType::U2);
        const double u3 = frame.GetNodeData(3, EnumKeyword::NodeResultType::U1);
        const double w3 = frame.GetNodeData(3, EnumKeyword::NodeResultType::U2);
        const double phi2Output = frame.GetNodeData(2, EnumKeyword::NodeResultType::UR3);
        const double phi3 = frame.GetNodeData(3, EnumKeyword::NodeResultType::UR3);
        const double phi2 = 2.0 * std::acos(-1.0) * frame.GetTime();
        const double sinPhi = std::sin(phi2);
        const double cosPhi = std::cos(phi2);
        const double expected[4] = {sinPhi / phi2 - 1.0, (1.0 - cosPhi) / phi2,
                                    sinPhi / phi2 - 1.0 + 2.0 * std::tan(phi2) * (1.0 - cosPhi) / phi2,
                                    -(1.0 - cosPhi) / phi2};
        const double calculated[4] = {u2, w2, u3, w3};
        for (int component = 0; component < 4; ++component)
        {
            if (!std::isfinite(calculated[component]))
                return 5;
            if (std::abs(cosPhi) >= 0.10)
            {
                const double error = std::abs(calculated[component] - expected[component]) /
                                     std::max(1.0, std::abs(expected[component]));
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
        maximumConstraintGap = std::max(maximumConstraintGap, std::abs(cosPhi * (u2 - u3) + sinPhi * (w2 - w3)));
        maximumRotationGap =
            std::max(maximumRotationGap, std::abs(std::remainder(phi2Output - phi3, 2.0 * std::acos(-1.0))));
    }

    const DataFrame& last = frames.back();
    QTextStream(stdout) << "shear-release direct frames=" << frames.size()
                        << " max_constraint_gap=" << maximumConstraintGap << " max_rotation_gap=" << maximumRotationGap
                        << " max_normalized_analytical_error=" << maximumNormalizedError << " worst_phi=" << worstPhi
                        << " worst_component=" << worstComponent << " worst_calculated=" << worstCalculated
                        << " worst_expected=" << worstExpected
                        << " final_u2=" << last.GetNodeData(2, EnumKeyword::NodeResultType::U1)
                        << " final_w2=" << last.GetNodeData(2, EnumKeyword::NodeResultType::U2)
                        << " final_u3=" << last.GetNodeData(3, EnumKeyword::NodeResultType::U1)
                        << " final_w3=" << last.GetNodeData(3, EnumKeyword::NodeResultType::U2) << Qt::endl;
    return maximumConstraintGap <= 1.0e-8 && maximumRotationGap <= 1.0e-8 && maximumNormalizedError <= 0.25 ? 0 : 6;
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
        if (!element || element->m_pNode.size() != 2 || element->m_pNode[0].expired() || element->m_pNode[1].expired())
        {
            QTextStream(stderr) << "invalid element=" << elementId << Qt::endl;
            return 3;
        }
    }
    QTextStream(stdout) << "hdf5 model nodes=" << structure->m_Nodes.size()
                        << " elements=" << structure->m_Elements.size() << " materials=" << structure->m_Material.size()
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
    const int secondElementId = source->m_Elements.size() > 2 ? source->m_Elements.crbegin()->first : 2;
    const int firstSetId = source->AddModelSet(QStringLiteral("区域A单元"), ModelSetType::Element, {1}, &error);
    const int secondSetId =
        source->AddModelSet(QStringLiteral("区域B单元"), ModelSetType::Element, {secondElementId}, &error);
    const int firstRegionId = source->AddComputeRegionFromSets(QStringLiteral("区域A"), {firstSetId}, true, &error);
    const int secondRegionId = source->AddComputeRegionFromSets(QStringLiteral("区域B"), {secondSetId}, true, &error);
    if (firstSetId <= 0 || secondSetId <= 0 || firstRegionId <= 0 || secondRegionId <= 0)
        return 3;

    source->EnsureDefaultAnalysisConfiguration();
    if (source->m_AnalysisStep.empty() || !source->m_AnalysisStep.begin()->second)
        return 4;
    const int equilibriumStepId = source->m_AnalysisStep.begin()->first;
    auto equilibriumStep = source->m_AnalysisStep.begin()->second;
    equilibriumStep->m_Name = QStringLiteral("静力平衡");
    equilibriumStep->m_Type = EnumKeyword::StepType::STATIC;
    equilibriumStep->isDynamic = false;
    equilibriumStep->m_InitialStaticStepId = 0;
    AnalysisStepConfig dynamicConfig;
    dynamicConfig.id = source->m_AnalysisStep.crbegin()->first + 1;
    dynamicConfig.name = QStringLiteral("导线区域分析");
    dynamicConfig.type = EnumKeyword::StepType::DYNAMIC;
    dynamicConfig.totalTime = 1.0;
    dynamicConfig.stepSize = 0.1;
    dynamicConfig.tolerance = 1.0e-5;
    dynamicConfig.maxIterations = 20;
    dynamicConfig.initialStaticStepId = equilibriumStepId;
    source->AddAnalysisStep(dynamicConfig);
    auto sourceStep = source->m_AnalysisStep.at(dynamicConfig.id);
    sourceStep->m_DynamicSolverType = SolverNameSpace::SolverType::AdaptiveTSSBN;
    sourceStep->m_AdaptiveTssbn.spectralRadiusInfinity = 0.73;
    sourceStep->m_AdaptiveTssbn.minimumTimeStep = 2.5e-6;
    sourceStep->m_AdaptiveTssbn.maximumTimeStep = 0.37;
    sourceStep->m_AdaptiveTssbn.relativeTolerance = 7.5e-4;
    sourceStep->m_AdaptiveTssbn.absoluteTolerance = 3.5e-7;
    sourceStep->m_AdaptiveTssbn.safetyFactor = 0.82;
    sourceStep->m_AdaptiveTssbn.shrinkFactor = 0.76;
    sourceStep->m_AdaptiveTssbn.maximumGrowthFactor = 2.4;
    sourceStep->m_GallopingAerodynamicTangentMode =
        SolverNameSpace::AerodynamicTangentMode::OncePerTimeStepOrStage;
    sourceStep->m_AdaptiveTssbn.targetNewtonIterations = 11;
    sourceStep->m_AdaptiveTssbn.maximumTotalNewtonIterations = 37;
    sourceStep->m_AdaptiveTssbn.derivativeGain = 0.13;
    sourceStep->m_AdaptiveTssbn.minimumDerivativeFactor = 0.44;
    sourceStep->m_AdaptiveTssbn.maximumDerivativeFactor = 1.72;
    sourceStep->m_AdaptiveTssbn.maximumRejectedAttempts = 19;
    sourceStep->m_EnableGalloping = true;
    sourceStep->m_GallopingIceThickness = 28;
    sourceStep->m_GallopingInitialAttackDegrees = 357.0;
    sourceStep->m_RegionScope = AnalysisRegionScope::SelectedRegions;
    sourceStep->m_ComputeRegionIds = {firstRegionId, secondRegionId};
    const AeroCaseKey sourceAeroKey{1, 18, 28};
    const std::filesystem::path aerodynamicDataDirectory =
        std::filesystem::path(ApplicationPaths::aerodynamicDataDirectory().toStdWString());
    if (!source->m_AeroManager.loadCase(aerodynamicDataDirectory, sourceAeroKey))
        return 5;
    const int windLoadId = source->m_Load.empty() ? 1 : source->m_Load.crbegin()->first + 1;
    auto sourceWind = std::make_shared<Force_Wind>();
    sourceWind->m_Id = windLoadId;
    sourceWind->m_Name = QStringLiteral("任意方向风");
    sourceWind->m_StepId = sourceStep->m_Id;
    sourceWind->m_velocity = 18.0;
    sourceWind->m_direction = Eigen::Vector3d(1.0, 2.0, 0.5).normalized();
    source->m_Load[windLoadId] = sourceWind;

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
    const auto restoredWindEntry = restored->m_Load.find(windLoadId);
    const auto restoredWind = restoredWindEntry != restored->m_Load.cend()
                                  ? std::dynamic_pointer_cast<Force_Wind>(restoredWindEntry->second)
                                  : nullptr;
    const double windDirectionError = restoredWind ? (restoredWind->m_direction - sourceWind->m_direction).norm()
                                                   : std::numeric_limits<double>::infinity();
    if (restored->m_ModelSets.size() != 2 || restored->m_ComputeRegions.size() != 2 ||
        restored->m_MPCConstraints.size() != source->m_MPCConstraints.size() ||
        restoredStep == restored->m_AnalysisStep.cend() || !restoredStep->second ||
        restoredStep->second->m_Name != sourceStep->m_Name ||
        restoredStep->second->m_InitialStaticStepId != equilibriumStepId || !restoredStep->second->m_EnableGalloping ||
        restoredStep->second->m_GallopingIceThickness != 28 ||
        restoredStep->second->m_DynamicSolverType != SolverNameSpace::SolverType::AdaptiveTSSBN ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.spectralRadiusInfinity - 0.73) > 1.0e-12 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.minimumTimeStep - 2.5e-6) > 1.0e-15 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.maximumTimeStep - 0.37) > 1.0e-12 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.relativeTolerance - 7.5e-4) > 1.0e-15 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.absoluteTolerance - 3.5e-7) > 1.0e-15 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.safetyFactor - 0.82) > 1.0e-12 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.shrinkFactor - 0.76) > 1.0e-12 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.maximumGrowthFactor - 2.4) > 1.0e-12 ||
        restoredStep->second->m_GallopingAerodynamicTangentMode !=
            SolverNameSpace::AerodynamicTangentMode::OncePerTimeStepOrStage ||
        restoredStep->second->m_AdaptiveTssbn.targetNewtonIterations != 11 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.derivativeGain - 0.13) > 1.0e-12 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.minimumDerivativeFactor - 0.44) > 1.0e-12 ||
        std::abs(restoredStep->second->m_AdaptiveTssbn.maximumDerivativeFactor - 1.72) > 1.0e-12 ||
        restoredStep->second->m_AdaptiveTssbn.maximumRejectedAttempts != 19 ||
        restoredStep->second->m_AdaptiveTssbn.maximumTotalNewtonIterations != 37 ||
        std::abs(restoredStep->second->m_GallopingInitialAttackDegrees - 357.0) > 1.0e-12 ||
        !restored->m_AeroManager.hasCase(sourceAeroKey) ||
        std::abs(restored->m_AeroManager.getData(sourceAeroKey, 0, LIFT, 17.5) -
                 source->m_AeroManager.getData(sourceAeroKey, 0, LIFT, 17.5)) > 1.0e-12 ||
        restoredStep->second->m_RegionScope != AnalysisRegionScope::SelectedRegions ||
        restoredStep->second->m_ComputeRegionIds.size() != 2 || !restoredWind ||
        std::abs(restoredWind->m_velocity - 18.0) > 1.0e-12 || windDirectionError > 1.0e-12)
    {
        return 8;
    }
    QTextStream(stdout) << "hdf5 contract sets=" << restored->m_ModelSets.size()
                        << " regions=" << restored->m_ComputeRegions.size()
                        << " mpcs=" << restored->m_MPCConstraints.size() << " steps=" << restored->m_AnalysisStep.size()
                        << " galloping=" << (restoredStep->second->m_EnableGalloping ? 1 : 0)
                        << " ice=" << restoredStep->second->m_GallopingIceThickness
                        << " initial_attack=" << restoredStep->second->m_GallopingInitialAttackDegrees
                        << " tssbn_rel_tol=" << restoredStep->second->m_AdaptiveTssbn.relativeTolerance
                        << " tssbn_max_reject=" << restoredStep->second->m_AdaptiveTssbn.maximumRejectedAttempts
                        << " aero_cases=" << restored->m_AeroManager.getLoadedCaseCount()
                        << " wind_direction_error=" << windDirectionError << " result=" << (summary.hasResult ? 1 : 0)
                        << Qt::endl;
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
    const int firstRegionId = structure->AddComputeRegion(QStringLiteral("区域 A"), {}, {1}, {}, true, &error);
    const int secondRegionId = structure->AddComputeRegion(QStringLiteral("区域 B"), {}, {2}, {}, true, &error);
    if (firstRegionId <= 0 || secondRegionId <= 0 || structure->m_ComputeRegions.size() != 2)
    {
        QTextStream(stderr) << "region setup failed: " << error << Qt::endl;
        return 3;
    }
    const int mergedRegionId =
        structure->AddComputeRegion(QStringLiteral("区域 A 重叠候选"), {}, {1}, {}, true, &error);
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
    QTextStream(stdout) << "compute regions=" << structure->m_ComputeRegions.size() << " merged_id=" << mergedRegionId
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
    if (!importer.InputData(arguments.at(index + 1), structure) || structure->m_MPCConstraints.empty())
    {
        return 2;
    }
    const auto sourceMPC = structure->m_MPCConstraints.cbegin()->second;
    const std::vector<int> nodeIds = sourceMPC ? sourceMPC->GetNodeIds() : std::vector<int>();
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
                containsSlave = containsSlave || (node && node->m_Id == slaveNodeId);
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
    const int regionId = structure->AddComputeRegion(QStringLiteral("MPC 主节点区域"), {masterNodeId},
                                                     {detachedElementId}, {}, true, &error);
    const auto region = structure->m_ComputeRegions.find(regionId);
    if (regionId <= 0 || region == structure->m_ComputeRegions.cend() || !region->second ||
        !region->second->ContainsNode(masterNodeId) || !region->second->ContainsNode(slaveNodeId) ||
        structure->m_MPCConstraints.size() != 1)
    {
        QTextStream(stderr) << "MPC region expansion failed: " << error << Qt::endl;
        return 5;
    }

    const int stepId = structure->m_AnalysisStep.empty() ? 0 : structure->m_AnalysisStep.cbegin()->first;
    if (stepId <= 0)
        return 6;
    auto clone = structure->CloneRegionForAnalysis(regionId, stepId, &error);
    if (!clone || clone->m_MPCConstraints.size() != 1 || clone->m_Nodes.find(masterNodeId) == clone->m_Nodes.cend() ||
        clone->m_Nodes.find(slaveNodeId) == clone->m_Nodes.cend() || structure->m_MPCConstraints.size() != 1)
    {
        QTextStream(stderr) << "MPC regional clone failed: " << error << Qt::endl;
        return 7;
    }

    QTextStream(stdout) << "mpc regions=" << structure->m_ComputeRegions.size() << " master=" << masterNodeId
                        << " slave=" << slaveNodeId << " source_mpcs=" << structure->m_MPCConstraints.size()
                        << " clone_mpcs=" << clone->m_MPCConstraints.size() << Qt::endl;
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
    config.conductor.initialStress = 50.0e6;
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

    if (result.subConductors.size() != 4 || result.leftSupportNodeId <= 0 || result.rightSupportNodeId <= 0 ||
        result.leftTensionEnd.groupNodeIds.size() != 2 || result.rightTensionEnd.groupNodeIds.size() != 2 ||
        result.leftTensionEnd.yokeElementIds.size() != 2 || result.rightTensionEnd.yokeElementIds.size() != 2 ||
        result.leftTensionEnd.stabilizerElementId <= 0 || result.rightTensionEnd.stabilizerElementId <= 0 ||
        result.innerSpacers.empty() || structure->m_ModelSets.size() != 8)
    {
        return 4;
    }

    for (const auto& [wireId, sub] : result.subConductors)
    {
        if (sub.nodeIds.empty() || sub.nodeSetId <= 0 || sub.elementSetId <= 0)
        {
            return 5;
        }
    }
    if (result.subConductors.at(0).nodeIds.front() != result.subConductors.at(1).nodeIds.front() ||
        result.subConductors.at(2).nodeIds.front() != result.subConductors.at(3).nodeIds.front() ||
        result.subConductors.at(0).nodeIds.front() == result.subConductors.at(2).nodeIds.front())
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
    if (result.leftSupportNodeId != expectedConductorNodeId++ || result.rightSupportNodeId != expectedConductorNodeId++)
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
            if (elementId != expectedConductorElementId++ || !element || element->m_Role != ElementRole::Conductor ||
                element->m_WireId != wireId || element->m_AeroBundleCount != 4 || element->m_AeroProfileId != wireId ||
                !element->HasAerodynamicLoad())
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
            if (!element || element->m_Role != ElementRole::TensionHardware ||
                std::abs(element->m_InitStress - config.conductor.initialStress) > 1.0e-6 ||
                element->HasAerodynamicLoad())
                return 18;
        }
    }
    for (const auto& spacer : result.innerSpacers)
    {
        for (int elementId : spacer.elementIds)
        {
            const auto element = structure->FindElement(elementId);
            if (!element || element->m_Role != ElementRole::IntraPhaseSpacer ||
                std::abs(element->m_InitStress) > 1.0e-12 || element->HasAerodynamicLoad())
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

    // 文本导线模型的简化端部：四根子导线均直接锚固在端部截面，
    // 不应生成汇集节点、耐张联板或稳定梁。
    auto directStructure = std::make_shared<StructureData>();
    auto directProperty = library.instantiateProperty(0, 0, *directStructure, propertyError);
    if (!directProperty)
        return 22;
    auto directConfig = config;
    directConfig.property = directProperty;
    directConfig.endTopology = Conductor::BundleEndTopology::DirectWireSupports;
    Conductor::ConductorModelBuilder directBuilder(directStructure);
    Conductor::LineBuildResult directResult;
    if (!directBuilder.BuildLine(directConfig, directResult, buildError))
        return 23;
    if (directResult.leftTensionEnd.supportNodeIds.size() != 4 ||
        directResult.rightTensionEnd.supportNodeIds.size() != 4 || !directResult.leftTensionEnd.groupNodeIds.empty() ||
        !directResult.rightTensionEnd.groupNodeIds.empty() || !directResult.leftTensionEnd.yokeElementIds.empty() ||
        !directResult.rightTensionEnd.yokeElementIds.empty() || directResult.leftTensionEnd.stabilizerElementId > 0 ||
        directResult.rightTensionEnd.stabilizerElementId > 0)
        return 24;
    for (const auto& [wireId, sub] : directResult.subConductors)
    {
        if (sub.nodeIds.empty() || sub.nodeIds.front() != directResult.leftTensionEnd.supportNodeIds[wireId] ||
            sub.nodeIds.back() != directResult.rightTensionEnd.supportNodeIds[wireId])
            return 25;
        for (int elementId : sub.elementIds)
        {
            const auto element = directStructure->FindElement(elementId);
            if (!element || element->m_Role != ElementRole::Conductor || !element->HasAerodynamicLoad())
                return 26;
        }
    }

    QTemporaryDir temporaryDirectory;
    const QString h5Path = temporaryDirectory.filePath(QStringLiteral("conductor_tags.h5"));
    Hdf5ModelIO hdf5;
    StructureData restored;
    if (!temporaryDirectory.isValid() ||
        !hdf5.ExportModelHdf5(h5Path, structure.get(), QStringLiteral("verification")) ||
        !hdf5.ImportHdf5(h5Path, &restored) || restored.m_Elements.size() != structure->m_Elements.size())
        return 20;
    for (const auto& [elementId, source] : structure->m_Elements)
    {
        const auto target = restored.FindElement(elementId);
        if (!source || !target || target->m_Role != source->m_Role || target->m_WireId != source->m_WireId ||
            target->m_AeroBundleCount != source->m_AeroBundleCount ||
            target->m_AeroProfileId != source->m_AeroProfileId)
            return 21;
    }

    QTextStream(stdout) << "conductor bundle nodes=" << structure->m_Nodes.size()
                        << " elements=" << structure->m_Elements.size() << " sets=" << structure->m_ModelSets.size()
                        << " supports=" << result.leftSupportNodeId << "," << result.rightSupportNodeId << Qt::endl;
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
    line.conductor.initialStress = 50.0e6;
    line.convergeBundleEnds = true;
    line.bundleEndTransitionLength = 1.0;
    line.setNamePrefix = QStringLiteral("验证多档");

    Conductor::MultiSpanConductorBuildConfig config;
    config.span.line = line;
    config.stationCenters = {Vector3d(0.0, 0.0, 0.0), Vector3d(100.0, 0.0, 0.0), Vector3d(220.0, 0.0, 0.0)};
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
        QTextStream(stderr) << "multispan conductor build failed: " << QString::fromStdString(buildError) << Qt::endl;
        return 3;
    }
    if (result.spanCount != 2 || result.subConductors.size() != 4 || result.suspensionPoints.size() != 1 ||
        result.leftTensionEnd.groupNodeIds.size() != 2 || result.rightTensionEnd.groupNodeIds.size() != 2)
        return 4;

    const auto& suspension = result.suspensionPoints.front();
    if (suspension.wireNodeIds.size() != 4 || suspension.yokeElementIds.size() != 2 ||
        suspension.spacerElementIds.size() != 4 || suspension.stringElementId <= 0 || suspension.supportNodeId <= 0 ||
        suspension.junctionNodeId <= 0)
        return 5;

    const auto junctionIt = structure->m_Nodes.find(suspension.junctionNodeId);
    const auto supportIt = structure->m_Nodes.find(suspension.supportNodeId);
    if (junctionIt == structure->m_Nodes.end() || supportIt == structure->m_Nodes.end() ||
        std::abs(junctionIt->second->m_X - 100.0) > 1.0e-9 || std::abs(junctionIt->second->m_Z - 0.45) > 1.0e-9 ||
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
        if (found == structure->m_Elements.end() || found->second->m_Role != ElementRole::SuspensionHardware ||
            !std::dynamic_pointer_cast<ElementTruss>(found->second))
            return 8;
    }
    for (int elementId : suspension.spacerElementIds)
    {
        const auto found = structure->m_Elements.find(elementId);
        if (found == structure->m_Elements.end() || found->second->m_Role != ElementRole::IntraPhaseSpacer ||
            found->second->HasAerodynamicLoad())
            return 8;
    }
    const auto stringIt = structure->m_Elements.find(suspension.stringElementId);
    if (stringIt == structure->m_Elements.end() || stringIt->second->m_Role != ElementRole::SuspensionHardware ||
        stringIt->second->HasAerodynamicLoad())
        return 9;

    // 多档也必须支持简化直接端部：只有首末站端部不同，中间 H 型悬垂串、
    // 线身与相内间隔棒均保持同一套逻辑。
    auto directStructure = std::make_shared<StructureData>();
    auto directProperty = library.instantiateProperty(0, 0, *directStructure, propertyError);
    if (!directProperty)
        return 14;
    auto directConfig = config;
    directConfig.span.line.property = directProperty;
    directConfig.span.line.endTopology = Conductor::BundleEndTopology::DirectWireSupports;
    directConfig.suspensionProperty = directProperty;
    directConfig.span.innerSpacerLayout.spacer.property = directProperty;
    Conductor::ConductorModelBuilder directBuilder(directStructure);
    Conductor::LineBuildResult directResult;
    if (!directBuilder.BuildMultiSpanConductor(directConfig, directResult, buildError))
        return 15;
    if (directResult.suspensionPoints.size() != 1 || directResult.leftTensionEnd.supportNodeIds.size() != 4 ||
        directResult.rightTensionEnd.supportNodeIds.size() != 4 || !directResult.leftTensionEnd.groupNodeIds.empty() ||
        !directResult.rightTensionEnd.groupNodeIds.empty() || !directResult.leftTensionEnd.yokeElementIds.empty() ||
        !directResult.rightTensionEnd.yokeElementIds.empty())
        return 16;
    for (int wireId = 0; wireId < 4; ++wireId)
    {
        const auto& nodes = directResult.subConductors.at(wireId).nodeIds;
        if (nodes.empty() || nodes.front() != directResult.leftTensionEnd.supportNodeIds[wireId] ||
            nodes.back() != directResult.rightTensionEnd.supportNodeIds[wireId])
            return 17;
    }

    QTemporaryDir temporaryDirectory;
    const QString h5Path = temporaryDirectory.filePath(QStringLiteral("conductor_multispan.h5"));
    Hdf5ModelIO hdf5;
    StructureData restored;
    if (!temporaryDirectory.isValid() ||
        !hdf5.ExportModelHdf5(h5Path, structure.get(), QStringLiteral("multispan verification")) ||
        !hdf5.ImportHdf5(h5Path, &restored))
        return 10;
    const auto restoredString = restored.FindElement(suspension.stringElementId);
    if (!restoredString || restoredString->m_Role != ElementRole::SuspensionHardware ||
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
    if (!uiBuild.succeeded() || uiBuild.structure->m_Constraint.size() != 36)
    {
        QTextStream(stderr) << "multispan conductor UI build failed: " << uiBuild.error
                            << " constraints=" << (uiBuild.structure ? uiBuild.structure->m_Constraint.size() : 0)
                            << Qt::endl;
        return 13;
    }

    QTextStream(stdout) << "conductor multispan spans=" << result.spanCount << " nodes=" << result.NodeCount()
                        << " elements=" << result.ElementCount() << " suspension_center_z=" << suspension.center.z()
                        << " junction_z=" << junctionIt->second->m_Z << " support_z=" << supportIt->second->m_Z
                        << Qt::endl;
    return 0;
}

std::optional<int> verifyConductorSpacerMPC(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-conductor-spacer-mpc")))
        return std::nullopt;

    auto inertiaNode = std::make_shared<Node>();
    inertiaNode->SetNumDOFs(6);
    inertiaNode->m_Velocity = {0.0, 0.0, 0.0, 0.3, -0.2, 0.4};
    inertiaNode->m_Acceleration = {1.0, -2.0, 3.0, -0.5, 0.7, 0.2};
    RigidBodyInertia inertiaVerification;
    inertiaVerification.m_pNode = inertiaNode;
    inertiaVerification.m_Mass = 2.5;
    inertiaVerification.m_RotaryInertia = Vector3d(0.8, 1.1, 1.4).asDiagonal();
    MatrixXd massMatrix;
    VectorXd inertiaForce;
    MatrixXd gyroscopicMatrix;
    MatrixXd configurationTangent;
    inertiaVerification.GetDynamicContributions(massMatrix, inertiaForce, gyroscopicMatrix,
                                                configurationTangent);
    const Vector3d angularVelocity(0.3, -0.2, 0.4);
    const Vector3d angularAcceleration(-0.5, 0.7, 0.2);
    const Vector3d expectedMoment = inertiaVerification.m_RotaryInertia * angularAcceleration +
                                    angularVelocity.cross(inertiaVerification.m_RotaryInertia * angularVelocity);
    if (!inertiaVerification.IsValid() ||
        !massMatrix.block<3, 3>(0, 0).isApprox(2.5 * Matrix3d::Identity(), 1.0e-12) ||
        !massMatrix.block<3, 3>(3, 3).isApprox(inertiaVerification.m_RotaryInertia, 1.0e-12) ||
        !inertiaForce.head<3>().isApprox(2.5 * Vector3d(1.0, -2.0, 3.0), 1.0e-12) ||
        !inertiaForce.tail<3>().isApprox(expectedMoment, 1.0e-12))
        return 8;

    Conductor::PropertyLibrary library;
    QString libraryError;
    if (!library.load(libraryError) || !library.isReady())
    {
        QTextStream(stderr) << "spacer MPC property library failed: " << libraryError << Qt::endl;
        return 1;
    }

    const Vector3d verificationAxis = Vector3d(2.0, -3.0, 4.0).normalized();
    auto twistMaster = std::make_shared<Node>();
    auto twistSlave = std::make_shared<Node>();
    twistMaster->SetNumDOFs(6);
    twistSlave->SetNumDOFs(4);
    for (int direction = 0; direction < 6; ++direction)
        twistMaster->m_DOF[direction] = direction;
    for (int direction = 0; direction < 4; ++direction)
        twistSlave->m_DOF[direction] = direction + 6;
    constexpr double verificationTwist = 0.17;
    twistMaster->m_Rg = Eigen::AngleAxisd(verificationTwist, verificationAxis).toRotationMatrix();
    twistSlave->m_Displacement[3] = verificationTwist;
    AxialTwistTieMPCConstraint twistVerification;
    twistVerification.m_pMasterNode = twistMaster;
    twistVerification.m_pSlaveNode = twistSlave;
    twistVerification.m_Axis = -verificationAxis;
    twistVerification.m_SlaveDirections = {3};
    SolverNameSpace::NonlinearMPCData twistData;
    if (!twistVerification.Evaluate(0, 10, twistData) || std::abs(twistData.value[0]) > 1.0e-12 ||
        (twistData.jacobian.block<1, 3>(0, 3).transpose() + verificationAxis).norm() > 1.0e-12 ||
        std::abs(twistData.jacobian(0, 9) - 1.0) > 1.0e-12)
        return 7;

    struct CaseResult
    {
        std::shared_ptr<StructureData> structure;
        qint64 solveMilliseconds = 0;
        int rawDofs = 0;
        int reducedDofs = 0;
        int spacerElements = 0;
        int spacerInertias = 0;
        double spacerMass = 0.0;
        double maximumMpcGap = 0.0;
        double maximumTwistGap = 0.0;
    };
    constexpr int verificationSegments = 50;
    constexpr int verificationSpacers = 4;
    const auto runCase = [&](Conductor::InnerSpacerStyle style, CaseResult& result) -> bool
    {
        QTemporaryDir outputDirectory;
        ConductorModule module;
        module.setPropertyLibrary(&library);
        auto* styleCombo = module.findChild<QComboBox*>(QStringLiteral("spacerStyleCombo"));
        const int cableIndex = module.elementCombo()->findData(static_cast<int>(EnumKeyword::ElementType::CABLE));
        const int bundleIndex = module.bundleCombo()->findData(4);
        const int styleIndex = styleCombo ? styleCombo->findData(static_cast<int>(style)) : -1;
        if (!outputDirectory.isValid() || cableIndex < 0 || bundleIndex < 0 || styleIndex < 0)
            return false;

        module.elementCombo()->setCurrentIndex(cableIndex);
        module.bundleCombo()->setCurrentIndex(bundleIndex);
        module.startX()->setValue(12.0);
        module.startY()->setValue(-35.0);
        module.startZ()->setValue(18.0);
        module.endX()->setValue(312.0);
        module.endY()->setValue(125.0);
        module.endZ()->setValue(58.0);
        module.segmentsSpin()->setValue(verificationSegments);
        module.innerSpacerCheck()->setChecked(true);
        module.spacerLayoutCombo()->setCurrentIndex(1);
        module.spacerCountSpin()->setValue(verificationSpacers);
        styleCombo->setCurrentIndex(styleIndex);
        module.analysisCheck()->setChecked(true);
        const auto build = module.buildModel(library, outputDirectory.path());
        if (!build.succeeded() || build.structure->m_AnalysisStep.empty())
            return false;

        result.structure = build.structure;
        for (const auto& [elementId, element] : result.structure->m_Elements)
        {
            Q_UNUSED(elementId);
            if (element && element->m_Role == ElementRole::IntraPhaseSpacer)
                ++result.spacerElements;
        }
        for (const auto& [inertiaId, inertia] : result.structure->m_RigidBodyInertias)
        {
            Q_UNUSED(inertiaId);
            if (!inertia || !inertia->IsValid())
                return false;
            ++result.spacerInertias;
            result.spacerMass += inertia->m_Mass;
        }

        const auto step = result.structure->m_AnalysisStep.begin()->second;
        step->SetStructure(result.structure);
        QElapsedTimer timer;
        timer.start();
        if (!step->Solve(false))
            return false;
        result.solveMilliseconds = timer.elapsed();
        for (const auto& [nodeId, node] : result.structure->m_Nodes)
        {
            Q_UNUSED(nodeId);
            if (node)
                result.rawDofs += node->m_DOF.size();
        }
        result.reducedDofs = result.rawDofs;

        for (const auto& [mpcId, constraint] : result.structure->m_MPCConstraints)
        {
            Q_UNUSED(mpcId);
            const auto rigid = std::dynamic_pointer_cast<RigidOffsetMPCConstraint>(constraint);
            const auto twist = std::dynamic_pointer_cast<AxialTwistTieMPCConstraint>(constraint);
            if (twist)
            {
                SolverNameSpace::NonlinearMPCData data;
                if (!twist->Evaluate(0, result.rawDofs, data))
                    return false;
                result.maximumTwistGap = std::max(result.maximumTwistGap, std::abs(data.value[0]));
                --result.reducedDofs;
                continue;
            }
            if (!rigid)
                return false;
            const auto master = rigid->m_pMasterNode.lock();
            const auto slave = rigid->m_pSlaveNode.lock();
            if (!master || !slave)
                return false;
            auto currentPosition = [](const Node& node)
            {
                Vector3d position(node.m_X, node.m_Y, node.m_Z);
                for (int direction = 0; direction < 3 && direction < static_cast<int>(node.m_Displacement.size());
                     ++direction)
                    position[direction] += node.m_Displacement[direction];
                return position;
            };
            const double gap =
                (currentPosition(*slave) - currentPosition(*master) - master->m_Rg * rigid->m_Offset).norm();
            result.maximumMpcGap = std::max(result.maximumMpcGap, gap);
            result.reducedDofs -= 3;
        }
        return true;
    };

    CaseResult beamCase;
    CaseResult mpcCase;
    if (!runCase(Conductor::InnerSpacerStyle::CenterBraced, beamCase))
        return 2;
    if (!runCase(Conductor::InnerSpacerStyle::RigidCenterMPC, mpcCase))
        return 3;
    if (beamCase.spacerElements != 8 * verificationSpacers || beamCase.spacerInertias != 0 ||
        !beamCase.structure->m_MPCConstraints.empty())
        return 4;
    if (mpcCase.spacerElements != 0 || mpcCase.structure->m_MPCConstraints.size() != 8 * verificationSpacers ||
        mpcCase.spacerInertias != verificationSpacers || mpcCase.spacerMass <= 0.0 ||
        mpcCase.maximumMpcGap > 1.0e-7 || mpcCase.maximumTwistGap > 1.0e-7)
        return 5;

    QTemporaryDir hdf5Directory;
    const QString hdf5Path = hdf5Directory.filePath(QStringLiteral("rigid_center_spacer.h5"));
    Hdf5ModelIO hdf5;
    StructureData restored;
    const bool directoryValid = hdf5Directory.isValid();
    const bool exported = directoryValid && hdf5.ExportModelHdf5(
                                                       hdf5Path, mpcCase.structure.get(),
                                                       QStringLiteral("rigid center spacer verification"));
    const bool imported = exported && hdf5.ImportHdf5(hdf5Path, &restored);
    if (!directoryValid || !exported || !imported || restored.m_MPCConstraints.size() != 8 * verificationSpacers ||
        restored.m_RigidBodyInertias.size() != verificationSpacers)
    {
        QTextStream(stderr) << "spacer MPC HDF5 roundtrip failed: directory=" << directoryValid
                            << " exported=" << exported << " imported=" << imported
                            << " source_inertias=" << mpcCase.structure->m_RigidBodyInertias.size()
                            << " restored_mpcs=" << restored.m_MPCConstraints.size()
                            << " restored_inertias=" << restored.m_RigidBodyInertias.size() << Qt::endl;
        return 6;
    }

    QTextStream(stdout) << "conductor spacer comparison beam_elements=" << beamCase.spacerElements
                        << " beam_raw_dofs=" << beamCase.rawDofs << " beam_reduced_dofs=" << beamCase.reducedDofs
                        << " beam_solve_ms=" << beamCase.solveMilliseconds << " mpc_elements=" << mpcCase.spacerElements
                        << " mpc_constraints=" << mpcCase.structure->m_MPCConstraints.size()
                        << " mpc_inertias=" << mpcCase.spacerInertias << " spacer_mass=" << mpcCase.spacerMass
                        << " mpc_raw_dofs=" << mpcCase.rawDofs << " mpc_reduced_dofs=" << mpcCase.reducedDofs
                        << " mpc_solve_ms=" << mpcCase.solveMilliseconds << " max_gap=" << mpcCase.maximumMpcGap
                        << " max_twist_gap=" << mpcCase.maximumTwistGap
                        << Qt::endl;
    return 0;
}

std::optional<int> verifyConductorStatic(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-conductor-static")))
        return std::nullopt;

    Conductor::PropertyLibrary library;
    QString libraryError;
    if (!library.load(libraryError) || !library.isReady())
    {
        QTextStream(stderr) << "conductor property library failed: " << libraryError << Qt::endl;
        return 1;
    }

    const EnumKeyword::ElementType elementTypes[] = {EnumKeyword::ElementType::T3D2, EnumKeyword::ElementType::CABLE,
                                                     EnumKeyword::ElementType::CR3D};
    for (EnumKeyword::ElementType elementType : elementTypes)
    {
        QTemporaryDir outputDirectory;
        ConductorModule module;
        module.setPropertyLibrary(&library);
        const int elementIndex = module.elementCombo()->findData(static_cast<int>(elementType));
        const int bundleIndex = module.bundleCombo()->findData(4);
        const int beamSpacerIndex =
            module.spacerElementCombo()->findData(static_cast<int>(EnumKeyword::ElementType::CR3D));
        if (!outputDirectory.isValid() || elementIndex < 0 || bundleIndex < 0 || beamSpacerIndex < 0)
        {
            return 2;
        }

        module.elementCombo()->setCurrentIndex(elementIndex);
        module.bundleCombo()->setCurrentIndex(bundleIndex);
        // 回归算例覆盖曾出现静力切线奇异的默认界面组合：桁架导线、四个梁式间隔棒和完整 50 等分。
        module.segmentsSpin()->setValue(elementType == EnumKeyword::ElementType::T3D2 ? 50 : 10);
        module.innerSpacerCheck()->setChecked(true);
        module.spacerCountSpin()->setValue(elementType == EnumKeyword::ElementType::T3D2 ? 4 : 2);
        module.spacerElementCombo()->setCurrentIndex(beamSpacerIndex);
        module.analysisCheck()->setChecked(true);

        const auto build = module.buildModel(library, outputDirectory.path());
        if (!build.succeeded() || build.structure->m_AnalysisStep.empty())
        {
            QTextStream(stderr) << "conductor static build failed type=" << static_cast<int>(elementType)
                                << " error=" << build.error << Qt::endl;
            return 3;
        }

        int spacerElementCount = 0;
        int beamSpacerElementCount = 0;
        int conductorElementCount = 0;
        int matchingConductorElementCount = 0;
        for (const auto& [elementId, element] : build.structure->m_Elements)
        {
            Q_UNUSED(elementId);
            if (!element)
                continue;
            if (element->m_Role == ElementRole::Conductor)
            {
                ++conductorElementCount;
                const bool matches = (elementType == EnumKeyword::ElementType::T3D2 &&
                                      std::dynamic_pointer_cast<ElementTruss>(element)) ||
                                     (elementType == EnumKeyword::ElementType::CABLE &&
                                      std::dynamic_pointer_cast<ElementCable>(element)) ||
                                     (elementType == EnumKeyword::ElementType::CR3D &&
                                      std::dynamic_pointer_cast<ElementBeam_CR>(element));
                if (matches)
                    ++matchingConductorElementCount;
            }
            else if (element->m_Role == ElementRole::IntraPhaseSpacer)
            {
                ++spacerElementCount;
                if (std::dynamic_pointer_cast<ElementBeam_CR>(element))
                    ++beamSpacerElementCount;
            }
        }
        if (conductorElementCount == 0 || matchingConductorElementCount != conductorElementCount ||
            spacerElementCount == 0 || beamSpacerElementCount != spacerElementCount ||
            !build.structure->m_MPCConstraints.empty())
        {
            QTextStream(stderr) << "conductor mixed topology invalid type=" << static_cast<int>(elementType)
                                << " conductor_elements=" << conductorElementCount
                                << " matching_conductor_elements=" << matchingConductorElementCount
                                << " spacers=" << spacerElementCount << " beam_spacers=" << beamSpacerElementCount
                                << " mpcs=" << build.structure->m_MPCConstraints.size() << Qt::endl;
            return 5;
        }

        std::shared_ptr<StructureData> solveStructure = build.structure;
        const auto step = solveStructure->m_AnalysisStep.begin()->second;
        step->SetStructure(solveStructure);
        if (!step->Solve(false))
        {
            QTextStream(stderr) << "conductor static solve failed type=" << static_cast<int>(elementType)
                                << " nodes=" << solveStructure->m_Nodes.size()
                                << " elements=" << solveStructure->m_Elements.size()
                                << " constraints=" << solveStructure->m_Constraint.size() << Qt::endl;
            return 4;
        }
        QTextStream(stdout) << "conductor static solved type=" << static_cast<int>(elementType)
                            << " nodes=" << solveStructure->m_Nodes.size()
                            << " elements=" << solveStructure->m_Elements.size()
                            << " conductor_elements=" << conductorElementCount
                            << " matching_conductor_elements=" << matchingConductorElementCount
                            << " constraints=" << solveStructure->m_Constraint.size()
                            << " mpcs=" << solveStructure->m_MPCConstraints.size() << Qt::endl;
    }
    return 0;
}

std::optional<int> verifyConductorGallopingDynamics(const QStringList& arguments)
{
    const int exportIndex = arguments.indexOf(QStringLiteral("--export-conductor-galloping"));
    const bool verifyRequested = arguments.contains(QStringLiteral("--verify-conductor-galloping"));
    if (!verifyRequested && exportIndex < 0)
        return std::nullopt;
    if (exportIndex >= 0 && exportIndex + 1 >= arguments.size())
        return 1;

    const QString exportPath = exportIndex >= 0 ? arguments.at(exportIndex + 1) : QString();
    const auto readDoubleOption = [&arguments](const QString& option, double defaultValue, bool requirePositive,
                                               double& value) -> bool
    {
        value = defaultValue;
        const int index = arguments.indexOf(option);
        if (index < 0)
            return true;
        if (index + 1 >= arguments.size())
            return false;
        bool converted = false;
        const double parsed = arguments.at(index + 1).toDouble(&converted);
        if (!converted || !std::isfinite(parsed) || (requirePositive && parsed <= 0.0))
        {
            return false;
        }
        value = parsed;
        return true;
    };
    double gallopingDuration = 0.02;
    double initialAttackDegrees = 45.0;
    double gallopingIceThicknessValue = 25.0;
    double gallopingBundleValue = 4.0;
    double gallopingSegmentsValue = 50.0;
    double gallopingMaximumTimeStep = 0.005;
    if (!readDoubleOption(QStringLiteral("--galloping-duration"), 0.02, true, gallopingDuration) ||
        !readDoubleOption(QStringLiteral("--galloping-attack"), 45.0, false, initialAttackDegrees) ||
        !readDoubleOption(QStringLiteral("--galloping-ice"), 25.0, true, gallopingIceThicknessValue) ||
        !readDoubleOption(QStringLiteral("--galloping-bundle"), 4.0, true, gallopingBundleValue) ||
        !readDoubleOption(QStringLiteral("--galloping-segments"), 50.0, true, gallopingSegmentsValue) ||
        !readDoubleOption(QStringLiteral("--galloping-max-step"), 0.005, true, gallopingMaximumTimeStep))
    {
        QTextStream(stderr) << "invalid galloping duration, attack, ice, bundle, segments, or maximum step" << Qt::endl;
        return 1;
    }
    const int gallopingBundleCount = static_cast<int>(std::llround(gallopingBundleValue));
    if (std::abs(gallopingBundleValue - gallopingBundleCount) > 1.0e-9 ||
        !AeroManager::isSupportedCase(AeroCaseKey{gallopingBundleCount, 14, 25}))
    {
        QTextStream(stderr) << "unsupported --galloping-bundle: " << gallopingBundleValue << Qt::endl;
        return 1;
    }
    const int gallopingIceThickness = static_cast<int>(std::llround(gallopingIceThicknessValue));
    if (std::abs(gallopingIceThicknessValue - gallopingIceThickness) > 1.0e-9 ||
        !AeroManager::isSupportedIceThickness(gallopingIceThickness))
    {
        QTextStream(stderr) << "unsupported --galloping-ice thickness: " << gallopingIceThicknessValue << Qt::endl;
        return 1;
    }
    QString historyPath;
    const int historyIndex = arguments.indexOf(QStringLiteral("--galloping-history"));
    if (historyIndex >= 0)
    {
        if (historyIndex + 1 >= arguments.size())
            return 1;
        historyPath = arguments.at(historyIndex + 1);
    }
    double historyInterval = 0.1;
    if (!readDoubleOption(QStringLiteral("--galloping-history-interval"), 0.1, true, historyInterval))
    {
        QTextStream(stderr) << "invalid --galloping-history-interval" << Qt::endl;
        return 1;
    }
    double tssbnSpectralRadius = 0.0;
    if (!readDoubleOption(QStringLiteral("--galloping-tssbn-rho"), 0.0, false, tssbnSpectralRadius) ||
        tssbnSpectralRadius < 0.0 || tssbnSpectralRadius > 1.0)
    {
        QTextStream(stderr) << "invalid --galloping-tssbn-rho (must be in [0, 1])" << Qt::endl;
        return 1;
    }
    const int gallopingSegments = static_cast<int>(std::llround(gallopingSegmentsValue));
    if (std::abs(gallopingSegmentsValue - gallopingSegments) > 1.0e-9 || gallopingSegments < 4)
    {
        QTextStream(stderr) << "--galloping-segments must be an integer no smaller than 4" << Qt::endl;
        return 1;
    }
    QString aerodynamicTangentModeText = QStringLiteral("every-newton");
    int tangentModeIndex = arguments.indexOf(QStringLiteral("--galloping-aero-tangent"));
    if (tangentModeIndex >= 0)
    {
        if (tangentModeIndex + 1 >= arguments.size())
            return 1;
        aerodynamicTangentModeText = arguments.at(tangentModeIndex + 1).trimmed().toLower();
    }
    SolverNameSpace::AerodynamicTangentMode aerodynamicTangentMode;
    if (aerodynamicTangentModeText == QStringLiteral("off"))
        aerodynamicTangentMode = SolverNameSpace::AerodynamicTangentMode::Disabled;
    else if (aerodynamicTangentModeText == QStringLiteral("step") ||
             aerodynamicTangentModeText == QStringLiteral("stage"))
        aerodynamicTangentMode = SolverNameSpace::AerodynamicTangentMode::OncePerTimeStepOrStage;
    else if (aerodynamicTangentModeText == QStringLiteral("every-newton"))
        aerodynamicTangentMode = SolverNameSpace::AerodynamicTangentMode::EveryNewtonIteration;
    else
    {
        QTextStream(stderr) << "invalid --galloping-aero-tangent (off|step|every-newton)" << Qt::endl;
        return 1;
    }
    QString gallopingSolver = QStringLiteral("both");
    const int solverOptionIndex = arguments.indexOf(QStringLiteral("--galloping-solver"));
    if (solverOptionIndex >= 0)
    {
        if (solverOptionIndex + 1 >= arguments.size())
            return 1;
        gallopingSolver = arguments.at(solverOptionIndex + 1).trimmed().toLower();
    }
    const bool runNewmark = gallopingSolver == QStringLiteral("both") || gallopingSolver == QStringLiteral("newmark") ||
                            gallopingSolver == QStringLiteral("newmark-pair");
    const bool runAdaptiveNewmark = gallopingSolver == QStringLiteral("adaptive-newmark") ||
                                    gallopingSolver == QStringLiteral("newmark-pair");
    const bool runTssbn = gallopingSolver == QStringLiteral("both") || gallopingSolver == QStringLiteral("tssbn");
    const bool disableAerodynamicLoad = arguments.contains(QStringLiteral("--galloping-disable-aero"));
    const bool enableStructuralDamping = arguments.contains(QStringLiteral("--galloping-structural-damping"));
    const bool streamResult = arguments.contains(QStringLiteral("--galloping-stream-result"));
    const bool exportOnly = arguments.contains(QStringLiteral("--galloping-export-only"));
    if (!runNewmark && !runAdaptiveNewmark && !runTssbn)
    {
        QTextStream(stderr) << "invalid --galloping-solver (both|newmark|adaptive-newmark|newmark-pair|tssbn)"
                            << Qt::endl;
        return 1;
    }

    QString gallopingWindDirection = QStringLiteral("-y");
    const int windDirectionIndex = arguments.indexOf(QStringLiteral("--galloping-wind-direction"));
    if (windDirectionIndex >= 0)
    {
        if (windDirectionIndex + 1 >= arguments.size())
            return 1;
        gallopingWindDirection = arguments.at(windDirectionIndex + 1).trimmed().toLower();
    }
    Eigen::Vector3d windDirection = Eigen::Vector3d::Zero();
    if (gallopingWindDirection == QStringLiteral("+y"))
        windDirection = Eigen::Vector3d::UnitY();
    else if (gallopingWindDirection == QStringLiteral("-y"))
        windDirection = -Eigen::Vector3d::UnitY();
    else if (gallopingWindDirection == QStringLiteral("+z"))
        windDirection = Eigen::Vector3d::UnitZ();
    else if (gallopingWindDirection == QStringLiteral("-z"))
        windDirection = -Eigen::Vector3d::UnitZ();
    else
    {
        QTextStream(stderr) << "invalid --galloping-wind-direction (+y|-y|+z|-z)" << Qt::endl;
        return 1;
    }

    QString spacerMode = QStringLiteral("beam");
    const int spacerModeIndex = arguments.indexOf(QStringLiteral("--galloping-spacer-mode"));
    if (spacerModeIndex >= 0)
    {
        if (spacerModeIndex + 1 >= arguments.size())
            return 1;
        spacerMode = arguments.at(spacerModeIndex + 1).trimmed().toLower();
    }
    const Conductor::InnerSpacerStyle spacerStyle =
        spacerMode == QStringLiteral("beam")    ? Conductor::InnerSpacerStyle::CenterBraced
        : spacerMode == QStringLiteral("outer") ? Conductor::InnerSpacerStyle::OuterPolygon
        : spacerMode == QStringLiteral("mpc")   ? Conductor::InnerSpacerStyle::RigidCenterMPC
                                                : Conductor::InnerSpacerStyle::OuterPolygon;
    if (spacerMode != QStringLiteral("beam") && spacerMode != QStringLiteral("outer") &&
        spacerMode != QStringLiteral("mpc"))
    {
        QTextStream(stderr) << "invalid --galloping-spacer-mode (beam|outer|mpc)" << Qt::endl;
        return 1;
    }

    // 关联动力分支继承其静力来源和自身定义，不能仅因其他同级动力分支编号较小就继承其定义。
    AnalysisStep branchScopeProbe;
    branchScopeProbe.m_Id = 3;
    branchScopeProbe.m_Type = EnumKeyword::StepType::DYNAMIC;
    branchScopeProbe.m_InitialStaticStepId = 1;
    if (!branchScopeProbe.IsStepScopedDataActive(0) || !branchScopeProbe.IsStepScopedDataActive(1) ||
        branchScopeProbe.IsStepScopedDataActive(2) || !branchScopeProbe.IsStepScopedDataActive(3) ||
        branchScopeProbe.IsStepScopedDataActive(4))
    {
        QTextStream(stderr) << "dynamic branch step scope isolation failed" << Qt::endl;
        return 2;
    }

    Conductor::PropertyLibrary library;
    QString libraryError;
    if (!library.load(libraryError) || !library.isReady())
    {
        QTextStream(stderr) << "conductor property library failed: " << libraryError << Qt::endl;
        return 1;
    }

    const auto solve = [&library, &exportPath, gallopingDuration, initialAttackDegrees, gallopingIceThickness,
                        gallopingBundleCount, gallopingSegments, gallopingMaximumTimeStep, runNewmark,
                        runAdaptiveNewmark, runTssbn,
                        spacerStyle,
                        windDirection, &historyPath, historyInterval,
                         tssbnSpectralRadius, aerodynamicTangentMode, disableAerodynamicLoad,
                         enableStructuralDamping, streamResult, exportOnly](SolverNameSpace::SolverType solverType,
                                              double& maximumDisplacement,
                                             double& inheritanceGap, double& dynamicMilliseconds, double& maximumMpcGap,
                                             int& acceptedSteps, double& averageAcceptedStep, int& maximumIterations)
    {
        QTemporaryDir outputDirectory;
        ConductorModule module;
        module.setPropertyLibrary(&library);
        const int cableIndex = module.elementCombo()->findData(static_cast<int>(EnumKeyword::ElementType::CABLE));
        const int bundleIndex = module.bundleCombo()->findData(gallopingBundleCount);
        auto* spacerStyleCombo = module.findChild<QComboBox*>(QStringLiteral("spacerStyleCombo"));
        auto* endTopologyCombo = module.findChild<QComboBox*>(QStringLiteral("endTopologyCombo"));
        const int spacerStyleIndex = spacerStyleCombo ? spacerStyleCombo->findData(static_cast<int>(spacerStyle)) : -1;
        const int directEndIndex =
            endTopologyCombo
                ? endTopologyCombo->findData(static_cast<int>(Conductor::BundleEndTopology::DirectWireSupports))
                : -1;
        if (!outputDirectory.isValid() || cableIndex < 0 || bundleIndex < 0 || spacerStyleIndex < 0 ||
            directEndIndex < 0)
        {
            QTextStream(stderr) << "conductor galloping setup failed" << Qt::endl;
            return false;
        }

        module.elementCombo()->setCurrentIndex(cableIndex);
        module.bundleCombo()->setCurrentIndex(bundleIndex);
        module.segmentsSpin()->setValue(gallopingSegments);
        // 动力拓扑比较采用四个不同的等距位置，保证两种模式使用相同间隔棒位置。
        module.spacerLayoutCombo()->setCurrentIndex(1);
        module.spacerCountSpin()->setValue(4);
        spacerStyleCombo->setCurrentIndex(spacerStyleIndex);
        endTopologyCombo->setCurrentIndex(directEndIndex);
        module.analysisCheck()->setChecked(true);
        const auto build = module.buildModel(library, outputDirectory.path());
        if (!build.succeeded() || build.structure->m_AnalysisStep.empty())
        {
            QTextStream(stderr) << "conductor galloping build failed: " << build.error << Qt::endl;
            return false;
        }

        const auto structure = build.structure;
        const int staticStepId = structure->m_AnalysisStep.begin()->first;
        // 保留生成的 0.01 静力荷载增量。刚性 MPC 方案具有相同平衡路径，但对旧版短验证采用的
        // 0.05 粗增量更敏感。
        AnalysisStepConfig dynamicConfig;
        dynamicConfig.id = structure->m_AnalysisStep.rbegin()->first + 1;
        dynamicConfig.name = QStringLiteral("verification dynamics");
        dynamicConfig.type = EnumKeyword::StepType::DYNAMIC;
        dynamicConfig.totalTime = gallopingDuration;
        dynamicConfig.stepSize = 0.005;
        dynamicConfig.tolerance = 1.0e-5;
        dynamicConfig.maxIterations = 48;
        dynamicConfig.dynamicSolverType = solverType;
        dynamicConfig.initialStaticStepId = staticStepId;
        dynamicConfig.enableGalloping = !disableAerodynamicLoad;
        dynamicConfig.structuralDamping.enabled = enableStructuralDamping;
        dynamicConfig.structuralDamping.translationDampingRatio = 0.005;
        dynamicConfig.structuralDamping.torsionDampingRatio = 0.038;
        dynamicConfig.structuralDamping.maximumFrequencyHz = 3.0;
        dynamicConfig.gallopingIceThickness = gallopingIceThickness;
        dynamicConfig.gallopingInitialAttackDegrees = initialAttackDegrees;
        dynamicConfig.adaptiveTssbn.minimumTimeStep = 1.0e-5;
        dynamicConfig.adaptiveTssbn.maximumTimeStep = gallopingMaximumTimeStep;
        // 与 D:\VS\TSSBN\Wind_method\AnalysisStep.h（SaTSSBNParams）保持一致，
        // 使舞动比较只改变控制器参数。
        dynamicConfig.adaptiveTssbn.relativeTolerance = 5.0e-4;
        dynamicConfig.adaptiveTssbn.absoluteTolerance = 1.0e-6;
        dynamicConfig.adaptiveTssbn.targetNewtonIterations = 8;
        dynamicConfig.adaptiveTssbn.spectralRadiusInfinity = tssbnSpectralRadius;
        dynamicConfig.gallopingAerodynamicTangentMode = aerodynamicTangentMode;
        structure->AddAnalysisStep(dynamicConfig);

        auto wind = std::make_shared<Force_Wind>();
        wind->m_Id = structure->m_Load.empty() ? 1 : structure->m_Load.rbegin()->first + 1;
        wind->m_Name = QStringLiteral("verification galloping wind");
        wind->m_StepId = dynamicConfig.id;
        wind->m_velocity = 14.0;
        wind->m_direction = windDirection;
        wind->m_windDensity = 1.225;
        if (!disableAerodynamicLoad)
            structure->m_Load.emplace(wind->m_Id, wind);

        // BDF 不含导线气动标签和自适应 TSSBN 设置字段，因此求解前先导出 HDF5 配置模型，
        // 保证重新打开时不丢失舞动定义。
        const bool exportThisSolver =
            !exportPath.isEmpty() &&
            ((static_cast<int>(runNewmark) + static_cast<int>(runAdaptiveNewmark) + static_cast<int>(runTssbn) == 1) ||
             solverType == SolverNameSpace::SolverType::AdaptiveTSSBN);
        if (exportThisSolver)
        {
            Hdf5ModelIO hdf5;
            if (!hdf5.ExportModelHdf5(exportPath, structure.get(),
                                      QStringLiteral("ice galloping conductor - adaptive TSSBN")))
                return false;
        }
        if (exportOnly)
        {
            maximumDisplacement = 1.0;
            inheritanceGap = 0.0;
            return true;
        }

        QString dynamicResultPath = outputDirectory.filePath(QStringLiteral("verification_chain.h5"));
        if (exportThisSolver)
        {
            const QFileInfo modelFile(exportPath);
            dynamicResultPath = modelFile.dir().filePath(modelFile.completeBaseName() + QStringLiteral("_result.h5"));
        }
        structure->m_OutputControl.m_Hdf5FileName = dynamicResultPath;
        structure->m_OutputControl.m_StreamResult = streamResult;

        // 独立求解平衡分析步的参考副本。后续实际动力任务必须在 t=0 帧复现该收敛状态，
        // 但不能把静力帧写入动力 H5。
        QString referenceCloneError;
        auto equilibriumReference = structure->CloneForAnalysis(&referenceCloneError);
        if (!equilibriumReference)
        {
            QTextStream(stderr) << "conductor equilibrium clone failed: " << referenceCloneError << Qt::endl;
            return false;
        }
        equilibriumReference->m_OutputControl.m_Hdf5FileName =
            outputDirectory.filePath(QStringLiteral("verification_static_reference.h5"));
        const auto equilibriumStep = equilibriumReference->m_AnalysisStep.at(staticStepId);
        equilibriumStep->SetStructure(equilibriumReference);
        if (!equilibriumStep->Solve(false))
        {
            QTextStream(stderr) << "conductor equilibrium reference solve failed" << Qt::endl;
            return false;
        }
        const DataFrame* finalStaticFrame = nullptr;
        for (const DataFrame& frame : equilibriumReference->GetOutputter().GetFrames())
        {
            if (frame.GetStepId() == staticStepId &&
                (!finalStaticFrame || frame.GetTime() > finalStaticFrame->GetTime()))
            {
                finalStaticFrame = &frame;
            }
        }
        if (!finalStaticFrame)
        {
            QTextStream(stderr) << "conductor equilibrium reference has no frame" << Qt::endl;
            return false;
        }

        // 独立平衡结果不能写入指定的动力结果文件，否则静力写入器会先创建 H5，后续动力副本可能
        // 留下仅含分析步 1 的文件，误导长时间舞动检查。
        structure->m_OutputControl.m_Hdf5FileName =
            outputDirectory.filePath(QStringLiteral("verification_standalone_static.h5"));
        const auto staticStep = structure->m_AnalysisStep.at(staticStepId);
        staticStep->SetStructure(structure);
        if (!staticStep->Solve(false))
        {
            QTextStream(stderr) << "conductor standalone equilibrium task failed"
                                << " solver=" << static_cast<int>(solverType) << Qt::endl;
            return false;
        }
        QString inheritedCloneError;
        auto dynamicStructure = structure->CloneForAnalysis(&inheritedCloneError);
        if (!dynamicStructure)
        {
            QTextStream(stderr) << "conductor dynamic clone failed: " << inheritedCloneError << Qt::endl;
            return false;
        }
        dynamicStructure->GetOutputter().Clear();
        dynamicStructure->m_OutputControl.m_Hdf5FileName = dynamicResultPath;
        QString solverHistoryPath = historyPath;
        if (!historyPath.isEmpty() &&
            static_cast<int>(runNewmark) + static_cast<int>(runAdaptiveNewmark) + static_cast<int>(runTssbn) > 1)
        {
            const QFileInfo historyInfo(historyPath);
            const QString suffix = solverType == SolverNameSpace::SolverType::Newmark
                                       ? QStringLiteral("_newmark")
                                   : solverType == SolverNameSpace::SolverType::AdaptiveNewmark
                                       ? QStringLiteral("_adaptive_newmark")
                                       : QStringLiteral("_tssbn");
            solverHistoryPath = historyInfo.dir().filePath(
                historyInfo.completeBaseName() + suffix +
                (historyInfo.suffix().isEmpty() ? QStringLiteral(".csv") : QStringLiteral(".") + historyInfo.suffix()));
        }

        std::shared_ptr<Node> historyNode;
        if (!solverHistoryPath.isEmpty())
        {
            double minimumX = std::numeric_limits<double>::infinity();
            double maximumX = -std::numeric_limits<double>::infinity();
            for (const auto& [nodeId, node] : dynamicStructure->m_Nodes)
            {
                Q_UNUSED(nodeId);
                if (node)
                {
                    minimumX = std::min(minimumX, node->m_X);
                    maximumX = std::max(maximumX, node->m_X);
                }
            }
            const double middleX = 0.5 * (minimumX + maximumX);
            double bestDistance = std::numeric_limits<double>::infinity();
            for (const auto& [nodeId, node] : dynamicStructure->m_Nodes)
            {
                Q_UNUSED(nodeId);
                if (node && std::abs(node->m_X - middleX) < bestDistance)
                {
                    bestDistance = std::abs(node->m_X - middleX);
                    historyNode = node;
                }
            }
            if (!historyNode)
                return false;
        }

        QFile historyFile(solverHistoryPath);
        std::unique_ptr<QTextStream> historyStream;
        double nextHistoryTime = 0.0;
        if (historyNode)
        {
            QDir().mkpath(QFileInfo(solverHistoryPath).absolutePath());
            if (!historyFile.open(QIODevice::WriteOnly | QIODevice::Text))
                return false;
            historyStream = std::make_unique<QTextStream>(&historyFile);
            *historyStream << "time,node_id,ux,uy,uz,twist,twist_rate\n";
            const auto displacement = [&historyNode](int component)
            {
                return component < static_cast<int>(historyNode->m_Displacement.size())
                           ? historyNode->m_Displacement[component]
                           : 0.0;
            };
            const double twist = historyNode->m_Displacement.size() > 3 ? historyNode->m_Displacement[3] : 0.0;
            const double twistRate = historyNode->m_Velocity.size() > 3 ? historyNode->m_Velocity[3] : 0.0;
            *historyStream << 0.0 << ',' << historyNode->m_Id << ',' << displacement(0) << ',' << displacement(1) << ','
                           << displacement(2) << ',' << twist << ',' << twistRate << '\n';
            nextHistoryTime = historyInterval;
        }

        double lightweightInheritanceGap = 0.0;
        if (historyNode)
        {
            for (const auto& [nodeId, nodeData] : finalStaticFrame->GetNodeDatas())
            {
                Q_UNUSED(nodeData);
                const auto nodeIt = dynamicStructure->m_Nodes.find(nodeId);
                if (nodeIt == dynamicStructure->m_Nodes.cend() || !nodeIt->second)
                    continue;
                for (int component = 0; component < 3; ++component)
                {
                    const double current = component < static_cast<int>(nodeIt->second->m_Displacement.size())
                                               ? nodeIt->second->m_Displacement[component]
                                               : 0.0;
                    const auto resultType = static_cast<EnumKeyword::NodeResultType>(
                        static_cast<int>(EnumKeyword::NodeResultType::U1) + component);
                    lightweightInheritanceGap =
                        std::max(lightweightInheritanceGap,
                                 std::abs(finalStaticFrame->GetNodeData(nodeId, resultType) - current));
                }
            }
        }

        bool dynamicSolved = false;
        QElapsedTimer dynamicTimer;
        dynamicTimer.start();
        if (historyNode)
        {
            // 长时间验证保留原积分时间步，但只采样一个跨中节点，不改变正式求解器、单元、荷载或模型输出行为。
            dynamicStructure->m_OutputControl.m_StreamResult = true;
            const auto dynamicStep = dynamicStructure->m_AnalysisStep.at(dynamicConfig.id);
            dynamicStep->SetStructure(dynamicStructure);
            dynamicStep->SetInitializeFromCurrentState(true);
            dynamicStep->SetRuntimeCallbacks(
                [&](double progress, const QString& status)
                {
                    const double currentTime = progress * gallopingDuration;
                    const auto displacement = [&historyNode](int component)
                    {
                        return component < static_cast<int>(historyNode->m_Displacement.size())
                                   ? historyNode->m_Displacement[component]
                                   : 0.0;
                    };
                    maximumDisplacement = std::max(maximumDisplacement, std::sqrt(displacement(0) * displacement(0) +
                                                                                  displacement(1) * displacement(1) +
                                                                                  displacement(2) * displacement(2)));
                    if (currentTime + 1.0e-10 >= nextHistoryTime)
                    {
                        const double twist =
                            historyNode->m_Displacement.size() > 3 ? historyNode->m_Displacement[3] : 0.0;
                        const double twistRate = historyNode->m_Velocity.size() > 3 ? historyNode->m_Velocity[3] : 0.0;
                        *historyStream << currentTime << ',' << historyNode->m_Id << ',' << displacement(0) << ','
                                       << displacement(1) << ',' << displacement(2) << ',' << twist << ',' << twistRate
                                       << '\n';
                        nextHistoryTime += historyInterval;
                    }
                    if (!status.isEmpty() && status.contains(QStringLiteral("失败")))
                        QTextStream(stderr) << status << Qt::endl;
                },
                []()
                {
                    return false;
                });
            // 导出验证模型时，在采样轻量 CSV 历程的同时生成对应动力 H5，覆盖与界面相同的流式写入和结束路径。
            dynamicSolved = dynamicStep->Solve(exportThisSolver);
            dynamicStep->ClearRuntimeCallbacks();
            historyStream->flush();
        }
        else
        {
            AnalysisRunner dynamicRunner;
            dynamicRunner.SetStructure(dynamicStructure);
            dynamicRunner.SetRuntimeCallbacks({},
                                              []()
                                              {
                                                  return false;
                                              });
            dynamicSolved = dynamicRunner.RunStepFromCurrentState(dynamicConfig.id);
        }
        if (!dynamicSolved)
        {
            double maximumTwistResidual = 0.0;
            double minimumRelativeQuaternionScalar = 1.0;
            double maximumStepRotation = 0.0;
            double minimumAeroKnotDistance = 180.0;
            double minimumAttackAngle = 360.0;
            double maximumAttackAngle = 0.0;
            for (const auto& [mpcId, constraint] : dynamicStructure->m_MPCConstraints)
            {
                Q_UNUSED(mpcId);
                const auto twist = std::dynamic_pointer_cast<AxialTwistTieMPCConstraint>(constraint);
                if (!twist)
                    continue;
                const auto master = twist->m_pMasterNode.lock();
                const auto slave = twist->m_pSlaveNode.lock();
                if (!master || !slave || twist->m_SlaveDirection >= static_cast<int>(slave->m_Displacement.size()))
                    continue;
                const double previousSlaveTwist =
                    twist->m_SlaveDirection < static_cast<int>(slave->m_Displacement_n.size())
                        ? slave->m_Displacement_n[twist->m_SlaveDirection]
                        : 0.0;
                const Eigen::Matrix3d relativeRotation = master->m_Rg * master->m_Rg_n.transpose();
                const double twistResidual = slave->m_Displacement[twist->m_SlaveDirection] - previousSlaveTwist -
                                             Utility::CR::ExtractAxialTwist(relativeRotation, twist->m_Axis);
                maximumTwistResidual = std::max(maximumTwistResidual, std::abs(twistResidual));
                const Eigen::Quaterniond quaternion(relativeRotation);
                minimumRelativeQuaternionScalar =
                    std::min(minimumRelativeQuaternionScalar, std::abs(quaternion.w()));
                Vector3d rotation;
                Utility::CR::Extract_RotationVector(relativeRotation, rotation);
                maximumStepRotation = std::max(maximumStepRotation, rotation.norm());
            }
            const Force_Wind* dynamicWind = nullptr;
            for (const auto& [loadId, load] : dynamicStructure->m_Load)
            {
                Q_UNUSED(loadId);
                const auto wind = std::dynamic_pointer_cast<Force_Wind>(load);
                if (wind && wind->m_StepId == dynamicConfig.id)
                {
                    dynamicWind = wind.get();
                    break;
                }
            }
            if (dynamicWind)
            {
                for (const auto& [elementId, element] : dynamicStructure->m_Elements)
                {
                    Q_UNUSED(elementId);
                    const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
                    if (!cable || !cable->HasAerodynamicLoad() || cable->m_pNode.size() != 2)
                        continue;
                    const auto first = cable->m_pNode[0].lock();
                    const auto second = cable->m_pNode[1].lock();
                    const auto property = cable->m_pProperty.lock();
                    const auto section = property ? property->m_pSection.lock() : nullptr;
                    if (!first || !second || !section)
                        continue;
                    AerodynamicSectionState state;
                    state.firstPosition = Vector3d(first->m_X + first->m_Displacement[0],
                                                   first->m_Y + first->m_Displacement[1],
                                                   first->m_Z + first->m_Displacement[2]);
                    state.secondPosition = Vector3d(second->m_X + second->m_Displacement[0],
                                                    second->m_Y + second->m_Displacement[1],
                                                    second->m_Z + second->m_Displacement[2]);
                    state.firstVelocity = Vector3d(first->m_Velocity[0], first->m_Velocity[1], first->m_Velocity[2]);
                    state.secondVelocity =
                        Vector3d(second->m_Velocity[0], second->m_Velocity[1], second->m_Velocity[2]);
                    state.windVelocity = dynamicWind->GetWindVelocityGlobal();
                    state.radius = section->m_Radius;
                    state.initialAttack = initialAttackDegrees * std::acos(-1.0) / 180.0;
                    state.firstTwist = cable->GetNodalTwist(0);
                    state.secondTwist = cable->GetNodalTwist(1);
                    state.firstTwistRate = cable->GetNodalTwistRate(0);
                    state.secondTwistRate = cable->GetNodalTwistRate(1);
                    const AerodynamicSectionResult aero = AerodynamicLoadCalculator::ComputeKinematics(state);
                    const double attackDegrees = AeroManager::normalizeAngleDegrees(
                        aero.attackAngle * 180.0 / std::acos(-1.0));
                    minimumAttackAngle = std::min(minimumAttackAngle, attackDegrees);
                    maximumAttackAngle = std::max(maximumAttackAngle, attackDegrees);
                    const double knotDistance = std::abs(attackDegrees - 5.0 * std::round(attackDegrees / 5.0));
                    minimumAeroKnotDistance = std::min(minimumAeroKnotDistance, knotDistance);
                }
            }
            QTextStream(stderr) << "conductor dynamic task failed after equilibrium inheritance"
                                << " solver=" << static_cast<int>(solverType)
                                << " regions=" << dynamicStructure->m_ComputeRegions.size()
                                << " nodes=" << dynamicStructure->m_Nodes.size()
                                << " elements=" << dynamicStructure->m_Elements.size()
                                << " max_twist_residual=" << maximumTwistResidual
                                << " min_relative_quaternion_scalar=" << minimumRelativeQuaternionScalar
                                << " max_step_rotation=" << maximumStepRotation
                                << " min_aero_knot_distance_deg=" << minimumAeroKnotDistance
                                << " attack_range_deg=[" << minimumAttackAngle << ',' << maximumAttackAngle << ']'
                                << Qt::endl;
            return false;
        }
        dynamicMilliseconds = dynamicTimer.elapsed();
        acceptedSteps = 0;
        maximumIterations = 0;
        for (const SolverIterationRecord& record : dynamicStructure->GetOutputter().GetSolverIterationRecords())
        {
            if (record.stepId != dynamicConfig.id)
                continue;
            ++acceptedSteps;
            maximumIterations = std::max(maximumIterations, record.iterations);
        }
        averageAcceptedStep = acceptedSteps > 0 ? gallopingDuration / static_cast<double>(acceptedSteps) : 0.0;
        maximumMpcGap = 0.0;
        for (const auto& [mpcId, constraint] : dynamicStructure->m_MPCConstraints)
        {
            Q_UNUSED(mpcId);
            const auto rigid = std::dynamic_pointer_cast<RigidOffsetMPCConstraint>(constraint);
            if (!rigid)
                continue;
            const auto master = rigid->m_pMasterNode.lock();
            const auto slave = rigid->m_pSlaveNode.lock();
            if (!master || !slave)
                return false;
            const auto currentPosition = [](const Node& node)
            {
                Vector3d position(node.m_X, node.m_Y, node.m_Z);
                for (int direction = 0; direction < 3 && direction < static_cast<int>(node.m_Displacement.size());
                     ++direction)
                    position[direction] += node.m_Displacement[direction];
                return position;
            };
            maximumMpcGap =
                std::max(maximumMpcGap,
                         (currentPosition(*slave) - currentPosition(*master) - master->m_Rg * rigid->m_Offset).norm());
        }
        if (!historyNode)
            maximumDisplacement = 0.0;
        inheritanceGap = std::numeric_limits<double>::infinity();
        const DataFrame* initialDynamicFrame = nullptr;
        bool publishedStaticFrame = false;
        for (const DataFrame& frame : dynamicStructure->GetOutputter().GetFrames())
        {
            publishedStaticFrame = publishedStaticFrame || frame.GetStepId() == staticStepId;
            if (frame.GetStepId() != dynamicConfig.id)
                continue;
            if (!initialDynamicFrame || frame.GetTime() < initialDynamicFrame->GetTime())
                initialDynamicFrame = &frame;
            for (const auto& [nodeId, nodeData] : frame.GetNodeDatas())
            {
                Q_UNUSED(nodeData);
                const double u1 = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U1);
                const double u2 = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U2);
                const double u3 = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U3);
                maximumDisplacement = std::max(maximumDisplacement, std::sqrt(u1 * u1 + u2 * u2 + u3 * u3));
            }
        }
        if (historyNode)
        {
            inheritanceGap = lightweightInheritanceGap;
        }
        else if (!publishedStaticFrame && initialDynamicFrame)
        {
            inheritanceGap = 0.0;
            for (const auto& [nodeId, nodeData] : finalStaticFrame->GetNodeDatas())
            {
                Q_UNUSED(nodeData);
                if (initialDynamicFrame->GetNodeDatas().find(nodeId) == initialDynamicFrame->GetNodeDatas().cend())
                    continue;
                for (const auto component : {EnumKeyword::NodeResultType::U1, EnumKeyword::NodeResultType::U2,
                                             EnumKeyword::NodeResultType::U3})
                {
                    inheritanceGap =
                        std::max(inheritanceGap, std::abs(finalStaticFrame->GetNodeData(nodeId, component) -
                                                          initialDynamicFrame->GetNodeData(nodeId, component)));
                }
            }
        }
        return std::isfinite(maximumDisplacement) && std::isfinite(inheritanceGap) && inheritanceGap <= 1.0e-10;
    };

    double newmarkMaximumDisplacement = 0.0;
    double adaptiveNewmarkMaximumDisplacement = 0.0;
    double tssbnMaximumDisplacement = 0.0;
    double newmarkInheritanceGap = 0.0;
    double adaptiveNewmarkInheritanceGap = 0.0;
    double tssbnInheritanceGap = 0.0;
    double newmarkDynamicMilliseconds = 0.0;
    double adaptiveNewmarkDynamicMilliseconds = 0.0;
    double tssbnDynamicMilliseconds = 0.0;
    double newmarkMpcGap = 0.0;
    double adaptiveNewmarkMpcGap = 0.0;
    double tssbnMpcGap = 0.0;
    int newmarkAcceptedSteps = 0;
    int adaptiveNewmarkAcceptedSteps = 0;
    int tssbnAcceptedSteps = 0;
    double newmarkAverageStep = 0.0;
    double adaptiveNewmarkAverageStep = 0.0;
    double tssbnAverageStep = 0.0;
    int newmarkMaximumIterations = 0;
    int adaptiveNewmarkMaximumIterations = 0;
    int tssbnMaximumIterations = 0;
    const bool newmarkSolved = !runNewmark || solve(SolverNameSpace::SolverType::Newmark, newmarkMaximumDisplacement,
                                                    newmarkInheritanceGap, newmarkDynamicMilliseconds, newmarkMpcGap,
                                                    newmarkAcceptedSteps, newmarkAverageStep, newmarkMaximumIterations);
    const bool adaptiveNewmarkSolved =
        !runAdaptiveNewmark ||
        solve(SolverNameSpace::SolverType::AdaptiveNewmark, adaptiveNewmarkMaximumDisplacement,
              adaptiveNewmarkInheritanceGap, adaptiveNewmarkDynamicMilliseconds, adaptiveNewmarkMpcGap,
              adaptiveNewmarkAcceptedSteps, adaptiveNewmarkAverageStep, adaptiveNewmarkMaximumIterations);
    const bool tssbnSolved = !runTssbn || solve(SolverNameSpace::SolverType::AdaptiveTSSBN, tssbnMaximumDisplacement,
                                                tssbnInheritanceGap, tssbnDynamicMilliseconds, tssbnMpcGap,
                                                tssbnAcceptedSteps, tssbnAverageStep, tssbnMaximumIterations);
    const double adaptiveNewmarkRelativeDifference =
        runNewmark && runAdaptiveNewmark
            ? std::abs(adaptiveNewmarkMaximumDisplacement - newmarkMaximumDisplacement) /
                  std::max(1.0e-12, newmarkMaximumDisplacement)
            : 0.0;
    const double tssbnRelativeDifference = runNewmark && runTssbn
                                               ? std::abs(tssbnMaximumDisplacement - newmarkMaximumDisplacement) /
                                                     std::max(1.0e-12, newmarkMaximumDisplacement)
                                               : 0.0;
    QTextStream(stdout) << "conductor galloping duration=" << gallopingDuration << " attack=" << initialAttackDegrees
                        << " ice=" << gallopingIceThickness << " bundle=" << gallopingBundleCount
                        << " segments=" << gallopingSegments
                        << " maximum_step=" << gallopingMaximumTimeStep << " wind_direction=" << gallopingWindDirection
                        << " spacer_mode=" << spacerMode << " solver=" << gallopingSolver
                        << " aero_tangent=" << aerodynamicTangentModeText
                        << " newmark_solved=" << newmarkSolved
                        << " adaptive_newmark_solved=" << adaptiveNewmarkSolved << " tssbn_solved=" << tssbnSolved
                        << " newmark_max_displacement=" << newmarkMaximumDisplacement
                        << " newmark_dynamic_ms=" << newmarkDynamicMilliseconds
                        << " newmark_steps=" << newmarkAcceptedSteps << " newmark_average_step=" << newmarkAverageStep
                        << " newmark_max_iterations=" << newmarkMaximumIterations
                        << " newmark_mpc_gap=" << newmarkMpcGap
                        << " adaptive_newmark_max_displacement=" << adaptiveNewmarkMaximumDisplacement
                        << " adaptive_newmark_dynamic_ms=" << adaptiveNewmarkDynamicMilliseconds
                        << " adaptive_newmark_steps=" << adaptiveNewmarkAcceptedSteps
                        << " adaptive_newmark_average_step=" << adaptiveNewmarkAverageStep
                        << " adaptive_newmark_max_iterations=" << adaptiveNewmarkMaximumIterations
                        << " adaptive_newmark_mpc_gap=" << adaptiveNewmarkMpcGap
                        << " tssbn_max_displacement=" << tssbnMaximumDisplacement
                        << " tssbn_dynamic_ms=" << tssbnDynamicMilliseconds << " tssbn_steps=" << tssbnAcceptedSteps
                        << " tssbn_average_step=" << tssbnAverageStep
                        << " tssbn_max_iterations=" << tssbnMaximumIterations << " tssbn_mpc_gap=" << tssbnMpcGap
                        << " newmark_inheritance_gap=" << newmarkInheritanceGap
                        << " adaptive_newmark_inheritance_gap=" << adaptiveNewmarkInheritanceGap
                        << " tssbn_inheritance_gap=" << tssbnInheritanceGap
                        << " adaptive_newmark_relative_difference=" << adaptiveNewmarkRelativeDifference
                        << " tssbn_relative_difference=" << tssbnRelativeDifference << Qt::endl;

    if (!exportPath.isEmpty())
    {
        QFile report(exportPath + QStringLiteral(".diagnostic.txt"));
        if (report.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QTextStream reportStream(&report);
            reportStream << "duration=" << gallopingDuration << '\n'
                         << "attack=" << initialAttackDegrees << '\n'
                         << "ice=" << gallopingIceThickness << '\n'
                         << "wind_direction=" << gallopingWindDirection << '\n'
                         << "solver=" << gallopingSolver << '\n'
                         << "aero_tangent=" << aerodynamicTangentModeText << '\n'
                         << "newmark_solved=" << newmarkSolved << '\n'
                         << "adaptive_newmark_solved=" << adaptiveNewmarkSolved << '\n'
                         << "tssbn_solved=" << tssbnSolved << '\n'
                         << "newmark_max_displacement=" << newmarkMaximumDisplacement << '\n'
                         << "adaptive_newmark_max_displacement=" << adaptiveNewmarkMaximumDisplacement << '\n'
                         << "tssbn_max_displacement=" << tssbnMaximumDisplacement << '\n'
                         << "newmark_inheritance_gap=" << newmarkInheritanceGap << '\n'
                         << "adaptive_newmark_inheritance_gap=" << adaptiveNewmarkInheritanceGap << '\n'
                         << "tssbn_inheritance_gap=" << tssbnInheritanceGap << '\n'
                         << "adaptive_newmark_relative_difference=" << adaptiveNewmarkRelativeDifference << '\n'
                         << "tssbn_relative_difference=" << tssbnRelativeDifference << '\n';
        }
    }

    return newmarkSolved && adaptiveNewmarkSolved && tssbnSolved &&
                   (!runNewmark || newmarkMaximumDisplacement > 0.0) &&
                   (!runAdaptiveNewmark || adaptiveNewmarkMaximumDisplacement > 0.0) &&
                   (!runTssbn || tssbnMaximumDisplacement > 0.0) &&
                   (!(runNewmark && runAdaptiveNewmark) || adaptiveNewmarkRelativeDifference <= 0.05) &&
                   (!(runNewmark && runTssbn) || tssbnRelativeDifference <= 0.05)
               ? 0
               : 2;
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
                     [&application](const QString&, const QString&)
                     {
                         application.exit(3);
                     });
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
    QTimer::singleShot(120000, &application,
                       [&application]()
                       {
                           application.exit(5);
                       });
    if (!modelController.loadModel(filePath))
        return 1;
    return application.exec();
}

std::optional<int> verifyStaticDynamicTaskChain(QApplication& application, const QStringList& arguments)
{
    const int index = arguments.indexOf(QStringLiteral("--verify-static-dynamic-task-chain"));
    if (index < 0)
        return std::nullopt;
    if (index + 1 >= arguments.size())
        return 1;

    const QString filePath = arguments.at(index + 1);
    auto model = std::make_shared<StructureData>();
    Hdf5ModelIO hdf5;
    if (!hdf5.ImportHdf5(filePath, model.get()))
        return 2;

    int staticStepId = 0;
    int dynamicStepId = 0;
    for (const auto& [stepId, step] : model->m_AnalysisStep)
    {
        if (!step)
            continue;
        if (step->m_Type == EnumKeyword::StepType::STATIC)
            staticStepId = stepId;
        if (step->m_Type == EnumKeyword::StepType::DYNAMIC && step->m_InitialStaticStepId > 0)
        {
            dynamicStepId = stepId;
            staticStepId = step->m_InitialStaticStepId;
            break;
        }
    }
    if (staticStepId <= 0 || dynamicStepId <= 0)
        return 3;

    SolveTaskController taskController;
    const int staticTaskId = taskController.prepare(model, filePath, staticStepId);
    const int dynamicTaskId = taskController.prepare(model, filePath, dynamicStepId);
    if (staticTaskId < 0 || dynamicTaskId < 0)
        return 4;

    bool staticCompleted = false;
    bool dynamicStartedBeforeStatic = false;
    QObject::connect(
        &taskController, &SolveTaskController::taskUpdated,
        [&taskController, staticTaskId, dynamicTaskId, &staticCompleted, &dynamicStartedBeforeStatic,
         &application](int taskId)
        {
            const auto info = taskController.taskInfo(taskId);
            if (taskId == staticTaskId && info.status == SolveTaskController::Status::Completed)
                staticCompleted = true;
            if (taskId == dynamicTaskId && info.status == SolveTaskController::Status::Running && !staticCompleted)
                dynamicStartedBeforeStatic = true;
            if (taskId != dynamicTaskId || (info.status != SolveTaskController::Status::Completed &&
                                            info.status != SolveTaskController::Status::Failed &&
                                            info.status != SolveTaskController::Status::Cancelled))
                return;

            const auto staticInfo = taskController.taskInfo(staticTaskId);
            const bool passed = info.status == SolveTaskController::Status::Completed &&
                                staticInfo.status == SolveTaskController::Status::Completed &&
                                !dynamicStartedBeforeStatic;
            QTextStream(stdout) << "task chain static=" << SolveTaskController::statusText(staticInfo.status)
                                << " dynamic=" << SolveTaskController::statusText(info.status)
                                << " dynamic_started_before_static=" << (dynamicStartedBeforeStatic ? "true" : "false")
                                << Qt::endl;
            application.exit(passed ? 0 : 5);
        });
    QTimer::singleShot(120000, &application,
                       [&application]()
                       {
                           application.exit(6);
                       });
    if (!taskController.start(dynamicTaskId))
        return 7;
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
                     [&application](const QString&, const QString&)
                     {
                         application.exit(3);
                     });
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
    QTimer::singleShot(120000, &application,
                       [&application]()
                       {
                           application.exit(5);
                       });
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
    QTimer::singleShot(30000, &application,
                       [&application]()
                       {
                           application.exit(2);
                       });
    if (files.isEmpty() || controller.loadModels(files) != files.size())
        return 1;
    return application.exec();
}

std::optional<int> verifyGpuSparseSolver(const QStringList& arguments)
{
    if (!arguments.contains(QStringLiteral("--verify-gpu-solver")))
        return std::nullopt;

    constexpr int dimension = 20000;
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(dimension) * 3);
    for (int row = 0; row < dimension; ++row)
    {
        triplets.emplace_back(row, row, 5.0);
        if (row > 0)
            triplets.emplace_back(row, row - 1, -1.0);
        if (row + 1 < dimension)
            triplets.emplace_back(row, row + 1, -0.5);
    }
    SolverNameSpace::SpMat matrix(dimension, dimension);
    matrix.setFromTriplets(triplets.begin(), triplets.end());
    matrix.makeCompressed();
    const SolverNameSpace::Vec expected = SolverNameSpace::Vec::Ones(dimension);
    const SolverNameSpace::Vec rhs = matrix * expected;

    const auto solveAndMeasure = [&](SolverNameSpace::CudaSparseSolver& solver, _OUT double& milliseconds)
    {
        SolverNameSpace::Vec solution;
        QElapsedTimer timer;
        timer.start();
        const bool solved = solver.Solve(matrix, rhs, solution, 1.0e-11, 500);
        milliseconds = static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
        if (!solved)
            return false;
        const double relativeResidual = (matrix * solution - rhs).norm() / rhs.norm();
        const double relativeError = (solution - expected).norm() / expected.norm();
        return relativeResidual <= 1.0e-9 && relativeError <= 1.0e-8;
    };

    // 先完成 CUDA 上下文初始化，避免把驱动冷启动时间计入资源复用对比。
    SolverNameSpace::CudaSparseSolver warmupSolver;
    double warmupMilliseconds = 0.0;
    if (!solveAndMeasure(warmupSolver, warmupMilliseconds))
        return 2;

    constexpr int repetitions = 5;
    double recreatedTotal = 0.0;
    for (int repetition = 0; repetition < repetitions; ++repetition)
    {
        SolverNameSpace::CudaSparseSolver solver;
        double milliseconds = 0.0;
        if (!solveAndMeasure(solver, milliseconds))
            return 3;
        recreatedTotal += milliseconds;
    }

    SolverNameSpace::CudaSparseSolver reusedSolver;
    double reusedTotal = 0.0;
    for (int repetition = 0; repetition < repetitions; ++repetition)
    {
        double milliseconds = 0.0;
        if (!solveAndMeasure(reusedSolver, milliseconds))
            return 4;
        reusedTotal += milliseconds;
    }

    const double recreatedAverage = recreatedTotal / repetitions;
    const double reusedAverage = reusedTotal / repetitions;
    QTextStream(stdout) << "gpu sparse dimension=" << dimension << " recreated_average_ms=" << recreatedAverage
                        << " reused_average_ms=" << reusedAverage << " speedup=" << recreatedAverage / reusedAverage
                        << Qt::endl;

    const auto verifyDirectSolution = [&](const SolverNameSpace::Vec& solution)
    {
        if (solution.size() != expected.size())
            return false;
        const double relativeResidual = (matrix * solution - rhs).norm() / rhs.norm();
        const double relativeError = (solution - expected).norm() / expected.norm();
        return relativeResidual <= 1.0e-10 && relativeError <= 1.0e-9;
    };

    SolverNameSpace::PardisoSolver pardisoSolver;
    SolverNameSpace::Vec pardisoSolution;
    QElapsedTimer pardisoTimer;
    pardisoTimer.start();
    if (!pardisoSolver.Solve(matrix, rhs, false, 1, pardisoSolution) || !verifyDirectSolution(pardisoSolution))
        return 5;
    const double pardisoMilliseconds = static_cast<double>(pardisoTimer.nsecsElapsed()) / 1.0e6;

    SolverNameSpace::CudssSolver cudssSolver;
    SolverNameSpace::Vec cudssSolution;
    QElapsedTimer cudssTimer;
    cudssTimer.start();
    if (!cudssSolver.Solve(matrix, rhs, cudssSolution) || !verifyDirectSolution(cudssSolution))
        return 6;
    const double cudssFirstMilliseconds = static_cast<double>(cudssTimer.nsecsElapsed()) / 1.0e6;
    cudssTimer.restart();
    if (!cudssSolver.Solve(matrix, rhs, cudssSolution) || !verifyDirectSolution(cudssSolution))
        return 7;
    const double cudssReusedMilliseconds = static_cast<double>(cudssTimer.nsecsElapsed()) / 1.0e6;
    QTextStream(stdout) << "direct sparse pardiso_ms=" << pardisoMilliseconds
                        << " cudss_first_ms=" << cudssFirstMilliseconds
                        << " cudss_reused_ms=" << cudssReusedMilliseconds << Qt::endl;
    return 0;
}
} // namespace

std::optional<int> VerificationRunner::runHeadless(const QStringList& arguments)
{
    if (const auto result = verifyLowRankDampingSolver(arguments))
        return result;
    if (const auto result = verifyStructuralDamping(arguments))
        return result;
    if (const auto result = verifyGpuSparseSolver(arguments))
        return result;
    if (const auto result = verifyTimeStepIntegrators(arguments))
        return result;
    if (const auto result = verifyDynamicMpc(arguments))
        return result;
    if (const auto result = verifyDynamicMpcModel(arguments))
        return result;
    if (const auto result = verifyAdaptiveTssbn(arguments))
        return result;
    if (const auto result = verifyLe2012Example1(arguments))
        return result;
    if (const auto result = verifyLe2012Example4(arguments))
        return result;
    if (const auto result = verifyCableReference(arguments))
        return result;
    if (const auto result = verifyCableTorsion(arguments))
        return result;
    if (const auto result = verifySpatialWindLoad(arguments))
        return result;
    if (const auto result = verifyExactAerodynamicAngle(arguments))
        return result;
    if (const auto result = verifyGallopingCaseSelection(arguments))
        return result;
    if (const auto result = verifyGallopingStability(arguments))
        return result;
    return verifyBeamDynamics(arguments);
}

std::optional<int> VerificationRunner::run(QApplication& application, const QStringList& arguments)
{
    if (const auto result = verifySpringElements(arguments))
        return result;
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
    if (const auto result = verifyConductorSpacerMPC(arguments))
        return result;
    if (const auto result = verifyConductorStatic(arguments))
        return result;
    if (const auto result = verifyConductorGallopingDynamics(arguments))
        return result;
    if (const auto result = verifyStaticDynamicTaskChain(application, arguments))
        return result;
    if (const auto result = verifySolveTask(application, arguments))
        return result;
    if (const auto result = verifyPartialResult(application, arguments))
        return result;
    return verifyModelImport(application, arguments);
}
