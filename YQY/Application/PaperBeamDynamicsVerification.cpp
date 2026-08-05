#include "Application/PaperBeamDynamicsVerification.h"

#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionBase.h"
#include "Solver/Dynamic/SolverAdaptiveTSSBN.h"
#include "Solver/Dynamic/SolverNewmark.h"

#include <QDir>
#include <QFile>
#include <QTextStream>
#include <array>
#include <limits>
#include <memory>

namespace
{
double PositiveEnvironmentValue(const char* name, double fallback)
{
    bool valid = false;
    const double value =
        QString::fromLocal8Bit(qgetenv(name)).toDouble(&valid);
    return valid && value > 0.0 ? value : fallback;
}

class Le2012Section final : public SectionBase
{
public:
    explicit Le2012Section(double sectionInertia)
        : m_sectionInertia(sectionInertia)
    {
        m_Area = 1.0;
        m_MassInertiaPerLength = Eigen::Vector3d(20.0, 10.0, 10.0);
    }

    void Calculate_I(double& Iy, double& Iz, double& J) override
    {
        Iy = m_sectionInertia;
        Iz = m_sectionInertia;
        J = m_sectionInertia;
    }

private:
    double m_sectionInertia = 0.0;
};

enum class Le2012Example
{
    RightAngleCantilever,
    FreeFreeBeamWithDisks
};

class Le2012BeamModel final : public SolverNameSpace::IAnalysisModel
{
public:
    explicit Le2012BeamModel(Le2012Example example)
        : m_example(example)
    {
        const bool rightAngle =
            m_example == Le2012Example::RightAngleCantilever;
        const int elementCount = rightAngle ? 20 : 10;
        constexpr double elementLength = 1.0;

        m_material = std::make_shared<Material>();
        m_material->m_Young = rightAngle ? 1.0e6 : 1.0e4;
        m_material->m_Poisson = -0.5; // G = E, as required by Table 2.
        m_material->m_Density = 1.0;  // A_rho = rho*A = 1.
        // Table 2 gives EI = GJ = 1000 for Example 1 and 500 for Example 4.
        m_section = std::make_shared<Le2012Section>(
            rightAngle ? 1.0e-3 : 5.0e-2);
        m_property = std::make_shared<Property>();
        m_property->m_pMaterial = m_material;
        m_property->m_pSection = m_section;

        m_nodes.reserve(elementCount + 1);
        m_freeDofs.resize(elementCount + 1);
        int freeDof = 0;
        for (int nodeIndex = 0; nodeIndex <= elementCount; ++nodeIndex)
        {
            auto node = std::make_shared<Node>();
            node->SetNumDOFs(6);
            if (rightAngle)
            {
                if (nodeIndex <= 10)
                    node->m_Y = nodeIndex * elementLength;
                else
                {
                    node->m_X = -(nodeIndex - 10) * elementLength;
                    node->m_Y = 10.0;
                }
            }
            else
            {
                // Fig. 3.2: the 10 m beam initially spans the 6-8-10
                // triangle in the global x-y plane.
                const double fraction = static_cast<double>(nodeIndex)
                    / static_cast<double>(elementCount);
                node->m_X = 6.0 * fraction;
                node->m_Y = 8.0 * (1.0 - fraction);
            }
            m_nodes.push_back(node);

            m_freeDofs[nodeIndex].fill(-1);
            if (nodeIndex == 0 && rightAngle)
            {
                // Fig. 2: the first end of the right-angle frame is clamped.
            }
            else
            {
                for (int component = 0; component < 6; ++component)
                    m_freeDofs[nodeIndex][component] = freeDof++;
            }
        }
        m_freeDofCount = freeDof;

        m_elements.reserve(elementCount);
        for (int elementIndex = 0; elementIndex < elementCount; ++elementIndex)
        {
            auto element = std::make_shared<ElementBeam_CR>();
            element->m_pNode[0] = m_nodes[elementIndex];
            element->m_pNode[1] = m_nodes[elementIndex + 1];
            element->m_pProperty = m_property;
            element->q0 = rightAngle
                ? Eigen::Vector3d::UnitZ()
                : Eigen::Vector3d::UnitZ();
            element->Get_L0();
            m_elements.push_back(element);
        }
        m_inertiaForce = Eigen::VectorXd::Zero(m_freeDofCount);
    }

    int GetFreeDofs() const override { return m_freeDofCount; }
    int GetFixedDofs() const override
    {
        return m_example == Le2012Example::RightAngleCantilever ? 6 : 0;
    }

    void ApplyIncrement(const SolverNameSpace::Vec& dx) override
    {
        ApplyDynamicCorrection(dx, 0.0, 0.0);
    }

    void BeginDynamicStep(double dt, double beta, double gamma) override
    {
        for (int nodeIndex = 0;
             nodeIndex < static_cast<int>(m_nodes.size());
             ++nodeIndex)
        {
            std::array<bool, 3> translationActive{};
            std::array<bool, 3> rotationActive{};
            for (int component = 0; component < 3; ++component)
            {
                translationActive[component] =
                    m_freeDofs[nodeIndex][component] >= 0;
                rotationActive[component] =
                    m_freeDofs[nodeIndex][component + 3] >= 0;
            }
            m_nodes[nodeIndex]->BeginNewmarkStep(
                dt, beta, gamma, translationActive, rotationActive);
        }
    }

    void ApplyDynamicCorrection(
        const SolverNameSpace::Vec& dx, double a0, double a1) override
    {
        for (int nodeIndex = 0;
             nodeIndex < static_cast<int>(m_nodes.size());
             ++nodeIndex)
        {
            Eigen::Vector3d translation = Eigen::Vector3d::Zero();
            Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
            for (int component = 0; component < 3; ++component)
            {
                const int translationDof =
                    m_freeDofs[nodeIndex][component];
                const int rotationDof =
                    m_freeDofs[nodeIndex][component + 3];
                if (translationDof >= 0)
                    translation(component) = dx[translationDof];
                if (rotationDof >= 0)
                    rotation(component) = dx[rotationDof];
            }
            m_nodes[nodeIndex]->ApplyNewmarkCorrection(
                translation, rotation, a0, a1);
        }
    }

    void RollbackDynamicStep() override
    {
        for (const auto& node : m_nodes)
            node->RollbackNewmarkStep();
    }

    void SetTrialKinematics(
        const SolverNameSpace::Vec& velocity,
        const SolverNameSpace::Vec& acceleration) override
    {
        for (int nodeIndex = 0;
             nodeIndex < static_cast<int>(m_nodes.size());
             ++nodeIndex)
        {
            for (int component = 0; component < 6; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component];
                if (dof >= 0)
                {
                    m_nodes[nodeIndex]->m_Velocity[component] = velocity[dof];
                    m_nodes[nodeIndex]->m_Acceleration[component] =
                        acceleration[dof];
                }
            }
            auto& node = *m_nodes[nodeIndex];
            const Eigen::Vector3d spatialVelocity(
                node.m_Velocity[3],
                node.m_Velocity[4],
                node.m_Velocity[5]);
            const Eigen::Vector3d spatialAcceleration(
                node.m_Acceleration[3],
                node.m_Acceleration[4],
                node.m_Acceleration[5]);
            node.m_OmegaMaterial =
                node.m_Rg.transpose() * spatialVelocity;
            node.m_AlphaMaterial =
                node.m_Rg.transpose() * spatialAcceleration;
        }
    }

    void SetTssbnStageKinematics(
        int stageIndex,
        double timeStep,
        double firstStageTime,
        double secondStageTime,
        double secondStageDiagonalFraction,
        SolverNameSpace::Vec& velocity,
        SolverNameSpace::Vec& acceleration) override
    {
        SetTrialKinematics(velocity, acceleration);
        for (int nodeIndex = 0;
             nodeIndex < static_cast<int>(m_nodes.size());
             ++nodeIndex)
        {
            Eigen::Vector3d angularVelocity;
            Eigen::Vector3d angularAcceleration;
            m_nodes[nodeIndex]->SetTssbnStageKinematics(
                stageIndex, timeStep,
                firstStageTime, secondStageTime,
                secondStageDiagonalFraction,
                angularVelocity, angularAcceleration);
            for (int component = 0; component < 3; ++component)
            {
                const int dof =
                    m_freeDofs[nodeIndex][component + 3];
                if (dof < 0)
                    continue;
                velocity[dof] = angularVelocity[component];
                acceleration[dof] = angularAcceleration[component];
            }
        }
    }

    void CorrectTssbnStepStates(
        double timeStep,
        double firstStageTime,
        double secondStageTime,
        double lastStageTime,
        double baseFirstWeight,
        double embeddedFirstWeight,
        double embeddedSecondWeight,
        double embeddedLastWeight,
        double lastStageFirstCoefficient,
        double lastStageSecondCoefficient,
        SolverNameSpace::Vec& baseIncrement,
        SolverNameSpace::Vec& baseVelocity,
        SolverNameSpace::Vec& embeddedIncrement,
        SolverNameSpace::Vec& embeddedVelocity,
        SolverNameSpace::Vec& acceptedAcceleration) override
    {
        const double extrapolation =
            (lastStageTime - secondStageTime)
            / (secondStageTime - firstStageTime);
        for (int nodeIndex = 0;
             nodeIndex < static_cast<int>(m_nodes.size());
             ++nodeIndex)
        {
            const auto& node = m_nodes[nodeIndex];
            const Node::TssbnRotationState rotation =
                node->IntegrateTssbnRotation(
                    timeStep, extrapolation, baseFirstWeight,
                    embeddedFirstWeight, embeddedSecondWeight,
                    embeddedLastWeight, lastStageFirstCoefficient,
                    lastStageSecondCoefficient);

            for (int component = 0; component < 3; ++component)
            {
                const int dof =
                    m_freeDofs[nodeIndex][component + 3];
                if (dof < 0)
                    continue;
                baseIncrement[dof] =
                    rotation.baseSpatialIncrement[component];
                embeddedIncrement[dof] =
                    rotation.embeddedSpatialIncrement[component];
                baseVelocity[dof] =
                    rotation.baseSpatialVelocity[component];
                embeddedVelocity[dof] =
                    rotation.embeddedSpatialVelocity[component];
                acceptedAcceleration[dof] =
                    rotation.acceptedSpatialAcceleration[component];
            }
        }
    }

    void GetState(
        SolverNameSpace::Vec& displacement,
        SolverNameSpace::Vec& velocity,
        SolverNameSpace::Vec& acceleration) const override
    {
        displacement = Eigen::VectorXd::Zero(m_freeDofCount);
        velocity = Eigen::VectorXd::Zero(m_freeDofCount);
        acceleration = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int nodeIndex = 0;
             nodeIndex < static_cast<int>(m_nodes.size());
             ++nodeIndex)
        {
            for (int component = 0; component < 6; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component];
                if (dof < 0)
                    continue;
                displacement[dof] =
                    m_nodes[nodeIndex]->m_Displacement[component];
                velocity[dof] = m_nodes[nodeIndex]->m_Velocity[component];
                acceleration[dof] =
                    m_nodes[nodeIndex]->m_Acceleration[component];
            }
        }
    }

    void Assemble_Matrix(
        SolverNameSpace::SpMat& stiffness, bool) override
    {
        std::vector<Eigen::Triplet<double>> triplets;
        m_internalForce = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int elementIndex = 0;
             elementIndex < static_cast<int>(m_elements.size());
             ++elementIndex)
        {
            Eigen::MatrixXd elementStiffness;
            m_elements[elementIndex]->Get_ke(elementStiffness);
            AssembleElementMatrix(
                elementIndex, elementStiffness, triplets);
            AssembleElementVector(
                elementIndex, m_elements[elementIndex]->m_inforce,
                m_internalForce);
        }
        stiffness.resize(m_freeDofCount, m_freeDofCount);
        stiffness.setFromTriplets(triplets.begin(), triplets.end());
    }

    void AssembleDynamicSystem(
        SolverNameSpace::SpMat& mass,
        SolverNameSpace::SpMat& gyroscopic,
        SolverNameSpace::SpMat& centrifugal) override
    {
        std::vector<Eigen::Triplet<double>> massTriplets;
        std::vector<Eigen::Triplet<double>> gyroscopicTriplets;
        std::vector<Eigen::Triplet<double>> centrifugalTriplets;
        m_inertiaForce = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int elementIndex = 0;
             elementIndex < static_cast<int>(m_elements.size());
             ++elementIndex)
        {
            Eigen::MatrixXd elementMass;
            Eigen::MatrixXd elementGyroscopic;
            Eigen::MatrixXd elementCentrifugal;
            Eigen::VectorXd elementInertia;
            m_elements[elementIndex]->Get_me_Consistent(elementMass);
            m_elements[elementIndex]->Get_GyroscopicMatrix(
                elementGyroscopic);
            m_elements[elementIndex]->Get_CentrifugalMatrix(
                elementCentrifugal);
            m_elements[elementIndex]->Get_InertiaForce(elementInertia);
            AssembleElementMatrix(
                elementIndex, elementMass, massTriplets);
            AssembleElementMatrix(
                elementIndex, elementGyroscopic, gyroscopicTriplets);
            AssembleElementMatrix(
                elementIndex, elementCentrifugal, centrifugalTriplets);
            AssembleElementVector(
                elementIndex, elementInertia, m_inertiaForce);
        }
        if (m_example == Le2012Example::FreeFreeBeamWithDisks)
        {
            // Fig. 3.2 / Section 3.4: identical rigid disks are attached to
            // both beam ends.  Their point mass and principal inertias are
            // represented as nodal inertia terms.
            constexpr double diskMass = 10.0;
            constexpr double diskInertia[3] = { 200.0, 100.0, 100.0 };
            for (const int nodeIndex : { 0,
                    static_cast<int>(m_nodes.size()) - 1 })
            {
                for (int component = 0; component < 3; ++component)
                {
                    massTriplets.emplace_back(
                        m_freeDofs[nodeIndex][component],
                        m_freeDofs[nodeIndex][component], diskMass);
                    massTriplets.emplace_back(
                        m_freeDofs[nodeIndex][component + 3],
                        m_freeDofs[nodeIndex][component + 3],
                        diskInertia[component]);
                    m_inertiaForce[m_freeDofs[nodeIndex][component]] +=
                        diskMass * m_nodes[nodeIndex]->m_Acceleration[component];
                    m_inertiaForce[
                        m_freeDofs[nodeIndex][component + 3]] +=
                        diskInertia[component]
                        * m_nodes[nodeIndex]->m_Acceleration[component + 3];
                }
            }
        }
        mass.resize(m_freeDofCount, m_freeDofCount);
        gyroscopic.resize(m_freeDofCount, m_freeDofCount);
        centrifugal.resize(m_freeDofCount, m_freeDofCount);
        mass.setFromTriplets(massTriplets.begin(), massTriplets.end());
        gyroscopic.setFromTriplets(
            gyroscopicTriplets.begin(), gyroscopicTriplets.end());
        centrifugal.setFromTriplets(
            centrifugalTriplets.begin(), centrifugalTriplets.end());
    }

    void ComputeExternalForce(
        double time, double,
        SolverNameSpace::Vec& fixedForce,
        SolverNameSpace::Vec& freeForce) override
    {
        fixedForce = Eigen::VectorXd::Zero(GetFixedDofs());
        freeForce = Eigen::VectorXd::Zero(m_freeDofCount);
        if (m_example == Le2012Example::RightAngleCantilever)
        {
            double forceZ = 0.0;
            if (time <= 1.0)
                forceZ = 50.0 * time;
            else if (time <= 2.0)
                forceZ = 50.0 * (2.0 - time);
            constexpr int elbowNode = 10;
            freeForce[m_freeDofs[elbowNode][2]] = forceZ;
            return;
        }

        const int endNode = static_cast<int>(m_nodes.size()) - 1;
        double forceZ = 0.0;
        if (time <= 2.5)
            forceZ = 8.0 * time;
        else if (time <= 5.0)
            forceZ = 8.0 * (5.0 - time);

        // Fig. 3.2: an in-plane force Fx=2Fz at the lower-right disk and
        // an opposite pair of out-of-plane forces at the two disks.
        freeForce[m_freeDofs[endNode][0]] = -2.0 * forceZ;
        freeForce[m_freeDofs[endNode][2]] = forceZ;
        freeForce[m_freeDofs[0][2]] = -forceZ;
    }

    void Assemble_Constraint(
        SolverNameSpace::Vec& displacement,
        double, double) override
    {
        displacement = Eigen::VectorXd::Zero(GetFixedDofs());
    }

    void ComputeResidual(
        const SolverNameSpace::Vec& externalForce,
        SolverNameSpace::Vec& residual) override
    {
        residual = externalForce - m_internalForce - m_inertiaForce;
    }

    void ComputeStaticResidual(
        const SolverNameSpace::Vec& externalForce,
        SolverNameSpace::Vec& residual) override
    {
        residual = externalForce - m_internalForce;
    }

    void CalculateReactions(SolverNameSpace::Vec& force) override
    {
        force = Eigen::VectorXd::Zero(GetFixedDofs());
    }

    void OnStepCompleted(double) override {}
    void CommitState() override {}
    void BackupStepState() override
    {
        for (const auto& node : m_nodes)
        {
            node->m_Displacement_n = node->m_Displacement;
            node->m_Velocity_n = node->m_Velocity;
            node->m_Acceleration_n = node->m_Acceleration;
            node->m_Rg_n = node->m_Rg;
            node->m_OmegaMaterial_n = node->m_OmegaMaterial;
            node->m_AlphaMaterial_n = node->m_AlphaMaterial;
            node->m_StepRotation.setZero();
        }
    }

    void GetStepIncrement(SolverNameSpace::Vec& increment) const override
    {
        increment = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int nodeIndex = 0;
             nodeIndex < static_cast<int>(m_nodes.size());
             ++nodeIndex)
        {
            const auto& node = *m_nodes[nodeIndex];
            for (int component = 0; component < 3; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component];
                if (dof >= 0)
                {
                    increment[dof] =
                        node.m_Displacement[component]
                        - node.m_Displacement_n[component];
                }
            }
            for (int component = 0; component < 3; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component + 3];
                if (dof >= 0)
                    increment[dof] = node.m_StepRotation[component];
            }
        }
    }

    Eigen::Vector3d EndDisplacement() const
    {
        return NodeDisplacement(static_cast<int>(m_nodes.size()) - 1);
    }

    Eigen::Vector3d NodeDisplacement(int nodeIndex) const
    {
        const auto& displacement = m_nodes[nodeIndex]->m_Displacement;
        return Eigen::Vector3d(
            displacement[0], displacement[1], displacement[2]);
    }

private:
    void AssembleElementMatrix(
        int elementIndex,
        const Eigen::MatrixXd& elementMatrix,
        std::vector<Eigen::Triplet<double>>& triplets) const
    {
        for (int localRow = 0; localRow < 12; ++localRow)
        {
            const int rowNode = elementIndex + localRow / 6;
            const int row = m_freeDofs[rowNode][localRow % 6];
            if (row < 0)
                continue;
            for (int localColumn = 0; localColumn < 12; ++localColumn)
            {
                const int columnNode =
                    elementIndex + localColumn / 6;
                const int column =
                    m_freeDofs[columnNode][localColumn % 6];
                if (column >= 0 && elementMatrix(localRow, localColumn) != 0.0)
                    triplets.emplace_back(
                        row, column,
                        elementMatrix(localRow, localColumn));
            }
        }
    }

    void AssembleElementVector(
        int elementIndex,
        const Eigen::VectorXd& elementVector,
        Eigen::VectorXd& vector) const
    {
        for (int localRow = 0; localRow < 12; ++localRow)
        {
            const int nodeIndex = elementIndex + localRow / 6;
            const int row = m_freeDofs[nodeIndex][localRow % 6];
            if (row >= 0)
                vector[row] += elementVector[localRow];
        }
    }

    int m_freeDofCount = 0;
    Le2012Example m_example;
    std::shared_ptr<Material> m_material;
    std::shared_ptr<Le2012Section> m_section;
    std::shared_ptr<Property> m_property;
    std::vector<std::shared_ptr<Node>> m_nodes;
    std::vector<std::shared_ptr<ElementBeam_CR>> m_elements;
    std::vector<std::array<int, 6>> m_freeDofs;
    Eigen::VectorXd m_internalForce;
    Eigen::VectorXd m_inertiaForce;
};
} // namespace

int PaperBeamDynamicsVerification::RunExample1(
    const QString& outputDirectory)
{
    return RunExample1(outputDirectory, false);
}

int PaperBeamDynamicsVerification::RunExample1Adaptive(
    const QString& outputDirectory)
{
    return RunExample1(outputDirectory, true);
}

int PaperBeamDynamicsVerification::RunExample1(
    const QString& outputDirectory,
    bool useAdaptiveTssbn)
{
    Le2012BeamModel model(Le2012Example::RightAngleCantilever);
    const double initialTimeStep =
        PositiveEnvironmentValue("YQY_LE2012_EXAMPLE1_DT", 0.25);
    const double duration =
        PositiveEnvironmentValue("YQY_LE2012_DURATION", 30.0);
    std::unique_ptr<SolverNameSpace::ISolver> solver;
    if (useAdaptiveTssbn)
    {
        SolverNameSpace::SolverAdaptiveTSSBN::Params parameters;
        parameters.initialTimeStep = std::min(0.10, initialTimeStep);
        parameters.minimumTimeStep = 1.0e-4;
        parameters.maximumTimeStep = initialTimeStep;
        parameters.relativeTolerance =
            PositiveEnvironmentValue(
                "YQY_TSSBN_REL_TOL", parameters.relativeTolerance);
        parameters.absoluteTolerance =
            PositiveEnvironmentValue("YQY_TSSBN_ABS_TOL", 1.0e-6);
        parameters.nonlinearTolerance = 1.0e-5;
        parameters.maximumNewtonIterations = 48;
        solver = std::make_unique<SolverNameSpace::SolverAdaptiveTSSBN>(
            parameters);
    }
    else
    {
        SolverNameSpace::SolverNewmark::Params parameters;
        parameters.dt = initialTimeStep;
        parameters.beta = 0.25;
        parameters.gamma = 0.5;
        parameters.maxIter = 48;
        parameters.tol = 1.0e-5;
        solver =
            std::make_unique<SolverNameSpace::SolverNewmark>(parameters);
    }

    QDir directory;
    if (!directory.mkpath(outputDirectory))
        return 2;
    QFile resultFile(
        QDir(outputDirectory).filePath(QStringLiteral("response.csv")));
    if (!resultFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return 3;
    QTextStream stream(&resultFile);
    stream << "time,tip_u_z,elbow_u_z\n";

    double minimumTipZ = std::numeric_limits<double>::infinity();
    double maximumTipZ = -std::numeric_limits<double>::infinity();
    double minimumElbowZ = std::numeric_limits<double>::infinity();
    double maximumElbowZ = -std::numeric_limits<double>::infinity();
    double lastCompletedTime = 0.0;
    solver->SetStepCallback(
        [&model, &stream, &minimumTipZ, &maximumTipZ,
         &minimumElbowZ, &maximumElbowZ, &lastCompletedTime](
            int, double time, const Eigen::VectorXd&)
        {
            const double tipZ = model.EndDisplacement().z();
            const double elbowZ = model.NodeDisplacement(10).z();
            stream << QString::number(time, 'f', 6) << ','
                   << QString::number(tipZ, 'g', 16) << ','
                   << QString::number(elbowZ, 'g', 16) << '\n';
            minimumTipZ = std::min(minimumTipZ, tipZ);
            maximumTipZ = std::max(maximumTipZ, tipZ);
            minimumElbowZ = std::min(minimumElbowZ, elbowZ);
            maximumElbowZ = std::max(maximumElbowZ, elbowZ);
            lastCompletedTime = time;
        });

    if (!solver->Solve(model, duration))
    {
        resultFile.close();
        QFile failureSummary(
            QDir(outputDirectory).filePath(QStringLiteral("summary.txt")));
        if (failureSummary.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            QTextStream summary(&failureSummary);
            summary << "Le et al. (2012), Example 1 (Fig. 2)\n"
                    << "status,nonconverged\n"
                    << "last_completed_time," << lastCompletedTime << '\n'
                    << "minimum_tip_u_z," << minimumTipZ << '\n'
                    << "maximum_tip_u_z," << maximumTipZ << '\n'
                    << "minimum_elbow_u_z," << minimumElbowZ << '\n'
                    << "maximum_elbow_u_z," << maximumElbowZ << '\n';
        }
        QTextStream(stdout)
            << "Le2012 Example1 nonconverged last_time="
            << lastCompletedTime << " result=" << resultFile.fileName()
            << Qt::endl;
        return 4;
    }
    resultFile.close();

    // Fig. 3 values are read from the published plot axes.
    constexpr double referenceMinimumTipZ = -9.7;
    constexpr double referenceMaximumTipZ = 8.2;
    constexpr double referenceMinimumElbowZ = -4.0;
    constexpr double referenceMaximumElbowZ = 6.2;
    const auto relativeError = [](double calculated, double reference)
    {
        return std::abs(calculated - reference) / std::abs(reference);
    };
    const double minimumTipError =
        relativeError(minimumTipZ, referenceMinimumTipZ);
    const double maximumTipError =
        relativeError(maximumTipZ, referenceMaximumTipZ);
    const double minimumElbowError =
        relativeError(minimumElbowZ, referenceMinimumElbowZ);
    const double maximumElbowError =
        relativeError(maximumElbowZ, referenceMaximumElbowZ);

    QFile summaryFile(
        QDir(outputDirectory).filePath(QStringLiteral("summary.txt")));
    if (summaryFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream summary(&summaryFile);
        summary << "Le et al. (2012), Example 1 (Fig. 2)\n"
                << "quantity,calculated,figure_reference,relative_error\n"
                << "minimum_tip_u_z," << minimumTipZ << ','
                << referenceMinimumTipZ << ',' << minimumTipError << '\n'
                << "maximum_tip_u_z," << maximumTipZ << ','
                << referenceMaximumTipZ << ',' << maximumTipError << '\n'
                << "minimum_elbow_u_z," << minimumElbowZ << ','
                << referenceMinimumElbowZ << ',' << minimumElbowError << '\n'
                << "maximum_elbow_u_z," << maximumElbowZ << ','
                << referenceMaximumElbowZ << ',' << maximumElbowError << '\n';
    }

    QTextStream(stdout)
        << "Le2012 Example1 tip_uz=[" << minimumTipZ << ','
        << maximumTipZ << "] elbow_uz=[" << minimumElbowZ << ','
        << maximumElbowZ << "] errors=[" << minimumTipError << ','
        << maximumTipError << ',' << minimumElbowError << ','
        << maximumElbowError << "] result=" << resultFile.fileName()
        << Qt::endl;

    const bool matchesPaper =
        minimumTipZ >= -10.5 && minimumTipZ <= -8.5
        && maximumTipZ >= 7.0 && maximumTipZ <= 9.0
        && minimumElbowZ >= -5.0 && minimumElbowZ <= -3.0
        && maximumElbowZ >= 5.0 && maximumElbowZ <= 7.0;
    return matchesPaper ? 0 : 5;
}

int PaperBeamDynamicsVerification::Run(const QString& outputDirectory)
{
    return Run(outputDirectory, false);
}

int PaperBeamDynamicsVerification::RunAdaptive(
    const QString& outputDirectory)
{
    return Run(outputDirectory, true);
}

int PaperBeamDynamicsVerification::Run(
    const QString& outputDirectory,
    bool useAdaptiveTssbn)
{
    Le2012BeamModel model(Le2012Example::FreeFreeBeamWithDisks);
    const double initialTimeStep =
        PositiveEnvironmentValue("YQY_LE2012_EXAMPLE4_DT", 0.10);
    const double duration =
        PositiveEnvironmentValue("YQY_LE2012_DURATION", 30.0);

    std::unique_ptr<SolverNameSpace::ISolver> solver;
    if (useAdaptiveTssbn)
    {
        SolverNameSpace::SolverAdaptiveTSSBN::Params parameters;
        parameters.initialTimeStep = initialTimeStep;
        parameters.minimumTimeStep = 1.0e-4;
        parameters.maximumTimeStep = initialTimeStep;
        parameters.relativeTolerance =
            PositiveEnvironmentValue(
                "YQY_TSSBN_REL_TOL", parameters.relativeTolerance);
        parameters.absoluteTolerance =
            PositiveEnvironmentValue("YQY_TSSBN_ABS_TOL", 1.0e-6);
        parameters.nonlinearTolerance = 1.0e-5;
        parameters.maximumNewtonIterations = 32;
        solver = std::make_unique<SolverNameSpace::SolverAdaptiveTSSBN>(
            parameters);
    }
    else
    {
        SolverNameSpace::SolverNewmark::Params parameters;
        parameters.dt = initialTimeStep;
        parameters.beta = 0.25;
        parameters.gamma = 0.5;
        parameters.maxIter = 32;
        parameters.tol = 1.0e-5;
        solver =
            std::make_unique<SolverNameSpace::SolverNewmark>(parameters);
    }

    QDir directory;
    if (!directory.mkpath(outputDirectory))
        return 2;
    QFile resultFile(
        QDir(outputDirectory).filePath(QStringLiteral("response.csv")));
    if (!resultFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return 3;
    QTextStream stream(&resultFile);
    stream << "time,u_x,u_y,u_z\n";

    double minimumX = std::numeric_limits<double>::infinity();
    double maximumAbsY = 0.0;
    double maximumAbsZ = 0.0;
    solver->SetStepCallback(
        [&model, &stream, &minimumX, &maximumAbsY, &maximumAbsZ](
            int, double time, const Eigen::VectorXd&)
        {
            const Eigen::Vector3d displacement = model.EndDisplacement();
            stream << QString::number(time, 'f', 6) << ','
                   << QString::number(displacement.x(), 'g', 16) << ','
                   << QString::number(displacement.y(), 'g', 16) << ','
                   << QString::number(displacement.z(), 'g', 16) << '\n';
            minimumX = std::min(minimumX, displacement.x());
            maximumAbsY = std::max(maximumAbsY, std::abs(displacement.y()));
            maximumAbsZ = std::max(maximumAbsZ, std::abs(displacement.z()));
        });

    if (!solver->Solve(model, duration))
        return 4;
    resultFile.close();

    constexpr double referenceMinimumX = -20.0;
    constexpr double referenceMaximumAbsY = 10.0;
    constexpr double referenceMaximumAbsZ = 3.5;
    const double errorX =
        std::abs(minimumX - referenceMinimumX)
        / std::abs(referenceMinimumX);
    const double errorY =
        std::abs(maximumAbsY - referenceMaximumAbsY)
        / referenceMaximumAbsY;
    const double errorZ =
        std::abs(maximumAbsZ - referenceMaximumAbsZ)
        / referenceMaximumAbsZ;

    QFile summaryFile(
        QDir(outputDirectory).filePath(QStringLiteral("summary.txt")));
    if (summaryFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream summary(&summaryFile);
        summary << "Le et al. (2012), Example 4\n"
                << "quantity,calculated,figure_reference,relative_error\n"
                << "minimum_u_x," << minimumX << ','
                << referenceMinimumX << ',' << errorX << '\n'
                << "maximum_abs_u_y," << maximumAbsY << ','
                << referenceMaximumAbsY << ',' << errorY << '\n'
                << "maximum_abs_u_z," << maximumAbsZ << ','
                << referenceMaximumAbsZ << ',' << errorZ << '\n';
    }

    QTextStream(stdout)
        << "Le2012 Example4 min_ux=" << minimumX
        << " max_abs_uy=" << maximumAbsY
        << " max_abs_uz=" << maximumAbsZ
        << " error_ux=" << errorX
        << " error_uy=" << errorY
        << " error_uz=" << errorZ
        << " result=" << resultFile.fileName() << Qt::endl;

    // Fig. 9 reference envelopes read from the published axes:
    // ux_min ~= -20, |uy|max ~= 10, |uz|max ~= 3.5.
    const bool matchesPaper =
        minimumX >= -22.0 && minimumX <= -18.0
        && maximumAbsY >= 9.0 && maximumAbsY <= 11.0
        && maximumAbsZ >= 2.5 && maximumAbsZ <= 4.5;
    return matchesPaper ? 0 : 5;
}
