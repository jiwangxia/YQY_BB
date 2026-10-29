#include "Dynamics/SpatialBeam/SpatialFrameTest.h"
#include "Dynamics/SpatialBeam/SpatialRigidMotionTest.h"

#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Inertia/RigidBodyInertia.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionBase.h"
#include "Solver/Dynamic/Integrators/SolverAdaptiveTSSBN.h"
#include "Solver/Dynamic/Integrators/SolverNewmark.h"
#include "Solver/Dynamic/Integrators/SolverRungeKutta4.h"
#include "Solver/Factory/SolverFactory.h"

#include <Eigen/Geometry>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace
{
double PositiveEnvironmentValue(const char* name, double fallback)
{
    bool valid = false;
    const double value = QString::fromLocal8Bit(qgetenv(name)).toDouble(&valid);
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
    FreeFreeBeamWithDisks,
    Duan2026Ring
};

class Le2012BeamModel final : public SolverNameSpace::IAnalysisModel
{
public:
    explicit Le2012BeamModel(Le2012Example example, int beamElementsPerLeg = 10, bool fusedAssembly = false)
        : m_example(example)
        , m_beamElementsPerLeg(beamElementsPerLeg)
        , m_fusedAssembly(fusedAssembly)
    {
        const bool rightAngle = m_example == Le2012Example::RightAngleCantilever;
        const bool ring = m_example == Le2012Example::Duan2026Ring;
        const int elementCount = rightAngle ? 2 * beamElementsPerLeg : ring ? 32 : 10;
        const double elementLength = rightAngle ? 10.0 / beamElementsPerLeg : 1.0;

        m_material = std::make_shared<Material>();
        m_material->m_Young = rightAngle ? 1.0e6 : 1.0e4;
        m_material->m_Poisson = -0.5; // G = E, as required by Table 2.
        m_material->m_Density = 1.0;  // A_rho = rho*A = 1.
        // 表 2 给出算例 1 的 EI = GJ = 1000，算例 4 为 500。
        m_section = std::make_shared<Le2012Section>(rightAngle ? 1.0e-3 : 5.0e-2);
        m_property = std::make_shared<Property>();
        m_property->m_pMaterial = m_material;
        m_property->m_pSection = m_section;

        const int nodeCount = ring ? elementCount : elementCount + 1;
        m_nodes.reserve(nodeCount);
        m_freeDofs.resize(nodeCount);
        int freeDof = 0;
        for (int nodeIndex = 0; nodeIndex < nodeCount; ++nodeIndex)
        {
            auto node = std::make_shared<Node>();
            node->SetNumDOFs(6);
            if (rightAngle)
            {
                if (nodeIndex <= beamElementsPerLeg)
                    node->m_Y = nodeIndex * elementLength;
                else
                {
                    node->m_X = -(nodeIndex - beamElementsPerLeg) * elementLength;
                    node->m_Y = 10.0;
                }
            }
            else if (ring)
            {
                const double angle = 2.0 * std::acos(-1.0) * nodeIndex / elementCount;
                node->m_X = 10.0 * std::cos(angle);
                node->m_Y = 10.0 * std::sin(angle);
            }
            else
            {
                // 图 3.2：10 m 梁的初始位置跨越全局 x-y 平面内的 6-8-10 三角形。
                const double fraction = static_cast<double>(nodeIndex) / static_cast<double>(elementCount);
                node->m_X = 6.0 * fraction;
                node->m_Y = 8.0 * (1.0 - fraction);
            }
            m_nodes.push_back(node);

            m_freeDofs[nodeIndex].fill(-1);
            if (nodeIndex == 0 && rightAngle)
            {
                // 图 2：直角框架的第一端固定。
            }
            else
            {
                for (int component = 0; component < 6; ++component)
                    m_freeDofs[nodeIndex][component] = freeDof++;
            }
        }
        m_freeDofCount = freeDof;

        m_elements.reserve(elementCount);
        m_elementNodes.reserve(elementCount);
        for (int elementIndex = 0; elementIndex < elementCount; ++elementIndex)
        {
            const int secondNode = ring ? (elementIndex + 1) % nodeCount : elementIndex + 1;
            auto element = std::make_shared<ElementBeam_CR>();
            element->m_pNode[0] = m_nodes[elementIndex];
            element->m_pNode[1] = m_nodes[secondNode];
            element->m_pProperty = m_property;
            element->q0 = rightAngle ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitZ();
            element->Get_L0();
            m_elements.push_back(element);
            m_elementNodes.push_back({elementIndex, secondNode});
        }
        m_inertiaForce = Eigen::VectorXd::Zero(m_freeDofCount);
    }

    int GetFreeDofs() const override
    {
        return m_freeDofCount;
    }
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
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            std::array<bool, 3> translationActive{};
            std::array<bool, 3> rotationActive{};
            for (int component = 0; component < 3; ++component)
            {
                translationActive[component] = m_freeDofs[nodeIndex][component] >= 0;
                rotationActive[component] = m_freeDofs[nodeIndex][component + 3] >= 0;
            }
            m_nodes[nodeIndex]->BeginNewmarkStep(dt, beta, gamma, translationActive, rotationActive);
        }
    }

    void ApplyDynamicCorrection(const SolverNameSpace::Vec& dx, double a0, double a1) override
    {
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            Eigen::Vector3d translation = Eigen::Vector3d::Zero();
            Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
            for (int component = 0; component < 3; ++component)
            {
                const int translationDof = m_freeDofs[nodeIndex][component];
                const int rotationDof = m_freeDofs[nodeIndex][component + 3];
                if (translationDof >= 0)
                    translation(component) = dx[translationDof];
                if (rotationDof >= 0)
                    rotation(component) = dx[rotationDof];
            }
            m_nodes[nodeIndex]->ApplyNewmarkCorrection(translation, rotation, a0, a1);
        }
    }

    void RollbackDynamicStep() override
    {
        for (const auto& node : m_nodes)
            node->RollbackNewmarkStep();
        for (const auto& element : m_elements)
            element->RestoreState();
    }

    bool PushStateCheckpoint() override
    {
        if (!m_checkpointNodes.empty())
            return false;
        m_checkpointNodes.clear();
        m_checkpointNodes.reserve(m_nodes.size());
        for (const auto& node : m_nodes)
            m_checkpointNodes.push_back(*node);
        for (const auto& element : m_elements)
        {
            if (!element->PushStateCheckpoint())
            {
                for (const auto& previous : m_elements)
                {
                    if (previous == element)
                        break;
                    previous->DiscardStateCheckpoint();
                }
                m_checkpointNodes.clear();
                return false;
            }
        }
        return true;
    }

    bool RestoreStateCheckpoint() override
    {
        if (m_checkpointNodes.size() != m_nodes.size())
            return false;
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
            *m_nodes[nodeIndex] = m_checkpointNodes[nodeIndex];
        for (const auto& element : m_elements)
            if (!element->RestoreStateCheckpoint())
                return false;
        return true;
    }

    void DiscardStateCheckpoint() override
    {
        for (const auto& element : m_elements)
            element->DiscardStateCheckpoint();
        m_checkpointNodes.clear();
    }

    void ReportProgress(double progress, const QString& message) override
    {
        Q_UNUSED(progress);
        m_lastSolverMessage = message;
    }

    QString LastSolverMessage() const
    {
        return m_lastSolverMessage;
    }

    void SetTrialKinematics(const SolverNameSpace::Vec& velocity, const SolverNameSpace::Vec& acceleration) override
    {
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            for (int component = 0; component < 6; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component];
                if (dof >= 0)
                {
                    m_nodes[nodeIndex]->m_Velocity[component] = velocity[dof];
                    m_nodes[nodeIndex]->m_Acceleration[component] = acceleration[dof];
                }
            }
            auto& node = *m_nodes[nodeIndex];
            const Eigen::Vector3d spatialVelocity(node.m_Velocity[3], node.m_Velocity[4], node.m_Velocity[5]);
            const Eigen::Vector3d spatialAcceleration(node.m_Acceleration[3], node.m_Acceleration[4],
                                                      node.m_Acceleration[5]);
            node.m_OmegaMaterial = node.m_Rg.transpose() * spatialVelocity;
            node.m_AlphaMaterial = node.m_Rg.transpose() * spatialAcceleration;
        }
    }

    bool NormalizeTssbnAcceptedRotations() override
    {
        for (const auto& node : m_nodes)
            if (!node || !node->IsTssbnAcceptedRotationValid())
                return false;
        for (const auto& node : m_nodes)
            if (!node->NormalizeTssbnAcceptedRotation())
                return false;
        return true;
    }

    void BeginTssbnImplicitStage(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                 double secondStageDiagonalFraction) override
    {
        for (const auto& node : m_nodes)
            node->BeginTssbnImplicitStage(stageIndex, timeStep, firstStageTime, secondStageTime,
                                          secondStageDiagonalFraction);
    }

    void ApplyTssbnImplicitStageCorrection(const SolverNameSpace::Vec& increment) override
    {
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            Eigen::Vector3d translation = Eigen::Vector3d::Zero();
            Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
            for (int component = 0; component < 3; ++component)
            {
                const int translationDof = m_freeDofs[nodeIndex][component];
                const int rotationDof = m_freeDofs[nodeIndex][component + 3];
                if (translationDof >= 0)
                    translation[component] = increment[translationDof];
                if (rotationDof >= 0)
                    rotation[component] = increment[rotationDof];
            }
            m_nodes[nodeIndex]->ApplyTssbnImplicitStageCorrection(translation, rotation);
        }
    }

    void SetTssbnStageKinematics(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                 double secondStageDiagonalFraction, _OUT SolverNameSpace::Vec& velocity,
                                 _OUT SolverNameSpace::Vec& acceleration, bool useStoredIncrement = false) override
    {
        SetTrialKinematics(velocity, acceleration);
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            Eigen::Vector3d angularVelocity;
            Eigen::Vector3d angularAcceleration;
            m_nodes[nodeIndex]->SetTssbnStageKinematics(stageIndex, timeStep, firstStageTime, secondStageTime,
                                                        secondStageDiagonalFraction, angularVelocity,
                                                        angularAcceleration, useStoredIncrement);
            for (int component = 0; component < 3; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component + 3];
                if (dof < 0)
                    continue;
                velocity[dof] = angularVelocity[component];
                acceleration[dof] = angularAcceleration[component];
            }
        }
    }

    void CorrectTssbnExplicitLastStage(double timeStep, double lastStageFirstCoefficient,
                                       double lastStageSecondCoefficient, _OUT SolverNameSpace::Vec& stepIncrement,
                                       _OUT SolverNameSpace::Vec& velocity) override
    {
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            const Node::TssbnExplicitRotationState rotation = m_nodes[nodeIndex]->IntegrateTssbnExplicitLastStage(
                timeStep, lastStageFirstCoefficient, lastStageSecondCoefficient);
            for (int component = 0; component < 3; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component + 3];
                if (dof < 0)
                    continue;
                stepIncrement[dof] = rotation.spatialIncrement[component];
                velocity[dof] = rotation.spatialVelocity[component];
            }
        }
    }

    void CorrectTssbnStepStates(double timeStep, double firstStageTime, double secondStageTime, double lastStageTime,
                                double baseFirstWeight, double embeddedFirstWeight, double embeddedSecondWeight,
                                double embeddedLastWeight, double lastStageFirstCoefficient,
                                double lastStageSecondCoefficient, _OUT SolverNameSpace::Vec& baseIncrement,
                                _OUT SolverNameSpace::Vec& baseVelocity, _OUT SolverNameSpace::Vec& embeddedIncrement,
                                _OUT SolverNameSpace::Vec& embeddedVelocity,
                                _OUT SolverNameSpace::Vec& acceptedAcceleration) override
    {
        const double extrapolation = (lastStageTime - secondStageTime) / (secondStageTime - firstStageTime);
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            const auto& node = m_nodes[nodeIndex];
            const Node::TssbnRotationState rotation = node->IntegrateTssbnRotation(
                timeStep, extrapolation, baseFirstWeight, embeddedFirstWeight, embeddedSecondWeight, embeddedLastWeight,
                lastStageFirstCoefficient, lastStageSecondCoefficient);

            for (int component = 0; component < 3; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component + 3];
                if (dof < 0)
                    continue;
                baseIncrement[dof] = rotation.baseSpatialIncrement[component];
                embeddedIncrement[dof] = rotation.embeddedSpatialIncrement[component];
                baseVelocity[dof] = rotation.baseSpatialVelocity[component];
                embeddedVelocity[dof] = rotation.embeddedSpatialVelocity[component];
                acceptedAcceleration[dof] = rotation.acceptedSpatialAcceleration[component];
            }
        }
    }

    void CorrectTssbnEmbeddedVelocity(double timeStep, double embeddedFirstWeight, double embeddedSecondWeight,
                                      double embeddedLastWeight, double lastStageFirstCoefficient,
                                      double lastStageSecondCoefficient,
                                      const SolverNameSpace::Vec& explicitLastStageAcceleration,
                                      _OUT SolverNameSpace::Vec& embeddedVelocity) override
    {
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            Eigen::Vector3d explicitSpatialAcceleration = Eigen::Vector3d::Zero();
            for (int component = 0; component < 3; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component + 3];
                explicitSpatialAcceleration[component] =
                    dof >= 0 ? explicitLastStageAcceleration[dof] : m_nodes[nodeIndex]->m_Acceleration[component + 3];
            }
            const Eigen::Vector3d correctedVelocity = m_nodes[nodeIndex]->IntegrateTssbnEmbeddedVelocity(
                timeStep, embeddedFirstWeight, embeddedSecondWeight, embeddedLastWeight, lastStageFirstCoefficient,
                lastStageSecondCoefficient, explicitSpatialAcceleration);
            for (int component = 0; component < 3; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component + 3];
                if (dof >= 0)
                    embeddedVelocity[dof] = correctedVelocity[component];
            }
        }
    }

    void GetState(SolverNameSpace::Vec& displacement, SolverNameSpace::Vec& velocity,
                  SolverNameSpace::Vec& acceleration) const override
    {
        displacement = Eigen::VectorXd::Zero(m_freeDofCount);
        velocity = Eigen::VectorXd::Zero(m_freeDofCount);
        acceleration = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            for (int component = 0; component < 6; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component];
                if (dof < 0)
                    continue;
                displacement[dof] = m_nodes[nodeIndex]->m_Displacement[component];
                velocity[dof] = m_nodes[nodeIndex]->m_Velocity[component];
                acceleration[dof] = m_nodes[nodeIndex]->m_Acceleration[component];
            }
        }
    }

    void Assemble_Matrix(SolverNameSpace::SpMat& stiffness, bool) override
    {
        std::vector<Eigen::Triplet<double>> triplets;
        m_internalForce = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int elementIndex = 0; elementIndex < static_cast<int>(m_elements.size()); ++elementIndex)
        {
            Eigen::MatrixXd elementStiffness;
            m_elements[elementIndex]->Get_ke(elementStiffness);
            AssembleElementMatrix(elementIndex, elementStiffness, triplets);
            AssembleElementVector(elementIndex, m_elements[elementIndex]->m_inforce, m_internalForce);
        }
        stiffness.resize(m_freeDofCount, m_freeDofCount);
        stiffness.setFromTriplets(triplets.begin(), triplets.end());
    }

    void AssembleDynamicSystem(SolverNameSpace::SpMat& mass, SolverNameSpace::SpMat& gyroscopic,
                               SolverNameSpace::SpMat& centrifugal) override
    {
        std::vector<Eigen::Triplet<double>> massTriplets;
        std::vector<Eigen::Triplet<double>> gyroscopicTriplets;
        std::vector<Eigen::Triplet<double>> centrifugalTriplets;
        m_inertiaForce = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int elementIndex = 0; elementIndex < static_cast<int>(m_elements.size()); ++elementIndex)
        {
            Eigen::MatrixXd elementMass;
            Eigen::MatrixXd elementGyroscopic;
            Eigen::MatrixXd elementCentrifugal;
            Eigen::VectorXd elementInertia;
            if (m_fusedAssembly)
            {
                // 仅复用生产融合入口，单元公式、装配次序及求解控制保持不变。
                m_elements[elementIndex]->GetDynamicContributions(elementMass, elementInertia, elementGyroscopic,
                                                                  elementCentrifugal);
            }
            else
            {
                m_elements[elementIndex]->Get_me_Consistent(elementMass);
                m_elements[elementIndex]->Get_GyroscopicMatrix(elementGyroscopic);
                m_elements[elementIndex]->Get_CentrifugalMatrix(elementCentrifugal);
                m_elements[elementIndex]->Get_InertiaForce(elementInertia);
            }
            AssembleElementMatrix(elementIndex, elementMass, massTriplets);
            AssembleElementMatrix(elementIndex, elementGyroscopic, gyroscopicTriplets);
            AssembleElementMatrix(elementIndex, elementCentrifugal, centrifugalTriplets);
            AssembleElementVector(elementIndex, elementInertia, m_inertiaForce);
        }
        if (m_example == Le2012Example::FreeFreeBeamWithDisks)
        {
            // 圆盘主惯量固定在初始随体轴中；复用正式刚体惯性力和有限转动隐式切线。
            for (const int nodeIndex : {0, static_cast<int>(m_nodes.size()) - 1})
            {
                const RigidBodyInertia disk = GetEndDisk(nodeIndex);
                Eigen::MatrixXd diskMass, diskGyroscopic, diskCentrifugal;
                Eigen::VectorXd diskInertiaForce;
                disk.GetDynamicContributions(diskMass, diskInertiaForce, diskGyroscopic, diskCentrifugal);
                AssembleNodeMatrix(nodeIndex, diskMass, massTriplets);
                AssembleNodeMatrix(nodeIndex, diskGyroscopic, gyroscopicTriplets);
                AssembleNodeMatrix(nodeIndex, diskCentrifugal, centrifugalTriplets);
                for (int component = 0; component < 6; ++component)
                    m_inertiaForce[m_freeDofs[nodeIndex][component]] += diskInertiaForce[component];
            }
        }
        mass.resize(m_freeDofCount, m_freeDofCount);
        gyroscopic.resize(m_freeDofCount, m_freeDofCount);
        centrifugal.resize(m_freeDofCount, m_freeDofCount);
        mass.setFromTriplets(massTriplets.begin(), massTriplets.end());
        gyroscopic.setFromTriplets(gyroscopicTriplets.begin(), gyroscopicTriplets.end());
        centrifugal.setFromTriplets(centrifugalTriplets.begin(), centrifugalTriplets.end());
    }

    void AssembleMassMatrix(_OUT SolverNameSpace::SpMat& mass) override
    {
        std::vector<Eigen::Triplet<double>> triplets;
        for (int elementIndex = 0; elementIndex < static_cast<int>(m_elements.size()); ++elementIndex)
        {
            Eigen::MatrixXd elementMass;
            m_elements[elementIndex]->Get_me_Physical(elementMass);
            AssembleElementMatrix(elementIndex, elementMass, triplets);
        }
        if (m_example == Le2012Example::FreeFreeBeamWithDisks)
        {
            for (const int nodeIndex : {0, static_cast<int>(m_nodes.size()) - 1})
            {
                Eigen::MatrixXd diskMass;
                GetEndDisk(nodeIndex).GetPhysicalMass(diskMass);
                AssembleNodeMatrix(nodeIndex, diskMass, triplets);
            }
        }
        mass.resize(m_freeDofCount, m_freeDofCount);
        mass.setFromTriplets(triplets.begin(), triplets.end());
    }

    bool VerifyDiskInertia(_OUT QString& message)
    {
        if (m_example != Le2012Example::FreeFreeBeamWithDisks || !PushStateCheckpoint())
            return false;
        const Eigen::Vector3d increment(0.3, -0.2, 0.4);
        const Eigen::Matrix3d previousRotation =
            Eigen::AngleAxisd(0.6, Eigen::Vector3d(1.0, 2.0, -1.0).normalized()).toRotationMatrix();
        const Eigen::Matrix3d currentRotation =
            Eigen::AngleAxisd(increment.norm(), increment.normalized()).toRotationMatrix() * previousRotation;
        const Eigen::Vector3d omega(0.4, 0.7, 1.1);
        const Eigen::Vector3d alpha(-0.2, 0.3, 0.5);
        const Eigen::Vector3d translationAcceleration(0.1, -0.3, 0.2);
        SolverNameSpace::Vec velocity = SolverNameSpace::Vec::Zero(m_freeDofCount);
        SolverNameSpace::Vec acceleration = SolverNameSpace::Vec::Zero(m_freeDofCount);
        for (const int nodeIndex : {0, static_cast<int>(m_nodes.size()) - 1})
        {
            m_nodes[nodeIndex]->m_Rg_n = previousRotation;
            m_nodes[nodeIndex]->m_Rg = currentRotation;
            m_nodes[nodeIndex]->m_StepRotation = increment;
            for (int component = 0; component < 3; ++component)
            {
                velocity[m_freeDofs[nodeIndex][component + 3]] = omega[component];
                acceleration[m_freeDofs[nodeIndex][component]] = translationAcceleration[component];
                acceleration[m_freeDofs[nodeIndex][component + 3]] = alpha[component];
            }
        }
        SetTrialKinematics(velocity, acceleration);

        // 独立装配梁贡献，再按空间 Euler 方程加入圆盘，核对实际模型入口。
        std::vector<Eigen::Triplet<double>> expectedTriplets;
        Eigen::VectorXd expectedForce = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int elementIndex = 0; elementIndex < static_cast<int>(m_elements.size()); ++elementIndex)
        {
            Eigen::MatrixXd elementMass;
            Eigen::VectorXd elementForce;
            m_elements[elementIndex]->Get_me_Physical(elementMass);
            m_elements[elementIndex]->Get_InertiaForce(elementForce);
            AssembleElementMatrix(elementIndex, elementMass, expectedTriplets);
            AssembleElementVector(elementIndex, elementForce, expectedForce);
        }
        const Eigen::Matrix3d materialInertia = Eigen::Vector3d(200.0, 100.0, 100.0).asDiagonal();
        const Eigen::Matrix3d spatialInertia = currentRotation * materialInertia * currentRotation.transpose();
        const Eigen::Vector3d gyroscopicMoment = omega.cross(spatialInertia * omega);
        const Eigen::Vector3d expectedMoment = spatialInertia * alpha + gyroscopicMoment;
        for (const int nodeIndex : {0, static_cast<int>(m_nodes.size()) - 1})
        {
            Eigen::MatrixXd expectedDiskMass = Eigen::MatrixXd::Zero(6, 6);
            expectedDiskMass.block<3, 3>(0, 0) = 10.0 * Eigen::Matrix3d::Identity();
            expectedDiskMass.block<3, 3>(3, 3) = spatialInertia;
            AssembleNodeMatrix(nodeIndex, expectedDiskMass, expectedTriplets);
            for (int component = 0; component < 3; ++component)
            {
                expectedForce[m_freeDofs[nodeIndex][component]] += 10.0 * translationAcceleration[component];
                expectedForce[m_freeDofs[nodeIndex][component + 3]] += expectedMoment[component];
            }
        }
        SolverNameSpace::SpMat expectedMass(m_freeDofCount, m_freeDofCount);
        expectedMass.setFromTriplets(expectedTriplets.begin(), expectedTriplets.end());
        SolverNameSpace::SpMat actualMass, implicitMass, gyroscopic, centrifugal;
        AssembleMassMatrix(actualMass);
        AssembleDynamicSystem(implicitMass, gyroscopic, centrifugal);
        const double massError = (actualMass - expectedMass).norm();
        const double forceError = (m_inertiaForce - expectedForce).norm();
        const double oldMassDifference = (spatialInertia - materialInertia).norm();
        const double oldForceDifference = (expectedMoment - materialInertia * alpha).norm();
        const double gyroMagnitude = gyroscopicMoment.norm();
        const bool restored = RestoreStateCheckpoint();
        DiscardStateCheckpoint();
        m_inertiaForce.setZero();
        message = QStringLiteral("mass_error=%1 force_error=%2 old_global_mass_difference=%3 "
                                 "old_global_force_difference=%4 gyro_moment=%5")
                      .arg(massError, 0, 'g', 16)
                      .arg(forceError, 0, 'g', 16)
                      .arg(oldMassDifference, 0, 'g', 16)
                      .arg(oldForceDifference, 0, 'g', 16)
                      .arg(gyroMagnitude, 0, 'g', 16);
        return restored && std::isfinite(massError) && std::isfinite(forceError) &&
               massError <= 1.0e-12 * std::max(1.0, expectedMass.norm()) &&
               forceError <= 1.0e-12 * std::max(1.0, expectedForce.norm()) && oldMassDifference > 1.0 &&
               oldForceDifference > 1.0 && gyroMagnitude > 1.0;
    }

    void ComputeExternalForce(double time, double, SolverNameSpace::Vec& fixedForce,
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
            freeForce[m_freeDofs[m_beamElementsPerLeg][2]] = forceZ;
            return;
        }

        if (m_example == Le2012Example::Duan2026Ring)
        {
            const double forceZ = time <= 2.5 ? 4.0 * time : time <= 5.0 ? 4.0 * (5.0 - time) : 0.0;
            freeForce[m_freeDofs[0][2]] = forceZ;
            freeForce[m_freeDofs[8][2]] = -forceZ;
            return;
        }

        const int endNode = static_cast<int>(m_nodes.size()) - 1;
        double forceZ = 0.0;
        if (time <= 2.5)
            forceZ = 8.0 * time;
        else if (time <= 5.0)
            forceZ = 8.0 * (5.0 - time);

        // 图 3.2：右下圆盘施加平面内力 Fx=2Fz，两圆盘施加方向相反的平面外力。
        freeForce[m_freeDofs[endNode][0]] = -2.0 * forceZ;
        freeForce[m_freeDofs[endNode][2]] = forceZ;
        freeForce[m_freeDofs[0][2]] = -forceZ;
    }

    void Assemble_Constraint(SolverNameSpace::Vec& displacement, double, double) override
    {
        displacement = Eigen::VectorXd::Zero(GetFixedDofs());
    }

    void ComputeResidual(const SolverNameSpace::Vec& externalForce, SolverNameSpace::Vec& residual) override
    {
        residual = externalForce - m_internalForce - m_inertiaForce;
    }

    void ComputeStaticResidual(const SolverNameSpace::Vec& externalForce, SolverNameSpace::Vec& residual) override
    {
        residual = externalForce - m_internalForce;
    }

    void CalculateReactions(_OUT SolverNameSpace::Vec& force) override
    {
        force = Eigen::VectorXd::Zero(GetFixedDofs());
    }

    void OnStepCompleted(double) override
    {
    }
    void CommitState() override
    {
        for (const auto& element : m_elements)
            element->CommitState();
        for (const auto& node : m_nodes)
        {
            node->m_Displacement_n = node->m_Displacement;
            node->m_Velocity_n = node->m_Velocity;
            node->m_Acceleration_n = node->m_Acceleration;
            node->m_Rg_n = node->m_Rg;
            node->m_OmegaMaterial_n = node->m_OmegaMaterial;
            node->m_AlphaMaterial_n = node->m_AlphaMaterial;
            node->m_StepTranslation.setZero();
            node->m_StepScalarRotation = 0.0;
            node->m_StepRotation.setZero();
        }
    }
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
            node->m_StepTranslation.setZero();
            node->m_StepScalarRotation = 0.0;
            node->m_StepRotation.setZero();
        }
        for (const auto& element : m_elements)
            element->BackupState();
    }

    void GetStepIncrement(SolverNameSpace::Vec& increment) const override
    {
        increment = Eigen::VectorXd::Zero(m_freeDofCount);
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            const auto& node = *m_nodes[nodeIndex];
            for (int component = 0; component < 3; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component];
                if (dof >= 0)
                {
                    increment[dof] = node.m_StepTranslation[component];
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

    bool SupportsTssbnRotationPrecisionEstimate() const override
    {
        return true;
    }

    void GetFiniteRotationStates(_OUT std::vector<SolverNameSpace::FiniteRotationState>& states) const override
    {
        states.clear();
        states.reserve(m_nodes.size());
        for (int nodeIndex = 0; nodeIndex < static_cast<int>(m_nodes.size()); ++nodeIndex)
        {
            SolverNameSpace::FiniteRotationState state;
            state.rotation = m_nodes[nodeIndex]->m_Rg;
            for (int component = 0; component < 3; ++component)
                state.freeDofs[component] = m_freeDofs[nodeIndex][component + 3];
            states.push_back(state);
        }
    }

    void GetAuxiliaryFiniteRotationStates(_OUT std::vector<Eigen::Matrix3d>& states) const override
    {
        states.clear();
        states.reserve(2 * m_elements.size());
        for (const auto& element : m_elements)
        {
            if (element->IsReferenceTssbnDynamicMode())
                continue;
            std::array<Matrix3d, 2> rotations;
            element->GetCurrentGaussRotations(rotations);
            states.insert(states.end(), rotations.begin(), rotations.end());
        }
    }

    Eigen::Vector3d EndDisplacement() const
    {
        return NodeDisplacement(static_cast<int>(m_nodes.size()) - 1);
    }

    Eigen::Vector3d NodeDisplacement(int nodeIndex) const
    {
        const auto& displacement = m_nodes[nodeIndex]->m_Displacement;
        return Eigen::Vector3d(displacement[0], displacement[1], displacement[2]);
    }

    int NodeCount() const
    {
        return static_cast<int>(m_nodes.size());
    }

    int ElementCount() const
    {
        return static_cast<int>(m_elements.size());
    }

    int ElbowNodeIndex() const
    {
        return m_beamElementsPerLeg;
    }

    Eigen::Vector3d NodePosition(int nodeIndex) const
    {
        const auto& node = *m_nodes[nodeIndex];
        return Eigen::Vector3d(node.m_X, node.m_Y, node.m_Z) + NodeDisplacement(nodeIndex);
    }

    void WriteDynamicStates(double time, _OUT QTextStream& output) const
    {
        for (int index = 0; index < static_cast<int>(m_nodes.size()); ++index)
        {
            const auto& node = *m_nodes[index];
            const auto position = NodePosition(index);
            output << time << ',' << index;
            for (int axis = 0; axis < 3; ++axis)
                output << ',' << position[axis];
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    output << ',' << node.m_Rg(row, column);
            for (const auto* values : {&node.m_Velocity, &node.m_Acceleration})
                for (int axis = 0; axis < 6; ++axis)
                    output << ',' << (*values)[axis];
            output << '\n';
        }
    }

    std::array<double, 3> MechanicalEnergy()
    {
        SolverNameSpace::SpMat mass;
        AssembleMassMatrix(mass);
        SolverNameSpace::Vec displacement, velocity, acceleration;
        GetState(displacement, velocity, acceleration);
        const double kinetic = 0.5 * velocity.dot(mass * velocity);
        double strain = 0.0;
        for (const auto& element : m_elements)
            strain += element->GetStrainEnergy();
        return {kinetic, strain, kinetic + strain};
    }

    std::array<double, 3> ObservedMechanicalEnergy(_OUT Le2012BeamModel& observer) const
    {
        if (observer.NodeCount() != NodeCount() || observer.ElementCount() != ElementCount())
            throw std::runtime_error("Duan energy observer model dimensions differ");
        // 复用独立观察模型；完整复制节点、单元缓存及 Gauss 历史，不改写求解模型。
        for (int index = 0; index < NodeCount(); ++index)
            *observer.m_nodes[index] = *m_nodes[index];
        for (int index = 0; index < ElementCount(); ++index)
        {
            *observer.m_elements[index] = *m_elements[index];
            observer.m_elements[index]->m_pProperty = observer.m_property;
            for (int localNode = 0; localNode < 2; ++localNode)
                observer.m_elements[index]->m_pNode[localNode] =
                    observer.m_nodes[m_elementNodes[index][localNode]];
        }
        return observer.MechanicalEnergy();
    }

    bool WritePhysicalDiagnostics(double time, const QString& outputDirectory, const QString& phase) const
    {
        // 完整复制节点、单元缓存及 Gauss 历史；所有临时加速度和装配仅作用于副本。
        QString originalStates;
        QTextStream originalStream(&originalStates);
        originalStream.setRealNumberPrecision(17);
        WriteDynamicStates(time, originalStream);
        originalStream.flush();
        const Eigen::VectorXd originalInternal = m_internalForce;
        const Eigen::VectorXd originalInertia = m_inertiaForce;
        SolverNameSpace::Vec originalQ, originalV, originalA;
        GetState(originalQ, originalV, originalA);
        std::vector<Eigen::Matrix3d> originalRotations;
        for (const auto& node : m_nodes)
            originalRotations.push_back(node->m_Rg);
        Le2012BeamModel diagnostic(m_example, m_beamElementsPerLeg, m_fusedAssembly);
        for (int index = 0; index < NodeCount(); ++index)
            *diagnostic.m_nodes[index] = *m_nodes[index];
        for (int index = 0; index < ElementCount(); ++index)
        {
            *diagnostic.m_elements[index] = *m_elements[index];
            diagnostic.m_elements[index]->m_pProperty = diagnostic.m_property;
            for (int localNode = 0; localNode < 2; ++localNode)
                diagnostic.m_elements[index]->m_pNode[localNode] =
                    diagnostic.m_nodes[m_elementNodes[index][localNode]];
        }
        diagnostic.m_internalForce = m_internalForce;
        diagnostic.m_inertiaForce = m_inertiaForce;

        SolverNameSpace::SpMat stiffness, mass;
        diagnostic.Assemble_Matrix(stiffness, true);
        diagnostic.AssembleMassMatrix(mass);
        const Eigen::MatrixXd denseMass(mass);
        const Eigen::MatrixXd denseStiffness(stiffness);
        SolverNameSpace::Vec displacement, velocity, rawAcceleration, fixedForce, externalForce;
        diagnostic.GetState(displacement, velocity, rawAcceleration);
        diagnostic.ComputeExternalForce(time, 1.0, fixedForce, externalForce);
        const Eigen::VectorXd internalForce = diagnostic.m_internalForce;
        const Eigen::VectorXd rawInertia = diagnostic.PhysicalInertiaForDiagnostics();
        const Eigen::VectorXd massAcceleration = mass * rawAcceleration;
        const auto mechanicalEnergy = diagnostic.MechanicalEnergy();
        diagnostic.SetTrialKinematics(velocity, Eigen::VectorXd::Zero(GetFreeDofs()));
        const Eigen::VectorXd gyroscopicForce = diagnostic.PhysicalInertiaForDiagnostics();
        Eigen::LDLT<Eigen::MatrixXd> factor(denseMass);
        if (factor.info() != Eigen::Success || !factor.isPositive())
            throw std::runtime_error("Duan physical diagnostic mass factorization is not positive definite");
        const Eigen::VectorXd recoveryRightHandSide = externalForce - internalForce - gyroscopicForce;
        const Eigen::VectorXd recoveredAcceleration = factor.solve(recoveryRightHandSide);
        if (factor.info() != Eigen::Success || !recoveredAcceleration.allFinite())
            throw std::runtime_error("Duan physical diagnostic acceleration recovery failed");
        diagnostic.SetTrialKinematics(velocity, recoveredAcceleration);
        const Eigen::VectorXd recoveredInertia = diagnostic.PhysicalInertiaForDiagnostics();
        const Eigen::VectorXd rawResidual = externalForce - internalForce - rawInertia;
        const Eigen::VectorXd recoveredResidual = externalForce - internalForce - recoveredInertia;
        const Eigen::VectorXd inertiaSplitError = rawInertia - massAcceleration - gyroscopicForce;
        const Eigen::VectorXd recoveryLinearResidual = mass * recoveredAcceleration - recoveryRightHandSide;

        for (const auto& namedMatrix : {std::make_pair(QStringLiteral("K"), &denseStiffness),
                                       std::make_pair(QStringLiteral("M"), &denseMass)})
        {
            QFile file(QDir(outputDirectory).filePath(phase + "-" + namedMatrix.first + ".csv"));
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
                return false;
            QTextStream output(&file);
            output.setRealNumberPrecision(17);
            output << "row,column,value\n";
            for (int row = 0; row < namedMatrix.second->rows(); ++row)
                for (int column = 0; column < namedMatrix.second->cols(); ++column)
                    output << row << ',' << column << ',' << (*namedMatrix.second)(row, column) << '\n';
        }

        QFile componentsFile(QDir(outputDirectory).filePath(phase + "-components.csv"));
        if (!componentsFile.open(QIODevice::WriteOnly | QIODevice::Text))
            return false;
        QTextStream components(&componentsFile);
        components.setRealNumberPrecision(17);
        components << "time,dof,node,component,q,v,raw_a,equilibrium_a,fext,fint,finertia,Ma,fgyro,"
                      "raw_residual,recovered_finertia,recovered_residual,inertia_split_error\n";
        double maximumRawTranslationResidual = 0.0;
        double maximumRawRotationResidual = 0.0;
        double maximumRecoveredTranslationResidual = 0.0;
        double maximumRecoveredRotationResidual = 0.0;
        double maximumRawScaledResidual = 0.0;
        double maximumRecoveredScaledResidual = 0.0;
        for (int nodeIndex = 0; nodeIndex < NodeCount(); ++nodeIndex)
        {
            for (int component = 0; component < 6; ++component)
            {
                const int dof = m_freeDofs[nodeIndex][component];
                if (dof < 0)
                    continue;
                components << time << ',' << dof << ',' << nodeIndex << ',' << component << ','
                           << displacement[dof] << ',' << velocity[dof] << ',' << rawAcceleration[dof] << ','
                           << recoveredAcceleration[dof] << ',' << externalForce[dof] << ',' << internalForce[dof]
                           << ',' << rawInertia[dof] << ',' << massAcceleration[dof] << ',' << gyroscopicForce[dof]
                           << ',' << rawResidual[dof] << ',' << recoveredInertia[dof] << ',' << recoveredResidual[dof]
                           << ',' << inertiaSplitError[dof] << '\n';
                double& rawMaximum =
                    component < 3 ? maximumRawTranslationResidual : maximumRawRotationResidual;
                double& recoveredMaximum =
                    component < 3 ? maximumRecoveredTranslationResidual : maximumRecoveredRotationResidual;
                rawMaximum = std::max(rawMaximum, std::abs(rawResidual[dof]));
                recoveredMaximum = std::max(recoveredMaximum, std::abs(recoveredResidual[dof]));
                const double forceScale =
                    1.0 + std::abs(externalForce[dof]) + std::abs(internalForce[dof]);
                maximumRawScaledResidual =
                    std::max(maximumRawScaledResidual, std::abs(rawResidual[dof]) / (forceScale + std::abs(rawInertia[dof])));
                maximumRecoveredScaledResidual =
                    std::max(maximumRecoveredScaledResidual,
                             std::abs(recoveredResidual[dof]) / (forceScale + std::abs(recoveredInertia[dof])));
            }
        }

        QFile gaussFile(QDir(outputDirectory).filePath(phase + "-gauss-rotations.csv"));
        if (!gaussFile.open(QIODevice::WriteOnly | QIODevice::Text))
            return false;
        QTextStream gauss(&gaussFile);
        gauss.setRealNumberPrecision(17);
        gauss << "time,element,point,r00,r01,r02,r10,r11,r12,r20,r21,r22\n";
        for (int element = 0; element < ElementCount(); ++element)
        {
            std::array<Eigen::Matrix3d, 2> rotations;
            diagnostic.m_elements[element]->GetCommittedGaussRotations(rotations);
            for (int point = 0; point < 2; ++point)
            {
                gauss << time << ',' << element << ',' << point;
                for (int row = 0; row < 3; ++row)
                    for (int column = 0; column < 3; ++column)
                        gauss << ',' << rotations[point](row, column);
                gauss << '\n';
            }
        }

        QString checkedStates;
        QTextStream checkedStream(&checkedStates);
        checkedStream.setRealNumberPrecision(17);
        WriteDynamicStates(time, checkedStream);
        checkedStream.flush();
        SolverNameSpace::Vec checkedQ, checkedV, checkedA;
        GetState(checkedQ, checkedV, checkedA);
        const auto exactlyEqual = [](const Eigen::VectorXd& first, const Eigen::VectorXd& second)
        {
            return first.size() == second.size() &&
                   (first.size() == 0 || std::memcmp(first.data(), second.data(), first.size() * sizeof(double)) == 0);
        };
        bool unchanged = originalStates == checkedStates && exactlyEqual(originalQ, checkedQ) &&
                         exactlyEqual(originalV, checkedV) && exactlyEqual(originalA, checkedA) &&
                         exactlyEqual(originalInternal, m_internalForce) &&
                         exactlyEqual(originalInertia, m_inertiaForce);
        for (int index = 0; index < NodeCount(); ++index)
            unchanged = unchanged && std::memcmp(originalRotations[index].data(), m_nodes[index]->m_Rg.data(),
                                                 9 * sizeof(double)) == 0;
        QFile summaryFile(QDir(outputDirectory).filePath(phase + "-physical-summary.csv"));
        if (!summaryFile.open(QIODevice::WriteOnly | QIODevice::Text))
            return false;
        QTextStream summary(&summaryFile);
        summary.setRealNumberPrecision(17);
        summary << "metric,value\ntime," << time << "\nfree_dofs," << GetFreeDofs()
                << "\nsource_state_and_force_cache_unchanged_exact," << unchanged
                << "\ndiagnostic_isolation,complete_node_and_element_value_copy\nphysical_mass_definition,Get_me_Physical"
                << "\nmass_relative_asymmetry," << (denseMass - denseMass.transpose()).norm() / denseMass.norm()
                << "\nstiffness_relative_asymmetry,"
                << (denseStiffness - denseStiffness.transpose()).norm() / denseStiffness.norm()
                << "\nmass_ldlt_minimum_D," << factor.vectorD().minCoeff()
                << "\nkinetic_energy," << mechanicalEnergy[0] << "\nstrain_energy," << mechanicalEnergy[1]
                << "\ntotal_energy," << mechanicalEnergy[2]
                << "\nraw_translation_residual_max," << maximumRawTranslationResidual
                << "\nraw_rotation_residual_max," << maximumRawRotationResidual
                << "\nrecovered_translation_residual_max," << maximumRecoveredTranslationResidual
                << "\nrecovered_rotation_residual_max," << maximumRecoveredRotationResidual
                << "\nraw_component_scaled_residual_max," << maximumRawScaledResidual
                << "\nrecovered_component_scaled_residual_max," << maximumRecoveredScaledResidual
                << "\nscaled_residual_force_floor_N,1\nscaled_residual_moment_floor_Nm,1"
                << "\ninertia_split_error_max," << inertiaSplitError.cwiseAbs().maxCoeff()
                << "\nacceleration_recovery_difference_max,"
                << (rawAcceleration - recoveredAcceleration).cwiseAbs().maxCoeff()
                << "\nrecovery_mass_equation_residual_max," << recoveryLinearResidual.cwiseAbs().maxCoeff() << '\n';
        if (!unchanged)
            throw std::runtime_error("Duan physical diagnostics changed the original model state");
        return true;
    }

private:
    Eigen::VectorXd PhysicalInertiaForDiagnostics()
    {
        Eigen::VectorXd result = Eigen::VectorXd::Zero(GetFreeDofs());
        for (int index = 0; index < ElementCount(); ++index)
        {
            Eigen::VectorXd force;
            m_elements[index]->Get_InertiaForce(force);
            AssembleElementVector(index, force, result);
        }
        return result;
    }

    RigidBodyInertia GetEndDisk(int nodeIndex) const
    {
        RigidBodyInertia disk;
        disk.m_pNode = m_nodes[nodeIndex];
        disk.m_Mass = 10.0;
        disk.m_RotaryInertia = Eigen::Vector3d(200.0, 100.0, 100.0).asDiagonal();
        return disk;
    }

    void AssembleNodeMatrix(int nodeIndex, const Eigen::MatrixXd& nodeMatrix,
                            _OUT std::vector<Eigen::Triplet<double>>& triplets) const
    {
        for (int componentRow = 0; componentRow < 6; ++componentRow)
        {
            const int row = m_freeDofs[nodeIndex][componentRow];
            if (row < 0)
                continue;
            for (int componentColumn = 0; componentColumn < 6; ++componentColumn)
            {
                const int column = m_freeDofs[nodeIndex][componentColumn];
                if (column >= 0 && nodeMatrix(componentRow, componentColumn) != 0.0)
                    triplets.emplace_back(row, column, nodeMatrix(componentRow, componentColumn));
            }
        }
    }

    void AssembleElementMatrix(int elementIndex, const Eigen::MatrixXd& elementMatrix,
                               std::vector<Eigen::Triplet<double>>& triplets) const
    {
        for (int localRow = 0; localRow < 12; ++localRow)
        {
            const int rowNode = m_elementNodes[elementIndex][localRow / 6];
            const int row = m_freeDofs[rowNode][localRow % 6];
            if (row < 0)
                continue;
            for (int localColumn = 0; localColumn < 12; ++localColumn)
            {
                const int columnNode = m_elementNodes[elementIndex][localColumn / 6];
                const int column = m_freeDofs[columnNode][localColumn % 6];
                if (column >= 0 && elementMatrix(localRow, localColumn) != 0.0)
                    triplets.emplace_back(row, column, elementMatrix(localRow, localColumn));
            }
        }
    }

    void AssembleElementVector(int elementIndex, const Eigen::VectorXd& elementVector, Eigen::VectorXd& vector) const
    {
        for (int localRow = 0; localRow < 12; ++localRow)
        {
            const int nodeIndex = m_elementNodes[elementIndex][localRow / 6];
            const int row = m_freeDofs[nodeIndex][localRow % 6];
            if (row >= 0)
                vector[row] += elementVector[localRow];
        }
    }

    int m_freeDofCount = 0;
    Le2012Example m_example;
    int m_beamElementsPerLeg = 10;
    bool m_fusedAssembly = false;
    QString m_lastSolverMessage;
    std::shared_ptr<Material> m_material;
    std::shared_ptr<Le2012Section> m_section;
    std::shared_ptr<Property> m_property;
    std::vector<std::shared_ptr<Node>> m_nodes;
    std::vector<Node> m_checkpointNodes;
    std::vector<std::shared_ptr<ElementBeam_CR>> m_elements;
    std::vector<std::array<int, 2>> m_elementNodes;
    std::vector<std::array<int, 6>> m_freeDofs;
    Eigen::VectorXd m_internalForce;
    Eigen::VectorXd m_inertiaForce;
};
} // namespace

struct SpatialTssbnOptions
{
    bool reference = false;
    bool paperComparison = false;
    bool diagnostics = false;
    bool profile = false;
    bool finalStateOnly = false;
    bool fusedAssembly = false;
    bool energyEveryStep = false;
    double initialTimeStep = .001;
    double minimumTimeStep = 1.0e-7;
    double nonlinearTolerance = 1.0e-6;
    double spectralRadiusInfinity = 1.0;
    SolverNameSpace::TssbnErrorNormMode errorNormMode = SolverNameSpace::TssbnErrorNormMode::GlobalMax;
};

static const char* TssbnErrorNormName(SolverNameSpace::TssbnErrorNormMode mode)
{
    switch (mode)
    {
    case SolverNameSpace::TssbnErrorNormMode::ComponentwiseMax:
        return "ComponentwiseMax";
    case SolverNameSpace::TssbnErrorNormMode::LegacyGlobal:
        return "LegacyGlobal";
    case SolverNameSpace::TssbnErrorNormMode::DisplacementMaxVelocityRms:
        return "DisplacementMaxVelocityRms";
    case SolverNameSpace::TssbnErrorNormMode::PaperMixedRms:
        return "PaperMixedRms";
    case SolverNameSpace::TssbnErrorNormMode::DisplacementGlobalVelocityMax:
        return "DisplacementGlobalVelocityMax";
    case SolverNameSpace::TssbnErrorNormMode::GlobalMax:
        return "GlobalMax";
    }
    return "Unknown";
}

static int RunSpatialBeamDynamics(bool ring, bool useAdaptiveTssbn, const QString& outputDirectory, double duration,
                                  double maximumTimeStep, double relativeTolerance, int beamElementsPerLeg,
                                  bool useFixedNewmark, double newmarkAbsoluteTolerance,
                                  double newmarkVelocityAbsoluteTolerance, bool useRungeKutta,
                                  const SpatialTssbnOptions* tssbnOptions = nullptr, int fullStateStride = 1)
{
    Le2012BeamModel model(ring ? Le2012Example::Duan2026Ring : Le2012Example::RightAngleCantilever, beamElementsPerLeg,
                          tssbnOptions && tssbnOptions->fusedAssembly);
    std::unique_ptr<SolverNameSpace::ISolver> solver;
    if (useAdaptiveTssbn && tssbnOptions)
    {
        AnalysisStep step;
        step.m_Type = EnumKeyword::StepType::DYNAMIC;
        step.m_DynamicSolverType = tssbnOptions->reference ? SolverNameSpace::SolverType::ReferenceTSSBN
                                                           : SolverNameSpace::SolverType::AdaptiveTSSBN;
        step.m_StepSize = tssbnOptions->initialTimeStep;
        step.m_Tolerance = tssbnOptions->nonlinearTolerance;
        step.m_MaxIterations = 48;
        step.m_EnableGalloping = false;
        step.m_DiagnosticTssbnErrorTrace = tssbnOptions->profile;
        step.m_AdaptiveTssbn.spectralRadiusInfinity = tssbnOptions->spectralRadiusInfinity;
        step.m_AdaptiveTssbn.errorNormMode = tssbnOptions->errorNormMode;
        step.m_AdaptiveTssbn.minimumTimeStep = tssbnOptions->minimumTimeStep;
        step.m_AdaptiveTssbn.maximumTimeStep = maximumTimeStep;
        step.m_AdaptiveTssbn.relativeTolerance = relativeTolerance;
        step.m_AdaptiveTssbn.absoluteTolerance = newmarkAbsoluteTolerance;
        step.m_AdaptiveTssbn.velocityAbsoluteTolerance = newmarkVelocityAbsoluteTolerance;
        // 只把显式测试参数传入正式工厂；不改生产默认值和 Reference 策略。
        solver = SolverNameSpace::SolverFactory::Create(step);
        if (!solver || solver->GetType() != step.m_DynamicSolverType)
            throw std::runtime_error("Spatial TSSBN production factory solver identity differs");
    }
    else if (useRungeKutta)
    {
        SolverNameSpace::SolverRungeKutta4::Params parameters;
        parameters.timeStep = tssbnOptions ? tssbnOptions->initialTimeStep : std::min(0.01, maximumTimeStep);
        parameters.timeStepMode =
            useFixedNewmark ? SolverNameSpace::TimeStepMode::Fixed : SolverNameSpace::TimeStepMode::Adaptive;
        parameters.adaptiveTimeStep.minimumTimeStep = tssbnOptions ? tssbnOptions->minimumTimeStep : 1.0e-6;
        parameters.adaptiveTimeStep.maximumTimeStep = maximumTimeStep;
        parameters.adaptiveTimeStep.relativeTolerance = relativeTolerance;
        parameters.adaptiveTimeStep.velocityRelativeTolerance = relativeTolerance;
        parameters.adaptiveTimeStep.absoluteTolerance = newmarkAbsoluteTolerance;
        parameters.adaptiveTimeStep.velocityAbsoluteTolerance = newmarkVelocityAbsoluteTolerance;
        solver = std::make_unique<SolverNameSpace::SolverRungeKutta4>(parameters);
        if (solver->GetType() != (useFixedNewmark ? SolverNameSpace::SolverType::RungeKutta4
                                                  : SolverNameSpace::SolverType::AdaptiveRungeKutta4))
            throw std::runtime_error("Spatial RK4 production solver identity differs");
    }
    else if (useAdaptiveTssbn)
    {
        SolverNameSpace::SolverAdaptiveTSSBN::Params parameters;
        parameters.spectralRadiusInfinity = 1.0;
        parameters.initialTimeStep = std::min(0.01, maximumTimeStep);
        parameters.minimumTimeStep = 1.0e-6;
        parameters.maximumTimeStep = maximumTimeStep;
        parameters.relativeTolerance = relativeTolerance;
        parameters.absoluteTolerance = 1.0e-6;
        parameters.velocityAbsoluteTolerance = 1.0e-4;
        parameters.errorNormMode = SolverNameSpace::TssbnErrorNormMode::PaperMixedRms;
        parameters.useDuan2026Controller = true;
        parameters.derivativeGain = 0.15;
        parameters.maximumDerivativeFactor = 2.0;
        parameters.nonlinearTolerance = 1.0e-6;
        parameters.maximumNewtonIterations = 48;
        solver = std::make_unique<SolverNameSpace::SolverAdaptiveTSSBN>(parameters);
    }
    else
    {
        SolverNameSpace::SolverNewmark::Params parameters;
        parameters.dt = tssbnOptions ? tssbnOptions->initialTimeStep : std::min(0.01, maximumTimeStep);
        parameters.beta = 0.25;
        parameters.gamma = 0.5;
        parameters.maxIter = 48;
        parameters.tol = tssbnOptions ? tssbnOptions->nonlinearTolerance : 1.0e-6;
        parameters.timeStepMode =
            useFixedNewmark ? SolverNameSpace::TimeStepMode::Fixed : SolverNameSpace::TimeStepMode::Adaptive;
        if (useFixedNewmark)
            parameters.minimumTimeStep = parameters.dt;
        parameters.adaptiveTimeStep.minimumTimeStep = tssbnOptions ? tssbnOptions->minimumTimeStep : 1.0e-6;
        parameters.adaptiveTimeStep.maximumTimeStep = maximumTimeStep;
        parameters.adaptiveTimeStep.relativeTolerance = relativeTolerance;
        parameters.adaptiveTimeStep.velocityRelativeTolerance = relativeTolerance;
        parameters.adaptiveTimeStep.absoluteTolerance = newmarkAbsoluteTolerance;
        parameters.adaptiveTimeStep.velocityAbsoluteTolerance = newmarkVelocityAbsoluteTolerance;
        solver = std::make_unique<SolverNameSpace::SolverNewmark>(parameters);
    }

    if (!QDir().mkpath(outputDirectory))
        return 2;
    if (tssbnOptions && tssbnOptions->diagnostics &&
        !model.WritePhysicalDiagnostics(0.0, outputDirectory, QStringLiteral("initial")))
        return 6;
    QFile historyFile(QDir(outputDirectory).filePath(QStringLiteral("response.csv")));
    if (!historyFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return 3;
    QFile snapshotFile(QDir(outputDirectory).filePath(QStringLiteral("snapshots.csv")));
    if (!snapshotFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return 3;
    QFile statesFile(QDir(outputDirectory).filePath(QStringLiteral("full-state.csv")));
    if (!statesFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return 3;
    QFile energyFile(QDir(outputDirectory).filePath(QStringLiteral("energy.csv")));
    if (ring && !energyFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return 3;
    const bool energyEveryStep = tssbnOptions && tssbnOptions->energyEveryStep;
    QFile fullEnergyFile(QDir(outputDirectory).filePath(QStringLiteral("energy-full.csv")));
    if (energyEveryStep && !fullEnergyFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return 3;
    QTextStream fullEnergy(&fullEnergyFile);
    fullEnergy.setRealNumberPrecision(17);
    std::unique_ptr<Le2012BeamModel> energyObserver;
    int fullEnergyRecords = 0;
    if (energyEveryStep)
    {
        energyObserver = std::make_unique<Le2012BeamModel>(
            ring ? Le2012Example::Duan2026Ring : Le2012Example::RightAngleCantilever, beamElementsPerLeg,
            tssbnOptions->fusedAssembly);
        const auto initialEnergy = model.ObservedMechanicalEnergy(*energyObserver);
        fullEnergy << "time,kinetic,strain,total\n0," << initialEnergy[0] << ',' << initialEnergy[1] << ','
                   << initialEnergy[2] << '\n';
        ++fullEnergyRecords;
    }
    QTextStream history(&historyFile);
    QTextStream snapshots(&snapshotFile);
    QTextStream energy(&energyFile);
    QTextStream states(&statesFile);
    states.setRealNumberPrecision(17);
    states << "time,node,px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22,"
              "vx,vy,vz,wx,wy,wz,ax,ay,az,alphax,alphay,alphaz\n";
    model.WriteDynamicStates(0.0, states);
    history << "time,point_a_ux,point_a_uy,point_a_uz,point_b_ux,point_b_uy,point_b_uz\n";
    snapshots << "time,node,x,y,z\n";
    if (ring)
        energy << "time,kinetic,strain,total\n0,0,0,0\n";
    const int pointA = ring ? 0 : model.NodeCount() - 1;
    const int pointB = ring ? 8 : model.ElbowNodeIndex();
    const std::array<double, 3> snapshotTimes = {5.0, 50.0, 100.0};
    int nextSnapshot = 0;
    int acceptedSteps = 0;
    double lastTime = 0.0;
    double minimumAcceptedStep = std::numeric_limits<double>::infinity();
    double maximumAcceptedStep = 0.0;
    double minimumAz = std::numeric_limits<double>::infinity();
    double maximumAz = -std::numeric_limits<double>::infinity();
    double nextEnergyTime = 0.5;
    double minimumFreeEnergy = std::numeric_limits<double>::infinity();
    double maximumFreeEnergy = -std::numeric_limits<double>::infinity();
    QString pendingAcceptedState;
    const bool profile = tssbnOptions && tssbnOptions->profile;
    const bool finalStateOnly = tssbnOptions && tssbnOptions->finalStateOnly;
    double callbackSeconds = 0.0;
    solver->SetStepCallback(
        [&](int step, double time, const Eigen::VectorXd&)
        {
            const auto callbackStart = profile ? std::chrono::steady_clock::now()
                                               : std::chrono::steady_clock::time_point{};
            acceptedSteps = step;
            minimumAcceptedStep = std::min(minimumAcceptedStep, time - lastTime);
            maximumAcceptedStep = std::max(maximumAcceptedStep, time - lastTime);
            lastTime = time;
            if (!finalStateOnly && fullStateStride == 1)
            {
                model.WriteDynamicStates(time, states);
            }
            else if (!finalStateOnly)
            {
                // 缓存最后一个已接受帧，失败退出时也不导出尚未接受的试算状态。
                pendingAcceptedState.clear();
                QTextStream pending(&pendingAcceptedState);
                pending.setRealNumberPrecision(17);
                model.WriteDynamicStates(time, pending);
                pending.flush();
                if (step % fullStateStride == 0)
                {
                    states << pendingAcceptedState;
                    pendingAcceptedState.clear();
                }
            }
            const Eigen::Vector3d a = model.NodeDisplacement(pointA);
            const Eigen::Vector3d b = model.NodeDisplacement(pointB);
            history << QString::number(time, 'g', 16) << ',' << QString::number(a.x(), 'g', 16) << ','
                    << QString::number(a.y(), 'g', 16) << ',' << QString::number(a.z(), 'g', 16) << ','
                    << QString::number(b.x(), 'g', 16) << ',' << QString::number(b.y(), 'g', 16) << ','
                    << QString::number(b.z(), 'g', 16) << '\n';
            minimumAz = std::min(minimumAz, a.z());
            maximumAz = std::max(maximumAz, a.z());
            const bool legacyEnergySample = ring && time + 1.0e-12 >= nextEnergyTime;
            if (legacyEnergySample || energyEveryStep)
            {
                // 旧抽样仍在原模型上执行一次；新增逐步抽样只更新独立副本。
                const auto values = legacyEnergySample ? model.MechanicalEnergy()
                                                       : model.ObservedMechanicalEnergy(*energyObserver);
                if (legacyEnergySample)
                {
                    energy << QString::number(time, 'g', 16) << ',' << QString::number(values[0], 'g', 16) << ','
                           << QString::number(values[1], 'g', 16) << ',' << QString::number(values[2], 'g', 16) << '\n';
                    if (time >= 5.0)
                    {
                        minimumFreeEnergy = std::min(minimumFreeEnergy, values[2]);
                        maximumFreeEnergy = std::max(maximumFreeEnergy, values[2]);
                    }
                    nextEnergyTime += 0.5;
                }
                if (energyEveryStep)
                {
                    fullEnergy << time << ',' << values[0] << ',' << values[1] << ',' << values[2] << '\n';
                    ++fullEnergyRecords;
                }
            }
            if (ring && nextSnapshot < static_cast<int>(snapshotTimes.size()) && time >= snapshotTimes[nextSnapshot])
            {
                for (int nodeIndex = 0; nodeIndex < model.NodeCount(); ++nodeIndex)
                {
                    const Eigen::Vector3d position = model.NodePosition(nodeIndex);
                    snapshots << QString::number(time, 'g', 16) << ',' << nodeIndex << ','
                              << QString::number(position.x(), 'g', 16) << ',' << QString::number(position.y(), 'g', 16)
                              << ',' << QString::number(position.z(), 'g', 16) << '\n';
                }
                ++nextSnapshot;
            }
            if (profile)
                callbackSeconds +=
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - callbackStart).count();
        });

    const auto start = std::chrono::steady_clock::now();
    const bool solved = solver->Solve(model, duration);
    const double elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (!pendingAcceptedState.isEmpty())
        states << pendingAcceptedState;
    if (finalStateOnly && solved)
        model.WriteDynamicStates(lastTime, states);
    states.flush();
    statesFile.close();
    history.flush();
    snapshots.flush();
    if (ring)
        energy.flush();
    historyFile.close();
    snapshotFile.close();
    energyFile.close();
    if (energyEveryStep)
    {
        fullEnergy.flush();
        if (fullEnergy.status() != QTextStream::Ok || fullEnergyFile.error() != QFileDevice::NoError)
            return 3;
        fullEnergyFile.close();
    }
    QFile summaryFile(QDir(outputDirectory).filePath(QStringLiteral("summary.csv")));
    if (summaryFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream summary(&summaryFile);
        if (tssbnOptions)
            summary.setRealNumberPrecision(17);
        summary << "metric,value\ncase," << (ring ? "ring" : "right_angle_beam") << "\nsolver,"
                << (useRungeKutta ? (useFixedNewmark ? "fixed_rk4" : "adaptive_rk4")
                    : useAdaptiveTssbn
                        ? (tssbnOptions && tssbnOptions->reference ? "reference_tssbn" : "adaptive_tssbn")
                    : useFixedNewmark ? "fixed_newmark"
                                      : "adaptive_newmark")
                << "\nstatus," << (solved ? "complete" : "failed") << "\nduration," << duration << "\nlast_time,"
                << lastTime << "\nmaximum_time_step," << maximumTimeStep << "\nrelative_tolerance," << relativeTolerance
                << "\nnewmark_absolute_tolerance," << newmarkAbsoluteTolerance
                << "\nnewmark_velocity_absolute_tolerance," << newmarkVelocityAbsoluteTolerance
                << "\nbeam_elements_per_leg," << beamElementsPerLeg << "\naccepted_steps," << acceptedSteps
                << "\nwall_seconds," << elapsedSeconds << "\nminimum_point_a_uz," << minimumAz
                << "\nmaximum_point_a_uz," << maximumAz << '\n';
        if (profile)
            summary << "profile_enabled,1\ncallback_wall_seconds," << callbackSeconds
                    << "\nsolve_excluding_callback_seconds," << elapsedSeconds - callbackSeconds << '\n';
        if (!useAdaptiveTssbn && tssbnOptions)
            summary << "initial_time_step," << tssbnOptions->initialTimeStep << "\nminimum_time_step,"
                    << tssbnOptions->minimumTimeStep << "\nnonlinear_tolerance," << tssbnOptions->nonlinearTolerance
                    << "\nfull_state_stride," << fullStateStride << "\npaper_fused_assembly,"
                    << tssbnOptions->fusedAssembly << '\n';
        QString escapedSolverMessage = model.LastSolverMessage();
        escapedSolverMessage.replace(QLatin1Char('"'), QStringLiteral("\"\""));
        summary << "last_solver_message,\"" << escapedSolverMessage << "\"\n";
        if (ring)
            summary << "minimum_free_flight_energy," << minimumFreeEnergy << "\nmaximum_free_flight_energy,"
                    << maximumFreeEnergy << '\n';
        if (energyEveryStep)
            summary << "energy_every_step,1\nenergy_full_records," << fullEnergyRecords
                    << "\nenergy_full_sampling,initial_and_every_accepted_macro_step"
                    << "\nenergy_full_observation,isolated_state_copy_except_legacy_samples\n";
        if (useRungeKutta)
        {
            const auto* rk = static_cast<SolverNameSpace::SolverRungeKutta4*>(solver.get());
            const auto& statistics = rk->GetTimeStepStatistics();
            std::vector<Eigen::Matrix3d> auxiliary;
            model.GetAuxiliaryFiniteRotationStates(auxiliary);
            summary << "rejected_steps," << statistics.rejectedAttempts << "\nminimum_accepted_step,"
                    << statistics.minimumAcceptedTimeStep << "\nmaximum_accepted_step,"
                    << statistics.maximumAcceptedTimeStep << "\nauxiliary_rotation_count," << auxiliary.size()
                    << "\nadaptive_augmented_method_order," << (auxiliary.empty() ? 4 : 2) << "\nmass_factorizations,"
                    << rk->GetMassFactorizationCount() << '\n';
        }
        else if (useAdaptiveTssbn)
        {
            const auto* tssbn = static_cast<SolverNameSpace::SolverAdaptiveTSSBN*>(solver.get());
            summary << "rejected_steps," << tssbn->GetRejectedStepCount() << "\nmean_newton_iterations,"
                    << tssbn->GetAverageStageNewtonIterations() << '\n';
            if (tssbnOptions)
            {
                const auto& parameters = tssbn->GetParams();
                std::vector<int> permutation;
                summary << "solver_factory_created,1\nsolver_type_id," << static_cast<int>(solver->GetType())
                        << "\nsolver_name," << solver->GetName()
                        << "\ncomparison_scope,"
                        << (tssbnOptions->paperComparison
                                ? "Duan2026_paper_model_current_factory_solver_strategies"
                                : "pure_structural_integrator_same_default_CR_model")
                        << "\nreference_workflow_applied,0\naerodynamic_load_enabled,0\nreference_CR_physical_mode,0"
                        << "\nreference_linear_solve_permutation_available,"
                        << model.GetReferenceLinearSolvePermutation(permutation) << "\nspectral_radius_infinity,"
                        << parameters.spectralRadiusInfinity << "\ninitial_time_step," << parameters.initialTimeStep
                        << "\nminimum_time_step," << parameters.minimumTimeStep << "\nnonlinear_tolerance,"
                        << parameters.nonlinearTolerance << "\nmaximum_newton_iterations,"
                        << parameters.maximumNewtonIterations << "\neffective_relative_tolerance,"
                        << parameters.relativeTolerance << "\neffective_absolute_tolerance,"
                        << parameters.absoluteTolerance << "\neffective_velocity_absolute_tolerance,"
                        << parameters.velocityAbsoluteTolerance << "\nerror_norm_mode,"
                        << TssbnErrorNormName(parameters.errorNormMode)
                        << "\nuse_duan_controller," << parameters.useDuan2026Controller << "\ncontroller,"
                        << (parameters.useDuan2026Controller ? "Duan2026" : "Mahnken")
                        << "\nreference_galloping_integrator_strategy," << parameters.referenceGalloping
                        << "\nreference_independent_error_control," << parameters.referenceIndependentErrorControl
                        << "\nsafety_factor," << parameters.safetyFactor << "\nshrink_factor,"
                        << parameters.shrinkFactor << "\nmaximum_growth_factor," << parameters.maximumGrowthFactor
                        << "\ntarget_newton_iterations," << parameters.targetNewtonIterations
                        << "\nmaximum_total_newton_iterations," << parameters.maximumTotalNewtonIterations
                        << "\nderivative_gain," << parameters.derivativeGain << "\nminimum_derivative_factor,"
                        << parameters.minimumDerivativeFactor << "\nmaximum_derivative_factor,"
                        << parameters.maximumDerivativeFactor << "\nmaximum_rejected_attempts,"
                        << parameters.maximumRejectedAttempts << "\nuse_reference_extrapolated_acceleration,"
                        << parameters.useReferenceExtrapolatedAcceleration << "\naerodynamic_tangent_mode,"
                        << static_cast<int>(parameters.aerodynamicTangentMode) << "\nsolver_accepted_steps,"
                        << tssbn->GetAcceptedStepCount() << "\nminimum_accepted_step,"
                        << (acceptedSteps > 0 ? minimumAcceptedStep : 0.0) << "\nmaximum_accepted_step,"
                        << maximumAcceptedStep << "\nmean_accepted_step," << tssbn->GetAverageTimeStep()
                        << "\nmaximum_stage_newton_iterations," << tssbn->GetMaximumStageNewtonIterations()
                        << "\nprecision_limited_solves," << tssbn->GetPrecisionLimitedStageCount()
                        << "\nprecision_counter_unit,implicit_stage\nprecision_candidate_solves,"
                        << tssbn->GetPrecisionCandidateSolveCount() << "\nlocal_residual_delay_count,"
                        << tssbn->GetLocalResidualDelayCount() << "\nnumerical_accuracy_assessed,0\n";
                if (tssbnOptions->paperComparison)
                    summary << "node_count," << model.NodeCount() << "\nelement_count," << model.ElementCount()
                            << "\nfree_dof_count," << model.GetFreeDofs() << "\nfixed_dof_count,"
                            << model.GetFixedDofs() << "\nfull_state_stride," << fullStateStride
                            << "\nresponse_stride,1\nfull_state_final_only," << finalStateOnly
                            << "\npaper_fused_assembly," << tssbnOptions->fusedAssembly
                            << "\nfull_state_final_accepted_frame_retained," << (!finalStateOnly || solved) << '\n';
            }
        }
        else
        {
            const auto* newmark = static_cast<SolverNameSpace::SolverNewmark*>(solver.get());
            summary << "rejected_steps," << newmark->GetTimeStepStatistics().rejectedAttempts << '\n';
            summary << "precision_limited_solves," << newmark->GetPrecisionLimitedSolveCount() << '\n';
        }
    }
    if (tssbnOptions && tssbnOptions->diagnostics &&
        !model.WritePhysicalDiagnostics(lastTime, outputDirectory, QStringLiteral("final")))
        return 6;
    QTextStream(stdout) << "Duan2026 " << (ring ? "ring" : "beam") << ' '
                        << (useRungeKutta ? (useFixedNewmark ? "rk4" : "adaptive-rk4")
                            : useAdaptiveTssbn
                                ? (tssbnOptions && tssbnOptions->reference ? "reference-tssbn" : "adaptive-tssbn")
                                : "adaptive-newmark")
                        << " status=" << (solved ? "complete" : "failed") << " last_time=" << lastTime
                        << " steps=" << acceptedSteps << " wall_s=" << elapsedSeconds << " output=" << outputDirectory
                        << " last_solver_message=" << model.LastSolverMessage() << Qt::endl;
    return solved ? 0 : 4;
}

int PaperBeamDynamicsVerification::RunDuan2026(bool ring, bool useAdaptiveTssbn, const QString& outputDirectory,
                                               double duration, double maximumTimeStep, double relativeTolerance,
                                               int beamElementsPerLeg, bool useFixedNewmark,
                                               double newmarkAbsoluteTolerance, double newmarkVelocityAbsoluteTolerance,
                                               bool useReferenceTssbn, int fullStateStride,
                                               const PaperTssbnRunOptions* runOptions)
{
    if (fullStateStride < 1 ||
        !QDir(outputDirectory).entryList(QDir::AllEntries | QDir::NoDotAndDotDot).isEmpty())
    {
        QTextStream(stderr) << "Duan2026 verification requires a fresh output directory and a positive state stride"
                            << Qt::endl;
        return 2;
    }
    SpatialTssbnOptions options;
    options.reference = useReferenceTssbn;
    options.paperComparison = true;
    options.initialTimeStep = std::min(0.01, maximumTimeStep);
    options.minimumTimeStep = 1.0e-6;
    if (runOptions)
    {
        options.initialTimeStep = runOptions->initialTimeStep;
        options.minimumTimeStep = runOptions->minimumTimeStep;
        options.nonlinearTolerance = runOptions->nonlinearTolerance;
        options.spectralRadiusInfinity = runOptions->spectralRadiusInfinity;
        options.errorNormMode = runOptions->errorNormMode;
        options.diagnostics = runOptions->diagnostics;
        options.profile = runOptions->profile;
        options.finalStateOnly = runOptions->finalStateOnly;
        options.fusedAssembly = runOptions->fusedAssembly;
        options.energyEveryStep = runOptions->energyEveryStep;
    }
    // 使用论文的物理算例（默认直角梁 20 单元、圆环 32 单元），并保留当前生产工厂的求解策略。
    const bool useTssbn = useAdaptiveTssbn || useReferenceTssbn;
    const bool validErrorNorm =
        options.errorNormMode == SolverNameSpace::TssbnErrorNormMode::ComponentwiseMax ||
        options.errorNormMode == SolverNameSpace::TssbnErrorNormMode::PaperMixedRms ||
        options.errorNormMode == SolverNameSpace::TssbnErrorNormMode::LegacyGlobal ||
        options.errorNormMode == SolverNameSpace::TssbnErrorNormMode::DisplacementGlobalVelocityMax ||
        options.errorNormMode == SolverNameSpace::TssbnErrorNormMode::GlobalMax;
    const bool overrideAlgorithmControls =
        (runOptions && runOptions->overrideAlgorithmControls) || options.spectralRadiusInfinity != 1.0 ||
        options.errorNormMode != SolverNameSpace::TssbnErrorNormMode::GlobalMax;
    if (!std::isfinite(options.spectralRadiusInfinity) || options.spectralRadiusInfinity < 0.0 ||
        options.spectralRadiusInfinity > 1.0 || !validErrorNorm ||
        (overrideAlgorithmControls && (!useAdaptiveTssbn || useReferenceTssbn)))
    {
        QTextStream(stderr) << "Duan2026 rho/norm overrides require ordinary TSSBN, finite rho in [0,1], "
                               "and norm max, paper-rms, global, global-displacement or global-max" << Qt::endl;
        return 2;
    }
    if (options.minimumTimeStep > options.initialTimeStep || options.initialTimeStep > maximumTimeStep)
    {
        QTextStream(stderr) << "Duan2026 TSSBN steps must satisfy minimum <= initial <= maximum" << Qt::endl;
        return 2;
    }
    return RunSpatialBeamDynamics(ring, useTssbn, outputDirectory, duration, maximumTimeStep, relativeTolerance,
                                  beamElementsPerLeg, useFixedNewmark, newmarkAbsoluteTolerance,
                                  newmarkVelocityAbsoluteTolerance, runOptions && runOptions->useRungeKutta, &options,
                                  fullStateStride);
}

static double SpatialRungeKuttaOption(const QStringList& arguments, const QString& option, double fallback)
{
    const int index = arguments.indexOf(option);
    if (index < 0)
        return fallback;
    bool valid = false;
    const double value = index + 1 < arguments.size() ? arguments[index + 1].toDouble(&valid) : 0.0;
    if (!valid || !std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("Spatial RK4 option must be a positive finite number");
    return value;
}

std::optional<int> VerifyRungeKuttaSpatial(const QStringList& arguments)
{
    const bool adaptive = arguments.contains(QStringLiteral("--verify-adaptive-rk4-spatial"));
    const int option = arguments.indexOf(adaptive ? QStringLiteral("--verify-adaptive-rk4-spatial")
                                                  : QStringLiteral("--verify-rk4-spatial"));
    if (option < 0)
        return std::nullopt;
    try
    {
        if (option + 1 >= arguments.size() || arguments[option + 1].startsWith("--"))
            throw std::invalid_argument("Spatial RK4 verification requires an output directory");
        const double step = SpatialRungeKuttaOption(arguments, "--rk-spatial-step", .0025);
        const double relative = SpatialRungeKuttaOption(arguments, "--rk-spatial-rtol", 1.0e-5);
        const double absolute = SpatialRungeKuttaOption(arguments, "--rk-spatial-atol", 1.0e-11);
        const double velocityAbsolute = SpatialRungeKuttaOption(arguments, "--rk-spatial-velocity-atol", absolute);
        const QString directory = arguments[option + 1];
        if (QFile::exists(QDir(directory).filePath("summary.csv")))
            throw std::invalid_argument("Spatial RK4 output already exists; use a fresh directory");
        return RunSpatialBeamDynamics(false, false, directory, 2.5, step, relative, 1, !adaptive, absolute,
                                      velocityAbsolute, true);
    }
    catch (const std::exception& error)
    {
        QTextStream(stderr) << "Spatial RK4 verification: " << error.what() << Qt::endl;
        return 2;
    }
}

std::optional<int> VerifyTssbnSpatial(const QStringList& arguments)
{
    const bool reference = arguments.contains(QStringLiteral("--verify-reference-tssbn-spatial"));
    const int option = arguments.indexOf(reference ? QStringLiteral("--verify-reference-tssbn-spatial")
                                                   : QStringLiteral("--verify-tssbn-spatial"));
    if (option < 0)
        return std::nullopt;
    try
    {
        if (arguments.contains(QStringLiteral("--verify-tssbn-spatial")) && reference)
            throw std::invalid_argument("Choose exactly one spatial TSSBN solver strategy");
        if (option + 1 >= arguments.size() || arguments[option + 1].startsWith("--"))
            throw std::invalid_argument("Spatial TSSBN verification requires an output directory");
        const double maximumStep = SpatialRungeKuttaOption(arguments, "--tssbn-spatial-step", .01);
        const double relative = SpatialRungeKuttaOption(arguments, "--tssbn-spatial-rtol", 1.0e-5);
        const double absolute = SpatialRungeKuttaOption(arguments, "--tssbn-spatial-atol", 1.0e-11);
        const double velocityAbsolute = SpatialRungeKuttaOption(arguments, "--tssbn-spatial-velocity-atol", absolute);
        SpatialTssbnOptions options;
        options.reference = reference;
        options.initialTimeStep = SpatialRungeKuttaOption(arguments, "--tssbn-spatial-initial-step", .001);
        options.minimumTimeStep = SpatialRungeKuttaOption(arguments, "--tssbn-spatial-min-step", 1.0e-7);
        options.nonlinearTolerance = SpatialRungeKuttaOption(arguments, "--tssbn-spatial-newton-tol", 1.0e-6);
        if (options.minimumTimeStep > options.initialTimeStep || options.initialTimeStep > maximumStep)
            throw std::invalid_argument("Spatial TSSBN steps must satisfy minimum <= initial <= maximum");
        const QString directory = arguments[option + 1];
        if (QFile::exists(QDir(directory).filePath("summary.csv")) ||
            QFile::exists(QDir(directory).filePath("full-state.csv")))
            throw std::invalid_argument("Spatial TSSBN output already exists; use a fresh directory");
        return RunSpatialBeamDynamics(false, true, directory, 2.5, maximumStep, relative, 1, false, absolute,
                                      velocityAbsolute, false, &options);
    }
    catch (const std::exception& error)
    {
        QTextStream(stderr) << "Spatial TSSBN verification: " << error.what() << Qt::endl;
        return 2;
    }
}

int PaperBeamDynamicsVerification::RunExample1(const QString& outputDirectory)
{
    return RunExample1(outputDirectory, false);
}

int PaperBeamDynamicsVerification::RunExample1Adaptive(const QString& outputDirectory)
{
    return RunExample1(outputDirectory, true);
}

int PaperBeamDynamicsVerification::RunExample1(const QString& outputDirectory, bool useAdaptiveTssbn)
{
    Le2012BeamModel model(Le2012Example::RightAngleCantilever);
    const double initialTimeStep = PositiveEnvironmentValue("YQY_LE2012_EXAMPLE1_DT", 0.25);
    const double duration = PositiveEnvironmentValue("YQY_LE2012_DURATION", 30.0);
    std::unique_ptr<SolverNameSpace::ISolver> solver;
    if (useAdaptiveTssbn)
    {
        SolverNameSpace::SolverAdaptiveTSSBN::Params parameters;
        parameters.initialTimeStep = std::min(0.10, initialTimeStep);
        parameters.minimumTimeStep = 1.0e-4;
        parameters.maximumTimeStep = initialTimeStep;
        parameters.relativeTolerance = PositiveEnvironmentValue("YQY_TSSBN_REL_TOL", parameters.relativeTolerance);
        parameters.absoluteTolerance = PositiveEnvironmentValue("YQY_TSSBN_ABS_TOL", 1.0e-6);
        parameters.nonlinearTolerance = 1.0e-5;
        parameters.maximumNewtonIterations = 48;
        solver = std::make_unique<SolverNameSpace::SolverAdaptiveTSSBN>(parameters);
    }
    else
    {
        SolverNameSpace::SolverNewmark::Params parameters;
        parameters.dt = initialTimeStep;
        parameters.beta = 0.25;
        parameters.gamma = 0.5;
        parameters.maxIter = 48;
        parameters.tol = 1.0e-5;
        solver = std::make_unique<SolverNameSpace::SolverNewmark>(parameters);
    }

    QDir directory;
    if (!directory.mkpath(outputDirectory))
        return 2;
    QFile resultFile(QDir(outputDirectory).filePath(QStringLiteral("response.csv")));
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
        [&model, &stream, &minimumTipZ, &maximumTipZ, &minimumElbowZ, &maximumElbowZ,
         &lastCompletedTime](int, double time, const Eigen::VectorXd&)
        {
            const double tipZ = model.EndDisplacement().z();
            const double elbowZ = model.NodeDisplacement(10).z();
            stream << QString::number(time, 'f', 6) << ',' << QString::number(tipZ, 'g', 16) << ','
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
        QFile failureSummary(QDir(outputDirectory).filePath(QStringLiteral("summary.txt")));
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
        QTextStream(stdout) << "Le2012 Example1 nonconverged last_time=" << lastCompletedTime
                            << " result=" << resultFile.fileName() << Qt::endl;
        return 4;
    }
    resultFile.close();

    // 图 3 的数值从论文坐标轴读取。
    constexpr double referenceMinimumTipZ = -9.7;
    constexpr double referenceMaximumTipZ = 8.2;
    constexpr double referenceMinimumElbowZ = -4.0;
    constexpr double referenceMaximumElbowZ = 6.2;
    const auto relativeError = [](double calculated, double reference)
    {
        return std::abs(calculated - reference) / std::abs(reference);
    };
    const double minimumTipError = relativeError(minimumTipZ, referenceMinimumTipZ);
    const double maximumTipError = relativeError(maximumTipZ, referenceMaximumTipZ);
    const double minimumElbowError = relativeError(minimumElbowZ, referenceMinimumElbowZ);
    const double maximumElbowError = relativeError(maximumElbowZ, referenceMaximumElbowZ);

    QFile summaryFile(QDir(outputDirectory).filePath(QStringLiteral("summary.txt")));
    if (summaryFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream summary(&summaryFile);
        summary << "Le et al. (2012), Example 1 (Fig. 2)\n"
                << "quantity,calculated,figure_reference,relative_error\n"
                << "minimum_tip_u_z," << minimumTipZ << ',' << referenceMinimumTipZ << ',' << minimumTipError << '\n'
                << "maximum_tip_u_z," << maximumTipZ << ',' << referenceMaximumTipZ << ',' << maximumTipError << '\n'
                << "minimum_elbow_u_z," << minimumElbowZ << ',' << referenceMinimumElbowZ << ',' << minimumElbowError
                << '\n'
                << "maximum_elbow_u_z," << maximumElbowZ << ',' << referenceMaximumElbowZ << ',' << maximumElbowError
                << '\n';
    }

    QTextStream(stdout) << "Le2012 Example1 tip_uz=[" << minimumTipZ << ',' << maximumTipZ << "] elbow_uz=["
                        << minimumElbowZ << ',' << maximumElbowZ << "] errors=[" << minimumTipError << ','
                        << maximumTipError << ',' << minimumElbowError << ',' << maximumElbowError
                        << "] result=" << resultFile.fileName() << Qt::endl;

    const bool matchesPaper = minimumTipZ >= -10.5 && minimumTipZ <= -8.5 && maximumTipZ >= 7.0 && maximumTipZ <= 9.0 &&
                              minimumElbowZ >= -5.0 && minimumElbowZ <= -3.0 && maximumElbowZ >= 5.0 &&
                              maximumElbowZ <= 7.0;
    return matchesPaper ? 0 : 5;
}

int PaperBeamDynamicsVerification::Run(const QString& outputDirectory)
{
    return Run(outputDirectory, false);
}

int PaperBeamDynamicsVerification::RunAdaptive(const QString& outputDirectory)
{
    return Run(outputDirectory, true);
}

int PaperBeamDynamicsVerification::Run(const QString& outputDirectory, bool useAdaptiveTssbn)
{
    Le2012BeamModel model(Le2012Example::FreeFreeBeamWithDisks);
    QString diskCheckMessage;
    const bool diskCheckPassed = model.VerifyDiskInertia(diskCheckMessage);
    QTextStream(stdout) << "Le2012 disk mechanics " << (diskCheckPassed ? "PASS " : "FAIL ") << diskCheckMessage
                        << Qt::endl;
    if (!diskCheckPassed)
        return 6;
    if (qgetenv("YQY_LE2012_DISK_CHECK_ONLY") == "1")
        return 0;
    const double initialTimeStep = PositiveEnvironmentValue("YQY_LE2012_EXAMPLE4_DT", 0.10);
    const double duration = PositiveEnvironmentValue("YQY_LE2012_DURATION", 30.0);

    std::unique_ptr<SolverNameSpace::ISolver> solver;
    if (useAdaptiveTssbn)
    {
        SolverNameSpace::SolverAdaptiveTSSBN::Params parameters;
        parameters.initialTimeStep = initialTimeStep;
        parameters.minimumTimeStep = 1.0e-4;
        parameters.maximumTimeStep = initialTimeStep;
        parameters.relativeTolerance = PositiveEnvironmentValue("YQY_TSSBN_REL_TOL", parameters.relativeTolerance);
        parameters.absoluteTolerance = PositiveEnvironmentValue("YQY_TSSBN_ABS_TOL", 1.0e-6);
        parameters.nonlinearTolerance = 1.0e-5;
        parameters.maximumNewtonIterations = 32;
        solver = std::make_unique<SolverNameSpace::SolverAdaptiveTSSBN>(parameters);
    }
    else
    {
        SolverNameSpace::SolverNewmark::Params parameters;
        parameters.dt = initialTimeStep;
        parameters.beta = 0.25;
        parameters.gamma = 0.5;
        parameters.maxIter = 32;
        parameters.tol = 1.0e-5;
        solver = std::make_unique<SolverNameSpace::SolverNewmark>(parameters);
    }

    QDir directory;
    if (!directory.mkpath(outputDirectory))
        return 2;
    QFile resultFile(QDir(outputDirectory).filePath(QStringLiteral("response.csv")));
    if (!resultFile.open(QIODevice::WriteOnly | QIODevice::Text))
        return 3;
    QTextStream stream(&resultFile);
    stream << "time,u_x,u_y,u_z\n";

    double minimumX = std::numeric_limits<double>::infinity();
    double maximumAbsY = 0.0;
    double maximumAbsZ = 0.0;
    solver->SetStepCallback(
        [&model, &stream, &minimumX, &maximumAbsY, &maximumAbsZ](int, double time, const Eigen::VectorXd&)
        {
            const Eigen::Vector3d displacement = model.EndDisplacement();
            stream << QString::number(time, 'f', 6) << ',' << QString::number(displacement.x(), 'g', 16) << ','
                   << QString::number(displacement.y(), 'g', 16) << ',' << QString::number(displacement.z(), 'g', 16)
                   << '\n';
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
    const double errorX = std::abs(minimumX - referenceMinimumX) / std::abs(referenceMinimumX);
    const double errorY = std::abs(maximumAbsY - referenceMaximumAbsY) / referenceMaximumAbsY;
    const double errorZ = std::abs(maximumAbsZ - referenceMaximumAbsZ) / referenceMaximumAbsZ;

    QFile summaryFile(QDir(outputDirectory).filePath(QStringLiteral("summary.txt")));
    if (summaryFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream summary(&summaryFile);
        summary << "Le et al. (2012), Example 4\n"
                << "quantity,calculated,figure_reference,relative_error\n"
                << "minimum_u_x," << minimumX << ',' << referenceMinimumX << ',' << errorX << '\n'
                << "maximum_abs_u_y," << maximumAbsY << ',' << referenceMaximumAbsY << ',' << errorY << '\n'
                << "maximum_abs_u_z," << maximumAbsZ << ',' << referenceMaximumAbsZ << ',' << errorZ << '\n';
    }

    QTextStream(stdout) << "Le2012 Example4 min_ux=" << minimumX << " max_abs_uy=" << maximumAbsY
                        << " max_abs_uz=" << maximumAbsZ << " error_ux=" << errorX << " error_uy=" << errorY
                        << " error_uz=" << errorZ << " result=" << resultFile.fileName() << Qt::endl;

    // 图 9 的参考包络从论文坐标轴读取：ux_min ~= -20，|uy|max ~= 10，|uz|max ~= 3.5。
    const bool matchesPaper = minimumX >= -22.0 && minimumX <= -18.0 && maximumAbsY >= 9.0 && maximumAbsY <= 11.0 &&
                              maximumAbsZ >= 2.5 && maximumAbsZ <= 4.5;
    return matchesPaper ? 0 : 5;
}
