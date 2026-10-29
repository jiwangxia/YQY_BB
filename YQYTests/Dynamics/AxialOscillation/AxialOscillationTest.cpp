#include <gtest/gtest.h>

#include "Support/SolveModel.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "Export/Hdf5ResultReader.h"
#include "Export/Hdf5ResultWriter.h"
#include <QTemporaryDir>
#include <QDir>
#include <QFile>
#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
static void CheckAxialBeamHistory(SolverNameSpace::SolverType solver, double timeStep, _OUT double& maximumError,
                                  _OUT double& maximumEnergyError, double gravityAcceleration = 0.0)
{
    maximumError = std::numeric_limits<double>::quiet_NaN();
    maximumEnergyError = std::numeric_limits<double>::quiet_NaN();
    QString error;
    const auto model = SolveModel::Read(QStringLiteral("Dynamics/AxialOscillation/AxialBeam.bdf"), error);
    ASSERT_TRUE(model) << error.toStdString();
    ASSERT_EQ(model->m_AnalysisStep.size(), 1u);
    ASSERT_EQ(model->m_Elements.size(), 1u);
    const auto step = model->m_AnalysisStep.at(1);
    ASSERT_TRUE(step);
    step->m_DynamicSolverType = solver;
    step->m_StepSize = timeStep;
    step->m_AdaptiveTssbn.spectralRadiusInfinity = 1.0;
    step->m_AdaptiveTssbn.minimumTimeStep = timeStep / 64.0;
    step->m_AdaptiveTssbn.maximumTimeStep = timeStep;
    step->m_AdaptiveTssbn.relativeTolerance = 0.01;
    step->m_AdaptiveTssbn.absoluteTolerance = 1.0e-7;
    step->m_AdaptiveTssbn.velocityAbsoluteTolerance = 1.0e-5;
    if (gravityAcceleration != 0.0)
    {
        auto gravity = std::make_shared<Force_Gravity>();
        gravity->m_Id = 2;
        gravity->m_StepId = 1;
        gravity->m_Direction = EnumKeyword::Direction::X;
        gravity->m_g = gravityAcceleration;
        model->m_Load.emplace(gravity->m_Id, gravity);
    }
    ASSERT_TRUE(SolveModel::Run(model, error)) << error.toStdString();
    const auto& frames = model->GetOutputter().GetDataSet();
    ASSERT_FALSE(frames.empty());
    EXPECT_NEAR(frames.back().GetTime(), 0.5, 1.0e-12);
    EXPECT_GE(frames.size(), static_cast<size_t>(std::floor(0.5 / timeStep)));

    const double length = 2.0;
    const double area = 0.04 * 0.02;
    const double gravityForce = 0.5 * 1000.0 * area * length * gravityAcceleration;
    const double force = 0.004 + gravityForce;
    const double stiffness = 1.0e6 * area / length;
    // 线性轴向形函数积分得到 M22=rho*A*L/3、M12=M22/2；这是约化有限元系统精确解。
    const double mass = 1000.0 * area * length / 3.0;
    // 普通梁现有重力等效力按当前弦长分配，轴向力增加 q*u/2；保持该求解行为。
    const double gravityStiffness = 0.5 * 1000.0 * area * gravityAcceleration;
    const double effectiveStiffness = stiffness - gravityStiffness;
    const double omega = std::sqrt(effectiveStiffness / mass);
    const double equilibrium = force / effectiveStiffness;
    const double velocityScale = equilibrium * omega;
    const double accelerationScale = force / mass;
    maximumError = 0.0;
    maximumEnergyError = 0.0;
    double previousTime = 0.0;
    for (const auto& frame : frames)
    {
        SCOPED_TRACE(frame.GetTime());
        ASSERT_EQ(frame.GetNodeDatas().size(), 2u);
        ASSERT_EQ(frame.GetNodeDatas().count(2), 1u);
        ASSERT_EQ(frame.GetElementDatas().count(1), 1u);
        const double time = frame.GetTime();
        EXPECT_GE(time, previousTime);
        EXPECT_LE(time - previousTime, timeStep * (1.0 + 1.0e-9));
        previousTime = time;
        const double expectedU = equilibrium * (1.0 - std::cos(omega * time));
        const double expectedV = velocityScale * std::sin(omega * time);
        const double expectedA = accelerationScale * std::cos(omega * time);
        const double u = frame.GetNodeData(2, EnumKeyword::NodeResultType::U1);
        const double v = frame.GetNodeData(2, EnumKeyword::NodeResultType::V1);
        const double a = frame.GetNodeData(2, EnumKeyword::NodeResultType::A1);
        const double axialForce = frame.GetElementData(1, EnumKeyword::ElementResultType::AxialForce);
        const double rootReaction = frame.GetNodeData(1, EnumKeyword::NodeResultType::R1);
        const double expectedReaction = 0.5 * mass * expectedA - stiffness * expectedU -
                                          gravityForce - gravityStiffness * expectedU;
        maximumError =
            std::max({maximumError, std::abs(u - expectedU) / equilibrium, std::abs(v - expectedV) / velocityScale,
                      std::abs(a - expectedA) / accelerationScale, std::abs(axialForce - stiffness * expectedU) / force,
                      std::abs(rootReaction - expectedReaction) / (1.5 * force)});
        EXPECT_NEAR(axialForce, stiffness * u, 1.0e-10);
        EXPECT_NEAR(rootReaction, 0.5 * mass * a - stiffness * u - gravityForce - gravityStiffness * u, 2.0e-9);
        const double energy = 0.5 * mass * v * v + 0.5 * effectiveStiffness * u * u - force * u;
        maximumEnergyError = std::max(maximumEnergyError, std::abs(energy) / (force * equilibrium));
        const auto& storedEnergy = frame.GetEnergy();
        ASSERT_EQ(storedEnergy.availableMask, EnergyResult::All);
        EXPECT_NEAR(storedEnergy.kinetic, 0.5 * mass * v * v, 1.0e-14);
        EXPECT_NEAR(storedEnergy.strain, 0.5 * stiffness * u * u, 1.0e-14);
        EXPECT_NEAR(storedEnergy.gravity, -gravityForce * u, 1.0e-14);
        EXPECT_NEAR(storedEnergy.mechanical,
                    0.5 * mass * v * v + 0.5 * stiffness * u * u - gravityForce * u, 1.0e-14);
        const auto& tip = frame.GetNodeDatas().at(2);
        for (int component = 1; component < 6; ++component)
        {
            EXPECT_NEAR(tip.GetVelocityComponent(component), 0.0, 1.0e-12);
            EXPECT_NEAR(tip.GetAccelerationComponent(component), 0.0, 1.0e-12);
        }
        EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::U2), 0.0, 1.0e-12);
        EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::U3), 0.0, 1.0e-12);
        EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::UR1), 0.0, 1.0e-12);
        EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::UR2), 0.0, 1.0e-12);
        EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::UR3), 0.0, 1.0e-12);
    }
    EXPECT_LE(maximumEnergyError, 0.005);
    if (gravityAcceleration == 0.0)
        return;

    // 在同一物理算例中检查两种实际输出路径，回读值仍与上面的独立质量/刚度公式比较。
    QTemporaryDir directory;
    ASSERT_TRUE(directory.isValid());
    for (const bool streaming : {false, true})
    {
        const QString path = directory.filePath(streaming ? QStringLiteral("stream.h5") : QStringLiteral("full.h5"));
        if (streaming)
        {
            Hdf5ResultWriter writer;
            ASSERT_TRUE(writer.begin(path, model.get(), QStringLiteral("AxialBeam")));
            ASSERT_TRUE(writer.writeFrames(1, frames));
            ASSERT_TRUE(writer.end());
        }
        else
            ASSERT_TRUE(model->GetOutputter().SaveHdf5File(path, model.get()));
        Hdf5ResultReader reader;
        Hdf5ResultSummary summary;
        ASSERT_TRUE(reader.inspect(path, summary));
        EXPECT_TRUE(summary.hasEnergy);
        EXPECT_EQ(summary.energyRecordCount, static_cast<qint64>(frames.size()));
        std::vector<Hdf5ResultFrameInfo> information;
        ASSERT_TRUE(reader.openEnergyHistory(path, information));
        std::vector<Hdf5EnergyHistorySample> samples;
        ASSERT_TRUE(reader.readEnergyHistory(1, samples));
        ASSERT_EQ(samples.size(), frames.size());
        for (std::size_t index = 0; index < samples.size(); ++index)
        {
            const auto& expected = frames[index].GetEnergy();
            EXPECT_EQ(samples[index].info.stepId, 1);
            EXPECT_DOUBLE_EQ(samples[index].info.time, frames[index].GetTime());
            EXPECT_EQ(samples[index].energy.availableMask, EnergyResult::All);
            EXPECT_DOUBLE_EQ(samples[index].energy.kinetic, expected.kinetic);
            EXPECT_DOUBLE_EQ(samples[index].energy.strain, expected.strain);
            EXPECT_DOUBLE_EQ(samples[index].energy.gravity, expected.gravity);
            EXPECT_DOUBLE_EQ(samples[index].energy.mechanical, expected.mechanical);
        }
        reader.close();
        ASSERT_TRUE(reader.open(path, information));
        Hdf5ResultFrame lastFrame;
        ASSERT_TRUE(reader.readFrame(static_cast<int>(frames.size() - 1), lastFrame));
        EXPECT_EQ(lastFrame.energy.availableMask, EnergyResult::All);
        EXPECT_DOUBLE_EQ(lastFrame.energy.mechanical, frames.back().GetEnergy().mechanical);
        reader.close();
        const QString evidenceDirectory = qEnvironmentVariable("YQY_ENERGY_TEST_OUTPUT");
        if (!evidenceDirectory.isEmpty())
        {
            ASSERT_TRUE(QDir().mkpath(evidenceDirectory));
            ASSERT_TRUE(QFile::copy(path, QDir(evidenceDirectory).filePath(streaming ? QStringLiteral("stream.h5")
                                                                                  : QStringLiteral("full.h5"))));
        }
    }
}

static void CheckAxialBeamConvergence(SolverNameSpace::SolverType solver, double coarseStep, double fineLimit,
                                      double reductionLimit)
{
    double coarseError = 0.0;
    double fineError = 0.0;
    double coarseEnergyError = 0.0;
    double fineEnergyError = 0.0;
    CheckAxialBeamHistory(solver, coarseStep, coarseError, coarseEnergyError);
    CheckAxialBeamHistory(solver, coarseStep / 2.0, fineError, fineEnergyError);
    ::testing::Test::RecordProperty("coarse_time_step", QString::number(coarseStep, 'g', 17).toStdString());
    ::testing::Test::RecordProperty("fine_time_step", QString::number(coarseStep / 2.0, 'g', 17).toStdString());
    ::testing::Test::RecordProperty("coarse_maximum_normalized_error",
                                    QString::number(coarseError, 'g', 17).toStdString());
    ::testing::Test::RecordProperty("fine_maximum_normalized_error", QString::number(fineError, 'g', 17).toStdString());
    ::testing::Test::RecordProperty("fine_to_coarse_error_ratio",
                                    QString::number(fineError / coarseError, 'g', 17).toStdString());
    ::testing::Test::RecordProperty("coarse_maximum_normalized_energy_residual",
                                    QString::number(coarseEnergyError, 'g', 17).toStdString());
    ::testing::Test::RecordProperty("fine_maximum_normalized_energy_residual",
                                    QString::number(fineEnergyError, 'g', 17).toStdString());
    ASSERT_TRUE(std::isfinite(coarseError));
    ASSERT_TRUE(std::isfinite(fineError));
    EXPECT_GT(coarseError, 1.0e-9);
    EXPECT_LE(fineError, fineLimit);
    EXPECT_LT(fineError, reductionLimit * coarseError);
}

/** @brief Newmark 对一致质量轴向振动的全时程精度及步长收敛验证
 *
 * @details 输入：Dynamics/AxialOscillation/AxialBeam.bdf，约化 m=1000*0.0008*2/3、k=400、恒力 P=0.004，终时刻 0.5。
 *          参考：u=P/k*(1-cos(omega*t)) 及其速度、加速度，omega=sqrt(k/m)；反力含一致质量交叉项。
 *          判据：步长 0.002 和 0.001；细步各物理量最大归一化误差不超过 0.0025 且小于粗步的 0.4 倍。
 *          每帧检查轴力 k*u（1e-10）、支座反力（2e-9）及归一化能量残差不超过 0.005，终时刻误差不超过 1e-12。
 *          范围：约化有限元单自由度系统的精确解，包含初始加速度、约束分量及 HDF5 回读，不代替连续杆波动解。
 *          增补沿轴线重力 g=0.01 的工况，四项输出与一致质量/弹性能公式比较至 1e-14，并覆盖两种 HDF5 写入。
 * @author   yangqiuyou
 * @date     2026.10.14
 */
TEST(AxialOscillation, NewmarkFixed)
{
    CheckAxialBeamConvergence(SolverNameSpace::SolverType::Newmark, 0.002, 0.0025, 0.4);
    double gravityError = 0.0;
    double gravityEnergyError = 0.0;
    CheckAxialBeamHistory(SolverNameSpace::SolverType::Newmark, 0.001, gravityError, gravityEnergyError, 0.01);
    EXPECT_LE(gravityError, 0.0025);
}

/** @brief 四阶 Runge-Kutta 对一致质量轴向振动的全时程精度及步长收敛验证
 *
 * @details 输入：Dynamics/AxialOscillation/AxialBeam.bdf，约化 m=1000*0.0008*2/3、k=400、恒力 P=0.004，终时刻 0.5。
 *          参考：u=P/k*(1-cos(omega*t)) 及其速度、加速度、轴力和含质量交叉项的反力，omega=sqrt(k/m)。
 *          判据：步长 0.004 和 0.002；细步最大归一化误差不超过 2e-5 且小于粗步的 0.15 倍。
 *          每帧检查轴力 k*u（1e-10）、支座反力（2e-9）及归一化能量残差不超过 0.005，终时刻误差不超过 1e-12。
 *          范围：同一约化有限元系统的独立闭式解，包含初始加速度、约束分量及 HDF5 回读。
 * @author   yangqiuyou
 * @date     2026.10.14
 */
TEST(AxialOscillation, Rk4Fixed)
{
    CheckAxialBeamConvergence(SolverNameSpace::SolverType::RungeKutta4, 0.004, 2.0e-5, 0.15);
}

/** @brief 自适应 TSSBN 在最大步长约束下的轴向振动精度与收敛验证
 *
 * @details 输入：Dynamics/AxialOscillation/AxialBeam.bdf；控制器启用，rho_inf=1，最大步长依次为 0.004 和 0.002。
 *          参考：约化 m=1000*0.0008*2/3、k=400、P=0.004 系统的余弦位移及速度、加速度、轴力和反力闭式解。
 *          判据：细步最大归一化误差不超过 0.0025 且小于粗步的 0.4 倍；能量残差不超过 0.005。
 *          接受步长不超过上限，终时刻 0.5 的误差不超过 1e-12；轴力和反力平衡容差分别为 1e-10 和 2e-9。
 *          范围：保持自适应控制器实际运行，不要求每个接受步长相同；同时检查初始状态、约束分量及 HDF5 回读。
 * @author   yangqiuyou
 * @date     2026.10.14
 */
TEST(AxialOscillation, TssbnAdaptive)
{
    // 控制器保持启用；收敛比较只约束最大步长，不假设所有接受步长完全相同。
    CheckAxialBeamConvergence(SolverNameSpace::SolverType::AdaptiveTSSBN, 0.004, 0.0025, 0.4);
}
}
