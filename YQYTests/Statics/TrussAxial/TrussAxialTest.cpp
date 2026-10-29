#include <gtest/gtest.h>

#include "Support/SolveModel.h"
#include "DataStructure/Structure/StructureData.h"
#include <array>
#include <cmath>

namespace
{
// 稳定支上的独立标量平衡，用二分法求伸长比，不复用生产 Newton 或单元切线。
static double ReferenceLogStretch(double load)
{
    double lower = 0.5;
    double upper = 2.0;
    for (int iteration = 0; iteration < 80; ++iteration)
    {
        const double stretch = 0.5 * (lower + upper);
        const double force = 1500.0 * std::log(stretch) / stretch;
        if (force < load)
            lower = stretch;
        else
            upper = stretch;
    }
    return 0.5 * (lower + upper);
}

static void CheckTrussFrames(const StructureData& model)
{
    const std::array<int, 2> elements{1, 2};
    const auto& frames = model.GetOutputter().GetDataSet();
    ASSERT_EQ(frames.size(), 5u);
    const std::array<double, 2> loads{200.0, -120.0};
    const std::array<double, 2> lengths{3.0, 1.5};
    for (std::size_t index = 0; index < frames.size(); ++index)
    {
        const auto& frame = frames[index];
        SCOPED_TRACE(::testing::Message() << "frame=" << index << " time=" << frame.GetTime());
        EXPECT_EQ(frame.GetStepId(), 1);
        EXPECT_NEAR(frame.GetTime(), 0.25 * index, 1.0e-12);
        EXPECT_EQ(frame.GetElementDatas().size(), elements.size());
        EXPECT_EQ(frame.GetNodeDatas().size(), 2 * elements.size());
        double expectedStrainEnergy = 0.0;
        double energyTolerance = 1.0e-10;
        for (int id : elements)
        {
            const int fixedNode = 2 * id - 1;
            const int freeNode = 2 * id;
            ASSERT_EQ(frame.GetElementDatas().count(id), 1u);
            ASSERT_EQ(frame.GetNodeDatas().count(fixedNode), 1u);
            ASSERT_EQ(frame.GetNodeDatas().count(freeNode), 1u);
            const double load = loads[id - 1] * frame.GetTime();
            const double stretch = ReferenceLogStretch(load);
            // 对 N=EA*ln(lambda)/lambda 沿长度积分：U=EA*L0*ln(lambda)^2/2。
            const double logarithmicStrain = std::log(stretch);
            expectedStrainEnergy += 750.0 * lengths[id - 1] * logarithmicStrain * logarithmicStrain;
            // 由既有 2e-9 位移误差门传播能量误差，不改变原力学结果判据。
            energyTolerance += std::abs(load) * 2.0e-9;
            EXPECT_NEAR(frame.GetNodeData(freeNode, EnumKeyword::NodeResultType::U1), lengths[id - 1] * (stretch - 1.0),
                        2.0e-9);
            EXPECT_NEAR(frame.GetNodeData(fixedNode, EnumKeyword::NodeResultType::U1), 0.0, 1.0e-12);
            EXPECT_NEAR(frame.GetNodeData(fixedNode, EnumKeyword::NodeResultType::R1), -load, 2.0e-7);
            EXPECT_NEAR(frame.GetNodeData(freeNode, EnumKeyword::NodeResultType::R1), 0.0, 2.0e-7);
            EXPECT_NEAR(frame.GetElementData(id, EnumKeyword::ElementResultType::AxialForce), load, 2.0e-7);
            EXPECT_NEAR(frame.GetElementData(id, EnumKeyword::ElementResultType::Strain), std::log(stretch), 2.0e-9);
            EXPECT_NEAR(frame.GetElementData(id, EnumKeyword::ElementResultType::CurrentStress),
                        2000.0 * std::log(stretch), 4.0e-6);
            EXPECT_NEAR(frame.GetNodeData(freeNode, EnumKeyword::NodeResultType::U2), 0.0, 1.0e-12);
            EXPECT_NEAR(frame.GetNodeData(freeNode, EnumKeyword::NodeResultType::U3), 0.0, 1.0e-12);
        }
        const EnergyResult& energy = frame.GetEnergy();
        ASSERT_EQ(energy.availableMask, EnergyResult::All);
        EXPECT_DOUBLE_EQ(energy.kinetic, 0.0);
        EXPECT_DOUBLE_EQ(energy.gravity, 0.0);
        EXPECT_NEAR(energy.strain, expectedStrainEnergy, energyTolerance);
        EXPECT_NEAR(energy.mechanical, expectedStrainEnergy, energyTolerance);
    }
}

/** @brief 真实文件桁架拉压模型从读取到各增量平衡结果的验证
 *
 * @details 输入：Statics/TrussAxial/Trusses.bdf，包含 L0=3 和 1.5 的独立杆，末端荷载分别为 200 和 -120。
 *          参考：对稳定区间的 N=1500*ln(lambda)/lambda 独立二分求根，得到位移、轴力、应变及应力。
 *          判据：核对 0 至 1 的五帧；时间误差不超过 1e-12、位移和应变误差不超过 2e-9，
 *          轴力和反力误差不超过 2e-7、应力误差不超过 4e-6，固定及横向位移不超过 1e-12。
 *          2026.10.29 补充输出应变能与解析积分 EA*L0*ln(lambda)^2/2 的比较，
 *          容差为 1e-10+sum(abs(N))*2e-9；无重力静力帧的动能/重力势能为零，总机械能等于应变能。
 *          范围：实际静力求解及各增量的独立平衡结果比较。
 * @author   yangqiuyou
 * @date     2026.10.14
 */
TEST(TrussAxial, NewtonTensionCompressionAtEveryIncrement)
{
    QString error;
    auto model = SolveModel::Read(QStringLiteral("Statics/TrussAxial/Trusses.bdf"), error);
    ASSERT_TRUE(model) << error.toStdString();
    ASSERT_TRUE(SolveModel::Run(model, error)) << error.toStdString();
    CheckTrussFrames(*model);
}
}
