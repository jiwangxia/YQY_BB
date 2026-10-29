#include <gtest/gtest.h>

#include "Support/SolveModel.h"
#include "DataStructure/Structure/StructureData.h"
#include <cmath>
#include <map>
#include <memory>

namespace
{
static void CheckBeamRootReaction(const std::map<int, std::shared_ptr<StructureData>>& regions, int nodeId,
                                  int component, double expected, double tolerance)
{
    int matches = 0;
    for (const auto& [regionId, region] : regions)
    {
        ASSERT_TRUE(region);
        const auto node = region->FindNode(nodeId);
        if (!node)
            continue;
        SCOPED_TRACE(regionId);
        ++matches;
        ASSERT_EQ(node->m_ReactionForce.size(), 6u);
        EXPECT_NEAR(node->m_ReactionForce[component], expected, tolerance);
        ASSERT_EQ(node->m_Displacement.size(), 6u);
        for (double value : node->m_Displacement)
            EXPECT_NEAR(value, 0.0, 1.0e-14);
    }
    EXPECT_EQ(matches, 1);
}

/** @brief 矩形悬臂梁轴拉、双向弯曲及端部反力的解析解验证
 *
 * @details 输入：Statics/CantileverBeam/Beams.bdf，L=2、矩形截面 0.04x0.02、E=210e9；横向力 0.1、轴向力 100。
 *          参考：Euler-Bernoulli 位移 P*L^3/(3*E*I)、转角 P*L^2/(2*E*I)、轴拉 P*L/(E*A) 及端部平衡。
 *          判据：弯曲位移和转角允许 2e-5 相对误差加 2e-12；端剪力和端矩误差不超过 1e-7。
 *          轴拉位移及应变误差不超过 1e-13，轴力和反力误差不超过 1e-6，应力误差不超过 0.002。
 *          范围：t=1 终帧、小转角约 3.6e-5；检查节点内力矩和实际支座反力，不使用未定义截面位置的单元弯矩。
 * @author   yangqiuyou
 * @date     2026.10.14
 */
TEST(CantileverBeam, NewtonAxialAndBothBendingSolutions)
{
    QString error;
    const auto model = SolveModel::Read(QStringLiteral("Statics/CantileverBeam/Beams.bdf"), error);
    ASSERT_TRUE(model) << error.toStdString();
    ASSERT_EQ(model->m_Nodes.size(), 8u);
    ASSERT_EQ(model->m_Elements.size(), 4u);
    std::map<int, std::shared_ptr<StructureData>> regions;
    ASSERT_TRUE(SolveModel::Run(model, error, &regions)) << error.toStdString();
    const auto& frames = model->GetOutputter().GetDataSet();
    ASSERT_FALSE(frames.empty());
    const auto& frame = frames.back();
    EXPECT_NEAR(frame.GetTime(), 1.0, 1.0e-12);
    ASSERT_EQ(frame.GetNodeDatas().size(), 8u);
    ASSERT_EQ(frame.GetElementDatas().size(), 4u);

    const double young = 210.0e9;
    const double length = 2.0;
    const double width = 0.04;
    const double height = 0.02;
    const double area = width * height;
    const double iy = width * height * height * height / 12.0;
    const double iz = height * width * width * width / 12.0;
    const double force = 0.1;
    // 单元含几何非线性；该荷载下最大转角约 3.6e-5，线性解的高阶误差远小于比较容限。
    const double uy = force * length * length * length / (3.0 * young * iz);
    const double uz = force * length * length * length / (3.0 * young * iy);
    const double rz = force * length * length / (2.0 * young * iz);
    const double ry = -force * length * length / (2.0 * young * iy);
    EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::U2), uy, 2.0e-5 * uy + 2.0e-12);
    EXPECT_NEAR(frame.GetNodeData(4, EnumKeyword::NodeResultType::U3), uz, 2.0e-5 * uz + 2.0e-12);
    EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::UR3), rz, 2.0e-5 * rz + 2.0e-12);
    EXPECT_NEAR(frame.GetNodeData(4, EnumKeyword::NodeResultType::UR2), ry, 2.0e-5 * std::abs(ry) + 2.0e-12);
    EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::U3), 0.0, 2.0e-12);
    EXPECT_NEAR(frame.GetNodeData(4, EnumKeyword::NodeResultType::U2), 0.0, 2.0e-12);
    EXPECT_NEAR(frame.GetNodeData(1, EnumKeyword::NodeResultType::R2), -force, 1.0e-7);
    EXPECT_NEAR(frame.GetNodeData(3, EnumKeyword::NodeResultType::R3), -force, 1.0e-7);
    // M 为节点内力矩，不用尚未定义截面位置的单元 MomentY/Z 字段代替端弯矩。
    EXPECT_NEAR(frame.GetNodeData(1, EnumKeyword::NodeResultType::M3), -force * length, 1.0e-7);
    EXPECT_NEAR(frame.GetNodeData(3, EnumKeyword::NodeResultType::M2), force * length, 1.0e-7);
    EXPECT_NEAR(frame.GetNodeData(2, EnumKeyword::NodeResultType::M3), 0.0, 1.0e-7);
    EXPECT_NEAR(frame.GetNodeData(4, EnumKeyword::NodeResultType::M2), 0.0, 1.0e-7);
    CheckBeamRootReaction(regions, 1, 1, -force, 1.0e-7);
    CheckBeamRootReaction(regions, 1, 5, -force * length, 1.0e-7);
    CheckBeamRootReaction(regions, 3, 2, -force, 1.0e-7);
    CheckBeamRootReaction(regions, 3, 4, force * length, 1.0e-7);

    const double axialForce = 100.0;
    const double axialDisplacement = axialForce * length / (young * area);
    EXPECT_NEAR(frame.GetNodeData(8, EnumKeyword::NodeResultType::U1), axialDisplacement, 1.0e-13);
    EXPECT_NEAR(frame.GetNodeData(8, EnumKeyword::NodeResultType::U2), 0.0, 1.0e-13);
    EXPECT_NEAR(frame.GetNodeData(8, EnumKeyword::NodeResultType::U3), 0.0, 1.0e-13);
    EXPECT_NEAR(frame.GetElementData(4, EnumKeyword::ElementResultType::AxialForce), axialForce, 1.0e-6);
    EXPECT_NEAR(frame.GetElementData(4, EnumKeyword::ElementResultType::CurrentStress), axialForce / area, 0.002);
    EXPECT_NEAR(frame.GetElementData(4, EnumKeyword::ElementResultType::Strain), axialDisplacement / length, 1.0e-13);
    EXPECT_NEAR(frame.GetNodeData(7, EnumKeyword::NodeResultType::R1), -axialForce, 1.0e-6);
    CheckBeamRootReaction(regions, 7, 0, -axialForce, 1.0e-6);
}

/** @brief 矩形悬臂梁 Saint-Venant 扭转角及端部力矩验证
 *
 * @details 输入：Statics/CantileverBeam/Beams.bdf，L=2、截面 0.04x0.02、E=210e9、nu=0.3、端扭矩 T=0.01。
 *          参考：独立矩形奇数级数 J=7.317813667826265e-8，转角 theta=T*L/(G*J)，两端内力矩为 -T 和 T。
 *          判据：扭转角相对误差不超过 2e-5，端部内力矩和支座反扭矩误差不超过 1e-8，
 *          非扭转自由度误差不超过 1e-13，固定端位移不超过 1e-14。
 *          范围：矩形截面小转角纯扭转；使用 Saint-Venant 常数而非截面极惯量。
 * @author   yangqiuyou
 * @date     2026.10.14
 */
TEST(CantileverBeam, NewtonSaintVenantTorsionAndEndMoments)
{
    QString error;
    const auto model = SolveModel::Read(QStringLiteral("Statics/CantileverBeam/Beams.bdf"), error);
    ASSERT_TRUE(model) << error.toStdString();
    std::map<int, std::shared_ptr<StructureData>> regions;
    ASSERT_TRUE(SolveModel::Run(model, error, &regions)) << error.toStdString();
    const auto& frames = model->GetOutputter().GetDataSet();
    ASSERT_FALSE(frames.empty());
    const auto& frame = frames.back();
    ASSERT_EQ(frame.GetNodeDatas().count(5), 1u);
    ASSERT_EQ(frame.GetNodeDatas().count(6), 1u);

    const double torque = 0.01;
    const double shear = 210.0e9 / (2.0 * (1.0 + 0.3));
    // 独立高精度矩形 Saint-Venant 奇数级数参考，非极惯量 Iy+Iz。
    const double torsionConstant = 7.317813667826265e-8;
    const double twist = torque * 2.0 / (shear * torsionConstant);
    EXPECT_NEAR(frame.GetNodeData(6, EnumKeyword::NodeResultType::UR1), twist, 2.0e-5 * twist);
    EXPECT_NEAR(frame.GetNodeData(6, EnumKeyword::NodeResultType::UR2), 0.0, 1.0e-13);
    EXPECT_NEAR(frame.GetNodeData(6, EnumKeyword::NodeResultType::UR3), 0.0, 1.0e-13);
    EXPECT_NEAR(frame.GetNodeData(6, EnumKeyword::NodeResultType::U1), 0.0, 1.0e-13);
    EXPECT_NEAR(frame.GetNodeData(6, EnumKeyword::NodeResultType::U2), 0.0, 1.0e-13);
    EXPECT_NEAR(frame.GetNodeData(6, EnumKeyword::NodeResultType::U3), 0.0, 1.0e-13);
    EXPECT_NEAR(frame.GetNodeData(5, EnumKeyword::NodeResultType::M1), -torque, 1.0e-8);
    EXPECT_NEAR(frame.GetNodeData(6, EnumKeyword::NodeResultType::M1), torque, 1.0e-8);
    CheckBeamRootReaction(regions, 5, 3, -torque, 1.0e-8);
}
}
