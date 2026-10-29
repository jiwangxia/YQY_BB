#include "Support/SolveModel.h"

#include "DataStructure/Element/ElementSpring1.h"
#include "DataStructure/Element/ElementSpring2.h"
#include "DataStructure/Element/ElementSpringA.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Outputter.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
static void ExpectSpringResult(double actual, double expected, double absoluteTolerance = 2.0e-9)
{
    ASSERT_TRUE(std::isfinite(actual));
    ASSERT_TRUE(std::isfinite(expected));
    EXPECT_NEAR(actual, expected, absoluteTolerance + 2.0e-9 * std::abs(expected));
}

static double SpringExtensionFromForce(double force)
{
    // Springs.bdf 的分段直线反函数，答案不读取生产本构缓存。
    if (force < -20.0)
        return (force - 60.0) / 400.0;
    if (force < 0.0)
        return force / 100.0;
    if (force < 30.0)
        return force / 300.0;
    if (force < 180.0)
        return (force + 20.0) / 500.0;
    return (force + 140.0) / 800.0;
}

static double SpringForceFromExtension(double extension)
{
    if (extension < -0.2)
        return 400.0 * extension + 60.0;
    if (extension < 0.0)
        return 100.0 * extension;
    if (extension < 0.1)
        return 300.0 * extension;
    if (extension < 0.4)
        return 500.0 * extension - 20.0;
    return 800.0 * extension - 140.0;
}

static double SpringEnergyFromExtension(double extension)
{
    // Springs.bdf 各段 F(s) 的解析原函数，令 U(0)=0，并在全部折点连续。
    if (extension < -0.6)
        return -180.0 * extension - 66.0;
    if (extension < -0.2)
        return 200.0 * extension * extension + 60.0 * extension + 6.0;
    if (extension < 0.0)
        return 50.0 * extension * extension;
    if (extension < 0.1)
        return 150.0 * extension * extension;
    if (extension < 0.4)
        return 250.0 * extension * extension - 20.0 * extension + 1.0;
    if (extension < 0.8)
        return 400.0 * extension * extension - 140.0 * extension + 25.0;
    return 500.0 * extension - 231.0;
}

static double SymmetricSpringHeight(double appliedForce)
{
    // 对称 V 形的独立标量平衡：2*F(sqrt(1+y*y)-sqrt(3.25))*y/sqrt(1+y*y)=P。
    double lower = 1.0;
    double upper = 2.3;
    for (int iteration = 0; iteration < 80; ++iteration)
    {
        const double height = 0.5 * (lower + upper);
        const double length = std::hypot(1.0, height);
        const double verticalForce = 2.0 * SpringForceFromExtension(length - std::sqrt(3.25)) * height / length;
        if (verticalForce < appliedForce)
            lower = height;
        else
            upper = height;
    }
    return 0.5 * (lower + upper);
}

static void CheckSymmetricSpringFrame(const DataFrame& frame, int firstSupport, int secondSupport, int tip,
                                      int firstElement, double appliedForce)
{
    const double height = SymmetricSpringHeight(appliedForce);
    const double length = std::hypot(1.0, height);
    const double extension = length - std::sqrt(3.25);
    const double force = SpringForceFromExtension(extension);
    ExpectSpringResult(frame.GetNodeData(tip, EnumKeyword::NodeResultType::U1), 0.0);
    ExpectSpringResult(frame.GetNodeData(tip, EnumKeyword::NodeResultType::U2), height - 1.5);
    ExpectSpringResult(frame.GetNodeData(tip, EnumKeyword::NodeResultType::U3), 0.0);
    ExpectSpringResult(frame.GetNodeData(tip, EnumKeyword::NodeResultType::F2), appliedForce, 1.0e-7);
    ExpectSpringResult(frame.GetNodeData(firstSupport, EnumKeyword::NodeResultType::R1), -force / length, 1.0e-7);
    ExpectSpringResult(frame.GetNodeData(secondSupport, EnumKeyword::NodeResultType::R1), force / length, 1.0e-7);
    ExpectSpringResult(frame.GetNodeData(firstSupport, EnumKeyword::NodeResultType::R2), -0.5 * appliedForce, 1.0e-7);
    ExpectSpringResult(frame.GetNodeData(secondSupport, EnumKeyword::NodeResultType::R2), -0.5 * appliedForce, 1.0e-7);
    for (int elementId : {firstElement, firstElement + 1})
    {
        ExpectSpringResult(frame.GetElementData(elementId, EnumKeyword::ElementResultType::RelativeDisplacement),
                           extension);
        ExpectSpringResult(frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce), force, 1.0e-7);
    }
}

/** @brief 三类弹簧真实文件的拉压、预定位移及转轴静力求解验证
 *
 * @details 输入：Statics/NonlinearSprings/Springs.bdf，12 节点、8 弹簧，含 SPRING1、混合方向 SPRING2 和 V 形 SPRINGA。
 *          参考：分段本构反函数求单向弹簧位移；独立二分求 V 形竖向平衡，按当前轴向分解支座反力。
 *          判据：核对 0 至 1 的 11 帧，时间误差不超过 1e-12；位移和伸长允许 2e-9+2e-9*abs(reference)，
 *          力和反力允许 1e-7+2e-9*abs(reference)，节点及单元数量、类型精确匹配。
 *          2026.10.29 补充四种帧能量：独立积分分段 F(s)，应变能及总机械能采用上述默认绝对/相对容差；
 *          无质量、无重力的静力模型要求动能和重力势能为零，四分量均有效。
 *          范围：分段非线性静力、正负载荷、移动支座、空间轴线转动及各增量的物理结果。
 * @author   yangqiuyou
 * @date     2026.10.14
 */
TEST(NonlinearSprings, NewtonSignedLoadsAndRotatingAxes)
{
    QString error;
    const auto model = SolveModel::Read(QStringLiteral("Statics/NonlinearSprings/Springs.bdf"), error);
    ASSERT_TRUE(model) << error.toStdString();
    ASSERT_EQ(model->m_Nodes.size(), 12u);
    ASSERT_EQ(model->m_Elements.size(), 8u);
    ASSERT_EQ(model->m_SpringBehaviors.size(), 1u);
    ASSERT_TRUE(std::dynamic_pointer_cast<ElementSpring1>(model->FindElement(1)));
    ASSERT_TRUE(std::dynamic_pointer_cast<ElementSpring1>(model->FindElement(2)));
    ASSERT_TRUE(std::dynamic_pointer_cast<ElementSpring2>(model->FindElement(3)));
    ASSERT_TRUE(std::dynamic_pointer_cast<ElementSpring2>(model->FindElement(4)));
    for (int elementId = 5; elementId <= 8; ++elementId)
        ASSERT_TRUE(std::dynamic_pointer_cast<ElementSpringA>(model->FindElement(elementId)));
    ASSERT_TRUE(SolveModel::Run(model, error)) << error.toStdString();
    const auto& frames = model->GetOutputter().GetDataSet();
    ASSERT_EQ(frames.size(), 11u);
    for (std::size_t index = 0; index < frames.size(); ++index)
    {
        const DataFrame& frame = frames[index];
        SCOPED_TRACE(::testing::Message() << "increment=" << index << " time=" << frame.GetTime());
        ASSERT_EQ(frame.GetStepId(), 1);
        ASSERT_EQ(frame.GetAnalysisType(), static_cast<int>(EnumKeyword::StepType::STATIC));
        ASSERT_EQ(frame.GetNodeDatas().size(), 12u);
        ASSERT_EQ(frame.GetElementDatas().size(), 8u);
        const double factor = 0.1 * static_cast<double>(index);
        EXPECT_NEAR(frame.GetTime(), factor, 1.0e-12);
        const double firstForce = 230.0 * factor;
        const double secondForce = -100.0 * factor;
        const double thirdForce = 270.0 * factor;
        const double fourthForce = -130.0 * factor;
        ExpectSpringResult(frame.GetNodeData(1, EnumKeyword::NodeResultType::U1), SpringExtensionFromForce(firstForce));
        ExpectSpringResult(frame.GetNodeData(2, EnumKeyword::NodeResultType::U2),
                           SpringExtensionFromForce(secondForce));
        ExpectSpringResult(frame.GetNodeData(3, EnumKeyword::NodeResultType::U3),
                           SpringExtensionFromForce(thirdForce) + 0.07 * factor);
        ExpectSpringResult(frame.GetNodeData(5, EnumKeyword::NodeResultType::U2),
                           SpringExtensionFromForce(fourthForce) - 0.09 * factor);
        ExpectSpringResult(frame.GetNodeData(4, EnumKeyword::NodeResultType::U2), 0.07 * factor);
        ExpectSpringResult(frame.GetNodeData(6, EnumKeyword::NodeResultType::U3), -0.09 * factor);
        ExpectSpringResult(frame.GetNodeData(1, EnumKeyword::NodeResultType::F1), firstForce, 1.0e-7);
        ExpectSpringResult(frame.GetNodeData(2, EnumKeyword::NodeResultType::F2), secondForce, 1.0e-7);
        ExpectSpringResult(frame.GetNodeData(3, EnumKeyword::NodeResultType::F3), thirdForce, 1.0e-7);
        ExpectSpringResult(frame.GetNodeData(5, EnumKeyword::NodeResultType::F2), fourthForce, 1.0e-7);
        ExpectSpringResult(frame.GetNodeData(4, EnumKeyword::NodeResultType::R2), -thirdForce, 1.0e-7);
        ExpectSpringResult(frame.GetNodeData(6, EnumKeyword::NodeResultType::R3), -fourthForce, 1.0e-7);
        const double forces[] = {firstForce, secondForce, thirdForce, fourthForce};
        for (int elementId = 1; elementId <= 4; ++elementId)
        {
            ExpectSpringResult(frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce),
                               forces[elementId - 1], 1.0e-7);
            ExpectSpringResult(frame.GetElementData(elementId, EnumKeyword::ElementResultType::RelativeDisplacement),
                               SpringExtensionFromForce(forces[elementId - 1]));
        }
        CheckSymmetricSpringFrame(frame, 7, 8, 9, 5, 100.0 * factor);
        CheckSymmetricSpringFrame(frame, 10, 11, 12, 7, -30.0 * factor);
        double expectedStrainEnergy = 0.0;
        for (const double force : forces)
            expectedStrainEnergy += SpringEnergyFromExtension(SpringExtensionFromForce(force));
        for (const double appliedForce : {100.0 * factor, -30.0 * factor})
        {
            const double height = SymmetricSpringHeight(appliedForce);
            const double extension = std::hypot(1.0, height) - std::sqrt(3.25);
            expectedStrainEnergy += 2.0 * SpringEnergyFromExtension(extension);
        }
        const EnergyResult& energy = frame.GetEnergy();
        ASSERT_EQ(energy.availableMask, EnergyResult::All);
        EXPECT_DOUBLE_EQ(energy.kinetic, 0.0);
        EXPECT_DOUBLE_EQ(energy.gravity, 0.0);
        ExpectSpringResult(energy.strain, expectedStrainEnergy);
        ExpectSpringResult(energy.mechanical, expectedStrainEnergy);
    }
}
}
