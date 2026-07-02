#pragma once

/**
 * @brief 一维材料模型类型
 */
enum class MaterialModel
{
    Elastic,             ///< 线弹性
    IdealPlastic1D,      ///< 理想弹塑性，H = 0
    HardeningPlastic1D   ///< 线性硬化弹塑性，H > 0
};

/**
 * @brief 杆单元材料点在一个已收敛增量步上的状态
 *
 * 该状态属于单元，不能保存在多个单元共享的 Material 对象中。
 */
struct Material1DState
{
    double strain = 0.0;             ///< 总应变
    double stress = 0.0;             ///< 总应力，包含初始应力
    double plasticStrain = 0.0;      ///< 有符号塑性应变
    double accumulatedPlastic = 0.0; ///< 非负累积塑性应变
};

/**
 * @brief 一维材料本构计算结果
 */
struct Material1DResult
{
    double stress = 0.0;       ///< 当前试算应力
    double stiffness = 0.0;    ///< 一维切线刚度，即 1×1 矩阵的唯一分量
    Material1DState state;     ///< 当前试算状态，收敛前不能提交
};

/**
 * @brief 计算一维弹性或双线性弹塑性材料响应
 * @param model 材料模型类型
 * @param young 弹性模量 E
 * @param yieldStress 初始屈服应力
 * @param hardening 线性硬化模量 H，0 表示理想塑性
 * @param strain 当前总应变
 * @param oldState 上一个已收敛增量步的状态
 * @return 当前试算应力、切线刚度和试算状态
 *
 * 每次调用都从 oldState 重新计算，因此 Newton 迭代反复调用时
 * 不会重复累积塑性应变。
 */
Material1DResult CalculateMaterial1D(
    MaterialModel model,
    double young,
    double yieldStress,
    double hardening,
    double strain,
    const Material1DState& oldState);
