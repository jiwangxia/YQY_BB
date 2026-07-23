#pragma once
#include <QMap>
#include <QString>

/**
 * @brief 枚举关键字类 - 定义输入文件的关键字枚举和映射表
 */
class EnumKeyword
{
public:
    /**
     * @brief 数据文件关键字枚举
     */
    enum class KeyData
    {
        UNKNOWN,        ///< 未知
        MATERIAL,       ///< 材料
        SECTION,        ///< 截面
        NODE,           ///< 节点
        ELEMENT,        ///< 单元
        CONSTRAINT,     ///< 约束
        CONSTRAINT_TABULAR, ///< 约束分段线性时程
        LOAD,           ///< 荷载
        STRESS,         ///< 初始应力
        ANALYSIS_STEP   ///< 分析步
    };
    static const QMap<QString, KeyData> MapKeyData;  ///< 关键字字符串到枚举的映射

    /**
     * @brief 三维坐标系方向枚举
     */
    enum class Direction
    {
        X,       ///< X 方向平移
        Y,       ///< Y 方向平移
        Z,       ///< Z 方向平移
        RX,      ///< 绕 X 轴转动
        RY,      ///< 绕 Y 轴转动
        RZ,      ///< 绕 Z 轴转动
        UNKNOWN  ///< 未知
    };
    static const QMap<QString, Direction> MapDirection;  ///< 方向字符串到枚举的映射

    /**
     * @brief 单元类型枚举
     */
    enum class ElementType
    {
        UNKNOWN,  ///< 未知
        T3D2,     ///< 桁架单元
        CABLE,    ///< 索单元
        CR3D      ///< 3维CR梁单元
    };
    static const QMap<QString, ElementType> MapElementType;  ///< 单元类型字符串到枚举的映射

    /**
     * @brief 截面类型枚举
     */
    enum class SectionType
    {
        UNKNOWN,      ///< 未知
        CIRCULAR,     ///< 圆形截面
        L,            ///< L形截面
        RECTANGULAR   ///< 矩形截面
    };
    static const QMap<QString, SectionType> MapSectionType;  ///< 截面类型字符串到枚举的映射

    /**
     * @brief 荷载类型枚举
     */
    enum class LoadType
    {
        FORCE_NODE,     ///< 节点力
        FORCE_ELEMENT,  ///< 单元荷载
        FORCE_GRAVITY,  ///< 重力荷载
        FORCE_WIND,     ///< 风荷载
        FORCE_ICE,      ///< 冰荷载
        UNKNOWN         ///< 未知
    };
    static const QMap<QString, LoadType> MapLoadType;  ///< 荷载类型字符串到枚举的映射

    /**
     * @brief 分析步类型枚举
     */
    enum class StepType
    {
        STATIC,   ///< 静力分析
        DYNAMIC,  ///< 动力分析
        UNKNOWN   ///< 未知
    };
    static const QMap<QString, StepType> MapStepType;  ///< 分析步类型字符串到枚举的映射

    /**
     * @brief 节点结果数据类型枚举
     */
    enum class NodeResultType
    {
        U1, U2, U3, MagnitudeU,       ///< 位移及位移幅值
        CX, CY, CZ,                   ///< 当前坐标 = 初始坐标 + 位移
        V1, V2, V3,                   ///< 速度
        A1, A2, A3,                   ///< 加速度
        UR1, UR2, UR3,                ///< 转角
        F1, F2, F3,                   ///< 节点内力
        M1, M2, M3,                   ///< 节点力矩
        R1, R2, R3                    ///< 节点反力
    };

    /**
     * @brief 单元结果数据类型枚举
     */
    enum class ElementResultType
    {
        AxialForce,                   ///< 轴力
        ShearY,                       ///< 局部 y 向剪力
        ShearZ,                       ///< 局部 z 向剪力
        Torque,                       ///< 扭矩
        MomentY,                      ///< 绕局部 y 轴弯矩
        MomentZ,                      ///< 绕局部 z 轴弯矩
        Strain,                       ///< 应变
        InitStress,                   ///< 初始应力
        CurrentStress,                ///< 当前应力
        DeltaStress                   ///< 应力增量
    };
};

