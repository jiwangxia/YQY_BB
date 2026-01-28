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
        LOAD,           ///< 荷载
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
        B31       ///< 梁单元
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
};

