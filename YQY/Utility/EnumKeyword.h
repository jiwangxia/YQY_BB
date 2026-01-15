#pragma once
#include <QMap>
#include <QString>

class EnumKeyword
{
public:
    enum class KeyData
    {//数据文件关键字
        UNKNOWN, MATERIAL, SECTION, NODE, ELEMENT, CONSTRAINT, LOAD, ANALYSIS_STEP
    };
    static const QMap<QString, KeyData> MapKeyData;

    enum class Direction
    {//三维坐标系的方向
        X, Y, Z, RX, RY, RZ, UNKNOWN 
    };
    static const QMap<QString, Direction> MapDirection;

    enum class ElementType
    {//单元类型
        UNKNOWN, T3D2, CABLE, B31
    };
    static const QMap<QString, ElementType> MapElementType;

    enum class SectionType
    {//截面类型
        UNKNOWN, CIRCULAR, L, RECTANGULAR
    };
    static const QMap<QString, SectionType> MapSectionType;

    enum class LoadType
    {//荷载类型
        FORCE_NODE, FORCE_ELEMENT, UNKNOWN
    };
    static const QMap<QString, LoadType> MapLoadType;

    enum class StepType
    {//分析步类型
        STATIC, DYNAMIC, UNKNOWN
    };
    static const QMap<QString, StepType> MapStepType;
};

