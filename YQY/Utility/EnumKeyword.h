#include <QMap>
#include <QString>

class classKeyword
{
public:
    enum class KeyData
    {//数据文件关键字
        UNKNOWN, MATERIAL, SECTION, NODE, ELEMENT, BEAM3D, TRUSS3D
    };
    static const QMap<QString, KeyData> MapKeyData;

    enum class Direction
    {//三维坐标系的方向
        UNKNOWN, X, Y, Z, RX, RY, RZ
    };
    static const QMap<QString, Direction> MapDirection;

    enum class ElementType
    {//单元类型
        UNKNOWN, T3D2, B31
    };
    static const QMap<QString, ElementType> MapElementType;

    enum class SectionType
    {//截面类型
        UNKNOWN, CIRCULAR, L, RECTANGULAR
    };
    static const QMap<QString, SectionType> MapSectionType;
};

