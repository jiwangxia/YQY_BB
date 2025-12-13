#include "EnumKeyword.h"

const QMap<QString, EnumKeyword::KeyData> EnumKeyword::MapKeyData = 
{
    {"NODE", EnumKeyword::KeyData::NODE},
    {"ELEMENT", EnumKeyword::KeyData::ELEMENT},
    {"MATERIAL", EnumKeyword::KeyData::MATERIAL},
    {"SECTION", EnumKeyword::KeyData::SECTION},
    {"CONSTRAINT", EnumKeyword::KeyData::CONSTRAINT},
    {"LOAD", EnumKeyword::KeyData::LOAD}
};

const QMap<QString, EnumKeyword::Direction> EnumKeyword::MapDirection = 
{
    {"X", EnumKeyword::Direction::X},
    {"Y", EnumKeyword::Direction::Y},
    {"Z", EnumKeyword::Direction::Z},
    {"RX", EnumKeyword::Direction::RX},
    {"RY", EnumKeyword::Direction::RY},
    {"RZ", EnumKeyword::Direction::RZ}
};

const QMap<QString, EnumKeyword::ElementType> EnumKeyword::MapElementType = 
{
    {"T3D2", EnumKeyword::ElementType::T3D2},
    {"B31", EnumKeyword::ElementType::B31}
};

const QMap<QString, EnumKeyword::SectionType> EnumKeyword::MapSectionType = 
{
    {"CIRCULAR", EnumKeyword::SectionType::CIRCULAR},
    {"L", EnumKeyword::SectionType::L},
    {"RECTANGULAR", EnumKeyword::SectionType::RECTANGULAR}
};

const QMap<QString, EnumKeyword::LoadType> EnumKeyword::MapLoadType =
{
    {"NODE_FORCE", EnumKeyword::LoadType::NODE_FORCE},
    {"ELEMENT_PRESSURE", EnumKeyword::LoadType::ELEMENT_PRESSURE}
};