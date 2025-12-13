#include "EnumKeyword.h"

const QMap<QString, classKeyword::KeyData> classKeyword::MapKeyData = 
{
    {"NODE", classKeyword::KeyData::NODE},
    {"ELEMENT", classKeyword::KeyData::ELEMENT},
    {"MATERIAL", classKeyword::KeyData::MATERIAL},
    {"SECTION", classKeyword::KeyData::SECTION}
};

const QMap<QString, classKeyword::Direction> classKeyword::MapDirection = 
{
    {"X", classKeyword::Direction::X},
    {"Y", classKeyword::Direction::Y},
    {"Z", classKeyword::Direction::Z},
    {"RX", classKeyword::Direction::RX},
    {"RY", classKeyword::Direction::RY},
    {"RZ", classKeyword::Direction::RZ}
};

const QMap<QString, classKeyword::ElementType> classKeyword::MapElementType = 
{
    {"T3D2", classKeyword::ElementType::T3D2},
    {"B31", classKeyword::ElementType::B31}
};

const QMap<QString, classKeyword::SectionType> classKeyword::MapSectionType = 
{
    {"CIRCULAR", classKeyword::SectionType::CIRCULAR},
    {"L", classKeyword::SectionType::L},
    {"RECTANGULAR", classKeyword::SectionType::RECTANGULAR}
};
