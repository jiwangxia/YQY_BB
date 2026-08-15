#include "EnumKeyword.h"

const QMap<QString, EnumKeyword::KeyData> EnumKeyword::MapKeyData = { //读取需要在此处增加关键字
    {"NODE", EnumKeyword::KeyData::NODE},
    {"ELEMENT", EnumKeyword::KeyData::ELEMENT},
    {"MATERIAL", EnumKeyword::KeyData::MATERIAL},
    {"SECTION", EnumKeyword::KeyData::SECTION},
    {"CONSTRAINT", EnumKeyword::KeyData::CONSTRAINT},
    {"CONSTRAINT_TABULAR", EnumKeyword::KeyData::CONSTRAINT_TABULAR},
    {"MPC", EnumKeyword::KeyData::MPC},
    {"LOAD", EnumKeyword::KeyData::LOAD},
    {"STRESS", EnumKeyword::KeyData::STRESS},
    {"ANALYSIS_STEP", EnumKeyword::KeyData::ANALYSIS_STEP}};

const QMap<QString, EnumKeyword::Direction> EnumKeyword::MapDirection = {
    {"X", EnumKeyword::Direction::X},   {"Y", EnumKeyword::Direction::Y},   {"Z", EnumKeyword::Direction::Z},
    {"RX", EnumKeyword::Direction::RX}, {"RY", EnumKeyword::Direction::RY}, {"RZ", EnumKeyword::Direction::RZ}};

const QMap<QString, EnumKeyword::ElementType> EnumKeyword::MapElementType = {
    {"T3D2", EnumKeyword::ElementType::T3D2},
    {"CABLE", EnumKeyword::ElementType::CABLE},
    {"CR3D", EnumKeyword::ElementType::CR3D},

};

const QMap<QString, EnumKeyword::SectionType> EnumKeyword::MapSectionType = {
    {"CIRCULAR", EnumKeyword::SectionType::CIRCULAR},
    {"L", EnumKeyword::SectionType::L},
    {"RECTANGULAR", EnumKeyword::SectionType::RECTANGULAR}};

const QMap<QString, EnumKeyword::LoadType> EnumKeyword::MapLoadType = {
    {"FORCE_NODE", EnumKeyword::LoadType::FORCE_NODE},
    {"FORCE_ELEMENT", EnumKeyword::LoadType::FORCE_ELEMENT},
    {"FORCE_GRAVITY", EnumKeyword::LoadType::FORCE_GRAVITY},
    {"FORCE_WIND", EnumKeyword::LoadType::FORCE_WIND},
    {"FORCE_ICE", EnumKeyword::LoadType::FORCE_ICE}};

const QMap<QString, EnumKeyword::StepType> EnumKeyword::MapStepType = {{"STATIC", EnumKeyword::StepType::STATIC},
                                                                       {"DYNAMIC", EnumKeyword::StepType::DYNAMIC}};
