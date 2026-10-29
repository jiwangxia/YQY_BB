#include "LoadFactory.h"

#include "DataStructure/Load/Force_Element.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Node.h"
#include "DataStructure/Load/Force_Wind.h"

std::shared_ptr<LoadBase> LoadFactory::Create(EnumKeyword::LoadType type, double value)
{
    switch (type)
    {
        case EnumKeyword::LoadType::FORCE_NODE:
        {
            auto load = std::make_shared<Force_Node>();
            load->m_Value = value;
            return load;
        }
        case EnumKeyword::LoadType::FORCE_ELEMENT:
        {
            auto load = std::make_shared<Force_Element>();
            load->m_Value = value;
            return load;
        }
        case EnumKeyword::LoadType::FORCE_GRAVITY:
        {
            auto load = std::make_shared<Force_Gravity>();
            load->m_g = value;
            return load;
        }
        case EnumKeyword::LoadType::FORCE_WIND:
        {
            auto load = std::make_shared<Force_Wind>();
            load->m_velocity = value;
            return load;
        }
        default:
            return nullptr;
    }
}

std::shared_ptr<LoadBase> LoadFactory::Clone(const std::shared_ptr<LoadBase>& source)
{
    if (const auto load = std::dynamic_pointer_cast<Force_Node>(source))
        return std::make_shared<Force_Node>(*load);
    if (const auto load = std::dynamic_pointer_cast<Force_Element>(source))
        return std::make_shared<Force_Element>(*load);
    if (const auto load = std::dynamic_pointer_cast<Force_Gravity>(source))
        return std::make_shared<Force_Gravity>(*load);
    if (const auto load = std::dynamic_pointer_cast<Force_Wind>(source))
        return std::make_shared<Force_Wind>(*load);
    return nullptr;
}
