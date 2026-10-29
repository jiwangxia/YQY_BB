#include "SectionFactory.h"

#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Section/SectionRectangle.h"

std::shared_ptr<SectionBase> SectionFactory::Create(EnumKeyword::SectionType type)
{
    switch (type)
    {
        case EnumKeyword::SectionType::CIRCULAR:
            return std::make_shared<SectionCircular>();
        case EnumKeyword::SectionType::RECTANGULAR:
            return std::make_shared<SectionRectangle>();
        default:
            return nullptr;
    }
}

std::shared_ptr<SectionCircular> SectionFactory::CreateCircular(int id, double area)
{
    auto section = std::make_shared<SectionCircular>();
    section->m_Id = id;
    section->m_Area = area;
    section->Calculate_Radius();
    return section;
}

std::shared_ptr<SectionRectangle> SectionFactory::CreateRectangle(int id, double width, double height)
{
    auto section = std::make_shared<SectionRectangle>();
    section->m_Id = id;
    section->m_Width = width;
    section->m_Height = height;
    section->Calculate_Area();
    return section;
}

std::shared_ptr<SectionBase> SectionFactory::Clone(const std::shared_ptr<SectionBase>& source)
{
    if (const auto circular = std::dynamic_pointer_cast<SectionCircular>(source))
        return std::make_shared<SectionCircular>(*circular);
    if (const auto rectangle = std::dynamic_pointer_cast<SectionRectangle>(source))
        return std::make_shared<SectionRectangle>(*rectangle);
    return nullptr;
}
