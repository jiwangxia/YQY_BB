#include "StructureData.h"

StructureData::~StructureData()
{
}

std::shared_ptr<Node> StructureData::FindNode(int id)
{
    auto result = m_Nodes.find(id);
    if (result != m_Nodes.end())
    {
        return result->second;
    }
    return nullptr;
}

std::shared_ptr<Material> StructureData::FindMaterial(int id)
{
    auto result = m_Material.find(id);
    if (result != m_Material.end())
    {
        return result->second;
    }
    return nullptr;
}

std::shared_ptr<SectionBase> StructureData::FindSection(int id)
{
    auto result = m_Section.find(id);
    if (result != m_Section.end())
    {
        return result->second;
    }
    return nullptr;
}

std::shared_ptr<Property> StructureData::FindProperty(int id)
{
    auto result = m_Property.find(id);
    if (result != m_Property.end())
    {
        return result->second;
    }
    return nullptr;
}

std::shared_ptr<Property> StructureData::Create_Property(int id_material, int id_section)
{
    static int property_id = 1;
    auto property = std::make_shared<Property>();

    auto iterMat = m_Material.find(id_material);
    if (iterMat != m_Material.end())
    {
        property->m_pMaterial = iterMat->second;
    }

    auto iterSec = m_Section.find(id_section);
    if (iterSec != m_Section.end())
    {
        property->m_pSection = iterSec->second;
    }

    property->m_Id = property_id;   //确保Property的id和m_Property的id一样
    m_Property.insert(std::make_pair(property_id, property));
    property_id++;
    return property;
}
