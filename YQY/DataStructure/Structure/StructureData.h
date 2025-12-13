#pragma once
#include "Base/Base.h"
#include "DataStructure/Node/Node.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Section/SectionBase.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Property/Property.h"

class StructureData : public Base
{
public:


	//存储数据
	std::map<int, std::shared_ptr<Node>>              m_Nodes;
	std::map<int, std::shared_ptr<ElementBase>>       m_Elements;
	std::map<int, std::shared_ptr<Material>>          m_Material;
	std::map<int, std::shared_ptr<SectionBase>>       m_Section;
	std::map<int, std::shared_ptr<Property>>          m_Property;

	~StructureData();


	std::shared_ptr<Node>           FindNode(int id);
	std::shared_ptr<Material>       FindMaterial(int id);
	std::shared_ptr<SectionBase>    FindSection(int id);
	std::shared_ptr<Property>       FindProperty(int id);
	std::shared_ptr<Property>       Create_Property(int id_material, int id_section);
};

