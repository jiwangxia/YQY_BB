#include "ElementFactory.h"

#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementSpring1.h"
#include "DataStructure/Element/ElementSpring2.h"
#include "DataStructure/Element/ElementSpringA.h"
#include "DataStructure/Element/ElementTruss.h"

std::shared_ptr<ElementBase> ElementFactory::Create(EnumKeyword::ElementType type,
                                                    const ElementCreationOptions& options)
{
    switch (type)
    {
        case EnumKeyword::ElementType::T3D2:
            return std::make_shared<ElementTruss>();
        case EnumKeyword::ElementType::CABLE:
            return std::make_shared<ElementCable>();
        case EnumKeyword::ElementType::CR3D:
        {
            auto element = std::make_shared<ElementBeam_CR>();
            element->q0 = options.beamDirection;
            return element;
        }
        case EnumKeyword::ElementType::SPRING1:
        {
            auto element = std::make_shared<ElementSpring1>();
            element->m_DOF = options.firstDOF;
            return element;
        }
        case EnumKeyword::ElementType::SPRING2:
        {
            auto element = std::make_shared<ElementSpring2>();
            element->m_FirstDOF = options.firstDOF;
            element->m_SecondDOF = options.secondDOF;
            return element;
        }
        case EnumKeyword::ElementType::SPRINGA:
            return std::make_shared<ElementSpringA>();
        default:
            return nullptr;
    }
}
