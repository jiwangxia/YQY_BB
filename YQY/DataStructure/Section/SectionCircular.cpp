#include "SectionCircular.h"

#include "Base/Base.h"

void SectionCircular::Calculate_Radius()
{
    m_Radius = sqrt(m_Area / PI);
}
