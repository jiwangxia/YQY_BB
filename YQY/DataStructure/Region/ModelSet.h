#pragma once

#include "Base/Base.h"

#include <QString>
#include <set>

enum class ModelSetType
{
    Node,
    Element
};

class ModelSet : public Base
{
public:
    ModelSet() = default;
    explicit ModelSet(ModelSetType type);

    QString m_Name;
    ModelSetType m_Type = ModelSetType::Node;
    std::set<int> m_Ids;
};
