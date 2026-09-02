#pragma once

#include "Base/Base.h"

#include <QString>
#include <set>

// 模型集保存的对象类型。
enum class ModelSetType
{
    Node,
    Element
};

// 以编号集合表示的节点集或单元集。
class ModelSet : public Base
{
public:
    ModelSet() = default;
    explicit ModelSet(ModelSetType type);

    QString m_Name;                         // 模型集名称
    ModelSetType m_Type = ModelSetType::Node; // 成员对象类型
    std::set<int> m_Ids;                    // 节点或单元编号集合
};
