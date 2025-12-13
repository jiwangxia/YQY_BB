#pragma once
#include "Base/Base.h"

class Node;
class Property;
class ElementBase : public Base
{
public:
    ElementBase();
    QVector<std::weak_ptr<Node>> m_pNode;       //节点指针
    std::weak_ptr<Property>      m_pProperty;   //所属属性(材料+截面)

    virtual int Get_NodeDOF() const = 0;        //获取单元自由度个数
};

