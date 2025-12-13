#pragma once
#include "Base/Base.h"
class Node : public Base
{
public:
    Node();

    double m_X, m_Y, m_Z; //节点坐标
    QVector<int> m_DOF;  //节点自由度
};

