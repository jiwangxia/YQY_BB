#pragma once
#include <QVector>
#include <memory>
#include <map>
#include <qdebug>
#include <iostream>

#define PI 3.141592653589793
class Base
{
public:
    Base();
    int m_Id;

    virtual ~Base() = default;   //虚析构函数
};

