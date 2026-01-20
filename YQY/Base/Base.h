#pragma once
#include <QVector>
#include <memory>
#include <map>
#include <qdebug>
#include <iostream>
#include <Eigen/Dense> 
#include <Eigen/Sparse>

#include "Utility/EnumKeyword.h"
using namespace Eigen;

#define PI 3.141592653589793

/**
 * @brief 基类 - 所有数据结构的公共基类
 */
class Base
{
public:
    Base();
    int m_Id;  ///< 对象ID

    virtual ~Base() = default;  ///< 虚析构函数，确保派生类正确析构
};

