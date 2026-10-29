#pragma once

#include "Utility/EnumKeyword.h"

#include <Eigen/Core>
#include <memory>

class ElementBase;

struct ElementCreationOptions
{
    Eigen::Vector3d beamDirection = Eigen::Vector3d::UnitZ(); // 仅 CR3D 梁使用的截面方向。
    int firstDOF = 0;  // SPRING1 或 SPRING2 第一节点的内部自由度，范围为 0 至 5。
    int secondDOF = 0; // 仅 SPRING2 第二节点使用的内部自由度，范围为 0 至 5。
};

class ElementFactory
{
public:
    // 仅创建单元并设置类型专属参数；调用方负责节点、属性关联及运行状态初始化，并纳入模型。
    // 不支持的类型返回 nullptr。
    static std::shared_ptr<ElementBase> Create(EnumKeyword::ElementType type,
                                               const ElementCreationOptions& options = {});
};
