#pragma once

#include "Utility/EnumKeyword.h"

#include <memory>

class LoadBase;

class LoadFactory final
{
public:
    // value 对应集中力、单元力、重力加速度或风速；调用方负责实体关联并纳入模型。
    // 不支持的类型返回 nullptr。
    static std::shared_ptr<LoadBase> Create(EnumKeyword::LoadType type, double value);
    // 保留全部荷载参数，弱引用仍指向源模型实体，须由调用方按目标模型重新绑定。
    // 空对象或不支持的类型返回 nullptr。
    static std::shared_ptr<LoadBase> Clone(const std::shared_ptr<LoadBase>& source);
};
