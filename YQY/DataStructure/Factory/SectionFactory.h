#pragma once

#include "Utility/EnumKeyword.h"
#include <memory>

class SectionBase;
class SectionCircular;
class SectionRectangle;

class SectionFactory
{
public:
    // 仅分配对应类型，供导入时恢复原始几何字段；不支持的类型返回 nullptr。
    static std::shared_ptr<SectionBase> Create(EnumKeyword::SectionType type);
    // 按面积计算半径，供新建圆截面使用。
    static std::shared_ptr<SectionCircular> CreateCircular(int id, double area);
    // 按宽、高计算面积，供新建矩形截面使用。
    static std::shared_ptr<SectionRectangle> CreateRectangle(int id, double width, double height);
    // 保留原始几何、惯性及诊断字段，不重新计算；空对象或不支持的类型返回 nullptr。
    static std::shared_ptr<SectionBase> Clone(const std::shared_ptr<SectionBase>& source);
};
