#pragma once

#include "Hdf5ResultData.h"

#include <memory>

class StructureData;

namespace ResultFrameUtilities
{
// 判断当前模型与结果文件内嵌模型的节点和单元是否一致。
bool modelsMatch(const std::shared_ptr<StructureData>& current, const std::shared_ptr<StructureData>& embedded);
// 在线性时间位置插值两帧结果。
Hdf5ResultFrame interpolate(const Hdf5ResultFrame& first, const Hdf5ResultFrame& second, double interpolation);
}
