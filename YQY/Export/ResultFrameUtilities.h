#pragma once

#include "Hdf5ResultData.h"

#include <memory>

class StructureData;

namespace ResultFrameUtilities
{
bool modelsMatch(const std::shared_ptr<StructureData>& current, const std::shared_ptr<StructureData>& embedded);
Hdf5ResultFrame interpolate(const Hdf5ResultFrame& first, const Hdf5ResultFrame& second, double interpolation);
}
