#pragma once

#include "Utility/EnumKeyword.h"
#include "Export/Hdf5ResultData.h"

#include <QString>

#include <vector>
#include <array>

class ResultExporter final
{
public:
    // 导出选定能量分量的完整时程；不可用值标为 NA，文件写完后再替换目标。
    bool exportEnergyBdf(const QString& outputFile, const std::vector<Hdf5EnergyHistorySample>& samples,
                         unsigned int selectedMask) const;
    // 导出直接读取的节点时程，保留 BDF 结果表的 TIME、N编号-分量列名。
    bool exportNodalHistoryBdf(const QString& outputFile, int nodeId,
                               const std::vector<Hdf5NodalDisplacementHistorySample>& samples,
                               const std::array<bool, 6>& selected) const;
    // 从 HDF5 结果导出指定节点和单元的 BDF 格式结果。
    bool exportBdf(const QString& sourceFile, const QString& outputFile, const std::vector<int>& nodeIds,
                   const std::vector<EnumKeyword::NodeResultType>& nodeTypes, const std::vector<int>& elementIds,
                   const std::vector<EnumKeyword::ElementResultType>& elementTypes) const;
};
