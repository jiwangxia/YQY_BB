#pragma once

#include "Utility/EnumKeyword.h"

#include <QString>

#include <vector>

class ResultExporter final
{
public:
    // 从 HDF5 结果导出指定节点和单元的 BDF 格式结果。
    bool exportBdf(const QString& sourceFile, const QString& outputFile, const std::vector<int>& nodeIds,
                   const std::vector<EnumKeyword::NodeResultType>& nodeTypes, const std::vector<int>& elementIds,
                   const std::vector<EnumKeyword::ElementResultType>& elementTypes) const;
};
