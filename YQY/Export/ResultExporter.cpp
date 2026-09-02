#include "Export/ResultExporter.h"

#include "Export/Hdf5ModelIO.h"

bool ResultExporter::exportBdf(const QString& sourceFile, const QString& outputFile, const std::vector<int>& nodeIds,
                               const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
                               const std::vector<int>& elementIds,
                               const std::vector<EnumKeyword::ElementResultType>& elementTypes) const
{
    Hdf5ModelIO modelIO;
    return modelIO.ExportBdfResultFromHdf5(sourceFile, outputFile, nodeIds, nodeTypes, elementIds, elementTypes);
}
