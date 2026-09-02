#include "Export/Hdf5ModelSerializer.h"

#include "Export/Hdf5ModelIO.h"

bool Hdf5ModelSerializer::importModel(const QString& filePath, _OUT StructureData* structure) const
{
    Hdf5ModelIO modelIO;
    return modelIO.ImportHdf5(filePath, structure);
}

bool Hdf5ModelSerializer::exportModel(const QString& filePath, const StructureData* structure,
                                      const QString& sourceModelName) const
{
    Hdf5ModelIO modelIO;
    return modelIO.ExportModelHdf5(filePath, structure, sourceModelName);
}

bool Hdf5ModelSerializer::restoreLastDynamicState(const QString& filePath, _OUT StructureData* structure,
                                                  _OUT double* time, _OUT int* stepId) const
{
    Hdf5ModelIO modelIO;
    return modelIO.RestoreLastDynamicState(filePath, structure, time, stepId);
}
