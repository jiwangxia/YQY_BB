#include "Export/Hdf5ResultWriter.h"

#include "Export/Hdf5ModelIO.h"

Hdf5ResultWriter::Hdf5ResultWriter()
    : m_modelIO(std::make_unique<Hdf5ModelIO>())
{
}

Hdf5ResultWriter::~Hdf5ResultWriter() = default;

bool Hdf5ResultWriter::begin(const QString& filePath, const StructureData* structure, const QString& sourceModelName)
{
    return m_modelIO->BeginResultStream(filePath, structure, sourceModelName);
}

bool Hdf5ResultWriter::writeFrames(int firstDomainId, const std::vector<DataFrame>& frames)
{
    return m_modelIO->WriteResultFrames(firstDomainId, frames);
}

bool Hdf5ResultWriter::writeIterationHistory(const std::vector<SolverIterationRecord>& records)
{
    return m_modelIO->WriteSolverIterationHistory(records);
}

bool Hdf5ResultWriter::end(bool resultComplete)
{
    return m_modelIO->EndResultStream(resultComplete);
}
