#include "Export/Hdf5ResultReader.h"

#include "Export/Hdf5ModelIO.h"

Hdf5ResultReader::Hdf5ResultReader()
    : m_modelIO(std::make_unique<Hdf5ModelIO>())
{
}

Hdf5ResultReader::~Hdf5ResultReader() = default;

bool Hdf5ResultReader::inspect(const QString& filePath, _OUT Hdf5ResultSummary& summary) const
{
    return m_modelIO->InspectHdf5(filePath, summary);
}

bool Hdf5ResultReader::open(const QString& filePath, _OUT std::vector<Hdf5ResultFrameInfo>& frames)
{
    close();
    return m_modelIO->OpenResultFile(filePath, frames);
}

bool Hdf5ResultReader::readRanges(_OUT Hdf5ResultRanges& ranges) const
{
    return m_modelIO->ReadResultRanges(ranges);
}

bool Hdf5ResultReader::readFrame(int frameIndex, _OUT Hdf5ResultFrame& frame) const
{
    return m_modelIO->ReadResultFrame(frameIndex, frame);
}

bool Hdf5ResultReader::readIterationHistory(_OUT std::vector<SolverIterationRecord>& records) const
{
    return m_modelIO->ReadSolverIterationHistory(records);
}

void Hdf5ResultReader::close()
{
    m_modelIO->CloseResultFile();
}
