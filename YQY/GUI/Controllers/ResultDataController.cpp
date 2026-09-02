#include "GUI/Controllers/ResultDataController.h"

#include "Export/Hdf5ResultReader.h"

ResultDataController::ResultDataController()
    : m_reader(std::make_unique<Hdf5ResultReader>())
{
}

ResultDataController::~ResultDataController() = default;

bool ResultDataController::inspect(const QString& filePath, _OUT Hdf5ResultSummary& summary) const
{
    return m_reader->inspect(filePath, summary);
}

bool ResultDataController::open(const QString& filePath, _OUT std::vector<Hdf5ResultFrameInfo>& frames,
                                _OUT Hdf5ResultRanges& ranges)
{
    close();
    if (!m_reader->open(filePath, frames) || frames.empty())
        return false;

    if (!m_reader->readRanges(ranges))
        ranges = {};
    return true;
}

bool ResultDataController::readFrame(int frameIndex, _OUT Hdf5ResultFrame& frame)
{
    return m_reader->readFrame(frameIndex, frame);
}

bool ResultDataController::readFramePair(int firstIndex, int secondIndex, _OUT Hdf5ResultFrame& firstFrame,
                                         _OUT Hdf5ResultFrame& secondFrame)
{
    if (m_cachedFrameIndex != firstIndex)
    {
        if (m_cachedNextFrameIndex == firstIndex)
        {
            std::swap(m_cachedFrameIndex, m_cachedNextFrameIndex);
            std::swap(m_cachedFrame, m_cachedNextFrame);
        }
        else if (!m_reader->readFrame(firstIndex, m_cachedFrame))
        {
            return false;
        }
        m_cachedFrameIndex = firstIndex;
    }

    if (m_cachedNextFrameIndex != secondIndex)
    {
        if (!m_reader->readFrame(secondIndex, m_cachedNextFrame))
            return false;
        m_cachedNextFrameIndex = secondIndex;
    }
    firstFrame = m_cachedFrame;
    secondFrame = m_cachedNextFrame;
    return true;
}

void ResultDataController::close()
{
    m_reader->close();
    m_cachedFrameIndex = -1;
    m_cachedNextFrameIndex = -1;
    m_cachedFrame = {};
    m_cachedNextFrame = {};
}
