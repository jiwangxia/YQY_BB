#pragma once

#include "Base/EmptyOUT.h"
#include "Export/Hdf5ResultData.h"

#include <QString>

#include <memory>
#include <vector>

class Hdf5ModelIO;
struct SolverIterationRecord;

class Hdf5ResultReader final
{
public:
    Hdf5ResultReader();
    ~Hdf5ResultReader();

    Hdf5ResultReader(const Hdf5ResultReader&) = delete;
    Hdf5ResultReader& operator=(const Hdf5ResultReader&) = delete;

    bool inspect(const QString& filePath, _OUT Hdf5ResultSummary& summary) const; // 快速读取结果摘要
    bool open(const QString& filePath, _OUT std::vector<Hdf5ResultFrameInfo>& frames); // 打开文件并读取帧索引
    bool readRanges(_OUT Hdf5ResultRanges& ranges) const; // 读取结果数值范围
    bool readFrame(int frameIndex, _OUT Hdf5ResultFrame& frame) const; // 按索引读取一帧结果
    bool readIterationHistory(_OUT std::vector<SolverIterationRecord>& records) const; // 读取求解迭代记录
    void close(); // 关闭当前结果文件

private:
    std::unique_ptr<Hdf5ModelIO> m_modelIO; // 底层 HDF5 读写实现
};
