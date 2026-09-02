#pragma once

#include <QString>

#include <memory>
#include <vector>

class DataFrame;
class Hdf5ModelIO;
class StructureData;
struct SolverIterationRecord;

class Hdf5ResultWriter final
{
public:
    Hdf5ResultWriter();
    ~Hdf5ResultWriter();

    Hdf5ResultWriter(const Hdf5ResultWriter&) = delete;
    Hdf5ResultWriter& operator=(const Hdf5ResultWriter&) = delete;

    bool begin(const QString& filePath, const StructureData* structure, const QString& sourceModelName); // 创建结果流
    bool writeFrames(int firstDomainId, const std::vector<DataFrame>& frames); // 追加连续结果帧
    bool writeIterationHistory(const std::vector<SolverIterationRecord>& records); // 写入求解迭代记录
    bool end(bool resultComplete = true); // 结束结果流并标记完整性

private:
    std::unique_ptr<Hdf5ModelIO> m_modelIO; // 底层 HDF5 读写实现
};
