#pragma once

#include "Base/EmptyOUT.h"
#include "Export/Hdf5ResultData.h"

#include <QString>

#include <memory>
#include <vector>

class Hdf5ResultReader;

// 为界面提供带相邻帧缓存的 HDF5 结果读取服务。
class ResultDataController final
{
public:
    ResultDataController();  // 创建底层结果读取器
    ~ResultDataController(); // 释放已打开的结果文件

    bool inspect(const QString& filePath, _OUT Hdf5ResultSummary& summary) const; // 快速读取文件摘要
    bool open(const QString& filePath, _OUT std::vector<Hdf5ResultFrameInfo>& frames,
              _OUT Hdf5ResultRanges& ranges); // 打开文件并读取索引与范围
    bool readFrame(int frameIndex, _OUT Hdf5ResultFrame& frame); // 读取指定结果帧
    // 读取相邻或指定的两帧结果，供动画插值使用。
    bool readFramePair(int firstIndex, int secondIndex, _OUT Hdf5ResultFrame& firstFrame,
                       _OUT Hdf5ResultFrame& secondFrame);
    void close(); // 关闭结果文件并清空帧缓存

private:
    std::unique_ptr<Hdf5ResultReader> m_reader; // 底层 HDF5 读取器
    int m_cachedFrameIndex = -1;                // 第一帧缓存索引
    int m_cachedNextFrameIndex = -1;            // 第二帧缓存索引
    Hdf5ResultFrame m_cachedFrame;              // 第一帧缓存数据
    Hdf5ResultFrame m_cachedNextFrame;          // 第二帧缓存数据
};
