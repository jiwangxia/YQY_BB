#pragma once

#include "Base/EmptyOUT.h"

#include <cuda_runtime_api.h>

#include <cstddef>
#include <vector>

namespace SolverNameSpace
{
// 根据预先建立的索引，把 Eigen 的列压缩数值重排为 cuDSS 所需的行压缩数值。
bool ReorderDeviceCscValuesToCsr(const double* sourceValues, const int* sourceIndices, int valueCount,
                                 _OUT double* destinationValues, cudaStream_t stream);

// 将不同 GPU 单元批次的整体 CSC 数值直接合并到同一设备缓冲区。
struct DeviceCscValueAddition
{
    int index = -1;
    double value = 0.0;
};

class DeviceCscSum final
{
public:
    DeviceCscSum() = default;
    ~DeviceCscSum();
    DeviceCscSum(const DeviceCscSum&);
    DeviceCscSum& operator=(const DeviceCscSum&);

    bool Combine(const double* first, const double* second, const double* third, int valueCount,
                 const double* fourth = nullptr);
    bool AddHostValues(const std::vector<DeviceCscValueAddition>& additions);
    const double* Values() const;
    int ValueCount() const;

private:
    void Reset();
    double* m_values = nullptr;
    DeviceCscValueAddition* m_additions = nullptr;
    int m_capacity = 0;
    int m_valueCount = 0;
    std::size_t m_additionCapacity = 0;
};
}
