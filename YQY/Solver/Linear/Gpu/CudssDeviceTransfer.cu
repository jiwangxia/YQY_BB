#include "CudssDeviceTransfer.h"

#include <cstddef>

namespace SolverNameSpace
{
static __global__ void ReorderCscValuesKernel(const double* sourceValues, const int* sourceIndices, int valueCount,
                                              double* destinationValues)
{
    const int valueIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (valueIndex < valueCount)
        destinationValues[valueIndex] = sourceValues[sourceIndices[valueIndex]];
}

bool ReorderDeviceCscValuesToCsr(const double* sourceValues, const int* sourceIndices, int valueCount,
                                 _OUT double* destinationValues, cudaStream_t stream)
{
    if (!sourceValues || !sourceIndices || !destinationValues || valueCount <= 0)
        return false;
    constexpr int threadsPerBlock = 256;
    const int blocks = (valueCount + threadsPerBlock - 1) / threadsPerBlock;
    ReorderCscValuesKernel<<<blocks, threadsPerBlock, 0, stream>>>(sourceValues, sourceIndices, valueCount,
                                                                   destinationValues);
    return cudaGetLastError() == cudaSuccess;
}

static __global__ void SumCscValuesKernel(const double* first, const double* second, const double* third,
                                          const double* fourth, int valueCount, double* destination)
{
    const int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= valueCount)
        return;
    destination[index] = (first ? first[index] : 0.0) + (second ? second[index] : 0.0) + (third ? third[index] : 0.0) +
                         (fourth ? fourth[index] : 0.0);
}

static __global__ void AddCscValuesKernel(const DeviceCscValueAddition* additions, std::size_t additionCount,
                                          double* destination)
{
    const std::size_t index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
    if (index < additionCount)
        atomicAdd(destination + additions[index].index, additions[index].value);
}

DeviceCscSum::~DeviceCscSum()
{
    Reset();
}

DeviceCscSum::DeviceCscSum(const DeviceCscSum&)
{
}

DeviceCscSum& DeviceCscSum::operator=(const DeviceCscSum&)
{
    Reset();
    return *this;
}

bool DeviceCscSum::Combine(const double* first, const double* second, const double* third, int valueCount,
                           const double* fourth)
{
    m_valueCount = 0;
    if (valueCount <= 0 || (!first && !second && !third && !fourth))
        return false;
    if (valueCount > m_capacity)
    {
        Reset();
        if (cudaMalloc(reinterpret_cast<void**>(&m_values), sizeof(double) * static_cast<std::size_t>(valueCount)) !=
            cudaSuccess)
            return false;
        m_capacity = valueCount;
    }
    constexpr int threadsPerBlock = 256;
    const int blocks = (valueCount + threadsPerBlock - 1) / threadsPerBlock;
    SumCscValuesKernel<<<blocks, threadsPerBlock>>>(first, second, third, fourth, valueCount, m_values);
    if (cudaGetLastError() != cudaSuccess || cudaDeviceSynchronize() != cudaSuccess)
        return false;
    m_valueCount = valueCount;
    return true;
}

bool DeviceCscSum::AddHostValues(const std::vector<DeviceCscValueAddition>& additions)
{
    if (!m_values || m_valueCount <= 0)
        return false;
    if (additions.empty())
        return true;
    for (const DeviceCscValueAddition& addition : additions)
        if (addition.index < 0 || addition.index >= m_valueCount)
            return false;
    if (additions.size() > m_additionCapacity)
    {
        if (m_additions)
            cudaFree(m_additions);
        m_additions = nullptr;
        m_additionCapacity = 0;
        if (cudaMalloc(reinterpret_cast<void**>(&m_additions), additions.size() * sizeof(DeviceCscValueAddition)) !=
            cudaSuccess)
            return false;
        m_additionCapacity = additions.size();
    }
    if (cudaMemcpy(m_additions, additions.data(), additions.size() * sizeof(DeviceCscValueAddition),
                   cudaMemcpyHostToDevice) != cudaSuccess)
        return false;
    constexpr int threadsPerBlock = 256;
    const int blocks = static_cast<int>((additions.size() + threadsPerBlock - 1) / threadsPerBlock);
    AddCscValuesKernel<<<blocks, threadsPerBlock>>>(m_additions, additions.size(), m_values);
    return cudaGetLastError() == cudaSuccess && cudaDeviceSynchronize() == cudaSuccess;
}

const double* DeviceCscSum::Values() const
{
    return m_valueCount > 0 ? m_values : nullptr;
}

int DeviceCscSum::ValueCount() const
{
    return m_valueCount;
}

void DeviceCscSum::Reset()
{
    if (m_additions)
        cudaFree(m_additions);
    if (m_values)
        cudaFree(m_values);
    m_additions = nullptr;
    m_values = nullptr;
    m_capacity = 0;
    m_valueCount = 0;
    m_additionCapacity = 0;
}
}
