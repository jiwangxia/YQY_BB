#include "CudaCableBatchEvaluator.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <climits>
#include <cstring>

namespace SolverNameSpace
{
static __global__ void EvaluateCableKernel(const GpuCableElementInput* input, double accelerationDerivative,
                                           GpuCableElementResult* result, GpuCableElementState* state,
                                           const GpuCableAssemblyMap* assemblyMap, double* matrixValues,
                                           double* inertiaForce, int elementCount)
{
    const int elementIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (elementIndex >= elementCount)
        return;

    const GpuCableElementInput element = input[elementIndex];
    GpuCableElementResult output{};
    double chord[3]{};
    double axis[3]{};
    double lengthSquared = 0.0;
    for (int component = 0; component < 3; ++component)
    {
        chord[component] = element.initialPosition[component + 3] + element.displacement[component + 4] -
                           element.initialPosition[component] - element.displacement[component];
        lengthSquared += chord[component] * chord[component];
    }
    const double currentLength = sqrt(lengthSquared);
    if (currentLength <= 1.0e-12 || element.initialLength <= 1.0e-12 || element.young <= 0.0 || element.area <= 0.0 ||
        element.polarMoment < 0.0 || element.density < 0.0 || 1.0 + element.poisson <= 1.0e-12)
    {
        if (result)
            result[elementIndex] = output;
        if (state)
            state[elementIndex] = {};
        return;
    }

    for (int component = 0; component < 3; ++component)
        axis[component] = chord[component] / currentLength;

    const double axialStiffness = element.young * element.area / element.initialLength;
    const double shearModulus = element.young / (2.0 * (1.0 + element.poisson));
    const double torsionalStiffness = shearModulus * element.polarMoment / element.initialLength;
    const double trialAxialForce =
        element.initialStress * element.area + axialStiffness * (currentLength - element.initialLength);
    const double axialForce = trialAxialForce > 0.0 ? trialAxialForce : 0.0;
    const double twistForce = torsionalStiffness * (element.displacement[7] - element.displacement[3]);
    const double geometricStiffness = axialForce / currentLength;

    for (int localRow = 0; localRow < 6; ++localRow)
    {
        const int rowNode = localRow < 3 ? 0 : 1;
        const int rowComponent = localRow % 3;
        const int rowIndex = rowNode == 0 ? rowComponent : rowComponent + 4;
        const double rowSign = rowNode == 0 ? -1.0 : 1.0;
        for (int localColumn = 0; localColumn < 6; ++localColumn)
        {
            const int columnNode = localColumn < 3 ? 0 : 1;
            const int columnComponent = localColumn % 3;
            const int columnIndex = columnNode == 0 ? columnComponent : columnComponent + 4;
            const double columnSign = columnNode == 0 ? -1.0 : 1.0;
            const double material =
                axialForce > 0.0 ? axialStiffness * rowSign * columnSign * axis[rowComponent] * axis[columnComponent]
                                 : 0.0;
            const double geometric =
                geometricStiffness * rowSign * columnSign *
                ((rowComponent == columnComponent ? 1.0 : 0.0) - axis[rowComponent] * axis[columnComponent]);
            output.effectiveTangent[rowIndex * 8 + columnIndex] = material + geometric;
        }
    }

    output.effectiveTangent[3 * 8 + 3] = torsionalStiffness;
    output.effectiveTangent[3 * 8 + 7] = -torsionalStiffness;
    output.effectiveTangent[7 * 8 + 3] = -torsionalStiffness;
    output.effectiveTangent[7 * 8 + 7] = torsionalStiffness;

    const double translationalDensity = element.area * element.density;
    const double rotaryDensity = element.polarMoment * element.density;
    for (int component = 0; component < 4; ++component)
    {
        const double density = component < 3 ? translationalDensity : rotaryDensity;
        const int firstIndex = component;
        const int secondIndex = component + 4;
        const double diagonalMass = element.initialLength * density / 3.0;
        const double offDiagonalMass = element.initialLength * density / 6.0;
        output.effectiveTangent[firstIndex * 8 + firstIndex] += accelerationDerivative * diagonalMass;
        output.effectiveTangent[secondIndex * 8 + secondIndex] += accelerationDerivative * diagonalMass;
        output.effectiveTangent[firstIndex * 8 + secondIndex] += accelerationDerivative * offDiagonalMass;
        output.effectiveTangent[secondIndex * 8 + firstIndex] += accelerationDerivative * offDiagonalMass;
        output.inertiaForce[firstIndex] =
            diagonalMass * element.acceleration[firstIndex] + offDiagonalMass * element.acceleration[secondIndex];
        output.inertiaForce[secondIndex] =
            offDiagonalMass * element.acceleration[firstIndex] + diagonalMass * element.acceleration[secondIndex];
    }

    for (int component = 0; component < 3; ++component)
    {
        output.internalForce[component] = -axialForce * axis[component];
        output.internalForce[component + 4] = axialForce * axis[component];
    }
    output.internalForce[3] = -twistForce;
    output.internalForce[7] = twistForce;
    output.stress = axialForce / element.area;
    output.currentLength = currentLength;
    if (result)
        result[elementIndex] = output;
    if (state)
    {
        GpuCableElementState elementState{};
        for (int localDof = 0; localDof < 8; ++localDof)
            elementState.internalForce[localDof] = output.internalForce[localDof];
        elementState.stress = output.stress;
        elementState.currentLength = output.currentLength;
        state[elementIndex] = elementState;
    }
    if (matrixValues)
    {
        for (int valueIndex = 0; valueIndex < 64; ++valueIndex)
        {
            const int matrixValueIndex = assemblyMap[elementIndex].matrixValueIndices[valueIndex];
            if (matrixValueIndex >= 0)
                atomicAdd(&matrixValues[matrixValueIndex], output.effectiveTangent[valueIndex]);
        }
    }
    if (inertiaForce)
    {
        for (int localDof = 0; localDof < 8; ++localDof)
        {
            const int freeDof = assemblyMap[elementIndex].freeDofs[localDof];
            if (freeDof >= 0)
                atomicAdd(&inertiaForce[freeDof], output.inertiaForce[localDof]);
        }
    }
}

static bool CreateCudaCableTimingEvent(cudaEvent_t& event)
{
    return event || cudaEventCreate(&event) == cudaSuccess;
}

class CudaCableBatchEvaluator::Impl final
{
public:
    ~Impl()
    {
        Reset();
    }

    bool Evaluate(const std::vector<GpuCableElementInput>& input, double accelerationDerivative,
                  _OUT const GpuCableElementResult*& result, _OUT std::size_t& resultCount)
    {
        result = nullptr;
        resultCount = 0;
        m_lastTiming = {};
        if (input.empty() || input.size() > static_cast<std::size_t>(INT_MAX) || !EnsureStorage(input.size()))
            return false;

        std::copy(input.cbegin(), input.cend(), m_hostInput);
        const std::size_t byteCount = input.size() * sizeof(GpuCableElementInput);
        cudaEventRecord(m_uploadStart, m_stream);
        if (cudaMemcpyAsync(m_deviceInput, m_hostInput, byteCount, cudaMemcpyHostToDevice, m_stream) != cudaSuccess)
            return false;
        cudaEventRecord(m_uploadEnd, m_stream);
        cudaEventRecord(m_kernelStart, m_stream);
        constexpr int threadsPerBlock = 256;
        const int blocks = (static_cast<int>(input.size()) + threadsPerBlock - 1) / threadsPerBlock;
        EvaluateCableKernel<<<blocks, threadsPerBlock, 0, m_stream>>>(m_deviceInput, accelerationDerivative,
                                                                      m_deviceResult, nullptr, nullptr, nullptr,
                                                                      nullptr, static_cast<int>(input.size()));
        if (cudaGetLastError() != cudaSuccess)
            return false;
        cudaEventRecord(m_kernelEnd, m_stream);
        cudaEventRecord(m_downloadStart, m_stream);
        if (cudaMemcpyAsync(m_hostResult, m_deviceResult, input.size() * sizeof(GpuCableElementResult),
                            cudaMemcpyDeviceToHost, m_stream) != cudaSuccess)
            return false;
        cudaEventRecord(m_downloadEnd, m_stream);
        if (cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;

        float uploadMilliseconds = 0.0F;
        float kernelMilliseconds = 0.0F;
        float downloadMilliseconds = 0.0F;
        cudaEventElapsedTime(&uploadMilliseconds, m_uploadStart, m_uploadEnd);
        cudaEventElapsedTime(&kernelMilliseconds, m_kernelStart, m_kernelEnd);
        cudaEventElapsedTime(&downloadMilliseconds, m_downloadStart, m_downloadEnd);
        m_lastTiming.uploadMilliseconds = uploadMilliseconds;
        m_lastTiming.kernelMilliseconds = kernelMilliseconds;
        m_lastTiming.downloadMilliseconds = downloadMilliseconds;
        m_resultCount = input.size();
        result = m_hostResult;
        resultCount = m_resultCount;
        return true;
    }

    bool EvaluateAndAssemble(const std::vector<GpuCableElementInput>& input,
                             const std::vector<GpuCableAssemblyMap>& assemblyMap, double accelerationDerivative,
                             int matrixValueCount, int freeDofCount, bool assembleMatrix)
    {
        m_lastTiming = {};
        m_assemblyResultCount = 0;
        if (input.empty() || input.size() != assemblyMap.size() || input.size() > static_cast<std::size_t>(INT_MAX) ||
            matrixValueCount <= 0 || freeDofCount <= 0 || !EnsureStorage(input.size()) ||
            !EnsureAssemblyStorage(input.size(), matrixValueCount, freeDofCount))
            return false;

        std::copy(input.cbegin(), input.cend(), m_hostInput);
        const std::size_t inputBytes = input.size() * sizeof(GpuCableElementInput);
        const std::size_t matrixBytes = sizeof(double) * static_cast<std::size_t>(matrixValueCount);
        const std::size_t inertiaBytes = sizeof(double) * static_cast<std::size_t>(freeDofCount);
        bool mappingChanged = false;
        if (!PrepareAssemblyMap(assemblyMap, mappingChanged))
            return false;
        if (cudaEventRecord(m_uploadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_deviceInput, m_hostInput, inputBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
            (mappingChanged &&
             cudaMemcpyAsync(m_deviceAssemblyMap, m_hostAssemblyMap, input.size() * sizeof(GpuCableAssemblyMap),
                             cudaMemcpyHostToDevice, m_stream) != cudaSuccess) ||
            (assembleMatrix && cudaMemsetAsync(m_deviceAssemblyValues, 0, matrixBytes, m_stream) != cudaSuccess) ||
            cudaMemsetAsync(m_deviceAssemblyInertia, 0, inertiaBytes, m_stream) != cudaSuccess ||
            cudaEventRecord(m_uploadEnd, m_stream) != cudaSuccess)
            return false;

        constexpr int threadsPerBlock = 256;
        const int blocks = (static_cast<int>(input.size()) + threadsPerBlock - 1) / threadsPerBlock;
        if (cudaEventRecord(m_kernelStart, m_stream) != cudaSuccess)
            return false;
        EvaluateCableKernel<<<blocks, threadsPerBlock, 0, m_stream>>>(
            m_deviceInput, accelerationDerivative, nullptr, m_deviceState, m_deviceAssemblyMap,
            assembleMatrix ? m_deviceAssemblyValues : nullptr, m_deviceAssemblyInertia,
            static_cast<int>(input.size()));
        if (cudaGetLastError() != cudaSuccess || cudaEventRecord(m_kernelEnd, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_hostState, m_deviceState, input.size() * sizeof(GpuCableElementState),
                            cudaMemcpyDeviceToHost, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;

        float uploadMilliseconds = 0.0F;
        float kernelMilliseconds = 0.0F;
        float downloadMilliseconds = 0.0F;
        cudaEventElapsedTime(&uploadMilliseconds, m_uploadStart, m_uploadEnd);
        cudaEventElapsedTime(&kernelMilliseconds, m_kernelStart, m_kernelEnd);
        cudaEventElapsedTime(&downloadMilliseconds, m_downloadStart, m_downloadEnd);
        m_lastTiming.uploadMilliseconds = uploadMilliseconds;
        m_lastTiming.kernelMilliseconds = kernelMilliseconds;
        m_lastTiming.downloadMilliseconds = downloadMilliseconds;
        m_assemblyResultCount = input.size();
        return true;
    }

    bool CopyAssembledData(_OUT double* matrixValues, int matrixValueCount, _OUT double* inertiaForce, int freeDofCount)
    {
        if (!matrixValues || !inertiaForce || matrixValueCount != m_assemblyMatrixValueCount ||
            freeDofCount != m_assemblyFreeDofCount || !m_deviceAssemblyValues || !m_deviceAssemblyInertia)
            return false;

        const std::size_t matrixBytes = sizeof(double) * static_cast<std::size_t>(matrixValueCount);
        const std::size_t inertiaBytes = sizeof(double) * static_cast<std::size_t>(freeDofCount);
        if (cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(matrixValues, m_deviceAssemblyValues, matrixBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaMemcpyAsync(inertiaForce, m_deviceAssemblyInertia, inertiaBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;

        float downloadMilliseconds = 0.0F;
        if (cudaEventElapsedTime(&downloadMilliseconds, m_downloadStart, m_downloadEnd) != cudaSuccess)
            return false;
        m_lastTiming.assemblyDownloadMilliseconds = downloadMilliseconds;
        return true;
    }

    bool CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount)
    {
        if (!inertiaForce || freeDofCount != m_assemblyFreeDofCount || !m_deviceAssemblyInertia)
            return false;
        const std::size_t inertiaBytes = sizeof(double) * static_cast<std::size_t>(freeDofCount);
        if (cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(inertiaForce, m_deviceAssemblyInertia, inertiaBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;
        float downloadMilliseconds = 0.0F;
        if (cudaEventElapsedTime(&downloadMilliseconds, m_downloadStart, m_downloadEnd) != cudaSuccess)
            return false;
        m_lastTiming.assemblyDownloadMilliseconds = downloadMilliseconds;
        return true;
    }

    const GpuCableBatchTiming& LastTiming() const
    {
        return m_lastTiming;
    }

    const GpuCableElementResult* ResultData() const
    {
        return m_resultCount > 0 ? m_hostResult : nullptr;
    }

    std::size_t ResultCount() const
    {
        return m_resultCount;
    }

    const GpuCableElementState* StateData() const
    {
        return m_assemblyResultCount > 0 ? m_hostState : nullptr;
    }

    const double* DeviceMatrixValues() const
    {
        return m_deviceAssemblyValues;
    }

    int DeviceMatrixValueCount() const
    {
        return m_assemblyMatrixValueCount;
    }

private:
    bool EnsureStorage(std::size_t requestedCapacity)
    {
        if (cudaSetDevice(0) != cudaSuccess)
            return false;
        if (!m_stream && cudaStreamCreate(&m_stream) != cudaSuccess)
            return false;
        if (!CreateCudaCableTimingEvent(m_uploadStart) || !CreateCudaCableTimingEvent(m_uploadEnd) ||
            !CreateCudaCableTimingEvent(m_kernelStart) || !CreateCudaCableTimingEvent(m_kernelEnd) ||
            !CreateCudaCableTimingEvent(m_downloadStart) || !CreateCudaCableTimingEvent(m_downloadEnd))
            return false;
        if (requestedCapacity <= m_capacity && m_deviceInput && m_deviceResult && m_hostInput && m_hostResult)
            return true;

        ReleaseStorage();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceInput), requestedCapacity * sizeof(GpuCableElementInput)) !=
                cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceResult), requestedCapacity * sizeof(GpuCableElementResult)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostInput), requestedCapacity * sizeof(GpuCableElementInput)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostResult),
                           requestedCapacity * sizeof(GpuCableElementResult)) != cudaSuccess)
        {
            ReleaseStorage();
            return false;
        }
        m_capacity = requestedCapacity;
        return true;
    }

    bool EnsureAssemblyStorage(std::size_t requestedCapacity, int matrixValueCount, int freeDofCount)
    {
        const bool hasMatchingState = requestedCapacity <= m_assemblyCapacity && m_deviceState && m_hostState &&
                                      m_deviceAssemblyMap && m_hostAssemblyMap;
        const bool hasMatchingMatrix = matrixValueCount == m_assemblyMatrixValueCount &&
                                       freeDofCount == m_assemblyFreeDofCount && m_deviceAssemblyValues &&
                                       m_deviceAssemblyInertia;
        if (hasMatchingState && hasMatchingMatrix)
            return true;

        ReleaseAssemblyStorage();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceState), requestedCapacity * sizeof(GpuCableElementState)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostState), requestedCapacity * sizeof(GpuCableElementState)) !=
                cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceAssemblyMap),
                       requestedCapacity * sizeof(GpuCableAssemblyMap)) != cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostAssemblyMap),
                           requestedCapacity * sizeof(GpuCableAssemblyMap)) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceAssemblyValues),
                       static_cast<std::size_t>(matrixValueCount) * sizeof(double)) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceAssemblyInertia),
                       static_cast<std::size_t>(freeDofCount) * sizeof(double)) != cudaSuccess)
        {
            ReleaseAssemblyStorage();
            return false;
        }
        m_assemblyCapacity = requestedCapacity;
        m_assemblyMatrixValueCount = matrixValueCount;
        m_assemblyFreeDofCount = freeDofCount;
        return true;
    }

    bool PrepareAssemblyMap(const std::vector<GpuCableAssemblyMap>& assemblyMap, _OUT bool& changed)
    {
        changed = assemblyMap.size() != m_assemblyMapCount;
        if (!changed)
        {
            changed = !std::equal(assemblyMap.cbegin(), assemblyMap.cend(), m_hostAssemblyMap,
                                  [](const GpuCableAssemblyMap& lhs, const GpuCableAssemblyMap& rhs)
                                  {
                                      return std::equal(lhs.matrixValueIndices, lhs.matrixValueIndices + 64,
                                                        rhs.matrixValueIndices) &&
                                             std::equal(lhs.freeDofs, lhs.freeDofs + 8, rhs.freeDofs);
                                  });
        }
        if (!changed)
            return true;
        std::copy(assemblyMap.cbegin(), assemblyMap.cend(), m_hostAssemblyMap);
        m_assemblyMapCount = assemblyMap.size();
        return true;
    }

    void ReleaseAssemblyStorage()
    {
        if (m_deviceAssemblyInertia)
            cudaFree(m_deviceAssemblyInertia);
        if (m_deviceAssemblyValues)
            cudaFree(m_deviceAssemblyValues);
        if (m_hostAssemblyMap)
            cudaFreeHost(m_hostAssemblyMap);
        if (m_deviceAssemblyMap)
            cudaFree(m_deviceAssemblyMap);
        if (m_hostState)
            cudaFreeHost(m_hostState);
        if (m_deviceState)
            cudaFree(m_deviceState);
        m_deviceAssemblyInertia = nullptr;
        m_deviceAssemblyValues = nullptr;
        m_hostAssemblyMap = nullptr;
        m_deviceAssemblyMap = nullptr;
        m_hostState = nullptr;
        m_deviceState = nullptr;
        m_assemblyCapacity = 0;
        m_assemblyMatrixValueCount = 0;
        m_assemblyFreeDofCount = 0;
        m_assemblyResultCount = 0;
        m_assemblyMapCount = 0;
    }

    void ReleaseStorage()
    {
        ReleaseAssemblyStorage();
        if (m_hostResult)
            cudaFreeHost(m_hostResult);
        if (m_hostInput)
            cudaFreeHost(m_hostInput);
        if (m_deviceResult)
            cudaFree(m_deviceResult);
        if (m_deviceInput)
            cudaFree(m_deviceInput);
        m_hostResult = nullptr;
        m_hostInput = nullptr;
        m_deviceResult = nullptr;
        m_deviceInput = nullptr;
        m_capacity = 0;
        m_resultCount = 0;
    }

    void Reset()
    {
        ReleaseStorage();
        if (m_downloadEnd)
            cudaEventDestroy(m_downloadEnd);
        if (m_downloadStart)
            cudaEventDestroy(m_downloadStart);
        if (m_kernelEnd)
            cudaEventDestroy(m_kernelEnd);
        if (m_kernelStart)
            cudaEventDestroy(m_kernelStart);
        if (m_uploadEnd)
            cudaEventDestroy(m_uploadEnd);
        if (m_uploadStart)
            cudaEventDestroy(m_uploadStart);
        if (m_stream)
            cudaStreamDestroy(m_stream);
        m_uploadStart = nullptr;
        m_uploadEnd = nullptr;
        m_kernelStart = nullptr;
        m_kernelEnd = nullptr;
        m_downloadStart = nullptr;
        m_downloadEnd = nullptr;
        m_stream = nullptr;
    }

    cudaStream_t m_stream = nullptr;
    cudaEvent_t m_uploadStart = nullptr;
    cudaEvent_t m_uploadEnd = nullptr;
    cudaEvent_t m_kernelStart = nullptr;
    cudaEvent_t m_kernelEnd = nullptr;
    cudaEvent_t m_downloadStart = nullptr;
    cudaEvent_t m_downloadEnd = nullptr;
    GpuCableElementInput* m_deviceInput = nullptr;
    GpuCableElementResult* m_deviceResult = nullptr;
    GpuCableElementInput* m_hostInput = nullptr;
    GpuCableElementResult* m_hostResult = nullptr;
    GpuCableElementState* m_deviceState = nullptr;
    GpuCableElementState* m_hostState = nullptr;
    GpuCableAssemblyMap* m_deviceAssemblyMap = nullptr;
    GpuCableAssemblyMap* m_hostAssemblyMap = nullptr;
    double* m_deviceAssemblyValues = nullptr;
    double* m_deviceAssemblyInertia = nullptr;
    std::size_t m_capacity = 0;
    std::size_t m_resultCount = 0;
    std::size_t m_assemblyCapacity = 0;
    std::size_t m_assemblyResultCount = 0;
    std::size_t m_assemblyMapCount = 0;
    int m_assemblyMatrixValueCount = 0;
    int m_assemblyFreeDofCount = 0;
    GpuCableBatchTiming m_lastTiming;
};

CudaCableBatchEvaluator::CudaCableBatchEvaluator()
    : m_impl(std::make_unique<Impl>())
{
}

CudaCableBatchEvaluator::~CudaCableBatchEvaluator() = default;

CudaCableBatchEvaluator::CudaCableBatchEvaluator(const CudaCableBatchEvaluator& other)
    : m_impl(std::make_unique<Impl>())
{
    (void)other;
}

CudaCableBatchEvaluator& CudaCableBatchEvaluator::operator=(const CudaCableBatchEvaluator& other)
{
    if (this != &other)
    {
        (void)other;
        m_impl = std::make_unique<Impl>();
    }
    return *this;
}

bool CudaCableBatchEvaluator::Evaluate(const std::vector<GpuCableElementInput>& input, double accelerationDerivative)
{
    const GpuCableElementResult* result = nullptr;
    std::size_t resultCount = 0;
    return m_impl->Evaluate(input, accelerationDerivative, result, resultCount);
}

bool CudaCableBatchEvaluator::Evaluate(const std::vector<GpuCableElementInput>& input, double accelerationDerivative,
                                       _OUT std::vector<GpuCableElementResult>& result)
{
    const GpuCableElementResult* resultData = nullptr;
    std::size_t resultCount = 0;
    if (!m_impl->Evaluate(input, accelerationDerivative, resultData, resultCount))
        return false;
    result.assign(resultData, resultData + resultCount);
    return true;
}

bool CudaCableBatchEvaluator::EvaluateAndAssemble(const std::vector<GpuCableElementInput>& input,
                                                  const std::vector<GpuCableAssemblyMap>& assemblyMap,
                                                  double accelerationDerivative, int matrixValueCount, int freeDofCount)
{
    return m_impl->EvaluateAndAssemble(input, assemblyMap, accelerationDerivative, matrixValueCount, freeDofCount,
                                       true);
}

bool CudaCableBatchEvaluator::EvaluateResidualAndAssemble(
    const std::vector<GpuCableElementInput>& input, const std::vector<GpuCableAssemblyMap>& assemblyMap,
    int matrixValueCount, int freeDofCount)
{
    return m_impl->EvaluateAndAssemble(input, assemblyMap, 0.0, matrixValueCount, freeDofCount, false);
}

bool CudaCableBatchEvaluator::CopyAssembledData(_OUT double* matrixValues, int matrixValueCount,
                                                _OUT double* inertiaForce, int freeDofCount)
{
    return m_impl->CopyAssembledData(matrixValues, matrixValueCount, inertiaForce, freeDofCount);
}

bool CudaCableBatchEvaluator::CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount)
{
    return m_impl->CopyAssembledInertia(inertiaForce, freeDofCount);
}

const GpuCableElementResult* CudaCableBatchEvaluator::ResultData() const
{
    return m_impl->ResultData();
}

std::size_t CudaCableBatchEvaluator::ResultCount() const
{
    return m_impl->ResultCount();
}

const GpuCableElementState* CudaCableBatchEvaluator::StateData() const
{
    return m_impl->StateData();
}

const double* CudaCableBatchEvaluator::DeviceMatrixValues() const
{
    return m_impl->DeviceMatrixValues();
}

int CudaCableBatchEvaluator::DeviceMatrixValueCount() const
{
    return m_impl->DeviceMatrixValueCount();
}

const GpuCableBatchTiming& CudaCableBatchEvaluator::LastTiming() const
{
    return m_impl->LastTiming();
}

template <bool AssembleMatrix>
static __global__ void EvaluateSharedCableKernel(const GpuSharedCableElementInput* input,
                                                 const GpuSharedCableAssemblyMap* assemblyMap,
                                                 GpuSharedCableElementState* state, double* matrixValues,
                                                 double* inertiaForce, double accelerationDerivative, int elementCount)
{
    const int elementIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (elementIndex >= elementCount)
        return;
    const GpuSharedCableElementInput element = input[elementIndex];
    GpuSharedCableElementState output{};
    const int secondOffset = element.firstNodeDofs;
    const int localDofs = secondOffset + element.secondNodeDofs;
    double chord[3]{};
    double axis[3]{};
    double lengthSquared = 0.0;
    for (int component = 0; component < 3; ++component)
    {
        chord[component] = element.initialPosition[component + 3] + element.displacement[component + 3] -
                           element.initialPosition[component] - element.displacement[component];
        lengthSquared += chord[component] * chord[component];
    }
    const double currentLength = sqrt(lengthSquared);
    if (currentLength <= 1.0e-12 || element.initialLength <= 1.0e-12 || element.young <= 0.0 || element.area <= 0.0 ||
        element.density < 0.0 || element.polarMoment < 0.0 || 1.0 + element.poisson <= 1.0e-12 || localDofs > 12)
    {
        state[elementIndex] = output;
        return;
    }
    for (int component = 0; component < 3; ++component)
        axis[component] = chord[component] / currentLength;

    const double axialStiffness = element.young * element.area / element.initialLength;
    const double shearModulus = element.young / (2.0 * (1.0 + element.poisson));
    const double torsionalStiffness = shearModulus * element.polarMoment / element.initialLength;
    const double trialForce =
        element.initialStress * element.area + axialStiffness * (currentLength - element.initialLength);
    const double axialForce = fmax(0.0, trialForce);
    const double twistForce = torsionalStiffness * (element.twist[1] - element.twist[0]);
    output.stress = axialForce / element.area;
    output.currentLength = currentLength;

    double axialB[12]{};
    double torsionB[12]{};
    double rotationMap[12]{};
    for (int component = 0; component < 3; ++component)
    {
        axialB[component] = -axis[component];
        axialB[secondOffset + component] = axis[component];
    }
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const int offset = nodeIndex == 0 ? 0 : secondOffset;
        const int nodeDofs = nodeIndex == 0 ? element.firstNodeDofs : element.secondNodeDofs;
        const double sign = nodeIndex == 0 ? -1.0 : 1.0;
        for (int component = 0; component < nodeDofs - 3; ++component)
        {
            const int index = offset + 3 + component;
            rotationMap[index] = nodeDofs == 4 ? 1.0 : element.torsionAxis[component];
            torsionB[index] = sign * rotationMap[index];
        }
    }
    for (int localDof = 0; localDof < localDofs; ++localDof)
        output.internalForce[localDof] = axialForce * axialB[localDof] + twistForce * torsionB[localDof];

    const double translationalMass = element.area * element.density * element.initialLength / 6.0;
    const double rotationalMass = element.polarMoment * element.density * element.initialLength / 6.0;
    double nodalTwistAcceleration[2]{};
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const int offset = nodeIndex == 0 ? 0 : secondOffset;
        const int nodeDofs = nodeIndex == 0 ? element.firstNodeDofs : element.secondNodeDofs;
        for (int component = 0; component < nodeDofs - 3; ++component)
            nodalTwistAcceleration[nodeIndex] +=
                rotationMap[offset + 3 + component] * element.acceleration[offset + 3 + component];
    }
    for (int nodeA = 0; nodeA < 2; ++nodeA)
    {
        const int offsetA = nodeA == 0 ? 0 : secondOffset;
        const int nodeDofsA = nodeA == 0 ? element.firstNodeDofs : element.secondNodeDofs;
        for (int nodeB = 0; nodeB < 2; ++nodeB)
        {
            const int offsetB = nodeB == 0 ? 0 : secondOffset;
            const int nodeDofsB = nodeB == 0 ? element.firstNodeDofs : element.secondNodeDofs;
            const double massFactor = nodeA == nodeB ? 2.0 : 1.0;
            for (int component = 0; component < 3; ++component)
            {
                const int row = offsetA + component;
                const int column = offsetB + component;
                const double mass = translationalMass * massFactor;
                const int freeDof = assemblyMap[elementIndex].freeDofs[row];
                if (freeDof >= 0)
                    atomicAdd(inertiaForce + freeDof, mass * element.acceleration[column]);
                if constexpr (AssembleMatrix)
                {
                    const int matrixIndex = assemblyMap[elementIndex].matrixValueIndices[row * 12 + column];
                    if (matrixIndex >= 0)
                        atomicAdd(matrixValues + matrixIndex, accelerationDerivative * mass);
                }
            }
            for (int componentA = 0; componentA < nodeDofsA - 3; ++componentA)
            {
                const int row = offsetA + 3 + componentA;
                const double mass = rotationalMass * massFactor * rotationMap[row];
                const int freeDof = assemblyMap[elementIndex].freeDofs[row];
                if (freeDof >= 0)
                    atomicAdd(inertiaForce + freeDof, mass * nodalTwistAcceleration[nodeB]);
                if constexpr (AssembleMatrix)
                {
                    for (int componentB = 0; componentB < nodeDofsB - 3; ++componentB)
                    {
                        const int column = offsetB + 3 + componentB;
                        const int matrixIndex = assemblyMap[elementIndex].matrixValueIndices[row * 12 + column];
                        if (matrixIndex >= 0)
                            atomicAdd(matrixValues + matrixIndex, accelerationDerivative * mass * rotationMap[column]);
                    }
                }
            }
        }
    }
    if constexpr (AssembleMatrix)
    {
        const double geometricStiffness = axialForce / currentLength;
        for (int row = 0; row < localDofs; ++row)
            for (int column = 0; column < localDofs; ++column)
            {
                const int matrixIndex = assemblyMap[elementIndex].matrixValueIndices[row * 12 + column];
                if (matrixIndex < 0)
                    continue;
                double value = (trialForce > 0.0 ? axialStiffness * axialB[row] * axialB[column] : 0.0) +
                               torsionalStiffness * torsionB[row] * torsionB[column];
                const int rowNode = row >= secondOffset ? 1 : 0;
                const int columnNode = column >= secondOffset ? 1 : 0;
                const int rowComponent = row - (rowNode == 0 ? 0 : secondOffset);
                const int columnComponent = column - (columnNode == 0 ? 0 : secondOffset);
                if (rowComponent < 3 && columnComponent < 3)
                {
                    const double sign = rowNode == columnNode ? 1.0 : -1.0;
                    value +=
                        geometricStiffness * sign *
                        ((rowComponent == columnComponent ? 1.0 : 0.0) - axis[rowComponent] * axis[columnComponent]);
                }
                atomicAdd(matrixValues + matrixIndex, value);
            }
    }
    state[elementIndex] = output;
}

CudaSharedCableBatchEvaluator::~CudaSharedCableBatchEvaluator()
{
    Reset();
}

CudaSharedCableBatchEvaluator::CudaSharedCableBatchEvaluator(const CudaSharedCableBatchEvaluator&)
{
}

CudaSharedCableBatchEvaluator& CudaSharedCableBatchEvaluator::operator=(const CudaSharedCableBatchEvaluator&)
{
    Reset();
    return *this;
}

bool CudaSharedCableBatchEvaluator::EvaluateAndAssemble(const std::vector<GpuSharedCableElementInput>& input,
                                                        const std::vector<GpuSharedCableAssemblyMap>& assemblyMap,
                                                        double accelerationDerivative, int matrixValueCount,
                                                        int freeDofCount, bool assembleMatrix)
{
    m_hasMatrix = false;
    if (input.empty() || input.size() != assemblyMap.size() || input.size() > static_cast<std::size_t>(INT_MAX) ||
        matrixValueCount <= 0 || freeDofCount <= 0)
        return false;
    if (input.size() > m_capacity || matrixValueCount != m_matrixValueCount || freeDofCount != m_freeDofCount)
    {
        Reset();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceInput), input.size() * sizeof(GpuSharedCableElementInput)) !=
                cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceMap), input.size() * sizeof(GpuSharedCableAssemblyMap)) !=
                cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceState), input.size() * sizeof(GpuSharedCableElementState)) !=
                cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceValues),
                       static_cast<std::size_t>(matrixValueCount) * sizeof(double)) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceInertia),
                       static_cast<std::size_t>(freeDofCount) * sizeof(double)) != cudaSuccess)
        {
            Reset();
            return false;
        }
        m_capacity = input.size();
        m_matrixValueCount = matrixValueCount;
        m_freeDofCount = freeDofCount;
    }
    if (cudaMemcpy(m_deviceInput, input.data(), input.size() * sizeof(GpuSharedCableElementInput),
                   cudaMemcpyHostToDevice) != cudaSuccess)
        return false;
    const bool mappingChanged = m_cachedMap.size() != assemblyMap.size() ||
                                std::memcmp(m_cachedMap.data(), assemblyMap.data(),
                                            assemblyMap.size() * sizeof(GpuSharedCableAssemblyMap)) != 0;
    if (mappingChanged)
    {
        if (cudaMemcpy(m_deviceMap, assemblyMap.data(), assemblyMap.size() * sizeof(GpuSharedCableAssemblyMap),
                       cudaMemcpyHostToDevice) != cudaSuccess)
            return false;
        m_cachedMap = assemblyMap;
    }
    if (cudaMemset(m_deviceInertia, 0, static_cast<std::size_t>(freeDofCount) * sizeof(double)) != cudaSuccess ||
        (assembleMatrix &&
         cudaMemset(m_deviceValues, 0, static_cast<std::size_t>(matrixValueCount) * sizeof(double)) != cudaSuccess))
        return false;
    constexpr int threadsPerBlock = 128;
    const int blocks = (static_cast<int>(input.size()) + threadsPerBlock - 1) / threadsPerBlock;
    if (assembleMatrix)
        EvaluateSharedCableKernel<true>
            <<<blocks, threadsPerBlock>>>(m_deviceInput, m_deviceMap, m_deviceState, m_deviceValues, m_deviceInertia,
                                          accelerationDerivative, static_cast<int>(input.size()));
    else
        EvaluateSharedCableKernel<false><<<blocks, threadsPerBlock>>>(
            m_deviceInput, m_deviceMap, m_deviceState, nullptr, m_deviceInertia, 0.0, static_cast<int>(input.size()));
    if (cudaGetLastError() != cudaSuccess)
        return false;
    m_hostState.resize(input.size());
    if (cudaMemcpy(m_hostState.data(), m_deviceState, input.size() * sizeof(GpuSharedCableElementState),
                   cudaMemcpyDeviceToHost) != cudaSuccess)
        return false;
    m_hasMatrix = assembleMatrix;
    return true;
}

bool CudaSharedCableBatchEvaluator::CopyAssembledData(_OUT double* matrixValues, int matrixValueCount,
                                                      _OUT double* inertiaForce, int freeDofCount)
{
    return m_hasMatrix && matrixValues && inertiaForce && matrixValueCount == m_matrixValueCount &&
           freeDofCount == m_freeDofCount &&
           cudaMemcpy(matrixValues, m_deviceValues, static_cast<std::size_t>(matrixValueCount) * sizeof(double),
                      cudaMemcpyDeviceToHost) == cudaSuccess &&
           CopyAssembledInertia(inertiaForce, freeDofCount);
}

bool CudaSharedCableBatchEvaluator::CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount)
{
    return inertiaForce && freeDofCount == m_freeDofCount &&
           cudaMemcpy(inertiaForce, m_deviceInertia, static_cast<std::size_t>(freeDofCount) * sizeof(double),
                      cudaMemcpyDeviceToHost) == cudaSuccess;
}

const GpuSharedCableElementState* CudaSharedCableBatchEvaluator::StateData() const
{
    return m_hostState.empty() ? nullptr : m_hostState.data();
}

const double* CudaSharedCableBatchEvaluator::DeviceMatrixValues() const
{
    return m_hasMatrix ? m_deviceValues : nullptr;
}

int CudaSharedCableBatchEvaluator::DeviceMatrixValueCount() const
{
    return m_hasMatrix ? m_matrixValueCount : 0;
}

void CudaSharedCableBatchEvaluator::Reset()
{
    if (m_deviceInertia)
        cudaFree(m_deviceInertia);
    if (m_deviceValues)
        cudaFree(m_deviceValues);
    if (m_deviceState)
        cudaFree(m_deviceState);
    if (m_deviceMap)
        cudaFree(m_deviceMap);
    if (m_deviceInput)
        cudaFree(m_deviceInput);
    m_deviceInput = nullptr;
    m_deviceMap = nullptr;
    m_deviceState = nullptr;
    m_deviceValues = nullptr;
    m_deviceInertia = nullptr;
    m_cachedMap.clear();
    m_hostState.clear();
    m_capacity = 0;
    m_matrixValueCount = 0;
    m_freeDofCount = 0;
    m_hasMatrix = false;
}
}
