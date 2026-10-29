#include "CudaTrussBatchEvaluator.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstddef>

namespace SolverNameSpace
{
static __global__ void EvaluateTrussKernel(const GpuTrussElementInput* input, double accelerationDerivative,
                                           GpuTrussElementResult* result, GpuTrussElementState* state,
                                           const GpuTrussAssemblyMap* assemblyMap, double* matrixValues,
                                           double* inertiaForce, int elementCount)
{
    const int elementIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (elementIndex >= elementCount)
        return;

    const GpuTrussElementInput element = input[elementIndex];
    GpuTrussElementResult output{};
    double chord[3]{};
    double direction[3]{};
    double lengthSquared = 0.0;
    for (int component = 0; component < 3; ++component)
    {
        chord[component] = element.initialPosition[component + 3] + element.displacement[component + 3] -
                           element.initialPosition[component] - element.displacement[component];
        lengthSquared += chord[component] * chord[component];
    }
    const double currentLength = sqrt(lengthSquared);
    if (currentLength <= 1.0e-12 || element.initialLength <= 1.0e-12 || element.area <= 0.0 || element.young <= 0.0)
    {
        if (result)
            result[elementIndex] = output;
        if (state)
            state[elementIndex] = {};
        return;
    }

    for (int component = 0; component < 3; ++component)
        direction[component] = chord[component] / currentLength;

    const double strain = element.useEngineeringStrain != 0
                              ? (currentLength - element.initialLength) / element.initialLength
                              : log(currentLength / element.initialLength);
    const double stress = element.young * strain + element.initialStress;
    const double currentArea = element.useEngineeringStrain != 0
                                   ? element.area
                                   : element.area * element.initialLength / currentLength;
    const double axialForce = stress * currentArea;
    const double materialStiffness = element.useEngineeringStrain != 0
                                         ? element.young * element.area / element.initialLength
                                         : currentArea * (element.young - stress) / currentLength;
    const double geometricStiffness = axialForce / currentLength;
    const double consistentMass = element.initialLength * element.area * element.density;

    for (int localRow = 0; localRow < 6; ++localRow)
    {
        const int rowNode = localRow / 3;
        const int rowComponent = localRow % 3;
        const double rowSign = rowNode == 0 ? -1.0 : 1.0;
        for (int localColumn = 0; localColumn < 6; ++localColumn)
        {
            const int columnNode = localColumn / 3;
            const int columnComponent = localColumn % 3;
            const double columnSign = columnNode == 0 ? -1.0 : 1.0;
            const double material =
                materialStiffness * rowSign * columnSign * direction[rowComponent] * direction[columnComponent];
            const double geometric =
                geometricStiffness * rowSign * columnSign *
                ((rowComponent == columnComponent ? 1.0 : 0.0) - direction[rowComponent] * direction[columnComponent]);
            const double mass = rowComponent == columnComponent
                                    ? consistentMass * (rowNode == columnNode ? 1.0 / 3.0 : 1.0 / 6.0)
                                    : 0.0;
            output.effectiveTangent[localRow * 6 + localColumn] = material + geometric + accelerationDerivative * mass;
            output.inertiaForce[localRow] += mass * element.acceleration[localColumn];
        }
    }

    for (int component = 0; component < 3; ++component)
    {
        output.internalForce[component] = -axialForce * direction[component];
        output.internalForce[component + 3] = axialForce * direction[component];
    }
    output.stress = stress;
    output.currentLength = currentLength;
    if (result)
        result[elementIndex] = output;
    if (state)
    {
        GpuTrussElementState elementState{};
        for (int localDof = 0; localDof < 6; ++localDof)
            elementState.internalForce[localDof] = output.internalForce[localDof];
        elementState.stress = output.stress;
        elementState.currentLength = output.currentLength;
        state[elementIndex] = elementState;
    }
    if (matrixValues)
    {
        for (int valueIndex = 0; valueIndex < 36; ++valueIndex)
        {
            const int matrixValueIndex = assemblyMap[elementIndex].matrixValueIndices[valueIndex];
            if (matrixValueIndex >= 0)
                atomicAdd(&matrixValues[matrixValueIndex], output.effectiveTangent[valueIndex]);
        }
    }
    if (inertiaForce)
    {
        for (int localDof = 0; localDof < 6; ++localDof)
        {
            const int freeDof = assemblyMap[elementIndex].freeDofs[localDof];
            if (freeDof >= 0)
                atomicAdd(&inertiaForce[freeDof], output.inertiaForce[localDof]);
        }
    }
}

static bool CreateCudaTrussTimingEvent(_OUT cudaEvent_t& event)
{
    return event || cudaEventCreate(&event) == cudaSuccess;
}

static bool ReadCudaTrussElapsedMilliseconds(cudaEvent_t start, cudaEvent_t end, _OUT double& milliseconds)
{
    float elapsed = 0.0F;
    if (cudaEventElapsedTime(&elapsed, start, end) != cudaSuccess)
        return false;
    milliseconds = static_cast<double>(elapsed);
    return true;
}

class CudaTrussBatchEvaluator::Impl
{
public:
    ~Impl()
    {
        Reset();
    }

    bool Evaluate(const std::vector<GpuTrussElementInput>& input, double accelerationDerivative,
                  _OUT const GpuTrussElementResult*& result, _OUT std::size_t& resultCount)
    {
        m_lastTiming = {};
        m_resultCount = 0;
        result = nullptr;
        resultCount = 0;
        if (input.empty() || !EnsureStorage(input.size()))
            return false;

        std::copy(input.cbegin(), input.cend(), m_hostInput);
        const std::size_t inputBytes = input.size() * sizeof(GpuTrussElementInput);
        const std::size_t resultBytes = input.size() * sizeof(GpuTrussElementResult);
        if (cudaEventRecord(m_uploadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_deviceInput, m_hostInput, inputBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
            cudaEventRecord(m_uploadEnd, m_stream) != cudaSuccess)
            return false;

        constexpr int threadsPerBlock = 128;
        const int blocks = static_cast<int>((input.size() + threadsPerBlock - 1) / threadsPerBlock);
        if (cudaEventRecord(m_kernelStart, m_stream) != cudaSuccess)
            return false;
        EvaluateTrussKernel<<<blocks, threadsPerBlock, 0, m_stream>>>(
            m_deviceInput, accelerationDerivative, m_deviceResult, nullptr, nullptr, nullptr, nullptr,
            static_cast<int>(input.size()));
        if (cudaGetLastError() != cudaSuccess || cudaEventRecord(m_kernelEnd, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_hostResult, m_deviceResult, resultBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;

        if (!ReadCudaTrussElapsedMilliseconds(m_uploadStart, m_uploadEnd, m_lastTiming.uploadMilliseconds) ||
            !ReadCudaTrussElapsedMilliseconds(m_kernelStart, m_kernelEnd, m_lastTiming.kernelMilliseconds) ||
            !ReadCudaTrussElapsedMilliseconds(m_downloadStart, m_downloadEnd, m_lastTiming.downloadMilliseconds))
            return false;
        result = m_hostResult;
        resultCount = input.size();
        m_resultCount = resultCount;
        return true;
    }

    bool EvaluateAndAssemble(const std::vector<GpuTrussElementInput>& input,
                             const std::vector<GpuTrussAssemblyMap>& assemblyMap, double accelerationDerivative,
                             int matrixValueCount, int freeDofCount, bool assembleMatrix)
    {
        m_lastTiming = {};
        m_assemblyResultCount = 0;
        if (input.empty() || input.size() != assemblyMap.size() || input.size() > static_cast<std::size_t>(INT_MAX) ||
            matrixValueCount <= 0 || freeDofCount <= 0 || !EnsureStorage(input.size()) ||
            !EnsureAssemblyStorage(input.size(), matrixValueCount, freeDofCount))
            return false;

        std::copy(input.cbegin(), input.cend(), m_hostInput);
        std::copy(assemblyMap.cbegin(), assemblyMap.cend(), m_hostAssemblyMap);
        const std::size_t inputBytes = input.size() * sizeof(GpuTrussElementInput);
        const std::size_t mapBytes = input.size() * sizeof(GpuTrussAssemblyMap);
        const std::size_t matrixBytes = static_cast<std::size_t>(matrixValueCount) * sizeof(double);
        const std::size_t inertiaBytes = static_cast<std::size_t>(freeDofCount) * sizeof(double);
        if (cudaEventRecord(m_uploadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_deviceInput, m_hostInput, inputBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_deviceAssemblyMap, m_hostAssemblyMap, mapBytes, cudaMemcpyHostToDevice, m_stream) !=
                cudaSuccess ||
            (assembleMatrix && cudaMemsetAsync(m_deviceAssemblyValues, 0, matrixBytes, m_stream) != cudaSuccess) ||
            cudaMemsetAsync(m_deviceAssemblyInertia, 0, inertiaBytes, m_stream) != cudaSuccess ||
            cudaEventRecord(m_uploadEnd, m_stream) != cudaSuccess)
            return false;

        constexpr int threadsPerBlock = 128;
        const int blocks = static_cast<int>((input.size() + threadsPerBlock - 1) / threadsPerBlock);
        if (cudaEventRecord(m_kernelStart, m_stream) != cudaSuccess)
            return false;
        EvaluateTrussKernel<<<blocks, threadsPerBlock, 0, m_stream>>>(
            m_deviceInput, accelerationDerivative, nullptr, m_deviceState, m_deviceAssemblyMap,
            assembleMatrix ? m_deviceAssemblyValues : nullptr, m_deviceAssemblyInertia,
            static_cast<int>(input.size()));
        if (cudaGetLastError() != cudaSuccess || cudaEventRecord(m_kernelEnd, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_hostState, m_deviceState, input.size() * sizeof(GpuTrussElementState),
                            cudaMemcpyDeviceToHost, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;

        if (!ReadCudaTrussElapsedMilliseconds(m_uploadStart, m_uploadEnd, m_lastTiming.uploadMilliseconds) ||
            !ReadCudaTrussElapsedMilliseconds(m_kernelStart, m_kernelEnd, m_lastTiming.kernelMilliseconds) ||
            !ReadCudaTrussElapsedMilliseconds(m_downloadStart, m_downloadEnd, m_lastTiming.downloadMilliseconds))
            return false;
        m_assemblyResultCount = input.size();
        return true;
    }

    bool CopyAssembledData(_OUT double* matrixValues, int matrixValueCount, _OUT double* inertiaForce,
                           int freeDofCount)
    {
        if (!matrixValues || !inertiaForce || matrixValueCount != m_assemblyMatrixValueCount ||
            freeDofCount != m_assemblyFreeDofCount || !m_deviceAssemblyValues || !m_deviceAssemblyInertia)
            return false;
        const std::size_t matrixBytes = static_cast<std::size_t>(matrixValueCount) * sizeof(double);
        const std::size_t inertiaBytes = static_cast<std::size_t>(freeDofCount) * sizeof(double);
        if (cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(matrixValues, m_deviceAssemblyValues, matrixBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaMemcpyAsync(inertiaForce, m_deviceAssemblyInertia, inertiaBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;
        return ReadCudaTrussElapsedMilliseconds(m_downloadStart, m_downloadEnd,
                                                m_lastTiming.assemblyDownloadMilliseconds);
    }

    bool CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount)
    {
        if (!inertiaForce || freeDofCount != m_assemblyFreeDofCount || !m_deviceAssemblyInertia)
            return false;
        const std::size_t inertiaBytes = static_cast<std::size_t>(freeDofCount) * sizeof(double);
        if (cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(inertiaForce, m_deviceAssemblyInertia, inertiaBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;
        return ReadCudaTrussElapsedMilliseconds(m_downloadStart, m_downloadEnd,
                                                m_lastTiming.assemblyDownloadMilliseconds);
    }

    const GpuTrussBatchTiming& LastTiming() const
    {
        return m_lastTiming;
    }

    const GpuTrussElementResult* ResultData() const
    {
        return m_resultCount > 0 ? m_hostResult : nullptr;
    }

    std::size_t ResultCount() const
    {
        return m_resultCount;
    }

    const GpuTrussElementState* StateData() const
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
        if (!CreateCudaTrussTimingEvent(m_uploadStart) || !CreateCudaTrussTimingEvent(m_uploadEnd) ||
            !CreateCudaTrussTimingEvent(m_kernelStart) || !CreateCudaTrussTimingEvent(m_kernelEnd) ||
            !CreateCudaTrussTimingEvent(m_downloadStart) || !CreateCudaTrussTimingEvent(m_downloadEnd))
            return false;
        if (requestedCapacity <= m_capacity && m_deviceInput && m_deviceResult && m_hostInput && m_hostResult)
            return true;

        ReleaseStorage();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceInput), requestedCapacity * sizeof(GpuTrussElementInput)) !=
                cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceResult), requestedCapacity * sizeof(GpuTrussElementResult)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostInput), requestedCapacity * sizeof(GpuTrussElementInput)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostResult),
                           requestedCapacity * sizeof(GpuTrussElementResult)) != cudaSuccess)
        {
            ReleaseStorage();
            return false;
        }
        m_capacity = requestedCapacity;
        return true;
    }

    bool EnsureAssemblyStorage(std::size_t requestedCapacity, int matrixValueCount, int freeDofCount)
    {
        const bool capacityMatches = requestedCapacity <= m_assemblyCapacity && m_deviceState && m_hostState &&
                                     m_deviceAssemblyMap && m_hostAssemblyMap;
        const bool matrixMatches = matrixValueCount == m_assemblyMatrixValueCount &&
                                   freeDofCount == m_assemblyFreeDofCount && m_deviceAssemblyValues &&
                                   m_deviceAssemblyInertia;
        if (capacityMatches && matrixMatches)
            return true;

        ReleaseAssemblyStorage();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceState), requestedCapacity * sizeof(GpuTrussElementState)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostState), requestedCapacity * sizeof(GpuTrussElementState)) !=
                cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceAssemblyMap),
                       requestedCapacity * sizeof(GpuTrussAssemblyMap)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostAssemblyMap),
                           requestedCapacity * sizeof(GpuTrussAssemblyMap)) !=
                cudaSuccess ||
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
        m_assemblyResultCount = 0;
        m_assemblyMatrixValueCount = 0;
        m_assemblyFreeDofCount = 0;
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
    GpuTrussElementInput* m_deviceInput = nullptr;
    GpuTrussElementResult* m_deviceResult = nullptr;
    GpuTrussElementInput* m_hostInput = nullptr;
    GpuTrussElementResult* m_hostResult = nullptr;
    GpuTrussElementState* m_deviceState = nullptr;
    GpuTrussElementState* m_hostState = nullptr;
    GpuTrussAssemblyMap* m_deviceAssemblyMap = nullptr;
    GpuTrussAssemblyMap* m_hostAssemblyMap = nullptr;
    double* m_deviceAssemblyValues = nullptr;
    double* m_deviceAssemblyInertia = nullptr;
    std::size_t m_capacity = 0;
    std::size_t m_resultCount = 0;
    std::size_t m_assemblyCapacity = 0;
    std::size_t m_assemblyResultCount = 0;
    int m_assemblyMatrixValueCount = 0;
    int m_assemblyFreeDofCount = 0;
    GpuTrussBatchTiming m_lastTiming;
};

CudaTrussBatchEvaluator::CudaTrussBatchEvaluator()
    : m_impl(std::make_unique<Impl>())
{
}

CudaTrussBatchEvaluator::~CudaTrussBatchEvaluator() = default;

CudaTrussBatchEvaluator::CudaTrussBatchEvaluator(const CudaTrussBatchEvaluator& other)
    : m_impl(std::make_unique<Impl>())
{
    (void)other;
}

CudaTrussBatchEvaluator& CudaTrussBatchEvaluator::operator=(const CudaTrussBatchEvaluator& other)
{
    if (this != &other)
    {
        (void)other;
        m_impl = std::make_unique<Impl>();
    }
    return *this;
}

bool CudaTrussBatchEvaluator::Evaluate(const std::vector<GpuTrussElementInput>& input, double accelerationDerivative)
{
    const GpuTrussElementResult* result = nullptr;
    std::size_t resultCount = 0;
    return m_impl->Evaluate(input, accelerationDerivative, result, resultCount);
}

bool CudaTrussBatchEvaluator::Evaluate(const std::vector<GpuTrussElementInput>& input, double accelerationDerivative,
                                       _OUT std::vector<GpuTrussElementResult>& result)
{
    const GpuTrussElementResult* resultData = nullptr;
    std::size_t resultCount = 0;
    if (!m_impl->Evaluate(input, accelerationDerivative, resultData, resultCount))
        return false;
    result.assign(resultData, resultData + resultCount);
    return true;
}

bool CudaTrussBatchEvaluator::EvaluateAndAssemble(const std::vector<GpuTrussElementInput>& input,
                                                  const std::vector<GpuTrussAssemblyMap>& assemblyMap,
                                                  double accelerationDerivative, int matrixValueCount, int freeDofCount)
{
    return m_impl->EvaluateAndAssemble(input, assemblyMap, accelerationDerivative, matrixValueCount, freeDofCount,
                                       true);
}

bool CudaTrussBatchEvaluator::EvaluateResidualAndAssemble(
    const std::vector<GpuTrussElementInput>& input, const std::vector<GpuTrussAssemblyMap>& assemblyMap,
    int matrixValueCount, int freeDofCount)
{
    return m_impl->EvaluateAndAssemble(input, assemblyMap, 0.0, matrixValueCount, freeDofCount, false);
}

bool CudaTrussBatchEvaluator::CopyAssembledData(_OUT double* matrixValues, int matrixValueCount,
                                                _OUT double* inertiaForce, int freeDofCount)
{
    return m_impl->CopyAssembledData(matrixValues, matrixValueCount, inertiaForce, freeDofCount);
}

bool CudaTrussBatchEvaluator::CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount)
{
    return m_impl->CopyAssembledInertia(inertiaForce, freeDofCount);
}

const GpuTrussElementResult* CudaTrussBatchEvaluator::ResultData() const
{
    return m_impl->ResultData();
}

std::size_t CudaTrussBatchEvaluator::ResultCount() const
{
    return m_impl->ResultCount();
}

const GpuTrussElementState* CudaTrussBatchEvaluator::StateData() const
{
    return m_impl->StateData();
}

const double* CudaTrussBatchEvaluator::DeviceMatrixValues() const
{
    return m_impl->DeviceMatrixValues();
}

int CudaTrussBatchEvaluator::DeviceMatrixValueCount() const
{
    return m_impl->DeviceMatrixValueCount();
}

const GpuTrussBatchTiming& CudaTrussBatchEvaluator::LastTiming() const
{
    return m_impl->LastTiming();
}
}
