#include "CudssSolver.h"
#include "CudssDeviceTransfer.h"

#include <cudss.h>

#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>

namespace SolverNameSpace
{
class CudssSolver::Impl
{
public:
    ~Impl()
    {
        Reset();
    }

    bool Solve(const SpMat& inputMatrix, const Vec& rhs, _OUT Vec& solution)
    {
        m_lastTiming = {};
        SpMat compressedMatrix;
        const SpMat* matrix = &inputMatrix;
        if (!inputMatrix.isCompressed())
        {
            compressedMatrix = inputMatrix;
            compressedMatrix.makeCompressed();
            matrix = &compressedMatrix;
        }

        const int dimension = static_cast<int>(matrix->rows());
        const int nonZeros = static_cast<int>(matrix->nonZeros());
        if (!EnsureStorage(dimension, nonZeros))
            return false;

        bool structureChanged = false;
        if (!PrepareHostCsr(*matrix, structureChanged) || !Upload(rhs, structureChanged))
            return false;
        const bool reusedAnalysis = !structureChanged && m_analysisValid;
        m_lastTiming.reusedAnalysis = reusedAnalysis;

        if (structureChanged || !m_analysisValid)
        {
            const auto analysisStart = std::chrono::steady_clock::now();
            if (!Execute(CUDSS_PHASE_ANALYSIS))
                return false;
            m_lastTiming.analysisMilliseconds =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - analysisStart).count();
            m_analysisValid = true;
        }
        if (cudaEventRecord(m_factorizationStart, m_stream) != cudaSuccess || !Execute(CUDSS_PHASE_FACTORIZATION) ||
            cudaEventRecord(m_factorizationEnd, m_stream) != cudaSuccess ||
            cudaEventRecord(m_solveStart, m_stream) != cudaSuccess || !Execute(CUDSS_PHASE_SOLVE) ||
            cudaEventRecord(m_solveEnd, m_stream) != cudaSuccess)
            return false;

        const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(dimension);
        if (cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_hostSolution, m_solution, vectorBytes, cudaMemcpyDeviceToHost, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess || cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;

        if (!ReadElapsedMilliseconds(m_uploadStart, m_uploadEnd, m_lastTiming.uploadMilliseconds) ||
            !ReadElapsedMilliseconds(m_factorizationStart, m_factorizationEnd, m_lastTiming.factorizationMilliseconds) ||
            !ReadElapsedMilliseconds(m_solveStart, m_solveEnd, m_lastTiming.solveMilliseconds) ||
            !ReadElapsedMilliseconds(m_downloadStart, m_downloadEnd, m_lastTiming.downloadMilliseconds))
            return false;

        solution.resize(dimension);
        std::copy_n(m_hostSolution, dimension, solution.data());
        return solution.allFinite();
    }

    bool SolveDeviceCscValues(const SpMat& inputMatrix, const double* deviceValues, int valueCount, const Vec& rhs,
                              _OUT Vec& solution)
    {
        m_lastTiming = {};
        if (!deviceValues || valueCount != inputMatrix.nonZeros())
            return false;
        SpMat compressedMatrix;
        const SpMat* matrix = &inputMatrix;
        if (!inputMatrix.isCompressed())
        {
            compressedMatrix = inputMatrix;
            compressedMatrix.makeCompressed();
            matrix = &compressedMatrix;
        }

        const int dimension = static_cast<int>(matrix->rows());
        const int nonZeros = static_cast<int>(matrix->nonZeros());
        if (!EnsureStorage(dimension, nonZeros))
            return false;

        bool structureChanged = false;
        if (!PrepareHostCsr(*matrix, structureChanged) || !UploadDeviceCscValues(rhs, deviceValues, structureChanged))
            return false;
        const bool reusedAnalysis = !structureChanged && m_analysisValid;
        m_lastTiming.reusedAnalysis = reusedAnalysis;
        if (structureChanged || !m_analysisValid)
        {
            const auto analysisStart = std::chrono::steady_clock::now();
            if (!Execute(CUDSS_PHASE_ANALYSIS))
                return false;
            m_lastTiming.analysisMilliseconds =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - analysisStart).count();
            m_analysisValid = true;
        }
        if (cudaEventRecord(m_factorizationStart, m_stream) != cudaSuccess || !Execute(CUDSS_PHASE_FACTORIZATION) ||
            cudaEventRecord(m_factorizationEnd, m_stream) != cudaSuccess ||
            cudaEventRecord(m_solveStart, m_stream) != cudaSuccess || !Execute(CUDSS_PHASE_SOLVE) ||
            cudaEventRecord(m_solveEnd, m_stream) != cudaSuccess)
            return false;

        const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(dimension);
        if (cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_hostSolution, m_solution, vectorBytes, cudaMemcpyDeviceToHost, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess || cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;
        if (!ReadElapsedMilliseconds(m_uploadStart, m_uploadEnd, m_lastTiming.uploadMilliseconds) ||
            !ReadElapsedMilliseconds(m_factorizationStart, m_factorizationEnd, m_lastTiming.factorizationMilliseconds) ||
            !ReadElapsedMilliseconds(m_solveStart, m_solveEnd, m_lastTiming.solveMilliseconds) ||
            !ReadElapsedMilliseconds(m_downloadStart, m_downloadEnd, m_lastTiming.downloadMilliseconds))
            return false;
        solution.resize(dimension);
        std::copy_n(m_hostSolution, dimension, solution.data());
        return solution.allFinite();
    }

    const CudssSolveTiming& LastTiming() const
    {
        return m_lastTiming;
    }

private:
    bool EnsureBaseResources()
    {
        if (cudaSetDevice(0) != cudaSuccess)
            return false;
        if (!m_stream && cudaStreamCreate(&m_stream) != cudaSuccess)
            return false;
        if (!EnsureTimingEvents())
            return false;
        if (!m_handle && cudssCreate(&m_handle) != CUDSS_STATUS_SUCCESS)
            return false;
        if (cudssSetStream(m_handle, m_stream) != CUDSS_STATUS_SUCCESS)
            return false;
        if (!m_config && cudssConfigCreate(&m_config) != CUDSS_STATUS_SUCCESS)
            return false;
        if (!m_data && cudssDataCreate(m_handle, &m_data) != CUDSS_STATUS_SUCCESS)
            return false;
        return true;
    }

    bool EnsureStorage(int dimension, int nonZeros)
    {
        if (!EnsureBaseResources() || dimension <= 0 || nonZeros <= 0)
            return false;
        if (m_dimension == dimension && m_nonZeros == nonZeros && m_matrix && m_rhsMatrix && m_solutionMatrix &&
            m_hostRows && m_hostColumns && m_hostValues && m_hostRhs && m_hostSolution)
            return true;

        ReleaseMatrixResources();
        const std::size_t rowBytes = sizeof(int) * static_cast<std::size_t>(dimension + 1);
        const std::size_t columnBytes = sizeof(int) * static_cast<std::size_t>(nonZeros);
        const std::size_t valueBytes = sizeof(double) * static_cast<std::size_t>(nonZeros);
        const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(dimension);
        if (cudaMalloc(reinterpret_cast<void**>(&m_rows), rowBytes) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_columns), columnBytes) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_values), valueBytes) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_rhs), vectorBytes) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_solution), vectorBytes) != cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostRows), rowBytes) != cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostColumns), columnBytes) != cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostValues), valueBytes) != cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostRhs), vectorBytes) != cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostSolution), vectorBytes) != cudaSuccess)
        {
            ReleaseMatrixResources();
            return false;
        }

        m_dimension = dimension;
        m_nonZeros = nonZeros;
        const int64_t size = dimension;
        const bool matricesCreated =
            cudssMatrixCreateCsr(&m_matrix, size, size, nonZeros, m_rows, nullptr, m_columns, m_values, CUDSS_R_32I,
                                 CUDSS_R_32I, CUDSS_R_64F, CUDSS_MTYPE_GENERAL, CUDSS_MVIEW_FULL,
                                 CUDSS_BASE_ZERO) == CUDSS_STATUS_SUCCESS &&
            cudssMatrixCreateDn(&m_rhsMatrix, size, 1, size, m_rhs, CUDSS_R_64F, CUDSS_LAYOUT_COL_MAJOR) ==
                CUDSS_STATUS_SUCCESS &&
            cudssMatrixCreateDn(&m_solutionMatrix, size, 1, size, m_solution, CUDSS_R_64F, CUDSS_LAYOUT_COL_MAJOR) ==
                CUDSS_STATUS_SUCCESS;
        if (!matricesCreated)
        {
            ReleaseMatrixResources();
            return false;
        }
        return true;
    }

    bool PrepareHostCsr(const SpMat& matrix, _OUT bool& structureChanged)
    {
        std::uint64_t hash = 1469598103934665603ull;
        AppendHash(matrix.outerIndexPtr(),
                   sizeof(SpMat::StorageIndex) * static_cast<std::size_t>(matrix.outerSize() + 1), hash);
        AppendHash(matrix.innerIndexPtr(), sizeof(SpMat::StorageIndex) * static_cast<std::size_t>(matrix.nonZeros()),
                   hash);
        structureChanged = m_structureHash != hash;
        if (structureChanged)
        {
            std::fill_n(m_hostRows, m_dimension + 1, 0);
            m_valueSourceIndices.resize(static_cast<std::size_t>(m_nonZeros));
            for (Eigen::Index source = 0; source < matrix.nonZeros(); ++source)
                ++m_hostRows[matrix.innerIndexPtr()[source] + 1];
            for (int row = 0; row < m_dimension; ++row)
                m_hostRows[row + 1] += m_hostRows[row];

            std::vector<int> next(m_hostRows, m_hostRows + m_dimension + 1);
            for (int column = 0; column < matrix.outerSize(); ++column)
            {
                for (Eigen::Index source = matrix.outerIndexPtr()[column]; source < matrix.outerIndexPtr()[column + 1];
                     ++source)
                {
                    const int row = matrix.innerIndexPtr()[source];
                    const int destination = next[static_cast<std::size_t>(row)]++;
                    m_hostColumns[destination] = column;
                    m_valueSourceIndices[static_cast<std::size_t>(destination)] = source;
                }
            }
            m_structureHash = hash;
            m_analysisValid = false;
            m_csrValueSourceIndices.resize(static_cast<std::size_t>(m_nonZeros));
            for (int destination = 0; destination < m_nonZeros; ++destination)
            {
                const Eigen::Index source = m_valueSourceIndices[static_cast<std::size_t>(destination)];
                if (source < 0 || source > std::numeric_limits<int>::max())
                    return false;
                m_csrValueSourceIndices[static_cast<std::size_t>(destination)] = static_cast<int>(source);
            }
        }

        for (int destination = 0; destination < m_nonZeros; ++destination)
        {
            m_hostValues[destination] = matrix.valuePtr()[m_valueSourceIndices[static_cast<std::size_t>(destination)]];
        }
        return true;
    }

    bool Upload(const Vec& rhs, bool structureChanged)
    {
        const std::size_t rowBytes = sizeof(int) * static_cast<std::size_t>(m_dimension + 1);
        const std::size_t columnBytes = sizeof(int) * static_cast<std::size_t>(m_nonZeros);
        const std::size_t valueBytes = sizeof(double) * static_cast<std::size_t>(m_nonZeros);
        const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(m_dimension);
        std::copy_n(rhs.data(), m_dimension, m_hostRhs);
        if (cudaEventRecord(m_uploadStart, m_stream) != cudaSuccess)
            return false;
        if (structureChanged &&
            (cudaMemcpyAsync(m_rows, m_hostRows, rowBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
             cudaMemcpyAsync(m_columns, m_hostColumns, columnBytes, cudaMemcpyHostToDevice, m_stream) !=
                 cudaSuccess))
            return false;
        return cudaMemcpyAsync(m_values, m_hostValues, valueBytes, cudaMemcpyHostToDevice, m_stream) ==
                   cudaSuccess &&
               cudaMemcpyAsync(m_rhs, m_hostRhs, vectorBytes, cudaMemcpyHostToDevice, m_stream) == cudaSuccess &&
               cudaEventRecord(m_uploadEnd, m_stream) == cudaSuccess;
    }

    bool UploadDeviceCscValues(const Vec& rhs, const double* deviceValues, bool structureChanged)
    {
        const std::size_t rowBytes = sizeof(int) * static_cast<std::size_t>(m_dimension + 1);
        const std::size_t columnBytes = sizeof(int) * static_cast<std::size_t>(m_nonZeros);
        const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(m_dimension);
        std::copy_n(rhs.data(), m_dimension, m_hostRhs);
        if (cudaEventRecord(m_uploadStart, m_stream) != cudaSuccess)
            return false;
        if (structureChanged)
        {
            if (m_deviceValueSourceIndices)
            {
                cudaFree(m_deviceValueSourceIndices);
                m_deviceValueSourceIndices = nullptr;
            }
            if (cudaMemcpyAsync(m_rows, m_hostRows, rowBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
                cudaMemcpyAsync(m_columns, m_hostColumns, columnBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
                cudaMalloc(reinterpret_cast<void**>(&m_deviceValueSourceIndices),
                           sizeof(int) * static_cast<std::size_t>(m_nonZeros)) != cudaSuccess ||
                cudaMemcpyAsync(m_deviceValueSourceIndices, m_csrValueSourceIndices.data(),
                                sizeof(int) * static_cast<std::size_t>(m_nonZeros), cudaMemcpyHostToDevice,
                                m_stream) != cudaSuccess)
                return false;
        }
        return ReorderDeviceCscValuesToCsr(deviceValues, m_deviceValueSourceIndices, m_nonZeros, m_values, m_stream) &&
               cudaMemcpyAsync(m_rhs, m_hostRhs, vectorBytes, cudaMemcpyHostToDevice, m_stream) == cudaSuccess &&
               cudaEventRecord(m_uploadEnd, m_stream) == cudaSuccess;
    }

    bool Execute(cudssPhase_t phase)
    {
        return cudssExecute(m_handle, phase, m_config, m_data, m_matrix, m_solutionMatrix, m_rhsMatrix) ==
               CUDSS_STATUS_SUCCESS;
    }

    static void AppendHash(const void* bytes, std::size_t size, _OUT std::uint64_t& hash)
    {
        const auto* data = static_cast<const unsigned char*>(bytes);
        for (std::size_t index = 0; index < size; ++index)
        {
            hash ^= data[index];
            hash *= 1099511628211ull;
        }
    }

    bool EnsureTimingEvents()
    {
        return CreateTimingEvent(m_uploadStart) && CreateTimingEvent(m_uploadEnd) &&
               CreateTimingEvent(m_factorizationStart) && CreateTimingEvent(m_factorizationEnd) &&
               CreateTimingEvent(m_solveStart) && CreateTimingEvent(m_solveEnd) &&
               CreateTimingEvent(m_downloadStart) && CreateTimingEvent(m_downloadEnd);
    }

    static bool CreateTimingEvent(_OUT cudaEvent_t& event)
    {
        return event || cudaEventCreate(&event) == cudaSuccess;
    }

    static bool ReadElapsedMilliseconds(cudaEvent_t start, cudaEvent_t end, _OUT double& milliseconds)
    {
        float elapsed = 0.0F;
        if (cudaEventElapsedTime(&elapsed, start, end) != cudaSuccess)
            return false;
        milliseconds = static_cast<double>(elapsed);
        return true;
    }

    void ReleaseTimingEvents()
    {
        if (m_downloadEnd)
            cudaEventDestroy(m_downloadEnd);
        if (m_downloadStart)
            cudaEventDestroy(m_downloadStart);
        if (m_solveEnd)
            cudaEventDestroy(m_solveEnd);
        if (m_solveStart)
            cudaEventDestroy(m_solveStart);
        if (m_factorizationEnd)
            cudaEventDestroy(m_factorizationEnd);
        if (m_factorizationStart)
            cudaEventDestroy(m_factorizationStart);
        if (m_uploadEnd)
            cudaEventDestroy(m_uploadEnd);
        if (m_uploadStart)
            cudaEventDestroy(m_uploadStart);
        m_uploadStart = nullptr;
        m_uploadEnd = nullptr;
        m_factorizationStart = nullptr;
        m_factorizationEnd = nullptr;
        m_solveStart = nullptr;
        m_solveEnd = nullptr;
        m_downloadStart = nullptr;
        m_downloadEnd = nullptr;
    }

    void ReleaseMatrixResources()
    {
        if (m_solutionMatrix)
            cudssMatrixDestroy(m_solutionMatrix);
        if (m_rhsMatrix)
            cudssMatrixDestroy(m_rhsMatrix);
        if (m_matrix)
            cudssMatrixDestroy(m_matrix);
        if (m_solution)
            cudaFree(m_solution);
        if (m_rhs)
            cudaFree(m_rhs);
        if (m_values)
            cudaFree(m_values);
        if (m_deviceValueSourceIndices)
            cudaFree(m_deviceValueSourceIndices);
        if (m_columns)
            cudaFree(m_columns);
        if (m_rows)
            cudaFree(m_rows);
        if (m_hostSolution)
            cudaFreeHost(m_hostSolution);
        if (m_hostRhs)
            cudaFreeHost(m_hostRhs);
        if (m_hostValues)
            cudaFreeHost(m_hostValues);
        if (m_hostColumns)
            cudaFreeHost(m_hostColumns);
        if (m_hostRows)
            cudaFreeHost(m_hostRows);
        m_solutionMatrix = nullptr;
        m_rhsMatrix = nullptr;
        m_matrix = nullptr;
        m_solution = nullptr;
        m_rhs = nullptr;
        m_values = nullptr;
        m_deviceValueSourceIndices = nullptr;
        m_columns = nullptr;
        m_rows = nullptr;
        m_hostSolution = nullptr;
        m_hostRhs = nullptr;
        m_hostValues = nullptr;
        m_hostColumns = nullptr;
        m_hostRows = nullptr;
        m_dimension = 0;
        m_nonZeros = 0;
        m_structureHash = 0;
        m_analysisValid = false;
        m_valueSourceIndices.clear();
        m_csrValueSourceIndices.clear();
    }

    void Reset()
    {
        ReleaseMatrixResources();
        if (m_data)
            cudssDataDestroy(m_handle, m_data);
        if (m_config)
            cudssConfigDestroy(m_config);
        if (m_handle)
            cudssDestroy(m_handle);
        ReleaseTimingEvents();
        if (m_stream)
            cudaStreamDestroy(m_stream);
        m_data = nullptr;
        m_config = nullptr;
        m_handle = nullptr;
        m_stream = nullptr;
    }

    cudaStream_t m_stream = nullptr;
    cudssHandle_t m_handle = nullptr;
    cudssConfig_t m_config = nullptr;
    cudssData_t m_data = nullptr;
    cudssMatrix_t m_matrix = nullptr;
    cudssMatrix_t m_rhsMatrix = nullptr;
    cudssMatrix_t m_solutionMatrix = nullptr;
    int* m_rows = nullptr;
    int* m_columns = nullptr;
    int* m_hostRows = nullptr;
    int* m_hostColumns = nullptr;
    double* m_values = nullptr;
    double* m_rhs = nullptr;
    double* m_solution = nullptr;
    double* m_hostValues = nullptr;
    double* m_hostRhs = nullptr;
    double* m_hostSolution = nullptr;
    int* m_deviceValueSourceIndices = nullptr;
    cudaEvent_t m_uploadStart = nullptr;
    cudaEvent_t m_uploadEnd = nullptr;
    cudaEvent_t m_factorizationStart = nullptr;
    cudaEvent_t m_factorizationEnd = nullptr;
    cudaEvent_t m_solveStart = nullptr;
    cudaEvent_t m_solveEnd = nullptr;
    cudaEvent_t m_downloadStart = nullptr;
    cudaEvent_t m_downloadEnd = nullptr;
    int m_dimension = 0;
    int m_nonZeros = 0;
    std::uint64_t m_structureHash = 0;
    bool m_analysisValid = false;
    std::vector<Eigen::Index> m_valueSourceIndices;
    std::vector<int> m_csrValueSourceIndices;
    CudssSolveTiming m_lastTiming;
};

CudssSolver::CudssSolver()
    : m_impl(std::make_unique<Impl>())
{
}

CudssSolver::~CudssSolver() = default;

bool CudssSolver::Solve(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution)
{
    if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() || matrix.rows() == 0)
        return false;
    return m_impl->Solve(matrix, rhs, solution);
}

bool CudssSolver::SolveDeviceCscValues(const SpMat& matrix, const double* deviceValues, int valueCount,
                                        const Vec& rhs, _OUT Vec& solution)
{
    if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() || matrix.rows() == 0)
        return false;
    return m_impl->SolveDeviceCscValues(matrix, deviceValues, valueCount, rhs, solution);
}

const CudssSolveTiming& CudssSolver::LastTiming() const
{
    return m_impl->LastTiming();
}
}
