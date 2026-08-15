#include "Solver/CudssSolver.h"

#include <cudss.h>

#include <cuda_runtime_api.h>

#include <cstdint>
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
        if (structureChanged || !m_analysisValid)
        {
            if (!Execute(CUDSS_PHASE_ANALYSIS))
                return false;
            m_analysisValid = true;
        }
        if (!Execute(CUDSS_PHASE_FACTORIZATION) || !Execute(CUDSS_PHASE_SOLVE))
            return false;

        solution.resize(dimension);
        const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(dimension);
        return cudaMemcpy(solution.data(), m_solution, vectorBytes, cudaMemcpyDeviceToHost) == cudaSuccess &&
               solution.allFinite();
    }

private:
    bool EnsureBaseResources()
    {
        if (cudaSetDevice(0) != cudaSuccess)
            return false;
        if (!m_stream && cudaStreamCreate(&m_stream) != cudaSuccess)
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
        if (m_dimension == dimension && m_nonZeros == nonZeros && m_matrix && m_rhsMatrix && m_solutionMatrix)
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
            cudaMalloc(reinterpret_cast<void**>(&m_solution), vectorBytes) != cudaSuccess)
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
        structureChanged = m_structureHash != hash || static_cast<int>(m_hostValues.size()) != m_nonZeros;
        if (structureChanged)
        {
            m_hostRows.assign(static_cast<std::size_t>(m_dimension + 1), 0);
            m_hostColumns.resize(static_cast<std::size_t>(m_nonZeros));
            m_valueSourceIndices.resize(static_cast<std::size_t>(m_nonZeros));
            m_hostValues.resize(static_cast<std::size_t>(m_nonZeros));
            for (Eigen::Index source = 0; source < matrix.nonZeros(); ++source)
                ++m_hostRows[static_cast<std::size_t>(matrix.innerIndexPtr()[source] + 1)];
            for (int row = 0; row < m_dimension; ++row)
                m_hostRows[static_cast<std::size_t>(row + 1)] += m_hostRows[static_cast<std::size_t>(row)];

            std::vector<int> next = m_hostRows;
            for (int column = 0; column < matrix.outerSize(); ++column)
            {
                for (Eigen::Index source = matrix.outerIndexPtr()[column]; source < matrix.outerIndexPtr()[column + 1];
                     ++source)
                {
                    const int row = matrix.innerIndexPtr()[source];
                    const int destination = next[static_cast<std::size_t>(row)]++;
                    m_hostColumns[static_cast<std::size_t>(destination)] = column;
                    m_valueSourceIndices[static_cast<std::size_t>(destination)] = source;
                }
            }
            m_structureHash = hash;
            m_analysisValid = false;
        }

        for (int destination = 0; destination < m_nonZeros; ++destination)
        {
            m_hostValues[static_cast<std::size_t>(destination)] =
                matrix.valuePtr()[m_valueSourceIndices[static_cast<std::size_t>(destination)]];
        }
        return true;
    }

    bool Upload(const Vec& rhs, bool structureChanged)
    {
        const std::size_t rowBytes = sizeof(int) * static_cast<std::size_t>(m_dimension + 1);
        const std::size_t columnBytes = sizeof(int) * static_cast<std::size_t>(m_nonZeros);
        const std::size_t valueBytes = sizeof(double) * static_cast<std::size_t>(m_nonZeros);
        const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(m_dimension);
        if (structureChanged &&
            (cudaMemcpyAsync(m_rows, m_hostRows.data(), rowBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
             cudaMemcpyAsync(m_columns, m_hostColumns.data(), columnBytes, cudaMemcpyHostToDevice, m_stream) !=
                 cudaSuccess))
            return false;
        return cudaMemcpyAsync(m_values, m_hostValues.data(), valueBytes, cudaMemcpyHostToDevice, m_stream) ==
                   cudaSuccess &&
               cudaMemcpyAsync(m_rhs, rhs.data(), vectorBytes, cudaMemcpyHostToDevice, m_stream) == cudaSuccess;
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
        if (m_columns)
            cudaFree(m_columns);
        if (m_rows)
            cudaFree(m_rows);
        m_solutionMatrix = nullptr;
        m_rhsMatrix = nullptr;
        m_matrix = nullptr;
        m_solution = nullptr;
        m_rhs = nullptr;
        m_values = nullptr;
        m_columns = nullptr;
        m_rows = nullptr;
        m_dimension = 0;
        m_nonZeros = 0;
        m_structureHash = 0;
        m_analysisValid = false;
        m_hostRows.clear();
        m_hostColumns.clear();
        m_valueSourceIndices.clear();
        m_hostValues.clear();
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
    double* m_values = nullptr;
    double* m_rhs = nullptr;
    double* m_solution = nullptr;
    int m_dimension = 0;
    int m_nonZeros = 0;
    std::uint64_t m_structureHash = 0;
    bool m_analysisValid = false;
    std::vector<int> m_hostRows;
    std::vector<int> m_hostColumns;
    std::vector<Eigen::Index> m_valueSourceIndices;
    std::vector<double> m_hostValues;
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
}
