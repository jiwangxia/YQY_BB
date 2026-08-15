#include "Solver/CudaSparseSolver.h"

#include <cublas_v2.h>
#include <cuda_runtime_api.h>

// CUDA 13 将旧版声明和通用稀疏 API 放在同一头文件中，此处只使用通用 API。
#define DISABLE_CUSPARSE_DEPRECATED
#include <cusparse.h>
#undef DISABLE_CUSPARSE_DEPRECATED

#include <Eigen/SparseCore>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

static bool IsSuccess(cudaError_t status)
{
    return status == cudaSuccess;
}

static bool IsSuccess(cublasStatus_t status)
{
    return status == CUBLAS_STATUS_SUCCESS;
}

static bool IsSuccess(cusparseStatus_t status)
{
    return status == CUSPARSE_STATUS_SUCCESS;
}

template <typename T> static bool AllocateDevice(T** pointer, std::size_t bytes)
{
    return bytes == 0 || IsSuccess(cudaMalloc(reinterpret_cast<void**>(pointer), bytes));
}

namespace SolverNameSpace
{
struct CudaSparseSolver::Impl
{
    cublasHandle_t blas = nullptr;
    cusparseHandle_t sparse = nullptr;
    cusparseSpMatDescr_t sparseMatrix = nullptr;
    cusparseDnVecDescr_t vectorX = nullptr;
    cusparseDnVecDescr_t vectorPHat = nullptr;
    cusparseDnVecDescr_t vectorSHat = nullptr;
    cusparseDnVecDescr_t vectorV = nullptr;
    cusparseDnVecDescr_t vectorT = nullptr;
    double* values = nullptr;
    double* diagonalInverse = nullptr;
    int* rows = nullptr;
    int* columns = nullptr;
    double* x = nullptr;
    double* r = nullptr;
    double* rHat = nullptr;
    double* p = nullptr;
    double* pHat = nullptr;
    double* v = nullptr;
    double* s = nullptr;
    double* sHat = nullptr;
    double* t = nullptr;
    void* spmvBuffer = nullptr;
    std::size_t spmvBufferCapacity = 0;
    int dimension = 0;
    int nonZeros = 0;
    std::uint64_t structureHash = 0;
    std::vector<int> hostRows;
    std::vector<int> hostColumns;
    std::vector<Eigen::Index> valueSourceIndices;
    std::vector<double> hostValues;
    std::vector<double> hostDiagonalInverse;

    ~Impl()
    {
        Reset();
    }

    void ReleaseDescriptors()
    {
        if (vectorT)
            cusparseDestroyDnVec(vectorT);
        if (vectorV)
            cusparseDestroyDnVec(vectorV);
        if (vectorSHat)
            cusparseDestroyDnVec(vectorSHat);
        if (vectorPHat)
            cusparseDestroyDnVec(vectorPHat);
        if (vectorX)
            cusparseDestroyDnVec(vectorX);
        if (sparseMatrix)
            cusparseDestroySpMat(sparseMatrix);
        vectorT = nullptr;
        vectorV = nullptr;
        vectorSHat = nullptr;
        vectorPHat = nullptr;
        vectorX = nullptr;
        sparseMatrix = nullptr;
    }

    void ReleaseMemory()
    {
        if (spmvBuffer)
            cudaFree(spmvBuffer);
        if (t)
            cudaFree(t);
        if (s)
            cudaFree(s);
        if (sHat)
            cudaFree(sHat);
        if (v)
            cudaFree(v);
        if (p)
            cudaFree(p);
        if (pHat)
            cudaFree(pHat);
        if (rHat)
            cudaFree(rHat);
        if (r)
            cudaFree(r);
        if (x)
            cudaFree(x);
        if (columns)
            cudaFree(columns);
        if (rows)
            cudaFree(rows);
        if (values)
            cudaFree(values);
        if (diagonalInverse)
            cudaFree(diagonalInverse);
        spmvBuffer = nullptr;
        t = nullptr;
        s = nullptr;
        sHat = nullptr;
        v = nullptr;
        p = nullptr;
        pHat = nullptr;
        rHat = nullptr;
        r = nullptr;
        x = nullptr;
        columns = nullptr;
        rows = nullptr;
        values = nullptr;
        diagonalInverse = nullptr;
        spmvBufferCapacity = 0;
        dimension = 0;
        nonZeros = 0;
        structureHash = 0;
        hostRows.clear();
        hostColumns.clear();
        valueSourceIndices.clear();
        hostValues.clear();
        hostDiagonalInverse.clear();
    }

    void Reset()
    {
        ReleaseDescriptors();
        ReleaseMemory();
        if (sparse)
            cusparseDestroy(sparse);
        if (blas)
            cublasDestroy(blas);
        sparse = nullptr;
        blas = nullptr;
    }

    bool EnsureHandles()
    {
        if (!IsSuccess(cudaSetDevice(0)))
            return false;
        if (!blas && !IsSuccess(cublasCreate(&blas)))
            return false;
        if (!sparse && !IsSuccess(cusparseCreate(&sparse)))
            return false;
        return true;
    }

    bool EnsureStorage(int requestedDimension, int requestedNonZeros)
    {
        if (!EnsureHandles() || requestedDimension <= 0 || requestedNonZeros <= 0)
            return false;
        if (dimension == requestedDimension && nonZeros == requestedNonZeros && sparseMatrix)
            return true;

        ReleaseDescriptors();
        ReleaseMemory();
        const std::size_t valueBytes = sizeof(double) * static_cast<std::size_t>(requestedNonZeros);
        const std::size_t rowBytes = sizeof(int) * static_cast<std::size_t>(requestedDimension + 1);
        const std::size_t columnBytes = sizeof(int) * static_cast<std::size_t>(requestedNonZeros);
        const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(requestedDimension);
        const bool allocated =
            AllocateDevice(&values, valueBytes) && AllocateDevice(&rows, rowBytes) &&
            AllocateDevice(&columns, columnBytes) && AllocateDevice(&diagonalInverse, vectorBytes) &&
            AllocateDevice(&x, vectorBytes) && AllocateDevice(&r, vectorBytes) && AllocateDevice(&rHat, vectorBytes) &&
            AllocateDevice(&p, vectorBytes) && AllocateDevice(&pHat, vectorBytes) && AllocateDevice(&v, vectorBytes) &&
            AllocateDevice(&s, vectorBytes) && AllocateDevice(&sHat, vectorBytes) && AllocateDevice(&t, vectorBytes);
        if (!allocated)
        {
            ReleaseMemory();
            return false;
        }

        dimension = requestedDimension;
        nonZeros = requestedNonZeros;
        const bool described = IsSuccess(cusparseCreateCsr(&sparseMatrix, dimension, dimension, nonZeros, rows, columns,
                                                           values, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I,
                                                           CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F)) &&
                               IsSuccess(cusparseCreateDnVec(&vectorX, dimension, x, CUDA_R_64F)) &&
                               IsSuccess(cusparseCreateDnVec(&vectorPHat, dimension, pHat, CUDA_R_64F)) &&
                               IsSuccess(cusparseCreateDnVec(&vectorSHat, dimension, sHat, CUDA_R_64F)) &&
                               IsSuccess(cusparseCreateDnVec(&vectorV, dimension, v, CUDA_R_64F)) &&
                               IsSuccess(cusparseCreateDnVec(&vectorT, dimension, t, CUDA_R_64F));
        if (!described)
        {
            ReleaseDescriptors();
            ReleaseMemory();
            return false;
        }
        return true;
    }

    bool EnsureSpmvBuffer()
    {
        const double one = 1.0;
        const double zero = 0.0;
        std::size_t requiredSize = 0;
        if (!IsSuccess(cusparseSpMV_bufferSize(sparse, CUSPARSE_OPERATION_NON_TRANSPOSE, &one, sparseMatrix, vectorPHat,
                                               &zero, vectorV, CUDA_R_64F, CUSPARSE_SPMV_ALG_DEFAULT, &requiredSize)))
            return false;
        if (requiredSize <= spmvBufferCapacity)
            return true;
        if (spmvBuffer)
            cudaFree(spmvBuffer);
        spmvBuffer = nullptr;
        spmvBufferCapacity = 0;
        if (requiredSize > 0 && !IsSuccess(cudaMalloc(&spmvBuffer, requiredSize)))
            return false;
        spmvBufferCapacity = requiredSize;
        return true;
    }

    bool PrepareHostCsr(const SpMat& matrix, _OUT bool& structureChanged)
    {
        std::uint64_t hash = 1469598103934665603ull;
        const auto append = [&hash](const void* bytes, std::size_t size)
        {
            const auto* data = static_cast<const unsigned char*>(bytes);
            for (std::size_t index = 0; index < size; ++index)
            {
                hash ^= data[index];
                hash *= 1099511628211ull;
            }
        };
        append(matrix.outerIndexPtr(), sizeof(SpMat::StorageIndex) * static_cast<std::size_t>(matrix.outerSize() + 1));
        append(matrix.innerIndexPtr(), sizeof(SpMat::StorageIndex) * static_cast<std::size_t>(matrix.nonZeros()));
        structureChanged = structureHash != hash || static_cast<int>(hostValues.size()) != nonZeros;
        if (structureChanged)
        {
            hostRows.assign(static_cast<std::size_t>(dimension + 1), 0);
            hostColumns.resize(static_cast<std::size_t>(nonZeros));
            valueSourceIndices.resize(static_cast<std::size_t>(nonZeros));
            hostValues.resize(static_cast<std::size_t>(nonZeros));
            hostDiagonalInverse.resize(static_cast<std::size_t>(dimension));
            for (Eigen::Index sourceIndex = 0; sourceIndex < matrix.nonZeros(); ++sourceIndex)
            {
                const int row = matrix.innerIndexPtr()[sourceIndex];
                ++hostRows[static_cast<std::size_t>(row + 1)];
            }
            for (int row = 0; row < dimension; ++row)
                hostRows[static_cast<std::size_t>(row + 1)] += hostRows[static_cast<std::size_t>(row)];
            std::vector<int> next = hostRows;
            for (int column = 0; column < matrix.outerSize(); ++column)
            {
                for (Eigen::Index sourceIndex = matrix.outerIndexPtr()[column];
                     sourceIndex < matrix.outerIndexPtr()[column + 1]; ++sourceIndex)
                {
                    const int row = matrix.innerIndexPtr()[sourceIndex];
                    const int destination = next[static_cast<std::size_t>(row)]++;
                    hostColumns[static_cast<std::size_t>(destination)] = column;
                    valueSourceIndices[static_cast<std::size_t>(destination)] = sourceIndex;
                }
            }
            structureHash = hash;
        }
        for (int destination = 0; destination < nonZeros; ++destination)
        {
            hostValues[static_cast<std::size_t>(destination)] =
                matrix.valuePtr()[valueSourceIndices[static_cast<std::size_t>(destination)]];
        }
        std::fill(hostDiagonalInverse.begin(), hostDiagonalInverse.end(), 1.0);
        for (int row = 0; row < dimension; ++row)
        {
            for (int index = hostRows[static_cast<std::size_t>(row)];
                 index < hostRows[static_cast<std::size_t>(row + 1)]; ++index)
            {
                if (hostColumns[static_cast<std::size_t>(index)] != row)
                    continue;
                const double diagonal = hostValues[static_cast<std::size_t>(index)];
                if (std::abs(diagonal) > 1.0e-30)
                    hostDiagonalInverse[static_cast<std::size_t>(row)] = 1.0 / diagonal;
                break;
            }
        }
        return true;
    }
};

CudaSparseSolver::CudaSparseSolver()
    : m_impl(std::make_unique<Impl>())
{
}

CudaSparseSolver::~CudaSparseSolver() = default;

bool CudaSparseSolver::Solve(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, double relativeTolerance,
                             int maximumIterations)
{
    if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() || rhs.size() == 0)
        return false;

    SpMat compressedMatrix;
    const SpMat* sourceMatrix = &matrix;
    if (!matrix.isCompressed())
    {
        compressedMatrix = matrix;
        compressedMatrix.makeCompressed();
        sourceMatrix = &compressedMatrix;
    }
    const int n = static_cast<int>(rhs.size());
    const int nnz = static_cast<int>(sourceMatrix->nonZeros());
    if (maximumIterations <= 0)
        maximumIterations = std::max(200, std::min(10000, n * 2));
    if (!m_impl->EnsureStorage(n, nnz))
        return false;
    bool structureChanged = false;
    if (!m_impl->PrepareHostCsr(*sourceMatrix, structureChanged))
        return false;

    const std::size_t valueBytes = sizeof(double) * static_cast<std::size_t>(nnz);
    const std::size_t rowBytes = sizeof(int) * static_cast<std::size_t>(n + 1);
    const std::size_t columnBytes = sizeof(int) * static_cast<std::size_t>(nnz);
    const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(n);
    if (!IsSuccess(cudaMemcpy(m_impl->values, m_impl->hostValues.data(), valueBytes, cudaMemcpyHostToDevice)) ||
        !IsSuccess(cudaMemcpy(m_impl->diagonalInverse, m_impl->hostDiagonalInverse.data(), vectorBytes,
                              cudaMemcpyHostToDevice)) ||
        (structureChanged &&
         (!IsSuccess(cudaMemcpy(m_impl->rows, m_impl->hostRows.data(), rowBytes, cudaMemcpyHostToDevice)) ||
          !IsSuccess(cudaMemcpy(m_impl->columns, m_impl->hostColumns.data(), columnBytes, cudaMemcpyHostToDevice)))) ||
        !IsSuccess(cudaMemset(m_impl->x, 0, vectorBytes)) ||
        !IsSuccess(cudaMemcpy(m_impl->r, rhs.data(), vectorBytes, cudaMemcpyHostToDevice)) ||
        !IsSuccess(cudaMemcpy(m_impl->rHat, rhs.data(), vectorBytes, cudaMemcpyHostToDevice)) ||
        !IsSuccess(cudaMemset(m_impl->p, 0, vectorBytes)) || !IsSuccess(cudaMemset(m_impl->v, 0, vectorBytes)) ||
        !m_impl->EnsureSpmvBuffer())
        return false;

    const double one = 1.0;
    const auto multiply = [this](cusparseDnVecDescr_t input, cusparseDnVecDescr_t output)
    {
        const double one = 1.0;
        const double zero = 0.0;
        return IsSuccess(cusparseSpMV(m_impl->sparse, CUSPARSE_OPERATION_NON_TRANSPOSE, &one, m_impl->sparseMatrix,
                                      input, &zero, output, CUDA_R_64F, CUSPARSE_SPMV_ALG_DEFAULT, m_impl->spmvBuffer));
    };
    const auto applyPreconditioner = [this, n](const double* input, double* output)
    {
        return IsSuccess(
            cublasDdgmm(m_impl->blas, CUBLAS_SIDE_LEFT, n, 1, input, n, m_impl->diagonalInverse, 1, output, n));
    };
    double normB = 0.0;
    if (!IsSuccess(cublasDnrm2(m_impl->blas, n, m_impl->r, 1, &normB)))
        return false;
    if (normB == 0.0)
    {
        solution.setZero(n);
        return true;
    }

    const double tolerance = relativeTolerance * normB;
    double rhoOld = 1.0;
    double alpha = 1.0;
    double omega = 1.0;
    for (int iteration = 0; iteration < maximumIterations; ++iteration)
    {
        double rho = 0.0;
        if (!IsSuccess(cublasDdot(m_impl->blas, n, m_impl->rHat, 1, m_impl->r, 1, &rho)) || std::abs(rho) < 1.0e-300)
            return false;
        const double beta = (rho / rhoOld) * (alpha / omega);
        const double negativePreviousOmega = -omega;
        if (!IsSuccess(cublasDaxpy(m_impl->blas, n, &negativePreviousOmega, m_impl->v, 1, m_impl->p, 1)) ||
            !IsSuccess(cublasDscal(m_impl->blas, n, &beta, m_impl->p, 1)) ||
            !IsSuccess(cublasDaxpy(m_impl->blas, n, &one, m_impl->r, 1, m_impl->p, 1)) ||
            !applyPreconditioner(m_impl->p, m_impl->pHat) || !multiply(m_impl->vectorPHat, m_impl->vectorV))
            return false;

        double rHatV = 0.0;
        if (!IsSuccess(cublasDdot(m_impl->blas, n, m_impl->rHat, 1, m_impl->v, 1, &rHatV)) ||
            std::abs(rHatV) < 1.0e-300)
            return false;
        alpha = rho / rHatV;
        const double negativeAlpha = -alpha;
        if (!IsSuccess(cudaMemcpy(m_impl->s, m_impl->r, vectorBytes, cudaMemcpyDeviceToDevice)) ||
            !IsSuccess(cublasDaxpy(m_impl->blas, n, &negativeAlpha, m_impl->v, 1, m_impl->s, 1)))
            return false;

        double normS = 0.0;
        if (!IsSuccess(cublasDnrm2(m_impl->blas, n, m_impl->s, 1, &normS)))
            return false;
        if (normS <= tolerance)
        {
            if (!IsSuccess(cublasDaxpy(m_impl->blas, n, &alpha, m_impl->pHat, 1, m_impl->x, 1)))
                return false;
            break;
        }
        if (!applyPreconditioner(m_impl->s, m_impl->sHat) || !multiply(m_impl->vectorSHat, m_impl->vectorT))
            return false;

        double ts = 0.0;
        double tt = 0.0;
        if (!IsSuccess(cublasDdot(m_impl->blas, n, m_impl->t, 1, m_impl->s, 1, &ts)) ||
            !IsSuccess(cublasDdot(m_impl->blas, n, m_impl->t, 1, m_impl->t, 1, &tt)) || std::abs(tt) < 1.0e-300)
            return false;
        omega = ts / tt;
        const double negativeOmega = -omega;
        if (!IsSuccess(cublasDaxpy(m_impl->blas, n, &alpha, m_impl->pHat, 1, m_impl->x, 1)) ||
            !IsSuccess(cublasDaxpy(m_impl->blas, n, &omega, m_impl->sHat, 1, m_impl->x, 1)) ||
            !IsSuccess(cudaMemcpy(m_impl->r, m_impl->s, vectorBytes, cudaMemcpyDeviceToDevice)) ||
            !IsSuccess(cublasDaxpy(m_impl->blas, n, &negativeOmega, m_impl->t, 1, m_impl->r, 1)))
            return false;

        double normR = 0.0;
        if (!IsSuccess(cublasDnrm2(m_impl->blas, n, m_impl->r, 1, &normR)))
            return false;
        if (normR <= tolerance)
            break;
        if (std::abs(omega) < 1.0e-300)
            return false;
        rhoOld = rho;
        if (iteration + 1 == maximumIterations)
            return false;
    }

    solution.resize(n);
    return IsSuccess(cudaMemcpy(solution.data(), m_impl->x, vectorBytes, cudaMemcpyDeviceToHost)) &&
           solution.allFinite();
}
}
