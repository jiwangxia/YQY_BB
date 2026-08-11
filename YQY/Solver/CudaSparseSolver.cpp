#include "Solver/CudaSparseSolver.h"

#include <cublas_v2.h>
#include <cuda_runtime_api.h>
#include <cusparse.h>

#include <Eigen/SparseCore>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
bool IsSuccess(cudaError_t status) { return status == cudaSuccess; }
bool IsSuccess(cublasStatus_t status) { return status == CUBLAS_STATUS_SUCCESS; }
bool IsSuccess(cusparseStatus_t status) { return status == CUSPARSE_STATUS_SUCCESS; }
template <typename T>
bool Allocate(T** pointer, std::size_t bytes)
{
    return IsSuccess(cudaMalloc(reinterpret_cast<void**>(pointer), bytes));
}
}

namespace SolverNameSpace
{
bool CudaSparseSolver::Solve(const SpMat& matrix, const Vec& rhs, Vec& solution,
    double relativeTolerance, int maximumIterations) const
{
    if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() || rhs.size() == 0)
        return false;

    using RowMajorMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor, int>;
    RowMajorMatrix rowMatrix(matrix);
    rowMatrix.makeCompressed();
    const int n = static_cast<int>(rhs.size());
    const int nnz = static_cast<int>(rowMatrix.nonZeros());
    if (maximumIterations <= 0)
        maximumIterations = std::max(200, std::min(10000, n * 2));

    cudaError_t cudaStatus = cudaSetDevice(0);
    if (!IsSuccess(cudaStatus))
        return false;
    cublasHandle_t blas = nullptr;
    cusparseHandle_t sparse = nullptr;
    cusparseSpMatDescr_t sparseMatrix = nullptr;
    cusparseDnVecDescr_t vectorX = nullptr, vectorP = nullptr, vectorS = nullptr;
    cusparseDnVecDescr_t vectorV = nullptr, vectorT = nullptr;
    double* values = nullptr;
    int* rows = nullptr;
    int* columns = nullptr;
    double* x = nullptr; double* r = nullptr; double* rHat = nullptr;
    double* p = nullptr; double* v = nullptr; double* s = nullptr; double* t = nullptr;
    void* spmvBuffer = nullptr;
    auto release = [&]()
    {
        if (spmvBuffer) cudaFree(spmvBuffer);
        if (vectorT) cusparseDestroyDnVec(vectorT);
        if (vectorV) cusparseDestroyDnVec(vectorV);
        if (vectorS) cusparseDestroyDnVec(vectorS);
        if (vectorP) cusparseDestroyDnVec(vectorP);
        if (vectorX) cusparseDestroyDnVec(vectorX);
        if (sparseMatrix) cusparseDestroySpMat(sparseMatrix);
        if (t) cudaFree(t); if (s) cudaFree(s); if (v) cudaFree(v); if (p) cudaFree(p);
        if (rHat) cudaFree(rHat); if (r) cudaFree(r); if (x) cudaFree(x);
        if (columns) cudaFree(columns); if (rows) cudaFree(rows); if (values) cudaFree(values);
        if (sparse) cusparseDestroy(sparse);
        if (blas) cublasDestroy(blas);
    };
    auto failed = [&]() { release(); return false; };
    const std::size_t valueBytes = sizeof(double) * static_cast<std::size_t>(nnz);
    const std::size_t rowBytes = sizeof(int) * static_cast<std::size_t>(n + 1);
    const std::size_t vectorBytes = sizeof(double) * static_cast<std::size_t>(n);
    if (!IsSuccess(cublasCreate(&blas)) || !IsSuccess(cusparseCreate(&sparse))
        || !Allocate(&values, valueBytes) || !Allocate(&rows, rowBytes)
        || !Allocate(&columns, sizeof(int) * static_cast<std::size_t>(nnz))
        || !IsSuccess(cudaMemcpy(values, rowMatrix.valuePtr(), valueBytes, cudaMemcpyHostToDevice))
        || !IsSuccess(cudaMemcpy(rows, rowMatrix.outerIndexPtr(), rowBytes, cudaMemcpyHostToDevice))
        || !IsSuccess(cudaMemcpy(columns, rowMatrix.innerIndexPtr(), sizeof(int) * static_cast<std::size_t>(nnz), cudaMemcpyHostToDevice)))
        return failed();
    if (!Allocate(&x, vectorBytes) || !Allocate(&r, vectorBytes)
        || !Allocate(&rHat, vectorBytes) || !Allocate(&p, vectorBytes)
        || !Allocate(&v, vectorBytes) || !Allocate(&s, vectorBytes)
        || !Allocate(&t, vectorBytes)) return failed();
    if (!IsSuccess(cudaMemset(x, 0, vectorBytes)) || !IsSuccess(cudaMemcpy(r, rhs.data(), vectorBytes, cudaMemcpyHostToDevice))
        || !IsSuccess(cudaMemcpy(rHat, rhs.data(), vectorBytes, cudaMemcpyHostToDevice))
        || !IsSuccess(cudaMemset(p, 0, vectorBytes)) || !IsSuccess(cudaMemset(v, 0, vectorBytes))) return failed();
    if (!IsSuccess(cusparseCreateCsr(&sparseMatrix, n, n, nnz, rows, columns, values,
            CUSPARSE_INDEX_32I, CUSPARSE_INDEX_32I, CUSPARSE_INDEX_BASE_ZERO, CUDA_R_64F))
        || !IsSuccess(cusparseCreateDnVec(&vectorX, n, x, CUDA_R_64F))
        || !IsSuccess(cusparseCreateDnVec(&vectorP, n, p, CUDA_R_64F))
        || !IsSuccess(cusparseCreateDnVec(&vectorS, n, s, CUDA_R_64F))
        || !IsSuccess(cusparseCreateDnVec(&vectorV, n, v, CUDA_R_64F))
        || !IsSuccess(cusparseCreateDnVec(&vectorT, n, t, CUDA_R_64F))) return failed();
    const double one = 1.0, zero = 0.0;
    std::size_t bufferSize = 0;
    if (!IsSuccess(cusparseSpMV_bufferSize(sparse, CUSPARSE_OPERATION_NON_TRANSPOSE, &one,
            sparseMatrix, vectorP, &zero, vectorV, CUDA_R_64F, CUSPARSE_SPMV_ALG_DEFAULT, &bufferSize))
        || !IsSuccess(cudaMalloc(&spmvBuffer, bufferSize))) return failed();
    const auto multiply = [&](cusparseDnVecDescr_t input, cusparseDnVecDescr_t output)
    {
        return IsSuccess(cusparseSpMV(sparse, CUSPARSE_OPERATION_NON_TRANSPOSE, &one,
            sparseMatrix, input, &zero, output, CUDA_R_64F, CUSPARSE_SPMV_ALG_DEFAULT, spmvBuffer));
    };
    double normB = 0.0;
    if (!IsSuccess(cublasDnrm2(blas, n, r, 1, &normB))) return failed();
    if (normB == 0.0) { solution.setZero(n); release(); return true; }
    const double tolerance = relativeTolerance * normB;
    double rhoOld = 1.0, alpha = 1.0, omega = 1.0;
    for (int iteration = 0; iteration < maximumIterations; ++iteration)
    {
        double rho = 0.0;
        if (!IsSuccess(cublasDdot(blas, n, rHat, 1, r, 1, &rho)) || std::abs(rho) < 1.0e-300)
            return failed();
        const double beta = (rho / rhoOld) * (alpha / omega);
        const double negativePreviousOmega = -omega;
        if (!IsSuccess(cublasDaxpy(blas, n, &negativePreviousOmega, v, 1, p, 1))
            || !IsSuccess(cublasDscal(blas, n, &beta, p, 1))
            || !IsSuccess(cublasDaxpy(blas, n, &one, r, 1, p, 1)) || !multiply(vectorP, vectorV)) return failed();
        double rHatV = 0.0;
        if (!IsSuccess(cublasDdot(blas, n, rHat, 1, v, 1, &rHatV)) || std::abs(rHatV) < 1.0e-300) return failed();
        alpha = rho / rHatV;
        const double negativeAlpha = -alpha;
        if (!IsSuccess(cudaMemcpy(s, r, vectorBytes, cudaMemcpyDeviceToDevice))
            || !IsSuccess(cublasDaxpy(blas, n, &negativeAlpha, v, 1, s, 1))) return failed();
        double normS = 0.0;
        if (!IsSuccess(cublasDnrm2(blas, n, s, 1, &normS))) return failed();
        if (normS <= tolerance)
        {
            if (!IsSuccess(cublasDaxpy(blas, n, &alpha, p, 1, x, 1))) return failed();
            break;
        }
        if (!multiply(vectorS, vectorT)) return failed();
        double ts = 0.0, tt = 0.0;
        if (!IsSuccess(cublasDdot(blas, n, t, 1, s, 1, &ts)) || !IsSuccess(cublasDdot(blas, n, t, 1, t, 1, &tt)) || std::abs(tt) < 1.0e-300) return failed();
        omega = ts / tt;
        const double negativeOmega = -omega;
        if (!IsSuccess(cublasDaxpy(blas, n, &alpha, p, 1, x, 1))
            || !IsSuccess(cublasDaxpy(blas, n, &omega, s, 1, x, 1))
            || !IsSuccess(cudaMemcpy(r, s, vectorBytes, cudaMemcpyDeviceToDevice))
            || !IsSuccess(cublasDaxpy(blas, n, &negativeOmega, t, 1, r, 1))) return failed();
        double normR = 0.0;
        if (!IsSuccess(cublasDnrm2(blas, n, r, 1, &normR))) return failed();
        if (normR <= tolerance) break;
        if (std::abs(omega) < 1.0e-300) return failed();
        rhoOld = rho;
        if (iteration + 1 == maximumIterations) return failed();
    }
    solution.resize(n);
    if (!IsSuccess(cudaMemcpy(solution.data(), x, vectorBytes, cudaMemcpyDeviceToHost)) || !solution.allFinite()) return failed();
    release();
    return true;
}
}
