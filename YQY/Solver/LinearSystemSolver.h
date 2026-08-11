#pragma once

#include "Solver/Interface/IAnalysisModel.h"
#include "Solver/CudaSparseSolver.h"
#include "Solver/GpuSettings.h"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

#include <cstdint>

namespace SolverNameSpace
{
    // Shared direct-solver backend. Additional implementations (for example
    // Pardiso) can later retain this public contract without changing solvers.
    class LinearSystemSolver
    {
    public:
        void Reset()
        {
            preferLdlt_ = true;
            ldltPatternValid_ = false;
            luPatternValid_ = false;
        }

        void SetPreferLdlt(bool enabled)
        {
            preferLdlt_ = enabled;
            if (!enabled)
                luPatternValid_ = false;
        }

        bool Solve(const SpMat& matrix, const Vec& rhs, Vec& solution)
        {
            if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size())
                return false;

            GpuSettings::ObserveMatrixDofs(static_cast<int>(matrix.rows()));
            if (GpuSettings::IsEnabled()
                && matrix.rows() >= GpuSettings::MinimumGpuDofs)
            {
                GpuSettings::RecordAttempt();
                if (gpuSolver_.Solve(matrix, rhs, solution))
                {
                    const double residual = (matrix * solution - rhs).norm();
                    const double scale = std::max(rhs.norm(), 1.0);
                    if (std::isfinite(residual) && residual <= 1.0e-8 * scale)
                    {
                        GpuSettings::RecordSuccess();
                        return true;
                    }
                }
                GpuSettings::RecordFallback();
            }

            const std::uint64_t pattern = PatternFingerprint(matrix);
            if (preferLdlt_)
            {
                if (!ldltPatternValid_ || ldltPattern_ != pattern)
                {
                    ldlt_.analyzePattern(matrix);
                    ldltPattern_ = pattern;
                    ldltPatternValid_ = true;
                }
                ldlt_.factorize(matrix);
                if (ldlt_.info() == Eigen::Success)
                {
                    solution = ldlt_.solve(rhs);
                    if (ldlt_.info() == Eigen::Success && solution.allFinite())
                        return true;
                }

                preferLdlt_ = false;
                luPatternValid_ = false;
            }

            if (!luPatternValid_ || luPattern_ != pattern)
            {
                lu_.analyzePattern(matrix);
                luPattern_ = pattern;
                luPatternValid_ = true;
            }
            lu_.factorize(matrix);
            if (lu_.info() != Eigen::Success)
            {
                luPatternValid_ = false;
                return false;
            }

            solution = lu_.solve(rhs);
            if (lu_.info() != Eigen::Success || !solution.allFinite())
            {
                luPatternValid_ = false;
                return false;
            }
            return true;
        }

    private:
        static std::uint64_t PatternFingerprint(const SpMat& matrix)
        {
            // Size and nnz alone are insufficient: MPC can change nonzero
            // locations while preserving both values.
            std::uint64_t hash = 1469598103934665603ull;
            const auto append = [&hash](const void* bytes, std::size_t size)
            {
                const auto* data = static_cast<const unsigned char*>(bytes);
                for (std::size_t i = 0; i < size; ++i)
                {
                    hash ^= data[i];
                    hash *= 1099511628211ull;
                }
            };

            const Eigen::Index rows = matrix.rows();
            const Eigen::Index cols = matrix.cols();
            const Eigen::Index nonZeros = matrix.nonZeros();
            append(&rows, sizeof(rows));
            append(&cols, sizeof(cols));
            append(&nonZeros, sizeof(nonZeros));
            append(matrix.outerIndexPtr(),
                sizeof(SpMat::StorageIndex)
                * static_cast<std::size_t>(matrix.outerSize() + 1));
            append(matrix.innerIndexPtr(),
                sizeof(SpMat::StorageIndex)
                * static_cast<std::size_t>(nonZeros));
            return hash;
        }

        Eigen::SimplicialLDLT<SpMat> ldlt_;
        Eigen::SparseLU<SpMat> lu_;
        CudaSparseSolver gpuSolver_;
        bool preferLdlt_ = true;
        bool ldltPatternValid_ = false;
        bool luPatternValid_ = false;
        std::uint64_t ldltPattern_ = 0;
        std::uint64_t luPattern_ = 0;
    };
}
