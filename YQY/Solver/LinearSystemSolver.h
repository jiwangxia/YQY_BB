#pragma once

#include "Solver/Interface/IAnalysisModel.h"
#include "Solver/CudssSolver.h"
#include "Solver/CudaSparseSolver.h"
#include "Solver/GpuSettings.h"
#include "Solver/LinearSolverSettings.h"
#include "Solver/PardisoSolver.h"

#include <Eigen/LU>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace SolverNameSpace
{
// 统一的稀疏直接求解接口，后续增加 PARDISO 等实现时不需要修改上层求解器。
class LinearSystemSolver
{
public:
    void Reset()
    {
        preferLdlt_ = true;
        ldltPatternValid_ = false;
        luPatternValid_ = false;
        automaticBackend_ = AutomaticBackend::Uncalibrated;
        automaticPattern_ = 0;
    }

    void SetPreferLdlt(bool enabled)
    {
        preferLdlt_ = enabled;
        if (!enabled)
        {
            luPatternValid_ = false;
            automaticBackend_ = AutomaticBackend::Uncalibrated;
        }
    }

    bool Solve(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution)
    {
        if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size())
            return false;

        const LinearSolverMode mode = LinearSolverSettings::Mode();
        GpuSettings::ObserveMatrixDofs(static_cast<int>(matrix.rows()));
        const std::uint64_t pattern = PatternFingerprint(matrix);
        if (mode == LinearSolverMode::CudaIterative)
            return gpuSolver_.Solve(matrix, rhs, solution) && IsSolutionValid(matrix, rhs, solution);
        if (mode == LinearSolverMode::Cudss)
            return cudssSolver_.Solve(matrix, rhs, solution) && IsSolutionValid(matrix, rhs, solution);
        if (mode == LinearSolverMode::Pardiso)
            return pardisoSolver_.Solve(matrix, rhs, preferLdlt_, pattern, solution) &&
                   IsSolutionValid(matrix, rhs, solution);

        if (mode == LinearSolverMode::Automatic && GpuSettings::IsEnabled() &&
            matrix.rows() >= GpuSettings::MinimumGpuDofs)
        {
            GpuSettings::RecordAttempt();
            if (cudssSolver_.Solve(matrix, rhs, solution) && IsSolutionValid(matrix, rhs, solution))
            {
                GpuSettings::RecordSuccess();
                return true;
            }
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

        if (mode == LinearSolverMode::Ldlt)
            return SolveLdlt(matrix, rhs, solution, pattern);
        if (mode == LinearSolverMode::Lu)
            return SolveLu(matrix, rhs, solution, pattern);

        if (automaticPattern_ != pattern)
        {
            automaticPattern_ = pattern;
            automaticBackend_ = AutomaticBackend::Uncalibrated;
        }

        if (automaticBackend_ == AutomaticBackend::Ldlt)
        {
            if (SolveLdlt(matrix, rhs, solution, pattern) && IsSolutionValid(matrix, rhs, solution))
                return true;
            automaticBackend_ = AutomaticBackend::Uncalibrated;
        }
        if (automaticBackend_ == AutomaticBackend::Lu)
        {
            if (SolveLu(matrix, rhs, solution, pattern) && IsSolutionValid(matrix, rhs, solution))
                return true;
            automaticBackend_ = AutomaticBackend::Uncalibrated;
        }
        if (automaticBackend_ == AutomaticBackend::Pardiso)
        {
            if (pardisoSolver_.Solve(matrix, rhs, preferLdlt_, pattern, solution) &&
                IsSolutionValid(matrix, rhs, solution))
                return true;
            automaticBackend_ = AutomaticBackend::Uncalibrated;
        }

        return CalibrateAutomaticBackend(matrix, rhs, solution, pattern);
    }

    bool SolveLowRank(const SpMat& matrix, const Eigen::MatrixXd& factor, double scale, const Vec& rhs,
                      _OUT Vec& solution)
    {
        if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() || factor.rows() != matrix.rows() ||
            !factor.allFinite() || !std::isfinite(scale))
            return false;
        if (factor.cols() == 0 || scale == 0.0)
            return Solve(matrix, rhs, solution);

        const LinearSolverMode mode = LinearSolverSettings::Mode();
        const std::uint64_t pattern = PatternFingerprint(matrix);
        if (mode == LinearSolverMode::Ldlt &&
            SolveLowRankWoodbury(matrix, factor, scale, rhs, pattern, AutomaticBackend::Ldlt, solution))
            return true;
        if (mode == LinearSolverMode::Lu &&
            SolveLowRankWoodbury(matrix, factor, scale, rhs, pattern, AutomaticBackend::Lu, solution))
            return true;
        if (mode == LinearSolverMode::Pardiso &&
            SolveLowRankWoodbury(matrix, factor, scale, rhs, pattern, AutomaticBackend::Pardiso, solution))
            return true;
        if (mode == LinearSolverMode::Automatic &&
            !(GpuSettings::IsEnabled() && matrix.rows() >= GpuSettings::MinimumGpuDofs))
        {
            if (automaticPattern_ != pattern || automaticBackend_ == AutomaticBackend::Uncalibrated)
            {
                Vec calibrationSolution;
                if (!Solve(matrix, rhs, calibrationSolution))
                    return false;
            }
            if (SolveLowRankWoodbury(matrix, factor, scale, rhs, pattern, automaticBackend_, solution))
                return true;
        }

        return SolveLowRankAugmented(matrix, factor, scale, rhs, solution);
    }

private:
    enum class AutomaticBackend
    {
        Uncalibrated,
        Ldlt,
        Lu,
        Pardiso
    };

    bool SolveLowRankAugmented(const SpMat& matrix, const Eigen::MatrixXd& factor, double scale, const Vec& rhs,
                               _OUT Vec& solution)
    {
        std::vector<int> activeColumns;
        activeColumns.reserve(factor.cols());
        for (int column = 0; column < factor.cols(); ++column)
            if (factor.col(column).squaredNorm() > 0.0)
                activeColumns.push_back(column);
        if (activeColumns.empty())
            return Solve(matrix, rhs, solution);

        const int dimension = static_cast<int>(matrix.rows());
        const int rank = static_cast<int>(activeColumns.size());
        const double factorScale = std::sqrt(std::abs(scale));
        const double sign = scale > 0.0 ? 1.0 : -1.0;
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(static_cast<std::size_t>(matrix.nonZeros()) +
                         2 * static_cast<std::size_t>(dimension) * rank + rank);
        for (int outer = 0; outer < matrix.outerSize(); ++outer)
            for (SpMat::InnerIterator entry(matrix, outer); entry; ++entry)
                triplets.emplace_back(entry.row(), entry.col(), entry.value());
        for (int lowRankColumn = 0; lowRankColumn < rank; ++lowRankColumn)
        {
            const int sourceColumn = activeColumns[lowRankColumn];
            const int augmentedDof = dimension + lowRankColumn;
            for (int row = 0; row < dimension; ++row)
            {
                const double value = factorScale * factor(row, sourceColumn);
                if (value == 0.0)
                    continue;
                triplets.emplace_back(row, augmentedDof, value);
                triplets.emplace_back(augmentedDof, row, value);
            }
            triplets.emplace_back(augmentedDof, augmentedDof, -sign);
        }

        SpMat augmentedMatrix(dimension + rank, dimension + rank);
        augmentedMatrix.setFromTriplets(triplets.begin(), triplets.end());
        Vec augmentedRhs = Vec::Zero(dimension + rank);
        augmentedRhs.head(dimension) = rhs;
        Vec augmentedSolution;
        if (!Solve(augmentedMatrix, augmentedRhs, augmentedSolution))
            return false;
        solution = augmentedSolution.head(dimension);
        const Vec residual = matrix * solution + scale * factor * (factor.transpose() * solution) - rhs;
        return solution.allFinite() && residual.allFinite() && residual.norm() <= 1.0e-8 * std::max(1.0, rhs.norm());
    }

    bool SolveLowRankWoodbury(const SpMat& matrix, const Eigen::MatrixXd& factor, double scale, const Vec& rhs,
                              std::uint64_t pattern, AutomaticBackend backend, _OUT Vec& solution)
    {
        Eigen::MatrixXd multipleRhs(matrix.rows(), factor.cols() + 1);
        multipleRhs.col(0) = rhs;
        multipleRhs.rightCols(factor.cols()) = factor;

        Eigen::MatrixXd multipleSolution;
        bool solved = false;
        switch (backend)
        {
        case AutomaticBackend::Ldlt:
            PrepareLdltPattern(matrix, pattern);
            solved = FactorizeAndSolveLdlt(matrix, multipleRhs, multipleSolution);
            break;
        case AutomaticBackend::Lu:
            PrepareLuPattern(matrix, pattern);
            solved = FactorizeAndSolveLu(matrix, multipleRhs, multipleSolution);
            break;
        case AutomaticBackend::Pardiso:
            solved = pardisoSolver_.Solve(matrix, multipleRhs, preferLdlt_, pattern, multipleSolution);
            break;
        case AutomaticBackend::Uncalibrated:
            return false;
        }
        if (!solved || multipleSolution.rows() != matrix.rows() || multipleSolution.cols() != factor.cols() + 1)
            return false;

        const Vec baseSolution = multipleSolution.col(0);
        const Eigen::MatrixXd inverseFactor = multipleSolution.rightCols(factor.cols());
        const Eigen::MatrixXd correctionMatrix =
            Eigen::MatrixXd::Identity(factor.cols(), factor.cols()) + scale * factor.transpose() * inverseFactor;
        Eigen::FullPivLU<Eigen::MatrixXd> correctionSolver(correctionMatrix);
        if (!correctionSolver.isInvertible())
            return false;
        const Vec correction = correctionSolver.solve(factor.transpose() * baseSolution);
        if (!correction.allFinite())
            return false;

        solution = baseSolution - scale * inverseFactor * correction;
        const Vec residual = matrix * solution + scale * factor * (factor.transpose() * solution) - rhs;
        return solution.allFinite() && residual.allFinite() && residual.norm() <= 1.0e-8 * std::max(1.0, rhs.norm());
    }

    bool CalibrateAutomaticBackend(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, std::uint64_t pattern)
    {
        if (preferLdlt_)
            PrepareLdltPattern(matrix, pattern);
        PrepareLuPattern(matrix, pattern);

        Vec ldltSolution;
        const auto ldltStart = std::chrono::steady_clock::now();
        const bool ldltValid = preferLdlt_ && FactorizeAndSolveLdlt(matrix, rhs, ldltSolution) &&
                               IsSolutionValid(matrix, rhs, ldltSolution);
        const auto ldltEnd = std::chrono::steady_clock::now();

        Vec luSolution;
        const auto luStart = std::chrono::steady_clock::now();
        const bool luValid = FactorizeAndSolveLu(matrix, rhs, luSolution) && IsSolutionValid(matrix, rhs, luSolution);
        const auto luEnd = std::chrono::steady_clock::now();

        Vec pardisoSolution;
        Vec pardisoWarmupSolution;
        const bool pardisoPrepared = pardisoSolver_.Solve(matrix, rhs, preferLdlt_, pattern, pardisoWarmupSolution) &&
                                     IsSolutionValid(matrix, rhs, pardisoWarmupSolution);
        const auto pardisoStart = std::chrono::steady_clock::now();
        const bool pardisoValid = pardisoPrepared &&
                                  pardisoSolver_.Solve(matrix, rhs, preferLdlt_, pattern, pardisoSolution) &&
                                  IsSolutionValid(matrix, rhs, pardisoSolution);
        const auto pardisoEnd = std::chrono::steady_clock::now();

        if (!ldltValid && !luValid && !pardisoValid)
            return false;
        const auto ldltDuration = ldltEnd - ldltStart;
        const auto luDuration = luEnd - luStart;
        const auto pardisoDuration = pardisoEnd - pardisoStart;
        if (pardisoValid && (!ldltValid || pardisoDuration <= ldltDuration) &&
            (!luValid || pardisoDuration <= luDuration))
        {
            automaticBackend_ = AutomaticBackend::Pardiso;
            solution = std::move(pardisoSolution);
        }
        else if (ldltValid && (!luValid || ldltDuration <= luDuration))
        {
            automaticBackend_ = AutomaticBackend::Ldlt;
            solution = std::move(ldltSolution);
        }
        else
        {
            automaticBackend_ = AutomaticBackend::Lu;
            solution = std::move(luSolution);
        }
        return true;
    }

    static bool IsSolutionValid(const SpMat& matrix, const Vec& rhs, const Vec& solution)
    {
        if (!solution.allFinite())
            return false;
        const double residual = (matrix * solution - rhs).norm();
        const double scale = std::max(rhs.norm(), 1.0);
        return std::isfinite(residual) && residual <= 1.0e-8 * scale;
    }

    bool SolveLdlt(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, std::uint64_t pattern)
    {
        PrepareLdltPattern(matrix, pattern);
        return FactorizeAndSolveLdlt(matrix, rhs, solution);
    }

    void PrepareLdltPattern(const SpMat& matrix, std::uint64_t pattern)
    {
        if (ldltPatternValid_ && ldltPattern_ == pattern)
            return;
        ldlt_.analyzePattern(matrix);
        ldltPattern_ = pattern;
        ldltPatternValid_ = true;
    }

    bool FactorizeAndSolveLdlt(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution)
    {
        ldlt_.factorize(matrix);
        if (ldlt_.info() != Eigen::Success)
            return false;

        solution = ldlt_.solve(rhs);
        return ldlt_.info() == Eigen::Success && solution.allFinite();
    }

    bool FactorizeAndSolveLdlt(const SpMat& matrix, const Eigen::MatrixXd& rhs, _OUT Eigen::MatrixXd& solution)
    {
        ldlt_.factorize(matrix);
        if (ldlt_.info() != Eigen::Success)
            return false;
        solution = ldlt_.solve(rhs);
        return ldlt_.info() == Eigen::Success && solution.allFinite();
    }

    bool SolveLu(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, std::uint64_t pattern)
    {
        PrepareLuPattern(matrix, pattern);
        return FactorizeAndSolveLu(matrix, rhs, solution);
    }

    void PrepareLuPattern(const SpMat& matrix, std::uint64_t pattern)
    {
        if (luPatternValid_ && luPattern_ == pattern)
            return;
        lu_.analyzePattern(matrix);
        luPattern_ = pattern;
        luPatternValid_ = true;
    }

    bool FactorizeAndSolveLu(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution)
    {
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

    bool FactorizeAndSolveLu(const SpMat& matrix, const Eigen::MatrixXd& rhs, _OUT Eigen::MatrixXd& solution)
    {
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
    static std::uint64_t PatternFingerprint(const SpMat& matrix)
    {
        // 仅比较矩阵尺寸和非零数量不够，MPC 可能保持两者不变但改变非零位置。
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
        append(matrix.outerIndexPtr(), sizeof(SpMat::StorageIndex) * static_cast<std::size_t>(matrix.outerSize() + 1));
        append(matrix.innerIndexPtr(), sizeof(SpMat::StorageIndex) * static_cast<std::size_t>(nonZeros));
        return hash;
    }

    Eigen::SimplicialLDLT<SpMat> ldlt_;
    Eigen::SparseLU<SpMat> lu_;
    PardisoSolver pardisoSolver_;
    CudaSparseSolver gpuSolver_;
    CudssSolver cudssSolver_;
    bool preferLdlt_ = true;
    bool ldltPatternValid_ = false;
    bool luPatternValid_ = false;
    AutomaticBackend automaticBackend_ = AutomaticBackend::Uncalibrated;
    std::uint64_t ldltPattern_ = 0;
    std::uint64_t luPattern_ = 0;
    std::uint64_t automaticPattern_ = 0;
};
}
