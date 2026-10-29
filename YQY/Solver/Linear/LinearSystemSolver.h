#pragma once

#include "Solver/Interface/IAnalysisModel.h"
#include "Gpu/CudssSolver.h"
#include "Gpu/CudaSparseSolver.h"
#include "Gpu/GpuSettings.h"
#include "LinearSolverSettings.h"
#include "Direct/PardisoSolver.h"

#include <Eigen/LU>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <mkl_lapacke.h>
#include <mkl_service.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace SolverNameSpace
{
// 统一的稀疏直接求解接口，后续增加 PARDISO 等实现时不需要修改上层求解器。
class LinearSystemSolver
{
public:
    void Reset()
    {
        modeOverride_.reset();
        preferLdlt_ = true;
        ldltPatternValid_ = false;
        luPatternValid_ = false;
        denseLuFactorValid_ = false;
        denseLuFactors_.resize(0, 0);
        denseLuPivots_.clear();
        automaticBackend_ = AutomaticBackend::Uncalibrated;
        automaticPattern_ = 0;
        automaticGpuPattern_ = 0;
        automaticGpuSamples_ = 0;
        automaticGpuPreferCpu_ = false;
        pardisoLuPatternValid_ = false;
        cachedDensePrimalFactor_.resize(0, 0);
        cachedDensePrimalGram_.resize(0, 0);
        signedSource_.clear();
        canonicalTargetOfSource_.clear();
        canonicalSigns_.clear();
        canonicalPatternValid_ = false;
        canonicalMatrix_.resize(0, 0);
        canonicalRhs_.resize(0);
        canonicalFactorSource_.resize(0, 0);
        canonicalFactor_.resize(0, 0);
        canonicalSourceValueIndices_.clear();
        canonicalValueSigns_.clear();
    }

    void SetPreferLdlt(bool enabled)
    {
        if (preferLdlt_ == enabled)
            return;
        preferLdlt_ = enabled;
        luPatternValid_ = false;
        automaticBackend_ = AutomaticBackend::Uncalibrated;
        pardisoLuPatternValid_ = false;
    }

    void SetAllowGpuAutomatic(bool enabled)
    {
        allowGpuAutomatic_ = enabled;
    }

    // 实例覆盖不改进程设置；各后端保留模式缓存，每次重新数值分解。
    void SetModeOverride(LinearSolverMode mode)
    {
        modeOverride_ = mode;
    }

    // 仅配置到参考线路实例；编号为目标参考自由 DOF 对应的有符号源 DOF（从 1 开始）。
    bool SetSignedPermutation(const std::vector<int>& signedSource)
    {
        std::vector<int> targetOfSource(signedSource.size(), -1);
        std::vector<double> signs(signedSource.size());
        for (std::size_t target = 0; target < signedSource.size(); ++target)
        {
            const std::int64_t absoluteSource = std::abs(static_cast<std::int64_t>(signedSource[target]));
            if (absoluteSource < 1 || absoluteSource > static_cast<std::int64_t>(signedSource.size()))
                return false;
            const std::size_t source = static_cast<std::size_t>(absoluteSource - 1);
            if (targetOfSource[source] >= 0)
                return false;
            targetOfSource[source] = static_cast<int>(target);
            signs[target] = signedSource[target] > 0 ? 1.0 : -1.0;
        }
        signedSource_ = signedSource;
        canonicalTargetOfSource_ = std::move(targetOfSource);
        canonicalSigns_ = std::move(signs);
        canonicalFactorSource_.resize(0, 0);
        canonicalFactor_.resize(0, 0);
        canonicalPatternValid_ = false;
        luPatternValid_ = false;
        denseLuFactorValid_ = false;
        ldltPatternValid_ = false;
        pardisoLuPatternValid_ = false;
        automaticBackend_ = AutomaticBackend::Uncalibrated;
        return true;
    }

    bool Solve(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, std::uint64_t patternVersion = 0)
    {
        if (signedSource_.empty())
            return SolveUnpermuted(matrix, rhs, solution, patternVersion);
        if (!PrepareCanonicalSystem(matrix, rhs, patternVersion))
            return false;
        Vec canonicalSolution;
        if (!SolveUnpermuted(canonicalMatrix_, canonicalRhs_, canonicalSolution, canonicalPattern_))
            return false;
        RecoverCanonicalSolution(canonicalSolution, solution);
        return true;
    }

    // 参考静力的高刚度梁索系统在残差相消时，用同一次 LU 分解修正解，并保留原有验收阈值。
    bool SolveRefined(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, std::uint64_t patternVersion = 0)
    {
        if (EffectiveMode() != LinearSolverMode::Lu)
            return Solve(matrix, rhs, solution, patternVersion);
        if (signedSource_.empty())
            return SolveLuRefined(matrix, rhs, solution, ResolvePatternFingerprint(matrix, patternVersion));
        if (!PrepareCanonicalSystem(matrix, rhs, patternVersion))
            return false;
        Vec canonicalSolution;
        if (!SolveLuRefined(canonicalMatrix_, canonicalRhs_, canonicalSolution, canonicalPattern_))
            return false;
        RecoverCanonicalSolution(canonicalSolution, solution);
        return true;
    }

    bool SolveDeviceCscValues(const SpMat& matrix, const double* deviceValues, int valueCount, const Vec& rhs,
                              _OUT Vec& solution, std::uint64_t patternVersion = 0)
    {
        if (!deviceValues || valueCount != matrix.nonZeros() || matrix.rows() != matrix.cols() ||
            matrix.rows() != rhs.size())
            return false;
        if (!signedSource_.empty())
            return false;
        GpuSettings::ObserveMatrixDofs(static_cast<int>(matrix.rows()));
        const LinearSolverMode mode = EffectiveMode();
        const std::uint64_t pattern = ResolvePatternFingerprint(matrix, patternVersion);
        if (mode == LinearSolverMode::Automatic)
            UpdateAutomaticGpuPattern(pattern);
        if (mode == LinearSolverMode::Cudss ||
            (mode == LinearSolverMode::Automatic && allowGpuAutomatic_ && !automaticGpuPreferCpu_ &&
             GpuSettings::IsEnabled() &&
             matrix.rows() >= GpuSettings::MinimumGpuDofs()))
        {
            const auto gpuStart = std::chrono::steady_clock::now();
            if (TryCudssDeviceCscValues(matrix, deviceValues, valueCount, rhs, solution))
            {
                if (mode == LinearSolverMode::Automatic)
                    EvaluateAutomaticGpuTiming(matrix, rhs, solution, pattern, gpuStart);
                return true;
            }
            GpuSettings::RecordFallback(QStringLiteral("GPU 设备端 CSC 求解失败，已回退 CPU"));
            if (mode == LinearSolverMode::Automatic)
                automaticGpuPreferCpu_ = true;
        }
        return Solve(matrix, rhs, solution, patternVersion);
    }

    bool SolveLowRank(const SpMat& matrix, const Eigen::MatrixXd& factor, double scale, const Vec& rhs,
                      _OUT Vec& solution, std::uint64_t patternVersion = 0)
    {
        if (signedSource_.empty())
            return SolveLowRankUnpermuted(matrix, factor, scale, rhs, solution, patternVersion);
        if (factor.rows() != matrix.rows() || !factor.allFinite() || !std::isfinite(scale))
            return false;
        if (factor.cols() == 0 || scale == 0.0)
            return Solve(matrix, rhs, solution, patternVersion);
        if (!PrepareCanonicalSystem(matrix, rhs, patternVersion))
            return false;
        // 固定模态阻尼只重排一次；约束或阻尼因子改变时重新生成。
        if (canonicalFactorSource_.rows() != factor.rows() || canonicalFactorSource_.cols() != factor.cols() ||
            !(canonicalFactorSource_.array() == factor.array()).all())
        {
            canonicalFactorSource_ = factor;
            canonicalFactor_.resize(factor.rows(), factor.cols());
            for (Eigen::Index target = 0; target < factor.rows(); ++target)
            {
                const int source = static_cast<int>(std::abs(static_cast<std::int64_t>(signedSource_[target])) - 1);
                canonicalFactor_.row(target) = canonicalSigns_[target] * factor.row(source);
            }
        }
        Vec canonicalSolution;
        if (!SolveLowRankUnpermuted(canonicalMatrix_, canonicalFactor_, scale, canonicalRhs_, canonicalSolution,
                                   canonicalPattern_))
            return false;
        RecoverCanonicalSolution(canonicalSolution, solution);
        return true;
    }

private:
    bool PrepareCanonicalSystem(const SpMat& matrix, const Vec& rhs, std::uint64_t patternVersion)
    {
        if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() ||
            matrix.rows() != static_cast<Eigen::Index>(signedSource_.size()))
            return false;
        SpMat compressed;
        const SpMat* input = &matrix;
        if (!matrix.isCompressed())
        {
            compressed = matrix;
            compressed.makeCompressed();
            input = &compressed;
        }
        const std::uint64_t sourcePattern = ResolvePatternFingerprint(*input, patternVersion);
        if (!canonicalPatternValid_ || sourcePattern != canonicalSourcePattern_)
        {
            std::vector<Eigen::Triplet<double>> entries;
            entries.reserve(static_cast<std::size_t>(input->nonZeros()));
            for (int column = 0; column < input->outerSize(); ++column)
                for (SpMat::InnerIterator entry(*input, column); entry; ++entry)
                    entries.emplace_back(canonicalTargetOfSource_[entry.row()],
                                         canonicalTargetOfSource_[column], 1.0);
            canonicalMatrix_.resize(matrix.rows(), matrix.cols());
            canonicalMatrix_.setFromTriplets(entries.begin(), entries.end());
            canonicalMatrix_.makeCompressed();
            canonicalSourceValueIndices_.resize(static_cast<std::size_t>(input->nonZeros()));
            canonicalValueSigns_.resize(static_cast<std::size_t>(input->nonZeros()));
            for (int column = 0; column < input->outerSize(); ++column)
                for (int sourceIndex = input->outerIndexPtr()[column];
                     sourceIndex < input->outerIndexPtr()[column + 1]; ++sourceIndex)
                {
                    const int targetRow = canonicalTargetOfSource_[input->innerIndexPtr()[sourceIndex]];
                    const int targetColumn = canonicalTargetOfSource_[column];
                    const int* begin =
                        canonicalMatrix_.innerIndexPtr() + canonicalMatrix_.outerIndexPtr()[targetColumn];
                    const int* end =
                        canonicalMatrix_.innerIndexPtr() + canonicalMatrix_.outerIndexPtr()[targetColumn + 1];
                    const auto targetIndex = std::lower_bound(begin, end, targetRow) - canonicalMatrix_.innerIndexPtr();
                    canonicalSourceValueIndices_[targetIndex] = sourceIndex;
                    canonicalValueSigns_[targetIndex] = canonicalSigns_[targetRow] * canonicalSigns_[targetColumn];
                }
            canonicalSourcePattern_ = sourcePattern;
            canonicalPattern_ = PatternFingerprint(canonicalMatrix_);
            canonicalPatternValid_ = true;
        }
        for (Eigen::Index target = 0; target < canonicalMatrix_.nonZeros(); ++target)
        {
            const double value = canonicalValueSigns_[target] * input->valuePtr()[canonicalSourceValueIndices_[target]];
            canonicalMatrix_.valuePtr()[target] = value == 0.0 ? 0.0 : value;
        }
        canonicalRhs_.resize(rhs.size());
        for (Eigen::Index target = 0; target < rhs.size(); ++target)
        {
            const int source = static_cast<int>(std::abs(static_cast<std::int64_t>(signedSource_[target])) - 1);
            canonicalRhs_[target] = canonicalSigns_[target] * rhs[source];
        }
        return true;
    }

    void RecoverCanonicalSolution(const Vec& canonicalSolution, _OUT Vec& solution) const
    {
        solution.resize(canonicalSolution.size());
        for (Eigen::Index target = 0; target < canonicalSolution.size(); ++target)
        {
            const int source = static_cast<int>(std::abs(static_cast<std::int64_t>(signedSource_[target])) - 1);
            solution[source] = canonicalSigns_[target] * canonicalSolution[target];
        }
    }

    bool SolveUnpermuted(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, std::uint64_t patternVersion = 0)
    {
        if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size())
            return false;

        const LinearSolverMode mode = EffectiveMode();
        GpuSettings::ObserveMatrixDofs(static_cast<int>(matrix.rows()));
        const std::uint64_t pattern = ResolvePatternFingerprint(matrix, patternVersion);
        if (mode == LinearSolverMode::Automatic)
            UpdateAutomaticGpuPattern(pattern);
        if (mode == LinearSolverMode::CudaIterative)
        {
            const bool solved = TryCudaIterative(matrix, rhs, solution);
            if (!solved)
                GpuSettings::RecordSkipped(QStringLiteral("CUDA 迭代求解失败"));
            return solved;
        }
        if (mode == LinearSolverMode::Cudss)
        {
            const bool solved = TryCudss(matrix, rhs, solution);
            if (!solved)
                GpuSettings::RecordSkipped(QStringLiteral("cuDSS 直接求解失败"));
            return solved;
        }
        if (mode == LinearSolverMode::Pardiso)
            return SolvePardiso(matrix, rhs, pattern, solution);

        if (mode == LinearSolverMode::Automatic && allowGpuAutomatic_ && !automaticGpuPreferCpu_ &&
            GpuSettings::IsEnabled() &&
            matrix.rows() >= GpuSettings::MinimumGpuDofs())
        {
            const auto gpuStart = std::chrono::steady_clock::now();
            if (TryCudss(matrix, rhs, solution))
            {
                EvaluateAutomaticGpuTiming(matrix, rhs, solution, pattern, gpuStart);
                return true;
            }
            if (TryCudaIterative(matrix, rhs, solution))
            {
                EvaluateAutomaticGpuTiming(matrix, rhs, solution, pattern, gpuStart);
                return true;
            }
            GpuSettings::RecordFallback(QStringLiteral("GPU 线性求解失败，已回退 CPU"));
            automaticGpuPreferCpu_ = true;
        }
        else if (mode == LinearSolverMode::Automatic && GpuSettings::IsEnabled())
        {
            if (!allowGpuAutomatic_)
                GpuSettings::RecordSkipped(QStringLiteral("当前模型不适合自动 GPU 线性求解，已使用 CPU"));
            else
                GpuSettings::RecordSkipped(QStringLiteral("矩阵自由度 %1 低于 GPU 阈值 %2，已使用 CPU")
                                               .arg(matrix.rows())
                                               .arg(GpuSettings::MinimumGpuDofs()));
        }

        if (mode == LinearSolverMode::Ldlt)
            return SolveLdlt(matrix, rhs, solution, pattern) && IsSolutionValid(matrix, rhs, solution);
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
            if (SolveLu(matrix, rhs, solution, pattern))
                return true;
            automaticBackend_ = AutomaticBackend::Uncalibrated;
        }
        if (automaticBackend_ == AutomaticBackend::Pardiso)
        {
            if (SolvePardiso(matrix, rhs, pattern, solution))
                return true;
            automaticBackend_ = AutomaticBackend::Uncalibrated;
        }

        return CalibrateAutomaticBackend(matrix, rhs, solution, pattern);
    }

    bool SolveLowRankUnpermuted(const SpMat& matrix, const Eigen::MatrixXd& factor, double scale, const Vec& rhs,
                      _OUT Vec& solution, std::uint64_t patternVersion = 0)
    {
        if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() || factor.rows() != matrix.rows() ||
            !factor.allFinite() || !std::isfinite(scale))
            return false;
        if (factor.cols() == 0 || scale == 0.0)
            return SolveUnpermuted(matrix, rhs, solution, patternVersion);

        const LinearSolverMode mode = EffectiveMode();
        const std::uint64_t pattern = ResolvePatternFingerprint(matrix, patternVersion);
        if (mode == LinearSolverMode::Automatic &&
            !(allowGpuAutomatic_ && !automaticGpuPreferCpu_ && GpuSettings::IsEnabled() &&
              matrix.rows() >= GpuSettings::MinimumGpuDofs()) &&
            factor.cols() * 2 >= matrix.rows() && matrix.rows() <= 768 &&
            SolveLowRankPrimalDense(matrix, factor, scale, rhs, solution))
            return true;
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
            !(allowGpuAutomatic_ && !automaticGpuPreferCpu_ && GpuSettings::IsEnabled() &&
              matrix.rows() >= GpuSettings::MinimumGpuDofs()))
        {
            if (automaticPattern_ != pattern || automaticBackend_ == AutomaticBackend::Uncalibrated)
            {
                Vec calibrationSolution;
                if (!SolveUnpermuted(matrix, rhs, calibrationSolution, patternVersion))
                    return false;
            }
            if (SolveLowRankWoodbury(matrix, factor, scale, rhs, pattern, automaticBackend_, solution))
                return true;
        }

        return SolveLowRankAugmented(matrix, factor, scale, rhs, solution);
    }


    LinearSolverMode EffectiveMode() const
    {
        return modeOverride_ ? *modeOverride_ : LinearSolverSettings::Mode();
    }

    bool SolveLowRankPrimalDense(const SpMat& matrix, const Eigen::MatrixXd& factor, double scale, const Vec& rhs,
                                 _OUT Vec& solution)
    {
        if (cachedDensePrimalFactor_.rows() != factor.rows() || cachedDensePrimalFactor_.cols() != factor.cols() ||
            !(cachedDensePrimalFactor_.array() == factor.array()).all())
        {
            cachedDensePrimalFactor_ = factor;
            cachedDensePrimalGram_.noalias() = factor * factor.transpose();
        }
        Eigen::MatrixXd effectiveMatrix = Eigen::MatrixXd(matrix);
        effectiveMatrix.noalias() += scale * cachedDensePrimalGram_;
        if (!effectiveMatrix.allFinite())
            return false;

        const auto acceptCandidate = [&](const Vec& candidate) {
            if (!candidate.allFinite())
                return false;
            const Vec residual = matrix * candidate + scale * factor * (factor.transpose() * candidate) - rhs;
            if (!residual.allFinite() || residual.norm() > 1.0e-8 * std::max(1.0, rhs.norm()))
                return false;
            solution = candidate;
            return true;
        };

        // 高秩阻尼的原系统规模较小但稠密，优先使用 oneMKL 分块 LU，并遵守外层线程限制。
        std::vector<MKL_INT> pivots(static_cast<std::size_t>(matrix.rows()));
        Vec mklCandidate = rhs;
        const int previousMklThreads =
            mkl_set_num_threads_local(std::min(4, std::max(1, mkl_get_max_threads())));
        const MKL_INT factorInfo = LAPACKE_dgetrf(LAPACK_COL_MAJOR, static_cast<MKL_INT>(matrix.rows()),
                                                   static_cast<MKL_INT>(matrix.cols()), effectiveMatrix.data(),
                                                   static_cast<MKL_INT>(matrix.rows()), pivots.data());
        const MKL_INT solveInfo = factorInfo == 0
                                      ? LAPACKE_dgetrs(LAPACK_COL_MAJOR, 'N', static_cast<MKL_INT>(matrix.rows()),
                                                       1, effectiveMatrix.data(), static_cast<MKL_INT>(matrix.rows()),
                                                       pivots.data(), mklCandidate.data(),
                                                       static_cast<MKL_INT>(matrix.rows()))
                                      : factorInfo;
        mkl_set_num_threads_local(previousMklThreads);
        if (solveInfo == 0 && acceptCandidate(mklCandidate))
            return true;

        // dgetrf 会覆盖输入矩阵，回退前重新装配原矩阵。
        effectiveMatrix = Eigen::MatrixXd(matrix);
        effectiveMatrix.noalias() += scale * cachedDensePrimalGram_;
        return acceptCandidate(effectiveMatrix.partialPivLu().solve(rhs));
    }

    void UpdateAutomaticGpuPattern(std::uint64_t pattern)
    {
        if (automaticGpuPattern_ == pattern)
            return;
        automaticGpuPattern_ = pattern;
        automaticGpuSamples_ = 0;
        automaticGpuPreferCpu_ = false;
    }

    void EvaluateAutomaticGpuTiming(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution,
                                    std::uint64_t pattern, std::chrono::steady_clock::time_point gpuStart)
    {
        if (++automaticGpuSamples_ != 2)
            return;

        const double gpuMilliseconds =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - gpuStart).count();
        Vec cpuSolution;
        Vec warmupSolution;
        if (!SolvePardiso(matrix, rhs, pattern, warmupSolution))
            return;
        const auto cpuStart = std::chrono::steady_clock::now();
        const bool cpuValid = SolvePardiso(matrix, rhs, pattern, cpuSolution);
        const double cpuMilliseconds =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cpuStart).count();
        if (!cpuValid)
            return;

        const double difference = (solution - cpuSolution).norm() / std::max(1.0, cpuSolution.norm());
        if (difference <= 1.0e-8 && gpuMilliseconds < 0.9 * cpuMilliseconds)
            return;

        automaticGpuPreferCpu_ = true;
        automaticBackend_ = AutomaticBackend::Pardiso;
        automaticPattern_ = pattern;
        solution = std::move(cpuSolution);
        GpuSettings::RecordSkipped(
            difference > 1.0e-8
                ? QStringLiteral("模型实测 GPU/CPU 求解结果偏差超限，后续使用 PARDISO")
                : QStringLiteral("模型实测 GPU 求解 %1 ms、PARDISO %2 ms，后续使用 CPU")
                      .arg(gpuMilliseconds, 0, 'f', 2)
                      .arg(cpuMilliseconds, 0, 'f', 2));
    }

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
        if (!SolveUnpermuted(augmentedMatrix, augmentedRhs, augmentedSolution))
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
            solved = pardisoSolver_.Solve(matrix, multipleRhs, !ShouldUsePardisoLu(matrix, pattern), pattern,
                                          multipleSolution);
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
        const Vec correctionRhs = factor.transpose() * baseSolution;
        const auto validCorrection = [&](const Vec& correction) -> bool
        {
            if (!correction.allFinite())
                return false;
            solution = baseSolution - scale * inverseFactor * correction;
            const Vec residual = matrix * solution + scale * factor * (factor.transpose() * solution) - rhs;
            return solution.allFinite() && residual.allFinite() &&
                   residual.norm() <= 1.0e-8 * std::max(1.0, rhs.norm());
        };

        // 常见的可逆模态修正先用较快的部分主元分解；残差不过关时回退完整主元。
        Eigen::PartialPivLU<Eigen::MatrixXd> fastCorrectionSolver(correctionMatrix);
        if (validCorrection(fastCorrectionSolver.solve(correctionRhs)))
            return true;
        Eigen::FullPivLU<Eigen::MatrixXd> robustCorrectionSolver(correctionMatrix);
        return robustCorrectionSolver.isInvertible() &&
               validCorrection(robustCorrectionSolver.solve(correctionRhs));
    }

    bool CalibrateAutomaticBackend(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, std::uint64_t pattern)
    {
        const bool allowLdlt = preferLdlt_ && !ShouldUsePardisoLu(matrix, pattern);
        if (allowLdlt)
            PrepareLdltPattern(matrix, pattern);
        PrepareLuPattern(matrix, pattern);

        Vec ldltSolution;
        const auto ldltStart = std::chrono::steady_clock::now();
        const bool ldltValid = allowLdlt && FactorizeAndSolveLdlt(matrix, rhs, ldltSolution) &&
                               IsSolutionValid(matrix, rhs, ldltSolution);
        const auto ldltEnd = std::chrono::steady_clock::now();

        Vec luSolution;
        const auto luStart = std::chrono::steady_clock::now();
        const bool luValid = SolveLu(matrix, rhs, luSolution, pattern);
        const auto luEnd = std::chrono::steady_clock::now();

        Vec pardisoSolution;
        Vec pardisoWarmupSolution;
        // 单次校准不能代表重复非对称分解的耗时；Eigen LU 校验通过时直接使用，失败再尝试 PARDISO LU。
        const bool pardisoPrepared = (allowLdlt || !luValid) &&
                                     SolvePardiso(matrix, rhs, pattern, pardisoWarmupSolution);
        const auto pardisoStart = std::chrono::steady_clock::now();
        const bool pardisoValid = pardisoPrepared &&
                                  SolvePardiso(matrix, rhs, pattern, pardisoSolution);
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

    bool ShouldUsePardisoLu(const SpMat& matrix, std::uint64_t pattern)
    {
        if (!preferLdlt_ || (pardisoLuPatternValid_ && pardisoLuPattern_ == pattern))
            return true;
        if (matrix.isApprox(matrix.transpose(), 1.0e-12))
            return false;
        pardisoLuPattern_ = pattern;
        pardisoLuPatternValid_ = true;
        return true;
    }

    bool SolvePardiso(const SpMat& matrix, const Vec& rhs, std::uint64_t pattern, _OUT Vec& solution)
    {
        const bool symmetric = !ShouldUsePardisoLu(matrix, pattern);
        if (pardisoSolver_.Solve(matrix, rhs, symmetric, pattern, solution) && IsSolutionValid(matrix, rhs, solution))
            return true;
        if (!symmetric)
            return false;
        pardisoLuPattern_ = pattern;
        pardisoLuPatternValid_ = true;
        return pardisoSolver_.Solve(matrix, rhs, false, pattern, solution) && IsSolutionValid(matrix, rhs, solution);
    }

    static bool IsSolutionValid(const SpMat& matrix, const Vec& rhs, const Vec& solution)
    {
        if (!solution.allFinite() || !rhs.allFinite())
            return false;
        const Vec residual = matrix * solution - rhs;
        if (!residual.allFinite())
            return false;
        const double rightHandSideScale = std::max(rhs.norm(), 1.0);
        const double residualNorm = residual.norm();
        if (!std::isfinite(rightHandSideScale) || !std::isfinite(residualNorm))
            return false;
        if (residualNorm <= 1.0e-8 * rightHandSideScale)
            return true;

        // 高刚度矩阵相消时，真实的双精度解也可能无法满足仅以较小右端项归一化的残差。
        // 逐行后向误差衡量每个方程的相消尺度，同时保留总残差上限以拒绝错误解。
        if (residualNorm > 1.0e-6 * rightHandSideScale)
            return false;
        const Vec rowScale = matrix.cwiseAbs() * solution.cwiseAbs() + rhs.cwiseAbs();
        if (!rowScale.allFinite())
            return false;
        for (Eigen::Index row = 0; row < residual.size(); ++row)
        {
            if (std::abs(residual[row]) > 1.0e-9 * rowScale[row] + 1.0e-12)
                return false;
        }
        return true;
    }

    bool TryCudss(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution)
    {
        GpuSettings::RecordAttempt();
        const bool solved = cudssSolver_.Solve(matrix, rhs, solution);
        GpuSettings::RecordCudssTiming(cudssSolver_.LastTiming());
        if (!solved || !IsSolutionValid(matrix, rhs, solution))
            return false;
        GpuSettings::RecordSuccess();
        return true;
    }

    bool TryCudssDeviceCscValues(const SpMat& matrix, const double* deviceValues, int valueCount, const Vec& rhs,
                                 _OUT Vec& solution)
    {
        GpuSettings::RecordAttempt();
        const bool solved = cudssSolver_.SolveDeviceCscValues(matrix, deviceValues, valueCount, rhs, solution);
        GpuSettings::RecordCudssTiming(cudssSolver_.LastTiming());
        if (!solved || !IsSolutionValid(matrix, rhs, solution))
            return false;
        GpuSettings::RecordSuccess();
        return true;
    }

    bool TryCudaIterative(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution)
    {
        GpuSettings::RecordAttempt();
        if (!gpuSolver_.Solve(matrix, rhs, solution) || !IsSolutionValid(matrix, rhs, solution))
            return false;
        GpuSettings::RecordSuccess();
        return true;
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
        if (!FactorizeAndSolveLu(matrix, rhs, solution))
            return false;
        // 稠密分支已通过同一残差验收；稀疏分支仍在此完整检查。
        return denseLuFactorValid_ || IsSolutionValid(matrix, rhs, solution);
    }

    void PrepareLuPattern(const SpMat& matrix, std::uint64_t pattern)
    {
        if (!ShouldUseDenseLu(matrix))
            PrepareSparseLuPattern(matrix, pattern);
    }

    void PrepareSparseLuPattern(const SpMat& matrix, std::uint64_t pattern)
    {
        if (luPatternValid_ && luPattern_ == pattern)
            return;
        lu_.analyzePattern(matrix);
        luPattern_ = pattern;
        luPatternValid_ = true;
    }

    bool SolveLuRefined(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution, std::uint64_t pattern)
    {
        if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() || !rhs.allFinite())
            return false;
        PrepareLuPattern(matrix, pattern);
        if (!FactorizeAndSolveLu(matrix, rhs, solution))
            return false;
        if (denseLuFactorValid_ || IsSolutionValid(matrix, rhs, solution))
            return true;
        Vec residual = rhs - matrix * solution;
        const double maximumResidual = 1.0e-6 * std::max(rhs.norm(), 1.0);
        if (!residual.allFinite() || residual.norm() > maximumResidual)
            return false;
        for (int iteration = 0; iteration < 3; ++iteration)
        {
            Vec correction;
            if (!SolveFactoredLu(residual, correction))
                return false;
            solution += correction;
            if (IsSolutionValid(matrix, rhs, solution))
                return true;
            residual = rhs - matrix * solution;
            if (!residual.allFinite() || residual.norm() > maximumResidual)
                return false;
        }
        return false;
    }

    bool FactorizeAndSolveLu(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution)
    {
        denseLuFactorValid_ = false;
        if (ShouldUseDenseLu(matrix))
        {
            Vec candidate;
            if (FactorizeDenseLu(matrix) && SolveFactoredLu(rhs, candidate) &&
                IsSolutionValid(matrix, rhs, candidate))
            {
                solution = std::move(candidate);
                return true;
            }
            denseLuFactorValid_ = false;
            PrepareSparseLuPattern(matrix, PatternFingerprint(matrix));
        }
        lu_.factorize(matrix);
        if (lu_.info() != Eigen::Success)
        {
            luPatternValid_ = false;
            return false;
        }

        if (!SolveFactoredLu(rhs, solution))
        {
            luPatternValid_ = false;
            return false;
        }
        return true;
    }

    bool FactorizeAndSolveLu(const SpMat& matrix, const Eigen::MatrixXd& rhs, _OUT Eigen::MatrixXd& solution)
    {
        denseLuFactorValid_ = false;
        if (ShouldUseDenseLu(matrix))
        {
            Eigen::MatrixXd candidate;
            bool valid = FactorizeDenseLu(matrix) && SolveFactoredLu(rhs, candidate);
            for (Eigen::Index column = 0; valid && column < rhs.cols(); ++column)
                valid = IsSolutionValid(matrix, rhs.col(column), candidate.col(column));
            if (valid)
            {
                solution = std::move(candidate);
                return true;
            }
            denseLuFactorValid_ = false;
            PrepareSparseLuPattern(matrix, PatternFingerprint(matrix));
        }
        lu_.factorize(matrix);
        if (lu_.info() != Eigen::Success)
        {
            luPatternValid_ = false;
            return false;
        }
        if (!SolveFactoredLu(rhs, solution))
        {
            luPatternValid_ = false;
            return false;
        }
        return true;
    }

    static bool ShouldUseDenseLu(const SpMat& matrix)
    {
        // 分组阻尼产生的中小规模满图样矩阵采用分块 LU；大模型和稀疏系统保留原路径。
        const Eigen::Index dimension = matrix.rows();
        return dimension >= 128 && dimension <= 1536 && matrix.cols() == dimension &&
               static_cast<double>(matrix.nonZeros()) >= 0.35 * static_cast<double>(dimension) * dimension;
    }

    bool FactorizeDenseLu(const SpMat& matrix)
    {
        // 仅复用工作区，每次都装入当前切线，避免将相同图样误认为相同数值。
        denseLuFactors_ = Eigen::MatrixXd(matrix);
        if (!denseLuFactors_.allFinite())
            return false;
        denseLuPivots_.resize(static_cast<std::size_t>(matrix.rows()));
        const auto dimension = static_cast<MKL_INT>(matrix.rows());
        const int previousThreads = mkl_set_num_threads_local(std::min(4, std::max(1, mkl_get_max_threads())));
        const MKL_INT info = LAPACKE_dgetrf(LAPACK_COL_MAJOR, dimension, dimension, denseLuFactors_.data(),
                                          dimension, denseLuPivots_.data());
        mkl_set_num_threads_local(previousThreads);
        denseLuFactorValid_ = info == 0;
        return denseLuFactorValid_;
    }

    bool SolveFactoredLu(const Vec& rhs, _OUT Vec& solution)
    {
        if (!denseLuFactorValid_)
        {
            solution = lu_.solve(rhs);
            return lu_.info() == Eigen::Success && solution.allFinite();
        }
        solution = rhs;
        const auto dimension = static_cast<MKL_INT>(denseLuFactors_.rows());
        const int previousThreads = mkl_set_num_threads_local(std::min(4, std::max(1, mkl_get_max_threads())));
        const MKL_INT info = LAPACKE_dgetrs(LAPACK_COL_MAJOR, 'N', dimension, 1, denseLuFactors_.data(), dimension,
                                          denseLuPivots_.data(), solution.data(), dimension);
        mkl_set_num_threads_local(previousThreads);
        return info == 0 && solution.allFinite();
    }

    bool SolveFactoredLu(const Eigen::MatrixXd& rhs, _OUT Eigen::MatrixXd& solution)
    {
        if (!denseLuFactorValid_)
        {
            solution = lu_.solve(rhs);
            return lu_.info() == Eigen::Success && solution.allFinite();
        }
        solution = rhs;
        const auto dimension = static_cast<MKL_INT>(denseLuFactors_.rows());
        const int previousThreads = mkl_set_num_threads_local(std::min(4, std::max(1, mkl_get_max_threads())));
        const MKL_INT info = LAPACKE_dgetrs(LAPACK_COL_MAJOR, 'N', dimension, static_cast<MKL_INT>(rhs.cols()),
                                          denseLuFactors_.data(), dimension, denseLuPivots_.data(), solution.data(),
                                          dimension);
        mkl_set_num_threads_local(previousThreads);
        return info == 0 && solution.allFinite();
    }

    static std::uint64_t PatternFingerprint(const SpMat& matrix)
    {
        static thread_local Eigen::Index completePatternDimension = 0;
        static thread_local std::uint64_t completePatternFingerprint = 0;
        const Eigen::Index dimension = matrix.rows();
        // 合法 Eigen 压缩矩阵的列内索引有序且唯一；满方阵图样只取决于尺寸。
        bool completePattern = dimension > 0 && matrix.cols() == dimension && matrix.isCompressed() &&
                               matrix.nonZeros() / dimension == dimension && matrix.nonZeros() % dimension == 0;
        for (Eigen::Index column = 0; completePattern && column <= dimension; ++column)
            completePattern = matrix.outerIndexPtr()[column] == column * dimension;
        if (completePattern && completePatternDimension == dimension)
            return completePatternFingerprint;

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
        if (completePattern)
        {
            completePatternDimension = dimension;
            completePatternFingerprint = hash;
        }
        return hash;
    }

    static std::uint64_t ResolvePatternFingerprint(const SpMat& matrix, std::uint64_t patternVersion)
    {
        // 高位用于区分模型版本号和按内容计算的哈希值。
        return patternVersion == 0 ? PatternFingerprint(matrix) : patternVersion ^ (1ull << 63U);
    }

    std::vector<int> signedSource_;
    std::vector<int> canonicalTargetOfSource_;
    std::vector<double> canonicalSigns_;
    std::vector<int> canonicalSourceValueIndices_;
    std::vector<double> canonicalValueSigns_;
    SpMat canonicalMatrix_;
    Vec canonicalRhs_;
    Eigen::MatrixXd canonicalFactorSource_;
    Eigen::MatrixXd canonicalFactor_;
    std::uint64_t canonicalSourcePattern_ = 0;
    std::uint64_t canonicalPattern_ = 0;
    bool canonicalPatternValid_ = false;
    Eigen::SimplicialLDLT<SpMat> ldlt_;
    Eigen::SparseLU<SpMat> lu_;
    Eigen::MatrixXd denseLuFactors_;
    std::vector<MKL_INT> denseLuPivots_;
    bool denseLuFactorValid_ = false;
    PardisoSolver pardisoSolver_;
    CudaSparseSolver gpuSolver_;
    CudssSolver cudssSolver_;
    Eigen::MatrixXd cachedDensePrimalFactor_;
    Eigen::MatrixXd cachedDensePrimalGram_;
    std::optional<LinearSolverMode> modeOverride_;
    bool preferLdlt_ = true;
    bool allowGpuAutomatic_ = true;
    bool automaticGpuPreferCpu_ = false;
    int automaticGpuSamples_ = 0;
    std::uint64_t automaticGpuPattern_ = 0;
    bool ldltPatternValid_ = false;
    bool luPatternValid_ = false;
    bool pardisoLuPatternValid_ = false;
    AutomaticBackend automaticBackend_ = AutomaticBackend::Uncalibrated;
    std::uint64_t ldltPattern_ = 0;
    std::uint64_t luPattern_ = 0;
    std::uint64_t pardisoLuPattern_ = 0;
    std::uint64_t automaticPattern_ = 0;
};
}
