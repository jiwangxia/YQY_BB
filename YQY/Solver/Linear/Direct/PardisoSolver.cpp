#include "PardisoSolver.h"

#include <Eigen/PardisoSupport>

namespace SolverNameSpace
{
class PardisoSolver::Impl
{
public:
    bool Solve(const SpMat& matrix, const Vec& rhs, bool symmetric, std::uint64_t pattern, _OUT Vec& solution)
    {
        if (symmetric)
            return SolveLdlt(matrix, rhs, pattern, solution);
        return SolveLu(matrix, rhs, pattern, solution);
    }

    bool Solve(const SpMat& matrix, const Eigen::MatrixXd& rhs, bool symmetric, std::uint64_t pattern,
               _OUT Eigen::MatrixXd& solution)
    {
        if (symmetric)
            return SolveLdlt(matrix, rhs, pattern, solution);
        return SolveLu(matrix, rhs, pattern, solution);
    }

private:
    bool SolveLdlt(const SpMat& matrix, const Vec& rhs, std::uint64_t pattern, _OUT Vec& solution)
    {
        if (!m_ldltPatternValid || m_ldltPattern != pattern)
        {
            m_ldlt.analyzePattern(matrix);
            if (m_ldlt.info() != Eigen::Success)
                return false;
            m_ldltPattern = pattern;
            m_ldltPatternValid = true;
        }

        m_ldlt.factorize(matrix);
        if (m_ldlt.info() != Eigen::Success)
            return false;
        solution = m_ldlt.solve(rhs);
        return m_ldlt.info() == Eigen::Success && solution.allFinite();
    }

    bool SolveLu(const SpMat& matrix, const Vec& rhs, std::uint64_t pattern, _OUT Vec& solution)
    {
        if (!m_luPatternValid || m_luPattern != pattern)
        {
            m_lu.analyzePattern(matrix);
            if (m_lu.info() != Eigen::Success)
                return false;
            m_luPattern = pattern;
            m_luPatternValid = true;
        }

        m_lu.factorize(matrix);
        if (m_lu.info() != Eigen::Success)
            return false;
        solution = m_lu.solve(rhs);
        return m_lu.info() == Eigen::Success && solution.allFinite();
    }

    bool SolveLdlt(const SpMat& matrix, const Eigen::MatrixXd& rhs, std::uint64_t pattern,
                   _OUT Eigen::MatrixXd& solution)
    {
        if (!m_ldltPatternValid || m_ldltPattern != pattern)
        {
            m_ldlt.analyzePattern(matrix);
            if (m_ldlt.info() != Eigen::Success)
                return false;
            m_ldltPattern = pattern;
            m_ldltPatternValid = true;
        }

        m_ldlt.factorize(matrix);
        if (m_ldlt.info() != Eigen::Success)
            return false;
        solution = m_ldlt.solve(rhs);
        return m_ldlt.info() == Eigen::Success && solution.allFinite();
    }

    bool SolveLu(const SpMat& matrix, const Eigen::MatrixXd& rhs, std::uint64_t pattern,
                 _OUT Eigen::MatrixXd& solution)
    {
        if (!m_luPatternValid || m_luPattern != pattern)
        {
            m_lu.analyzePattern(matrix);
            if (m_lu.info() != Eigen::Success)
                return false;
            m_luPattern = pattern;
            m_luPatternValid = true;
        }

        m_lu.factorize(matrix);
        if (m_lu.info() != Eigen::Success)
            return false;
        solution = m_lu.solve(rhs);
        return m_lu.info() == Eigen::Success && solution.allFinite();
    }

    Eigen::PardisoLDLT<SpMat, Eigen::Upper> m_ldlt;
    Eigen::PardisoLU<SpMat> m_lu;
    std::uint64_t m_ldltPattern = 0;
    std::uint64_t m_luPattern = 0;
    bool m_ldltPatternValid = false;
    bool m_luPatternValid = false;
};

PardisoSolver::PardisoSolver()
    : m_impl(std::make_unique<Impl>())
{
}

PardisoSolver::~PardisoSolver() = default;

bool PardisoSolver::Solve(const SpMat& matrix, const Vec& rhs, bool symmetric, std::uint64_t pattern,
                          _OUT Vec& solution)
{
    if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.size() || matrix.rows() == 0)
        return false;
    return m_impl->Solve(matrix, rhs, symmetric, pattern, solution);
}

bool PardisoSolver::Solve(const SpMat& matrix, const Eigen::MatrixXd& rhs, bool symmetric, std::uint64_t pattern,
                          _OUT Eigen::MatrixXd& solution)
{
    if (matrix.rows() != matrix.cols() || matrix.rows() != rhs.rows() || matrix.rows() == 0 || rhs.cols() == 0)
        return false;
    return m_impl->Solve(matrix, rhs, symmetric, pattern, solution);
}
}
