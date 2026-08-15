#pragma once

#include "Solver/Interface/IAnalysisModel.h"

#include <cstdint>
#include <memory>

namespace SolverNameSpace
{
class PardisoSolver final
{
public:
    PardisoSolver();
    ~PardisoSolver();
    PardisoSolver(const PardisoSolver&) = delete;
    PardisoSolver& operator=(const PardisoSolver&) = delete;

    bool Solve(const SpMat& matrix, const Vec& rhs, bool symmetric, std::uint64_t pattern, _OUT Vec& solution);
    bool Solve(const SpMat& matrix, const Eigen::MatrixXd& rhs, bool symmetric, std::uint64_t pattern,
               _OUT Eigen::MatrixXd& solution);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
}
