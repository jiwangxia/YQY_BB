#pragma once

#include "Solver/Interface/IAnalysisModel.h"

#include <memory>

namespace SolverNameSpace
{
class CudssSolver final
{
public:
    CudssSolver();
    ~CudssSolver();
    CudssSolver(const CudssSolver&) = delete;
    CudssSolver& operator=(const CudssSolver&) = delete;

    bool Solve(const SpMat& matrix, const Vec& rhs, _OUT Vec& solution);

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
}
