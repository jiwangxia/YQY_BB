#pragma once

#include "Solver/Interface/IAnalysisModel.h"

namespace SolverNameSpace
{
// GPU BiCGSTAB backend. Failure to converge is deliberately reported to the
// caller so that the established CPU direct solver remains the safe fallback.
class CudaSparseSolver final
{
public:
    bool Solve(const SpMat& matrix, const Vec& rhs, Vec& solution,
        double relativeTolerance = 1.0e-10, int maximumIterations = 0) const;
};
}
