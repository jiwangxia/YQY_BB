#include "GpuSettings.h"

namespace SolverNameSpace
{
std::atomic_bool GpuSettings::s_enabled{ false };
std::atomic_int GpuSettings::s_attempts{ 0 };
std::atomic_int GpuSettings::s_successes{ 0 };
std::atomic_int GpuSettings::s_fallbacks{ 0 };
std::atomic_int GpuSettings::s_maximumMatrixDofs{ 0 };
}
