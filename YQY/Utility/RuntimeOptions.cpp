#include "Utility/RuntimeOptions.h"

#include "Export/ResultOutputSettings.h"
#include "Solver/Assembly/AssemblySettings.h"
#include "Solver/Linear/Gpu/GpuSettings.h"
#include "Solver/Linear/LinearSolverSettings.h"

#include <QByteArray>
#include <QString>
#include <cstdio>
#include <cstdlib>

static void PrintGpuSolverStatistics()
{
    const QByteArray calibration = SolverNameSpace::GpuSettings::CalibrationDescription().toUtf8();
    std::fprintf(stdout, "gpu_calibration %s\n", calibration.constData());
    std::fprintf(stdout, "gpu_solver attempts=%d successes=%d fallbacks=%d max_matrix_dofs=%d threshold=%d\n",
                 SolverNameSpace::GpuSettings::Attempts(), SolverNameSpace::GpuSettings::Successes(),
                 SolverNameSpace::GpuSettings::Fallbacks(), SolverNameSpace::GpuSettings::MaximumMatrixDofs(),
                 SolverNameSpace::GpuSettings::MinimumGpuDofs());
    std::fprintf(stdout,
                 "gpu_cudss upload_ms=%.3f analysis_ms=%.3f factorization_ms=%.3f solve_ms=%.3f "
                 "download_ms=%.3f analysis_reuses=%d\n",
                 SolverNameSpace::GpuSettings::CudssUploadMilliseconds(),
                 SolverNameSpace::GpuSettings::CudssAnalysisMilliseconds(),
                 SolverNameSpace::GpuSettings::CudssFactorizationMilliseconds(),
                 SolverNameSpace::GpuSettings::CudssSolveMilliseconds(),
                 SolverNameSpace::GpuSettings::CudssDownloadMilliseconds(),
                 SolverNameSpace::GpuSettings::CudssAnalysisReuses());
}
void RuntimeOptions::Apply(int argc, char* argv[])
{
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        const QString argument = QString::fromLocal8Bit(argv[argumentIndex]);
        if (argument == QStringLiteral("--"))
            break;
        if (argument == QStringLiteral("--linear-solver=ldlt"))
            SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::Ldlt);
        else if (argument == QStringLiteral("--linear-solver=lu"))
            SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::Lu);
        else if (argument == QStringLiteral("--linear-solver=auto"))
            SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::Automatic);
        else if (argument == QStringLiteral("--linear-solver=pardiso"))
            SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::Pardiso);
        else if (argument == QStringLiteral("--linear-solver=cuda"))
            SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::CudaIterative);
        else if (argument == QStringLiteral("--linear-solver=cudss"))
            SolverNameSpace::LinearSolverSettings::SetMode(SolverNameSpace::LinearSolverMode::Cudss);
        else if (argument.startsWith(QStringLiteral("--assembly-threads=")))
        {
            bool converted = false;
            const int threadCount = argument.sliced(QStringLiteral("--assembly-threads=").size()).toInt(&converted);
            if (converted && threadCount > 0)
                SolverNameSpace::AssemblySettings::SetThreadCount(threadCount);
        }
        else if (argument.startsWith(QStringLiteral("--result-batch-frames=")))
        {
            bool converted = false;
            const int frameCount = argument.sliced(QStringLiteral("--result-batch-frames=").size()).toInt(&converted);
            if (converted)
                ResultOutputSettings::SetFrameBatchSize(frameCount);
        }
        else if (argument == QStringLiteral("--background-result-write"))
            ResultOutputSettings::SetBackgroundWriteEnabled(true);
        else if (argument == QStringLiteral("--no-background-result-write"))
            ResultOutputSettings::SetBackgroundWriteEnabled(false);
    }
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        if (QString::fromLocal8Bit(argv[argumentIndex]) == QStringLiteral("--"))
            break;
        if (QString::fromLocal8Bit(argv[argumentIndex]) == QStringLiteral("--gpu-solver"))
        {
            SolverNameSpace::GpuSettings::SetEnabled(true);
            SolverNameSpace::GpuSettings::ResetStatistics();
            std::atexit(PrintGpuSolverStatistics);
            break;
        }
    }
}

QStringList RuntimeOptions::ModelFiles(const QStringList& arguments)
{
    QStringList files;
    bool positionalOnly = false;
    for (qsizetype index = 1; index < arguments.size(); ++index)
    {
        const QString& argument = arguments.at(index);
        if (!positionalOnly && argument == QStringLiteral("--"))
        {
            positionalOnly = true;
            continue;
        }
        if (!positionalOnly && argument.startsWith(QLatin1Char('-')))
            continue;
        if (!argument.trimmed().isEmpty())
            files.append(argument);
    }
    return files;
}
