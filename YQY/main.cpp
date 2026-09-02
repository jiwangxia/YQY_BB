#include "Application/ApplicationBootstrap.h"
#ifdef _DEBUG
#include "Application/UiAuditRunner.h"
#include "Application/VerificationRunner.h"
#endif
#include "GUI/YQY.h"
#include "Export/ResultOutputSettings.h"
#include "Solver/GpuSettings.h"
#include "Solver/LinearSolverSettings.h"

#include <cstdio>

namespace
{
void PrintGpuSolverStatistics()
{
    std::fprintf(stdout, "gpu_solver attempts=%d successes=%d fallbacks=%d max_matrix_dofs=%d threshold=%d\n",
                 SolverNameSpace::GpuSettings::Attempts(), SolverNameSpace::GpuSettings::Successes(),
                 SolverNameSpace::GpuSettings::Fallbacks(), SolverNameSpace::GpuSettings::MaximumMatrixDofs(),
                 SolverNameSpace::GpuSettings::MinimumGpuDofs);
}
}

int main(int argc, char* argv[])
{
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        const QString argument = QString::fromLocal8Bit(argv[argumentIndex]);
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
        if (QString::fromLocal8Bit(argv[argumentIndex]) == QStringLiteral("--gpu-solver"))
        {
            SolverNameSpace::GpuSettings::SetEnabled(true);
            SolverNameSpace::GpuSettings::ResetStatistics();
            std::atexit(PrintGpuSolverStatistics);
            break;
        }
    }
#ifdef _DEBUG
    bool headlessVerification = false;
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        const QString argument = QString::fromLocal8Bit(argv[argumentIndex]);
        headlessVerification = argument == QStringLiteral("--verify-adaptive-tssbn") ||
                               argument == QStringLiteral("--verify-gpu-solver") ||
                               argument == QStringLiteral("--verify-beam-dynamics") ||
                               argument == QStringLiteral("--verify-le2012-example1") ||
                               argument == QStringLiteral("--verify-le2012-example1-tssbn") ||
                               argument == QStringLiteral("--verify-le2012-example4") ||
                               argument == QStringLiteral("--verify-le2012-example4-tssbn") ||
                               argument == QStringLiteral("--verify-spatial-wind-load") ||
                               argument == QStringLiteral("--verify-exact-aero-angle") ||
                               argument == QStringLiteral("--verify-galloping-case") ||
                               argument == QStringLiteral("--verify-structural-damping") ||
                               argument == QStringLiteral("--verify-low-rank-damping") ||
                               argument == QStringLiteral("--verify-structural-damping-hdf5");
        if (headlessVerification)
            break;
    }
    if (headlessVerification)
    {
        QCoreApplication application(argc, argv);
        if (const auto result = VerificationRunner::runHeadless(application.arguments()))
            return *result;
        return 1;
    }

    ApplicationBootstrap::prepareEnvironment();
    QApplication application(argc, argv);
    ApplicationBootstrap::configureApplication(application);

    const QStringList arguments = application.arguments();
    if (const auto verificationResult = VerificationRunner::run(application, arguments))
        return *verificationResult;

    const UiAuditRunner auditRunner(arguments);
    if (!auditRunner.isValid())
        return auditRunner.errorCode();
#else
    ApplicationBootstrap::prepareEnvironment();
    QApplication application(argc, argv);
    ApplicationBootstrap::configureApplication(application);
    const QStringList arguments = application.arguments();
#endif

    YQY window;
    ApplicationBootstrap::prepareMainWindow(window);
    window.show();

#ifdef _DEBUG
    if (const auto auditResult = auditRunner.run(application, window))
        return *auditResult;
#endif

    const QStringList modelFiles = arguments.mid(1);
    if (!modelFiles.isEmpty())
    {
        QTimer::singleShot(0, &window,
                           [&window, modelFiles]()
                           {
                               for (const QString& filePath : modelFiles)
                                   window.openModel(filePath);
                           });
    }
    return application.exec();
}
