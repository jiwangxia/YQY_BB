#include "Application/ApplicationBootstrap.h"
#include "Application/UiAuditRunner.h"
#include "Application/VerificationRunner.h"
#include "GUI/YQY.h"
#include "Solver/GpuSettings.h"

#include <cstdio>

namespace
{
void PrintGpuSolverStatistics()
{
    std::fprintf(stdout, "gpu_solver attempts=%d successes=%d fallbacks=%d max_matrix_dofs=%d threshold=%d\n",
        SolverNameSpace::GpuSettings::Attempts(),
        SolverNameSpace::GpuSettings::Successes(),
        SolverNameSpace::GpuSettings::Fallbacks(),
        SolverNameSpace::GpuSettings::MaximumMatrixDofs(),
        SolverNameSpace::GpuSettings::MinimumGpuDofs);
}
}

int main(int argc, char* argv[])
{
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        if (QString::fromLocal8Bit(argv[argumentIndex])
            == QStringLiteral("--gpu-solver"))
        {
            SolverNameSpace::GpuSettings::SetEnabled(true);
            SolverNameSpace::GpuSettings::ResetStatistics();
            std::atexit(PrintGpuSolverStatistics);
            break;
        }
    }
    bool headlessVerification = false;
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        const QString argument = QString::fromLocal8Bit(argv[argumentIndex]);
        headlessVerification =
            argument == QStringLiteral("--verify-adaptive-tssbn")
            || argument == QStringLiteral("--verify-beam-dynamics")
            || argument == QStringLiteral("--verify-le2012-example1")
            || argument == QStringLiteral("--verify-le2012-example1-tssbn")
            || argument == QStringLiteral("--verify-le2012-example4")
            || argument == QStringLiteral("--verify-le2012-example4-tssbn")
            || argument == QStringLiteral("--verify-spatial-wind-load")
            || argument == QStringLiteral("--verify-exact-aero-angle")
            || argument == QStringLiteral("--verify-galloping-case");
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
    QCoreApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
    QApplication application(argc, argv);
    ApplicationBootstrap::configureApplication(application);

    const QStringList arguments = application.arguments();
    if (const auto verificationResult = VerificationRunner::run(application, arguments))
        return *verificationResult;

    const UiAuditRunner auditRunner(arguments);
    if (!auditRunner.isValid())
        return auditRunner.errorCode();

    YQY window;
    ApplicationBootstrap::prepareMainWindow(window);
    window.show();

    if (const auto auditResult = auditRunner.run(application, window))
        return *auditResult;

    const QStringList modelFiles = arguments.mid(1);
    if (!modelFiles.isEmpty())
    {
        QTimer::singleShot(0, &window, [&window, modelFiles]()
        {
            for (const QString& filePath : modelFiles)
                window.openModel(filePath);
        });
    }
    return application.exec();
}
