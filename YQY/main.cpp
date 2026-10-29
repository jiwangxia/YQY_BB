#include "Application/ApplicationBootstrap.h"
#include "GUI/YQY.h"
#include "Application/SolveWorker.h"
#include "Utility/RuntimeOptions.h"

#include <QCoreApplication>

int main(int argc, char* argv[])
{
    RuntimeOptions::Apply(argc, argv);
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex)
    {
        if (QString::fromLocal8Bit(argv[argumentIndex]) == QStringLiteral("--"))
            break;
        if (QString::fromLocal8Bit(argv[argumentIndex]) == QStringLiteral("--solve-worker"))
        {
            ApplicationBootstrap::prepareEnvironment();
            QCoreApplication application(argc, argv);
            return RunIsolatedSolveWorker(application.arguments());
        }
    }

    ApplicationBootstrap::prepareEnvironment();
    QApplication application(argc, argv);
    ApplicationBootstrap::configureApplication(application);
    const QStringList arguments = application.arguments();

    YQY window;
    ApplicationBootstrap::prepareMainWindow(window);
    window.show();

    const QStringList modelFiles = RuntimeOptions::ModelFiles(arguments);
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
