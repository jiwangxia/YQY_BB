#include "Application/ApplicationBootstrap.h"
#include "Application/UiAuditRunner.h"
#include "Application/VerificationRunner.h"
#include "GUI/YQY.h"

int main(int argc, char* argv[])
{
    ApplicationBootstrap::prepareEnvironment();
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
