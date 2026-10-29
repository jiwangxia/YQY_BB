#include <gtest/gtest.h>

#include "Application/ApplicationBootstrap.h"
#include "Application/SolveWorker.h"
#include "Runner/CaseRunner.h"
#include "Utility/RuntimeOptions.h"

#include <QApplication>
#include <QCoreApplication>
#include <QTextStream>
#include <exception>

int main(int argc, char* argv[])
{
    QStringList arguments;
    for (int index = 0; index < argc; ++index)
        arguments.append(QString::fromLocal8Bit(argv[index]));
    if (arguments.contains(QStringLiteral("--gtest_list_tests")))
    {
        ::testing::InitGoogleTest(&argc, argv);
        return RUN_ALL_TESTS();
    }
    Q_INIT_RESOURCE(YQY);
    RuntimeOptions::Apply(argc, argv);
    ApplicationBootstrap::prepareEnvironment();
    if (arguments.contains(QStringLiteral("--solve-worker")))
    {
        QCoreApplication application(argc, argv);
        return RunIsolatedSolveWorker(application.arguments());
    }
    bool explicitCase = arguments.contains(QStringLiteral("--list-cases"));
    for (const QString& argument : arguments)
    {
        if (argument.startsWith(QStringLiteral("--audit-")) || argument.startsWith(QStringLiteral("--diagnose-")) ||
            argument == QStringLiteral("--export-static-case"))
        {
            QTextStream(stderr) << "该工程诊断入口已移出算例验收，请查看 --list-cases。" << Qt::endl;
            return 2;
        }
        const bool numericalCase =
            argument.startsWith(QStringLiteral("--verify-")) || argument.startsWith(QStringLiteral("--benchmark-"));
        explicitCase |= numericalCase;
#if defined(YQY_TEST_VERBOSE_DEFAULT)
        if (numericalCase && qEnvironmentVariableIsEmpty("YQY_VERBOSE_LOG"))
            qputenv("YQY_VERBOSE_LOG", QByteArrayLiteral("1"));
#endif
    }
    if (explicitCase)
    {
        QCoreApplication application(argc, argv);
        try
        {
            if (const auto result = RunNumericalCase(arguments))
                return *result;
        }
        catch (const std::exception& exception)
        {
            QTextStream(stderr) << "算例执行失败：" << exception.what() << Qt::endl;
            return 2;
        }
        QTextStream(stderr) << "未登记的算例入口，请查看 --list-cases 和 README。" << Qt::endl;
        return 2;
    }
    ::testing::InitGoogleTest(&argc, argv);
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM"))
        qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("offscreen"));
    QApplication application(argc, argv);
    ApplicationBootstrap::configureApplication(application);
    return RUN_ALL_TESTS();
}
