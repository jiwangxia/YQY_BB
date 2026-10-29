#include "Support/TestPaths.h"

#include "Application/ApplicationPaths.h"

#include <QCoreApplication>
#include <QDir>
#include <stdexcept>

QString TestPaths::RepositoryRoot()
{
    QDir directory(QCoreApplication::applicationDirPath());
    do
    {
        if (directory.exists(QStringLiteral("YQY/Import/ImportFile")) &&
            directory.exists(QStringLiteral("YQYTests/YQYTests.vcxproj")))
            return directory.absolutePath();
    } while (directory.cdUp());

    throw std::runtime_error("无法从测试程序位置找到 YQY 仓库");
}

QString TestPaths::TestRoot()
{
    return QDir(RepositoryRoot()).filePath(QStringLiteral("YQYTests"));
}

QString TestPaths::VerificationOutputDirectory(const QString& verificationName)
{
    const QString directory = QDir(ApplicationPaths::workspaceRootDirectory())
                                  .absoluteFilePath(QStringLiteral("Verification/%1").arg(verificationName));
    QDir().mkpath(directory);
    return QDir(directory).absolutePath();
}
