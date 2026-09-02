#include "ApplicationPaths.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>

static QString existingDirectory(const QStringList& candidates, const QString& fallback)
{
    for (const QString& candidate : candidates)
    {
        const QDir directory(candidate);
        if (directory.exists())
            return directory.absolutePath();
    }
    return fallback;
}

static QString sourceDirectory()
{
    const QDir applicationDirectory(QCoreApplication::applicationDirPath());
    const QStringList candidates = {QDir::current().absoluteFilePath(QStringLiteral("YQY")), QDir::currentPath(),
                                    applicationDirectory.absoluteFilePath(QStringLiteral("../../YQY")),
                                    applicationDirectory.absoluteFilePath(QStringLiteral("../YQY"))};
    for (const QString& candidate : candidates)
    {
        if (QFileInfo(QDir(candidate).filePath(QStringLiteral("YQY.vcxproj"))).isFile())
            return QDir(candidate).absolutePath();
    }
    return QDir::current().absoluteFilePath(QStringLiteral("YQY"));
}

static QStringList sourceDirectoryCandidates(const QString& projectRelativePath)
{
    return {QDir(sourceDirectory()).absoluteFilePath(projectRelativePath)};
}

static QString createDirectory(const QString& directory)
{
    QDir().mkpath(directory);
    return QDir(directory).absolutePath();
}

static QString projectDirectory(const QString& sourceModelFile)
{
    const QFileInfo sourceInfo(sourceModelFile);
    const QString name = sourceInfo.completeBaseName().trimmed().isEmpty() ? QStringLiteral("未命名模型")
                                                                             : sourceInfo.completeBaseName().trimmed();
    const QByteArray key = QCryptographicHash::hash(sourceInfo.absoluteFilePath().toUtf8(), QCryptographicHash::Sha1)
                               .toHex()
                               .left(10);
    return QDir(ApplicationPaths::workspaceRootDirectory())
        .absoluteFilePath(QStringLiteral("Projects/%1_%2").arg(name, QString::fromLatin1(key)));
}

static QString projectDirectoryForResult(const QString& resultFile)
{
    const QFileInfo resultInfo(resultFile);
    const QDir resultDirectory = resultInfo.absoluteDir();
    if (resultDirectory.dirName().compare(QStringLiteral("Results"), Qt::CaseInsensitive) != 0)
        return QString();

    const QDir projectDirectory = QDir(resultDirectory.absoluteFilePath(QStringLiteral("..")));
    const QDir projectsDirectory = QDir(projectDirectory.absoluteFilePath(QStringLiteral("..")));
    const QDir workspaceDirectory = QDir(projectsDirectory.absoluteFilePath(QStringLiteral("..")));
    if (projectsDirectory.dirName() != QStringLiteral("Projects") ||
        workspaceDirectory.absolutePath() != QDir(ApplicationPaths::workspaceRootDirectory()).absolutePath())
    {
        return QString();
    }
    return projectDirectory.absolutePath();
}

namespace ApplicationPaths
{
QString workspaceRootDirectory()
{
    QString root = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    if (root.isEmpty())
        root = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    return createDirectory(QDir(root).absoluteFilePath(QStringLiteral("YQY")));
}

QString importFileDirectory()
{
    return existingDirectory(sourceDirectoryCandidates(QStringLiteral("Import/ImportFile")), QDir::currentPath());
}

QString aerodynamicDataDirectory()
{
    return existingDirectory(sourceDirectoryCandidates(QStringLiteral("Import/Aero_Data/Input_Data")),
                             QDir::currentPath());
}

QString hdf5ResultDirectory(const QString& sourceModelFile)
{
    const QString baseDirectory = sourceModelFile.trimmed().isEmpty() ? workspaceRootDirectory()
                                                                        : projectDirectory(sourceModelFile);
    return createDirectory(QDir(baseDirectory).absoluteFilePath(QStringLiteral("Results")));
}

QString resultExportDirectory(const QString& resultFile)
{
    const QString projectPath = projectDirectoryForResult(resultFile);
    const QString baseDirectory = projectPath.isEmpty() ? workspaceRootDirectory() : projectPath;
    return createDirectory(QDir(baseDirectory).absoluteFilePath(QStringLiteral("Exports")));
}

QString iterationResultDirectory(const QString& resultFile)
{
    return createDirectory(QDir(resultExportDirectory(resultFile)).absoluteFilePath(QStringLiteral("Iteration")));
}

QString generatedModelDirectory()
{
    return createDirectory(QDir(workspaceRootDirectory()).absoluteFilePath(QStringLiteral("GeneratedModels")));
}

QString verificationOutputDirectory(const QString& verificationName)
{
    return createDirectory(
        QDir(workspaceRootDirectory()).absoluteFilePath(QStringLiteral("Verification/%1").arg(verificationName)));
}
}
