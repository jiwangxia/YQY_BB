#include "ApplicationPaths.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>

namespace
{
QString existingDirectory(const QStringList& candidates, const QString& fallback)
{
    for (const QString& candidate : candidates)
    {
        const QDir directory(candidate);
        if (directory.exists())
            return directory.absolutePath();
    }
    return fallback;
}

QStringList applicationDirectoryCandidates(const QString& projectRelativePath)
{
    const QDir applicationDirectory(QCoreApplication::applicationDirPath());
    return {
        QDir::current().absoluteFilePath(QStringLiteral("YQY/") + projectRelativePath),
        applicationDirectory.absoluteFilePath(QStringLiteral("../../YQY/") + projectRelativePath),
        applicationDirectory.absoluteFilePath(QStringLiteral("../../") + projectRelativePath),
        applicationDirectory.absoluteFilePath(QStringLiteral("../") + projectRelativePath)};
}
}

namespace ApplicationPaths
{
QString importFileDirectory()
{
    return existingDirectory(
        applicationDirectoryCandidates(QStringLiteral("Import/ImportFile")),
        QDir::currentPath());
}

QString aerodynamicDataDirectory()
{
    return existingDirectory(
        applicationDirectoryCandidates(QStringLiteral("Import/Aero_Data/Input_Data")),
        QDir::currentPath());
}

QString hdf5ResultDirectory()
{
    return existingDirectory(
        applicationDirectoryCandidates(QStringLiteral("Export/ExportH5")),
        importFileDirectory());
}

QString iterationResultDirectory()
{
    const QStringList candidates =
        applicationDirectoryCandidates(QStringLiteral("Export/ExportIteration"));
    for (const QString& candidate : candidates)
    {
        const QFileInfo parentInfo(QDir(candidate).absoluteFilePath(QStringLiteral("..")));
        if (QDir(candidate).exists()
            || (parentInfo.isDir() && QDir().mkpath(candidate)))
        {
            return QDir(candidate).absolutePath();
        }
    }
    return QDir::currentPath();
}
}
