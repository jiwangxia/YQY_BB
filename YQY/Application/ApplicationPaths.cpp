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

QString sourceDirectory()
{
    const QDir applicationDirectory(QCoreApplication::applicationDirPath());
    const QStringList candidates =
    {
        QDir::current().absoluteFilePath(QStringLiteral("YQY")),
        QDir::currentPath(),
        applicationDirectory.absoluteFilePath(QStringLiteral("../../YQY")),
        applicationDirectory.absoluteFilePath(QStringLiteral("../YQY"))
    };
    for (const QString& candidate : candidates)
    {
        if (QFileInfo(QDir(candidate).filePath(QStringLiteral("YQY.vcxproj"))).isFile())
            return QDir(candidate).absolutePath();
    }
    return QDir::current().absoluteFilePath(QStringLiteral("YQY"));
}

QStringList sourceDirectoryCandidates(const QString& projectRelativePath)
{
    return { QDir(sourceDirectory()).absoluteFilePath(projectRelativePath) };
}
}

namespace ApplicationPaths
{
QString importFileDirectory()
{
    return existingDirectory(
        sourceDirectoryCandidates(QStringLiteral("Import/ImportFile")),
        QDir::currentPath());
}

QString aerodynamicDataDirectory()
{
    return existingDirectory(
        sourceDirectoryCandidates(QStringLiteral("Import/Aero_Data/Input_Data")),
        QDir::currentPath());
}

QString hdf5ResultDirectory()
{
    const QString directory = QDir(sourceDirectory()).absoluteFilePath(QStringLiteral("Export/ExportH5"));
    QDir().mkpath(directory);
    return QDir(directory).absolutePath();
}

QString resultExportDirectory()
{
    const QString directory = QDir(sourceDirectory()).absoluteFilePath(QStringLiteral("Export/ExportFile"));
    QDir().mkpath(directory);
    return QDir(directory).absolutePath();
}

QString iterationResultDirectory()
{
    const QStringList candidates =
        sourceDirectoryCandidates(QStringLiteral("Export/ExportIteration"));
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
