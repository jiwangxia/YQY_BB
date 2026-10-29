#include "ApplicationPaths.h"
#include "Utility/ResultExportPaths.h"

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

static QStringList resourceDirectoryCandidates(const QString& projectRelativePath)
{
    const QDir applicationDirectory(QCoreApplication::applicationDirPath());
    // 开发环境保留原源码查找顺序；部署时也支持不携带工程文件的相邻资源目录。
    return {QDir(sourceDirectory()).absoluteFilePath(projectRelativePath),
            applicationDirectory.absoluteFilePath(projectRelativePath),
            applicationDirectory.absoluteFilePath(QStringLiteral("YQY/%1").arg(projectRelativePath)),
            applicationDirectory.absoluteFilePath(QStringLiteral("../%1").arg(projectRelativePath))};
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
    // 用绝对源路径区分不同目录中的同名模型，避免结果目录相互覆盖。
    const QByteArray key =
        QCryptographicHash::hash(sourceInfo.absoluteFilePath().toUtf8(), QCryptographicHash::Sha1).toHex().left(10);
    return QDir(ApplicationPaths::workspaceRootDirectory())
        .absoluteFilePath(QStringLiteral("Projects/%1_%2").arg(name, QString::fromLatin1(key)));
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
    return existingDirectory(resourceDirectoryCandidates(QStringLiteral("Import/ImportFile")), QDir::currentPath());
}

QString aerodynamicDataDirectory()
{
    // Output_Data 保留原始工作簿测量值；Input_Data 含后续半周镜像数据，不能静默替代原始数据。
    return existingDirectory(resourceDirectoryCandidates(QStringLiteral("Import/Aero_Data/Output_Data")), QString());
}

QString hdf5ResultDirectory(const QString& sourceModelFile, const QString& configuredResultFile)
{
    const QString configuredFile = configuredResultFile.trimmed();
    if (!configuredFile.isEmpty())
        return createDirectory(QFileInfo(configuredFile).absolutePath());

    const QString baseDirectory =
        sourceModelFile.trimmed().isEmpty() ? workspaceRootDirectory() : projectDirectory(sourceModelFile);
    return createDirectory(QDir(baseDirectory).absoluteFilePath(QStringLiteral("Results")));
}

QString hdf5ResultFilePath(const QString& sourceModelFile)
{
    return QDir(hdf5ResultDirectory(sourceModelFile))
        .filePath(QFileInfo(sourceModelFile).completeBaseName() + QStringLiteral(".h5"));
}

QString logDirectory()
{
    return createDirectory(QDir(workspaceRootDirectory()).absoluteFilePath(QStringLiteral("Logs")));
}

QString resultExportDirectory(const QString& resultFile)
{
    if (!resultFile.trimmed().isEmpty())
        return ResultExportPaths::directory(resultFile);
    return createDirectory(QDir(workspaceRootDirectory()).absoluteFilePath(QStringLiteral("Export")));
}

QString iterationResultDirectory(const QString& resultFile)
{
    return resultExportDirectory(resultFile);
}

QString generatedModelDirectory()
{
    return createDirectory(QDir(workspaceRootDirectory()).absoluteFilePath(QStringLiteral("GeneratedModels")));
}

}
