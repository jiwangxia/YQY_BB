#include "ResultExportPaths.h"
#include "Application/ApplicationPaths.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QHash>
#include <QRegularExpression>

static QHash<QString, QString> sourceModels;
static QHash<QString, QString> exportDirectories;

static QString resultKey(const QString& resultFile)
{
    const QFileInfo info(resultFile);
    const QString canonical = info.canonicalFilePath();
    return QDir::cleanPath(canonical.isEmpty() ? info.absoluteFilePath() : canonical).toCaseFolded();
}

namespace ResultExportPaths
{
void setSourceModel(const QString& resultFile, const QString& sourceModelFile)
{
    if (!sourceModelFile.trimmed().isEmpty())
        sourceModels.insert(resultKey(resultFile), sourceModelFile);
}

QString caseName(const QString& resultFile)
{
    const QString source = sourceModels.value(resultKey(resultFile));
    QString name = QFileInfo(source.isEmpty() ? resultFile : source).completeBaseName().trimmed();
    if (source.isEmpty() || resultKey(source) == resultKey(resultFile))
        name.remove(QRegularExpression(QStringLiteral("(?:_case_\\d+)+$")));
    name.replace(QRegularExpression(QStringLiteral("[<>:\"/\\\\|?*\\x00-\\x1f]")), QStringLiteral("_"));
    name.remove(QRegularExpression(QStringLiteral("[. ]+$")));
    return name.isEmpty() ? QStringLiteral("未命名算例") : name;
}

QString directory(const QString& resultFile)
{
    if (resultFile.trimmed().isEmpty())
        return QString();
    const QFileInfo info(resultFile);
    const QString key = QStringLiteral("%1|%2|%3")
                            .arg(resultKey(resultFile))
                            .arg(info.lastModified().toMSecsSinceEpoch())
                            .arg(info.size());
    const QString cached = exportDirectories.value(key);
    if (!cached.isEmpty() && QDir(cached).exists())
        return cached;

    const QDir root(ApplicationPaths::resultExportDirectory());
    if (!root.exists())
        return QString();
    const QString baseName = caseName(resultFile) + QStringLiteral("结果");
    for (int index = 0;; ++index)
    {
        const QString name = index == 0 ? baseName : QStringLiteral("%1_%2").arg(baseName).arg(index);
        const QString path = root.absoluteFilePath(name);
        // mkdir 同时承担占用目录名的作用，其他进程先创建时继续选择下一个编号。
        if (root.mkdir(name))
        {
            exportDirectories.insert(key, path);
            return path;
        }
        if (!QFileInfo::exists(path))
            return QString();
    }
}

QString filePath(const QString& resultFile, const QString& description, const QString& suffix)
{
    const QString path = directory(resultFile);
    return path.isEmpty() ? QString()
                          : QDir(path).absoluteFilePath(caseName(resultFile) + QLatin1Char('-') + description +
                                                        QLatin1Char('.') + suffix);
}
}
