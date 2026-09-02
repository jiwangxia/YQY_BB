#include "GUI/Dialogs/FileDialogService.h"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QSettings>

namespace
{
QString lastDirectory(const QString& settingsKey, const QString& fallbackDirectory)
{
    const QString storedDirectory = QSettings().value(QStringLiteral("fileDialog/%1").arg(settingsKey)).toString();
    return QFileInfo(storedDirectory).isDir() ? storedDirectory : fallbackDirectory;
}

void rememberDirectory(const QString& settingsKey, const QString& filePath)
{
    const QFileInfo fileInfo(filePath);
    if (fileInfo.absoluteDir().exists())
        QSettings().setValue(QStringLiteral("fileDialog/%1").arg(settingsKey), fileInfo.absolutePath());
}
}

namespace FileDialogService
{
QStringList selectOpenFiles(QWidget* parent, const QString& title, const QString& initialDirectory,
                            const QString& nameFilter, const QString& settingsKey)
{
    const QStringList selectedFiles =
        QFileDialog::getOpenFileNames(parent, title, lastDirectory(settingsKey, initialDirectory), nameFilter);
    if (!selectedFiles.isEmpty())
        rememberDirectory(settingsKey, selectedFiles.constFirst());
    return selectedFiles;
}

QString selectSaveFile(QWidget* parent, const QString& title, const QString& defaultFilePath, const QString& nameFilter,
                       const QString& settingsKey)
{
    const QFileInfo defaultFileInfo(defaultFilePath);
    const QString initialPath =
        QDir(lastDirectory(settingsKey, defaultFileInfo.absolutePath())).absoluteFilePath(defaultFileInfo.fileName());
    const QString selectedFile = QFileDialog::getSaveFileName(parent, title, initialPath, nameFilter);
    if (!selectedFile.isEmpty())
        rememberDirectory(settingsKey, selectedFile);
    return selectedFile;
}
}
