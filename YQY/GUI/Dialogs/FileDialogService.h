#pragma once

#include <QString>
#include <QStringList>

class QWidget;

namespace FileDialogService
{
QStringList selectOpenFiles(QWidget* parent, const QString& title, const QString& initialDirectory,
                            const QString& nameFilter, const QString& settingsKey); // 选择待打开文件并记忆最近使用目录
QString selectSaveFile(QWidget* parent, const QString& title, const QString& defaultFilePath, const QString& nameFilter,
                       const QString& settingsKey, bool rememberLastDirectory = true); // 可优先使用传入的默认目录
}
