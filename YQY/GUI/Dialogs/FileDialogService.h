#pragma once

#include <QString>
#include <QStringList>

class QWidget;

namespace FileDialogService
{
QStringList selectOpenFiles(QWidget* parent, const QString& title, const QString& initialDirectory,
                            const QString& nameFilter, const QString& settingsKey);
QString selectSaveFile(QWidget* parent, const QString& title, const QString& defaultFilePath, const QString& nameFilter,
                       const QString& settingsKey);
}
