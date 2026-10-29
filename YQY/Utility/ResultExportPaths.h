#pragma once

#include <QString>

// 界面线程共用的结果导出命名；同一结果版本复用目录，重新计算后分配新目录。
namespace ResultExportPaths
{
void setSourceModel(const QString& resultFile, const QString& sourceModelFile);
QString caseName(const QString& resultFile);
QString directory(const QString& resultFile);
QString filePath(const QString& resultFile, const QString& description, const QString& suffix);
}
