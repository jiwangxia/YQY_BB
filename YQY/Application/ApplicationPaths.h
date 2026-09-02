#pragma once

#include <QString>

namespace ApplicationPaths
{
// 返回工作区根目录。
QString workspaceRootDirectory();
// 返回模型导入文件目录。
QString importFileDirectory();
// 返回气动数据目录。
QString aerodynamicDataDirectory();
// 返回结果文本导出目录。
QString resultExportDirectory(const QString& resultFile = QString());
// 返回 HDF5 结果目录。
QString hdf5ResultDirectory(const QString& sourceModelFile = QString());
// 返回迭代过程结果目录。
QString iterationResultDirectory(const QString& resultFile = QString());
// 返回自动生成模型目录。
QString generatedModelDirectory();
// 返回指定验证项的输出目录。
QString verificationOutputDirectory(const QString& verificationName);
}
