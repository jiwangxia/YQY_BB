#pragma once

#include <QString>

namespace ApplicationPaths
{
// 返回程序产物根目录，与源码目录及进程当前目录分离。
QString workspaceRootDirectory();
// 返回模型导入文件目录。
QString importFileDirectory();
// 返回气动数据目录。
QString aerodynamicDataDirectory();
// 返回 Export 根目录；指定结果文件时返回本次算例共用的结果目录。
QString resultExportDirectory(const QString& resultFile = QString());
// 返回并创建 HDF5 结果目录；已有输出文件路径优先，保留用户指定的父目录。
QString hdf5ResultDirectory(const QString& sourceModelFile = QString(),
                            const QString& configuredResultFile = QString());
// 按源模型文件名生成默认 HDF5 结果路径。
QString hdf5ResultFilePath(const QString& sourceModelFile);
// 返回并创建统一日志目录。
QString logDirectory();
// 迭代过程与其他结果使用同一导出目录。
QString iterationResultDirectory(const QString& resultFile = QString());
// 返回自动生成模型目录。
QString generatedModelDirectory();
}
