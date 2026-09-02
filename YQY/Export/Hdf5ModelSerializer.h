#pragma once

#include "Base/EmptyOUT.h"

#include <QString>

class StructureData;

class Hdf5ModelSerializer final
{
public:
    bool importModel(const QString& filePath, _OUT StructureData* structure) const; // 读取模型和分析配置
    bool exportModel(const QString& filePath, const StructureData* structure,
                     const QString& sourceModelName = QString()) const; // 写入模型和分析配置
    bool restoreLastDynamicState(const QString& filePath, _OUT StructureData* structure, _OUT double* time = nullptr,
                                 _OUT int* stepId = nullptr) const; // 恢复最后一帧动力学状态
};
