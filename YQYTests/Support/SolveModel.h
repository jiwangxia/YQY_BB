#pragma once

#include "Base/EmptyOUT.h"
#include <QString>
#include <map>
#include <memory>

class StructureData;

namespace SolveModel
{
// 路径相对 TestModels；每次读取独立模型，参数调整在 Run 前完成。
std::shared_ptr<StructureData> Read(const QString& relative, _OUT QString& error);
bool Run(const std::shared_ptr<StructureData>& model, _OUT QString& error,
         _OUT std::map<int, std::shared_ptr<StructureData>>* solvedRegions = nullptr);
}
