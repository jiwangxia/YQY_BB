#pragma once

#include "Base/EmptyOUT.h"

#include <QJsonObject>
#include <QString>
#include <memory>

class StructureData;

namespace SingleConductorCase
{
struct ModelOptions
{
    bool beam = false;
    double duration = 300.0;
    double attackDegrees = 40.0;
    double initialStress = 60.0e6;
    double windSpeed = 10.0;
    double airDensity = 1.225;
    int segments = 50;
};

// 材料和气动工况取已核对参数；论文未给的初张力、面积和边界补充假设写入 metadata。
std::shared_ptr<StructureData> BuildModel(const ModelOptions& options, _OUT QJsonObject& metadata,
                                         _OUT QString& error);
bool ExportModel(const ModelOptions& options, const QString& filePath, _OUT QJsonObject& metadata,
                  _OUT QString& error);
}
