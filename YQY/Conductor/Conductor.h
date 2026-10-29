#pragma once
#include <vector>
#include <map>
#include <string>
#include "Base/EmptyOUT.h"

namespace Conductor
{
struct RawNode
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int id = 0;
};
struct RawElement
{
    int iNode = 0;
    int jNode = 0;
    double initialStress = 0.0;
};

struct BundleResult
{
    std::map<int, std::vector<RawNode>> wiresNode;
    std::map<int, std::vector<RawElement>> wiresElement;
};

struct ConductorConfig
{
    int nBundle = 0;            // 分裂数：1、2、4、6 或 8，由调用方显式提供
    double spacing = 0.0;       // 相邻子导线间距，单位 m
    double bundleRotationDegrees = 0.0; // 束截面相对默认方位的旋角：局部右向量朝 +Z 为正
    int segments = 0;           // 每根子导线的离散段数
    double initialStress = 0.0; // 初始水平应力，单位 Pa
};

// 当前耐张分组和悬垂顶部连板按默认 wireId 方位建立，旋角不能跨过相邻方位的半步。
bool IsValidBundleRotation(const ConductorConfig& config);

class Generator
{
public:
    // horizontalTension 的单位为 N，lineWeight 的单位为 N/m。
    // 二者由 ConductorModelBuilder 根据材料密度、截面面积和初始应力推导。
    static BundleResult CreateBundle(const double start[3], const double end[3], double leftCutLength,
                                     double rightCutLength, double horizontalTension, double lineWeight,
                                     const ConductorConfig& config,
                                     const std::vector<double>& requiredHorizontalPositions = {},
                                     _OUT std::string* error = nullptr);
    // 单档和多档悬垂站共用相同方位与 wireId 约定。
    static void Offset(const ConductorConfig& config, double dx, double dy, _OUT std::vector<RawNode>& offsets);
};
}
