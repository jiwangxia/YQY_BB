#include "Conductor.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <numbers>
#include <vector>

namespace Conductor
{
bool IsValidBundleRotation(const ConductorConfig& config)
{
    if (!std::isfinite(config.bundleRotationDegrees))
        return false;
    if (config.nBundle == 1)
        return config.bundleRotationDegrees == 0.0;
    if (config.nBundle != 2 && config.nBundle != 4 && config.nBundle != 6 && config.nBundle != 8)
        return false;
    return std::abs(config.bundleRotationDegrees) < 180.0 / config.nBundle;
}

// =====================================================================
// [内部类] Catenarymodel: 负责悬链线数学计算
// =====================================================================
class CatenaryModel
{
public:
    double L, h, ArcLength, lx, ly, lineWeight, horizontalTension;
    double x1, y1; // 起点坐标
    double a;      //应力比容重
    double X0;     //最低点位置（水平距离起点的距离）
    double C;      //悬链线公式的 常数项

    /**
        * @param Point1 起点坐标 [x, y, z]
        * @param Point2 终点坐标 [x, y, z]
        * @param weight 单位长度线重，单位 N/m
        * @param tension 最低点水平张力，单位 N
        */
    CatenaryModel(const double point1[3], const double point2[3], double weight, double tension)
        : x1(point1[0])
        , y1(point1[1])
        , lineWeight(weight)
        , horizontalTension(tension)
    {
        double dx = point2[0] - point1[0], dy = point2[1] - point1[1], dz = point2[2] - point1[2];
        h = dz;
        L = std::sqrt(dx * dx + dy * dy);
        if (L < 1e-7)
        {
            lx = 0;
            ly = 0;
        }
        else
        {
            lx = dx / L;
            ly = dy / L;
        }

        a = horizontalTension / lineWeight;
        double hsinhLa = h / (2 * a * std::sinh(L / 2.0 / a));
        X0 = L / 2.0 - a * std::asinh(hsinhLa);
        ArcLength = a * (std::sinh((L - X0) / a) + std::sinh(X0 / a));
        // X 使用沿档距方向的局部水平坐标，左挂点对应 X = 0。
        C = point1[2] - a * std::cosh(X0 / a);
    }

    /**
        * @param s1         左侧耐张串长度 (扣除长度)
        * @param s2         右侧耐张串长度 (扣除长度)
        * @param nPT        分段数
        * @param OutNode    输出节点集合  （不包括挂点）
        */
    bool GetPoints(double s1, double s2, int nPT, _OUT std::vector<RawNode>& OutNode)
    {
        if (nPT <= 0 || nPT > std::numeric_limits<int>::max() - 3 || !std::isfinite(ArcLength) ||
            !std::isfinite(a) || a <= 0.0 || !std::isfinite(X0) || !std::isfinite(C))
            return false;
        double eps = 1e-4;
        s1 = std::max(s1, eps);
        s2 = std::max(s2, eps);
        double sinhXa = std::sinh(X0 / a);

        double effectiveLength = ArcLength - s1 - s2;
        if (!std::isfinite(effectiveLength) || effectiveLength <= 0.0)
        {
            return false;
        }

        OutNode.resize(nPT + 1);
        double ds = effectiveLength / nPT;

        for (int i = 0; i < nPT + 1; ++i)
        {
            double si = s1 + i * ds; //用当前弧长反推水平X
            double X = X0 + a * std::asinh(si / a - sinhXa);
            double Z = a * cosh((X - X0) / a) + C;
            const RawNode node{x1 + X * lx, y1 + X * ly, Z};
            if (!std::isfinite(node.x) || !std::isfinite(node.y) || !std::isfinite(node.z))
                return false;
            OutNode[i] = node;
        }
        return true;
    }

    RawNode PointAtHorizontalDistance(double X) const
    {
        return {x1 + X * lx, y1 + X * ly, a * std::cosh((X - X0) / a) + C};
    }
};

// 将指定的间隔棒位置作为悬链线网格的锚点，不增加节点或单元。
// 锚点之间按原始等弧长节点的水平坐标单调映射，再由悬链线公式重算高度。
static bool PlaceRequiredNodes(const CatenaryModel& model, const std::vector<double>& requiredPositions,
                               _OUT std::vector<RawNode>& generatedNodes, _OUT std::string& error)
{
    if (requiredPositions.empty())
        return true;

    const std::size_t nodeCount = generatedNodes.size() + 2; // 包含两端挂点
    std::vector<double> positions = requiredPositions;
    if (std::any_of(positions.cbegin(), positions.cend(), [](double position) { return !std::isfinite(position); }))
    {
        error = "间隔棒水平位置必须为有限数值";
        return false;
    }
    std::sort(positions.begin(), positions.end());
    const double minElementLength =
        std::max(1.0e-6, std::min(0.01, model.L / (10.0 * static_cast<double>(nodeCount - 1))));
    if (positions.size() > generatedNodes.size())
    {
        error = "间隔棒位置数量超过可分配的导线内部节点数；请增加导线离散段数";
        return false;
    }
    for (std::size_t j = 0; j < positions.size(); ++j)
    {
        if (!std::isfinite(positions[j]) || positions[j] <= minElementLength ||
            positions[j] >= model.L - minElementLength)
        {
            error = "间隔棒必须位于水平档距内部，且距两端至少 " + std::to_string(minElementLength) + " m";
            return false;
        }
        if (j > 0 && positions[j] - positions[j - 1] <= minElementLength)
        {
            error = "相邻间隔棒过近或位置重复；最小允许水平间距为 " +
                    std::to_string(minElementLength) + " m";
            return false;
        }
    }

    std::vector<double> originalX(nodeCount);
    originalX.front() = 0.0;
    originalX.back() = model.L;
    for (std::size_t i = 1; i + 1 < nodeCount; ++i)
    {
        const auto& point = generatedNodes[i - 1];
        originalX[i] = (point.x - model.x1) * model.lx + (point.y - model.y1) * model.ly;
    }
    for (std::size_t i = 1; i < nodeCount; ++i)
    {
        if (!std::isfinite(originalX[i]) || originalX[i] <= originalX[i - 1])
        {
            error = "导线原始悬链线节点的水平位置未严格递增";
            return false;
        }
    }

    // 在严格递增的内部节点序号中寻找与目标 x 最接近的分配。
    // 检查每一段映射后单元的最短长度，避免精确落点产生极短单元。
    const double infinity = std::numeric_limits<double>::infinity();
    const std::size_t targetCount = positions.size();
    std::vector<std::vector<double>> cost(targetCount, std::vector<double>(nodeCount, infinity));
    std::vector<std::vector<std::size_t>> parent(targetCount, std::vector<std::size_t>(nodeCount, 0));
    double prefixMinGap = infinity;
    for (std::size_t i = 1; i + 1 < nodeCount; ++i)
    {
        prefixMinGap = std::min(prefixMinGap, originalX[i] - originalX[i - 1]);
        if (nodeCount - 2 - i < targetCount - 1)
            continue;
        const double scale = positions.front() / originalX[i];
        if (scale * prefixMinGap + 1.0e-10 >= minElementLength)
            cost[0][i] = std::pow(originalX[i] - positions.front(), 2);
    }
    for (std::size_t j = 1; j < targetCount; ++j)
    {
        for (std::size_t i = j + 1; i + 1 < nodeCount; ++i)
        {
            if (nodeCount - 2 - i < targetCount - j - 1)
                continue;
            double intervalMinGap = infinity;
            for (std::size_t k = i - 1; k >= j; --k)
            {
                intervalMinGap = std::min(intervalMinGap, originalX[k + 1] - originalX[k]);
                if (std::isfinite(cost[j - 1][k]))
                {
                    const double scale = (positions[j] - positions[j - 1]) / (originalX[i] - originalX[k]);
                    if (scale * intervalMinGap + 1.0e-10 >= minElementLength)
                    {
                        const double candidate = cost[j - 1][k] + std::pow(originalX[i] - positions[j], 2);
                        if (candidate < cost[j][i])
                        {
                            cost[j][i] = candidate;
                            parent[j][i] = k;
                        }
                    }
                }
                if (k == j)
                    break;
            }
        }
    }

    std::size_t lastIndex = 0;
    double bestCost = infinity;
    double suffixMinGap = infinity;
    for (std::size_t i = nodeCount - 2; i >= targetCount; --i)
    {
        suffixMinGap = std::min(suffixMinGap, originalX[i + 1] - originalX[i]);
        const double scale = (model.L - positions.back()) / (model.L - originalX[i]);
        if (scale * suffixMinGap + 1.0e-10 >= minElementLength && cost.back()[i] < bestCost)
        {
            bestCost = cost.back()[i];
            lastIndex = i;
        }
        if (i == targetCount)
            break;
    }
    if (!std::isfinite(bestCost))
    {
        error = "无法把间隔棒位置分配到互不重复且有序的导线节点；请增加离散段数或增大间隔棒间距";
        return false;
    }

    std::vector<std::size_t> assignedIndices(targetCount);
    assignedIndices.back() = lastIndex;
    for (std::size_t j = targetCount - 1; j > 0; --j)
        assignedIndices[j - 1] = parent[j][assignedIndices[j]];

    std::vector<std::size_t> anchorIndices = {0};
    anchorIndices.insert(anchorIndices.end(), assignedIndices.begin(), assignedIndices.end());
    anchorIndices.push_back(nodeCount - 1);
    std::vector<double> anchorPositions = {0.0};
    anchorPositions.insert(anchorPositions.end(), positions.begin(), positions.end());
    anchorPositions.push_back(model.L);
    for (std::size_t a = 1; a < anchorIndices.size(); ++a)
    {
        const std::size_t left = anchorIndices[a - 1];
        const std::size_t right = anchorIndices[a];
        const double scale = (anchorPositions[a] - anchorPositions[a - 1]) /
                             (originalX[right] - originalX[left]);
        for (std::size_t i = left + 1; i <= right && i + 1 < nodeCount; ++i)
        {
            const double x = anchorPositions[a - 1] + (originalX[i] - originalX[left]) * scale;
            generatedNodes[i - 1] = model.PointAtHorizontalDistance(x);
        }
    }
    return true;
}

// =====================================================================
// 导线模型生成器
// =====================================================================
BundleResult Generator::CreateBundle(const double start[3], const double end[3], double leftCutLength,
                                     double rightCutLength, double horizontalTension, double lineWeight,
                                     const ConductorConfig& config,
                                     const std::vector<double>& requiredHorizontalPositions,
                                     _OUT std::string* error)
{
    BundleResult Result;

    if (error)
        error->clear();
    if (!start || !end || !IsValidBundleRotation(config) || config.segments <= 0 ||
        config.segments > std::numeric_limits<int>::max() - 3 || !std::isfinite(config.initialStress) ||
        config.initialStress <= 0.0 || !std::isfinite(config.spacing) || config.spacing < 0.0 ||
        !std::isfinite(lineWeight) || lineWeight <= 0.0 || !std::isfinite(horizontalTension) ||
        horizontalTension <= 0.0 || !std::isfinite(leftCutLength) || leftCutLength < 0.0 ||
        !std::isfinite(rightCutLength) || rightCutLength < 0.0)
    {
        if (error)
            *error = "导线生成参数无效，数值必须有限且满足取值范围";
        return Result;
    }
    for (int component = 0; component < 3; ++component)
    {
        if (!std::isfinite(start[component]) || !std::isfinite(end[component]))
        {
            if (error)
                *error = "导线挂点坐标必须为有限数值";
            return Result;
        }
    }
    const double horizontalSpan = std::hypot(end[0] - start[0], end[1] - start[1]);
    if (!std::isfinite(horizontalSpan) || horizontalSpan <= 1.0e-7)
    {
        if (error)
            *error = "导线水平档距过小，无法生成悬链线";
        return Result;
    }

    // --- 1. 基础悬链线路径 ---
    CatenaryModel model(start, end, lineWeight, horizontalTension);
    std::vector<RawNode> GenerateNodes; // 生成的导线节点，中间节点不含挂点

    if (!model.GetPoints(leftCutLength, rightCutLength, config.segments, GenerateNodes))
    {
        return Result;
    }
    if (!requiredHorizontalPositions.empty())
    {
        std::string positionError;
        if (!PlaceRequiredNodes(model, requiredHorizontalPositions, GenerateNodes, positionError))
        {
            if (error)
                *error = std::move(positionError);
            return Result;
        }
    }

    // --- 2. 局部坐标系与偏移计算 ---
    double dx = end[0] - start[0], dy = end[1] - start[1];
    std::vector<RawNode> offsets; // 偏置
    Offset(config, dx, dy, offsets);

    // --- 3. 生成节点与导线单元---
    int nBundle = config.nBundle;
    int Nodesize = (int)GenerateNodes.size();
    for (int i = 0; i < nBundle; ++i)
    {
        //传入起始挂点坐标
        RawNode startAttach = {start[0] + offsets[i].x, start[1] + offsets[i].y, start[2] + offsets[i].z};
        Result.wiresNode[i].push_back(startAttach);

        //传入中间点坐标
        for (int j = 0; j < Nodesize; ++j)
        {
            RawNode node = GenerateNodes[j];

            node.x += offsets[i].x;
            node.y += offsets[i].y;
            node.z += offsets[i].z;

            Result.wiresNode[i].push_back(node);
            int currIndex = (int)Result.wiresNode[i].size();

            Result.wiresElement[i].push_back({currIndex - 1, currIndex, config.initialStress});
        }

        //传入最终挂点坐标
        RawNode endAttach = {end[0] + offsets[i].x, end[1] + offsets[i].y, end[2] + offsets[i].z};
        Result.wiresNode[i].push_back(endAttach);

        int lastIdx = (int)Result.wiresNode[i].size();
        Result.wiresElement[i].push_back({lastIdx - 1, lastIdx, config.initialStress});
    }

    return Result;
}

void Generator::Offset(const ConductorConfig& config, double dx, double dy, std::vector<RawNode>& offsets)
{
    offsets.clear();
    double horiz = std::sqrt(dx * dx + dy * dy);
    if (horiz <= 1e-7)
    {
        offsets.push_back({0.0, 0.0, 0.0});
        return;
    }
    // 气动剖面绑定会把单元轴规范到主分量为正的方向。截面布置也使用同一
    // 方向约定，否则反向建立同一档导线时，wireId 对应的物理方位会翻转，
    // 而 BundleAeroMapper 仍按正向方位查表。
    const bool reverseAxis = std::abs(dx) >= std::abs(dy) ? dx < 0.0 : dy < 0.0;
    if (reverseAxis)
    {
        dx = -dx;
        dy = -dy;
    }
    double vrx = dy / horiz, vry = -dx / horiz; // 规范轴向下的右向量 (V_right)
    double vvx = 0, vvy = 0, vvz = 1.0;         // 上向量 (V_up)

    // 自动计算分裂半径与初始旋转角
    int nBundle = config.nBundle;
    if (nBundle != 1 && nBundle != 2 && nBundle != 4 && nBundle != 6 && nBundle != 8)
        nBundle = 1; // 仅支持指定的分裂数

    double startAng = 0.0;
    double radius = 0.0;

    if (nBundle == 2)
    {
        startAng = 0.0;
        radius = config.spacing / 2.0;
    }
    else if (nBundle == 4)
    {
        startAng = -std::numbers::pi / 4.0; // 正方形排列
        radius = config.spacing / (2.0 * std::sin(std::numbers::pi / 4.0));
    }
    else if (nBundle == 6)
    {
        startAng = -std::numbers::pi / 3.0; // 六边形
        radius = config.spacing / (2.0 * std::sin(std::numbers::pi / 6.0));
    }
    else if (nBundle == 8)
    {
        startAng = -3 * std::numbers::pi / 8.0; // 八边形
        radius = config.spacing / (2.0 * std::sin(std::numbers::pi / 8.0));
    }
    startAng += config.bundleRotationDegrees * std::numbers::pi / 180.0;

    if (nBundle == 1)
    {
        offsets.push_back({0.0, 0.0, 0.0});
    }
    else
    {
        for (int i = 0; i < nBundle; ++i)
        {
            double anger = startAng + (2.0 * std::numbers::pi * i / nBundle);
            offsets.push_back({(vrx * std::cos(anger) + vvx * std::sin(anger)) * radius,
                               (vry * std::cos(anger) + vvy * std::sin(anger)) * radius,
                               vvz * std::sin(anger) * radius});
        }
    }
}
}
