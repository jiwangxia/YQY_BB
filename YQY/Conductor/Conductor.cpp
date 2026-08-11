#include "Conductor.h"
#include <cmath>
#include <algorithm>
#include <numbers>
#include <vector>

namespace Conductor
{
    // =====================================================================
    // [内部类] Catenarymodel: 负责悬链线数学计算
    // =====================================================================
    class CatenaryModel
    {
    public:
        double L, h, ArcLength, lx, ly, lineWeight, horizontalTension;
        double x1, y1; // 起点坐标
        double a;//应力比容重
        double X0;//最低点位置（水平距离起点的距离）
        double C;//悬链线公式的 常数项

        /**
        * @param Point1 起点坐标 [x, y, z]
        * @param Point2 终点坐标 [x, y, z]
        * @param weight 单位长度线重，单位 N/m
        * @param tension 最低点水平张力，单位 N
        */
        CatenaryModel(const double point1[3], const double point2[3], double weight, double tension)
            : x1(point1[0]), y1(point1[1]), lineWeight(weight), horizontalTension(tension)
        {
            double dx = point2[0] - point1[0], dy = point2[1] - point1[1], dz = point2[2] - point1[2];
            h = dz;
            L = std::sqrt(dx * dx + dy * dy);
            if (L < 1e-7)
            {
                lx = 0; ly = 0;
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
        bool GetPoints(double s1, double s2, int nPT, std::vector<RawNode>& OutNode)
        {
            double eps = 1e-4;
            s1 = std::max(s1, eps);
            s2 = std::max(s2, eps);
            double sinhXa = std::sinh(X0 / a);

            double effectiveLength = ArcLength - s1 - s2;
            if (effectiveLength < 0)
            {
                return false;
            }

            OutNode.resize(nPT + 1);
            double ds = effectiveLength / nPT;

            for (int i = 0; i < nPT + 1; ++i)
            {
                double si = s1 + i * ds;  //用当前弧长反推水平X
                double X = X0 + a * std::asinh(si / a - sinhXa);
                double Z = a * cosh((X - X0) / a) + C;

                OutNode[i] = { x1 + X * lx,y1 + X * ly,Z };
            }
            return true;
        }
    };

    // =====================================================================
    // 导线模型生成器
    // =====================================================================
    BundleResult Generator::CreateBundle(const double start[3], const double end[3],
        double leftCutLength, double rightCutLength, double horizontalTension,
        double lineWeight, const ConductorConfig& config)
    {
        BundleResult Result;

        if (config.nBundle <= 0 || config.segments <= 0 || config.initialStress <= 0.0 || lineWeight <= 0.0 || horizontalTension <= 0.0)
        {
            return Result;
        }

        // --- 1. 基础悬链线路径 ---
        CatenaryModel model(start, end, lineWeight, horizontalTension);
        std::vector<RawNode> GenerateNodes;       // 生成的导线节点，中间节点不含挂点

        if (!model.GetPoints(leftCutLength, rightCutLength, config.segments, GenerateNodes))
        {
            return Result;
        }

        // --- 2. 局部坐标系与偏移计算 ---
        double dx = end[0] - start[0], dy = end[1] - start[1];
        std::vector<RawNode> offsets;             // 偏置
        Offset(config, dx, dy, offsets);

        // --- 3. 生成节点与导线单元---
        int    nBundle = config.nBundle;
        int    Nodesize = (int)GenerateNodes.size();
        for (int i = 0; i < nBundle; ++i)
        {
            //传入起始挂点坐标
            RawNode startAttach = { start[0] + offsets[i].x, start[1] + offsets[i].y, start[2] + offsets[i].z };
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

                Result.wiresElement[i].push_back({ currIndex - 1, currIndex, config.initialStress });
            }

            //传入最终挂点坐标
            RawNode endAttach = { end[0] + offsets[i].x, end[1] + offsets[i].y, end[2] + offsets[i].z };
            Result.wiresNode[i].push_back(endAttach);

            int lastIdx = (int)Result.wiresNode[i].size();
            Result.wiresElement[i].push_back({ lastIdx - 1, lastIdx, config.initialStress });
        }

        return Result;
    }

    void Generator::Offset(const ConductorConfig& config, double dx, double dy,
        std::vector<RawNode>& offsets)
    {
        double horiz = std::sqrt(dx * dx + dy * dy);
        if (horiz <= 1e-7)
        {
            offsets.clear();
            offsets.push_back({ 0.0, 0.0, 0.0 });
            return;
        }
        double vrx = dy / horiz, vry = -dx / horiz; // 右向量 (V_right)
        double vvx = 0, vvy = 0, vvz = 1.0;         // 上向量 (V_up)

        // 自动计算分裂半径与初始旋转角
        int nBundle = config.nBundle;
        if (nBundle != 1 && nBundle != 2 && nBundle != 4 && nBundle != 6 && nBundle != 8) nBundle = 1; // 仅支持指定的分裂数

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


        if (nBundle == 1)
        {
            offsets.push_back({ 0.0, 0.0, 0.0 });
        }
        else
        {
            for (int i = 0; i < nBundle; ++i)
            {
                double anger = startAng + (2.0 * std::numbers::pi * i / nBundle);
                offsets.push_back({ (vrx * std::cos(anger) + vvx * std::sin(anger)) * radius,
                                    (vry * std::cos(anger) + vvy * std::sin(anger)) * radius,
                                     vvz * std::sin(anger) * radius });
            }
        }

    }
}
