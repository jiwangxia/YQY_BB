#include "ConductorLib.h"
#include <cmath>
#include <algorithm>
#include <vector>

namespace ConductorLib
{

    // =====================================================================
    // [内部类] CatenaryEngine: 负责悬链线数学计算
    // =====================================================================
    class CatenaryEngine
    {
    public:
        double L, h, a, length, lx, ly, gamma, s0, x0, y0, z0;

        CatenaryEngine(const double p1[3], const double p2[3], double g, double stress) : gamma(g), s0(stress), x0(p1[0]), y0(p1[1]), z0(p1[2])
        {
            double dx = p2[0] - p1[0], dy = p2[1] - p1[1], dz = p2[2] - p1[2];
            h = dz;
            L = std::sqrt(dx * dx + dy * dy);
            lx = (L < 1e-6) ? 0 : dx / L;
            ly = (L < 1e-6) ? 0 : dy / L;

            double b = s0 / gamma;
            double c = std::sinh(L / (2.0 * b));
            a = L / 2.0 - b * std::asinh(h / (2.0 * b * c));
            length = b * (std::sinh((L - a) / b) + std::sinh(a / b));
        }

        void GetPoints(double s1, double s2, int nPT, std::vector<RawNode>& outPts, std::vector<double>& outStress)
        {
            outPts.resize(nPT + 1);
            outStress.resize(nPT + 1);
            double b = s0 / gamma;
            double chab = std::cosh(a / b), shab = std::sinh(a / b);
            double ds = (length - s1 - s2) / (nPT);

            for (int i = 0; i < nPT + 1; ++i)
            {
                double si = s1 + i * ds;
                double xi = a + b * std::asinh(si / b - shab);
                double cx = std::cosh((xi - a) / b);
                outPts[i] = { x0 + xi * lx, y0 + xi * ly, z0 + b * (cx - chab) };
                outStress[i] = s0 * cx;
            }
        }
    };

    // =====================================================================
    // [核心逻辑] 导线模型生成器
    // =====================================================================
    BundleResult Generator::CreateBundle(const double start[3], const double end[3], const Config& cfg)
    {
        BundleResult Result;
        const double PI = 3.1415926535897932;

        // --- 1. 基础悬链线路径 ---
        CatenaryEngine engine(start, end, cfg.unitWeight, cfg.stress0);
        std::vector<RawNode> baseCurve;
        std::vector<double> baseStress;
        if(cfg.mode == ConnectionMode::Parallel)
        {
            engine.GetPoints(0.0, 0.0, cfg.segments, baseCurve, baseStress);
        }
        else//竖直三角形连接
        engine.GetPoints(cfg.insulatorL, cfg.insulatorL, cfg.segments, baseCurve, baseStress);

        // --- 2. 局部坐标系与偏移计算 ---
        double dx = end[0] - start[0], dy = end[1] - start[1];
        double horiz = std::sqrt(dx * dx + dy * dy);
        double vrx = dy / horiz, vry = -dx / horiz; // 右向量 (V_right)
        double vvx = 0, vvy = 0, vvz = 1.0;         // 上向量 (V_up)

        // 自动计算分裂半径与初始旋转角
        int nBundle = cfg.nBundle;
        if (nBundle != 1 && nBundle != 2 && nBundle != 4 && nBundle != 6 && nBundle != 8) nBundle = 1; // 仅支持指定的分裂数

        double startAng = 0.0;
        double radius = 0.0;

        if (nBundle == 2)
        {
            startAng = 0.0;
            radius = cfg.spacing / 2.0;
        }
        else if (nBundle == 4)
        {
            startAng = -PI / 4.0; // 正方形排列
            radius = cfg.spacing / (2.0 * std::sin(PI / 4.0));
        }
        else if (nBundle == 6)
        {
            startAng = -PI / 6.0; // 六边形
            radius = cfg.spacing / (2.0 * std::sin(PI / 6.0));
        }
        else if (nBundle == 8)
        {
            startAng = -3 * PI / 8.0; // 八边形
            radius = cfg.spacing / (2.0 * std::sin(PI / 8.0));
        }

        std::vector<RawNode> offsets;                // 偏置
        if(nBundle == 1 )
        {
            offsets.push_back({ 0.0, 0.0, 0.0 });
        }
        else
        {
            for (int i = 0; i < nBundle; ++i)
            {
                double anger = startAng + (2.0 * PI * i / nBundle);
                offsets.push_back({ (vrx * std::cos(anger) + vvx * std::sin(anger)) * radius,
                                    (vry * std::cos(anger) + vvy * std::sin(anger)) * radius,
                                     vvz * std::sin(anger) * radius });
            }
        }

        // --- 3. 生成节点与导线单元---
        int nPtsPerWire = (int)baseCurve.size();
        double yokeH = cfg.yokeSpacing / 2.0;
        if (cfg.mode == ConnectionMode::VerticalTriangle)
        {

        }

        if(cfg.mode == ConnectionMode::Parallel)
        {
            for (int i = 0; i < nBundle; ++i)
            {
                for (int j = 0; j < nPtsPerWire; ++j)
                {
                    RawNode node = baseCurve[j];
                    node.x += offsets[i].x; node.y += offsets[i].y; node.z += offsets[i].z;
                    Result.nodes.push_back(node);
                    if (j > 0)
                    {
                        Result.elements.push_back({ (int)Result.nodes.size() - 1, (int)Result.nodes.size(), baseStress[j], 1 });
                    }
                }
            }
        }
        //
        //int nPtsPerWire = (int)baseCurve.size();
        //double yokeH = cfg.yokeSpacing / 2.0;

        //for (int i = 0; i < nBundle; ++i)
        //{
        //    for (int j = 0; j < nPtsPerWire; ++j)
        //    {
        //        RawNode node = baseCurve[j];
        //        // 端部汇聚逻辑
        //        if (j == 0 || j == cfg.segments)
        //        {
        //            if (cfg.mode == ConnectionMode::VerticalTriangle)
        //            {
        //                double proj = offsets[i].x * vrx + offsets[i].y * vry;
        //                double side = (proj >= -1e-7) ? yokeH : -yokeH;     // 左右分组
        //                node.x += vrx * side; node.y += vry * side;
        //            }
        //            else
        //            {
        //                node.x += offsets[i].x; node.y += offsets[i].y; node.z += offsets[i].z;
        //            }
        //        }
        //        else
        //        {
        //            node.x += offsets[i].x; node.y += offsets[i].y; node.z += offsets[i].z;
        //        }
        //        Result.nodes.push_back(node);
        //        if (j > 0) 
        //        {
        //            Result.elements.push_back({ (int)Result.nodes.size() - 2, (int)Result.nodes.size() - 1, 0, baseStress[j] });
        //        }
        //    }
        //}

        // --- 4. 生成间隔棒单元 ---
        if (cfg.numSpacers > 0 && nBundle > 1)
        {
            double step = (double)cfg.segments / (cfg.numSpacers + 1);
            for (int s = 1; s <= cfg.numSpacers; ++s)
            {
                int j = (int)std::round(s * step);
                for (int i = 0; i < nBundle; ++i)
                {
                    Result.elements.push_back({ i * nPtsPerWire + j, ((i + 1) % nBundle) * nPtsPerWire + j, 0.0, 2 });//间隔棒无应力
                }
            }
        }

        if (cfg.ToYUP)
        {
            for (auto& node : Result.nodes)
            {
                double tempZ = node.z;
                node.z = -node.y;
                node.y = tempZ;
            }
        }
        return Result;
    }

} // namespace ConductorLib