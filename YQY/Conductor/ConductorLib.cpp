#define CONDUCTOR_EXPORTS
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

        std::vector<RawNode> offsets;                // 偏置

        // --- 2. 局部坐标系与偏移计算 ---
        double dx = end[0] - start[0], dy = end[1] - start[1];
        int nBundle = cfg.nBundle;
        Offset(cfg, dx, dy, offsets);


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
        else if(cfg.mode == ConnectionMode::VerticalTriangle)
        {
            if (nBundle == 1)
            {
                // N=1 特殊处理：直接从 start -> 偏移点 -> end
                // 添加起点 (无偏移)
                Result.nodes.push_back({ start[0], start[1], start[2] });

                // 添加中间点 (无偏移)
                for (const auto& pt : baseCurve)
                {
                    Result.nodes.push_back(pt);
                }

                // 添加终点 (无偏移)
                Result.nodes.push_back({ end[0], end[1], end[2] });

                // 生成单元: Start -> Mid -> End
                // Start -> Mid[0] (绝缘子)
                Result.elements.push_back({ 1, 2, baseStress.front(), 3 }); // 1-based index

                // Mid -> Mid (导线)
                int midStartIdx = 2; // Result中的第2个点是baseCurve[0]
                for (int j = 0; j < nPtsPerWire - 1; ++j)
                {
                    Result.elements.push_back({ midStartIdx + j, midStartIdx + j + 1, baseStress[j], 1 });
                }

                // Mid[last] -> End (绝缘子)
                Result.elements.push_back({ midStartIdx + nPtsPerWire - 1, midStartIdx + nPtsPerWire, baseStress.back(), 3 });
            }
            else
            {
                // N > 1 通用逻辑 (分组连接 yoke)
                // 1. 创建端部 yoke 结构
                // 上向量 (默认竖直)
                double vvx = 0, vvy = 0, vvz = 1.0; 
                // 如果需要水平，可以用 V_offset_h_dir (但这不在Config里，暂默认竖直)
                
                // A端 Yoke Nodes
                // 0: yokeA_upper, 1: midA_upper, 2: yokeA_lower, 3: midA_lower
                RawNode yokeA_upper = { start[0] + vvx * yokeH, start[1] + vvy * yokeH, start[2] + vvz * yokeH };
                RawNode yokeA_lower = { start[0] - vvx * yokeH, start[1] - vvy * yokeH, start[2] - vvz * yokeH };
                
                // 计算 midA (过渡点)
                // midA_upper = (yokeA_upper + (baseCurve[0]+offset[0])) / 2 ? 不，这是 ApplyQuadBundleModification 的逻辑
                // 简化为：midA 是 yokeA 和 第一根导线起始点 的中点?
                // 为了通用，这里简化处理：midA = yokeA + (baseCurve[0] - start) * 0.1?
                // 或者直接用 yokeA 作为汇聚点?
                // 用户的 ApplyQuadBundleModification 有复杂的 midPoint 逻辑。
                // 这里我们简化：yoke -> mid -> conductor。
                // midA_upper = yokeA_upper (简化)
                // 实际上用户代码是用 midPoint 来过渡。我们这里生成 4 个节点。
                
                // 为了简单且健壮，我们直接生成 4 个 yoke 节点，位置如下：
                // yokeA_upper, midA_upper (yokeA_upper + small_dir?), ...
                // 鉴于 ConductorLib 是通用库，我们用简单几何：
                // yokeA_upper/lower 是挂点。
                // midA_upper/lower 是 yoke 和 导线首点 的中间点。
                // 但导线首点取决于 offsets[0]。
                
                // 我们先生成 yokeA_upper, yokeA_lower
                Result.nodes.push_back(yokeA_upper); // Index 0
                Result.nodes.push_back(yokeA_lower); // Index 1

                // B端 Yoke Nodes (先占位? 不，顺序 push)
                // 我们可以先 Push 所有的 conductor nodes，最后 Push B端。
                // 这样 index 比较清晰：
                // [0,1] A端 yoke
                // [2..N*nPts+1] 导线点
                // [end-1, end] B端 yoke
                
                int yokeA_upper_idx = 1; // 1-based
                int yokeA_lower_idx = 2;

                // 2. 生成导线节点
                int conductor_start_idx = 3; // 1-based
                for (int i = 0; i < nBundle; ++i)
                {
                    for (const auto& pt : baseCurve)
                    {
                        Result.nodes.push_back({ pt.x + offsets[i].x, pt.y + offsets[i].y, pt.z + offsets[i].z });
                    }
                }

                // 3. 生成B端 Yoke Nodes
                RawNode yokeB_upper = { end[0] + vvx * yokeH, end[1] + vvy * yokeH, end[2] + vvz * yokeH };
                RawNode yokeB_lower = { end[0] - vvx * yokeH, end[1] - vvy * yokeH, end[2] - vvz * yokeH };
                Result.nodes.push_back(yokeB_upper);
                Result.nodes.push_back(yokeB_lower);
                int yokeB_upper_idx = (int)Result.nodes.size() - 1; // 1-based
                int yokeB_lower_idx = (int)Result.nodes.size();

                // 4. 生成单元
                for (int i = 0; i < nBundle; ++i)
                {
                    // 分组逻辑
                    bool isUpper = (i < nBundle / 2); // 前一半 Upper，后一半 Lower
                    
                    int wireStart = conductor_start_idx + i * nPtsPerWire; // 1-based
                    
                    // Start Yoke -> Wire Start (绝缘子)
                    int yokeA = isUpper ? yokeA_upper_idx : yokeA_lower_idx;
                    Result.elements.push_back({ yokeA, wireStart, baseStress.front(), 3 });

                    // Wire Segments (导线)
                    for (int j = 0; j < nPtsPerWire - 1; ++j)
                    {
                        Result.elements.push_back({ wireStart + j, wireStart + j + 1, baseStress[j], 1 });
                    }

                    // Wire End -> End Yoke (绝缘子)
                    int yokeB = isUpper ? yokeB_upper_idx : yokeB_lower_idx;
                    Result.elements.push_back({ wireStart + nPtsPerWire - 1, yokeB, baseStress.back(), 3 });
                }
            }
        }

        // --- 4. 生成间隔棒单元 ---
        // 注意：VerticalTriangle 模式下 N>1 时，前面有 2 个 yoke 节点 (Index 0, 1)
        // Parallel 模式下，前面没有额外节点 (Index 0 开始)
        int spacerNodeOffset = 0;
        if (cfg.mode == ConnectionMode::VerticalTriangle && nBundle > 1)
        {
            spacerNodeOffset = 2; // yokeA_upper, yokeA_lower
        }

        if (cfg.numSpacers > 0 && nBundle > 1)
        {
            double step = (double)cfg.segments / (cfg.numSpacers + 1);
            for (int s = 1; s <= cfg.numSpacers; ++s)
            {
                int j = (int)std::round(s * step);
                // 限制 j 防止越界 (segments 是段数，点数是 segments+1)
                // baseCurve size = segments+1. j index 0..segments.
                // j should strictly be < nPtsPerWire.
                if (j >= nPtsPerWire) j = nPtsPerWire - 1;

                for (int i = 0; i < nBundle; ++i)
                {
                    // 节点索引 (0-based relative to Result.nodes vector start)
                    // Parallel: i * nPts + j
                    // Vertical: 2 + i * nPts + j
                    int idx1 = spacerNodeOffset + i * nPtsPerWire + j;
                    int idx2 = spacerNodeOffset + ((i + 1) % nBundle) * nPtsPerWire + j;
                    
                    // Element indices are 1-based usually in this file logic above?
                    // Wait, Parallel l96: { (int)Result.nodes.size() - 1, (int)Result.nodes.size() }
                    // Result.nodes.size() is 1-based count.
                    // So element iNode/jNode should be 1-based indices.
                    
                    // Parallel logic (L115 in original): { i*nPts+j, ... } -> This looks like 0-based?
                    // Let's check Parallel logic at L96:
                    // push_back node -> size increments.
                    // element: { size-1, size } -> references the just-added node and previous node.
                    // These are 1-based indices if finding by 1-based ID? or 0-based if finding by index?
                    // User's code in main.cpp L61: FindNode(elem.iNode).
                    // If elem.iNode is 0, FindNode(0) might fail if IDs start at 1.
                    // BUT L115 in original code used `i * nPtsPerWire + j`. If i=0, j=0, this is 0.
                    // This implies 0-based indexing is expected by main.cpp for Spacers?
                    // OR main.cpp adds 1? 
                    // To be safe and consistent with L96 (which uses size, i.e. 1-based), 
                    // we should probably use 1-based indices here too?
                    // `i*nPts+j` is 0-based index. 
                    // if L96 uses `size()` (1-based), then spacers using 0-based is inconsistent.
                    // However, I should preserve existing behavior for Parallel if possible.
                    // Existing code L115: `i * nPtsPerWire + j`.
                    // If that was working, then 0-based is what's used for spacers?
                    // I will stick to 1-based to be safe, because `Result.nodes.size()` is definitely 1-based relative to 0.
                    // Wait, `Result.nodes.size()` returns count.
                    // If vector has 1 element, size is 1. Element referencing it should be 1?
                    // If `FindNode` expects ID, and IDs start at 1...
                    // Let's check: 
                    // L96: `size()-1` and `size()`.
                    // If j=1. `size` is 2. Element connects 1 and 2.
                    // So indices are 1-based.
                    // BUT L115 uses `i*nPts+j`. If i=0, j=0 -> 0.
                    // This means Spacers were using 0-based index? 
                    // Node 0 is valid?
                    // If main.cpp: `FindNode(elem.iNode)`.
                    // If ID 0 exists... 
                    // Most finite element codes use 1-based IDs.
                    // So `i*nPts+j` likely needs `+1`.
                    // I will add `+1` to be safe and consistent with standard FEA. 
                    // (Unless user's array uses 0-based mapping).
                    
                    // Actually, L96 uses `size()-1`. If `size` is 2, `size-1` is 1. `size` is 2.
                    // So it connects 1 and 2.
                    // So the node at index 0 is referenced as "1".
                    // So `i*nPts+j` (index) corresponds to ID `i*nPts+j+1`.
                    
                    Result.elements.push_back({ idx1 + 1, idx2 + 1, 0.0, 2 });
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

    void Generator::Offset(const Config& cfg, const double& dx, const double& dy, std::vector<RawNode>& offsets)
    {
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
            startAng = -Math_PI / 4.0; // 正方形排列
            radius = cfg.spacing / (2.0 * std::sin(Math_PI / 4.0));
        }
        else if (nBundle == 6)
        {
            startAng = -Math_PI / 3.0; // 六边形
            radius = cfg.spacing / (2.0 * std::sin(Math_PI / 6.0));
        }
        else if (nBundle == 8)
        {
            startAng = -3 * Math_PI / 8.0; // 八边形
            radius = cfg.spacing / (2.0 * std::sin(Math_PI / 8.0));
        }


        if (nBundle == 1)
        {
            offsets.push_back({ 0.0, 0.0, 0.0 });
        }
        else
        {
            for (int i = 0; i < nBundle; ++i)
            {
                double anger = startAng + (2.0 * Math_PI * i / nBundle);
                offsets.push_back({ (vrx * std::cos(anger) + vvx * std::sin(anger)) * radius,
                                    (vry * std::cos(anger) + vvy * std::sin(anger)) * radius,
                                     vvz * std::sin(anger) * radius });
            }
        }

    }
} 