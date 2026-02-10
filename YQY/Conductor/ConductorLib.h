#pragma once
#include <vector>

#ifdef CONDUCTOR_EXPORTS
#define CONDUCTOR_API __declspec(dllexport) // 生成 DLL 时使用
#else
#define CONDUCTOR_API __declspec(dllimport) // 其他项目使用 DLL 时使用
#endif

namespace ConductorLib 
{
    const double Math_PI = 3.1415926535897932;

    enum class ConnectionMode 
    {
        VerticalTriangle = 0,          // Vertical=左右分组
        Parallel                       // 平行连接
    };

    struct RawNode { double x, y, z; };
    struct RawElement 
    {
        int iNode, jNode;
        double stress0;
        int type = 1; // 1=导线单元, 2=间隔棒单元， 3= 绝缘子单元
    };

    struct BundleResult 
    {
        std::vector<RawNode> nodes;
        std::vector<RawElement> elements;
    };

    struct Config 
    {
        int nBundle = 1;           // 分裂数
        double spacing = 0.4;      // 子导线间距 S
        double insulatorL = 1.0;   // 端部段长度
        int segments = 50;         // 离散段数
        double stress0 = 50e6;     // 初始应力 (Pa)
        double unitWeight = 38612.16;  // 容重 (N/m)

        ConnectionMode mode = ConnectionMode::VerticalTriangle;
        double yokeSpacing = 0.4;  // 联板挂点间距
        int numSpacers = 2;        // 间隔棒数量
        bool ToYUP = false;
    };

    class CONDUCTOR_API Generator 
    {
    public:
        static BundleResult CreateBundle(const double start[3], const double end[3], const Config& cfg);

    private:
        static void Offset(const Config& cfg, const double& dx, const double& dy, std::vector<RawNode>& offsets);
    };
}