#pragma once

#include "Base/EmptyOUT.h"

#include <memory>
#include <vector>

namespace SolverNameSpace
{
// 三维桁架单元在 GPU 上计算所需的连续数据。该结构不依赖 Eigen，便于直接复制到显存。
struct GpuTrussElementInput
{
    double initialPosition[6]{};
    double displacement[6]{};
    double acceleration[6]{};
    double young = 0.0;
    double area = 0.0;
    double density = 0.0;
    double initialLength = 0.0;
    double initialStress = 0.0;
    int useEngineeringStrain = 0;
};

// 单元输出采用行优先局部矩阵布局，避免在 GPU 与 CPU 间进行额外转置。
struct GpuTrussElementResult
{
    double effectiveTangent[36]{};
    double inertiaForce[6]{};
    double internalForce[6]{};
    double stress = 0.0;
    double currentLength = 0.0;
};

// 稀疏矩阵位置由拓扑唯一决定，可在时间积分过程中重复使用。
struct GpuTrussAssemblyMap
{
    int matrixValueIndices[36]{};
    int freeDofs[6]{};
};

// 设备端完成整体装配后，仅回传用于更新单元状态的最小数据。
struct GpuTrussElementState
{
    double internalForce[6]{};
    double stress = 0.0;
    double currentLength = 0.0;
};

struct GpuTrussBatchTiming
{
    double uploadMilliseconds = 0.0;
    double kernelMilliseconds = 0.0;
    double downloadMilliseconds = 0.0;
    double assemblyDownloadMilliseconds = 0.0;
};

// 面向固定拓扑桁架批次的 GPU 单元计算器。设备缓冲区会按容量复用。
class CudaTrussBatchEvaluator final
{
public:
    CudaTrussBatchEvaluator();
    ~CudaTrussBatchEvaluator();
    CudaTrussBatchEvaluator(const CudaTrussBatchEvaluator& other);
    CudaTrussBatchEvaluator& operator=(const CudaTrussBatchEvaluator& other);

    bool Evaluate(const std::vector<GpuTrussElementInput>& input, double accelerationDerivative);
    bool Evaluate(const std::vector<GpuTrussElementInput>& input, double accelerationDerivative,
                  _OUT std::vector<GpuTrussElementResult>& result);
    bool EvaluateAndAssemble(const std::vector<GpuTrussElementInput>& input,
                             const std::vector<GpuTrussAssemblyMap>& assemblyMap, double accelerationDerivative,
                             int matrixValueCount, int freeDofCount);
    bool EvaluateResidualAndAssemble(const std::vector<GpuTrussElementInput>& input,
                                     const std::vector<GpuTrussAssemblyMap>& assemblyMap, int matrixValueCount,
                                     int freeDofCount);
    bool CopyAssembledData(_OUT double* matrixValues, int matrixValueCount, _OUT double* inertiaForce,
                           int freeDofCount);
    bool CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount);

    // 返回最近一次计算结果的页锁定主机缓冲区，直到下一次 Evaluate 调用前保持有效。
    const GpuTrussElementResult* ResultData() const;
    std::size_t ResultCount() const;
    const GpuTrussElementState* StateData() const;
    const double* DeviceMatrixValues() const;
    int DeviceMatrixValueCount() const;

    const GpuTrussBatchTiming& LastTiming() const;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
}
