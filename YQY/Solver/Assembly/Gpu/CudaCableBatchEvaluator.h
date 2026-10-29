#pragma once

#include "Base/EmptyOUT.h"

#include <cstddef>
#include <memory>
#include <vector>

namespace SolverNameSpace
{
// 常规索单元在 GPU 上计算所需的连续数据。局部自由度固定为两节点各四个。
struct GpuCableElementInput
{
    double initialPosition[6]{};
    double displacement[8]{};
    double acceleration[8]{};
    double young = 0.0;
    double poisson = 0.0;
    double area = 0.0;
    double polarMoment = 0.0;
    double density = 0.0;
    double initialLength = 0.0;
    double initialStress = 0.0;
};

// 稀疏装配位置只取决于拓扑，不随时间步变化；该数据会在设备端缓存复用。
struct GpuCableAssemblyMap
{
    int matrixValueIndices[64]{};
    int freeDofs[8]{};
};

// 行优先局部矩阵与 CPU 中普通索单元的八自由度布局一致。
struct GpuCableElementResult
{
    double effectiveTangent[64]{};
    double inertiaForce[8]{};
    double internalForce[8]{};
    double stress = 0.0;
    double currentLength = 0.0;
};

// 设备端装配时回传的最小状态，避免传输每根索的完整 8×8 局部矩阵。
struct GpuCableElementState
{
    double internalForce[8]{};
    double stress = 0.0;
    double currentLength = 0.0;
};

struct GpuCableBatchTiming
{
    double uploadMilliseconds = 0.0;
    double kernelMilliseconds = 0.0;
    double downloadMilliseconds = 0.0;
    double assemblyDownloadMilliseconds = 0.0;
};

// 面向固定八自由度常规索批次的 GPU 单元计算器。设备缓冲区会按容量复用。
class CudaCableBatchEvaluator final
{
public:
    CudaCableBatchEvaluator();
    ~CudaCableBatchEvaluator();
    CudaCableBatchEvaluator(const CudaCableBatchEvaluator& other);
    CudaCableBatchEvaluator& operator=(const CudaCableBatchEvaluator& other);

    bool Evaluate(const std::vector<GpuCableElementInput>& input, double accelerationDerivative);
    bool Evaluate(const std::vector<GpuCableElementInput>& input, double accelerationDerivative,
                  _OUT std::vector<GpuCableElementResult>& result);

    // 在 GPU 上直接累加到整体稀疏矩阵值数组和惯性向量。输入中的索引由调用方预先填充。
    bool EvaluateAndAssemble(const std::vector<GpuCableElementInput>& input,
                             const std::vector<GpuCableAssemblyMap>& assemblyMap, double accelerationDerivative,
                             int matrixValueCount, int freeDofCount);
    bool EvaluateResidualAndAssemble(const std::vector<GpuCableElementInput>& input,
                                     const std::vector<GpuCableAssemblyMap>& assemblyMap, int matrixValueCount,
                                     int freeDofCount);
    bool CopyAssembledData(_OUT double* matrixValues, int matrixValueCount, _OUT double* inertiaForce,
                           int freeDofCount);
    bool CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount);

    // 返回最近一次计算结果的页锁定主机缓冲区，直到下一次 Evaluate 调用前保持有效。
    const GpuCableElementResult* ResultData() const;
    std::size_t ResultCount() const;
    const GpuCableElementState* StateData() const;
    const double* DeviceMatrixValues() const;
    int DeviceMatrixValueCount() const;
    const GpuCableBatchTiming& LastTiming() const;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

// 与梁共节点的索使用四/六自由度混合布局，轴向扭转由当前索轴投影到空间转动自由度。
struct GpuSharedCableElementInput
{
    double initialPosition[6]{};
    double displacement[6]{};
    double acceleration[12]{};
    double twist[2]{};
    double torsionAxis[3]{};
    double young = 0.0;
    double poisson = 0.0;
    double area = 0.0;
    double polarMoment = 0.0;
    double density = 0.0;
    double initialLength = 0.0;
    double initialStress = 0.0;
    int firstNodeDofs = 4;
    int secondNodeDofs = 4;
};

struct GpuSharedCableAssemblyMap
{
    int matrixValueIndices[144]{};
    int freeDofs[12]{};
};

struct GpuSharedCableElementState
{
    double internalForce[12]{};
    double stress = 0.0;
    double currentLength = 0.0;
};

class CudaSharedCableBatchEvaluator final
{
public:
    CudaSharedCableBatchEvaluator() = default;
    ~CudaSharedCableBatchEvaluator();
    CudaSharedCableBatchEvaluator(const CudaSharedCableBatchEvaluator&);
    CudaSharedCableBatchEvaluator& operator=(const CudaSharedCableBatchEvaluator&);

    bool EvaluateAndAssemble(const std::vector<GpuSharedCableElementInput>& input,
                             const std::vector<GpuSharedCableAssemblyMap>& assemblyMap, double accelerationDerivative,
                             int matrixValueCount, int freeDofCount, bool assembleMatrix);
    bool CopyAssembledData(_OUT double* matrixValues, int matrixValueCount, _OUT double* inertiaForce,
                           int freeDofCount);
    bool CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount);
    const GpuSharedCableElementState* StateData() const;
    const double* DeviceMatrixValues() const;
    int DeviceMatrixValueCount() const;

private:
    void Reset();
    GpuSharedCableElementInput* m_deviceInput = nullptr;
    GpuSharedCableAssemblyMap* m_deviceMap = nullptr;
    GpuSharedCableElementState* m_deviceState = nullptr;
    double* m_deviceValues = nullptr;
    double* m_deviceInertia = nullptr;
    std::vector<GpuSharedCableAssemblyMap> m_cachedMap;
    std::vector<GpuSharedCableElementState> m_hostState;
    std::size_t m_capacity = 0;
    int m_matrixValueCount = 0;
    int m_freeDofCount = 0;
    bool m_hasMatrix = false;
};
}
