#pragma once

#include "Base/EmptyOUT.h"

#include <cstddef>
#include <memory>
#include <vector>

namespace SolverNameSpace
{
// 三维共回转梁在 GPU 上计算所需的连续数据。矩阵统一采用行优先布局。
struct GpuBeamElementInput
{
    double initialPosition[6]{};
    double displacement[6]{};
    double velocity[12]{};
    double acceleration[12]{};
    double currentRotation[18]{};
    double previousRotation[18]{};
    double previousGaussRotation[18]{};
    bool hasGaussRotationHistory = false;
    double stepRotation[6]{};
    double initialRotation[9]{};
    double young = 0.0;
    double poisson = 0.0;
    double area = 0.0;
    double inertiaY = 0.0;
    double inertiaZ = 0.0;
    double torsionConstant = 0.0;
    double density = 0.0;
    double rotaryInertia[3]{};
    double initialLength = 0.0;
    double initialStress = 0.0;
};

// 稀疏装配位置只由拓扑决定，评估器会在拓扑不变时复用设备端副本。
struct GpuBeamAssemblyMap
{
    int matrixValueIndices[144]{};
    int freeDofs[12]{};
};

struct GpuBeamElementResult
{
    double effectiveTangent[144]{};
    double inertiaForce[12]{};
    double internalForce[12]{};
    double stress = 0.0;
    double currentLength = 0.0;
};

struct GpuBeamElementState
{
    double internalForce[12]{};
    double stress = 0.0;
    double currentLength = 0.0;
};

struct GpuBeamBatchTiming
{
    double uploadMilliseconds = 0.0;
    double kernelMilliseconds = 0.0;
    double downloadMilliseconds = 0.0;
    double assemblyDownloadMilliseconds = 0.0;
};

// 面向固定拓扑十二自由度共回转梁批次的 GPU 评估器。
class CudaBeamBatchEvaluator final
{
public:
    CudaBeamBatchEvaluator();
    ~CudaBeamBatchEvaluator();
    CudaBeamBatchEvaluator(const CudaBeamBatchEvaluator& other);
    CudaBeamBatchEvaluator& operator=(const CudaBeamBatchEvaluator& other);

    bool Evaluate(const std::vector<GpuBeamElementInput>& input, double accelerationDerivative,
                  double velocityDerivative, _OUT std::vector<GpuBeamElementResult>& result);
    bool EvaluateAndAssemble(const std::vector<GpuBeamElementInput>& input,
                             const std::vector<GpuBeamAssemblyMap>& assemblyMap, double accelerationDerivative,
                             double velocityDerivative, int matrixValueCount, int freeDofCount);
    bool EvaluateResidualAndAssemble(const std::vector<GpuBeamElementInput>& input,
                                     const std::vector<GpuBeamAssemblyMap>& assemblyMap, int matrixValueCount,
                                     int freeDofCount);
    bool CopyAssembledData(_OUT double* matrixValues, int matrixValueCount, _OUT double* inertiaForce,
                           int freeDofCount);
    bool CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount);

    const GpuBeamElementState* StateData() const;
    const double* DeviceMatrixValues() const;
    int DeviceMatrixValueCount() const;
    const GpuBeamBatchTiming& LastTiming() const;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
}
