#pragma once

#include "Solver/Interface/ISolver.h"

#include <array>
#include <memory>

class AnalysisStep;

namespace SolverNameSpace
{
enum class InitialStaticMode
{
    Configured,
    ReferenceWindless
};

struct DynamicExecutionPolicy
{
    bool implemented = false;
    InitialStaticMode initialStaticMode = InitialStaticMode::Configured;
    bool requiresStaticState = false;
    bool requiresGalloping = false;
    bool publishInitialStaticResults = true;
    bool supportsNonlinearMpc = true;
    bool supportsPrescribedBoundaryMotion = false;
};

struct DynamicAlgorithm
{
    SolverType type;
    const char* displayName;
    DynamicExecutionPolicy policy;
};

// 将分析步配置转换为求解器实例，调用方只依赖统一接口。
class SolverFactory
{
public:
    // 界面、参数验证和任务编排共用的动力算法登记及执行策略。
    static const std::array<DynamicAlgorithm, 6>& DynamicAlgorithms();
    static DynamicExecutionPolicy GetDynamicExecutionPolicy(SolverType type);

    // 返回由调用方独占的求解器，分析步配置保持只读；构造参数异常交由调用方处理。
    // 未实现或未知的分析步、动力算法返回 nullptr。
    static std::unique_ptr<ISolver> Create(const AnalysisStep& step);

    // 保留旧接口，兼容现有调用方。
    static std::unique_ptr<ISolver> Create_StepForSlover(const AnalysisStep& step);

private:
    static std::unique_ptr<ISolver> CreateStaticSolver(const AnalysisStep& step);
    static std::unique_ptr<ISolver> CreateDynamicSolver(const AnalysisStep& step);
    static AerodynamicTangentMode ResolveAerodynamicTangentMode(const AnalysisStep& step);
    static std::unique_ptr<ISolver> CreateNewmarkFromStep(const AnalysisStep& step, bool adaptive = false);
    static std::unique_ptr<ISolver> CreateRungeKutta4FromStep(const AnalysisStep& step, bool adaptive = false);
    static std::unique_ptr<ISolver> CreateAdaptiveTssbnFromStep(const AnalysisStep& step,
                                                                bool referenceGalloping = false);
};
}
