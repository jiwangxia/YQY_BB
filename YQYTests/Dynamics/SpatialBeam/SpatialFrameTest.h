#pragma once

#include "Solver/Dynamic/TimeStepping/AdaptiveTssbnSettings.h"
#include <QString>

// 论文动力学入口的附加诊断与步长参数；不改变生产求解策略。
struct PaperTssbnRunOptions
{
    double initialTimeStep = 0.01;
    double minimumTimeStep = 1.0e-6;
    double nonlinearTolerance = 1.0e-6;
    double spectralRadiusInfinity = 1.0;
    SolverNameSpace::TssbnErrorNormMode errorNormMode = SolverNameSpace::TssbnErrorNormMode::GlobalMax;
    bool overrideAlgorithmControls = false;
    bool diagnostics = false;
    bool profile = false;
    bool finalStateOnly = false;
    bool fusedAssembly = false;
    bool energyEveryStep = false;
    bool useRungeKutta = false;
};

// 运行论文梁动力学算例的验证入口。
class PaperBeamDynamicsVerification final
{
public:
    // 运行固定步长的例 1。
    static int RunExample1(const QString& outputDirectory);
    // 运行自适应步长的例 1。
    static int RunExample1Adaptive(const QString& outputDirectory);
    // 运行固定步长的全部论文算例。
    static int Run(const QString& outputDirectory);
    // 运行自适应步长的全部论文算例。
    static int RunAdaptive(const QString& outputDirectory);
    static int RunDuan2026(bool ring, bool useAdaptiveTssbn, const QString& outputDirectory, double duration,
                           double maximumTimeStep, double relativeTolerance, int beamElementsPerLeg,
                           bool useFixedNewmark = false, double newmarkAbsoluteTolerance = 1.0e-6,
                           double newmarkVelocityAbsoluteTolerance = 1.0e-6, bool useReferenceTssbn = false,
                           int fullStateStride = 1, const PaperTssbnRunOptions* runOptions = nullptr);

private:
    // 按指定时间积分策略运行例 1。
    static int RunExample1(const QString& outputDirectory, bool useAdaptiveTssbn);
    // 按指定时间积分策略运行全部算例。
    static int Run(const QString& outputDirectory, bool useAdaptiveTssbn);
};
