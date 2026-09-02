#pragma once

#include <QString>

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

private:
    // 按指定时间积分策略运行例 1。
    static int RunExample1(const QString& outputDirectory, bool useAdaptiveTssbn);
    // 按指定时间积分策略运行全部算例。
    static int Run(const QString& outputDirectory, bool useAdaptiveTssbn);
};
