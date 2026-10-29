#include "Utility/AnalysisDiagnostics.h"

#include "DataStructure/Structure/StructureData.h"

#include <QStringList>
#include <Windows.h>
#include <Psapi.h>

double AnalysisDiagnostics::ProcessPeakMemoryMegabytes()
{
    PROCESS_MEMORY_COUNTERS counters{};
    counters.cb = sizeof(counters);
    if (!GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)))
        return 0.0;
    return static_cast<double>(counters.PeakWorkingSetSize) / (1024.0 * 1024.0);
}

QString AnalysisDiagnostics::PerformanceReport(const StructureData& structure, int selectedStepId)
{
    QStringList reports;
    for (const auto& [stepId, step] : structure.m_AnalysisStep)
    {
        if (!step || (selectedStepId > 0 && stepId != selectedStepId))
            continue;
        const AnalysisPerformanceStatistics& value = step->GetPerformanceStatistics();
        if (value.totalMilliseconds <= 0.0 && value.systemEvaluationCount <= 0)
            continue;
        const QString stepName =
            step->m_Name.trimmed().isEmpty() ? QStringLiteral("Step-%1").arg(stepId) : step->m_Name.trimmed();
        reports.push_back(
            QStringLiteral("%1内部耗时(ms)：求解步 %2；系统评估 %3（自身 %14；包含：有效切线 %4、动力系统 %5、质量 %6、"
                           "惯性力 %7、残差状态 %8、外载切线 %9、残差汇总 %10）；结果输出 %11。"
                           "包含项不可与系统评估或求解步直接相加；GPU批次：有效切线 %12、残差 %13")
                .arg(stepName)
                .arg(value.totalMilliseconds, 0, 'f', 1)
                .arg(value.systemEvaluationMilliseconds, 0, 'f', 1)
                .arg(value.effectiveTangentMilliseconds, 0, 'f', 1)
                .arg(value.dynamicSystemMilliseconds, 0, 'f', 1)
                .arg(value.massMatrixMilliseconds, 0, 'f', 1)
                .arg(value.inertiaForceMilliseconds, 0, 'f', 1)
                .arg(value.residualStateMilliseconds, 0, 'f', 1)
                .arg(value.externalTangentMilliseconds, 0, 'f', 1)
                .arg(value.residualMilliseconds, 0, 'f', 1)
                .arg(value.outputMilliseconds, 0, 'f', 1)
                .arg(value.gpuEffectiveTangentCount)
                .arg(value.gpuResidualStateCount)
                .arg(value.systemEvaluationExclusiveMilliseconds, 0, 'f', 1));
    }
    return reports.join(QStringLiteral("；"));
}
