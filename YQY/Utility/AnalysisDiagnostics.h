#pragma once

#include <QString>

class StructureData;

namespace AnalysisDiagnostics
{
// 线程模式及独立工作进程共用的诊断格式。
double ProcessPeakMemoryMegabytes();
QString PerformanceReport(const StructureData& structure, int selectedStepId);
}
