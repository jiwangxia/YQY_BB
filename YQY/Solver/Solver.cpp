#include "Solver.h"
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include <QElapsedTimer>

void Solver::SetStructure(std::shared_ptr<StructureData> pStructure)
{
    m_pStructure = pStructure;
}

void Solver::RunAll()
{
    auto pStructure = m_pStructure.lock();
    if (!pStructure)
    {
        qDebug().noquote() << QStringLiteral("Error: Solver 未关联结构数据");
        return;
    }

    if (pStructure->m_AnalysisStep.empty())
    {
        qDebug().noquote() << QStringLiteral("Warning: 没有分析步，无法运行分析");
        return;
    }

    qDebug().noquote() << QStringLiteral("\n========== 开始分析 ==========");

    // 循环执行所有分析步
    for (auto& pair : pStructure->m_AnalysisStep)
    {
        int stepId = pair.first;
        auto& step = pair.second;

        // 运行分析步（求解内部会处理初始化）
        step->Solve();
        std::vector<int> nodeIds = {  2};
        std::vector<DataType> types = { DataType::U1, DataType::U2, DataType::U3 };
        step->GetOutputter().ExportNodes("Export/ExportFile/1.txt", nodeIds, types);
    }

    qDebug().noquote() << QStringLiteral("\n========== 分析完成 ==========\n");
}

bool Solver::RunStep(int stepId)
{
    auto pStructure = m_pStructure.lock();
    if (!pStructure)
    {
        qDebug().noquote() << QStringLiteral("Error: Solver 未关联结构数据");
        return false;
    }

    auto it = pStructure->m_AnalysisStep.find(stepId);
    if (it == pStructure->m_AnalysisStep.end())
    {
        qDebug().noquote() << QStringLiteral("Error: 未找到分析步 ID=") << stepId;
        return false;
    }

    auto& step = it->second;
    qDebug().noquote() << QStringLiteral("\n----- 运行分析步 ") << stepId
        << " [" << step->GetTypeName() << "] -----";

    step->Solve();

    return true;
}
