#include "Solver.h"
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include <QElapsedTimer>

void Solver::SetStructure(std::shared_ptr<StructureData> pStructure)
{
    m_pStructure = pStructure;
}

bool Solver::RunAll()
{
    QElapsedTimer timer;
    timer.start();

    auto pStructure = m_pStructure.lock();
    if (!pStructure)
    {
        qDebug().noquote() << QStringLiteral("Error: Solver 未关联结构数据");
        return false;
    }

    if (pStructure->m_AnalysisStep.empty())
    {
        qDebug().noquote() << QStringLiteral("Warning: 没有分析步，无法运行分析");
        return false;
    }

    bool m_IsFirst = true;

    // 循环执行所有分析步
    for (auto& pair : pStructure->m_AnalysisStep)
    {
        int stepId = pair.first;
        auto& step = pair.second;
        step->m_Id = stepId;
        step->SetStructure(pStructure);           // 关联结构数据

        step->Init();
        // 运行分析步（求解内部会处理初始化）
        if (!step->Solve())
        {
            qDebug().noquote() << QStringLiteral("Error: 分析步失败，停止后续分析。Step ID=") << stepId;
            return false;
        }
    }
    qint64 elapsedMs = timer.elapsed();
    return true;
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

    return step->Solve();
}
