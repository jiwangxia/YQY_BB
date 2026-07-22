#include "AnalysisSolve.h"
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include <QDebug>
#include <QElapsedTimer>
#include <algorithm>

void AnalysisRunner::SetStructure(std::shared_ptr<StructureData> pStructure)
{
    m_pStructure = pStructure;
}

void AnalysisRunner::SetRuntimeCallbacks(ProgressCallback progressCallback, CancelCallback cancelCallback)
{
    m_progressCallback = std::move(progressCallback);
    m_cancelCallback = std::move(cancelCallback);
}

bool AnalysisRunner::RunAll()
{
    m_wasCancelled = false;
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

    double totalWeight = 0.0;
    for (const auto& pair : pStructure->m_AnalysisStep)
        totalWeight += std::max(1.0, pair.second ? pair.second->m_Time : 1.0);

    double completedWeight = 0.0;
    for (auto& pair : pStructure->m_AnalysisStep)
    {
        int stepId = pair.first;
        auto& step = pair.second;
        if (!step)
            return false;
        if (m_cancelCallback && m_cancelCallback())
        {
            m_wasCancelled = true;
            return false;
        }
        step->m_Id = stepId;
        step->SetStructure(pStructure);
        const double stepWeight = std::max(1.0, step->m_Time);
        step->SetRuntimeCallbacks(
            [this, completedWeight, stepWeight, totalWeight, stepId](double progress, const QString& message) {
                if (m_progressCallback)
                {
                    const double overall = (completedWeight + stepWeight * progress) / totalWeight;
                    m_progressCallback(overall, message.isEmpty()
                        ? QStringLiteral("分析步 %1").arg(stepId) : message);
                }
            },
            m_cancelCallback);

        if (!step->Solve())
        {
            step->ClearRuntimeCallbacks();
            m_wasCancelled = m_cancelCallback && m_cancelCallback();
            qDebug().noquote() << QStringLiteral("Error: 分析步失败，停止后续分析。Step ID=") << stepId;
            return false;
        }
        step->ClearRuntimeCallbacks();
        completedWeight += stepWeight;
        if (m_progressCallback)
            m_progressCallback(completedWeight / totalWeight,
                QStringLiteral("分析步 %1 完成").arg(stepId));
    }

    qint64 elapsedMs = timer.elapsed();
    Q_UNUSED(elapsedMs);
    return true;
}

bool AnalysisRunner::RunStep(int stepId)
{
    m_wasCancelled = false;
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
    step->m_Id = stepId;
    step->SetStructure(pStructure);
    step->SetRuntimeCallbacks(m_progressCallback, m_cancelCallback);
    qDebug().noquote() << QStringLiteral("\n----- 运行分析步 ") << stepId
        << " [" << step->GetTypeName() << "] -----";

    const bool ok = step->Solve();
    step->ClearRuntimeCallbacks();
    m_wasCancelled = !ok && m_cancelCallback && m_cancelCallback();
    return ok;
}
