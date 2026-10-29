#include "Support/SolveModel.h"
#include "Support/TestPaths.h"

#include "DataStructure/Structure/StructureData.h"
#include "Import/Input_Model.h"
#include "Solver/Workflow/AnalysisSolve.h"
#include <QDir>
#include <QFileInfo>
#include <QTemporaryDir>
#include <algorithm>
#include <cmath>
#include <exception>

std::shared_ptr<StructureData> SolveModel::Read(const QString& relative, _OUT QString& error)
{
    error.clear();
    const QString path = QDir(TestPaths::TestRoot()).filePath(QStringLiteral("TestModels/") + relative);
    if (!QFileInfo::exists(path))
    {
        error = QStringLiteral("缺少求解输入：%1").arg(path);
        return nullptr;
    }
    auto model = std::make_shared<StructureData>();
    Input_Model reader;
    if (!reader.InputData(path, model))
    {
        error = reader.LastError();
        return nullptr;
    }
    return model;
}

bool SolveModel::Run(const std::shared_ptr<StructureData>& model, _OUT QString& error,
                     _OUT std::map<int, std::shared_ptr<StructureData>>* solvedRegions)
{
    error.clear();
    if (solvedRegions)
        solvedRegions->clear();
    if (!model || model->m_AnalysisStep.empty())
    {
        error = QStringLiteral("求解模型或分析步为空。");
        return false;
    }
    QTemporaryDir outputDirectory;
    if (!outputDirectory.isValid())
    {
        error = QStringLiteral("无法建立独立求解输出目录。");
        return false;
    }
    model->m_OutputControl.m_StreamResult = false;
    model->m_OutputControl.m_Hdf5FileName = outputDirectory.filePath(QStringLiteral("result.h5"));
    AnalysisRunner runner;
    runner.SetStructure(model);
    runner.SetExecutionThreadBudget(1);
    runner.SetMaximumRegionThreads(1);
    try
    {
        if (!runner.RunAll() || runner.WasCancelled())
        {
            error = QStringLiteral("完整求解失败或被取消。");
            for (const auto& [id, step] : model->m_AnalysisStep)
                if (step && !step->LastFailureReason().isEmpty())
                    error += QStringLiteral(" 步%1：%2").arg(id).arg(step->LastFailureReason());
            return false;
        }
    }
    catch (const std::exception& exception)
    {
        error = QString::fromUtf8(exception.what());
        return false;
    }
    const auto& frames = model->GetOutputter().GetDataSet();
    if (frames.empty())
    {
        error = QStringLiteral("完整求解未产生结果帧。");
        return false;
    }
    for (const auto& frame : frames)
    {
        if (!std::isfinite(frame.GetTime()) || frame.GetNodeDatas().empty() || frame.GetElementDatas().empty())
        {
            error = QStringLiteral("结果帧时间非有限或实体结果为空。");
            return false;
        }
        for (const auto& [id, node] : frame.GetNodeDatas())
            for (int type = static_cast<int>(EnumKeyword::NodeResultType::U1);
                 type <= static_cast<int>(EnumKeyword::NodeResultType::R3); ++type)
                if (!std::isfinite(node.GetValue(static_cast<EnumKeyword::NodeResultType>(type))))
                {
                    error = QStringLiteral("节点%1结果非有限。").arg(id);
                    return false;
                }
        for (const auto& [id, element] : frame.GetElementDatas())
            for (int type = static_cast<int>(EnumKeyword::ElementResultType::AxialForce);
                 type <= static_cast<int>(EnumKeyword::ElementResultType::RelativeDisplacement); ++type)
                if (!std::isfinite(element.GetValue(static_cast<EnumKeyword::ElementResultType>(type))))
                {
                    error = QStringLiteral("单元%1结果非有限。").arg(id);
                    return false;
                }
    }
    for (const auto& [id, step] : model->m_AnalysisStep)
    {
        const double tolerance = 1.0e-11 * std::max(1.0, std::abs(step->m_Time));
        const bool reachedEnd =
            std::any_of(frames.cbegin(), frames.cend(),
                        [id, step, tolerance](const DataFrame& frame)
                        {
                            return frame.GetStepId() == id && std::abs(frame.GetTime() - step->m_Time) <= tolerance;
                        });
        if (!reachedEnd)
        {
            error = QStringLiteral("分析步%1未达到终止时刻%2。").arg(id).arg(step->m_Time, 0, 'g', 17);
            return false;
        }
    }
    if (solvedRegions)
        *solvedRegions = runner.GetSolvedRegionStates();
    return true;
}
