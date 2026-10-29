#pragma once
#include <QString>
#include <memory>
#include <functional>
#include <map>
#include <vector>

class AnalysisStep;
class StructureData;

/// 将继承的静力状态按分析步配置施加舞动竖向初扰；供运行器和几何回归验证复用。
bool ApplyGallopingInitialVerticalPerturbation(StructureData& structure, AnalysisStep& step);

/**
 * @brief 分析运行器 - 编排模型中的一个或多个分析步
 */
class AnalysisRunner
{
public:
    /**
     * @brief 设置要分析的模型
     */
    void SetStructure(std::shared_ptr<StructureData> pStructure);

    /**
     * @brief 运行所有分析步
     */
    bool RunAll();

    /**
     * @brief 运行指定ID的分析步
     */
    bool RunStep(int stepId);

    /**
     * @brief 从模型当前已收敛状态直接运行指定分析步
     *
     * 用于任务调度器已经单独完成前置静力步的情形。该接口不会再次执行
     * m_InitialStaticStepId，只把当前节点/单元状态作为动力步初态。
     */
    bool RunStepFromCurrentState(int stepId);

    using ProgressCallback = std::function<void(double, const QString&)>;
    using CancelCallback = std::function<bool()>;
    void SetRuntimeCallbacks(ProgressCallback progressCallback, CancelCallback cancelCallback);
    void SetMaximumRegionThreads(int count);
    /// 为本运行器限定 CPU 预算。零表示保持单独运行时的默认策略。
    void SetExecutionThreadBudget(int count);
    /// 独立动力任务按区域继承对应的已收敛静力状态。
    void SetInitialRegionStates(const std::map<int, std::shared_ptr<StructureData>>& states);
    const std::map<int, std::shared_ptr<StructureData>>& GetSolvedRegionStates() const
    {
        return m_solvedRegionStates;
    }
    bool WasCancelled() const
    {
        return m_wasCancelled;
    }

private:
    bool RunSelectedByRegions(const std::vector<int>& stepIds);
    bool RunWithStaticDependencies(const std::vector<int>& stepIds);
    bool RunStepDirect(int stepId, bool persistHdf5 = true, bool inheritedStaticState = false);

    std::weak_ptr<StructureData> m_pStructure;
    std::map<int, std::shared_ptr<StructureData>> m_initialRegionStates;
    std::map<int, std::shared_ptr<StructureData>> m_solvedRegionStates;
    ProgressCallback m_progressCallback;
    CancelCallback m_cancelCallback;
    bool m_wasCancelled = false;
    int m_maximumRegionThreads = 0;
    int m_executionThreadBudget = 0;
};
