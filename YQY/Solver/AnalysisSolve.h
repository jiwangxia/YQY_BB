#pragma once
#include "DataStructure/Structure/StructureData.h"
#include <memory>
#include <functional>
#include <vector>

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

    using ProgressCallback = std::function<void(double, const QString&)>;
    using CancelCallback = std::function<bool()>;
    void SetRuntimeCallbacks(ProgressCallback progressCallback, CancelCallback cancelCallback);
    void SetMaximumRegionThreads(int count);
	bool WasCancelled() const { return m_wasCancelled; }

private:
    bool RunSelectedByRegions(const std::vector<int>& stepIds);
    bool RunStepDirect(int stepId, bool persistHdf5 = true);

    std::weak_ptr<StructureData> m_pStructure;
    ProgressCallback m_progressCallback;
    CancelCallback m_cancelCallback;
    bool m_wasCancelled = false;
    int m_maximumRegionThreads = 0;
};
