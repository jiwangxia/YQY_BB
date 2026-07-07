#pragma once
#include "DataStructure/Structure/StructureData.h"
#include <memory>

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

private:
    std::weak_ptr<StructureData> m_pStructure;
};
