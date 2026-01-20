#pragma once
#include "DataStructure/Structure/StructureData.h"
#include <memory>

/**
 * @brief 求解器类 - 协调整体分析流程
 */
class Solver
{
public:
    /**
     * @brief 设置要分析的模型
     * @param [in] pStructure 结构数据的共享指针
     */
    void SetStructure(std::shared_ptr<StructureData> pStructure);

    /**
     * @brief 运行所有分析步
     */
    void RunAll();
    
    /**
     * @brief 运行指定ID的分析步
     * @param [in] stepId 分析步ID
     * @return 运行成功返回 true，失败返回 false
     */
    bool RunStep(int stepId);

private:
    std::weak_ptr<StructureData> m_pStructure;  ///< 结构数据的弱引用
};
