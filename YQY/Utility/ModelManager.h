#pragma once
#include "DataStructure/Structure/StructureData.h"
#include <map>
#include <memory>
#include <vector>

/**
 * @brief 模型管理器（单例模式）- 管理程序中的多个模型，支持创建、切换、删除模型
 */
class ModelManager
{
public:
    /**
     * @brief 获取单例实例
     * @return 单例引用
     */
    static ModelManager& Instance()
    {
        static ModelManager instance;
        return instance;
    }

    /**
     * @brief 创建新模型
     * @return 新模型的ID
     */
    int CreateModel();

    /**
     * @brief 获取指定ID的模型
     * @param [in] id 模型ID
     * @return 模型的共享指针，未找到返回 nullptr
     */
    std::shared_ptr<StructureData> GetModel(int id);

    /**
     * @brief 获取当前活动模型
     * @return 活动模型的共享指针
     */
    std::shared_ptr<StructureData> GetActiveModel();

    /**
     * @brief 设置活动模型
     * @param [in] id 模型ID
     * @return 设置成功返回 true
     */
    bool SetActiveModel(int id);

    /**
     * @brief 获取活动模型ID
     * @return 活动模型ID
     */
    int GetActiveModelId() const
    {
        return m_ActiveModelId;
    }

    /**
     * @brief 删除指定模型
     * @param [in] id 模型ID
     * @return 删除成功返回 true
     */
    bool DeleteModel(int id);

    /**
     * @brief 清空所有模型
     */
    void ClearAllModels();

    /**
     * @brief 获取所有模型ID列表
     * @return 模型ID的向量
     */
    std::vector<int> GetAllModelIds() const;

    /**
     * @brief 获取模型数量
     * @return 模型数量
     */
    int GetModelCount() const
    {
        return static_cast<int>(m_Models.size());
    }

private:
    ModelManager() = default;
    ModelManager(const ModelManager&) = delete;
    ModelManager& operator=(const ModelManager&) = delete;

    std::map<int, std::shared_ptr<StructureData>> m_Models;  ///< 模型集合
    int m_ActiveModelId = 0;  ///< 当前活动模型ID
    int m_NextId = 1;         ///< 下一个模型ID
};
