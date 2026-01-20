#pragma once
#include "DataStructure/Structure/StructureData.h"
#include <map>
#include <memory>
#include <vector>

// 模型管理器（单例模式）
// 用于管理程序中的多个模型，支持创建、切换、删除模型
class ModelManager
{
public:
    // 获取单例实例
    static ModelManager& Instance()
    {
        static ModelManager instance;
        return instance;
    }

    // 创建新模型，返回模型ID
    int CreateModel()
    {
        int newId = m_NextId++;
        m_Models[newId] = std::make_shared<StructureData>();
        m_Models[newId]->m_Id = newId;
        m_ActiveModelId = newId;
        qDebug().noquote() << QStringLiteral("创建模型 ID=") << newId;
        return newId;
    }

    // 获取指定ID的模型
    std::shared_ptr<StructureData> GetModel(int id)
    {
        auto it = m_Models.find(id);
        return (it != m_Models.end()) ? it->second : nullptr;
    }

    // 获取当前活动模型
    std::shared_ptr<StructureData> GetActiveModel()
    {
        return GetModel(m_ActiveModelId);
    }

    // 设置活动模型
    bool SetActiveModel(int id)
    {
        if (m_Models.find(id) != m_Models.end())
        {
            m_ActiveModelId = id;
            qDebug().noquote() << QStringLiteral("切换到模型 ID=") << id;
            return true;
        }
        qDebug().noquote() << QStringLiteral("模型不存在 ID=") << id;
        return false;
    }

    // 获取活动模型ID
    int GetActiveModelId() const
    {
        return m_ActiveModelId;
    }

    // 删除指定模型
    bool DeleteModel(int id)
    {
        auto it = m_Models.find(id);
        if (it == m_Models.end())
        {
            qDebug().noquote() << QStringLiteral("删除失败，模型不存在 ID=") << id;
            return false;
        }
        
        m_Models.erase(it);
        qDebug().noquote() << QStringLiteral("删除模型 ID=") << id;
        
        // 如果删除的是活动模型，切换到第一个可用模型
        if (m_ActiveModelId == id)
        {
            m_ActiveModelId = m_Models.empty() ? 0 : m_Models.begin()->first;
        }
        return true;
    }

    // 清空所有模型
    void ClearAllModels()
    {
        m_Models.clear();
        m_ActiveModelId = 0;
        m_NextId = 1;
        qDebug().noquote() << QStringLiteral("已清空所有模型");
    }

    // 获取所有模型ID列表
    std::vector<int> GetAllModelIds() const
    {
        std::vector<int> ids;
        ids.reserve(m_Models.size());
        for (const auto& pair : m_Models)
        {
            ids.push_back(pair.first);
        }
        return ids;
    }

    // 获取模型数量
    int GetModelCount() const
    {
        return static_cast<int>(m_Models.size());
    }

private:
    ModelManager() = default;
    ModelManager(const ModelManager&) = delete;
    ModelManager& operator=(const ModelManager&) = delete;

    std::map<int, std::shared_ptr<StructureData>> m_Models;  // 模型集合
    int m_ActiveModelId = 0;  // 当前活动模型ID
    int m_NextId = 1;         // 下一个模型ID
};
