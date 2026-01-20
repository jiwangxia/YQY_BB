#include "ModelManager.h"

int ModelManager::CreateModel()
{
    int newId = m_NextId++;
    m_Models[newId] = std::make_shared<StructureData>();
    m_Models[newId]->m_Id = newId;
    m_ActiveModelId = newId;
    qDebug().noquote() << QStringLiteral("创建模型 ID=") << newId;
    return newId;
}

std::shared_ptr<StructureData> ModelManager::GetModel(int id)
{
    auto it = m_Models.find(id);
    return (it != m_Models.end()) ? it->second : nullptr;
}

std::shared_ptr<StructureData> ModelManager::GetActiveModel()
{
    return GetModel(m_ActiveModelId);
}

bool ModelManager::SetActiveModel(int id)
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

bool ModelManager::DeleteModel(int id)
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

void ModelManager::ClearAllModels()
{
    m_Models.clear();
    m_ActiveModelId = 0;
    m_NextId = 1;
    qDebug().noquote() << QStringLiteral("已清空所有模型");
}

std::vector<int> ModelManager::GetAllModelIds() const
{
    std::vector<int> ids;
    ids.reserve(m_Models.size());
    for (const auto& pair : m_Models)
    {
        ids.push_back(pair.first);
    }
    return ids;
}
