#include "StructureData.h"
#include <set>
#include <cmath>
#include <vector>
#include <tuple>
#include <map>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

StructureData::~StructureData()
{
}

std::shared_ptr<Node> StructureData::FindNode(int id)
{
    auto result = m_Nodes.find(id);
    if (result != m_Nodes.end())
    {
        return result->second;
    }
    return nullptr;
}

std::shared_ptr<Material> StructureData::FindMaterial(int id)
{
    auto result = m_Material.find(id);
    if (result != m_Material.end())
    {
        return result->second;
    }
    qDebug().noquote() << "未找到材料id =" << id;
    return nullptr;
}

std::shared_ptr<SectionBase> StructureData::FindSection(int id)
{
    auto result = m_Section.find(id);
    if (result != m_Section.end())
    {
        return result->second;
    }
    qDebug().noquote() << "未找到截面id =" << id;
    return nullptr;
}

std::shared_ptr<Property> StructureData::FindProperty(int id)
{
    auto result = m_Property.find(id);
    if (result != m_Property.end())
    {
        return result->second;
    }
    qDebug().noquote() << "未找到属性id =" << id;
    return nullptr;
}

std::shared_ptr<Property> StructureData::Create_Property(int id_material, int id_section)
{
    // 检查是否已存在相同材料+截面组合的 Property
    for (const auto& pair : m_Property)
    {
        auto existingProp = pair.second;
        auto existingMat = existingProp->m_pMaterial.lock();
        auto existingSec = existingProp->m_pSection.lock();
        
        if (existingMat && existingSec &&
            existingMat->m_Id == id_material && existingSec->m_Id == id_section)
        {
            // 已存在相同组合，直接返回
            return existingProp;
        }
    }
    
    // 不存在，创建新的 Property
    int property_id = static_cast<int>(m_Property.size()) + 1;
    auto property = std::make_shared<Property>();

    auto iterMat = m_Material.find(id_material);
    if (iterMat != m_Material.end())
    {
        property->m_pMaterial = iterMat->second;
    }
    else
    {
        qDebug().noquote() << QStringLiteral("Error: Create_Property 未找到材料 id=") << id_material;
        exit(1);
    }

    auto iterSec = m_Section.find(id_section);
    if (iterSec != m_Section.end())
    {
        property->m_pSection = iterSec->second;
    }
    else
    {
        qDebug().noquote() << QStringLiteral("Error: Create_Property 未找到截面 id=") << id_section;
        exit(1);
    }

    property->m_Id = property_id;
    m_Property.insert(std::make_pair(property_id, property));
    return property;
}

// ===== 模型检查函数 =====
void StructureData::CleanupModel(double tolerance)
{
    int nodesBefore = static_cast<int>(m_Nodes.size());
    int elementsBefore = static_cast<int>(m_Elements.size());
    
    MergeDuplicateNodes(tolerance);
    RemoveDuplicateElements();
    RemoveOrphanNodes();
    RenumberAll();
    
    int nodesAfter = static_cast<int>(m_Nodes.size());
    int elementsAfter = static_cast<int>(m_Elements.size());
    
    //qDebug().noquote() << QStringLiteral("节点: ") << nodesBefore << QStringLiteral(" -> ") << nodesAfter;
    //qDebug().noquote() << QStringLiteral("单元: ") << elementsBefore << QStringLiteral(" -> ") << elementsAfter;
    qDebug().noquote() << QStringLiteral("===== 模型检查完成 =====\n");
}

//数据优化-----不用看这些代码-------------------
// 临时节点结构（用于扁平化数据，提升缓存效率）
struct TempNode 
{
    int id;
    double x, y, z;
};

void StructureData::MergeDuplicateNodes(double tolerance)
{
    if (m_Nodes.empty()) return;

    // 1. 预处理：计算边界并扁平化数据
    std::vector<TempNode> linearNodes;
    linearNodes.reserve(m_Nodes.size());
    
    double minX = std::numeric_limits<double>::max();
    double minY = minX, minZ = minX;

    for (const auto& pair : m_Nodes) 
    {
        auto node = pair.second;
        // 计算最小坐标，用于防止哈希溢出
        if (node->m_X < minX) minX = node->m_X;
        if (node->m_Y < minY) minY = node->m_Y;
        if (node->m_Z < minZ) minZ = node->m_Z;
        
        linearNodes.push_back({pair.first, node->m_X, node->m_Y, node->m_Z});
    }

    // 2. 计算 CellSize
    double cellSize = tolerance * 1.01;
    
    // 3. 空间哈希定义
    using HashKey = std::tuple<long long, long long, long long>;
    
    struct SpaceHash 
    {
        std::size_t operator()(const HashKey& k) const
        {
            auto [x, y, z] = k;
            // 更加离散的哈希组合
            size_t h1 = std::hash<long long>{}(x);
            size_t h2 = std::hash<long long>{}(y);
            size_t h3 = std::hash<long long>{}(z);
            return h1 ^ (h2 << 1) ^ (h3 << 2); 
        }
    };

    // 4. 分桶
    std::unordered_map<HashKey, std::vector<int>, SpaceHash> buckets;
    buckets.reserve(linearNodes.size()); 

    for (int i = 0; i < static_cast<int>(linearNodes.size()); ++i) 
    {
        const auto& node = linearNodes[i];
        // 关键优化：减去 minX，确保从 0 开始计数，大大降低溢出风险
        long long kx = static_cast<long long>(std::floor((node.x - minX) / cellSize));
        long long ky = static_cast<long long>(std::floor((node.y - minY) / cellSize));
        long long kz = static_cast<long long>(std::floor((node.z - minZ) / cellSize));
        
        buckets[std::make_tuple(kx, ky, kz)].push_back(i);
    }

    // 5. 查找重复
    std::map<int, int> nodeIdMapping;
    std::vector<int> nodesToRemove;
    double tolSq = tolerance * tolerance;

    for (const auto& bucketPair : buckets) 
    {
        const auto& key = bucketPair.first;
        const auto& indices = bucketPair.second;
        
        long long kx = std::get<0>(key);
        long long ky = std::get<1>(key);
        long long kz = std::get<2>(key);

        for (int idx1 : indices) 
        {
            int originalId1 = linearNodes[idx1].id;
            
            // 如果这个节点已经被合并了，跳过
            if (nodeIdMapping.count(originalId1)) continue;

            const auto& n1 = linearNodes[idx1];

            // 搜索 3x3x3 邻域
            for (long long dx = -1; dx <= 1; ++dx) 
            {
                for (long long dy = -1; dy <= 1; ++dy) 
                {
                    for (long long dz = -1; dz <= 1; ++dz) 
                    {
                        
                        HashKey neighborKey = std::make_tuple(kx + dx, ky + dy, kz + dz);
                        auto it = buckets.find(neighborKey);
                        if (it == buckets.end()) continue;

                        for (int idx2 : it->second) 
                        {
                            int originalId2 = linearNodes[idx2].id;
                            
                            // 确保 id2 > id1，且避免自身比较
                            if (originalId2 <= originalId1) continue;
                            
                            // 如果对方已经被合并，跳过
                            if (nodeIdMapping.count(originalId2)) continue;

                            const auto& n2 = linearNodes[idx2];

                            // 快速轴比较
                            double dxx = std::abs(n1.x - n2.x);
                            if (dxx > tolerance) continue;
                            double dyy = std::abs(n1.y - n2.y);
                            if (dyy > tolerance) continue;
                            double dzz = std::abs(n1.z - n2.z);
                            if (dzz > tolerance) continue;

                            // 距离平方比较
                            if (dxx*dxx + dyy*dyy + dzz*dzz < tolSq) 
                            {
                                nodeIdMapping[originalId2] = originalId1;
                                nodesToRemove.push_back(originalId2);
                            }
                        }
                    }
                }
            }
        }
    }

    if (nodesToRemove.empty()) return;

    // 6. 更新引用
    auto getNewId = [&](int oldId) -> int 
    {
        auto it = nodeIdMapping.find(oldId);
        return (it != nodeIdMapping.end()) ? it->second : oldId;
    };

    // 更新单元
    for (auto& elemPair : m_Elements) 
    {
        auto& elem = elemPair.second;
        for (int i = 0; i < elem->m_pNode.size(); ++i) 
        {
            auto ptr = elem->m_pNode[i].lock();
            if (ptr) 
            {
                int newId = getNewId(ptr->m_Id);
                if (newId != ptr->m_Id) 
                {
                    elem->m_pNode[i] = m_Nodes[newId]; 
                }
            }
        }
    }

    // 更新约束
    for (auto& conPair : m_Constraint) 
    {
        auto ptr = conPair.second->m_pNode.lock();
        if (ptr) 
        {
            int newId = getNewId(ptr->m_Id);
            if (newId != ptr->m_Id) 
            {
                conPair.second->m_pNode = m_Nodes[newId];
            }
        }
    }

    // 更新荷载
    for (auto& loadPair : m_Load) 
    {
        auto forceNode = std::dynamic_pointer_cast<Force_Node>(loadPair.second);
        if (forceNode) 
        {
            auto ptr = forceNode->m_pNode.lock();
            if (ptr) 
            {
                int newId = getNewId(ptr->m_Id);
                if (newId != ptr->m_Id) 
                {
                    forceNode->m_pNode = m_Nodes[newId];
                }
            }
        }
    }

    // 7. 批量删除
    for (int id : nodesToRemove) 
    {
        m_Nodes.erase(id);
    }
}

struct VectorHash 
{
    std::size_t operator()(const std::vector<int>& v) const 
    {
        std::size_t seed = 0;
        for (int i : v) 
        {
            // 经典的 hash combine 算法
            seed ^= std::hash<int>{}(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

void StructureData::RemoveDuplicateElements()
{
    if (m_Elements.empty()) return;

    std::unordered_set<std::vector<int>, VectorHash> seenTopologies;
    seenTopologies.reserve(m_Elements.size());

    std::vector<int> elementsToRemove;
    
    std::vector<int> nodeIds;
    nodeIds.reserve(4); // 预留常见单元节点数

    for (auto& elemPair : m_Elements)
    {
        auto& elem = elemPair.second;
        
        nodeIds.clear();
        
        bool isValid = true;
        for (auto& weakNode : elem->m_pNode) 
        {
            auto node = weakNode.lock();
            if (!node) 
            {
                isValid = false; 
                break; 
            }
            nodeIds.push_back(node->m_Id);
        }
        
        if (!isValid || nodeIds.empty()) 
        {
            elementsToRemove.push_back(elemPair.first);
            continue;
        }

        std::sort(nodeIds.begin(), nodeIds.end());

        if (!seenTopologies.insert(nodeIds).second)
        {
            elementsToRemove.push_back(elemPair.first);
        }
    }
    
    if (elementsToRemove.empty()) return;
    
    for (int id : elementsToRemove)
    {
        m_Elements.erase(id);
    }
}

void StructureData::RemoveOrphanNodes()
{
    if (m_Nodes.empty()) return;

    // 1. 找出当前最大的节点 ID，确定 vector 大小
    int maxNodeId = m_Nodes.rbegin()->first;
    
    // 2. 创建标记数组 (比 set 快几十倍)
    std::vector<bool> isNodeUsed(maxNodeId + 1, false);
    
    // 3. 遍历所有单元，标记用到的节点
    for (const auto& elemPair : m_Elements)
    {
        for (const auto& nodeWeak : elemPair.second->m_pNode)
        {
            auto node = nodeWeak.lock();
            if (node && node->m_Id <= maxNodeId) 
            {
                isNodeUsed[node->m_Id] = true;
            }
        }
    }

    // 约束引用的节点也不能删
    for (const auto& conPair : m_Constraint) 
    {
        auto node = conPair.second->m_pNode.lock();
        if (node && node->m_Id <= maxNodeId) 
        {
            isNodeUsed[node->m_Id] = true;
        }
    }

    // 荷载引用的节点也不能删
    for (const auto& loadPair : m_Load) 
    {
        auto forceNode = std::dynamic_pointer_cast<Force_Node>(loadPair.second);
        if (forceNode) 
        {
            auto node = forceNode->m_pNode.lock();
            if (node && node->m_Id <= maxNodeId) 
            {
                isNodeUsed[node->m_Id] = true;
            }
        }
    }

    // 4. 收集孤立节点
    std::vector<int> orphanNodes;
    orphanNodes.reserve(m_Nodes.size() / 10);

    for (const auto& nodePair : m_Nodes)
    {
        int id = nodePair.first;
        if (id > maxNodeId || !isNodeUsed[id])
        {
            orphanNodes.push_back(id);
        }
    }
    
    if (orphanNodes.empty()) return;
    
    for (int id : orphanNodes)
    {
        m_Nodes.erase(id);
    }
}

void StructureData::RenumberAll()
{
    // 重新编号节点
    std::map<int, std::shared_ptr<Node>> newNodes;
    int newId = 1;
    auto hint = newNodes.begin();
    
    for (auto& pair : m_Nodes)
    {
        pair.second->m_Id = newId;
        // 使用 emplace_hint 加速有序插入
        hint = newNodes.emplace_hint(hint, newId, pair.second);
        newId++;
    }
    m_Nodes = std::move(newNodes);
    
    // 重新编号单元
    std::map<int, std::shared_ptr<ElementBase>> newElements;
    newId = 1;
    for (auto& pair : m_Elements)
    {
        pair.second->m_Id = newId;
        newElements[newId] = pair.second;
        newId++;
    }
    m_Elements = std::move(newElements);
    
    // 重新编号约束
    std::map<int, std::shared_ptr<Constraint>> newConstraints;
    newId = 1;
    for (auto& pair : m_Constraint)
    {
        pair.second->m_Id = newId;
        newConstraints[newId] = pair.second;
        newId++;
    }
    m_Constraint = std::move(newConstraints);
    
    // 重新编号荷载
    std::map<int, std::shared_ptr<LoadBase>> newLoads;
    newId = 1;
    for (auto& pair : m_Load)
    {
        pair.second->m_Id = newId;
        newLoads[newId] = pair.second;
        newId++;
    }
    m_Load = std::move(newLoads);
}
