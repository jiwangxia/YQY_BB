#pragma once
/**
 * @file Outputter.h
 * @brief 分析结果输出管理器 - 用于静力/动力学分析结果的保存和导出
 */

#include <map>
#include <vector>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <Eigen/Dense>

class StructureData;
class Node;

/**
 * @brief 数据类型枚举
 */
enum class DataType : int {
    U1, U2, U3, MagnitudeU,      ///< 位移
    V1, V2, V3,                   ///< 速度
    A1, A2, A3,                   ///< 加速度
    UR1, UR2, UR3,                ///< 转角
    N, M2, M3, Mises              ///< 内力
};

/**
 * @brief 单个节点在某一时刻的数据快照
 */
class NodeData
{
public:
    NodeData() = default;

    /**
     * @brief 从向量中提取节点数据
     * @param [in] dofs 节点的DOF编号数组
     * @param [in] nFixed 约束自由度数量
     * @param [in] x 位移向量 (仅自由DOF部分)
     * @param [in] v 速度向量 (仅自由DOF部分，可为空)
     * @param [in] a 加速度向量 (仅自由DOF部分，可为空)
     */
    void ExtractFromVectors(const QVector<int>& dofs, int nFixed,
                            const Eigen::VectorXd& x,
                            const Eigen::VectorXd* v = nullptr,
                            const Eigen::VectorXd* a = nullptr);

    /**
     * @brief 根据类型获取数据值
     */
    double GetValue(DataType type) const;

private:
    double m_u1 = 0, m_u2 = 0, m_u3 = 0;       ///< 位移
    double m_magnitudeU = 0;                   ///< 位移幅值
    double m_v1 = 0, m_v2 = 0, m_v3 = 0;       ///< 速度
    double m_a1 = 0, m_a2 = 0, m_a3 = 0;       ///< 加速度
    double m_ur1 = 0, m_ur2 = 0, m_ur3 = 0;    ///< 转角
};

/**
 * @brief 单帧数据 (某一时间点所有节点的状态)
 */
class DataFrame
{
    friend class Outputter;  ///< 允许 Outputter 访问私有成员

public:
    /**
     * @brief 获取当前时间
     */
    double GetTime() const { return m_currentTime; }

    /**
     * @brief 获取指定节点的指定类型数据
     */
    double GetNodeData(int idNode, DataType type) const;

private:
    double m_currentTime = 0;                  ///< 当前时间
    std::map<int, NodeData> m_nodeDatas;       ///< 节点ID -> 节点数据
};

/**
 * @brief 输出管理器 - 管理多帧分析结果数据
 * 
 * 支持增量式保存：每完成一个时间步/荷载步，调用 SaveData() 保存当前状态。
 * 即使分析未完成，已保存的帧也可以随时导出。
 */
class Outputter
{
public:
    Outputter() = default;
    ~Outputter() { Clear(); }

    /**
     * @brief 保存当前时刻数据 (增量式)
     * @param [in] time 当前时间
     * @param [in] pData 结构数据指针
     * @param [in] nFixed 约束自由度数量
     * @param [in] x 位移向量 (自由DOF部分)
     * @param [in] v 速度向量 (自由DOF部分，静力学可传nullptr)
     * @param [in] a 加速度向量 (自由DOF部分，静力学可传nullptr)
     */
    void SaveData(double time, StructureData* pData, int nFixed,
                  const Eigen::VectorXd& x,
                  const Eigen::VectorXd* v = nullptr,
                  const Eigen::VectorXd* a = nullptr);

    /**
     * @brief 导出指定节点的时程数据到文件
     * @param [in] fileName 输出文件名
     * @param [in] nodeIds 要输出的节点ID列表
     * @param [in] types 要输出的数据类型列表
     */
    void ExportNodes(const QString& fileName,
                     const std::vector<int>& nodeIds,
                     const std::vector<DataType>& types) const;

    /**
     * @brief 获取帧数
     */
    size_t GetFrameCount() const { return m_DataSet.size(); }

    /**
     * @brief 清除所有数据
     */
    void Clear() { m_DataSet.clear(); }

    /**
     * @brief 获取数据集 (只读)
     */
    const std::vector<DataFrame>& GetDataSet() const { return m_DataSet; }

private:
    std::vector<DataFrame> m_DataSet;  ///< 帧数据集合

    /**
     * @brief 获取数据类型名称
     */
    static QString GetTypeName(DataType type);
};
