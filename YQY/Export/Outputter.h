#pragma once
/**
 * @file Outputter.h
 * @brief 分析结果输出管理器 - 用于静力/动力分析结果的缓存和导出
 */

#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <QString>
#include "Utility/EnumKeyword.h"

class QFile;
class QTextStream;
class StructureData;
class Node;
class ElementBase;
class Hdf5ModelIO;

struct SolverIterationRecord
{
    int stepId = 0;
    int analysisType = 0;
    double time = 0.0;
    int iterations = 0;
};

/**
 * @brief 单个节点在某一时刻的结果快照
 */
class NodeData
{
public:
    NodeData() = default;

    /**
     * @brief 从节点对象中提取当前结果
     * @param [in] pNode 节点指针
     */
    void ExtractFromNode(const Node* pNode);

    /**
     * @brief 根据结果类型获取节点结果值
     * @param [in] type 节点结果类型
     * @return 对应结果值
     */
    double GetValue(EnumKeyword::NodeResultType type) const;

private:
    double m_cx = 0, m_cy = 0, m_cz = 0;       // 当前坐标
    double m_u1 = 0, m_u2 = 0, m_u3 = 0;       // 位移
    double m_magnitudeU = 0;                   // 位移幅值
    double m_v1 = 0, m_v2 = 0, m_v3 = 0;       // 速度
    double m_a1 = 0, m_a2 = 0, m_a3 = 0;       // 加速度
    double m_ur1 = 0, m_ur2 = 0, m_ur3 = 0;    // 转角
    double m_f1 = 0, m_f2 = 0, m_f3 = 0;       // 节点内力
    double m_m1 = 0, m_m2 = 0, m_m3 = 0;       // 节点力矩
    double m_r1 = 0, m_r2 = 0, m_r3 = 0;       // 节点反力
};

/**
 * @brief 单个单元在某一时刻的结果快照
 */
class ElementData
{
public:
    ElementData() = default;

    /**
     * @brief 从单元对象中提取当前结果
     * @param [in] pElement 单元指针
     */
    void ExtractFromElement(const ElementBase* pElement);

    /**
     * @brief 根据结果类型获取单元结果值
     * @param [in] type 单元结果类型
     * @return 对应结果值
     */
    double GetValue(EnumKeyword::ElementResultType type) const;

private:
    double m_axialForce = 0.0;                 // 轴力
    double m_shearY = 0.0;                     // 局部 y 向剪力
    double m_shearZ = 0.0;                     // 局部 z 向剪力
    double m_torque = 0.0;                     // 扭矩
    double m_momentY = 0.0;                    // 绕局部 y 轴弯矩
    double m_momentZ = 0.0;                    // 绕局部 z 轴弯矩
    double m_strain = 0.0;                     // 应变
    double m_initStress = 0.0;                 // 初始应力
    double m_currentStress = 0.0;              // 当前应力
    double m_deltaStress = 0.0;                // 应力增量
};

/**
 * @brief 单帧结果数据 - 保存某一时刻的节点和单元结果
 */
class DataFrame
{
    friend class Outputter;

public:
    /**
     * @brief 获取当前帧时间
     * @return 当前时间
     */
	double GetTime() const { return m_currentTime; }
    int GetStepId() const { return m_stepId; }
    int GetIncrement() const { return m_increment; }
    int GetAnalysisType() const { return m_analysisType; }

    /**
     * @brief 获取指定节点的指定结果
     * @param [in] idNode 节点编号
     * @param [in] type 节点结果类型
     * @return 对应结果值
     */
    double GetNodeData(int idNode, EnumKeyword::NodeResultType type) const;

    /**
     * @brief 获取指定单元的指定结果
     * @param [in] idElement 单元编号
     * @param [in] type 单元结果类型
     * @return 对应结果值
     */
    double GetElementData(int idElement, EnumKeyword::ElementResultType type) const;

    /**
     * @brief 获取当前帧的全部节点结果
     * @return 节点结果只读集合
     */
	const std::map<int, NodeData>& GetNodeDatas() const { return m_nodeDatas; }

    /**
     * @brief 获取当前帧的全部单元结果
     * @return 单元结果只读集合
     */
	const std::map<int, ElementData>& GetElementDatas() const { return m_elementDatas; }

private:
    double m_currentTime = 0;                         // 当前时间
    int m_stepId = 0;                                 // 所属分析步编号
    int m_increment = 0;                              // 分析步内帧序号
    int m_analysisType = 0;                           // EnumKeyword::StepType
    std::map<int, NodeData> m_nodeDatas;              // 节点编号 -> 节点结果
    std::map<int, ElementData> m_elementDatas;        // 单元编号 -> 单元结果
};

/**
 * @brief 输出管理器 - 管理多帧分析结果数据
 *
 * 默认将每帧结果保存在内存中，适合静力分析或小规模动力分析。
 * 当动力分析帧数较多时，可以开启 BDF 流式输出，边计算边写入文件。
 */
class Outputter
{
public:
    Outputter();
    ~Outputter();

    /**
     * @brief 保存当前时刻的节点和单元结果
     * @param [in] time 当前时间
     * @param [in] pData 结构数据指针
     */
    void SaveDataFromNodes(double time, StructureData* pData);

    /**
     * @brief 设置是否将结果帧保存在内存中
     * @param [in] keep true 表示保存到内存，false 表示只进行流式输出
     */
	void SetKeepFramesInMemory(bool keep) { m_keepFramesInMemory = keep; }
    void SetResultContext(int stepId, int analysisType);

    /**
     * @brief 导出指定节点的时程结果
     * @param [in] fileName 输出文件名
     * @param [in] nodeIds 节点编号数组
     * @param [in] types 节点结果类型数组
     */
    void ExportNodes(const QString& fileName,
        const std::vector<int>& nodeIds,
        const std::vector<EnumKeyword::NodeResultType>& types) const;

    /**
     * @brief 导出指定单元的时程结果
     * @param [in] fileName 输出文件名
     * @param [in] elementIds 单元编号数组
     * @param [in] types 单元结果类型数组
     */
    void ExportElements(const QString& fileName,
        const std::vector<int>& elementIds,
        const std::vector<EnumKeyword::ElementResultType>& types) const;

    /**
     * @brief 开始 BDF 风格结果流式输出
     * @param [in] fileName 输出文件名
     * @param [in] nodeIds 需要输出的节点编号数组
     * @param [in] nodeTypes 需要输出的节点结果类型数组
     * @param [in] elementIds 需要输出的单元编号数组
     * @param [in] elementTypes 需要输出的单元结果类型数组
     * @return 成功返回 true，失败返回 false
     */
    bool BeginBdfResultStream(const QString& fileName,
        const std::vector<int>& nodeIds,
        const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
        const std::vector<int>& elementIds,
        const std::vector<EnumKeyword::ElementResultType>& elementTypes);

    /**
     * @brief 结束 BDF 风格结果流式输出
     */
    void EndBdfResultStream();

    /**
     * @brief 开始 H5/HDF5 结果流式输出
     * @param [in] fileName 输出文件名
     * @param [in] pData 结构数据指针
     * @param [in] sourceModelName 原始模型文件名，可为空
     * @return 成功返回 true，失败返回 false
     */
    bool BeginHdf5ResultStream(const QString& fileName, StructureData* pData, const QString& sourceModelName = QString());

    /**
     * @brief 结束 H5/HDF5 结果流式输出
     */
    void EndHdf5ResultStream(bool resultComplete = true);

    /**
     * @brief 导出 BDF 模型文件
     * @param [in] fileName 输出文件名
     * @param [in] pData 结构数据指针
     * @return 成功返回 true
     */
    bool SaveBdfModel(const QString& fileName, StructureData* pData);

    /**
     * @brief 导出 H5/HDF5 文件
     * @param [in] fileName 输出文件名
     * @param [in] pData 结构数据指针
     * @param [in] sourceModelName 原始模型文件名，可为空
     * @return 成功返回 true，失败返回 false
     */
    bool SaveHdf5File(const QString& fileName, StructureData* pData,
        const QString& sourceModelName = QString(), bool resultComplete = true);

    /**
     * @brief 从 H5/HDF5 文件转换输出 BDF 风格结果文件
     */
    bool ExportBdfResultFromHdf5(const QString& hdf5FileName,
        const QString& bdfFileName,
        const std::vector<int>& nodeIds,
        const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
        const std::vector<int>& elementIds,
        const std::vector<EnumKeyword::ElementResultType>& elementTypes) const;

    /**
     * @brief 导出模型数据
     * @param [in] fileName 输出文件名
     * @param [in] pData 结构数据指针
     */
    void SaveModel(const QString& fileName, StructureData* pData);

    /**
     * @brief 获取当前缓存的结果帧数量
     * @return 结果帧数量
     */
	size_t GetFrameCount() const { return m_DataSet.size(); }
    const std::vector<DataFrame>& GetFrames() const { return m_DataSet; }

    /**
     * @brief 清除所有缓存结果并关闭流式输出
     */
    void Clear();

    void MergeFramesFrom(const Outputter& source);

    void RecordSolverIteration(double time, int iterations);
    const std::vector<SolverIterationRecord>& GetSolverIterationRecords() const
    {
        return m_solverIterationRecords;
    }
    void SetSolverIterationRecords(std::vector<SolverIterationRecord> records)
    {
        m_solverIterationRecords = std::move(records);
    }
    bool ExportSolverIterationHistory(const QString& fileName) const;

    /**
     * @brief 获取结果数据集
     * @return 结果数据集只读引用
     */
	const std::vector<DataFrame>& GetDataSet() const { return m_DataSet; }

private:
    std::vector<SolverIterationRecord> m_solverIterationRecords;
    std::vector<DataFrame> m_DataSet;                 // 结果帧集合
    bool m_keepFramesInMemory = true;                 // 是否将结果帧保存到内存
    std::unique_ptr<QFile> m_streamFile;              // 流式输出文件
    std::unique_ptr<QTextStream> m_stream;            // 流式输出文本流
    std::unique_ptr<Hdf5ModelIO> m_hdf5Stream;       // H5/HDF5 流式输出对象
    int m_hdf5NextDomainId = 1;                       // H5/HDF5 下一帧域编号
    int m_hdf5NextIncrement = 0;                      // H5/HDF5 下一增量编号
    int m_currentStepId = 0;                          // 当前输出分析步编号
    int m_currentAnalysisType = 0;                    // 当前输出分析类型
    std::vector<int> m_streamNodeIds;                 // 流式输出节点编号
    std::vector<EnumKeyword::NodeResultType> m_streamNodeTypes;       // 流式输出节点结果类型
    std::vector<int> m_streamElementIds;              // 流式输出单元编号
    std::vector<EnumKeyword::ElementResultType> m_streamElementTypes; // 流式输出单元结果类型

    /**
     * @brief 获取节点结果类型名称
     * @param [in] type 节点结果类型
     * @return 类型名称字符串
     */
    static QString GetTypeName(EnumKeyword::NodeResultType type);

    /**
     * @brief 获取单元结果类型名称
     * @param [in] type 单元结果类型
     * @return 类型名称字符串
     */
    static QString GetTypeName(EnumKeyword::ElementResultType type);

    /**
     * @brief 写入结果表头
     */
    void WriteResultTableHeader(QTextStream& stream,
        const std::vector<int>& nodeIds,
        const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
        const std::vector<int>& elementIds,
        const std::vector<EnumKeyword::ElementResultType>& elementTypes) const;

    /**
     * @brief 写入单帧结果
     */
    void WriteResultFrame(QTextStream& stream, const DataFrame& frame,
        const std::vector<int>& nodeIds,
        const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
        const std::vector<int>& elementIds,
        const std::vector<EnumKeyword::ElementResultType>& elementTypes) const;
};
