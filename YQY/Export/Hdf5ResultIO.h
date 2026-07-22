#pragma once
/**
 * @file Hdf5ResultIO.h
 * @brief H5/HDF5 文件读写接口，用于模型数据和计算结果的结构化导入导出。
 */

#include <QString>
#include <memory>
#include <vector>
#include "Utility/EnumKeyword.h"

class StructureData;
class DataFrame;

struct Hdf5ResultSummary
{
    bool hasModel = false;
    bool hasResult = false;
    qint64 frameCount = 0;
    qint64 displacementRecordCount = 0;
    qint64 stressRecordCount = 0;
    qint64 strainRecordCount = 0;
    bool partialResult = false;
};

struct Hdf5ResultFrameInfo
{
    int domainId = 0;
    int stepId = 0;
    int increment = 0;
    int analysis = 0;
    double time = 0.0;
    double loadFactor = 1.0;
};

struct Hdf5NodalResult
{
    int id = 0;
    double displacement[3] = {};
    double currentCoordinate[3] = {};
};

struct Hdf5ElementResult
{
    int id = 0;
    double axialForce = 0.0;
    double currentStress = 0.0;
    double strain = 0.0;
};

struct Hdf5ResultFrame
{
    Hdf5ResultFrameInfo info;
    std::vector<Hdf5NodalResult> nodes;
    std::vector<Hdf5ElementResult> elements;
};

struct Hdf5ResultRange
{
    double minimum = 0.0;
    double maximum = 0.0;
    bool valid = false;
};

struct Hdf5ResultRanges
{
    Hdf5ResultRange displacementMagnitude;
    Hdf5ResultRange displacementX;
    Hdf5ResultRange displacementY;
    Hdf5ResultRange displacementZ;
    Hdf5ResultRange axialForce;
    Hdf5ResultRange stress;
    Hdf5ResultRange strain;
};

/**
 * @brief H5/HDF5 文件读写类。
 *
 * H5 文件用于保存前处理模型数据和后处理结果数据。模型数据写入 INPUT 分组，
 * 计算结果写入 RESULT 分组，并通过 INDEX 分组记录每一帧结果在结果数据集中的位置。
 */
class Hdf5ResultIO
{
public:
    Hdf5ResultIO();
    ~Hdf5ResultIO();

    Hdf5ResultIO(const Hdf5ResultIO&) = delete;
    Hdf5ResultIO& operator=(const Hdf5ResultIO&) = delete;

    /**
     * @brief 导出 H5/HDF5 文件。
     * @param [in] fileName 输出文件名。
     * @param [in] pData 结构数据指针。
     * @return 成功返回 true，失败返回 false。
     */
    bool ExportHdf5(const QString& fileName, const StructureData* pData) const;

    /**
     * @brief 导出 H5/HDF5 文件。
     * @param [in] fileName 输出文件名。
     * @param [in] pData 结构数据指针。
     * @param [in] sourceModelName 原始模型文件名，可以为空。
     * @return 成功返回 true，失败返回 false。
     */
    bool ExportHdf5(const QString& fileName, const StructureData* pData,
        const QString& sourceModelName, bool resultComplete = true) const;

    /**
     * @brief 开始 H5/HDF5 结果流式输出。
     * @param [in] fileName 输出文件名。
     * @param [in] pData 结构数据指针。
     * @param [in] sourceModelName 原始模型文件名，可以为空。
     * @return 成功返回 true，失败返回 false。
     */
    bool BeginResultStream(const QString& fileName, const StructureData* pData, const QString& sourceModelName);

    /**
     * @brief 写入单帧结果。
     * @param [in] domainId 结果域编号。
     * @param [in] stepId 分析步编号。
     * @param [in] increment 增量步编号。
     * @param [in] analysis 分析类型。
     * @param [in] time 当前时间。
     * @param [in] frame 单帧结果数据。
     * @return 成功返回 true，失败返回 false。
     */
    bool WriteResultFrame(int domainId, int stepId, int increment, int analysis, double time, const DataFrame& frame);

    /**
     * @brief 结束 H5/HDF5 结果流式输出。
     */
    void EndResultStream(bool resultComplete = true);

    /**
     * @brief 从 H5/HDF5 结果文件转换输出 BDF 风格结果文件。
     * @param [in] hdf5FileName H5/HDF5 文件名。
     * @param [in] bdfFileName BDF 结果文件名。
     * @param [in] nodeIds 需要输出的节点编号数组。
     * @param [in] nodeTypes 需要输出的节点结果类型数组。
     * @param [in] elementIds 需要输出的单元编号数组。
     * @param [in] elementTypes 需要输出的单元结果类型数组。
     * @return 成功返回 true，失败返回 false。
     */
    bool ExportBdfResultFromHdf5(const QString& hdf5FileName,
        const QString& bdfFileName,
        const std::vector<int>& nodeIds,
        const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
        const std::vector<int>& elementIds,
        const std::vector<EnumKeyword::ElementResultType>& elementTypes) const;

    /**
     * @brief 导入 H5/HDF5 文件。
     * @param [in] fileName 输入文件名。
     * @param [out] pData 结构数据指针。
     * @return 成功返回 true，失败返回 false。
     */
    bool ImportHdf5(const QString& fileName, StructureData* pData) const;

    /**
     * @brief 快速检查 H5 文件中的模型和结果数据规模，不读取完整结果数组。
     */
    bool InspectHdf5(const QString& fileName, Hdf5ResultSummary& summary) const;

    // Opens one result file and keeps an ASCII-path temporary copy alive so
    // individual frames can be read efficiently by their HDF5 index records.
    bool OpenResultFile(const QString& fileName, std::vector<Hdf5ResultFrameInfo>& frames);
    bool ReadResultRanges(Hdf5ResultRanges& ranges) const;
    bool ReadResultFrame(int frameIndex, Hdf5ResultFrame& frame) const;
    void CloseResultFile();

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
