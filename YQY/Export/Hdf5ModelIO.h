#pragma once
/**
 * @file Hdf5ModelIO.h
 * @brief H5/HDF5 文件读写接口，用于模型数据和计算结果的结构化导入导出。
 */

#include "Hdf5ResultData.h"
#include "Base/EmptyOUT.h"
#include "Utility/EnumKeyword.h"

#include <QString>
#include <memory>
#include <vector>

class StructureData;
class DataFrame;
struct SolverIterationRecord;

/**
 * @brief H5/HDF5 文件读写类。
 *
 * H5 文件用于保存前处理模型数据和后处理结果数据。模型数据写入 INPUT 分组，
 * 计算结果写入 RESULT 分组，并通过 INDEX 分组记录每一帧结果在结果数据集中的位置。
 */
class Hdf5ModelIO
{
public:
    Hdf5ModelIO();
    ~Hdf5ModelIO();

    Hdf5ModelIO(const Hdf5ModelIO&) = delete;
    Hdf5ModelIO& operator=(const Hdf5ModelIO&) = delete;

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
    bool ExportHdf5(const QString& fileName, const StructureData* pData, const QString& sourceModelName,
                    bool resultComplete = true) const;

    /**
     * @brief 仅导出模型与分析配置，不创建 RESULT 分组。
     */
    bool ExportModelHdf5(const QString& fileName, const StructureData* pData,
                         const QString& sourceModelName = QString()) const;

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
    bool WriteResultFrames(int firstDomainId, const std::vector<DataFrame>& frames);
    bool WriteSolverIterationHistory(const std::vector<SolverIterationRecord>& records);

    /**
     * @brief 结束 H5/HDF5 结果流式输出。
     */
    bool EndResultStream(bool resultComplete = true);

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
    bool ExportBdfResultFromHdf5(const QString& hdf5FileName, const QString& bdfFileName,
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
    bool InspectHdf5(const QString& fileName, _OUT Hdf5ResultSummary& summary) const;

    // Opens one result file and keeps an ASCII-path temporary copy alive so
    // individual frames can be read efficiently by their HDF5 index records.
    bool OpenResultFile(const QString& fileName, _OUT std::vector<Hdf5ResultFrameInfo>& frames);
    bool ReadResultRanges(_OUT Hdf5ResultRanges& ranges) const;
    bool ReadResultFrame(int frameIndex, _OUT Hdf5ResultFrame& frame) const;
    bool RestoreLastDynamicState(const QString& fileName, StructureData* pData, _OUT double* time = nullptr,
                                 _OUT int* stepId = nullptr);
    bool ReadSolverIterationHistory(_OUT std::vector<SolverIterationRecord>& records) const;
    void CloseResultFile();

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};
