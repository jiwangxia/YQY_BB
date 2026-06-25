#pragma once

#include "Base/Base.h"
#include "Conductor.h"
#include "Utility/EnumKeyword.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

class Property;
class StructureData;
class ElementBase;

namespace Conductor
{
    /**
     * @brief 单档导线生成参数
     *
     * 该结构只描述导线生成所需的几何、离散、单元类型和属性。
     * 材料、截面由外部模型系统创建，并通过 property 传入。
     */
    struct LineBuildConfig
    {
        Vector3d start = Vector3d::Zero();                                  ///< 左挂点坐标
        Vector3d end = Vector3d::Zero();                                    ///< 右挂点坐标
        double leftCutLength = 0.0;                                         ///< 左端扣除弧长，单位 m
        double rightCutLength = 0.0;                                        ///< 右端扣除弧长，单位 m
        EnumKeyword::ElementType elementType = EnumKeyword::ElementType::T3D2; ///< 导线单元类型，默认桁架单元
        ConductorConfig conductor;                                          ///< 裸导线几何和初始应力参数
        std::shared_ptr<Property> property;                                 ///< 导线材料和截面属性
    };

    /**
     * @brief 单根子导线的节点和单元索引
     */
    struct SubConductorModel
    {
        int wireId = 0;                    ///< 子导线编号
        std::vector<int> nodeIds;          ///< 子导线节点 ID
        std::vector<int> elementIds;       ///< 子导线单元 ID
    };

    /**
     * @brief 相内间隔棒生成参数
     *
     * 间隔棒有独立的单元类型和属性，不要求与导线单元一致。
     */
    struct InnerSpacerConfig
    {
        double position = 0.0;                                             ///< 间隔棒位置，按距离或比例解释
        bool useRatio = false;                                             ///< true=position 为比例，false=position 为距左端距离
        bool createCenterNode = true;                                      ///< 多分裂时是否创建截面中心节点
        EnumKeyword::ElementType elementType = EnumKeyword::ElementType::CR3D; ///< 间隔棒单元类型，默认 CR3D
        std::shared_ptr<Property> property;                                ///< 间隔棒材料和截面属性
    };

    /**
     * @brief 相内间隔棒生成结果
     */
    struct InnerSpacerModel
    {
        int id = 0;                         ///< 间隔棒主 ID，取首个生成单元 ID
        double position = 0.0;              ///< 实际位置，距左端水平距离，单位 m
        double ratio = 0.0;                 ///< 档距比例
        int centerNodeId = -1;              ///< 多分裂截面中心节点 ID
        std::vector<int> nodeIds;           ///< 间隔棒关联节点 ID
        std::vector<int> elementIds;        ///< 间隔棒单元 ID
    };

    /**
     * @brief 单档导线生成结果
     */
    struct LineBuildResult
    {
        Vector3d start = Vector3d::Zero();                         ///< 左挂点坐标
        Vector3d end = Vector3d::Zero();                           ///< 右挂点坐标
        double spanLength = 0.0;                                   ///< 水平档距，单位 m
        std::map<int, SubConductorModel> subConductors;            ///< 子导线索引
        std::vector<InnerSpacerModel> innerSpacers;                ///< 相内间隔棒索引
        std::shared_ptr<Property> property;                        ///< 导线属性

        /**
         * @brief 获取导线结果中登记的唯一节点数量
         */
        int NodeCount() const;

        /**
         * @brief 获取导线结果中登记的唯一单元数量
         */
        int ElementCount() const;
    };

    /**
     * @brief 导线模型生成器
     *
     * 该类负责把裸导线几何结果转换为 StructureData 中的节点和单元。
     * 当前阶段支持单档相线/地线生成以及多分裂相内间隔棒生成。
     */
    class ConductorModelBuilder
    {
    public:
        explicit ConductorModelBuilder(std::shared_ptr<StructureData> structure);
        explicit ConductorModelBuilder(StructureData* structure);

        /**
         * @brief 生成单档导线
         * @param [in] config 导线生成参数
         * @param [out] result 导线节点、单元和子导线索引
         * @param [out] error 失败原因
         * @return 成功返回 true
         */
        bool BuildLine(const LineBuildConfig& config, LineBuildResult& result, std::string& error);

        /**
         * @brief 在已有导线上生成一个相内间隔棒
         * @param [in,out] line 已生成的导线结果
         * @param [in] config 间隔棒生成参数
         * @param [out] spacer 间隔棒生成结果
         * @param [out] error 失败原因
         * @return 成功返回 true
         */
        bool BuildInnerSpacer(LineBuildResult& line, const InnerSpacerConfig& config, InnerSpacerModel& spacer, std::string& error);

        /**
         * @brief 在已有导线上批量生成相内间隔棒
         * @param [in,out] line 已生成的导线结果
         * @param [in] configs 间隔棒生成参数集合
         * @param [out] error 失败原因
         * @return 成功返回 true
         */
        bool BuildInnerSpacers(LineBuildResult& line, const std::vector<InnerSpacerConfig>& configs, std::string& error);

    private:
        std::shared_ptr<StructureData> m_ownedStructure;
        StructureData* m_structure = nullptr;

        int NextNodeId() const;
        int NextElementId() const;

        bool ValidateProperty(std::shared_ptr<Property> property, const std::string& objectName, std::string& error) const;
        std::shared_ptr<ElementBase> CreateLineElement(EnumKeyword::ElementType elementType, std::string& error) const;
        void PrepareElementLocalFrame(std::shared_ptr<ElementBase> element) const;
        int FindNodeIdByRatio(const SubConductorModel& sub, double ratio) const;
        bool AddElement(int iNodeId, int jNodeId, EnumKeyword::ElementType elementType, std::shared_ptr<Property> property, double initStress, int& elementId, std::string& error);
        bool AddNodes(BundleResult& raw, LineBuildResult& result, std::string& error);
        bool AddElements(const BundleResult& raw, const LineBuildConfig& config, std::shared_ptr<Property> property, LineBuildResult& result, std::string& error);
    };
}
