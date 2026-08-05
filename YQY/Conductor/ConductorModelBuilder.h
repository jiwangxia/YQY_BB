#pragma once

#include "Base/Base.h"
#include "Conductor.h"
#include "DataStructure/Element/ElementBase.h"
#include "Utility/EnumKeyword.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

class Property;
class StructureData;

namespace Conductor
{
    enum class BundleEndTopology
    {
        SingleSupport = 0,   ///< 两个分组节点汇集至一个公共挂点
        DualSupportByGroup,  ///< 两个分组节点分别连接至两个自动生成的挂点
        DirectWireSupports   ///< 每根子导线端点直接作为独立挂点，不生成端部金具
    };

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
        bool convergeBundleEnds = true;                                     ///< 多分裂导线两端是否汇集到公共支点
        BundleEndTopology endTopology = BundleEndTopology::SingleSupport;   ///< 耐张端部挂点形式
        double dualSupportSpacing = 0.0;                                    ///< 双挂点间距；非正值时取子导线间距
        double bundleEndTransitionLength = 1.0;                             ///< 公共支点到分裂截面的默认过渡弧长，单位 m
        EnumKeyword::ElementType endFittingElementType = EnumKeyword::ElementType::CR3D; ///< 耐张稳定梁单元类型
        std::shared_ptr<Property> endFittingProperty;                       ///< 耐张稳定梁属性；为空时复用导线属性
        QString setNamePrefix = QStringLiteral("单档导线");                  ///< 自动生成子导线集合的名称前缀
    };

    /**
     * @brief 单根子导线的节点和单元索引
     */
    struct SubConductorModel
    {
        int wireId = 0;                    ///< 子导线编号
        std::vector<int> nodeIds;          ///< 子导线节点 ID
        std::vector<int> elementIds;       ///< 子导线单元 ID
        int nodeSetId = -1;                ///< 子导线节点集合 ID
        int elementSetId = -1;             ///< 子导线单元集合 ID，可用于气动参数绑定
    };

    /**
     * @brief 相内间隔棒生成参数
     *
     * 间隔棒有独立的单元类型和属性，不要求与导线单元一致。
     */
    enum class InnerSpacerStyle
    {
        OuterPolygon = 0,
        CenterBraced,
        InnerPolygon
    };

    struct InnerSpacerConfig
    {
        double position = 0.0;                                             ///< 间隔棒位置，按距离或比例解释
        bool useRatio = false;                                             ///< true=position 为比例，false=position 为距左端距离
        InnerSpacerStyle style = InnerSpacerStyle::CenterBraced;           ///< 相内间隔棒形式
        double innerPolygonScale = 0.5;                                    ///< 内圈相对外框的比例
        EnumKeyword::ElementType elementType = EnumKeyword::ElementType::CR3D; ///< 间隔棒单元类型，默认 CR3D
        std::shared_ptr<Property> property;                                ///< 间隔棒材料和截面属性
    };

    /**
     * @brief 相内间隔棒自动布置参数
     */
    struct InnerSpacerLayoutConfig
    {
        int count = 0;                                                      ///< 间隔棒数量
        double startOffset = 0.0;                                           ///< 距左端避让距离，单位 m
        double endOffset = 0.0;                                             ///< 距右端避让距离，单位 m
        bool useEqualSpacing = true;                                        ///< true=均匀布置；false=按 THOP 标准次档距布置
        InnerSpacerConfig spacer;                                           ///< 单个间隔棒的单元类型和属性
    };

    struct TensionEndModel
    {
        int supportNodeId = -1;                                             ///< 无塔模型时的耐张挂点
        std::vector<int> supportNodeIds;                                    ///< 按分组顺序排列的耐张挂点；单挂点模式中仅含一个 ID
        std::vector<int> groupNodeIds;                                      ///< 子导线分组汇集节点
        std::vector<int> yokeElementIds;                                    ///< 挂点到分组节点的联板单元
        int stabilizerElementId = -1;                                       ///< 两个分组节点之间的稳定梁
    };

    /**
     * @brief 单档导线业务生成参数
     */
    struct SpanConductorBuildConfig
    {
        LineBuildConfig line;                                               ///< 导线生成参数
        std::vector<InnerSpacerConfig> innerSpacers;                        ///< 指定位置间隔棒
        InnerSpacerLayoutConfig innerSpacerLayout;                          ///< 自动布置间隔棒
        bool useInnerSpacerLayout = false;                                  ///< 是否使用自动布置
    };

    /**
     * @brief 多档中间直线塔的 H 型悬垂拓扑
     *
     * center 是用户输入的分裂多边形中心；junctionNodeId 位于其上方，
     * supportNodeId 再向上一个悬垂串长度，且仅 supportNodeId 施加边界约束。
     */
    struct SuspensionPointModel
    {
        int stationIndex = -1;
        Vector3d center = Vector3d::Zero();
        int junctionNodeId = -1;
        int supportNodeId = -1;
        std::vector<int> wireNodeIds;
        std::vector<int> yokeElementIds;
        int stringElementId = -1;
        int spacerCenterNodeId = -1;
        std::vector<int> spacerInnerNodeIds;
        std::vector<int> spacerElementIds;
    };

    /**
     * @brief 同一耐张段内的多档导线生成参数
     *
     * stationCenters 至少包含三个导线束中心坐标。首末站生成耐张端，
     * 中间站生成 H 型悬垂端；span 中的导线和间隔棒参数逐档复用。
     */
    struct MultiSpanConductorBuildConfig
    {
        SpanConductorBuildConfig span;
        std::vector<Vector3d> stationCenters;
        double suspensionStringLength = 1.0;
        EnumKeyword::ElementType suspensionElementType = EnumKeyword::ElementType::T3D2;
        std::shared_ptr<Property> suspensionProperty;
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
        std::vector<int> innerNodeIds;      ///< 内圈小多边形节点 ID
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
        int spanCount = 1;                                         ///< 结果中包含的档数
        int leftSupportNodeId = -1;                                ///< 左端公共支点节点 ID
        int rightSupportNodeId = -1;                               ///< 右端公共支点节点 ID
        TensionEndModel leftTensionEnd;                             ///< 左侧 THOP 式耐张端部
        TensionEndModel rightTensionEnd;                            ///< 右侧 THOP 式耐张端部
        std::vector<SuspensionPointModel> suspensionPoints;         ///< 中间 H 型悬垂点
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
         * @brief 生成单档导线，并按配置生成相内间隔棒
         * @param [in] config 单档导线业务生成参数
         * @param [out] result 导线和间隔棒生成结果
         * @param [out] error 失败原因
         * @return 成功返回 true
         */
        bool BuildSpanConductor(const SpanConductorBuildConfig& config, LineBuildResult& result, std::string& error);

        /**
         * @brief 生成同一耐张段内的多档导线
         *
         * 首末站使用当前单档耐张端逻辑，中间站按 THOP 的 H 型悬垂端逻辑生成；
         * 相邻两档共享中间站的各子导线端节点。
         */
        bool BuildMultiSpanConductor(const MultiSpanConductorBuildConfig& config, LineBuildResult& result, std::string& error);

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

        /**
         * @brief 按内置 THOP 次档距规则计算间隔棒位置
         * @param [in] spanLength 水平档距，单位 m
         * @return 各间隔棒距左挂点的位置，单位 m
         */
        static std::vector<double> CalculateStandardInnerSpacerPositions(double spanLength);

        /**
         * @brief 按规则生成相内间隔棒位置配置
         * @param [in] line 已生成的导线结果
         * @param [in] layout 间隔棒自动布置参数
         * @param [out] configs 生成的间隔棒配置
         * @param [out] error 失败原因
         * @return 成功返回 true
         */
        bool CalculateInnerSpacerConfigs(const LineBuildResult& line, const InnerSpacerLayoutConfig& layout, std::vector<InnerSpacerConfig>& configs, std::string& error) const;

    private:
        std::shared_ptr<StructureData> m_ownedStructure;
        StructureData* m_structure = nullptr;

        int NextNodeId() const;
        int NextElementId() const;

        bool ValidateProperty(std::shared_ptr<Property> property, const std::string& objectName, std::string& error) const;
        std::shared_ptr<ElementBase> CreateLineElement(EnumKeyword::ElementType elementType, std::string& error) const;
        void PrepareElementLocalFrame(std::shared_ptr<ElementBase> element) const;
        int FindNearestNodeOnSubConductor(const SubConductorModel& sub, const Vector3d& leftBase, const Vector2d& direction, double targetDistance, bool excludeEndpoints) const;
        bool AddElement(int iNodeId, int jNodeId, EnumKeyword::ElementType elementType, std::shared_ptr<Property> property,
            double initStress, int& elementId, std::string& error, ElementRole role = ElementRole::Generic,
            int wireId = -1, int aeroProfileId = -1, int aeroBundleCount = 0);
        bool AddNodes(BundleResult& raw, const LineBuildConfig& config, LineBuildResult& result, std::string& error);
        bool AddElements(const BundleResult& raw, const LineBuildConfig& config, std::shared_ptr<Property> property, LineBuildResult& result, std::string& error);
        bool AddTensionEndElements(const LineBuildConfig& config, LineBuildResult& result, std::string& error);
        bool BuildSuspensionSpacer(SuspensionPointModel& suspension, const InnerSpacerConfig& config, std::string& error);
        bool CreateSubConductorSets(const LineBuildConfig& config, LineBuildResult& result, std::string& error);
        bool RenumberLineModel(LineBuildResult& result, std::string& error);
    };
}
