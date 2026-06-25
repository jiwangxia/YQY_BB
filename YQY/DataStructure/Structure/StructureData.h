#pragma once
#include "Base/Base.h"
#include "DataStructure/Node/Node.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Section/SectionBase.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Section/SectionRectangle.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Constraint/Constraint.h"
#include "DataStructure/Load/LoadBase.h"
#include "DataStructure/Load/Force_Node.h"
#include "DataStructure/Load/Force_Element.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Wind.h"
#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "Export/Outputter.h"

/**
 * @brief 输出控制参数 - 保存输入文件中定义的结果输出请求
 */
struct OutputControl
{
    bool m_EnableHdf5 = false;       ///< 是否输出 H5/HDF5 文件
    bool m_OutputModel = true;       ///< 是否写入模型数据
    bool m_OutputResult = true;      ///< 是否写入结果数据
    bool m_StreamResult = true;      ///< 动力分析是否按时间步流式写入结果
    QString m_Hdf5FileName;          ///< H5/HDF5 输出文件名
    QString m_SourceModelName;       ///< 原始输入模型文件名
};

/**
 * @brief 结构数据类 - 存储和管理整个有限元模型的所有数据
 */
class StructureData : public Base
{
public:
    /// @name 数据存储容器
    /// @{
    std::map<int, std::shared_ptr<Node>>              m_Nodes;        ///< 节点集合
    std::map<int, std::shared_ptr<ElementBase>>       m_Elements;     ///< 单元集合
    std::map<int, std::shared_ptr<Material>>          m_Material;     ///< 材料集合
    std::map<int, std::shared_ptr<SectionBase>>       m_Section;      ///< 截面集合
    std::map<int, std::shared_ptr<Property>>          m_Property;     ///< 属性集合
    std::map<int, std::shared_ptr<Constraint>>        m_Constraint;   ///< 约束集合
    std::map<int, std::shared_ptr<LoadBase>>          m_Load;         ///< 荷载集合
    std::map<int, std::shared_ptr<AnalysisStep>>      m_AnalysisStep; ///< 分析步集合
    /// @}

    OutputControl m_OutputControl;                                      ///< 输出控制参数

    ~StructureData();

    /// @name 查找函数
    /// @{
    /**
     * @brief 根据ID查找节点
     * @param [in] id 节点ID
     * @return 节点的共享指针，未找到返回 nullptr
     */
    std::shared_ptr<Node> FindNode(int id);

    /**
     * @brief 根据ID查找单元
     * @param [in] id 单元ID
     * @return 单元的共享指针，未找到返回 nullptr
     */
    std::shared_ptr<ElementBase> FindElement(int id);

    /**
     * @brief 根据ID查找材料
     * @param [in] id 材料ID
     * @return 材料的共享指针，未找到返回 nullptr
     */
    std::shared_ptr<Material> FindMaterial(int id);

    /**
     * @brief 根据ID查找截面
     * @param [in] id 截面ID
     * @return 截面的共享指针，未找到返回 nullptr
     */
    std::shared_ptr<SectionBase> FindSection(int id);

    /**
     * @brief 根据ID查找属性
     * @param [in] id 属性ID
     * @return 属性的共享指针，未找到返回 nullptr
     */
    std::shared_ptr<Property> FindProperty(int id);
    /// @}

    /**
     * @brief 创建属性对象
     * @param [in] id_material 材料ID
     * @param [in] id_section 截面ID
     * @return 创建的属性对象
     */
    std::shared_ptr<Property> Create_Property(int id_material, int id_section);
    void Add_Property(double E, double density, double Area, double* v = nullptr, double* S = nullptr, double* e = nullptr);
    /**
    * @brief 创建约束
    * @param [in] Nodeid 约束节点id集合
    * @param [in] direaction 所选节点的约束方向集合
    * @param [in] value 所选节点的约束方向对应的值的集合
    * @return 新增约束自由度数量
    */
    int Add_Constraint(std::vector<int> Nodeid, std::vector<int> direaction, std::vector<double> value);

    void Add_Gravity(int direction, int idStep);
    void Add_AnalysisStep(QString Name, double TotalTime, double increament, double tolerance, double maxIteration);


    /**
     * @brief 模型清理（合并重复节点、删除重复单元、删除孤立节点、重新编号）
     * @param [in] tolerance 节点合并容差
     */
    void CleanupModel(double tolerance = 1e-6);

    /**
     * @brief 清空所有数据
     */
    void Clear();

private:
    /**
     * @brief 合并重复节点
     * @param [in] tolerance 合并容差
     */
    void MergeDuplicateNodes(double tolerance);

    /**
     * @brief 删除重复单元
     */
    void RemoveDuplicateElements();

    /**
     * @brief 删除孤立节点（未被任何单元引用的节点）
     */
    void RemoveOrphanNodes();

    /**
     * @brief 重新编号所有数据
     */
    void RenumberAll();

public:
    Outputter m_Outputter;          // 分析结果输出

    /**
     * @brief 获取输出
     */
    Outputter& GetOutputter() { return m_Outputter; }
    const Outputter& GetOutputter() const { return m_Outputter; }
};
