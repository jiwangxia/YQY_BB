#pragma once
#include "Base/Base.h"
#include <memory>

typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> Tri;

class StructureData;
class Force_Node;
class Force_Element;

/**
 * @brief 分析步类 - 负责单次分析的完整流程
 */
class AnalysisStep : public Base
{
public:
    int m_Id = 0;
    EnumKeyword::StepType m_Type = EnumKeyword::StepType::UNKNOWN;
    double m_Time = 0.0;           ///< 总时间
    double m_StepSize = 0.0;       ///< 每步大小
    double m_Tolerance = 1e-5;     ///< 容差
    int m_MaxIterations = 32;      ///< 最大迭代次数

    int m_nFixed = 0;              ///< 约束自由度个数
    int m_nFree = 0;               ///< 自由自由度个数
    SpMat m_K11, m_K21, m_K22;

    /**
     * @brief 获取分析步类型名称
     * @return 类型名称字符串
     */
    QString GetTypeName() const { return EnumKeyword::MapStepType.key(m_Type, "UNKNOWN"); }

    /**
     * @brief 设置关联的结构数据
     * @param [in] pStructure 结构数据的共享指针
     */
    void SetStructure(std::shared_ptr<StructureData> pStructure);

    /**
     * @brief 获取关联的结构数据
     * @return 结构数据的裸指针
     */
    StructureData* GetStructure() const;

    /**
     * @brief 初始化分析步（DOF编号、刚度矩阵组装等）
     */
    void Init();

private:
    std::weak_ptr<StructureData> m_pStructure;  ///< 结构数据的弱引用
    StructureData* m_pData = nullptr;           ///< 结构数据的缓存指针

    /**
     * @brief 准备数据，缓存结构指针
     * @return 成功返回 true，失败返回 false
     */
    bool PrepareData();

    /**
     * @brief 初始化自由度编号
     */
    void Init_DOF();

    /**
     * @brief 组装整体刚度矩阵
     */
    void AssembleKs();

    /**
     * @brief 将单元刚度矩阵组装到整体刚度矩阵
     * @param [in] DOFs 单元自由度编号数组
     * @param [in] T 单元刚度矩阵
     * @param [in,out] L11 K11 矩阵的三元组列表
     * @param [in,out] L21 K21 矩阵的三元组列表
     * @param [in,out] L22 K22 矩阵的三元组列表
     */
    void Assemble(std::vector<int>& DOFs, Eigen::MatrixXd& T, std::list<Tri>& L11, std::list<Tri>& L21, std::list<Tri>& L22);

    /**
     * @brief 组装所有荷载到力向量
     * @param [out] F1 约束自由度对应的力向量
     * @param [out] F2 自由自由度对应的力向量
     */
    void Assemble_AllLoads(VectorXd& F1, VectorXd& F2);

    /**
     * @brief 获取当前时刻的力向量
     * @param [in] current_time 当前时间
     * @return 当前时刻的力向量
     */
    VectorXd Get_currentForce(double& current_time);

    /**
     * @brief 组装节点力荷载
     * @param [in] pForceNode 节点力荷载指针
     * @param [in,out] F1 约束自由度对应的力向量（累加）
     * @param [in,out] F2 自由自由度对应的力向量（累加）
     * @param [in] current_time 当前时间
     */
    void Assemble_ForceNode(Force_Node* pForceNode, VectorXd& F1, VectorXd& F2, double& current_time);

    /**
     * @brief 组装单元荷载
     * @param [in] pForceElement 单元荷载指针
     * @param [in,out] F1 约束自由度对应的力向量（累加）
     * @param [in,out] F2 自由自由度对应的力向量（累加）
     * @param [in] current_time 当前时间
     */
    void Assemble_ForceElement(Force_Element* pForceElement, VectorXd& F1, VectorXd& F2, double& current_time);

    /**
     * @brief 组装约束位移
     * @param [out] x1 约束位移向量
     */
    void AssembleConstraint(VectorXd& x1);
};
