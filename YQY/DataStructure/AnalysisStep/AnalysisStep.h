#pragma once
#include "Base/Base.h"
#include "Solver/Interface/IAnalysisModel.h"
#include "Solver/Interface/ISolver.h"
#include <memory>
#include <Eigen/SparseLU>
typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> Tri;

class StructureData;
class Force_Node;
class Force_Element;
class Force_Gravity;
class Force_Wind;

/**
 * @brief 分析步类 - 负责单次分析的完整流程
 * 
 * 实现 SolverNS::IAnalysisModel 接口，可以使用独立的求解器进行求解
 */
class AnalysisStep : public Base, public SolverNS::IAnalysisModel
{
public:
    EnumKeyword::StepType m_Type = EnumKeyword::StepType::UNKNOWN;
    double m_Time = 0.0;           // 总时间
    double m_StepSize = 0.0;       // 每步大小
    double m_Tolerance = 1e-5;     // 容差
    int m_MaxIterations = 32;      // 最大迭代次数

    /// @brief 动力求解器类型（仅动力分析时有效）
    /// 可选: Newmark, CentralDifference, HHT
    /// 目前只实现了 Newmark，其他为预留
    SolverNS::SolverType m_DynamicSolverType = SolverNS::SolverType::Newmark;

    int m_nFixed = 0;              // 约束自由度个数
    int m_nFree = 0;               // 自由自由度个数
    SpMat m_K11, m_K21, m_K22;
    SpMat m_M11, m_M21, m_M22;
    SpMat m_C11, m_C21, m_C22;

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

    /**
     * @brief 根据分析步类型调度求解
     */
    void Solve();

    /**
     * @brief 静力求解
     */
     /**
      * @brief 静力求解
      * @param [in] bResetState 是否重置状态（默认为 true，即从零开始求解；false 则在当前变形基础上继续求解）
      */
    void Solve_Static();

    /**
     * @brief 动力求解 (调用 SolverNewmark)
     */
    void Solve_Dynamic();

    // ============ IAnalysisModel 接口实现 ============
    int GetFreeDofs() const override { return m_nFree; }
    int GetFixedDofs() const override { return m_nFixed; }
    void ApplyIncrement(const SolverNS::Vec& dx, Phase phase) override;
    void SetTrialKinematics(const SolverNS::Vec& v, const SolverNS::Vec& a) override;
    void GetState(SolverNS::Vec& u, SolverNS::Vec& v, SolverNS::Vec& a) const override;
    void AssembleMatrices(SolverNS::SpMat& K, SolverNS::SpMat* M = nullptr, SolverNS::SpMat* C = nullptr) override;
    void ComputeResidual(double time, double loadFactor, SolverNS::Vec& R) override;
    void OnStepCompleted(double time) override;

private:
    std::weak_ptr<StructureData> m_pStructure;  // 结构数据的弱引用
    StructureData* m_pData = nullptr;           // 结构数据的缓存指针

    struct SolverCache 
    {
        Eigen::SimplicialLDLT<SpMat> ldlt; // 首选 (快)
        Eigen::SparseLU<SpMat> lu;         // 备选 (稳)
        bool use_ldlt = true;              // 当前策略
        bool pattern_analyzed = false;     // 模式是否已分析

        void reset() 
        {
            use_ldlt = true;
            pattern_analyzed = false;
        }
    };

    SolverCache m_solverCache;

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
    * @brief 初始化节点内部变量数据
    */
    void Init_Nodevector();

    void Get_ElementLength();

    /**
     * @brief 静力组装整体刚度矩阵
     */
    void AssembleKs_Static();
    void Assemble_Matrix();

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
    void Assemble_AllLoads(VectorXd& F1, VectorXd& F2, double& Factor);

    /**
     * @brief 获取当前时刻的力向量
     * @param [in] current_time 当前时间
     * @return 当前时刻的力向量
     */
    void Updata_NodeData(VectorXd& x1, VectorXd& x2, VectorXd& F1, VectorXd* v2 = nullptr, VectorXd* a2 = nullptr);

    void Get_CurrentInforce(VectorXd& Inforce);

    bool Check_Rhs(Eigen::VectorXd& F2, Eigen::VectorXd& f2, Eigen::VectorXd& Rhs);
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
     * @brief 组装重力
     * @param [in] pForceGravity 单元重力指针
     * @param [in,out] F1 约束自由度对应的力向量（累加）
     * @param [in,out] F2 自由自由度对应的力向量（累加）
     * @param [in] current_time 当前时间
     */
    void Assemble_ForceGravity(Force_Gravity* pForceGravity, VectorXd& F1, VectorXd& F2, double& current_time);

    /**
     * @brief 组装风荷载
     * @param [in] pForceWind 单元风荷载指针
     * @param [in,out] F1 约束自由度对应的力向量（累加）
     * @param [in,out] F2 自由自由度对应的力向量（累加）
     * @param [in] current_time 当前时间
     */
    void Assemble_ForceWind(Force_Wind* pForceWind, VectorXd& F1, VectorXd& F2, double& current_time);

    /**
     * @brief 组装约束位移
     * @param [out] x1 约束位移向量
     */
    void Assemble_Constraint(VectorXd& x1);

    void Get_CurrentStepState(VectorXd& U, VectorXd& V, VectorXd& A) const;

    // ============ IAnalysisModel 接口辅助函数 ============
    VectorXd GetCurrentVelocity() const;
    VectorXd GetCurrentAcceleration() const;

};
