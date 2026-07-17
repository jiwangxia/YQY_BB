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
 * @brief 分析步配置 - 描述一次分析任务的输入参数
 */
struct AnalysisStepConfig
{
    int id = 0;
    EnumKeyword::StepType type = EnumKeyword::StepType::UNKNOWN;
    double totalTime = 0.0;
    double stepSize = 0.0;
    double tolerance = 1e-5;
    int maxIterations = 32;
    SolverNameSpace::SolverType dynamicSolverType = SolverNameSpace::SolverType::Newmark;
};

/**
 * @brief 分析步类 - 负责单次分析的完整流程
 * 
 * 实现 SolverNameSpace::IAnalysisModel 接口，可以使用独立的求解器进行求解
 */
class AnalysisStep : public Base, public SolverNameSpace::IAnalysisModel
{
public:
    EnumKeyword::StepType m_Type = EnumKeyword::StepType::UNKNOWN;
    double m_Time = 0.0;           // 总时间
    double m_StepSize = 0.0;       // 每步大小
    double m_Tolerance = 1e-5;     // 容差
    int m_MaxIterations = 32;      // 最大迭代次数

    bool isDynamic = false;        // 是否为动力分析
    /// @brief 动力求解器类型（仅动力分析时有效）
    /// 可选: Newmark, CentralDifference, HHT
    /// 目前只实现了 Newmark，其他为预留
    SolverNameSpace::SolverType m_DynamicSolverType = SolverNameSpace::SolverType::Newmark;

    int m_nFixed = 0;              // 约束自由度个数
    int m_nFree = 0;               // 自由自由度个数
    SpMat m_Keff11, m_Keff21, m_Keff22;

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
    bool Solve();

    // ============ IAnalysisModel 接口实现 ============
    int  GetFreeDofs() const override { return m_nFree; }
    int  GetFixedDofs() const override { return m_nFixed; }
    void ApplyIncrement(const SolverNameSpace::Vec& dx) override;
    void BeginDynamicStep(double dt, double beta, double gamma) override;
    void ApplyDynamicCorrection(const SolverNameSpace::Vec& dx, double a0, double a1) override;
    void RollbackDynamicStep() override;
    void SetTrialKinematics(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a) override;
    void GetState(SolverNameSpace::Vec& u, SolverNameSpace::Vec& v, SolverNameSpace::Vec& a) const override;
    void Assemble_Matrix(SpMat& Keff, bool isDynamic);          //组装整体等效刚度矩阵
    void ComputeResidual(const SolverNameSpace::Vec& F_ext, SolverNameSpace::Vec& R) override;
    void OnStepCompleted(double time) override;
    void CommitState() override;

private:
    std::weak_ptr<StructureData> m_pStructure;  // 结构数据的弱引用
    StructureData* m_pData = nullptr;           // 结构数据的缓存指针

    struct SolverCache 
    {
        Eigen::SimplicialLDLT<SpMat> ldlt; // 首选 (快)
        Eigen::SparseLU<SpMat> lu;         // 备选 (稳)
        bool use_ldlt = false;              // 当前策略
        bool pattern_analyzed = false;     // 模式是否已分析

        void reset() 
        {
            use_ldlt = false;
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
     * @param [in] Factor 荷载缩放系数
     * @param [in] currentTime 当前时间
     */
    void Assemble_AllLoads(VectorXd& F1, VectorXd& F2, double& Factor, double currentTime);

    /**
     * @brief 计算外荷载（不包含内力和惯性力）
     * @param [in] time 当前时间
     * @param [in] loadFactor 荷载缩放系数
     * @param [out] F1 约束自由度对应的外力向量
     * @param [out] F2 自由自由度对应的外力向量
     */
    void ComputeExternalForce(double time, double loadFactor, VectorXd& F1, VectorXd& F2);

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
     * @param [in] loadScale 荷载缩放系数
     */
    void Assemble_ForceNode(Force_Node* pForceNode, VectorXd& F1, VectorXd& F2, double& current_time, double loadScale);

    /**
     * @brief 组装单元荷载
     * @param [in] pForceElement 单元荷载指针
     * @param [in,out] F1 约束自由度对应的力向量（累加）
     * @param [in,out] F2 自由自由度对应的力向量（累加）
     * @param [in] current_time 当前时间
     * @param [in] loadScale 荷载缩放系数
     */
    void Assemble_ForceElement(Force_Element* pForceElement, VectorXd& F1, VectorXd& F2, double& current_time, double loadScale);

    /**
     * @brief 组装重力
     * @param [in] pForceGravity 单元重力指针
     * @param [in,out] F1 约束自由度对应的力向量（累加）
     * @param [in,out] F2 自由自由度对应的力向量（累加）
     * @param [in] current_time 当前时间
     * @param [in] loadScale 荷载缩放系数
     */
    void Assemble_ForceGravity(Force_Gravity* pForceGravity, VectorXd& F1, VectorXd& F2, double& current_time, double loadScale);

    /**
     * @brief 组装风荷载
     * @param [in] pForceWind 单元风荷载指针
     * @param [in,out] F1 约束自由度对应的力向量（累加）
     * @param [in,out] F2 自由自由度对应的力向量（累加）
     * @param [in] current_time 当前时间
     * @param [in] loadScale 荷载缩放系数
     */
    void Assemble_ForceWind(Force_Wind* pForceWind, VectorXd& F1, VectorXd& F2, double& current_time, double loadScale);

    /**
     * @brief 组装约束位移
     * @param [out] x1 约束位移向量
     */
    void Assemble_Constraint(
        VectorXd& x1,
        double currentTime,
        double factor);

    /**
    * @brief 计算反力向量
    * @param [out] F1 约束自由度对应的反力向量
    */
    void CalculateReactions(VectorXd& F1);

    void Get_CurrentStepState(VectorXd& U, VectorXd& V, VectorXd& A) const;

    // ============ IAnalysisModel 接口辅助函数 ============
    VectorXd GetCurrentVelocity() const;
    VectorXd GetCurrentAcceleration() const;

    void BackupStepState() override;
    void GetStepIncrement(SolverNameSpace::Vec& dx_step) const override;
};
