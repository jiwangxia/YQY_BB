#pragma once
#include "Base/Base.h"
#include "Solver/Interface/IAnalysisModel.h"
#include "Solver/Interface/ISolver.h"
#include "Solver/Dynamic/AdaptiveTssbnSettings.h"
#include <memory>
#include <functional>
#include <set>
#include <Eigen/SparseLU>
typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> Tri;

class StructureData;
class Force_Node;
class Force_Element;
class Force_Gravity;
class Force_Wind;
class ElementBase;
struct AeroCaseKey;

enum class AnalysisRegionScope
{
    AllEnabledRegions,
    SelectedRegions
};

/**
 * @brief 分析步配置 - 描述一次分析任务的输入参数
 */
struct AnalysisStepConfig
{
    int id = 0;
    QString name;
    EnumKeyword::StepType type = EnumKeyword::StepType::UNKNOWN;
    double totalTime = 0.0;
    double stepSize = 0.0;
    double tolerance = 1e-5;
    int maxIterations = 32;
    SolverNameSpace::SolverType dynamicSolverType = SolverNameSpace::SolverType::Newmark;
    int initialStaticStepId = 0; ///< 动力步继承的静力平衡步；0 表示从原始状态开始
    SolverNameSpace::AdaptiveTssbnSettings adaptiveTssbn;
    bool enableGalloping = false;
    int gallopingIceThickness = 12;
    double gallopingInitialAttackDegrees = 45.0;
    AnalysisRegionScope regionScope = AnalysisRegionScope::AllEnabledRegions;
    std::set<int> computeRegionIds;
};

/**
 * @brief 分析步类 - 负责单次分析的完整流程
 * 
 * 实现 SolverNameSpace::IAnalysisModel 接口，可以使用独立的求解器进行求解
 */
class AnalysisStep : public Base, public SolverNameSpace::IAnalysisModel
{
public:
    QString m_Name;                 // 用户可读名称；为空时界面回退为 Step-ID
    EnumKeyword::StepType m_Type = EnumKeyword::StepType::UNKNOWN;
    double m_Time = 0.0;           // 总时间
    double m_StepSize = 0.0;       // 每步大小
    double m_Tolerance = 1e-5;     // 容差
    int m_MaxIterations = 32;      // 最大迭代次数

    bool isDynamic = false;        // 是否为动力分析
    /// @brief 动力求解器类型（仅动力分析时有效）
    /// 当前可用: Newmark、AdaptiveTSSBN；其他枚举值为预留。
    SolverNameSpace::SolverType m_DynamicSolverType = SolverNameSpace::SolverType::Newmark;
    int m_InitialStaticStepId = 0; ///< 动力步的前置静力平衡步
    SolverNameSpace::AdaptiveTssbnSettings m_AdaptiveTssbn;
    bool m_EnableGalloping = false;
    int m_GallopingIceThickness = 12;    // 气动数据离散工况，单位 mm
    double m_GallopingInitialAttackDegrees = 45.0;
    AnalysisRegionScope m_RegionScope = AnalysisRegionScope::AllEnabledRegions;
    std::set<int> m_ComputeRegionIds;

    int m_nFixed = 0;              // 约束自由度个数
    int m_nFree = 0;               // 自由自由度个数
    SpMat m_Keff22;

    /**
     * @brief 获取分析步类型名称
     * @return 类型名称字符串
     */
    QString GetTypeName() const { return EnumKeyword::MapStepType.key(m_Type, "UNKNOWN"); }
    AeroCaseKey GetGallopingAeroCase(int bundleCount, const Force_Wind& wind) const;
    bool ShouldAssembleGalloping(int bundleCount, const Force_Wind& wind) const;

    /// Returns whether a load or constraint introduced in sourceStepId belongs
    /// to this analysis branch.  A dynamic step with an initial static step
    /// inherits the static branch and its own definitions only; sibling
    /// dynamic steps are deliberately excluded.
    bool IsStepScopedDataActive(int sourceStepId) const;

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
     * @param [in] persistHdf5 是否由当前分析步直接持久化 H5；区域子任务传 false，由上层合并后统一写入
     */
    bool Solve(bool persistHdf5 = true);

    /// 仅供求解调度器设置：下一次求解从模型当前已提交状态开始。
    void SetInitializeFromCurrentState(bool enabled) { m_initializeFromCurrentState = enabled; }

    using ProgressCallback = std::function<void(double, const QString&)>;
    using CancelCallback = std::function<bool()>;
    void SetRuntimeCallbacks(ProgressCallback progressCallback, CancelCallback cancelCallback);
    void ClearRuntimeCallbacks();

    // ============ IAnalysisModel 接口实现 ============
    int  GetFreeDofs() const override { return m_nFree; }
    int  GetFixedDofs() const override { return m_nFixed; }
    void ApplyIncrement(const SolverNameSpace::Vec& dx) override;
    void BeginDynamicStep(double dt, double beta, double gamma) override;
    void ApplyDynamicCorrection(const SolverNameSpace::Vec& dx, double a0, double a1) override;
    void RollbackDynamicStep() override;
    void SetTrialKinematics(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a) override;
    void SetTssbnStageKinematics(
        int stageIndex,
        double timeStep,
        double firstStageTime,
        double secondStageTime,
        double secondStageDiagonalFraction,
        SolverNameSpace::Vec& velocity,
        SolverNameSpace::Vec& acceleration) override;
    void CorrectTssbnStepStates(
        double timeStep,
        double firstStageTime,
        double secondStageTime,
        double lastStageTime,
        double baseFirstWeight,
        double embeddedFirstWeight,
        double embeddedSecondWeight,
        double embeddedLastWeight,
        double lastStageFirstCoefficient,
        double lastStageSecondCoefficient,
        SolverNameSpace::Vec& baseIncrement,
        SolverNameSpace::Vec& baseVelocity,
        SolverNameSpace::Vec& embeddedIncrement,
        SolverNameSpace::Vec& embeddedVelocity,
        SolverNameSpace::Vec& acceptedAcceleration) override;
    void GetState(SolverNameSpace::Vec& u, SolverNameSpace::Vec& v, SolverNameSpace::Vec& a) const override;
    void Assemble_Matrix(SpMat& Keff, bool isDynamic);          //组装整体等效刚度矩阵
    void AssembleDynamicSystem(
        SpMat& mass, SpMat& gyroscopic, SpMat& centrifugal) override;
    void AssembleEffectiveDynamicSystem(
        double accelerationDerivative,
        double velocityDerivative,
        SpMat& effectiveDynamicTangent) override;
    bool AssembleNonlinearMPC(
        SolverNameSpace::NonlinearMPCData& constraints) override;
    void SetNonlinearMPCMultipliers(
        const SolverNameSpace::Vec& multipliers) override;
    void ComputeResidual(const SolverNameSpace::Vec& F_ext, SolverNameSpace::Vec& R) override;
    void ComputeStaticResidual(
        const SolverNameSpace::Vec& F_ext,
        SolverNameSpace::Vec& R) override;
    void OnStepCompleted(double time) override;
    void RecordStepIterations(double time, int iterations) override;
    void CommitState() override;
    bool IsCancellationRequested() const override;
    void ReportProgress(double progress, const QString& message = QString()) override;

private:
    bool m_initializeFromCurrentState = false;
    std::weak_ptr<StructureData> m_pStructure;  // 结构数据的弱引用
    StructureData* m_pData = nullptr;           // 结构数据的缓存指针
    ProgressCallback m_progressCallback;
    CancelCallback m_cancelCallback;
    SolverNameSpace::Vec m_mpcMultipliers;
    SolverNameSpace::Vec m_dynamicInertiaForce;

    struct DynamicElementData
    {
        std::vector<int> dofs;
        Eigen::MatrixXd mass;
        Eigen::VectorXd inertiaForce;
        Eigen::MatrixXd velocityTangent;
        Eigen::MatrixXd configurationTangent;
    };

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
    bool ValidateGallopingConfiguration(QString* errorMessage) const;
    bool PrepareGallopingData(QString* errorMessage);
    Eigen::Vector3d GetModelUpDirection() const;

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
     * @param [in,out] freeFree 自由-自由块的三元组列表
     */
    void AssembleFreeFree(
        const std::vector<int>& dofs,
        const Eigen::MatrixXd& elementMatrix,
        std::vector<Tri>& freeFree);

    void EvaluateDynamicElement(
        ElementBase& element,
        DynamicElementData& result);
    void AccumulateDynamicInertiaForce(
        const DynamicElementData& elementData);

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
