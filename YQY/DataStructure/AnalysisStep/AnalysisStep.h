#pragma once
#include "Base/Base.h"
#include "DataStructure/Load/LoadAssembler.h"
#include "Solver/Interface/IAnalysisModel.h"
#include "Solver/Interface/ISolver.h"
#include "Solver/Dynamic/TimeStepping/AdaptiveTssbnSettings.h"
#include "Solver/Dynamic/TimeStepping/TimeStepControl.h"
#include "Solver/Dynamic/Damping/StructuralDamping.h"
#include "Solver/Assembly/ParallelAssemblyEngine.h"
#include "Solver/Assembly/Gpu/CudaBeamBatchEvaluator.h"
#include "Solver/Assembly/Gpu/CudaCableBatchEvaluator.h"
#include "Solver/Assembly/Gpu/CudaTrussBatchEvaluator.h"
#include "Solver/Linear/Gpu/CudssDeviceTransfer.h"
#include <QByteArray>
#include <QFuture>
#include <array>
#include <exception>
#include <memory>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <utility>
#include <vector>
typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> Tri;

class StructureData;
class Force_Wind;
class ElementBase;
class ElementBeam_CR;
class ElementCable;
class ElementTruss;
class SectionBase;
class RigidBodyInertia;
class Node;
struct AeroCaseKey;
struct EnergyResult;

enum class AnalysisRegionScope
{
    AllEnabledRegions,
    SelectedRegions
};

enum class GallopingProfileBindingMode
{
    AutoGeometry = 0,           ///< 按物理子导线编号、风向及档距几何选列，旧模型默认行为
    ExplicitElementProfile = 1 ///< 直接使用单元 AERO_PROFILE_ID；WIRE_ID 仍为物理线编号
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
    SolverNameSpace::AdaptiveTimeStepSettings adaptiveTimeStep;
    SolverNameSpace::AdaptiveTssbnSettings adaptiveTssbn;
    SolverNameSpace::StructuralDampingSettings structuralDamping;
    bool enableGalloping = false;
    GallopingProfileBindingMode gallopingProfileBindingMode = GallopingProfileBindingMode::AutoGeometry;
    SolverNameSpace::AerodynamicTangentMode gallopingAerodynamicTangentMode =
        SolverNameSpace::AerodynamicTangentMode::EveryNewtonIteration;
    int gallopingIceThickness = 12;
    double gallopingInitialAttackDegrees = 45.0;
    double gallopingInitialVerticalPerturbationMeters = 0.0;
    int gallopingInitialVerticalPerturbationWaves = 1;
    AnalysisRegionScope regionScope = AnalysisRegionScope::AllEnabledRegions;
    std::set<int> computeRegionIds;
};

/// @brief 单个分析步的累计性能统计，所有时间单位为毫秒。
struct AnalysisPerformanceStatistics
{
    double totalMilliseconds = 0.0;
    double systemEvaluationMilliseconds = 0.0;
    double systemEvaluationExclusiveMilliseconds = 0.0;
    double effectiveTangentMilliseconds = 0.0;
    double dynamicSystemMilliseconds = 0.0;
    double massMatrixMilliseconds = 0.0;
    double inertiaForceMilliseconds = 0.0;
    double externalTangentMilliseconds = 0.0;
    double gallopingForceAssemblyMilliseconds = 0.0;
    double gallopingElementLoopMilliseconds = 0.0;
    double residualStateMilliseconds = 0.0;
    double referenceCableEvaluationMilliseconds = 0.0;
    double referenceCablePreparationMilliseconds = 0.0;
    double referenceCableMergeMilliseconds = 0.0;
    double elementBatchPlanMilliseconds = 0.0;
    double assemblyRangePlanMilliseconds = 0.0;
    double residualMilliseconds = 0.0;
    double outputMilliseconds = 0.0;
    double gpuTrussUploadMilliseconds = 0.0;
    double gpuTrussKernelMilliseconds = 0.0;
    double gpuTrussDownloadMilliseconds = 0.0;
    double gpuCableUploadMilliseconds = 0.0;
    double gpuCableKernelMilliseconds = 0.0;
    double gpuCableDownloadMilliseconds = 0.0;
    double gpuCableAssemblyDownloadMilliseconds = 0.0;
    double gpuBeamUploadMilliseconds = 0.0;
    double gpuBeamKernelMilliseconds = 0.0;
    double gpuBeamDownloadMilliseconds = 0.0;
    double gpuBeamAssemblyDownloadMilliseconds = 0.0;
    int systemEvaluationCount = 0;
    int effectiveTangentCount = 0;
    int dynamicSystemCount = 0;
    int massMatrixCount = 0;
    int inertiaForceCount = 0;
    int externalTangentCount = 0;
    int gallopingForceAssemblyCount = 0;
    long long gallopingElementEvaluations = 0;
    int residualStateCount = 0;
    int referenceCableEvaluationCount = 0;
    int referenceCableWorkerCountMaximum = 0;
    long long referenceCableElementEvaluations = 0;
    int elementBatchPlanBuildCount = 0;
    int assemblyRangePlanBuildCount = 0;
    int residualCount = 0;
    int outputCount = 0;
    int gpuTrussBatchCount = 0;
    int gpuCableBatchCount = 0;
    int gpuCableTopologyBuildCount = 0;
    int gpuBeamBatchCount = 0;
    int gpuEffectiveTangentCount = 0;
    int gpuResidualStateCount = 0;
};

/**
 * @brief 分析步类 - 负责单次分析的完整流程
 * 
 * 实现 SolverNameSpace::IAnalysisModel 接口，可以使用独立的求解器进行求解
 */
class AnalysisStep : public Base, public SolverNameSpace::IAnalysisModel
{
public:
    QString m_Name; // 用户可读名称；为空时界面回退为 Step-ID
    EnumKeyword::StepType m_Type = EnumKeyword::StepType::UNKNOWN;
    double m_Time = 0.0;       // 总时间
    double m_StepSize = 0.0;   // 每步大小
    double m_Tolerance = 1e-5; // 容差
    int m_MaxIterations = 32;  // 最大迭代次数

    bool isDynamic = false; // 是否为动力分析
    /// @brief 动力求解器类型（仅动力分析时有效）
    /// 当前可用：Newmark、自适应 Newmark、RK4、自适应 RK4 和自适应 TSSBN。
    SolverNameSpace::SolverType m_DynamicSolverType = SolverNameSpace::SolverType::Newmark;
    int m_InitialStaticStepId = 0; ///< 动力步的前置静力平衡步
    SolverNameSpace::AdaptiveTimeStepSettings m_AdaptiveTimeStep;
    SolverNameSpace::AdaptiveTssbnSettings m_AdaptiveTssbn;
    SolverNameSpace::StructuralDampingModel m_StructuralDamping;
    bool m_EnableGalloping = false;
    GallopingProfileBindingMode m_GallopingProfileBindingMode = GallopingProfileBindingMode::AutoGeometry;
    SolverNameSpace::AerodynamicTangentMode m_GallopingAerodynamicTangentMode =
        SolverNameSpace::AerodynamicTangentMode::EveryNewtonIteration;
    bool m_DiagnosticTssbnCacheExternalForcePerStage = false; // 验证专用，不写入模型。
    bool m_DiagnosticTssbnErrorTrace = false; // 验证专用，不写入模型。
    bool m_DiagnosticProfileGallopingForce = false; // 验证专用，不写入模型。
    int m_DiagnosticReferenceParallelMinimumCableElements = 0; // 验证专用，不写入模型。
    int m_GallopingIceThickness = 12; // 气动数据离散工况，单位 mm
    double m_GallopingInitialAttackDegrees = 45.0;
    double m_GallopingInitialVerticalPerturbationMeters = 0.0;
    int m_GallopingInitialVerticalPerturbationWaves = 1;
    AnalysisRegionScope m_RegionScope = AnalysisRegionScope::AllEnabledRegions;
    std::set<int> m_ComputeRegionIds;

    int m_nFixed = 0; // 约束自由度个数
    int m_nFree = 0;  // 自由自由度个数
    SpMat m_Keff22;

    /**
     * @brief 获取分析步类型名称
     * @return 类型名称字符串
     */
    QString GetTypeName() const
    {
        return EnumKeyword::MapStepType.key(m_Type, "UNKNOWN");
    }
    AeroCaseKey GetGallopingAeroCase(int bundleCount, const Force_Wind& wind) const;
    bool ShouldAssembleGalloping(int bundleCount, const Force_Wind& wind) const;

    /// 判断 sourceStepId 引入的荷载或约束是否属于当前分析分支。
    /// 动力步继承其前置静力步与自身定义，不继承同级动力步的定义。
    bool IsStepScopedDataActive(int sourceStepId) const;

    /**
     * @brief 设置关联的结构数据
     * @param [in] pStructure 结构数据的共享指针
     */
    void SetStructure(std::shared_ptr<StructureData> pStructure);
    bool PreloadGallopingDataForSnapshot(const std::shared_ptr<StructureData>& structure,
                                        QString* errorMessage);

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
    void SetInitializeFromCurrentState(bool enabled)
    {
        m_initializeFromCurrentState = enabled;
    }

    // 仅用于参考路径的独立前置静力副本，动力阶段恢复正常风荷载。
    void SetReferenceGravityEquilibrium(bool enabled)
    {
        m_referenceGravityEquilibrium = enabled;
    }

    bool IsReferenceGravityEquilibrium() const
    {
        return m_referenceGravityEquilibrium;
    }

    // 验证 CLI 专用：在自适应 TSSBN 已提交的接受步边界保存或恢复完整动力状态。
    struct TssbnCheckpointOptions
    {
        QString savePath;
        QString resumePath;
        QByteArray caseSignature;
        double targetTime = 0.0;
        bool stopAfterSave = false;
    };
    void SetTssbnCheckpointOptions(const TssbnCheckpointOptions& options)
    {
        m_tssbnCheckpointOptions = options;
    }
    bool WasTssbnStoppedAtCheckpoint() const
    {
        return m_tssbnStoppedAtCheckpoint;
    }
    double LastTssbnAverageTimeStep() const
    {
        return m_tssbnAverageTimeStep;
    }
    int LastTssbnAcceptedStepCount() const
    {
        return m_tssbnAcceptedStepCount;
    }

    /// 仅供区域并发调度器使用：限制本分析步调用装配器的工作线程数。
    /// 传入 0 时恢复全局装配线程设置。
    void SetAssemblyThreadCountOverride(int count)
    {
        m_assemblyThreadCountOverride = count > 0 ? count : 0;
    }

    /// 仅供区域并发调度器使用：限制当前分析线程内 PARDISO 的线程数。
    /// 传入 0 时保持 oneMKL 的默认设置。
    void SetPardisoThreadCountOverride(int count)
    {
        m_pardisoThreadCountOverride = count > 0 ? count : 0;
    }

    using ProgressCallback = std::function<void(double, const QString&)>;
    using CancelCallback = std::function<bool()>;
    void SetRuntimeCallbacks(ProgressCallback progressCallback, CancelCallback cancelCallback);
    void ClearRuntimeCallbacks();

    // 继承状态的初扰在气动注册与阻尼标定之后、初始加速度恢复之前施加。
    using InitialDynamicStateCallback = std::function<bool(QString&)>;
    void SetInitialDynamicStateCallback(InitialDynamicStateCallback callback)
    {
        m_initialDynamicStateCallback = std::move(callback);
    }
    void ClearInitialDynamicStateCallback()
    {
        m_initialDynamicStateCallback = {};
    }

    QString LastFailureReason() const
    {
        return m_lastFailureReason;
    }

    const AnalysisPerformanceStatistics& GetPerformanceStatistics() const
    {
        return m_performanceStatistics;
    }
    void SetLastFailureReason(const QString& reason)
    {
        m_lastFailureReason = reason;
    }

    // ============ IAnalysisModel 接口实现 ============
    int GetFreeDofs() const override
    {
        return m_nFree;
    }
    int GetFixedDofs() const override
    {
        return m_nFixed;
    }
    bool ShouldPreferGpuLinearSolver() const override;
    std::uint64_t GetSparsePatternVersion() const override;
    bool HasStateDependentMass() const override;
    QString DescribeFreeDof(int freeDof) const override;
    void ApplyIncrement(const SolverNameSpace::Vec& dx) override;
    void BeginDynamicStep(double dt, double beta, double gamma) override;
    void ApplyDynamicCorrection(const SolverNameSpace::Vec& dx, double a0, double a1) override;
    void RollbackDynamicStep() override;
    bool SaveTrialState() override;
    bool RestoreTrialState() override;
    bool PushStateCheckpoint() override;
    bool RestoreStateCheckpoint() override;
    void DiscardStateCheckpoint() override;
    void SetTrialKinematics(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a) override;
    void SetTssbnTrialKinematics(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a) override;
    void BeginTssbnImplicitStage(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                 double secondStageDiagonalFraction) override;
    void ApplyTssbnImplicitStageCorrection(const SolverNameSpace::Vec& increment) override;
    bool NormalizeTssbnAcceptedRotations() override;
    void SetTssbnStageKinematics(int stageIndex, double timeStep, double firstStageTime, double secondStageTime,
                                 double secondStageDiagonalFraction, _OUT SolverNameSpace::Vec& velocity,
                                 _OUT SolverNameSpace::Vec& acceleration, bool useStoredIncrement = false) override;
    void CorrectTssbnExplicitLastStage(double timeStep, double lastStageFirstCoefficient,
                                       double lastStageSecondCoefficient, _OUT SolverNameSpace::Vec& stepIncrement,
                                       _OUT SolverNameSpace::Vec& velocity) override;
    void CorrectTssbnStepStates(double timeStep, double firstStageTime, double secondStageTime, double lastStageTime,
                                double baseFirstWeight, double embeddedFirstWeight, double embeddedSecondWeight,
                                double embeddedLastWeight, double lastStageFirstCoefficient,
                                double lastStageSecondCoefficient, _OUT SolverNameSpace::Vec& baseIncrement,
                                _OUT SolverNameSpace::Vec& baseVelocity, _OUT SolverNameSpace::Vec& embeddedIncrement,
                                _OUT SolverNameSpace::Vec& embeddedVelocity,
                                _OUT SolverNameSpace::Vec& acceptedAcceleration) override;
    bool CorrectReferenceTssbnErrorDisplacements(double timeStep, double baseFirstWeight, double embeddedFirstWeight,
                                                 double embeddedSecondWeight, double embeddedLastWeight,
                                                 double lastStageFirstCoefficient, double lastStageSecondCoefficient,
                                                 _OUT SolverNameSpace::Vec& stepStartDisplacement,
                                                 _OUT SolverNameSpace::Vec& baseDisplacement,
                                                 _OUT SolverNameSpace::Vec& embeddedDisplacement) const override;
    void CorrectTssbnEmbeddedVelocity(double timeStep, double embeddedFirstWeight, double embeddedSecondWeight,
                                      double embeddedLastWeight, double lastStageFirstCoefficient,
                                      double lastStageSecondCoefficient,
                                      const SolverNameSpace::Vec& explicitLastStageAcceleration,
                                      _OUT SolverNameSpace::Vec& embeddedVelocity) override;
    void GetState(_OUT SolverNameSpace::Vec& u, _OUT SolverNameSpace::Vec& v,
                  _OUT SolverNameSpace::Vec& a) const override;
    void GetFiniteRotationStates(_OUT std::vector<SolverNameSpace::FiniteRotationState>& states) const override;
    bool SupportsTssbnRotationPrecisionEstimate() const override
    {
        return true;
    }
    void GetAuxiliaryFiniteRotationStates(_OUT std::vector<Eigen::Matrix3d>& states) const override;
    void EvaluateSystem(const SolverNameSpace::SystemEvaluationRequest& request,
                        _OUT SolverNameSpace::SystemEvaluationResult& result) override;
    void Assemble_Matrix(_OUT SpMat& Keff, bool isDynamic); //组装整体等效刚度矩阵
    void AssembleDynamicSystem(_OUT SpMat& mass, _OUT SpMat& gyroscopic, _OUT SpMat& centrifugal) override;
    void AssembleMassMatrix(_OUT SpMat& mass) override;
    void AssembleEffectiveDynamicSystem(double accelerationDerivative, double velocityDerivative,
                                        _OUT SpMat& effectiveDynamicTangent) override;
    void AssembleEffectiveTangent(double accelerationDerivative, double velocityDerivative,
                                  _OUT SpMat& effectiveTangent) override;
    void UpdateInternalForce() override;
    void RefreshResidualState() override;
    void RefreshInertiaForce() override;
    void GetDynamicInertiaForce(_OUT SolverNameSpace::Vec& inertiaForce) const override;
    void AssembleExternalLoadTangent(double time, double loadFactor, double velocityDerivative,
                                     _OUT SpMat& externalTangent) override;
    bool AssembleNonlinearMPC(_OUT SolverNameSpace::NonlinearMPCData& constraints) override;
    bool HasActiveNonlinearMPC() const override;
    void SetNonlinearMPCMultipliers(const SolverNameSpace::Vec& multipliers) override;
    void ComputeResidual(const SolverNameSpace::Vec& F_ext, _OUT SolverNameSpace::Vec& R) override;
    void GetReferenceConvergenceResidual(const SolverNameSpace::Vec& freeResidual,
                                         const SolverNameSpace::Vec& constrainedExternalForce,
                                         _OUT SolverNameSpace::Vec& residual) const override;
    double GetReferenceVectorNorm(const SolverNameSpace::Vec& vector) const override;
    bool GetReferenceLinearSolvePermutation(_OUT std::vector<int>& signedSource) const override;
    void ComputeStaticResidual(const SolverNameSpace::Vec& F_ext, _OUT SolverNameSpace::Vec& R) override;
    void OnStepCompleted(double time) override;
    void RecordStepIterations(double time, int iterations) override;
    void CommitState() override;
    bool IsCancellationRequested() const override;
    void ReportProgress(double progress, const QString& message = QString()) override;

    // 验证历程仅在接受步提交后读取，使用本分析步实际解析的气动列绑定。
    bool SampleGallopingElementAero(int elementId, double time,
                                   _OUT LoadAssembler::GallopingElementAeroSample& sample) const;
    int GetGallopingBoundProfileId(int elementId) const
    {
        const auto selected = m_gallopingProfileBindings.find(elementId);
        return selected != m_gallopingProfileBindings.cend() ? selected->second : -1;
    }
    double SampleStructuralDampingPower();
    // 接受态的四项能量；不装配整体矩阵，不累计功或保存额外历史。
    bool EvaluateEnergy(double time, _OUT EnergyResult& energy);
    /** @brief 当前分析步固定的单元局部 Rayleigh 系数，顺序为平动质量、平动刚度、自扭质量、自扭刚度。 */
    std::array<double, 4> GetElementRayleighCoefficients(const ElementBase& element) const;
    /** @brief 当前分析步固定的刚体惯性局部 Rayleigh 系数。 */
    std::array<double, 4> GetRigidBodyRayleighCoefficients(const RigidBodyInertia& inertia) const;

private:
    void SetTrialKinematicsImpl(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a,
                                bool preserveStepRotation);
    bool SolveUnchecked(bool persistHdf5);
    InitialDynamicStateCallback m_initialDynamicStateCallback;
    bool m_initializeFromCurrentState = false;
    bool m_referenceGravityEquilibrium = false;
    std::vector<int> m_referenceNormSignedDofs;
    std::optional<TssbnCheckpointOptions> m_tssbnCheckpointOptions;
    bool m_tssbnStoppedAtCheckpoint = false;
    double m_tssbnAverageTimeStep = 0.0;
    int m_tssbnAcceptedStepCount = 0;
    int m_assemblyThreadCountOverride = 0;
    int m_pardisoThreadCountOverride = 0;
    std::weak_ptr<StructureData> m_pStructure; // 结构数据的弱引用
    StructureData* m_pData = nullptr;          // 结构数据的缓存指针
    ProgressCallback m_progressCallback;
    CancelCallback m_cancelCallback;
    QString m_lastFailureReason;
    bool m_dynamicReactionSupportChecked = false;
    QString m_dynamicReactionUnsupportedReason;
    QString m_lastProgressMessage;
    AnalysisPerformanceStatistics m_performanceStatistics;
    SolverNameSpace::Vec m_mpcMultipliers;
    SolverNameSpace::Vec m_dynamicInertiaForce;
    // 原生四自由度参考路径分别保留外载，按参考次序形成完整残差。
    std::uint64_t m_referenceRhsStateVersion = 1;
    std::uint64_t m_referenceLoadStateVersion = 0;
    SolverNameSpace::Vec m_referenceBaseExternalForce;
    SolverNameSpace::Vec m_referenceAerodynamicForce;
    SolverNameSpace::Vec m_referenceTotalExternalForce;
    SolverNameSpace::Vec m_referenceOrderedResidual;
    bool m_localRayleighDampingReady = false;
    SpMat m_localRayleighAssembledCache;
    std::array<double, 4> m_localRayleighCacheCoefficients{};
    std::vector<SolverNameSpace::LocalRayleighGroupReport> m_localRayleighCacheGroups;
    std::vector<QString> m_localRayleighGroupLabels;
    std::map<int, std::size_t> m_localRayleighElementGroups;
    std::map<int, std::size_t> m_localRayleighInertiaGroups;
    bool m_localRayleighCacheValid = false;
    std::map<int, std::shared_ptr<Node>> m_trialNodeStates;
    struct StateCheckpoint
    {
        std::map<int, std::shared_ptr<Node>> nodes;
        SolverNameSpace::Vec mpcMultipliers;
    };
    std::optional<StateCheckpoint> m_stateCheckpoint;
    // 根据当前分析步风向解析导线标签后固定，仅攻角行随时间积分变化。
    std::map<int, int> m_gallopingProfileBindings;

    struct DynamicElementData
    {
        Eigen::MatrixXd mass;
        Eigen::VectorXd inertiaForce;
        Eigen::MatrixXd velocityTangent;
        Eigen::MatrixXd configurationTangent;
    };

    struct ElementAssemblyMap
    {
        std::shared_ptr<ElementBase> element;
        std::vector<int> dofs;
        std::vector<Eigen::Index> valueIndices;
        mutable Eigen::VectorXd referenceInertiaForce;
        mutable std::uint64_t referenceInertiaStateVersion = 0;
    };

    struct ReferenceCableEvaluationSlot
    {
        std::shared_ptr<ElementCable> cable;
        std::shared_ptr<SectionBase> section;
        std::size_t sectionIndex = 0;
        Eigen::MatrixXd stiffness;
        DynamicElementData dynamicData;
    };

    struct ReferenceCableSectionSnapshot
    {
        std::shared_ptr<SectionBase> section;
        std::array<double, 3> inertia{};
    };

    struct RigidBodyInertiaAssemblyMap
    {
        std::shared_ptr<RigidBodyInertia> inertia;
        std::vector<int> dofs;
        std::vector<Eigen::Index> valueIndices;
        mutable Eigen::VectorXd referenceInertiaForce;
        mutable std::uint64_t referenceInertiaStateVersion = 0;
    };

    struct SparseAssemblyPattern
    {
        SpMat matrix;
        std::vector<ElementAssemblyMap> elements;
        std::vector<RigidBodyInertiaAssemblyMap> rigidBodyInertias;
        std::vector<std::vector<std::size_t>> elementColorGroups;
        std::size_t matrixEntryCapacity = 0;
        bool valid = false;
    };

    struct PreparedPatternStamp
    {
        const SpMat::StorageIndex* outerIndices = nullptr;
        const SpMat::StorageIndex* innerIndices = nullptr;
        const double* values = nullptr;
        Eigen::Index rows = 0;
        Eigen::Index columns = 0;
        Eigen::Index nonZeros = 0;
        std::uint64_t version = 0;
    };

    struct IndexedAssemblyValue
    {
        Eigen::Index index = -1;
        double value = 0.0;
    };

    struct AssemblyWorkerBuffer
    {
        // 仅保存本线程实际产生的矩阵项，避免每个线程复制整份全局稀疏矩阵。
        std::vector<IndexedAssemblyValue> matrixValues;
        Eigen::VectorXd inertiaForce;
    };

    // 单元批次只描述调度归属，不改变任何单元本身的数值公式。
    struct ElementBatchPlan
    {
        std::vector<std::size_t> trussElementIndices;
        std::vector<std::size_t> cableElementIndices;
        std::vector<std::size_t> beamElementIndices;
        std::vector<std::size_t> otherElementIndices;
    };

    struct DynamicAssemblyWorkerBuffer
    {
        std::vector<IndexedAssemblyValue> massValues;
        std::vector<IndexedAssemblyValue> velocityValues;
        std::vector<IndexedAssemblyValue> configurationValues;
        Eigen::VectorXd inertiaForce;
    };

    SparseAssemblyPattern m_assemblyPattern;
    std::uint64_t m_sparsePatternVersion = 0;
    mutable std::map<const SpMat*, PreparedPatternStamp> m_preparedPatternStamps;
    ElementBatchPlan m_elementBatchPlan;
    std::map<int, std::vector<SolverNameSpace::ParallelWorkRange>> m_assemblyRangesByWorkerCount;
    std::vector<bool> m_gpuAssemblyElementMask;
    std::vector<double> m_gpuMatrixValueScratch;
    std::vector<AssemblyWorkerBuffer> m_assemblyWorkerBuffers;
    std::vector<DynamicAssemblyWorkerBuffer> m_dynamicAssemblyWorkerBuffers;
    std::vector<ReferenceCableEvaluationSlot> m_referenceCableEvaluationSlots;
    std::vector<ReferenceCableSectionSnapshot> m_referenceCableSectionSnapshots;
    std::vector<std::exception_ptr> m_referenceCableWorkerExceptions;
    std::vector<std::size_t> m_referenceCableWorkerElementCounts;
    std::vector<QFuture<void>> m_referenceCableWorkerFutures;
    std::uint64_t m_referenceCableWorkspacePatternVersion = 0;
    SolverNameSpace::ParallelAssemblyEngine m_parallelAssemblyEngine;
    SolverNameSpace::CudaTrussBatchEvaluator m_gpuTrussBatchEvaluator;
    SolverNameSpace::CudaCableBatchEvaluator m_gpuCableBatchEvaluator;
    SolverNameSpace::CudaSharedCableBatchEvaluator m_gpuSharedCableBatchEvaluator;
    SolverNameSpace::CudaBeamBatchEvaluator m_gpuBeamBatchEvaluator;
    SolverNameSpace::DeviceCscSum m_gpuMixedCscSum;
    std::vector<std::size_t> m_gpuTrussBatchElementIndices;
    std::vector<std::shared_ptr<ElementTruss>> m_gpuTrussBatchElements;
    std::vector<std::shared_ptr<Node>> m_gpuTrussBatchFirstNodes;
    std::vector<std::shared_ptr<Node>> m_gpuTrussBatchSecondNodes;
    std::vector<SolverNameSpace::GpuTrussElementInput> m_gpuTrussBatchInput;
    std::vector<SolverNameSpace::GpuTrussAssemblyMap> m_gpuTrussBatchAssemblyMaps;
    std::vector<std::size_t> m_gpuCableBatchSourceElementIndices;
    std::vector<std::size_t> m_gpuCableBatchElementIndices;
    std::vector<std::shared_ptr<ElementCable>> m_gpuCableBatchElements;
    std::vector<std::shared_ptr<Node>> m_gpuCableBatchFirstNodes;
    std::vector<std::shared_ptr<Node>> m_gpuCableBatchSecondNodes;
    std::vector<SolverNameSpace::GpuCableElementInput> m_gpuCableBatchInput;
    std::vector<SolverNameSpace::GpuCableAssemblyMap> m_gpuCableBatchAssemblyMaps;
    std::vector<std::size_t> m_gpuSharedCableBatchSourceElementIndices;
    std::vector<std::size_t> m_gpuSharedCableBatchElementIndices;
    std::vector<std::shared_ptr<ElementCable>> m_gpuSharedCableBatchElements;
    std::vector<std::shared_ptr<Node>> m_gpuSharedCableBatchFirstNodes;
    std::vector<std::shared_ptr<Node>> m_gpuSharedCableBatchSecondNodes;
    std::vector<SolverNameSpace::GpuSharedCableElementInput> m_gpuSharedCableBatchInput;
    std::vector<SolverNameSpace::GpuSharedCableAssemblyMap> m_gpuSharedCableBatchAssemblyMaps;
    std::vector<std::size_t> m_gpuBeamBatchElementIndices;
    std::vector<std::shared_ptr<ElementBeam_CR>> m_gpuBeamBatchElements;
    std::vector<std::shared_ptr<Node>> m_gpuBeamBatchFirstNodes;
    std::vector<std::shared_ptr<Node>> m_gpuBeamBatchSecondNodes;
    std::vector<SolverNameSpace::GpuBeamElementInput> m_gpuBeamBatchInput;
    std::vector<SolverNameSpace::GpuBeamAssemblyMap> m_gpuBeamBatchAssemblyMaps;
    bool m_gpuTrussCscValuesReady = false;
    bool m_gpuTrussCscValuesComplete = false;
    bool m_gpuCableCscValuesReady = false;
    bool m_gpuCableCscValuesComplete = false;
    bool m_gpuBeamCscValuesReady = false;
    bool m_gpuBeamCscValuesComplete = false;
    bool m_gpuMixedCscValuesReady = false;
    std::uint64_t m_gpuAdaptivePatternVersion = 0;
    int m_gpuAdaptiveWarmupCount = 0;
    bool m_gpuAdaptiveUseCpu = false;

    /**
     * @brief 准备数据，缓存结构指针
     * @return 成功返回 true，失败返回 false
     */
    bool PrepareData();
    bool ValidateSolveConfiguration(_OUT QString& errorMessage) const;
    bool ValidateDynamicPrescribedConstraints(_OUT QString& errorMessage) const;
    bool ValidateGallopingConfiguration(QString* errorMessage) const;
    bool PrepareGallopingData(QString* errorMessage);
    bool BindGallopingProfiles(QString* errorMessage);
    Eigen::Vector3d GetModelUpDirection() const;
    bool PrepareReferenceVectorNormMapping(_OUT QString& reason);

    /**
     * @brief 初始化自由度编号
     */
    void Init_DOF();

    /**
    * @brief 初始化节点内部变量数据
    */
    void Init_Nodevector();

    void Get_ElementLength();
    void BuildElementBatchPlan();
    bool TryAssembleAcceleratedEffectiveTangent(const ElementBatchPlan& batchPlan, double accelerationDerivative,
                                                double velocityDerivative, _OUT SpMat& effectiveTangent);
    bool TryAssembleGpuTrussEffectiveTangent(const ElementBatchPlan& batchPlan, double accelerationDerivative,
                                             bool matrixPrepared, bool copyMatrix,
                                             _OUT std::vector<bool>& gpuElements,
                                             _OUT SpMat& effectiveTangent);
    bool TryAssembleGpuCableEffectiveTangent(const ElementBatchPlan& batchPlan, double accelerationDerivative,
                                             bool matrixPrepared, bool copyMatrix,
                                             _OUT std::vector<bool>& gpuElements,
                                             _OUT SpMat& effectiveTangent);
    bool TryAssembleGpuSharedCableEffectiveTangent(const ElementBatchPlan& batchPlan, double accelerationDerivative,
                                                   bool matrixPrepared, bool copyMatrix,
                                                   _OUT std::vector<bool>& gpuElements,
                                                   _OUT SpMat& effectiveTangent);
    bool TryAssembleGpuBeamEffectiveTangent(const ElementBatchPlan& batchPlan, double accelerationDerivative,
                                            double velocityDerivative, bool matrixPrepared, bool copyMatrix,
                                            _OUT std::vector<bool>& gpuElements, _OUT SpMat& effectiveTangent);
    bool TryRefreshAcceleratedResidualState(const ElementBatchPlan& batchPlan);
    void AssembleCpuEffectiveTangentRemainder(const std::vector<bool>& gpuElements, double accelerationDerivative,
                                              double velocityDerivative, _OUT SpMat& effectiveTangent);
    void RefreshResidualStateImpl();
    void RefreshCpuResidualStateRemainder(const std::vector<bool>& gpuElements);
    void RefreshCpuResidualStateAndMass(_OUT SpMat& mass);
    bool HasActiveLocalRayleighDamping() const;
    std::array<double, 4> GetLocalRayleighCoefficients() const;
    std::array<double, 4> GetLocalRayleighGroupCoefficients(std::size_t groupIndex) const;
    bool BuildLocalRayleighGroupBasis(_OUT std::vector<SolverNameSpace::LocalRayleighGroupBasis>& groups,
                                      _OUT QString& errorMessage);
    QString LocalRayleighElementGroupLabel(const ElementBase& element, int zone) const;
    std::size_t AddLocalRayleighGroup(const QString& label,
                                      _OUT std::vector<SolverNameSpace::LocalRayleighGroupBasis>& groups);
    void BuildRigidBodyRayleighDamping(const RigidBodyInertia& inertia, const std::array<double, 4>& coefficients,
                                       _OUT Eigen::MatrixXd& damping) const;
    void BuildLocalRayleighDamping(const std::array<double, 4>& coefficients, _OUT SpMat& damping,
                                   bool useGroupCoefficients = false);
    const SpMat& GetLocalRayleighDamping();
    double ComputeLocalRayleighForce(_OUT Eigen::VectorXd& force);
    void ComputeLocalRayleighFixedForce(_OUT Eigen::VectorXd& fixedForce) const;
    void AddLocalRayleighTangent(double velocityDerivative, _OUT SpMat& tangent);
    void AssembleEffectiveTangentImpl(double accelerationDerivative, double velocityDerivative,
                                      _OUT SpMat& effectiveTangent);
    void AssembleEffectiveDynamicSystemImpl(double accelerationDerivative, double velocityDerivative,
                                            _OUT SpMat& effectiveDynamicTangent);
    void InvalidateLocalRayleighCache();
    bool UsesReferenceRhsOrder() const;
    void InvalidateReferenceRhsCache();
    bool ComputeReferenceOrderedResidual(const SolverNameSpace::Vec& externalForce,
                                         _OUT SolverNameSpace::Vec& residual);
    void CacheReferenceInertia(const ElementAssemblyMap& assemblyMap, const Eigen::VectorXd& inertiaForce) const;
    void CacheReferenceInertia(const RigidBodyInertiaAssemblyMap& assemblyMap,
                                const Eigen::VectorXd& inertiaForce) const;
    void AssembleLocalRayleighDamping(_OUT SpMat& damping);

    /**
     * @brief 将单元刚度矩阵组装到整体刚度矩阵
     * @param [in] DOFs 单元自由度编号数组
     * @param [in] T 单元刚度矩阵
     * @param [in,out] freeFree 自由-自由块的三元组列表
     */
    void AssembleFreeFree(const std::vector<int>& dofs, const Eigen::MatrixXd& elementMatrix,
                          _OUT std::vector<Tri>& freeFree);
    void EnsureAssemblyPattern();
    void PreparePatternMatrix(_OUT SpMat& matrix) const;
    void BuildAssemblyColorGroups();
    void PrepareAssemblyWorkerBuffers(int workerCount, bool includeInertiaForce);
    void PrepareResidualWorkerBuffers(int workerCount);
    void PrepareDynamicAssemblyWorkerBuffers(int workerCount);
    int SelectAssemblyWorkerCount() const;
    const std::vector<SolverNameSpace::ParallelWorkRange>& GetAssemblyRanges(int workerCount);
    bool ShouldUseColoredAssembly(int workerCount) const;
    void PrepareReferenceCableEvaluationWorkspace();
    void ClearReferenceCableSectionSnapshots();
    bool TryAssembleReferenceCableEffectiveTangent(const SolverNameSpace::SystemEvaluationRequest& request,
                                                   _OUT SpMat& effectiveTangent);
    void AccumulateElementMatrix(const ElementAssemblyMap& assemblyMap, const Eigen::MatrixXd& elementMatrix,
                                 _OUT double* values) const;
    void AccumulateMatrixValues(const std::vector<Eigen::Index>& valueIndices, int dofCount,
                                const Eigen::MatrixXd& elementMatrix, _OUT double* values) const;
    void AppendElementMatrixValues(const std::vector<Eigen::Index>& valueIndices, int dofCount,
                                   const Eigen::MatrixXd& elementMatrix,
                                   _OUT std::vector<IndexedAssemblyValue>& values) const;
    void AccumulateEffectiveMatrixValues(const std::vector<Eigen::Index>& valueIndices, int dofCount,
                                         const Eigen::MatrixXd* stiffness, const DynamicElementData& dynamicData,
                                         double accelerationDerivative, double velocityDerivative,
                                         _OUT double* values) const;
    void AppendEffectiveMatrixValues(const std::vector<Eigen::Index>& valueIndices, int dofCount,
                                     const Eigen::MatrixXd* stiffness, const DynamicElementData& dynamicData,
                                     double accelerationDerivative, double velocityDerivative,
                                     _OUT std::vector<IndexedAssemblyValue>& values) const;
    static void MergeAssemblyValues(const std::vector<IndexedAssemblyValue>& source, _OUT double* destination);

    void EvaluateDynamicElement(const ElementAssemblyMap& assemblyMap, _OUT DynamicElementData& result);
    void EvaluateRigidBodyInertia(const RigidBodyInertiaAssemblyMap& assemblyMap,
                                  _OUT DynamicElementData& result);
    void AccumulateInertiaForce(const std::vector<int>& dofs, const Eigen::VectorXd& elementForce,
                                _OUT Eigen::VectorXd& globalForce) const;
    std::vector<bool> GetRotationalFreeDofs() const;
    bool PrepareStructuralDamping(_OUT QString& errorMessage);
    bool InitializeConsistentDynamicAcceleration(double time, _OUT QString& errorMessage);

    /**
     * @brief 组装所有荷载到力向量
     * @param [out] F1 约束自由度对应的力向量
     * @param [out] F2 自由自由度对应的力向量
     * @param [in] Factor 荷载缩放系数
     * @param [in] currentTime 当前时间
     */
    void Assemble_AllLoads(_OUT Eigen::VectorXd& F1, _OUT Eigen::VectorXd& F2, _OUT double& Factor,
                           double currentTime);

    /**
     * @brief 计算外荷载（不包含内力和惯性力）
     * @param [in] time 当前时间
     * @param [in] loadFactor 荷载缩放系数
     * @param [out] F1 约束自由度对应的外力向量
     * @param [out] F2 自由自由度对应的外力向量
     */
    void ComputeExternalForce(double time, double loadFactor, _OUT Eigen::VectorXd& F1,
                              _OUT Eigen::VectorXd& F2);

    /**
     * @brief 获取当前时刻的力向量
     * @param [in] current_time 当前时间
     * @return 当前时刻的力向量
     */
    void Updata_NodeData(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, const Eigen::VectorXd& F1,
                         const Eigen::VectorXd* v2 = nullptr, const Eigen::VectorXd* a2 = nullptr);

    void Get_CurrentInforce(_OUT Eigen::VectorXd& Inforce);

    bool Check_Rhs(const Eigen::VectorXd& F2, const Eigen::VectorXd& f2, _OUT Eigen::VectorXd& Rhs);

    /**
     * @brief 组装约束位移
     * @param [out] x1 约束位移向量
     */
    void Assemble_Constraint(_OUT Eigen::VectorXd& x1, double currentTime, double factor);

    /**
    * @brief 根据约束自由度的外力计算并写入节点反力
    * @param [in] F1 约束自由度对应的外力向量
    */
    void CalculateReactions(_OUT Eigen::VectorXd& F1);
    void RecoverDynamicReactions(double time);

    void Get_CurrentStepState(_OUT Eigen::VectorXd& U, _OUT Eigen::VectorXd& V,
                              _OUT Eigen::VectorXd& A) const;

    // ============ IAnalysisModel 接口辅助函数 ============
    Eigen::VectorXd GetCurrentVelocity() const;
    Eigen::VectorXd GetCurrentAcceleration() const;

    void StoreCurrentNodeState();
    void BackupStepState() override;
    void GetStepIncrement(_OUT SolverNameSpace::Vec& dx_step) const override;
};
