#pragma once
#include "ElementBase.h"

#include <array>
#include <optional>

/**
 * @brief 索单元类 - 只承受拉力的二节点单元
 */
class ElementCable : public ElementBase
{
private:
    double Sy = 0.0;
    double Sz = 0.0;
    bool m_ReferenceTssbnConstitutive = false;
    bool m_ReferenceTssbnStaticEquilibrium = false;
    // 缓存参考零偏心质量及其 4/6 节点映射，布局或缩放系数变化时重建。
    MatrixXd m_ReferenceCenteredMass;
    MatrixXd m_ReferenceCenteredMassInReferenceOrder;
    bool m_ReferenceCenteredMassOrderCacheReady = false;
    std::array<double, 2> m_ReferenceCenteredMassCoefficients = {0.0, 0.0};
    std::array<int, 8> m_ReferenceCenteredMassDofMapping{};
    // 各单元独立复用工作空间，每次评估完整覆盖，不保存物理历史。
    VectorXd m_ReferenceAxialGradientScratch;
    MatrixXd m_ReferenceStrainDisplacementScratch;
    MatrixXd m_ReferenceGeometricBlockScratch;
    MatrixXd m_ReferenceGeometricScratch;
    VectorXd m_ReferenceForceScratch;
    MatrixXd m_ReferenceStiffnessScratch;
    MatrixXd m_ReferenceGeometricStiffnessScratch;
    VectorXd m_DynamicAccelerationScratch;
    VectorXd m_ReferenceAccelerationScratch;
    // 仅在一次并行评估内借用主线程准备的截面惯性，派发结束即清除。
    const SectionBase* m_PreparedSectionIdentity = nullptr;
    std::array<double, 3> m_PreparedSectionInertia = {0.0, 0.0, 0.0};
    std::array<double, 2> m_CommittedSpatialTwist = {0.0, 0.0};
    std::array<double, 2> m_BackupCommittedSpatialTwist = {0.0, 0.0};
    struct SpatialTwistCheckpoint
    {
        std::array<double, 2> committed;
        std::array<double, 2> backup;
    };
    std::optional<SpatialTwistCheckpoint> m_CheckpointCommittedSpatialTwist;
    void EvaluateDynamicContributions(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce,
                                      _OUT MatrixXd& gyroscopicMatrix, _OUT MatrixXd& centrifugalMatrix,
                                      bool includeTangents);
    void GetSectionInertia(SectionBase& section, _OUT double& Iy, _OUT double& Iz, _OUT double& J) const;
    int GetReferenceDofMapping(_OUT std::array<int, 8>& referenceToNative) const;

public:
    /**
     * @brief 构造函数
     */
    ElementCable();

    /**
     * @brief 获取单元每个节点的自由度个数
     * @return 最小为 4（平移 X/Y/Z + 标量扭转）；与梁共节点时局部布局自动扩展为 6
     */
    int Get_NodeDOF() const override
    {
        return 4;
    };

    void GetNodeLocalDOFCounts(_OUT std::vector<int>& counts) const override;
    double GetNodalTwist(int nodeIndex) const;
    double GetNodalTwistRate(int nodeIndex) const;
    // 平均轴向扭角对平移和空间左乘转角的梯度及 Hessian。
    // Hessian 是指数坐标中的对称部分；力切线还需加入 SO(3) 转动连接项。
    void GetMeanTwistDerivatives(_OUT VectorXd& gradient, _OUT MatrixXd* hessian = nullptr) const;
    Eigen::Vector3d GetMassFirstMomentGlobal() const;
    void AddNodalAxialTorque(int nodeIndex, double torque, _OUT VectorXd& elementForce) const;
    void CopyRuntimeState(const ElementCable& source);
    // 参考线路单独设置静力和动力本构；关闭后恢复默认只受拉工程应变。
    void SetReferenceTssbnConstitutiveMode(bool enabled, bool staticEquilibrium);
    void SetPreparedSectionInertia(const SectionBase* sectionIdentity, double Iy, double Iz, double J);
    void ClearPreparedSectionInertia();
    bool IsReferenceTssbnConstitutiveMode() const
    {
        return m_ReferenceTssbnConstitutive;
    }

    const std::array<double, 2>& GetCommittedSpatialTwist() const
    {
        return m_CommittedSpatialTwist;
    }
    bool RestoreCommittedSpatialTwist(const std::array<double, 2>& twist);
    void CommitState() override;
    void BackupState() override;
    void RestoreState() override;
    bool PushStateCheckpoint() override;
    bool RestoreStateCheckpoint() override;
    void DiscardStateCheckpoint() override;

    /**
     * @brief 计算单元刚度矩阵
     * @param [out] ke 单元刚度矩阵（普通节点为 8x8，含共享节点时按 4/6 混合布局扩展）
     */
    void Get_ke(_OUT MatrixXd& ke);
    bool TryGetStrainEnergy(_OUT double& energy) const override;
    bool TryGetKineticEnergy(_OUT double& energy) override;
    // 在隔离的单元、属性和截面副本上计算参考静力标定刚度。
    void GetReferenceStaticCalibrationStiffness(_OUT MatrixXd& ke) const;
    void Get_me_Lumped(_OUT MatrixXd& me);     //集中质量矩阵
    void Get_me_Consistent(_OUT MatrixXd& me); //一致质量矩阵
    using ElementBase::GetRayleighDamping;
    void GetRayleighDamping(const std::array<double, 4>& coefficients, const MatrixXd& stiffness,
                            const MatrixXd& physicalMass, _OUT MatrixXd& damping) const override;
    void Get_InertiaForce(_OUT VectorXd& inertiaForce) override;
    // 显式动力残差与质量同态组装时，避免重复计算随构型变化的一致质量矩阵。
    void GetMassAndInertiaForce(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce);
    void Get_GyroscopicMatrix(_OUT MatrixXd& gyroscopicMatrix) override;
    void Get_CentrifugalMatrix(_OUT MatrixXd& centrifugalMatrix) override;
    void GetDynamicContributions(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce,
                                 _OUT MatrixXd& gyroscopicMatrix, _OUT MatrixXd& centrifugalMatrix) override;
    void Get_L0();
    void Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce);
    void Assemble(const std::vector<double>& damping, const MatrixXd& stiffness, const MatrixXd& mass,
                  _OUT MatrixXd& ce);
};
