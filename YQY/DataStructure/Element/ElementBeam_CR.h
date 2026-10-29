#pragma once
#include "ElementBase.h"
#include <array>
#include <optional>
class ElementBeam_CR : public ElementBase
{
public:
    ElementBeam_CR();

    Vector3d q0; // 单元参考向量,截面向量

    int Get_NodeDOF() const override
    {
        return 6;
    };
    Eigen::Matrix3d R0 = Matrix3d::Identity();

    void Get_ke(_OUT MatrixXd& ke);
    double GetStrainEnergy();
    bool TryGetStrainEnergy(_OUT double& energy) const override;
    bool TryGetKineticEnergy(_OUT double& energy) override;
    void Get_me_Lumped(_OUT MatrixXd& me);     //集中质量矩阵
    void Get_me_Consistent(_OUT MatrixXd& me); //一致质量矩阵
    void Get_me_Physical(_OUT MatrixXd& me) override;
    void Get_InertiaForce(_OUT VectorXd& inertiaForce) override;
    void Get_GyroscopicMatrix(_OUT MatrixXd& gyroscopicMatrix) override;
    void Get_CentrifugalMatrix(_OUT MatrixXd& centrifugalMatrix) override;
    void GetDynamicContributions(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce,
                                 _OUT MatrixXd& gyroscopicMatrix, _OUT MatrixXd& centrifugalMatrix) override;
    void Get_L0();
    void CommitState() override;
    void BackupState() override;
    void RestoreState() override;
    bool PushStateCheckpoint() override;
    bool RestoreStateCheckpoint() override;
    void DiscardStateCheckpoint() override;
    void ResetGaussRotationHistory();
    void SetGaussHistoryDynamicMode(bool dynamicMode);
    void GetCommittedGaussRotations(_OUT std::array<Matrix3d, 2>& rotations);
    void GetCurrentGaussRotations(_OUT std::array<Matrix3d, 2>& rotations);
    bool RestoreCommittedGaussRotations(const std::array<Matrix3d, 2>& rotations);
    void Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce);
    void SetReferenceTssbnDynamicMode(bool enabled)
    {
        m_ReferenceTssbnDynamics = enabled;
    }
    bool IsReferenceTssbnDynamicMode() const
    {
        return m_ReferenceTssbnDynamics;
    }
    void SetReferenceTssbnConstitutiveMode(bool enabled, bool staticEquilibrium)
    {
        m_ReferenceTssbnConstitutive = enabled;
        m_ReferenceTssbnStaticEquilibrium = enabled && staticEquilibrium;
        m_ReferenceTssbnDynamics = enabled && !staticEquilibrium;
    }
    bool IsReferenceTssbnConstitutiveMode() const
    {
        return m_ReferenceTssbnConstitutive;
    }
    void CopyRuntimeState(const ElementBeam_CR& source)
    {
        m_ReferenceTssbnDynamics = source.m_ReferenceTssbnDynamics;
        m_ReferenceTssbnConstitutive = source.m_ReferenceTssbnConstitutive;
        m_ReferenceTssbnStaticEquilibrium = source.m_ReferenceTssbnStaticEquilibrium;
        m_CommittedGaussHistory = source.m_CommittedGaussHistory;
        m_BackupGaussHistory = source.m_BackupGaussHistory;
        m_GaussHistoryDynamicMode = source.m_GaussHistoryDynamicMode;
        m_GaussCheckpoint.reset();
    }
    void GetReferenceStaticCalibrationStiffness(_OUT MatrixXd& stiffness) const;

private:
    struct GaussHistoryState
    {
        bool initialized = false;
        std::array<Matrix3d, 2> rotations{Matrix3d::Identity(), Matrix3d::Identity()};
    };
    GaussHistoryState m_CommittedGaussHistory;
    GaussHistoryState m_BackupGaussHistory;
    bool m_GaussHistoryDynamicMode = true;
    std::optional<std::array<GaussHistoryState, 2>> m_GaussCheckpoint;
    void EnsureGaussRotationHistory();
    bool m_ReferenceTssbnDynamics = false;
    bool m_ReferenceTssbnConstitutive = false;
    bool m_ReferenceTssbnStaticEquilibrium = false;
    Vector3d def_p1, def_p2; //当前节点坐标
    Matrix3d Rr;             // 局部随动坐标系

    // 按相同的共回转运动学同时计算一致质量、惯性残量和速度切线。
    // 三个输出对应论文离散式中的 M、f_in 和 C_k；允许传入空指针跳过输出。
    void EvaluateDynamicSystem(_OUT MatrixXd* massMatrix, _OUT VectorXd* inertiaForce, _OUT MatrixXd* velocityTangent,
                               _OUT MatrixXd* configurationTangent = nullptr, bool physicalAccelerationMass = false);
    void EvaluateReferenceTssbnDynamicSystem(_OUT MatrixXd* massMatrix, _OUT VectorXd* inertiaForce,
                                            _OUT MatrixXd* velocityTangent, _OUT MatrixXd* configurationTangent);
    void GetReferenceTssbnStiffness(_OUT MatrixXd& stiffness);
    void GetReferenceMaterialLaw(const VectorXd& deformation, double length, _OUT MatrixXd& stiffness,
                                  _OUT VectorXd& force);
    static Matrix3d ReferenceSkew(const Vector3d& vector);
    static Matrix3d ReferenceRotationMap(const Vector3d& rotation, bool inverse);
    static Matrix3d ReferenceLocalRotationTangent(const Vector3d& rotation, const Vector3d& moment);
    static Matrix3d ReferenceSpatialRotationTangent(const Vector3d& rotation, const Vector3d& moment);
    static void AssembleReferenceGeometricStiffness(const Vector3d& chord, const Matrix3d& localFrame,
                                                     const MatrixXd& frameSpin, const MatrixXd& localStiffness,
                                                     const VectorXd& localForce, _OUT MatrixXd& stiffness,
                                                     _OUT VectorXd& force);

    void Get_kl(const VectorXd& pl, _OUT MatrixXd& kl, _OUT VectorXd& fl); // 参考长度上的局部弹性本构
    Vector3d ComputeRelativeChord(_OUT double& axialExtension) const;
    void ComputeDeformedState(_OUT Vector3d& def_p1, _OUT Vector3d& def_p2, _OUT Matrix3d& Rg_1, _OUT Matrix3d& Rg_2,
                              _OUT Vector3d& q1, _OUT Vector3d& q2, _OUT Vector3d& q);

    // 计算局部随动坐标系 Rr
    Matrix3d ComputeLocalFrame(const Vector3d& chord, const Vector3d& q);

    // 计算局部变形向量 pl (7x1)
    VectorXd ComputeLocalDeformation(double axialExtension, const Matrix3d& Rr, const Matrix3d& Rg_1,
                                     const Matrix3d& Rg_2);

    // 计算局部材料刚度与内力 (kl, fl, fa, ka)
    void ComputeMaterialStiffness(const VectorXd& pl, _OUT MatrixXd& kl, _OUT VectorXd& fl, _OUT VectorXd& fa,
                                  _OUT MatrixXd& ka);

    // 计算全局投影矩阵与材料刚度贡献
    void ComputeGlobalProjection(const Vector3d& chord, const Vector3d& q1, const Vector3d& q2, const Matrix3d& Rr,
                                 const MatrixXd& ka, const VectorXd& fa, _OUT MatrixXd& K_material, _OUT VectorXd& fp,
                                 _OUT MatrixXd& P, _OUT MatrixXd& G);
};
