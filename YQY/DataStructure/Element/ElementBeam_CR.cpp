#include "ElementBeam_CR.h"

#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Section/SectionRectangle.h"
#include "DataStructure/Section/Section_Ice.h"

#include <cmath>
#include <limits>

ElementBeam_CR::ElementBeam_CR()
{
    m_pNode.resize(2);
}

void ElementBeam_CR::EnsureGaussRotationHistory()
{
    if (m_GaussHistoryDynamicMode && m_CommittedGaussHistory.initialized)
        return;
    if (L0 <= 0.0)
        Get_L0();
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("CR Gauss rotation history requires two nodes");
    const Matrix3d firstSection = first->m_Rg_n * R0;
    Vector3d relative;
    Utility::CR::Extract_RotationVector(second->m_Rg_n * R0 * firstSection.transpose(), relative);
    for (int index = 0; index < 2; ++index)
    {
        const double coordinate = (index == 0 ? -1.0 : 1.0) / std::sqrt(3.0);
        Matrix3d increment;
        Utility::CR::Calculate_RotationMatrix(0.5 * (1.0 + coordinate) * relative, increment);
        m_CommittedGaussHistory.rotations[index] = increment * firstSection;
    }
    m_CommittedGaussHistory.initialized = true;
    // 首次动力装配可能晚于步备份；此时初始已接受状态也是合法回滚基准。
    m_BackupGaussHistory = m_CommittedGaussHistory;
}

void ElementBeam_CR::CommitState()
{
    if (!m_GaussHistoryDynamicMode)
    {
        ResetGaussRotationHistory();
        return;
    }
    if (m_ReferenceTssbnDynamics || !m_CommittedGaussHistory.initialized)
        return;
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("CR Gauss rotation commit requires two nodes");
    // 必须先提交 Gauss 姿态，再由分析步把节点增量清零。
    for (int index = 0; index < 2; ++index)
    {
        const double coordinate = (index == 0 ? -1.0 : 1.0) / std::sqrt(3.0);
        const double secondShape = 0.5 * (1.0 + coordinate);
        Matrix3d increment;
        Utility::CR::Calculate_RotationMatrix((1.0 - secondShape) * first->m_StepRotation +
                                                 secondShape * second->m_StepRotation,
                                             increment);
        m_CommittedGaussHistory.rotations[index] = increment * m_CommittedGaussHistory.rotations[index];
    }
}

void ElementBeam_CR::BackupState()
{
    m_BackupGaussHistory = m_CommittedGaussHistory;
}

void ElementBeam_CR::RestoreState()
{
    m_CommittedGaussHistory = m_BackupGaussHistory;
}

bool ElementBeam_CR::PushStateCheckpoint()
{
    if (m_GaussCheckpoint)
        return false;
    m_GaussCheckpoint = std::array<GaussHistoryState, 2>{m_CommittedGaussHistory, m_BackupGaussHistory};
    return true;
}

bool ElementBeam_CR::RestoreStateCheckpoint()
{
    if (!m_GaussCheckpoint)
        return false;
    m_CommittedGaussHistory = (*m_GaussCheckpoint)[0];
    m_BackupGaussHistory = (*m_GaussCheckpoint)[1];
    return true;
}

void ElementBeam_CR::DiscardStateCheckpoint()
{
    m_GaussCheckpoint.reset();
}

void ElementBeam_CR::ResetGaussRotationHistory()
{
    m_CommittedGaussHistory = GaussHistoryState{};
    m_BackupGaussHistory = GaussHistoryState{};
    m_GaussCheckpoint.reset();
}

void ElementBeam_CR::SetGaussHistoryDynamicMode(bool dynamicMode)
{
    if (!dynamicMode || !m_GaussHistoryDynamicMode)
        ResetGaussRotationHistory();
    m_GaussHistoryDynamicMode = dynamicMode;
}

void ElementBeam_CR::GetCommittedGaussRotations(_OUT std::array<Matrix3d, 2>& rotations)
{
    EnsureGaussRotationHistory();
    rotations = m_CommittedGaussHistory.rotations;
}

void ElementBeam_CR::GetCurrentGaussRotations(_OUT std::array<Matrix3d, 2>& rotations)
{
    EnsureGaussRotationHistory();
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("CR current Gauss rotations require two nodes");
    Vector3d increments[2];
    // 查询不依赖上一次装配是否刷新了节点步内转角缓存。
    Utility::CR::Extract_RotationVector(first->m_Rg * first->m_Rg_n.transpose(), increments[0]);
    Utility::CR::Extract_RotationVector(second->m_Rg * second->m_Rg_n.transpose(), increments[1]);
    for (int index = 0; index < 2; ++index)
    {
        const double coordinate = (index == 0 ? -1.0 : 1.0) / std::sqrt(3.0);
        const double secondShape = 0.5 * (1.0 + coordinate);
        Matrix3d increment;
        Utility::CR::Calculate_RotationMatrix((1.0 - secondShape) * increments[0] + secondShape * increments[1],
                                             increment);
        rotations[index] = increment * m_CommittedGaussHistory.rotations[index];
    }
}

bool ElementBeam_CR::RestoreCommittedGaussRotations(const std::array<Matrix3d, 2>& rotations)
{
    for (const auto& rotation : rotations)
        if (!rotation.allFinite() || (rotation.transpose() * rotation - Matrix3d::Identity()).norm() > 1.0e-8 ||
            std::abs(rotation.determinant() - 1.0) > 1.0e-8)
            return false;
    m_CommittedGaussHistory.initialized = true;
    m_CommittedGaussHistory.rotations = rotations;
    m_BackupGaussHistory = m_CommittedGaussHistory;
    m_GaussCheckpoint.reset();
    return true;
}

void ElementBeam_CR::GetReferenceStaticCalibrationStiffness(_OUT MatrixXd& stiffness) const
{
    const auto property = m_pProperty.lock();
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!property || !section)
        throw std::runtime_error("ElementBeam_CR has incomplete calibration property/section references");
    // Calculate_I 会更新截面缓存，隔离截面与属性；节点及材料保持只读共享。
    std::shared_ptr<SectionBase> calibrationSection;
    if (const auto circular = std::dynamic_pointer_cast<SectionCircular>(section))
        calibrationSection = std::make_shared<SectionCircular>(*circular);
    else if (const auto rectangle = std::dynamic_pointer_cast<SectionRectangle>(section))
        calibrationSection = std::make_shared<SectionRectangle>(*rectangle);
    else if (const auto ice = std::dynamic_pointer_cast<Section_Ice>(section))
        calibrationSection = std::make_shared<Section_Ice>(*ice);
    else
        throw std::runtime_error("unsupported ElementBeam_CR calibration section type");
    const auto calibrationProperty = std::make_shared<Property>(*property);
    calibrationProperty->m_pSection = calibrationSection;
    ElementBeam_CR calibrationCopy(*this);
    calibrationCopy.m_pProperty = calibrationProperty;
    calibrationCopy.SetReferenceTssbnConstitutiveMode(true, true);
    calibrationCopy.Get_ke(stiffness);
}

void ElementBeam_CR::Get_ke(_OUT MatrixXd& ke)
{
    if (m_ReferenceTssbnConstitutive)
    {
        GetReferenceTssbnStiffness(ke);
        return;
    }
    if (L0 <= 0.0)
        Get_L0();
    // ---- 计算变形状态 ----
    Matrix3d Rg_1, Rg_2;
    Vector3d q1, q2, q;
    ComputeDeformedState(def_p1, def_p2, Rg_1, Rg_2, q1, q2, q);
    double axialExtension = 0.0;
    const Vector3d chord = ComputeRelativeChord(axialExtension);

    // ---- 局部坐标系 ----
    Rr = ComputeLocalFrame(chord, q);
    Vector3d r1 = chord.normalized();

    // ---- 局部变形向量 pl ----
    VectorXd pl = ComputeLocalDeformation(axialExtension, Rr, Rg_1, Rg_2);
    L = chord.norm();

    // ---- 局部材料刚度与内力 ----
    MatrixXd kl, ka;
    VectorXd fl, fa;
    ComputeMaterialStiffness(pl, kl, fl, fa, ka);

    // ---- 全局投影与材料刚度贡献 ----
    MatrixXd K_material, P, G;
    VectorXd fg;
    ComputeGlobalProjection(chord, q1, q2, Rr, ka, fa, K_material, fg, P, G);
    m_inforce = fg;

    // ---- 应力刚度矩阵（几何刚度） ----
    MatrixXd K_sigma;
    Utility::CR::Assemble_stress_k(L, fa, G, P, Rr, q1, q2, r1, K_sigma);

    // ---- 总刚度矩阵 ----
    ke = K_material + K_sigma;
    m_ke = ke;
}

Matrix3d ElementBeam_CR::ReferenceSkew(const Vector3d& vector)
{
    Matrix3d result;
    Utility::CR::SkewSymmetric(vector, result);
    return result;
}

Matrix3d ElementBeam_CR::ReferenceRotationMap(const Vector3d& rotation, bool inverse)
{
    const Matrix3d identity = Matrix3d::Identity();
    if (rotation.isApprox(Vector3d::Zero(), 1.0e-8))
        return identity;
    const double angle = rotation.norm();
    const Vector3d axis = rotation.normalized();
    const Matrix3d skew = ReferenceSkew(rotation);
    if (inverse)
    {
        const double coefficient = (0.5 * angle) / std::tan(0.5 * angle);
        return coefficient * identity + (1.0 - coefficient) * (axis * axis.transpose()) - 0.5 * skew;
    }
    return std::sin(angle) / angle * identity + (1.0 - std::sin(angle) / angle) * (axis * axis.transpose()) +
           0.5 * (std::sin(0.5 * angle) / (0.5 * angle)) * (std::sin(0.5 * angle) / (0.5 * angle)) * skew;
}

Matrix3d ElementBeam_CR::ReferenceLocalRotationTangent(const Vector3d& rotation, const Vector3d& moment)
{
    if (rotation.isApprox(Vector3d::Zero(), 1.0e-15))
        return Matrix3d::Zero();
    const double angle = rotation.norm();
    const double squared = angle * angle;
    const double fourth = squared * squared;
    const double eta = (2.0 * std::sin(angle) - angle * (1.0 + std::cos(angle))) / (2.0 * squared * std::sin(angle));
    const double coefficient =
        (angle * (angle + std::sin(angle)) - 8.0 * std::sin(angle * 0.5) * std::sin(angle * 0.5)) /
        (4.0 * fourth * std::sin(angle * 0.5) * std::sin(angle * 0.5));
    const Matrix3d skew = ReferenceSkew(rotation);
    const double product = rotation.transpose() * moment;
    const Matrix3d outer = moment * rotation.transpose();
    return (eta * (rotation * moment.transpose() - 2.0 * outer + product * Matrix3d::Identity()) +
            coefficient * skew * skew * outer - 0.5 * ReferenceSkew(moment)) *
           ReferenceRotationMap(rotation, true);
}

Matrix3d ElementBeam_CR::ReferenceSpatialRotationTangent(const Vector3d& rotation, const Vector3d& moment)
{
    if (rotation.isApprox(Vector3d::Zero(), 1.0e-15))
        return Matrix3d::Zero();
    const double angle = rotation.norm();
    const Vector3d axis = rotation.normalized();
    const Matrix3d outer = axis * axis.transpose();
    const double product = axis.transpose() * moment;
    double e1 = 0.0;
    double e2 = 0.0;
    double e3 = 0.0;
    double e4 = 0.0;
    if (angle < 1.0e-3)
    {
        const double squared = angle * angle;
        const double fourth = squared * squared;
        e1 = -0.0833333 * squared + 0.00555556 * fourth;
        e2 = 0.5 - 0.0416667 * squared + 0.00138889 * fourth;
        e3 = -(angle / 3.0) + squared * angle / 30.0;
        e4 = angle / 6.0 - angle * squared / 120.0;
    }
    else
    {
        e1 =
            std::sin(angle) / angle - (std::sin(0.5 * angle) / (0.5 * angle)) * (std::sin(0.5 * angle) / (0.5 * angle));
        e2 = 0.5 * (std::sin(0.5 * angle) / (0.5 * angle)) * (std::sin(0.5 * angle) / (0.5 * angle));
        e3 = (std::cos(angle) - std::sin(angle) / angle) / angle;
        e4 = (1.0 - std::sin(angle) / angle) / angle;
    }
    return -e1 * axis.cross(moment) * axis.transpose() + e2 * ReferenceSkew(moment) +
           e3 * (moment * axis.transpose() - product * outer) +
           e4 * (axis * moment.transpose() - 2.0 * product * outer + product * Matrix3d::Identity());
}

void ElementBeam_CR::AssembleReferenceGeometricStiffness(const Vector3d& chord, const Matrix3d& localFrame,
                                                         const MatrixXd& frameSpin, const MatrixXd& localStiffness,
                                                         const VectorXd& localForce, _OUT MatrixXd& stiffness,
                                                         _OUT VectorXd& force)
{
    const Vector3d axis = chord.normalized();
    const double length = chord.norm();
    VectorXd lengthGradient = VectorXd::Zero(12);
    lengthGradient.segment<3>(0) = -axis;
    lengthGradient.segment<3>(6) = axis;
    MatrixXd axes = MatrixXd::Zero(12, 12);
    for (int block = 0; block < 4; ++block)
        axes.block<3, 3>(3 * block, 3 * block) = localFrame;
    MatrixXd rotationMap = MatrixXd::Zero(6, 12);
    rotationMap.block<3, 3>(0, 3).setIdentity();
    rotationMap.block<3, 3>(3, 9).setIdentity();
    rotationMap.block<3, 12>(0, 0) -= frameSpin;
    rotationMap.block<3, 12>(3, 0) -= frameSpin;
    MatrixXd projection = MatrixXd::Zero(7, 12);
    projection.row(0) = lengthGradient.transpose();
    projection.block<6, 12>(1, 0) = rotationMap * axes.transpose();
    const Matrix3d axisDerivative = 1.0 / length * (Matrix3d::Identity() - axis * axis.transpose());
    MatrixXd lengthHessian = MatrixXd::Zero(12, 12);
    lengthHessian.block<3, 3>(0, 0) = lengthHessian.block<3, 3>(6, 6) = axisDerivative;
    lengthHessian.block<3, 3>(0, 6) = lengthHessian.block<3, 3>(6, 0) = -axisDerivative;
    const MatrixXd geometric1 = lengthHessian * localForce[0];
    const VectorXd projectedMoment = rotationMap.transpose() * localForce.segment<6>(1);
    MatrixXd momentSkews = MatrixXd::Zero(12, 3);
    for (int block = 0; block < 4; ++block)
        momentSkews.block<3, 3>(3 * block, 0) = ReferenceSkew(projectedMoment.segment<3>(3 * block));
    const MatrixXd geometric2 = axes * momentSkews * frameSpin * axes.transpose();
    const double eta = frameSpin(0, 2) * length;
    Vector3d momentLengthGradient;
    momentLengthGradient[0] = 0.0;
    momentLengthGradient[1] =
        eta / length * (localForce[1] + localForce[4]) + 1.0 / length * (localForce[2] + localForce[5]);
    momentLengthGradient[2] = 1.0 / length * (localForce[3] + localForce[6]);
    const MatrixXd geometric3 = axes * frameSpin.transpose() * momentLengthGradient * lengthGradient.transpose();
    const MatrixXd geometric = geometric1 - geometric2 + geometric3;
    stiffness = projection.transpose() * localStiffness * projection + geometric;
    force = projection.transpose() * localForce;
}

void ElementBeam_CR::GetReferenceTssbnStiffness(_OUT MatrixXd& stiffness)
{
    if (L0 <= 0.0)
        Get_L0();
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("Reference TSSBN beam node reference is invalid");
    const Vector3d initialChord(second->m_X - first->m_X, second->m_Y - first->m_Y, second->m_Z - first->m_Z);
    const Vector3d displacement0(first->m_Displacement[0], first->m_Displacement[1], first->m_Displacement[2]);
    const Vector3d displacement1(second->m_Displacement[0], second->m_Displacement[1], second->m_Displacement[2]);
    const Vector3d chord = initialChord + displacement1 - displacement0;
    L = chord.norm();
    const Vector3d axis = chord.normalized();
    const Vector3d q1 = first->m_Rg * R0.col(1);
    const Vector3d q2 = second->m_Rg * R0.col(1);
    const Vector3d meanSectionAxis = 0.5 * (q1 + q2);
    const Vector3d cross = axis.cross(meanSectionAxis);
    if (L <= 0.0 || cross.norm() <= 1.0e-12)
        throw std::runtime_error("Reference TSSBN beam local frame is singular");
    Rr.col(0) = axis;
    Rr.col(2) = cross.normalized();
    Rr.col(1) = Rr.col(2).cross(axis);
    VectorXd deformation = VectorXd::Zero(7);
    deformation[0] = L - L0;
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto node = nodeIndex == 0 ? first : second;
        const Matrix3d relativeRotation = Rr.transpose() * node->m_Rg * R0;
        const Eigen::Quaterniond quaternion(relativeRotation);
        const double vectorNorm = quaternion.vec().norm();
        if (vectorNorm != 0.0)
            deformation.segment<3>(1 + 3 * nodeIndex) = 2.0 * std::asin(vectorNorm) * quaternion.vec() / vectorNorm;
    }
    MatrixXd materialStiffness;
    VectorXd materialForce;
    GetReferenceMaterialLaw(deformation, L, materialStiffness, materialForce);
    MatrixXd deformationMap = MatrixXd::Zero(7, 7);
    deformationMap(0, 0) = 1.0;
    MatrixXd localRotationTangent = MatrixXd::Zero(7, 7);
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const int offset = 1 + 3 * nodeIndex;
        deformationMap.block<3, 3>(offset, offset) = ReferenceRotationMap(deformation.segment<3>(offset), true);
        localRotationTangent.block<3, 3>(offset, offset) =
            ReferenceLocalRotationTangent(deformation.segment<3>(offset), materialForce.segment<3>(offset));
    }
    const VectorXd localForce = deformationMap.transpose() * materialForce;
    const MatrixXd localStiffness =
        deformationMap.transpose() * materialStiffness * deformationMap + localRotationTangent;
    const Vector3d localQ = Rr.transpose() * meanSectionAxis;
    const Vector3d localQ1 = Rr.transpose() * q1;
    const Vector3d localQ2 = Rr.transpose() * q2;
    if (std::abs(localQ[1]) <= 1.0e-12)
        throw std::runtime_error("Reference TSSBN beam frame-spin denominator is singular");
    const double eta = localQ[0] / localQ[1];
    MatrixXd frameSpin = MatrixXd::Zero(3, 12);
    frameSpin(0, 2) = eta / L;
    frameSpin(0, 3) = localQ1[1] / localQ[1] / 2.0;
    frameSpin(0, 4) = -localQ1[0] / localQ[1] / 2.0;
    frameSpin(0, 8) = -eta / L;
    frameSpin(0, 9) = localQ2[1] / localQ[1] / 2.0;
    frameSpin(0, 10) = -localQ2[0] / localQ[1] / 2.0;
    frameSpin(1, 2) = 1.0 / L;
    frameSpin(1, 8) = -1.0 / L;
    frameSpin(2, 1) = -1.0 / L;
    frameSpin(2, 7) = 1.0 / L;
    MatrixXd geometricStiffness;
    VectorXd spatialForce;
    AssembleReferenceGeometricStiffness(chord, Rr, frameSpin, localStiffness, localForce, geometricStiffness,
                                        spatialForce);
    MatrixXd spatialRotationMap = MatrixXd::Identity(12, 12);
    MatrixXd spatialRotationTangent = MatrixXd::Zero(12, 12);
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto node = nodeIndex == 0 ? first : second;
        const int offset = 3 + 6 * nodeIndex;
        spatialRotationMap.block<3, 3>(offset, offset) = ReferenceRotationMap(node->m_StepRotation, false);
        if (m_ReferenceTssbnStaticEquilibrium)
            spatialRotationTangent.block<3, 3>(offset, offset) =
                ReferenceSpatialRotationTangent(node->m_StepRotation, spatialForce.segment<3>(offset));
    }
    m_inforce = spatialRotationMap.transpose() * spatialForce;
    if (m_ReferenceTssbnStaticEquilibrium)
        stiffness = spatialRotationMap.transpose() * geometricStiffness * spatialRotationMap + spatialRotationTangent;
    else
        // 参考动态函数最终覆盖 Get_Kr 的切线，但保留 Br^T*fg 内力。
        stiffness = geometricStiffness;
    m_ke = stiffness;
}

void ElementBeam_CR::GetReferenceMaterialLaw(const VectorXd& deformation, double length, _OUT MatrixXd& stiffness,
                                             _OUT VectorXd& force)
{
    // 直接移植参考非线性七自由度材料表达式，两种分析均取当前长度。
    stiffness.setZero(7, 7);
    force.setZero(7);
    double u = deformation(0);
    double t11 = deformation(1);
    double t21 = deformation(2);
    double t31 = deformation(3);
    double t12 = deformation(4);
    double t22 = deformation(5);
    double t32 = deformation(6);

    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!material || !section)
        throw std::runtime_error("Reference TSSBN beam material data is incomplete");
    const double E = material->m_Young;
    const double v = material->m_Poisson;
    const double A = section->m_Area;
    double I22 = 0.0;
    double I33 = 0.0;
    double J = 0.0;
    section->Calculate_I(I22, I33, J);
    const double G = E / (2.0 + 2.0 * v);
    double Io = I22 + I33;

    double r = sqrt(A / PI);
    double r6 = r * r * r * r * r * r;
    double Irr = PI * r6 / 3;

    double L0 = length;
    double L2 = L0 * L0;
    double L3 = L2 * L0;

    stiffness(0, 0) = (A * E) / L0;

    stiffness(1, 1) =
        (30 * G * J * L2 + 45 * E * Irr * (t11 - t12) * (t11 - t12) +
         E * Io * L0 *
             (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) + 30 * u)) /
        (30 * L3);

    stiffness(2, 2) =
        (E *
         (3600 * I33 + 60 * Io * (t11 - t12) * (t11 - t12) +
          A * L0 *
              (L0 * (24 * t21 * t21 - 12 * t21 * t22 + 9 * t22 * t22 + 8 * t31 * t31 - 4 * t31 * t32 + 8 * t32 * t32) +
               120 * u))) /
        (900 * L0);

    stiffness(3, 3) =
        (E *
         (3600 * I22 + 60 * Io * (t11 - t12) * (t11 - t12) +
          A * L0 *
              (L0 * (8 * t21 * t21 - 4 * t21 * t22 + 8 * t22 * t22 + 24 * t31 * t31 - 12 * t31 * t32 + 9 * t32 * t32) +
               120 * u))) /
        (900 * L0);

    stiffness(4, 4) = stiffness(1, 1);

    stiffness(5, 5) =
        (E *
         (3600 * I33 + 60 * Io * (t11 - t12) * (t11 - t12) +
          A * L0 *
              (L0 * (9 * t21 * t21 - 12 * t21 * t22 + 24 * t22 * t22 + 8 * t31 * t31 - 4 * t31 * t32 + 8 * t32 * t32) +
               120 * u))) /
        (900 * L0);

    stiffness(6, 6) =
        (E *
         (3600 * I22 + 60 * Io * (t11 - t12) * (t11 - t12) +
          A * L0 *
              (L0 * (8 * t21 * t21 - 4 * t21 * t22 + 8 * t22 * t22 + 9 * t31 * t31 - 12 * t31 * t32 + 24 * t32 * t32) +
               120 * u))) /
        (900 * L0);

    stiffness(0, 1) = (E * Io * (t11 - t12)) / L2;
    stiffness(1, 0) = stiffness(0, 1);

    stiffness(0, 2) = 1. / 30 * A * E * (4 * t21 - t22);
    stiffness(2, 0) = stiffness(0, 2);

    stiffness(0, 3) = 1. / 30 * A * E * (4 * t31 - t32);
    stiffness(3, 0) = stiffness(0, 3);

    stiffness(0, 4) = (E * Io * (t12 - t11)) / L2;
    stiffness(4, 0) = stiffness(0, 4);

    stiffness(0, 5) = -(1. / 30) * A * E * (t21 - 4 * t22);
    stiffness(5, 0) = stiffness(0, 5);

    stiffness(0, 6) = -(1. / 30) * A * E * (t31 - 4 * t32);
    stiffness(6, 0) = stiffness(0, 6);

    stiffness(1, 2) = (E * Io * (t11 - t12) * (4 * t21 - t22)) / (30 * L0);
    stiffness(2, 1) = stiffness(1, 2);

    stiffness(1, 3) = (E * Io * (t11 - t12) * (4 * t31 - t32)) / (30 * L0);
    stiffness(3, 1) = stiffness(1, 3);

    stiffness(1, 4) =
        -((G * J) / L0) -
        (E * (45 * Irr * (t11 - t12) * (t11 - t12) +
              Io * L0 *
                  (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) +
                   30 * u))) /
            (30 * L3);
    stiffness(4, 1) = stiffness(1, 4);

    stiffness(1, 5) = -((E * Io * (t11 - t12) * (t21 - 4 * t22)) / (30 * L0));
    stiffness(5, 1) = stiffness(1, 5);

    stiffness(1, 6) = -((E * Io * (t11 - t12) * (t31 - 4 * t32)) / (30 * L0));
    stiffness(6, 1) = stiffness(1, 6);

    stiffness(2, 3) = 1. / 900 * A * E * L0 * (4 * t21 - t22) * (4 * t31 - t32);
    stiffness(3, 2) = stiffness(2, 3);

    stiffness(2, 4) = -((E * Io * (t11 - t12) * (4 * t21 - t22)) / (30 * L0));
    stiffness(4, 2) = stiffness(2, 4);

    stiffness(2, 5) =
        (E * (1800 * I33 - 15 * Io * (t11 - t12) * (t11 - t12) -
              A * L0 *
                  (L0 * (6 * t21 * t21 - 18 * t21 * t22 + 6 * t22 * t22 + 2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) +
                   30 * u))) /
        (900 * L0);
    stiffness(5, 2) = stiffness(2, 5);

    stiffness(2, 6) = -(1. / 900) * A * E * L0 * (4 * t21 - t22) * (t31 - 4 * t32);
    stiffness(6, 2) = stiffness(2, 6);

    stiffness(3, 4) = -((E * Io * (t11 - t12) * (4 * t31 - t32)) / (30 * L0));
    stiffness(4, 3) = stiffness(3, 4);

    stiffness(3, 5) = -(1. / 900) * A * E * L0 * (t21 - 4 * t22) * (4 * t31 - t32);
    stiffness(5, 3) = stiffness(3, 5);

    stiffness(3, 6) =
        (E * (1800 * I22 - 15 * Io * (t11 - t12) * (t11 - t12) -
              A * L0 *
                  (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 6 * (t31 * t31 - 3 * t31 * t32 + t32 * t32)) +
                   30 * u))) /
        (900 * L0);
    stiffness(6, 3) = stiffness(3, 6);

    stiffness(4, 5) = (E * Io * (t11 - t12) * (t21 - 4 * t22)) / (30 * L0);
    stiffness(5, 4) = stiffness(4, 5);

    stiffness(4, 6) = (E * Io * (t11 - t12) * (t31 - 4 * t32)) / (30 * L0);
    stiffness(6, 4) = stiffness(4, 6);

    stiffness(5, 6) = 1. / 900 * A * E * L0 * (t21 - 4 * t22) * (t31 - 4 * t32);
    stiffness(6, 5) = stiffness(5, 6);

    force(0) = (E * (15 * Io * (t11 - t12) * (t11 - t12) +
                     A * L0 *
                         (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) +
                          30 * u))) /
               (30 * L2);

    force(1) = ((t11 - t12) *
                (30 * G * J * L2 + 15 * E * Irr * (t11 - t12) * (t11 - t12) +
                 E * Io * L0 *
                     (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 + 2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) +
                      30 * u))) /
               (30 * L3);

    force(2) =
        (E * (1800 * I33 * (2 * t21 + t22) + (4 * t21 - t22) * (15 * Io * (t11 - t12) * (t11 - t12) +
                                                                A * L0 *
                                                                    (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 +
                                                                           2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) +
                                                                     30 * u)))) /
        (900 * L0);

    force(3) =
        (E * (1800 * I22 * (2 * t31 + t32) + (4 * t31 - t32) * (15 * Io * (t11 - t12) * (t11 - t12) +
                                                                A * L0 *
                                                                    (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 +
                                                                           2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) +
                                                                     30 * u)))) /
        (900 * L0);

    force(4) = -force(1);

    force(5) =
        (E * (1800 * I33 * (t21 + 2 * t22) - (t21 - 4 * t22) * (15 * Io * (t11 - t12) * (t11 - t12) +
                                                                A * L0 *
                                                                    (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 +
                                                                           2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) +
                                                                     30 * u)))) /
        (900 * L0);

    force(6) =
        (E * (1800 * I22 * (t31 + 2 * t32) - (t31 - 4 * t32) * (15 * Io * (t11 - t12) * (t11 - t12) +
                                                                A * L0 *
                                                                    (L0 * (2 * t21 * t21 - t21 * t22 + 2 * t22 * t22 +
                                                                           2 * t31 * t31 - t31 * t32 + 2 * t32 * t32) +
                                                                     30 * u)))) /
        (900 * L0);
    force(0) += m_InitStress * A;
    m_Stress = force(0) / A;
    // 参考静力 Get_B3D_Kl 未填写(1,0)，动态 Get_B3D_Kl_TSSBN 则填写。
    if (m_ReferenceTssbnStaticEquilibrium)
        stiffness(1, 0) = 0.0;
}

bool ElementBeam_CR::TryGetStrainEnergy(_OUT double& energy) const
{
    energy = std::numeric_limits<double>::quiet_NaN();
    // 参考非线性本构使用当前长度，现有内力未包含该长度依赖的势能导数。
    if (m_ReferenceTssbnConstitutive)
        return false;
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!first || !second || !material || !section || L0 <= 0.0 || material->m_Poisson <= -1.0 ||
        first->m_Displacement.size() < 3 || second->m_Displacement.size() < 3)
        return false;
    double Iy = 0.0, Iz = 0.0, J = 0.0;
    if (!TryGetEnergySectionInertia(Iy, Iz, J))
        return false;
    double extension = 0.0;
    const Vector3d chord = ComputeRelativeChord(extension);
    const Vector3d meanAxis = 0.5 * (first->m_Rg * R0.col(1) + second->m_Rg * R0.col(1));
    if (chord.norm() <= 1.0e-12 || chord.normalized().cross(meanAxis).norm() <= 1.0e-12)
        return false;
    Matrix3d frame;
    Utility::CR::Calculate_Rr(Vector3d::Zero(), chord, meanAxis, frame);
    Vector3d firstRotation, secondRotation;
    Utility::CR::Extract_RotationVector(frame.transpose() * first->m_Rg * R0, firstRotation);
    Utility::CR::Extract_RotationVector(frame.transpose() * second->m_Rg * R0, secondRotation);
    const double young = material->m_Young;
    const double twist = firstRotation.x() - secondRotation.x();
    const double bendY = firstRotation.y() * firstRotation.y() + firstRotation.y() * secondRotation.y() +
                         secondRotation.y() * secondRotation.y();
    const double bendZ = firstRotation.z() * firstRotation.z() + firstRotation.z() * secondRotation.z() +
                         secondRotation.z() * secondRotation.z();
    const double value = 0.5 * young * section->m_Area / L0 * extension * extension +
                         m_InitStress * section->m_Area * extension +
                         young * J / (4.0 * (1.0 + material->m_Poisson) * L0) * twist * twist +
                         2.0 * young / L0 * (Iy * bendY + Iz * bendZ);
    if (!std::isfinite(value))
        return false;
    energy = value;
    return true;
}

bool ElementBeam_CR::TryGetKineticEnergy(_OUT double& energy)
{
    if (m_ReferenceTssbnDynamics)
        return ElementBase::TryGetKineticEnergy(energy);
    energy = std::numeric_limits<double>::quiet_NaN();
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!first || !second || !material || !section || L0 <= 0.0 ||
        first->m_Velocity.size() < 6 || second->m_Velocity.size() < 6)
        return false;
    double Iy = 0.0, Iz = 0.0, J = 0.0;
    if (!TryGetEnergySectionInertia(Iy, Iz, J))
        return false;
    Vector3d rotaryDensity(material->m_Density * (Iy + Iz), material->m_Density * Iy, material->m_Density * Iz);
    if (section->m_MassInertiaPerLength.allFinite() && (section->m_MassInertiaPerLength.array() >= 0.0).all() &&
        (section->m_MassInertiaPerLength.array() > 0.0).any())
        rotaryDensity = section->m_MassInertiaPerLength;
    const double lineMass = material->m_Density * section->m_Area;
    Vector3d firstVelocity, secondVelocity, firstSpin, secondSpin;
    for (int component = 0; component < 3; ++component)
    {
        firstVelocity[component] = first->m_Velocity[component];
        secondVelocity[component] = second->m_Velocity[component];
        firstSpin[component] = first->m_Velocity[component + 3];
        secondSpin[component] = second->m_Velocity[component + 3];
    }
    double value = 0.0;
    for (int index = 0; index < 2; ++index)
    {
        const double secondShape = 0.5 * (1.0 + (index == 0 ? -1.0 : 1.0) / std::sqrt(3.0));
        const double firstShape = 1.0 - secondShape;
        Matrix3d previousRotation;
        if (m_CommittedGaussHistory.initialized && m_GaussHistoryDynamicMode)
            previousRotation = m_CommittedGaussHistory.rotations[index];
        else
        {
            const Matrix3d firstSection = first->m_Rg_n * R0;
            Vector3d relative;
            Utility::CR::Extract_RotationVector(second->m_Rg_n * R0 * firstSection.transpose(), relative);
            Matrix3d increment;
            Utility::CR::Calculate_RotationMatrix(secondShape * relative, increment);
            previousRotation = increment * firstSection;
        }
        Matrix3d increment;
        Utility::CR::Calculate_RotationMatrix(firstShape * first->m_StepRotation + secondShape * second->m_StepRotation,
                                               increment);
        const Matrix3d rotation = increment * previousRotation;
        const Vector3d velocity = firstShape * firstVelocity + secondShape * secondVelocity;
        const Vector3d materialSpin = rotation.transpose() * (firstShape * firstSpin + secondShape * secondSpin);
        value += 0.25 * L0 * (lineMass * velocity.squaredNorm() +
                               (rotaryDensity.array() * materialSpin.array().square()).sum());
    }
    if (!std::isfinite(value) || value < 0.0)
        return false;
    energy = value;
    return true;
}

double ElementBeam_CR::GetStrainEnergy()
{
    if (L0 <= 0.0)
        Get_L0();
    Matrix3d firstRotation, secondRotation;
    Vector3d firstSectionAxis, secondSectionAxis, meanSectionAxis;
    ComputeDeformedState(def_p1, def_p2, firstRotation, secondRotation, firstSectionAxis, secondSectionAxis,
                         meanSectionAxis);
    double axialExtension = 0.0;
    const Vector3d chord = ComputeRelativeChord(axialExtension);
    const Matrix3d localFrame = ComputeLocalFrame(chord, meanSectionAxis);
    const VectorXd deformation = ComputeLocalDeformation(axialExtension, localFrame, firstRotation, secondRotation);
    MatrixXd stiffness;
    VectorXd internalForce;
    Get_kl(deformation, stiffness, internalForce);
    // 线弹性势能为二次项；初应力做功是一次项，不能随内力一起乘半。
    const double initialAxialForce = m_InitStress * m_pProperty.lock()->m_pSection.lock()->m_Area;
    return 0.5 * deformation.dot(stiffness * deformation) + initialAxialForce * deformation(0);
}

void ElementBeam_CR::Get_me_Lumped(_OUT MatrixXd& me) //集中质量矩阵
{
    if (m_ReferenceTssbnDynamics)
    {
        MatrixXd consistentMass;
        Get_me_Consistent(consistentMass);
        me = MatrixXd::Zero(12, 12);
        for (int i = 0; i < consistentMass.rows(); ++i)
            me(i, i) = consistentMass.row(i).sum();
        m_me = me;
        return;
    }

    MatrixXd physicalMass;
    Get_me_Physical(physicalMass);
    me = MatrixXd::Zero(12, 12);
    // 节点间按三维张量块集总，保留旋转后惯量的非对角项。
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        for (int componentOffset : {0, 3})
        {
            const int row = 6 * nodeIndex + componentOffset;
            me.block<3, 3>(row, row) = physicalMass.block<3, 3>(row, componentOffset) +
                                      physicalMass.block<3, 3>(row, 6 + componentOffset);
        }
    m_me = me;
}

void ElementBeam_CR::EvaluateDynamicSystem(_OUT MatrixXd* massMatrix, _OUT VectorXd* inertiaForce,
                                           _OUT MatrixXd* velocityTangent, _OUT MatrixXd* configurationTangent,
                                           bool physicalAccelerationMass)
{
    if (m_ReferenceTssbnDynamics)
    {
        EvaluateReferenceTssbnDynamicSystem(massMatrix, inertiaForce, velocityTangent, configurationTangent);
        return;
    }
    if (L0 <= 0.0)
        Get_L0();

    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty ? pProperty->m_pMaterial.lock() : nullptr;
    auto pSection = pProperty ? pProperty->m_pSection.lock() : nullptr;
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pMaterial || !pSection || !pNode0 || !pNode1)
        throw std::runtime_error("ElementBeam_CR dynamic data is incomplete");

    // 节点空间角速度/角加速度线性插值，截面姿态采用独立 Gauss 历史。
    // 对该离散惯性力完整求导；这不是论文中完整的三次插值惯性模型。
    const double lineMass = pMaterial->m_Density * pSection->m_Area;
    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);

    Matrix3d materialRotaryInertia = Matrix3d::Zero();
    materialRotaryInertia(0, 0) = pMaterial->m_Density * (Iy + Iz);
    materialRotaryInertia(1, 1) = pMaterial->m_Density * Iy;
    materialRotaryInertia(2, 2) = pMaterial->m_Density * Iz;
    if (pSection->m_MassInertiaPerLength.allFinite() &&
        (pSection->m_MassInertiaPerLength.array() >= 0.0).all() &&
        (pSection->m_MassInertiaPerLength.array() > 0.0).any())
        materialRotaryInertia.diagonal() = pSection->m_MassInertiaPerLength;

    Eigen::Matrix<double, 12, 12> assembledMass = Eigen::Matrix<double, 12, 12>::Zero();
    Eigen::Matrix<double, 12, 12> assembledGyroscopic = Eigen::Matrix<double, 12, 12>::Zero();
    Eigen::Matrix<double, 12, 12> assembledCentrifugal = Eigen::Matrix<double, 12, 12>::Zero();
    Eigen::Matrix<double, 12, 1> assembledInertiaForce = Eigen::Matrix<double, 12, 1>::Zero();

    EnsureGaussRotationHistory();

    Vector3d nodalVelocity[2];
    Vector3d nodalAcceleration[2];
    Vector3d nodalAngularVelocity[2];
    Vector3d nodalAngularAcceleration[2];
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto& node = nodeIndex == 0 ? pNode0 : pNode1;
        for (int component = 0; component < 3; ++component)
        {
            nodalVelocity[nodeIndex](component) = node->m_Velocity[component];
            nodalAcceleration[nodeIndex](component) = node->m_Acceleration[component];
            nodalAngularVelocity[nodeIndex](component) = node->m_Velocity[component + 3];
            nodalAngularAcceleration[nodeIndex](component) = node->m_Acceleration[component + 3];
        }
    }

    Matrix3d nodalInverseSpin[2];
    Matrix3d nodalRotationMapping[2];
    Matrix3d nodalAngularVelocitySkew[2];
    Matrix3d nodalAngularAccelerationSkew[2];
    const bool needsImplicitMapping =
        (massMatrix && !physicalAccelerationMass) || velocityTangent || configurationTangent;
    if (needsImplicitMapping)
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            const auto& node = nodeIndex == 0 ? pNode0 : pNode1;
            Utility::CR::Calculate_Ts_Inv(node->m_StepRotation, nodalInverseSpin[nodeIndex]);
            nodalRotationMapping[nodeIndex] = node->m_Rg * node->m_Rg_n.transpose() * nodalInverseSpin[nodeIndex];
            Utility::CR::SkewSymmetric(nodalAngularVelocity[nodeIndex], nodalAngularVelocitySkew[nodeIndex]);
            Utility::CR::SkewSymmetric(nodalAngularAcceleration[nodeIndex], nodalAngularAccelerationSkew[nodeIndex]);
        }

    const double gaussPoints[2] = {-1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)};
    for (int gaussIndex = 0; gaussIndex < 2; ++gaussIndex)
    {
        const double coordinate = gaussPoints[gaussIndex];
        const double N[2] = {0.5 * (1.0 - coordinate), 0.5 * (1.0 + coordinate)};
        const Matrix3d& previousGaussRotation = m_CommittedGaussHistory.rotations[gaussIndex];

        const Vector3d incrementalGaussRotation = N[0] * pNode0->m_StepRotation + N[1] * pNode1->m_StepRotation;
        Matrix3d incrementalGaussRotationMatrix;
        Utility::CR::Calculate_RotationMatrix(incrementalGaussRotation, incrementalGaussRotationMatrix);
        const Matrix3d currentGaussRotation = incrementalGaussRotationMatrix * previousGaussRotation;

        const Vector3d translationalAcceleration = N[0] * nodalAcceleration[0] + N[1] * nodalAcceleration[1];
        Vector3d materialAngularVelocity;
        Vector3d materialAngularAcceleration;
        // 保留现有节点空间量插值；它与论文 Gauss 材料量更新并非同一离散。
        // 姿态基准必须保留已接受 Gauss 历史，不能在节点重锚定后重新插值。
        const Vector3d spatialAngularVelocity = N[0] * nodalAngularVelocity[0] + N[1] * nodalAngularVelocity[1];
        const Vector3d spatialAngularAcceleration =
            N[0] * nodalAngularAcceleration[0] + N[1] * nodalAngularAcceleration[1];
        materialAngularVelocity = currentGaussRotation.transpose() * spatialAngularVelocity;
        materialAngularAcceleration = currentGaussRotation.transpose() * spatialAngularAcceleration;

        const Vector3d materialAngularMomentum = materialRotaryInertia * materialAngularVelocity;
        const Vector3d materialDynamicMoment = materialRotaryInertia * materialAngularAcceleration +
                                               materialAngularVelocity.cross(materialAngularMomentum);
        const Vector3d spatialDynamicMoment = currentGaussRotation * materialDynamicMoment;

        const Matrix3d spatialInertia =
            currentGaussRotation * materialRotaryInertia * currentGaussRotation.transpose();
        // 节点空间量插值须逐节点求完整链式导数，不能把高斯点当成独立 Newmark 转动节点。
        Matrix3d spatialVelocityDerivative;
        Matrix3d spatialConfigurationDerivative;
        Matrix3d gaussSpatialSpin;
        if (velocityTangent || configurationTangent)
        {
            Matrix3d omegaSkew;
            Matrix3d momentumSkew;
            Utility::CR::SkewSymmetric(spatialAngularVelocity, omegaSkew);
            Utility::CR::SkewSymmetric(spatialInertia * spatialAngularVelocity, momentumSkew);
            spatialVelocityDerivative = omegaSkew * spatialInertia - momentumSkew;
            if (configurationTangent)
            {
                Matrix3d alphaSkew;
                Matrix3d inertiaAlphaSkew;
                Utility::CR::SkewSymmetric(spatialAngularAcceleration, alphaSkew);
                Utility::CR::SkewSymmetric(spatialInertia * spatialAngularAcceleration, inertiaAlphaSkew);
                spatialConfigurationDerivative = spatialInertia * alphaSkew - inertiaAlphaSkew +
                                                 omegaSkew * (spatialInertia * omegaSkew - momentumSkew);
                Utility::CR::Calculate_Ts(incrementalGaussRotation, gaussSpatialSpin);
            }
        }

        for (int a = 0; a < 2; ++a)
        {
            if (inertiaForce)
            {
                assembledInertiaForce.segment<3>(6 * a) += N[a] * lineMass * translationalAcceleration;
                assembledInertiaForce.segment<3>(6 * a + 3) += N[a] * spatialDynamicMoment;
            }
            if (!massMatrix && !velocityTangent && !configurationTangent)
                continue;
            for (int b = 0; b < 2; ++b)
            {
                const double interpolationWeight = N[a] * N[b];
                if (massMatrix)
                {
                    assembledMass.block<3, 3>(6 * a, 6 * b) += interpolationWeight * lineMass * Matrix3d::Identity();
                    const Matrix3d rotationalMass = physicalAccelerationMass
                                                        ? spatialInertia
                                                        : Matrix3d(spatialInertia * nodalRotationMapping[b]);
                    assembledMass.block<3, 3>(6 * a + 3, 6 * b + 3) += interpolationWeight * rotationalMass;
                }
                if (velocityTangent)
                    assembledGyroscopic.block<3, 3>(6 * a + 3, 6 * b + 3) +=
                        interpolationWeight * spatialVelocityDerivative * nodalRotationMapping[b];
                if (configurationTangent)
                    assembledCentrifugal.block<3, 3>(6 * a + 3, 6 * b + 3) +=
                        interpolationWeight *
                        (spatialConfigurationDerivative * gaussSpatialSpin * nodalInverseSpin[b] -
                         spatialInertia * nodalAngularAccelerationSkew[b] -
                         spatialVelocityDerivative * nodalAngularVelocitySkew[b]);
            }
        }
    }

    const double integrationScale = 0.5 * L0;
    if (massMatrix)
        *massMatrix = integrationScale * assembledMass;
    if (inertiaForce)
        *inertiaForce = integrationScale * assembledInertiaForce;
    if (velocityTangent)
        *velocityTangent = integrationScale * assembledGyroscopic;
    if (configurationTangent)
        *configurationTangent = integrationScale * assembledCentrifugal;
}

void ElementBeam_CR::EvaluateReferenceTssbnDynamicSystem(_OUT MatrixXd* massMatrix, _OUT VectorXd* inertiaForce,
                                                         _OUT MatrixXd* velocityTangent,
                                                         _OUT MatrixXd* configurationTangent)
{
    if (!massMatrix && !inertiaForce && !velocityTangent)
    {
        if (configurationTangent)
            configurationTangent->setZero(12, 12);
        return;
    }
    if (L0 <= 0.0)
        Get_L0();
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (!material || !section || !first || !second || L0 <= 0.0)
        throw std::runtime_error("Reference TSSBN beam dynamic data is incomplete");

    // 移植参考 CR_GetGuassPoint_TSSBN 与 CR_Dynamic_K_sa_TSSBN；保持初始长度和两点积分。
    const Vector3d initialChord(second->m_X - first->m_X, second->m_Y - first->m_Y, second->m_Z - first->m_Z);
    const Vector3d displacement0(first->m_Displacement[0], first->m_Displacement[1], first->m_Displacement[2]);
    const Vector3d displacement1(second->m_Displacement[0], second->m_Displacement[1], second->m_Displacement[2]);
    const Vector3d chord = initialChord + displacement1 - displacement0;
    const double currentLength = chord.norm();
    const Vector3d q1 = first->m_Rg * R0.col(1);
    const Vector3d q2 = second->m_Rg * R0.col(1);
    const Vector3d meanSectionAxis = 0.5 * (q1 + q2);
    const Vector3d axis1 = chord.normalized();
    const Vector3d frameCross = axis1.cross(meanSectionAxis);
    if (currentLength <= 0.0 || frameCross.norm() <= 1.0e-12)
        throw std::runtime_error("Reference TSSBN beam local frame is singular");
    Matrix3d localFrame;
    localFrame.col(0) = axis1;
    localFrame.col(2) = frameCross.normalized();
    localFrame.col(1) = localFrame.col(2).cross(axis1);
    VectorXd localRotations = VectorXd::Zero(6);
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto node = nodeIndex == 0 ? first : second;
        const Matrix3d relativeRotation = localFrame.transpose() * node->m_Rg * R0;
        const Eigen::Quaterniond quaternion(relativeRotation);
        const double vectorNorm = quaternion.vec().norm();
        if (vectorNorm != 0.0)
            localRotations.segment<3>(3 * nodeIndex) = 2.0 * std::asin(vectorNorm) * quaternion.vec() / vectorNorm;
    }

    const Vector3d localQ = localFrame.transpose() * meanSectionAxis;
    const Vector3d localQ1 = localFrame.transpose() * q1;
    const Vector3d localQ2 = localFrame.transpose() * q2;
    if (std::abs(localQ[1]) <= 1.0e-12)
        throw std::runtime_error("Reference TSSBN beam frame-spin denominator is singular");
    MatrixXd frameSpin = MatrixXd::Zero(3, 12);
    const double eta = localQ[0] / localQ[1];
    const double eta11 = localQ1[0] / localQ[1];
    const double eta12 = localQ1[1] / localQ[1];
    const double eta21 = localQ2[0] / localQ[1];
    const double eta22 = localQ2[1] / localQ[1];
    frameSpin(0, 2) = eta / currentLength;
    frameSpin(0, 3) = eta12 / 2.0;
    frameSpin(0, 4) = -eta11 / 2.0;
    frameSpin(0, 8) = -eta / currentLength;
    frameSpin(0, 9) = eta22 / 2.0;
    frameSpin(0, 10) = -eta21 / 2.0;
    frameSpin(1, 2) = 1.0 / currentLength;
    frameSpin(1, 8) = -1.0 / currentLength;
    frameSpin(2, 1) = -1.0 / currentLength;
    frameSpin(2, 7) = 1.0 / currentLength;
    MatrixXd localAxes = MatrixXd::Zero(12, 12);
    MatrixXd relativeRotationMap = MatrixXd::Zero(6, 12);
    for (int block = 0; block < 4; ++block)
        localAxes.block<3, 3>(3 * block, 3 * block) = localFrame;
    relativeRotationMap.block<3, 3>(0, 3).setIdentity();
    relativeRotationMap.block<3, 3>(3, 9).setIdentity();
    relativeRotationMap.block<3, 12>(0, 0) -= frameSpin;
    relativeRotationMap.block<3, 12>(3, 0) -= frameSpin;
    VectorXd velocity = VectorXd::Zero(12);
    VectorXd acceleration = VectorXd::Zero(12);
    for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
    {
        const auto node = nodeIndex == 0 ? first : second;
        for (int component = 0; component < 6; ++component)
        {
            velocity[6 * nodeIndex + component] = node->m_Velocity[component];
            acceleration[6 * nodeIndex + component] = node->m_Acceleration[component];
        }
    }
    const VectorXd localVelocity = localAxes.transpose() * velocity;
    const VectorXd localAcceleration = localAxes.transpose() * acceleration;
    const Vector3d frameAngularVelocity = frameSpin * localVelocity;
    Matrix3d frameAngularVelocitySkew;
    Utility::CR::SkewSymmetric(frameAngularVelocity, frameAngularVelocitySkew);
    MatrixXd localFrameSpin = MatrixXd::Zero(12, 12);
    MatrixXd localVelocitySkews = MatrixXd::Zero(12, 3);
    for (int block = 0; block < 4; ++block)
    {
        localFrameSpin.block<3, 3>(3 * block, 3 * block) = frameAngularVelocitySkew;
        Matrix3d velocitySkew;
        Utility::CR::SkewSymmetric(localVelocity.segment<3>(3 * block), velocitySkew);
        localVelocitySkews.block<3, 3>(3 * block, 0) = velocitySkew;
    }
    VectorXd lengthGradient = VectorXd::Zero(12);
    lengthGradient.segment<3>(0) = -axis1;
    lengthGradient.segment<3>(6) = axis1;
    VectorXd localLengthGradient = VectorXd::Zero(12);
    localLengthGradient[0] = -1.0;
    localLengthGradient[6] = 1.0;
    MatrixXd axisDerivative1 = MatrixXd::Zero(3, 12);
    axisDerivative1(1, 1) = axisDerivative1(2, 2) = -1.0;
    axisDerivative1(1, 7) = axisDerivative1(2, 8) = 1.0;
    MatrixXd axisDerivative2 = MatrixXd::Zero(3, 12);
    axisDerivative2(1, 2) = axisDerivative2(2, 7) = 1.0;
    axisDerivative2(2, 1) = axisDerivative2(1, 8) = -1.0;
    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    section->Calculate_I(Iy, Iz, J);
    Matrix3d rotaryInertia = Matrix3d::Zero();
    rotaryInertia.diagonal() << material->m_Density * (Iy + Iz), material->m_Density * Iz, material->m_Density * Iy;
    const double lineMass = material->m_Density * section->m_Area;
    MatrixXd localMass = MatrixXd::Zero(12, 12);
    MatrixXd localVelocityTangent = MatrixXd::Zero(12, 12);
    VectorXd localInertia = VectorXd::Zero(12);
    const double gaussPoints[2] = {-std::sqrt(3.0) / 3.0, std::sqrt(3.0) / 3.0};
    for (double coordinate : gaussPoints)
    {
        const double N[6] = {0.5 - 0.5 * coordinate,
                             0.5 + 0.5 * coordinate,
                             (0.5 - 0.5 * coordinate) * (0.5 - 0.5 * coordinate) * (0.5 + 0.5 * coordinate) * L0,
                             (-0.5 + 0.5 * coordinate) * (0.5 + 0.5 * coordinate) * (0.5 + 0.5 * coordinate) * L0,
                             -0.25 - 0.5 * coordinate + 0.75 * coordinate * coordinate,
                             -0.25 + 0.5 * coordinate + 0.75 * coordinate * coordinate};
        MatrixXd P1 = MatrixXd::Zero(3, 6);
        P1(1, 2) = N[2];
        P1(1, 5) = N[3];
        P1(2, 1) = -N[2];
        P1(2, 4) = -N[3];
        MatrixXd P2 = MatrixXd::Zero(3, 6);
        P2(0, 0) = N[0];
        P2(0, 3) = N[1];
        P2(1, 1) = P2(2, 2) = N[4];
        P2(1, 4) = P2(2, 5) = N[5];
        MatrixXd nodalTranslation = MatrixXd::Zero(3, 12);
        nodalTranslation.block<3, 3>(0, 0) = N[0] * Matrix3d::Identity();
        nodalTranslation.block<3, 3>(0, 6) = N[1] * Matrix3d::Identity();
        const Vector3d gaussTranslation = P1 * localRotations;
        const Vector3d gaussRotation = P2 * localRotations;
        Matrix3d gaussTranslationSkew;
        Matrix3d gaussRotationSkew;
        Utility::CR::SkewSymmetric(gaussTranslation, gaussTranslationSkew);
        Utility::CR::SkewSymmetric(gaussRotation, gaussRotationSkew);
        const MatrixXd H1 = nodalTranslation + P1 * relativeRotationMap - gaussTranslationSkew * frameSpin;
        const MatrixXd H2 = P2 * relativeRotationMap + frameSpin;
        const Matrix3d gaussRotationMap = Matrix3d::Identity() + gaussRotationSkew;
        const Matrix3d currentRotaryInertia = gaussRotationMap * rotaryInertia * gaussRotationMap.transpose();
        if (massMatrix)
            localMass += H1.transpose() * lineMass * H1 + H2.transpose() * currentRotaryInertia * H2;
        if (!inertiaForce && !velocityTangent)
            continue;
        const MatrixXd lengthDerivative1 = (N[2] + N[3]) / (currentLength * currentLength) * axisDerivative1;
        const MatrixXd lengthDerivative2 = (N[4] + N[5] - 1.0) / (currentLength * currentLength) * axisDerivative2;
        const Vector3d localBendingVelocity = P1 * relativeRotationMap * localVelocity;
        Matrix3d localBendingVelocitySkew;
        Utility::CR::SkewSymmetric(localBendingVelocity, localBendingVelocitySkew);
        const MatrixXd dH1 = lengthDerivative1 * lengthGradient.dot(velocity) - localBendingVelocitySkew * frameSpin;
        const MatrixXd dH2 = lengthDerivative2 * lengthGradient.dot(velocity);
        const MatrixXd C1 = frameAngularVelocitySkew * H1 + dH1 - H1 * localFrameSpin;
        const MatrixXd C2 = frameAngularVelocitySkew * H2 + dH2 - H2 * localFrameSpin;
        const Vector3d angularVelocity = H2 * localVelocity;
        Matrix3d angularVelocitySkew;
        Utility::CR::SkewSymmetric(angularVelocity, angularVelocitySkew);
        if (inertiaForce)
        {
            const Vector3d translationalAcceleration = H1 * localAcceleration + C1 * localVelocity;
            const Vector3d angularAcceleration = H2 * localAcceleration + C2 * localVelocity;
            localInertia += H1.transpose() * lineMass * translationalAcceleration +
                            H2.transpose() * (currentRotaryInertia * angularAcceleration +
                                              angularVelocitySkew * currentRotaryInertia * angularVelocity);
        }
        if (!velocityTangent)
            continue;
        const Vector3d translationalVelocity = H1 * localVelocity;
        Matrix3d translationalVelocitySkew;
        Utility::CR::SkewSymmetric(translationalVelocity, translationalVelocitySkew);
        const MatrixXd C3 = -translationalVelocitySkew * frameSpin +
                            lengthDerivative1 * localVelocity * localLengthGradient.transpose() +
                            frameAngularVelocitySkew * P1 * relativeRotationMap + H1 * localVelocitySkews * frameSpin;
        const MatrixXd C4 = -angularVelocitySkew * frameSpin +
                            lengthDerivative2 * localVelocity * localLengthGradient.transpose() +
                            H2 * localVelocitySkews * frameSpin;
        localVelocityTangent +=
            H1.transpose() * lineMass * (C1 + C3) + H2.transpose() * currentRotaryInertia * (C2 + C4) +
            H2.transpose() * (angularVelocitySkew * currentRotaryInertia - currentRotaryInertia * angularVelocitySkew) *
                H2;
    }
    if (massMatrix)
        *massMatrix = 0.5 * L0 * localAxes * localMass * localAxes.transpose();
    if (inertiaForce)
        *inertiaForce = 0.5 * L0 * localAxes * localInertia;
    if (velocityTangent)
        *velocityTangent = 0.5 * L0 * localAxes * localVelocityTangent * localAxes.transpose();
    // 参考源有效切线只组装 Kg+a*M+v*Ck，不附加通用梁的构型惯性切线。
    if (configurationTangent)
        configurationTangent->setZero(12, 12);
}

void ElementBeam_CR::Get_me_Consistent(_OUT MatrixXd& me) //一致质量矩阵
{
    // Le 空间自旋公式的隐式增量切线质量；物理 M(q)*alpha 必须使用 Get_me_Physical。
    EvaluateDynamicSystem(&me, nullptr, nullptr);
    m_me = me;
}

void ElementBeam_CR::Get_me_Physical(_OUT MatrixXd& me)
{
    // 原生参考线路保留其历史质量定义及计算次序；正式三维线路返回惯性力对空间角加速度的系数。
    EvaluateDynamicSystem(&me, nullptr, nullptr, nullptr, true);
}

void ElementBeam_CR::GetDynamicContributions(_OUT MatrixXd& massMatrix, _OUT VectorXd& inertiaForce,
                                             _OUT MatrixXd& gyroscopicMatrix, _OUT MatrixXd& centrifugalMatrix)
{
    EvaluateDynamicSystem(&massMatrix, &inertiaForce, &gyroscopicMatrix, &centrifugalMatrix);
    m_me = massMatrix;
}

void ElementBeam_CR::Get_InertiaForce(_OUT VectorXd& inertiaForce)
{
    EvaluateDynamicSystem(nullptr, &inertiaForce, nullptr);
}

void ElementBeam_CR::Get_GyroscopicMatrix(_OUT MatrixXd& gyroscopicMatrix)
{
    EvaluateDynamicSystem(nullptr, nullptr, &gyroscopicMatrix);
}

void ElementBeam_CR::Get_CentrifugalMatrix(_OUT MatrixXd& centrifugalMatrix)
{
    EvaluateDynamicSystem(nullptr, nullptr, nullptr, &centrifugalMatrix);
}

void ElementBeam_CR::Get_L0()
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();

    if (pNode0 == nullptr || pNode1 == nullptr)
    {
        qDebug().noquote() << QStringLiteral("Error: ElementBeam_CR 节点指针为空");
        return;
    }

    Vector3d p1(pNode0->m_X, pNode0->m_Y, pNode0->m_Z);
    Vector3d p2(pNode1->m_X, pNode1->m_Y, pNode1->m_Z);

    Vector3d d = p2 - p1;
    L0 = d.norm();

    Vector3d r1 = d.normalized();
    Vector3d cross_result = r1.cross(q0);
    Vector3d r3 = cross_result.normalized();
    Vector3d r2 = r3.cross(r1);

    R0.col(0) = r1;
    R0.col(1) = r2;
    R0.col(2) = r3;
    q0 = r2;
}

void ElementBeam_CR::Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce)
{
    // 非线性速度切线属于惯性残量的运动学贡献，不是可选材料阻尼。
    // 保留该旧入口，并与 Newmark 使用的动力切线保持一致。
    (void)damping;
    EvaluateDynamicSystem(nullptr, nullptr, &ce);
}

void ElementBeam_CR::Get_kl(const VectorXd& pl, _OUT MatrixXd& kl, _OUT VectorXd& fl)
{
    if (!std::isfinite(L0) || L0 <= 1.0e-12)
        throw std::runtime_error("CR elastic law requires a positive reference length");
    // 1. 提取变形分量
    const double u = pl(0);
    const double t11 = pl(1), t21 = pl(2), t31 = pl(3);
    const double t12 = pl(4), t22 = pl(5), t32 = pl(6);

    auto pProperty = m_pProperty.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();
    auto pSection = pProperty->m_pSection.lock();

    double A = pSection->m_Area;
    double E = pMaterial->m_Young;
    double G = E / (2. * (1 + pMaterial->m_Poisson));

    double Iy = 0.0;
    double Iz = 0.0;
    double J = 0.0;
    pSection->Calculate_I(Iy, Iz, J);

    // 共旋几何可发生大转动，局部小应变弹性势能仍在参考长度上积分。
    const double EA_L = E * A / L0;
    const double GJ_L = G * J / L0;
    const double EIy_L = E * Iy / L0;
    const double EIz_L = E * Iz / L0;

    // 组装局部切线刚度矩阵 kl (7 x 7)
    kl = MatrixXd::Zero(7, 7); // 必须清零初始化

    // 轴向拉压刚度
    kl(0, 0) = EA_L;

    // 扭转刚度 (绕局部 x 轴)
    kl(1, 1) = GJ_L;
    kl(1, 4) = -GJ_L;
    kl(4, 1) = -GJ_L;
    kl(4, 4) = GJ_L;

    // 弯曲刚度 - 局部 xz 平面 (绕局部 y 轴旋转)
    kl(2, 2) = 4.0 * EIy_L;
    kl(2, 5) = 2.0 * EIy_L;
    kl(5, 2) = 2.0 * EIy_L;
    kl(5, 5) = 4.0 * EIy_L;

    // 弯曲刚度 - 局部 xy 平面 (绕局部 z 轴旋转)
    kl(3, 3) = 4.0 * EIz_L;
    kl(3, 6) = 2.0 * EIz_L;
    kl(6, 3) = 2.0 * EIz_L;
    kl(6, 6) = 4.0 * EIz_L;

    // 计算局部内力向量 fl (7 x 1)
    fl = VectorXd::Zero(7);

    // 轴向力
    fl(0) = EA_L * u + m_InitStress * A;

    // 扭矩
    fl(1) = GJ_L * (t11 - t12);
    fl(4) = GJ_L * (t12 - t11);

    // 绕 y 轴的端弯矩
    fl(2) = 2.0 * EIy_L * (2.0 * t21 + t22);
    fl(5) = 2.0 * EIy_L * (t21 + 2.0 * t22);

    // 绕 z 轴的端弯矩
    fl(3) = 2.0 * EIz_L * (2.0 * t31 + t32);
    fl(6) = 2.0 * EIz_L * (t31 + 2.0 * t32);

    m_Stress = fl(0) / A; // 轴向应力
}

Vector3d ElementBeam_CR::ComputeRelativeChord(_OUT double& axialExtension) const
{
    const auto first = m_pNode[0].lock();
    const auto second = m_pNode[1].lock();
    if (!first || !second)
        throw std::runtime_error("CR relative chord requires two nodes");

    const Vector3d initialChord(second->m_X - first->m_X, second->m_Y - first->m_Y, second->m_Z - first->m_Z);
    Vector3d relativeDisplacement;
    double squaredLengthChange = 0.0;
    for (int component = 0; component < 3; ++component)
    {
        relativeDisplacement[component] = second->m_Displacement[component] - first->m_Displacement[component];
        squaredLengthChange = std::fma(relativeDisplacement[component],
                                      2.0 * initialChord[component] + relativeDisplacement[component],
                                      squaredLengthChange);
    }
    // 先形成相对弦；有理化长度差保留节点中仍可表示的微小轴伸长。
    const Vector3d chord = initialChord + relativeDisplacement;
    axialExtension = squaredLengthChange / (chord.norm() + L0);
    return chord;
}

void ElementBeam_CR::ComputeDeformedState(_OUT Vector3d& def_p1, _OUT Vector3d& def_p2, _OUT Matrix3d& Rg_1,
                                          _OUT Matrix3d& Rg_2, _OUT Vector3d& q1, _OUT Vector3d& q2, _OUT Vector3d& q)
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pNode0 || !pNode1)
        return;

    // 节点初始坐标
    Vector3d init_p1(pNode0->m_X, pNode0->m_Y, pNode0->m_Z);
    Vector3d init_p2(pNode1->m_X, pNode1->m_Y, pNode1->m_Z);

    // 平动位移
    Vector3d ug_p1(pNode0->m_Displacement[0], pNode0->m_Displacement[1], pNode0->m_Displacement[2]);
    Vector3d ug_p2(pNode1->m_Displacement[0], pNode1->m_Displacement[1], pNode1->m_Displacement[2]);

    // 变形后坐标
    def_p1 = init_p1 + ug_p1;
    def_p2 = init_p2 + ug_p2;

    Rg_1 = pNode0->m_Rg;
    Rg_2 = pNode1->m_Rg;

    // 截面方向向量
    q0.normalize(); // 确保初始截面方向向量是单位向量
    Vector3d beam_axis = init_p2 - init_p1;
    beam_axis.normalize();
    if (q0.cross(beam_axis).norm() < 1e-6)
    {
        qDebug().noquote() << QStringLiteral("初始截面向量与轴线重合");
        q0 = Vector3d(0, 0, 1); // 如果重合，选择一个默认的垂直向量
    }

    Vector3d ey(0, 1, 0);
    q1 = Rg_1 * R0 * ey;
    q2 = Rg_2 * R0 * ey;
    q = 0.5 * (q1 + q2);
}

Matrix3d ElementBeam_CR::ComputeLocalFrame(const Vector3d& chord, const Vector3d& q)
{
    Matrix3d Rr;
    Utility::CR::Calculate_Rr(Vector3d::Zero(), chord, q, Rr);
    return Rr;
}

VectorXd ElementBeam_CR::ComputeLocalDeformation(double axialExtension, const Matrix3d& Rr, const Matrix3d& Rg_1,
                                                 const Matrix3d& Rg_2)
{
    // 局部旋转矩阵
    Matrix3d R1_ = Rr.transpose() * Rg_1 * R0;
    Matrix3d R2_ = Rr.transpose() * Rg_2 * R0;

    // 提取局部旋转向量
    Vector3d vartheta1, vartheta2;
    Utility::CR::Extract_RotationVector(R1_, vartheta1);
    Utility::CR::Extract_RotationVector(R2_, vartheta2);

    VectorXd pl(7);
    pl(0) = axialExtension;
    pl.segment<3>(1) = vartheta1;
    pl.segment<3>(4) = vartheta2;
    return pl;
}

void ElementBeam_CR::ComputeMaterialStiffness(const VectorXd& pl, _OUT MatrixXd& kl, _OUT VectorXd& fl,
                                              _OUT VectorXd& fa, _OUT MatrixXd& ka)
{
    // 局部刚度与内力
    Get_kl(pl, kl, fl);

    // 变形映射矩阵 Ba
    Vector3d vartheta1 = pl.segment<3>(1);
    Vector3d vartheta2 = pl.segment<3>(4);
    MatrixXd Ba;
    Utility::CR::Assemble_Matrix_Ba(vartheta1, vartheta2, Ba);

    // 内力投影 fa = Ba^T * fl
    fa = Ba.transpose() * fl;
    if (fa.size() != 7)
    {
        qDebug().noquote() << QStringLiteral("梁 内力向量 fa 大小不为 7");
        return;
    }
    // 局部几何刚度 Kh 和材料刚度 Ka
    MatrixXd Kh;
    Utility::CR::Assemble_Matrix_Kh(vartheta1, vartheta2, fl, Kh);
    ka = Ba.transpose() * kl * Ba + Kh;
}

void ElementBeam_CR::ComputeGlobalProjection(const Vector3d& chord, const Vector3d& q1, const Vector3d& q2,
                                             const Matrix3d& Rr, const MatrixXd& ka, const VectorXd& fa,
                                             _OUT MatrixXd& K_material, _OUT VectorXd& fp, _OUT MatrixXd& P,
                                             _OUT MatrixXd& G)
{
    // 组装转换矩阵 E (12x12) 和投影矩阵 P (7x12)、自旋矩阵 G (3x12)
    MatrixXd ET;
    Utility::CR::Assemble_Matrix_E(Rr, ET);
    Utility::CR::Assemble_Matrix_PG(Vector3d::Zero(), chord, q1, q2, Rr, P, G);

    VectorXd r = VectorXd::Zero(12);
    Vector3d r1 = chord.normalized();
    r.segment<3>(0) = -r1;
    r.segment<3>(6) = r1;
    Eigen::Matrix<double, 7, 12> Bg;
    Bg.setZero();
    Bg.row(0) = r.transpose();
    Bg.block<6, 12>(1, 0) = P * ET;

    fp = Bg.transpose() * fa;

    K_material = Bg.transpose() * ka * Bg;
}
