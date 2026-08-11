#include "ElementCable.h"
#include <stdexcept>

ElementCable::ElementCable()
{
    m_pNode.resize(2);
}

void ElementCable::Get_ke(MatrixXd& ke)
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    auto pProperty = m_pProperty.lock();
    if (!pNode0 || !pNode1 || !pProperty)
        throw std::runtime_error("ElementCable has incomplete node/property references");

    auto pMaterial = pProperty->m_pMaterial.lock();
    auto pSection = pProperty->m_pSection.lock();
    if (!pMaterial || !pSection)
        throw std::runtime_error("ElementCable has incomplete material/section references");

    Get_L0();
    const Eigen::Vector3d x0(
        pNode0->m_X + pNode0->m_Displacement[0],
        pNode0->m_Y + pNode0->m_Displacement[1],
        pNode0->m_Z + pNode0->m_Displacement[2]);
    const Eigen::Vector3d x1(
        pNode1->m_X + pNode1->m_Displacement[0],
        pNode1->m_Y + pNode1->m_Displacement[1],
        pNode1->m_Z + pNode1->m_Displacement[2]);
    const Eigen::Vector3d chord = x1 - x0;
    const double currentLength = chord.norm();
    if (currentLength <= 1.0e-12 || L0 <= 1.0e-12)
        throw std::runtime_error("ElementCable length must be positive");

    const Eigen::Vector3d axis = chord / currentLength;
    const double area = pSection->m_Area;
    if (area <= 0.0)
        throw std::runtime_error("ElementCable section area must be positive");

    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    pSection->Calculate_I(Iy, Iz, polarMoment);
    if (polarMoment <= 0.0 && pSection->m_Radius > 0.0)
        polarMoment = 0.5 * area * pSection->m_Radius * pSection->m_Radius;
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    if (1.0 + pMaterial->m_Poisson <= 1.0e-12)
        throw std::runtime_error("ElementCable Poisson ratio must be greater than -1");

    const double shearModulus = pMaterial->m_Young
        / (2.0 * (1.0 + pMaterial->m_Poisson));
    // 与 TSSBN 参考实现 Element_Cable_CR::Calculate_ke_TSSBN 保持一致：
    // 采用初始构形（材料坐标）上的工程应变和扭转率。这样
    // N = N0 + EA/L0*(L-L0)、M = M0 + GJ/L0*theta 的导数与下方
    // 材料切线严格一致。旧实现使用当前长度作分母，却仍把 EA/L
    // 直接当作切线，内力与切线并不共轭，会破坏 Newton 收敛和动力频率。
    const double axialStiffness = pMaterial->m_Young * area / L0;
    const double torsionalStiffness = shearModulus * polarMoment / L0;
    const double extension = currentLength - L0;
    const double twist = pNode1->m_Displacement[3] - pNode0->m_Displacement[3];

    Eigen::Vector2d generalizedForce;
    // 索不能传递压力。轴向力被截断到零后，轴向材料刚度和几何刚度
    // 也必须同时移除，否则松弛索会在压缩状态下产生非物理反力。
    const double trialAxialForce = m_InitStress * area + axialStiffness * extension;
    const bool isTaut = trialAxialForce > 0.0;
    generalizedForce(0) = isTaut ? trialAxialForce : 0.0;
    generalizedForce(1) = torsionalStiffness * twist;
    m_Stress = area > 0.0 ? generalizedForce(0) / area : 0.0;

    Eigen::Matrix<double, 2, 8> B = Eigen::Matrix<double, 2, 8>::Zero();
    B.block<1, 3>(0, 0) = -axis.transpose();
    B.block<1, 3>(0, 4) = axis.transpose();
    B(1, 3) = -1.0;
    B(1, 7) = 1.0;

    Eigen::Matrix2d material = Eigen::Matrix2d::Zero();
    material(0, 0) = isTaut ? axialStiffness : 0.0;
    material(1, 1) = torsionalStiffness;
    ke = B.transpose() * material * B;

    const Eigen::Matrix3d geometricBlock = generalizedForce(0) / currentLength
        * (Eigen::Matrix3d::Identity() - axis * axis.transpose());
    ke.block<3, 3>(0, 0) += geometricBlock;
    ke.block<3, 3>(4, 4) += geometricBlock;
    ke.block<3, 3>(0, 4) -= geometricBlock;
    ke.block<3, 3>(4, 0) -= geometricBlock;

    m_inforce = B.transpose() * generalizedForce;
    m_ke = ke;
    L = currentLength;

}

void ElementCable::Get_me_Lumped(MatrixXd& me)//集中质量矩阵
{
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!material || !section)
        throw std::runtime_error("ElementCable has incomplete material/section references");

    Get_L0();
    if (section->m_Area <= 0.0 || material->m_Density < 0.0)
        throw std::runtime_error("ElementCable mass properties must be nonnegative");
    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    section->Calculate_I(Iy, Iz, polarMoment);
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    const double linearDensity = section->m_Area * material->m_Density;
    const double rotaryDensity = polarMoment * material->m_Density;
    constexpr double Sy = 0.0;
    constexpr double Sz = 0.0;

    me.setZero(8, 8);

    me(0, 0) = me(1, 1) = me(2, 2) = me(4, 4) = me(5, 5) = me(6, 6) = linearDensity;
    me(3, 3) = me(7, 7) = rotaryDensity;
    me(1, 3) = me(3, 1) = me(5, 7) = me(7, 5) = -Sy;
    me(2, 3) = me(3, 2) = me(6, 7) = me(7, 6) = Sz;

    me *= (L0 / 2.0);
}

void ElementCable::Get_me_Consistent(MatrixXd& me) //一致质量矩阵
{
    const auto property = m_pProperty.lock();
    const auto material = property ? property->m_pMaterial.lock() : nullptr;
    const auto section = property ? property->m_pSection.lock() : nullptr;
    if (!material || !section)
        throw std::runtime_error("ElementCable has incomplete material/section references");

    Get_L0();
    if (section->m_Area <= 0.0 || material->m_Density < 0.0)
        throw std::runtime_error("ElementCable mass properties must be nonnegative");
    double Iy = 0.0, Iz = 0.0, polarMoment = 0.0;
    section->Calculate_I(Iy, Iz, polarMoment);
    if (polarMoment < 0.0)
        throw std::runtime_error("ElementCable polar moment must not be negative");

    const double linearDensity = section->m_Area * material->m_Density;
    const double rotaryDensity = polarMoment * material->m_Density;
    constexpr double Sy = 0.0;
    constexpr double Sz = 0.0; // 当前只考虑质心与剪心重合的对称截面
    me.setZero(8, 8);

    MatrixXd mu = MatrixXd::Zero(4, 4);
    mu(0, 0) = mu(1, 1) = mu(2, 2) = linearDensity;
    mu(3, 3) = rotaryDensity;
    mu(1, 3) = mu(3, 1) = -Sy;
    mu(2, 3) = mu(3, 2) =  Sz;

    me.block<4, 4>(0, 0) = 2.0 * mu;
    me.block<4, 4>(0, 4) = mu;
    me.block<4, 4>(4, 4) = 2.0 * mu;
    me.block<4, 4>(4, 0) = mu;
    me *= (L0 / 6.0);
}

void ElementCable::Get_L0()
{
    auto pNode0 = m_pNode[0].lock();
    auto pNode1 = m_pNode[1].lock();
    if (!pNode0 || !pNode1)
        throw std::runtime_error("ElementCable node reference is invalid");

    const Eigen::Vector3d x0(pNode0->m_X, pNode0->m_Y, pNode0->m_Z);
    const Eigen::Vector3d x1(pNode1->m_X, pNode1->m_Y, pNode1->m_Z);
    L0 = (x1 - x0).norm();
    if (L0 <= 1.0e-12)
        throw std::runtime_error("ElementCable initial length must be positive");
    if (L <= 0.0) L = L0;

}

void ElementCable::Assemble(const std::vector<double>& damping, _OUT MatrixXd& ce)
{
    if (damping.size() < 4) 
    {
        // 处理错误：抛出异常或设置空矩阵并返回
        throw std::invalid_argument("damping vector must have at least 4 elements");
    }

    MatrixXd ke, me;
    Get_ke(ke);
    Get_me_Consistent(me);//一致质量矩阵

    ce = damping[0] * me + damping[1] * ke;

    // 平动对角块
    ce.block<3, 3>(0, 0) = damping[0] * me.block<3, 3>(0, 0) + damping[1] * ke.block<3, 3>(0, 0);
    ce.block<3, 3>(4, 4) = damping[0] * me.block<3, 3>(4, 4) + damping[1] * ke.block<3, 3>(4, 4);

    // 扭转对角块
    ce(3, 3) = damping[2] * me(3, 3) + damping[3] * ke(3, 3);
    ce(7, 7) = damping[2] * me(7, 7) + damping[3] * ke(7, 7);
    ce(3, 7) = damping[2] * me(3, 7) + damping[3] * ke(3, 7);
    ce(7, 3) = ce(3, 7);
}
