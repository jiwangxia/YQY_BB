#include "CR.h"
#include <cmath>

namespace Utility
{
namespace CR
{

// ========================================================================
// 1. 反对称矩阵
// ========================================================================
void SkewSymmetric(const Eigen::Vector3d& v, _OUT Eigen::Matrix3d& result)
{
    result << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
}

// ========================================================================
// 2. 计算 Ts 矩阵
// ========================================================================
void Calculate_Ts(const Eigen::Vector3d& vartheta, _OUT Eigen::Matrix3d& result)
{
    const double theta = vartheta.norm();
    double c1;
    double c2;

    // 泰勒展开避免极小角度下的除零和相消误差。
    if (theta < 1e-4)
    {
        const double t2 = theta * theta;
        const double t4 = t2 * t2;

        c1 = 0.5 - t2 / 24.0 + t4 / 720.0;
        c2 = 1.0 / 6.0 - t2 / 120.0 + t4 / 5040.0;
    }
    else
    {
        const double halfSine = std::sin(0.5 * theta);
        c1 = 2.0 * halfSine * halfSine / (theta * theta);
        c2 = (theta - std::sin(theta)) / (theta * theta * theta);
    }

    Eigen::Matrix3d skewRotation;
    SkewSymmetric(vartheta, skewRotation);
    result = Eigen::Matrix3d::Identity() + c1 * skewRotation + c2 * skewRotation * skewRotation;
}

// ========================================================================
// 3. 计算 Ts 的逆矩阵
// ========================================================================
void Calculate_Ts_Inv(const Eigen::Vector3d& vartheta, _OUT Eigen::Matrix3d& result)
{
    double theta = vartheta.norm();
    Eigen::Vector3d n;
    double c;

    // 泰勒展开防止极小角度下的除零奇异性
    if (theta < 1e-8)
    {
        n = Eigen::Vector3d(1, 0, 0);
        double t2 = theta * theta;
        double t4 = t2 * t2;

        c = 1.0 - t2 / 12.0 - t4 / 720.0;
    }
    else
    {
        n = vartheta / theta;
        double half_angle = theta / 2.0;
        c = (half_angle) / tan(half_angle);
    }

    // 调用刚才写的反对称矩阵函数
    Eigen::Matrix3d skew_v;
    SkewSymmetric(vartheta, skew_v);

    // 组装最终结果存入 result
    result = c * Eigen::Matrix3d::Identity() - 0.5 * skew_v + (1 - c) * (n * n.transpose());
}

// ========================================================================
// 4. 罗德里格斯公式 (向量 -> 矩阵)
// ========================================================================
void Calculate_RotationMatrix(const Eigen::Vector3d& vartheta, _OUT Eigen::Matrix3d& result)
{
    double theta = vartheta.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d skew_v;
    SkewSymmetric(vartheta, skew_v);

    if (theta < 1e-8)
    {
        // 泰勒展开防奇异
        double t2 = theta * theta;
        double c1 = 1.0 - t2 / 6.0 + (t2 * t2) / 120.0;
        double c2 = 0.5 - t2 / 24.0 + (t2 * t2) / 720.0;
        result = I + c1 * skew_v + c2 * (skew_v * skew_v);
    }
    else
    {
        // 精确公式
        double c1 = std::sin(theta) / theta;
        double c2 = (1.0 - std::cos(theta)) / (theta * theta);
        result = I + c1 * skew_v + c2 * (skew_v * skew_v);
    }
}

// ========================================================================
// 5. 计算随动坐标系 Rr (局部坐标系架设)
// ========================================================================
void Calculate_Rr(const Eigen::Vector3d& p1_def, const Eigen::Vector3d& p2_def, const Eigen::Vector3d& q,
                  _OUT Eigen::Matrix3d& result)
{
    // 1. 局部 x 轴 (r1)：沿弦线方向
    Eigen::Vector3d d = p2_def - p1_def;
    Eigen::Vector3d r1 = d.normalized();

    // 2. 局部 z 轴 (r3)：垂直于 r1 和 q 构成的平面
    Eigen::Vector3d r3_temp = r1.cross(q);
    double r3_norm = r3_temp.norm();

    // 数值保护：如果 r1 和 q 接近平行，叉乘结果接近零
    if (r3_norm < 1e-10)
    {
        // 选择一个与 r1 不平行的向量
        Eigen::Vector3d arbitrary = (std::abs(r1(0)) < 0.9) ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(0, 1, 0);
        r3_temp = r1.cross(arbitrary);
        r3_norm = r3_temp.norm();
    }

    Eigen::Vector3d r3 = r3_temp / r3_norm;

    // 3. 局部 y 轴 (r2)：利用右手定则闭合
    // 两个正交单位向量的叉乘必然是单位向量，无需再次 normalize()
    Eigen::Vector3d r2 = r3.cross(r1);

    // 4. 组装 Rr = [r1, r2, r3]
    result.col(0) = r1;
    result.col(1) = r2;
    result.col(2) = r3;
}

// ========================================================================
// 6. 对数映射 (矩阵 -> 向量)
// ========================================================================
void Extract_RotationVector(const Eigen::Matrix3d& R, _OUT Eigen::Vector3d& result)
{
    Eigen::Quaterniond q(R);
    q.normalize();

    // q 与 -q 表示同一姿态。固定到 w >= 0 后得到 [0, pi] 的主值，
    // 避免相邻 Newton 迭代因四元数换号造成转角跳变。
    if (q.w() < 0.0)
        q.coeffs() *= -1.0;

    const Eigen::Vector3d vectorPart = q.vec();
    const double vectorNorm = vectorPart.norm();
    if (vectorNorm < 1e-12)
    {
        result = 2.0 * vectorPart;
        return;
    }

    const double angle = 2.0 * std::atan2(vectorNorm, q.w());
    result = (angle / vectorNorm) * vectorPart;
}

Eigen::Vector3d CanonicalAxis(const Eigen::Vector3d& direction)
{
    const double length = direction.norm();
    if (!std::isfinite(length) || length <= 1.0e-14)
        return Eigen::Vector3d::UnitX();
    Eigen::Vector3d axis = direction / length;
    Eigen::Index dominant = 0;
    axis.cwiseAbs().maxCoeff(&dominant);
    if (axis[dominant] < 0.0)
        axis = -axis;
    return axis;
}

double ExtractAxialTwist(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& inputAxis)
{
    const Eigen::Vector3d axis = CanonicalAxis(inputAxis);
    Eigen::Quaterniond quaternion(rotation);
    quaternion.normalize();
    if (quaternion.w() < 0.0)
        quaternion.coeffs() *= -1.0;
    return 2.0 * std::atan2(quaternion.vec().dot(axis), quaternion.w());
}

// 7. 节点姿态增量更新 (乘法更新法则)
// R_new = exp(delta_theta) * R_old
// ========================================================================
void Update_NodalRotation(const Eigen::Vector3d& delta_theta, const Eigen::Matrix3d& R_old, _OUT Eigen::Matrix3d& R_new)
{
    Eigen::Matrix3d delta_R;
    // 1. 先用罗德里格斯公式把转角增量变成增量旋转矩阵
    Calculate_RotationMatrix(delta_theta, delta_R);

    // 2. 矩阵左乘更新姿态 (基于全局坐标系的增量)
    R_new = delta_R * R_old;
    // 每次 SVD 会抹去可表示的极小转角；仅在累计正交误差超限时修复姿态。
    if ((R_new.transpose() * R_new - Eigen::Matrix3d::Identity()).norm() > 1.0e-12 ||
        std::abs(R_new.determinant() - 1.0) > 1.0e-12)
        Orthonormalize_SVD(R_new);
}

// ========================================================================
// 9. 欧拉角转旋转矩阵 (以绕 X-Y-Z 顺序为例，对应 Roll-Pitch-Yaw)
// ========================================================================
void EulerAngles_To_RotationMatrix(const Eigen::Vector3d& eulerAngles, _OUT Eigen::Matrix3d& result)
{
    double cx = std::cos(eulerAngles(0));
    double sx = std::sin(eulerAngles(0));
    double cy = std::cos(eulerAngles(1));
    double sy = std::sin(eulerAngles(1));
    double cz = std::cos(eulerAngles(2));
    double sz = std::sin(eulerAngles(2));

    Eigen::Matrix3d Rx, Ry, Rz;
    Rx << 1.0, 0.0, 0.0, 0.0, cx, -sx, 0.0, sx, cx;

    Ry << cy, 0.0, sy, 0.0, 1.0, 0.0, -sy, 0.0, cy;

    Rz << cz, -sz, 0.0, sz, cz, 0.0, 0.0, 0.0, 1.0;

    // 连乘得到初始绝对姿态
    result = Rz * Ry * Rx;
}

// ========================================================================
// 组装三维梁专属的 Ba 矩阵 (7x7)
// 自由度排布: [u, tx1, ty1, tz1, tx2, ty2, tz2]
// ========================================================================
void Assemble_Matrix_Ba(const Eigen::Vector3d& vartheta1, const Eigen::Vector3d& vartheta2,
                        _OUT Eigen::MatrixXd& result)
{
    // 初始化为 7x7 的单位矩阵
    // 这保证了轴向自由度 u 的映射系数为 1.0
    result = Eigen::MatrixXd::Identity(7, 7);

    Eigen::Matrix3d Ts_Inv_1, Ts_Inv_2;

    // 计算两端的 Ts 逆矩阵
    Calculate_Ts_Inv(vartheta1, Ts_Inv_1);
    Calculate_Ts_Inv(vartheta2, Ts_Inv_2);

    // 塞入对应的自由度块 (索引 1 和 4)
    result.block<3, 3>(1, 1) = Ts_Inv_1;
    result.block<3, 3>(4, 4) = Ts_Inv_2;
}

// ========================================================================
// 计算单个节点的 3x3 局部几何刚度块 Khi
// ========================================================================
void Calculate_Khi(const Eigen::Vector3d& v, const Eigen::Vector3d& m, _OUT Eigen::Matrix3d& result)
{
    double theta = v.norm();
    double c1, c2, c3;

    double t2 = theta * theta;
    double t4 = t2 * t2;

    // 同样必须使用泰勒展开防止极小角度下的除零奇异性
    if (theta < 1e-8)
    {
        c1 = -1.0 / 6.0 - t2 / 180.0 - t4 / 5040.0;
        c2 = 1.0 / 12.0 + t2 / 720.0 + t4 / 30240.0;
        c3 = 1.0 / 360.0 + t2 / 7560.0 + t4 / 201600.0;
    }
    else
    {
        double sin_angle = sin(theta);
        double cos_angle = cos(theta);
        double sin_half = sin(theta * 0.5);
        double sin2_half = sin_half * sin_half;

        c1 = (sin_angle - theta) / (4.0 * theta * sin2_half);
        c2 = (2.0 * sin_angle - theta * (1.0 + cos_angle)) / (2.0 * t2 * sin_angle);
        c3 = (theta * (theta + sin_angle) - 8.0 * sin2_half) / (4.0 * t4 * sin2_half);
    }

    // 内力弯矩的反对称矩阵
    Eigen::Matrix3d skew_m;
    SkewSymmetric(m, skew_m);
    Eigen::RowVector3d vT = v.transpose();
    result = c1 * m * vT + c3 * vT * m * v * vT + c2 * (vT * m * Eigen::Matrix3d::Identity() + v * m.transpose()) -
             0.5 * skew_m;
    // 上式对局部旋转向量 v 求导；Newton 列是空间左乘微转，dv = Ts^{-1}(v) dphi。
    Eigen::Matrix3d spinInverse;
    Calculate_Ts_Inv(v, spinInverse);
    result *= spinInverse;
}

// ========================================================================
// 组装三维梁专属的 Kh 矩阵 (7x7)
// ========================================================================
void Assemble_Matrix_Kh(const Eigen::Vector3d& vartheta1, const Eigen::Vector3d& vartheta2, const Eigen::VectorXd& fl,
                        _OUT Eigen::MatrixXd& result)
{
    result = Eigen::MatrixXd::Zero(7, 7);

    // 从局部内力 fl 中精确提取两端节点的弯矩向量
    // fl(1,2,3) 对应节点 1；fl(4,5,6) 对应节点 2
    Eigen::Vector3d m1 = fl.segment<3>(1);
    Eigen::Vector3d m2 = fl.segment<3>(4);

    Eigen::Matrix3d Kh1, Kh2;
    Calculate_Khi(vartheta1, m1, Kh1);
    Calculate_Khi(vartheta2, m2, Kh2);

    // 填入对角块
    result.block<3, 3>(1, 1) = Kh1;
    result.block<3, 3>(4, 4) = Kh2;
}

// ========================================================================
// 组装三维梁专属的转换矩阵 E (12x12)
// ========================================================================
void Assemble_Matrix_E(const Eigen::Matrix3d& Rr, _OUT Eigen::MatrixXd& result)
{
    // 1. 初始化为 12x12 的单位矩阵 (对角线为 1)
    result = Eigen::MatrixXd::Identity(12, 12);

    // 2. 准备局部到全局的转置矩阵
    Eigen::Matrix3d Rr_T = Rr.transpose();

    // 3. 针对三维梁的 4 个空间向量自由度 (u1, theta1, u2, theta2)
    // 直接在索引 0, 3, 6, 9 处进行块覆盖 (比用 vector 循环速度更快)
    result.block<3, 3>(0, 0) = Rr_T;
    result.block<3, 3>(3, 3) = Rr_T;
    result.block<3, 3>(6, 6) = Rr_T;
    result.block<3, 3>(9, 9) = Rr_T;
}

// ========================================================================
// 组装投影矩阵 P (7x12) 和自旋矩阵 G (3x12)
// ========================================================================
void Assemble_Matrix_PG(const Eigen::Vector3d& p1_def, const Eigen::Vector3d& p2_def, const Eigen::Vector3d& q1,
                        const Eigen::Vector3d& q2, const Eigen::Matrix3d& Rr, _OUT Eigen::MatrixXd& P_result,
                        _OUT Eigen::MatrixXd& G_result)
{
    double Ln = (p2_def - p1_def).norm();

    // 将全局参考向量 q 转换到局部坐标系下
    Eigen::Matrix3d Rr_T = Rr.transpose();
    Eigen::Vector3d q1l = Rr_T * q1;
    Eigen::Vector3d q2l = Rr_T * q2;
    Eigen::Vector3d q = 0.5 * (q1l + q2l);

    double q_x = q(0);
    double q_y = q(1);
    if (std::abs(q_y) < 1e-12)
        q_y = (q_y < 0 ? -1e-12 : 1e-12);

    // 计算扭转自旋系数
    double eta = q_x / (Ln * q_y);
    Eigen::Vector3d nu1(q1l(1) / (2.0 * q_y), -q1l(0) / (2.0 * q_y), 0.0);
    Eigen::Vector3d nu2(q2l(1) / (2.0 * q_y), -q2l(0) / (2.0 * q_y), 0.0);

    G_result = Eigen::MatrixXd::Zero(3, 12);

    // G_u1: 节点1平移带来的自旋
    G_result(0, 2) = eta;
    G_result(1, 2) = 1.0 / Ln;
    G_result(2, 1) = -1.0 / Ln;
    G_result(1, 8) = -1.0 / Ln;
    G_result(2, 7) = 1.0 / Ln;
    G_result.block<3, 3>(0, 6) = -G_result.block<3, 3>(0, 0);

    G_result(0, 3) = nu1(0);
    G_result(0, 4) = nu1(1);
    G_result(0, 8) = -eta;
    G_result(0, 9) = nu2(0);
    G_result(0, 10) = nu2(1);

    P_result = Eigen::MatrixXd::Zero(6, 12);

    P_result.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    P_result.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();

    // 减去刚体自旋部分！纯变形 = 总变形 - 物理自旋
    P_result.block<3, 12>(0, 0) -= G_result;
    P_result.block<3, 12>(3, 0) -= G_result;
}

void Assemble_stress_k(double L, const Eigen::VectorXd& fa, const Eigen::MatrixXd& G, const Eigen::MatrixXd& P,
                       const Eigen::Matrix3d& Rr, const Eigen::Vector3d& q1_global, const Eigen::Vector3d& q2_global,
                       const Eigen::Vector3d& r1, _OUT Eigen::MatrixXd& result)
{
    Eigen::Matrix3d Rr_T = Rr.transpose();
    Eigen::Vector3d q1_l = Rr_T * q1_global;
    Eigen::Vector3d q2_l = Rr_T * q2_global;
    Eigen::Vector3d q_l = 0.5 * (q1_l + q2_l);

    double q_x = q_l(0);
    double q_y = q_l(1);
    if (std::abs(q_y) < 1e-12)
        q_y = (q_y < 0 ? -1e-12 : 1e-12);

    const double q1x = q1_l(0);
    const double q2x = q2_l(0);

    const double eta = q_x / q_y;

    Eigen::MatrixXd E = Eigen::MatrixXd::Zero(12, 12);
    E.block<3, 3>(0, 0) = Rr;
    E.block<3, 3>(3, 3) = Rr;
    E.block<3, 3>(6, 6) = Rr;
    E.block<3, 3>(9, 9) = Rr;
    Eigen::MatrixXd ET = E.transpose();

    // Km1
    const double N = fa(0);
    Eigen::MatrixXd Km1 = Eigen::MatrixXd::Zero(12, 12);
    Eigen::Matrix3d D3 = (N / L) * (Eigen::Matrix3d::Identity() - r1 * r1.transpose());
    Km1.block<3, 3>(0, 0) = D3;
    Km1.block<3, 3>(0, 6) = -D3;
    Km1.block<3, 3>(6, 0) = -D3;
    Km1.block<3, 3>(6, 6) = D3;

    // Km2
    VectorXd m(6);
    m << fa(1), fa(2), fa(3), fa(4), fa(5), fa(6);

    Eigen::VectorXd PTm = P.transpose() * m; // 12x1 结果

    Vector3d n1 = PTm.segment<3>(0);
    Vector3d m1 = PTm.segment<3>(3);
    Vector3d n2 = PTm.segment<3>(6);
    Vector3d m2 = PTm.segment<3>(9);

    Matrix3d skew_n1, skew_m1, skew_n2, skew_m2;
    SkewSymmetric(n1, skew_n1);
    SkewSymmetric(m1, skew_m1);
    SkewSymmetric(n2, skew_n2);
    SkewSymmetric(m2, skew_m2);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(12, 3);
    Q.block<3, 3>(0, 0) = skew_n1;
    Q.block<3, 3>(3, 0) = skew_m1;
    Q.block<3, 3>(6, 0) = skew_n2;
    Q.block<3, 3>(9, 0) = skew_m2;

    Eigen::MatrixXd Km2 = E * Q * G * E.transpose();

    // Km3
    Eigen::VectorXd Ga = VectorXd::Zero(12);
    Ga(1) = -(fa(3) + fa(6)) / L / L;
    Ga(2) = (fa(1) + fa(4)) / L / L * eta + (fa(2) + fa(5)) / L / L;
    Ga(7) = (fa(3) + fa(6)) / L / L;
    Ga(8) = -(fa(1) + fa(4)) / L / L * eta - (fa(2) + fa(5)) / L / L;

    VectorXd r = VectorXd::Zero(12);
    r.segment<3>(0) = -r1;
    r.segment<3>(6) = r1;

    Eigen::MatrixXd Km3 = E * Ga * r.transpose(); // 12x12

    // G 的第一行还依赖两端截面方向；Km3 只包含其长度导数。
    // 以 t=q_x/q_y、h=(q1_x-q2_x)/(2q_y) 分解方向变化，
    // 固定局部弯矩合力 T 时遗漏的 -E(dG)^T T 是两个秩一项。
    const double torsionalMomentSum = fa(1) + fa(4);
    if (torsionalMomentSum != 0.0 && std::abs(q_l(1)) >= 1e-12)
    {
        const double t = eta;
        const double h = (q1x - q2x) / (2.0 * q_y);
        const double c = (q1_l.z() - q2_l.z()) / 2.0;
        using Vector12 = Eigen::Matrix<double, 12, 1>;
        Vector12 vt = Vector12::Zero();
        Vector12 vh = Vector12::Zero();
        Vector12 gt = Vector12::Zero();
        Vector12 gh = Vector12::Zero();
        vt(2) = 1.0 / L;
        vt(8) = -1.0 / L;
        vt(3) = -h / 2.0;
        vt(4) = -0.5;
        vt(9) = h / 2.0;
        vt(10) = -0.5;
        vh(3) = -t / 2.0;
        vh(4) = -0.5;
        vh(9) = t / 2.0;
        vh(10) = 0.5;
        gt(1) = -(1.0 + t * t) / L;
        gt(7) = -gt(1);
        gt(3) = c * t / (2.0 * q_y);
        gt(4) = c / (2.0 * q_y);
        gt(5) = -(1.0 + t * t) / 2.0;
        gt(9) = -gt(3);
        gt(10) = -gt(4);
        gt(11) = gt(5);
        gh(2) = -c / (q_y * L);
        gh(8) = -gh(2);
        gh(3) = c * h / (2.0 * q_y);
        gh(4) = c / (2.0 * q_y);
        gh(5) = -(1.0 + h * h) / 2.0;
        gh(9) = -gh(3);
        gh(10) = gh(4);
        gh(11) = -gh(5);
        const Vector12 vtGlobal = E * vt;
        const Vector12 vhGlobal = E * vh;
        const Vector12 gtGlobal = E * gt;
        const Vector12 ghGlobal = E * gh;
        Km3.noalias() -= torsionalMomentSum *
                         (vtGlobal * gtGlobal.transpose() + vhGlobal * ghGlobal.transpose());
    }

    result = Km1 - Km2 + Km3;
}

void Orthonormalize_SVD(_OUT Eigen::Matrix3d& R)
{
    // 使用SVD方法对旋转矩阵进行正交化，确保其为合法的旋转矩阵
    // ComputeFullU | ComputeFullV 表示我们需要计算 U 和 V 矩阵
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // 重构正交矩阵
    Eigen::Matrix3d R_ortho = svd.matrixU() * svd.matrixV().transpose();

    // 安全检查：确保行列式为 +1（防止因为数值噪音变成反射矩阵，即 det=-1）
    if (R_ortho.determinant() < 0)
    {
        // 如果变成了镜像，反转最后一列（这种情况在微小漂移中极少发生，但在崩坏时可能出现）
        Eigen::Matrix3d U = svd.matrixU();
        U.col(2) *= -1.0;
        R_ortho = U * svd.matrixV().transpose();
    }

    R = R_ortho;
}
}
}
