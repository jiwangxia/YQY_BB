#include "ElementSpringA.h"

#include <stdexcept>

ElementSpringA::ElementSpringA()
{
    m_pNode.resize(2);
}

void ElementSpringA::Get_ke(_OUT MatrixXd& ke)
{
    const auto firstNode = m_pNode[0].lock();
    const auto secondNode = m_pNode[1].lock();
    if (!firstNode || !secondNode)
        throw std::runtime_error("SPRINGA node is invalid.");
    if (firstNode->m_Displacement.size() < 3 || secondNode->m_Displacement.size() < 3)
        throw std::runtime_error("SPRINGA node displacement is invalid.");

    const Vector3d firstPosition(firstNode->m_X + firstNode->m_Displacement[0],
                                 firstNode->m_Y + firstNode->m_Displacement[1],
                                 firstNode->m_Z + firstNode->m_Displacement[2]);
    const Vector3d secondPosition(secondNode->m_X + secondNode->m_Displacement[0],
                                  secondNode->m_Y + secondNode->m_Displacement[1],
                                  secondNode->m_Z + secondNode->m_Displacement[2]);
    const Vector3d currentAxis = secondPosition - firstPosition;
    const double currentLength = currentAxis.norm();
    constexpr double lengthTolerance = 1.0e-12;
    if (currentLength <= lengthTolerance)
        throw std::runtime_error("SPRINGA current length is too small.");

    if (L0 <= lengthTolerance)
        Get_L0();
    if (L0 <= lengthTolerance)
        throw std::runtime_error("SPRINGA initial length is too small.");

    const Vector3d direction = currentAxis / currentLength;
    double force = 0.0;
    double tangent = 0.0;
    EvaluateBehavior(currentLength - L0, force, tangent);

    const Matrix3d directionProjection = direction * direction.transpose();
    const Matrix3d transverseProjection = Matrix3d::Identity() - directionProjection;
    const Matrix3d block = tangent * directionProjection + force / currentLength * transverseProjection;
    ke = MatrixXd::Zero(6, 6);
    ke.block<3, 3>(0, 0) = block;
    ke.block<3, 3>(0, 3) = -block;
    ke.block<3, 3>(3, 0) = -block;
    ke.block<3, 3>(3, 3) = block;
    m_inforce = VectorXd::Zero(6);
    m_inforce.segment<3>(0) = -force * direction;
    m_inforce.segment<3>(3) = force * direction;
}

void ElementSpringA::Get_L0()
{
    const auto firstNode = m_pNode[0].lock();
    const auto secondNode = m_pNode[1].lock();
    if (!firstNode || !secondNode)
        throw std::runtime_error("SPRINGA node is invalid.");
    const Vector3d axis(secondNode->m_X - firstNode->m_X, secondNode->m_Y - firstNode->m_Y,
                        secondNode->m_Z - firstNode->m_Z);
    L0 = axis.norm();
}
