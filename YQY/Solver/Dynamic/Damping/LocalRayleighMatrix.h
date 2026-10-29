#pragma once

#include "Base/EmptyOUT.h"

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace SolverNameSpace
{
/** @brief 由完整对称质量、刚度构造比例阻尼，保留单元平移和转动的耦合。 */
inline void BuildProportionalRayleighMatrix(const Eigen::MatrixXd& stiffness, const Eigen::MatrixXd& mass,
                                           double alpha, double beta, _OUT Eigen::MatrixXd& damping)
{
    if (mass.rows() != mass.cols() || stiffness.rows() != mass.rows() || stiffness.cols() != mass.cols())
        throw std::invalid_argument("Proportional Rayleigh matrix dimensions do not match");
    if (!std::isfinite(alpha) || !std::isfinite(beta) || alpha < 0.0 || beta < 0.0)
        throw std::invalid_argument("Proportional Rayleigh coefficients must be finite and nonnegative");
    if (!mass.allFinite() || !stiffness.allFinite())
        throw std::invalid_argument("Proportional Rayleigh mass and stiffness must be finite");
    damping = (0.5 * alpha * (mass + mass.transpose()) +
               0.5 * beta * (stiffness + stiffness.transpose())).eval();
    if (!damping.allFinite())
        throw std::overflow_error("Proportional Rayleigh damping contains nonfinite values");
}

/** @brief 根据平移和转动自由度的对称质量、刚度子块构造单元阻尼。 */
inline void BuildLocalRayleighMatrix(const Eigen::MatrixXd& stiffness, const Eigen::MatrixXd& mass,
                                     const std::vector<bool>& rotational, const std::array<double, 4>& coefficients,
                                     _OUT Eigen::MatrixXd& damping)
{
    const Eigen::Index size = mass.rows();
    if (mass.cols() != size || stiffness.rows() != size || stiffness.cols() != size ||
        rotational.size() != static_cast<std::size_t>(size))
        throw std::invalid_argument("Local Rayleigh matrix dimensions do not match the DOF layout");
    for (double coefficient : coefficients)
    {
        if (!std::isfinite(coefficient) || coefficient < 0.0)
            throw std::invalid_argument("Local Rayleigh coefficients must be finite and nonnegative");
    }
    if (!mass.allFinite() || !stiffness.allFinite())
        throw std::invalid_argument("Local Rayleigh mass and stiffness must be finite");

    damping.setZero(size, size);
    for (Eigen::Index row = 0; row < size; ++row)
    {
        const bool rowRotation = rotational[static_cast<std::size_t>(row)];
        const double alpha = coefficients[rowRotation ? 2 : 0];
        const double beta = coefficients[rowRotation ? 3 : 1];
        for (Eigen::Index column = row; column < size; ++column)
        {
            if (rowRotation != rotational[static_cast<std::size_t>(column)])
                continue;
            const double value = alpha * 0.5 * (mass(row, column) + mass(column, row)) +
                                 beta * 0.5 * (stiffness(row, column) + stiffness(column, row));
            damping(row, column) = value;
            damping(column, row) = value;
        }
    }
    if (!damping.allFinite())
        throw std::overflow_error("Local Rayleigh damping contains nonfinite values");
}
}
