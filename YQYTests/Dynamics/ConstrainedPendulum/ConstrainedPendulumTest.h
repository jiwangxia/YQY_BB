#pragma once

#include <QStringList>
#include <optional>

/** @brief 显式运行 Betsch（2005）双球面摆的生产 MPC 动力回归并导出接受态。 */
std::optional<int> VerifyBetschConstraintDynamics(const QStringList& arguments);
