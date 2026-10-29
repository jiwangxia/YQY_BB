#pragma once

#include <QStringList>
#include <optional>

std::optional<int> VerifySpatialBeamDynamics(const QStringList& arguments);
/** @brief 用生产固定或自适应RK4运行真实CR空间双梁动力，并导出全部接受状态。 */
std::optional<int> VerifyRungeKuttaSpatial(const QStringList& arguments);
/** @brief 用工厂普通或Reference TSSBN策略运行同一纯结构CR空间双梁，导出全部接受状态。 */
std::optional<int> VerifyTssbnSpatial(const QStringList& arguments);
