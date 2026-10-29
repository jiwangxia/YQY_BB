#pragma once

#include <QJsonObject>
#include <QStringList>
#include <optional>

namespace NewmarkBeamBending
{
/** @brief 运行实际 CR 梁微幅弯曲；可选容差仅供显式诊断，默认冻结验收设置不变。 */
QJsonObject RunCase(const QString& directory, bool damped, int level,
                    std::optional<double> diagnosticTolerance = std::nullopt, bool adaptive = false, bool rk4 = false);
/** @brief 运行已冻结的无阻尼及正式模态阻尼六组并判定完整状态误差与阶数。 */
QJsonObject RunSuite(const QString& directory, bool rk4 = false);
}

/** @brief 分派真实梁弯曲验证入口。 */
std::optional<int> VerifyNewmarkBeamBending(const QStringList& arguments);
