#pragma once

#include <QJsonObject>
#include <QStringList>
#include <optional>

namespace NewmarkBeamNonlinear
{
/** @brief 真实平面 CR 梁有限幅值验收；可选容差仅用于另命名诊断，默认冻结设置不变。 */
QJsonObject RunCase(const QString& directory, int level, std::optional<double> diagnosticTolerance = std::nullopt,
                    bool adaptive = false, bool rk4 = false);
/** @brief 运行独立冻结的三个时间步层级并检查全部状态的二阶收敛。 */
QJsonObject RunSuite(const QString& directory, bool rk4 = false);
}

/** @brief 分派真实平面 CR 梁有限幅值验证入口。 */
std::optional<int> VerifyNewmarkBeamNonlinear(const QStringList& arguments);
