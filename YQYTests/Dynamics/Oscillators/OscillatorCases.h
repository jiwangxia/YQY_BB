#pragma once

#include <QJsonObject>
#include <QStringList>
#include <optional>

namespace NewmarkLiterature
{
/** @brief 直接运行工厂 Newmark 文献物理模型；只报告完成状态，数值验收由调用方执行。 */
QJsonObject RunCase(const QString& name, const QString& outputDirectory, double timeStep, double duration,
                    double tolerance);
}

/** @brief 分派 Newmark 专属文献算例；未请求该入口时返回空值。 */
std::optional<int> VerifyNewmarkLiterature(const QStringList& arguments);
