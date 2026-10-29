#pragma once

#include <QStringList>
#include <optional>

/** @brief 分派真实三杆 Newmark 动力全时程验收；未请求该入口时返回空值。 */
std::optional<int> VerifyNewmarkTruss(const QStringList& arguments);
