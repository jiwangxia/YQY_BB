#pragma once

#include <QStringList>
#include <optional>

// 真实索单元、生产 Newmark 与冻结独立参考的时程验证。
std::optional<int> VerifyNewmarkCable(const QStringList& arguments);
