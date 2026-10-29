#pragma once

#include <QStringList>
#include <optional>

// 仅分派静力、动力物理算例；未匹配时返回 nullopt。
std::optional<int> RunNumericalCase(const QStringList& arguments);
