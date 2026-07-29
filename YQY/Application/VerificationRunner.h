#pragma once

#include <optional>

class QApplication;

class VerificationRunner final
{
public:
    static std::optional<int> runHeadless(const QStringList& arguments);
    static std::optional<int> run(QApplication& application, const QStringList& arguments);
};
