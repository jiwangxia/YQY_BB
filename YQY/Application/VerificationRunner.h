#pragma once

#include <optional>

class QApplication;

// 程序内置验证算例的统一入口。
class VerificationRunner final
{
public:
    // 运行不依赖主窗口的验证项。
    static std::optional<int> runHeadless(const QStringList& arguments);
    // 运行需要 QApplication 的验证项。
    static std::optional<int> run(QApplication& application, const QStringList& arguments);
};
