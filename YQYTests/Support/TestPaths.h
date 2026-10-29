#pragma once

#include <QString>

namespace TestPaths
{
// 从测试程序所在目录向上查找仓库，避免依赖 VS 设置的工作目录。
QString RepositoryRoot();
QString TestRoot();
// 返回并创建指定验证项的输出目录。
QString VerificationOutputDirectory(const QString& verificationName);
}
