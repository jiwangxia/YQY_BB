#pragma once

#include <QStringList>

namespace RuntimeOptions
{
// 主程序和测试子进程使用相同的求解及结果输出参数。
void Apply(int argc, char* argv[]);
// 提取桌面启动的模型路径；双横线之后允许路径以横线开头。
QStringList ModelFiles(const QStringList& arguments);
}
