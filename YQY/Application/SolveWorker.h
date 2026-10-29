#pragma once

#include <QStringList>

// 由应用入口调用，与桌面控制器独立的无界面求解工作进程。
int RunIsolatedSolveWorker(const QStringList& arguments);
