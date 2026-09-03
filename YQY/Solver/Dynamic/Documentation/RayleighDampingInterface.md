# Rayleigh 阻尼参数接口

当前接口只负责由两个参考频率和目标阻尼比计算并输出参数，不依赖静力步、动力步、矩阵组装或 Newmark 求解器。

## 调用方式

```cpp
#include "Solver/Dynamic/Damping/RayleighDamping.h"

const auto result = SolverNameSpace::SolveRayleighDamping(
    0.5,   // f1，Hz
    5.0,   // f2，Hz
    0.02); // 两个频率点均为 2% 阻尼比

qDebug().noquote() << QString::fromStdString(result.ToText());

const double y1 = result.coefficients.alphaM;
const double y2 = result.coefficients.betaK;
```

不同目标阻尼比使用四参数重载：

```cpp
const auto result = SolverNameSpace::SolveRayleighDamping(
    0.5, 0.02, // f1、zeta1
    5.0, 0.03  // f2、zeta2
);
```

接口内部会完成 `omega = 2*pi*f` 的单位转换，并拒绝零频率、相同频率、负阻尼比以及会产生负 Rayleigh 系数的不合理输入。

## 后续接入点

模态分析完成后，只需把选中的两个非零模态频率传给此接口。将来动力模块可读取
`result.coefficients.alphaM` 和 `result.coefficients.betaK`，再构造
`C = alphaM*M + betaK*K`；本接口本身不执行这一过程。

输电线若需要分别拟合平动和扭转阻尼，可分别调用两次：一次传平动主导模态频率，另一次传扭转主导模态频率。
