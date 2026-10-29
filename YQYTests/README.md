# 物理算例验收

测试代码集中在 `YQYTests`，按静力／动力和物理算例组织；生产 `YQY` 提供函数实现。测试链接同配置的 `YQYRuntime.lib`，复用生产读取、单元、装配和求解器，不复制数值算法。

## 目录

| 目录 | 内容 |
| --- | --- |
| `Statics`、`Dynamics` | 正式物理算例的 C++ 测试 |
| `Runner` | 统一 GoogleTest 和显式算例入口 |
| `Support` | 模型读取、求解、比较和路径定位辅助 |
| [TestModels](TestModels/README.md) | C++ 测试需要的模型、独立参考和验收参数 |
| `Pending` | 未完成、历史及诊断源码，不参与默认编译 |

算例生成、独立比较、绘图和批量运行的 Python 脚本及缓存已移除。生产目录的 `YQY/Tools/result_plot.py` 是界面实际调用的结果绘图实现，仍保留。

`input.json`、`reference.json`、`validation-plan.json` 等文件中，凡被 C++ 测试直接读取的模型参数、参考数值和误差门都属于必要测试输入，不能按文件扩展名当作缓存删除。算例登记见 [cases.json](cases.json)。

## 运行

在含 `YQY.sln` 的仓库根目录构建同配置的生产与测试工程后执行：

```powershell
.\x64\Release\YQYTests.exe --gtest_list_tests
.\x64\Release\YQYTests.exe --list-cases
.\x64\Release\YQYTests.exe
.\x64\Release\YQYTests.exe --gtest_filter=CantileverBeam.*
```

耗时算例使用显式 C++ 入口，例如：

```powershell
.\x64\Release\YQYTests.exe --verify-battini-helical-beam <新的输出目录>
.\x64\Release\YQYTests.exe --verify-spatial-beam-dynamics <新的输出目录>
```

输出不写回模型目录。部分显式入口只求解和导出，退出码为 0 不代表已完成独立参考比较；原 Python 比较工具已移除，不再提供其执行命令。

## 验收范围与已知限制

- 每项明确物理模型、参考来源、算法、参数、比较量和预设误差门。静力加载进度与动力时间、闭式解与文献读图、同模型细化与外部真值分别说明。
- 默认测试直接调用生产流程并比较独立参考；读取成功、格式正确、文件往返和仅求解完成不单独计作力学通过。
- 2026-10-29 按用户要求删除 `PrestressedCable.NewmarkFixed` 与 `NonlinearCable.NewmarkFixed` 两项默认 GoogleTest；共享显式 CLI 仍保留。`TestModels/Dynamics/CableVibration` 缺少原始模型、独立参考 CSV 和 `validation-plan.json`，显式入口当前仍不能执行，登记为 `blocked_missing_input`。
- 删除后默认 GoogleTest 为 15 项，Debug、Release 完整运行均为 15 项通过、0 项失败。Debug 解决方案与 Release 测试工程均构建成功，零警告、零错误；Release 主程序正在运行，完整解决方案链接因 EXE 占用未完成，测试工程使用现有同配置生产库单独重建。证据位于 `C:/APP/VS/YQY_CAE/tmp/remove-two-cable-tests-20261029/Debug.xml` 和 `Release.xml`；删除测试不构成这两个索算例通过。
- 历史记录（2026-10-29，本次删除前）：清理后的 Release 构建为零警告、零错误；清理前后默认 17 项测试逐项状态一致，15 项通过，上述 2 项因缺失输入失败。原失败记录保持不变。
- 单导线 300 s 历史候选未过时间细化响应门；梁自适应提前失败，后续短程修复证据不代表完整长期验收，见 [单导线说明](TestModels/Dynamics/SingleConductor/README.md)。

历史记录（2026-10-29，本次删除前）的四项能量输出补充验证：轴向梁增加沿轴重力工况，独立核对动能、应变能、重力势能及其和，
同时检查整文件和流式 HDF5 的能量回读；三类非线性弹簧按分段力—位移曲线解析积分核对应变能，
桁架按对数应变势能核对；平面梁大小振幅算例核对新接口与物理质量、既有势能的结果一致，
并检查能量查询不改变节点、内力和应力状态。补充检查写在原物理算例内，不增加读取或存储机制测试项。
该次完整运行为 17 项中 15 项通过，上述两项索测试因原有输入缺失失败；
证据位于 `C:/APP/VS/YQY_CAE/energy-validation/physical-tests.xml`。

历史资料中的旧可执行文件、输出目录和实验脚本只描述当时运行，不能当作当前入口；已删除的 `tmp`、`output` 和归档产物不保证仍可访问。
