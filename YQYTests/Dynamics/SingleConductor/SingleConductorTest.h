#pragma once

#include <QStringList>
#include <optional>

/** @brief 执行单导线固定或自适应 Newmark 的完整静力继承及动力计算
 * @details 从冻结 HDF5 输入读取单导线模型，使用生产 AnalysisRunner 求解静力和指定时长的动力。
 *          保存真实接受步时程、初态、计算参数和耗时；返回成功只证明完整计算，数值精度与舞动
 *          判定仍需与独立参考和无气动对照比较，不将求解完成单独登记为算法通过。
 * @param arguments 显式算例参数；未请求本入口时返回空值
 * @return 完整计算返回0，失败返回非零
 * @author   yangqiuyou
 * @date     2026.10.26
 */
std::optional<int> VerifySingleConductorNewmark(const QStringList& arguments);
