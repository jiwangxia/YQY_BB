/**
 * @file StateCheckpointGuard.h
 * @brief 自适应时间步试算使用的模型状态检查点守卫。
 */
#pragma once

#include "Solver/Interface/IAnalysisModel.h"

namespace SolverNameSpace
{
/**
 * @brief 保证成功创建的检查点只释放一次。
 *
 * Restore 只恢复状态并保留检查点，便于同一初态继续执行两半步；Release 在接受或拒绝路径结束时释放检查点。
 */
class StateCheckpointGuard final
{
public:
    explicit StateCheckpointGuard(IAnalysisModel& model)
        : model_(model)
        , active_(model.PushStateCheckpoint())
    {
    }

    ~StateCheckpointGuard()
    {
        Release();
    }

    StateCheckpointGuard(const StateCheckpointGuard&) = delete;
    StateCheckpointGuard& operator=(const StateCheckpointGuard&) = delete;

    bool IsActive() const
    {
        return active_;
    }

    bool Restore()
    {
        return active_ && model_.RestoreStateCheckpoint();
    }

    void Release()
    {
        if (!active_)
            return;
        model_.DiscardStateCheckpoint();
        active_ = false;
    }

private:
    IAnalysisModel& model_;
    bool active_ = false;
};
}
