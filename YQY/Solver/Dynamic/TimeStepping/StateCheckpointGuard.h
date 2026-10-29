/**
 * @file StateCheckpointGuard.h
 * @brief 自适应时间步试算使用的模型状态检查点守卫。
 */
#pragma once

#include "Solver/Interface/IAnalysisModel.h"

#include <QDebug>
#include <exception>

namespace SolverNameSpace
{
/**
 * @brief 保证成功创建的检查点只释放一次。
 *
 * Restore 保留检查点，便于同一初态继续执行两半步；Release 提交并释放检查点。
 * 可选退出回滚用于静力增量，提前失败或异常时恢复最近已提交状态。
 */
class StateCheckpointGuard final
{
public:
    explicit StateCheckpointGuard(IAnalysisModel& model, bool restoreOnExit = false)
        : model_(model)
        , active_(model.PushStateCheckpoint())
        , restoreOnExit_(restoreOnExit)
    {
    }

    ~StateCheckpointGuard()
    {
        try
        {
            if (active_ && restoreOnExit_)
            {
                if (model_.RestoreStateCheckpoint())
                    model_.UpdateInternalForce();
                else
                    qCritical() << "求解失败后的模型检查点恢复失败";
            }
        }
        catch (const std::exception& error)
        {
            qCritical() << "求解失败后的模型状态刷新失败:" << error.what();
        }
        catch (...)
        {
            qCritical() << "求解失败后的模型状态刷新发生未知异常";
        }
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
    bool restoreOnExit_ = false;
};
}
