/**
 * @file ISolver.h
 * @brief 求解器统一接口
 * 
 * 所有求解器（静力、动力）都实现此接口，
 * 调用方式统一：solver.Solve(model, duration)
 */
#pragma once
#include "IAnalysisModel.h"
#include <memory>

namespace SolverNS
{
    /**
     * @brief 求解器类型枚举
     * 用于工厂模式创建求解器
     */
    enum class SolverType
    {
        Static,             ///< 静力 Newton-Raphson
        Newmark,            ///< Newmark-β 隐式动力
        TSSBN,              ///< TSSBN 隐式动力（两阶段单步方法）
        AdaptiveTSSBN,      ///< 自适应TSSBN 隐式动力（带误差估计和步长控制）
        CentralDifference,  ///< 中心差分 显式动力（未来扩展）
        HHT                 ///< HHT-α 隐式动力（未来扩展）
    };

    /**
     * @brief 求解器基类接口
     * 
     * 所有求解器都继承此接口，提供统一的调用方式。
     */
    class ISolver
    {
    public:
        virtual ~ISolver() = default;

        /**
         * @brief 执行求解
         * @param[in,out] model 实现 IAnalysisModel 的对象
         * @param[in] duration 总时长（静力=加载时间，动力=模拟时长）
         * @return 是否成功收敛
         */
        virtual bool Solve(IAnalysisModel& model, double duration) = 0;

        /**
         * @brief 获取求解器类型名称
         */
        virtual const char* GetName() const = 0;

        /**
         * @brief 获取求解器类型
         */
        virtual SolverType GetType() const = 0;

        /**
         * @brief 设置步回调函数
         * @param[in] callback 每步结束时调用的回调
         */
        virtual void SetStepCallback(StepCallback callback) = 0;
    };

    // 前向声明求解器工厂函数
    std::unique_ptr<ISolver> CreateSolver(SolverType type, double stepSize = 0.01, 
                                          double tolerance = 1e-5, int maxIterations = 32);
}
