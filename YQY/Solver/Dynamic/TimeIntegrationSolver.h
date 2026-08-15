#pragma once
#include "Base/EmptyOUT.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <functional>
#include <vector>

using namespace Eigen;

/**
 * @brief 时间积分求解器基类
 * 提供Newmark、TSSBN、自适应TSSBN三种方法的独立实现
 */
class TimeIntegrationSolver
{
public:
    // 状态向量结构
    struct State
    {
        VectorXd u; // 位移
        VectorXd v; // 速度
        VectorXd a; // 加速度
        double t;   // 时间
    };

    // 系统函数接口
    using MassMatrixFunc = std::function<void(SparseMatrix<double>&)>;
    using StiffnessMatrixFunc = std::function<void(SparseMatrix<double>&, const State&)>;
    using ForceFunc = std::function<void(VectorXd&, const State&)>;
    using UpdatePropertiesFunc = std::function<void(const State&)>;

    TimeIntegrationSolver(int ndof)
        : m_ndof(ndof)
    {
    }
    virtual ~TimeIntegrationSolver() = default;

    // 设置系统函数
    void setMassMatrix(MassMatrixFunc func)
    {
        m_getMass = func;
    }
    void setStiffnessMatrix(StiffnessMatrixFunc func)
    {
        m_getStiffness = func;
    }
    void setExternalForce(ForceFunc func)
    {
        m_getExternalForce = func;
    }
    void setInternalForce(ForceFunc func)
    {
        m_getInternalForce = func;
    }
    void setUpdateProperties(UpdatePropertiesFunc func)
    {
        m_updateProperties = func;
    }

    // 求解接口
    virtual bool solve(_OUT State& state, double dt, double t_end) = 0;

protected:
    int m_ndof;
    MassMatrixFunc m_getMass;
    StiffnessMatrixFunc m_getStiffness;
    ForceFunc m_getExternalForce;
    ForceFunc m_getInternalForce;
    UpdatePropertiesFunc m_updateProperties;
};

/**
 * @brief Newmark方法求解器
 * 经典的隐式时间积分方法，无条件稳定（β=0.25, γ=0.5）
 */
class NewmarkSolver : public TimeIntegrationSolver
{
public:
    NewmarkSolver(int ndof, double beta = 0.25, double gamma = 0.5)
        : TimeIntegrationSolver(ndof)
        , m_beta(beta)
        , m_gamma(gamma)
    {
    }

    bool solve(_OUT State& state, double dt, double t_end) override;

private:
    double m_beta;
    double m_gamma;
};

/**
 * @brief TSSBN方法求解器
 * Two-Stage Single-Step Block Newmark方法
 */
class TSSBNSolver : public TimeIntegrationSolver
{
public:
    struct Params
    {
        double rho_inf = 0.9; // 高频耗散参数
        // 以下参数由rho_inf自动计算
        double c1, c2, alpha, b1;
    };

    TSSBNSolver(int ndof, const Params& params = Params())
        : TimeIntegrationSolver(ndof)
        , m_params(params)
    {
        calculateParameters();
    }

    bool solve(_OUT State& state, double dt, double t_end) override;

    const Params& getParams() const
    {
        return m_params;
    }

private:
    Params m_params;

    // 中间状态
    State m_state_c1;
    State m_state_c2;

    void calculateParameters();
};

/**
 * @brief 自适应TSSBN方法求解器
 * 带误差估计和自适应步长控制的TSSBN方法
 */
class AdaptiveTSSBNSolver : public TimeIntegrationSolver
{
public:
    struct Params
    {
        // TSSBN参数
        double rho_inf = 0.9;
        double c1, c2, alpha, b1;

        // 嵌入式公式参数
        double c3_hat = 1.0;
        double b1_hat, b2_hat, b3_hat;
        double a31_hat, a32_hat;

        // 步长控制参数
        double dt_min = 1e-6;
        double dt_max = 0.1;
        double eps_LTE = 1e-3;     // 局部截断误差容限
        double S = 0.9;            // 安全因子
        double p_order = 2.0;      // 方法阶数
        double gamma_grow = 1.5;   // 步长增长因子
        double gamma_shrink = 0.5; // 步长缩小因子
        int N_target = 5;          // 目标迭代次数
        int N_max = 20;            // 最大迭代次数
        double k_d = 0.2;          // 数字滤波系数
        double FacD_min = 0.5;
        double FacD_max = 2.0;
    };

    AdaptiveTSSBNSolver(int ndof, const Params& params = Params())
        : TimeIntegrationSolver(ndof)
        , m_params(params)
    {
        calculateParameters();
    }

    bool solve(_OUT State& state, double dt, double t_end) override;

    const Params& getParams() const
    {
        return m_params;
    }

    // 获取统计信息
    int getTotalSteps() const
    {
        return m_total_steps;
    }
    int getRejectedSteps() const
    {
        return m_rejected_steps;
    }
    double getAverageStepSize() const
    {
        return m_avg_dt;
    }

private:
    Params m_params;

    // 中间状态
    State m_state_c1;
    State m_state_c2;
    State m_state_ELD; // 用于误差估计的额外状态

    // 统计信息
    int m_total_steps = 0;
    int m_rejected_steps = 0;
    double m_avg_dt = 0.0;
    double m_LTE_prev = 1e-6;

    void calculateParameters();
    double estimateError(const State& state_order2, const State& state_order3, const State& state_base);
    double computeNewStepSize(double dt_current, double LTE, int n_iter);
};
