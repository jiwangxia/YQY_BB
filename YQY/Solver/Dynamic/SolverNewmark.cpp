/**
 * @file SolverNewmark.cpp
 * @brief Newmark-beta nonlinear dynamic solver
 */
#include "SolverNewmark.h"
#include "Solver/Constraint/NonlinearMPC.h"

#include <QDebug>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace SolverNameSpace
{
SolverNewmark::SolverNewmark(Params p)
    : m_param(p)
{
    ComputeCoeffs(p.dt);
}

void SolverNewmark::ComputeCoeffs(double dt)
{
    if (dt <= 0.0 || m_param.beta <= 0.0)
    {
        throw std::invalid_argument("Newmark dt and beta must be positive");
    }

    m_c.a0 = 1.0 / (m_param.beta * dt * dt);
    m_c.a1 = m_param.gamma / (m_param.beta * dt);
}

bool SolverNewmark::Solve(IAnalysisModel& model, double duration)
{
    qDebug().noquote() << QStringLiteral("开始 Newmark 动力非线性求解...");

    const int nDofs = model.GetFreeDofs();
    if (nDofs <= 0)
    {
        qDebug().noquote() << QStringLiteral("错误: 自由度数量无效");
        return false;
    }
    Eigen::MatrixXd dampingFactor;
    model.GetStructuralDampingFactor(dampingFactor);
    if (dampingFactor.rows() != nDofs)
        return false;
    if (m_param.dt <= 0.0 || m_param.beta <= 0.0 || duration <= 0.0 || m_param.minimumTimeStep <= 0.0 ||
        m_param.minimumTimeStep > m_param.dt || m_param.cutbackFactor <= 0.0 || m_param.cutbackFactor >= 1.0 ||
        m_param.recoveryFactor < 1.0 || m_param.maximumCutbacks < 0 ||
        m_param.aerodynamicTangentMode < AerodynamicTangentMode::Disabled ||
        m_param.aerodynamicTangentMode > AerodynamicTangentMode::EveryNewtonIteration)
    {
        qDebug().noquote() << QStringLiteral("错误: 时间步长、beta 和总时间必须为正");
        return false;
    }

    m_dx.resize(nDofs);
    m_R.resize(nDofs);
    m_Un.resize(nDofs);
    m_Vn.resize(nDofs);
    m_An.resize(nDofs);
    m_linearSolver.Reset();
    // 陀螺切线矩阵通常非对称，因此动力有效矩阵使用通用稀疏 LU 路径。
    m_linearSolver.SetPreferLdlt(false);
    model.GetState(m_Un, m_Vn, m_An);

    m_M.resize(nDofs, nDofs);
    m_C.resize(nDofs, nDofs);
    m_Kc.resize(nDofs, nDofs);

    const int numSteps = static_cast<int>(std::ceil(duration / m_param.dt));
    qDebug().noquote() << QStringLiteral("总时间步数: %1, 标准时间步长: %2 s, 总时间: %3 s")
                              .arg(numSteps)
                              .arg(m_param.dt)
                              .arg(duration);

    double time = 0.0;
    double nextTimeStep = m_param.dt;
    int cutbackCount = 0;
    for (int step = 1; time < duration - 1.0e-12; ++step)
    {
        if (model.IsCancellationRequested())
            return false;
        const double dt = std::min(nextTimeStep, duration - time);
        if (dt <= 0.0)
            break;

        ComputeCoeffs(dt);
        const double currentTime = time + dt;

        model.BeginDynamicStep(dt, m_param.beta, m_param.gamma);

        bool converged = false;
        double error = std::numeric_limits<double>::infinity();
        double residualLimit = m_param.tol;
        double constraintError = std::numeric_limits<double>::infinity();
        Vec convergedMultipliers;
        int iterationCount = 0;
        bool externalTangentAssembled = false;
        if (m_param.aerodynamicTangentMode == AerodynamicTangentMode::Disabled)
        {
            m_externalTangent.resize(nDofs, nDofs);
            m_externalTangent.setZero();
            externalTangentAssembled = true;
        }
        for (int iter = 0; iter < m_param.maxIter; ++iter)
        {
            iterationCount = iter + 1;
            // Galloping load is state-dependent: its relative wind and
            // attack angle use the current trial displacement and
            // velocity.  Reassemble it after the Newmark predictor and
            // after every Newton correction, matching the implicit-stage
            // treatment in Adaptive TSSBN.  Keeping it outside this loop
            // freezes a t_n load throughout the t_(n+1) equilibrium
            // iteration and yields a different aeroelastic system.
            Vec F1, F2;
            model.ComputeExternalForce(currentTime, 1.0, F1, F2);
            model.AssembleEffectiveTangent(m_c.a0, m_c.a1, m_Keff);
            if (!externalTangentAssembled ||
                m_param.aerodynamicTangentMode == AerodynamicTangentMode::EveryNewtonIteration)
            {
                model.AssembleExternalLoadTangent(currentTime, 1.0, m_c.a1, m_externalTangent);
                externalTangentAssembled = true;
            }
            m_Keff -= m_externalTangent;
            model.ComputeResidual(F2, m_R);

            NonlinearMPCData constraints;
            if (!model.AssembleNonlinearMPC(constraints))
            {
                qDebug().noquote() << QStringLiteral("Error: dynamic MPC assembly failed at step %1, iteration %2")
                                          .arg(step)
                                          .arg(iter + 1);
                break;
            }

            SpMat solveTangent;
            Vec solveRhs;
            Eigen::MatrixXd solveDampingFactor;
            NonlinearMPCReduction reduction;
            if (constraints.Empty())
            {
                solveTangent = m_Keff;
                solveRhs = m_R;
                solveDampingFactor = dampingFactor;
                constraintError = 0.0;
                convergedMultipliers.resize(0);
            }
            else
            {
                if (!NonlinearMPC::Reduce(m_Keff, m_R, constraints, reduction))
                {
                    qDebug().noquote() << QStringLiteral("Error: dynamic MPC reduction failed at step %1, iteration %2")
                                              .arg(step)
                                              .arg(iter + 1);
                    break;
                }
                solveTangent = reduction.tangent;
                solveRhs = reduction.rhs;
                solveDampingFactor = reduction.masterTransformation.transpose() * dampingFactor;
                if (dampingFactor.cols() > 0)
                {
                    solveRhs -= m_c.a1 * solveDampingFactor *
                                (dampingFactor.transpose() * reduction.particularCorrection);
                }
                constraintError = reduction.constraintNorm;
                convergedMultipliers = reduction.multipliers;
            }

            error = solveRhs.norm();
            residualLimit = m_param.tol * std::max(1.0, F2.norm());
            if (!std::isfinite(error) || !std::isfinite(constraintError))
            {
                qDebug().noquote() << QStringLiteral("错误: 时间步 %1 的残差不是有限数").arg(step);
                break;
            }
            if (error <= residualLimit && constraintError < m_param.constraintTolerance)
            {
                converged = true;
                break;
            }

            Vec independentCorrection;
            if (!SolveLinear(solveTangent, solveDampingFactor, m_c.a1, solveRhs, independentCorrection))
            {
                qDebug().noquote() << QStringLiteral("错误: 时间步 %1 线性求解失败").arg(step);
                break;
            }

            m_dx = constraints.Empty() ? independentCorrection : reduction.RecoverFullIncrement(independentCorrection);
            if (m_dx.size() != nDofs || !m_dx.allFinite())
            {
                qDebug().noquote() << QStringLiteral(
                                          "Error: non-finite dynamic MPC correction at step %1, iteration %2")
                                          .arg(step)
                                          .arg(iter + 1);
                break;
            }
            const bool canRestoreTrial = model.SaveTrialState();
            bool correctionAccepted = !canRestoreTrial;
            double correctionScale = 1.0;
            if (!canRestoreTrial)
            {
                model.ApplyDynamicCorrection(m_dx, m_c.a0, m_c.a1);
            }
            else
            {
                constexpr int maximumLineSearchTrials = 8;
                constexpr double armijoSlope = 1.0e-4;
                for (int trial = 0; trial < maximumLineSearchTrials; ++trial)
                {
                    if (!model.RestoreTrialState())
                        break;
                    model.ApplyDynamicCorrection(correctionScale * m_dx, m_c.a0, m_c.a1);

                    Vec trialFixedForce;
                    Vec trialExternalForce;
                    Vec trialResidual;
                    SpMat trialEffectiveTangent;
                    model.ComputeExternalForce(currentTime, 1.0, trialFixedForce, trialExternalForce);
                    model.AssembleEffectiveTangent(m_c.a0, m_c.a1, trialEffectiveTangent);
                    if (m_param.aerodynamicTangentMode == AerodynamicTangentMode::EveryNewtonIteration)
                    {
                        SpMat trialExternalTangent;
                        model.AssembleExternalLoadTangent(currentTime, 1.0, m_c.a1, trialExternalTangent);
                        trialEffectiveTangent -= trialExternalTangent;
                    }
                    else
                    {
                        trialEffectiveTangent -= m_externalTangent;
                    }
                    model.ComputeResidual(trialExternalForce, trialResidual);
                    NonlinearMPCData trialConstraints;
                    if (!model.AssembleNonlinearMPC(trialConstraints))
                    {
                        correctionScale *= 0.5;
                        continue;
                    }

                    double trialError = trialResidual.norm();
                    double trialConstraintError = 0.0;
                    NonlinearMPCReduction trialReduction;
                    if (!trialConstraints.Empty())
                    {
                        if (!NonlinearMPC::Reduce(trialEffectiveTangent, trialResidual, trialConstraints,
                                                  trialReduction))
                        {
                            correctionScale *= 0.5;
                            continue;
                        }
                        trialError = trialReduction.rhs.norm();
                        if (dampingFactor.cols() > 0)
                        {
                            const Eigen::MatrixXd trialDampingFactor =
                                trialReduction.masterTransformation.transpose() * dampingFactor;
                            const Vec trialReducedRhs =
                                trialReduction.rhs -
                                m_c.a1 * trialDampingFactor *
                                    (dampingFactor.transpose() * trialReduction.particularCorrection);
                            trialError = trialReducedRhs.norm();
                        }
                        trialConstraintError = trialReduction.constraintNorm;
                    }
                    const double allowedConstraint =
                        1.1 * std::max(constraintError, m_param.constraintTolerance);
                    if (std::isfinite(trialError) && std::isfinite(trialConstraintError) &&
                        trialError <= (1.0 - armijoSlope * correctionScale) * error &&
                        trialConstraintError <= allowedConstraint)
                    {
                        correctionAccepted = true;
                        if (!trialConstraints.Empty())
                            convergedMultipliers = trialReduction.multipliers;
                        break;
                    }
                    correctionScale *= 0.5;
                }
            }
            if (!correctionAccepted)
            {
                model.RestoreTrialState();
                break;
            }
        }

        if (!converged)
        {
            Vec trialDisplacement;
            Vec trialVelocity;
            Vec trialAcceleration;
            model.GetState(trialDisplacement, trialVelocity, trialAcceleration);
            const double maximumDisplacement =
                trialDisplacement.size() > 0 ? trialDisplacement.cwiseAbs().maxCoeff() : 0.0;
            Eigen::Index maximumDisplacementIndex = -1;
            if (trialDisplacement.size() > 0)
                trialDisplacement.cwiseAbs().maxCoeff(&maximumDisplacementIndex);
            const double maximumVelocity = trialVelocity.size() > 0 ? trialVelocity.cwiseAbs().maxCoeff() : 0.0;
            const double maximumAcceleration =
                trialAcceleration.size() > 0 ? trialAcceleration.cwiseAbs().maxCoeff() : 0.0;
            model.RollbackDynamicStep();
            const bool minimumTimeStepReached = dt <= m_param.minimumTimeStep * (1.0 + 1.0e-12);
            if (!minimumTimeStepReached && cutbackCount < m_param.maximumCutbacks)
            {
                nextTimeStep = std::max(m_param.minimumTimeStep, dt * m_param.cutbackFactor);
                ++cutbackCount;
                qDebug().noquote() << QStringLiteral("Newmark cutback at t=%1: retry %2 with dt=%3 (residual=%4)")
                                          .arg(time, 0, 'g', 10)
                                          .arg(cutbackCount)
                                          .arg(nextTimeStep, 0, 'g', 10)
                                          .arg(error, 0, 'g', 8);
                --step; // retry the same physical time point after rollback
                continue;
            }
            QTextStream(stderr) << "Newmark failed after cutback at step=" << step << " time=" << currentTime
                                << " dt=" << dt << " residual=" << error << " limit=" << residualLimit
                                << " max_u=" << maximumDisplacement << " max_v=" << maximumVelocity
                                << " max_a=" << maximumAcceleration << ' '
                                << model.DescribeFreeDof(static_cast<int>(maximumDisplacementIndex))
                                << Qt::endl;
            qDebug().noquote() << QStringLiteral("警告: 时间步 %1 未收敛 (残差=%2)").arg(step).arg(error);
            return false;
        }

        model.SetNonlinearMPCMultipliers(convergedMultipliers);
        model.CommitState();
        model.GetState(m_Un, m_Vn, m_An);
        model.RecordStepIterations(currentTime, iterationCount);
        if (m_callback)
        {
            m_callback(step, currentTime, m_Un);
        }
        model.OnStepCompleted(currentTime);
        time = currentTime;
        nextTimeStep = std::min(m_param.dt, dt * m_param.recoveryFactor);
        cutbackCount = 0;
        model.ReportProgress(std::min(1.0, currentTime / duration),
                             QStringLiteral("动力时间步 %1/%2").arg(step).arg(numSteps));
    }

    qDebug().noquote() << QStringLiteral("动力求解完成");
    return true;
}

bool SolverNewmark::SolveLinear(const SpMat& K, const Eigen::MatrixXd& dampingFactor, double dampingScale,
                                const Vec& b, Vec& x)
{
    return m_linearSolver.SolveLowRank(K, dampingFactor, dampingScale, b, x);
}
}
