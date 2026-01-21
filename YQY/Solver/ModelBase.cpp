// ModelBase.cpp
#include "ModelBase.h"
#include <iostream>

namespace Dynamics
{//动力学名字空间

	void ModelBase::ComputeKeff(const State& s, 
		double kCoeff, double cCoeff, double mCoeff,
		SpMat& outKeff, SpMat& kBuf, SpMat& cBuf, SpMat& mBuf) const
	{//计算切线矩阵
		// 获取矩阵引用
		const SpMat& K = GetK(s, kBuf);
		const SpMat& C = GetC(s, cBuf);
		const SpMat& M = GetM(s, mBuf);

		// 1. 初始化第一项
		if (std::abs(kCoeff - 1.0) < EPSILON_ZERO) 
			outKeff = K;
		else 
			outKeff = kCoeff * K;

		// 2. 累加后续项（Eigen 稀疏矩阵的 += 会处理好内存，无需 noalias）
		if (C.nonZeros() > 0) 
		{
			if (std::abs(cCoeff - 1.0) < EPSILON_ZERO) 
				outKeff += C;
			else
				outKeff += cCoeff * C;
		}

		if (std::abs(mCoeff - 1.0) < EPSILON_ZERO)
			outKeff += M;
		else
			outKeff += mCoeff * M;
	}

	// 给定(x,v,t), 求解满足 Φ(x, v, a, t) = 0 的加速度 a
	void ModelBase::SolveAcceleration(State& state) const
	{// 默认使用 Newton-Raphson 迭代, 允许用户覆盖（如果有解析解，覆盖后速度更快）
		const int max_iter = 10;
		Vec R(m_Dofs);
		Eigen::SparseLU<SpMat> solver; // 通用求解器，LDLT只适用于对称正定
		Vec da;

		// 预分配 buffer，避免循环内反复构造析构 (虽然 Eigen 内部可能会重分配，但对象本身复用)
		SpMat mBuffer;

		for (int iter = 0; iter < max_iter; ++iter)
		{
			// A. 计算残差
			ComputeResidual(state, R);

			// B. 检查收敛
			if (R.norm() < TOLERANCE_ACCEL) return;

			// C. 获取切线质量矩阵 (利用 Buffer)
			// 这里的 GetM 会根据子类不同，决定是填入 mBuffer 还是直接返回内部引用
			const SpMat& Mt = GetM(state, mBuffer);

			// D. 求解 (注意：频繁调用 analyzePattern 比较慢，如果结构不变可优化，此处暂且保留)
			solver.compute(Mt);
			if (solver.info() != Eigen::Success) 
			{
				throw std::runtime_error("Tangent Matrix factorization failed.");
			}
			da = solver.solve(-R);

			// E. 更新
			state.a += da;
		}

		throw std::runtime_error("solve_acceleration failed to converge.");
	}

	//接口实现
	void GeneralModel::ComputeResidual(const State& s, Vec& R_out) const
	{//计算残差
		if (m_CalcResidual)
			m_CalcResidual(s, R_out);
		else throw
			std::runtime_error("Residual function missing");
	}

	void GeneralModel::SolveAcceleration(State& s) const
	{//求解加速度
		if (m_CalcAccel)
		{// 1. 如果用户提供了自定义的加速度求解函数（通常是解析解，速度快），则使用它
			m_CalcAccel(s);
		}
		else
		{// 2. 否则，回退到基类的默认实现（使用 Newton-Raphson 迭代求解，速度稍慢但通用）
			ModelBase::SolveAcceleration(s);
		}
	}

	ModelLinear::ModelLinear(const SpMat& m,
		const SpMat& c,
		const SpMat& k) :
		ModelBase(m.rows()), m_M(m), m_C(c), m_K(k)
	{
		assert(m_M.cols() == m_Dofs);
		assert(m_C.cols() == m_Dofs && m_C.rows() == m_Dofs);
		assert(m_K.cols() == m_Dofs && m_K.rows() == m_Dofs);

		m_solverM.compute(m_M);
		if (m_solverM.info() != Eigen::Success)
		{// 检查质量矩阵是否分解成功
			std::cerr << "Warning: Mass matrix factorization failed or matrix is singular.\n";
		}
	}

	void ModelLinear::ComputeKeff(const State& s, double kCoeff, double cCoeff, double mCoeff, SpMat& outKeff, SpMat& kBuf, SpMat& cBuf, SpMat& mBuf) const
	{// 计算切线矩阵
		// 1. 初始化第一项
		outKeff = m_K;
		if (std::abs(kCoeff - 1.0) > EPSILON_ZERO)
			outKeff *= kCoeff; // 原地乘法，不改变结构

		// 2. 累加后续项（Eigen 稀疏矩阵的 += 会处理好内存，无需 noalias）
		if (m_C.nonZeros() > 0)
		{
			if (std::abs(cCoeff - 1.0) < EPSILON_ZERO)
				outKeff += m_C;
			else
				outKeff += cCoeff * m_C;
		}

		if (std::abs(mCoeff - 1.0) < EPSILON_ZERO)
			outKeff += m_M;
		else
			outKeff += mCoeff * m_M;
	}

	void ModelLinear::SolveAcceleration(State& state) const
	{//求解加速度
		// 1. 显式初始化维度，避免推导错误
		Vec RHS(m_Dofs);

		// 2. 使用 noalias 提高效率并确保维度检查
		RHS.noalias() = -m_K * state.x;
		RHS.noalias() -= m_C * state.v;

		// 3. 处理外力
		if (m_func_force != nullptr)
		{
			// 建议在这里也加一个断言，防止 lambda 返回错误维度的向量
			Vec force = m_func_force(state.t);
			assert(force.size() == m_Dofs && "Force vector size mismatch!");
			RHS.noalias() += force;
		}

		// 4. 求解
		state.a = m_solverM.solve(RHS);
	}

	void ModelLinear::ComputeResidual(const State& s, Vec& R_out) const
	{//计算残量
		R_out.noalias() = m_M * s.a;
		R_out.noalias() += m_C * s.v;
		R_out.noalias() += m_K * s.x;
		if (m_func_force) R_out.noalias() -= m_func_force(s.t);
	}

}// namespace Dynamics
