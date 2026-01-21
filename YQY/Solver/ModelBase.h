// ModelBase.h
#pragma once
#include <Eigen/Sparse>
#include <functional>

namespace Dynamics
{//动力学名字空间
    using SpMat = Eigen::SparseMatrix<double>; // 默认为列优先
    using Vec = Eigen::VectorXd;

    constexpr double TOLERANCE_ACCEL = 1e-10;// 加速度求解容差
    constexpr double EPSILON_ZERO = 1e-15; // 非零判断阈值

    struct State
    {// 状态量
        double t = 0.0;// 时间
        Vec x, v, a;// 位移、速度、加速度
        // 构造函数，位移、速度、加速度初始化为 0
        State(size_t dofs) : 
            x(Vec::Zero(dofs)), 
            v(Vec::Zero(dofs)), 
            a(Vec::Zero(dofs)) 
        {}
        State() = default;
    };

    // 定义观察者函数类型
    using Observer = std::function<void(const State&)>;

    class ModelBase 
    {//模型基类，通用模型：Φ(x,v,a,t) = 0
    protected:
        size_t m_Dofs;//自由度

    public:// 构造与析构
        ModelBase(size_t Dofs) : m_Dofs(Dofs) {}
        virtual ~ModelBase() = default;

    public:// 成员函数
        size_t GetDofs() const { return m_Dofs; }
        virtual bool IsLinear() const { return false; }

        // 核心组装接口：outKeff = kCoeff*K + cCoeff*C + mCoeff*M
        virtual void ComputeKeff(const State& s, 
            double kCoeff, double cCoeff, double mCoeff,
            SpMat& outKeff, SpMat& kBuf, SpMat& cBuf, SpMat& mBuf) const;

        virtual void SolveAcceleration(State& state) const;

    public://公共接口
        virtual void ComputeResidual(const State& s, Vec& R_out) const = 0;

    protected://保护接口
        virtual const SpMat& GetM(const State& s, SpMat& buffer) const = 0;
        virtual const SpMat& GetC(const State& s, SpMat& buffer) const = 0;
        virtual const SpMat& GetK(const State& s, SpMat& buffer) const = 0;
    };

    class ModelLinear : public ModelBase
    {//线性模型（要求外部的m,c,k在求解过程中一直存在）
    protected:
        Eigen::SimplicialLDLT<SpMat> m_solverM;//质量矩阵的Cholesky分解
        const SpMat &m_M, &m_C, &m_K;//质量矩阵、阻尼矩阵、刚度矩阵
        std::function<Vec(double)> m_func_force = nullptr;//外力函数

    public://构造函数（要求外部的m,c,k在求解过程中一直存在）
        ModelLinear(const SpMat& m,//质量矩阵
            const SpMat& c,//阻尼矩阵
            const SpMat& k);//刚度矩阵

    public:// 成员函数（设置外力）
        void SetForceFunc(std::function<Vec(double)> f) { m_func_force = std::move(f); }

    protected:
        // 取得切线质量矩阵（忽略缓存）
        virtual const SpMat& GetM(const State& s, SpMat& buffer) const override { return m_M; }
        // 取得切线阻尼矩阵（忽略缓存）
        virtual const SpMat& GetC(const State& s, SpMat& buffer) const override { return m_C; }
        // 取得切线刚度矩阵（忽略缓存）
        virtual const SpMat& GetK(const State& s, SpMat& buffer) const override { return m_K; }

    public://基类接口实现
         bool IsLinear() const override { return true; }//线性模型
         // 计算切线矩阵
         virtual void ComputeKeff(const State& s,
             double kCoeff, double cCoeff, double mCoeff,
             SpMat& outKeff, SpMat& kBuf, SpMat& cBuf, SpMat& mBuf) const override;
       // 求解满足 Φ(x, v, a, t) = 0 的加速度 a
        void SolveAcceleration(State& s) const override;
        // 计算总残差 Φ(x,v,a,t)
        void ComputeResidual(const State& s, Vec& R_out) const override;
    };

    class GeneralModel : public ModelBase
    {//通过函数接口，实现自定义非线性模型
    public://函数类型定义
        using FuncRes = std::function<void(const State& s, Vec& R_out)>;
        using FuncAccel = std::function<void(State& s)>;
        using FuncMCK = std::function<const SpMat& (const State& s, SpMat&)>;
        //利用Lambda表达式实现FuncMCK函数的话，既可以捕获外部的矩阵来转发
		//也可以对传入的矩阵进行修改后返回

    private://成员变量
        FuncRes m_CalcResidual = nullptr;
        FuncMCK m_CalcM = nullptr;
        FuncMCK m_CalcC = nullptr;
        FuncMCK m_CalcK = nullptr;
        FuncAccel m_CalcAccel = nullptr;

    public:// 构造函数
        GeneralModel(size_t dofs) : ModelBase(dofs) {}

    public:// 设置函数接口
        void SetResidualFunc(FuncRes f) { m_CalcResidual = std::move(f); }//设置残差函数，必须设置
        void SetFuncM(FuncMCK f) { m_CalcM = std::move(f); }//设置切线质量矩阵函数，必须设置
        void SetFuncC(FuncMCK f) { m_CalcC = std::move(f); }//设置切线阻尼矩阵函数，必须设置
        void SetFuncK(FuncMCK f) { m_CalcK = std::move(f); }//设置切线刚度矩阵函数，必须设置
        void SetAccelFunc(FuncAccel f) { m_CalcAccel = std::move(f); }//设置加速度函数，可以不设置，默认使用 Newton-Raphson 迭代求解

	public://基类接口实现
		void ComputeResidual(const State& s, Vec& R_out) const override;
        void SolveAcceleration(State& s) const override;

    protected:
        const SpMat& GetM(const State& s, SpMat& buffer) const override 
        {// 取得切线质量矩阵（使用缓存）
            if (!m_CalcM) throw std::runtime_error("Mass matrix function not set!"); // 增加安全检查
            return m_CalcM(s, buffer);
        }

		const SpMat& GetC(const State& s, SpMat& buffer) const override
		{// 取得切线阻尼矩阵（使用缓存）
			if (m_CalcC)
			{// 若设置了切线阻尼矩阵函数，则计算
				return m_CalcC(s, buffer); // 计算并将结果填入 buffer
			}
			else
			{// 若未设置，则返回零矩阵
				buffer.resize(m_Dofs, m_Dofs); // 若未设置，则返回零矩阵
				buffer.setZero();
    			return buffer;// 返回 buffer 的引用
			}
		}
		const SpMat& GetK(const State& s, SpMat& buffer) const override
		{// 取得切线刚度矩阵（使用缓存）
			if (m_CalcK)
			{// 若设置了切线刚度矩阵函数，则计算
				return m_CalcK(s, buffer); // 计算并将结果填入 buffer
			}
			else
			{// 若未设置，则返回零矩阵
				buffer.resize(m_Dofs, m_Dofs); // 若未设置，则返回零矩阵
				buffer.setZero();
    			return buffer;// 返回 buffer 的引用
			}
		}
    };
}
