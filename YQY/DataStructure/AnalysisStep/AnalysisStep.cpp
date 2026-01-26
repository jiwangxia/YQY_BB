#include "AnalysisStep.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementBase.h"
#include "Solver/SolverNewmark.h"
#include <Eigen/SparseCholesky>

void AnalysisStep::SetStructure(std::shared_ptr<StructureData> pStructure)
{
    m_pStructure = pStructure;
}

StructureData* AnalysisStep::GetStructure() const
{
    auto ptr = m_pStructure.lock();
    return ptr ? ptr.get() : nullptr;
}

bool AnalysisStep::PrepareData()
{
    m_pData = GetStructure();
    if (!m_pData)
    {
        qDebug().noquote() << QStringLiteral("Error: 分析步未关联结构数据");
        return false;
    }
    return true;
}

void AnalysisStep::Init()
{
    if (!PrepareData()) return;
    Init_DOF();
    Init_Nodevector();
}

void AnalysisStep::Init_DOF()
{
    // m_pData 已由 PrepareData() 准备好

    // 重置所有节点的DOF标记，确保重新计算
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto& pNode = nodePair.second;
        std::fill(pNode->m_DOF.begin(), pNode->m_DOF.end(), -1);
    }

    // 根据单元节点自由度初始化节点DOF数组

    for (auto& elements : m_pData->m_Elements)
    {
        auto pelement = elements.second;
        int NodeDOF = pelement->Get_NodeDOF();
        for (auto& nodes : pelement->m_pNode)
        {
            auto pNode = nodes.lock();
            if (pNode && pNode->m_DOF.size() < NodeDOF)
            {
                pNode->m_DOF.resize(NodeDOF, -1);
            }
        }
    }

    int iStart = 0;

    // 先处理约束自由度
    for (auto& constraints : m_pData->m_Constraint)
    {
        auto pConstrain = constraints.second;
        auto pNode = pConstrain->m_pNode.lock();
        if (!pNode) continue;

        int iDirection = static_cast<int>(pConstrain->m_Direction);
        if (iDirection < 0 || iDirection >= pNode->m_DOF.size()) continue;

        if (-1 == pNode->m_DOF[iDirection])
        {
            pNode->m_DOF[iDirection] = iStart++;
        }
    }

    m_nFixed = iStart;

    // 再处理自由自由度
    for (auto& nodes : m_pData->m_Nodes)
    {
        auto pNode = nodes.second;
        for (auto& dofValue : pNode->m_DOF)
        {
            if (-1 == dofValue) dofValue = iStart++;
        }
    }

    m_nFree = iStart - m_nFixed;
}

void AnalysisStep::Init_Nodevector()
{
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();
        if (pNode->m_Displacement.size() < numDOF)
            pNode->m_Displacement.resize(numDOF, 0.0);
        if (pNode->m_Force.size() < numDOF)
            pNode->m_Force.resize(numDOF, 0.0);
        if (pNode->m_Velocity.size() < numDOF)
            pNode->m_Velocity.resize(numDOF, 0.0);
        if (pNode->m_Acceleration.size() < numDOF)
            pNode->m_Acceleration.resize(numDOF, 0.0);
    }
}

void AnalysisStep::AssembleKs()
{
    std::list<Tri> L11, L21, L22;

    m_K11.resize(m_nFixed, m_nFixed);
    m_K21.resize(m_nFree, m_nFixed);
    m_K22.resize(m_nFree, m_nFree);

    MatrixXd ke;
    std::vector<int> DOFs;

    for (auto& element : m_pData->m_Elements)
    {
        auto pelement = element.second;
        //pelement->Get_ke(ke);
        pelement->Get_ke_non(ke);
        pelement->GetDOFs(DOFs);
        Assemble(DOFs, ke, L11, L21, L22);
    }

    m_K11.setFromTriplets(L11.begin(), L11.end());
    m_K21.setFromTriplets(L21.begin(), L21.end());
    m_K22.setFromTriplets(L22.begin(), L22.end());

    // Fix: 为了防止刚度矩阵奇异（例如竖直杆件受到横向力时初始切线刚度为0），
    // 在对角线上添加一个极小值 epsilon
    double epsilon = 1e-10;
    for (int i = 0; i < m_nFree; ++i)
    {
        m_K22.coeffRef(i, i) += epsilon;
    }

    qDebug();
    //std::cout << MatrixXd(m_K22);
}

void AnalysisStep::Assemble(std::vector<int>& DOFs, Eigen::MatrixXd& T, std::list<Tri>& L11, std::list<Tri>& L21, std::list<Tri>& L22)
{
    auto nDOF = DOFs.size();

    for (int i = 0; i < nDOF; ++i)
    {
        int ii = DOFs[i];
        for (int j = 0; j < nDOF; ++j)
        {
            int jj = DOFs[j];
            auto kij = T(i, j);

            if (ii < m_nFixed && jj < m_nFixed)
            {
                L11.push_back(Tri(ii, jj, kij));
            }
            else if (ii >= m_nFixed && jj < m_nFixed)
            {
                L21.push_back(Tri(ii - m_nFixed, jj, kij));
            }
            else if (ii >= m_nFixed && jj >= m_nFixed)
            {
                L22.push_back(Tri(ii - m_nFixed, jj - m_nFixed, kij));
            }
        }
    }
}

void AnalysisStep::Assemble_AllLoads(VectorXd& F1, VectorXd& F2, double& Factor)
{
    F1.resize(m_nFixed);
    F1.setZero();
    F2.resize(m_nFree);
    F2.setZero();
    for (auto& Load : m_pData->m_Load)
    {
        auto pLoadBase = Load.second;
        double currentScale = 0.0;

        if (pLoadBase->m_StepId < this->m_Id)
        {
            // 历史步的荷载：全额
            currentScale = 1.0;
        }
        else if (pLoadBase->m_StepId == this->m_Id)
        {
            // 当前步的荷载：随 Factor 增加
            currentScale = Factor;
        }
        else
        {
            // 未来步的荷载：不加载
            continue;
        }

        switch (pLoadBase->m_LoadType)
        {
        case EnumKeyword::LoadType::FORCE_NODE:
        {
            // 向下转型为 Force_Node
            auto pForceNode = std::dynamic_pointer_cast<Force_Node>(pLoadBase);
            if (!pForceNode) continue;

            Assemble_ForceNode(pForceNode.get(), F1, F2, m_Time);
            F2 *= currentScale;
            break;
        }
        case EnumKeyword::LoadType::FORCE_ELEMENT:
        {
            // 向下转型为 Force_Element
            auto pForceElement = std::dynamic_pointer_cast<Force_Element>(pLoadBase);
            if (!pForceElement) continue;

            Assemble_ForceElement(pForceElement.get(), F1, F2, m_Time);
            F2 *= currentScale;
            break;
        }
        default:
            break;
        }
    }
    //std::cout << "F2:\n" << VectorXd(F2);
}

void AnalysisStep::UpData(VectorXd& x1, VectorXd& x2, VectorXd& F1, VectorXd* v2, VectorXd* a2)
{
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        // 确保节点向量大小正确
        if (pNode->m_Displacement.size() < numDOF)
            pNode->m_Displacement.resize(numDOF, 0.0);
        if (pNode->m_Velocity.size() < numDOF)
            pNode->m_Velocity.resize(numDOF, 0.0);
        if (pNode->m_Acceleration.size() < numDOF)
            pNode->m_Acceleration.resize(numDOF, 0.0);
        if (pNode->m_Force.size() < numDOF)
            pNode->m_Force.resize(numDOF, 0.0);

        // 更新各自由度方向的数据
        for (int dofIdx = 0; dofIdx < numDOF; dofIdx++)
        {
            int dof = pNode->m_DOF[dofIdx];

            // 约束自由度：位移增量为0（除非是位移控制，那由调用者直接设置）
            if (dof < m_nFixed)
            {
                pNode->m_Displacement[dofIdx] = x1[dof];
                pNode->m_Force[dofIdx] = F1[dof];
            }
            // 自由自由度：从x2向量获取增量
            else if (dof < m_nFixed + m_nFree)
            {
                pNode->m_Displacement[dofIdx] += x2[dof - m_nFixed];
                if (v2)
                {
                    pNode->m_Velocity[dofIdx] += (*v2)[dof - m_nFixed];
                }
                if (a2)
                {
                    pNode->m_Acceleration[dofIdx] += (*a2)[dof - m_nFixed];
                }
            }
        }
    }
}

void AnalysisStep::Get_CurrentInforce(VectorXd& Inforce)
{
    for (auto& elementPair : m_pData->m_Elements)
    {
        auto pElement = elementPair.second;
        std::vector<int> elementDOFs;
        pElement->GetDOFs(elementDOFs);

        // 将单元内力累加到节点
        auto& nodeWeakPtrs = pElement->m_pNode;
        int nodeDOF = pElement->Get_NodeDOF();

        for (auto nodeIdx = 0; nodeIdx < nodeWeakPtrs.size(); ++nodeIdx)
        {
            auto pNode = nodeWeakPtrs[nodeIdx].lock();
            if (!pNode) continue;

            for (auto dofIdx = 0; dofIdx < nodeDOF; ++dofIdx)
            {
                auto localIdx = nodeIdx * nodeDOF + dofIdx;
                if ((localIdx < pElement->m_inforce.size()) && (dofIdx < pNode->m_Force.size()))
                {
                    pNode->m_Force[dofIdx] += pElement->m_inforce[localIdx];
                }
            }
        }

        // 累加自由自由度的内力用于收敛判断
        for (size_t dofIdx = 0; dofIdx < elementDOFs.size(); ++dofIdx)
        {
            int globalDOF = elementDOFs[dofIdx];
            if (globalDOF >= m_nFixed)
            {
                Inforce[globalDOF - m_nFixed] += pElement->m_inforce[dofIdx];
            }
        }
    }
}

bool AnalysisStep::Check_Rhs(Eigen::VectorXd& Exteralforce, Eigen::VectorXd& Inforce, Eigen::VectorXd& Rhs)
{
    Rhs = Exteralforce - Inforce;
    double RhsNorm = Rhs.norm();
    if (RhsNorm < m_Tolerance)
    {
        //qDebug().noquote() << QStringLiteral("\n收敛，残差范数: %1\n").arg(RhsNorm);
        return true;
    }
    //qDebug().noquote() << QStringLiteral("\n残差范数: %1\n").arg(RhsNorm);
    return false;
}




void AnalysisStep::Assemble_ForceNode(Force_Node* pForceNode, VectorXd& F1, VectorXd& F2, double& current_time)
{
    auto pNode = pForceNode->m_pNode.lock();
    if (!pNode) return;

    int iDirection = static_cast<int>(pForceNode->m_Direction);
    if (iDirection < 0 || iDirection >= pNode->m_DOF.size()) return;

    int dof = pNode->m_DOF[iDirection];

    // 根据 DOF，加到对应向量
    if (dof >= 0 && dof < m_nFixed)
    {
        F1[dof] += pForceNode->m_Value;
    }
    else if (dof >= m_nFixed)
    {
        F2[dof - m_nFixed] += pForceNode->m_Value;
    }
}

void AnalysisStep::Assemble_ForceElement(Force_Element* pForceElement, VectorXd& F1, VectorXd& F2, double& current_time)
{
    auto pElement = pForceElement->m_pElement.lock();
    if (!pElement) return;

    int iDirection = static_cast<int>(pForceElement->m_Direction);
    for (auto& weakNodePtr : pElement->m_pNode)
    {
        auto pNode = weakNodePtr.lock();
        int dof = pNode->m_DOF[iDirection];
        if (dof >= 0 && dof < m_nFixed)
        {
            F1[dof] += pForceElement->m_Value / 2.;
        }
        else if (dof >= m_nFixed && dof < (m_nFixed + m_nFree))
        {
            F2[dof - m_nFixed] += pForceElement->m_Value / 2.;
        }
    }
}


void AnalysisStep::Assemble_Constraint(VectorXd& x1)
{
    x1.resize(m_nFixed);
    for (auto& constraintPair : m_pData->m_Constraint)
    {
        auto pConstraint = constraintPair.second;
        auto pNode = pConstraint->m_pNode.lock();
        if (!pNode) continue;

        int iDirection = static_cast<int>(pConstraint->m_Direction);
        if (iDirection < 0 || iDirection >= pNode->m_DOF.size()) continue;

        int dof = pNode->m_DOF[iDirection];
        if (dof >= 0 && dof < m_nFixed)
        {
            x1[dof] = pConstraint->m_Value;
            pNode->m_Displacement[iDirection] = pConstraint->m_Value;
        }
    }
}

// ==========================================
// 求解方法实现
// ==========================================

void AnalysisStep::Solve()
{
    if (!PrepareData()) return;

    switch (m_Type)
    {
    case EnumKeyword::StepType::STATIC:
        Solve_Static();
        break;
    case EnumKeyword::StepType::DYNAMIC:
        Solve_Dynamic();
        break;
    default:
        break;
        qDebug().noquote() << QStringLiteral("警告: 未知的分析步类型，无法求解");
    }
}

void AnalysisStep::Solve_Static()
{
    qDebug().noquote() << QStringLiteral("开始静力求解...");

    // 定义力向量和约束向量
    VectorXd F1, F2, x1;

    // 定义位移向量
    VectorXd x2, totalx2;
    x2.setZero(m_nFree);
    totalx2.setZero(m_nFree);

    // 内力向量
    VectorXd internalForce;
    internalForce.setZero(m_nFree);

    // 组装约束
    Assemble_Constraint(x1);

    // 残差向量
    VectorXd residual;

    int numIncrements = m_Time / m_StepSize; // 可由用户在输入文件中定义
    qDebug().noquote() << QStringLiteral("分%1步施加荷载").arg(numIncrements);
    for (int inc = 1; inc <= numIncrements; ++inc)
    {
        double currentFactor = (double)inc / numIncrements;
        //组装外荷载和
        Assemble_AllLoads(F1, F2, currentFactor);

        // Newton-Raphson 迭代
        for (int iter = 0; iter < m_MaxIterations; iter++)
        {
            // 1. 组装刚度矩阵 (基于当前变形状态)
            AssembleKs();

            // 2. 清零节点内力，然后计算单元内力并累加到节点
            for (auto& nodePair : m_pData->m_Nodes)
            {
                std::fill(nodePair.second->m_Force.begin(), nodePair.second->m_Force.end(), 0.0);
            }

            internalForce.setZero();
            Get_CurrentInforce(internalForce);

            // 3. 检查收敛性
            if (Check_Rhs(F2, internalForce, residual) && iter > 0)
            {
                qDebug().noquote() << QStringLiteral("迭代在第 %1 步收敛").arg(iter);
                break;
            }

            // 4. 计算有效荷载 (考虑约束影响)
            VectorXd effectiveForce = residual;

            // 5. 求解线性方程组 K22 * Δu = F_eff
            Eigen::SimplicialLDLT<SpMat> ldltSolver;
            ldltSolver.analyzePattern(m_K22);
            ldltSolver.factorize(m_K22);
            if (ldltSolver.info() != Success)
            {
                qDebug().noquote() << QStringLiteral("LDLT分解失败!");
                return;
            }

            x2 = ldltSolver.solve(effectiveForce);

            F1 = m_K11 * x1 + m_K21.transpose() * x2;

            // 6. 累加位移增量
            totalx2 += x2;

            // 7. 更新节点位移
            UpData(x1, x2, F1);

            // 8. 检查是否达到最大迭代次数
            if (iter == m_MaxIterations - 1)
            {
                qDebug().noquote() << QStringLiteral("\n达最大迭代次数\n");
            }
        }
    } 
    // 保存结果到输出器 (直接从节点读取所有数据)
    if (m_pData)
    {
        m_pData->GetOutputter().SaveDataFromNodes(m_Time, m_pData);
    }
    qDebug().noquote() << QStringLiteral("\n静力求解完成 ");
}

void AnalysisStep::Solve_Dynamic()
{
    //using namespace Dynamics;

    //qDebug().noquote() << QStringLiteral("开始动力求解...");

    //// 1. 初始化
    //Init_DOF();
    //AssembleKs();

    //// 2. 创建通用模型
    //GeneralModel model(m_nFree);

    //// 绑定刚度矩阵
    //model.SetFuncK([this](const State& s, SpMat& buffer) -> const SpMat& {
    //    return this->m_K22;
    //    });

    //// 绑定质量矩阵 (TODO: 需要实现 AssembleMs)
    //// model.SetFuncM([this](const State& s, SpMat& buffer) -> const SpMat& {
    ////     return this->m_M22;
    //// });

    //// 绑定阻尼矩阵 (TODO: 需要实现 AssembleCs)
    //// model.SetFuncC([this](const State& s, SpMat& buffer) -> const SpMat& {
    ////     return this->m_C22;
    //// });

    //// 绑定残差计算
    //model.SetResidualFunc([this](const State& s, Vec& R_out) {
    //    // 计算残差 R = M*a + C*v + K*x - F(t)
    //    // TODO: 实现完整的残差计算
    //    R_out = this->m_K22 * s.x;
    //    });

    //// 3. 配置求解器
    //SolverNewmark::Parameters params;
    //params.dt = m_StepSize > 0 ? m_StepSize : 0.01;
    //params.bAdaptive = true;

    //SolverNewmark solver(params);

    //// 4. 初始状态
    //State state(m_nFree);

    //// 5. 观察者回调 (每步保存结果)
    //auto observer = [this](const State& s) {
    //    // 增量保存当前时刻数据
    //    m_Outputter.SaveData(s.t, m_pData, m_nFixed, s.x, &s.v, &s.a);
    //    qDebug().noquote() << QStringLiteral("t = ") << s.t
    //        << QStringLiteral(", frame = ") << m_Outputter.GetFrameCount();
    //    };

    //// 6. 求解
    //// solver.solve(model, state, m_Time, observer);

    //qDebug().noquote() << QStringLiteral("动力求解完成 (框架已就绪，需要实现质量/阻尼矩阵组装)");
}
