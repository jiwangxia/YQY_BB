#include "AnalysisStep.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementBase.h"
#include "Solver/Interface/ISolver.h"
#include "Solver/Static/SolverStatic.h"
#include "Solver/Dynamic/SolverNewmark.h"
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

void AnalysisStep::Get_ElementLength()
{
    for (auto& ele : m_pData->m_Elements)
    {
        auto pElement = ele.second;
        pElement->Get_L0();
    }
}

void AnalysisStep::AssembleKs_Static()
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
        //pelement->Get_ke(ke);//无内力，目前会出错
        pelement->Get_ke_non(ke);

        //std::cout << MatrixXd(ke) << "\n";
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

void AnalysisStep::Assemble_Matrix()
{
    std::list<Tri> m11, m21, m22;
    std::list<Tri> k11, k21, k22;
    std::list<Tri> c11, c21, c22;

    m_M11.resize(m_nFixed, m_nFixed);
    m_M21.resize(m_nFree, m_nFixed);
    m_M22.resize(m_nFree, m_nFree);

    m_K11.resize(m_nFixed, m_nFixed);
    m_K21.resize(m_nFree, m_nFixed);
    m_K22.resize(m_nFree, m_nFree);

    m_C11.resize(m_nFixed, m_nFixed);
    m_C21.resize(m_nFree, m_nFixed);
    m_C22.resize(m_nFree, m_nFree);

    std::vector<int> DOFs;
    std::vector<double> damping{ 0.0 ,0.0 ,0.0 ,0.0 };

    MatrixXd me, ke, ce;
    for (auto& element : m_pData->m_Elements)
    {
        auto pelement = element.second;
        pelement->Get_ke_non(ke);
        pelement->Get_me_Consistent(me);
        //pelement->Get_me_Lumped(me);//杆单元用集中质量矩阵求解动力学与abaqus一致
        pelement->Assemble(damping, ce);

        pelement->GetDOFs(DOFs);
        Assemble(DOFs, me, m11, m21, m22);
        Assemble(DOFs, ke, k11, k21, k22);
        Assemble(DOFs, ce, c11, c21, c22);
    }

    m_M11.setFromTriplets(m11.begin(), m11.end());
    m_M21.setFromTriplets(m21.begin(), m21.end());
    m_M22.setFromTriplets(m22.begin(), m22.end());

    m_K11.setFromTriplets(k11.begin(), k11.end());
    m_K21.setFromTriplets(k21.begin(), k21.end());
    m_K22.setFromTriplets(k22.begin(), k22.end());

    m_C11.setFromTriplets(c11.begin(), c11.end());
    m_C21.setFromTriplets(c21.begin(), c21.end());
    m_C22.setFromTriplets(c22.begin(), c22.end());
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

void AnalysisStep::Assemble_AllLoads(VectorXd& F1, VectorXd& F2, double& Factor, double currentTime)
{
    F1.resize(m_nFixed);
    F1.setZero();
    F2.resize(m_nFree);
    F2.setZero();

    for (auto& Load : m_pData->m_Load)
    {
        auto pLoadBase = Load.second;

        double loadScale = 0.0;  // 每个荷载独立的缩放系数

        if (pLoadBase->m_StepId < this->m_Id)
        {
            // 历史步的荷载：全额
            loadScale = 1.0;
        }
        else if (pLoadBase->m_StepId == this->m_Id)
        {
            // 当前步的荷载：检查时间窗口
            if (!pLoadBase->IsActive(currentTime))
            {
                continue; // 不在当前时间范围内，跳过
            }
            loadScale = Factor;
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

            Assemble_ForceNode(pForceNode.get(), F1, F2, currentTime, loadScale);
            break;
        }
        case EnumKeyword::LoadType::FORCE_ELEMENT:
        {
            // 向下转型为 Force_Element
            auto pForceElement = std::dynamic_pointer_cast<Force_Element>(pLoadBase);
            if (!pForceElement) continue;

            Assemble_ForceElement(pForceElement.get(), F1, F2, currentTime, loadScale);
            break;
        }
        case EnumKeyword::LoadType::FORCE_GRAVITY:
        {
            // 向下转型为 Force_Gravity
            auto pForceGravity = std::dynamic_pointer_cast<Force_Gravity>(pLoadBase);
            if (!pForceGravity) continue;

            Assemble_ForceGravity(pForceGravity.get(), F1, F2, currentTime, loadScale);
            break;
        }
        case EnumKeyword::LoadType::FORCE_WIND:
        {
            // 向下转型为 Force_Gravity
            auto pForceWind = std::dynamic_pointer_cast<Force_Wind>(pLoadBase);
            if (!pForceWind) continue;

            Assemble_ForceWind(pForceWind.get(), F1, F2, currentTime, loadScale);
            break;
        }
        default:
            break;
        }
    }
    //std::cout << "\nF2:" << F2[56] << "\n";
}

void AnalysisStep::Updata_NodeData(VectorXd& x1, VectorXd& x2, VectorXd& F1, VectorXd* v2, VectorXd* a2)
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
                    pNode->m_Velocity[dofIdx] = (*v2)[dof - m_nFixed];
                }
                if (a2)
                {
                    pNode->m_Acceleration[dofIdx] = (*a2)[dof - m_nFixed];
                }
            }
        }
    }

    // 输出特定节点的位移（用于调试和监控）
    // 可以根据需要修改节点 ID
    int monitorNodeId = 11;  // 监控节点 11
    auto it = m_pData->m_Nodes.find(monitorNodeId);
    if (it != m_pData->m_Nodes.end())
    {
        auto pNode = it->second;
        if (pNode->m_Displacement.size() >= 6)
        {
            qDebug().noquote() << QStringLiteral("      节点 %1: u1=%2, u2=%3, u3=%4, θ1=%5, θ2=%6, θ3=%7")
                .arg(monitorNodeId)
                .arg(pNode->m_Displacement[0], 8, 'e', 2)
                .arg(pNode->m_Displacement[1], 8, 'e', 2)
                .arg(pNode->m_Displacement[2], 8, 'e', 2)
                .arg(pNode->m_Displacement[3], 8, 'e', 2)
                .arg(pNode->m_Displacement[4], 8, 'e', 2)
                .arg(pNode->m_Displacement[5], 8, 'e', 2);
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




void AnalysisStep::Assemble_ForceNode(Force_Node* pForceNode, VectorXd& F1, VectorXd& F2, double& current_time, double loadScale)
{
    auto pNode = pForceNode->m_pNode.lock();
    if (!pNode) return;

    int iDirection = static_cast<int>(pForceNode->m_Direction);
    if (iDirection < 0 || iDirection >= pNode->m_DOF.size()) return;

    int dof = pNode->m_DOF[iDirection];

    // 计算时间函数的缩放因子
    double timeFactor = pForceNode->GetScaleFactor(current_time);
    double actualValue = pForceNode->m_Value * loadScale * timeFactor;

    // 根据 DOF，加到对应向量
    if (dof >= 0 && dof < m_nFixed)
    {
        F1[dof] += actualValue;
    }
    else if (dof >= m_nFixed)
    {
        F2[dof - m_nFixed] += actualValue;
    }
}

void AnalysisStep::Assemble_ForceElement(Force_Element* pForceElement, VectorXd& F1, VectorXd& F2, double& current_time, double loadScale)
{
    auto pElement = pForceElement->m_pElement.lock();
    if (!pElement) return;

    int iDirection = static_cast<int>(pForceElement->m_Direction);
    double actualValue = pForceElement->m_Value * loadScale / 2.;

    for (auto& weakNodePtr : pElement->m_pNode)
    {
        auto pNode = weakNodePtr.lock();
        int dof = pNode->m_DOF[iDirection];
        if (dof >= 0 && dof < m_nFixed)
        {
            F1[dof] += 0.;
        }
        else if (dof >= m_nFixed && dof < (m_nFixed + m_nFree))
        {
            F2[dof - m_nFixed] += actualValue;
        }
    }
}


void AnalysisStep::Assemble_ForceGravity(Force_Gravity* pForceGravity, VectorXd& F1, VectorXd& F2, double& current_time, double loadScale)
{
    int iDirection = static_cast<int>(pForceGravity->m_Direction);
    double m_g = pForceGravity->m_g;
    for (auto& pElement : m_pData->m_Elements)
    {
        auto pele = pElement.second;
        auto pPorety = pele->m_pProperty.lock();
        auto m_Density = pPorety->m_pMaterial.lock()->m_Density;
        auto m_A = pPorety->m_pSection.lock()->m_Area;
        double m_quality = pele->L0 * m_Density * m_A;
        double m_G = m_quality * m_g * loadScale / 2.;

        for (auto& NodePtr : pele->m_pNode)
        {
            auto pNode = NodePtr.lock();
            int dof = pNode->m_DOF[iDirection];
            if (dof >= 0 && dof < m_nFixed)
            {
                F1[dof] += 0.;
            }
            else if (dof >= m_nFixed && dof < (m_nFixed + m_nFree))
            {
                F2[dof - m_nFixed] += m_G;
            }
        }
    }
}

void AnalysisStep::Assemble_ForceWind(Force_Wind* pForceWind, VectorXd& F1, VectorXd& F2, double& current_time, double loadScale)
{
    int iDirection = static_cast<int>(pForceWind->m_Direction);
    double m_v = pForceWind->m_velocity;
    double m_vDensity = pForceWind->m_windDensity;
    for (auto& pElement : m_pData->m_Elements)
    {
        auto pele = pElement.second;
        auto pPorety = pele->m_pProperty.lock();

        auto m_r = pPorety->m_pSection.lock()->m_Radius;
        double m_A = pele->L0 * m_r;
        double m_Fwind = 0.5 * m_vDensity * m_v * m_v * m_A * loadScale / 2.0;

        for (auto& NodePtr : pele->m_pNode)
        {
            auto pNode = NodePtr.lock();
            int dof = pNode->m_DOF[iDirection];
            if (dof >= 0 && dof < m_nFixed)
            {
                F1[dof] += 0.;
            }
            else if (dof >= m_nFixed && dof < (m_nFixed + m_nFree))
            {
                F2[dof - m_nFixed] += m_Fwind;
            }
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
    Init();

    // 初始化约束和单元长度
    VectorXd x1;
    Assemble_Constraint(x1);
    Get_ElementLength();

    // 使用工厂模式创建求解器
    std::unique_ptr<SolverNameSpace::ISolver> solver;

    switch (m_Type)
    {
    case EnumKeyword::StepType::STATIC:
    {
        SolverNameSpace::SolverStatic::Params p;
        p.numIncrements = static_cast<int>(m_Time / m_StepSize);
        if (p.numIncrements < 1) p.numIncrements = 1;
        p.maxIter = m_MaxIterations;
        solver = std::make_unique<SolverNameSpace::SolverStatic>(p);
        break;
    }
    case EnumKeyword::StepType::DYNAMIC:
    {
        // 根据 m_DynamicSolverType 选择动力求解器
        switch (m_DynamicSolverType)
        {
        case SolverNameSpace::SolverType::Newmark:
        {
            SolverNameSpace::SolverNewmark::Params p;
            p.dt = m_StepSize;
            p.maxIter = m_MaxIterations;
            p.tol = m_Tolerance;
            solver = std::make_unique<SolverNameSpace::SolverNewmark>(p);
            break;
        }
        case SolverNameSpace::SolverType::CentralDifference:
            qDebug().noquote() << QStringLiteral("警告: CentralDifference 求解器尚未实现，使用 Newmark");
            // TODO: solver = std::make_unique<SolverNameSpace::SolverCentralDiff>(p);
            {
                SolverNameSpace::SolverNewmark::Params p;
                p.dt = m_StepSize;
                p.maxIter = m_MaxIterations;
                p.tol = m_Tolerance;
                solver = std::make_unique<SolverNameSpace::SolverNewmark>(p);
            }
            break;
        case SolverNameSpace::SolverType::HHT:
            qDebug().noquote() << QStringLiteral("警告: HHT 求解器尚未实现，使用 Newmark");
            // TODO: solver = std::make_unique<SolverNameSpace::SolverHHT>(p);
            {
                SolverNameSpace::SolverNewmark::Params p;
                p.dt = m_StepSize;
                p.maxIter = m_MaxIterations;
                p.tol = m_Tolerance;
                solver = std::make_unique<SolverNameSpace::SolverNewmark>(p);
            }
            break;
        default:
        {
            SolverNameSpace::SolverNewmark::Params p;
            p.dt = m_StepSize;
            p.maxIter = m_MaxIterations;
            p.tol = m_Tolerance;
            solver = std::make_unique<SolverNameSpace::SolverNewmark>(p);
            break;
        }
        }
        break;
    }
    default:
        qDebug().noquote() << QStringLiteral("警告: 未知的分析步类型，无法求解");
        return;
    }

    // 执行求解
    if (solver && !solver->Solve(*this, m_Time))
    {
        qDebug().noquote() << QStringLiteral("求解失败: %1").arg(solver->GetName());
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
    Get_ElementLength();
    // 残差向量
    VectorXd residual;

    int numIncrements = m_Time / m_StepSize; // 可由用户在输入文件中定义
    qDebug().noquote() << QStringLiteral("分%1步施加荷载").arg(numIncrements);
    for (int inc = 1; inc <= numIncrements; ++inc)
    {
        double currentFactor = (double)inc / numIncrements;
        double currentTime = inc * m_StepSize;  // 当前时刻

        // 组装外荷载（时间步开始时做一次）
        ComputeExternalForce(currentTime, currentFactor, F1, F2);

        //std::cout << "\nF2:" << VectorXd(F2).transpose();
        // Newton-Raphson 迭代
        for (int iter = 0; iter < m_MaxIterations; iter++)
        {
            // 1. 组装刚度矩阵 (基于当前变形状态)
            AssembleKs_Static();
            std::cout << "\nK22:\n" << MatrixXd(m_K22);

            // 2. 计算残差（使用已组装的外荷载）
            ComputeResidual(F2, residual);

            // 3. 检查收敛性
            if (residual.norm() < m_Tolerance && iter > 0)
            {
                //qDebug().noquote() << QStringLiteral("迭代在第 %1 步收敛").arg(iter);
                break;
            }

            // 4. 求解线性方程组 K22 * Δu = residual
            Eigen::SimplicialLDLT<SpMat> ldltSolver;
            ldltSolver.analyzePattern(m_K22);
            ldltSolver.factorize(m_K22);
            if (ldltSolver.info() != Success)
            {
                qDebug().noquote() << QStringLiteral("LDLT分解失败!");
                return;
            }

            x2 = ldltSolver.solve(residual);
            //std::cout << "\nx2: " << VectorXd(x2).transpose();
            F1 = m_K11 * x1 + m_K21.transpose() * x2;

            // 5. 累加位移增量
            totalx2 += x2;

            // 6. 更新节点位移
            Updata_NodeData(x1, x2, F1);

            // 7. 检查是否达到最大迭代次数
            if (iter == m_MaxIterations - 1)
            {
                qDebug().noquote() << QStringLiteral("\n达最大迭代次数\n");
                exit(1);
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
    qDebug().noquote() << QStringLiteral("开始 Newmark 动力非线性求解...");

    // 1. 初始化 Newmark 参数
    double beta = 0.25;
    double gamma = 0.5;
    double dt = m_StepSize;

    if (dt <= 0.0)
    {
        qDebug() << "Error: Time step size <= 0";
        return;
    }

    double a0 = 1.0 / (beta * dt * dt);
    double a1 = gamma / (beta * dt);
    double a2 = 1.0 / (beta * dt);
    double a3 = 1.0 / (2.0 * beta) - 1.0;
    double a4 = gamma / beta - 1.0;
    double a5 = dt * 0.5 * (gamma / beta - 2.0);
    double a6 = dt * (1.0 - gamma);
    double a7 = gamma * dt;

    // 准备向量
    VectorXd F1, F2, x1;
    VectorXd residual(m_nFree), internalForce(m_nFree);

    // 状态备份向量 (上一时刻 t)
    VectorXd U_n(m_nFree), V_n(m_nFree), A_n(m_nFree);

    // 过程向量
    VectorXd total_x2(m_nFree);      // 当前步的总位移增量
    VectorXd dx2(m_nFree);           // 每轮迭代的位移修正量

    // 重置求解器缓存
    m_solverCache.reset();

    Assemble_Constraint(x1);
    Get_ElementLength();

    int numSteps = (int)(m_Time / m_StepSize);

    for (int step = 1; step <= numSteps; ++step)
    {
        double currentTime = step * m_StepSize;


        // --- 步骤 1: 备份上一时刻 (t) 状态 ---
        Get_CurrentStepState(U_n, V_n, A_n);


        // 初始化当前步累积增量
        total_x2.setZero();

        // 组装外荷载（时间步开始时做一次）
        double factor = 1.0;
        ComputeExternalForce(currentTime, factor, F1, F2);

        // --- Newton-Raphson 迭代 ---
        for (int iter = 0; iter < m_MaxIterations; iter++)
        {

            // A. 组装 M, C, K
            Assemble_Matrix();

            // B. 计算有效刚度矩阵: K_eff = K + a0*M + a1*C
            SpMat K_eff = m_K22 + a0 * m_M22 + a1 * m_C22;

            // C. 设置试探的速度和加速度
            VectorXd A_curr = a0 * total_x2 - a2 * V_n - a3 * A_n;
            VectorXd V_curr = V_n + a6 * A_n + a7 * A_curr;

            // 将速度和加速度设置到节点（用于计算惯性力）
            VectorXd zero_dx = VectorXd::Zero(m_nFree);
            Updata_NodeData(x1, zero_dx, F1, &V_curr, &A_curr);

            // D. 计算残差（使用已组装的外荷载）
            ComputeResidual(F2, residual);

            // E. 检查收敛
            if (residual.norm() < m_Tolerance && iter > 0)
            {
                //qDebug().noquote() << QStringLiteral("迭代在第 %1 步收敛").arg(iter);
                break;
            }

            // F. 求解线性方程组 (集成 LDLT/LU 自动切换)
            bool solved = false;

            // G.1 尝试 LDLT
            if (m_solverCache.use_ldlt)
            {
                if (!m_solverCache.pattern_analyzed)
                {
                    m_solverCache.ldlt.analyzePattern(K_eff);
                }
                // 只有当 pattern 已分析后才进行 factorize
                m_solverCache.ldlt.factorize(K_eff);

                if (m_solverCache.ldlt.info() == Eigen::Success)
                {
                    dx2 = m_solverCache.ldlt.solve(residual);
                    if (m_solverCache.ldlt.info() == Eigen::Success)
                    {
                        solved = true;
                        m_solverCache.pattern_analyzed = true; // 标记分析成功，下一次可复用
                    }
                }

                if (!solved)
                {
                    qDebug() << "LDLT failed at step" << step << "iter" << iter << ", switching to LU...";
                    m_solverCache.use_ldlt = false;
                    m_solverCache.pattern_analyzed = false; // 切换求解器，模式需重置
                }
            }

            // G.2 如果 LDLT 失败或已禁用，尝试 LU
            if (!solved)
            {
                if (!m_solverCache.pattern_analyzed)
                {
                    m_solverCache.lu.analyzePattern(K_eff);
                    m_solverCache.pattern_analyzed = true;
                }
                m_solverCache.lu.factorize(K_eff);

                if (m_solverCache.lu.info() == Eigen::Success)
                {
                    dx2 = m_solverCache.lu.solve(residual);
                    solved = true;
                }
                else
                {
                    qDebug() << "LU factorization failed!";
                    // 这里可以尝试重置 pattern 再试一次，或者直接报错
                    m_solverCache.pattern_analyzed = false;
                }
            }

            if (!solved)
            {
                throw std::runtime_error("矩阵分解失败！");
            }

            if (iter == m_MaxIterations - 1)
            {
                throw std::runtime_error("Newton-Raphson迭代未收敛，已达最大迭代次数！");
            }

            // H. 累加总位移增量
            total_x2 += dx2;

            // I. 更新节点位移 (仅位移)
            // Updata_NodeData 负责将 dx2 累加到 m_Displacement，从而影响下一次 Assemble_Matrix 和 Get_CurrentInforce
            Updata_NodeData(x1, dx2, F1);
        }

        VectorXd A_final = a0 * total_x2 - a2 * V_n - a3 * A_n;
        VectorXd V_final = V_n + a6 * A_n + a7 * A_final;

        VectorXd zero_dx(m_nFree);
        zero_dx.setZero();

        Updata_NodeData(x1, zero_dx, F1, &V_final, &A_final);
        // 保存输出
        if (m_pData) m_pData->GetOutputter().SaveDataFromNodes(currentTime, m_pData);
    }

    qDebug().noquote() << QStringLiteral("动力求解完成");
}

// ==========================================
// 新增辅助函数实现
// ==========================================

void AnalysisStep::Get_CurrentStepState(VectorXd& U, VectorXd& V, VectorXd& A) const
{
    // 调整向量大小
    if (U.size() != m_nFree) U.resize(m_nFree);
    if (V.size() != m_nFree) V.resize(m_nFree);
    if (A.size() != m_nFree) A.resize(m_nFree);

    U.setZero(); V.setZero(); A.setZero();

    // 遍历节点提取自由自由度数据
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        for (int i = 0; i < numDOF; ++i)
        {
            int globalDof = pNode->m_DOF[i];

            // 仅提取自由自由度 (>= m_nFixed)
            if (globalDof >= m_nFixed && globalDof < (m_nFixed + m_nFree))
            {
                int idx = globalDof - m_nFixed;

                if (i < pNode->m_Displacement.size()) U[idx] = pNode->m_Displacement[i];
                if (i < pNode->m_Velocity.size())     V[idx] = pNode->m_Velocity[i];
                if (i < pNode->m_Acceleration.size()) A[idx] = pNode->m_Acceleration[i];
            }
        }
    }
}

// ==========================================
// IAnalysisModel 接口实现
// ==========================================

void AnalysisStep::ApplyIncrement(const SolverNameSpace::Vec& dx)
{
    // 将位移增量应用到节点
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        if (numDOF == 6) // 针对 6 自由度空间节点
        {
            for (int dofIdx = 0; dofIdx < 3; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    int idx = dof - m_nFixed;
                    pNode->m_Displacement[dofIdx] += dx[idx];
                }
            }

            // 提取本次迭代产生的微小旋转增量 delta_theta
            Eigen::Vector3d delta_theta = Eigen::Vector3d::Zero();
            for (int dofIdx = 3; dofIdx < 6; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    int idx = dof - m_nFixed;
                    delta_theta(dofIdx - 3) = dx[idx];
                }
            }

            Eigen::Matrix3d R_temp;
            Utility::CR::Update_NodalRotation(delta_theta, pNode->m_Rg, R_temp);
            pNode->m_Rg = R_temp;
            Eigen::Vector3d theta;
            Utility::CR::Extract_RotationVector(R_temp, theta);
            pNode->m_Displacement[3] = theta(0);
            pNode->m_Displacement[4] = theta(1);
            pNode->m_Displacement[5] = theta(2);
            //std::cout << "\ntheta:" << theta.transpose() << std::endl;
        }
        else // 对于非 6 自由度的普通节点（保留你原有的逻辑）
        {

            for (int dofIdx = 0; dofIdx < numDOF; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    int idx = dof - m_nFixed;
                    pNode->m_Displacement[dofIdx] += dx[idx];
                }
            }

        }
    }
}

void AnalysisStep::SetTrialKinematics(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a)
{
    // 将速度和加速度设置到节点
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        // 确保向量大小足够
        if (pNode->m_Velocity.size() < numDOF)
            pNode->m_Velocity.resize(numDOF, 0.0);
        if (pNode->m_Acceleration.size() < numDOF)
            pNode->m_Acceleration.resize(numDOF, 0.0);

        for (int dofIdx = 0; dofIdx < numDOF; ++dofIdx)
        {
            int dof = pNode->m_DOF[dofIdx];

            if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
            {
                int idx = dof - m_nFixed;
                pNode->m_Velocity[dofIdx] = v[idx];
                pNode->m_Acceleration[dofIdx] = a[idx];
            }
        }
    }
}

void AnalysisStep::GetState(SolverNameSpace::Vec& u, SolverNameSpace::Vec& v, SolverNameSpace::Vec& a) const
{
    // 直接调用现有的 Get_CurrentStepState
    Get_CurrentStepState(u, v, a);
}

void AnalysisStep::AssembleMatrices(SolverNameSpace::SpMat& K, SolverNameSpace::SpMat* M, SolverNameSpace::SpMat* C)
{
    if (M && C)
    {
        // 动力学：组装 K, M, C
        Assemble_Matrix();
        K = m_K22;
        *M = m_M22;
        *C = m_C22;
    }
    else
    {
        // 静力：只组装 K
        AssembleKs_Static();
        K = m_K22;
    }
}

void AnalysisStep::ComputeExternalForce(double time, double loadFactor, VectorXd& F1, VectorXd& F2)
{
    double factor = loadFactor;
    Assemble_AllLoads(F1, F2, factor, time);
}

void AnalysisStep::ComputeResidual(const SolverNameSpace::Vec& F_ext, SolverNameSpace::Vec& R)
{
    // 计算内力
    VectorXd f_int(m_nFree);
    f_int.setZero();

    // 清零节点内力
    for (auto& nodePair : m_pData->m_Nodes)
    {
        std::fill(nodePair.second->m_Force.begin(), nodePair.second->m_Force.end(), 0.0);
    }
    Get_CurrentInforce(f_int);

    // 动力学：加上惯性力和阻尼力
    if (m_Type == EnumKeyword::StepType::DYNAMIC)
    {
        VectorXd v_curr = GetCurrentVelocity();
        VectorXd a_curr = GetCurrentAcceleration();
        f_int += m_M22 * a_curr + m_C22 * v_curr;
    }

    R = F_ext - f_int;
}

void AnalysisStep::OnStepCompleted(double time)
{
    if (m_pData)
    {
        m_pData->GetOutputter().SaveDataFromNodes(time, m_pData);
    }
}

VectorXd AnalysisStep::GetCurrentVelocity() const
{
    VectorXd v(m_nFree);
    v.setZero();

    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        for (int i = 0; i < numDOF; ++i)
        {
            int dof = pNode->m_DOF[i];
            if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
            {
                int idx = dof - m_nFixed;
                if (i < pNode->m_Velocity.size())
                    v[idx] = pNode->m_Velocity[i];
            }
        }
    }
    return v;
}

VectorXd AnalysisStep::GetCurrentAcceleration() const
{
    VectorXd a(m_nFree);
    a.setZero();

    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        for (int i = 0; i < numDOF; ++i)
        {
            int dof = pNode->m_DOF[i];
            if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
            {
                int idx = dof - m_nFixed;
                if (i < pNode->m_Acceleration.size())
                    a[idx] = pNode->m_Acceleration[i];
            }
        }
    }
    return a;
}

void AnalysisStep::BackupStepState()
{
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        pNode->m_Displacement_n = pNode->m_Displacement;
        pNode->m_Rg_n = pNode->m_Rg; // 备份上一步的绝对旋转矩阵
    }
}

void AnalysisStep::GetStepIncrement(SolverNameSpace::Vec& dx_step) const
{
    dx_step.resize(m_nFree);
    dx_step.setZero();

    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        if (numDOF == 6)
        {
            // 1. 平动自由度：直接线性相减
            for (int dofIdx = 0; dofIdx < 3; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    dx_step[dof - m_nFixed] = pNode->m_Displacement[dofIdx] - pNode->m_Displacement_n[dofIdx];
                }
            }

            // 2. 转动自由度：必须通过矩阵相对变换求出步内增量！
            // 由于你在 ApplyIncrement 中使用的是左乘 R_new = delta_R * R_old
            // 所以当前的增量旋转矩阵 R_step = R_curr * R_n^T
            Eigen::Matrix3d R_step = pNode->m_Rg * pNode->m_Rg_n.transpose();
            Eigen::Vector3d delta_theta_step;
            Utility::CR::Extract_RotationVector(R_step, delta_theta_step);

            for (int dofIdx = 3; dofIdx < 6; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    // 填入真实的 SO(3) 空间角位移增量
                    dx_step[dof - m_nFixed] = delta_theta_step(dofIdx - 3);
                }
            }
        }
        else // 对于非 6 自由度节点
        {
            for (int dofIdx = 0; dofIdx < numDOF; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    dx_step[dof - m_nFixed] = pNode->m_Displacement[dofIdx] - pNode->m_Displacement_n[dofIdx];
                }
            }
        }
    }
}
