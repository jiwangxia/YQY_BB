#include "AnalysisStep.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementBase.h"
#include "Solver/Interface/ISolver.h"
#include "Solver/SolverFactory.h"
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

//void AnalysisStep::Assemble_Matrix()
//{
//    std::list<Tri> m11, m21, m22;
//    std::list<Tri> k11, k21, k22;
//    std::list<Tri> c11, c21, c22;
//
//    m_M11.resize(m_nFixed, m_nFixed);
//    m_M21.resize(m_nFree, m_nFixed);
//    m_M22.resize(m_nFree, m_nFree);
//
//    m_K11.resize(m_nFixed, m_nFixed);
//    m_K21.resize(m_nFree, m_nFixed);
//    m_K22.resize(m_nFree, m_nFree);
//
//    m_C11.resize(m_nFixed, m_nFixed);
//    m_C21.resize(m_nFree, m_nFixed);
//    m_C22.resize(m_nFree, m_nFree);
//
//    std::vector<int> DOFs;
//    std::vector<double> damping{ 0.0 ,0.0 ,0.0 ,0.0 };
//
//    MatrixXd me, ke, ce;
//    for (auto& element : m_pData->m_Elements)
//    {
//        auto pelement = element.second;
//        pelement->Get_ke_non(ke);
//        pelement->Get_me_Consistent(me);
//        //pelement->Get_me_Lumped(me);//杆单元用集中质量矩阵求解动力学与abaqus一致
//        pelement->Assemble(damping, ce);
//
//        pelement->GetDOFs(DOFs);
//        Assemble(DOFs, me, m11, m21, m22);
//        Assemble(DOFs, ke, k11, k21, k22);
//        Assemble(DOFs, ce, c11, c21, c22);
//    }
//
//    m_M11.setFromTriplets(m11.begin(), m11.end());
//    m_M21.setFromTriplets(m21.begin(), m21.end());
//    m_M22.setFromTriplets(m22.begin(), m22.end());
//
//    m_K11.setFromTriplets(k11.begin(), k11.end());
//    m_K21.setFromTriplets(k21.begin(), k21.end());
//    m_K22.setFromTriplets(k22.begin(), k22.end());
//
//    m_C11.setFromTriplets(c11.begin(), c11.end());
//    m_C21.setFromTriplets(c21.begin(), c21.end());
//    m_C22.setFromTriplets(c22.begin(), c22.end());
//}

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

void AnalysisStep::Assemble_Constraint(
    VectorXd& x1,
    double currentTime,
    double factor)
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
            const double value =
                pConstraint->GetValue(currentTime, factor);
            x1[dof] = value;
            pNode->m_Displacement[iDirection] = value;
        }
    }
}

// ==========================================
// 求解方法实现
// ==========================================

bool AnalysisStep::Solve()
{
    if (!PrepareData()) return false;
    Init();

    // 初始化单元长度
    Get_ElementLength();

    //solver为求解器类型指针，使用工厂模式创建对应的求解器实例
    auto solver = SolverNameSpace::SolverFactory::Create_StepForSlover(*this);
    if (!solver)
    {
        qDebug().noquote() << QStringLiteral("警告: 未知的分析步类型，无法求解");
        return false;
    }

    const bool outputHdf5 = m_pData->m_OutputControl.m_EnableHdf5;
    const bool dynamicAnalysis = (m_Type == EnumKeyword::StepType::DYNAMIC);
    m_pData->GetOutputter().SetKeepFramesInMemory(!dynamicAnalysis);

    if (dynamicAnalysis && outputHdf5)
    {
        if (!m_pData->GetOutputter().BeginHdf5ResultStream(
            m_pData->m_OutputControl.m_Hdf5FileName,
            m_pData,
            m_pData->m_OutputControl.m_SourceModelName))
        {
            qDebug().noquote() << QStringLiteral("Error: H5/HDF5 动力结果流式输出初始化失败");
            return false;
        }
    }

    // 执行求解
    bool solveOk = true;
    if (solver && !solver->Solve(*this, m_Time))
    {
        qDebug().noquote() << QStringLiteral("求解失败: %1").arg(solver->GetName());
        solveOk = false;
    }

    if (dynamicAnalysis && outputHdf5)
    {
        m_pData->GetOutputter().EndHdf5ResultStream();
    }
    else if (!dynamicAnalysis && outputHdf5 && solveOk)
    {
        m_pData->GetOutputter().SaveHdf5File(
            m_pData->m_OutputControl.m_Hdf5FileName,
            m_pData,
            m_pData->m_OutputControl.m_SourceModelName);
    }
    else if (!solveOk)
    {
        qDebug().noquote() << QStringLiteral("求解失败，跳过结果文件保存");
    }

    return solveOk;
}

// ==========================================
// 新增辅助函数实现
// ==========================================

void AnalysisStep::CalculateReactions(VectorXd& F1)
{
    // 迭代已收敛，节点里的 m_Force 是结构处于平衡状态下的真实总内力
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        // 确保数组大小足够
        if (pNode->m_ReactionForce.size() < numDOF)
            pNode->m_ReactionForce.resize(numDOF, 0.0);

        for (int i = 0; i < numDOF; ++i)
        {
            int dof = pNode->m_DOF[i];

            if (dof >= 0 && dof < m_nFixed)
            {
                // 约束自由度：计算真实反力
                // F1 是之前 ComputeExternalForce 存入的约束节点外荷载
                double extForce = (dof < F1.size()) ? F1[dof] : 0.0;

                // 公式：反力 = 内力 - 外力
                pNode->m_ReactionForce[i] = pNode->m_Force[i] - extForce;
            }
            else
            {
                // 自由自由度：没有支座反力
                pNode->m_ReactionForce[i] = 0.0;
            }
        }
    }
}

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

void AnalysisStep::BeginDynamicStep(double dt, double beta, double gamma)
{
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        std::array<bool, 3> translationActive{ false, false, false };
        std::array<bool, 3> rotationActive{ false, false, false };

        for (int i = 0; i < pNode->m_DOF.size() && i < 6; ++i)
        {
            const int dof = pNode->m_DOF[i];
            const bool isFree = dof >= m_nFixed && dof < m_nFixed + m_nFree;
            if (i < 3) translationActive[i] = isFree;
            else rotationActive[i - 3] = isFree;
        }

        pNode->BeginNewmarkStep(dt, beta, gamma,
            translationActive, rotationActive);
    }
}

void AnalysisStep::ApplyDynamicCorrection(const SolverNameSpace::Vec& dx,
    double a0, double a1)
{
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        Eigen::Vector3d deltaTranslation = Eigen::Vector3d::Zero();
        Eigen::Vector3d deltaRotation = Eigen::Vector3d::Zero();

        for (int i = 0; i < pNode->m_DOF.size() && i < 6; ++i)
        {
            const int dof = pNode->m_DOF[i];
            if (dof < m_nFixed || dof >= m_nFixed + m_nFree) continue;

            const double correction = dx[dof - m_nFixed];
            if (i < 3) deltaTranslation(i) = correction;
            else deltaRotation(i - 3) = correction;
        }

        pNode->ApplyNewmarkCorrection(deltaTranslation, deltaRotation, a0, a1);
    }
}

void AnalysisStep::RollbackDynamicStep()
{
    for (auto& nodePair : m_pData->m_Nodes)
    {
        nodePair.second->RollbackNewmarkStep();
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

void AnalysisStep::Assemble_Matrix(SpMat& Keff, bool isDynamic)
{
    std::list<Tri> L11, L21, L22;

    m_Keff11.resize(m_nFixed, m_nFixed);
    m_Keff21.resize(m_nFree, m_nFixed);
    m_Keff22.resize(m_nFree, m_nFree);

    MatrixXd ke;
    std::vector<int> DOFs;

    for (auto& element : m_pData->m_Elements)
    {
        auto pelement = element.second;
        //pelement->Get_ke(ke);//无内力，目前会出错
        pelement->Get_ke(ke);

        //std::cout << MatrixXd(ke) << "\n";
        pelement->GetDOFs(DOFs);
        Assemble(DOFs, ke, L11, L21, L22);
    }

    m_Keff11.setFromTriplets(L11.begin(), L11.end());
    m_Keff21.setFromTriplets(L21.begin(), L21.end());
    m_Keff22.setFromTriplets(L22.begin(), L22.end());

    Keff = m_Keff22;
    // Fix: 为了防止刚度矩阵奇异（例如竖直杆件受到横向力时初始切线刚度为0），
    // 在对角线上添加一个极小值 epsilon
    //double epsilon = 1e-10;
    //for (int i = 0; i < m_nFree; ++i)
    //{
    //    m_K22.coeffRef(i, i) += epsilon;
    //}

    //qDebug();
    //std::cout << MatrixXd(Keff);
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
    //std::cout << "\nF:" << f_int.transpose() << "\n";
    // 动力学：加上惯性力和阻尼力
    if (m_Type == EnumKeyword::StepType::DYNAMIC)
    {
        VectorXd v_curr = GetCurrentVelocity();
        VectorXd a_curr = GetCurrentAcceleration();
        //f_int += m_M22 * a_curr + m_C22 * v_curr;
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

void AnalysisStep::CommitState()
{
    if (!m_pData)
    {
        return;
    }

    for (auto& elementPair : m_pData->m_Elements)
    {
        elementPair.second->CommitState();
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
