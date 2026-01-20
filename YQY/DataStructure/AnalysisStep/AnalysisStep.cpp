#include "AnalysisStep.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementBase.h"

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
    AssembleKs();

    // TODO: 根据类型调用求解
    // Solve();
}

void AnalysisStep::Init_DOF()
{
    // m_pData 已由 PrepareData() 准备好

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
        for (auto& a : pNode->m_DOF)
        {
            if (-1 == a) a = iStart++;
        }
    }

    m_nFree = iStart - m_nFixed;
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
        pelement->Get_ke(ke);
        pelement->GetDOFs(DOFs);
        Assemble(DOFs, ke, L11, L21, L22);
    }

    m_K11.setFromTriplets(L11.begin(), L11.end());
    m_K21.setFromTriplets(L21.begin(), L21.end());
    m_K22.setFromTriplets(L22.begin(), L22.end());

    std::cout << MatrixXd(m_K22);
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

void AnalysisStep::Assemble_AllLoads(VectorXd& F1, VectorXd& F2)
{
    F1.resize(m_nFixed);
    F1.setZero();
    F2.resize(m_nFree);
    F2.setZero();
    for (auto& Load : m_pData->m_Load)
    {
        auto pLoadBase = Load.second;
        auto Loadtype = pLoadBase->m_LoadType;
        switch (Loadtype)
        {
        case EnumKeyword::LoadType::FORCE_NODE:
        {
            // 向下转型为 Force_Node
            auto pForceNode = std::dynamic_pointer_cast<Force_Node>(pLoadBase);
            if (!pForceNode) continue;

            Assemble_ForceNode(pForceNode.get(), F1, F2, m_Time);
            break;
        }
        case EnumKeyword::LoadType::FORCE_ELEMENT:
        {
            // 向下转型为 Force_Element
            auto pForceElement = std::dynamic_pointer_cast<Force_Element>(pLoadBase);
            if (!pForceElement) continue;

            Assemble_ForceElement(pForceElement.get(), F1, F2, m_Time);
            break;
        }
        default:
            break;
        }
    }
}

VectorXd AnalysisStep::Get_currentForce(double& current_time)
{
    return VectorXd();
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
}


void AnalysisStep::AssembleConstraint(VectorXd& x1)
{
    x1.resize(m_nFixed);
    for (auto& a : m_pData->m_Constraint)
    {
        auto pConstraint = a.second;
        auto pNode = pConstraint->m_pNode.lock();
        if (!pNode) continue;

        int iDirection = static_cast<int>(pConstraint->m_Direction);
        if (iDirection < 0 || iDirection >= pNode->m_DOF.size()) continue;

        int dof = pNode->m_DOF[iDirection];
        if (dof >= 0 && dof < m_nFixed)
        {
            x1[dof] = pConstraint->m_Value;
        }
    }
}

