#include "AnalysisStep.h"
#include "Application/ApplicationPaths.h"
#include <algorithm>
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Aerodynamics/AeroManager.h"
#include "DataStructure/Load/LoadAssembler.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Wind.h"
#include "Solver/Interface/ISolver.h"
#include "Solver/SolverFactory.h"
#include <Eigen/SparseCholesky>

AeroCaseKey AnalysisStep::GetGallopingAeroCase(
    int bundleCount, const Force_Wind& wind) const
{
    return AeroCaseKey{ bundleCount, static_cast<int>(std::llround(wind.m_velocity)),
        m_GallopingIceThickness };
}

bool AnalysisStep::ShouldAssembleGalloping(
    int bundleCount, const Force_Wind& wind) const
{
    return isDynamic && m_EnableGalloping
        && std::abs(wind.m_velocity - std::llround(wind.m_velocity)) <= 1.0e-9
        && AeroManager::isSupportedCase(GetGallopingAeroCase(bundleCount, wind));
}

bool AnalysisStep::IsStepScopedDataActive(int sourceStepId) const
{
    // Step 0 is the legacy/global scope and is active in every branch.
    if (sourceStepId <= 0)
        return true;
    if (sourceStepId == m_Id)
        return true;

    if (m_Type == EnumKeyword::StepType::DYNAMIC
        && m_InitialStaticStepId > 0)
    {
        // A dependent dynamic analysis starts from the selected static
        // equilibrium.  Only definitions that were already active in that
        // equilibrium are inherited.  Definitions from earlier sibling
        // dynamics must not leak into this branch merely because their IDs
        // are smaller.
        return sourceStepId <= m_InitialStaticStepId;
    }

    // Preserve the existing sequential semantics for static analyses and
    // standalone dynamics that do not reference an equilibrium step.
    return sourceStepId < m_Id;
}

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
        if (!pConstrain || !IsStepScopedDataActive(pConstrain->m_StepId)) continue;
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
        if (m_initializeFromCurrentState)
        {
            // 静力找形后的位移和有限转动就是动力步的 t=0 构型。
            // 速度与加速度不从静力伪时间继承，默认从静止平衡状态启动。
            pNode->m_Displacement.resize(numDOF, 0.0);
            pNode->m_Displacement_n = pNode->m_Displacement;
            pNode->m_Rg_n = pNode->m_Rg;
        }
        else
        {
            // 独立求解从原始模型几何开始，避免重复计算继承上一次结果。
            pNode->m_Displacement.assign(numDOF, 0.0);
            pNode->m_Displacement_n.assign(numDOF, 0.0);
            pNode->m_Rg.setIdentity();
            pNode->m_Rg_n.setIdentity();
        }
        pNode->m_Velocity.assign(numDOF, 0.0);
        pNode->m_Velocity_n.assign(numDOF, 0.0);
        pNode->m_Acceleration.assign(numDOF, 0.0);
        pNode->m_Acceleration_n.assign(numDOF, 0.0);
        pNode->m_Force.assign(numDOF, 0.0);
        pNode->m_ReactionForce.assign(numDOF, 0.0);
        pNode->m_OmegaMaterial.setZero();
        pNode->m_AlphaMaterial.setZero();
        pNode->m_OmegaMaterial_n.setZero();
        pNode->m_AlphaMaterial_n.setZero();
        pNode->m_StepRotation.setZero();
    }
    m_initializeFromCurrentState = false;
}

bool AnalysisStep::ValidateGallopingConfiguration(QString* errorMessage) const
{
    if (!m_EnableGalloping)
        return true;
    if (!isDynamic)
    {
        if (errorMessage) *errorMessage = QStringLiteral("舞动气动力只能用于动力分析步。");
        return false;
    }

    const Force_Wind* selectedWind = nullptr;
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        const auto wind = std::dynamic_pointer_cast<Force_Wind>(load);
        if (!wind || !IsStepScopedDataActive(wind->m_StepId))
            continue;
        if (selectedWind)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("舞动分析步只能继承一个风荷载，请合并或删除重复风荷载。");
            return false;
        }
        selectedWind = wind.get();
    }
    if (!selectedWind)
    {
        if (errorMessage) *errorMessage = QStringLiteral("启用舞动前必须给分析步配置风荷载。");
        return false;
    }
    if (!selectedWind->m_direction.allFinite()
        || selectedWind->m_direction.norm() <= 1.0e-12)
    {
        if (errorMessage) *errorMessage = QStringLiteral("舞动风荷载必须提供非零的全局风向向量。");
        return false;
    }
    if (!std::isfinite(m_GallopingInitialAttackDegrees))
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("初始风攻角必须是有限数值。");
        return false;
    }
    if (selectedWind->m_FunctionType != TimeFunctionType::CONSTANT)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("当前离散气动参数只支持恒定风速，不能使用时变风速函数。");
        return false;
    }
    const double roundedSpeed = std::llround(selectedWind->m_velocity);
    if (std::abs(selectedWind->m_velocity - roundedSpeed) > 1.0e-9
        || !AeroManager::isSupportedWindSpeed(static_cast<int>(roundedSpeed)))
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("风速必须为 10、12、14 或 18 m/s。");
        return false;
    }
    return true;
}

bool AnalysisStep::PrepareGallopingData(QString* errorMessage)
{
    if (!m_EnableGalloping)
        return true;

    std::shared_ptr<Force_Wind> wind;
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        const auto candidate = std::dynamic_pointer_cast<Force_Wind>(load);
        if (candidate && IsStepScopedDataActive(candidate->m_StepId))
        {
            wind = candidate;
            break;
        }
    }
    if (!wind)
        return false;

    int legacyBundleCount = 1;
    std::set<int> bundleCounts;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad())
            continue;
        if (element->m_AeroBundleCount > 0)
            bundleCounts.insert(element->m_AeroBundleCount);
        legacyBundleCount = std::max(
            legacyBundleCount, element->m_AeroProfileId + 1);
    }
    if (bundleCounts.empty())
        bundleCounts.insert(legacyBundleCount > 1 ? 4 : 1);

    const std::filesystem::path dataDirectories[] = {
        std::filesystem::path(ApplicationPaths::aerodynamicDataDirectory().toStdWString()),
        std::filesystem::path("YQY/Import/Aero_Data/Input_Data"),
        std::filesystem::path("Import/Aero_Data/Input_Data")
    };
    for (int bundleCount : bundleCounts)
    {
        const AeroCaseKey key = GetGallopingAeroCase(bundleCount, *wind);
        if (!AeroManager::isSupportedCase(key))
        {
            if (errorMessage)
                *errorMessage = QStringLiteral(
                    "当前模型包含 %1 分裂导线，但没有对应的气动参数工况。")
                    .arg(bundleCount);
            return false;
        }
        if (m_pData->m_AeroManager.hasCase(key))
            continue;

        bool loaded = false;
        for (const auto& directory : dataDirectories)
        {
            if (m_pData->m_AeroManager.loadCase(directory, key))
            {
                loaded = true;
                break;
            }
        }
        if (!loaded)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral(
                    "无法加载气动参数：%1 分裂，%2 m/s，覆冰 %3 mm。")
                    .arg(key.bundleCount).arg(key.windSpeed)
                    .arg(key.iceThickness);
            return false;
        }
    }
    return true;
}

Eigen::Vector3d AnalysisStep::GetModelUpDirection() const
{
    if (!m_pData)
        return Eigen::Vector3d::UnitZ();
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        const auto gravity = std::dynamic_pointer_cast<Force_Gravity>(load);
        if (!gravity || !IsStepScopedDataActive(gravity->m_StepId))
            continue;
        const int component = static_cast<int>(gravity->m_Direction);
        if (component >= 0 && component < 3
            && std::abs(gravity->m_g) > 1.0e-12)
        {
            Eigen::Vector3d gravityVector = Eigen::Vector3d::Zero();
            gravityVector[component] = gravity->m_g;
            return -gravityVector.normalized();
        }
    }
    return Eigen::Vector3d::UnitZ();
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

void AnalysisStep::AssembleFreeFree(
    const std::vector<int>& dofs,
    const Eigen::MatrixXd& elementMatrix,
    std::vector<Tri>& freeFree)
{
    const int elementDofCount = static_cast<int>(dofs.size());

    for (int i = 0; i < elementDofCount; ++i)
    {
        const int globalRow = dofs[i];
        if (globalRow < m_nFixed
            || globalRow >= m_nFixed + m_nFree)
        {
            continue;
        }
        for (int j = 0; j < elementDofCount; ++j)
        {
            const int globalColumn = dofs[j];
            if (globalColumn >= m_nFixed
                && globalColumn < m_nFixed + m_nFree)
            {
                freeFree.emplace_back(
                    globalRow - m_nFixed,
                    globalColumn - m_nFixed,
                    elementMatrix(i, j));
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

    const Force_Wind* gallopingWind = nullptr;
    double gallopingScale = 0.0;
    for (auto& Load : m_pData->m_Load)
    {
        auto pLoadBase = Load.second;
        if (!pLoadBase)
            continue;

        double loadScale = 0.0;  // 每个荷载独立的缩放系数

        if (!IsStepScopedDataActive(pLoadBase->m_StepId))
        {
            continue;
        }
        if (pLoadBase->m_StepId != this->m_Id)
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
            loadScale = Factor * pLoadBase->GetScaleFactor(currentTime);
        }
        LoadAssembler::Assemble(*pLoadBase, *m_pData, m_nFixed, loadScale, F1, F2);
        if (m_EnableGalloping
            && pLoadBase->m_LoadType == EnumKeyword::LoadType::FORCE_WIND)
        {
            gallopingWind = static_cast<const Force_Wind*>(pLoadBase.get());
            gallopingScale = loadScale;
        }
    }
    if (gallopingWind && gallopingScale != 0.0)
        LoadAssembler::AssembleGalloping(
            *gallopingWind, *m_pData, m_GallopingIceThickness,
            m_GallopingInitialAttackDegrees,
            GetModelUpDirection(), m_nFixed, gallopingScale, F1, F2);
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




void AnalysisStep::Assemble_Constraint(
    VectorXd& x1,
    double currentTime,
    double factor)
{
    x1.resize(m_nFixed);
    for (auto& constraintPair : m_pData->m_Constraint)
    {
        auto pConstraint = constraintPair.second;
        if (!pConstraint || !IsStepScopedDataActive(pConstraint->m_StepId)) continue;
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
            if (pNode->m_DOF.size() == 6 && iDirection >= 3)
            {
                // Six-DOF beam elements use m_Rg as the authoritative
                // rotational state. Keep an imposed rotation synchronized
                // with the scalar boundary-condition value; otherwise a
                // prescribed UR would be visible in output but invisible to
                // both the element and nonlinear MPC equations.
                const Eigen::Vector3d rotationVector(
                    pNode->m_Displacement[3],
                    pNode->m_Displacement[4],
                    pNode->m_Displacement[5]);
                Eigen::Matrix3d imposedRotation;
                Utility::CR::Update_NodalRotation(
                    rotationVector,
                    Eigen::Matrix3d::Identity(),
                    imposedRotation);
                pNode->m_Rg = imposedRotation;
            }
        }
    }
}

// ==========================================
// 求解方法实现
// ==========================================

bool AnalysisStep::Solve(bool persistHdf5)
{
    if (!PrepareData()) return false;
    QString gallopingError;
    if (!ValidateGallopingConfiguration(&gallopingError))
    {
        qDebug().noquote() << QStringLiteral("舞动配置错误:") << gallopingError;
        return false;
    }
    if (!PrepareGallopingData(&gallopingError))
    {
        qDebug().noquote() << QStringLiteral("舞动气动参数错误:") << gallopingError;
        return false;
    }
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

    const bool dynamicAnalysis = (m_Type == EnumKeyword::StepType::DYNAMIC);
    m_pData->GetOutputter().SetResultContext(m_Id, static_cast<int>(m_Type));
    m_pData->GetOutputter().SetKeepFramesInMemory(
        !dynamicAnalysis || !m_pData->m_OutputControl.m_StreamResult);

    if (dynamicAnalysis && persistHdf5)
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

    // Every analysis result starts with the accepted model state at t = 0.
    // Dynamic analyses write it directly to the opened H5 stream; static
    // analyses keep it in memory and include it in the final H5 export.
    OnStepCompleted(0.0);

    // 执行求解
    bool solveOk = true;
    if (solver && !solver->Solve(*this, m_Time))
    {
        qDebug().noquote() << QStringLiteral("求解失败: %1").arg(solver->GetName());
        solveOk = false;
    }
    const bool solveCompleted = solveOk;

    if (dynamicAnalysis && persistHdf5)
    {
        m_pData->GetOutputter().EndHdf5ResultStream(solveOk);
    }
    else if (!dynamicAnalysis && persistHdf5)
    {
        bool hdf5Saved = false;
        if (m_pData->GetOutputter().GetFrameCount() == 0)
        {
            qDebug().noquote() << QStringLiteral("Error: 静力分析没有可写入 H5 的结果帧");
            solveOk = false;
        }
        else
        {
            hdf5Saved = m_pData->GetOutputter().SaveHdf5File(
                m_pData->m_OutputControl.m_Hdf5FileName,
                m_pData,
                m_pData->m_OutputControl.m_SourceModelName,
                solveOk);
            if (!hdf5Saved)
            {
                qDebug().noquote() << QStringLiteral("Error: H5/HDF5 结果文件保存失败");
                solveOk = false;
            }
        }
        if (!solveCompleted && hdf5Saved)
        {
            qDebug().noquote() << QStringLiteral("静力求解未完成，已保存此前收敛的 %1 帧部分结果")
                .arg(m_pData->GetOutputter().GetFrameCount());
        }
    }
    else if (!solveOk)
    {
        qDebug().noquote() << QStringLiteral("求解失败，跳过结果文件保存");
    }

    return solveOk;
}

void AnalysisStep::SetRuntimeCallbacks(ProgressCallback progressCallback, CancelCallback cancelCallback)
{
    m_progressCallback = std::move(progressCallback);
    m_cancelCallback = std::move(cancelCallback);
}

void AnalysisStep::ClearRuntimeCallbacks()
{
    m_progressCallback = {};
    m_cancelCallback = {};
}

bool AnalysisStep::IsCancellationRequested() const
{
    return m_cancelCallback && m_cancelCallback();
}

void AnalysisStep::ReportProgress(double progress, const QString& message)
{
    if (m_progressCallback)
        m_progressCallback(std::clamp(progress, 0.0, 1.0), message);
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
    int multiplierOffset = 0;
    for (const auto& [id, mpc] : m_pData->m_MPCConstraints)
    {
        if (!mpc || !IsStepScopedDataActive(mpc->m_StepId))
            continue;
        SolverNameSpace::NonlinearMPCData contribution;
        if (!mpc->Evaluate(m_nFixed, m_nFree, contribution))
            continue;
        const int count = static_cast<int>(contribution.value.size());
        if (multiplierOffset + count > m_mpcMultipliers.size())
            break;
        mpc->AccumulateReactions(
            m_nFixed,
            m_mpcMultipliers.segment(multiplierOffset, count));
        multiplierOffset += count;
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

        if (numDOF >= 6)
        {
            Eigen::Vector3d spatialVelocity = Eigen::Vector3d::Zero();
            Eigen::Vector3d spatialAcceleration = Eigen::Vector3d::Zero();
            for (int component = 0; component < 3; ++component)
            {
                spatialVelocity[component] = pNode->m_Velocity[component + 3];
                spatialAcceleration[component] =
                    pNode->m_Acceleration[component + 3];
            }
            pNode->m_OmegaMaterial =
                pNode->m_Rg.transpose() * spatialVelocity;
            pNode->m_AlphaMaterial =
                pNode->m_Rg.transpose() * spatialAcceleration;
            Utility::CR::Extract_RotationVector(
                pNode->m_Rg * pNode->m_Rg_n.transpose(),
                pNode->m_StepRotation);
        }
    }

}

void AnalysisStep::SetTssbnStageKinematics(
    int stageIndex,
    double timeStep,
    double firstStageTime,
    double secondStageTime,
    double secondStageDiagonalFraction,
    SolverNameSpace::Vec& velocity,
    SolverNameSpace::Vec& acceleration)
{
    SetTrialKinematics(velocity, acceleration);
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node || node->m_DOF.size() < 6)
            continue;
        Eigen::Vector3d angularVelocity;
        Eigen::Vector3d angularAcceleration;
        node->SetTssbnStageKinematics(
            stageIndex, timeStep,
            firstStageTime, secondStageTime,
            secondStageDiagonalFraction,
            angularVelocity, angularAcceleration);
        for (int component = 0; component < 3; ++component)
        {
            const int globalDof = node->m_DOF[component + 3];
            if (globalDof < m_nFixed
                || globalDof >= m_nFixed + m_nFree)
            {
                continue;
            }
            const int freeDof = globalDof - m_nFixed;
            velocity[freeDof] = angularVelocity[component];
            acceleration[freeDof] = angularAcceleration[component];
        }
    }
}

void AnalysisStep::CorrectTssbnStepStates(
    double timeStep,
    double firstStageTime,
    double secondStageTime,
    double lastStageTime,
    double baseFirstWeight,
    double embeddedFirstWeight,
    double embeddedSecondWeight,
    double embeddedLastWeight,
    double lastStageFirstCoefficient,
    double lastStageSecondCoefficient,
    SolverNameSpace::Vec& baseIncrement,
    SolverNameSpace::Vec& baseVelocity,
    SolverNameSpace::Vec& embeddedIncrement,
    SolverNameSpace::Vec& embeddedVelocity,
    SolverNameSpace::Vec& acceptedAcceleration)
{
    const double stageSeparation =
        secondStageTime - firstStageTime;
    const double extrapolation =
        (lastStageTime - secondStageTime) / stageSeparation;
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node || node->m_DOF.size() < 6)
            continue;

        const Node::TssbnRotationState rotation =
            node->IntegrateTssbnRotation(
                timeStep, extrapolation, baseFirstWeight,
                embeddedFirstWeight, embeddedSecondWeight,
                embeddedLastWeight, lastStageFirstCoefficient,
                lastStageSecondCoefficient);

        for (int component = 0; component < 3; ++component)
        {
            const int globalDof = node->m_DOF[component + 3];
            if (globalDof < m_nFixed
                || globalDof >= m_nFixed + m_nFree)
            {
                continue;
            }
            const int freeDof = globalDof - m_nFixed;
            baseIncrement[freeDof] =
                rotation.baseSpatialIncrement[component];
            embeddedIncrement[freeDof] =
                rotation.embeddedSpatialIncrement[component];
            baseVelocity[freeDof] =
                rotation.baseSpatialVelocity[component];
            embeddedVelocity[freeDof] =
                rotation.embeddedSpatialVelocity[component];
            acceptedAcceleration[freeDof] =
                rotation.acceptedSpatialAcceleration[component];
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
    std::vector<Tri> freeFree;
    const std::size_t initialTripletCapacity =
        m_pData->m_Elements.size() * 36;
    freeFree.reserve(initialTripletCapacity);

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
        AssembleFreeFree(DOFs, ke, freeFree);
    }

    m_Keff22.setFromTriplets(freeFree.begin(), freeFree.end());

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

void AnalysisStep::AssembleDynamicSystem(
    SpMat& mass, SpMat& gyroscopic, SpMat& centrifugal)
{
    std::vector<Tri> massTriplets;
    std::vector<Tri> gyroscopicTriplets;
    std::vector<Tri> centrifugalTriplets;
    const std::size_t initialTripletCapacity =
        m_pData->m_Elements.size() * 36;
    massTriplets.reserve(initialTripletCapacity);
    gyroscopicTriplets.reserve(initialTripletCapacity);
    centrifugalTriplets.reserve(initialTripletCapacity);
    m_dynamicInertiaForce = VectorXd::Zero(m_nFree);

    DynamicElementData elementData;
    for (const auto& elementPair : m_pData->m_Elements)
    {
        const auto& element = elementPair.second;
        EvaluateDynamicElement(*element, elementData);
        AssembleFreeFree(elementData.dofs, elementData.mass, massTriplets);
        AssembleFreeFree(elementData.dofs,
            elementData.velocityTangent, gyroscopicTriplets);
        AssembleFreeFree(elementData.dofs,
            elementData.configurationTangent, centrifugalTriplets);
        AccumulateDynamicInertiaForce(elementData);
    }

    mass.resize(m_nFree, m_nFree);
    gyroscopic.resize(m_nFree, m_nFree);
    centrifugal.resize(m_nFree, m_nFree);
    mass.setFromTriplets(massTriplets.begin(), massTriplets.end());
    gyroscopic.setFromTriplets(
        gyroscopicTriplets.begin(), gyroscopicTriplets.end());
    centrifugal.setFromTriplets(
        centrifugalTriplets.begin(), centrifugalTriplets.end());
}

void AnalysisStep::AssembleEffectiveDynamicSystem(
    double accelerationDerivative,
    double velocityDerivative,
    SpMat& effectiveDynamicTangent)
{
    std::vector<Tri> triplets;
    triplets.reserve(m_pData->m_Elements.size() * 36);
    m_dynamicInertiaForce = VectorXd::Zero(m_nFree);

    MatrixXd elementEffective;
    DynamicElementData elementData;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        EvaluateDynamicElement(*element, elementData);
        elementEffective =
            accelerationDerivative * elementData.mass
            + velocityDerivative * elementData.velocityTangent
            + elementData.configurationTangent;
        AssembleFreeFree(elementData.dofs, elementEffective, triplets);
        AccumulateDynamicInertiaForce(elementData);
    }

    effectiveDynamicTangent.resize(m_nFree, m_nFree);
    effectiveDynamicTangent.setFromTriplets(
        triplets.begin(), triplets.end());
}

void AnalysisStep::EvaluateDynamicElement(
    ElementBase& element,
    DynamicElementData& result)
{
    element.GetDOFs(result.dofs);
    const int dofCount = static_cast<int>(result.dofs.size());
    result.mass.setZero(dofCount, dofCount);
    result.inertiaForce.setZero(dofCount);
    result.velocityTangent.setZero(dofCount, dofCount);
    result.configurationTangent.setZero(dofCount, dofCount);
    element.GetDynamicContributions(
        result.mass,
        result.inertiaForce,
        result.velocityTangent,
        result.configurationTangent);
}

void AnalysisStep::AccumulateDynamicInertiaForce(
    const DynamicElementData& elementData)
{
    for (int localDof = 0;
        localDof < static_cast<int>(elementData.dofs.size());
        ++localDof)
    {
        const int freeDof = elementData.dofs[localDof] - m_nFixed;
        if (freeDof >= 0 && freeDof < m_nFree)
            m_dynamicInertiaForce[freeDof] += elementData.inertiaForce[localDof];
    }
}

bool AnalysisStep::AssembleNonlinearMPC(
    SolverNameSpace::NonlinearMPCData& constraints)
{
    constraints.Clear();
    if (!m_pData || m_pData->m_MPCConstraints.empty())
        return true;

    std::vector<SolverNameSpace::NonlinearMPCData> contributions;
    int equationCount = 0;
    for (const auto& [id, mpc] : m_pData->m_MPCConstraints)
    {
        if (!mpc || !IsStepScopedDataActive(mpc->m_StepId))
            continue;
        SolverNameSpace::NonlinearMPCData contribution;
        if (!mpc->Evaluate(m_nFixed, m_nFree, contribution))
        {
            qDebug().noquote()
                << QStringLiteral("Error: MPC %1在当前状态无法组装，"
                                  "请检查从自由度和约束Jacobian。")
                       .arg(id);
            return false;
        }
        equationCount += static_cast<int>(contribution.value.size());
        contributions.push_back(std::move(contribution));
    }
    if (equationCount == 0)
        return true;

    constraints.value = Eigen::VectorXd::Zero(equationCount);
    constraints.jacobian =
        Eigen::MatrixXd::Zero(equationCount, m_nFree);
    constraints.hessians.reserve(equationCount);
    constraints.slaveDofs.reserve(equationCount);

    int row = 0;
    for (const auto& contribution : contributions)
    {
        const int rows = static_cast<int>(contribution.value.size());
        constraints.value.segment(row, rows) = contribution.value;
        constraints.jacobian.middleRows(row, rows) =
            contribution.jacobian;
        constraints.hessians.insert(
            constraints.hessians.end(),
            contribution.hessians.begin(),
            contribution.hessians.end());
        constraints.slaveDofs.insert(
            constraints.slaveDofs.end(),
            contribution.slaveDofs.begin(),
            contribution.slaveDofs.end());
        row += rows;
    }
    return constraints.IsValid(m_nFree);
}

void AnalysisStep::SetNonlinearMPCMultipliers(
    const SolverNameSpace::Vec& multipliers)
{
    m_mpcMultipliers = multipliers;
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
        if (m_dynamicInertiaForce.size() == m_nFree)
            f_int += m_dynamicInertiaForce;
    }

    R = F_ext - f_int;
}

void AnalysisStep::ComputeStaticResidual(
    const SolverNameSpace::Vec& F_ext,
    SolverNameSpace::Vec& R)
{
    VectorXd internalForce = VectorXd::Zero(m_nFree);
    for (auto& nodePair : m_pData->m_Nodes)
        std::fill(
            nodePair.second->m_Force.begin(),
            nodePair.second->m_Force.end(), 0.0);
    Get_CurrentInforce(internalForce);
    R = F_ext - internalForce;
}

void AnalysisStep::OnStepCompleted(double time)
{
    if (m_pData)
    {
        m_pData->GetOutputter().SaveDataFromNodes(time, m_pData);
    }
}

void AnalysisStep::RecordStepIterations(double time, int iterations)
{
    if (m_pData)
        m_pData->GetOutputter().RecordSolverIteration(time, iterations);
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
        pNode->m_Velocity_n = pNode->m_Velocity;
        pNode->m_Acceleration_n = pNode->m_Acceleration;
        pNode->m_Rg_n = pNode->m_Rg;
        pNode->m_OmegaMaterial_n = pNode->m_OmegaMaterial;
        pNode->m_AlphaMaterial_n = pNode->m_AlphaMaterial;
        pNode->m_StepRotation.setZero();
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
