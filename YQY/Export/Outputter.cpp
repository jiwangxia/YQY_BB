#include "Outputter.h"
#include "DataStructure/Structure/StructureData.h"
#include <iomanip>
#include <sstream>
#include <cmath>

void NodeData::ExtractFromNode(const Node* pNode)
{
    if (!pNode) return;

    // 直接从节点读取位移
    if (pNode->m_Displacement.size() >= 1) m_u1 = pNode->m_Displacement[0];
    if (pNode->m_Displacement.size() >= 2) m_u2 = pNode->m_Displacement[1];
    if (pNode->m_Displacement.size() >= 3) m_u3 = pNode->m_Displacement[2];
    m_magnitudeU = std::sqrt(m_u1 * m_u1 + m_u2 * m_u2 + m_u3 * m_u3);

    // 直接从节点读取速度
    if (pNode->m_Velocity.size() >= 1) m_v1 = pNode->m_Velocity[0];
    if (pNode->m_Velocity.size() >= 2) m_v2 = pNode->m_Velocity[1];
    if (pNode->m_Velocity.size() >= 3) m_v3 = pNode->m_Velocity[2];

    // 直接从节点读取加速度
    if (pNode->m_Acceleration.size() >= 1) m_a1 = pNode->m_Acceleration[0];
    if (pNode->m_Acceleration.size() >= 2) m_a2 = pNode->m_Acceleration[1];
    if (pNode->m_Acceleration.size() >= 3) m_a3 = pNode->m_Acceleration[2];

    // 直接从节点读取内力
    if (pNode->m_Force.size() >= 1) m_f1 = pNode->m_Force[0];
    if (pNode->m_Force.size() >= 2) m_f2 = pNode->m_Force[1];
    if (pNode->m_Force.size() >= 3) m_f3 = pNode->m_Force[2];

    // 读取转角 (DOF 3-5)
    if (pNode->m_Displacement.size() >= 4) m_ur1 = pNode->m_Displacement[3];
    if (pNode->m_Displacement.size() >= 5) m_ur2 = pNode->m_Displacement[4];
    if (pNode->m_Displacement.size() >= 6) m_ur3 = pNode->m_Displacement[5];

    // 读取反力
    if (pNode->m_ReactionForce.size() >= 1) m_r1 = pNode->m_ReactionForce[0];
    if (pNode->m_ReactionForce.size() >= 2) m_r2 = pNode->m_ReactionForce[1];
    if (pNode->m_ReactionForce.size() >= 3) m_r3 = pNode->m_ReactionForce[2];
}

double NodeData::GetValue(DataType type) const
{
    switch (type)
    {
    case DataType::U1:         return m_u1;
    case DataType::U2:         return m_u2;
    case DataType::U3:         return m_u3;
    case DataType::MagnitudeU: return m_magnitudeU;
    case DataType::V1:         return m_v1;
    case DataType::V2:         return m_v2;
    case DataType::V3:         return m_v3;
    case DataType::A1:         return m_a1;
    case DataType::A2:         return m_a2;
    case DataType::A3:         return m_a3;
    case DataType::UR1:        return m_ur1;
    case DataType::UR2:        return m_ur2;
    case DataType::UR3:        return m_ur3;
    case DataType::F1:         return m_f1;
    case DataType::F2:         return m_f2;
    case DataType::F3:         return m_f3;
    case DataType::M1:         return m_m1;
    case DataType::M2:         return m_m2;
    case DataType::M3:         return m_m3;
    case DataType::R1:         return m_r1;
    case DataType::R2:         return m_r2;
    case DataType::R3:         return m_r3;
    default:                   return 0.0;
    }
}

double DataFrame::GetNodeData(int idNode, DataType type) const
{
    auto it = m_nodeDatas.find(idNode);
    if (it != m_nodeDatas.end())
    {
        return it->second.GetValue(type);
    }
    return 0.0;
}

void Outputter::SaveDataFromNodes(double time, StructureData* pData)
{
    if (!pData) return;

    // 创建新帧
    DataFrame frame;
    frame.m_currentTime = time;

    for (auto& nodePair : pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        if (!pNode) continue;

        NodeData data;
        data.ExtractFromNode(pNode.get());  // 直接从节点读取
        frame.m_nodeDatas[pNode->m_Id] = data;
    }

    m_DataSet.push_back(frame);
}

// 智能格式化类实现
class OutputFormatter
{
public:
    enum FormatStyle
    {
        Scientific,      // 科学计数法
        FixedDecimal,    // 固定小数
        SmartFormat,     // 智能选择
        modelering      // 工程计数法
    };

    static QString Format(double value, FormatStyle style = SmartFormat, int width = 16, int precision = 6)
    {
        char buffer[128];

        // 新增判断：如果绝对值小于1e-7，则视为0
        const double ZERO_THRESHOLD = 1e-7;
        if (std::fabs(value) < ZERO_THRESHOLD)
        {
            value = 0.0;
        }

        switch (style)
        {
        case Scientific:
            // 科学计数法，保留正号对齐
            snprintf(buffer, sizeof(buffer), "%+*.*E", width, precision, value);
            break;

        case FixedDecimal:
            // 固定小数格式
            if (value >= 0)
            {
                // 正数前加空格对齐负号
                snprintf(buffer, sizeof(buffer), " %*.*f", width - 1, precision, value);
            }
            else
            {
                snprintf(buffer, sizeof(buffer), "%*.*f", width, precision, value);
            }
            break;

        case modelering:
            // 工程计数法（指数为3的倍数）
            // 这里简化为科学计数法，实际工程计数法需要更复杂的处理
            snprintf(buffer, sizeof(buffer), "%+*.*E", width, precision, value);
            break;

        case SmartFormat:
        default:
            // 智能选择：当数值绝对值在 [1e-4, 1e6] 范围内用固定小数，否则用科学计数法
            if (fabs(value) >= 1e6 || (fabs(value) <= 1e-4 && value != 0.0))
            {
                // 太大或太小时用科学计数法
                snprintf(buffer, sizeof(buffer), "%+*.*E", width, precision, value);
            }
            else
            {
                // 正常范围内用固定小数
                if (value >= 0)
                {
                    snprintf(buffer, sizeof(buffer), " %*.*f", width - 1, precision, value);
                }
                else
                {
                    snprintf(buffer, sizeof(buffer), "%*.*f", width, precision, value);
                }
            }
            break;
        }

        // 如果字符串长度小于width，右对齐
        QString result(buffer);
        if (result.length() < width)
        {
            result = result.rightJustified(width, ' ');
        }
        return result;
    }
};

// 辅助格式化函数：使用智能格式
static QString FormatValue(double val, int width = 16)
{
    return OutputFormatter::Format(val, OutputFormatter::SmartFormat, width, 8);
}

void Outputter::ExportNodes(const QString& fileName,
    const std::vector<int>& nodeIds,
    const std::vector<DataType>& types) const
{
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open file:" << fileName;
        return;
    }

    QTextStream stream(&file);
    const int colWidth = 16;
    const QChar padChar = ' ';

    // --- 写表头 ---
    stream << QString("TIME").rightJustified(colWidth, padChar);

    for (int nodeId : nodeIds)
    {
        for (DataType type : types)
        {
            QString header = QString("N%1-%2").arg(nodeId).arg(GetTypeName(type));
            // 确保表头不超长
            if (header.length() > colWidth) header = header.right(colWidth);
            stream << header.rightJustified(colWidth, padChar);
        }
    }
    stream << "\n";

    // 分隔线 (长度自适应)
    size_t totalWidth = colWidth + types.size() * nodeIds.size() * colWidth;
    // 简单的重复字符生成分隔线
    QString line;
    line.fill('-', totalWidth);
    stream << line << "\n";

    // --- 写数据 (时间历程) ---
    for (const auto& frame : m_DataSet)
    {
        // 时间列
        stream << FormatValue(frame.m_currentTime, colWidth);

        // 数据列
        for (int nodeId : nodeIds)
        {
            if (frame.m_nodeDatas.find(nodeId) == frame.m_nodeDatas.end())
            {
                qDebug() << "Warning: Node" << nodeId << "does not exist!";
                return; // 或者 return;
            }
            for (DataType type : types)
            {
                double val = frame.GetNodeData(nodeId, type);
                stream << FormatValue(val, colWidth);
            }
        }
        stream << "\n";
    }

    file.close();
    qDebug().noquote() << QStringLiteral("\n输出至") << fileName;
}

QString Outputter::GetTypeName(DataType type)
{
    switch (type)
    {
    case DataType::U1:         return "U1";
    case DataType::U2:         return "U2";
    case DataType::U3:         return "U3";
    case DataType::UR1:        return "UR1";
    case DataType::UR2:        return "UR2";
    case DataType::UR3:        return "UR3";
    case DataType::MagnitudeU: return "MAG";
    case DataType::V1:         return "V1";
    case DataType::V2:         return "V2";
    case DataType::V3:         return "V3";
    case DataType::A1:         return "A1";
    case DataType::A2:         return "A2";
    case DataType::A3:         return "A3";
    case DataType::F1:         return "F1";
    case DataType::F2:         return "F2";
    case DataType::F3:         return "F3";
    case DataType::M1:         return "M1";
    case DataType::M2:         return "M2";
    case DataType::M3:         return "M3";
    case DataType::R1:         return "R1";
    case DataType::R2:         return "R2";
    case DataType::R3:         return "R3";
    default:                   return "UNKNOWN";
    }
}

// 辅助格式化函数 (局部)
static QString FmtInt(int val, int width = 6, bool leftAlign = false)
{
    QString s = QString::number(val);
    return leftAlign ? s.leftJustified(width, ' ') : s.rightJustified(width, ' ');
}

static QString FmtDouble(double val, int width = 16, bool leftAlign = false)
{
    // 使用智能格式
    QString s = OutputFormatter::Format(val, OutputFormatter::SmartFormat, width, 6);
    return leftAlign ? s.leftJustified(width, ' ') : s.rightJustified(width, ' ');
}

static QString FmtStr(const QString& str, int width = 10, bool leftAlign = false)
{
    return leftAlign ? str.leftJustified(width, ' ') : str.rightJustified(width, ' ');
}

void Outputter::SaveModel(const QString& fileName, StructureData* pData)
{
    if (!pData) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open file for saving model:" << fileName;
        return;
    }

    QTextStream stream(&file);

    // 1. 材料 *MATERIAL, N
    if (!pData->m_Material.empty())
    {
        stream << "*MATERIAL, " << pData->m_Material.size() << "\n";
        stream << "** ID  弹性模量  泊松比  密度  许用应力  膨胀系数\n";
        for (const auto& pair : pData->m_Material)
        {
            auto pMat = pair.second;
            // ID, E, v, Density, MaxStress, Expansion
            stream << FmtInt(pMat->m_Id, 10, true) << " "
                << FmtDouble(pMat->m_Young) << " "
                << FmtDouble(pMat->m_Poisson) << " "
                << FmtDouble(pMat->m_Density) << " "
                << FmtDouble(pMat->m_MaxStress) << " "
                << FmtDouble(pMat->m_Expansion) << "\n";
        }
    }

    // 2. 截面 *SECTION, N
    if (!pData->m_Section.empty())
    {
        stream << "\n*SECTION, " << pData->m_Section.size() << "\n";
        stream << "** ID  面积\n";
        for (const auto& pair : pData->m_Section)
        {
            auto pSec = pair.second;
            // ID, Area
            stream << FmtInt(pSec->m_Id, 10, true) << " " << FmtDouble(pSec->m_Area) << "\n";
        }
    }

    // 3. 节点 *NODE, N
    if (!pData->m_Nodes.empty())
    {
        stream << "\n*NODE, " << pData->m_Nodes.size() << "\n";
        stream << "** ID  X  Y  Z\n";
        for (const auto& pair : pData->m_Nodes)
        {
            auto pNode = pair.second;
            // ID, X, Y, Z
            stream << FmtInt(pNode->m_Id, 10, true) << " "
                << FmtDouble(pNode->m_X) << " "
                << FmtDouble(pNode->m_Y) << " "
                << FmtDouble(pNode->m_Z) << "\n";
        }
    }

    // 4. 单元 *ELEMENT, TYPE, N
    std::map<QString, std::vector<std::shared_ptr<ElementBase>>> elemGroups;
    for (const auto& pair : pData->m_Elements)
    {
        auto pElem = pair.second;
        QString typeName = "UNKNOWN";
        if (std::dynamic_pointer_cast<ElementTruss>(pElem)) typeName = "T3D2";
        else if (std::dynamic_pointer_cast<ElementCable>(pElem)) typeName = "CABLE";
        else if (std::dynamic_pointer_cast<ElementBeam_CR>(pElem)) typeName = "CR3D";
        elemGroups[typeName].push_back(pElem);
    }

    for (const auto& group : elemGroups)
    {
        if (group.first == "UNKNOWN" || group.second.empty()) continue;

        stream << "\n*ELEMENT, " << group.first << ", " << group.second.size() << "\n";
        stream << "** ID  Node1  Node2  MaterialID  SectionID\n";
        for (const auto& pElem : group.second)
        {
            int nodeId1 = pElem->m_pNode[0].lock() ? pElem->m_pNode[0].lock()->m_Id : 0;
            int nodeId2 = pElem->m_pNode[1].lock() ? pElem->m_pNode[1].lock()->m_Id : 0;

            int matId = 0;
            int secId = 0;
            auto pProp = pElem->m_pProperty.lock();
            if (pProp)
            {
                if (auto pMat = pProp->m_pMaterial.lock()) matId = pMat->m_Id;
                if (auto pSec = pProp->m_pSection.lock())  secId = pSec->m_Id;
            }

            stream << FmtInt(pElem->m_Id, 10, true) << " "
                << FmtInt(nodeId1) << " "
                << FmtInt(nodeId2) << " "
                << FmtInt(matId) << " "
                << FmtInt(secId) << "\n";
        }
    }

    // 5. 约束 *CONSTRAINT, N
    if (!pData->m_Constraint.empty())
    {
        stream << "\n*CONSTRAINT, " << pData->m_Constraint.size() << "\n";
        stream << "** ID  NodeID  Direction  Value\n";
        for (const auto& pair : pData->m_Constraint)
        {
            auto pCon = pair.second;
            // ID, NodeID, Dir, Value
            stream << FmtInt(pCon->m_Id, 10, true) << " "
                << FmtInt(pCon->m_pNode.lock() ? pCon->m_pNode.lock()->m_Id : 0) << " "
                << FmtInt(static_cast<int>(pCon->m_Direction)) << " "
                << FmtDouble(pCon->m_Value) << "\n";
        }
    }

    // 6. 荷载 *LOAD, TYPE, N
    std::map<QString, std::vector<std::shared_ptr<LoadBase>>> loadGroups;
    for (const auto& pair : pData->m_Load)
    {
        auto pLoad = pair.second;
        QString typeName = "UNKNOWN";
        if (std::dynamic_pointer_cast<Force_Node>(pLoad)) typeName = "FORCE_NODE";
        else if (std::dynamic_pointer_cast<Force_Element>(pLoad)) typeName = "FORCE_ELEMENT";
        else if (std::dynamic_pointer_cast<Force_Gravity>(pLoad)) typeName = "FORCE_GRAVITY";
        else if (std::dynamic_pointer_cast<Force_Wind>(pLoad)) typeName = "FORCE_WIND";
        loadGroups[typeName].push_back(pLoad);
    }

    for (const auto& group : loadGroups)
    {
        if (group.first == "UNKNOWN" || group.second.empty()) continue;

        stream << "\n*LOAD, " << group.first << ", " << group.second.size() << "\n";
        stream << "** ID  NodeID/ElementID  Direction  Value\n";
        for (const auto& pLoad : group.second)
        {
            if (group.first == "FORCE_NODE")
            {
                auto pL = std::dynamic_pointer_cast<Force_Node>(pLoad);
                stream << FmtInt(pL->m_Id, 10, true) << " "
                    << FmtInt(pL->m_pNode.lock() ? pL->m_pNode.lock()->m_Id : 0) << " "
                    << FmtInt(static_cast<int>(pL->m_Direction)) << " "
                    << FmtDouble(pL->m_Value) << " "
                    << FmtInt(pL->m_StepId) << "\n";
            }
            else if (group.first == "FORCE_ELEMENT")
            {
                auto pL = std::dynamic_pointer_cast<Force_Element>(pLoad);
                stream << FmtInt(pL->m_Id, 10, true) << " "
                    << FmtInt(pL->m_pElement.lock() ? pL->m_pElement.lock()->m_Id : 0) << " "
                    << FmtInt(static_cast<int>(pL->m_Direction)) << " "
                    << FmtDouble(pL->m_Value) << "\n";
            }
            else if (group.first == "FORCE_GRAVITY")
            {
                auto pL = std::dynamic_pointer_cast<Force_Gravity>(pLoad);
                stream << FmtInt(pL->m_Id, 10, true) << " "
                    << FmtInt(static_cast<int>(pL->m_Direction)) << " "
                    << FmtDouble(pL->m_g) << " "
                    << FmtInt(pL->m_StepId) << "\n";
            }
            else if (group.first == "FORCE_WIND")
            {
                auto pL = std::dynamic_pointer_cast<Force_Wind>(pLoad);
                stream << FmtInt(pL->m_Id, 10, true) << " "
                    << FmtInt(static_cast<int>(pL->m_Direction)) << " "
                    << FmtDouble(pL->m_velocity) << " "
                    << FmtInt(pL->m_StepId) << "\n";
            }
        }
    }

    // 7. 初始应力 *STRESS, N
    if (!pData->m_Elements.empty())
    {
        stream << "\n*STRESS, " << pData->m_Elements.size() << "\n";
        stream << "** 单元编号  应力\n";

        for (auto& pair : pData->m_Elements)
        {
            auto pElem = pair.second;
            // 仅输出有初始应力的单元？目前全部输出
            stream << FmtInt(pElem->m_Id, 10, true) << " " << FmtDouble(pElem->m_InitStress) << "\n";
        }
    }

    // 8. 分析步 *ANALYSIS_STEP, N
    if (!pData->m_AnalysisStep.empty())
    {
        stream << "\n*ANALYSIS_STEP, " << pData->m_AnalysisStep.size() << "\n";
        stream << "** ID  Type  Time  StepSize  Tolerance  MaxIterations\n";
        for (const auto& pair : pData->m_AnalysisStep)
        {
            auto pStep = pair.second;
            stream << FmtInt(pStep->m_Id, 10, true) << " "
                << FmtStr(pStep->GetTypeName()) << " "
                << FmtDouble(pStep->m_Time) << " "
                << FmtDouble(pStep->m_StepSize) << " "
                << FmtDouble(pStep->m_Tolerance) << " "
                << FmtInt(pStep->m_MaxIterations) << "\n";
        }
    }

    file.close();
    qDebug() << "Model saved to" << fileName;
}