#include "Outputter.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Node/Node.h"
#include <iomanip>
#include <sstream>

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

    // 直接从节点读取内力/反力
    if (pNode->m_Force.size() >= 1) m_f1 = pNode->m_Force[0];
    if (pNode->m_Force.size() >= 2) m_f2 = pNode->m_Force[1];
    if (pNode->m_Force.size() >= 3) m_f3 = pNode->m_Force[2];

    // 读取转角 (DOF 3-5)
    if (pNode->m_Displacement.size() >= 4) m_ur1 = pNode->m_Displacement[3];
    if (pNode->m_Displacement.size() >= 5) m_ur2 = pNode->m_Displacement[4];
    if (pNode->m_Displacement.size() >= 6) m_ur3 = pNode->m_Displacement[5];
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

// 辅助格式化函数：科学计数法，固定宽度，保留6位小数
static QString FormatValue(double val, int width = 16)
{
    char buffer[64];
    // 使用 %+.6E 确保正负号始终存在，保证对齐
    snprintf(buffer, sizeof(buffer), "%+.6E", val);
    return QString(buffer).rightJustified(width, ' ');
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
    case DataType::U1:         return "T1";
    case DataType::U2:         return "T2";
    case DataType::U3:         return "T3";
    case DataType::MagnitudeU: return "MAG";
    case DataType::V1:         return "V1";
    case DataType::V2:         return "V2";
    case DataType::V3:         return "V3";
    case DataType::A1:         return "A1";
    case DataType::A2:         return "A2";
    case DataType::A3:         return "A3";
    case DataType::UR1:        return "R1";
    case DataType::UR2:        return "R2";
    case DataType::UR3:        return "R3";
    case DataType::F1:         return "F1";
    case DataType::F2:         return "F2";
    case DataType::F3:         return "F3";
    case DataType::M1:         return "M1";
    case DataType::M2:         return "M2";
    case DataType::M3:         return "M3";
    default:                   return "UNKNOWN";
    }
}
