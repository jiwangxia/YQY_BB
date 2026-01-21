#include "Outputter.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Node/Node.h"
#include <iomanip>
#include <sstream>

// ==========================================
// NodeData 实现
// ==========================================

void NodeData::ExtractFromVectors(const QVector<int>& dofs, int nFixed,
                                   const Eigen::VectorXd& x,
                                   const Eigen::VectorXd* v,
                                   const Eigen::VectorXd* a)
{
    // 提取位移
    for (int i = 0; i < dofs.size() && i < 3; ++i)
    {
        int dof = dofs[i];
        if (dof >= nFixed)
        {
            int idx = dof - nFixed;
            if (idx < x.size())
            {
                if (i == 0) u1 = x[idx];
                else if (i == 1) u2 = x[idx];
                else if (i == 2) u3 = x[idx];
            }
        }
    }
    magnitudeU = std::sqrt(u1 * u1 + u2 * u2 + u3 * u3);

    // 提取速度
    if (v && v->size() > 0)
    {
        for (int i = 0; i < dofs.size() && i < 3; ++i)
        {
            int dof = dofs[i];
            if (dof >= nFixed)
            {
                int idx = dof - nFixed;
                if (idx < v->size())
                {
                    if (i == 0) v1 = (*v)[idx];
                    else if (i == 1) v2 = (*v)[idx];
                    else if (i == 2) v3 = (*v)[idx];
                }
            }
        }
    }

    // 提取加速度
    if (a && a->size() > 0)
    {
        for (int i = 0; i < dofs.size() && i < 3; ++i)
        {
            int dof = dofs[i];
            if (dof >= nFixed)
            {
                int idx = dof - nFixed;
                if (idx < a->size())
                {
                    if (i == 0) a1 = (*a)[idx];
                    else if (i == 1) a2 = (*a)[idx];
                    else if (i == 2) a3 = (*a)[idx];
                }
            }
        }
    }

    // 提取转角 (如果有6个自由度)
    if (dofs.size() >= 6)
    {
        for (int i = 3; i < 6; ++i)
        {
            int dof = dofs[i];
            if (dof >= nFixed)
            {
                int idx = dof - nFixed;
                if (idx < x.size())
                {
                    if (i == 3) ur1 = x[idx];
                    else if (i == 4) ur2 = x[idx];
                    else if (i == 5) ur3 = x[idx];
                }
            }
        }
    }
}

double NodeData::GetValue(DataType type) const
{
    switch (type)
    {
    case DataType::U1:         return u1;
    case DataType::U2:         return u2;
    case DataType::U3:         return u3;
    case DataType::MagnitudeU: return magnitudeU;
    case DataType::V1:         return v1;
    case DataType::V2:         return v2;
    case DataType::V3:         return v3;
    case DataType::A1:         return a1;
    case DataType::A2:         return a2;
    case DataType::A3:         return a3;
    case DataType::UR1:        return ur1;
    case DataType::UR2:        return ur2;
    case DataType::UR3:        return ur3;
    default:                   return 0.0;
    }
}

// ==========================================
// DataFrame 实现
// ==========================================

double DataFrame::GetNodeData(int idNode, DataType type) const
{
    auto it = nodeDatas.find(idNode);
    if (it != nodeDatas.end())
    {
        return it->second.GetValue(type);
    }
    return 0.0;
}

void Outputter::SaveData(double time, StructureData* pData, int nFixed,
                          const Eigen::VectorXd& x,
                          const Eigen::VectorXd* v,
                          const Eigen::VectorXd* a)
{
    if (!pData) return;

    // 检查是否已存在相同时间点
    for (auto& frame : m_DataSet)
    {
        if (std::abs(frame.currentTime - time) < 1e-10)
        {
            // 更新已存在的帧
            frame.nodeDatas.clear();
            for (auto& nodePair : pData->m_Nodes)
            {
                auto pNode = nodePair.second;
                if (!pNode) continue;

                NodeData data;
                data.ExtractFromVectors(pNode->m_DOF, nFixed, x, v, a);
                frame.nodeDatas[pNode->m_Id] = data;
            }
            return;
        }
    }

    // 创建新帧
    DataFrame frame;
    frame.currentTime = time;

    for (auto& nodePair : pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        if (!pNode) continue;

        NodeData data;
        data.ExtractFromVectors(pNode->m_DOF, nFixed, x, v, a);
        frame.nodeDatas[pNode->m_Id] = data;
    }

    m_DataSet.push_back(frame);
}

// 辅助格式化函数：科学计数法，宽度16，保留6位小数
static QString FormatValue(double val, int width = 16)
{
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "% .6E", val); 
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
        stream << FormatValue(frame.currentTime, colWidth);

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
    default:                   return "UNKNOWN";
    }
}
