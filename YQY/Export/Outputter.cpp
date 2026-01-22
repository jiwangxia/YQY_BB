#include "Outputter.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Node/Node.h"
#include <iomanip>
#include <sstream>

void NodeData::ExtractFromVectors(const QVector<int>& dofs, int nFixed,
                                   const Eigen::VectorXd& x,
                                   const Eigen::VectorXd* v,
                                   const Eigen::VectorXd* a)
{
    // 辅助lambda：从向量中提取指定范围DOF的值
    auto extractDOFs = [&dofs, nFixed](const Eigen::VectorXd& vec, int start, int end, double* out)
    {
        for (int i = start; i < end && i < dofs.size(); ++i)
        {
            int dof = dofs[i];
            if (dof >= nFixed)
            {
                int idx = dof - nFixed;
                if (idx < vec.size())
                    out[i - start] = vec[idx];
            }
        }
    };

    // 提取位移 (DOF 0-2)
    double Disp[3] = { 0, 0, 0 };
    extractDOFs(x, 0, 3, Disp);
    m_u1 = Disp[0]; m_u2 = Disp[1]; m_u3 = Disp[2];
    m_magnitudeU = std::sqrt(m_u1 * m_u1 + m_u2 * m_u2 + m_u3 * m_u3);

    // 提取速度 (DOF 0-2)
    if (v && v->size() > 0)
    {
        double Vel[3] = { 0, 0, 0 };
        extractDOFs(*v, 0, 3, Vel);
        m_v1 = Vel[0]; m_v2 = Vel[1]; m_v3 = Vel[2];
    }

    // 提取加速度 (DOF 0-2)
    if (a && a->size() > 0)
    {
        double Acc[3] = { 0, 0, 0 };
        extractDOFs(*a, 0, 3, Acc);
        m_a1 = Acc[0]; m_a2 = Acc[1]; m_a3 = Acc[2];
    }

    // 提取转角 (DOF 3-5, 如果有6个自由度)
    if (dofs.size() >= 6)
    {
        double Rot[3] = { 0, 0, 0 };
        extractDOFs(x, 3, 6, Rot);
        m_ur1 = Rot[0]; m_ur2 = Rot[1]; m_ur3 = Rot[2];
    }
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

void Outputter::SaveData(double time, StructureData* pData, int nFixed,
                          const Eigen::VectorXd& x,
                          const Eigen::VectorXd* v,
                          const Eigen::VectorXd* a)
{
    if (!pData) return;

    // 检查是否已存在相同时间点
    for (auto& frame : m_DataSet)
    {
        if (std::abs(frame.m_currentTime - time) < 1e-10)
        {
            // 更新已存在的帧
            frame.m_nodeDatas.clear();
            for (auto& nodePair : pData->m_Nodes)
            {
                auto pNode = nodePair.second;
                if (!pNode) continue;

                NodeData data;
                data.ExtractFromVectors(pNode->m_DOF, nFixed, x, v, a);
                frame.m_nodeDatas[pNode->m_Id] = data;
            }
            return;
        }
    }

    // 创建新帧
    DataFrame frame;
    frame.m_currentTime = time;

    for (auto& nodePair : pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        if (!pNode) continue;

        NodeData data;
        data.ExtractFromVectors(pNode->m_DOF, nFixed, x, v, a);
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
    default:                   return "UNKNOWN";
    }
}
