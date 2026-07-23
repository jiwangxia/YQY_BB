#include "Outputter.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Hdf5ModelIO.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>

Outputter::Outputter() = default;

Outputter::~Outputter()
{
    Clear();
}

void Outputter::MergeFramesFrom(const Outputter& source)
{
    for (const DataFrame& sourceFrame : source.m_DataSet)
    {
        auto targetIt = std::find_if(m_DataSet.begin(), m_DataSet.end(),
            [&sourceFrame](const DataFrame& targetFrame)
        {
            return targetFrame.m_stepId == sourceFrame.m_stepId
                && targetFrame.m_increment == sourceFrame.m_increment
                && targetFrame.m_analysisType == sourceFrame.m_analysisType
                && std::abs(targetFrame.m_currentTime - sourceFrame.m_currentTime) <= 1.0e-12;
        });
        if (targetIt == m_DataSet.end())
        {
            m_DataSet.push_back(sourceFrame);
            continue;
        }
        targetIt->m_nodeDatas.insert(sourceFrame.m_nodeDatas.cbegin(), sourceFrame.m_nodeDatas.cend());
        targetIt->m_elementDatas.insert(sourceFrame.m_elementDatas.cbegin(), sourceFrame.m_elementDatas.cend());
    }
    std::sort(m_DataSet.begin(), m_DataSet.end(), [](const DataFrame& lhs, const DataFrame& rhs)
    {
        if (lhs.m_stepId != rhs.m_stepId)
            return lhs.m_stepId < rhs.m_stepId;
        if (lhs.m_currentTime != rhs.m_currentTime)
            return lhs.m_currentTime < rhs.m_currentTime;
        return lhs.m_increment < rhs.m_increment;
    });
}

void NodeData::ExtractFromNode(const Node* pNode)
{
    if (!pNode) return;

    // 直接从节点读取位移
    if (pNode->m_Displacement.size() >= 1) m_u1 = pNode->m_Displacement[0];
    if (pNode->m_Displacement.size() >= 2) m_u2 = pNode->m_Displacement[1];
    if (pNode->m_Displacement.size() >= 3) m_u3 = pNode->m_Displacement[2];
    m_magnitudeU = std::sqrt(m_u1 * m_u1 + m_u2 * m_u2 + m_u3 * m_u3);
    m_cx = pNode->m_X + m_u1;
    m_cy = pNode->m_Y + m_u2;
    m_cz = pNode->m_Z + m_u3;

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

double NodeData::GetValue(EnumKeyword::NodeResultType type) const
{
    switch (type)
    {
    case EnumKeyword::NodeResultType::U1:         return m_u1;
    case EnumKeyword::NodeResultType::U2:         return m_u2;
    case EnumKeyword::NodeResultType::U3:         return m_u3;
    case EnumKeyword::NodeResultType::MagnitudeU: return m_magnitudeU;
    case EnumKeyword::NodeResultType::CX:         return m_cx;
    case EnumKeyword::NodeResultType::CY:         return m_cy;
    case EnumKeyword::NodeResultType::CZ:         return m_cz;
    case EnumKeyword::NodeResultType::V1:         return m_v1;
    case EnumKeyword::NodeResultType::V2:         return m_v2;
    case EnumKeyword::NodeResultType::V3:         return m_v3;
    case EnumKeyword::NodeResultType::A1:         return m_a1;
    case EnumKeyword::NodeResultType::A2:         return m_a2;
    case EnumKeyword::NodeResultType::A3:         return m_a3;
    case EnumKeyword::NodeResultType::UR1:        return m_ur1;
    case EnumKeyword::NodeResultType::UR2:        return m_ur2;
    case EnumKeyword::NodeResultType::UR3:        return m_ur3;
    case EnumKeyword::NodeResultType::F1:         return m_f1;
    case EnumKeyword::NodeResultType::F2:         return m_f2;
    case EnumKeyword::NodeResultType::F3:         return m_f3;
    case EnumKeyword::NodeResultType::M1:         return m_m1;
    case EnumKeyword::NodeResultType::M2:         return m_m2;
    case EnumKeyword::NodeResultType::M3:         return m_m3;
    case EnumKeyword::NodeResultType::R1:         return m_r1;
    case EnumKeyword::NodeResultType::R2:         return m_r2;
    case EnumKeyword::NodeResultType::R3:         return m_r3;
    default:                   return 0.0;
    }
}

void ElementData::ExtractFromElement(const ElementBase* pElement)
{
    if (!pElement) return;

    m_initStress = pElement->m_InitStress;
    m_currentStress = pElement->m_Stress;
    m_deltaStress = m_currentStress - m_initStress;

    auto pProperty = pElement->m_pProperty.lock();
    if (!pProperty) return;

    auto pSection = pProperty->m_pSection.lock();
    auto pMaterial = pProperty->m_pMaterial.lock();

    const double area = pSection ? pSection->m_Area : 0.0;
    const double young = pMaterial ? pMaterial->m_Young : 0.0;

    if (std::fabs(area) > 0.0)
    {
        m_axialForce = m_currentStress * area;
    }

    if (std::fabs(young) > 0.0)
    {
        m_strain = m_deltaStress / young;
    }
}

double ElementData::GetValue(EnumKeyword::ElementResultType type) const
{
    switch (type)
    {
    case EnumKeyword::ElementResultType::AxialForce:    return m_axialForce;
    case EnumKeyword::ElementResultType::ShearY:        return m_shearY;
    case EnumKeyword::ElementResultType::ShearZ:        return m_shearZ;
    case EnumKeyword::ElementResultType::Torque:        return m_torque;
    case EnumKeyword::ElementResultType::MomentY:       return m_momentY;
    case EnumKeyword::ElementResultType::MomentZ:       return m_momentZ;
    case EnumKeyword::ElementResultType::Strain:        return m_strain;
    case EnumKeyword::ElementResultType::InitStress:    return m_initStress;
    case EnumKeyword::ElementResultType::CurrentStress: return m_currentStress;
    case EnumKeyword::ElementResultType::DeltaStress:   return m_deltaStress;
    default:                             return 0.0;
    }
}

double DataFrame::GetNodeData(int idNode, EnumKeyword::NodeResultType type) const
{
    auto it = m_nodeDatas.find(idNode);
    if (it != m_nodeDatas.end())
    {
        return it->second.GetValue(type);
    }
    return 0.0;
}

double DataFrame::GetElementData(int idElement, EnumKeyword::ElementResultType type) const
{
    auto it = m_elementDatas.find(idElement);
    if (it != m_elementDatas.end())
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
    frame.m_stepId = m_currentStepId;
    frame.m_increment = m_hdf5NextIncrement;
    frame.m_analysisType = m_currentAnalysisType;

    for (auto& nodePair : pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        if (!pNode) continue;

        NodeData data;
        data.ExtractFromNode(pNode.get());  // 直接从节点读取
        frame.m_nodeDatas[pNode->m_Id] = data;
    }

    for (auto& elementPair : pData->m_Elements)
    {
        auto pElement = elementPair.second;
        if (!pElement) continue;

        ElementData data;
        data.ExtractFromElement(pElement.get());
        frame.m_elementDatas[pElement->m_Id] = data;
    }

    if (m_stream)
    {
        WriteResultFrame(*m_stream, frame,
            m_streamNodeIds, m_streamNodeTypes,
            m_streamElementIds, m_streamElementTypes);
        m_stream->flush();
    }

    if (m_hdf5Stream)
    {
        m_hdf5Stream->WriteResultFrame(m_hdf5NextDomainId, frame.GetStepId(),
            frame.GetIncrement(), frame.GetAnalysisType(), time, frame);
        ++m_hdf5NextDomainId;
    }

    if (m_keepFramesInMemory)
    {
        m_DataSet.push_back(frame);
    }
    ++m_hdf5NextIncrement;
}

void Outputter::SetResultContext(int stepId, int analysisType)
{
    m_currentStepId = stepId;
    m_currentAnalysisType = analysisType;
    m_hdf5NextIncrement = 0;
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
    const std::vector<EnumKeyword::NodeResultType>& types) const
{
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open file:" << fileName;
        return;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    const int colWidth = 16;
    const QChar padChar = ' ';

    // --- 写表头 ---
    stream << QString("TIME").rightJustified(colWidth, padChar);

    for (int nodeId : nodeIds)
    {
        for (EnumKeyword::NodeResultType type : types)
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
            for (EnumKeyword::NodeResultType type : types)
            {
                double val = frame.GetNodeData(nodeId, type);
                stream << FormatValue(val, colWidth);
            }
        }
        stream << "\n";
    }

    file.close();
}

void Outputter::ExportElements(const QString& fileName,
    const std::vector<int>& elementIds,
    const std::vector<EnumKeyword::ElementResultType>& types) const
{
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open file:" << fileName;
        return;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    WriteResultTableHeader(stream, {}, {}, elementIds, types);

    for (const auto& frame : m_DataSet)
    {
        WriteResultFrame(stream, frame, {}, {}, elementIds, types);
    }

    file.close();
}

bool Outputter::BeginBdfResultStream(const QString& fileName,
    const std::vector<int>& nodeIds,
    const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
    const std::vector<int>& elementIds,
    const std::vector<EnumKeyword::ElementResultType>& elementTypes)
{
    EndBdfResultStream();

    m_streamFile = std::make_unique<QFile>(fileName);
    if (!m_streamFile->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open stream file:" << fileName;
        m_streamFile.reset();
        return false;
    }

    m_stream = std::make_unique<QTextStream>(m_streamFile.get());
    m_stream->setEncoding(QStringConverter::Utf8);
    m_streamNodeIds = nodeIds;
    m_streamNodeTypes = nodeTypes;
    m_streamElementIds = elementIds;
    m_streamElementTypes = elementTypes;

    WriteResultTableHeader(*m_stream,
        m_streamNodeIds, m_streamNodeTypes,
        m_streamElementIds, m_streamElementTypes);
    m_stream->flush();
    return true;
}

void Outputter::EndBdfResultStream()
{
    if (m_stream)
    {
        m_stream->flush();
        m_stream.reset();
    }

    if (m_streamFile)
    {
        m_streamFile->close();
        m_streamFile.reset();
    }

    m_streamNodeIds.clear();
    m_streamNodeTypes.clear();
    m_streamElementIds.clear();
    m_streamElementTypes.clear();
}

bool Outputter::BeginHdf5ResultStream(const QString& fileName, StructureData* pData, const QString& sourceModelName)
{
    EndHdf5ResultStream(false);

    m_hdf5Stream = std::make_unique<Hdf5ModelIO>();
    if (!m_hdf5Stream->BeginResultStream(fileName, pData, sourceModelName))
    {
        m_hdf5Stream.reset();
        return false;
    }

    m_hdf5NextDomainId = 1;
    m_hdf5NextIncrement = 0;
    return true;
}

void Outputter::EndHdf5ResultStream(bool resultComplete)
{
    if (m_hdf5Stream)
    {
        m_hdf5Stream->EndResultStream(resultComplete);
        m_hdf5Stream.reset();
    }
}

void Outputter::Clear()
{
    EndBdfResultStream();
    EndHdf5ResultStream(false);
    m_DataSet.clear();
}

void Outputter::WriteResultTableHeader(QTextStream& stream,
    const std::vector<int>& nodeIds,
    const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
    const std::vector<int>& elementIds,
    const std::vector<EnumKeyword::ElementResultType>& elementTypes) const
{
    const int colWidth = 16;
    const QChar padChar = ' ';

    stream << QString("TIME").rightJustified(colWidth, padChar);

    for (int nodeId : nodeIds)
    {
        for (EnumKeyword::NodeResultType type : nodeTypes)
        {
            QString header = QString("N%1-%2").arg(nodeId).arg(GetTypeName(type));
            if (header.length() > colWidth) header = header.right(colWidth);
            stream << header.rightJustified(colWidth, padChar);
        }
    }

    for (int elementId : elementIds)
    {
        for (EnumKeyword::ElementResultType type : elementTypes)
        {
            QString header = QString("E%1-%2").arg(elementId).arg(GetTypeName(type));
            if (header.length() > colWidth) header = header.right(colWidth);
            stream << header.rightJustified(colWidth, padChar);
        }
    }

    stream << "\n";

    size_t totalWidth = colWidth
        + nodeIds.size() * nodeTypes.size() * colWidth
        + elementIds.size() * elementTypes.size() * colWidth;
    QString line;
    line.fill('-', totalWidth);
    stream << line << "\n";
}

void Outputter::WriteResultFrame(QTextStream& stream, const DataFrame& frame,
    const std::vector<int>& nodeIds,
    const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
    const std::vector<int>& elementIds,
    const std::vector<EnumKeyword::ElementResultType>& elementTypes) const
{
    const int colWidth = 16;

    stream << FormatValue(frame.m_currentTime, colWidth);

    for (int nodeId : nodeIds)
    {
        for (EnumKeyword::NodeResultType type : nodeTypes)
        {
            stream << FormatValue(frame.GetNodeData(nodeId, type), colWidth);
        }
    }

    for (int elementId : elementIds)
    {
        for (EnumKeyword::ElementResultType type : elementTypes)
        {
            stream << FormatValue(frame.GetElementData(elementId, type), colWidth);
        }
    }

    stream << "\n";
}

QString Outputter::GetTypeName(EnumKeyword::NodeResultType type)
{
    switch (type)
    {
    case EnumKeyword::NodeResultType::U1:         return "U1";
    case EnumKeyword::NodeResultType::U2:         return "U2";
    case EnumKeyword::NodeResultType::U3:         return "U3";
    case EnumKeyword::NodeResultType::CX:         return "CX";
    case EnumKeyword::NodeResultType::CY:         return "CY";
    case EnumKeyword::NodeResultType::CZ:         return "CZ";
    case EnumKeyword::NodeResultType::UR1:        return "UR1";
    case EnumKeyword::NodeResultType::UR2:        return "UR2";
    case EnumKeyword::NodeResultType::UR3:        return "UR3";
    case EnumKeyword::NodeResultType::MagnitudeU: return "MAG";
    case EnumKeyword::NodeResultType::V1:         return "V1";
    case EnumKeyword::NodeResultType::V2:         return "V2";
    case EnumKeyword::NodeResultType::V3:         return "V3";
    case EnumKeyword::NodeResultType::A1:         return "A1";
    case EnumKeyword::NodeResultType::A2:         return "A2";
    case EnumKeyword::NodeResultType::A3:         return "A3";
    case EnumKeyword::NodeResultType::F1:         return "F1";
    case EnumKeyword::NodeResultType::F2:         return "F2";
    case EnumKeyword::NodeResultType::F3:         return "F3";
    case EnumKeyword::NodeResultType::M1:         return "M1";
    case EnumKeyword::NodeResultType::M2:         return "M2";
    case EnumKeyword::NodeResultType::M3:         return "M3";
    case EnumKeyword::NodeResultType::R1:         return "R1";
    case EnumKeyword::NodeResultType::R2:         return "R2";
    case EnumKeyword::NodeResultType::R3:         return "R3";
    default:                   return "UNKNOWN";
    }
}

QString Outputter::GetTypeName(EnumKeyword::ElementResultType type)
{
    switch (type)
    {
    case EnumKeyword::ElementResultType::AxialForce:    return "AXIAL";
    case EnumKeyword::ElementResultType::ShearY:        return "SHEARY";
    case EnumKeyword::ElementResultType::ShearZ:        return "SHEARZ";
    case EnumKeyword::ElementResultType::Torque:        return "TORQUE";
    case EnumKeyword::ElementResultType::MomentY:       return "MY";
    case EnumKeyword::ElementResultType::MomentZ:       return "MZ";
    case EnumKeyword::ElementResultType::Strain:        return "STRAIN";
    case EnumKeyword::ElementResultType::InitStress:    return "S0";
    case EnumKeyword::ElementResultType::CurrentStress: return "S";
    case EnumKeyword::ElementResultType::DeltaStress:   return "DS";
    default:                             return "UNKNOWN";
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

bool Outputter::SaveBdfModel(const QString& fileName, StructureData* pData)
{
    if (!pData) return false;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open file for saving model:" << fileName;
        return false;
    }

    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);

    std::map<int, int> materialIdMap;
    std::map<int, int> sectionIdMap;
    std::map<int, int> nodeIdMap;
    std::map<int, int> elementIdMap;
    std::map<int, int> constraintIdMap;
    std::map<int, int> loadIdMap;
    std::map<int, int> stepIdMap;

    auto buildIdMap = [](const auto& source, std::map<int, int>& target)
        {
            int id = 1;
            for (const auto& pair : source)
            {
                target[pair.first] = id++;
            }
        };

    auto mapId = [](const std::map<int, int>& idMap, int id)
        {
            auto iter = idMap.find(id);
            return iter != idMap.end() ? iter->second : 0;
        };

    buildIdMap(pData->m_Material, materialIdMap);
    buildIdMap(pData->m_Section, sectionIdMap);
    buildIdMap(pData->m_Nodes, nodeIdMap);
    buildIdMap(pData->m_Elements, elementIdMap);
    buildIdMap(pData->m_Constraint, constraintIdMap);
    buildIdMap(pData->m_Load, loadIdMap);
    buildIdMap(pData->m_AnalysisStep, stepIdMap);

    // 1. 材料 *MATERIAL, N
    if (!pData->m_Material.empty())
    {
        stream << "*MATERIAL, " << pData->m_Material.size() << "\n";
        stream << "** ID  弹性模量  泊松比  密度  许用应力  膨胀系数\n";
        for (const auto& pair : pData->m_Material)
        {
            auto pMat = pair.second;
            // ID, E, v, Density, MaxStress, Expansion
            stream << FmtInt(mapId(materialIdMap, pair.first), 10, true) << " "
                << FmtDouble(pMat->m_Young) << " "
                << FmtDouble(pMat->m_Poisson) << " "
                << FmtDouble(pMat->m_Density) << " "
                << FmtDouble(pMat->m_MaxStress) << " "
                << FmtDouble(pMat->m_Expansion);
            stream << "\n";
        }
    }

    // 2. 截面 *SECTION, N
    if (!pData->m_Section.empty())
    {
        stream << "\n*SECTION, " << pData->m_Section.size() << "\n";
        stream << "** ID  Area 或 Width Height\n";
        for (const auto& pair : pData->m_Section)
        {
            auto pSec = pair.second;
            stream << FmtInt(mapId(sectionIdMap, pair.first), 10, true) << " ";
            if (auto pRect = std::dynamic_pointer_cast<SectionRectangle>(pSec))
            {
                stream << FmtDouble(pRect->m_Width) << " " << FmtDouble(pRect->m_Height) << "\n";
            }
            else
            {
                stream << FmtDouble(pSec->m_Area) << "\n";
            }
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
            stream << FmtInt(mapId(nodeIdMap, pair.first), 10, true) << " "
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
        if (group.first == "CR3D")
        {
            stream << "** ID  Node1  Node2  MaterialID  SectionID  q0x  q0y  q0z\n";
        }
        else
        {
            stream << "** ID  Node1  Node2  MaterialID  SectionID\n";
        }
        for (const auto& pElem : group.second)
        {
            auto elemNode1 = pElem->m_pNode[0].lock();
            auto elemNode2 = pElem->m_pNode[1].lock();
            int nodeId1 = elemNode1 ? mapId(nodeIdMap, elemNode1->m_Id) : 0;
            int nodeId2 = elemNode2 ? mapId(nodeIdMap, elemNode2->m_Id) : 0;

            int matId = 0;
            int secId = 0;
            auto pProp = pElem->m_pProperty.lock();
            if (pProp)
            {
                if (auto pMat = pProp->m_pMaterial.lock()) matId = mapId(materialIdMap, pMat->m_Id);
                if (auto pSec = pProp->m_pSection.lock())  secId = mapId(sectionIdMap, pSec->m_Id);
            }

            stream << FmtInt(mapId(elementIdMap, pElem->m_Id), 10, true) << " "
                << FmtInt(nodeId1) << " "
                << FmtInt(nodeId2) << " "
                << FmtInt(matId) << " "
                << FmtInt(secId);

            if (auto pBeam = std::dynamic_pointer_cast<ElementBeam_CR>(pElem))
            {
                stream << " "
                    << FmtDouble(pBeam->q0.x()) << " "
                    << FmtDouble(pBeam->q0.y()) << " "
                    << FmtDouble(pBeam->q0.z());
            }
            stream << "\n";
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
            auto constraintNode = pCon->m_pNode.lock();
            // ID, NodeID, Dir, Value
            stream << FmtInt(mapId(constraintIdMap, pair.first), 10, true) << " "
                << FmtInt(constraintNode ? mapId(nodeIdMap, constraintNode->m_Id) : 0) << " "
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
                auto loadNode = pL->m_pNode.lock();
                stream << FmtInt(mapId(loadIdMap, pL->m_Id), 10, true) << " "
                    << FmtInt(loadNode ? mapId(nodeIdMap, loadNode->m_Id) : 0) << " "
                    << FmtInt(static_cast<int>(pL->m_Direction)) << " "
                    << FmtDouble(pL->m_Value) << " "
                    << FmtInt(mapId(stepIdMap, pL->m_StepId)) << " "
                    << FmtDouble(pL->m_StartTime) << " "
                    << FmtDouble(pL->m_EndTime) << "\n";
            }
            else if (group.first == "FORCE_ELEMENT")
            {
                auto pL = std::dynamic_pointer_cast<Force_Element>(pLoad);
                auto loadElement = pL->m_pElement.lock();
                stream << FmtInt(mapId(loadIdMap, pL->m_Id), 10, true) << " "
                    << FmtInt(loadElement ? mapId(elementIdMap, loadElement->m_Id) : 0) << " "
                    << FmtInt(static_cast<int>(pL->m_Direction)) << " "
                    << FmtDouble(pL->m_Value) << "\n";
            }
            else if (group.first == "FORCE_GRAVITY")
            {
                auto pL = std::dynamic_pointer_cast<Force_Gravity>(pLoad);
                stream << FmtInt(mapId(loadIdMap, pL->m_Id), 10, true) << " "
                    << FmtInt(static_cast<int>(pL->m_Direction)) << " "
                    << FmtDouble(pL->m_g) << " "
                    << FmtInt(mapId(stepIdMap, pL->m_StepId)) << "\n";
            }
            else if (group.first == "FORCE_WIND")
            {
                auto pL = std::dynamic_pointer_cast<Force_Wind>(pLoad);
                stream << FmtInt(mapId(loadIdMap, pL->m_Id), 10, true) << " "
                    << FmtInt(static_cast<int>(pL->m_Direction)) << " "
                    << FmtDouble(pL->m_velocity) << " "
                    << FmtInt(mapId(stepIdMap, pL->m_StepId)) << "\n";
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
            stream << FmtInt(mapId(elementIdMap, pair.first), 10, true) << " " << FmtDouble(pElem->m_InitStress) << "\n";
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
            stream << FmtInt(mapId(stepIdMap, pair.first), 10, true) << " "
                << FmtStr(pStep->GetTypeName()) << " "
                << FmtDouble(pStep->m_Time) << " "
                << FmtDouble(pStep->m_StepSize) << " "
                << FmtDouble(pStep->m_Tolerance) << " "
                << FmtInt(pStep->m_MaxIterations) << "\n";
        }
    }

    file.close();
    qDebug() << "Model saved to" << fileName;
    return true;
}

bool Outputter::SaveHdf5File(const QString& fileName, StructureData* pData,
    const QString& sourceModelName, bool resultComplete)
{
    Hdf5ModelIO hdf5ResultIO;
    return hdf5ResultIO.ExportHdf5(fileName, pData, sourceModelName, resultComplete);
}

bool Outputter::ExportBdfResultFromHdf5(const QString& hdf5FileName,
    const QString& bdfFileName,
    const std::vector<int>& nodeIds,
    const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
    const std::vector<int>& elementIds,
    const std::vector<EnumKeyword::ElementResultType>& elementTypes) const
{
    Hdf5ModelIO hdf5ResultIO;
    return hdf5ResultIO.ExportBdfResultFromHdf5(hdf5FileName, bdfFileName,
        nodeIds, nodeTypes, elementIds, elementTypes);
}

void Outputter::SaveModel(const QString& fileName, StructureData* pData)
{
    SaveBdfModel(fileName, pData);
}
