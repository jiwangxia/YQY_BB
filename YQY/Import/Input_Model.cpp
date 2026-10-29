#include "Input_Model.h"
#include "Application/ApplicationPaths.h"
#include "DataStructure/Factory/ElementFactory.h"
#include "DataStructure/Factory/SectionFactory.h"
#include "DataStructure/Structure/StructureData.h"
#include "Utility/Logger/Logger.h"
#include <QRegularExpression>
#include <QStringDecoder>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

namespace
{
QStringConverter::Encoding DetectTextEncoding(QFile& file)
{
    const QByteArray bytes = file.peek(file.size());
    QStringDecoder utf8Decoder(QStringConverter::Utf8);
    utf8Decoder.decode(bytes);
    return utf8Decoder.hasError() ? QStringConverter::System : QStringConverter::Utf8;
}

bool RequireKeywordFieldCount(const QStringList& fields, int expected, const QString& keyword, _OUT QString& error)
{
    if (fields.size() == expected)
    {
        return true;
    }

    error = QStringLiteral("%1 关键字字段数量错误，期望 %2，实际 %3").arg(keyword).arg(expected).arg(fields.size());
    qDebug().noquote() << QStringLiteral("Error: %1 关键字字段数量错误，期望 %2，实际 %3")
                              .arg(keyword)
                              .arg(expected)
                              .arg(fields.size());
    return false;
}

bool IsSpringElement(const ElementBase& element)
{
    return dynamic_cast<const ElementSpringBase*>(&element) != nullptr;
}

bool ParsePositiveId(const QString& text, _OUT int& value)
{
    bool ok = false;
    value = text.toInt(&ok);
    return ok && value > 0;
}

bool ParseNonNegativeInteger(const QString& text, _OUT int& value)
{
    bool ok = false;
    value = text.toInt(&ok);
    return ok && value >= 0;
}

bool ParseFiniteDouble(const QString& text, _OUT double& value)
{
    bool ok = false;
    value = text.toDouble(&ok);
    return ok && std::isfinite(value);
}

bool ParseSpringDOF(const QString& text, _OUT int& dof)
{
    bool ok = false;
    dof = text.toInt(&ok);
    return ok && dof >= 0 && dof <= 5;
}
}

bool Input_Model::ReadLine(QTextStream& flow, _OUT QString& str)
{
    while (!flow.atEnd())
    {
        str = flow.readLine().trimmed();
        if (str.left(2).compare("**") == 0)
            continue; //注释行，继续读取下一行
        if (str.left(2).compare("//") == 0)
            continue;

        if (str.isEmpty())
            continue; //注释行，继续读取下一行

        return true; //读取到有效数据，完成一行读取
    }

    return false; //文件读完，没有得到有效的数据行
}

bool Input_Model::InputData(const QString& FileName, std::shared_ptr<StructureData> pStructure)
{
    m_LastError.clear();
    if (!pStructure)
    {
        m_LastError = QStringLiteral("结构数据为空。");
        qDebug().noquote() << QStringLiteral("Error:") << m_LastError;
        return false;
    }

    // 解析、清理和校验仅修改临时模型，失败不会污染调用方已有数据。
    m_Structure = std::make_shared<StructureData>();
    m_Structure->m_OutputControl.m_SourceModelName = FileName;

    m_Structure->m_OutputControl.m_Hdf5FileName = ApplicationPaths::hdf5ResultFilePath(FileName);

    QFile file(FileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        m_LastError = QStringLiteral("文件不存在或无法打开：%1").arg(FileName);
        qDebug().noquote() << QStringLiteral("Error:") << m_LastError;
        return false;
    }

    QTextStream flow(&file);
    flow.setEncoding(DetectTextEncoding(file));
    QString str;
    while (ReadLine(flow, str))
    {
        // 预处理：判断是否是关键字行
        if (!str.startsWith("*"))
            continue; // 如果不是以*开头，跳过（或者根据需求处理非关键字行）

        QString cleanStr = str.mid(1); // 去掉开头的*
        QStringList list_str = cleanStr.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (list_str.isEmpty())
            continue;

        QString keyword = list_str[0].trimmed();
        // 使用 Map 进行映射，统一转大写
        auto key = EnumKeyword::MapKeyData.value(keyword.toUpper(), EnumKeyword::KeyData::UNKNOWN);

        if (key == EnumKeyword::KeyData::UNKNOWN)
            continue;

        // 数量字段必须先校验，避免非法文本被 toInt 静默转换为零。
        const bool countIsThirdField = key == EnumKeyword::KeyData::ELEMENT || key == EnumKeyword::KeyData::LOAD ||
                                       key == EnumKeyword::KeyData::SPRING ||
                                       key == EnumKeyword::KeyData::CONSTRAINT_TABULAR;
        const int countIndex = countIsThirdField ? 2 : 1;
        int recordCount = 0;
        if (list_str.size() <= countIndex || !ParseNonNegativeInteger(list_str[countIndex], recordCount))
        {
            m_LastError = QStringLiteral("%1 的数量字段必须为非负整数：%2").arg(keyword, str);
            return false;
        }

        bool inputSucceeded = true;
        switch (key)
        {
            case EnumKeyword::KeyData::NODE:
                inputSucceeded = InputNodes(flow, list_str);
                break;
            case EnumKeyword::KeyData::ELEMENT:
                inputSucceeded = InputElement(flow, list_str);
                break;
            case EnumKeyword::KeyData::AERO_ELEMENT:
                inputSucceeded = InputAeroElements(flow, list_str);
                break;
            case EnumKeyword::KeyData::SPRING:
                inputSucceeded = InputSpringBehavior(flow, list_str);
                break;
            case EnumKeyword::KeyData::SECTION:
                inputSucceeded = InputSection(flow, list_str);
                break;
            case EnumKeyword::KeyData::MATERIAL:
                inputSucceeded = InputMaterial(flow, list_str);
                break;
            case EnumKeyword::KeyData::CONSTRAINT:
                inputSucceeded = InputConstraint(flow, list_str);
                break;
            case EnumKeyword::KeyData::CONSTRAINT_TABULAR:
                inputSucceeded = InputConstraintTabular(flow, list_str);
                break;
            case EnumKeyword::KeyData::MPC:
                inputSucceeded = InputMPC(flow, list_str);
                break;
            case EnumKeyword::KeyData::LOAD:
                inputSucceeded = InputLoad(flow, list_str);
                break;
            case EnumKeyword::KeyData::STRESS:
                inputSucceeded = InputElement_Stress(flow, list_str);
                break;
            case EnumKeyword::KeyData::ANALYSIS_STEP:
                inputSucceeded = InputAnalysisStep(flow, list_str);
                break;

            default:
                break;
        }
        if (!inputSucceeded)
        {
            // 保留具体读取函数的诊断，为旧分支补齐可供界面和测试使用的错误信息。
            if (m_LastError.isEmpty())
                m_LastError = QStringLiteral("%1 数据读取失败，请检查字段、数量及对象引用：%2").arg(keyword, FileName);
            return false;
        }
    }
    if (flow.status() != QTextStream::Ok || file.error() != QFileDevice::NoError)
    {
        m_LastError = QStringLiteral("模型文件读取失败：%1").arg(FileName);
        return false;
    }
    file.close();

    // 合并重复节点、删除重复单元、删除孤立节点、重新编号
    m_Structure->CleanupModel();
    m_Structure->EnsureDefaultAnalysisConfiguration();

    if (!ValidateStructure(m_LastError))
    {
        qDebug().noquote() << QStringLiteral("Error: 模型完整性验证失败：") << m_LastError;
        return false;
    }

    // 按类型统计单元数量
    QMap<QString, int> elementTypeCount;
    for (const auto& pair : m_Structure->m_Elements)
    {
        QString typeName = "UNKNOWN";
        if (dynamic_cast<ElementTruss*>(pair.second.get()))
            typeName = "T3D2";
        else if (dynamic_cast<ElementCable*>(pair.second.get()))
            typeName = "CABLE";
        else if (dynamic_cast<ElementBeam_CR*>(pair.second.get()))
            typeName = "CR3D";
        else if (dynamic_cast<ElementSpring1*>(pair.second.get()))
            typeName = "SPRING1";
        else if (dynamic_cast<ElementSpring2*>(pair.second.get()))
            typeName = "SPRING2";
        else if (dynamic_cast<ElementSpringA*>(pair.second.get()))
            typeName = "SPRINGA";
        elementTypeCount[typeName]++;
    }

    // 按类型统计荷载数量
    QMap<QString, int> loadTypeCount;
    for (const auto& pair : m_Structure->m_Load)
    {
        QString typeName = "UNKNOWN";
        if (dynamic_cast<Force_Node*>(pair.second.get()))
            typeName = "FORCE_NODE";
        else if (dynamic_cast<Force_Element*>(pair.second.get()))
            typeName = "FORCE_ELEMENT";
        else if (dynamic_cast<Force_Gravity*>(pair.second.get()))
            typeName = "FORCE_GRAVITY";
        else if (dynamic_cast<Force_Wind*>(pair.second.get()))
            typeName = "FORCE_WIND";
        // 可添加其他荷载类型
        loadTypeCount[typeName]++;
    }

    QStringList summary;
    summary << QStringLiteral("模型信息:");
    summary << QStringLiteral("节点数量: %1").arg(m_Structure->m_Nodes.size());
    summary << QStringLiteral("单元总数: %1").arg(m_Structure->m_Elements.size());
    for (auto it = elementTypeCount.constBegin(); it != elementTypeCount.constEnd(); ++it)
    {
        summary << QStringLiteral("%1: %2").arg(it.key()).arg(it.value());
    }
    summary << QStringLiteral("材料数量: %1").arg(m_Structure->m_Material.size());
    summary << QStringLiteral("截面数量: %1").arg(m_Structure->m_Section.size());
    summary << QStringLiteral("约束数量: %1").arg(m_Structure->m_Constraint.size());
    summary << QStringLiteral("MPC数量: %1").arg(m_Structure->m_MPCConstraints.size());
    summary << QStringLiteral("荷载总数: %1").arg(m_Structure->m_Load.size());
    for (auto it = loadTypeCount.constBegin(); it != loadTypeCount.constEnd(); ++it)
    {
        summary << QStringLiteral("%1: %2").arg(it.key()).arg(it.value());
    }
    for (const auto& [loadId, load] : m_Structure->m_Load)
    {
        if (dynamic_cast<const Force_Gravity*>(load.get()))
            summary << QStringLiteral("重力方向: %1").arg(static_cast<int>(load->m_Direction));
    }
    summary << QStringLiteral("分析步数量: %1").arg(m_Structure->m_AnalysisStep.size());

    Logger::Instance().InfoToFile(summary.join(QStringLiteral("\n")));

    pStructure->ReplaceModelFrom(std::move(*m_Structure));
    m_Structure = pStructure;
    return true;
}

bool Input_Model::ValidateStructure(_OUT QString& errorMessage) const
{
    if (!m_Structure)
    {
        errorMessage = QStringLiteral("结构数据为空。");
        return false;
    }
    if (m_Structure->m_Nodes.empty())
    {
        errorMessage = QStringLiteral("模型没有有效节点。");
        return false;
    }
    if (m_Structure->m_Elements.empty())
    {
        errorMessage = QStringLiteral("模型没有有效单元。");
        return false;
    }
    const bool hasNonSpringElement = std::any_of(m_Structure->m_Elements.cbegin(), m_Structure->m_Elements.cend(),
                                                 [](const auto& item)
                                                 {
                                                     return item.second && !IsSpringElement(*item.second);
                                                 });
    if (hasNonSpringElement && m_Structure->m_Material.empty())
    {
        errorMessage = QStringLiteral("模型缺少材料属性。");
        return false;
    }
    if (hasNonSpringElement && m_Structure->m_Section.empty())
    {
        errorMessage = QStringLiteral("模型缺少截面属性。");
        return false;
    }
    if (hasNonSpringElement && m_Structure->m_Property.empty())
    {
        errorMessage = QStringLiteral("模型没有建立有效的材料-截面属性关联。");
        return false;
    }

    for (const auto& [nodeId, node] : m_Structure->m_Nodes)
    {
        if (!node)
        {
            errorMessage = QStringLiteral("节点 %1 为空。").arg(nodeId);
            return false;
        }
        if (!std::isfinite(node->m_X) || !std::isfinite(node->m_Y) || !std::isfinite(node->m_Z))
        {
            errorMessage = QStringLiteral("节点 %1 的坐标不是有限数值。").arg(nodeId);
            return false;
        }
    }

    for (const auto& [materialId, material] : m_Structure->m_Material)
    {
        if (!material || !std::isfinite(material->m_Young) || material->m_Young <= 0.0)
        {
            errorMessage = QStringLiteral("材料 %1 的弹性模量无效。").arg(materialId);
            return false;
        }
        if (!std::isfinite(material->m_Density) || material->m_Density < 0.0)
        {
            errorMessage = QStringLiteral("材料 %1 的密度无效。").arg(materialId);
            return false;
        }
        if (!std::isfinite(material->m_Poisson) || material->m_Poisson <= -1.0 || material->m_Poisson >= 0.5)
        {
            errorMessage = QStringLiteral("材料 %1 的泊松比必须大于 -1 且小于 0.5。").arg(materialId);
            return false;
        }
    }

    for (const auto& [sectionId, section] : m_Structure->m_Section)
    {
        if (!section || !std::isfinite(section->m_Area) || section->m_Area <= 0.0)
        {
            errorMessage = QStringLiteral("截面 %1 的面积无效。").arg(sectionId);
            return false;
        }
    }

    for (const auto& [propertyId, property] : m_Structure->m_Property)
    {
        if (!property)
        {
            errorMessage = QStringLiteral("属性 %1 为空。").arg(propertyId);
            return false;
        }
        const auto material = property->m_pMaterial.lock();
        const auto section = property->m_pSection.lock();
        if (!material || !section)
        {
            errorMessage = QStringLiteral("属性 %1 未同时关联有效材料和截面。").arg(propertyId);
            return false;
        }
        const auto materialIt = m_Structure->m_Material.find(material->m_Id);
        const auto sectionIt = m_Structure->m_Section.find(section->m_Id);
        if (materialIt == m_Structure->m_Material.end() || materialIt->second.get() != material.get() ||
            sectionIt == m_Structure->m_Section.end() || sectionIt->second.get() != section.get())
        {
            errorMessage = QStringLiteral("属性 %1 引用了不属于当前模型的材料或截面。").arg(propertyId);
            return false;
        }
    }

    for (const auto& [elementId, element] : m_Structure->m_Elements)
    {
        if (!element || element->m_pNode.isEmpty())
        {
            errorMessage = QStringLiteral("单元 %1 为空或没有节点。").arg(elementId);
            return false;
        }
        for (const auto& weakNode : element->m_pNode)
        {
            const auto node = weakNode.lock();
            const auto nodeIt = node ? m_Structure->m_Nodes.find(node->m_Id) : m_Structure->m_Nodes.end();
            if (!node || nodeIt == m_Structure->m_Nodes.end() || nodeIt->second.get() != node.get())
            {
                errorMessage = QStringLiteral("单元 %1 引用了无效节点。").arg(elementId);
                return false;
            }
        }
        if (!ValidateElementGeometry(*element, errorMessage))
            return false;
        if (const auto* spring = dynamic_cast<const ElementSpringBase*>(element.get()))
        {
            const auto behavior = spring->m_pSpringBehavior.lock();
            const auto behaviorIt =
                behavior ? m_Structure->m_SpringBehaviors.find(behavior->m_Id) : m_Structure->m_SpringBehaviors.end();
            if (!behavior || behaviorIt == m_Structure->m_SpringBehaviors.end() ||
                behaviorIt->second.get() != behavior.get())
            {
                errorMessage = QStringLiteral("弹簧单元 %1 没有关联有效的弹簧行为。").arg(elementId);
                return false;
            }
            continue;
        }

        const auto property = element->m_pProperty.lock();
        const auto propertyIt = property ? m_Structure->m_Property.find(property->m_Id) : m_Structure->m_Property.end();
        if (!property || propertyIt == m_Structure->m_Property.end() || propertyIt->second.get() != property.get())
        {
            errorMessage = QStringLiteral("单元 %1 没有关联有效的材料-截面属性。").arg(elementId);
            return false;
        }
    }

    if (!ValidateLoadsAndConstraints(errorMessage))
        return false;

    std::set<std::pair<int, int>> dependentDofs;
    for (const auto& [mpcId, mpc] : m_Structure->m_MPCConstraints)
    {
        if (!mpc)
        {
            errorMessage = QStringLiteral("MPC %1为空。").arg(mpcId);
            return false;
        }
        const std::vector<int> nodeIds = mpc->GetNodeIds();
        if (nodeIds.size() != 2)
        {
            errorMessage = QStringLiteral("MPC %1没有有效的主从节点。").arg(mpcId);
            return false;
        }
        const auto master = m_Structure->m_Nodes.find(nodeIds[0]);
        const auto slave = m_Structure->m_Nodes.find(nodeIds[1]);
        if (master == m_Structure->m_Nodes.cend() || slave == m_Structure->m_Nodes.cend() || !master->second ||
            !slave->second)
        {
            errorMessage = QStringLiteral("MPC %1引用了无效节点。").arg(mpcId);
            return false;
        }
        for (const int direction : mpc->m_SlaveDirections)
        {
            if (direction < 0 || direction >= slave->second->m_DOF.size())
            {
                errorMessage = QStringLiteral("MPC %1的从自由度%2超过节点%3的自由度范围。")
                                   .arg(mpcId)
                                   .arg(direction)
                                   .arg(nodeIds[1]);
                return false;
            }
            if (!dependentDofs.emplace(nodeIds[1], direction).second)
            {
                errorMessage =
                    QStringLiteral("节点%1的自由度%2被多个MPC重复指定为从自由度。").arg(nodeIds[1]).arg(direction);
                return false;
            }
            for (const auto& [constraintId, constraint] : m_Structure->m_Constraint)
            {
                const auto constrainedNode = constraint ? constraint->m_pNode.lock() : nullptr;
                if (constrainedNode && constrainedNode->m_Id == nodeIds[1] &&
                    static_cast<int>(constraint->m_Direction) == direction)
                {
                    errorMessage =
                        QStringLiteral("MPC %1的从自由度同时被位移约束%2固定。").arg(mpcId).arg(constraintId);
                    return false;
                }
            }
        }
    }

    errorMessage.clear();
    return true;
}

bool Input_Model::ValidateElementGeometry(const ElementBase& element, _OUT QString& errorMessage) const
{
    // 固定方向弹簧允许重合节点，轴向弹簧和杆梁必须具有有效轴线。
    if (dynamic_cast<const ElementSpring1*>(&element) || dynamic_cast<const ElementSpring2*>(&element))
        return true;
    if (element.m_pNode.size() != 2)
    {
        errorMessage = QStringLiteral("单元 %1 必须包含两个节点。").arg(element.m_Id);
        return false;
    }

    const auto first = element.m_pNode[0].lock();
    const auto second = element.m_pNode[1].lock();
    const Vector3d chord(second->m_X - first->m_X, second->m_Y - first->m_Y, second->m_Z - first->m_Z);
    const double length = chord.norm();
    if (!std::isfinite(length) || length <= 1.0e-12)
    {
        errorMessage = QStringLiteral("单元 %1 的初始长度无效或两个节点重合。").arg(element.m_Id);
        return false;
    }

    const auto* beam = dynamic_cast<const ElementBeam_CR*>(&element);
    if (!beam)
        return true;
    const double directionNorm = beam->q0.norm();
    if (!std::isfinite(directionNorm) || directionNorm <= 0.0 ||
        (chord / length).cross(beam->q0 / directionNorm).norm() <= 1.0e-12)
    {
        errorMessage = QStringLiteral("CR3D 单元 %1 的方向向量必须非零且不能平行于轴线。").arg(element.m_Id);
        return false;
    }
    return true;
}

bool Input_Model::ValidateLoadsAndConstraints(_OUT QString& errorMessage) const
{
    for (const auto& [constraintId, constraint] : m_Structure->m_Constraint)
    {
        const auto node = constraint ? constraint->m_pNode.lock() : nullptr;
        const auto nodeIt = node ? m_Structure->m_Nodes.find(node->m_Id) : m_Structure->m_Nodes.end();
        if (!node || nodeIt == m_Structure->m_Nodes.end() || nodeIt->second != node)
        {
            errorMessage = QStringLiteral("约束 %1 引用了无效节点。").arg(constraintId);
            return false;
        }
        const int direction = static_cast<int>(constraint->m_Direction);
        if (direction < 0 || direction >= node->m_DOF.size())
        {
            errorMessage = QStringLiteral("约束 %1 的方向 %2 超过节点 %3 的自由度范围。")
                               .arg(constraintId)
                               .arg(direction)
                               .arg(node->m_Id);
            return false;
        }
    }

    for (const auto& [loadId, load] : m_Structure->m_Load)
    {
        if (!load || (load->m_StepId > 0 && !m_Structure->m_AnalysisStep.contains(load->m_StepId)))
        {
            errorMessage = QStringLiteral("荷载 %1 为空或引用了不存在的分析步。").arg(loadId);
            return false;
        }
        const int direction = static_cast<int>(load->m_Direction);
        if (const auto* force = dynamic_cast<const Force_Node*>(load.get()))
        {
            const auto node = force->m_pNode.lock();
            const auto nodeIt = node ? m_Structure->m_Nodes.find(node->m_Id) : m_Structure->m_Nodes.end();
            if (!node || nodeIt == m_Structure->m_Nodes.end() || nodeIt->second != node || direction < 0 ||
                direction >= node->m_DOF.size())
            {
                errorMessage = QStringLiteral("节点荷载 %1 的节点引用或自由度方向无效。").arg(loadId);
                return false;
            }
        }
        else if (const auto* force = dynamic_cast<const Force_Element*>(load.get()))
        {
            const auto element = force->m_pElement.lock();
            const auto elementIt =
                element ? m_Structure->m_Elements.find(element->m_Id) : m_Structure->m_Elements.end();
            if (!element || elementIt == m_Structure->m_Elements.end() || elementIt->second != element ||
                element->m_pNode.size() != 2 || direction < 0 || direction >= element->Get_NodeDOF())
            {
                errorMessage =
                    QStringLiteral("单元荷载 %1 的单元引用或自由度方向无效，且必须作用于双节点单元。").arg(loadId);
                return false;
            }
        }
    }
    return true;
}

bool Input_Model::InputNodes(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "NODE", m_LastError))
        return false;

    int nNode = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nNode; i++)
    {
        // 读取下一行有效数据
        if (!ReadLine(flow, strdata))
        {
            m_LastError = QStringLiteral("节点数据不足");
            qDebug().noquote() << QStringLiteral("Error: 节点数据不足");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            m_LastError = QStringLiteral("节点数据不足，遇到下一个关键字: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 节点数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_node = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (strlist_node.size() != 4)
        {
            m_LastError = QStringLiteral("节点数据格式错误，需要4个字段: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 节点数据格式错误，需要4个字段: ") << strdata;
            return false;
        }

        double xi = 0.0, yi = 0.0, zi = 0.0;
        if (!ParseFiniteDouble(strlist_node[1], xi) || !ParseFiniteDouble(strlist_node[2], yi) ||
            !ParseFiniteDouble(strlist_node[3], zi))
        {
            m_LastError = QStringLiteral("节点坐标必须为有限数值：%1").arg(strdata);
            return false;
        }

        int autoId = static_cast<int>(m_Structure->m_Nodes.size()) + 1;

        // 保存到模型数据库
        auto pNode = std::make_shared<Node>();
        pNode->m_Id = autoId;
        pNode->m_X = xi;
        pNode->m_Y = yi;
        pNode->m_Z = zi;

        m_Structure->m_Nodes.insert(std::make_pair(autoId, pNode));
    }

    //for (auto& a : m_Structure->m_Nodes)
    //{
    //    qDebug() << a.first << " " << a.second->m_X << " " << a.second->m_Y << " " << a.second->m_Z;
    //}
    return true;
}

bool Input_Model::InputElement(QTextStream& flow, const QStringList& list_str)
{
    // *ELEMENT, TYPE_NAME, N
    if (!RequireKeywordFieldCount(list_str, 3, "ELEMENT", m_LastError))
        return false;

    QString typeStr = list_str[1].trimmed().toUpper();
    EnumKeyword::ElementType elementType =
        EnumKeyword::MapElementType.value(typeStr, EnumKeyword::ElementType::UNKNOWN);

    int nElement = list_str[2].toInt();

    switch (elementType)
    {
        case EnumKeyword::ElementType::T3D2:
        case EnumKeyword::ElementType::CABLE:
        case EnumKeyword::ElementType::CR3D:
            return InputLineElements(flow, elementType, nElement);
        case EnumKeyword::ElementType::SPRING1:
            return InputElementSpring1(flow, list_str, nElement);
        case EnumKeyword::ElementType::SPRING2:
            return InputElementSpring2(flow, list_str, nElement);
        case EnumKeyword::ElementType::SPRINGA:
            return InputElementSpringA(flow, list_str, nElement);
        default:
            m_LastError = QStringLiteral("未知的单元类型: %1").arg(typeStr);
            qDebug().noquote() << QStringLiteral("Error: 未知的单元类型: ") << typeStr;
            return false;
    }
}

bool Input_Model::InputAeroElements(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "AERO_ELEMENT", m_LastError))
        return false;

    bool countOk = false;
    const int count = list_str[1].toInt(&countOk);
    if (!countOk || count < 0)
    {
        m_LastError = QStringLiteral("AERO_ELEMENT 数量无效。");
        return false;
    }

    for (int index = 0; index < count; ++index)
    {
        QString line;
        if (!ReadLine(flow, line))
        {
            m_LastError = QStringLiteral("AERO_ELEMENT 数据不完整。");
            return false;
        }

        const QStringList fields = line.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (fields.size() != 5)
        {
            m_LastError = QStringLiteral("AERO_ELEMENT 第 %1 行字段数量无效。").arg(index + 1);
            return false;
        }

        bool elementOk = false;
        bool roleOk = false;
        bool wireOk = false;
        bool bundleOk = false;
        bool profileOk = false;
        const int elementId = fields[0].toInt(&elementOk);
        const int role = fields[1].toInt(&roleOk);
        const int wireId = fields[2].toInt(&wireOk);
        const int bundleCount = fields[3].toInt(&bundleOk);
        const int profileId = fields[4].toInt(&profileOk);
        if (!elementOk || !roleOk || !wireOk || !bundleOk || !profileOk ||
            role < static_cast<int>(ElementRole::Generic) || role > static_cast<int>(ElementRole::SuspensionHardware) ||
            bundleCount <= 0 || profileId < 0)
        {
            m_LastError = QStringLiteral("AERO_ELEMENT 第 %1 行数据无效。").arg(index + 1);
            return false;
        }

        const auto element = m_Structure->FindElement(elementId);
        if (!element)
        {
            m_LastError = QStringLiteral("AERO_ELEMENT 引用了不存在的单元 %1。").arg(elementId);
            return false;
        }

        element->m_Role = static_cast<ElementRole>(role);
        element->m_WireId = wireId;
        element->m_AeroBundleCount = bundleCount;
        element->m_AeroProfileId = profileId;
    }
    return true;
}

bool Input_Model::InputLineElements(QTextStream& flow, EnumKeyword::ElementType type, int nElement)
{
    QString typeName;
    QString idTypeName;
    switch (type)
    {
        case EnumKeyword::ElementType::T3D2:
            typeName = QStringLiteral("桁架");
            idTypeName = QStringLiteral("truss");
            break;
        case EnumKeyword::ElementType::CABLE:
            typeName = QStringLiteral("索");
            idTypeName = QStringLiteral("cable");
            break;
        case EnumKeyword::ElementType::CR3D:
            typeName = QStringLiteral("CR3D");
            idTypeName = QStringLiteral("CR3D");
            break;
        default:
            return false;
    }

    const bool isBeam = type == EnumKeyword::ElementType::CR3D;
    const int fieldCount = isBeam ? 8 : 5;
    const QString formatName = isBeam ? QStringLiteral("梁") : typeName;
    const QString referenceName = typeName + (isBeam ? QStringLiteral(" 单元") : QStringLiteral("单元"));
    QString strdata;
    for (int i = 0; i < nElement; ++i)
    {
        if (!ReadLine(flow, strdata))
        {
            m_LastError = QStringLiteral("%1单元数据不够").arg(typeName);
            qDebug().noquote() << QStringLiteral("Error: %1单元数据不够").arg(typeName);
            return false;
        }
        if (strdata.startsWith("*"))
        {
            m_LastError = QStringLiteral("%1单元数据不足，遇到下一个关键字: %2").arg(typeName).arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: %1单元数据不足，遇到下一个关键字: ").arg(typeName) << strdata;
            return false;
        }

        const QStringList fields = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (fields.size() != fieldCount)
        {
            m_LastError = QStringLiteral("%1单元数据格式错误: %2").arg(formatName).arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: %1单元数据格式错误: ").arg(formatName) << strdata;
            return false;
        }

        bool idOk = false;
        const int elementId = fields[0].toInt(&idOk);
        if (!idOk || elementId <= 0 || m_Structure->m_Elements.find(elementId) != m_Structure->m_Elements.end())
        {
            m_LastError = QStringLiteral("invalid or duplicate %1 element ID: %2").arg(idTypeName).arg(fields[0]);
            qDebug().noquote() << QStringLiteral("Error: invalid or duplicate %1 element ID: ").arg(idTypeName)
                               << fields[0];
            return false;
        }
        const int firstNodeId = fields[1].toInt();
        const int secondNodeId = fields[2].toInt();
        const int materialId = fields[3].toInt();
        const int sectionId = fields[4].toInt();

        ElementCreationOptions options;
        if (isBeam)
        {
            double x = 0.0, y = 0.0, z = 0.0;
            if (!ParseFiniteDouble(fields[5], x) || !ParseFiniteDouble(fields[6], y) ||
                !ParseFiniteDouble(fields[7], z))
            {
                m_LastError = QStringLiteral("CR3D 方向向量必须为有限数值：%1").arg(strdata);
                return false;
            }
            options.beamDirection = Vector3d(x, y, z);
        }
        auto element = ElementFactory::Create(type, options);
        element->m_Id = elementId;
        const auto firstNode = m_Structure->FindNode(firstNodeId);
        const auto secondNode = m_Structure->FindNode(secondNodeId);
        if (!firstNode || !secondNode)
        {
            m_LastError =
                QStringLiteral("%1引用了不存在的节点: %2 %3").arg(referenceName).arg(firstNodeId).arg(secondNodeId);
            qDebug().noquote() << QStringLiteral("Error: %1引用了不存在的节点: ").arg(referenceName) << firstNodeId
                               << secondNodeId;
            return false;
        }
        element->m_pNode[0] = firstNode;
        firstNode->SetNumDOFs(element->Get_NodeDOF());
        element->m_pNode[1] = secondNode;
        secondNode->SetNumDOFs(element->Get_NodeDOF());
        const auto property = m_Structure->Create_Property(materialId, sectionId);
        if (!property)
        {
            m_LastError =
                QStringLiteral("%1属性创建失败，材料/截面 id= %2 %3").arg(referenceName).arg(materialId).arg(sectionId);
            qDebug().noquote() << QStringLiteral("Error: %1属性创建失败，材料/截面 id=").arg(referenceName)
                               << materialId << sectionId;
            return false;
        }
        element->m_pProperty = property;
        m_Structure->m_Elements.insert(std::make_pair(elementId, std::move(element)));
    }
    return true;
}

bool Input_Model::InputSpringBehavior(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 3, "SPRING", m_LastError))
        return false;

    int behaviorId = 0;
    int pointCount = 0;
    if (!ParsePositiveId(list_str[1], behaviorId) || !ParsePositiveId(list_str[2], pointCount) ||
        m_Structure->m_SpringBehaviors.contains(behaviorId))
    {
        m_LastError = QStringLiteral("SPRING 行为编号或数据点数量无效。");
        return false;
    }

    auto behavior = std::make_shared<SpringBehavior>();
    behavior->m_Id = behaviorId;
    double previousDisplacement = 0.0;
    for (int index = 0; index < pointCount; ++index)
    {
        QString line;
        if (!ReadLine(flow, line) || line.startsWith('*'))
        {
            m_LastError = QStringLiteral("SPRING %1 的力-位移数据不完整。").arg(behaviorId);
            return false;
        }
        const QStringList fields = line.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (fields.size() != 2)
        {
            m_LastError =
                QStringLiteral("SPRING %1 的第 %2 个数据点必须包含力和相对位移。").arg(behaviorId).arg(index + 1);
            return false;
        }

        bool forceOk = false;
        bool displacementOk = false;
        const double force = fields[0].toDouble(&forceOk);
        const double displacement = fields[1].toDouble(&displacementOk);
        if (!forceOk || !displacementOk || !std::isfinite(force) || !std::isfinite(displacement) ||
            (index > 0 && displacement <= previousDisplacement))
        {
            m_LastError = QStringLiteral("SPRING %1 的相对位移必须为严格递增的有限数值。").arg(behaviorId);
            return false;
        }
        behavior->m_Points.push_back({force, displacement});
        previousDisplacement = displacement;
    }
    m_Structure->m_SpringBehaviors.emplace(behaviorId, std::move(behavior));
    return true;
}

bool Input_Model::InputElementSpring1(QTextStream& flow, const QStringList& /*list_str*/, int nElement)
{
    for (int index = 0; index < nElement; ++index)
    {
        QString line;
        if (!ReadLine(flow, line) || line.startsWith('*'))
        {
            m_LastError = QStringLiteral("SPRING1 单元数据不完整。");
            return false;
        }
        const QStringList fields = line.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        int elementId = 0, nodeId = 0, dof = 0, behaviorId = 0;
        if (fields.size() != 4 || !ParsePositiveId(fields[0], elementId) || !ParsePositiveId(fields[1], nodeId) ||
            !ParseSpringDOF(fields[2], dof) || !ParsePositiveId(fields[3], behaviorId) ||
            m_Structure->m_Elements.contains(elementId))
        {
            m_LastError = QStringLiteral("SPRING1 单元必须为：ID 节点 DOF 弹簧行为ID，DOF 可为 0 至 5。");
            return false;
        }
        const auto node = m_Structure->FindNode(nodeId);
        const auto behaviorIt = m_Structure->m_SpringBehaviors.find(behaviorId);
        if (!node || behaviorIt == m_Structure->m_SpringBehaviors.end())
        {
            m_LastError = QStringLiteral("SPRING1 %1 引用了不存在的节点或弹簧行为。").arg(elementId);
            return false;
        }
        ElementCreationOptions options;
        options.firstDOF = dof;
        auto element = std::static_pointer_cast<ElementSpringBase>(
            ElementFactory::Create(EnumKeyword::ElementType::SPRING1, options));
        element->m_Id = elementId;
        element->m_pNode[0] = node;
        element->m_pSpringBehavior = behaviorIt->second;
        if (dof >= 3)
            node->SetNumDOFs(6);
        m_Structure->m_Elements.emplace(elementId, std::move(element));
    }
    return true;
}

bool Input_Model::InputElementSpring2(QTextStream& flow, const QStringList& /*list_str*/, int nElement)
{
    for (int index = 0; index < nElement; ++index)
    {
        QString line;
        if (!ReadLine(flow, line) || line.startsWith('*'))
        {
            m_LastError = QStringLiteral("SPRING2 单元数据不完整。");
            return false;
        }
        const QStringList fields = line.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        int elementId = 0, firstNodeId = 0, firstDOF = 0, secondNodeId = 0, secondDOF = 0, behaviorId = 0;
        if (fields.size() != 6 || !ParsePositiveId(fields[0], elementId) || !ParsePositiveId(fields[1], firstNodeId) ||
            !ParseSpringDOF(fields[2], firstDOF) || !ParsePositiveId(fields[3], secondNodeId) ||
            !ParseSpringDOF(fields[4], secondDOF) || !ParsePositiveId(fields[5], behaviorId) ||
            m_Structure->m_Elements.contains(elementId))
        {
            m_LastError = QStringLiteral("SPRING2 单元必须为：ID 节点1 DOF1 节点2 DOF2 弹簧行为ID，DOF 可为 0 至 5。");
            return false;
        }
        const auto firstNode = m_Structure->FindNode(firstNodeId);
        const auto secondNode = m_Structure->FindNode(secondNodeId);
        const auto behaviorIt = m_Structure->m_SpringBehaviors.find(behaviorId);
        if (!firstNode || !secondNode || behaviorIt == m_Structure->m_SpringBehaviors.end())
        {
            m_LastError = QStringLiteral("SPRING2 %1 引用了不存在的节点或弹簧行为。").arg(elementId);
            return false;
        }
        ElementCreationOptions options;
        options.firstDOF = firstDOF;
        options.secondDOF = secondDOF;
        auto element = std::static_pointer_cast<ElementSpringBase>(
            ElementFactory::Create(EnumKeyword::ElementType::SPRING2, options));
        element->m_Id = elementId;
        element->m_pNode[0] = firstNode;
        element->m_pNode[1] = secondNode;
        element->m_pSpringBehavior = behaviorIt->second;
        if (firstDOF >= 3)
            firstNode->SetNumDOFs(6);
        if (secondDOF >= 3)
            secondNode->SetNumDOFs(6);
        m_Structure->m_Elements.emplace(elementId, std::move(element));
    }
    return true;
}

bool Input_Model::InputElementSpringA(QTextStream& flow, const QStringList& /*list_str*/, int nElement)
{
    for (int index = 0; index < nElement; ++index)
    {
        QString line;
        if (!ReadLine(flow, line) || line.startsWith('*'))
        {
            m_LastError = QStringLiteral("SPRINGA 单元数据不完整。");
            return false;
        }
        const QStringList fields = line.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        int elementId = 0, firstNodeId = 0, secondNodeId = 0, behaviorId = 0;
        if (fields.size() != 4 || !ParsePositiveId(fields[0], elementId) || !ParsePositiveId(fields[1], firstNodeId) ||
            !ParsePositiveId(fields[2], secondNodeId) || !ParsePositiveId(fields[3], behaviorId) ||
            m_Structure->m_Elements.contains(elementId))
        {
            m_LastError = QStringLiteral("SPRINGA 单元必须为：ID 节点1 节点2 弹簧行为ID。");
            return false;
        }
        const auto firstNode = m_Structure->FindNode(firstNodeId);
        const auto secondNode = m_Structure->FindNode(secondNodeId);
        const auto behaviorIt = m_Structure->m_SpringBehaviors.find(behaviorId);
        if (!firstNode || !secondNode || firstNode == secondNode || behaviorIt == m_Structure->m_SpringBehaviors.end())
        {
            m_LastError = QStringLiteral("SPRINGA %1 引用了不存在、重复的节点或弹簧行为。").arg(elementId);
            return false;
        }
        const Eigen::Vector3d distance(firstNode->m_X - secondNode->m_X, firstNode->m_Y - secondNode->m_Y,
                                       firstNode->m_Z - secondNode->m_Z);
        if (distance.squaredNorm() <= 1.0e-24)
        {
            m_LastError = QStringLiteral("SPRINGA %1 的两个节点初始坐标重合。").arg(elementId);
            return false;
        }
        auto element =
            std::static_pointer_cast<ElementSpringBase>(ElementFactory::Create(EnumKeyword::ElementType::SPRINGA));
        element->m_Id = elementId;
        element->m_pNode[0] = firstNode;
        element->m_pNode[1] = secondNode;
        element->m_pSpringBehavior = behaviorIt->second;
        m_Structure->m_Elements.emplace(elementId, std::move(element));
    }
    return true;
}

bool Input_Model::InputSection(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "SECTION", m_LastError))
        return false;
    int nSection = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nSection; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            m_LastError = QStringLiteral("截面数据不够");
            qDebug() << QStringLiteral("Error: 截面数据不够");
            return false;
        }

        QStringList strlist_pro = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        const int autoId = static_cast<int>(m_Structure->m_Section.size()) + 1;
        std::shared_ptr<SectionBase> section;

        if (strlist_pro.size() == 2) //圆截面
        {
            double area = 0.0;
            if (!ParseFiniteDouble(strlist_pro[1], area) || area <= 0.0)
            {
                m_LastError = QStringLiteral("圆截面面积必须为正的有限数值：%1").arg(strdata);
                return false;
            }
            section = SectionFactory::CreateCircular(autoId, area);
        }
        else if (strlist_pro.size() == 3) //矩形截面
        {
            double width = 0.0, height = 0.0;
            if (!ParseFiniteDouble(strlist_pro[1], width) || !ParseFiniteDouble(strlist_pro[2], height) ||
                width <= 0.0 || height <= 0.0)
            {
                m_LastError = QStringLiteral("矩形截面的宽和高必须为正的有限数值：%1").arg(strdata);
                return false;
            }
            section = SectionFactory::CreateRectangle(autoId, width, height);
        }
        else if (strlist_pro.size() == 8 || strlist_pro.size() == 11 || strlist_pro.size() == 12)
        {
            // 旧格式：ID, A, Iy, Iz, J, J_rho_x, J_rho_y, J_rho_z。
            // 新格式再附加质量一次矩三个分量，以及可选的独立物理半径。
            bool valuesAreValid = true;
            double values[11] = {};
            const int valueCount = strlist_pro.size() - 1;
            for (int valueIndex = 0; valueIndex < valueCount; ++valueIndex)
            {
                bool converted = false;
                values[valueIndex] = strlist_pro[valueIndex + 1].toDouble(&converted);
                valuesAreValid = valuesAreValid && converted && std::isfinite(values[valueIndex]);
            }
            for (int valueIndex = 0; valueIndex < 4; ++valueIndex)
                valuesAreValid = valuesAreValid && values[valueIndex] > 0.0;
            for (int valueIndex = 4; valueIndex < 7; ++valueIndex)
                valuesAreValid =
                    valuesAreValid && (strlist_pro.size() == 8 ? values[valueIndex] > 0.0 : values[valueIndex] >= 0.0);
            if (strlist_pro.size() == 12)
                valuesAreValid = valuesAreValid && values[10] >= 0.0;
            if (!valuesAreValid)
            {
                m_LastError = QStringLiteral("explicit section requires positive A, Iy, Iz, J; nonnegative mass "
                                             "inertias and radius; finite first moments");
                qDebug().noquote() << QStringLiteral("Error: explicit section requires positive A, Iy, Iz, J; "
                                                     "nonnegative mass inertias and radius; finite first moments");
                return false;
            }

            section = SectionFactory::CreateCircular(autoId, values[0]);
            section->m_HasExplicitSectionInertia = true;
            section->m_ExplicitIy = values[1];
            section->m_ExplicitIz = values[2];
            section->m_ExplicitJ = values[3];
            section->m_MassInertiaPerLength = Eigen::Vector3d(values[4], values[5], values[6]);
            if (strlist_pro.size() >= 11)
                section->m_MassFirstMomentPerLength = Eigen::Vector3d(values[7], values[8], values[9]);
            if (strlist_pro.size() == 12)
                section->m_Radius = values[10];
        }
        else
        {
            m_LastError = QStringLiteral("SECTION supports ID,A; ID,Width,Height; or "
                                         "ID,A,Iy,Iz,J,JrhoX,JrhoY,JrhoZ[,FirstX,FirstY,FirstZ[,Radius]]: %1")
                              .arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: SECTION supports ID,A; ID,Width,Height; "
                                                 "or ID,A,Iy,Iz,J,JrhoX,JrhoY,JrhoZ[,FirstX,FirstY,FirstZ[,Radius]]:")
                               << strdata;
            return false;
        }
        m_Structure->m_Section.insert(std::make_pair(autoId, std::move(section)));
    }

    return true;
}

bool Input_Model::InputMaterial(QTextStream& flow, const QStringList& list_str)
{
    //读取到材料
    if (!RequireKeywordFieldCount(list_str, 2, "MATERIAL", m_LastError))
        return false;
    int nMaterial = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nMaterial; i++)
    {
        //继续读一行有效数据
        if (!ReadLine(flow, strdata))
        { //没有读取到有效数据，退出
            m_LastError = QStringLiteral("材料数据不够");
            qDebug() << QStringLiteral("Error: 材料数据不够");
            return false;
        }

        QStringList strlist_mat = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts); //利用空格,分解字符串
        if (strlist_mat.size() != 6)
        {
            m_LastError = QStringLiteral("材料数据格式错误，需要6个字段: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 材料数据格式错误，需要6个字段: ") << strdata;
            return false;
        }

        double young = 0.0, poisson = 0.0, density = 0.0, maxStress = 0.0, expansion = 0.0;
        if (!ParseFiniteDouble(strlist_mat[1], young) || !ParseFiniteDouble(strlist_mat[2], poisson) ||
            !ParseFiniteDouble(strlist_mat[3], density) || !ParseFiniteDouble(strlist_mat[4], maxStress) ||
            !ParseFiniteDouble(strlist_mat[5], expansion) || young <= 0.0 || density < 0.0)
        {
            m_LastError = QStringLiteral("材料参数必须为有限数值，弹性模量大于零且密度非负：%1").arg(strdata);
            return false;
        }

        int autoId = static_cast<int>(m_Structure->m_Material.size()) + 1;

        //保存到模型数据库
        auto pMaterial = std::make_shared<Material>();
        pMaterial->m_Id = autoId;
        pMaterial->m_Young = young;
        pMaterial->m_Poisson = poisson;
        pMaterial->m_Density = density;
        pMaterial->m_MaxStress = maxStress;
        pMaterial->m_Expansion = expansion;

        m_Structure->m_Material.insert(std::make_pair(autoId, pMaterial));
    }

    return true;
}

bool Input_Model::InputLoad(QTextStream& flow, const QStringList& list_str)
{
    // *LOAD, TYPE_NAME, N
    if (!RequireKeywordFieldCount(list_str, 3, "LOAD", m_LastError))
        return false;

    const QString typeStr = list_str[1].trimmed().toUpper();
    const EnumKeyword::LoadType loadType = EnumKeyword::MapLoadType.value(typeStr, EnumKeyword::LoadType::UNKNOWN);
    const int nLoad = list_str[2].toInt();

    switch (loadType)
    {
        case EnumKeyword::LoadType::FORCE_NODE:
            return InputForceNode(flow, list_str, nLoad);
        case EnumKeyword::LoadType::FORCE_ELEMENT:
            return InputForceElement(flow, list_str, nLoad);
        case EnumKeyword::LoadType::FORCE_GRAVITY:
            return InputForceGravity(flow, list_str, nLoad);
        case EnumKeyword::LoadType::FORCE_WIND:
            return InputForceWind(flow, list_str, nLoad);
        default:
            m_LastError = QStringLiteral("未知的荷载类型");
            qDebug().noquote() << QStringLiteral("Error: 未知的荷载类型");
            return false;
    }
}

// 节点力荷载处理
bool Input_Model::InputForceNode(QTextStream& flow, const QStringList& /*list_str*/, int nLoad)
{
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            m_LastError = QStringLiteral("节点力荷载数据不够");
            qDebug().noquote() << QStringLiteral("Error: 节点力荷载数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            m_LastError = QStringLiteral("节点力荷载数据不足，遇到下一个关键字: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 节点力荷载数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // 兼容格式：ID, NodeID, Direction, Value, StepID
        // 基本格式：ID, NodeID, Direction, Value, StepID, StartTime, EndTime
        // 扩展格式：ID, NodeID, Direction, Value, StepID, StartTime, EndTime, FunctionType, Params...
        if (strlist_load.size() < 5)
        {
            m_LastError = QStringLiteral("节点力荷载数据格式错误: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 节点力荷载数据格式错误: ") << strdata;
            return false;
        }

        int idNode = 0;
        int direction = 0;
        int stepid = 0;
        double value = 0.0;
        double startTime = 0.0;
        double endTime = 1e10;
        if (!ParsePositiveId(strlist_load[1], idNode) || !ParseNonNegativeInteger(strlist_load[2], direction) ||
            direction > 5 || !ParseFiniteDouble(strlist_load[3], value) ||
            !ParseNonNegativeInteger(strlist_load[4], stepid))
        {
            m_LastError = QStringLiteral("节点力荷载基本参数无效。");
            qDebug().noquote() << QStringLiteral("Error:") << m_LastError << strdata;
            return false;
        }
        if (strlist_load.size() >= 6 && !ParseFiniteDouble(strlist_load[5], startTime))
        {
            m_LastError = QStringLiteral("节点力荷载 StartTime 必须为有限数值。");
            qDebug().noquote() << QStringLiteral("Error:") << m_LastError;
            return false;
        }
        if (strlist_load.size() >= 7 && !ParseFiniteDouble(strlist_load[6], endTime))
        {
            m_LastError = QStringLiteral("节点力荷载 EndTime 必须为有限数值。");
            qDebug().noquote() << QStringLiteral("Error:") << m_LastError;
            return false;
        }

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Node>();
        pLoad->m_Id = autoId;
        auto node = m_Structure->FindNode(idNode);
        if (!node)
        {
            m_LastError = QStringLiteral("节点力荷载引用了不存在的节点: %1").arg(idNode);
            qDebug().noquote() << QStringLiteral("Error: 节点力荷载引用了不存在的节点: ") << idNode;
            return false;
        }
        pLoad->m_pNode = node;
        pLoad->m_Direction = static_cast<EnumKeyword::Direction>(direction);
        pLoad->m_Value = value;
        pLoad->m_StepId = stepid;
        pLoad->m_StartTime = startTime;
        pLoad->m_EndTime = endTime;

        // 解析时间函数
        if (strlist_load.size() >= 8)
        {
            const QString funcType = strlist_load[7].trimmed().toUpper();
            bool parametersValid = true;
            if (funcType == "SIN" || funcType == "COS")
            {
                parametersValid = strlist_load.size() >= 12 && ParseFiniteDouble(strlist_load[8], pLoad->m_Amplitude) &&
                                  ParseFiniteDouble(strlist_load[9], pLoad->m_Frequency) &&
                                  ParseFiniteDouble(strlist_load[10], pLoad->m_Phase) &&
                                  ParseFiniteDouble(strlist_load[11], pLoad->m_Offset);
                pLoad->m_FunctionType = funcType == "SIN" ? TimeFunctionType::SIN : TimeFunctionType::COS;
            }
            else if (funcType == "RAMP")
            {
                parametersValid = strlist_load.size() >= 10 && ParseFiniteDouble(strlist_load[8], pLoad->m_RampT0) &&
                                  ParseFiniteDouble(strlist_load[9], pLoad->m_RampT1);
                pLoad->m_FunctionType = TimeFunctionType::RAMP;
            }
            else if (funcType == "EXPONENTIAL")
            {
                parametersValid = strlist_load.size() >= 9 && ParseFiniteDouble(strlist_load[8], pLoad->m_Decay);
                pLoad->m_FunctionType = TimeFunctionType::EXPONENTIAL;
            }
            else if (funcType == "TRIANGULAR")
            {
                parametersValid = strlist_load.size() >= 9 && ParseFiniteDouble(strlist_load[8], pLoad->m_Period);
                pLoad->m_FunctionType = TimeFunctionType::TRIANGULAR;
            }
            else if (funcType == "SQUARE")
            {
                parametersValid = strlist_load.size() >= 10 && ParseFiniteDouble(strlist_load[8], pLoad->m_Period) &&
                                  ParseFiniteDouble(strlist_load[9], pLoad->m_DutyCycle);
                pLoad->m_FunctionType = TimeFunctionType::SQUARE;
            }
            else if (funcType == "CONSTANT" || funcType.isEmpty())
            {
                pLoad->m_FunctionType = TimeFunctionType::CONSTANT;
            }
            else
            {
                m_LastError = QStringLiteral("节点力荷载包含未知的时间函数类型：%1。").arg(funcType);
                qDebug().noquote() << QStringLiteral("Error:") << m_LastError;
                return false;
            }

            if (!parametersValid)
            {
                m_LastError = QStringLiteral("节点力荷载时间函数参数不完整或不是有限数值。");
                qDebug().noquote() << QStringLiteral("Error:") << m_LastError;
                return false;
            }
        }

        QString timeFunctionError;
        if (!pLoad->ValidateTimeFunction(timeFunctionError))
        {
            m_LastError = QStringLiteral("节点力荷载时程校验失败：%1").arg(timeFunctionError);
            qDebug().noquote() << QStringLiteral("Error:") << m_LastError;
            return false;
        }

        m_Structure->m_Load.insert(std::make_pair(autoId, pLoad));
    }
    return true;
}

// 单元压力荷载处理
bool Input_Model::InputForceElement(QTextStream& flow, const QStringList& /*list_str*/, int nLoad)
{
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            m_LastError = QStringLiteral("单元压力荷载数据不够");
            qDebug() << QStringLiteral("Error: 单元压力荷载数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            m_LastError = QStringLiteral("单元力荷载数据不足，遇到下一个关键字: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 单元力荷载数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, EelmentID, Direction, Value
        if (strlist_load.size() != 4)
        {
            m_LastError = QStringLiteral("单元力荷载数据格式错误: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 单元力荷载数据格式错误: ") << strdata;
            return false;
        }

        int idElement = 0, direction = 0;
        double value = 0.0;
        if (!ParsePositiveId(strlist_load[1], idElement) || !ParseSpringDOF(strlist_load[2], direction) ||
            !ParseFiniteDouble(strlist_load[3], value))
        {
            m_LastError = QStringLiteral("单元荷载的单元编号、方向或数值无效：%1").arg(strdata);
            return false;
        }

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Element>();
        pLoad->m_Id = autoId;
        auto element = m_Structure->FindElement(idElement);
        if (!element)
        {
            m_LastError = QStringLiteral("单元荷载引用了不存在的单元: %1").arg(idElement);
            qDebug().noquote() << QStringLiteral("Error: 单元荷载引用了不存在的单元: ") << idElement;
            return false;
        }
        pLoad->m_pElement = element;

        pLoad->m_Direction = static_cast<EnumKeyword::Direction>(direction);
        pLoad->m_Value = value;
        m_Structure->m_Load.insert(std::make_pair(autoId, pLoad));
    }
    return true;
}

bool Input_Model::InputForceGravity(QTextStream& flow, const QStringList& /*list_str*/, int nLoad)
{
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            m_LastError = QStringLiteral("单元重力数据不够");
            qDebug() << QStringLiteral("Error: 单元重力数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            m_LastError = QStringLiteral("单元重力数据不足，遇到下一个关键字: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 单元重力数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, Direction, Value, StepID
        if (strlist_load.size() != 4)
        {
            m_LastError = QStringLiteral("单元重力数据格式错误: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 单元重力数据格式错误: ") << strdata;
            return false;
        }

        double value = 0.0;
        int stepid = 0, direction = 0;
        if (!ParseNonNegativeInteger(strlist_load[1], direction) || direction > 2 ||
            !ParseFiniteDouble(strlist_load[2], value) || !ParseNonNegativeInteger(strlist_load[3], stepid))
        {
            m_LastError = QStringLiteral("重力或风荷载的方向、数值或分析步编号无效：%1").arg(strdata);
            return false;
        }

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Gravity>();
        pLoad->m_Id = autoId;

        pLoad->m_Direction = static_cast<EnumKeyword::Direction>(direction);
        pLoad->m_g = value;
        pLoad->m_StepId = stepid;
        m_Structure->m_Load.insert(std::make_pair(autoId, pLoad));
    }
    return true;
}

bool Input_Model::InputForceWind(QTextStream& flow, const QStringList& /*list_str*/, int nLoad)
{
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            m_LastError = QStringLiteral("单元风荷载数据不够");
            qDebug() << QStringLiteral("Error: 单元风荷载数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            m_LastError = QStringLiteral("单元风荷载数据不足，遇到下一个关键字: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 单元风荷载数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, Direction, Value, StepID
        if (strlist_load.size() != 4)
        {
            m_LastError = QStringLiteral("单元风荷载数据格式错误: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 单元风荷载数据格式错误: ") << strdata;
            return false;
        }

        double value = 0.0;
        int stepid = 0, direction = 0;
        if (!ParseNonNegativeInteger(strlist_load[1], direction) || direction > 2 ||
            !ParseFiniteDouble(strlist_load[2], value) || !ParseNonNegativeInteger(strlist_load[3], stepid))
        {
            m_LastError = QStringLiteral("重力或风荷载的方向、数值或分析步编号无效：%1").arg(strdata);
            return false;
        }

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Wind>();
        pLoad->m_Id = autoId;

        pLoad->m_Direction = static_cast<EnumKeyword::Direction>(direction);
        pLoad->m_velocity = value;
        pLoad->m_StepId = stepid;
        m_Structure->m_Load.insert(std::make_pair(autoId, pLoad));
    }
    return true;
}

bool Input_Model::InputConstraint(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "CONSTRAINT", m_LastError))
        return false;
    int nConstraint = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nConstraint; i++)
    {
        if (!ReadLine(flow, strdata))
        { //没有读取到有效数据，退出
            m_LastError = QStringLiteral("约束数据不够");
            qDebug() << QStringLiteral("Error: 约束数据不够");
            return false;
        }

        QStringList strlist_con = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts); //利用空格,分解字符串
        if (strlist_con.size() != 4)
        {
            m_LastError = QStringLiteral("约束数据格式错误，需要4个字段: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 约束数据格式错误，需要4个字段: ") << strdata;
            return false;
        }

        bool idOk = false;
        bool nodeIdOk = false;
        bool directionOk = false;
        bool valueOk = false;
        const int constraintId = strlist_con[0].toInt(&idOk);
        const int idNode = strlist_con[1].toInt(&nodeIdOk);
        const int direction = strlist_con[2].toInt(&directionOk);
        const double value = strlist_con[3].toDouble(&valueOk);
        if (!idOk || !nodeIdOk || !directionOk || !valueOk || constraintId <= 0 || direction < 0 || direction > 5 ||
            !std::isfinite(value))
        {
            m_LastError = QStringLiteral("约束包含无效字段: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 约束包含无效字段: ") << strdata;
            return false;
        }

        if (m_Structure->m_Constraint.find(constraintId) != m_Structure->m_Constraint.end())
        {
            m_LastError = QStringLiteral("约束编号重复: %1").arg(constraintId);
            qDebug().noquote() << QStringLiteral("Error: 约束编号重复: ") << constraintId;
            return false;
        }

        auto node = m_Structure->FindNode(idNode);
        if (!node)
        {
            m_LastError = QStringLiteral("约束引用了不存在的节点: %1").arg(idNode);
            qDebug().noquote() << QStringLiteral("Error: 约束引用了不存在的节点: ") << idNode;
            return false;
        }

        auto pConstraint = std::make_shared<Constraint>();
        pConstraint->m_Id = constraintId;
        pConstraint->m_pNode = node;
        pConstraint->m_Direction = static_cast<EnumKeyword::Direction>(direction);
        pConstraint->m_Value = value;
        m_Structure->m_Constraint.insert(std::make_pair(constraintId, pConstraint));
    }

    return true;
}

bool Input_Model::InputMPC(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "MPC", m_LastError))
        return false;

    bool countOk = false;
    const int count = list_str[1].toInt(&countOk);
    if (!countOk || count < 1)
    {
        m_LastError = QStringLiteral("MPC数量必须为正整数");
        qDebug().noquote() << QStringLiteral("Error: MPC数量必须为正整数");
        return false;
    }

    for (int index = 0; index < count; ++index)
    {
        QString line;
        if (!ReadLine(flow, line) || line.startsWith("*"))
        {
            m_LastError = QStringLiteral("MPC数据不足");
            qDebug().noquote() << QStringLiteral("Error: MPC数据不足");
            return false;
        }
        const QStringList fields = line.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (fields.size() != 4)
        {
            m_LastError =
                QStringLiteral("MPC数据需要4个字段: MPC_ID, MasterNode, SlaveNode, SlaveDirection: %1").arg(line);
            qDebug().noquote() << QStringLiteral("Error: MPC数据需要4个字段: "
                                                 "MPC_ID, MasterNode, SlaveNode, SlaveDirection: ")
                               << line;
            return false;
        }

        bool idOk = false;
        bool masterOk = false;
        bool slaveOk = false;
        const int id = fields[0].toInt(&idOk);
        const int masterId = fields[1].toInt(&masterOk);
        const int slaveId = fields[2].toInt(&slaveOk);
        const QString directionText = fields[3].trimmed();
        if (!idOk || !masterOk || !slaveOk || id <= 0 || masterId <= 0 || slaveId <= 0 || masterId == slaveId ||
            directionText.isEmpty())
        {
            m_LastError = QStringLiteral("MPC包含无效字段: %1").arg(line);
            qDebug().noquote() << QStringLiteral("Error: MPC包含无效字段: ") << line;
            return false;
        }
        if (m_Structure->m_MPCConstraints.find(id) != m_Structure->m_MPCConstraints.cend())
        {
            m_LastError = QStringLiteral("MPC编号重复: %1").arg(id);
            qDebug().noquote() << QStringLiteral("Error: MPC编号重复: ") << id;
            return false;
        }

        const auto master = m_Structure->FindNode(masterId);
        const auto slave = m_Structure->FindNode(slaveId);
        if (!master || !slave)
        {
            m_LastError = QStringLiteral("MPC引用了不存在的节点: %1 %2").arg(masterId).arg(slaveId);
            qDebug().noquote() << QStringLiteral("Error: MPC引用了不存在的节点: ") << masterId << slaveId;
            return false;
        }

        std::vector<int> directions;
        std::set<int> uniqueDirections;
        for (const QChar character : directionText)
        {
            if (!character.isDigit())
            {
                m_LastError = QStringLiteral("MPC从自由度只能由0到5组成: %1").arg(directionText);
                qDebug().noquote() << QStringLiteral("Error: MPC从自由度只能由0到5组成: ") << directionText;
                return false;
            }
            const int direction = character.digitValue();
            if (direction < 0 || direction > 5 || !uniqueDirections.insert(direction).second)
            {
                m_LastError = QStringLiteral("MPC从自由度无效或重复: %1").arg(directionText);
                qDebug().noquote() << QStringLiteral("Error: MPC从自由度无效或重复: ") << directionText;
                return false;
            }
            directions.push_back(direction);
        }

        std::shared_ptr<NonlinearMPCConstraint> mpc;
        if (directions.size() == 1 && directions.front() < 3)
        {
            auto distance = std::make_shared<DistanceMPCConstraint>();
            distance->m_pNodeA = slave;
            distance->m_pNodeB = master;
            distance->m_pSlaveNode = slave;
            distance->m_SlaveDirection = directions.front();
            distance->m_Length =
                Eigen::Vector3d(slave->m_X - master->m_X, slave->m_Y - master->m_Y, slave->m_Z - master->m_Z).norm();
            mpc = std::move(distance);
        }
        else if (directions == std::vector<int>({0, 5}))
        {
            auto release = std::make_shared<PlanarShearReleaseMPCConstraint>();
            release->m_pMasterNode = master;
            release->m_pSlaveNode = slave;
            mpc = std::move(release);
        }
        else if (directions == std::vector<int>({0, 1, 2}))
        {
            auto rigid = std::make_shared<RigidOffsetMPCConstraint>();
            rigid->m_pMasterNode = master;
            rigid->m_pSlaveNode = slave;
            rigid->m_Offset =
                Eigen::Vector3d(slave->m_X - master->m_X, slave->m_Y - master->m_Y, slave->m_Z - master->m_Z);
            mpc = std::move(rigid);
        }
        else
        {
            m_LastError =
                QStringLiteral("当前MPC求解阶段支持单个平移从自由度或完整平移自由度012: %1").arg(directionText);
            qDebug().noquote() << QStringLiteral("Error: 当前MPC求解阶段支持单个平移从自由度"
                                                 "或完整平移自由度012: ")
                               << directionText;
            return false;
        }

        mpc->m_Id = id;
        mpc->m_SlaveDirections = directions;
        m_Structure->m_MPCConstraints.emplace(id, std::move(mpc));
    }
    return true;
}

bool Input_Model::InputConstraintTabular(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 3, "CONSTRAINT_TABULAR", m_LastError))
    {
        return false;
    }

    bool constraintIdOk = false;
    bool pointCountOk = false;
    const int constraintId = list_str[1].toInt(&constraintIdOk);
    const int pointCount = list_str[2].toInt(&pointCountOk);
    if (!constraintIdOk || !pointCountOk || constraintId <= 0 || pointCount < 2)
    {
        m_LastError = QStringLiteral("CONSTRAINT_TABULAR需要有效约束编号，且数据点不少于2个");
        qDebug().noquote() << QStringLiteral("Error: CONSTRAINT_TABULAR需要有效约束编号，且数据点不少于2个");
        return false;
    }

    const auto constraintIt = m_Structure->m_Constraint.find(constraintId);
    if (constraintIt == m_Structure->m_Constraint.end() || !constraintIt->second)
    {
        m_LastError = QStringLiteral("CONSTRAINT_TABULAR引用了不存在的约束: %1").arg(constraintId);
        qDebug().noquote() << QStringLiteral("Error: CONSTRAINT_TABULAR引用了不存在的约束: ") << constraintId;
        return false;
    }

    const std::shared_ptr<Constraint>& constraint = constraintIt->second;
    if (constraint->HasTimePoints())
    {
        m_LastError = QStringLiteral("约束已绑定TABULAR时程，不能重复绑定: %1").arg(constraintId);
        qDebug().noquote() << QStringLiteral("Error: 约束已绑定TABULAR时程，不能重复绑定: ") << constraintId;
        return false;
    }

    std::vector<ConstraintTimePoint> points;
    for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
    {
        QString pointLine;
        if (!ReadLine(flow, pointLine) || pointLine.startsWith("*"))
        {
            m_LastError = QStringLiteral("CONSTRAINT_TABULAR时程数据不足，约束编号: %1").arg(constraintId);
            qDebug().noquote() << QStringLiteral("Error: CONSTRAINT_TABULAR时程数据不足，约束编号: ") << constraintId;
            return false;
        }

        const QStringList pointFields = pointLine.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (pointFields.size() != 2)
        {
            m_LastError = QStringLiteral("CONSTRAINT_TABULAR数据行需要“时间, 系数”: %1").arg(pointLine);
            qDebug().noquote() << QStringLiteral("Error: CONSTRAINT_TABULAR数据行需要“时间, 系数”: ") << pointLine;
            return false;
        }

        bool timeOk = false;
        bool scaleOk = false;
        const double time = pointFields[0].toDouble(&timeOk);
        const double scale = pointFields[1].toDouble(&scaleOk);
        if (!timeOk || !scaleOk || !std::isfinite(time) || !std::isfinite(scale))
        {
            m_LastError = QStringLiteral("CONSTRAINT_TABULAR包含无效数值: %1").arg(pointLine);
            qDebug().noquote() << QStringLiteral("Error: CONSTRAINT_TABULAR包含无效数值: ") << pointLine;
            return false;
        }
        points.push_back({time, scale});
    }

    try
    {
        constraint->SetTimePoints(std::move(points));
    }
    catch (const std::invalid_argument& error)
    {
        m_LastError = QStringLiteral("CONSTRAINT_TABULAR时程无效: %1").arg(QString::fromUtf8(error.what()));
        qDebug().noquote() << QStringLiteral("Error: CONSTRAINT_TABULAR时程无效: ") << error.what();
        return false;
    }

    return true;
}

bool Input_Model::InputElement_Stress(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "STRESS", m_LastError))
        return false;
    int nStress = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nStress; i++)
    {
        if (!ReadLine(flow, strdata))
        { //没有读取到有效数据，退出
            m_LastError = QStringLiteral("单元应力数据不够");
            qDebug() << QStringLiteral("Error: 单元应力数据不够");
            return false;
        }

        QStringList strlist_con = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts); //利用空格,分解字符串
        if (strlist_con.size() != 2)
        {
            m_LastError = QStringLiteral("单元应力数据格式错误，需要2个字段: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 单元应力数据格式错误，需要2个字段: ") << strdata;
            return false;
        }

        int idElement = strlist_con[0].toInt();
        double stress = 0.0;
        if (!ParseFiniteDouble(strlist_con[1], stress))
        {
            m_LastError = QStringLiteral("初始应力必须为有限数值：%1").arg(strdata);
            return false;
        }

        auto pElement = m_Structure->FindElement(idElement);
        if (!pElement)
        {
            m_LastError = QStringLiteral("单元应力引用了不存在的单元: %1").arg(idElement);
            qDebug().noquote() << QStringLiteral("Error: 单元应力引用了不存在的单元: ") << idElement;
            return false;
        }
        pElement->m_InitStress = stress;
    }

    return true;
}

bool Input_Model::InputAnalysisStep(QTextStream& flow, const QStringList& list_str)
{
    // *ANALYSIS_STEP, N
    if (!RequireKeywordFieldCount(list_str, 2, "ANALYSIS_STEP", m_LastError))
        return false;
    int nStep = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nStep; ++i)
    {
        if (!ReadLine(flow, strdata))
        {
            m_LastError = QStringLiteral("分析步数据不够");
            qDebug().noquote() << QStringLiteral("Error: 分析步数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            m_LastError = QStringLiteral("分析步数据不足，遇到下一个关键字: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 分析步数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_step = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, Type, Time, StepSize, Tolerance, MaxIterations
        if (strlist_step.size() != 6)
        {
            m_LastError = QStringLiteral("分析步数据格式错误，需要6个字段: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 分析步数据格式错误，需要6个字段: ") << strdata;
            return false;
        }

        QString typeStr = strlist_step[1].toUpper();
        double time = 0.0, stepSize = 0.0, tolerance = 0.0;
        int maxIterations = 0;
        if (!ParseFiniteDouble(strlist_step[2], time) || !ParseFiniteDouble(strlist_step[3], stepSize) ||
            !ParseFiniteDouble(strlist_step[4], tolerance) || !ParsePositiveId(strlist_step[5], maxIterations) ||
            tolerance <= 0.0)
        {
            m_LastError = QStringLiteral("分析步参数必须为有限数值，容差和最大迭代数必须大于零：%1").arg(strdata);
            return false;
        }

        AnalysisStepConfig config;
        config.type = EnumKeyword::MapStepType.value(typeStr, EnumKeyword::StepType::UNKNOWN);
        if (config.type == EnumKeyword::StepType::UNKNOWN)
        {
            m_LastError = QStringLiteral("未知的分析步类型: %1").arg(typeStr);
            qDebug().noquote() << QStringLiteral("Error: 未知的分析步类型: ") << typeStr;
            return false;
        }
        if (time < 0.0 || stepSize <= 0.0)
        {
            m_LastError = QStringLiteral("分析步时间参数无效: %1").arg(strdata);
            qDebug().noquote() << QStringLiteral("Error: 分析步时间参数无效: ") << strdata;
            return false;
        }
        config.totalTime = time;
        config.stepSize = stepSize;
        config.tolerance = tolerance;
        config.maxIterations = maxIterations;

        m_Structure->AddAnalysisStep(config);
    }

    return true;
}
