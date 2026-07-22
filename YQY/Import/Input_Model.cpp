#include "Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
#include "Utility/Logger/Logger.h"
#include <QRegularExpression>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QDir>
#include <QStringDecoder>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace
{
QString GetDefaultHdf5FileName(const QString& inputFileName)
{
    const QFileInfo inputFileInfo(inputFileName);
    QDir outputDir(QDir::current().filePath("Export/ExportH5"));//输出H5文件的默认路径
    if (!outputDir.exists())
    {
        QDir().mkpath(outputDir.absolutePath());
    }

    return outputDir.filePath(inputFileInfo.completeBaseName() + ".h5");
}

QStringConverter::Encoding DetectTextEncoding(QFile& file)
{
    const QByteArray bytes = file.peek(file.size());
    QStringDecoder utf8Decoder(QStringConverter::Utf8);
    utf8Decoder.decode(bytes);
    return utf8Decoder.hasError() ? QStringConverter::System : QStringConverter::Utf8;
}

bool RequireKeywordFieldCount(const QStringList& fields, int expected, const QString& keyword)
{
    if (fields.size() == expected)
    {
        return true;
    }

    qDebug().noquote() << QStringLiteral("Error: %1 关键字字段数量错误，期望 %2，实际 %3")
        .arg(keyword)
        .arg(expected)
        .arg(fields.size());
    return false;
}
}

bool Input_Model::ReadLine(QTextStream& flow, QString& str)
{
    while (!flow.atEnd())
    {
        str = flow.readLine();
        if (str.left(2).compare("**") == 0) continue;//注释行，继续读取下一行
        if (str.left(2).compare("//") == 0) continue;

        str = str.trimmed();                         //去除开头和末尾的空格
        if (str.isEmpty()) continue;                 //注释行，继续读取下一行

        return true;                                 //读取到有效数据，完成一行读取
    }

    return false;                                    //文件读完，没有得到有效的数据行
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

    m_Structure = pStructure;
    m_Structure->m_OutputControl.m_SourceModelName = FileName;

    QFileInfo inputFileInfo(FileName);
    m_InputFileDir = inputFileInfo.absolutePath();
    m_Structure->m_OutputControl.m_Hdf5FileName = GetDefaultHdf5FileName(FileName);

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
        if (!str.startsWith("*")) continue; // 如果不是以*开头，跳过（或者根据需求处理非关键字行）

        QString cleanStr = str.mid(1); // 去掉开头的*
        QStringList list_str = cleanStr.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (list_str.isEmpty()) continue;

        QString keyword = list_str[0].trimmed();
        // 使用 Map 进行映射，统一转大写
        auto key = EnumKeyword::MapKeyData.value(keyword.toUpper(), EnumKeyword::KeyData::UNKNOWN);

        switch (key)
        {
        case EnumKeyword::KeyData::NODE:
            if (!InputNodes(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::ELEMENT:
            if (!InputElement(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::SECTION:
            if (!InputSection(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::MATERIAL:
            if (!InputMaterial(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::CONSTRAINT:
            if (!InputConstraint(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::CONSTRAINT_TABULAR:
            if (!InputConstraintTabular(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::LOAD:
            if (!InputLoad(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::STRESS:
            if (!InputElement_Stress(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::ANALYSIS_STEP:
            if (!InputAnalysisStep(flow, list_str)) return false;
            break;
        case EnumKeyword::KeyData::OUTPUT:
            if (!InputOutput(flow, list_str)) return false;
            break;

        default:
            break;
        }
    }
    file.close();

    // 合并重复节点、删除重复单元、删除孤立节点、重新编号
    m_Structure->CleanupModel();

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
        if (dynamic_cast<ElementTruss*>(pair.second.get())) typeName = "T3D2";
        else if (dynamic_cast<ElementCable*>(pair.second.get())) typeName = "CABLE";
        else if (dynamic_cast<ElementBeam_CR*>(pair.second.get())) typeName = "CR3D";
        elementTypeCount[typeName]++;
    }

    // 按类型统计荷载数量
    QMap<QString, int> loadTypeCount;
    for (const auto& pair : m_Structure->m_Load)
    {
        QString typeName = "UNKNOWN";
        if (dynamic_cast<Force_Node*>(pair.second.get()))         typeName = "FORCE_NODE";
        else if (dynamic_cast<Force_Element*>(pair.second.get())) typeName = "FORCE_ELEMENT";
        else if (dynamic_cast<Force_Gravity*>(pair.second.get())) typeName = "FORCE_GRAVITY";
        else if (dynamic_cast<Force_Wind*>(pair.second.get()))    typeName = "FORCE_WIND";
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
    summary << QStringLiteral("荷载总数: %1").arg(m_Structure->m_Load.size());
    for (auto it = loadTypeCount.constBegin(); it != loadTypeCount.constEnd(); ++it)
    {
        summary << QStringLiteral("%1: %2").arg(it.key()).arg(it.value());
        if (it.key() == "FORCE_GRAVITY")
        {
            summary << QStringLiteral("重力方向: %1").arg(g_Direction);
        }
    }
    summary << QStringLiteral("分析步数量: %1").arg(m_Structure->m_AnalysisStep.size());

    Logger::Instance().InfoToFile(summary.join(QStringLiteral("\n")));

    return true;
}

bool Input_Model::ValidateStructure(QString& errorMessage) const
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
	if (m_Structure->m_Material.empty())
	{
		errorMessage = QStringLiteral("模型缺少材料属性。");
		return false;
	}
	if (m_Structure->m_Section.empty())
	{
		errorMessage = QStringLiteral("模型缺少截面属性。");
		return false;
	}
	if (m_Structure->m_Property.empty())
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
		const auto property = element->m_pProperty.lock();
		const auto propertyIt = property ? m_Structure->m_Property.find(property->m_Id) : m_Structure->m_Property.end();
		if (!property || propertyIt == m_Structure->m_Property.end() || propertyIt->second.get() != property.get())
		{
			errorMessage = QStringLiteral("单元 %1 没有关联有效的材料-截面属性。").arg(elementId);
			return false;
		}
	}

	errorMessage.clear();
	return true;
}


bool Input_Model::InputNodes(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "NODE")) return false;

    int nNode = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nNode; i++)
    {
        // 读取下一行有效数据
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: 节点数据不足");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 节点数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_node = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (strlist_node.size() != 4)
        {
            qDebug().noquote() << QStringLiteral("Error: 节点数据格式错误，需要4个字段: ") << strdata;
            return false;
        }

        double xi = strlist_node[1].toDouble();
        double yi = strlist_node[2].toDouble();
        double zi = strlist_node[3].toDouble();

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
    if (!RequireKeywordFieldCount(list_str, 3, "ELEMENT")) return false;

    QString typeStr = list_str[1].trimmed().toUpper();
    EnumKeyword::ElementType elementType = EnumKeyword::MapElementType.value(typeStr, EnumKeyword::ElementType::UNKNOWN);

    int nElement = list_str[2].toInt();

    // 记录读取前的单元数量
    int countBefore = static_cast<int>(m_Structure->m_Elements.size());

    // 查找并调用对应的处理函数
    auto handler = s_ElementHandlers.value(elementType, nullptr);
    if (handler)
    {
        bool result = handler(this, flow, list_str, nElement);

        return result;
    }
    else
    {
        qDebug().noquote() << QStringLiteral("Error: 未知的单元类型: ") << typeStr;
        return false;
    }
}

// 静态单元处理函数映射表初始化
const QMap<EnumKeyword::ElementType, Input_Model::ElementHandler> Input_Model::s_ElementHandlers =
{
    { EnumKeyword::ElementType::T3D2, [](Input_Model* self, QTextStream& flow, const QStringList& list_str, int nElement) { return self->InputElementTruss(flow, list_str, nElement); } },
    { EnumKeyword::ElementType::CABLE, [](Input_Model* self, QTextStream& flow, const QStringList& list_str, int nElement) { return self->InputElementCable(flow, list_str, nElement); } },
    { EnumKeyword::ElementType::CR3D,  [](Input_Model* self, QTextStream& flow, const QStringList& list_str, int nElement) { return self->InputElementBeam_CR3D(flow, list_str, nElement); } },
};

// 桁架单元处理
bool Input_Model::InputElementTruss(QTextStream& flow, const QStringList& /*list_str*/, int nElement)
{
    QString strdata;
    for (int i = 0; i < nElement; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: 桁架单元数据不够");
            return false;
        }

        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 桁架单元数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_ele = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, Node1, Node2, Material, Section (5字段)
        if (strlist_ele.size() != 5)
        {
            qDebug().noquote() << QStringLiteral("Error: 桁架单元数据格式错误: ") << strdata;
            return false;
        }

        int idElement = static_cast<int>(m_Structure->m_Elements.size()) + 1;
        int idNode0 = strlist_ele[1].toInt();
        int idNode1 = strlist_ele[2].toInt();
        int idMaterial = strlist_ele[3].toInt();
        int idSection = strlist_ele[4].toInt();

        auto pElement_Truss = std::make_shared<ElementTruss>();
        pElement_Truss->m_Id = idElement;
        auto node0 = m_Structure->FindNode(idNode0);
        auto node1 = m_Structure->FindNode(idNode1);
        if (!node0 || !node1)
        {
            qDebug().noquote() << QStringLiteral("Error: 桁架单元引用了不存在的节点: ") << idNode0 << idNode1;
            return false;
        }
        pElement_Truss->m_pNode[0] = node0;
        node0->SetNumDOFs(pElement_Truss->Get_NodeDOF());
        pElement_Truss->m_pNode[1] = node1;
        node1->SetNumDOFs(pElement_Truss->Get_NodeDOF());
        auto Property = m_Structure->Create_Property(idMaterial, idSection);
        if (!Property)
        {
            qDebug().noquote() << QStringLiteral("Error: 桁架单元属性创建失败，材料/截面 id=") << idMaterial << idSection;
            return false;
        }
        pElement_Truss->m_pProperty = Property;

        m_Structure->m_Elements.insert(std::make_pair(idElement, pElement_Truss));
    }
    return true;
}

bool Input_Model::InputElementCable(QTextStream& flow, const QStringList& list_str, int nElement)
{
    QString strdata;
    for (int i = 0; i < nElement; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: 索单元数据不够");
            return false;
        }

        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 索单元数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_ele = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, Node1, Node2, Material, Section
        if (strlist_ele.size() != 5)
        {
            qDebug().noquote() << QStringLiteral("Error: 索单元数据格式错误: ") << strdata;
            return false;
        }

        int idElement = static_cast<int>(m_Structure->m_Elements.size()) + 1;
        int idNode0 = strlist_ele[1].toInt();
        int idNode1 = strlist_ele[2].toInt();
        int idMaterial = strlist_ele[3].toInt();
        int idSection = strlist_ele[4].toInt();

        auto pElement_Truss = std::make_shared<ElementCable>();
        pElement_Truss->m_Id = idElement;
        auto node0 = m_Structure->FindNode(idNode0);
        auto node1 = m_Structure->FindNode(idNode1);
        if (!node0 || !node1)
        {
            qDebug().noquote() << QStringLiteral("Error: 索单元引用了不存在的节点: ") << idNode0 << idNode1;
            return false;
        }
        pElement_Truss->m_pNode[0] = node0;
        node0->SetNumDOFs(pElement_Truss->Get_NodeDOF());
        pElement_Truss->m_pNode[1] = node1;
        node1->SetNumDOFs(pElement_Truss->Get_NodeDOF());
        auto Property = m_Structure->Create_Property(idMaterial, idSection);
        if (!Property)
        {
            qDebug().noquote() << QStringLiteral("Error: 索单元属性创建失败，材料/截面 id=") << idMaterial << idSection;
            return false;
        }
        pElement_Truss->m_pProperty = Property;

        m_Structure->m_Elements.insert(std::make_pair(idElement, pElement_Truss));
    }
    return true;
}

bool Input_Model::InputElementBeam_CR3D(QTextStream& flow, const QStringList& list_str, int nElement)
{
    QString strdata;
    for (int i = 0; i < nElement; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: CR3D单元数据不够");
            return false;
        }

        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: CR3D单元数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_ele = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, Node1, Node2, Material, Section ，1， 0， 0（方向向量）(8字段)
        if (strlist_ele.size() != 8)
        {
            qDebug().noquote() << QStringLiteral("Error: 梁单元数据格式错误: ") << strdata;
            return false;
        }

        int idElement = static_cast<int>(m_Structure->m_Elements.size()) + 1;
        int idNode0 = strlist_ele[1].toInt();
        int idNode1 = strlist_ele[2].toInt();
        int idMaterial = strlist_ele[3].toInt();
        int idSection = strlist_ele[4].toInt();

        auto pElement = std::make_shared<ElementBeam_CR>();
        pElement->m_Id = idElement;
        auto node0 = m_Structure->FindNode(idNode0);
        auto node1 = m_Structure->FindNode(idNode1);
        if (!node0 || !node1)
        {
            qDebug().noquote() << QStringLiteral("Error: CR3D 单元引用了不存在的节点: ") << idNode0 << idNode1;
            return false;
        }
        pElement->m_pNode[0] = node0;
        node0->SetNumDOFs(pElement->Get_NodeDOF());
        pElement->m_pNode[1] = node1;
        node1->SetNumDOFs(pElement->Get_NodeDOF());
        auto Property = m_Structure->Create_Property(idMaterial, idSection);
        if (!Property)
        {
            qDebug().noquote() << QStringLiteral("Error: CR3D 单元属性创建失败，材料/截面 id=") << idMaterial << idSection;
            return false;
        }
        pElement->m_pProperty = Property;
        pElement->q0 = Vector3d(strlist_ele[5].toDouble(), strlist_ele[6].toDouble(), strlist_ele[7].toDouble());
        m_Structure->m_Elements.insert(std::make_pair(idElement, pElement));
    }
    return true;
}

// 梁单元处理（待实现）
bool Input_Model::InputElementBeam(QTextStream& flow, const QStringList& /*list_str*/, int nElement)
{
    // TODO: 实现梁单元的读取逻辑
    QString strdata;
    for (int i = 0; i < nElement; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: 梁单元数据不够");
            return false;
        }

        qDebug().noquote() << QStringLiteral("读取梁单元: ") << strdata;
    }
    return true;
}

bool Input_Model::InputSection(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "SECTION")) return false;
    int nSection = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nSection; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug() << QStringLiteral("Error: 截面数据不够");
            return false;
        }

        QStringList strlist_pro = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);

        if (strlist_pro.size() == 2)  //圆截面
        {
            int autoId = static_cast<int>(m_Structure->m_Section.size()) + 1;

            auto pSection = std::make_shared<SectionCircular>();
            pSection->m_Id = autoId;
            pSection->m_Area = strlist_pro[1].toDouble();
            pSection->Calculate_Radius();
            m_Structure->m_Section.insert(std::make_pair(autoId, pSection));
        }
        else if (strlist_pro.size() == 3)  //矩形截面
        {
            int autoId = static_cast<int>(m_Structure->m_Section.size()) + 1;

            auto pSectI0n = std::make_shared<SectionRectangle>();
            pSectI0n->m_Id = autoId;
            pSectI0n->m_Width = strlist_pro[1].toDouble();
            pSectI0n->m_Height = strlist_pro[2].toDouble();
            m_Structure->m_Section.insert(std::make_pair(autoId, pSectI0n));
        }
    }

    return true;
}

bool Input_Model::InputMaterial(QTextStream& flow, const QStringList& list_str)
{
    //读取到材料
    if (!RequireKeywordFieldCount(list_str, 2, "MATERIAL")) return false;
    int nMaterial = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nMaterial; i++)
    {
        //继续读一行有效数据
        if (!ReadLine(flow, strdata))
        {//没有读取到有效数据，退出
            qDebug() << QStringLiteral("Error: 材料数据不够");
            return false;
        }

        QStringList strlist_mat = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);//利用空格,分解字符串
        if (strlist_mat.size() != 6)
        {
            qDebug().noquote()
                << QStringLiteral("Error: 材料数据格式错误，需要6个字段: ")
                << strdata;
            return false;
        }

        const double young = strlist_mat[1].toDouble();
        const double poisson = strlist_mat[2].toDouble();
        const double density = strlist_mat[3].toDouble();
        const double maxStress = strlist_mat[4].toDouble();
        const double expansion = strlist_mat[5].toDouble();

        if (young <= 0.0)
        {
            qDebug().noquote() << QStringLiteral("Error: 材料弹性模量必须大于0: ") << strdata;
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
    if (!RequireKeywordFieldCount(list_str, 3, "LOAD")) return false;

    QString typeStr = list_str[1].trimmed().toUpper();
    EnumKeyword::LoadType loadType = EnumKeyword::MapLoadType.value(typeStr, EnumKeyword::LoadType::UNKNOWN);

    int nLoad = list_str[2].toInt();

    // 记录读取前的荷载数量
    int countBefore = static_cast<int>(m_Structure->m_Load.size());

    // 查找并调用对应的处理函数
    auto handler = s_LoadHandlers.value(loadType, nullptr);
    if (handler)
    {
        bool result = handler(this, flow, list_str, nLoad);

        return result;
    }
    else
    {
        qDebug().noquote() << QStringLiteral("Error: 未知的荷载类型");
        return false;
    }
}

// 静态荷载处理函数映射表初始化
const QMap<EnumKeyword::LoadType, Input_Model::LoadHandler> Input_Model::s_LoadHandlers =
{
    { EnumKeyword::LoadType::FORCE_NODE,       [](Input_Model* self, QTextStream& flow, const QStringList& list_str, int nLoad) { return self->InputForceNode(flow, list_str, nLoad); } },
    { EnumKeyword::LoadType::FORCE_ELEMENT,    [](Input_Model* self, QTextStream& flow, const QStringList& list_str, int nLoad) { return self->InputForceElement(flow, list_str, nLoad); } },
    { EnumKeyword::LoadType::FORCE_GRAVITY,    [](Input_Model* self, QTextStream& flow, const QStringList& list_str, int nLoad) { return self->InputForceGravity(flow, list_str, nLoad); } },
    { EnumKeyword::LoadType::FORCE_WIND,       [](Input_Model* self, QTextStream& flow, const QStringList& list_str, int nLoad) { return self->InputForceWind(flow, list_str, nLoad); } }
    // 新增荷载类型只需在此添加映射
};

// 节点力荷载处理
bool Input_Model::InputForceNode(QTextStream& flow, const QStringList& /*list_str*/, int nLoad)
{
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: 节点力荷载数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 节点力荷载数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\\t, ]"), Qt::SkipEmptyParts);
        // 基本格式：ID, NodeID, Direction, Value, StepID, StartTime, EndTime
        // 扩展格式：ID, NodeID, Direction, Value, StepID, StartTime, EndTime, FunctionType, Params...
        if (strlist_load.size() < 7)
        {
            qDebug().noquote() << QStringLiteral("Error: 节点力荷载数据格式错误: ") << strdata;
            return false;
        }

        int idNode = strlist_load[1].toInt();
        int direction = strlist_load[2].toInt();
        double value = strlist_load[3].toDouble();
        int stepid = strlist_load[4].toInt();
        double startTime = strlist_load[5].toDouble();
        double endTime = strlist_load[6].toDouble();

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Node>();
        pLoad->m_Id = autoId;
        auto node = m_Structure->FindNode(idNode);
        if (!node)
        {
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
            QString funcType = strlist_load[7].trimmed().toUpper();

            if (funcType == "SIN" && strlist_load.size() >= 12)
            {
                pLoad->m_FunctionType = TimeFunctionType::SIN;
                pLoad->m_Amplitude = strlist_load[8].toDouble();
                pLoad->m_Frequency = strlist_load[9].toDouble();
                pLoad->m_Phase = strlist_load[10].toDouble();
                pLoad->m_Offset = strlist_load[11].toDouble();
                qDebug().noquote() << QStringLiteral("  荷载 %1: 正弦函数 (幅值=%2, 频率=%3 Hz)")
                    .arg(autoId).arg(pLoad->m_Amplitude).arg(pLoad->m_Frequency);
            }
            else if (funcType == "COS" && strlist_load.size() >= 12)
            {
                pLoad->m_FunctionType = TimeFunctionType::COS;
                pLoad->m_Amplitude = strlist_load[8].toDouble();
                pLoad->m_Frequency = strlist_load[9].toDouble();
                pLoad->m_Phase = strlist_load[10].toDouble();
                pLoad->m_Offset = strlist_load[11].toDouble();
                qDebug().noquote() << QStringLiteral("  荷载 %1: 余弦函数 (幅值=%2, 频率=%3 Hz)")
                    .arg(autoId).arg(pLoad->m_Amplitude).arg(pLoad->m_Frequency);
            }
            else if (funcType == "RAMP" && strlist_load.size() >= 10)
            {
                pLoad->m_FunctionType = TimeFunctionType::RAMP;
                pLoad->m_RampT0 = strlist_load[8].toDouble();
                pLoad->m_RampT1 = strlist_load[9].toDouble();
                qDebug().noquote() << QStringLiteral("  荷载 %1: 斜坡函数 (t0=%2, t1=%3)")
                    .arg(autoId).arg(pLoad->m_RampT0).arg(pLoad->m_RampT1);
            }
            else if (funcType == "EXPONENTIAL" && strlist_load.size() >= 9)
            {
                pLoad->m_FunctionType = TimeFunctionType::EXPONENTIAL;
                pLoad->m_Decay = strlist_load[8].toDouble();
                qDebug().noquote() << QStringLiteral("  荷载 %1: 指数衰减函数 (衰减系数=%2)")
                    .arg(autoId).arg(pLoad->m_Decay);
            }
            else if (funcType == "TRIANGULAR" && strlist_load.size() >= 9)
            {
                pLoad->m_FunctionType = TimeFunctionType::TRIANGULAR;
                pLoad->m_Period = strlist_load[8].toDouble();
                qDebug().noquote() << QStringLiteral("  荷载 %1: 三角波函数 (周期=%2)")
                    .arg(autoId).arg(pLoad->m_Period);
            }
            else if (funcType == "SQUARE" && strlist_load.size() >= 10)
            {
                pLoad->m_FunctionType = TimeFunctionType::SQUARE;
                pLoad->m_Period = strlist_load[8].toDouble();
                pLoad->m_DutyCycle = strlist_load[9].toDouble();
                qDebug().noquote() << QStringLiteral("  荷载 %1: 方波函数 (周期=%2, 占空比=%3)")
                    .arg(autoId).arg(pLoad->m_Period).arg(pLoad->m_DutyCycle);
            }
            else if (funcType == "CONSTANT" || funcType.isEmpty())
            {
                pLoad->m_FunctionType = TimeFunctionType::CONSTANT;
                qDebug().noquote() << QStringLiteral("  荷载 %1: 常数荷载").arg(autoId);
            }
            else
            {
                qDebug().noquote() << QStringLiteral("警告: 未知的时间函数类型 '%1'，使用常数荷载").arg(funcType);
                pLoad->m_FunctionType = TimeFunctionType::CONSTANT;
            }
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
            qDebug() << QStringLiteral("Error: 单元压力荷载数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 单元力荷载数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\\t, ]"), Qt::SkipEmptyParts);
        // ID, EelmentID, Direction, Value
        if (strlist_load.size() != 4)
        {
            qDebug().noquote() << QStringLiteral("Error: 单元力荷载数据格式错误: ") << strdata;
            return false;
        }

        int idElement = strlist_load[1].toInt();
        int direction = strlist_load[2].toInt();
        double value = strlist_load[3].toDouble();

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Element>();
        pLoad->m_Id = autoId;
        auto element = m_Structure->FindElement(idElement);
        if (!element)
        {
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

bool Input_Model::InputForceGravity(QTextStream& flow, const QStringList& list_str, int nLoad)
{
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug() << QStringLiteral("Error: 单元重力数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 单元重力数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\\t, ]"), Qt::SkipEmptyParts);
        // ID, Direction, Value, StepID
        if (strlist_load.size() != 4)
        {
            qDebug().noquote() << QStringLiteral("Error: 单元重力数据格式错误: ") << strdata;
            return false;
        }

        g_Direction = strlist_load[1].toInt();
        double value = strlist_load[2].toDouble();
        int stepid = strlist_load[3].toInt();

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Gravity>();
        pLoad->m_Id = autoId;

        pLoad->m_Direction = static_cast<EnumKeyword::Direction>(g_Direction);
        if (0 != value)
        {
            pLoad->m_g = value;
        }
        pLoad->m_StepId = stepid;
        m_Structure->m_Load.insert(std::make_pair(autoId, pLoad));
    }
    return true;
}

bool Input_Model::InputForceWind(QTextStream& flow, const QStringList& list_str, int nLoad)
{
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug() << QStringLiteral("Error: 单元风荷载数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 单元风荷载数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\\t, ]"), Qt::SkipEmptyParts);
        // ID, Direction, Value, StepID
        if (strlist_load.size() != 4)
        {
            qDebug().noquote() << QStringLiteral("Error: 单元风荷载数据格式错误: ") << strdata;
            return false;
        }

        g_Direction = strlist_load[1].toInt();
        double value = strlist_load[2].toDouble();
        int stepid = strlist_load[3].toInt();

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Wind>();
        pLoad->m_Id = autoId;

        pLoad->m_Direction = static_cast<EnumKeyword::Direction>(g_Direction);
        if (0 != value)
        {
            pLoad->m_velocity = value;
        }
        pLoad->m_StepId = stepid;
        m_Structure->m_Load.insert(std::make_pair(autoId, pLoad));
    }
    return true;
}

bool Input_Model::InputConstraint(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "CONSTRAINT")) return false;
    int nConstraint = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nConstraint; i++)
    {
        if (!ReadLine(flow, strdata))
        {//没有读取到有效数据，退出
            qDebug() << QStringLiteral("Error: 约束数据不够");
            return false;
        }

        QStringList strlist_con = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);//利用空格,分解字符串
        if (strlist_con.size() != 4)
        {
            qDebug().noquote()
                << QStringLiteral("Error: 约束数据格式错误，需要4个字段: ")
                << strdata;
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
        if (!idOk || !nodeIdOk || !directionOk || !valueOk
            || constraintId <= 0 || !std::isfinite(value))
        {
            qDebug().noquote()
                << QStringLiteral("Error: 约束包含无效字段: ")
                << strdata;
            return false;
        }

        if (m_Structure->m_Constraint.find(constraintId)
            != m_Structure->m_Constraint.end())
        {
            qDebug().noquote()
                << QStringLiteral("Error: 约束编号重复: ")
                << constraintId;
            return false;
        }

        auto node = m_Structure->FindNode(idNode);
        if (!node)
        {
            qDebug().noquote() << QStringLiteral("Error: 约束引用了不存在的节点: ") << idNode;
            return false;
        }

        auto pConstraint = std::make_shared<Constraint>();
        pConstraint->m_Id = constraintId;
        pConstraint->m_pNode = node;
        pConstraint->m_Direction =
            static_cast<EnumKeyword::Direction>(direction);
        pConstraint->m_Value = value;
        m_Structure->m_Constraint.insert(
            std::make_pair(constraintId, pConstraint));
    }

    return true;
}

bool Input_Model::InputConstraintTabular(
    QTextStream& flow,
    const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(
        list_str,
        3,
        "CONSTRAINT_TABULAR"))
    {
        return false;
    }

    bool constraintIdOk = false;
    bool pointCountOk = false;
    const int constraintId = list_str[1].toInt(&constraintIdOk);
    const int pointCount = list_str[2].toInt(&pointCountOk);
    if (!constraintIdOk || !pointCountOk
        || constraintId <= 0 || pointCount < 2)
    {
        qDebug().noquote()
            << QStringLiteral(
                "Error: CONSTRAINT_TABULAR需要有效约束编号，且数据点不少于2个");
        return false;
    }

    const auto constraintIt =
        m_Structure->m_Constraint.find(constraintId);
    if (constraintIt == m_Structure->m_Constraint.end()
        || !constraintIt->second)
    {
        qDebug().noquote()
            << QStringLiteral(
                "Error: CONSTRAINT_TABULAR引用了不存在的约束: ")
            << constraintId;
        return false;
    }

    const std::shared_ptr<Constraint>& constraint =
        constraintIt->second;
    if (constraint->HasTimePoints())
    {
        qDebug().noquote()
            << QStringLiteral(
                "Error: 约束已绑定TABULAR时程，不能重复绑定: ")
            << constraintId;
        return false;
    }

    std::vector<ConstraintTimePoint> points;
    points.reserve(static_cast<size_t>(pointCount));
    for (int pointIndex = 0; pointIndex < pointCount; ++pointIndex)
    {
        QString pointLine;
        if (!ReadLine(flow, pointLine) || pointLine.startsWith("*"))
        {
            qDebug().noquote()
                << QStringLiteral(
                    "Error: CONSTRAINT_TABULAR时程数据不足，约束编号: ")
                << constraintId;
            return false;
        }

        const QStringList pointFields =
            pointLine.split(
                QRegularExpression("[\t, ]"),
                Qt::SkipEmptyParts);
        if (pointFields.size() != 2)
        {
            qDebug().noquote()
                << QStringLiteral(
                    "Error: CONSTRAINT_TABULAR数据行需要“时间, 系数”: ")
                << pointLine;
            return false;
        }

        bool timeOk = false;
        bool scaleOk = false;
        const double time = pointFields[0].toDouble(&timeOk);
        const double scale = pointFields[1].toDouble(&scaleOk);
        if (!timeOk || !scaleOk
            || !std::isfinite(time) || !std::isfinite(scale))
        {
            qDebug().noquote()
                << QStringLiteral(
                    "Error: CONSTRAINT_TABULAR包含无效数值: ")
                << pointLine;
            return false;
        }
        points.push_back({ time, scale });
    }

    try
    {
        constraint->SetTimePoints(std::move(points));
    }
    catch (const std::invalid_argument& error)
    {
        qDebug().noquote()
            << QStringLiteral(
                "Error: CONSTRAINT_TABULAR时程无效: ")
            << error.what();
        return false;
    }

    return true;
}

bool Input_Model::InputElement_Stress(QTextStream& flow, const QStringList& list_str)
{
    if (!RequireKeywordFieldCount(list_str, 2, "STRESS")) return false;
    int nStress = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nStress; i++)
    {
        if (!ReadLine(flow, strdata))
        {//没有读取到有效数据，退出
            qDebug() << QStringLiteral("Error: 单元应力数据不够");
            return false;
        }

        QStringList strlist_con = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);//利用空格,分解字符串
        if (strlist_con.size() != 2)
        {
            qDebug().noquote() << QStringLiteral("Error: 单元应力数据格式错误，需要2个字段: ") << strdata;
            return false;
        }

        int idElement = strlist_con[0].toInt();
        double stress = strlist_con[1].toDouble();

        auto pElement = m_Structure->FindElement(idElement);
        if (!pElement)
        {
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
    if (!RequireKeywordFieldCount(list_str, 2, "ANALYSIS_STEP")) return false;
    int nStep = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nStep; ++i)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: 分析步数据不够");
            return false;
        }

        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 分析步数据不足，遇到下一个关键字: ") << strdata;
            return false;
        }

        QStringList strlist_step = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, Type, Time, StepSize, Tolerance, MaxIterations 
        if (strlist_step.size() != 6)
        {
            qDebug().noquote() << QStringLiteral("Error: 分析步数据格式错误，需要6个字段: ") << strdata;
            return false;
        }

        QString typeStr = strlist_step[1].toUpper();
        double  time = strlist_step[2].toDouble();
        double  stepSize = strlist_step[3].toDouble();
        double  tolerance = strlist_step[4].toDouble();
        int     maxIterations = strlist_step[5].toInt();

        AnalysisStepConfig config;
        config.type = EnumKeyword::MapStepType.value(typeStr, EnumKeyword::StepType::UNKNOWN);
        if (config.type == EnumKeyword::StepType::UNKNOWN)
        {
            qDebug().noquote() << QStringLiteral("Error: 未知的分析步类型: ") << typeStr;
            return false;
        }
        if (time < 0.0 || stepSize <= 0.0)
        {
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

bool Input_Model::InputOutput(QTextStream& flow, const QStringList& list_str)
{
    Q_UNUSED(flow);

    // 格式：*OUTPUT, H5, 1[, fileName]
    if (list_str.size() < 3)
    {
        qDebug().noquote() << QStringLiteral("Error: 输出控制格式错误，应为 *OUTPUT, H5, 0/1");
        return false;
    }

    const QString outputType = list_str[1].trimmed().toUpper();
    if (outputType != "H5" && outputType != "HDF5")
    {
        qDebug().noquote() << QStringLiteral("警告: 暂不支持的输出类型: ") << outputType;
        return true;
    }

    const int enabled = list_str[2].toInt();
    m_Structure->m_OutputControl.m_EnableHdf5 = (enabled != 0);
    m_Structure->m_OutputControl.m_OutputModel = true;
    m_Structure->m_OutputControl.m_OutputResult = true;
    m_Structure->m_OutputControl.m_StreamResult = true;

    if (list_str.size() >= 4)
    {
        QFileInfo hdf5FileInfo(list_str[3].trimmed());
        m_Structure->m_OutputControl.m_Hdf5FileName = hdf5FileInfo.isRelative()
            ? QDir(QDir::current().filePath("Export/ExportH5")).filePath(list_str[3].trimmed())
            : hdf5FileInfo.absoluteFilePath();
    }

    qDebug().noquote() << QStringLiteral("H5 输出控制: ")
        << (m_Structure->m_OutputControl.m_EnableHdf5 ? QStringLiteral("开启") : QStringLiteral("关闭"))
        << QStringLiteral(", 文件: ") << m_Structure->m_OutputControl.m_Hdf5FileName;

    return true;
}
