#include "Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
#include <QRegularExpression>
#include <QElapsedTimer>

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
    m_Structure = pStructure;
    QFile file(FileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Error: 文件 " << FileName << " 不存在";
        return false;
    }

    QTextStream flow(&file);
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
            InputNodes(flow, list_str);
            break;
        case EnumKeyword::KeyData::ELEMENT:
            InputElement(flow, list_str);
            break;
        case EnumKeyword::KeyData::SECTION:
            InputSection(flow, list_str);
            break;
        case EnumKeyword::KeyData::MATERIAL:
            InputMaterial(flow, list_str);
            break;
        case EnumKeyword::KeyData::CONSTRAINT:
            InputConstraint(flow, list_str);
            break;
        case EnumKeyword::KeyData::LOAD:
            InputLoad(flow, list_str);
            break;
        case EnumKeyword::KeyData::ANALYSIS_STEP:
            InputAnalysisStep(flow, list_str);
            break;

        default:
            break;
        }
    }
    file.close();
    
    // 合并重复节点、删除重复单元、删除孤立节点、重新编号
    QElapsedTimer timer;
    timer.start();
    m_Structure->CleanupModel();
    qint64 elapsedMs = timer.elapsed();
    qDebug().noquote() << QStringLiteral("模型整理耗时: ") << elapsedMs << QStringLiteral(" 毫秒");

    // 输出
    if (0 != m_Structure->m_Nodes.size())
    qDebug().noquote() << QStringLiteral("\n节点数量: ") << m_Structure->m_Nodes.size();
    
    // 按类型统计单元数量
    QMap<QString, int> elementTypeCount;
    for (const auto& pair : m_Structure->m_Elements)
    {
        QString typeName = "UNKNOWN";
        if (dynamic_cast<ElementTruss*>(pair.second.get())) typeName = "T3D2";
        else if (dynamic_cast<ElementCable*>(pair.second.get())) typeName = "CABLE";
        else if (dynamic_cast<ElementBeam*>(pair.second.get())) typeName = "B31";
        elementTypeCount[typeName]++;
    }
    if (0 != m_Structure->m_Elements.size())
    qDebug().noquote() << QStringLiteral("\n单元总数: ") << m_Structure->m_Elements.size();

    for (auto it = elementTypeCount.constBegin(); it != elementTypeCount.constEnd(); ++it)
    {
        qDebug().noquote() << it.key() << QStringLiteral(": ") << it.value();
    }
    
    if (0 != m_Structure->m_Material.size())
    qDebug().noquote() << QStringLiteral("\n材料数量: ") << m_Structure->m_Material.size();
    if (0 != m_Structure->m_Section.size())
    qDebug().noquote() << QStringLiteral("\n截面数量: ") << m_Structure->m_Section.size();
    if (0 != m_Structure->m_Constraint.size())
    qDebug().noquote() << QStringLiteral("\n约束数量: ") << m_Structure->m_Constraint.size();
    
    // 按类型统计荷载数量
    QMap<QString, int> loadTypeCount;
    for (const auto& pair : m_Structure->m_Load)
    {
        QString typeName = "UNKNOWN";
        if (dynamic_cast<Force_Node*>(pair.second.get())) typeName = "FORCE_NODE";
        // 可添加其他荷载类型
        loadTypeCount[typeName]++;
    }
    if (0 != m_Structure->m_Load.size())
    qDebug().noquote() << QStringLiteral("\n荷载总数: ") << m_Structure->m_Load.size();
    for (auto it = loadTypeCount.constBegin(); it != loadTypeCount.constEnd(); ++it)
    {
        qDebug().noquote()  << it.key() << QStringLiteral(": ") << it.value();
    }
    if (0 != m_Structure->m_AnalysisStep.size())
    qDebug().noquote() << QStringLiteral("\n分析步数量: ") << m_Structure->m_AnalysisStep.size();

    return true;
}


bool Input_Model::InputNodes(QTextStream& flow, const QStringList& list_str)
{
    Q_ASSERT(list_str.size() == 2);

    int nNode = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nNode; i++)
    {
        // 读取下一行有效数据
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: 节点数据不足");
            exit(1);
        }
        
        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 节点数据不足，遇到下一个关键字: ") << strdata;
            exit(1);
        }

        QStringList strlist_node = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        if (strlist_node.size() != 4)
        {
            qDebug().noquote() << QStringLiteral("Error: 节点数据格式错误，需要4个字段: ") << strdata;
            exit(1);
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
    Q_ASSERT(list_str.size() == 3);
    
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
    { EnumKeyword::ElementType::B31,  [](Input_Model* self, QTextStream& flow, const QStringList& list_str, int nElement) { return self->InputElementBeam(flow, list_str, nElement); } },
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
        pElement_Truss->m_pNode[0] = m_Structure->FindNode(idNode0);
        pElement_Truss->m_pNode[1] = m_Structure->FindNode(idNode1);
        auto Property = m_Structure->Create_Property(idMaterial, idSection);
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
        pElement_Truss->m_pNode[0] = m_Structure->FindNode(idNode0);
        pElement_Truss->m_pNode[1] = m_Structure->FindNode(idNode1);
        auto Property = m_Structure->Create_Property(idMaterial, idSection);
        pElement_Truss->m_pProperty = Property;

        m_Structure->m_Elements.insert(std::make_pair(idElement, pElement_Truss));
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
    Q_ASSERT(list_str.size() == 2);
    int nSection = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nSection; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug() << QStringLiteral("Error: 截面数据不够");
            exit(1);
        }

        QStringList strlist_pro = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);

        if (strlist_pro.size() == 2)  //圆截面
        {
            int autoId = static_cast<int>(m_Structure->m_Section.size()) + 1;
            
            auto pSection = std::make_shared<SectionCircular>();
            pSection->m_Id = autoId;
            pSection->m_Radius = strlist_pro[1].toDouble();
            pSection->Calculate_Area();
            m_Structure->m_Section.insert(std::make_pair(autoId, pSection));
        }

    }

    return true;
}

bool Input_Model::InputMaterial(QTextStream& flow, const QStringList& list_str)
{
    //读取到材料
    Q_ASSERT(list_str.size() == 2);
    int nMaterial = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nMaterial; i++)
    {
        //继续读一行有效数据
        if (!ReadLine(flow, strdata))
        {//没有读取到有效数据，退出
            qDebug() << QStringLiteral("Error: 材料数据不够");
            exit(1);
        }

        QStringList strlist_mat = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);//利用空格,分解字符串
        Q_ASSERT(strlist_mat.size() == 6);

        double  E = strlist_mat[1].toDouble();
        double  v = strlist_mat[2].toDouble();
        double  p = strlist_mat[3].toDouble();//密度
        double  S = strlist_mat[4].toDouble();
        double  e = strlist_mat[5].toDouble();

        int autoId = static_cast<int>(m_Structure->m_Material.size()) + 1;

        //保存到模型数据库
        auto pMaterial = std::make_shared<Material>();
        pMaterial->m_Id = autoId;
        pMaterial->m_Young = E;
        pMaterial->m_Poisson = v;
        pMaterial->m_Density = p;
        pMaterial->m_MaxStress = S;
        pMaterial->m_Expansion = e;

        m_Structure->m_Material.insert(std::make_pair(autoId, pMaterial));
    }

    return true;
}

bool Input_Model::InputLoad(QTextStream& flow, const QStringList& list_str)
{
    // *LOAD, TYPE_NAME, N
    Q_ASSERT(list_str.size() == 3);
    
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
        // ID, NodeID, Direction, Value
        if (strlist_load.size() != 4)
        {
            qDebug().noquote() << QStringLiteral("Error: 节点力荷载数据格式错误: ") << strdata;
            return false;
        }

        int idNode = strlist_load[1].toInt();
        int direction = strlist_load[2].toInt();
        double value = strlist_load[3].toDouble();

        int autoId = static_cast<int>(m_Structure->m_Load.size()) + 1;

        auto pLoad = std::make_shared<Force_Node>();
        pLoad->m_Id = autoId;
        pLoad->m_pNode = m_Structure->FindNode(idNode);
        pLoad->m_Direction = static_cast<EnumKeyword::Direction>(direction);
        pLoad->m_Value = value;
        m_Structure->m_Load.insert(std::make_pair(autoId, pLoad));
    }
    return true;
}

// 单元压力荷载处理
bool Input_Model::InputForceElement(QTextStream& flow, const QStringList& /*list_str*/, int nLoad)
{
    // TODO: 实现单元压力荷载的读取逻辑
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug() << QStringLiteral("Error: 单元压力荷载数据不够");
            return false;
        }
        
        qDebug() << QStringLiteral("读取单元压力: ") << strdata;
    }
    return true;
}

bool Input_Model::InputConstraint(QTextStream& flow, const QStringList& list_str)
{
    Q_ASSERT(list_str.size() == 2);
    int nConstraint = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nConstraint; i++)
    {
        if (!ReadLine(flow, strdata))
        {//没有读取到有效数据，退出
            qDebug() << QStringLiteral("Error: 约束数据不够");
            exit(1);
        }

        QStringList strlist_con = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);//利用空格,分解字符串
        Q_ASSERT(strlist_con.size() == 4);

        int idNode = strlist_con[1].toInt();
        auto  direaction = strlist_con[2].toInt();
        double  value = strlist_con[3].toDouble();

        int autoId = static_cast<int>(m_Structure->m_Constraint.size()) + 1;

        auto pConstraint = std::make_shared<Constraint>();
        pConstraint->m_Id = autoId;
        pConstraint->m_pNode = m_Structure->FindNode(idNode);
        pConstraint->m_Direction = static_cast<EnumKeyword::Direction>(direaction);
        pConstraint->m_Value = value;
        m_Structure->m_Constraint.insert(std::make_pair(autoId, pConstraint));
    }

    return true;
}

bool Input_Model::InputAnalysisStep(QTextStream& flow, const QStringList& list_str)
{
    // *ANALYSIS_STEP, N
    Q_ASSERT(list_str.size() == 2);
    int nStep = list_str[1].toInt();

    QString strdata;
    for (int i = 0; i < nStep; ++i)
    {
        if (!ReadLine(flow, strdata))
        {
            qDebug().noquote() << QStringLiteral("Error: 分析步数据不够");
            exit(1);
        }
        
        // 检查是否误读到下一个关键字行
        if (strdata.startsWith("*"))
        {
            qDebug().noquote() << QStringLiteral("Error: 分析步数据不足，遇到下一个关键字: ") << strdata;
            exit(1);
        }

        QStringList strlist_step = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        // ID, Type, Time, StepSize, Tolerance, MaxIterations 
        if (strlist_step.size() != 6)
        {
            qDebug().noquote() << QStringLiteral("Error: 分析步数据格式错误，需要6个字段: ") << strdata;
            exit(1);
        }

        QString typeStr       = strlist_step[1].toUpper();
        double  time          = strlist_step[2].toDouble();
        double  stepSize      = strlist_step[3].toDouble();
        double  tolerance     = strlist_step[4].toDouble();
        int     maxIterations = strlist_step[5].toInt();

        int autoId = static_cast<int>(m_Structure->m_AnalysisStep.size()) + 1;

        auto pStep = std::make_shared<AnalysisStep>();
        pStep->m_Id = autoId;
        pStep->m_Type = EnumKeyword::MapStepType.value(typeStr, EnumKeyword::StepType::UNKNOWN);
        pStep->m_Time = time;
        pStep->m_StepSize = stepSize;
        pStep->m_Tolerance = tolerance;
        pStep->m_MaxIterations = maxIterations;

        m_Structure->m_AnalysisStep.insert(std::make_pair(autoId, pStep));
    }

    return true;
}
