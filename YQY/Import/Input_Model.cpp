#include "Input_Model.h"
#include "DataStructure/Structure/StructureData.h"

#include <QRegularExpression>

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

        default:

            if (keyword.compare("ELEMENT_TRUSS3D", Qt::CaseInsensitive) == 0)
            {
                InputElement(flow, list_str);
            }
            else if (keyword.compare("STEP_STATIC", Qt::CaseInsensitive) == 0)
            {
                //InputStep(flow, list_str, "Step_Static");
            }
            break;
        }
    }
    file.close();
    return true;
}


bool Input_Model::InputNodes(QTextStream& flow, const QStringList& list_str)
{
    Q_ASSERT(list_str.size() == 2);

    int nNode = list_str[1].toInt();
    qDebug().noquote() << QStringLiteral("节点数: ") << nNode;

    QString strdata;
    for (int i = 0; i < nNode; i++)
    {
        // 读取下一行有效数据
        if (!ReadLine(flow, strdata))
        {
            qDebug() << QStringLiteral("Error: 节点数据不足");
            exit(1);
        }

        QStringList strlist_node = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);
        Q_ASSERT(strlist_node.size() == 4);

        int idNode = strlist_node[0].toInt();
        double xi = strlist_node[1].toDouble();
        double yi = strlist_node[2].toDouble();
        double zi = strlist_node[3].toDouble();

        // 保存到模型数据库
        auto pNode = std::make_shared<Node>();
        pNode->m_Id = idNode;
        pNode->m_X = xi;
        pNode->m_Y = yi;
        pNode->m_Z = zi;

        m_Structure->m_Nodes.insert(std::make_pair(idNode, pNode));
    }

    for (auto& a : m_Structure->m_Nodes)
    {
        qDebug() << a.first << " " << a.second->m_X << " " << a.second->m_Y << " " << a.second->m_Z;
    }
    return true;
}

bool Input_Model::InputElement(QTextStream& flow, const QStringList& list_str)
{
    static int nElement_Total = 1;

    Q_ASSERT(list_str.size() == 2);
    int nElement = list_str[1].toInt();//得到单元个数，可控制后续循环次数

    QString strdata;
    qDebug().noquote() << QStringLiteral("空间桁架单元数: ") << nElement;
    for (int i = 0; i < nElement; i++)
    {
        // 继续读一行有效数据
        if (!ReadLine(flow, strdata))
        {//没有读取到有效数据，退出
            qDebug() << QStringLiteral("Error: 单元数据不够");
            exit(1);
        }
        QStringList strlist_ele = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);//利用空格,分解字符串

        if (strlist_ele.size() == 5) //Truss    //读取到空间桁架单元
        {
            int idElement = nElement_Total;//取得单元号
            nElement_Total++;
            int idNode0 = strlist_ele[1].toInt();
            int idNode1 = strlist_ele[2].toInt();
            int idMaterial = strlist_ele[3].toInt();
            int idSection = strlist_ele[4].toInt();

            //保存到模型数据库

            auto pElement_Truss = std::make_shared<ElementTruss>();
            pElement_Truss->m_Id = idElement;
            //current_nEle++;
            pElement_Truss->m_pNode[0] = m_Structure->FindNode(idNode0);
            pElement_Truss->m_pNode[1] = m_Structure->FindNode(idNode1);
            auto Property = m_Structure->Create_Property(idMaterial, idSection);
            pElement_Truss->m_pProperty = Property;

            m_Structure->m_Elements.insert(std::make_pair(idElement, pElement_Truss));
        }

    }

    for (auto& a : m_Structure->m_Elements)
    {
        auto pEle = a.second;
        std::cout << pEle->m_Id << " ";

        for (auto& b : pEle->m_pNode)
        {
            auto pNode = b.lock();
            std::cout << pNode->m_Id << " ";
        }
        std::cout << pEle->m_pProperty.lock()->m_Id << std::endl;
    }
    return true;
}

bool Input_Model::InputSection(QTextStream& flow, const QStringList& list_str)
{
    Q_ASSERT(list_str.size() == 2);
    int nSection = list_str[1].toInt();
    qDebug().noquote() << QStringLiteral("截面数: ") << nSection;

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
            auto pSection = std::make_shared<SectionCircular>();
            pSection->m_Id = strlist_pro[0].toInt();
            pSection->m_Radius = strlist_pro[1].toDouble();
            pSection->Calculate_Area();
            m_Structure->m_Section.insert(std::make_pair(pSection->m_Id, pSection));
        }

    }
    return true;
}

bool Input_Model::InputMaterial(QTextStream& flow, const QStringList& list_str)
{
    //读取到材料
    Q_ASSERT(list_str.size() == 2);
    int nMaterial = list_str[1].toInt();
    qDebug().noquote() << QStringLiteral("材料数: ") << nMaterial;
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
        int    id = strlist_mat[0].toInt();
        double  E = strlist_mat[1].toDouble();
        double  v = strlist_mat[2].toDouble();
        double  p = strlist_mat[3].toDouble();//密度
        double  S = strlist_mat[4].toDouble();
        double  e = strlist_mat[5].toDouble();

        //保存到模型数据库
        auto pMaterial = std::make_shared<Material>();
        pMaterial->m_Id = id;
        pMaterial->m_Young = E;
        pMaterial->m_Poisson = v;
        pMaterial->m_Density = p;
        pMaterial->m_MaxStress = S;
        pMaterial->m_Expansion = e;

        m_Structure->m_Material.insert(std::make_pair(pMaterial->m_Id, pMaterial));
    }
    return true;
}

bool Input_Model::InputLoad(QTextStream& flow, const QStringList& list_str)
{
    Q_ASSERT(list_str.size() == 2);
    int nLoad = list_str[1].toInt();
    qDebug().noquote() << QStringLiteral("荷载数: ") << nLoad;
    QString strdata;
    for (int i = 0; i < nLoad; i++)
    {
        if (!ReadLine(flow, strdata))
        {//没有读取到有效数据，退出
            qDebug() << QStringLiteral("Error: 荷载数据不够");
            exit(1);
        }

        QStringList strlist_load = strdata.split(QRegularExpression("[\t, ]"), Qt::SkipEmptyParts);//利用空格,分解字符串
        Q_ASSERT(strlist_load.size() == 5);
        int    id = strlist_load[0].toInt();
        int   type = strlist_load[1].toInt();
        int idNode = strlist_load[2].toInt();
        int  direaction = strlist_load[3].toInt();
        double  value = strlist_load[4].toDouble();
        EnumKeyword::LoadType loadtype = static_cast<EnumKeyword::LoadType>(type);

        switch (loadtype)
        {
        case EnumKeyword::LoadType::NODE_FORCE:
        {
            auto pLoad = std::make_shared<Load_Node>();
            pLoad->m_Id = id;
            pLoad->m_pNode = m_Structure->FindNode(idNode);
            pLoad->m_Direction = static_cast<EnumKeyword::Direction>(direaction);
            pLoad->m_Value = value;
            m_Structure->m_Load.insert(std::make_pair(id, pLoad));
        }
            break;
        case EnumKeyword::LoadType::ELEMENT_PRESSURE:
            break;
        case EnumKeyword::LoadType::UNKNOWN:
            break;
        }

    }
    return true;
}

bool Input_Model::InputConstraint(QTextStream& flow, const QStringList& list_str)
{
    Q_ASSERT(list_str.size() == 2);
    int nConstraint = list_str[1].toInt();
    qDebug().noquote() << QStringLiteral("约束数: ") << nConstraint;
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
        int    id = strlist_con[0].toInt();
        int idNode = strlist_con[1].toInt();
        auto  direaction = strlist_con[2].toInt();
        double  value = strlist_con[3].toDouble();

        auto pConstraint = std::make_shared<Constraint>();
        pConstraint->m_Id = id;
        pConstraint->m_pNode = m_Structure->FindNode(idNode);
        pConstraint->m_Direction = static_cast<EnumKeyword::Direction>(direaction);
        pConstraint->m_Value = value;
        m_Structure->m_Constraint.insert(std::make_pair(id, pConstraint));
    }
    return true;
}
