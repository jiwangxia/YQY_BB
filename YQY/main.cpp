#include "GUI/YQY.h"
#include <QtWidgets/QApplication>
#include "Import/Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
#include "Solver/Solver.h"
#include "Conductor/ConductorLib.h"
int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    auto pStructure = std::make_shared<StructureData>();

    Input_Model importer;

    QString BaseName = "Conductor";
    QString InputPath = QString("Import/ImportFile/%1.txt").arg(BaseName);
    QString OutputPath = QString("Export/ExportFile/%1_TEP.bdf").arg(BaseName);

    qDebug().noquote() << QStringLiteral("\n读取文件为:") << InputPath << "\n";
    if (importer.InputData(InputPath, pStructure))
    {
        qDebug() << "\n=====Model loaded successfully!=====";

        // 使用 Solver 运行分析
        Solver solver;
        solver.SetStructure(pStructure);
        solver.RunAll();  // 运行所有分析步

        std::vector<int> nodeIds = {  3 };
        std::vector<DataType> types = { DataType::U1, DataType::U2, DataType::U3, DataType::F1, DataType::F2, DataType::F3 };

        pStructure->GetOutputter().ExportNodes(OutputPath, nodeIds, types);
    }

    //ConductorLib::Config cfg;
    //cfg.nBundle = 4; // 四分裂
    //cfg.segments = 50;
    //cfg.mode = ConductorLib::ConnectionMode::Parallel;
    //std::vector<double> startPt = { 0.0, 0.0, 0.0 };
    //std::vector<double> endPt = { 200.0, 0.0, 0.0 };
    //// 2. 生成数据
    //auto result = ConductorLib::Generator::CreateBundle(startPt.data(), endPt.data(), cfg);

    //for (auto& node : result.nodes)
    //{
    //    auto pNode = std::make_shared<Node>();
    //    pNode->m_X = node.x;
    //    pNode->m_Y = node.y;
    //    pNode->m_Z = node.z;

    //    auto maxid = pStructure->m_Nodes.size() + 1;
    //    pNode->m_Id = maxid;
    //    pStructure->m_Nodes.insert(std::make_pair(static_cast<int>(maxid), pNode));
    //}

    //for (auto& elem : result.elements)
    //{
    //    auto pElement = std::make_shared<ElementTruss>();
    //    pElement->m_pNode[0] = pStructure->FindNode(static_cast<int>(elem.iNode));
    //    pElement->m_pNode[1] = pStructure->FindNode(static_cast<int>(elem.jNode));
    //    pElement->m_Id = pStructure->m_Elements.size() + 1;
    //    pElement->m_InitStress = elem.stress0;
    //    pElement->m_pProperty = pStructure->Create_Property(1, 1);
    //    pStructure->m_Elements.insert(std::make_pair(static_cast<int>(pElement->m_Id), pElement));
    //}

    //pStructure->GetOutputter().SaveModel("Export/ExportFile/Conductor_TEP.bdf", pStructure.get());
    YQY window;
    //window.show();
    return app.exec();

}
