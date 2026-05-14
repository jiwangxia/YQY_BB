#include "GUI/YQY.h"
#include <QtWidgets/QApplication>
#include "Import/Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
#include "Solver/Solver.h"
#include "Conductor/Conductor.h"
#include <Windows.h>

#include "Import/AeroManager.h"
int main(int argc, char* argv[])
{

    QApplication app(argc, argv);

    auto pStructure = std::make_shared<StructureData>();

    Input_Model importer;

    QString BaseName = "拱形位移Truss";
    QString InputPath = QString("Import/ImportFile/%1.bdf").arg(BaseName);
    QString OutputPath = QString("Export/ExportFile/%1_TEP.bdf").arg(BaseName);

    qDebug().noquote() << QStringLiteral("\n读取文件为:") << InputPath << "\n";
    if (importer.InputData(InputPath, pStructure))
    {
        qDebug() << "\n=====Model loaded successfully!=====";

        //     使用 Solver 运行分析
        Solver solver;
        solver.SetStructure(pStructure);
        solver.RunAll();  // 运行所有分析步

        std::vector<int> nodeIds = { 2 };
        //for (auto& nodePair : pStructure->m_Nodes)
        //{
        //    nodeIds.push_back(nodePair.first);
        //}
        std::vector<DataType> types = {DataType::U2, DataType::R2};
        //std::vector<DataType> types = { DataType::U3};
        //std::vector<DataType> types = { DataType::U1, DataType::U2, DataType::U3, DataType::F1, DataType::F2, DataType::F3 };

        pStructure->GetOutputter().ExportNodes(OutputPath, nodeIds, types);
    }

    //Conductor::ConductorConfig Config;
    //Config.nBundle = 1; // 四分裂
    //Config.segments = 50;
    //Config.numSpacers = 4;
    //Config.spacing = 0.5656;
    //Config.connecttype = Conductor::ConnectionMode::Parallel;
    //std::vector<double> startPt = { 0.0, 0.0, 0.0 };
    //std::vector<double> endPt = { 100.0, 0.0, 0.0 };
    //// 2. 生成数据
    //auto result = Conductor::Generator::CreateBundle(startPt.data(), endPt.data(), 0.0, 0.0, Config);

    //for (auto& [wireid, nodesVector] : result.wiresNode)
    //{
    //    for (auto& node : nodesVector)
    //    {
    //        auto pNode = std::make_shared<Node>();
    //        pNode->m_X = node.x;
    //        pNode->m_Y = node.y;
    //        pNode->m_Z = node.z;

    //        auto maxid = int(pStructure->m_Nodes.size()) + 1;

    //        pNode->m_Id = maxid;
    //        pStructure->m_Nodes.insert(std::make_pair(maxid, pNode));

    //        node.id = maxid; // 更新原始节点的ID，记录全局id
    //    }
    //}

    //pStructure.get()->Add_Property(6.5E11, 3800, 0.005);

    //for (auto& [wireid, elementsVector] : result.wiresElement)
    //{
    //    auto& currentWireNodes = result.wiresNode[wireid];

    //    for (auto& elem : elementsVector)
    //    {
    //        auto pElement = std::make_shared<ElementTruss>();

    //        int global0 = currentWireNodes[elem.iNode - 1].id; // 获取全局节点ID
    //        int global1 = currentWireNodes[elem.jNode - 1].id;

    //        pElement->m_pNode[0] = pStructure->FindNode(global0);
    //        pElement->m_pNode[1] = pStructure->FindNode(global1);

    //        pElement->m_Id = int(pStructure->m_Elements.size()) + 1;
    //        pElement->m_InitStress = elem.stress0;
    //        pElement->m_pProperty = pStructure->Create_Property(1, 1);
    //        pStructure->m_Elements.insert(std::make_pair(static_cast<int>(pElement->m_Id), pElement));

    //    }
    //}
    //// 添加约束：固定导线两端节点的所有自由度
    //std::vector<int> constrainedNodes{ 1, 53 };  // 假设第一根导线的起点和终点节点ID
    //std::vector<int> direction{ 0, 1, 2 };  // 固定X, Y, Z三个方向
    //std::vector<double> value{ 0, 0, 0 };

    //pStructure.get()->Add_Constraint(constrainedNodes, direction, value);

    //// 添加重力荷载（Z方向，重力加速度系数为1）
    //pStructure.get()->Add_Gravity(2, 1);

    //// 添加动力学分析步
    //// 参数：类型, 总时间, 时间步长, 收敛容差, 最大迭代次数
    //pStructure.get()->Add_AnalysisStep("Dynamic", 10.0, 0.01, 1e-4, 1000);

    //// 运行求解器
    //Solver solver;
    //solver.SetStructure(pStructure);
    //solver.RunAll();

    //// 输出中间节点的结果
    //std::vector<int> nodeIds = { 26 };  // 导线中点节点
    //std::vector<DataType> types = { DataType::U1, DataType::U2, DataType::U3, DataType::F1, DataType::F2, DataType::F3 };
    //QString OutputPath = QString("Export/ExportFile/dynamic_result.bdf");
    //pStructure->GetOutputter().ExportNodes(OutputPath, nodeIds, types);

    //pStructure->GetOutputter().SaveModel("Export/ExportFile/分裂导线.bdf", pStructure.get());




    // 启用控制台颜色支持
    enableConsoleColor();

    // 设置控制台输出编码为UTF-8
    //SetConsoleOutputCP(CP_UTF8);

    //AeroManager manager;

    //std::string Filename = "1-14ms-12mm.csv";
    ////读取CSV文件，然后输出读取的数据到另一个CSV文件
    //bool success = manager.loadCSV("Import/Aero_Data/Input_Data/" + Filename);

    //if (!success)
    //{
    //    std::cerr << "\n 无法加载测试数据，程序终止。\n";
    //    return -1;
    //}
    ////
    ////manager.exportToCSV("C_Data/Output_Data/" + Filename, 6);

    ////for (double i = 1.5; i <= 7; i += 5)
    ////{
    ////    std::cout << "\nLift = " << manager.getData(0, CoefType::LIFT, i); // 测试插值
    ////    std::cout << "\nDrag = " << manager.getData(0, CoefType::DRAG, i); // 测试插值
    ////    std::cout << "\nMoment = " << manager.getData(0, CoefType::MOMENT, i); // 测试插值

    ////}


    //// 自动扫描Output_Data文件夹，与C_Data中的同名文件对比 
    ////验证气动参数读取是否有问题

    //bool allPassed = manager.ValiDateAllCSV(
    //    "Import/Aero_Data/Output_Data",   // 输出文件夹
    //    "Import/Aero_Data/Input_Data",    // 标准文件夹
    //    0.000001                // 误差容限
    //);


    //YQY window;
    //window.show();
    return app.exec();
}
