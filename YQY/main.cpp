#include "GUI/YQY.h"
#include <QtWidgets/QApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <QFileInfo>
#include "Import/Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
#include "Solver/Solver.h"
#include "Utility/Logger/Logger.h"
#include <Windows.h>

#include "Import/AeroManager.h"
int main(int argc, char* argv[])
{

    QApplication app(argc, argv);

    auto pStructure = std::make_shared<StructureData>();

    Input_Model importer;

    QString BaseName = "24杆星形穹顶桁架";
    QString InputPath = QString("Import/ImportFile/%1.bdf").arg(BaseName);
    QString OutputPath = QString("Export/ExportFile/%1_TEP.bdf").arg(BaseName);
    const QString inputFileName = QFileInfo(InputPath).fileName();
    const QString outputFileName = QFileInfo(OutputPath).fileName();

    auto reportInfo = [](const QString& message)
        {
            Logger::Instance().Info(message);
        };

    auto reportSuccess = [](const QString& message)
        {
            Logger::Instance().Success(message);
        };

    auto reportError = [](const QString& message)
        {
            Logger::Instance().Error(message);
        };

    if (Logger::Instance().Start(InputPath))
    {
        reportInfo(QStringLiteral("日志文件: %1").arg(QFileInfo(Logger::Instance().LogFilePath()).fileName()));
    }
    else
    {
        qDebug().noquote() << QStringLiteral("日志文件创建失败，程序继续运行");
    }

    QElapsedTimer totalTimer;
    totalTimer.start();

    QElapsedTimer stageTimer;
    reportInfo(QStringLiteral("开始读取模型: %1").arg(inputFileName));
    stageTimer.start();
    if (importer.InputData(InputPath, pStructure))
    {
        const qint64 importElapsedMs = stageTimer.elapsed();
        reportSuccess(QStringLiteral("模型读取成功，用时 %1 ms").arg(importElapsedMs));
        qDebug() << "\n=====Model loaded successfully!=====";

        //     使用 Solver 运行分析
        reportInfo(QStringLiteral("开始分析计算"));
        stageTimer.restart();
        Solver solver;
        solver.SetStructure(pStructure);
        if (!solver.RunAll())  // 运行所有分析步
        {
            const qint64 solveElapsedMs = stageTimer.elapsed();
            reportError(QStringLiteral("分析计算失败，用时 %1 ms，跳过结果导出").arg(solveElapsedMs));
            Logger::Instance().Stop(false, QStringLiteral("分析失败，跳过结果导出"));
            return -1;
        }
        const qint64 solveElapsedMs = stageTimer.elapsed();
        reportSuccess(QStringLiteral("分析计算成功，用时 %1 ms").arg(solveElapsedMs));

        std::vector<int> nodeIds = { 1 };
        //for (auto& nodePair : pStructure->m_Nodes)
        //{
        //    nodeIds.push_back(nodePair.first);
        //}
        std::vector<EnumKeyword::NodeResultType> types = {
            EnumKeyword::NodeResultType::U1,
            EnumKeyword::NodeResultType::U2,
            EnumKeyword::NodeResultType::U3,
            EnumKeyword::NodeResultType::CX,
            EnumKeyword::NodeResultType::CY,
            EnumKeyword::NodeResultType::CZ,
            EnumKeyword::NodeResultType::R3
        };
        //std::vector<EnumKeyword::NodeResultType> types = { EnumKeyword::NodeResultType::U3};
        //std::vector<EnumKeyword::NodeResultType> types = { EnumKeyword::NodeResultType::U1, EnumKeyword::NodeResultType::U2, EnumKeyword::NodeResultType::U3, EnumKeyword::NodeResultType::F1, EnumKeyword::NodeResultType::F2, EnumKeyword::NodeResultType::F3 };

        reportInfo(QStringLiteral("开始输出结果: %1").arg(outputFileName));
        stageTimer.restart();
        pStructure->GetOutputter().ExportNodes(OutputPath, nodeIds, types);
        const qint64 exportElapsedMs = stageTimer.elapsed();
        reportSuccess(QStringLiteral("结果输出完成，用时 %1 ms").arg(exportElapsedMs));

        const qint64 totalElapsedMs = totalTimer.elapsed();
        reportSuccess(QStringLiteral("程序运行成功，总耗时 %1 ms，日志文件: %2")
            .arg(totalElapsedMs)
            .arg(QFileInfo(Logger::Instance().LogFilePath()).fileName()));
        Logger::Instance().Stop(true, QStringLiteral("模型读取、分析和结果导出完成"));
    }
    else
    {
        const qint64 importElapsedMs = stageTimer.elapsed();
        reportError(QStringLiteral("模型读取失败，用时 %1 ms，程序终止").arg(importElapsedMs));
        Logger::Instance().Stop(false, QStringLiteral("模型读取失败"));
        return -1;
    }

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
