#include "GUI/YQY.h"
#include <QtWidgets/QApplication>
#include "Import/Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
#include "Solver/Solver.h"
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    auto pStructure = std::make_shared<StructureData>();

    Input_Model importer;

    QString BaseName   = "竖直杆受轴和水平力";
    QString InputPath  = QString("Import/ImportFile/%1.bdf").arg(BaseName);
    QString OutputPath = QString("Export/ExportFile/%1_TEP.bdf").arg(BaseName);

    qDebug().noquote() << QStringLiteral("\n读取文件为:") << InputPath << "\n";
    if (importer.InputData(InputPath, pStructure))
    {
        qDebug() << "\n=====Model loaded successfully!=====";

        // 使用 Solver 运行分析
        Solver solver;
        solver.SetStructure(pStructure);
        solver.RunAll();  // 运行所有分析步

        std::vector<int> nodeIds = { 2 };
        std::vector<DataType> types = { DataType::U1, DataType::U2, DataType::F1, DataType::F2, DataType::F3 };

        pStructure->GetOutputter().ExportNodes(OutputPath, nodeIds, types);
        // 或者运行指定分析步
        // solver.RunStep(1);
    }

    YQY window;
    window.show();
    return app.exec();

}
