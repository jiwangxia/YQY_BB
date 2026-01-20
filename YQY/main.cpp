#include "GUI/YQY.h"
#include <QtWidgets/QApplication>
#include "Import/Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
//#include "DataStructure/AnalysisStep/AnalysisStep.h"
#include "Solver/Solver.h"
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    auto pStructure = std::make_shared<StructureData>();

    Input_Model importer;


    if (importer.InputData("Import/ImportFile/200m.txt", pStructure))
    {
        qDebug() << "\n=====Model loaded successfully!=====";

        // 使用 Solver 运行分析
        Solver solver;
        solver.SetStructure(pStructure);
        solver.RunAll();  // 运行所有分析步

        // 或者运行指定分析步
        // solver.RunStep(1);
    }

    YQY window;
    window.show();
    return app.exec();

}
