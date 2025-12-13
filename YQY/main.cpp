#include "GUI/YQY.h"
#include <QtWidgets/QApplication>
#include "Import/Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    
    auto pStructure = std::make_shared<StructureData>();

    Input_Model importer;

    QString fileName = "Import/ImportFile/Start.txt";
    // 5. 调用读取函数
    if (importer.InputData(fileName, pStructure))
    {
        qDebug() << "Model loaded successfully!";
        // 此时数据已经存储在 pStructure 中了
    }
    else
    {
        qDebug() << "Failed to load model.";
    }

    YQY window;
    window.show();
    return app.exec();

}
