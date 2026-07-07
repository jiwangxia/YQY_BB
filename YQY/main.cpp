#include "GUI/YQY.h"
#include <QtWidgets/QApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QStringList>
#include "Import/Input_Model.h"
#include "DataStructure/Structure/StructureData.h"
#include "Solver/AnalysisSolve.h"
#include "Utility/Logger/Logger.h"

#include "Import/AeroManager.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    Logger::InitializeConsoleEncoding();

    auto pStructure = std::make_shared<StructureData>();

    Input_Model importer;

    QString BaseName = QStringLiteral("拱形位移Truss");
    QString InputPath = QStringLiteral("Import/ImportFile/%1.bdf").arg(BaseName);
    QString OutputPath = QStringLiteral("Export/ExportFile/%1_TEP.bdf").arg(BaseName);
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

    if (!Logger::Instance().Start(InputPath))
    {
        qDebug().noquote() << QStringLiteral("日志文件创建失败，程序继续运行");
    }

    QElapsedTimer totalTimer;
    totalTimer.start();

    QElapsedTimer stageTimer;
    reportInfo(QStringLiteral("开始读取模型文件: %1").arg(inputFileName));
    stageTimer.start();
    if (importer.InputData(InputPath, pStructure))
    {
        const qint64 importElapsedMs = stageTimer.elapsed();
        reportSuccess(QStringLiteral("模型读取成功，用时 %1 ms").arg(importElapsedMs));

        stageTimer.restart();
        AnalysisRunner analysisRunner;
        analysisRunner.SetStructure(pStructure);
        if (!analysisRunner.RunAll())
        {
            const qint64 solveElapsedMs = stageTimer.elapsed();
            reportError(QStringLiteral("分析计算失败，用时 %1 ms，跳过结果导出").arg(solveElapsedMs));
            Logger::Instance().Stop(false, QStringLiteral("分析失败，跳过结果导出"));
            return -1;
        }
        const qint64 solveElapsedMs = stageTimer.elapsed();
        reportSuccess(QStringLiteral("分析计算成功，用时 %1 ms").arg(solveElapsedMs));

        std::vector<int> nodeIds = { 2 };
        //for (auto& nodePair : pStructure->m_Nodes)
        //{
        //    nodeIds.push_back(nodePair.first);
        //}
        std::vector<EnumKeyword::NodeResultType> types = {
            EnumKeyword::NodeResultType::U2,
            EnumKeyword::NodeResultType::R2
        };
        //std::vector<EnumKeyword::NodeResultType> types = { EnumKeyword::NodeResultType::U3};
        //std::vector<EnumKeyword::NodeResultType> types = { EnumKeyword::NodeResultType::U1, EnumKeyword::NodeResultType::U2, EnumKeyword::NodeResultType::U3, EnumKeyword::NodeResultType::F1, EnumKeyword::NodeResultType::F2, EnumKeyword::NodeResultType::F3 };

        reportInfo(QStringLiteral("开始输出结果: %1").arg(outputFileName));
        stageTimer.restart();
        pStructure->GetOutputter().ExportNodes(OutputPath, nodeIds, types);
        const qint64 exportElapsedMs = stageTimer.elapsed();

        QStringList outputFiles;
        outputFiles << QStringLiteral("BDF: %1").arg(outputFileName);
        if (pStructure->m_OutputControl.m_EnableHdf5 && !pStructure->m_OutputControl.m_Hdf5FileName.isEmpty())
        {
            outputFiles << QStringLiteral("H5: %1").arg(QFileInfo(pStructure->m_OutputControl.m_Hdf5FileName).fileName());
        }
        reportSuccess(QStringLiteral("结果输出完成，用时 %1 ms，输出文件: %2")
            .arg(exportElapsedMs)
            .arg(outputFiles.join(QStringLiteral("; "))));

        const qint64 totalElapsedMs = totalTimer.elapsed();
        reportSuccess(QStringLiteral("程序运行成功，总耗时 %1 ms，运行日志文件: %2")
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

    //YQY window;
    //window.show();
    return app.exec();
}
