#include "Application/UiAuditRunner.h"

#include "GUI/Dialogs/ModelImportFileDialog.h"
#include "GUI/YQY.h"

UiAuditRunner::UiAuditRunner(const QStringList& arguments)
{
    const int analysisIndex = arguments.indexOf(QStringLiteral("--audit-analysis-manager"));
    const int modelImportIndex = arguments.indexOf(QStringLiteral("--audit-model-import-dialog"));
    const int solveIndex = arguments.indexOf(QStringLiteral("--audit-solve-tasks"));
    const int postprocessIndex = arguments.indexOf(QStringLiteral("--audit-postprocess"));
    const int nodeExportIndex = arguments.indexOf(QStringLiteral("--audit-node-export"));

    if (analysisIndex >= 0)
    {
        if (analysisIndex + 3 >= arguments.size())
        {
            m_valid = false;
            return;
        }
        bool themeOk = false;
        const int theme = arguments.at(analysisIndex + 1).toInt(&themeOk);
        if (!themeOk || theme < 0 || theme > 3)
        {
            m_valid = false;
            return;
        }
        QSettings().setValue(QStringLiteral("appearance/theme"), theme);
    }

    if (modelImportIndex >= 0)
    {
        if (modelImportIndex + 2 >= arguments.size())
        {
            m_valid = false;
            return;
        }
        m_kind = Kind::ModelImportDialog;
        m_outputFile = arguments.at(modelImportIndex + 1);
        m_modelFile = arguments.at(modelImportIndex + 2);
        return;
    }

    if (solveIndex >= 0)
    {
        if (solveIndex + 2 >= arguments.size())
        {
            m_valid = false;
            return;
        }
        m_kind = Kind::SolveTasks;
        m_outputFile = arguments.at(solveIndex + 1);
        m_modelFile = arguments.at(solveIndex + 2);
        return;
    }

    if (nodeExportIndex >= 0)
    {
        if (nodeExportIndex + 3 >= arguments.size())
        {
            m_valid = false;
            return;
        }
        m_kind = Kind::NodeExport;
        m_outputFile = arguments.at(nodeExportIndex + 1);
        m_modelFile = arguments.at(nodeExportIndex + 2);
        m_resultFile = arguments.at(nodeExportIndex + 3);
        return;
    }

    if (postprocessIndex >= 0)
    {
        if (postprocessIndex + 3 >= arguments.size())
        {
            m_valid = false;
            return;
        }
        m_kind = Kind::Postprocess;
        m_outputFile = arguments.at(postprocessIndex + 1);
        m_modelFile = arguments.at(postprocessIndex + 2);
        m_resultFile = arguments.at(postprocessIndex + 3);
        return;
    }

    if (analysisIndex >= 0)
    {
        m_kind = Kind::AnalysisManager;
        m_outputFile = arguments.at(analysisIndex + 2);
        m_modelFile = arguments.at(analysisIndex + 3);
        if (analysisIndex + 4 < arguments.size())
            m_analysisManager = qBound(0, arguments.at(analysisIndex + 4).toInt(), 3);
    }
}

std::optional<int> UiAuditRunner::run(QApplication& application, YQY& window) const
{
    if (m_kind == Kind::None)
        return std::nullopt;
    if (m_kind == Kind::ModelImportDialog)
    {
        ModelImportFileDialog dialog(m_modelFile, &window);
        dialog.show();
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        const bool saved = dialog.grab().save(m_outputFile);
        dialog.close();
        return saved ? 0 : 3;
    }
    if (!window.openModel(m_modelFile))
        return 1;

    if (m_kind == Kind::SolveTasks)
    {
        auto attempts = std::make_shared<int>(0);
        auto capture = std::make_shared<std::function<void()>>();
        *capture = [&application, &window, outputFile = m_outputFile, attempts, capture]()
        {
            if (window.saveSolveTaskManagerPreview(outputFile))
            {
                application.exit(0);
                return;
            }
            if (++(*attempts) >= 100)
            {
                application.exit(2);
                return;
            }
            QTimer::singleShot(100, &application, *capture);
        };
        QTimer::singleShot(100, &application, *capture);
        return application.exec();
    }

    if (m_kind == Kind::NodeExport || m_kind == Kind::Postprocess)
    {
        auto attempts = std::make_shared<int>(0);
        auto capture = std::make_shared<std::function<void()>>();
        *capture = [&application, &window, kind = m_kind, outputFile = m_outputFile, resultFile = m_resultFile,
                    attempts, capture]()
        {
            if (window.openResult(resultFile))
            {
                const int delay = kind == Kind::NodeExport ? 300 : 600;
                QTimer::singleShot(delay, &application,
                                   [&application, &window, kind, outputFile]()
                                   {
                                       const bool saved = kind == Kind::NodeExport
                                                              ? window.saveNodeExportPreview(outputFile)
                                                              : window.grab().save(outputFile);
                                       application.exit(saved ? 0 : 3);
                                   });
                return;
            }
            if (++(*attempts) >= 100)
            {
                application.exit(2);
                return;
            }
            QTimer::singleShot(100, &application, *capture);
        };
        QTimer::singleShot(100, &application, *capture);
        return application.exec();
    }

    auto attempts = std::make_shared<int>(0);
    auto capture = std::make_shared<std::function<void()>>();
    *capture = [&application, &window, outputFile = m_outputFile, manager = m_analysisManager, attempts, capture]()
    {
        if (window.saveAnalysisManagerPreview(outputFile, manager))
        {
            application.exit(0);
            return;
        }
        if (++(*attempts) >= 100)
        {
            application.exit(2);
            return;
        }
        QTimer::singleShot(100, &application, *capture);
    };
    QTimer::singleShot(100, &application, *capture);
    return application.exec();
}
