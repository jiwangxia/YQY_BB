#pragma once

#include <optional>

class QApplication;
class YQY;

class UiAuditRunner final
{
public:
    explicit UiAuditRunner(const QStringList& arguments);

    bool isValid() const { return m_valid; }
    int errorCode() const { return m_errorCode; }
    std::optional<int> run(QApplication& application, YQY& window) const;

private:
    enum class Kind
    {
        None,
        ModelImportDialog,
        SolveTasks,
        NodeExport,
        Postprocess,
        AnalysisManager
    };

    Kind m_kind = Kind::None;
    bool m_valid = true;
    int m_errorCode = 1;
    QString m_outputFile;
    QString m_modelFile;
    QString m_resultFile;
    int m_analysisManager = 0;
};
