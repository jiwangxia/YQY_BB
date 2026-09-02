#include "Controllers/WorkspaceController.h"

#include <QFileInfo>

void WorkspaceController::associateResult(int modelId, const QString& resultFile)
{
    if (modelId < 0 || resultFile.trimmed().isEmpty())
        return;
    m_resultFiles.insert(modelId, QFileInfo(resultFile).absoluteFilePath());
}

QString WorkspaceController::resultForModel(int modelId) const
{
    return m_resultFiles.value(modelId);
}

void WorkspaceController::removeModel(int modelId)
{
    m_resultFiles.remove(modelId);
}

void WorkspaceController::clear()
{
    m_resultFiles.clear();
}
