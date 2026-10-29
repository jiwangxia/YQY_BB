#pragma once

#include <QDialog>
#include <QSet>

class QCheckBox;
class QLabel;
class QLineEdit;
class QWidget;




struct ResultPlotSelection
{
    int nodeId = -1;
    bool displacementX = false;
    bool displacementY = true;
    bool displacementZ = true;
    bool rotationX = true;
    bool rotationY = false;
    bool rotationZ = false;
    QString outputFile;
};

class ResultPlotExportDialog final : public QDialog
{
public:
    ResultPlotExportDialog(const QString& resultFile, const QSet<int>& availableNodes, int selectedNodeId,
                           QWidget* parent); // 创建节点时程曲线导出窗口

    ResultPlotSelection selection() const;   // 读取节点、曲线分量和输出路径

private:
    void updateDefaultFile(); // 选择改变时更新建议文件名，保留手动路径
    QString m_resultFile;
    QString m_defaultFile;

    bool updateValidation();                 // 校验节点、绘图分量和输出路径

    QSet<int> m_availableNodes;
    QLineEdit* m_nodeEdit = nullptr;
    QCheckBox* m_ux = nullptr;
    QCheckBox* m_uy = nullptr;
    QCheckBox* m_uz = nullptr;
    QCheckBox* m_rx = nullptr;
    QCheckBox* m_ry = nullptr;
    QCheckBox* m_rz = nullptr;
    QLineEdit* m_fileEdit = nullptr;
    QLabel* m_errorLabel = nullptr;
};
