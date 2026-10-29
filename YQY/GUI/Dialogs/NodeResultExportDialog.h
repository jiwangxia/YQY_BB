#pragma once

#include "Base/EmptyOUT.h"
#include "Utility/EnumKeyword.h"
#include <QDialog>
#include <QSet>
#include <vector>

class QCheckBox;
class QDialogButtonBox;
class QFrame;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QWidget;

class NodeResultExportDialog final : public QDialog
{
public:
    NodeResultExportDialog(const QString& resultFile, const QSet<int>& availableNodes, int selectedNodeId,
                           QWidget* parent);                      // 创建节点结果导出窗口并预选当前节点

    std::vector<int> nodeIds() const;                             // 读取待导出的节点编号列表

    std::vector<EnumKeyword::NodeResultType> resultTypes() const; // 读取勾选的节点结果分量

    QString outputFile() const;                                   // 读取结果导出的目标文件路径

private:
    void updateDefaultFile(); // 选择改变时更新建议文件名，保留手动路径
    QString m_resultFile;
    QString m_defaultFile;

    void initializeUi();                                          // 创建控件和布局

    struct FieldCheckBox
    {
        QCheckBox* box = nullptr;
        EnumKeyword::NodeResultType type = EnumKeyword::NodeResultType::U1;
    };

    std::vector<int> parseNodeIds(_OUT QString& error) const;     // 解析节点编号并返回输入错误说明

    bool updateValidation();                                      // 校验节点、分量和路径并更新导出可用状态

    QSet<int> m_availableNodes;
    QLineEdit* m_nodeEdit = nullptr;
    QLineEdit* m_fileEdit = nullptr;
    QLabel* m_errorLabel = nullptr;
    QPushButton* m_exportButton = nullptr;
    std::vector<FieldCheckBox> m_fields;

    QLabel* m_titleLabel = nullptr;
    QLabel* m_subtitleLabel = nullptr;
    QFrame* m_nodeCard = nullptr;
    QLabel* m_nodeTitle = nullptr;
    QPushButton* m_currentNodeButton = nullptr;
    QLabel* m_nodeHint = nullptr;
    QFrame* m_fieldCard = nullptr;
    QLabel* m_fieldTitle = nullptr;
    QGroupBox* m_displacementGroup = nullptr;
    QCheckBox* m_u1 = nullptr;
    QCheckBox* m_u2 = nullptr;
    QCheckBox* m_u3 = nullptr;
    QCheckBox* m_um = nullptr;
    QGroupBox* m_rotationGroup = nullptr;
    QCheckBox* m_ur1 = nullptr;
    QCheckBox* m_ur2 = nullptr;
    QCheckBox* m_ur3 = nullptr;
    QGroupBox* m_velocityGroup = nullptr;
    QCheckBox* m_v1 = nullptr;
    QCheckBox* m_v2 = nullptr;
    QCheckBox* m_v3 = nullptr;
    QGroupBox* m_accelerationGroup = nullptr;
    QCheckBox* m_a1 = nullptr;
    QCheckBox* m_a2 = nullptr;
    QCheckBox* m_a3 = nullptr;
    QGroupBox* m_coordinateGroup = nullptr;
    QCheckBox* m_cx = nullptr;
    QCheckBox* m_cy = nullptr;
    QCheckBox* m_cz = nullptr;
    QGroupBox* m_reactionGroup = nullptr;
    QCheckBox* m_r1 = nullptr;
    QCheckBox* m_r2 = nullptr;
    QCheckBox* m_r3 = nullptr;
    QFrame* m_fileCard = nullptr;
    QLabel* m_fileTitle = nullptr;
    QPushButton* m_browseButton = nullptr;
    QDialogButtonBox* m_buttonBox = nullptr;
};
