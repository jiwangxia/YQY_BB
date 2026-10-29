#pragma once

#include "Base/EmptyOUT.h"
#include "Utility/EnumKeyword.h"
#include <QDialog>
#include <QSet>
#include <vector>
#include <initializer_list>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QVBoxLayout;
class QWidget;




class ElementResultExportDialog final : public QDialog
{
public:
    ElementResultExportDialog(const QString& resultFile, const QSet<int>& availableElements, QWidget* parent);           // 创建单元结果导出窗口

    std::vector<int> elementIds() const;                                                                                 // 读取待导出的单元编号列表
    std::vector<EnumKeyword::ElementResultType> resultTypes() const;                                                     // 读取勾选的单元结果分量
    QString outputFile() const;                                                                                          // 读取结果导出的目标文件路径

private:
    void updateDefaultFile(); // 选择改变时更新建议文件名，保留手动路径
    QString m_resultFile;
    QString m_defaultFile;

    struct Field
    {
        QCheckBox* box;
        EnumKeyword::ElementResultType type;
    };
    struct FieldDefinition
    {
        QString label;
        EnumKeyword::ElementResultType type;
        bool checked;
    };
    static QString exportDirectory(const QString& resultFile);                                                           // 根据结果文件确定默认导出目录
    void addFieldGroup(QVBoxLayout* layout, const QString& caption, std::initializer_list<FieldDefinition> definitions); // 创建一组可勾选的单元结果分量
    std::vector<int> parseElementIds(_OUT QString& error) const;                                                         // 解析单元编号并返回输入错误说明
    bool updateValidation();                                                                                             // 校验编号、分量和路径并更新导出可用状态
    QSet<int> m_availableElements;
    QLineEdit* m_elementEdit = nullptr;
    QLineEdit* m_fileEdit = nullptr;
    QLabel* m_errorLabel = nullptr;
    QPushButton* m_exportButton = nullptr;
    std::vector<Field> m_fields;
};
