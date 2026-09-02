#pragma once
#include "Application/ApplicationPaths.h"
#include "Base/EmptyOUT.h"
#include "GUI/Dialogs/FileDialogService.h"

#include "Utility/EnumKeyword.h"
#include "Widgets/DialogSizing.h"

#include <QCheckBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QSet>
#include <QStringList>
#include <QVBoxLayout>
#include <algorithm>
#include <vector>

class ElementResultExportDialog final : public QDialog
{
public:
    ElementResultExportDialog(const QString& resultFile, const QSet<int>& availableElements, QWidget* parent)
        : QDialog(parent)
        , m_availableElements(availableElements)
    {
        setObjectName(QStringLiteral("nodeResultExportDialog"));
        setWindowTitle(QStringLiteral("导出单元结果"));
        setModal(true);
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(22, 20, 22, 20);
        layout->setSpacing(12);
        layout->setAlignment(Qt::AlignTop);

        auto* title = new QLabel(QStringLiteral("导出单元时程数据"), this);
        title->setObjectName(QStringLiteral("exportDialogTitle"));
        layout->addWidget(title);
        auto* subtitle = new QLabel(QStringLiteral("从当前 H5 结果中选择单元和结果分量"), this);
        subtitle->setObjectName(QStringLiteral("exportDialogSubtitle"));
        layout->addWidget(subtitle);

        auto* selectionCard = new QGroupBox(QStringLiteral("1  选择单元"), this);
        selectionCard->setObjectName(QStringLiteral("exportSectionCard"));
        selectionCard->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        auto* selectionLayout = new QVBoxLayout(selectionCard);
        m_elementEdit = new QLineEdit(selectionCard);
        m_elementEdit->setPlaceholderText(QStringLiteral("例如：1, 3, 8-12"));
        m_elementEdit->setClearButtonEnabled(true);
        selectionLayout->addWidget(m_elementEdit);
        auto* selectionHint =
            new QLabel(QStringLiteral("支持逗号、空格和连续范围；导出前会检查单元是否存在。"), selectionCard);
        selectionHint->setObjectName(QStringLiteral("exportFieldHint"));
        selectionLayout->addWidget(selectionHint);
        layout->addWidget(selectionCard);

        auto* fieldCard = new QGroupBox(QStringLiteral("2  选择数据"), this);
        fieldCard->setObjectName(QStringLiteral("exportSectionCard"));
        fieldCard->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        auto* fields = new QVBoxLayout(fieldCard);
        fields->setSpacing(8);
        addFieldGroup(fields, QStringLiteral("单元内力"),
                      {{QStringLiteral("轴力"), EnumKeyword::ElementResultType::AxialForce, true}});
        addFieldGroup(fields, QStringLiteral("应力应变"),
                      {{QStringLiteral("应变"), EnumKeyword::ElementResultType::Strain, false},
                       {QStringLiteral("初始应力"), EnumKeyword::ElementResultType::InitStress, false},
                       {QStringLiteral("当前应力"), EnumKeyword::ElementResultType::CurrentStress, false},
                       {QStringLiteral("应力增量"), EnumKeyword::ElementResultType::DeltaStress, false}});
        layout->addWidget(fieldCard);

        auto* fileCard = new QGroupBox(QStringLiteral("3  保存文件"), this);
        fileCard->setObjectName(QStringLiteral("exportSectionCard"));
        fileCard->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        auto* fileLayout = new QVBoxLayout(fileCard);
        m_fileEdit = new QLineEdit(fileCard);
        const QFileInfo resultInfo(resultFile);
        m_fileEdit->setText(QDir(exportDirectory(resultFile))
                                .absoluteFilePath(resultInfo.completeBaseName() + QStringLiteral("_单元时程.bdf")));
        auto* browseButton = new QPushButton(QStringLiteral("浏览"), fileCard);
        browseButton->setObjectName(QStringLiteral("exportQuietButton"));
        auto* fileRow = new QHBoxLayout;
        fileRow->addWidget(m_fileEdit);
        fileRow->addWidget(browseButton);
        fileLayout->addLayout(fileRow);
        layout->addWidget(fileCard);

        m_errorLabel = new QLabel(this);
        m_errorLabel->setObjectName(QStringLiteral("exportValidationLabel"));
        layout->addWidget(m_errorLabel);
        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
        buttons->setObjectName(QStringLiteral("exportDialogButtons"));
        m_exportButton = buttons->button(QDialogButtonBox::Save);
        m_exportButton->setText(QStringLiteral("导出"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        layout->addWidget(buttons);

        connect(m_elementEdit, &QLineEdit::textChanged, this,
                [this]()
                {
                    updateValidation();
                });
        connect(m_fileEdit, &QLineEdit::textChanged, this,
                [this]()
                {
                    updateValidation();
                });
        connect(browseButton, &QPushButton::clicked, this,
                [this]()
                {
                    const QString file =
                        FileDialogService::selectSaveFile(this, QStringLiteral("保存单元结果"), m_fileEdit->text(),
                                                          QStringLiteral("BDF 结果文件 (*.bdf);;文本文件 (*.txt)"),
                                                          QStringLiteral("elementResultExport"));
                    if (!file.isEmpty())
                        m_fileEdit->setText(file);
                });
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(buttons, &QDialogButtonBox::accepted, this,
                [this]()
                {
                    if (updateValidation())
                        accept();
                });
        resize(700, 500);
        DialogSizing::lockHeightToContents(this);
        updateValidation();
    }

    std::vector<int> elementIds() const
    {
        QString error;
        return parseElementIds(error);
    }
    std::vector<EnumKeyword::ElementResultType> resultTypes() const
    {
        std::vector<EnumKeyword::ElementResultType> result;
        for (const auto& field : m_fields)
            if (field.box->isChecked())
                result.push_back(field.type);
        return result;
    }
    QString outputFile() const
    {
        return QFileInfo(m_fileEdit->text().trimmed()).absoluteFilePath();
    }

private:
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
    static QString exportDirectory(const QString& resultFile)
    {
        return ApplicationPaths::resultExportDirectory(resultFile);
    }
    void addFieldGroup(QVBoxLayout* layout, const QString& caption, std::initializer_list<FieldDefinition> definitions)
    {
        auto* group = new QWidget(this);
        auto* row = new QHBoxLayout(group);
        row->setContentsMargins(0, 0, 0, 0);
        row->setSpacing(12);
        auto* captionLabel = new QLabel(caption, group);
        captionLabel->setObjectName(QStringLiteral("exportFieldCaption"));
        captionLabel->setMinimumWidth(70);
        row->addWidget(captionLabel);
        for (const auto& definition : definitions)
        {
            auto* box = new QCheckBox(definition.label, group);
            box->setChecked(definition.checked);
            row->addWidget(box);
            m_fields.push_back({box, definition.type});
            connect(box, &QCheckBox::toggled, this,
                    [this]()
                    {
                        updateValidation();
                    });
        }
        row->addStretch();
        layout->addWidget(group);
    }
    std::vector<int> parseElementIds(_OUT QString& error) const
    {
        QSet<int> ids;
        const QStringList tokens =
            m_elementEdit->text().split(QRegularExpression(QStringLiteral("[,，;；\\s]+")), Qt::SkipEmptyParts);
        const QRegularExpression rangePattern(QStringLiteral("^(\\d+)\\s*[-~～]\\s*(\\d+)$"));
        for (const QString& token : tokens)
        {
            const auto match = rangePattern.match(token);
            bool ok = false;
            int first = token.toInt(&ok);
            int last = first;
            if (match.hasMatch())
            {
                first = match.captured(1).toInt();
                last = match.captured(2).toInt();
                ok = true;
            }
            if (!ok || first <= 0 || last < first || last - first > 100000)
            {
                error = QStringLiteral("无法识别单元编号“%1”").arg(token);
                return {};
            }
            for (int id = first; id <= last; ++id)
                ids.insert(id);
        }
        QList<int> sorted = ids.values();
        std::sort(sorted.begin(), sorted.end());
        for (int id : sorted)
            if (!m_availableElements.contains(id))
            {
                error = QStringLiteral("模型中不存在单元 %1").arg(id);
                return {};
            }
        return std::vector<int>(sorted.begin(), sorted.end());
    }
    bool updateValidation()
    {
        QString error;
        const auto ids = parseElementIds(error);
        if (error.isEmpty() && ids.empty())
            error = QStringLiteral("请输入至少一个单元编号");
        if (error.isEmpty() && resultTypes().empty())
            error = QStringLiteral("请至少勾选一个结果分量");
        if (error.isEmpty() && m_fileEdit->text().trimmed().isEmpty())
            error = QStringLiteral("请选择输出文件");
        if (error.isEmpty() && !QDir(QFileInfo(m_fileEdit->text().trimmed()).absolutePath()).exists())
            error = QStringLiteral("输出文件夹不存在");
        m_errorLabel->setText(error);
        m_exportButton->setEnabled(error.isEmpty());
        return error.isEmpty();
    }
    QSet<int> m_availableElements;
    QLineEdit* m_elementEdit = nullptr;
    QLineEdit* m_fileEdit = nullptr;
    QLabel* m_errorLabel = nullptr;
    QPushButton* m_exportButton = nullptr;
    std::vector<Field> m_fields;
};
