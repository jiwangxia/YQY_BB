#pragma once
#include "Application/ApplicationPaths.h"
#include "Base/EmptyOUT.h"
#include "Utility/EnumKeyword.h"
#include "Widgets/DialogSizing.h"
#include "ui_NodeResultExportDialog.h"
#include <QCheckBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QSet>
#include <algorithm>
#include <vector>

class NodeResultExportDialog final : public QDialog
{
public:
    NodeResultExportDialog(const QString& resultFile, const QSet<int>& availableNodes, int selectedNodeId,
                           QWidget* parent)
        : QDialog(parent)
        , m_availableNodes(availableNodes)
    {
        Ui::NodeResultExportDialogClass form;
        form.setupUi(this);
        setObjectName(QStringLiteral("nodeResultExportDialog"));
        setModal(true);
        resize(700, 650);
        DialogSizing::lockCurrentHeight(this);
        form.titleLabel->setObjectName(QStringLiteral("exportDialogTitle"));
        form.subtitleLabel->setObjectName(QStringLiteral("exportDialogSubtitle"));
        for (auto* card : {form.nodeCard, form.fieldCard, form.fileCard})
            card->setObjectName(QStringLiteral("exportSectionCard"));
        for (auto* title : {form.nodeTitle, form.fieldTitle, form.fileTitle})
            title->setObjectName(QStringLiteral("exportSectionTitle"));
        for (auto* group : {form.displacementGroup, form.rotationGroup, form.velocityGroup, form.accelerationGroup,
                            form.coordinateGroup, form.reactionGroup})
            group->setObjectName(QStringLiteral("exportFieldGroup"));
        form.nodeHint->setObjectName(QStringLiteral("exportFieldHint"));
        m_nodeEdit = form.nodeEdit;
        auto* currentButton = form.currentNodeButton;
        if (selectedNodeId > 0 && availableNodes.contains(selectedNodeId))
        {
            currentButton->setVisible(true);
            currentButton->setText(QStringLiteral("当前节点 %1").arg(selectedNodeId));
            currentButton->setObjectName(QStringLiteral("exportQuietButton"));
            QObject::connect(currentButton, &QPushButton::clicked, this,
                             [this, selectedNodeId]()
                             {
                                 m_nodeEdit->setText(QString::number(selectedNodeId));
                             });
            m_nodeEdit->setText(QString::number(selectedNodeId));
        }
        const std::initializer_list<FieldCheckBox> fieldDefinitions = {
            {form.u1, EnumKeyword::NodeResultType::U1},   {form.u2, EnumKeyword::NodeResultType::U2},
            {form.u3, EnumKeyword::NodeResultType::U3},   {form.um, EnumKeyword::NodeResultType::MagnitudeU},
            {form.ur1, EnumKeyword::NodeResultType::UR1}, {form.ur2, EnumKeyword::NodeResultType::UR2},
            {form.ur3, EnumKeyword::NodeResultType::UR3}, {form.v1, EnumKeyword::NodeResultType::V1},
            {form.v2, EnumKeyword::NodeResultType::V2},   {form.v3, EnumKeyword::NodeResultType::V3},
            {form.a1, EnumKeyword::NodeResultType::A1},   {form.a2, EnumKeyword::NodeResultType::A2},
            {form.a3, EnumKeyword::NodeResultType::A3},   {form.cx, EnumKeyword::NodeResultType::CX},
            {form.cy, EnumKeyword::NodeResultType::CY},   {form.cz, EnumKeyword::NodeResultType::CZ},
            {form.r1, EnumKeyword::NodeResultType::R1},   {form.r2, EnumKeyword::NodeResultType::R2},
            {form.r3, EnumKeyword::NodeResultType::R3}};
        for (const auto& field : fieldDefinitions)
        {
            m_fields.push_back(field);
            QObject::connect(field.box, &QCheckBox::toggled, this,
                             [this]()
                             {
                                 updateValidation();
                             });
        }

        m_fileEdit = form.fileEdit;
        const QFileInfo resultInfo(resultFile);
        m_fileEdit->setText(QDir(ApplicationPaths::resultExportDirectory())
                                .absoluteFilePath(resultInfo.completeBaseName() + QStringLiteral("_节点时程.bdf")));
        m_fileEdit->setCursorPosition(0);
        auto* browseButton = form.browseButton;
        browseButton->setObjectName(QStringLiteral("exportQuietButton"));
        m_errorLabel = form.errorLabel;
        m_errorLabel->setObjectName(QStringLiteral("exportValidationLabel"));
        auto* buttons = form.buttonBox;
        buttons->setObjectName(QStringLiteral("exportDialogButtons"));
        m_exportButton = buttons->button(QDialogButtonBox::Save);
        m_exportButton->setText(QStringLiteral("导出"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));

        QObject::connect(m_nodeEdit, &QLineEdit::textChanged, this,
                         [this]()
                         {
                             updateValidation();
                         });
        QObject::connect(m_fileEdit, &QLineEdit::textChanged, this,
                         [this]()
                         {
                             updateValidation();
                         });
        QObject::connect(browseButton, &QPushButton::clicked, this,
                         [this]()
                         {
                             const QString selected =
                                 QFileDialog::getSaveFileName(this, QStringLiteral("保存节点结果"), m_fileEdit->text(),
                                                              QStringLiteral("BDF 结果文件 (*.bdf);;文本文件 (*.txt)"),
                                                              nullptr, QFileDialog::DontUseNativeDialog);
                             if (!selected.isEmpty())
                                 m_fileEdit->setText(selected);
                         });
        QObject::connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        QObject::connect(buttons, &QDialogButtonBox::accepted, this,
                         [this]()
                         {
                             if (updateValidation())
                                 QDialog::accept();
                         });
        updateValidation();
    }

    std::vector<int> nodeIds() const
    {
        QString error;
        return parseNodeIds(error);
    }

    std::vector<EnumKeyword::NodeResultType> resultTypes() const
    {
        std::vector<EnumKeyword::NodeResultType> selected;
        for (const auto& field : m_fields)
        {
            if (field.box->isChecked())
                selected.push_back(field.type);
        }
        return selected;
    }

    QString outputFile() const
    {
        return QFileInfo(m_fileEdit->text().trimmed()).absoluteFilePath();
    }

private:
    struct FieldCheckBox
    {
        QCheckBox* box = nullptr;
        EnumKeyword::NodeResultType type = EnumKeyword::NodeResultType::U1;
    };

    std::vector<int> parseNodeIds(_OUT QString& error) const
    {
        QSet<int> ids;
        const QStringList tokens =
            m_nodeEdit->text().split(QRegularExpression(QStringLiteral("[,，;；、\\s]+")), Qt::SkipEmptyParts);
        const QRegularExpression rangePattern(QStringLiteral("^(\\d+)\\s*[-~～]\\s*(\\d+)$"));
        for (const QString& token : tokens)
        {
            const auto match = rangePattern.match(token);
            if (match.hasMatch())
            {
                const int first = match.captured(1).toInt();
                const int last = match.captured(2).toInt();
                if (first <= 0 || last < first || last - first > 100000)
                {
                    error = QStringLiteral("节点范围“%1”无效").arg(token);
                    return {};
                }
                for (int id = first; id <= last; ++id)
                    ids.insert(id);
            }
            else
            {
                bool ok = false;
                const int id = token.toInt(&ok);
                if (!ok || id <= 0)
                {
                    error = QStringLiteral("无法识别节点“%1”").arg(token);
                    return {};
                }
                ids.insert(id);
            }
        }

        QList<int> missing;
        for (int id : ids)
        {
            if (!m_availableNodes.contains(id))
                missing.append(id);
        }
        std::sort(missing.begin(), missing.end());
        if (!missing.isEmpty())
        {
            QStringList values;
            for (int index = 0; index < qMin(6, missing.size()); ++index)
                values.append(QString::number(missing.at(index)));
            error = QStringLiteral("模型中不存在节点：%1%2")
                        .arg(values.join(QStringLiteral(", ")), missing.size() > 6 ? QStringLiteral(" …") : QString());
            return {};
        }

        QList<int> sorted = ids.values();
        std::sort(sorted.begin(), sorted.end());
        return std::vector<int>(sorted.begin(), sorted.end());
    }

    bool updateValidation()
    {
        QString error;
        const auto ids = parseNodeIds(error);
        if (error.isEmpty() && ids.empty())
            error = QStringLiteral("请输入至少一个节点编号");
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

    QSet<int> m_availableNodes;
    QLineEdit* m_nodeEdit = nullptr;
    QLineEdit* m_fileEdit = nullptr;
    QLabel* m_errorLabel = nullptr;
    QPushButton* m_exportButton = nullptr;
    std::vector<FieldCheckBox> m_fields;
};
