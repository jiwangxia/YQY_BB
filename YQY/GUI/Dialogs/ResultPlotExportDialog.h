#pragma once

#include "Application/ApplicationPaths.h"
#include "GUI/Dialogs/FileDialogService.h"
#include "Widgets/DialogSizing.h"

#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSet>
#include <QVBoxLayout>

#include <algorithm>

struct ResultPlotSelection
{
    int nodeId = -1;
    bool displacementX = false;
    bool displacementY = true;
    bool displacementZ = true;
    bool twist = true;
    bool twistRate = false;
    QString outputFile;
};

class ResultPlotExportDialog final : public QDialog
{
public:
    ResultPlotExportDialog(const QString& resultFile, const QSet<int>& availableNodes, int selectedNodeId,
                           QWidget* parent)
        : QDialog(parent)
        , m_availableNodes(availableNodes)
    {
        setObjectName(QStringLiteral("resultPlotExportDialog"));
        setModal(true);
        setWindowTitle(QStringLiteral("导出结果曲线"));
        resize(620, 460);
        DialogSizing::lockHeightToContents(this);

        auto* rootLayout = new QVBoxLayout(this);
        rootLayout->setContentsMargins(22, 20, 22, 20);
        rootLayout->setSpacing(12);

        auto* title = new QLabel(QStringLiteral("导出结果曲线"), this);
        title->setObjectName(QStringLiteral("plotDialogTitle"));
        rootLayout->addWidget(title);

        auto* subtitle = new QLabel(QStringLiteral("选择节点和时程分量，程序将先导出 CSV，再生成 PNG/PDF 曲线图。"), this);
        subtitle->setObjectName(QStringLiteral("plotDialogSubtitle"));
        subtitle->setWordWrap(true);
        rootLayout->addWidget(subtitle);

        auto* nodeCard = new QGroupBox(QStringLiteral("1  选择节点"), this);
        nodeCard->setObjectName(QStringLiteral("resultPlotSectionCard"));
        auto* nodeLayout = new QVBoxLayout(nodeCard);
        m_nodeEdit = new QLineEdit(nodeCard);
        m_nodeEdit->setPlaceholderText(QStringLiteral("输入节点编号，例如：27"));
        nodeLayout->addWidget(m_nodeEdit);
        rootLayout->addWidget(nodeCard);

        auto* dataCard = new QGroupBox(QStringLiteral("2  选择数据"), this);
        dataCard->setObjectName(QStringLiteral("resultPlotSectionCard"));
        auto* dataLayout = new QFormLayout(dataCard);
        m_ux = new QCheckBox(QStringLiteral("X 向位移 ux"), dataCard);
        m_uy = new QCheckBox(QStringLiteral("Y 向位移 uy"), dataCard);
        m_uz = new QCheckBox(QStringLiteral("Z 向位移 uz"), dataCard);
        m_twist = new QCheckBox(QStringLiteral("扭转角 twist"), dataCard);
        m_twistRate = new QCheckBox(QStringLiteral("扭转角速度 twist_rate"), dataCard);
        m_uy->setChecked(true);
        m_uz->setChecked(true);
        m_twist->setChecked(true);
        dataLayout->addRow(m_ux);
        dataLayout->addRow(m_uy);
        dataLayout->addRow(m_uz);
        dataLayout->addRow(m_twist);
        dataLayout->addRow(m_twistRate);
        rootLayout->addWidget(dataCard);

        auto* fileCard = new QGroupBox(QStringLiteral("3  保存文件"), this);
        fileCard->setObjectName(QStringLiteral("resultPlotSectionCard"));
        auto* fileLayout = new QHBoxLayout(fileCard);
        m_fileEdit = new QLineEdit(fileCard);
        auto* browseButton = new QPushButton(QStringLiteral("浏览…"), fileCard);
        browseButton->setObjectName(QStringLiteral("plotDialogQuietButton"));
        fileLayout->addWidget(m_fileEdit);
        fileLayout->addWidget(browseButton);
        rootLayout->addWidget(fileCard);

        m_errorLabel = new QLabel(this);
        m_errorLabel->setObjectName(QStringLiteral("plotDialogErrorLabel"));
        m_errorLabel->setWordWrap(true);
        rootLayout->addWidget(m_errorLabel);

        auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
        buttons->setObjectName(QStringLiteral("plotDialogButtons"));
        buttons->button(QDialogButtonBox::Save)->setText(QStringLiteral("导出并生成"));
        buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
        rootLayout->addWidget(buttons);

        QList<int> sortedNodes = m_availableNodes.values();
        std::sort(sortedNodes.begin(), sortedNodes.end());
        const int defaultNode = m_availableNodes.contains(selectedNodeId)
                                    ? selectedNodeId
                                    : (sortedNodes.isEmpty() ? -1 : sortedNodes.front());
        if (defaultNode > 0)
            m_nodeEdit->setText(QString::number(defaultNode));

        const QFileInfo resultInfo(resultFile);
        m_fileEdit->setText(QDir(ApplicationPaths::resultExportDirectory(resultFile))
                                .absoluteFilePath(resultInfo.completeBaseName() +
                                                  QStringLiteral("_节点%1_时程.csv").arg(defaultNode > 0 ? defaultNode : 1)));
        m_fileEdit->setCursorPosition(0);

        connect(m_nodeEdit, &QLineEdit::textChanged, this, [this]() { updateValidation(); });
        connect(m_fileEdit, &QLineEdit::textChanged, this, [this]() { updateValidation(); });
        for (auto* checkBox : {m_ux, m_uy, m_uz, m_twist, m_twistRate})
            connect(checkBox, &QCheckBox::toggled, this, [this]() { updateValidation(); });
        connect(browseButton, &QPushButton::clicked, this,
                [this]()
                {
                    const QString selected = FileDialogService::selectSaveFile(
                        this, QStringLiteral("保存时程 CSV"), m_fileEdit->text(),
                        QStringLiteral("CSV 数据文件 (*.csv);;所有文件 (*.*)"), QStringLiteral("resultPlotExport"));
                    if (!selected.isEmpty())
                        m_fileEdit->setText(selected);
                });
        connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(buttons, &QDialogButtonBox::accepted, this,
                [this]()
                {
                    if (updateValidation())
                        accept();
                });
        updateValidation();
    }

    ResultPlotSelection selection() const
    {
        ResultPlotSelection result;
        result.nodeId = m_nodeEdit->text().trimmed().toInt();
        result.displacementX = m_ux->isChecked();
        result.displacementY = m_uy->isChecked();
        result.displacementZ = m_uz->isChecked();
        result.twist = m_twist->isChecked();
        result.twistRate = m_twistRate->isChecked();
        result.outputFile = QFileInfo(m_fileEdit->text().trimmed()).absoluteFilePath();
        return result;
    }

private:
    bool updateValidation()
    {
        bool nodeOk = false;
        const int nodeId = m_nodeEdit->text().trimmed().toInt(&nodeOk);
        QString error;
        if (!nodeOk || nodeId <= 0 || !m_availableNodes.contains(nodeId))
            error = QStringLiteral("请输入模型中存在的有效节点编号");
        else if (!m_ux->isChecked() && !m_uy->isChecked() && !m_uz->isChecked() && !m_twist->isChecked() &&
                 !m_twistRate->isChecked())
            error = QStringLiteral("请至少选择一个结果分量");
        else if (m_fileEdit->text().trimmed().isEmpty())
            error = QStringLiteral("请选择 CSV 输出文件");
        else if (!QDir(QFileInfo(m_fileEdit->text().trimmed()).absolutePath()).exists())
            error = QStringLiteral("输出文件夹不存在");

        m_errorLabel->setText(error);
        m_errorLabel->setVisible(!error.isEmpty());
        auto* saveButton = findChild<QDialogButtonBox*>(QStringLiteral("plotDialogButtons"))
                               ->button(QDialogButtonBox::Save);
        saveButton->setEnabled(error.isEmpty());
        return error.isEmpty();
    }

    QSet<int> m_availableNodes;
    QLineEdit* m_nodeEdit = nullptr;
    QCheckBox* m_ux = nullptr;
    QCheckBox* m_uy = nullptr;
    QCheckBox* m_uz = nullptr;
    QCheckBox* m_twist = nullptr;
    QCheckBox* m_twistRate = nullptr;
    QLineEdit* m_fileEdit = nullptr;
    QLabel* m_errorLabel = nullptr;
};
