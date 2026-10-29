#include "ResultPlotExportDialog.h"
#include "Utility/ResultExportPaths.h"
#include "GUI/Dialogs/FileDialogService.h"
#include "Utility/UiIcons.h"
#include "Widgets/DialogSizing.h"
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QSignalBlocker>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSet>
#include <QVBoxLayout>
#include <algorithm>

ResultPlotExportDialog::ResultPlotExportDialog(const QString& resultFile, const QSet<int>& availableNodes,
                                               int selectedNodeId, QWidget* parent)
    : QDialog(parent)
    , m_availableNodes(availableNodes)
    , m_resultFile(resultFile)
{
    setObjectName(QStringLiteral("resultPlotExportDialog"));
    setModal(true);
    setWindowTitle(QStringLiteral("导出结果曲线"));
    setWindowIcon(UiIcons::applicationIcon());
    resize(640, 460);
    DialogSizing::lockHeightToContents(this);

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(20, 18, 20, 18);
    rootLayout->setSpacing(12);

    auto* title = new QLabel(QStringLiteral("导出结果曲线"), this);
    title->setObjectName(QStringLiteral("plotDialogTitle"));
    rootLayout->addWidget(title);

    auto* subtitle = new QLabel(QStringLiteral("选择节点和时程分量，程序将先导出 BDF，再生成 PNG/PDF 曲线图。"), this);
    subtitle->setObjectName(QStringLiteral("plotDialogSubtitle"));
    subtitle->setWordWrap(true);
    rootLayout->addWidget(subtitle);

    auto* nodeCard = new QGroupBox(QStringLiteral("1  选择节点"), this);
    nodeCard->setObjectName(QStringLiteral("resultPlotSectionCard"));
    auto* nodeLayout = new QVBoxLayout(nodeCard);
    m_nodeEdit = new QLineEdit(nodeCard);
    m_nodeEdit->setPlaceholderText(QStringLiteral("输入节点编号，例如：27"));
    m_nodeEdit->setClearButtonEnabled(true);
    m_nodeEdit->setAccessibleName(QStringLiteral("节点编号"));
    nodeLayout->addWidget(m_nodeEdit);
    rootLayout->addWidget(nodeCard);

    auto* dataCard = new QGroupBox(QStringLiteral("2  选择数据"), this);
    dataCard->setObjectName(QStringLiteral("resultPlotSectionCard"));
    auto* dataLayout = new QGridLayout(dataCard);
    dataLayout->setHorizontalSpacing(24);
    dataLayout->setVerticalSpacing(10);
    auto* displacementLabel = new QLabel(QStringLiteral("位移"), dataCard);
    auto* rotationLabel = new QLabel(QStringLiteral("转角"), dataCard);
    displacementLabel->setProperty("secondaryText", true);
    rotationLabel->setProperty("secondaryText", true);
    dataLayout->addWidget(displacementLabel, 0, 0);
    dataLayout->addWidget(rotationLabel, 0, 1);
    m_ux = new QCheckBox(QStringLiteral("X 向位移 ux"), dataCard);
    m_uy = new QCheckBox(QStringLiteral("Y 向位移 uy"), dataCard);
    m_uz = new QCheckBox(QStringLiteral("Z 向位移 uz"), dataCard);
    m_rx = new QCheckBox(QStringLiteral("绕 X 转角 rx"), dataCard);
    m_ry = new QCheckBox(QStringLiteral("绕 Y 转角 ry"), dataCard);
    m_rz = new QCheckBox(QStringLiteral("绕 Z 转角 rz"), dataCard);
    m_uy->setChecked(true);
    m_uz->setChecked(true);
    m_rx->setChecked(true);
    dataLayout->addWidget(m_ux, 1, 0);
    dataLayout->addWidget(m_uy, 2, 0);
    dataLayout->addWidget(m_uz, 3, 0);
    dataLayout->addWidget(m_rx, 1, 1);
    dataLayout->addWidget(m_ry, 2, 1);
    dataLayout->addWidget(m_rz, 3, 1);
    dataLayout->setColumnStretch(0, 1);
    dataLayout->setColumnStretch(1, 1);
    rootLayout->addWidget(dataCard);

    auto* fileCard = new QGroupBox(QStringLiteral("3  保存文件"), this);
    fileCard->setObjectName(QStringLiteral("resultPlotSectionCard"));
    auto* fileLayout = new QHBoxLayout(fileCard);
    m_fileEdit = new QLineEdit(fileCard);
    m_fileEdit->setAccessibleName(QStringLiteral("BDF 输出文件路径"));
    auto* browseButton = new QPushButton(QStringLiteral("浏览…"), fileCard);
    browseButton->setObjectName(QStringLiteral("plotDialogQuietButton"));
    browseButton->setAutoDefault(false);
    fileLayout->addWidget(m_fileEdit);
    fileLayout->addWidget(browseButton);
    rootLayout->addWidget(fileCard);

    m_errorLabel = new QLabel(this);
    m_errorLabel->setObjectName(QStringLiteral("plotDialogErrorLabel"));
    m_errorLabel->setProperty("dialogFooter", true);
    m_errorLabel->setWordWrap(true);
    m_errorLabel->setMinimumHeight(fontMetrics().lineSpacing());
    rootLayout->addWidget(m_errorLabel);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    buttons->setObjectName(QStringLiteral("plotDialogButtons"));
    buttons->button(QDialogButtonBox::Save)->setText(QStringLiteral("导出并生成"));
    buttons->button(QDialogButtonBox::Save)->setProperty("primaryAction", true);
    buttons->button(QDialogButtonBox::Save)->setDefault(true);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    rootLayout->addWidget(buttons);

    QList<int> sortedNodes = m_availableNodes.values();
    std::sort(sortedNodes.begin(), sortedNodes.end());
    const int defaultNode =
        m_availableNodes.contains(selectedNodeId) ? selectedNodeId : (sortedNodes.isEmpty() ? -1 : sortedNodes.front());
    if (defaultNode > 0)
        m_nodeEdit->setText(QString::number(defaultNode));

    updateDefaultFile();
    m_fileEdit->setCursorPosition(m_fileEdit->text().size());
    m_fileEdit->setToolTip(m_fileEdit->text());

    connect(m_nodeEdit, &QLineEdit::textChanged, this,
            [this]()
            {
                updateValidation();
            });
    connect(m_fileEdit, &QLineEdit::textChanged, this,
            [this]()
            {
                m_fileEdit->setToolTip(m_fileEdit->text());
                updateValidation();
            });
    for (auto* checkBox : {m_ux, m_uy, m_uz, m_rx, m_ry, m_rz})
        connect(checkBox, &QCheckBox::toggled, this,
                [this]()
                {
                    updateValidation();
                });
    connect(browseButton, &QPushButton::clicked, this,
            [this]()
            {
                const QString selected = FileDialogService::selectSaveFile(
                    this, QStringLiteral("保存时程 BDF"), m_fileEdit->text(),
                    QStringLiteral("BDF 数据文件 (*.bdf);;所有文件 (*.*)"), QStringLiteral("resultPlotExport"), false);
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

ResultPlotSelection ResultPlotExportDialog::selection() const
{
    ResultPlotSelection result;
    result.nodeId = m_nodeEdit->text().trimmed().toInt();
    result.displacementX = m_ux->isChecked();
    result.displacementY = m_uy->isChecked();
    result.displacementZ = m_uz->isChecked();
    result.rotationX = m_rx->isChecked();
    result.rotationY = m_ry->isChecked();
    result.rotationZ = m_rz->isChecked();
    result.outputFile = QFileInfo(m_fileEdit->text().trimmed()).absoluteFilePath();
    return result;
}

void ResultPlotExportDialog::updateDefaultFile()
{
    if (!m_fileEdit || (!m_defaultFile.isEmpty() && m_fileEdit->text() != m_defaultFile))
        return;
    const int nodeId = m_nodeEdit->text().trimmed().toInt();
    const bool displacement = m_ux->isChecked() || m_uy->isChecked() || m_uz->isChecked();
    const bool rotation = m_rx->isChecked() || m_ry->isChecked() || m_rz->isChecked();
    const QString category = displacement && !rotation   ? QStringLiteral("位移")
                             : rotation && !displacement ? QStringLiteral("转角")
                                                         : QStringLiteral("时程");
    const QString description = (nodeId > 0 ? QStringLiteral("节点%1").arg(nodeId) : QStringLiteral("节点")) + category;
    m_defaultFile = ResultExportPaths::filePath(m_resultFile, description, QStringLiteral("bdf"));
    const QSignalBlocker blocker(m_fileEdit);
    m_fileEdit->setText(m_defaultFile);
    m_fileEdit->setToolTip(m_defaultFile);
    m_fileEdit->setCursorPosition(m_defaultFile.size());
}

bool ResultPlotExportDialog::updateValidation()
{
    updateDefaultFile();
    bool nodeOk = false;
    const int nodeId = m_nodeEdit->text().trimmed().toInt(&nodeOk);
    QString error;
    if (!nodeOk || nodeId <= 0 || !m_availableNodes.contains(nodeId))
        error = QStringLiteral("请输入模型中存在的有效节点编号");
    else if (!m_ux->isChecked() && !m_uy->isChecked() && !m_uz->isChecked() && !m_rx->isChecked() &&
             !m_ry->isChecked() && !m_rz->isChecked())
        error = QStringLiteral("请至少选择一个结果分量");
    else if (m_fileEdit->text().trimmed().isEmpty())
        error = QStringLiteral("请选择 BDF 输出文件");
    else if (!QDir(QFileInfo(m_fileEdit->text().trimmed()).absolutePath()).exists())
        error = QStringLiteral("输出文件夹不存在");

    const bool hasError = !error.isEmpty();
    const bool wasVisible = !m_errorLabel->isHidden();
    m_errorLabel->setText(error);
    m_errorLabel->setVisible(hasError);
    if (wasVisible != hasError && isVisible())
        DialogSizing::lockHeightToContents(this);
    auto* saveButton = findChild<QDialogButtonBox*>(QStringLiteral("plotDialogButtons"))
                           ->button(QDialogButtonBox::Save);
    saveButton->setEnabled(error.isEmpty());
    return error.isEmpty();
}
