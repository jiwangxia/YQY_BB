#include "NodeResultExportDialog.h"
#include <QVariant>
#include <QAbstractButton>
#include <QApplication>
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include "Utility/ResultExportPaths.h"
#include "Base/EmptyOUT.h"
#include "GUI/Dialogs/FileDialogService.h"
#include "Utility/EnumKeyword.h"
#include "Utility/UiIcons.h"
#include "Widgets/DialogSizing.h"
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSignalBlocker>
#include <QRegularExpression>
#include <QSet>
#include <algorithm>
#include <vector>

NodeResultExportDialog::NodeResultExportDialog(const QString& resultFile, const QSet<int>& availableNodes,
                                               int selectedNodeId, QWidget* parent)
    : QDialog(parent)
    , m_availableNodes(availableNodes)
    , m_resultFile(resultFile)
{
    initializeUi();
    setObjectName(QStringLiteral("nodeResultExportDialog"));
    setWindowIcon(UiIcons::applicationIcon());
    setModal(true);
    resize(700, 590);
    DialogSizing::lockHeightToContents(this);
    m_titleLabel->setObjectName(QStringLiteral("exportDialogTitle"));
    m_subtitleLabel->setObjectName(QStringLiteral("exportDialogSubtitle"));
    for (auto* card : {m_nodeCard, m_fieldCard, m_fileCard})
        card->setObjectName(QStringLiteral("exportSectionCard"));
    for (auto* title : {m_nodeTitle, m_fieldTitle, m_fileTitle})
        title->setObjectName(QStringLiteral("exportSectionTitle"));
    for (auto* group : {m_displacementGroup, m_rotationGroup, m_velocityGroup, m_accelerationGroup, m_coordinateGroup,
                        m_reactionGroup})
        group->setObjectName(QStringLiteral("exportFieldGroup"));
    m_nodeHint->setObjectName(QStringLiteral("exportFieldHint"));
    m_nodeHint->setWordWrap(true);
    m_subtitleLabel->setWordWrap(true);
    m_nodeEdit->setAccessibleName(QStringLiteral("节点编号"));
    auto* currentButton = m_currentNodeButton;
    currentButton->setAutoDefault(false);
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
        {m_u1, EnumKeyword::NodeResultType::U1},   {m_u2, EnumKeyword::NodeResultType::U2},
        {m_u3, EnumKeyword::NodeResultType::U3},   {m_um, EnumKeyword::NodeResultType::MagnitudeU},
        {m_ur1, EnumKeyword::NodeResultType::UR1}, {m_ur2, EnumKeyword::NodeResultType::UR2},
        {m_ur3, EnumKeyword::NodeResultType::UR3}, {m_v1, EnumKeyword::NodeResultType::V1},
        {m_v2, EnumKeyword::NodeResultType::V2},   {m_v3, EnumKeyword::NodeResultType::V3},
        {m_a1, EnumKeyword::NodeResultType::A1},   {m_a2, EnumKeyword::NodeResultType::A2},
        {m_a3, EnumKeyword::NodeResultType::A3},   {m_cx, EnumKeyword::NodeResultType::CX},
        {m_cy, EnumKeyword::NodeResultType::CY},   {m_cz, EnumKeyword::NodeResultType::CZ},
        {m_r1, EnumKeyword::NodeResultType::R1},   {m_r2, EnumKeyword::NodeResultType::R2},
        {m_r3, EnumKeyword::NodeResultType::R3}};
    for (const auto& field : fieldDefinitions)
    {
        m_fields.push_back(field);
        QObject::connect(field.box, &QCheckBox::toggled, this,
                         [this]()
                         {
                             updateValidation();
                         });
    }

    updateDefaultFile();
    m_fileEdit->setCursorPosition(m_fileEdit->text().size());
    m_fileEdit->setToolTip(m_fileEdit->text());
    m_fileEdit->setAccessibleName(QStringLiteral("输出文件路径"));
    auto* browseButton = m_browseButton;
    browseButton->setObjectName(QStringLiteral("exportQuietButton"));
    browseButton->setAutoDefault(false);
    m_errorLabel->setObjectName(QStringLiteral("exportValidationLabel"));
    m_errorLabel->setProperty("dialogFooter", true);
    m_errorLabel->setWordWrap(true);
    auto* buttons = m_buttonBox;
    buttons->setObjectName(QStringLiteral("exportDialogButtons"));
    m_exportButton = buttons->button(QDialogButtonBox::Save);
    m_exportButton->setText(QStringLiteral("导出"));
    m_exportButton->setProperty("primaryAction", true);
    m_exportButton->setDefault(true);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);

    QObject::connect(m_nodeEdit, &QLineEdit::textChanged, this,
                     [this]()
                     {
                         updateValidation();
                     });
    QObject::connect(m_fileEdit, &QLineEdit::textChanged, this,
                     [this]()
                     {
                         m_fileEdit->setToolTip(m_fileEdit->text());
                         updateValidation();
                     });
    QObject::connect(browseButton, &QPushButton::clicked, this,
                     [this]()
                     {
                         const QString selected =
                             FileDialogService::selectSaveFile(this, QStringLiteral("保存节点结果"), m_fileEdit->text(),
                                                               QStringLiteral("BDF 结果文件 (*.bdf);;文本文件 (*.txt)"),
                                                               QStringLiteral("nodeResultExport"), false);
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

std::vector<int> NodeResultExportDialog::nodeIds() const
{
    QString error;
    return parseNodeIds(error);
}

std::vector<EnumKeyword::NodeResultType> NodeResultExportDialog::resultTypes() const
{
    std::vector<EnumKeyword::NodeResultType> selected;
    for (const auto& field : m_fields)
    {
        if (field.box->isChecked())
            selected.push_back(field.type);
    }
    return selected;
}

QString NodeResultExportDialog::outputFile() const
{
    return QFileInfo(m_fileEdit->text().trimmed()).absoluteFilePath();
}

std::vector<int> NodeResultExportDialog::parseNodeIds(_OUT QString& error) const
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

void NodeResultExportDialog::updateDefaultFile()
{
    if (!m_fileEdit || (!m_defaultFile.isEmpty() && m_fileEdit->text() != m_defaultFile))
        return;
    QString error;
    const auto ids = parseNodeIds(error);
    QString category;
    const std::pair<QGroupBox*, QString> groups[] = {
        {m_displacementGroup, QStringLiteral("位移")}, {m_rotationGroup, QStringLiteral("转角")},
        {m_velocityGroup, QStringLiteral("速度")},     {m_accelerationGroup, QStringLiteral("加速度")},
        {m_coordinateGroup, QStringLiteral("坐标")},   {m_reactionGroup, QStringLiteral("反力")}};
    for (const auto& group : groups)
    {
        const auto checks = group.first->findChildren<QCheckBox*>();
        if (!std::any_of(checks.begin(), checks.end(),
                         [](QCheckBox* box)
                         {
                             return box->isChecked();
                         }))
            continue;
        if (!category.isEmpty())
        {
            category = QStringLiteral("结果");
            break;
        }
        category = group.second;
    }
    if (category.isEmpty())
        category = QStringLiteral("结果");
    const QString selection = ids.size() == 1  ? QStringLiteral("节点%1").arg(ids.front())
                              : ids.size() > 1 ? QStringLiteral("多节点")
                                               : QStringLiteral("节点");
    const QString description = selection + category;
    m_defaultFile = ResultExportPaths::filePath(m_resultFile, description, QStringLiteral("bdf"));
    const QSignalBlocker blocker(m_fileEdit);
    m_fileEdit->setText(m_defaultFile);
    m_fileEdit->setToolTip(m_defaultFile);
    m_fileEdit->setCursorPosition(m_defaultFile.size());
}

bool NodeResultExportDialog::updateValidation()
{
    updateDefaultFile();
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
    const bool hasError = !error.isEmpty();
    const bool wasVisible = !m_errorLabel->isHidden();
    m_errorLabel->setText(error);
    m_errorLabel->setVisible(hasError);
    if (wasVisible != hasError && isVisible())
        DialogSizing::lockHeightToContents(this);
    m_exportButton->setEnabled(error.isEmpty());
    return error.isEmpty();
}

void NodeResultExportDialog::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("NodeResultExportDialogClass");
    }
    setMinimumSize(QSize(520, 0));
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setSpacing(12);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(20, 18, 20, 18);
    m_titleLabel = new QLabel(this);
    m_titleLabel->setObjectName("titleLabel");

    rootLayout->addWidget(m_titleLabel);

    m_subtitleLabel = new QLabel(this);
    m_subtitleLabel->setObjectName("subtitleLabel");
    m_subtitleLabel->setWordWrap(true);

    rootLayout->addWidget(m_subtitleLabel);

    m_nodeCard = new QFrame(this);
    m_nodeCard->setObjectName("nodeCard");
    QVBoxLayout* nodeCardLayout = new QVBoxLayout(m_nodeCard);
    nodeCardLayout->setSpacing(8);
    nodeCardLayout->setObjectName("nodeCardLayout");
    nodeCardLayout->setContentsMargins(12, 10, 12, 10);
    m_nodeTitle = new QLabel(m_nodeCard);
    m_nodeTitle->setObjectName("nodeTitle");

    nodeCardLayout->addWidget(m_nodeTitle);

    QHBoxLayout* nodeRow = new QHBoxLayout();
    nodeRow->setSpacing(8);
    nodeRow->setObjectName("nodeRow");
    m_nodeEdit = new QLineEdit(m_nodeCard);
    m_nodeEdit->setObjectName("nodeEdit");
    m_nodeEdit->setClearButtonEnabled(true);

    nodeRow->addWidget(m_nodeEdit);

    m_currentNodeButton = new QPushButton(m_nodeCard);
    m_currentNodeButton->setObjectName("currentNodeButton");
    m_currentNodeButton->setVisible(false);

    nodeRow->addWidget(m_currentNodeButton);

    nodeCardLayout->addLayout(nodeRow);

    m_nodeHint = new QLabel(m_nodeCard);
    m_nodeHint->setObjectName("nodeHint");
    m_nodeHint->setWordWrap(true);

    nodeCardLayout->addWidget(m_nodeHint);

    rootLayout->addWidget(m_nodeCard);

    m_fieldCard = new QFrame(this);
    m_fieldCard->setObjectName("fieldCard");
    QVBoxLayout* fieldCardLayout = new QVBoxLayout(m_fieldCard);
    fieldCardLayout->setSpacing(8);
    fieldCardLayout->setObjectName("fieldCardLayout");
    fieldCardLayout->setContentsMargins(12, 10, 12, 10);
    m_fieldTitle = new QLabel(m_fieldCard);
    m_fieldTitle->setObjectName("fieldTitle");

    fieldCardLayout->addWidget(m_fieldTitle);

    QGridLayout* fieldGrid = new QGridLayout();
    fieldGrid->setObjectName("fieldGrid");
    fieldGrid->setHorizontalSpacing(10);
    fieldGrid->setVerticalSpacing(10);
    m_displacementGroup = new QGroupBox(m_fieldCard);
    m_displacementGroup->setObjectName("displacementGroup");
    QHBoxLayout* hboxLayout = new QHBoxLayout(m_displacementGroup);
    hboxLayout->setSpacing(10);
    hboxLayout->setObjectName("hboxLayout");
    hboxLayout->setContentsMargins(8, 6, 8, 6);
    m_u1 = new QCheckBox(m_displacementGroup);
    m_u1->setObjectName("u1");
    m_u1->setChecked(true);

    hboxLayout->addWidget(m_u1);

    m_u2 = new QCheckBox(m_displacementGroup);
    m_u2->setObjectName("u2");
    m_u2->setChecked(true);

    hboxLayout->addWidget(m_u2);

    m_u3 = new QCheckBox(m_displacementGroup);
    m_u3->setObjectName("u3");
    m_u3->setChecked(true);

    hboxLayout->addWidget(m_u3);

    m_um = new QCheckBox(m_displacementGroup);
    m_um->setObjectName("um");
    m_um->setChecked(false);

    hboxLayout->addWidget(m_um);

    fieldGrid->addWidget(m_displacementGroup, 0, 0, 1, 1);

    m_rotationGroup = new QGroupBox(m_fieldCard);
    m_rotationGroup->setObjectName("rotationGroup");
    QHBoxLayout* hboxLayout1 = new QHBoxLayout(m_rotationGroup);
    hboxLayout1->setSpacing(10);
    hboxLayout1->setObjectName("hboxLayout1");
    hboxLayout1->setContentsMargins(8, 6, 8, 6);
    m_ur1 = new QCheckBox(m_rotationGroup);
    m_ur1->setObjectName("ur1");

    hboxLayout1->addWidget(m_ur1);

    m_ur2 = new QCheckBox(m_rotationGroup);
    m_ur2->setObjectName("ur2");

    hboxLayout1->addWidget(m_ur2);

    m_ur3 = new QCheckBox(m_rotationGroup);
    m_ur3->setObjectName("ur3");

    hboxLayout1->addWidget(m_ur3);

    fieldGrid->addWidget(m_rotationGroup, 0, 1, 1, 1);

    m_velocityGroup = new QGroupBox(m_fieldCard);
    m_velocityGroup->setObjectName("velocityGroup");
    QHBoxLayout* hboxLayout2 = new QHBoxLayout(m_velocityGroup);
    hboxLayout2->setSpacing(10);
    hboxLayout2->setObjectName("hboxLayout2");
    hboxLayout2->setContentsMargins(8, 6, 8, 6);
    m_v1 = new QCheckBox(m_velocityGroup);
    m_v1->setObjectName("v1");

    hboxLayout2->addWidget(m_v1);

    m_v2 = new QCheckBox(m_velocityGroup);
    m_v2->setObjectName("v2");

    hboxLayout2->addWidget(m_v2);

    m_v3 = new QCheckBox(m_velocityGroup);
    m_v3->setObjectName("v3");

    hboxLayout2->addWidget(m_v3);

    fieldGrid->addWidget(m_velocityGroup, 1, 0, 1, 1);

    m_accelerationGroup = new QGroupBox(m_fieldCard);
    m_accelerationGroup->setObjectName("accelerationGroup");
    QHBoxLayout* hboxLayout3 = new QHBoxLayout(m_accelerationGroup);
    hboxLayout3->setSpacing(10);
    hboxLayout3->setObjectName("hboxLayout3");
    hboxLayout3->setContentsMargins(8, 6, 8, 6);
    m_a1 = new QCheckBox(m_accelerationGroup);
    m_a1->setObjectName("a1");

    hboxLayout3->addWidget(m_a1);

    m_a2 = new QCheckBox(m_accelerationGroup);
    m_a2->setObjectName("a2");

    hboxLayout3->addWidget(m_a2);

    m_a3 = new QCheckBox(m_accelerationGroup);
    m_a3->setObjectName("a3");

    hboxLayout3->addWidget(m_a3);

    fieldGrid->addWidget(m_accelerationGroup, 1, 1, 1, 1);

    m_coordinateGroup = new QGroupBox(m_fieldCard);
    m_coordinateGroup->setObjectName("coordinateGroup");
    QHBoxLayout* hboxLayout4 = new QHBoxLayout(m_coordinateGroup);
    hboxLayout4->setSpacing(10);
    hboxLayout4->setObjectName("hboxLayout4");
    hboxLayout4->setContentsMargins(8, 6, 8, 6);
    m_cx = new QCheckBox(m_coordinateGroup);
    m_cx->setObjectName("cx");

    hboxLayout4->addWidget(m_cx);

    m_cy = new QCheckBox(m_coordinateGroup);
    m_cy->setObjectName("cy");

    hboxLayout4->addWidget(m_cy);

    m_cz = new QCheckBox(m_coordinateGroup);
    m_cz->setObjectName("cz");

    hboxLayout4->addWidget(m_cz);

    fieldGrid->addWidget(m_coordinateGroup, 2, 0, 1, 1);

    m_reactionGroup = new QGroupBox(m_fieldCard);
    m_reactionGroup->setObjectName("reactionGroup");
    QHBoxLayout* hboxLayout5 = new QHBoxLayout(m_reactionGroup);
    hboxLayout5->setSpacing(10);
    hboxLayout5->setObjectName("hboxLayout5");
    hboxLayout5->setContentsMargins(8, 6, 8, 6);
    m_r1 = new QCheckBox(m_reactionGroup);
    m_r1->setObjectName("r1");

    hboxLayout5->addWidget(m_r1);

    m_r2 = new QCheckBox(m_reactionGroup);
    m_r2->setObjectName("r2");

    hboxLayout5->addWidget(m_r2);

    m_r3 = new QCheckBox(m_reactionGroup);
    m_r3->setObjectName("r3");

    hboxLayout5->addWidget(m_r3);

    fieldGrid->addWidget(m_reactionGroup, 2, 1, 1, 1);

    fieldCardLayout->addLayout(fieldGrid);

    rootLayout->addWidget(m_fieldCard);

    m_fileCard = new QFrame(this);
    m_fileCard->setObjectName("fileCard");
    QVBoxLayout* fileCardLayout = new QVBoxLayout(m_fileCard);
    fileCardLayout->setSpacing(8);
    fileCardLayout->setObjectName("fileCardLayout");
    fileCardLayout->setContentsMargins(12, 10, 12, 10);
    m_fileTitle = new QLabel(m_fileCard);
    m_fileTitle->setObjectName("fileTitle");

    fileCardLayout->addWidget(m_fileTitle);

    QHBoxLayout* fileRow = new QHBoxLayout();
    fileRow->setSpacing(8);
    fileRow->setObjectName("fileRow");
    m_fileEdit = new QLineEdit(m_fileCard);
    m_fileEdit->setObjectName("fileEdit");

    fileRow->addWidget(m_fileEdit);

    m_browseButton = new QPushButton(m_fileCard);
    m_browseButton->setObjectName("browseButton");

    fileRow->addWidget(m_browseButton);

    fileCardLayout->addLayout(fileRow);

    rootLayout->addWidget(m_fileCard);

    m_errorLabel = new QLabel(this);
    m_errorLabel->setObjectName("errorLabel");
    m_errorLabel->setMinimumSize(QSize(0, 20));
    m_errorLabel->setWordWrap(true);

    rootLayout->addWidget(m_errorLabel);

    m_buttonBox = new QDialogButtonBox(this);
    m_buttonBox->setObjectName("buttonBox");
    m_buttonBox->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Save);

    rootLayout->addWidget(m_buttonBox);

    setWindowTitle(QCoreApplication::translate("NodeResultExportDialogClass", "导出节点结果", nullptr));
    m_titleLabel->setText(QCoreApplication::translate("NodeResultExportDialogClass", "导出节点时程数据", nullptr));
    m_subtitleLabel->setText(
        QCoreApplication::translate("NodeResultExportDialogClass", "从当前 H5 结果中选择节点和结果分量", nullptr));
    m_nodeTitle->setText(QCoreApplication::translate("NodeResultExportDialogClass", "1  选择节点", nullptr));
    m_nodeEdit->setPlaceholderText(
        QCoreApplication::translate("NodeResultExportDialogClass", "例如：1, 3, 8-12", nullptr));
    m_currentNodeButton->setText(QCoreApplication::translate("NodeResultExportDialogClass", "当前节点", nullptr));
    m_nodeHint->setText(QCoreApplication::translate("NodeResultExportDialogClass",
                                                    "支持逗号、空格和连续范围；导出前会检查节点是否存在。", nullptr));
    m_fieldTitle->setText(QCoreApplication::translate("NodeResultExportDialogClass", "2  选择数据", nullptr));
    m_displacementGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "位移", nullptr));
    m_u1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "U1", nullptr));
    m_u2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "U2", nullptr));
    m_u3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "U3", nullptr));
    m_um->setText(QCoreApplication::translate("NodeResultExportDialogClass", "|U|", nullptr));
    m_rotationGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "转角", nullptr));
    m_ur1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "UR1", nullptr));
    m_ur2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "UR2", nullptr));
    m_ur3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "UR3", nullptr));
    m_velocityGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "速度", nullptr));
    m_v1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "V1", nullptr));
    m_v2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "V2", nullptr));
    m_v3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "V3", nullptr));
    m_accelerationGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "加速度", nullptr));
    m_a1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "A1", nullptr));
    m_a2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "A2", nullptr));
    m_a3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "A3", nullptr));
    m_coordinateGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "当前坐标", nullptr));
    m_cx->setText(QCoreApplication::translate("NodeResultExportDialogClass", "CX", nullptr));
    m_cy->setText(QCoreApplication::translate("NodeResultExportDialogClass", "CY", nullptr));
    m_cz->setText(QCoreApplication::translate("NodeResultExportDialogClass", "CZ", nullptr));
    m_reactionGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "节点反力", nullptr));
    m_r1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "R1", nullptr));
    m_r2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "R2", nullptr));
    m_r3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "R3", nullptr));
    m_fileTitle->setText(QCoreApplication::translate("NodeResultExportDialogClass", "3  保存文件", nullptr));
    m_fileEdit->setPlaceholderText(
        QCoreApplication::translate("NodeResultExportDialogClass", "选择输出文件路径", nullptr));
    m_browseButton->setText(QCoreApplication::translate("NodeResultExportDialogClass", "浏览…", nullptr));
}
