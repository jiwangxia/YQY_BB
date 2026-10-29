#include "ElementResultExportDialog.h"
#include "Utility/ResultExportPaths.h"
#include "Base/EmptyOUT.h"
#include "GUI/Dialogs/FileDialogService.h"
#include "Utility/EnumKeyword.h"
#include "Utility/UiIcons.h"
#include "Widgets/DialogSizing.h"
#include <QCheckBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QSignalBlocker>
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

ElementResultExportDialog::ElementResultExportDialog(const QString& resultFile, const QSet<int>& availableElements,
                                                     QWidget* parent)
    : QDialog(parent)
    , m_availableElements(availableElements)
    , m_resultFile(resultFile)
{
    setObjectName(QStringLiteral("elementResultExportDialog"));
    setWindowTitle(QStringLiteral("导出单元结果"));
    setWindowIcon(UiIcons::applicationIcon());
    setModal(true);
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(20, 18, 20, 18);
    layout->setSpacing(12);
    layout->setAlignment(Qt::AlignTop);

    auto* title = new QLabel(QStringLiteral("导出单元时程数据"), this);
    title->setObjectName(QStringLiteral("exportDialogTitle"));
    layout->addWidget(title);
    auto* subtitle = new QLabel(QStringLiteral("从当前 H5 结果中选择单元和结果分量"), this);
    subtitle->setObjectName(QStringLiteral("exportDialogSubtitle"));
    subtitle->setWordWrap(true);
    layout->addWidget(subtitle);

    auto* selectionCard = new QGroupBox(QStringLiteral("1  选择单元"), this);
    selectionCard->setObjectName(QStringLiteral("exportSectionCard"));
    selectionCard->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    auto* selectionLayout = new QVBoxLayout(selectionCard);
    m_elementEdit = new QLineEdit(selectionCard);
    m_elementEdit->setPlaceholderText(QStringLiteral("例如：1, 3, 8-12"));
    m_elementEdit->setClearButtonEnabled(true);
    m_elementEdit->setAccessibleName(QStringLiteral("单元编号"));
    selectionLayout->addWidget(m_elementEdit);
    auto* selectionHint =
        new QLabel(QStringLiteral("支持逗号、空格和连续范围；导出前会检查单元是否存在。"), selectionCard);
    selectionHint->setObjectName(QStringLiteral("exportFieldHint"));
    selectionHint->setWordWrap(true);
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
    updateDefaultFile();
    m_fileEdit->setCursorPosition(m_fileEdit->text().size());
    m_fileEdit->setToolTip(m_fileEdit->text());
    m_fileEdit->setAccessibleName(QStringLiteral("输出文件路径"));
    auto* browseButton = new QPushButton(QStringLiteral("浏览…"), fileCard);
    browseButton->setObjectName(QStringLiteral("exportQuietButton"));
    browseButton->setAutoDefault(false);
    auto* fileRow = new QHBoxLayout;
    fileRow->addWidget(m_fileEdit);
    fileRow->addWidget(browseButton);
    fileLayout->addLayout(fileRow);
    layout->addWidget(fileCard);

    m_errorLabel = new QLabel(this);
    m_errorLabel->setObjectName(QStringLiteral("exportValidationLabel"));
    m_errorLabel->setProperty("dialogFooter", true);
    m_errorLabel->setWordWrap(true);
    m_errorLabel->setMinimumHeight(fontMetrics().lineSpacing());
    layout->addWidget(m_errorLabel);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    buttons->setObjectName(QStringLiteral("exportDialogButtons"));
    m_exportButton = buttons->button(QDialogButtonBox::Save);
    m_exportButton->setText(QStringLiteral("导出"));
    m_exportButton->setProperty("primaryAction", true);
    m_exportButton->setDefault(true);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    layout->addWidget(buttons);

    connect(m_elementEdit, &QLineEdit::textChanged, this,
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
    connect(browseButton, &QPushButton::clicked, this,
            [this]()
            {
                const QString file =
                    FileDialogService::selectSaveFile(this, QStringLiteral("保存单元结果"), m_fileEdit->text(),
                                                      QStringLiteral("BDF 结果文件 (*.bdf);;文本文件 (*.txt)"),
                                                      QStringLiteral("elementResultExport"), false);
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
    resize(680, 470);
    DialogSizing::lockHeightToContents(this);
    updateValidation();
}

std::vector<int> ElementResultExportDialog::elementIds() const
{
    QString error;
    return parseElementIds(error);
}

std::vector<EnumKeyword::ElementResultType> ElementResultExportDialog::resultTypes() const
{
    std::vector<EnumKeyword::ElementResultType> result;
    for (const auto& field : m_fields)
        if (field.box->isChecked())
            result.push_back(field.type);
    return result;
}

QString ElementResultExportDialog::outputFile() const
{
    return QFileInfo(m_fileEdit->text().trimmed()).absoluteFilePath();
}

QString ElementResultExportDialog::exportDirectory(const QString& resultFile)
{
    return ResultExportPaths::directory(resultFile);
}

void ElementResultExportDialog::addFieldGroup(QVBoxLayout* layout, const QString& caption,
                                              std::initializer_list<FieldDefinition> definitions)
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

std::vector<int> ElementResultExportDialog::parseElementIds(_OUT QString& error) const
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

void ElementResultExportDialog::updateDefaultFile()
{
    if (!m_fileEdit || (!m_defaultFile.isEmpty() && m_fileEdit->text() != m_defaultFile))
        return;
    QString error;
    const auto ids = parseElementIds(error);
    const auto types = resultTypes();
    QString category = QStringLiteral("结果");
    if (types.size() == 1)
    {
        switch (types.front())
        {
            case EnumKeyword::ElementResultType::AxialForce:
                category = QStringLiteral("轴力");
                break;
            case EnumKeyword::ElementResultType::Strain:
                category = QStringLiteral("应变");
                break;
            case EnumKeyword::ElementResultType::InitStress:
                category = QStringLiteral("初始应力");
                break;
            case EnumKeyword::ElementResultType::CurrentStress:
                category = QStringLiteral("应力");
                break;
            case EnumKeyword::ElementResultType::DeltaStress:
                category = QStringLiteral("应力增量");
                break;
            default:
                break;
        }
    }
    const QString selection = ids.size() == 1  ? QStringLiteral("单元%1").arg(ids.front())
                              : ids.size() > 1 ? QStringLiteral("多单元")
                                               : QStringLiteral("单元");
    const QString description = selection + category;
    m_defaultFile = ResultExportPaths::filePath(m_resultFile, description, QStringLiteral("bdf"));
    const QSignalBlocker blocker(m_fileEdit);
    m_fileEdit->setText(m_defaultFile);
    m_fileEdit->setToolTip(m_defaultFile);
    m_fileEdit->setCursorPosition(m_defaultFile.size());
}

bool ElementResultExportDialog::updateValidation()
{
    updateDefaultFile();
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
    const bool hasError = !error.isEmpty();
    const bool wasVisible = !m_errorLabel->isHidden();
    m_errorLabel->setText(error);
    m_errorLabel->setVisible(hasError);
    if (wasVisible != hasError && isVisible())
        DialogSizing::lockHeightToContents(this);
    m_exportButton->setEnabled(error.isEmpty());
    return error.isEmpty();
}
