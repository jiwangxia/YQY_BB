#include "Dialogs/PropertyLibraryDialog.h"
#include <QVariant>
#include <QAbstractButton>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include "Utility/UiIcons.h"
#include "Widgets/TableAppearance.h"
#include "Widgets/DialogSizing.h"
#include <QPushButton>
#include <QScreen>
#include <QTimer>

PropertyLibraryDialog::PropertyLibraryDialog(QWidget* p)
    : QDialog(p)
{
    initializeUi();
    setWindowIcon(UiIcons::applicationIcon());
    m_buttonBox->button(QDialogButtonBox::Save)->setText(QStringLiteral("保存"));
    m_buttonBox->button(QDialogButtonBox::Save)->setProperty("primaryAction", true);
    m_buttonBox->button(QDialogButtonBox::Save)->setDefault(true);
    m_buttonBox->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    m_buttonBox->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    m_noteLabel->setProperty("secondaryText", true);
    m_selectionLabel->setProperty("secondaryText", true);
    resize(980, 620);
    for (auto* t : {m_materialTable, m_sectionTable})
    {
        applyCenteredTableAppearance(t);
        t->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
        t->setSelectionBehavior(QAbstractItemView::SelectRows);
        t->setSelectionMode(QAbstractItemView::SingleSelection);
        t->setAlternatingRowColors(true);
        t->setWordWrap(false);
        t->setTextElideMode(Qt::ElideRight);
        t->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        t->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        t->verticalHeader()->hide();
        t->verticalHeader()->setDefaultSectionSize(34);
        t->horizontalHeader()->setMinimumSectionSize(64);
        t->horizontalHeader()->setMaximumSectionSize(240);
        t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        t->horizontalHeader()->setStretchLastSection(false);
        connect(t, &QTableWidget::currentCellChanged, this,
                [this]()
                {
                    updateSelectionLabel();
                });
    }
    m_materialTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    m_sectionTable->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
    connect(m_syncCheck, &QCheckBox::toggled, this, &PropertyLibraryDialog::updateSyncControls);
    QTimer::singleShot(0, this,
                       [this]()
                       {
                           updateSelectionLabel();
                           updateSyncControls();
                           DialogSizing::lockCurrentHeight(this);
                       });
}
PropertyLibraryDialog::~PropertyLibraryDialog() = default;
void PropertyLibraryDialog::updateSelectionLabel()
{
    const auto selectedText = [](QTableWidget* table)
    {
        const int row = table->currentRow();
        const auto* id = table->item(row, 0);
        const auto* name = table->item(row, 2);
        return id && name ? QStringLiteral("%1 · %2").arg(id->text(), name->text()) : QStringLiteral("—");
    };
    m_selectionLabel->setText(QStringLiteral("选中材料：%1　｜　选中截面：%2")
                                  .arg(selectedText(m_materialTable), selectedText(m_sectionTable)));
}
void PropertyLibraryDialog::updateSyncControls()
{
    const bool enabled = m_syncCheck->isEnabled() && m_syncCheck->isChecked();
    m_materialLabel->setEnabled(enabled);
    m_sectionLabel->setEnabled(enabled);
    m_targetMaterialCombo->setEnabled(enabled && m_targetMaterialCombo->count() > 0);
    m_targetSectionCombo->setEnabled(enabled && m_targetSectionCombo->count() > 0);
}
QTabWidget* PropertyLibraryDialog::tabs() const
{
    return m_tabs;
}
QTableWidget* PropertyLibraryDialog::materialTable() const
{
    return m_materialTable;
}
QTableWidget* PropertyLibraryDialog::sectionTable() const
{
    return m_sectionTable;
}
QCheckBox* PropertyLibraryDialog::syncCheck() const
{
    return m_syncCheck;
}
QComboBox* PropertyLibraryDialog::targetMaterialCombo() const
{
    return m_targetMaterialCombo;
}
QComboBox* PropertyLibraryDialog::targetSectionCombo() const
{
    return m_targetSectionCombo;
}
QDialogButtonBox* PropertyLibraryDialog::buttons() const
{
    return m_buttonBox;
}

void PropertyLibraryDialog::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("PropertyLibraryDialogClass");
    }
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setSpacing(12);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(18, 18, 18, 18);
    m_tabs = new QTabWidget(this);
    m_tabs->setObjectName("tabs");
    m_materialTable = new QTableWidget();
    if (m_materialTable->columnCount() < 8)
        m_materialTable->setColumnCount(8);
    QTableWidgetItem* __qtablewidgetitem = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
    QTableWidgetItem* __qtablewidgetitem1 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
    QTableWidgetItem* __qtablewidgetitem2 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
    QTableWidgetItem* __qtablewidgetitem3 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
    QTableWidgetItem* __qtablewidgetitem4 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
    QTableWidgetItem* __qtablewidgetitem5 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
    QTableWidgetItem* __qtablewidgetitem6 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(6, __qtablewidgetitem6);
    QTableWidgetItem* __qtablewidgetitem7 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(7, __qtablewidgetitem7);
    m_materialTable->setObjectName("materialTable");
    m_materialTable->setColumnCount(8);
    m_tabs->addTab(m_materialTable, QString());
    m_sectionTable = new QTableWidget();
    if (m_sectionTable->columnCount() < 6)
        m_sectionTable->setColumnCount(6);
    QTableWidgetItem* __qtablewidgetitem8 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(0, __qtablewidgetitem8);
    QTableWidgetItem* __qtablewidgetitem9 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(1, __qtablewidgetitem9);
    QTableWidgetItem* __qtablewidgetitem10 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(2, __qtablewidgetitem10);
    QTableWidgetItem* __qtablewidgetitem11 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(3, __qtablewidgetitem11);
    QTableWidgetItem* __qtablewidgetitem12 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(4, __qtablewidgetitem12);
    QTableWidgetItem* __qtablewidgetitem13 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(5, __qtablewidgetitem13);
    m_sectionTable->setObjectName("sectionTable");
    m_sectionTable->setColumnCount(6);
    m_tabs->addTab(m_sectionTable, QString());

    rootLayout->addWidget(m_tabs);

    m_selectionLabel = new QLabel(this);
    m_selectionLabel->setObjectName("selectionLabel");
    m_selectionLabel->setWordWrap(true);

    rootLayout->addWidget(m_selectionLabel);

    QGroupBox* syncGroup = new QGroupBox(this);
    syncGroup->setObjectName("syncGroup");
    QFormLayout* syncLayout = new QFormLayout(syncGroup);
    syncLayout->setObjectName("syncLayout");
    syncLayout->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    syncLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    syncLayout->setHorizontalSpacing(16);
    syncLayout->setVerticalSpacing(10);
    m_syncCheck = new QCheckBox(syncGroup);
    m_syncCheck->setObjectName("syncCheck");

    syncLayout->setWidget(0, QFormLayout::SpanningRole, m_syncCheck);

    m_materialLabel = new QLabel(syncGroup);
    m_materialLabel->setObjectName("materialLabel");

    syncLayout->setWidget(1, QFormLayout::LabelRole, m_materialLabel);

    m_targetMaterialCombo = new QComboBox(syncGroup);
    m_targetMaterialCombo->setObjectName("targetMaterialCombo");

    syncLayout->setWidget(1, QFormLayout::FieldRole, m_targetMaterialCombo);

    m_sectionLabel = new QLabel(syncGroup);
    m_sectionLabel->setObjectName("sectionLabel");

    syncLayout->setWidget(2, QFormLayout::LabelRole, m_sectionLabel);

    m_targetSectionCombo = new QComboBox(syncGroup);
    m_targetSectionCombo->setObjectName("targetSectionCombo");

    syncLayout->setWidget(2, QFormLayout::FieldRole, m_targetSectionCombo);

    rootLayout->addWidget(syncGroup);

    m_noteLabel = new QLabel(this);
    m_noteLabel->setObjectName("noteLabel");
    m_noteLabel->setWordWrap(true);

    rootLayout->addWidget(m_noteLabel);

    m_buttonBox = new QDialogButtonBox(this);
    m_buttonBox->setObjectName("buttonBox");
    m_buttonBox->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Save);

    rootLayout->addWidget(m_buttonBox);

    rootLayout->setStretch(0, 1);
#if QT_CONFIG(shortcut)
    m_materialLabel->setBuddy(m_targetMaterialCombo);
    m_sectionLabel->setBuddy(m_targetSectionCombo);
#endif // QT_CONFIG(shortcut)

    setWindowTitle(QCoreApplication::translate("PropertyLibraryDialogClass", "材料与截面属性库（当前会话）", nullptr));
    QTableWidgetItem* ___qtablewidgetitem = m_materialTable->horizontalHeaderItem(0);
    ___qtablewidgetitem->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "ID", nullptr));
    QTableWidgetItem* ___qtablewidgetitem1 = m_materialTable->horizontalHeaderItem(1);
    ___qtablewidgetitem1->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "类别", nullptr));
    QTableWidgetItem* ___qtablewidgetitem2 = m_materialTable->horizontalHeaderItem(2);
    ___qtablewidgetitem2->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "名称", nullptr));
    QTableWidgetItem* ___qtablewidgetitem3 = m_materialTable->horizontalHeaderItem(3);
    ___qtablewidgetitem3->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "E (Pa)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem4 = m_materialTable->horizontalHeaderItem(4);
    ___qtablewidgetitem4->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "泊松比", nullptr));
    QTableWidgetItem* ___qtablewidgetitem5 = m_materialTable->horizontalHeaderItem(5);
    ___qtablewidgetitem5->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "密度 (kg/m³)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem6 = m_materialTable->horizontalHeaderItem(6);
    ___qtablewidgetitem6->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "许用应力 (Pa)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem7 = m_materialTable->horizontalHeaderItem(7);
    ___qtablewidgetitem7->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "膨胀系数", nullptr));
    m_tabs->setTabText(m_tabs->indexOf(m_materialTable),
                       QCoreApplication::translate("PropertyLibraryDialogClass", "材料", nullptr));
    QTableWidgetItem* ___qtablewidgetitem8 = m_sectionTable->horizontalHeaderItem(0);
    ___qtablewidgetitem8->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "ID", nullptr));
    QTableWidgetItem* ___qtablewidgetitem9 = m_sectionTable->horizontalHeaderItem(1);
    ___qtablewidgetitem9->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "类别", nullptr));
    QTableWidgetItem* ___qtablewidgetitem10 = m_sectionTable->horizontalHeaderItem(2);
    ___qtablewidgetitem10->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "名称", nullptr));
    QTableWidgetItem* ___qtablewidgetitem11 = m_sectionTable->horizontalHeaderItem(3);
    ___qtablewidgetitem11->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "面积 (m²)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem12 = m_sectionTable->horizontalHeaderItem(4);
    ___qtablewidgetitem12->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "面积 (mm²)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem13 = m_sectionTable->horizontalHeaderItem(5);
    ___qtablewidgetitem13->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "说明", nullptr));
    m_tabs->setTabText(m_tabs->indexOf(m_sectionTable),
                       QCoreApplication::translate("PropertyLibraryDialogClass", "截面", nullptr));
    m_selectionLabel->setText(
        QCoreApplication::translate("PropertyLibraryDialogClass", "选中材料：—　｜　选中截面：—", nullptr));
    syncGroup->setTitle(QCoreApplication::translate("PropertyLibraryDialogClass", "当前模型", nullptr));
    m_syncCheck->setText(
        QCoreApplication::translate("PropertyLibraryDialogClass", "将选中的材料和截面同步到当前模型", nullptr));
    m_materialLabel->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "目标材料", nullptr));
    m_sectionLabel->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "目标截面", nullptr));
    m_noteLabel->setText(QCoreApplication::translate(
        "PropertyLibraryDialogClass",
        "双击数值单元格可编辑。保存仅修改本次运行中的数据，不改写 CSV；新建模型将使用修改后的预设。", nullptr));
}
