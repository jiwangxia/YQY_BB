#include "Widgets/ConductorModule.h"
#include <QVariant>
#include <QApplication>
#include <QAbstractSpinBox>
#include <QEvent>
#include <QCheckBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpacerItem>
#include <QTabWidget>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QWidget>
#include "Widgets/CompactDoubleSpinBox.h"
#include "Utility/DialogScreenAdapter.h"
#include "Widgets/ResultControlInputWidgets.h"
#include "Conductor/ConductorModelBuilder.h"
#include "Conductor/PropertyLibrary.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionBase.h"
#include "DataStructure/Section/Section_Ice.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Outputter.h"
#include <QDialog>
#include <QScreen>
#include <QScrollBar>
#include <QScopedValueRollback>
#include <QSignalBlocker>
#include <QTabBar>
#include <QTableWidgetItem>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <vector>

ConductorModule::ConductorModule(QWidget* p)
    : QWidget(p)
{
    initializeUi();
    setObjectName(QStringLiteral("conductorPage"));
    m_scrollArea->setObjectName(QStringLiteral("conductorScrollArea"));
    m_content->setObjectName(QStringLiteral("conductorContent"));
    m_propertyGroup->setObjectName(QStringLiteral("conductorPropertyGroup"));
    m_spanDefinitionGroup->setObjectName(QStringLiteral("conductorSpanDefinitionGroup"));
    m_formGroup->setObjectName(QStringLiteral("conductorFormGroup"));
    m_structuralIceGroup->setObjectName(QStringLiteral("conductorStructuralIceGroup"));
    m_spacerGroup->setObjectName(QStringLiteral("conductorSpacerGroup"));
    m_createButton->setObjectName(QStringLiteral("conductorCreateButton"));
    const QList<QWidget*> fields = {m_nameEdit,
                                    m_materialCombo,
                                    m_sectionCombo,
                                    m_elementCombo,
                                    m_bundleCombo,
                                    m_spacingSpin,
                                    m_bundleRotationSpin,
                                    m_segmentsSpin,
                                    m_stressSpin,
                                    m_endTopologyCombo,
                                    m_dualSupportSpacingSpin,
                                    m_endTransitionSpin,
                                    m_iceThicknessSpin,
                                    m_bareDiameterSpin,
                                    m_iceCenterAngleSpin,
                                    m_iceHalfAngleSpin,
                                    m_torsionalRigiditySpin,
                                    m_modelModeCombo,
                                    m_suspensionLengthSpin,
                                    m_startX,
                                    m_startY,
                                    m_startZ,
                                    m_endX,
                                    m_endY,
                                    m_endZ,
                                    m_spacerLayoutCombo,
                                    m_spacerStyleCombo,
                                    m_spacerCountSpin,
                                    m_spacerMassSpin,
                                    m_spacerPresetCombo,
                                    m_spacerModelEdit,
                                    m_spacerSpacingSpin,
                                    m_spacerElementCombo,
                                    m_spacerMaterialCombo,
                                    m_spacerSectionCombo};
    for (auto* f : fields)
    {
        f->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        f->setMinimumHeight(32);
        f->setFocusPolicy(Qt::StrongFocus);
    }
    for (auto* form : {m_formLayout, m_materialFormLayout, m_endConnectionFormLayout, m_structuralIceFormLayout,
                       m_spacerFormLayout, m_torsionFormLayout, m_spacerPropertyFormLayout, m_suspensionFormLayout})
    {
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        form->setRowWrapPolicy(QFormLayout::DontWrapRows);
        form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        form->setHorizontalSpacing(12);
        form->setVerticalSpacing(6);
    }
    m_propertySummary->setObjectName(QStringLiteral("mutedText"));
    m_stationHelpLabel->setObjectName(QStringLiteral("mutedText"));
    m_spacerCountPreview->setObjectName(QStringLiteral("mutedText"));
    const QString spacerSectionHelp =
        QStringLiteral("用于杆件刚度，需按实物或等效刚度标定；每套质量单独控制，不能由质量反推真实截面");
    m_spacerSectionLabel->setToolTip(spacerSectionHelp);
    m_spacerMassSpin->setToolTip(
        QStringLiteral("一套间隔棒全部杆件的总质量，按杆长分配；不改变刚度或源材料密度，生成模型使用等效质量密度。"
                       "刚性中心 MPC 将质量和转动惯量等效到中心主节点。"));
    m_spacerMassLabel->setToolTip(m_spacerMassSpin->toolTip());
    m_spacingSpin->setMinimum(0.001);
    m_spacerSpacingSpin->setRange(m_spacingSpin->minimum(), m_spacingSpin->maximum());
    m_spacerSpacingSpin->setDecimals(m_spacingSpin->decimals());
    m_spacerSpacingSpin->setValue(m_spacingSpin->value());
    m_spacerSpacingSpin->setToolTip(QStringLiteral("相邻子导线中心距，与基础页的子导线间距同步；可直接输入"));
    m_spacerPresetCombo->setToolTip(QStringLiteral("选择预设填入质量、型号、间距和参考材料；下方数值可直接修改"));
    m_spacerModelEdit->setToolTip(QStringLiteral("可直接输入自定义型号；修改质量或间距后保留此型号作为参考"));
    for (auto* c :
         {m_materialCombo, m_sectionCombo, m_elementCombo, m_bundleCombo, m_endTopologyCombo, m_spacerLayoutCombo,
          m_spacerStyleCombo, m_spacerElementCombo, m_spacerMaterialCombo, m_spacerSectionCombo, m_spacerPresetCombo})
    {
        c->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        c->setMinimumContentsLength(8);
        connect(c, &QComboBox::currentTextChanged, c,
                [c](const QString& text)
                {
                    const QString detail = c->currentData(Qt::ToolTipRole).toString();
                    c->setToolTip(detail.isEmpty() ? text : text + QStringLiteral("\n") + detail);
                });
    }
    connect(m_spacerSectionCombo, &QComboBox::currentTextChanged, this,
            [this, spacerSectionHelp](const QString& text)
            {
                const QString detail = m_spacerSectionCombo->currentData(Qt::ToolTipRole).toString();
                m_spacerSectionCombo->setToolTip((detail.isEmpty() ? text : text + QLatin1Char('\n') + detail) +
                                                 QLatin1Char('\n') + spacerSectionHelp);
            });

    m_spacerLayoutCombo->setItemData(0, false);
    m_spacerLayoutCombo->setItemData(1, true);
    m_spacerStyleCombo->setItemData(0, static_cast<int>(Conductor::InnerSpacerStyle::OuterPolygon));
    m_spacerStyleCombo->setItemData(1, static_cast<int>(Conductor::InnerSpacerStyle::CenterBraced));
    m_spacerStyleCombo->setItemData(2, static_cast<int>(Conductor::InnerSpacerStyle::InnerPolygon));
    m_spacerStyleCombo->setItemData(3, static_cast<int>(Conductor::InnerSpacerStyle::RigidCenterMPC));
    m_spacerStyleCombo->setItemData(4, static_cast<int>(Conductor::InnerSpacerStyle::CrossBraced));
    m_spacerStyleCombo->setItemData(
        3,
        QStringLiteral(
            "一个 6 自由度中心主节点 + 刚性偏置 MPC；每套质量和转动惯量等效到中心，材料与截面用于等效惯量分布"),
        Qt::ToolTipRole);
    m_spacerStyleCombo->setItemData(4, QStringLiteral("四根子导线之间用四边和两条对角梁连接，不增加中心节点"),
                                    Qt::ToolTipRole);
    m_spacerElementCombo->setItemData(0, static_cast<int>(EnumKeyword::ElementType::CR3D));
    m_spacerElementCombo->setItemData(1, static_cast<int>(EnumKeyword::ElementType::T3D2));
    // 显式绑定导线本体单元类型。
    // 共旋索是导线分析默认项；桁架和共旋梁保留用于对照。
    m_elementCombo->clear();
    m_elementCombo->addItem(QStringLiteral("CABLE 共旋索单元"), static_cast<int>(EnumKeyword::ElementType::CABLE));
    m_elementCombo->addItem(QStringLiteral("T3D2 桁架单元"), static_cast<int>(EnumKeyword::ElementType::T3D2));
    m_elementCombo->addItem(QStringLiteral("CR3D 共旋梁单元"), static_cast<int>(EnumKeyword::ElementType::CR3D));
    m_elementCombo->setItemData(0, QStringLiteral("只受拉，包含初应力、几何刚度、扭转和一致质量"), Qt::ToolTipRole);
    m_elementCombo->setCurrentIndex(0);
    m_endTopologyCombo->setItemData(0, static_cast<int>(Conductor::BundleEndTopology::SingleSupport));
    m_endTopologyCombo->setItemData(1, static_cast<int>(Conductor::BundleEndTopology::DualSupportByGroup));
    m_endTopologyCombo->setItemData(2, static_cast<int>(Conductor::BundleEndTopology::DirectWireSupports));
    initializeStationTable();

    connect(m_nameEdit, &QLineEdit::textEdited, this,
            [this](const QString& text)
            {
                m_modelNameCustomized = !text.trimmed().isEmpty();
            });
    connect(m_modelModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]()
            {
                updateModelModeUi();
                updateDefaultModelName();
                updateSpacerPreview();
            });
    connect(m_addStationButton, &QPushButton::clicked, this,
            [this]()
            {
                const int lastRow = m_stationTable->rowCount() - 1;
                const int insertRow = std::max(1, lastRow);
                auto valueAt = [this](int row, int column)
                {
                    bool ok = false;
                    const double value = m_stationTable->item(row, column)->text().toDouble(&ok);
                    return ok ? value : 0.0;
                };
                const int previousRow = std::max(0, insertRow - 1);
                const int nextRow = std::max(previousRow, lastRow);
                const Vector3d center((valueAt(previousRow, 2) + valueAt(nextRow, 2)) * 0.5,
                                      (valueAt(previousRow, 3) + valueAt(nextRow, 3)) * 0.5,
                                      (valueAt(previousRow, 4) + valueAt(nextRow, 4)) * 0.5);
                m_stationTable->insertRow(insertRow);
                for (int column = 0; column < 5; ++column)
                    m_stationTable->setItem(insertRow, column, new QTableWidgetItem());
                m_stationTable->item(insertRow, 2)->setText(QString::number(center.x(), 'f', 3));
                m_stationTable->item(insertRow, 3)->setText(QString::number(center.y(), 'f', 3));
                m_stationTable->item(insertRow, 4)->setText(QString::number(center.z(), 'f', 3));
                refreshStationTypes();
                m_stationTable->selectRow(insertRow);
                updateSpacerPreview();
            });
    connect(m_removeStationButton, &QPushButton::clicked, this,
            [this]()
            {
                const int row = m_stationTable->currentRow();
                if (m_stationTable->rowCount() > 3 && row > 0 && row + 1 < m_stationTable->rowCount())
                {
                    m_stationTable->removeRow(row);
                    refreshStationTypes();
                    updateSpacerPreview();
                }
            });
    connect(m_stationTable, &QTableWidget::itemChanged, this,
            [this]()
            {
                updateSpacerPreview();
            });
    connect(m_innerSpacerCheck, &QCheckBox::toggled, this,
            [this]()
            {
                updateSpacerUi();
            });
    connect(m_spacerLayoutCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]()
            {
                updateSpacerUi();
            });
    connect(m_spacerStyleCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]()
            {
                updateSpacerUi();
            });
    connect(m_spacerCountSpin, qOverload<int>(&QSpinBox::valueChanged), this,
            [this]()
            {
                updateSpacerPreview();
            });
    connect(m_spacerMassSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this]()
            {
                updateSpacerPreview();
                updateSpacerPresetDescription();
            });
    connect(m_spacerPresetCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            &ConductorModule::applySpacerPreset);
    connect(m_spacerPresetCombo, qOverload<int>(&QComboBox::activated), this, &ConductorModule::applySpacerPreset);
    connect(m_spacerModelEdit, &QLineEdit::textChanged, this,
            [this]()
            {
                updateSpacerPresetDescription();
            });
    connect(m_spacerMaterialCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]()
            {
                updateSpacerPresetDescription();
            });
    connect(m_spacerSpacingSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double value)
            {
                m_spacingSpin->setValue(value);
                updateSpacerPresetDescription();
            });
    connect(m_endTopologyCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]()
            {
                updateEndTopologyUi();
            });
    connect(m_spacingSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double value)
            {
                if (!m_dualSupportSpacingCustomized)
                    m_dualSupportSpacingSpin->setValue(value);
                const QSignalBlocker blocker(m_spacerSpacingSpin);
                m_spacerSpacingSpin->setValue(value);
                updateSpacerPresetDescription();
            });
    connect(m_dualSupportSpacingSpin, &QDoubleSpinBox::editingFinished, this,
            [this]()
            {
                m_dualSupportSpacingCustomized = true;
            });
    connect(m_bundleCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]()
            {
                refreshSpacerPresets();
                updateSpacerUi();
                updateEndTopologyUi();
                updateDefaultModelName();
            });
    const auto updateStructuralIceUi = [this]()
    {
        const bool enabled = m_structuralIceCheck->isChecked();
        m_iceThicknessSpin->setEnabled(enabled);
        m_iceCenterAngleSpin->setEnabled(enabled);
        m_iceHalfAngleSpin->setEnabled(enabled);
        m_iceEccentricityCheck->setEnabled(
            enabled && static_cast<EnumKeyword::ElementType>(m_elementCombo->currentData().toInt()) ==
                           EnumKeyword::ElementType::CABLE);
    };
    connect(m_structuralIceCheck, &QCheckBox::toggled, this, updateStructuralIceUi);
    connect(m_elementCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, updateStructuralIceUi);
    for (auto* coordinate : {m_startX, m_startY, m_endX, m_endY})
    {
        connect(coordinate, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [this]()
                {
                    updateSpacerPreview();
                });
    }
    refreshSpacerPresets();
    updateSpacerUi();
    updateEndTopologyUi();
    updateModelModeUi();
    updateStructuralIceUi();
    updateDefaultModelName();
    initializeCreationDialog();
}
ConductorModule::~ConductorModule() = default;

static void alignConductorFormLabels(const QList<QFormLayout*>& forms, int reservedWidth = 0)
{
    QList<QLabel*> labels;
    for (auto* form : forms)
    {
        for (int row = 0; row < form->rowCount(); ++row)
        {
            const auto* item = form->itemAt(row, QFormLayout::LabelRole);
            auto* label = item ? qobject_cast<QLabel*>(item->widget()) : nullptr;
            if (label)
                labels.append(label);
        }
    }
    for (auto* label : labels)
        reservedWidth = qMax(reservedWidth, label->fontMetrics().horizontalAdvance(label->text()));
    for (auto* label : labels)
        label->setFixedWidth(reservedWidth + 2);
}

void ConductorModule::initializeCreationDialog()
{
    m_creationDialog = new QDialog(this);
    m_creationDialog->setObjectName(QStringLiteral("conductorCreationDialog"));
    m_creationDialog->setWindowModality(Qt::WindowModal);
    m_creationDialog->setSizeGripEnabled(true);
    m_creationDialog->installEventFilter(this);
    auto* layout = new QVBoxLayout(m_creationDialog);
    layout->setContentsMargins(12, 12, 12, 12);
    layout->setSpacing(0);
    layout->setSizeConstraint(QLayout::SetNoConstraint);
    m_rootLayout->removeWidget(m_editorContainer);
    m_editorContainer->setMaximumWidth(1080);
    layout->addWidget(m_editorContainer);
    m_editorContainer->show();
    m_modelModeCombo->hide();
    m_creationTabs->setObjectName(QStringLiteral("conductorSettingsTabs"));
    m_advancedScrollArea->setObjectName(QStringLiteral("conductorAdvancedScrollArea"));
    m_advancedContent->setObjectName(QStringLiteral("conductorAdvancedContent"));
    m_spacerScrollArea->setObjectName(QStringLiteral("conductorSpacerScrollArea"));
    m_spacerContent->setObjectName(QStringLiteral("conductorSpacerContent"));
    m_torsionGroup->setObjectName(QStringLiteral("conductorTorsionGroup"));
    m_spacerPropertyGroup->setObjectName(QStringLiteral("conductorSpacerPropertyGroup"));
    for (auto* contentLayout : {m_contentLayout, m_advancedContentLayout, m_spacerContentLayout})
        contentLayout->setSizeConstraint(QLayout::SetMinimumSize);
    for (auto* group : {m_propertyGroup, m_formGroup, m_spanDefinitionGroup, m_endConnectionGroup, m_structuralIceGroup,
                        m_spacerGroup, m_torsionGroup, m_spacerPropertyGroup})
        group->setProperty("conductorFormGroup", true);
    for (auto* form : {m_formLayout, m_materialFormLayout, m_endConnectionFormLayout, m_structuralIceFormLayout,
                       m_spacerFormLayout, m_torsionFormLayout, m_spacerPropertyFormLayout, m_suspensionFormLayout})
        form->setContentsMargins(12, 10, 12, 8);
    m_coordinateFormLayout->setContentsMargins(0, 0, 0, 0);
    m_coordinateFormLayout->setHorizontalSpacing(12);
    m_coordinateFormLayout->setVerticalSpacing(6);
    m_startLayout->setAlignment(Qt::AlignTop);
    m_endLayout->setAlignment(Qt::AlignTop);
    m_suspensionFormLayout->setContentsMargins(0, 0, 0, 0);
    m_propertyLayout->setContentsMargins(12, 10, 12, 8);
    m_materialFormLayout->setContentsMargins(0, 0, 0, 0);
    m_spanDefinitionLayout->setContentsMargins(12, 10, 12, 8);
    m_propertyGroup->setToolTip(m_propertySummary->text());
    m_propertySummary->hide();
    m_stationTable->setToolTip(m_stationHelpLabel->text());
    m_stationHelpLabel->hide();
    m_stationTable->setFixedHeight(136);
    m_suspensionLengthSpin->setMinimumWidth(90);
    m_stationButtonLayout->setSpacing(6);
    m_suspensionFormLayout->setHorizontalSpacing(6);
    m_dualSupportSpacingSpin->setToolTip(QStringLiteral("仅分组双挂点使用此间距"));
    m_spacerCountSpin->setToolTip(QStringLiteral("选择指定数量（等距）时使用此数量"));
    for (auto* coordinate : {m_startX, m_startY, m_startZ, m_endX, m_endY, m_endZ})
    {
        coordinate->setMinimumWidth(80);
        coordinate->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    }
    m_singleSpanButton->setToolTip(QStringLiteral("设置两端挂点，创建单档导线或分裂导线"));
    m_multiSpanButton->setToolTip(QStringLiteral("设置耐张与悬垂塔位，创建连续多档导线"));
    m_createButton->setDefault(true);
    m_cancelButton->setAutoDefault(false);
    m_viewLibraryButton->setAutoDefault(false);
    connect(m_singleSpanButton, &QPushButton::clicked, this,
            [this]()
            {
                m_modelModeCombo->setCurrentIndex(0);
            });
    connect(m_multiSpanButton, &QPushButton::clicked, this,
            [this]()
            {
                m_modelModeCombo->setCurrentIndex(1);
            });
    connect(m_cancelButton, &QPushButton::clicked, this, &ConductorModule::closeCreationDialog);
    connect(m_viewLibraryButton, &QPushButton::clicked, this, &ConductorModule::closeCreationDialog);
    setCreationEnabled(false);
}

QPushButton* ConductorModule::singleSpanButton() const
{
    return m_singleSpanButton;
}

QPushButton* ConductorModule::multiSpanButton() const
{
    return m_multiSpanButton;
}

void ConductorModule::setCreationEnabled(bool enabled)
{
    m_singleSpanButton->setEnabled(enabled);
    m_multiSpanButton->setEnabled(enabled);
    m_createButton->setEnabled(enabled);
}

void ConductorModule::openCreationDialog()
{
    openCreationDialog(m_modelModeCombo->currentIndex() == 1);
}

void ConductorModule::openCreationDialog(bool multiSpan)
{
    if (!m_createButton->isEnabled())
        return;
    m_modelModeCombo->setCurrentIndex(multiSpan ? 1 : 0);
    m_creationDialog->setWindowTitle(multiSpan ? QStringLiteral("创建多档导线") : QStringLiteral("创建单档导线"));
    m_creationDialog->setWindowIcon(window()->windowIcon());
    m_creationTabs->setCurrentIndex(0);
    m_scrollArea->verticalScrollBar()->setValue(0);
    QWidget* dialogContent = m_creationDialog;
    for (auto* widget : dialogContent->findChildren<QWidget*>())
    {
        widget->ensurePolished();
        if (qobject_cast<QAbstractSpinBox*>(widget))
        {
            widget->setMinimumHeight(qMax(widget->minimumHeight(), widget->sizeHint().height()));
        }
    }
    alignConductorFormLabels({m_materialFormLayout, m_formLayout, m_endConnectionFormLayout});
    alignConductorFormLabels({m_structuralIceFormLayout, m_torsionFormLayout});
    const QList<QWidget*> spacerFields = {
        m_spacerPresetCombo, m_spacerModelEdit,    m_spacerMassSpin,    m_spacerSpacingSpin, m_spacerMaterialCombo,
        m_spacerStyleCombo,  m_spacerElementCombo, m_spacerLayoutCombo, m_spacerCountSpin,   m_spacerSectionCombo};
    int spacerFieldHeight = 0;
    for (auto* field : spacerFields)
        spacerFieldHeight = qMax(spacerFieldHeight, qMax(field->minimumHeight(), field->sizeHint().height()));
    for (auto* field : spacerFields)
        field->setFixedHeight(spacerFieldHeight);
    const int coordinateRowHeight = qMax(m_startX->minimumHeight(), m_startX->sizeHint().height());
    m_startLabel->setFixedHeight(coordinateRowHeight);
    m_endLabel->setFixedHeight(coordinateRowHeight);

    // 两种建模模式预留相同的坐标区高度，切换时保持四块边界不变。
    const QMargins groupMargins = m_spanDefinitionGroup->contentsMargins();
    const QMargins spanMargins = m_spanDefinitionLayout->contentsMargins();
    m_suspensionFormLayout->invalidate();
    m_stationButtonLayout->invalidate();
    m_singleSpanEditor->layout()->invalidate();
    m_multiSpanEditor->layout()->invalidate();
    const int coordinateHeight = qMax(m_singleSpanEditor->layout()->totalMinimumSize().height(),
                                      m_multiSpanEditor->layout()->totalMinimumSize().height());
    m_spanDefinitionGroup->setMinimumHeight(coordinateHeight + groupMargins.top() + groupMargins.bottom() +
                                            spanMargins.top() + spanMargins.bottom());
    m_contentLayout->invalidate();
    m_advancedContentLayout->invalidate();
    m_spacerContentLayout->invalidate();
    m_spacerCreationLayout->invalidate();
    m_globalBundleLayout->invalidate();
    const QMargins dialogMargins = dialogContent->layout()->contentsMargins();
    const QMargins spacerMargins = m_spacerTabLayout->contentsMargins();
    const int spacerHeight = m_spacerContentLayout->minimumSize().height() +
                             m_spacerCreationLayout->minimumSize().height() + m_spacerStatusLabel->sizeHint().height() +
                             m_spacerTabLayout->spacing() * 2 + spacerMargins.top() + spacerMargins.bottom();
    // 单导线也按全部页面预留高度，新增间隔棒导航时不改变窗口尺寸。
    const int contentHeight = qMax(
        spacerHeight, qMax(m_contentLayout->minimumSize().height(), m_advancedContentLayout->minimumSize().height()));
    const int initialHeight = contentHeight + m_creationTabs->tabBar()->sizeHint().height() +
                              m_footerLayout->minimumSize().height() + m_globalBundleLayout->minimumSize().height() +
                              m_editorLayout->spacing() * 2 + dialogMargins.top() + dialogMargins.bottom() + 4;
    if (QScreen* currentScreen = m_creationDialogInitialized ? m_creationDialog->screen() : window()->screen())
    {
        const QRect available = currentScreen->availableGeometry().adjusted(24, 40, -24, -40);
        const QSize desired = m_creationDialogInitialized ? m_creationDialog->size() : QSize(1000, initialHeight);
        DialogScreenAdapter::install(m_creationDialog, QSize(560, 360));
        m_creationDialog->resize(desired);
        const QPoint position = m_creationDialogInitialized
                                    ? m_creationDialog->pos()
                                    : window()->frameGeometry().center() - m_creationDialog->rect().center();
        const int maximumX = available.left() + qMax(0, available.width() - m_creationDialog->width());
        const int maximumY = available.top() + qMax(0, available.height() - m_creationDialog->height());
        m_creationDialog->move(qBound(available.left(), position.x(), maximumX),
                               qBound(available.top(), position.y(), maximumY));
    }
    m_creationDialogInitialized = true;
    m_creationDialog->open();
    updateResponsiveLayout();
    m_creationDialog->raise();
    m_creationDialog->activateWindow();
}

void ConductorModule::closeCreationDialog()
{
    m_creationDialog->reject();
}

bool ConductorModule::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == m_creationDialog && m_creationDialog->isVisible() &&
        (event->type() == QEvent::Resize || event->type() == QEvent::Show))
        updateResponsiveLayout();
    return QWidget::eventFilter(watched, event);
}

void ConductorModule::updateResponsiveLayout()
{
    const int contentWidth = qMin(m_editorContainer->maximumWidth(), m_creationDialog->width() - 24);
    m_globalBundleLayout->setDirection(contentWidth < 560 ? QBoxLayout::TopToBottom : QBoxLayout::LeftToRight);
    const bool footerStacked = contentWidth < 730;
    if (m_footerStacked != footerStacked)
    {
        m_footerStacked = footerStacked;
        if (footerStacked)
        {
            m_footerLayout->removeWidget(m_analysisCheck);
            m_editorLayout->insertWidget(2, m_analysisCheck);
        }
        else
        {
            m_editorLayout->removeWidget(m_analysisCheck);
            m_footerLayout->insertWidget(1, m_analysisCheck);
        }
    }
    // 窄窗口改为单列，控件维持正常字号和高度。
    const bool singleColumn = contentWidth < 896;
    if (m_singleColumnLayout == singleColumn)
        return;
    m_singleColumnLayout = singleColumn;
    arrangeSections(m_contentLayout, {m_propertyGroup, m_formGroup, m_spanDefinitionGroup, m_endConnectionGroup},
                    singleColumn);
    arrangeSections(m_advancedContentLayout, {m_structuralIceGroup, m_torsionGroup}, singleColumn);
    arrangeSections(m_spacerContentLayout, {m_spacerPropertyGroup, m_spacerGroup}, singleColumn);
}

void ConductorModule::arrangeSections(QGridLayout* grid, const QList<QWidget*>& sections, bool singleColumn)
{
    for (auto* section : sections)
        grid->removeWidget(section);
    for (int index = 0; index < sections.size(); ++index)
        grid->addWidget(sections[index], singleColumn ? index : index / 2, singleColumn ? 0 : index % 2);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, singleColumn ? 0 : 1);
    grid->setAlignment(singleColumn ? Qt::AlignTop : Qt::Alignment());
    grid->invalidate();
}

QDialog* ConductorModule::creationDialog() const
{
    return m_creationDialog;
}

void ConductorModule::initializeStationTable()
{
    m_stationTable->setRowCount(3);
    m_stationTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_stationTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_stationTable->horizontalHeader()->setMinimumSectionSize(48);
    m_stationTable->verticalHeader()->hide();
    m_stationTable->verticalHeader()->setDefaultSectionSize(32);
    m_stationTable->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    m_stationTable->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    for (int column = 2; column < 5; ++column)
    {
        m_stationTable->horizontalHeader()->setSectionResizeMode(column, QHeaderView::Stretch);
    }
    const Vector3d defaults[] = {Vector3d(0.0, 0.0, 0.0), Vector3d(150.0, 0.0, 0.0), Vector3d(300.0, 0.0, 0.0)};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 5; ++column)
            m_stationTable->setItem(row, column, new QTableWidgetItem());
        m_stationTable->item(row, 2)->setText(QString::number(defaults[row].x(), 'f', 3));
        m_stationTable->item(row, 3)->setText(QString::number(defaults[row].y(), 'f', 3));
        m_stationTable->item(row, 4)->setText(QString::number(defaults[row].z(), 'f', 3));
    }
    refreshStationTypes();
}

void ConductorModule::updateModelModeUi()
{
    const bool multiSpan = m_modelModeCombo->currentIndex() == 1;
    m_singleSpanButton->setChecked(!multiSpan);
    m_multiSpanButton->setChecked(multiSpan);
    if (m_creationDialog)
        m_creationDialog->setWindowTitle(multiSpan ? QStringLiteral("创建多档导线") : QStringLiteral("创建单档导线"));
    m_multiSpanEditor->setVisible(multiSpan);
    m_singleSpanEditor->setVisible(!multiSpan);
    m_spanDefinitionGroup->setTitle(multiSpan ? QStringLiteral("塔位与坐标") : QStringLiteral("挂点坐标"));
    m_startLabel->setVisible(!multiSpan);
    m_startCoordinates->setVisible(!multiSpan);
    m_endLabel->setVisible(!multiSpan);
    m_endCoordinates->setVisible(!multiSpan);
    m_segmentsLabel->setText(multiSpan ? QStringLiteral("每档离散段数") : QStringLiteral("每根离散段数"));
    m_segmentsLabel->setToolTip(multiSpan ? QStringLiteral("每一档、每一根子导线的离散段数")
                                          : QStringLiteral("每一根子导线的离散段数"));
    m_analysisCheck->setToolTip(multiSpan
                                    ? QStringLiteral("约束首末耐张挂点和全部中间悬垂串上端，并创建静力分析步与重力")
                                    : QStringLiteral("创建静力分析步、重力和两端约束"));
}

QString ConductorModule::defaultModelName() const
{
    QString name;
    switch (m_bundleCombo->currentIndex())
    {
        case 1:
            name = QStringLiteral("双分裂导线");
            break;
        case 2:
            name = QStringLiteral("四分裂导线");
            break;
        case 3:
            name = QStringLiteral("六分裂导线");
            break;
        case 4:
            name = QStringLiteral("八分裂导线");
            break;
        default:
            name = QStringLiteral("单导线");
            break;
    }
    return m_modelModeCombo->currentIndex() == 1 ? QStringLiteral("多跨") + name : name;
}

void ConductorModule::updateDefaultModelName()
{
    const QString previousDefault = m_defaultModelName;
    m_defaultModelName = defaultModelName();
    const QString currentName = m_nameEdit->text().trimmed();
    if (!m_modelNameCustomized && (currentName.isEmpty() || currentName == previousDefault))
        m_nameEdit->setText(m_defaultModelName);
    m_nameEdit->setPlaceholderText(m_defaultModelName);
}

void ConductorModule::refreshStationTypes()
{
    const int rowCount = m_stationTable->rowCount();
    for (int row = 0; row < rowCount; ++row)
    {
        auto* sequenceItem = m_stationTable->item(row, 0);
        auto* typeItem = m_stationTable->item(row, 1);
        if (!sequenceItem)
        {
            sequenceItem = new QTableWidgetItem();
            m_stationTable->setItem(row, 0, sequenceItem);
        }
        if (!typeItem)
        {
            typeItem = new QTableWidgetItem();
            m_stationTable->setItem(row, 1, typeItem);
        }
        sequenceItem->setText(QString::number(row + 1));
        typeItem->setText(row == 0 ? QStringLiteral("左耐张")
                                   : (row + 1 == rowCount ? QStringLiteral("右耐张") : QStringLiteral("H 型悬垂")));
        sequenceItem->setFlags(sequenceItem->flags() & ~Qt::ItemIsEditable);
        typeItem->setFlags(typeItem->flags() & ~Qt::ItemIsEditable);
    }
}

std::vector<Vector3d> ConductorModule::stationCenters(_OUT QString& error) const
{
    std::vector<Vector3d> centers;
    if (m_stationTable->rowCount() < 3)
    {
        error = QStringLiteral("多档导线至少需要三个坐标点。");
        return centers;
    }
    centers.reserve(static_cast<std::size_t>(m_stationTable->rowCount()));
    for (int row = 0; row < m_stationTable->rowCount(); ++row)
    {
        double values[3] = {};
        for (int coordinate = 0; coordinate < 3; ++coordinate)
        {
            const auto* item = m_stationTable->item(row, coordinate + 2);
            bool ok = false;
            values[coordinate] = item ? item->text().toDouble(&ok) : 0.0;
            if (!ok)
            {
                error = QStringLiteral("第 %1 个坐标点的 %2 值无效。")
                            .arg(row + 1)
                            .arg(QStringLiteral("XYZ").at(coordinate));
                centers.clear();
                return centers;
            }
        }
        centers.emplace_back(values[0], values[1], values[2]);
        if (row > 0 && (centers[row] - centers[row - 1]).head<2>().norm() <= 1.0e-7)
        {
            error = QStringLiteral("第 %1 档的水平档距过小。").arg(row);
            centers.clear();
            return centers;
        }
    }
    return centers;
}

void ConductorModule::refreshSpacerPresets()
{
    const QString previousId = m_spacerPresetCombo->currentData().toString();
    QString previousModel;
    for (const auto& preset : Conductor::PropertyLibrary::spacerPresets())
    {
        if (preset.id == previousId)
        {
            previousModel = preset.model;
            break;
        }
    }
    const int bundleCount = m_bundleCombo->currentText().toInt();
    const QSignalBlocker blocker(m_spacerPresetCombo);
    m_spacerPresetCombo->clear();
    for (const auto& preset : Conductor::PropertyLibrary::spacerPresets())
    {
        if (preset.bundleCount != bundleCount)
            continue;
        const QString text = QStringLiteral("%1 kg · %2 · %3 mm")
                                 .arg(QString::number(preset.mass, 'g', 6), preset.model,
                                      QString::number(preset.spacing * 1000.0, 'g', 6));
        m_spacerPresetCombo->addItem(text, preset.id);
        const int index = m_spacerPresetCombo->count() - 1;
        m_spacerPresetCombo->setItemData(
            index, preset.manufacturer + QLatin1Char('\n') + preset.materialNote + QLatin1Char('\n') + preset.sourceUrl,
            Qt::ToolTipRole);
    }
    m_spacerPresetCombo->addItem(QStringLiteral("自定义（直接输入下方参数）"), QString());
    const int previousIndex = previousId.isEmpty() ? -1 : m_spacerPresetCombo->findData(previousId);
    m_spacerPresetCombo->setCurrentIndex(previousIndex >= 0 ? previousIndex : m_spacerPresetCombo->count() - 1);
    if (!previousId.isEmpty() && previousIndex < 0 && m_spacerModelEdit->text().trimmed() == previousModel)
        m_spacerModelEdit->setText(QStringLiteral("自定义间隔棒"));
    if (m_spacerModelEdit->text().isEmpty())
        m_spacerModelEdit->setText(QStringLiteral("自定义间隔棒"));
    updateSpacerPresetDescription();
}

void ConductorModule::applySpacerPreset(int index)
{
    if (index < 0)
        return;
    const QString id = m_spacerPresetCombo->itemData(index).toString();
    const auto& presets = Conductor::PropertyLibrary::spacerPresets();
    const auto found = std::find_if(presets.cbegin(), presets.cend(),
                                    [&id](const Conductor::SpacerPreset& preset)
                                    {
                                        return preset.id == id;
                                    });
    if (found == presets.cend())
    {
        updateSpacerPresetDescription();
        return;
    }
    {
        const QScopedValueRollback<bool> applying(m_applyingSpacerPreset, true);
        m_spacerMassSpin->setValue(found->mass);
        m_spacingSpin->setValue(found->spacing);
        m_spacerModelEdit->setText(found->model);
        if (m_propertyLibrary)
        {
            for (int materialIndex = 0; materialIndex < m_propertyLibrary->materials().size(); ++materialIndex)
            {
                if (m_propertyLibrary->materials().at(materialIndex).name != found->materialName)
                    continue;
                const QSignalBlocker materialBlocker(m_spacerMaterialCombo);
                m_spacerMaterialCombo->setCurrentIndex(materialIndex);
                break;
            }
        }
    }
    updateSpacerPresetDescription();
    updateSpacerPreview();
}

void ConductorModule::updateSpacerPresetDescription()
{
    if (m_applyingSpacerPreset)
        return;
    const QString id = m_spacerPresetCombo->currentData().toString();
    const auto& presets = Conductor::PropertyLibrary::spacerPresets();
    const auto found = std::find_if(presets.cbegin(), presets.cend(),
                                    [&id](const Conductor::SpacerPreset& preset)
                                    {
                                        return preset.id == id;
                                    });
    m_spacerMaterialLabel->setText(QStringLiteral("材料"));
    m_spacerModelLabel->setText(QStringLiteral("产品型号"));
    m_spacerMaterialCombo->setToolTip(m_spacerMaterialCombo->currentData(Qt::ToolTipRole).toString());
    if (found == presets.cend())
    {
        m_spacerPresetCombo->setToolTip(QStringLiteral("直接输入质量、间距和型号，并选择材料、截面"));
        m_spacerStatusLabel->setText(m_innerSpacerCheck->isChecked() ? QStringLiteral("使用直接输入的质量、尺寸和属性")
                                                                     : QStringLiteral("暂不创建间隔棒，保留当前参数"));
        m_spacerStatusLabel->setToolTip(m_spacerPresetCombo->toolTip());
        return;
    }

    QStringList changes;
    if (std::abs(m_spacerMassSpin->value() - found->mass) > 1.0e-9)
        changes.append(QStringLiteral("质量"));
    if (std::abs(m_spacingSpin->value() - found->spacing) > 1.0e-9)
        changes.append(QStringLiteral("间距"));
    if (m_spacerModelEdit->text().trimmed() != found->model)
        changes.append(QStringLiteral("型号"));
    if (!changes.isEmpty())
        m_spacerModelLabel->setText(QStringLiteral("参考型号"));

    const int materialIndex = m_spacerMaterialCombo->currentIndex();
    const bool matchesMaterial = m_propertyLibrary && materialIndex >= 0 &&
                                 materialIndex < m_propertyLibrary->materials().size() &&
                                 m_propertyLibrary->materials().at(materialIndex).name == found->materialName;
    if (matchesMaterial)
    {
        if (!found->materialVerified)
            m_spacerMaterialLabel->setText(QStringLiteral("材料（参考）"));
        m_spacerMaterialCombo->setToolTip(m_spacerMaterialCombo->toolTip() + QLatin1Char('\n') + found->materialNote);
    }
    QString detail = m_spacerPresetCombo->currentData(Qt::ToolTipRole).toString();
    if (!changes.isEmpty())
        detail.prepend(QStringLiteral("已直接修改%1；保留所选产品作为参考\n").arg(changes.join(QStringLiteral("、"))));
    m_spacerPresetCombo->setToolTip(detail);
    QString status = changes.isEmpty()
                         ? QStringLiteral("使用产品标称质量与间距")
                         : QStringLiteral("已修改%1，保留所选产品作为参考").arg(changes.join(QStringLiteral("、")));
    if (matchesMaterial && !found->materialVerified)
        status += QStringLiteral(" · 材料为参考值");
    if (!m_innerSpacerCheck->isChecked())
        status = QStringLiteral("暂不创建间隔棒，保留当前参数");
    m_spacerStatusLabel->setText(status);
    m_spacerStatusLabel->setToolTip(detail);
}

void ConductorModule::updateSpacerUi()
{
    const int bundleCount = m_bundleCombo->currentText().toInt();
    const bool multipleBundle = bundleCount > 1;
    const int spacerTabIndex = m_creationTabs->indexOf(m_spacerTab);
    if (!multipleBundle && m_creationTabs->currentWidget() == m_spacerTab)
        m_creationTabs->setCurrentWidget(m_basicTab);
    m_creationTabs->setTabVisible(spacerTabIndex, multipleBundle);
    const bool enabled = m_innerSpacerCheck->isChecked() && multipleBundle;
    const bool equalSpacing = m_spacerLayoutCombo->currentData().toBool();
    if (bundleCount <= 2 && static_cast<Conductor::InnerSpacerStyle>(m_spacerStyleCombo->currentData().toInt()) ==
                                Conductor::InnerSpacerStyle::RigidCenterMPC)
        m_spacerStyleCombo->setCurrentIndex(0);
    if (bundleCount != 4 && static_cast<Conductor::InnerSpacerStyle>(m_spacerStyleCombo->currentData().toInt()) ==
                                Conductor::InnerSpacerStyle::CrossBraced)
        m_spacerStyleCombo->setCurrentIndex(0);
    const bool rigidCenterMpc = static_cast<Conductor::InnerSpacerStyle>(m_spacerStyleCombo->currentData().toInt()) ==
                                Conductor::InnerSpacerStyle::RigidCenterMPC;
    for (auto* widget : {static_cast<QWidget*>(m_spacerLayoutCombo), static_cast<QWidget*>(m_spacerStyleCombo),
                         static_cast<QWidget*>(m_spacerElementCombo), static_cast<QWidget*>(m_spacerMaterialCombo),
                         static_cast<QWidget*>(m_spacerSectionCombo), static_cast<QWidget*>(m_spacerPresetCombo),
                         static_cast<QWidget*>(m_spacerModelEdit), static_cast<QWidget*>(m_spacerSpacingSpin)})
    {
        widget->setEnabled(enabled);
    }
    m_spacerStyleCombo->setEnabled(enabled && bundleCount > 2);
    m_spacerElementCombo->setEnabled(enabled && !rigidCenterMpc);
    m_spacerElementLabel->setEnabled(enabled && !rigidCenterMpc);
    m_spacerMassLabel->setEnabled(enabled);
    m_spacerMassSpin->setEnabled(enabled);
    m_spacerPresetLabel->setEnabled(enabled);
    m_spacerModelLabel->setEnabled(enabled);
    m_spacerSpacingLabel->setEnabled(enabled);
    m_spacerCountLabel->setEnabled(enabled && equalSpacing);
    m_spacerCountSpin->setEnabled(enabled && equalSpacing);
    updateSpacerPresetDescription();
    updateSpacerPreview();
}

void ConductorModule::updateEndTopologyUi()
{
    // 属性库尚未注入时 bundleCombo 的 itemData 为空；界面默认值仍应
    // 根据显示的分裂数正确启用双挂点选项。
    const int bundleCount = m_bundleCombo->currentText().toInt();
    const bool supportsAnyEndTopology = bundleCount > 1;
    m_bundleRotationSpin->setEnabled(supportsAnyEndTopology);
    m_bundleRotationLabel->setEnabled(supportsAnyEndTopology);
    const bool supportsDual = bundleCount > 2;
    m_endTopologyCombo->setEnabled(supportsAnyEndTopology);
    if (!supportsAnyEndTopology)
        m_endTopologyCombo->setCurrentIndex(0);
    else if (!supportsDual && m_endTopologyCombo->currentData().toInt() ==
                                  static_cast<int>(Conductor::BundleEndTopology::DualSupportByGroup))
        m_endTopologyCombo->setCurrentIndex(0);
    const bool dual = supportsDual && m_endTopologyCombo->currentData().toInt() ==
                                          static_cast<int>(Conductor::BundleEndTopology::DualSupportByGroup);
    // 保留参数行，只切换可用状态，避免切换端部形式时布局跳动。
    m_dualSupportSpacingLabel->setEnabled(dual);
    m_dualSupportSpacingSpin->setEnabled(dual);
}

void ConductorModule::updateSpacerPreview()
{
    if (!m_innerSpacerCheck->isChecked())
    {
        m_spacerCountPreview->setText(QStringLiteral("不创建"));
        m_spacerCountPreview->setToolTip(QString());
        return;
    }

    if (m_bundleCombo->currentText().toInt() <= 1)
    {
        m_spacerCountPreview->setText(QStringLiteral("单分裂导线不创建"));
        m_spacerCountPreview->setToolTip(QString());
        return;
    }

    const int bundleCount = m_bundleCombo->currentText().toInt();
    const bool rigidCenterMpc = static_cast<Conductor::InnerSpacerStyle>(m_spacerStyleCombo->currentData().toInt()) ==
                                Conductor::InnerSpacerStyle::RigidCenterMPC;
    const auto topologyDescription = [=](int spacerCount)
    {
        return rigidCenterMpc
                   ? QStringLiteral("%1 个中心节点、%2 个刚性偏置 MPC、0 个间隔棒单元；质量和转动惯量等效到中心")
                         .arg(spacerCount)
                         .arg(spacerCount * bundleCount)
                   : QString();
    };
    const QString massDescription =
        QStringLiteral("每套质量 %1 kg；按杆长分配，不改变刚度或源材料密度，生成模型使用等效质量密度")
            .arg(QString::number(m_spacerMassSpin->value(), 'g', 15));

    if (m_spacerLayoutCombo->currentData().toBool())
    {
        const int spanCount = m_modelModeCombo->currentIndex() == 1 ? std::max(0, m_stationTable->rowCount() - 1) : 1;
        const int spacerCount = m_spacerCountSpin->value() * spanCount;
        m_spacerCountPreview->setText(QStringLiteral("%1 个（%2 档等距布置）").arg(spacerCount).arg(spanCount));
        m_spacerCountPreview->setToolTip(
            (rigidCenterMpc ? topologyDescription(spacerCount) : QStringLiteral("每一档均按指定数量等距布置")) +
            QLatin1Char('\n') + massDescription);
        return;
    }

    std::vector<double> spanLengths;
    if (m_modelModeCombo->currentIndex() == 1)
    {
        QString coordinateError;
        const auto centers = stationCenters(coordinateError);
        if (centers.empty())
        {
            m_spacerCountPreview->setText(QStringLiteral("坐标无效"));
            m_spacerCountPreview->setToolTip(coordinateError);
            return;
        }
        for (std::size_t index = 0; index + 1 < centers.size(); ++index)
            spanLengths.push_back((centers[index + 1] - centers[index]).head<2>().norm());
    }
    else
    {
        const Vector2d span = Vector2d(m_endX->value() - m_startX->value(), m_endY->value() - m_startY->value());
        spanLengths.push_back(span.norm());
    }
    std::vector<double> positions;
    for (double spanLength : spanLengths)
    {
        const auto current = Conductor::ConductorModelBuilder::CalculateStandardInnerSpacerPositions(spanLength);
        positions.insert(positions.end(), current.begin(), current.end());
    }
    QStringList positionTexts;
    positionTexts.reserve(static_cast<qsizetype>(positions.size()));
    for (double position : positions)
        positionTexts.push_back(QString::number(position, 'f', 2));
    m_spacerCountPreview->setText(QStringLiteral("%1 个（%2 档内置规则）")
                                      .arg(static_cast<qulonglong>(positions.size()))
                                      .arg(static_cast<qulonglong>(spanLengths.size())));
    m_spacerCountPreview->setToolTip(
        positions.empty()
            ? QStringLiteral("当前水平档距无效")
            : (rigidCenterMpc ? topologyDescription(static_cast<int>(positions.size())) +
                                    QStringLiteral("；距左挂点位置：%1 m").arg(positionTexts.join(QStringLiteral("、")))
                              : QStringLiteral("距左挂点位置：%1 m").arg(positionTexts.join(QStringLiteral("、")))) +
                  QLatin1Char('\n') + massDescription);
}

static QString conductorSectionDisplayName(const Conductor::SectionPreset& section)
{
    QString name = section.name.trimmed();
    for (const QString& suffix : {QStringLiteral(" 等效截面"), QStringLiteral(" 等效圆截面")})
    {
        if (name.endsWith(suffix) && name.size() > suffix.size())
        {
            name.chop(suffix.size());
            break;
        }
    }
    return name;
}

static QStringList conductorPresetDisplayNames(const QStringList& names, const QStringList& categories,
                                               const QList<int>& ids)
{
    QStringList labels;
    for (int index = 0; index < names.size(); ++index)
    {
        QString label = names.at(index);
        if (names.count(label) > 1)
        {
            int categoryMatches = 0;
            for (int other = 0; other < names.size(); ++other)
            {
                if (names.at(other) == names.at(index) && categories.at(other) == categories.at(index))
                    ++categoryMatches;
            }
            QString detail = categories.at(index).trimmed();
            if (categoryMatches > 1 || detail.isEmpty())
                detail = detail.isEmpty() ? QString::number(ids.at(index))
                                          : QStringLiteral("%1 %2").arg(detail).arg(ids.at(index));
            label += QStringLiteral(" (%1)").arg(detail);
        }
        labels.append(label);
    }
    return labels;
}

void ConductorModule::setPropertyLibrary(Conductor::PropertyLibrary* library)
{
    m_propertyLibrary = library;
    const QVariant previousSpacerMaterial = m_spacerMaterialCombo->currentData();
    const QVariant previousSpacerSection = m_spacerSectionCombo->currentData();
    const int previousSpacerMaterialIndex = m_spacerMaterialCombo->currentIndex();
    const int previousSpacerSectionIndex = m_spacerSectionCombo->currentIndex();
    disconnect(m_materialLibraryConnection);
    disconnect(m_spacerMaterialLibraryConnection);
    m_materialCombo->clear();
    m_sectionCombo->clear();
    m_spacerMaterialCombo->clear();
    m_spacerSectionCombo->clear();
    setCreationEnabled(library && library->isReady());
    if (!library)
        return;

    QStringList materialNames, materialCategories, sectionNames, sectionCategories;
    QList<int> materialIds, sectionIds;
    for (const auto& material : library->materials())
    {
        materialNames.append(material.name);
        materialCategories.append(material.category);
        materialIds.append(material.id);
    }
    for (const auto& section : library->sections())
    {
        sectionNames.append(conductorSectionDisplayName(section));
        sectionCategories.append(section.category);
        sectionIds.append(section.id);
    }
    const QStringList materialLabels = conductorPresetDisplayNames(materialNames, materialCategories, materialIds);
    const QStringList sectionLabels = conductorPresetDisplayNames(sectionNames, sectionCategories, sectionIds);
    for (int index = 0; index < library->materials().size(); ++index)
    {
        const auto& material = library->materials().at(index);
        const QString text = materialLabels.at(index);
        const QString detail = QStringLiteral("%1 · %2").arg(material.category, material.name);
        m_materialCombo->addItem(text);
        m_materialCombo->setItemData(index, detail, Qt::ToolTipRole);
        m_spacerMaterialCombo->addItem(text, material.id);
        m_spacerMaterialCombo->setItemData(index, detail, Qt::ToolTipRole);
    }
    for (int index = 0; index < library->sections().size(); ++index)
    {
        const auto& section = library->sections().at(index);
        const QString text = sectionLabels.at(index);
        QString detail = QStringLiteral("%1 · %2").arg(section.category, section.name);
        if (!section.description.isEmpty())
            detail += QLatin1Char('\n') + section.description;
        m_sectionCombo->addItem(text);
        m_sectionCombo->setItemData(index, detail, Qt::ToolTipRole);
        m_spacerSectionCombo->addItem(text, section.id);
        m_spacerSectionCombo->setItemData(index, detail, Qt::ToolTipRole);
    }
    int spacerMaterialIndex = m_spacerMaterialCombo->findData(previousSpacerMaterial);
    if (previousSpacerMaterialIndex >= 0)
    {
        if (spacerMaterialIndex < 0)
            spacerMaterialIndex = qMin(previousSpacerMaterialIndex, m_spacerMaterialCombo->count() - 1);
    }
    else
    {
        for (int index = 0; index < library->materials().size(); ++index)
        {
            const auto& material = library->materials().at(index);
            if (material.name == QStringLiteral("ZL102（参考）") || material.name == QStringLiteral("FJZ-445/400"))
            {
                spacerMaterialIndex = index;
                break;
            }
            if (spacerMaterialIndex < 0 && material.category == QStringLiteral("间隔棒"))
                spacerMaterialIndex = index;
        }
    }
    if (spacerMaterialIndex >= 0)
        m_spacerMaterialCombo->setCurrentIndex(spacerMaterialIndex);
    if (previousSpacerSectionIndex >= 0)
    {
        const int spacerSectionIndex = m_spacerSectionCombo->findData(previousSpacerSection);
        m_spacerSectionCombo->setCurrentIndex(
            spacerSectionIndex >= 0 ? spacerSectionIndex
                                    : qMin(previousSpacerSectionIndex, m_spacerSectionCombo->count() - 1));
    }
    for (auto* combo : {m_materialCombo, m_sectionCombo, m_spacerMaterialCombo, m_spacerSectionCombo})
        combo->setToolTip(combo->currentData(Qt::ToolTipRole).toString());
    m_spacerSectionCombo->setToolTip(m_spacerSectionCombo->toolTip() + QLatin1Char('\n') +
                                     m_spacerSectionLabel->toolTip());
    const QList<int> bundleCounts = {1, 2, 4, 6, 8};
    for (int index = 0; index < bundleCounts.size(); ++index)
        m_bundleCombo->setItemData(index, bundleCounts.at(index));

    m_materialLibraryConnection =
        connect(m_materialCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this, library](int index)
                {
                    if (index < 0 || index >= library->materials().size())
                        return;
                    const QString materialName = library->materials().at(index).name;
                    for (int sectionIndex = 0; sectionIndex < library->sections().size(); ++sectionIndex)
                    {
                        if (library->sections().at(sectionIndex).name.startsWith(materialName))
                        {
                            m_sectionCombo->setCurrentIndex(sectionIndex);
                            break;
                        }
                    }
                });
    m_spacerMaterialLibraryConnection =
        connect(m_spacerMaterialCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
                [this, library](int index)
                {
                    if (index < 0 || index >= library->materials().size())
                        return;
                    const QString materialName = library->materials().at(index).name;
                    for (int sectionIndex = 0; sectionIndex < library->sections().size(); ++sectionIndex)
                    {
                        if (library->sections().at(sectionIndex).name.startsWith(materialName))
                        {
                            m_spacerSectionCombo->setCurrentIndex(sectionIndex);
                            break;
                        }
                    }
                });
    updateSpacerPresetDescription();
}

ConductorModule::BuildResult ConductorModule::buildModel(Conductor::PropertyLibrary& library,
                                                         const QString& generatedDirectory) const
{
    BuildResult build;
    if (!library.isReady())
    {
        build.error = QStringLiteral("启动属性库不可用。");
        return build;
    }

    const bool multiSpan = m_modelModeCombo->currentIndex() == 1;
    std::vector<Vector3d> stations;
    Vector3d start(m_startX->value(), m_startY->value(), m_startZ->value());
    Vector3d end(m_endX->value(), m_endY->value(), m_endZ->value());
    if (multiSpan)
    {
        stations = stationCenters(build.error);
        if (stations.empty())
            return build;
        start = stations.front();
        end = stations.back();
    }
    else if ((end - start).head<2>().norm() <= 1.0e-7)
    {
        build.error = QStringLiteral("左右挂点必须具有有效的水平档距。");
        return build;
    }

    build.structure = std::make_shared<StructureData>();
    auto property = library.instantiateProperty(m_materialCombo->currentIndex(), m_sectionCombo->currentIndex(),
                                                *build.structure, build.error);
    if (!property)
    {
        build.structure.reset();
        return build;
    }

    std::shared_ptr<Property> bareHardwareProperty;
    const auto endTopology = static_cast<Conductor::BundleEndTopology>(m_endTopologyCombo->currentData().toInt());
    const bool hasHardware = multiSpan || (m_bundleCombo->currentData().toInt() > 1 &&
                                           endTopology != Conductor::BundleEndTopology::DirectWireSupports);
    if ((m_structuralIceCheck->isChecked() || m_bareDiameterSpin->value() > 0.0 ||
         m_torsionalRigiditySpin->value() > 0.0) &&
        hasHardware)
    {
        bareHardwareProperty = library.instantiateProperty(
            m_materialCombo->currentIndex(), m_sectionCombo->currentIndex(), *build.structure, build.error);
        if (!bareHardwareProperty)
        {
            build.structure.reset();
            return build;
        }
    }

    Vector3d localMassFirstMomentPerLength = Vector3d::Zero();
    if (m_structuralIceCheck->isChecked())
    {
        const auto material = property->m_pMaterial.lock();
        const auto section = property->m_pSection.lock();
        if (!material || !section || material->m_Density <= 0.0 || section->m_Area <= 0.0)
        {
            build.error = QStringLiteral("覆冰导线的材料密度和金属截面积必须大于 0。");
            build.structure.reset();
            return build;
        }
        double bareIy = 0.0, bareIz = 0.0, bareJ = 0.0;
        section->Calculate_I(bareIy, bareIz, bareJ);
        const double bareDensity = material->m_Density;
        const double bareRadius =
            m_bareDiameterSpin->value() > 0.0 ? 0.0005 * m_bareDiameterSpin->value() : section->m_Radius;
        Section_Ice ice;
        ice.m_Area = section->m_Area;
        ice.m_Radius = bareRadius;
        ice.SetIceParameters(0.001 * m_iceThicknessSpin->value(), m_iceCenterAngleSpin->value(),
                             m_iceHalfAngleSpin->value());
        if (!std::isfinite(ice.m_IceArea) || !std::isfinite(ice.m_IcePolarMomentAboutWire) || ice.m_IceArea <= 0.0 ||
            ice.m_IcePolarMomentAboutWire <= 0.0 || bareIy <= 0.0 || bareIz <= 0.0 || bareJ <= 0.0 || bareRadius <= 0.0)
        {
            build.error = QStringLiteral("覆冰截面参数无效，请检查裸线外径、冰厚和角度。");
            build.structure.reset();
            return build;
        }
        const double iceLineMass = ice.m_IceDensity * ice.m_IceArea;
        material->m_Density = bareDensity + iceLineMass / section->m_Area;
        section->m_HasExplicitSectionInertia = true;
        section->m_ExplicitIy = bareIy;
        section->m_ExplicitIz = bareIz;
        section->m_ExplicitJ = bareJ;
        section->m_MassInertiaPerLength =
            Eigen::Vector3d(bareDensity * (bareIy + bareIz) + ice.m_IceDensity * ice.m_IcePolarMomentAboutWire,
                            bareDensity * bareIy + ice.m_IceDensity * ice.m_IceIyyAboutWire,
                            bareDensity * bareIz + ice.m_IceDensity * ice.m_IceIzzAboutWire);
        section->m_Radius = bareRadius;
        if (m_iceEccentricityCheck->isChecked() &&
            static_cast<EnumKeyword::ElementType>(m_elementCombo->currentData().toInt()) ==
                EnumKeyword::ElementType::CABLE)
        {
            localMassFirstMomentPerLength =
                Eigen::Vector3d(0.0, iceLineMass * ice.m_IceCentroidY, iceLineMass * ice.m_IceCentroidZ);
        }
    }
    else if (m_bareDiameterSpin->value() > 0.0)
    {
        const auto section = property->m_pSection.lock();
        if (!section || section->m_Area <= 0.0)
        {
            build.error = QStringLiteral("裸线截面积必须大于 0。");
            build.structure.reset();
            return build;
        }
        double iy = 0.0, iz = 0.0, polarMoment = 0.0;
        section->Calculate_I(iy, iz, polarMoment);
        section->m_HasExplicitSectionInertia = true;
        section->m_ExplicitIy = iy;
        section->m_ExplicitIz = iz;
        section->m_ExplicitJ = polarMoment;
        section->m_Radius = 0.0005 * m_bareDiameterSpin->value();
    }

    if (m_torsionalRigiditySpin->value() > 0.0)
    {
        const auto material = property->m_pMaterial.lock();
        const auto section = property->m_pSection.lock();
        const double shearModulus = material ? material->m_Young / (2.0 * (1.0 + material->m_Poisson)) : 0.0;
        if (!section || !std::isfinite(shearModulus) || shearModulus <= 0.0)
        {
            build.error = QStringLiteral("导线材料剪切模量无效，无法设置扭转刚度 GJ。");
            build.structure.reset();
            return build;
        }
        double iy = 0.0, iz = 0.0, originalJ = 0.0;
        section->Calculate_I(iy, iz, originalJ);
        Q_UNUSED(originalJ);
        if (!std::isfinite(iy) || !std::isfinite(iz) || iy <= 0.0 || iz <= 0.0)
        {
            build.error = QStringLiteral("导线截面惯性矩无效，无法设置扭转刚度 GJ。");
            build.structure.reset();
            return build;
        }
        // 质量极惯量必须由截面质量分布确定。若保持隐式回退，索单元会把
        // 新指定的弹性扭转常数 J 当成质量极惯量，改变 GJ 时质量也跟着改变。
        if (section->m_MassInertiaPerLength.x() <= 0.0)
            section->m_MassInertiaPerLength.x() = material->m_Density * (iy + iz);
        if (section->m_MassInertiaPerLength.y() <= 0.0)
            section->m_MassInertiaPerLength.y() = material->m_Density * iz;
        if (section->m_MassInertiaPerLength.z() <= 0.0)
            section->m_MassInertiaPerLength.z() = material->m_Density * iy;
        section->m_HasExplicitSectionInertia = true;
        section->m_ExplicitIy = iy;
        section->m_ExplicitIz = iz;
        section->m_ExplicitJ = m_torsionalRigiditySpin->value() / shearModulus;
    }

    Conductor::LineBuildConfig config;
    config.start = start;
    config.end = end;
    config.property = property;
    config.localMassFirstMomentPerLength = localMassFirstMomentPerLength;
    config.endFittingProperty = bareHardwareProperty;
    config.elementType = static_cast<EnumKeyword::ElementType>(m_elementCombo->currentData().toInt());
    config.conductor.nBundle = m_bundleCombo->currentData().toInt();
    config.conductor.spacing = m_spacingSpin->value();
    config.conductor.bundleRotationDegrees = config.conductor.nBundle > 1 ? m_bundleRotationSpin->value() : 0.0;
    config.conductor.segments = m_segmentsSpin->value();
    config.conductor.initialStress = m_stressSpin->value() * 1.0e6;
    config.bundleEndTransitionLength = m_endTransitionSpin->value();
    config.endTopology = static_cast<Conductor::BundleEndTopology>(m_endTopologyCombo->currentData().toInt());
    config.dualSupportSpacing = m_dualSupportSpacingSpin->value();
    const QString modelName =
        m_nameEdit->text().trimmed().isEmpty() ? defaultModelName() : m_nameEdit->text().trimmed();
    config.setNamePrefix = modelName;
    // CR3D 端部金具连接仅含平动自由度的导线时，会形成独立转动分量并产生零能滚转模态。
    config.endFittingElementType = config.elementType == EnumKeyword::ElementType::CR3D
                                       ? EnumKeyword::ElementType::CR3D
                                       : EnumKeyword::ElementType::T3D2;

    Conductor::ConductorModelBuilder builder(build.structure);
    Conductor::LineBuildResult lineResult;
    std::string buildError;
    Conductor::SpanConductorBuildConfig spanConfig;
    spanConfig.line = config;
    if (m_innerSpacerCheck->isChecked() && config.conductor.nBundle > 1)
    {
        const auto spacerStyle = static_cast<Conductor::InnerSpacerStyle>(m_spacerStyleCombo->currentData().toInt());
        std::shared_ptr<Property> spacerProperty = library.instantiateProperty(
            m_spacerMaterialCombo->currentIndex(), m_spacerSectionCombo->currentIndex(), *build.structure, build.error);
        if (!spacerProperty)
        {
            build.structure.reset();
            return build;
        }
        Conductor::InnerSpacerConfig spacerConfig;
        spacerConfig.elementType = static_cast<EnumKeyword::ElementType>(m_spacerElementCombo->currentData().toInt());
        spacerConfig.property = spacerProperty;
        spacerConfig.style = spacerStyle;
        spacerConfig.totalMass = m_spacerMassSpin->value();
        if (m_explicitInnerSpacerPositions.empty())
        {
            spanConfig.useInnerSpacerLayout = true;
            spanConfig.innerSpacerLayout.useEqualSpacing = m_spacerLayoutCombo->currentData().toBool();
            spanConfig.innerSpacerLayout.count = m_spacerCountSpin->value();
            spanConfig.innerSpacerLayout.spacer = spacerConfig;
        }
        else
        {
            for (double position : m_explicitInnerSpacerPositions)
            {
                spacerConfig.position = position;
                spanConfig.innerSpacers.push_back(spacerConfig);
            }
        }
    }
    bool built = false;
    if (multiSpan)
    {
        Conductor::MultiSpanConductorBuildConfig multiConfig;
        multiConfig.span = spanConfig;
        multiConfig.stationCenters = stations;
        multiConfig.suspensionStringLength = m_suspensionLengthSpin->value();
        multiConfig.suspensionElementType = EnumKeyword::ElementType::T3D2;
        multiConfig.suspensionProperty = bareHardwareProperty ? bareHardwareProperty : property;
        built = builder.BuildMultiSpanConductor(multiConfig, lineResult, buildError);
    }
    else
    {
        built = builder.BuildSpanConductor(spanConfig, lineResult, buildError);
    }
    if (!built)
    {
        build.error = QString::fromStdString(buildError);
        build.structure.reset();
        return build;
    }
    if (!m_explicitInnerSpacerPositions.empty())
    {
        std::set<int> selectedNodes;
        for (const auto& spacer : lineResult.innerSpacers)
        {
            if (spacer.nodeIds.empty() || !selectedNodes.insert(spacer.nodeIds.front()).second)
            {
                build.error = QStringLiteral("指定的间隔棒位置映射到相同导线节点，请增大位置间距");
                build.structure.reset();
                return build;
            }
        }
    }

    if (m_analysisCheck->isChecked())
    {
        std::set<int> endpointIds;
        if (!lineResult.leftTensionEnd.supportNodeIds.empty() || !lineResult.rightTensionEnd.supportNodeIds.empty())
        {
            endpointIds.insert(lineResult.leftTensionEnd.supportNodeIds.begin(),
                               lineResult.leftTensionEnd.supportNodeIds.end());
            endpointIds.insert(lineResult.rightTensionEnd.supportNodeIds.begin(),
                               lineResult.rightTensionEnd.supportNodeIds.end());
        }
        else
        {
            for (const auto& pair : lineResult.subConductors)
            {
                if (!pair.second.nodeIds.empty())
                {
                    endpointIds.insert(pair.second.nodeIds.front());
                    endpointIds.insert(pair.second.nodeIds.back());
                }
            }
        }
        // 节点尚未经过 AnalysisStep::Init_DOF，应根据实际相连的全部单元确定支座约束。
        auto requiredNodeDofs = [&](int nodeId)
        {
            int required = 3;
            for (const auto& [elementId, element] : build.structure->m_Elements)
            {
                Q_UNUSED(elementId);
                if (!element)
                    continue;
                for (const auto& nodeReference : element->m_pNode)
                {
                    const auto node = nodeReference.lock();
                    if (node && node->m_Id == nodeId)
                    {
                        required = std::max(required, element->Get_NodeDOF());
                        break;
                    }
                }
            }
            return required;
        };
        auto addZeroConstraint = [&](int nodeId, int constrainedDofCount)
        {
            std::vector<int> directions;
            std::vector<double> values;
            directions.reserve(constrainedDofCount);
            values.assign(constrainedDofCount, 0.0);
            for (int direction = 0; direction < constrainedDofCount; ++direction)
                directions.push_back(direction);
            build.structure->Add_Constraint({nodeId}, directions, values);
        };
        for (int endpointId : endpointIds)
            addZeroConstraint(endpointId, requiredNodeDofs(endpointId));
        // 中间 P 节点只连接 H 型悬垂串。当前悬垂串固定采用 T3D2，
        // 因此该节点只存在三个平动自由度，与线身采用索或梁无关。
        for (const auto& suspension : lineResult.suspensionPoints)
        {
            if (suspension.supportNodeId > 0)
                addZeroConstraint(suspension.supportNodeId, 3);
        }
        AnalysisStepConfig step;
        step.id = 1;
        step.type = EnumKeyword::StepType::STATIC;
        step.totalTime = 1.0;
        step.stepSize = 0.01;
        step.tolerance = 1.0e-5;
        step.maxIterations = 50;
        build.structure->AddAnalysisStep(step);
        build.structure->Add_Gravity(2, step.id);
    }

    QString fileName = modelName;
    fileName.replace(QRegularExpression(QStringLiteral("[\\\\/:*?\"<>|]")), QStringLiteral("_"));
    if (!QDir().mkpath(generatedDirectory))
    {
        build.error = QStringLiteral("无法创建输出目录：%1").arg(generatedDirectory);
        build.structure.reset();
        return build;
    }
    const QDir outputDirectory(generatedDirectory);
    build.filePath = outputDirectory.absoluteFilePath(fileName + QStringLiteral(".bdf"));
    for (int sequence = 2; QFileInfo::exists(build.filePath); ++sequence)
        build.filePath = outputDirectory.absoluteFilePath(QStringLiteral("%1_%2.bdf").arg(fileName).arg(sequence));
    Outputter outputter;
    if (!outputter.SaveBdfModel(build.filePath, build.structure.get()))
    {
        build.error = QStringLiteral("模型保存失败：%1").arg(build.filePath);
        build.filePath.clear();
        build.structure.reset();
        return build;
    }

    build.nodeCount = lineResult.NodeCount();
    build.elementCount = lineResult.ElementCount();
    build.spacerCount = static_cast<int>(lineResult.innerSpacers.size());
    return build;
}
QPushButton* ConductorModule::viewLibraryButton() const
{
    return m_viewLibraryButton;
}
QLineEdit* ConductorModule::nameEdit() const
{
    return m_nameEdit;
}
QComboBox* ConductorModule::materialCombo() const
{
    return m_materialCombo;
}
QComboBox* ConductorModule::sectionCombo() const
{
    return m_sectionCombo;
}
QComboBox* ConductorModule::elementCombo() const
{
    return m_elementCombo;
}
QDoubleSpinBox* ConductorModule::startX() const
{
    return m_startX;
}
QDoubleSpinBox* ConductorModule::startY() const
{
    return m_startY;
}
QDoubleSpinBox* ConductorModule::startZ() const
{
    return m_startZ;
}
QDoubleSpinBox* ConductorModule::endX() const
{
    return m_endX;
}
QDoubleSpinBox* ConductorModule::endY() const
{
    return m_endY;
}
QDoubleSpinBox* ConductorModule::endZ() const
{
    return m_endZ;
}
QComboBox* ConductorModule::bundleCombo() const
{
    return m_bundleCombo;
}
QDoubleSpinBox* ConductorModule::spacingSpin() const
{
    return m_spacingSpin;
}
QDoubleSpinBox* ConductorModule::bundleRotationSpin() const
{
    return m_bundleRotationSpin;
}
QSpinBox* ConductorModule::segmentsSpin() const
{
    return m_segmentsSpin;
}

QDoubleSpinBox* ConductorModule::endTransitionSpin() const
{
    return m_endTransitionSpin;
}
QDoubleSpinBox* ConductorModule::stressSpin() const
{
    return m_stressSpin;
}
QCheckBox* ConductorModule::innerSpacerCheck() const
{
    return m_innerSpacerCheck;
}
QComboBox* ConductorModule::spacerLayoutCombo() const
{
    return m_spacerLayoutCombo;
}
QSpinBox* ConductorModule::spacerCountSpin() const
{
    return m_spacerCountSpin;
}
QDoubleSpinBox* ConductorModule::spacerMassSpin() const
{
    return m_spacerMassSpin;
}
QComboBox* ConductorModule::spacerPresetCombo() const
{
    return m_spacerPresetCombo;
}
QDoubleSpinBox* ConductorModule::spacerSpacingSpin() const
{
    return m_spacerSpacingSpin;
}
QLineEdit* ConductorModule::spacerModelEdit() const
{
    return m_spacerModelEdit;
}
QComboBox* ConductorModule::spacerElementCombo() const
{
    return m_spacerElementCombo;
}
QComboBox* ConductorModule::spacerMaterialCombo() const
{
    return m_spacerMaterialCombo;
}
QComboBox* ConductorModule::spacerSectionCombo() const
{
    return m_spacerSectionCombo;
}
QCheckBox* ConductorModule::analysisCheck() const
{
    return m_analysisCheck;
}
QPushButton* ConductorModule::createButton() const
{
    return m_createButton;
}
