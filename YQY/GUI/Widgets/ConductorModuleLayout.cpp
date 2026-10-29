#include "ConductorModule.h"
#include "Widgets/ResultControlInputWidgets.h"
#include "Widgets/CompactDoubleSpinBox.h"
#include <QAbstractItemView>
#include <QButtonGroup>
#include <QCheckBox>
#include <QCoreApplication>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSize>
#include <QSizePolicy>
#include <QSpacerItem>
#include <QString>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

void ConductorModule::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("ConductorModuleClass");
    }
    m_rootLayout = new QVBoxLayout(this);
    m_rootLayout->setSpacing(12);
    m_rootLayout->setObjectName("rootLayout");
    m_rootLayout->setContentsMargins(8, 8, 8, 8);
    m_editorContainer = new QWidget(this);
    m_editorContainer->setObjectName("editorContainer");
    m_editorLayout = new QVBoxLayout(m_editorContainer);
    m_editorLayout->setSpacing(12);
    m_editorLayout->setObjectName("editorLayout");
    m_editorLayout->setContentsMargins(0, 0, 0, 0);
    m_globalBundleLayout = new QHBoxLayout();
    m_globalBundleLayout->setSpacing(10);
    m_globalBundleLayout->setObjectName("globalBundleLayout");
    m_globalBundleLayout->setContentsMargins(0, 0, 0, 0);
    auto* modeRow = new QWidget(m_editorContainer);
    auto* modeLayout = new QHBoxLayout(modeRow);
    modeLayout->setContentsMargins(0, 0, 0, 0);
    modeLayout->setSpacing(10);
    m_singleSpanButton = new QPushButton(m_editorContainer);
    m_singleSpanButton->setObjectName("singleSpanButton");
    m_multiSpanButton = new QPushButton(m_editorContainer);
    m_multiSpanButton->setObjectName("multiSpanButton");
    auto* modeButtons = new QButtonGroup(this);
    modeButtons->setExclusive(true);
    for (auto* button : {m_singleSpanButton, m_multiSpanButton})
    {
        button->setCheckable(true);
        button->setAutoDefault(false);
        button->setProperty("conductorModeSwitch", true);
        button->setMinimumHeight(36);
        button->setCursor(Qt::PointingHandCursor);
        modeButtons->addButton(button);
        modeLayout->addWidget(button);
    }
    m_singleSpanButton->setChecked(true);
    m_globalBundleLayout->addWidget(modeRow);
    m_globalBundleLayout->addStretch();
    auto* bundleRow = new QWidget(m_editorContainer);
    auto* bundleLayout = new QHBoxLayout(bundleRow);
    bundleLayout->setContentsMargins(0, 0, 0, 0);
    bundleLayout->setSpacing(10);
    QLabel* bundleLabel = new QLabel(m_editorContainer);
    bundleLabel->setObjectName("bundleLabel");

    bundleLayout->addWidget(bundleLabel);

    m_bundleCombo = new ChevronComboBox(m_editorContainer);
    m_bundleCombo->addItem(QString());
    m_bundleCombo->addItem(QString());
    m_bundleCombo->addItem(QString());
    m_bundleCombo->addItem(QString());
    m_bundleCombo->addItem(QString());
    m_bundleCombo->setObjectName("bundleCombo");
    m_bundleCombo->setMaximumSize(QSize(220, 16777215));

    bundleLayout->addWidget(m_bundleCombo);
    m_globalBundleLayout->addWidget(bundleRow);

    m_editorLayout->addLayout(m_globalBundleLayout);

    m_creationTabs = new QTabWidget(m_editorContainer);
    m_creationTabs->setObjectName("creationTabs");
    initializeBasicTab();

    initializeAdvancedTab();

    initializeSpacerTab();

    m_editorLayout->addWidget(m_creationTabs);

    m_footerLayout = new QHBoxLayout();
    m_footerLayout->setSpacing(10);
    m_footerLayout->setObjectName("footerLayout");
    m_footerLayout->setContentsMargins(0, 0, 0, 0);
    m_viewLibraryButton = new QPushButton(m_editorContainer);
    m_viewLibraryButton->setObjectName("viewLibraryButton");

    m_footerLayout->addWidget(m_viewLibraryButton);

    m_analysisCheck = new QCheckBox(m_editorContainer);
    m_analysisCheck->setObjectName("analysisCheck");
    m_analysisCheck->setChecked(true);

    m_footerLayout->addWidget(m_analysisCheck);

    QSpacerItem* footerSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

    m_footerLayout->addItem(footerSpacer);

    m_cancelButton = new QPushButton(m_editorContainer);
    m_cancelButton->setObjectName("cancelButton");

    m_footerLayout->addWidget(m_cancelButton);

    m_createButton = new QPushButton(m_editorContainer);
    m_createButton->setObjectName("createButton");
    m_createButton->setMinimumSize(QSize(0, 40));

    m_footerLayout->addWidget(m_createButton);

    m_editorLayout->addLayout(m_footerLayout);

    m_rootLayout->addWidget(m_editorContainer);

    m_singleSpanButton->setText(QCoreApplication::translate("ConductorModuleClass", "单档导线", nullptr));
    m_multiSpanButton->setText(QCoreApplication::translate("ConductorModuleClass", "多档导线", nullptr));
    bundleLabel->setText(QCoreApplication::translate("ConductorModuleClass", "分裂数", nullptr));
    m_bundleCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "1", nullptr));
    m_bundleCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "2", nullptr));
    m_bundleCombo->setItemText(2, QCoreApplication::translate("ConductorModuleClass", "4", nullptr));
    m_bundleCombo->setItemText(3, QCoreApplication::translate("ConductorModuleClass", "6", nullptr));
    m_bundleCombo->setItemText(4, QCoreApplication::translate("ConductorModuleClass", "8", nullptr));
    QTableWidgetItem* ___qtablewidgetitem = m_stationTable->horizontalHeaderItem(0);
    ___qtablewidgetitem->setText(QCoreApplication::translate("ConductorModuleClass", "序号", nullptr));
    QTableWidgetItem* ___qtablewidgetitem1 = m_stationTable->horizontalHeaderItem(1);
    ___qtablewidgetitem1->setText(QCoreApplication::translate("ConductorModuleClass", "类型", nullptr));
    QTableWidgetItem* ___qtablewidgetitem2 = m_stationTable->horizontalHeaderItem(2);
    ___qtablewidgetitem2->setText(QCoreApplication::translate("ConductorModuleClass", "X (m)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem3 = m_stationTable->horizontalHeaderItem(3);
    ___qtablewidgetitem3->setText(QCoreApplication::translate("ConductorModuleClass", "Y (m)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem4 = m_stationTable->horizontalHeaderItem(4);
    ___qtablewidgetitem4->setText(QCoreApplication::translate("ConductorModuleClass", "Z (m)", nullptr));
    m_creationTabs->setTabText(m_creationTabs->indexOf(m_basicTab),
                               QCoreApplication::translate("ConductorModuleClass", "基础与坐标", nullptr));
    m_creationTabs->setTabText(m_creationTabs->indexOf(m_spacerTab),
                               QCoreApplication::translate("ConductorModuleClass", "间隔棒", nullptr));
    m_viewLibraryButton->setText(QCoreApplication::translate("ConductorModuleClass", "查看材料与截面", nullptr));
    m_analysisCheck->setText(QCoreApplication::translate("ConductorModuleClass", "同时创建静力分析步与约束", nullptr));
    m_analysisCheck->setToolTip(
        QCoreApplication::translate("ConductorModuleClass", "创建静力分析步、重力和两端 XYZ 约束", nullptr));
    m_cancelButton->setText(QCoreApplication::translate("ConductorModuleClass", "取消", nullptr));
    m_createButton->setText(QCoreApplication::translate("ConductorModuleClass", "创建并导入", nullptr));

    m_bundleCombo->setCurrentIndex(2);
    m_creationTabs->setCurrentIndex(0);
}

void ConductorModule::initializePropertyGroup()
{
    m_propertyGroup = new QGroupBox(m_content);
    m_propertyGroup->setObjectName("propertyGroup");
    m_propertyLayout = new QVBoxLayout(m_propertyGroup);
    m_propertyLayout->setSpacing(0);
    m_propertyLayout->setObjectName("propertyLayout");
    m_propertyLayout->setContentsMargins(0, 0, 0, 0);
    QWidget* materialForm = new QWidget(m_propertyGroup);
    materialForm->setObjectName("materialForm");
    m_materialFormLayout = new QFormLayout(materialForm);
    m_materialFormLayout->setSpacing(8);
    m_materialFormLayout->setObjectName("materialFormLayout");
    m_materialFormLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_materialFormLayout->setRowWrapPolicy(QFormLayout::DontWrapRows);
    m_materialFormLayout->setContentsMargins(14, 14, 14, 14);
    QLabel* nameLabel = new QLabel(materialForm);
    nameLabel->setObjectName("nameLabel");

    m_materialFormLayout->setWidget(0, QFormLayout::LabelRole, nameLabel);

    m_nameEdit = new QLineEdit(materialForm);
    m_nameEdit->setObjectName("nameEdit");

    m_materialFormLayout->setWidget(0, QFormLayout::FieldRole, m_nameEdit);

    QLabel* materialLabel = new QLabel(materialForm);
    materialLabel->setObjectName("materialLabel");

    m_materialFormLayout->setWidget(1, QFormLayout::LabelRole, materialLabel);

    m_materialCombo = new ChevronComboBox(materialForm);
    m_materialCombo->setObjectName("materialCombo");

    m_materialFormLayout->setWidget(1, QFormLayout::FieldRole, m_materialCombo);

    QLabel* sectionLabel = new QLabel(materialForm);
    sectionLabel->setObjectName("sectionLabel");

    m_materialFormLayout->setWidget(2, QFormLayout::LabelRole, sectionLabel);

    m_sectionCombo = new ChevronComboBox(materialForm);
    m_sectionCombo->setObjectName("sectionCombo");

    m_materialFormLayout->setWidget(2, QFormLayout::FieldRole, m_sectionCombo);

    QLabel* elementLabel = new QLabel(materialForm);
    elementLabel->setObjectName("elementLabel");

    m_materialFormLayout->setWidget(3, QFormLayout::LabelRole, elementLabel);

    m_elementCombo = new ChevronComboBox(materialForm);
    m_elementCombo->addItem(QString());
    m_elementCombo->addItem(QString());
    m_elementCombo->addItem(QString());
    m_elementCombo->setObjectName("elementCombo");

    m_materialFormLayout->setWidget(3, QFormLayout::FieldRole, m_elementCombo);

    m_propertyLayout->addWidget(materialForm);

    m_propertySummary = new QLabel(m_propertyGroup);
    m_propertySummary->setObjectName("propertySummary");
    m_propertySummary->setWordWrap(true);

    m_propertyLayout->addWidget(m_propertySummary);

    m_contentLayout->addWidget(m_propertyGroup, 0, 0, 1, 1);

    nameLabel->setText(QCoreApplication::translate("ConductorModuleClass", "模型名称", nullptr));
    materialLabel->setText(QCoreApplication::translate("ConductorModuleClass", "材料", nullptr));
    sectionLabel->setText(QCoreApplication::translate("ConductorModuleClass", "截面", nullptr));
    elementLabel->setText(QCoreApplication::translate("ConductorModuleClass", "单元类型", nullptr));

    m_propertyGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "模型与属性", nullptr));
    m_nameEdit->setText(QCoreApplication::translate("ConductorModuleClass", "单导线", nullptr));
    m_nameEdit->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass", "默认名称随分裂数和档位模式更新；自定义名称保留，同名文件自动添加序号。", nullptr));
    m_elementCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "CABLE 共旋索单元", nullptr));
    m_elementCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "T3D2 桁架单元", nullptr));
    m_elementCombo->setItemText(2, QCoreApplication::translate("ConductorModuleClass", "CR3D 共旋梁单元", nullptr));

    m_propertySummary->setText(
        QCoreApplication::translate("ConductorModuleClass", "建模时复制为模型独立属性，可在属性页编辑预设。", nullptr));
}

void ConductorModule::initializeGeometryGroup()
{
    m_formGroup = new QGroupBox(m_content);
    m_formGroup->setObjectName("formGroup");
    m_formLayout = new QFormLayout(m_formGroup);
    m_formLayout->setSpacing(8);
    m_formLayout->setObjectName("formLayout");
    m_formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_formLayout->setRowWrapPolicy(QFormLayout::DontWrapRows);
    m_formLayout->setContentsMargins(14, 14, 14, 14);
    QLabel* spacingLabel = new QLabel(m_formGroup);
    spacingLabel->setObjectName("spacingLabel");

    m_formLayout->setWidget(0, QFormLayout::LabelRole, spacingLabel);

    m_spacingSpin = new ChevronDoubleSpinBox(m_formGroup);
    m_spacingSpin->setObjectName("spacingSpin");
    m_spacingSpin->setMinimum(0.001000000000000);
    m_spacingSpin->setMaximum(20.000000000000000);
    m_spacingSpin->setDecimals(3);
    m_spacingSpin->setValue(0.400000000000000);

    m_formLayout->setWidget(0, QFormLayout::FieldRole, m_spacingSpin);

    m_bundleRotationLabel = new QLabel(m_formGroup);
    m_bundleRotationLabel->setObjectName("bundleRotationLabel");

    m_formLayout->setWidget(1, QFormLayout::LabelRole, m_bundleRotationLabel);

    m_bundleRotationSpin = new ChevronDoubleSpinBox(m_formGroup);
    m_bundleRotationSpin->setObjectName("bundleRotationSpin");
    m_bundleRotationSpin->setMinimum(-360.000000000000000);
    m_bundleRotationSpin->setMaximum(360.000000000000000);
    m_bundleRotationSpin->setDecimals(6);
    m_bundleRotationSpin->setSingleStep(0.500000000000000);

    m_formLayout->setWidget(1, QFormLayout::FieldRole, m_bundleRotationSpin);

    m_segmentsLabel = new QLabel(m_formGroup);
    m_segmentsLabel->setObjectName("segmentsLabel");

    m_formLayout->setWidget(2, QFormLayout::LabelRole, m_segmentsLabel);

    m_segmentsSpin = new ChevronSpinBox(m_formGroup);
    m_segmentsSpin->setObjectName("segmentsSpin");
    m_segmentsSpin->setMinimum(2);
    m_segmentsSpin->setMaximum(10000);
    m_segmentsSpin->setValue(50);

    m_formLayout->setWidget(2, QFormLayout::FieldRole, m_segmentsSpin);

    m_contentLayout->addWidget(m_formGroup, 0, 1, 1, 1);

    spacingLabel->setText(QCoreApplication::translate("ConductorModuleClass", "子导线间距", nullptr));

    m_formGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "分裂与离散", nullptr));
    m_spacingSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " m", nullptr));
    m_bundleRotationLabel->setText(QCoreApplication::translate("ConductorModuleClass", "束截面旋转", nullptr));
    m_bundleRotationSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " °", nullptr));
    m_bundleRotationSpin->setToolTip(
        QCoreApplication::translate("ConductorModuleClass",
                                    "仅旋转子导线几何，不自动改变气动初始攻角或覆冰中心角。正角从规范化档距的局部右方向"
                                    "转向竖直向上；现有端部拓扑要求绝对值小于 180/分裂数 度。",
                                    nullptr));
    m_segmentsLabel->setText(QCoreApplication::translate("ConductorModuleClass", "每根导线段数", nullptr));
}

void ConductorModule::initializeSpanGroup()
{
    m_spanDefinitionGroup = new QGroupBox(m_content);
    m_spanDefinitionGroup->setObjectName("spanDefinitionGroup");
    m_spanDefinitionLayout = new QVBoxLayout(m_spanDefinitionGroup);
    m_spanDefinitionLayout->setSpacing(6);
    m_spanDefinitionLayout->setObjectName("spanDefinitionLayout");
    m_spanDefinitionLayout->setContentsMargins(14, 14, 14, 14);
    m_modelModeCombo = new ChevronComboBox(m_spanDefinitionGroup);
    m_modelModeCombo->addItem(QString());
    m_modelModeCombo->addItem(QString());
    m_modelModeCombo->setObjectName("modelModeCombo");
    m_modelModeCombo->setVisible(false);

    m_spanDefinitionLayout->addWidget(m_modelModeCombo);

    m_singleSpanEditor = new QWidget(m_spanDefinitionGroup);
    m_singleSpanEditor->setObjectName("singleSpanEditor");
    m_coordinateFormLayout = new QGridLayout(m_singleSpanEditor);
    m_coordinateFormLayout->setSpacing(8);
    m_coordinateFormLayout->setObjectName("coordinateFormLayout");
    m_coordinateFormLayout->setContentsMargins(0, 0, 0, 0);
    m_startLabel = new QLabel(m_singleSpanEditor);
    m_startLabel->setObjectName("startLabel");

    m_coordinateFormLayout->addWidget(m_startLabel, 0, 0, 1, 1);

    m_startCoordinates = new QWidget(m_singleSpanEditor);
    m_startCoordinates->setObjectName("startCoordinates");
    m_startLayout = new QVBoxLayout(m_startCoordinates);
    m_startLayout->setSpacing(6);
    m_startLayout->setObjectName("startLayout");
    m_startLayout->setContentsMargins(0, 0, 0, 0);
    m_startX = new ChevronDoubleSpinBox(m_startCoordinates);
    m_startX->setObjectName("startX");
    m_startX->setMinimum(-1000000.000000000000000);
    m_startX->setMaximum(1000000.000000000000000);
    m_startX->setDecimals(3);

    m_startLayout->addWidget(m_startX);

    m_startY = new ChevronDoubleSpinBox(m_startCoordinates);
    m_startY->setObjectName("startY");
    m_startY->setMinimum(-1000000.000000000000000);
    m_startY->setMaximum(1000000.000000000000000);
    m_startY->setDecimals(3);

    m_startLayout->addWidget(m_startY);

    m_startZ = new ChevronDoubleSpinBox(m_startCoordinates);
    m_startZ->setObjectName("startZ");
    m_startZ->setMinimum(-1000000.000000000000000);
    m_startZ->setMaximum(1000000.000000000000000);
    m_startZ->setDecimals(3);

    m_startLayout->addWidget(m_startZ);

    m_coordinateFormLayout->addWidget(m_startCoordinates, 1, 0, 1, 1);

    m_endLabel = new QLabel(m_singleSpanEditor);
    m_endLabel->setObjectName("endLabel");

    m_coordinateFormLayout->addWidget(m_endLabel, 0, 1, 1, 1);

    m_endCoordinates = new QWidget(m_singleSpanEditor);
    m_endCoordinates->setObjectName("endCoordinates");
    m_endLayout = new QVBoxLayout(m_endCoordinates);
    m_endLayout->setSpacing(6);
    m_endLayout->setObjectName("endLayout");
    m_endLayout->setContentsMargins(0, 0, 0, 0);
    m_endX = new ChevronDoubleSpinBox(m_endCoordinates);
    m_endX->setObjectName("endX");
    m_endX->setMinimum(-1000000.000000000000000);
    m_endX->setMaximum(1000000.000000000000000);
    m_endX->setDecimals(3);
    m_endX->setValue(300.000000000000000);

    m_endLayout->addWidget(m_endX);

    m_endY = new ChevronDoubleSpinBox(m_endCoordinates);
    m_endY->setObjectName("endY");
    m_endY->setMinimum(-1000000.000000000000000);
    m_endY->setMaximum(1000000.000000000000000);
    m_endY->setDecimals(3);

    m_endLayout->addWidget(m_endY);

    m_endZ = new ChevronDoubleSpinBox(m_endCoordinates);
    m_endZ->setObjectName("endZ");
    m_endZ->setMinimum(-1000000.000000000000000);
    m_endZ->setMaximum(1000000.000000000000000);
    m_endZ->setDecimals(3);

    m_endLayout->addWidget(m_endZ);

    m_coordinateFormLayout->addWidget(m_endCoordinates, 1, 1, 1, 1);

    m_coordinateFormLayout->setColumnStretch(0, 1);
    m_coordinateFormLayout->setColumnStretch(1, 1);

    m_spanDefinitionLayout->addWidget(m_singleSpanEditor);

    m_multiSpanEditor = new QWidget(m_spanDefinitionGroup);
    m_multiSpanEditor->setObjectName("multiSpanEditor");
    QVBoxLayout* multiSpanEditorLayout = new QVBoxLayout(m_multiSpanEditor);
    multiSpanEditorLayout->setObjectName("multiSpanEditorLayout");
    multiSpanEditorLayout->setContentsMargins(0, 0, 0, 0);
    m_stationHelpLabel = new QLabel(m_multiSpanEditor);
    m_stationHelpLabel->setObjectName("stationHelpLabel");
    m_stationHelpLabel->setWordWrap(true);

    multiSpanEditorLayout->addWidget(m_stationHelpLabel);

    m_stationTable = new QTableWidget(m_multiSpanEditor);
    if (m_stationTable->columnCount() < 5)
        m_stationTable->setColumnCount(5);
    QTableWidgetItem* __qtablewidgetitem = new QTableWidgetItem();
    m_stationTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
    QTableWidgetItem* __qtablewidgetitem1 = new QTableWidgetItem();
    m_stationTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
    QTableWidgetItem* __qtablewidgetitem2 = new QTableWidgetItem();
    m_stationTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
    QTableWidgetItem* __qtablewidgetitem3 = new QTableWidgetItem();
    m_stationTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
    QTableWidgetItem* __qtablewidgetitem4 = new QTableWidgetItem();
    m_stationTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
    m_stationTable->setObjectName("stationTable");
    m_stationTable->setMinimumSize(QSize(0, 170));
    m_stationTable->setAlternatingRowColors(true);
    m_stationTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_stationTable->setColumnCount(5);

    multiSpanEditorLayout->addWidget(m_stationTable);

    m_stationButtonLayout = new QHBoxLayout();
    m_stationButtonLayout->setObjectName("stationButtonLayout");
    m_addStationButton = new QPushButton(m_multiSpanEditor);
    m_addStationButton->setObjectName("addStationButton");

    m_stationButtonLayout->addWidget(m_addStationButton);

    m_removeStationButton = new QPushButton(m_multiSpanEditor);
    m_removeStationButton->setObjectName("removeStationButton");

    m_stationButtonLayout->addWidget(m_removeStationButton);

    m_suspensionFormLayout = new QFormLayout();
    m_suspensionFormLayout->setObjectName("suspensionFormLayout");
    QLabel* suspensionLengthLabel = new QLabel(m_multiSpanEditor);
    suspensionLengthLabel->setObjectName("suspensionLengthLabel");

    m_suspensionFormLayout->setWidget(0, QFormLayout::LabelRole, suspensionLengthLabel);

    m_suspensionLengthSpin = new ChevronDoubleSpinBox(m_multiSpanEditor);
    m_suspensionLengthSpin->setObjectName("suspensionLengthSpin");
    m_suspensionLengthSpin->setMinimum(0.001000000000000);
    m_suspensionLengthSpin->setMaximum(1000.000000000000000);
    m_suspensionLengthSpin->setDecimals(3);
    m_suspensionLengthSpin->setValue(1.000000000000000);

    m_suspensionFormLayout->setWidget(0, QFormLayout::FieldRole, m_suspensionLengthSpin);

    m_stationButtonLayout->addLayout(m_suspensionFormLayout);

    multiSpanEditorLayout->addLayout(m_stationButtonLayout);

    m_spanDefinitionLayout->addWidget(m_multiSpanEditor);

    m_contentLayout->addWidget(m_spanDefinitionGroup, 1, 0, 1, 1);

    suspensionLengthLabel->setText(QCoreApplication::translate("ConductorModuleClass", "悬垂串长", nullptr));

    m_spanDefinitionGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "挂点坐标", nullptr));
    m_modelModeCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "单档导线", nullptr));
    m_modelModeCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "多档导线", nullptr));

    m_startLabel->setText(QCoreApplication::translate("ConductorModuleClass", "左挂点 (m)", nullptr));
    m_startX->setPrefix(QCoreApplication::translate("ConductorModuleClass", "X ", nullptr));
    m_startY->setPrefix(QCoreApplication::translate("ConductorModuleClass", "Y ", nullptr));
    m_startZ->setPrefix(QCoreApplication::translate("ConductorModuleClass", "Z ", nullptr));
    m_endLabel->setText(QCoreApplication::translate("ConductorModuleClass", "右挂点 (m)", nullptr));
    m_endX->setPrefix(QCoreApplication::translate("ConductorModuleClass", "X ", nullptr));
    m_endY->setPrefix(QCoreApplication::translate("ConductorModuleClass", "Y ", nullptr));
    m_endZ->setPrefix(QCoreApplication::translate("ConductorModuleClass", "Z ", nullptr));
    m_stationHelpLabel->setText(
        QCoreApplication::translate("ConductorModuleClass", "坐标为各塔位处的导线束/分裂多边形中心", nullptr));
    m_addStationButton->setText(QCoreApplication::translate("ConductorModuleClass", "增加塔位", nullptr));
    m_removeStationButton->setText(QCoreApplication::translate("ConductorModuleClass", "删除塔位", nullptr));
    m_suspensionLengthSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " m", nullptr));
}

void ConductorModule::initializeEndConnectionGroup()
{
    m_endConnectionGroup = new QGroupBox(m_content);
    m_endConnectionGroup->setObjectName("endConnectionGroup");
    m_endConnectionFormLayout = new QFormLayout(m_endConnectionGroup);
    m_endConnectionFormLayout->setSpacing(8);
    m_endConnectionFormLayout->setObjectName("endConnectionFormLayout");
    m_endConnectionFormLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_endConnectionFormLayout->setRowWrapPolicy(QFormLayout::DontWrapRows);
    m_endConnectionFormLayout->setContentsMargins(14, 14, 14, 14);
    QLabel* endTopologyLabel = new QLabel(m_endConnectionGroup);
    endTopologyLabel->setObjectName("endTopologyLabel");

    m_endConnectionFormLayout->setWidget(0, QFormLayout::LabelRole, endTopologyLabel);

    m_endTopologyCombo = new ChevronComboBox(m_endConnectionGroup);
    m_endTopologyCombo->addItem(QString());
    m_endTopologyCombo->addItem(QString());
    m_endTopologyCombo->addItem(QString());
    m_endTopologyCombo->setObjectName("endTopologyCombo");

    m_endConnectionFormLayout->setWidget(0, QFormLayout::FieldRole, m_endTopologyCombo);

    m_dualSupportSpacingLabel = new QLabel(m_endConnectionGroup);
    m_dualSupportSpacingLabel->setObjectName("dualSupportSpacingLabel");

    m_endConnectionFormLayout->setWidget(1, QFormLayout::LabelRole, m_dualSupportSpacingLabel);

    m_dualSupportSpacingSpin = new ChevronDoubleSpinBox(m_endConnectionGroup);
    m_dualSupportSpacingSpin->setObjectName("dualSupportSpacingSpin");
    m_dualSupportSpacingSpin->setMinimum(0.001000000000000);
    m_dualSupportSpacingSpin->setMaximum(100.000000000000000);
    m_dualSupportSpacingSpin->setDecimals(3);
    m_dualSupportSpacingSpin->setValue(0.450000000000000);

    m_endConnectionFormLayout->setWidget(1, QFormLayout::FieldRole, m_dualSupportSpacingSpin);

    QLabel* endTransitionLabel = new QLabel(m_endConnectionGroup);
    endTransitionLabel->setObjectName("endTransitionLabel");

    m_endConnectionFormLayout->setWidget(2, QFormLayout::LabelRole, endTransitionLabel);

    m_endTransitionSpin = new ChevronDoubleSpinBox(m_endConnectionGroup);
    m_endTransitionSpin->setObjectName("endTransitionSpin");
    m_endTransitionSpin->setMinimum(0.001000000000000);
    m_endTransitionSpin->setMaximum(1000.000000000000000);
    m_endTransitionSpin->setDecimals(3);
    m_endTransitionSpin->setSingleStep(0.100000000000000);
    m_endTransitionSpin->setValue(1.000000000000000);

    m_endConnectionFormLayout->setWidget(2, QFormLayout::FieldRole, m_endTransitionSpin);

    QLabel* stressLabel = new QLabel(m_endConnectionGroup);
    stressLabel->setObjectName("stressLabel");

    m_endConnectionFormLayout->setWidget(3, QFormLayout::LabelRole, stressLabel);

    m_stressSpin = new ChevronDoubleSpinBox(m_endConnectionGroup);
    m_stressSpin->setObjectName("stressSpin");
    m_stressSpin->setMinimum(0.001000000000000);
    m_stressSpin->setMaximum(10000.000000000000000);
    m_stressSpin->setDecimals(3);
    m_stressSpin->setValue(50.000000000000000);

    m_endConnectionFormLayout->setWidget(3, QFormLayout::FieldRole, m_stressSpin);

    m_contentLayout->addWidget(m_endConnectionGroup, 1, 1, 1, 1);

    endTopologyLabel->setText(QCoreApplication::translate("ConductorModuleClass", "端部形式", nullptr));
    endTransitionLabel->setText(QCoreApplication::translate("ConductorModuleClass", "端部首段长度", nullptr));
    stressLabel->setText(QCoreApplication::translate("ConductorModuleClass", "初始应力", nullptr));

    m_endConnectionGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "端部与初始状态", nullptr));
    m_endTopologyCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "汇集单挂点", nullptr));
    m_endTopologyCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "分组双挂点", nullptr));
    m_endTopologyCombo->setItemText(2, QCoreApplication::translate("ConductorModuleClass", "子导线独立挂点", nullptr));

    m_dualSupportSpacingLabel->setText(QCoreApplication::translate("ConductorModuleClass", "双挂点间距", nullptr));
    m_dualSupportSpacingSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " m", nullptr));
    m_endTransitionSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " m", nullptr));
    m_endTransitionSpin->setToolTip(
        QCoreApplication::translate("ConductorModuleClass",
                                    "公共挂点到首个导线截面的长度；短端段会激发高频扭转并限制自适应步长。可试与主体段长"
                                    "接近的值，并复核舞动时程；独立挂点形式不使用此值",
                                    nullptr));
    m_stressSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " MPa", nullptr));
}

void ConductorModule::initializeStructuralIceGroup()
{
    m_structuralIceGroup = new QGroupBox(m_advancedContent);
    m_structuralIceGroup->setObjectName("structuralIceGroup");
    m_structuralIceFormLayout = new QFormLayout(m_structuralIceGroup);
    m_structuralIceFormLayout->setObjectName("structuralIceFormLayout");
    m_structuralIceFormLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_structuralIceCheck = new QCheckBox(m_structuralIceGroup);
    m_structuralIceCheck->setObjectName("structuralIceCheck");

    m_structuralIceFormLayout->setWidget(0, QFormLayout::SpanningRole, m_structuralIceCheck);

    QLabel* iceThicknessLabel = new QLabel(m_structuralIceGroup);
    iceThicknessLabel->setObjectName("iceThicknessLabel");

    m_structuralIceFormLayout->setWidget(1, QFormLayout::LabelRole, iceThicknessLabel);

    m_iceThicknessSpin = new ChevronDoubleSpinBox(m_structuralIceGroup);
    m_iceThicknessSpin->setObjectName("iceThicknessSpin");
    m_iceThicknessSpin->setMinimum(0.100000000000000);
    m_iceThicknessSpin->setMaximum(100.000000000000000);
    m_iceThicknessSpin->setDecimals(1);
    m_iceThicknessSpin->setValue(12.000000000000000);

    m_structuralIceFormLayout->setWidget(1, QFormLayout::FieldRole, m_iceThicknessSpin);

    QLabel* bareDiameterLabel = new QLabel(m_structuralIceGroup);
    bareDiameterLabel->setObjectName("bareDiameterLabel");

    m_structuralIceFormLayout->setWidget(2, QFormLayout::LabelRole, bareDiameterLabel);

    m_bareDiameterSpin = new ChevronDoubleSpinBox(m_structuralIceGroup);
    m_bareDiameterSpin->setObjectName("bareDiameterSpin");
    m_bareDiameterSpin->setMinimum(0.000000000000000);
    m_bareDiameterSpin->setMaximum(1000.000000000000000);
    m_bareDiameterSpin->setDecimals(2);
    m_bareDiameterSpin->setSingleStep(0.100000000000000);

    m_structuralIceFormLayout->setWidget(2, QFormLayout::FieldRole, m_bareDiameterSpin);

    m_iceEccentricityCheck = new QCheckBox(m_structuralIceGroup);
    m_iceEccentricityCheck->setObjectName("iceEccentricityCheck");
    m_iceEccentricityCheck->setChecked(true);

    m_structuralIceFormLayout->setWidget(3, QFormLayout::SpanningRole, m_iceEccentricityCheck);

    m_advancedContentLayout->addWidget(m_structuralIceGroup, 0, 0, 1, 1);

    iceThicknessLabel->setText(QCoreApplication::translate("ConductorModuleClass", "最大冰厚", nullptr));
    bareDiameterLabel->setText(QCoreApplication::translate("ConductorModuleClass", "裸线实际外径", nullptr));

    m_structuralIceGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "结构覆冰", nullptr));
    m_structuralIceCheck->setText(
        QCoreApplication::translate("ConductorModuleClass", "将覆冰质量计入导线模型", nullptr));
    m_structuralIceCheck->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass",
        "材料密度应为裸线密度；按新月形冰层计算线质量、转动惯量和初始悬链线。气动系数的冰厚在分析步单独选择", nullptr));
    m_iceThicknessSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " mm", nullptr));
    m_iceThicknessSpin->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass",
        "导线局部截面上的最大径向冰厚；从冰层中心按余弦规律减薄到覆盖边界的 0。导线改变走向时冰厚不变", nullptr));
    m_bareDiameterSpin->setSpecialValueText(
        QCoreApplication::translate("ConductorModuleClass", "按金属面积等效圆", nullptr));
    m_bareDiameterSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " mm", nullptr));
    m_bareDiameterSpin->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass",
        "气动力参考宽度采用裸线实际外径；覆冰开启后也用于冰层积分。0 表示按金属面积计算等效圆直径", nullptr));
    m_iceEccentricityCheck->setText(
        QCoreApplication::translate("ConductorModuleClass", "计入偏心质量与重力矩", nullptr));
    m_iceEccentricityCheck->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass", "计入覆冰偏心质量及相应重力矩。仅在启用结构覆冰且导线采用 CABLE 共旋索单元时可用。",
        nullptr));
}

void ConductorModule::initializeTorsionGroup()
{
    m_torsionGroup = new QGroupBox(m_advancedContent);
    m_torsionGroup->setObjectName("torsionGroup");
    m_torsionFormLayout = new QFormLayout(m_torsionGroup);
    m_torsionFormLayout->setObjectName("torsionFormLayout");
    QLabel* torsionalRigidityLabel = new QLabel(m_torsionGroup);
    torsionalRigidityLabel->setObjectName("torsionalRigidityLabel");

    m_torsionFormLayout->setWidget(2, QFormLayout::LabelRole, torsionalRigidityLabel);

    m_torsionalRigiditySpin = new ChevronDoubleSpinBox(m_torsionGroup);
    m_torsionalRigiditySpin->setObjectName("torsionalRigiditySpin");
    m_torsionalRigiditySpin->setMinimum(0.000000000000000);
    m_torsionalRigiditySpin->setMaximum(100000000.000000000000000);
    m_torsionalRigiditySpin->setDecimals(3);
    m_torsionalRigiditySpin->setValue(0.000000000000000);

    m_torsionFormLayout->setWidget(2, QFormLayout::FieldRole, m_torsionalRigiditySpin);

    QLabel* iceCenterAngleLabel = new QLabel(m_torsionGroup);
    iceCenterAngleLabel->setObjectName("iceCenterAngleLabel");

    m_torsionFormLayout->setWidget(0, QFormLayout::LabelRole, iceCenterAngleLabel);

    m_iceCenterAngleSpin = new ChevronDoubleSpinBox(m_torsionGroup);
    m_iceCenterAngleSpin->setObjectName("iceCenterAngleSpin");
    m_iceCenterAngleSpin->setMinimum(-180.000000000000000);
    m_iceCenterAngleSpin->setMaximum(180.000000000000000);
    m_iceCenterAngleSpin->setDecimals(1);
    m_iceCenterAngleSpin->setValue(-45.000000000000000);

    m_torsionFormLayout->setWidget(0, QFormLayout::FieldRole, m_iceCenterAngleSpin);

    QLabel* iceHalfAngleLabel = new QLabel(m_torsionGroup);
    iceHalfAngleLabel->setObjectName("iceHalfAngleLabel");

    m_torsionFormLayout->setWidget(1, QFormLayout::LabelRole, iceHalfAngleLabel);

    m_iceHalfAngleSpin = new ChevronDoubleSpinBox(m_torsionGroup);
    m_iceHalfAngleSpin->setObjectName("iceHalfAngleSpin");
    m_iceHalfAngleSpin->setMinimum(1.000000000000000);
    m_iceHalfAngleSpin->setMaximum(180.000000000000000);
    m_iceHalfAngleSpin->setDecimals(1);
    m_iceHalfAngleSpin->setValue(90.000000000000000);

    m_torsionFormLayout->setWidget(1, QFormLayout::FieldRole, m_iceHalfAngleSpin);

    m_advancedContentLayout->addWidget(m_torsionGroup, 0, 1, 1, 1);

    torsionalRigidityLabel->setText(QCoreApplication::translate("ConductorModuleClass", "导线扭转刚度 GJ", nullptr));
    iceCenterAngleLabel->setText(QCoreApplication::translate("ConductorModuleClass", "冰层中心角", nullptr));
    iceHalfAngleLabel->setText(QCoreApplication::translate("ConductorModuleClass", "冰层半角", nullptr));

    m_torsionGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "冰形与扭转", nullptr));
    m_torsionalRigiditySpin->setSpecialValueText(
        QCoreApplication::translate("ConductorModuleClass", "按材料与截面自动计算", nullptr));
    m_torsionalRigiditySpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " N·m²", nullptr));
    m_torsionalRigiditySpin->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass",
        "可输入实测或文献给出的导线扭转刚度；0 表示由材料剪切模量和截面扭转常数计算。此值不改变覆冰质量和转动惯量",
        nullptr));
    m_iceCenterAngleSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " °", nullptr));
    m_iceCenterAngleSpin->setToolTip(
        QCoreApplication::translate("ConductorModuleClass",
                                    "按每段导线的初始局部截面定义：0° 朝局部 -y，正角朝局部 +z。局部 x 为规范索轴，+z "
                                    "为全局向上在截面上的投影，+y = +z × +x；随导线走向转换到全局坐标",
                                    nullptr));
    m_iceHalfAngleSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " °", nullptr));
}

void ConductorModule::initializeSpacerPropertyGroup()
{
    m_spacerPropertyGroup = new QGroupBox(m_spacerContent);
    m_spacerPropertyGroup->setObjectName("spacerPropertyGroup");
    m_spacerPropertyFormLayout = new QFormLayout(m_spacerPropertyGroup);
    m_spacerPropertyFormLayout->setObjectName("spacerPropertyFormLayout");
    m_spacerPropertyFormLayout->setHorizontalSpacing(0);
    m_spacerPropertyFormLayout->setVerticalSpacing(6);
    m_spacerPropertyFormLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_spacerPropertyFormLayout->setRowWrapPolicy(QFormLayout::DontWrapRows);
    m_spacerPropertyFormLayout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_spacerPropertyFormLayout->setContentsMargins(14, 14, 14, 14);
    m_spacerPresetLabel = new QLabel(m_spacerPropertyGroup);
    m_spacerPresetLabel->setObjectName("spacerPresetLabel");

    m_spacerPropertyFormLayout->setWidget(0, QFormLayout::SpanningRole, m_spacerPresetLabel);

    m_spacerPresetCombo = new ChevronComboBox(m_spacerPropertyGroup);
    m_spacerPresetCombo->setObjectName("spacerPresetCombo");
    QSizePolicy sizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(m_spacerPresetCombo->sizePolicy().hasHeightForWidth());
    m_spacerPresetCombo->setSizePolicy(sizePolicy);
    m_spacerPresetCombo->setMinimumSize(QSize(0, 32));

    m_spacerPropertyFormLayout->setWidget(1, QFormLayout::SpanningRole, m_spacerPresetCombo);

    m_spacerModelLabel = new QLabel(m_spacerPropertyGroup);
    m_spacerModelLabel->setObjectName("spacerModelLabel");

    m_spacerPropertyFormLayout->setWidget(2, QFormLayout::SpanningRole, m_spacerModelLabel);

    m_spacerModelEdit = new QLineEdit(m_spacerPropertyGroup);
    m_spacerModelEdit->setObjectName("spacerModelEdit");
    sizePolicy.setHeightForWidth(m_spacerModelEdit->sizePolicy().hasHeightForWidth());
    m_spacerModelEdit->setSizePolicy(sizePolicy);
    m_spacerModelEdit->setMinimumSize(QSize(0, 32));

    m_spacerPropertyFormLayout->setWidget(3, QFormLayout::SpanningRole, m_spacerModelEdit);

    m_spacerMassLabel = new QLabel(m_spacerPropertyGroup);
    m_spacerMassLabel->setObjectName("spacerMassLabel");

    m_spacerPropertyFormLayout->setWidget(4, QFormLayout::SpanningRole, m_spacerMassLabel);

    m_spacerMassSpin = new CompactDoubleSpinBox(m_spacerPropertyGroup);
    m_spacerMassSpin->setObjectName("spacerMassSpin");
    m_spacerMassSpin->setMinimum(0.001000000000000);
    m_spacerMassSpin->setMaximum(1000000.000000000000000);
    m_spacerMassSpin->setSingleStep(0.100000000000000);
    m_spacerMassSpin->setValue(7.500000000000000);

    m_spacerPropertyFormLayout->setWidget(5, QFormLayout::SpanningRole, m_spacerMassSpin);

    m_spacerSpacingLabel = new QLabel(m_spacerPropertyGroup);
    m_spacerSpacingLabel->setObjectName("spacerSpacingLabel");

    m_spacerPropertyFormLayout->setWidget(6, QFormLayout::SpanningRole, m_spacerSpacingLabel);

    m_spacerSpacingSpin = new ChevronDoubleSpinBox(m_spacerPropertyGroup);
    m_spacerSpacingSpin->setObjectName("spacerSpacingSpin");
    sizePolicy.setHeightForWidth(m_spacerSpacingSpin->sizePolicy().hasHeightForWidth());
    m_spacerSpacingSpin->setSizePolicy(sizePolicy);
    m_spacerSpacingSpin->setMinimumSize(QSize(0, 32));
    m_spacerSpacingSpin->setDecimals(3);
    m_spacerSpacingSpin->setMinimum(0.001000000000000);
    m_spacerSpacingSpin->setMaximum(20.000000000000000);
    m_spacerSpacingSpin->setSingleStep(0.001000000000000);
    m_spacerSpacingSpin->setValue(0.400000000000000);

    m_spacerPropertyFormLayout->setWidget(7, QFormLayout::SpanningRole, m_spacerSpacingSpin);

    m_spacerMaterialLabel = new QLabel(m_spacerPropertyGroup);
    m_spacerMaterialLabel->setObjectName("spacerMaterialLabel");

    m_spacerPropertyFormLayout->setWidget(8, QFormLayout::SpanningRole, m_spacerMaterialLabel);

    m_spacerMaterialCombo = new ChevronComboBox(m_spacerPropertyGroup);
    m_spacerMaterialCombo->setObjectName("spacerMaterialCombo");

    m_spacerPropertyFormLayout->setWidget(9, QFormLayout::SpanningRole, m_spacerMaterialCombo);

    m_spacerContentLayout->addWidget(m_spacerPropertyGroup, 0, 0, 1, 1);

    m_spacerPropertyGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "产品与尺寸", nullptr));
    m_spacerPresetLabel->setText(QCoreApplication::translate("ConductorModuleClass", "质量预设", nullptr));
    m_spacerPresetCombo->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass", "选择预设后填入质量和分裂间距；下方参数仍可手动修改", nullptr));
    m_spacerModelLabel->setText(QCoreApplication::translate("ConductorModuleClass", "产品型号", nullptr));
    m_spacerModelEdit->setPlaceholderText(
        QCoreApplication::translate("ConductorModuleClass", "可填写产品型号", nullptr));
    m_spacerMassLabel->setText(QCoreApplication::translate("ConductorModuleClass", "每套质量", nullptr));
    m_spacerMassSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " kg", nullptr));
    m_spacerSpacingLabel->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass", "分裂间距：横截面上相邻子导线的中心距，与基本参数中的子导线间距使用同一数值", nullptr));
    m_spacerSpacingLabel->setText(QCoreApplication::translate("ConductorModuleClass", "分裂间距", nullptr));
    m_spacerSpacingSpin->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass", "相邻子导线中心距；修改后与基本参数中的子导线间距同步", nullptr));
    m_spacerSpacingSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " m", nullptr));
    m_spacerMaterialLabel->setText(QCoreApplication::translate("ConductorModuleClass", "材料", nullptr));
}

void ConductorModule::initializeSpacerGroup()
{
    m_spacerGroup = new QGroupBox(m_spacerContent);
    m_spacerGroup->setObjectName("spacerGroup");
    m_spacerFormLayout = new QFormLayout(m_spacerGroup);
    m_spacerFormLayout->setObjectName("spacerFormLayout");
    m_spacerFormLayout->setHorizontalSpacing(0);
    m_spacerFormLayout->setVerticalSpacing(6);
    m_spacerFormLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_spacerFormLayout->setRowWrapPolicy(QFormLayout::DontWrapRows);
    m_spacerFormLayout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_spacerFormLayout->setContentsMargins(14, 14, 14, 14);
    QLabel* spacerStyleLabel = new QLabel(m_spacerGroup);
    spacerStyleLabel->setObjectName("spacerStyleLabel");

    m_spacerFormLayout->setWidget(0, QFormLayout::SpanningRole, spacerStyleLabel);

    m_spacerStyleCombo = new ChevronComboBox(m_spacerGroup);
    m_spacerStyleCombo->addItem(QString());
    m_spacerStyleCombo->addItem(QString());
    m_spacerStyleCombo->addItem(QString());
    m_spacerStyleCombo->addItem(QString());
    m_spacerStyleCombo->addItem(QString());
    m_spacerStyleCombo->setObjectName("spacerStyleCombo");

    m_spacerFormLayout->setWidget(1, QFormLayout::SpanningRole, m_spacerStyleCombo);

    m_spacerElementLabel = new QLabel(m_spacerGroup);
    m_spacerElementLabel->setObjectName("spacerElementLabel");

    m_spacerFormLayout->setWidget(2, QFormLayout::SpanningRole, m_spacerElementLabel);

    m_spacerElementCombo = new ChevronComboBox(m_spacerGroup);
    m_spacerElementCombo->addItem(QString());
    m_spacerElementCombo->addItem(QString());
    m_spacerElementCombo->setObjectName("spacerElementCombo");

    m_spacerFormLayout->setWidget(3, QFormLayout::SpanningRole, m_spacerElementCombo);

    QLabel* spacerLayoutLabel = new QLabel(m_spacerGroup);
    spacerLayoutLabel->setObjectName("spacerLayoutLabel");

    m_spacerFormLayout->setWidget(4, QFormLayout::SpanningRole, spacerLayoutLabel);

    m_spacerLayoutCombo = new ChevronComboBox(m_spacerGroup);
    m_spacerLayoutCombo->addItem(QString());
    m_spacerLayoutCombo->addItem(QString());
    m_spacerLayoutCombo->setObjectName("spacerLayoutCombo");

    m_spacerFormLayout->setWidget(5, QFormLayout::SpanningRole, m_spacerLayoutCombo);

    m_spacerCountLabel = new QLabel(m_spacerGroup);
    m_spacerCountLabel->setObjectName("spacerCountLabel");

    m_spacerFormLayout->setWidget(6, QFormLayout::SpanningRole, m_spacerCountLabel);

    m_spacerCountSpin = new ChevronSpinBox(m_spacerGroup);
    m_spacerCountSpin->setObjectName("spacerCountSpin");
    m_spacerCountSpin->setMinimum(1);
    m_spacerCountSpin->setMaximum(1000);
    m_spacerCountSpin->setValue(4);

    m_spacerFormLayout->setWidget(7, QFormLayout::SpanningRole, m_spacerCountSpin);

    m_spacerSectionLabel = new QLabel(m_spacerGroup);
    m_spacerSectionLabel->setObjectName("spacerSectionLabel");

    m_spacerFormLayout->setWidget(8, QFormLayout::SpanningRole, m_spacerSectionLabel);

    m_spacerSectionCombo = new ChevronComboBox(m_spacerGroup);
    m_spacerSectionCombo->setObjectName("spacerSectionCombo");

    m_spacerFormLayout->setWidget(9, QFormLayout::SpanningRole, m_spacerSectionCombo);

    m_spacerContentLayout->addWidget(m_spacerGroup, 0, 1, 1, 1);

    spacerStyleLabel->setText(QCoreApplication::translate("ConductorModuleClass", "建模形式", nullptr));

    spacerLayoutLabel->setText(QCoreApplication::translate("ConductorModuleClass", "布置方式", nullptr));

    m_spacerGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "建模与布置", nullptr));
    m_spacerStyleCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "外框多边形", nullptr));
    m_spacerStyleCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "中心对角撑", nullptr));
    m_spacerStyleCombo->setItemText(2, QCoreApplication::translate("ConductorModuleClass", "内圈多边形", nullptr));
    m_spacerStyleCombo->setItemText(
        3, QCoreApplication::translate("ConductorModuleClass", "刚性中心 MPC（高效）", nullptr));
    m_spacerStyleCombo->setItemText(
        4, QCoreApplication::translate("ConductorModuleClass", "交叉对角撑（四分裂）", nullptr));

    m_spacerElementLabel->setText(QCoreApplication::translate("ConductorModuleClass", "单元类型", nullptr));
    m_spacerElementCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "CR3D 梁", nullptr));
    m_spacerElementCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "T3D2 桁架", nullptr));
    m_spacerLayoutCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "规范档距", nullptr));
    m_spacerLayoutCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "指定数量(等距)", nullptr));

    m_spacerCountLabel->setText(QCoreApplication::translate("ConductorModuleClass", "每档数量", nullptr));
    m_spacerSectionLabel->setText(QCoreApplication::translate("ConductorModuleClass", "截面", nullptr));
}

void ConductorModule::initializeBasicTab()
{
    m_basicTab = new QWidget();
    m_basicTab->setObjectName("basicTab");
    QVBoxLayout* basicTabLayout = new QVBoxLayout(m_basicTab);
    basicTabLayout->setSpacing(0);
    basicTabLayout->setObjectName("basicTabLayout");
    basicTabLayout->setContentsMargins(0, 0, 0, 0);
    m_scrollArea = new QScrollArea(m_basicTab);
    m_scrollArea->setObjectName("scrollArea");
    m_scrollArea->setFrameShape(QFrame::NoFrame);
    m_scrollArea->setWidgetResizable(true);
    m_content = new QWidget();
    m_content->setObjectName("content");
    m_contentLayout = new QGridLayout(m_content);
    m_contentLayout->setObjectName("contentLayout");
    m_contentLayout->setHorizontalSpacing(12);
    m_contentLayout->setVerticalSpacing(8);
    m_contentLayout->setContentsMargins(6, 6, 6, 6);
    initializePropertyGroup();

    initializeGeometryGroup();

    initializeSpanGroup();

    initializeEndConnectionGroup();

    m_contentLayout->setRowStretch(0, 1);
    m_contentLayout->setRowStretch(1, 1);
    m_contentLayout->setColumnStretch(0, 1);
    m_contentLayout->setColumnStretch(1, 1);
    m_scrollArea->setWidget(m_content);

    basicTabLayout->addWidget(m_scrollArea);

    m_creationTabs->addTab(m_basicTab, QString());
}

void ConductorModule::initializeAdvancedTab()
{
    QWidget* advancedTab = new QWidget();
    advancedTab->setObjectName("advancedTab");
    QVBoxLayout* advancedTabLayout = new QVBoxLayout(advancedTab);
    advancedTabLayout->setSpacing(0);
    advancedTabLayout->setObjectName("advancedTabLayout");
    advancedTabLayout->setContentsMargins(0, 0, 0, 0);
    m_advancedScrollArea = new QScrollArea(advancedTab);
    m_advancedScrollArea->setObjectName("advancedScrollArea");
    m_advancedScrollArea->setFrameShape(QFrame::NoFrame);
    m_advancedScrollArea->setWidgetResizable(true);
    m_advancedContent = new QWidget();
    m_advancedContent->setObjectName("advancedContent");
    m_advancedContentLayout = new QGridLayout(m_advancedContent);
    m_advancedContentLayout->setObjectName("advancedContentLayout");
    m_advancedContentLayout->setHorizontalSpacing(12);
    m_advancedContentLayout->setVerticalSpacing(8);
    m_advancedContentLayout->setContentsMargins(6, 6, 6, 6);
    initializeStructuralIceGroup();

    initializeTorsionGroup();

    m_advancedContentLayout->setColumnStretch(0, 1);
    m_advancedContentLayout->setColumnStretch(1, 1);
    m_advancedScrollArea->setWidget(m_advancedContent);

    advancedTabLayout->addWidget(m_advancedScrollArea);

    m_creationTabs->addTab(advancedTab, QString());

    m_creationTabs->setTabText(m_creationTabs->indexOf(advancedTab),
                               QCoreApplication::translate("ConductorModuleClass", "覆冰与扭转", nullptr));
}

void ConductorModule::initializeSpacerTab()
{
    m_spacerTab = new QWidget();
    m_spacerTab->setObjectName("spacerTab");
    m_spacerTabLayout = new QVBoxLayout(m_spacerTab);
    m_spacerTabLayout->setSpacing(0);
    m_spacerTabLayout->setObjectName("spacerTabLayout");
    m_spacerTabLayout->setContentsMargins(0, 0, 0, 0);
    m_spacerCreationLayout = new QHBoxLayout();
    m_spacerCreationLayout->setSpacing(12);
    m_spacerCreationLayout->setObjectName("spacerCreationLayout");
    m_innerSpacerCheck = new QCheckBox(m_spacerTab);
    m_innerSpacerCheck->setObjectName("innerSpacerCheck");
    m_innerSpacerCheck->setChecked(true);

    m_spacerCreationLayout->addWidget(m_innerSpacerCheck);

    m_spacerCountPreview = new QLabel(m_spacerTab);
    m_spacerCountPreview->setObjectName("spacerCountPreview");
    m_spacerCountPreview->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    m_spacerCreationLayout->addWidget(m_spacerCountPreview);

    m_spacerTabLayout->addLayout(m_spacerCreationLayout);

    m_spacerScrollArea = new QScrollArea(m_spacerTab);
    m_spacerScrollArea->setObjectName("spacerScrollArea");
    m_spacerScrollArea->setFrameShape(QFrame::NoFrame);
    m_spacerScrollArea->setWidgetResizable(true);
    m_spacerContent = new QWidget();
    m_spacerContent->setObjectName("spacerContent");
    m_spacerContentLayout = new QGridLayout(m_spacerContent);
    m_spacerContentLayout->setSpacing(8);
    m_spacerContentLayout->setObjectName("spacerContentLayout");
    m_spacerContentLayout->setHorizontalSpacing(12);
    m_spacerContentLayout->setVerticalSpacing(8);
    m_spacerContentLayout->setContentsMargins(6, 6, 6, 6);
    initializeSpacerPropertyGroup();

    initializeSpacerGroup();

    m_spacerContentLayout->setColumnStretch(0, 1);
    m_spacerContentLayout->setColumnStretch(1, 1);
    m_spacerScrollArea->setWidget(m_spacerContent);

    m_spacerTabLayout->addWidget(m_spacerScrollArea);

    m_spacerStatusLabel = new QLabel(m_spacerTab);
    m_spacerStatusLabel->setObjectName("spacerStatusLabel");
    m_spacerStatusLabel->setWordWrap(false);

    m_spacerTabLayout->addWidget(m_spacerStatusLabel);

    m_creationTabs->addTab(m_spacerTab, QString());

    m_innerSpacerCheck->setText(QCoreApplication::translate("ConductorModuleClass", "创建相内间隔棒", nullptr));
    m_innerSpacerCheck->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass", "二分裂生成横杆；三分裂及以上按所选形式生成间隔棒", nullptr));
    m_spacerCountPreview->setText(QCoreApplication::translate("ConductorModuleClass", "按当前档距自动计算", nullptr));
    m_spacerStatusLabel->setText(
        QCoreApplication::translate("ConductorModuleClass", "选择预设自动填入参数；质量与间距可直接修改。", nullptr));
    m_spacerStatusLabel->setToolTip(QCoreApplication::translate(
        "ConductorModuleClass", "产品预设填入质量、间距、型号和参考材料；手动修改单项参数时保留其他设置。", nullptr));
}
