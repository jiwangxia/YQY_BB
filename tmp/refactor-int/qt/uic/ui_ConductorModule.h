/********************************************************************************
** Form generated from reading UI file 'ConductorModule.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONDUCTORMODULE_H
#define UI_CONDUCTORMODULE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ConductorModuleClass
{
public:
    QVBoxLayout *rootLayout;
    QScrollArea *scrollArea;
    QWidget *content;
    QVBoxLayout *contentLayout;
    QGroupBox *propertyGroup;
    QVBoxLayout *propertyLayout;
    QLabel *propertySummary;
    QPushButton *viewLibraryButton;
    QGroupBox *spanDefinitionGroup;
    QVBoxLayout *spanDefinitionLayout;
    QComboBox *modelModeCombo;
    QWidget *multiSpanEditor;
    QVBoxLayout *multiSpanEditorLayout;
    QLabel *stationHelpLabel;
    QTableWidget *stationTable;
    QHBoxLayout *stationButtonLayout;
    QPushButton *addStationButton;
    QPushButton *removeStationButton;
    QFormLayout *suspensionFormLayout;
    QLabel *suspensionLengthLabel;
    QDoubleSpinBox *suspensionLengthSpin;
    QGroupBox *formGroup;
    QFormLayout *formLayout;
    QLabel *nameLabel;
    QLineEdit *nameEdit;
    QLabel *materialLabel;
    QComboBox *materialCombo;
    QLabel *sectionLabel;
    QComboBox *sectionCombo;
    QLabel *elementLabel;
    QComboBox *elementCombo;
    QLabel *startLabel;
    QWidget *startCoordinates;
    QHBoxLayout *startLayout;
    QDoubleSpinBox *startX;
    QDoubleSpinBox *startY;
    QDoubleSpinBox *startZ;
    QLabel *endLabel;
    QWidget *endCoordinates;
    QHBoxLayout *endLayout;
    QDoubleSpinBox *endX;
    QDoubleSpinBox *endY;
    QDoubleSpinBox *endZ;
    QLabel *bundleLabel;
    QComboBox *bundleCombo;
    QLabel *spacingLabel;
    QDoubleSpinBox *spacingSpin;
    QLabel *segmentsLabel;
    QSpinBox *segmentsSpin;
    QLabel *stressLabel;
    QDoubleSpinBox *stressSpin;
    QLabel *endTopologyLabel;
    QComboBox *endTopologyCombo;
    QLabel *dualSupportSpacingLabel;
    QDoubleSpinBox *dualSupportSpacingSpin;
    QGroupBox *spacerGroup;
    QFormLayout *spacerFormLayout;
    QCheckBox *innerSpacerCheck;
    QLabel *spacerLayoutLabel;
    QComboBox *spacerLayoutCombo;
    QLabel *spacerCountLabel;
    QSpinBox *spacerCountSpin;
    QLabel *spacerPreviewTitle;
    QLabel *spacerCountPreview;
    QLabel *spacerElementLabel;
    QComboBox *spacerElementCombo;
    QLabel *spacerMaterialLabel;
    QComboBox *spacerMaterialCombo;
    QLabel *spacerSectionLabel;
    QComboBox *spacerSectionCombo;
    QLabel *spacerStyleLabel;
    QComboBox *spacerStyleCombo;
    QCheckBox *analysisCheck;
    QPushButton *createButton;
    QSpacerItem *contentSpacer;

    void setupUi(QWidget *ConductorModuleClass)
    {
        if (ConductorModuleClass->objectName().isEmpty())
            ConductorModuleClass->setObjectName("ConductorModuleClass");
        rootLayout = new QVBoxLayout(ConductorModuleClass);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(0, 0, 0, 0);
        scrollArea = new QScrollArea(ConductorModuleClass);
        scrollArea->setObjectName("scrollArea");
        scrollArea->setFrameShape(QFrame::NoFrame);
        scrollArea->setWidgetResizable(true);
        scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        content = new QWidget();
        content->setObjectName("content");
        contentLayout = new QVBoxLayout(content);
        contentLayout->setSpacing(10);
        contentLayout->setObjectName("contentLayout");
        contentLayout->setContentsMargins(8, 8, 8, 8);
        propertyGroup = new QGroupBox(content);
        propertyGroup->setObjectName("propertyGroup");
        propertyLayout = new QVBoxLayout(propertyGroup);
        propertyLayout->setObjectName("propertyLayout");
        propertySummary = new QLabel(propertyGroup);
        propertySummary->setObjectName("propertySummary");
        propertySummary->setWordWrap(true);

        propertyLayout->addWidget(propertySummary);

        viewLibraryButton = new QPushButton(propertyGroup);
        viewLibraryButton->setObjectName("viewLibraryButton");

        propertyLayout->addWidget(viewLibraryButton);


        contentLayout->addWidget(propertyGroup);

        spanDefinitionGroup = new QGroupBox(content);
        spanDefinitionGroup->setObjectName("spanDefinitionGroup");
        spanDefinitionLayout = new QVBoxLayout(spanDefinitionGroup);
        spanDefinitionLayout->setObjectName("spanDefinitionLayout");
        modelModeCombo = new QComboBox(spanDefinitionGroup);
        modelModeCombo->addItem(QString());
        modelModeCombo->addItem(QString());
        modelModeCombo->setObjectName("modelModeCombo");

        spanDefinitionLayout->addWidget(modelModeCombo);

        multiSpanEditor = new QWidget(spanDefinitionGroup);
        multiSpanEditor->setObjectName("multiSpanEditor");
        multiSpanEditorLayout = new QVBoxLayout(multiSpanEditor);
        multiSpanEditorLayout->setObjectName("multiSpanEditorLayout");
        multiSpanEditorLayout->setContentsMargins(0, 0, 0, 0);
        stationHelpLabel = new QLabel(multiSpanEditor);
        stationHelpLabel->setObjectName("stationHelpLabel");
        stationHelpLabel->setWordWrap(true);

        multiSpanEditorLayout->addWidget(stationHelpLabel);

        stationTable = new QTableWidget(multiSpanEditor);
        if (stationTable->columnCount() < 5)
            stationTable->setColumnCount(5);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        stationTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        stationTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        stationTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        stationTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        stationTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        stationTable->setObjectName("stationTable");
        stationTable->setMinimumSize(QSize(0, 170));
        stationTable->setAlternatingRowColors(true);
        stationTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        stationTable->setColumnCount(5);

        multiSpanEditorLayout->addWidget(stationTable);

        stationButtonLayout = new QHBoxLayout();
        stationButtonLayout->setObjectName("stationButtonLayout");
        addStationButton = new QPushButton(multiSpanEditor);
        addStationButton->setObjectName("addStationButton");

        stationButtonLayout->addWidget(addStationButton);

        removeStationButton = new QPushButton(multiSpanEditor);
        removeStationButton->setObjectName("removeStationButton");

        stationButtonLayout->addWidget(removeStationButton);


        multiSpanEditorLayout->addLayout(stationButtonLayout);

        suspensionFormLayout = new QFormLayout();
        suspensionFormLayout->setObjectName("suspensionFormLayout");
        suspensionLengthLabel = new QLabel(multiSpanEditor);
        suspensionLengthLabel->setObjectName("suspensionLengthLabel");

        suspensionFormLayout->setWidget(0, QFormLayout::LabelRole, suspensionLengthLabel);

        suspensionLengthSpin = new QDoubleSpinBox(multiSpanEditor);
        suspensionLengthSpin->setObjectName("suspensionLengthSpin");
        suspensionLengthSpin->setMinimum(0.001000000000000);
        suspensionLengthSpin->setMaximum(1000.000000000000000);
        suspensionLengthSpin->setDecimals(3);
        suspensionLengthSpin->setValue(1.000000000000000);

        suspensionFormLayout->setWidget(0, QFormLayout::FieldRole, suspensionLengthSpin);


        multiSpanEditorLayout->addLayout(suspensionFormLayout);


        spanDefinitionLayout->addWidget(multiSpanEditor);


        contentLayout->addWidget(spanDefinitionGroup);

        formGroup = new QGroupBox(content);
        formGroup->setObjectName("formGroup");
        formLayout = new QFormLayout(formGroup);
        formLayout->setObjectName("formLayout");
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        formLayout->setRowWrapPolicy(QFormLayout::WrapLongRows);
        nameLabel = new QLabel(formGroup);
        nameLabel->setObjectName("nameLabel");

        formLayout->setWidget(0, QFormLayout::LabelRole, nameLabel);

        nameEdit = new QLineEdit(formGroup);
        nameEdit->setObjectName("nameEdit");

        formLayout->setWidget(0, QFormLayout::FieldRole, nameEdit);

        materialLabel = new QLabel(formGroup);
        materialLabel->setObjectName("materialLabel");

        formLayout->setWidget(1, QFormLayout::LabelRole, materialLabel);

        materialCombo = new QComboBox(formGroup);
        materialCombo->setObjectName("materialCombo");

        formLayout->setWidget(1, QFormLayout::FieldRole, materialCombo);

        sectionLabel = new QLabel(formGroup);
        sectionLabel->setObjectName("sectionLabel");

        formLayout->setWidget(2, QFormLayout::LabelRole, sectionLabel);

        sectionCombo = new QComboBox(formGroup);
        sectionCombo->setObjectName("sectionCombo");

        formLayout->setWidget(2, QFormLayout::FieldRole, sectionCombo);

        elementLabel = new QLabel(formGroup);
        elementLabel->setObjectName("elementLabel");

        formLayout->setWidget(3, QFormLayout::LabelRole, elementLabel);

        elementCombo = new QComboBox(formGroup);
        elementCombo->addItem(QString());
        elementCombo->addItem(QString());
        elementCombo->setObjectName("elementCombo");

        formLayout->setWidget(3, QFormLayout::FieldRole, elementCombo);

        startLabel = new QLabel(formGroup);
        startLabel->setObjectName("startLabel");

        formLayout->setWidget(4, QFormLayout::LabelRole, startLabel);

        startCoordinates = new QWidget(formGroup);
        startCoordinates->setObjectName("startCoordinates");
        startLayout = new QHBoxLayout(startCoordinates);
        startLayout->setSpacing(4);
        startLayout->setObjectName("startLayout");
        startLayout->setContentsMargins(0, 0, 0, 0);
        startX = new QDoubleSpinBox(startCoordinates);
        startX->setObjectName("startX");
        startX->setMinimum(-1000000.000000000000000);
        startX->setMaximum(1000000.000000000000000);
        startX->setDecimals(3);

        startLayout->addWidget(startX);

        startY = new QDoubleSpinBox(startCoordinates);
        startY->setObjectName("startY");
        startY->setMinimum(-1000000.000000000000000);
        startY->setMaximum(1000000.000000000000000);
        startY->setDecimals(3);

        startLayout->addWidget(startY);

        startZ = new QDoubleSpinBox(startCoordinates);
        startZ->setObjectName("startZ");
        startZ->setMinimum(-1000000.000000000000000);
        startZ->setMaximum(1000000.000000000000000);
        startZ->setDecimals(3);

        startLayout->addWidget(startZ);


        formLayout->setWidget(4, QFormLayout::FieldRole, startCoordinates);

        endLabel = new QLabel(formGroup);
        endLabel->setObjectName("endLabel");

        formLayout->setWidget(5, QFormLayout::LabelRole, endLabel);

        endCoordinates = new QWidget(formGroup);
        endCoordinates->setObjectName("endCoordinates");
        endLayout = new QHBoxLayout(endCoordinates);
        endLayout->setSpacing(1);
        endLayout->setObjectName("endLayout");
        endLayout->setContentsMargins(0, 0, 0, 0);
        endX = new QDoubleSpinBox(endCoordinates);
        endX->setObjectName("endX");
        endX->setMinimum(-1000000.000000000000000);
        endX->setMaximum(1000000.000000000000000);
        endX->setDecimals(3);
        endX->setValue(300.000000000000000);

        endLayout->addWidget(endX);

        endY = new QDoubleSpinBox(endCoordinates);
        endY->setObjectName("endY");
        endY->setMinimum(-1000000.000000000000000);
        endY->setMaximum(1000000.000000000000000);
        endY->setDecimals(3);

        endLayout->addWidget(endY);

        endZ = new QDoubleSpinBox(endCoordinates);
        endZ->setObjectName("endZ");
        endZ->setMinimum(-1000000.000000000000000);
        endZ->setMaximum(1000000.000000000000000);
        endZ->setDecimals(3);

        endLayout->addWidget(endZ);


        formLayout->setWidget(5, QFormLayout::FieldRole, endCoordinates);

        bundleLabel = new QLabel(formGroup);
        bundleLabel->setObjectName("bundleLabel");

        formLayout->setWidget(6, QFormLayout::LabelRole, bundleLabel);

        bundleCombo = new QComboBox(formGroup);
        bundleCombo->addItem(QString());
        bundleCombo->addItem(QString());
        bundleCombo->addItem(QString());
        bundleCombo->addItem(QString());
        bundleCombo->addItem(QString());
        bundleCombo->setObjectName("bundleCombo");

        formLayout->setWidget(6, QFormLayout::FieldRole, bundleCombo);

        spacingLabel = new QLabel(formGroup);
        spacingLabel->setObjectName("spacingLabel");

        formLayout->setWidget(7, QFormLayout::LabelRole, spacingLabel);

        spacingSpin = new QDoubleSpinBox(formGroup);
        spacingSpin->setObjectName("spacingSpin");
        spacingSpin->setMaximum(20.000000000000000);
        spacingSpin->setDecimals(3);
        spacingSpin->setValue(0.400000000000000);

        formLayout->setWidget(7, QFormLayout::FieldRole, spacingSpin);

        segmentsLabel = new QLabel(formGroup);
        segmentsLabel->setObjectName("segmentsLabel");

        formLayout->setWidget(8, QFormLayout::LabelRole, segmentsLabel);

        segmentsSpin = new QSpinBox(formGroup);
        segmentsSpin->setObjectName("segmentsSpin");
        segmentsSpin->setMinimum(2);
        segmentsSpin->setMaximum(10000);
        segmentsSpin->setValue(50);

        formLayout->setWidget(8, QFormLayout::FieldRole, segmentsSpin);

        stressLabel = new QLabel(formGroup);
        stressLabel->setObjectName("stressLabel");

        formLayout->setWidget(9, QFormLayout::LabelRole, stressLabel);

        stressSpin = new QDoubleSpinBox(formGroup);
        stressSpin->setObjectName("stressSpin");
        stressSpin->setMinimum(0.001000000000000);
        stressSpin->setMaximum(10000.000000000000000);
        stressSpin->setDecimals(3);
        stressSpin->setValue(50.000000000000000);

        formLayout->setWidget(9, QFormLayout::FieldRole, stressSpin);

        endTopologyLabel = new QLabel(formGroup);
        endTopologyLabel->setObjectName("endTopologyLabel");

        formLayout->setWidget(10, QFormLayout::LabelRole, endTopologyLabel);

        endTopologyCombo = new QComboBox(formGroup);
        endTopologyCombo->addItem(QString());
        endTopologyCombo->addItem(QString());
        endTopologyCombo->addItem(QString());
        endTopologyCombo->setObjectName("endTopologyCombo");

        formLayout->setWidget(10, QFormLayout::FieldRole, endTopologyCombo);

        dualSupportSpacingLabel = new QLabel(formGroup);
        dualSupportSpacingLabel->setObjectName("dualSupportSpacingLabel");

        formLayout->setWidget(11, QFormLayout::LabelRole, dualSupportSpacingLabel);

        dualSupportSpacingSpin = new QDoubleSpinBox(formGroup);
        dualSupportSpacingSpin->setObjectName("dualSupportSpacingSpin");
        dualSupportSpacingSpin->setMinimum(0.001000000000000);
        dualSupportSpacingSpin->setMaximum(100.000000000000000);
        dualSupportSpacingSpin->setDecimals(3);
        dualSupportSpacingSpin->setValue(0.450000000000000);

        formLayout->setWidget(11, QFormLayout::FieldRole, dualSupportSpacingSpin);


        contentLayout->addWidget(formGroup);

        spacerGroup = new QGroupBox(content);
        spacerGroup->setObjectName("spacerGroup");
        spacerFormLayout = new QFormLayout(spacerGroup);
        spacerFormLayout->setObjectName("spacerFormLayout");
        spacerFormLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        spacerFormLayout->setRowWrapPolicy(QFormLayout::WrapLongRows);
        innerSpacerCheck = new QCheckBox(spacerGroup);
        innerSpacerCheck->setObjectName("innerSpacerCheck");
        innerSpacerCheck->setChecked(true);

        spacerFormLayout->setWidget(0, QFormLayout::SpanningRole, innerSpacerCheck);

        spacerLayoutLabel = new QLabel(spacerGroup);
        spacerLayoutLabel->setObjectName("spacerLayoutLabel");

        spacerFormLayout->setWidget(2, QFormLayout::LabelRole, spacerLayoutLabel);

        spacerLayoutCombo = new QComboBox(spacerGroup);
        spacerLayoutCombo->addItem(QString());
        spacerLayoutCombo->addItem(QString());
        spacerLayoutCombo->setObjectName("spacerLayoutCombo");

        spacerFormLayout->setWidget(2, QFormLayout::FieldRole, spacerLayoutCombo);

        spacerCountLabel = new QLabel(spacerGroup);
        spacerCountLabel->setObjectName("spacerCountLabel");

        spacerFormLayout->setWidget(3, QFormLayout::LabelRole, spacerCountLabel);

        spacerCountSpin = new QSpinBox(spacerGroup);
        spacerCountSpin->setObjectName("spacerCountSpin");
        spacerCountSpin->setMinimum(1);
        spacerCountSpin->setMaximum(1000);
        spacerCountSpin->setValue(4);

        spacerFormLayout->setWidget(3, QFormLayout::FieldRole, spacerCountSpin);

        spacerPreviewTitle = new QLabel(spacerGroup);
        spacerPreviewTitle->setObjectName("spacerPreviewTitle");

        spacerFormLayout->setWidget(4, QFormLayout::LabelRole, spacerPreviewTitle);

        spacerCountPreview = new QLabel(spacerGroup);
        spacerCountPreview->setObjectName("spacerCountPreview");
        spacerCountPreview->setWordWrap(true);

        spacerFormLayout->setWidget(4, QFormLayout::FieldRole, spacerCountPreview);

        spacerElementLabel = new QLabel(spacerGroup);
        spacerElementLabel->setObjectName("spacerElementLabel");

        spacerFormLayout->setWidget(5, QFormLayout::LabelRole, spacerElementLabel);

        spacerElementCombo = new QComboBox(spacerGroup);
        spacerElementCombo->addItem(QString());
        spacerElementCombo->addItem(QString());
        spacerElementCombo->setObjectName("spacerElementCombo");

        spacerFormLayout->setWidget(5, QFormLayout::FieldRole, spacerElementCombo);

        spacerMaterialLabel = new QLabel(spacerGroup);
        spacerMaterialLabel->setObjectName("spacerMaterialLabel");

        spacerFormLayout->setWidget(6, QFormLayout::LabelRole, spacerMaterialLabel);

        spacerMaterialCombo = new QComboBox(spacerGroup);
        spacerMaterialCombo->setObjectName("spacerMaterialCombo");

        spacerFormLayout->setWidget(6, QFormLayout::FieldRole, spacerMaterialCombo);

        spacerSectionLabel = new QLabel(spacerGroup);
        spacerSectionLabel->setObjectName("spacerSectionLabel");

        spacerFormLayout->setWidget(7, QFormLayout::LabelRole, spacerSectionLabel);

        spacerSectionCombo = new QComboBox(spacerGroup);
        spacerSectionCombo->setObjectName("spacerSectionCombo");

        spacerFormLayout->setWidget(7, QFormLayout::FieldRole, spacerSectionCombo);

        spacerStyleLabel = new QLabel(spacerGroup);
        spacerStyleLabel->setObjectName("spacerStyleLabel");

        spacerFormLayout->setWidget(1, QFormLayout::LabelRole, spacerStyleLabel);

        spacerStyleCombo = new QComboBox(spacerGroup);
        spacerStyleCombo->addItem(QString());
        spacerStyleCombo->addItem(QString());
        spacerStyleCombo->addItem(QString());
        spacerStyleCombo->setObjectName("spacerStyleCombo");

        spacerFormLayout->setWidget(1, QFormLayout::FieldRole, spacerStyleCombo);


        contentLayout->addWidget(spacerGroup);

        analysisCheck = new QCheckBox(content);
        analysisCheck->setObjectName("analysisCheck");
        analysisCheck->setChecked(true);

        contentLayout->addWidget(analysisCheck);

        createButton = new QPushButton(content);
        createButton->setObjectName("createButton");
        createButton->setMinimumSize(QSize(0, 40));

        contentLayout->addWidget(createButton);

        contentSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        contentLayout->addItem(contentSpacer);

        scrollArea->setWidget(content);

        rootLayout->addWidget(scrollArea);


        retranslateUi(ConductorModuleClass);

        bundleCombo->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(ConductorModuleClass);
    } // setupUi

    void retranslateUi(QWidget *ConductorModuleClass)
    {
        propertyGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "\346\235\220\346\226\231\344\270\216\346\210\252\351\235\242", nullptr));
        propertySummary->setText(QCoreApplication::translate("ConductorModuleClass", "\346\235\220\346\226\231\345\222\214\346\210\252\351\235\242\345\234\250\347\250\213\345\272\217\345\220\257\345\212\250\346\227\266\345\212\240\350\275\275\357\274\233\345\217\257\345\234\250\345\275\223\345\211\215\344\274\232\350\257\235\344\270\255\344\277\256\346\224\271\357\274\214\345\273\272\346\250\241\346\227\266\345\244\215\345\210\266\344\270\272\346\250\241\345\236\213\347\213\254\347\253\213\345\261\236\346\200\247\343\200\202", nullptr));
        viewLibraryButton->setText(QCoreApplication::translate("ConductorModuleClass", "\346\237\245\347\234\213\346\211\200\346\234\211\345\267\262\345\212\240\350\275\275\346\250\241\345\236\213\345\261\236\346\200\247", nullptr));
        spanDefinitionGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "\346\241\243\344\275\215\344\270\216\345\235\220\346\240\207", nullptr));
        modelModeCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "\345\215\225\346\241\243\345\257\274\347\272\277", nullptr));
        modelModeCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "\345\244\232\346\241\243\345\257\274\347\272\277", nullptr));

        stationHelpLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\235\220\346\240\207\344\270\272\345\220\204\345\241\224\344\275\215\345\244\204\347\232\204\345\257\274\347\272\277\346\235\237/\345\210\206\350\243\202\345\244\232\350\276\271\345\275\242\344\270\255\345\277\203", nullptr));
        QTableWidgetItem *___qtablewidgetitem = stationTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("ConductorModuleClass", "\345\272\217\345\217\267", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = stationTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("ConductorModuleClass", "\347\261\273\345\236\213", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = stationTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("ConductorModuleClass", "X (m)", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = stationTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("ConductorModuleClass", "Y (m)", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = stationTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("ConductorModuleClass", "Z (m)", nullptr));
        addStationButton->setText(QCoreApplication::translate("ConductorModuleClass", "\345\242\236\345\212\240\344\270\255\351\227\264\346\202\254\345\236\202\347\202\271", nullptr));
        removeStationButton->setText(QCoreApplication::translate("ConductorModuleClass", "\345\210\240\351\231\244\346\211\200\351\200\211\347\202\271", nullptr));
        suspensionLengthLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\346\202\254\345\236\202\344\270\262\351\225\277\345\272\246", nullptr));
        suspensionLengthSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " m", nullptr));
        formGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "\345\257\274\347\272\277\345\217\202\346\225\260", nullptr));
        nameLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\346\250\241\345\236\213\345\220\215\347\247\260", nullptr));
        nameEdit->setText(QCoreApplication::translate("ConductorModuleClass", "\345\257\274\347\272\277\346\250\241\345\236\213", nullptr));
        materialLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\346\235\220\346\226\231", nullptr));
        sectionLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\346\210\252\351\235\242", nullptr));
        elementLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\215\225\345\205\203\347\261\273\345\236\213", nullptr));
        elementCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "T3D2 \346\241\201\346\236\266", nullptr));
        elementCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "CABLE \347\264\242", nullptr));

        startLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\267\246\346\214\202\347\202\271 (m)", nullptr));
        startX->setPrefix(QCoreApplication::translate("ConductorModuleClass", "X ", nullptr));
        startY->setPrefix(QCoreApplication::translate("ConductorModuleClass", "Y ", nullptr));
        startZ->setPrefix(QCoreApplication::translate("ConductorModuleClass", "Z ", nullptr));
        endLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\217\263\346\214\202\347\202\271 (m)", nullptr));
        endX->setPrefix(QCoreApplication::translate("ConductorModuleClass", "X ", nullptr));
        endY->setPrefix(QCoreApplication::translate("ConductorModuleClass", "Y ", nullptr));
        endZ->setPrefix(QCoreApplication::translate("ConductorModuleClass", "Z ", nullptr));
        bundleLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\210\206\350\243\202\346\225\260", nullptr));
        bundleCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "1", nullptr));
        bundleCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "2", nullptr));
        bundleCombo->setItemText(2, QCoreApplication::translate("ConductorModuleClass", "4", nullptr));
        bundleCombo->setItemText(3, QCoreApplication::translate("ConductorModuleClass", "6", nullptr));
        bundleCombo->setItemText(4, QCoreApplication::translate("ConductorModuleClass", "8", nullptr));

        spacingLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\255\220\345\257\274\347\272\277\351\227\264\350\267\235", nullptr));
        spacingSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " m", nullptr));
        segmentsLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\346\257\217\346\240\271\345\257\274\347\272\277\346\256\265\346\225\260", nullptr));
        stressLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\210\235\345\247\213\345\272\224\345\212\233", nullptr));
        stressSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " MPa", nullptr));
        endTopologyLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\347\253\257\351\203\250\345\275\242\345\274\217", nullptr));
        endTopologyCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "\346\261\207\351\233\206\345\215\225\346\214\202\347\202\271", nullptr));
        endTopologyCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "\345\210\206\347\273\204\345\217\214\346\214\202\347\202\271", nullptr));
        endTopologyCombo->setItemText(2, QCoreApplication::translate("ConductorModuleClass", "\345\255\220\345\257\274\347\272\277\347\213\254\347\253\213\346\214\202\347\202\271", nullptr));

        dualSupportSpacingLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\217\214\346\214\202\347\202\271\351\227\264\350\267\235", nullptr));
        dualSupportSpacingSpin->setSuffix(QCoreApplication::translate("ConductorModuleClass", " m", nullptr));
        spacerGroup->setTitle(QCoreApplication::translate("ConductorModuleClass", "\347\233\270\345\206\205\351\227\264\351\232\224\346\243\222", nullptr));
        innerSpacerCheck->setText(QCoreApplication::translate("ConductorModuleClass", "\345\210\233\345\273\272\347\233\270\345\206\205\351\227\264\351\232\224\346\243\222", nullptr));
#if QT_CONFIG(tooltip)
        innerSpacerCheck->setToolTip(QCoreApplication::translate("ConductorModuleClass", "\344\272\214\345\210\206\350\243\202\347\224\237\346\210\220\346\250\252\346\235\206\357\274\233\344\270\211\345\210\206\350\243\202\345\217\212\344\273\245\344\270\212\346\214\211\346\211\200\351\200\211\345\275\242\345\274\217\347\224\237\346\210\220\351\227\264\351\232\224\346\243\222", nullptr));
#endif // QT_CONFIG(tooltip)
        spacerLayoutLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\270\203\347\275\256\346\226\271\345\274\217", nullptr));
        spacerLayoutCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "\350\247\204\350\214\203\346\241\243\350\267\235", nullptr));
        spacerLayoutCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "\346\214\207\345\256\232\346\225\260\351\207\217(\347\255\211\350\267\235)", nullptr));

        spacerCountLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\351\227\264\351\232\224\346\243\222\346\225\260\351\207\217", nullptr));
        spacerPreviewTitle->setText(QCoreApplication::translate("ConductorModuleClass", "\351\242\204\350\256\241\347\224\237\346\210\220", nullptr));
        spacerCountPreview->setText(QCoreApplication::translate("ConductorModuleClass", "\346\214\211\345\275\223\345\211\215\346\241\243\350\267\235\350\207\252\345\212\250\350\256\241\347\256\227", nullptr));
        spacerElementLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\345\215\225\345\205\203\347\261\273\345\236\213", nullptr));
        spacerElementCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "CR3D \346\242\201", nullptr));
        spacerElementCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "T3D2 \346\241\201\346\236\266", nullptr));

        spacerMaterialLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\346\235\220\346\226\231", nullptr));
        spacerSectionLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\346\210\252\351\235\242", nullptr));
        spacerStyleLabel->setText(QCoreApplication::translate("ConductorModuleClass", "\347\233\270\345\206\205\351\227\264\351\232\224\346\243\222\345\275\242\345\274\217", nullptr));
        spacerStyleCombo->setItemText(0, QCoreApplication::translate("ConductorModuleClass", "\345\244\226\346\241\206\345\244\232\350\276\271\345\275\242", nullptr));
        spacerStyleCombo->setItemText(1, QCoreApplication::translate("ConductorModuleClass", "\344\270\255\345\277\203\345\257\271\350\247\222\346\222\221", nullptr));
        spacerStyleCombo->setItemText(2, QCoreApplication::translate("ConductorModuleClass", "\345\206\205\345\234\210\345\244\232\350\276\271\345\275\242", nullptr));

        analysisCheck->setText(QCoreApplication::translate("ConductorModuleClass", "\345\220\214\346\227\266\345\210\233\345\273\272\351\235\231\345\212\233\345\210\206\346\236\220\346\255\245\344\270\216\347\272\246\346\235\237", nullptr));
#if QT_CONFIG(tooltip)
        analysisCheck->setToolTip(QCoreApplication::translate("ConductorModuleClass", "\345\210\233\345\273\272\351\235\231\345\212\233\345\210\206\346\236\220\346\255\245\343\200\201\351\207\215\345\212\233\345\222\214\344\270\244\347\253\257 XYZ \347\272\246\346\235\237", nullptr));
#endif // QT_CONFIG(tooltip)
        createButton->setText(QCoreApplication::translate("ConductorModuleClass", "\345\210\233\345\273\272\345\271\266\345\257\274\345\205\245\345\257\274\347\272\277\346\250\241\345\236\213", nullptr));
        (void)ConductorModuleClass;
    } // retranslateUi

};

namespace Ui {
    class ConductorModuleClass: public Ui_ConductorModuleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONDUCTORMODULE_H
