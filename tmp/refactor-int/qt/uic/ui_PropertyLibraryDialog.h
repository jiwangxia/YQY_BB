/********************************************************************************
** Form generated from reading UI file 'PropertyLibraryDialog.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROPERTYLIBRARYDIALOG_H
#define UI_PROPERTYLIBRARYDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAbstractButton>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_PropertyLibraryDialogClass
{
public:
    QVBoxLayout *rootLayout;
    QTabWidget *tabs;
    QTableWidget *materialTable;
    QTableWidget *sectionTable;
    QGroupBox *syncGroup;
    QFormLayout *syncLayout;
    QCheckBox *syncCheck;
    QLabel *materialLabel;
    QComboBox *targetMaterialCombo;
    QLabel *sectionLabel;
    QComboBox *targetSectionCombo;
    QLabel *noteLabel;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *PropertyLibraryDialogClass)
    {
        if (PropertyLibraryDialogClass->objectName().isEmpty())
            PropertyLibraryDialogClass->setObjectName("PropertyLibraryDialogClass");
        rootLayout = new QVBoxLayout(PropertyLibraryDialogClass);
        rootLayout->setObjectName("rootLayout");
        tabs = new QTabWidget(PropertyLibraryDialogClass);
        tabs->setObjectName("tabs");
        materialTable = new QTableWidget();
        if (materialTable->columnCount() < 8)
            materialTable->setColumnCount(8);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        materialTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        materialTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        materialTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        materialTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        materialTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        materialTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        materialTable->setHorizontalHeaderItem(6, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        materialTable->setHorizontalHeaderItem(7, __qtablewidgetitem7);
        materialTable->setObjectName("materialTable");
        materialTable->setColumnCount(8);
        tabs->addTab(materialTable, QString());
        sectionTable = new QTableWidget();
        if (sectionTable->columnCount() < 6)
            sectionTable->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        sectionTable->setHorizontalHeaderItem(0, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        sectionTable->setHorizontalHeaderItem(1, __qtablewidgetitem9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        sectionTable->setHorizontalHeaderItem(2, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        sectionTable->setHorizontalHeaderItem(3, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        sectionTable->setHorizontalHeaderItem(4, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        sectionTable->setHorizontalHeaderItem(5, __qtablewidgetitem13);
        sectionTable->setObjectName("sectionTable");
        sectionTable->setColumnCount(6);
        tabs->addTab(sectionTable, QString());

        rootLayout->addWidget(tabs);

        syncGroup = new QGroupBox(PropertyLibraryDialogClass);
        syncGroup->setObjectName("syncGroup");
        syncLayout = new QFormLayout(syncGroup);
        syncLayout->setObjectName("syncLayout");
        syncCheck = new QCheckBox(syncGroup);
        syncCheck->setObjectName("syncCheck");

        syncLayout->setWidget(0, QFormLayout::SpanningRole, syncCheck);

        materialLabel = new QLabel(syncGroup);
        materialLabel->setObjectName("materialLabel");

        syncLayout->setWidget(1, QFormLayout::LabelRole, materialLabel);

        targetMaterialCombo = new QComboBox(syncGroup);
        targetMaterialCombo->setObjectName("targetMaterialCombo");

        syncLayout->setWidget(1, QFormLayout::FieldRole, targetMaterialCombo);

        sectionLabel = new QLabel(syncGroup);
        sectionLabel->setObjectName("sectionLabel");

        syncLayout->setWidget(2, QFormLayout::LabelRole, sectionLabel);

        targetSectionCombo = new QComboBox(syncGroup);
        targetSectionCombo->setObjectName("targetSectionCombo");

        syncLayout->setWidget(2, QFormLayout::FieldRole, targetSectionCombo);


        rootLayout->addWidget(syncGroup);

        noteLabel = new QLabel(PropertyLibraryDialogClass);
        noteLabel->setObjectName("noteLabel");
        noteLabel->setWordWrap(true);

        rootLayout->addWidget(noteLabel);

        buttonBox = new QDialogButtonBox(PropertyLibraryDialogClass);
        buttonBox->setObjectName("buttonBox");
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);

        rootLayout->addWidget(buttonBox);


        retranslateUi(PropertyLibraryDialogClass);

        QMetaObject::connectSlotsByName(PropertyLibraryDialogClass);
    } // setupUi

    void retranslateUi(QDialog *PropertyLibraryDialogClass)
    {
        PropertyLibraryDialogClass->setWindowTitle(QCoreApplication::translate("PropertyLibraryDialogClass", "\346\235\220\346\226\231\344\270\216\346\210\252\351\235\242\345\261\236\346\200\247\345\272\223\357\274\210\345\275\223\345\211\215\344\274\232\350\257\235\357\274\211", nullptr));
        QTableWidgetItem *___qtablewidgetitem = materialTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = materialTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\347\261\273\345\210\253", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = materialTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\345\220\215\347\247\260", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = materialTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "E (Pa)", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = materialTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\346\263\212\346\235\276\346\257\224", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = materialTable->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\345\257\206\345\272\246 (kg/m\302\263)", nullptr));
        QTableWidgetItem *___qtablewidgetitem6 = materialTable->horizontalHeaderItem(6);
        ___qtablewidgetitem6->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\350\256\270\347\224\250\345\272\224\345\212\233 (Pa)", nullptr));
        QTableWidgetItem *___qtablewidgetitem7 = materialTable->horizontalHeaderItem(7);
        ___qtablewidgetitem7->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\350\206\250\350\203\200\347\263\273\346\225\260", nullptr));
        tabs->setTabText(tabs->indexOf(materialTable), QCoreApplication::translate("PropertyLibraryDialogClass", "\346\235\220\346\226\231", nullptr));
        QTableWidgetItem *___qtablewidgetitem8 = sectionTable->horizontalHeaderItem(0);
        ___qtablewidgetitem8->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem9 = sectionTable->horizontalHeaderItem(1);
        ___qtablewidgetitem9->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\347\261\273\345\210\253", nullptr));
        QTableWidgetItem *___qtablewidgetitem10 = sectionTable->horizontalHeaderItem(2);
        ___qtablewidgetitem10->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\345\220\215\347\247\260", nullptr));
        QTableWidgetItem *___qtablewidgetitem11 = sectionTable->horizontalHeaderItem(3);
        ___qtablewidgetitem11->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\351\235\242\347\247\257 (m\302\262)", nullptr));
        QTableWidgetItem *___qtablewidgetitem12 = sectionTable->horizontalHeaderItem(4);
        ___qtablewidgetitem12->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\351\235\242\347\247\257 (mm\302\262)", nullptr));
        QTableWidgetItem *___qtablewidgetitem13 = sectionTable->horizontalHeaderItem(5);
        ___qtablewidgetitem13->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\350\257\264\346\230\216", nullptr));
        tabs->setTabText(tabs->indexOf(sectionTable), QCoreApplication::translate("PropertyLibraryDialogClass", "\346\210\252\351\235\242", nullptr));
        syncGroup->setTitle(QCoreApplication::translate("PropertyLibraryDialogClass", "\345\275\223\345\211\215\346\250\241\345\236\213", nullptr));
        syncCheck->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\345\260\206\344\270\212\351\235\242\345\275\223\345\211\215\351\200\211\344\270\255\347\232\204\346\235\220\346\226\231\350\241\214\345\222\214\346\210\252\351\235\242\350\241\214\345\220\214\346\255\245\345\210\260\345\275\223\345\211\215\346\250\241\345\236\213", nullptr));
        materialLabel->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\347\233\256\346\240\207\346\235\220\346\226\231", nullptr));
        sectionLabel->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\347\233\256\346\240\207\346\210\252\351\235\242", nullptr));
        noteLabel->setText(QCoreApplication::translate("PropertyLibraryDialogClass", "\344\277\235\345\255\230\345\217\252\344\277\256\346\224\271\346\234\254\346\254\241\347\250\213\345\272\217\350\277\220\350\241\214\344\270\255\347\232\204\345\206\205\345\255\230\346\225\260\346\215\256\357\274\214\344\270\215\344\274\232\346\224\271\345\206\231 CSV \346\226\207\346\234\254\357\274\233\346\226\260\345\273\272\346\250\241\345\236\213\344\274\232\344\275\277\347\224\250\344\277\256\346\224\271\345\220\216\347\232\204\351\242\204\350\256\276\343\200\202", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PropertyLibraryDialogClass: public Ui_PropertyLibraryDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROPERTYLIBRARYDIALOG_H
