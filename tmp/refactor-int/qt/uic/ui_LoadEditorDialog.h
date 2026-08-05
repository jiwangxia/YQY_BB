/********************************************************************************
** Form generated from reading UI file 'LoadEditorDialog.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOADEDITORDIALOG_H
#define UI_LOADEDITORDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAbstractButton>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_LoadEditorDialogClass
{
public:
    QVBoxLayout *rootLayout;
    QFormLayout *formLayout;
    QLabel *nameLabel;
    QLineEdit *nameEdit;
    QLabel *stepLabel;
    QComboBox *stepCombo;
    QLabel *typeLabel;
    QComboBox *typeCombo;
    QLabel *targetLabel;
    QSpinBox *targetSpin;
    QLabel *directionLabel;
    QComboBox *directionCombo;
    QLabel *valueLabel;
    QDoubleSpinBox *valueSpin;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *LoadEditorDialogClass)
    {
        if (LoadEditorDialogClass->objectName().isEmpty())
            LoadEditorDialogClass->setObjectName("LoadEditorDialogClass");
        LoadEditorDialogClass->setMinimumSize(QSize(440, 0));
        rootLayout = new QVBoxLayout(LoadEditorDialogClass);
        rootLayout->setSpacing(14);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(18, 18, 18, 18);
        formLayout = new QFormLayout();
        formLayout->setObjectName("formLayout");
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        nameLabel = new QLabel(LoadEditorDialogClass);
        nameLabel->setObjectName("nameLabel");

        formLayout->setWidget(0, QFormLayout::LabelRole, nameLabel);

        nameEdit = new QLineEdit(LoadEditorDialogClass);
        nameEdit->setObjectName("nameEdit");

        formLayout->setWidget(0, QFormLayout::FieldRole, nameEdit);

        stepLabel = new QLabel(LoadEditorDialogClass);
        stepLabel->setObjectName("stepLabel");

        formLayout->setWidget(1, QFormLayout::LabelRole, stepLabel);

        stepCombo = new QComboBox(LoadEditorDialogClass);
        stepCombo->setObjectName("stepCombo");

        formLayout->setWidget(1, QFormLayout::FieldRole, stepCombo);

        typeLabel = new QLabel(LoadEditorDialogClass);
        typeLabel->setObjectName("typeLabel");

        formLayout->setWidget(2, QFormLayout::LabelRole, typeLabel);

        typeCombo = new QComboBox(LoadEditorDialogClass);
        typeCombo->addItem(QString());
        typeCombo->addItem(QString());
        typeCombo->addItem(QString());
        typeCombo->addItem(QString());
        typeCombo->setObjectName("typeCombo");

        formLayout->setWidget(2, QFormLayout::FieldRole, typeCombo);

        targetLabel = new QLabel(LoadEditorDialogClass);
        targetLabel->setObjectName("targetLabel");

        formLayout->setWidget(3, QFormLayout::LabelRole, targetLabel);

        targetSpin = new QSpinBox(LoadEditorDialogClass);
        targetSpin->setObjectName("targetSpin");
        targetSpin->setMinimum(1);
        targetSpin->setMaximum(2147483647);

        formLayout->setWidget(3, QFormLayout::FieldRole, targetSpin);

        directionLabel = new QLabel(LoadEditorDialogClass);
        directionLabel->setObjectName("directionLabel");

        formLayout->setWidget(4, QFormLayout::LabelRole, directionLabel);

        directionCombo = new QComboBox(LoadEditorDialogClass);
        directionCombo->setObjectName("directionCombo");

        formLayout->setWidget(4, QFormLayout::FieldRole, directionCombo);

        valueLabel = new QLabel(LoadEditorDialogClass);
        valueLabel->setObjectName("valueLabel");

        formLayout->setWidget(5, QFormLayout::LabelRole, valueLabel);

        valueSpin = new QDoubleSpinBox(LoadEditorDialogClass);
        valueSpin->setObjectName("valueSpin");
        valueSpin->setMinimum(-1000000000000000.000000000000000);
        valueSpin->setMaximum(1000000000000000.000000000000000);
        valueSpin->setDecimals(10);

        formLayout->setWidget(5, QFormLayout::FieldRole, valueSpin);


        rootLayout->addLayout(formLayout);

        buttonBox = new QDialogButtonBox(LoadEditorDialogClass);
        buttonBox->setObjectName("buttonBox");
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        rootLayout->addWidget(buttonBox);


        retranslateUi(LoadEditorDialogClass);

        QMetaObject::connectSlotsByName(LoadEditorDialogClass);
    } // setupUi

    void retranslateUi(QDialog *LoadEditorDialogClass)
    {
        nameLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "\345\220\215\347\247\260", nullptr));
        stepLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "\346\211\200\345\261\236\345\210\206\346\236\220\346\255\245", nullptr));
        typeLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "\347\261\273\345\236\213", nullptr));
        typeCombo->setItemText(0, QCoreApplication::translate("LoadEditorDialogClass", "\350\212\202\347\202\271\345\212\233", nullptr));
        typeCombo->setItemText(1, QCoreApplication::translate("LoadEditorDialogClass", "\345\215\225\345\205\203\350\215\267\350\275\275", nullptr));
        typeCombo->setItemText(2, QCoreApplication::translate("LoadEditorDialogClass", "\351\207\215\345\212\233", nullptr));
        typeCombo->setItemText(3, QCoreApplication::translate("LoadEditorDialogClass", "\351\243\216\350\215\267\350\275\275", nullptr));

        targetLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "\347\233\256\346\240\207\347\274\226\345\217\267", nullptr));
        directionLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "\346\226\271\345\220\221", nullptr));
        valueLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "\346\225\260\345\200\274", nullptr));
        (void)LoadEditorDialogClass;
    } // retranslateUi

};

namespace Ui {
    class LoadEditorDialogClass: public Ui_LoadEditorDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOADEDITORDIALOG_H
