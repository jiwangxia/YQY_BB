/********************************************************************************
** Form generated from reading UI file 'StepEditorDialog.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_STEPEDITORDIALOG_H
#define UI_STEPEDITORDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAbstractButton>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include "Widgets/CompactDoubleSpinBox.h"

QT_BEGIN_NAMESPACE

class Ui_StepEditorDialogClass
{
public:
    QVBoxLayout *rootLayout;
    QFormLayout *formLayout;
    QLabel *nameLabel;
    QLineEdit *nameEdit;
    QLabel *typeLabel;
    QComboBox *typeCombo;
    QLabel *timeLabel;
    CompactDoubleSpinBox *timeSpin;
    QLabel *incrementLabel;
    CompactDoubleSpinBox *incrementSpin;
    QLabel *toleranceLabel;
    CompactDoubleSpinBox *toleranceSpin;
    QLabel *iterationsLabel;
    QSpinBox *iterationsSpin;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *StepEditorDialogClass)
    {
        if (StepEditorDialogClass->objectName().isEmpty())
            StepEditorDialogClass->setObjectName("StepEditorDialogClass");
        StepEditorDialogClass->setMinimumSize(QSize(420, 0));
        rootLayout = new QVBoxLayout(StepEditorDialogClass);
        rootLayout->setSpacing(14);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(18, 18, 18, 18);
        formLayout = new QFormLayout();
        formLayout->setObjectName("formLayout");
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        nameLabel = new QLabel(StepEditorDialogClass);
        nameLabel->setObjectName("nameLabel");

        formLayout->setWidget(0, QFormLayout::LabelRole, nameLabel);

        nameEdit = new QLineEdit(StepEditorDialogClass);
        nameEdit->setObjectName("nameEdit");

        formLayout->setWidget(0, QFormLayout::FieldRole, nameEdit);

        typeLabel = new QLabel(StepEditorDialogClass);
        typeLabel->setObjectName("typeLabel");

        formLayout->setWidget(1, QFormLayout::LabelRole, typeLabel);

        typeCombo = new QComboBox(StepEditorDialogClass);
        typeCombo->addItem(QString());
        typeCombo->addItem(QString());
        typeCombo->setObjectName("typeCombo");

        formLayout->setWidget(1, QFormLayout::FieldRole, typeCombo);

        timeLabel = new QLabel(StepEditorDialogClass);
        timeLabel->setObjectName("timeLabel");

        formLayout->setWidget(2, QFormLayout::LabelRole, timeLabel);

        timeSpin = new CompactDoubleSpinBox(StepEditorDialogClass);
        timeSpin->setObjectName("timeSpin");

        formLayout->setWidget(2, QFormLayout::FieldRole, timeSpin);

        incrementLabel = new QLabel(StepEditorDialogClass);
        incrementLabel->setObjectName("incrementLabel");

        formLayout->setWidget(3, QFormLayout::LabelRole, incrementLabel);

        incrementSpin = new CompactDoubleSpinBox(StepEditorDialogClass);
        incrementSpin->setObjectName("incrementSpin");

        formLayout->setWidget(3, QFormLayout::FieldRole, incrementSpin);

        toleranceLabel = new QLabel(StepEditorDialogClass);
        toleranceLabel->setObjectName("toleranceLabel");

        formLayout->setWidget(4, QFormLayout::LabelRole, toleranceLabel);

        toleranceSpin = new CompactDoubleSpinBox(StepEditorDialogClass);
        toleranceSpin->setObjectName("toleranceSpin");

        formLayout->setWidget(4, QFormLayout::FieldRole, toleranceSpin);

        iterationsLabel = new QLabel(StepEditorDialogClass);
        iterationsLabel->setObjectName("iterationsLabel");

        formLayout->setWidget(5, QFormLayout::LabelRole, iterationsLabel);

        iterationsSpin = new QSpinBox(StepEditorDialogClass);
        iterationsSpin->setObjectName("iterationsSpin");
        iterationsSpin->setMinimum(1);
        iterationsSpin->setMaximum(100000);

        formLayout->setWidget(5, QFormLayout::FieldRole, iterationsSpin);


        rootLayout->addLayout(formLayout);

        buttonBox = new QDialogButtonBox(StepEditorDialogClass);
        buttonBox->setObjectName("buttonBox");
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        rootLayout->addWidget(buttonBox);


        retranslateUi(StepEditorDialogClass);

        QMetaObject::connectSlotsByName(StepEditorDialogClass);
    } // setupUi

    void retranslateUi(QDialog *StepEditorDialogClass)
    {
        nameLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "\345\220\215\347\247\260", nullptr));
        typeLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "\347\261\273\345\236\213", nullptr));
        typeCombo->setItemText(0, QCoreApplication::translate("StepEditorDialogClass", "\351\235\231\345\212\233\345\210\206\346\236\220", nullptr));
        typeCombo->setItemText(1, QCoreApplication::translate("StepEditorDialogClass", "\345\212\250\345\212\233\345\210\206\346\236\220", nullptr));

        timeLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "\346\200\273\346\227\266\351\227\264", nullptr));
        incrementLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "\346\255\245\351\225\277 / \345\242\236\351\207\217", nullptr));
        toleranceLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "\346\224\266\346\225\233\345\256\271\345\267\256", nullptr));
        iterationsLabel->setText(QCoreApplication::translate("StepEditorDialogClass", "\346\234\200\345\244\247\350\277\255\344\273\243\346\225\260", nullptr));
        (void)StepEditorDialogClass;
    } // retranslateUi

};

namespace Ui {
    class StepEditorDialogClass: public Ui_StepEditorDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_STEPEDITORDIALOG_H
