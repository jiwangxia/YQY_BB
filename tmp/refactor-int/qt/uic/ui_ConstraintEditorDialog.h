/********************************************************************************
** Form generated from reading UI file 'ConstraintEditorDialog.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONSTRAINTEDITORDIALOG_H
#define UI_CONSTRAINTEDITORDIALOG_H

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

class Ui_ConstraintEditorDialogClass
{
public:
    QVBoxLayout *rootLayout;
    QFormLayout *formLayout;
    QLabel *nameLabel;
    QLineEdit *nameEdit;
    QLabel *stepLabel;
    QComboBox *stepCombo;
    QLabel *nodeLabel;
    QSpinBox *nodeSpin;
    QLabel *directionLabel;
    QComboBox *directionCombo;
    QLabel *valueLabel;
    QDoubleSpinBox *valueSpin;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *ConstraintEditorDialogClass)
    {
        if (ConstraintEditorDialogClass->objectName().isEmpty())
            ConstraintEditorDialogClass->setObjectName("ConstraintEditorDialogClass");
        ConstraintEditorDialogClass->setMinimumSize(QSize(440, 0));
        rootLayout = new QVBoxLayout(ConstraintEditorDialogClass);
        rootLayout->setSpacing(14);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(18, 18, 18, 18);
        formLayout = new QFormLayout();
        formLayout->setObjectName("formLayout");
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        nameLabel = new QLabel(ConstraintEditorDialogClass);
        nameLabel->setObjectName("nameLabel");

        formLayout->setWidget(0, QFormLayout::LabelRole, nameLabel);

        nameEdit = new QLineEdit(ConstraintEditorDialogClass);
        nameEdit->setObjectName("nameEdit");

        formLayout->setWidget(0, QFormLayout::FieldRole, nameEdit);

        stepLabel = new QLabel(ConstraintEditorDialogClass);
        stepLabel->setObjectName("stepLabel");

        formLayout->setWidget(1, QFormLayout::LabelRole, stepLabel);

        stepCombo = new QComboBox(ConstraintEditorDialogClass);
        stepCombo->setObjectName("stepCombo");

        formLayout->setWidget(1, QFormLayout::FieldRole, stepCombo);

        nodeLabel = new QLabel(ConstraintEditorDialogClass);
        nodeLabel->setObjectName("nodeLabel");

        formLayout->setWidget(2, QFormLayout::LabelRole, nodeLabel);

        nodeSpin = new QSpinBox(ConstraintEditorDialogClass);
        nodeSpin->setObjectName("nodeSpin");
        nodeSpin->setMinimum(1);
        nodeSpin->setMaximum(2147483647);

        formLayout->setWidget(2, QFormLayout::FieldRole, nodeSpin);

        directionLabel = new QLabel(ConstraintEditorDialogClass);
        directionLabel->setObjectName("directionLabel");

        formLayout->setWidget(3, QFormLayout::LabelRole, directionLabel);

        directionCombo = new QComboBox(ConstraintEditorDialogClass);
        directionCombo->setObjectName("directionCombo");

        formLayout->setWidget(3, QFormLayout::FieldRole, directionCombo);

        valueLabel = new QLabel(ConstraintEditorDialogClass);
        valueLabel->setObjectName("valueLabel");

        formLayout->setWidget(4, QFormLayout::LabelRole, valueLabel);

        valueSpin = new QDoubleSpinBox(ConstraintEditorDialogClass);
        valueSpin->setObjectName("valueSpin");
        valueSpin->setMinimum(-1000000000000000.000000000000000);
        valueSpin->setMaximum(1000000000000000.000000000000000);
        valueSpin->setDecimals(12);

        formLayout->setWidget(4, QFormLayout::FieldRole, valueSpin);


        rootLayout->addLayout(formLayout);

        buttonBox = new QDialogButtonBox(ConstraintEditorDialogClass);
        buttonBox->setObjectName("buttonBox");
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        rootLayout->addWidget(buttonBox);


        retranslateUi(ConstraintEditorDialogClass);

        QMetaObject::connectSlotsByName(ConstraintEditorDialogClass);
    } // setupUi

    void retranslateUi(QDialog *ConstraintEditorDialogClass)
    {
        nameLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "\345\220\215\347\247\260", nullptr));
        stepLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "\351\246\226\346\254\241\347\224\237\346\225\210\345\210\206\346\236\220\346\255\245", nullptr));
        nodeLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "\350\212\202\347\202\271\347\274\226\345\217\267", nullptr));
        directionLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "\346\226\271\345\220\221", nullptr));
        valueLabel->setText(QCoreApplication::translate("ConstraintEditorDialogClass", "\344\275\215\347\247\273\345\200\274", nullptr));
        (void)ConstraintEditorDialogClass;
    } // retranslateUi

};

namespace Ui {
    class ConstraintEditorDialogClass: public Ui_ConstraintEditorDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONSTRAINTEDITORDIALOG_H
