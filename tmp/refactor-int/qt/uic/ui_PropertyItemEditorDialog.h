/********************************************************************************
** Form generated from reading UI file 'PropertyItemEditorDialog.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROPERTYITEMEDITORDIALOG_H
#define UI_PROPERTYITEMEDITORDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAbstractButton>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_PropertyItemEditorDialogClass
{
public:
    QVBoxLayout *rootLayout;
    QFormLayout *formLayout;
    QLabel *sourceCaption;
    QLabel *sourceValue;
    QLabel *idCaption;
    QLabel *idValue;
    QLabel *typeCaption;
    QLabel *typeValue;
    QLabel *youngCaption;
    QDoubleSpinBox *youngSpin;
    QLabel *poissonCaption;
    QDoubleSpinBox *poissonSpin;
    QLabel *densityCaption;
    QDoubleSpinBox *densitySpin;
    QLabel *stressCaption;
    QDoubleSpinBox *stressSpin;
    QLabel *expansionCaption;
    QDoubleSpinBox *expansionSpin;
    QLabel *areaCaption;
    QDoubleSpinBox *areaSpin;
    QLabel *widthCaption;
    QDoubleSpinBox *widthSpin;
    QLabel *heightCaption;
    QDoubleSpinBox *heightSpin;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *PropertyItemEditorDialogClass)
    {
        if (PropertyItemEditorDialogClass->objectName().isEmpty())
            PropertyItemEditorDialogClass->setObjectName("PropertyItemEditorDialogClass");
        PropertyItemEditorDialogClass->setMinimumSize(QSize(520, 0));
        rootLayout = new QVBoxLayout(PropertyItemEditorDialogClass);
        rootLayout->setObjectName("rootLayout");
        formLayout = new QFormLayout();
        formLayout->setObjectName("formLayout");
        formLayout->setLabelAlignment(Qt::AlignRight|Qt::AlignVCenter);
        sourceCaption = new QLabel(PropertyItemEditorDialogClass);
        sourceCaption->setObjectName("sourceCaption");

        formLayout->setWidget(0, QFormLayout::LabelRole, sourceCaption);

        sourceValue = new QLabel(PropertyItemEditorDialogClass);
        sourceValue->setObjectName("sourceValue");

        formLayout->setWidget(0, QFormLayout::FieldRole, sourceValue);

        idCaption = new QLabel(PropertyItemEditorDialogClass);
        idCaption->setObjectName("idCaption");

        formLayout->setWidget(1, QFormLayout::LabelRole, idCaption);

        idValue = new QLabel(PropertyItemEditorDialogClass);
        idValue->setObjectName("idValue");

        formLayout->setWidget(1, QFormLayout::FieldRole, idValue);

        typeCaption = new QLabel(PropertyItemEditorDialogClass);
        typeCaption->setObjectName("typeCaption");

        formLayout->setWidget(2, QFormLayout::LabelRole, typeCaption);

        typeValue = new QLabel(PropertyItemEditorDialogClass);
        typeValue->setObjectName("typeValue");

        formLayout->setWidget(2, QFormLayout::FieldRole, typeValue);

        youngCaption = new QLabel(PropertyItemEditorDialogClass);
        youngCaption->setObjectName("youngCaption");

        formLayout->setWidget(3, QFormLayout::LabelRole, youngCaption);

        youngSpin = new QDoubleSpinBox(PropertyItemEditorDialogClass);
        youngSpin->setObjectName("youngSpin");

        formLayout->setWidget(3, QFormLayout::FieldRole, youngSpin);

        poissonCaption = new QLabel(PropertyItemEditorDialogClass);
        poissonCaption->setObjectName("poissonCaption");

        formLayout->setWidget(4, QFormLayout::LabelRole, poissonCaption);

        poissonSpin = new QDoubleSpinBox(PropertyItemEditorDialogClass);
        poissonSpin->setObjectName("poissonSpin");

        formLayout->setWidget(4, QFormLayout::FieldRole, poissonSpin);

        densityCaption = new QLabel(PropertyItemEditorDialogClass);
        densityCaption->setObjectName("densityCaption");

        formLayout->setWidget(5, QFormLayout::LabelRole, densityCaption);

        densitySpin = new QDoubleSpinBox(PropertyItemEditorDialogClass);
        densitySpin->setObjectName("densitySpin");

        formLayout->setWidget(5, QFormLayout::FieldRole, densitySpin);

        stressCaption = new QLabel(PropertyItemEditorDialogClass);
        stressCaption->setObjectName("stressCaption");

        formLayout->setWidget(6, QFormLayout::LabelRole, stressCaption);

        stressSpin = new QDoubleSpinBox(PropertyItemEditorDialogClass);
        stressSpin->setObjectName("stressSpin");

        formLayout->setWidget(6, QFormLayout::FieldRole, stressSpin);

        expansionCaption = new QLabel(PropertyItemEditorDialogClass);
        expansionCaption->setObjectName("expansionCaption");

        formLayout->setWidget(7, QFormLayout::LabelRole, expansionCaption);

        expansionSpin = new QDoubleSpinBox(PropertyItemEditorDialogClass);
        expansionSpin->setObjectName("expansionSpin");

        formLayout->setWidget(7, QFormLayout::FieldRole, expansionSpin);

        areaCaption = new QLabel(PropertyItemEditorDialogClass);
        areaCaption->setObjectName("areaCaption");

        formLayout->setWidget(8, QFormLayout::LabelRole, areaCaption);

        areaSpin = new QDoubleSpinBox(PropertyItemEditorDialogClass);
        areaSpin->setObjectName("areaSpin");

        formLayout->setWidget(8, QFormLayout::FieldRole, areaSpin);

        widthCaption = new QLabel(PropertyItemEditorDialogClass);
        widthCaption->setObjectName("widthCaption");

        formLayout->setWidget(9, QFormLayout::LabelRole, widthCaption);

        widthSpin = new QDoubleSpinBox(PropertyItemEditorDialogClass);
        widthSpin->setObjectName("widthSpin");

        formLayout->setWidget(9, QFormLayout::FieldRole, widthSpin);

        heightCaption = new QLabel(PropertyItemEditorDialogClass);
        heightCaption->setObjectName("heightCaption");

        formLayout->setWidget(10, QFormLayout::LabelRole, heightCaption);

        heightSpin = new QDoubleSpinBox(PropertyItemEditorDialogClass);
        heightSpin->setObjectName("heightSpin");

        formLayout->setWidget(10, QFormLayout::FieldRole, heightSpin);


        rootLayout->addLayout(formLayout);

        buttonBox = new QDialogButtonBox(PropertyItemEditorDialogClass);
        buttonBox->setObjectName("buttonBox");
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);

        rootLayout->addWidget(buttonBox);


        retranslateUi(PropertyItemEditorDialogClass);

        QMetaObject::connectSlotsByName(PropertyItemEditorDialogClass);
    } // setupUi

    void retranslateUi(QDialog *PropertyItemEditorDialogClass)
    {
        sourceCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\346\235\245\346\272\220", nullptr));
        idCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "ID", nullptr));
        typeCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\346\210\252\351\235\242\347\261\273\345\236\213", nullptr));
        youngCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\345\274\271\346\200\247\346\250\241\351\207\217 E (Pa)", nullptr));
        poissonCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\346\263\212\346\235\276\346\257\224", nullptr));
        densityCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\345\257\206\345\272\246 (kg/m\302\263)", nullptr));
        stressCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\350\256\270\347\224\250\345\272\224\345\212\233 (Pa)", nullptr));
        expansionCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\347\203\255\350\206\250\350\203\200\347\263\273\346\225\260", nullptr));
        areaCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\346\210\252\351\235\242\351\235\242\347\247\257 (m\302\262)", nullptr));
        widthCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\345\256\275\345\272\246 (m)", nullptr));
        heightCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "\351\253\230\345\272\246 (m)", nullptr));
        (void)PropertyItemEditorDialogClass;
    } // retranslateUi

};

namespace Ui {
    class PropertyItemEditorDialogClass: public Ui_PropertyItemEditorDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROPERTYITEMEDITORDIALOG_H
