/********************************************************************************
** Form generated from reading UI file 'SettingsPanel.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SETTINGSPANEL_H
#define UI_SETTINGSPANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SettingsPanelClass
{
public:
    QVBoxLayout *rootLayout;
    QFrame *infoCard;
    QVBoxLayout *cardLayout;
    QLabel *titleLabel;
    QLabel *descriptionLabel;
    QFormLayout *concurrencyForm;
    QLabel *concurrencyLabel;
    QSpinBox *concurrencySpin;
    QLabel *nodeLabelModeLabel;
    QComboBox *nodeLabelModeCombo;
    QLabel *statusLabel;

    void setupUi(QWidget *SettingsPanelClass)
    {
        if (SettingsPanelClass->objectName().isEmpty())
            SettingsPanelClass->setObjectName("SettingsPanelClass");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SettingsPanelClass->sizePolicy().hasHeightForWidth());
        SettingsPanelClass->setSizePolicy(sizePolicy);
        rootLayout = new QVBoxLayout(SettingsPanelClass);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(0, 0, 0, 0);
        infoCard = new QFrame(SettingsPanelClass);
        infoCard->setObjectName("infoCard");
        cardLayout = new QVBoxLayout(infoCard);
        cardLayout->setSpacing(10);
        cardLayout->setObjectName("cardLayout");
        cardLayout->setContentsMargins(18, 16, 18, 16);
        titleLabel = new QLabel(infoCard);
        titleLabel->setObjectName("titleLabel");

        cardLayout->addWidget(titleLabel);

        descriptionLabel = new QLabel(infoCard);
        descriptionLabel->setObjectName("descriptionLabel");
        descriptionLabel->setWordWrap(true);

        cardLayout->addWidget(descriptionLabel);

        concurrencyForm = new QFormLayout();
        concurrencyForm->setObjectName("concurrencyForm");
        concurrencyForm->setHorizontalSpacing(18);
        concurrencyForm->setVerticalSpacing(8);
        concurrencyLabel = new QLabel(infoCard);
        concurrencyLabel->setObjectName("concurrencyLabel");

        concurrencyForm->setWidget(0, QFormLayout::LabelRole, concurrencyLabel);

        concurrencySpin = new QSpinBox(infoCard);
        concurrencySpin->setObjectName("concurrencySpin");
        concurrencySpin->setMinimumSize(QSize(150, 0));
        concurrencySpin->setMinimum(1);
        concurrencySpin->setMaximum(12);
        concurrencySpin->setValue(5);

        concurrencyForm->setWidget(0, QFormLayout::FieldRole, concurrencySpin);

        nodeLabelModeLabel = new QLabel(infoCard);
        nodeLabelModeLabel->setObjectName("nodeLabelModeLabel");

        concurrencyForm->setWidget(1, QFormLayout::LabelRole, nodeLabelModeLabel);

        nodeLabelModeCombo = new QComboBox(infoCard);
        nodeLabelModeCombo->addItem(QString());
        nodeLabelModeCombo->addItem(QString());
        nodeLabelModeCombo->setObjectName("nodeLabelModeCombo");
        nodeLabelModeCombo->setMinimumSize(QSize(150, 0));

        concurrencyForm->setWidget(1, QFormLayout::FieldRole, nodeLabelModeCombo);


        cardLayout->addLayout(concurrencyForm);

        statusLabel = new QLabel(infoCard);
        statusLabel->setObjectName("statusLabel");
        statusLabel->setWordWrap(true);

        cardLayout->addWidget(statusLabel);


        rootLayout->addWidget(infoCard);


        retranslateUi(SettingsPanelClass);

        QMetaObject::connectSlotsByName(SettingsPanelClass);
    } // setupUi

    void retranslateUi(QWidget *SettingsPanelClass)
    {
        titleLabel->setText(QCoreApplication::translate("SettingsPanelClass", "\346\261\202\350\247\243\345\244\232\347\272\277\347\250\213", nullptr));
        descriptionLabel->setText(QCoreApplication::translate("SettingsPanelClass", "\346\216\247\345\210\266\345\217\257\344\273\245\345\220\214\346\227\266\350\256\241\347\256\227\347\232\204\345\210\206\346\236\220\346\255\245\346\225\260\351\207\217\343\200\202\350\266\205\345\207\272\344\270\212\351\231\220\347\232\204\344\273\273\345\212\241\350\207\252\345\212\250\346\216\222\351\230\237\357\274\214\345\273\272\350\256\256\344\277\235\347\225\231 CPU \345\222\214\345\206\205\345\255\230\350\265\204\346\272\220\347\273\231\346\250\241\345\236\213\346\230\276\347\244\272\345\217\212\345\205\266\344\273\226\346\250\241\345\235\227\343\200\202", nullptr));
        concurrencyLabel->setText(QCoreApplication::translate("SettingsPanelClass", "\346\234\200\345\244\247\345\271\266\345\217\221\345\210\206\346\236\220\346\255\245", nullptr));
        concurrencySpin->setSuffix(QCoreApplication::translate("SettingsPanelClass", " \344\270\252\345\210\206\346\236\220\346\255\245", nullptr));
        nodeLabelModeLabel->setText(QCoreApplication::translate("SettingsPanelClass", "\350\212\202\347\202\271\347\274\226\345\217\267\346\230\276\347\244\272", nullptr));
        nodeLabelModeCombo->setItemText(0, QCoreApplication::translate("SettingsPanelClass", "\345\205\250\351\203\250", nullptr));
        nodeLabelModeCombo->setItemText(1, QCoreApplication::translate("SettingsPanelClass", "\350\207\252\351\200\202\345\272\224", nullptr));

        statusLabel->setText(QCoreApplication::translate("SettingsPanelClass", "\346\255\243\345\234\250\346\243\200\346\265\213\346\234\254\346\234\272\345\217\257\347\224\250\347\272\277\347\250\213\342\200\246\342\200\246", nullptr));
        (void)SettingsPanelClass;
    } // retranslateUi

};

namespace Ui {
    class SettingsPanelClass: public Ui_SettingsPanelClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SETTINGSPANEL_H
