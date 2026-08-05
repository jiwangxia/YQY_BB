/********************************************************************************
** Form generated from reading UI file 'AnalysisSummaryPanel.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ANALYSISSUMMARYPANEL_H
#define UI_ANALYSISSUMMARYPANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AnalysisSummaryPanelClass
{
public:
    QVBoxLayout *rootLayout;
    QFrame *analysisParameterCard;
    QGridLayout *actionsLayout;
    QPushButton *stepsButton;
    QPushButton *loadsButton;
    QPushButton *constraintsButton;
    QPushButton *mpcsButton;
    QPushButton *regionsButton;

    void setupUi(QWidget *AnalysisSummaryPanelClass)
    {
        if (AnalysisSummaryPanelClass->objectName().isEmpty())
            AnalysisSummaryPanelClass->setObjectName("AnalysisSummaryPanelClass");
        rootLayout = new QVBoxLayout(AnalysisSummaryPanelClass);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(0, 0, 0, 0);
        analysisParameterCard = new QFrame(AnalysisSummaryPanelClass);
        analysisParameterCard->setObjectName("analysisParameterCard");
        actionsLayout = new QGridLayout(analysisParameterCard);
        actionsLayout->setObjectName("actionsLayout");
        actionsLayout->setHorizontalSpacing(8);
        actionsLayout->setVerticalSpacing(8);
        actionsLayout->setContentsMargins(10, 10, 10, 10);
        stepsButton = new QPushButton(analysisParameterCard);
        stepsButton->setObjectName("stepsButton");
        stepsButton->setMinimumSize(QSize(116, 54));

        actionsLayout->addWidget(stepsButton, 0, 0, 1, 1);

        loadsButton = new QPushButton(analysisParameterCard);
        loadsButton->setObjectName("loadsButton");
        loadsButton->setMinimumSize(QSize(116, 54));

        actionsLayout->addWidget(loadsButton, 0, 1, 1, 1);

        constraintsButton = new QPushButton(analysisParameterCard);
        constraintsButton->setObjectName("constraintsButton");
        constraintsButton->setMinimumSize(QSize(116, 54));

        actionsLayout->addWidget(constraintsButton, 1, 0, 1, 1);

        mpcsButton = new QPushButton(analysisParameterCard);
        mpcsButton->setObjectName("mpcsButton");
        mpcsButton->setMinimumSize(QSize(116, 54));

        actionsLayout->addWidget(mpcsButton, 1, 1, 1, 1);

        regionsButton = new QPushButton(analysisParameterCard);
        regionsButton->setObjectName("regionsButton");
        regionsButton->setMinimumSize(QSize(116, 54));

        actionsLayout->addWidget(regionsButton, 2, 0, 1, 2);


        rootLayout->addWidget(analysisParameterCard);


        retranslateUi(AnalysisSummaryPanelClass);

        QMetaObject::connectSlotsByName(AnalysisSummaryPanelClass);
    } // setupUi

    void retranslateUi(QWidget *AnalysisSummaryPanelClass)
    {
        stepsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "\345\210\206\346\236\220\346\255\245", nullptr));
        loadsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "\350\215\267\350\275\275", nullptr));
        constraintsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "\347\272\246\346\235\237", nullptr));
        mpcsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "MPC", nullptr));
        regionsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "\350\256\241\347\256\227\345\214\272\345\237\237", nullptr));
        (void)AnalysisSummaryPanelClass;
    } // retranslateUi

};

namespace Ui {
    class AnalysisSummaryPanelClass: public Ui_AnalysisSummaryPanelClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ANALYSISSUMMARYPANEL_H
