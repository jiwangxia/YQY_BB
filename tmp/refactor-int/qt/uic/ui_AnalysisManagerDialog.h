/********************************************************************************
** Form generated from reading UI file 'AnalysisManagerDialog.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ANALYSISMANAGERDIALOG_H
#define UI_ANALYSISMANAGERDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAbstractButton>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AnalysisManagerDialogClass
{
public:
    QVBoxLayout *rootLayout;
    QWidget *managerPage;
    QVBoxLayout *pageLayout;
    QTableWidget *resourceTable;
    QHBoxLayout *actionLayout;
    QSpacerItem *actionSpacer;
    QPushButton *addButton;
    QPushButton *editButton;
    QPushButton *deleteButton;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *AnalysisManagerDialogClass)
    {
        if (AnalysisManagerDialogClass->objectName().isEmpty())
            AnalysisManagerDialogClass->setObjectName("AnalysisManagerDialogClass");
        AnalysisManagerDialogClass->setMinimumSize(QSize(720, 440));
        rootLayout = new QVBoxLayout(AnalysisManagerDialogClass);
        rootLayout->setSpacing(12);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(16, 16, 16, 16);
        managerPage = new QWidget(AnalysisManagerDialogClass);
        managerPage->setObjectName("managerPage");
        pageLayout = new QVBoxLayout(managerPage);
        pageLayout->setSpacing(10);
        pageLayout->setObjectName("pageLayout");
        pageLayout->setContentsMargins(8, 10, 8, 8);
        resourceTable = new QTableWidget(managerPage);
        resourceTable->setObjectName("resourceTable");
        resourceTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        resourceTable->setSelectionMode(QAbstractItemView::SingleSelection);
        resourceTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        resourceTable->setAlternatingRowColors(true);

        pageLayout->addWidget(resourceTable);

        actionLayout = new QHBoxLayout();
        actionLayout->setObjectName("actionLayout");
        actionSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        actionLayout->addItem(actionSpacer);

        addButton = new QPushButton(managerPage);
        addButton->setObjectName("addButton");

        actionLayout->addWidget(addButton);

        editButton = new QPushButton(managerPage);
        editButton->setObjectName("editButton");
        editButton->setEnabled(false);

        actionLayout->addWidget(editButton);

        deleteButton = new QPushButton(managerPage);
        deleteButton->setObjectName("deleteButton");
        deleteButton->setEnabled(false);

        actionLayout->addWidget(deleteButton);


        pageLayout->addLayout(actionLayout);


        rootLayout->addWidget(managerPage);

        buttonBox = new QDialogButtonBox(AnalysisManagerDialogClass);
        buttonBox->setObjectName("buttonBox");
        buttonBox->setStandardButtons(QDialogButtonBox::Close);

        rootLayout->addWidget(buttonBox);


        retranslateUi(AnalysisManagerDialogClass);

        QMetaObject::connectSlotsByName(AnalysisManagerDialogClass);
    } // setupUi

    void retranslateUi(QDialog *AnalysisManagerDialogClass)
    {
        addButton->setText(QCoreApplication::translate("AnalysisManagerDialogClass", "\346\226\260\345\242\236", nullptr));
        editButton->setText(QCoreApplication::translate("AnalysisManagerDialogClass", "\347\274\226\350\276\221", nullptr));
        deleteButton->setText(QCoreApplication::translate("AnalysisManagerDialogClass", "\345\210\240\351\231\244", nullptr));
        (void)AnalysisManagerDialogClass;
    } // retranslateUi

};

namespace Ui {
    class AnalysisManagerDialogClass: public Ui_AnalysisManagerDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ANALYSISMANAGERDIALOG_H
