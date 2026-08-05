/********************************************************************************
** Form generated from reading UI file 'SolveTaskManagerDialog.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SOLVETASKMANAGERDIALOG_H
#define UI_SOLVETASKMANAGERDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAbstractButton>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_SolveTaskManagerDialogClass
{
public:
    QVBoxLayout *rootLayout;
    QHBoxLayout *summaryLayout;
    QLabel *hintLabel;
    QSpacerItem *summarySpacer;
    QPushButton *solveAllButton;
    QPushButton *restartAllButton;
    QTableWidget *taskTable;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *SolveTaskManagerDialogClass)
    {
        if (SolveTaskManagerDialogClass->objectName().isEmpty())
            SolveTaskManagerDialogClass->setObjectName("SolveTaskManagerDialogClass");
        SolveTaskManagerDialogClass->setMinimumSize(QSize(720, 380));
        rootLayout = new QVBoxLayout(SolveTaskManagerDialogClass);
        rootLayout->setSpacing(12);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(16, 16, 16, 12);
        summaryLayout = new QHBoxLayout();
        summaryLayout->setObjectName("summaryLayout");
        hintLabel = new QLabel(SolveTaskManagerDialogClass);
        hintLabel->setObjectName("hintLabel");

        summaryLayout->addWidget(hintLabel);

        summarySpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        summaryLayout->addItem(summarySpacer);

        solveAllButton = new QPushButton(SolveTaskManagerDialogClass);
        solveAllButton->setObjectName("solveAllButton");
        solveAllButton->setMinimumSize(QSize(132, 0));

        summaryLayout->addWidget(solveAllButton);

        restartAllButton = new QPushButton(SolveTaskManagerDialogClass);
        restartAllButton->setObjectName("restartAllButton");
        restartAllButton->setMinimumSize(QSize(132, 0));

        summaryLayout->addWidget(restartAllButton);


        rootLayout->addLayout(summaryLayout);

        taskTable = new QTableWidget(SolveTaskManagerDialogClass);
        if (taskTable->columnCount() < 6)
            taskTable->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        taskTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        taskTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        taskTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        taskTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        taskTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        taskTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        taskTable->setObjectName("taskTable");
        taskTable->setColumnCount(6);
        taskTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        taskTable->setSelectionMode(QAbstractItemView::SingleSelection);
        taskTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        taskTable->setAlternatingRowColors(true);

        rootLayout->addWidget(taskTable);

        buttonBox = new QDialogButtonBox(SolveTaskManagerDialogClass);
        buttonBox->setObjectName("buttonBox");
        buttonBox->setStandardButtons(QDialogButtonBox::Close);

        rootLayout->addWidget(buttonBox);


        retranslateUi(SolveTaskManagerDialogClass);
        QObject::connect(buttonBox, &QDialogButtonBox::rejected, SolveTaskManagerDialogClass, qOverload<>(&QDialog::close));

        QMetaObject::connectSlotsByName(SolveTaskManagerDialogClass);
    } // setupUi

    void retranslateUi(QDialog *SolveTaskManagerDialogClass)
    {
        SolveTaskManagerDialogClass->setWindowTitle(QCoreApplication::translate("SolveTaskManagerDialogClass", "\345\205\250\351\203\250\345\210\206\346\236\220\346\255\245\346\261\202\350\247\243", nullptr));
        hintLabel->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\345\205\250\351\203\250\345\210\206\346\236\220\346\255\245\347\256\227\344\276\213\357\274\214\345\217\257\345\234\250\346\255\244\347\233\264\346\216\245\350\277\220\350\241\214\343\200\201\345\201\234\346\255\242\345\271\266\346\237\245\347\234\213\345\256\214\346\225\264\350\277\233\345\272\246\343\200\202", nullptr));
        solveAllButton->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\345\205\250\351\203\250\350\256\241\347\256\227", nullptr));
#if QT_CONFIG(tooltip)
        solveAllButton->setToolTip(QCoreApplication::translate("SolveTaskManagerDialogClass", "\345\260\206\346\211\200\346\234\211\345\276\205\350\277\220\350\241\214\345\210\206\346\236\220\346\255\245\345\212\240\345\205\245\351\230\237\345\210\227\357\274\233\346\255\243\345\234\250\350\256\241\347\256\227\343\200\201\345\267\262\346\216\222\351\230\237\345\222\214\345\267\262\346\234\211\347\273\223\346\236\234\347\232\204\347\256\227\344\276\213\344\270\215\344\274\232\351\207\215\345\244\215\350\256\241\347\256\227\357\274\233\345\215\225\344\270\252\345\210\206\346\236\220\346\255\245\345\244\261\350\264\245\344\270\215\344\274\232\345\201\234\346\255\242\345\205\266\344\273\226\351\230\237\345\210\227\344\273\273\345\212\241", nullptr));
#endif // QT_CONFIG(tooltip)
        restartAllButton->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\351\207\215\346\226\260\350\256\241\347\256\227", nullptr));
#if QT_CONFIG(tooltip)
        restartAllButton->setToolTip(QCoreApplication::translate("SolveTaskManagerDialogClass", "\351\207\215\346\226\260\350\256\241\347\256\227\346\211\200\346\234\211\345\210\206\346\236\220\346\255\245\357\274\233\346\255\243\345\234\250\346\216\222\351\230\237\346\210\226\350\256\241\347\256\227\347\232\204\344\273\273\345\212\241\344\274\232\345\205\210\345\256\211\345\205\250\345\201\234\346\255\242\357\274\214\345\206\215\344\273\216\345\244\264\345\212\240\345\205\245\350\256\241\347\256\227\351\230\237\345\210\227", nullptr));
#endif // QT_CONFIG(tooltip)
        QTableWidgetItem *___qtablewidgetitem = taskTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\346\223\215\344\275\234", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = taskTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\347\256\227\344\276\213\345\220\215\347\247\260", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = taskTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\345\210\206\346\236\220\346\255\245", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = taskTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\350\277\233\345\272\246", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = taskTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\347\273\223\346\236\234", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = taskTable->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "\350\200\227\346\227\266", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SolveTaskManagerDialogClass: public Ui_SolveTaskManagerDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SOLVETASKMANAGERDIALOG_H
