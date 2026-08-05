/********************************************************************************
** Form generated from reading UI file 'PropertyModule.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROPERTYMODULE_H
#define UI_PROPERTYMODULE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PropertyModuleClass
{
public:
    QVBoxLayout *rootLayout;
    QLabel *descriptionLabel;
    QTabWidget *propertyTabs;
    QTreeWidget *materialTree;
    QTreeWidget *sectionTree;
    QTableWidget *materialTable;
    QTableWidget *sectionTable;
    QVBoxLayout *buttonLayout;
    QPushButton *refreshButton;
    QSpacerItem *buttonSpacer;
    QPushButton *applyButton;

    void setupUi(QWidget *PropertyModuleClass)
    {
        if (PropertyModuleClass->objectName().isEmpty())
            PropertyModuleClass->setObjectName("PropertyModuleClass");
        rootLayout = new QVBoxLayout(PropertyModuleClass);
        rootLayout->setSpacing(8);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(8, 8, 8, 8);
        descriptionLabel = new QLabel(PropertyModuleClass);
        descriptionLabel->setObjectName("descriptionLabel");
        descriptionLabel->setWordWrap(true);

        rootLayout->addWidget(descriptionLabel);

        propertyTabs = new QTabWidget(PropertyModuleClass);
        propertyTabs->setObjectName("propertyTabs");
        materialTree = new QTreeWidget();
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setText(0, QString::fromUtf8("1"));
        materialTree->setHeaderItem(__qtreewidgetitem);
        materialTree->setObjectName("materialTree");
        materialTree->setHeaderHidden(true);
        materialTree->setAlternatingRowColors(true);
        materialTree->setUniformRowHeights(true);
        materialTree->setRootIsDecorated(true);
        materialTree->setIndentation(12);
        materialTree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        propertyTabs->addTab(materialTree, QString());
        sectionTree = new QTreeWidget();
        QTreeWidgetItem *__qtreewidgetitem1 = new QTreeWidgetItem();
        __qtreewidgetitem1->setText(0, QString::fromUtf8("1"));
        sectionTree->setHeaderItem(__qtreewidgetitem1);
        sectionTree->setObjectName("sectionTree");
        sectionTree->setHeaderHidden(true);
        sectionTree->setAlternatingRowColors(true);
        sectionTree->setUniformRowHeights(true);
        sectionTree->setRootIsDecorated(true);
        sectionTree->setIndentation(12);
        sectionTree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        propertyTabs->addTab(sectionTree, QString());

        rootLayout->addWidget(propertyTabs);

        materialTable = new QTableWidget(PropertyModuleClass);
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
        materialTable->setVisible(false);
        materialTable->setColumnCount(8);

        rootLayout->addWidget(materialTable);

        sectionTable = new QTableWidget(PropertyModuleClass);
        if (sectionTable->columnCount() < 8)
            sectionTable->setColumnCount(8);
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
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        sectionTable->setHorizontalHeaderItem(6, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        sectionTable->setHorizontalHeaderItem(7, __qtablewidgetitem15);
        sectionTable->setObjectName("sectionTable");
        sectionTable->setVisible(false);
        sectionTable->setColumnCount(8);

        rootLayout->addWidget(sectionTable);

        buttonLayout = new QVBoxLayout();
        buttonLayout->setSpacing(6);
        buttonLayout->setObjectName("buttonLayout");
        refreshButton = new QPushButton(PropertyModuleClass);
        refreshButton->setObjectName("refreshButton");
        refreshButton->setMinimumSize(QSize(96, 0));

        buttonLayout->addWidget(refreshButton);

        buttonSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        buttonLayout->addItem(buttonSpacer);

        applyButton = new QPushButton(PropertyModuleClass);
        applyButton->setObjectName("applyButton");
        applyButton->setMinimumSize(QSize(190, 0));
        applyButton->setVisible(false);

        buttonLayout->addWidget(applyButton);


        rootLayout->addLayout(buttonLayout);


        retranslateUi(PropertyModuleClass);

        QMetaObject::connectSlotsByName(PropertyModuleClass);
    } // setupUi

    void retranslateUi(QWidget *PropertyModuleClass)
    {
        descriptionLabel->setText(QCoreApplication::translate("PropertyModuleClass", "\345\210\227\345\207\272\351\273\230\350\256\244\345\261\236\346\200\247\345\272\223\345\222\214\346\211\200\346\234\211\345\267\262\345\212\240\350\275\275\346\250\241\345\236\213\347\232\204\346\235\220\346\226\231\343\200\201\346\210\252\351\235\242\343\200\202\345\217\214\345\207\273\346\235\241\347\233\256\345\217\257\346\237\245\347\234\213\350\257\246\346\203\205\343\200\201\344\277\256\346\224\271\345\271\266\344\277\235\345\255\230\343\200\202", nullptr));
        propertyTabs->setTabText(propertyTabs->indexOf(materialTree), QCoreApplication::translate("PropertyModuleClass", "\346\235\220\346\226\231", nullptr));
        propertyTabs->setTabText(propertyTabs->indexOf(sectionTree), QCoreApplication::translate("PropertyModuleClass", "\346\210\252\351\235\242", nullptr));
        QTableWidgetItem *___qtablewidgetitem = materialTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("PropertyModuleClass", "\346\250\241\345\236\213\347\274\226\345\217\267", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = materialTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("PropertyModuleClass", "\346\250\241\345\236\213\346\226\207\344\273\266", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = materialTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("PropertyModuleClass", "\346\235\220\346\226\231 ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = materialTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("PropertyModuleClass", "E (Pa)", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = materialTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("PropertyModuleClass", "\346\263\212\346\235\276\346\257\224", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = materialTable->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("PropertyModuleClass", "\345\257\206\345\272\246 (kg/m\302\263)", nullptr));
        QTableWidgetItem *___qtablewidgetitem6 = materialTable->horizontalHeaderItem(6);
        ___qtablewidgetitem6->setText(QCoreApplication::translate("PropertyModuleClass", "\350\256\270\347\224\250\345\272\224\345\212\233 (Pa)", nullptr));
        QTableWidgetItem *___qtablewidgetitem7 = materialTable->horizontalHeaderItem(7);
        ___qtablewidgetitem7->setText(QCoreApplication::translate("PropertyModuleClass", "\350\206\250\350\203\200\347\263\273\346\225\260", nullptr));
        QTableWidgetItem *___qtablewidgetitem8 = sectionTable->horizontalHeaderItem(0);
        ___qtablewidgetitem8->setText(QCoreApplication::translate("PropertyModuleClass", "\346\250\241\345\236\213\347\274\226\345\217\267", nullptr));
        QTableWidgetItem *___qtablewidgetitem9 = sectionTable->horizontalHeaderItem(1);
        ___qtablewidgetitem9->setText(QCoreApplication::translate("PropertyModuleClass", "\346\250\241\345\236\213\346\226\207\344\273\266", nullptr));
        QTableWidgetItem *___qtablewidgetitem10 = sectionTable->horizontalHeaderItem(2);
        ___qtablewidgetitem10->setText(QCoreApplication::translate("PropertyModuleClass", "\346\210\252\351\235\242 ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem11 = sectionTable->horizontalHeaderItem(3);
        ___qtablewidgetitem11->setText(QCoreApplication::translate("PropertyModuleClass", "\347\261\273\345\236\213", nullptr));
        QTableWidgetItem *___qtablewidgetitem12 = sectionTable->horizontalHeaderItem(4);
        ___qtablewidgetitem12->setText(QCoreApplication::translate("PropertyModuleClass", "\351\235\242\347\247\257 (m\302\262)", nullptr));
        QTableWidgetItem *___qtablewidgetitem13 = sectionTable->horizontalHeaderItem(5);
        ___qtablewidgetitem13->setText(QCoreApplication::translate("PropertyModuleClass", "\345\215\212\345\276\204 (m)", nullptr));
        QTableWidgetItem *___qtablewidgetitem14 = sectionTable->horizontalHeaderItem(6);
        ___qtablewidgetitem14->setText(QCoreApplication::translate("PropertyModuleClass", "\345\256\275\345\272\246 (m)", nullptr));
        QTableWidgetItem *___qtablewidgetitem15 = sectionTable->horizontalHeaderItem(7);
        ___qtablewidgetitem15->setText(QCoreApplication::translate("PropertyModuleClass", "\351\253\230\345\272\246 (m)", nullptr));
        refreshButton->setText(QCoreApplication::translate("PropertyModuleClass", "\345\210\267\346\226\260", nullptr));
        applyButton->setText(QCoreApplication::translate("PropertyModuleClass", "\345\272\224\347\224\250\344\277\256\346\224\271\345\210\260\345\257\271\345\272\224\346\250\241\345\236\213", nullptr));
        (void)PropertyModuleClass;
    } // retranslateUi

};

namespace Ui {
    class PropertyModuleClass: public Ui_PropertyModuleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROPERTYMODULE_H
