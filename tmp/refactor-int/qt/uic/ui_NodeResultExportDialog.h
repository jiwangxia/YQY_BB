/********************************************************************************
** Form generated from reading UI file 'NodeResultExportDialog.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NODERESULTEXPORTDIALOG_H
#define UI_NODERESULTEXPORTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAbstractButton>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_NodeResultExportDialogClass
{
public:
    QVBoxLayout *rootLayout;
    QLabel *titleLabel;
    QLabel *subtitleLabel;
    QFrame *nodeCard;
    QVBoxLayout *nodeCardLayout;
    QLabel *nodeTitle;
    QHBoxLayout *nodeRow;
    QLineEdit *nodeEdit;
    QPushButton *currentNodeButton;
    QLabel *nodeHint;
    QFrame *fieldCard;
    QVBoxLayout *fieldCardLayout;
    QLabel *fieldTitle;
    QGridLayout *fieldGrid;
    QGroupBox *displacementGroup;
    QHBoxLayout *hboxLayout;
    QCheckBox *u1;
    QCheckBox *u2;
    QCheckBox *u3;
    QCheckBox *um;
    QGroupBox *rotationGroup;
    QHBoxLayout *hboxLayout1;
    QCheckBox *ur1;
    QCheckBox *ur2;
    QCheckBox *ur3;
    QGroupBox *velocityGroup;
    QHBoxLayout *hboxLayout2;
    QCheckBox *v1;
    QCheckBox *v2;
    QCheckBox *v3;
    QGroupBox *accelerationGroup;
    QHBoxLayout *hboxLayout3;
    QCheckBox *a1;
    QCheckBox *a2;
    QCheckBox *a3;
    QGroupBox *coordinateGroup;
    QHBoxLayout *hboxLayout4;
    QCheckBox *cx;
    QCheckBox *cy;
    QCheckBox *cz;
    QGroupBox *reactionGroup;
    QHBoxLayout *hboxLayout5;
    QCheckBox *r1;
    QCheckBox *r2;
    QCheckBox *r3;
    QFrame *fileCard;
    QVBoxLayout *fileCardLayout;
    QLabel *fileTitle;
    QHBoxLayout *fileRow;
    QLineEdit *fileEdit;
    QPushButton *browseButton;
    QLabel *errorLabel;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *NodeResultExportDialogClass)
    {
        if (NodeResultExportDialogClass->objectName().isEmpty())
            NodeResultExportDialogClass->setObjectName("NodeResultExportDialogClass");
        NodeResultExportDialogClass->setMinimumSize(QSize(660, 610));
        rootLayout = new QVBoxLayout(NodeResultExportDialogClass);
        rootLayout->setSpacing(14);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(22, 20, 22, 20);
        titleLabel = new QLabel(NodeResultExportDialogClass);
        titleLabel->setObjectName("titleLabel");

        rootLayout->addWidget(titleLabel);

        subtitleLabel = new QLabel(NodeResultExportDialogClass);
        subtitleLabel->setObjectName("subtitleLabel");

        rootLayout->addWidget(subtitleLabel);

        nodeCard = new QFrame(NodeResultExportDialogClass);
        nodeCard->setObjectName("nodeCard");
        nodeCardLayout = new QVBoxLayout(nodeCard);
        nodeCardLayout->setSpacing(9);
        nodeCardLayout->setObjectName("nodeCardLayout");
        nodeCardLayout->setContentsMargins(14, 12, 14, 12);
        nodeTitle = new QLabel(nodeCard);
        nodeTitle->setObjectName("nodeTitle");

        nodeCardLayout->addWidget(nodeTitle);

        nodeRow = new QHBoxLayout();
        nodeRow->setSpacing(8);
        nodeRow->setObjectName("nodeRow");
        nodeEdit = new QLineEdit(nodeCard);
        nodeEdit->setObjectName("nodeEdit");
        nodeEdit->setClearButtonEnabled(true);

        nodeRow->addWidget(nodeEdit);

        currentNodeButton = new QPushButton(nodeCard);
        currentNodeButton->setObjectName("currentNodeButton");
        currentNodeButton->setVisible(false);

        nodeRow->addWidget(currentNodeButton);


        nodeCardLayout->addLayout(nodeRow);

        nodeHint = new QLabel(nodeCard);
        nodeHint->setObjectName("nodeHint");

        nodeCardLayout->addWidget(nodeHint);


        rootLayout->addWidget(nodeCard);

        fieldCard = new QFrame(NodeResultExportDialogClass);
        fieldCard->setObjectName("fieldCard");
        fieldCardLayout = new QVBoxLayout(fieldCard);
        fieldCardLayout->setSpacing(9);
        fieldCardLayout->setObjectName("fieldCardLayout");
        fieldCardLayout->setContentsMargins(14, 12, 14, 12);
        fieldTitle = new QLabel(fieldCard);
        fieldTitle->setObjectName("fieldTitle");

        fieldCardLayout->addWidget(fieldTitle);

        fieldGrid = new QGridLayout();
        fieldGrid->setObjectName("fieldGrid");
        fieldGrid->setHorizontalSpacing(10);
        fieldGrid->setVerticalSpacing(10);
        displacementGroup = new QGroupBox(fieldCard);
        displacementGroup->setObjectName("displacementGroup");
        hboxLayout = new QHBoxLayout(displacementGroup);
        hboxLayout->setObjectName("hboxLayout");
        u1 = new QCheckBox(displacementGroup);
        u1->setObjectName("u1");
        u1->setChecked(true);

        hboxLayout->addWidget(u1);

        u2 = new QCheckBox(displacementGroup);
        u2->setObjectName("u2");
        u2->setChecked(true);

        hboxLayout->addWidget(u2);

        u3 = new QCheckBox(displacementGroup);
        u3->setObjectName("u3");
        u3->setChecked(true);

        hboxLayout->addWidget(u3);

        um = new QCheckBox(displacementGroup);
        um->setObjectName("um");
        um->setChecked(false);

        hboxLayout->addWidget(um);


        fieldGrid->addWidget(displacementGroup, 0, 0, 1, 1);

        rotationGroup = new QGroupBox(fieldCard);
        rotationGroup->setObjectName("rotationGroup");
        hboxLayout1 = new QHBoxLayout(rotationGroup);
        hboxLayout1->setObjectName("hboxLayout1");
        ur1 = new QCheckBox(rotationGroup);
        ur1->setObjectName("ur1");

        hboxLayout1->addWidget(ur1);

        ur2 = new QCheckBox(rotationGroup);
        ur2->setObjectName("ur2");

        hboxLayout1->addWidget(ur2);

        ur3 = new QCheckBox(rotationGroup);
        ur3->setObjectName("ur3");

        hboxLayout1->addWidget(ur3);


        fieldGrid->addWidget(rotationGroup, 0, 1, 1, 1);

        velocityGroup = new QGroupBox(fieldCard);
        velocityGroup->setObjectName("velocityGroup");
        hboxLayout2 = new QHBoxLayout(velocityGroup);
        hboxLayout2->setObjectName("hboxLayout2");
        v1 = new QCheckBox(velocityGroup);
        v1->setObjectName("v1");

        hboxLayout2->addWidget(v1);

        v2 = new QCheckBox(velocityGroup);
        v2->setObjectName("v2");

        hboxLayout2->addWidget(v2);

        v3 = new QCheckBox(velocityGroup);
        v3->setObjectName("v3");

        hboxLayout2->addWidget(v3);


        fieldGrid->addWidget(velocityGroup, 1, 0, 1, 1);

        accelerationGroup = new QGroupBox(fieldCard);
        accelerationGroup->setObjectName("accelerationGroup");
        hboxLayout3 = new QHBoxLayout(accelerationGroup);
        hboxLayout3->setObjectName("hboxLayout3");
        a1 = new QCheckBox(accelerationGroup);
        a1->setObjectName("a1");

        hboxLayout3->addWidget(a1);

        a2 = new QCheckBox(accelerationGroup);
        a2->setObjectName("a2");

        hboxLayout3->addWidget(a2);

        a3 = new QCheckBox(accelerationGroup);
        a3->setObjectName("a3");

        hboxLayout3->addWidget(a3);


        fieldGrid->addWidget(accelerationGroup, 1, 1, 1, 1);

        coordinateGroup = new QGroupBox(fieldCard);
        coordinateGroup->setObjectName("coordinateGroup");
        hboxLayout4 = new QHBoxLayout(coordinateGroup);
        hboxLayout4->setObjectName("hboxLayout4");
        cx = new QCheckBox(coordinateGroup);
        cx->setObjectName("cx");

        hboxLayout4->addWidget(cx);

        cy = new QCheckBox(coordinateGroup);
        cy->setObjectName("cy");

        hboxLayout4->addWidget(cy);

        cz = new QCheckBox(coordinateGroup);
        cz->setObjectName("cz");

        hboxLayout4->addWidget(cz);


        fieldGrid->addWidget(coordinateGroup, 2, 0, 1, 1);

        reactionGroup = new QGroupBox(fieldCard);
        reactionGroup->setObjectName("reactionGroup");
        hboxLayout5 = new QHBoxLayout(reactionGroup);
        hboxLayout5->setObjectName("hboxLayout5");
        r1 = new QCheckBox(reactionGroup);
        r1->setObjectName("r1");

        hboxLayout5->addWidget(r1);

        r2 = new QCheckBox(reactionGroup);
        r2->setObjectName("r2");

        hboxLayout5->addWidget(r2);

        r3 = new QCheckBox(reactionGroup);
        r3->setObjectName("r3");

        hboxLayout5->addWidget(r3);


        fieldGrid->addWidget(reactionGroup, 2, 1, 1, 1);


        fieldCardLayout->addLayout(fieldGrid);


        rootLayout->addWidget(fieldCard);

        fileCard = new QFrame(NodeResultExportDialogClass);
        fileCard->setObjectName("fileCard");
        fileCardLayout = new QVBoxLayout(fileCard);
        fileCardLayout->setSpacing(9);
        fileCardLayout->setObjectName("fileCardLayout");
        fileCardLayout->setContentsMargins(14, 12, 14, 12);
        fileTitle = new QLabel(fileCard);
        fileTitle->setObjectName("fileTitle");

        fileCardLayout->addWidget(fileTitle);

        fileRow = new QHBoxLayout();
        fileRow->setSpacing(8);
        fileRow->setObjectName("fileRow");
        fileEdit = new QLineEdit(fileCard);
        fileEdit->setObjectName("fileEdit");

        fileRow->addWidget(fileEdit);

        browseButton = new QPushButton(fileCard);
        browseButton->setObjectName("browseButton");

        fileRow->addWidget(browseButton);


        fileCardLayout->addLayout(fileRow);


        rootLayout->addWidget(fileCard);

        errorLabel = new QLabel(NodeResultExportDialogClass);
        errorLabel->setObjectName("errorLabel");
        errorLabel->setMinimumSize(QSize(0, 20));

        rootLayout->addWidget(errorLabel);

        buttonBox = new QDialogButtonBox(NodeResultExportDialogClass);
        buttonBox->setObjectName("buttonBox");
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Save);

        rootLayout->addWidget(buttonBox);


        retranslateUi(NodeResultExportDialogClass);

        QMetaObject::connectSlotsByName(NodeResultExportDialogClass);
    } // setupUi

    void retranslateUi(QDialog *NodeResultExportDialogClass)
    {
        NodeResultExportDialogClass->setWindowTitle(QCoreApplication::translate("NodeResultExportDialogClass", "\345\257\274\345\207\272\350\212\202\347\202\271\347\273\223\346\236\234", nullptr));
        titleLabel->setText(QCoreApplication::translate("NodeResultExportDialogClass", "\345\257\274\345\207\272\350\212\202\347\202\271\346\227\266\347\250\213\346\225\260\346\215\256", nullptr));
        subtitleLabel->setText(QCoreApplication::translate("NodeResultExportDialogClass", "\344\273\216\345\275\223\345\211\215 H5 \347\273\223\346\236\234\344\270\255\351\200\211\346\213\251\350\212\202\347\202\271\345\222\214\347\273\223\346\236\234\345\210\206\351\207\217", nullptr));
        nodeTitle->setText(QCoreApplication::translate("NodeResultExportDialogClass", "1  \351\200\211\346\213\251\350\212\202\347\202\271", nullptr));
        nodeEdit->setPlaceholderText(QCoreApplication::translate("NodeResultExportDialogClass", "\344\276\213\345\246\202\357\274\2321, 3, 8-12", nullptr));
        currentNodeButton->setText(QCoreApplication::translate("NodeResultExportDialogClass", "\345\275\223\345\211\215\350\212\202\347\202\271", nullptr));
        nodeHint->setText(QCoreApplication::translate("NodeResultExportDialogClass", "\346\224\257\346\214\201\351\200\227\345\217\267\343\200\201\347\251\272\346\240\274\345\222\214\350\277\236\347\273\255\350\214\203\345\233\264\357\274\233\345\257\274\345\207\272\345\211\215\344\274\232\346\243\200\346\237\245\350\212\202\347\202\271\346\230\257\345\220\246\345\255\230\345\234\250\343\200\202", nullptr));
        fieldTitle->setText(QCoreApplication::translate("NodeResultExportDialogClass", "2  \351\200\211\346\213\251\346\225\260\346\215\256", nullptr));
        displacementGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "\344\275\215\347\247\273", nullptr));
        u1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "U1", nullptr));
        u2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "U2", nullptr));
        u3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "U3", nullptr));
        um->setText(QCoreApplication::translate("NodeResultExportDialogClass", "|U|", nullptr));
        rotationGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "\350\275\254\350\247\222", nullptr));
        ur1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "UR1", nullptr));
        ur2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "UR2", nullptr));
        ur3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "UR3", nullptr));
        velocityGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "\351\200\237\345\272\246", nullptr));
        v1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "V1", nullptr));
        v2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "V2", nullptr));
        v3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "V3", nullptr));
        accelerationGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "\345\212\240\351\200\237\345\272\246", nullptr));
        a1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "A1", nullptr));
        a2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "A2", nullptr));
        a3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "A3", nullptr));
        coordinateGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "\345\275\223\345\211\215\345\235\220\346\240\207", nullptr));
        cx->setText(QCoreApplication::translate("NodeResultExportDialogClass", "CX", nullptr));
        cy->setText(QCoreApplication::translate("NodeResultExportDialogClass", "CY", nullptr));
        cz->setText(QCoreApplication::translate("NodeResultExportDialogClass", "CZ", nullptr));
        reactionGroup->setTitle(QCoreApplication::translate("NodeResultExportDialogClass", "\350\212\202\347\202\271\345\217\215\345\212\233", nullptr));
        r1->setText(QCoreApplication::translate("NodeResultExportDialogClass", "R1", nullptr));
        r2->setText(QCoreApplication::translate("NodeResultExportDialogClass", "R2", nullptr));
        r3->setText(QCoreApplication::translate("NodeResultExportDialogClass", "R3", nullptr));
        fileTitle->setText(QCoreApplication::translate("NodeResultExportDialogClass", "3  \344\277\235\345\255\230\346\226\207\344\273\266", nullptr));
        browseButton->setText(QCoreApplication::translate("NodeResultExportDialogClass", "\346\265\217\350\247\210\342\200\246", nullptr));
    } // retranslateUi

};

namespace Ui {
    class NodeResultExportDialogClass: public Ui_NodeResultExportDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NODERESULTEXPORTDIALOG_H
