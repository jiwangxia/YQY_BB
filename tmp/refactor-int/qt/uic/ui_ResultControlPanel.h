/********************************************************************************
** Form generated from reading UI file 'ResultControlPanel.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RESULTCONTROLPANEL_H
#define UI_RESULTCONTROLPANEL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ResultControlPanelClass
{
public:
    QVBoxLayout *rootLayout;
    QFrame *analysisParameterCard;
    QVBoxLayout *cardLayout;
    QHBoxLayout *fieldRow;
    QLabel *fieldLabel;
    QComboBox *fieldCombo;
    QHBoxLayout *scaleRow;
    QLabel *scaleLabel;
    QDoubleSpinBox *scaleSpin;
    QCheckBox *originalCheck;
    QHBoxLayout *frameRow;
    QPushButton *playButton;
    QSlider *frameSlider;
    QLabel *speedLabel;
    QComboBox *speedCombo;
    QGridLayout *resultFrameStatusLayout;
    QLabel *frameCaptionLabel;
    QLabel *frameLabel;
    QLabel *timeCaptionLabel;
    QLabel *timeValueLabel;
    QLabel *deformationCaptionLabel;
    QLabel *deformationValueLabel;
    QHBoxLayout *exportRow;
    QPushButton *exportButton;
    QPushButton *exportElementButton;
    QPushButton *exportIterationButton;

    void setupUi(QWidget *ResultControlPanelClass)
    {
        if (ResultControlPanelClass->objectName().isEmpty())
            ResultControlPanelClass->setObjectName("ResultControlPanelClass");
        rootLayout = new QVBoxLayout(ResultControlPanelClass);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(0, 0, 0, 0);
        analysisParameterCard = new QFrame(ResultControlPanelClass);
        analysisParameterCard->setObjectName("analysisParameterCard");
        cardLayout = new QVBoxLayout(analysisParameterCard);
        cardLayout->setSpacing(8);
        cardLayout->setObjectName("cardLayout");
        cardLayout->setContentsMargins(10, 10, 10, 10);
        fieldRow = new QHBoxLayout();
        fieldRow->setObjectName("fieldRow");
        fieldLabel = new QLabel(analysisParameterCard);
        fieldLabel->setObjectName("fieldLabel");

        fieldRow->addWidget(fieldLabel);

        fieldCombo = new QComboBox(analysisParameterCard);
        fieldCombo->addItem(QString());
        fieldCombo->addItem(QString());
        fieldCombo->addItem(QString());
        fieldCombo->addItem(QString());
        fieldCombo->addItem(QString());
        fieldCombo->addItem(QString());
        fieldCombo->addItem(QString());
        fieldCombo->setObjectName("fieldCombo");

        fieldRow->addWidget(fieldCombo);


        cardLayout->addLayout(fieldRow);

        scaleRow = new QHBoxLayout();
        scaleRow->setObjectName("scaleRow");
        scaleLabel = new QLabel(analysisParameterCard);
        scaleLabel->setObjectName("scaleLabel");

        scaleRow->addWidget(scaleLabel);

        scaleSpin = new QDoubleSpinBox(analysisParameterCard);
        scaleSpin->setObjectName("scaleSpin");
        scaleSpin->setMaximum(1000000.000000000000000);
        scaleSpin->setDecimals(4);

        scaleRow->addWidget(scaleSpin);

        originalCheck = new QCheckBox(analysisParameterCard);
        originalCheck->setObjectName("originalCheck");
        originalCheck->setChecked(true);

        scaleRow->addWidget(originalCheck);


        cardLayout->addLayout(scaleRow);

        frameRow = new QHBoxLayout();
        frameRow->setObjectName("frameRow");
        playButton = new QPushButton(analysisParameterCard);
        playButton->setObjectName("playButton");
        playButton->setEnabled(false);

        frameRow->addWidget(playButton);

        frameSlider = new QSlider(analysisParameterCard);
        frameSlider->setObjectName("frameSlider");
        frameSlider->setOrientation(Qt::Horizontal);
        frameSlider->setEnabled(false);

        frameRow->addWidget(frameSlider);

        speedLabel = new QLabel(analysisParameterCard);
        speedLabel->setObjectName("speedLabel");

        frameRow->addWidget(speedLabel);

        speedCombo = new QComboBox(analysisParameterCard);
        speedCombo->addItem(QString());
        speedCombo->addItem(QString());
        speedCombo->addItem(QString());
        speedCombo->addItem(QString());
        speedCombo->addItem(QString());
        speedCombo->addItem(QString());
        speedCombo->setObjectName("speedCombo");
        speedCombo->setMinimumSize(QSize(70, 0));

        frameRow->addWidget(speedCombo);


        cardLayout->addLayout(frameRow);

        resultFrameStatusLayout = new QGridLayout();
        resultFrameStatusLayout->setObjectName("resultFrameStatusLayout");
        resultFrameStatusLayout->setHorizontalSpacing(12);
        resultFrameStatusLayout->setVerticalSpacing(4);
        frameCaptionLabel = new QLabel(analysisParameterCard);
        frameCaptionLabel->setObjectName("frameCaptionLabel");
        frameCaptionLabel->setMinimumSize(QSize(72, 22));

        resultFrameStatusLayout->addWidget(frameCaptionLabel, 0, 0, 1, 1);

        frameLabel = new QLabel(analysisParameterCard);
        frameLabel->setObjectName("frameLabel");
        frameLabel->setMinimumSize(QSize(120, 22));

        resultFrameStatusLayout->addWidget(frameLabel, 0, 1, 1, 1);

        timeCaptionLabel = new QLabel(analysisParameterCard);
        timeCaptionLabel->setObjectName("timeCaptionLabel");
        timeCaptionLabel->setMinimumSize(QSize(72, 22));

        resultFrameStatusLayout->addWidget(timeCaptionLabel, 1, 0, 1, 1);

        timeValueLabel = new QLabel(analysisParameterCard);
        timeValueLabel->setObjectName("timeValueLabel");
        timeValueLabel->setMinimumSize(QSize(120, 22));

        resultFrameStatusLayout->addWidget(timeValueLabel, 1, 1, 1, 1);

        deformationCaptionLabel = new QLabel(analysisParameterCard);
        deformationCaptionLabel->setObjectName("deformationCaptionLabel");
        deformationCaptionLabel->setMinimumSize(QSize(72, 22));

        resultFrameStatusLayout->addWidget(deformationCaptionLabel, 2, 0, 1, 1);

        deformationValueLabel = new QLabel(analysisParameterCard);
        deformationValueLabel->setObjectName("deformationValueLabel");
        deformationValueLabel->setMinimumSize(QSize(120, 22));

        resultFrameStatusLayout->addWidget(deformationValueLabel, 2, 1, 1, 1);

        resultFrameStatusLayout->setColumnStretch(1, 1);

        cardLayout->addLayout(resultFrameStatusLayout);

        exportRow = new QHBoxLayout();
        exportRow->setObjectName("exportRow");
        exportButton = new QPushButton(analysisParameterCard);
        exportButton->setObjectName("exportButton");
        exportButton->setEnabled(false);

        exportRow->addWidget(exportButton);

        exportElementButton = new QPushButton(analysisParameterCard);
        exportElementButton->setObjectName("exportElementButton");
        exportElementButton->setEnabled(false);

        exportRow->addWidget(exportElementButton);


        cardLayout->addLayout(exportRow);

        exportIterationButton = new QPushButton(analysisParameterCard);
        exportIterationButton->setObjectName("exportIterationButton");
        exportIterationButton->setEnabled(false);

        cardLayout->addWidget(exportIterationButton);


        rootLayout->addWidget(analysisParameterCard);


        retranslateUi(ResultControlPanelClass);

        speedCombo->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(ResultControlPanelClass);
    } // setupUi

    void retranslateUi(QWidget *ResultControlPanelClass)
    {
        fieldLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "\344\272\221\345\233\276", nullptr));
        fieldCombo->setItemText(0, QCoreApplication::translate("ResultControlPanelClass", "\344\275\215\347\247\273\345\220\210\351\207\217 |U|", nullptr));
        fieldCombo->setItemText(1, QCoreApplication::translate("ResultControlPanelClass", "X \345\220\221\344\275\215\347\247\273 U-X", nullptr));
        fieldCombo->setItemText(2, QCoreApplication::translate("ResultControlPanelClass", "Y \345\220\221\344\275\215\347\247\273 U-Y", nullptr));
        fieldCombo->setItemText(3, QCoreApplication::translate("ResultControlPanelClass", "Z \345\220\221\344\275\215\347\247\273 U-Z", nullptr));
        fieldCombo->setItemText(4, QCoreApplication::translate("ResultControlPanelClass", "\350\275\264\345\212\233", nullptr));
        fieldCombo->setItemText(5, QCoreApplication::translate("ResultControlPanelClass", "\345\272\224\345\212\233", nullptr));
        fieldCombo->setItemText(6, QCoreApplication::translate("ResultControlPanelClass", "\345\272\224\345\217\230", nullptr));

        scaleLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "\345\217\230\345\275\242\346\257\224\344\276\213", nullptr));
        scaleSpin->setSpecialValueText(QCoreApplication::translate("ResultControlPanelClass", "\350\207\252\345\212\250", nullptr));
        originalCheck->setText(QCoreApplication::translate("ResultControlPanelClass", "\345\217\240\345\212\240\345\216\237\346\250\241\345\236\213", nullptr));
        playButton->setText(QCoreApplication::translate("ResultControlPanelClass", "\346\222\255\346\224\276", nullptr));
        speedLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "\351\200\237\345\272\246", nullptr));
        speedCombo->setItemText(0, QCoreApplication::translate("ResultControlPanelClass", "0.25\303\227", nullptr));
        speedCombo->setItemText(1, QCoreApplication::translate("ResultControlPanelClass", "0.5\303\227", nullptr));
        speedCombo->setItemText(2, QCoreApplication::translate("ResultControlPanelClass", "1\303\227", nullptr));
        speedCombo->setItemText(3, QCoreApplication::translate("ResultControlPanelClass", "2\303\227", nullptr));
        speedCombo->setItemText(4, QCoreApplication::translate("ResultControlPanelClass", "4\303\227", nullptr));
        speedCombo->setItemText(5, QCoreApplication::translate("ResultControlPanelClass", "8\303\227", nullptr));

#if QT_CONFIG(tooltip)
        speedCombo->setToolTip(QCoreApplication::translate("ResultControlPanelClass", "\347\273\223\346\236\234\345\212\250\347\224\273\346\222\255\346\224\276\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        frameCaptionLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "\345\270\247", nullptr));
        frameLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "\345\260\232\346\234\252\345\212\240\350\275\275\347\273\223\346\236\234\345\270\247", nullptr));
        timeCaptionLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "\346\227\266\351\227\264", nullptr));
        timeValueLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "--", nullptr));
        deformationCaptionLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "\345\217\230\345\275\242\346\257\224\344\276\213", nullptr));
        deformationValueLabel->setText(QCoreApplication::translate("ResultControlPanelClass", "--", nullptr));
        exportButton->setText(QCoreApplication::translate("ResultControlPanelClass", "\345\257\274\345\207\272\350\212\202\347\202\271\346\225\260\346\215\256", nullptr));
        exportElementButton->setText(QCoreApplication::translate("ResultControlPanelClass", "\345\257\274\345\207\272\345\215\225\345\205\203\346\225\260\346\215\256", nullptr));
        exportIterationButton->setText(QCoreApplication::translate("ResultControlPanelClass", "\345\257\274\345\207\272\350\277\255\344\273\243\346\255\245", nullptr));
        (void)ResultControlPanelClass;
    } // retranslateUi

};

namespace Ui {
    class ResultControlPanelClass: public Ui_ResultControlPanelClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RESULTCONTROLPANEL_H
