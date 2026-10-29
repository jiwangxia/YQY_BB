#include "Widgets/AnalysisSummaryPanel.h"
#include <QVariant>
#include <QApplication>
#include <QFrame>
#include <QGridLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

AnalysisSummaryPanel::AnalysisSummaryPanel(QWidget* p)
    : QWidget(p)
{
    initializeUi();
    initializeAppearance();
}
AnalysisSummaryPanel::~AnalysisSummaryPanel() = default;
QPushButton* AnalysisSummaryPanel::stepsButton() const
{
    return m_stepsButton;
}
QPushButton* AnalysisSummaryPanel::loadsButton() const
{
    return m_loadsButton;
}
QPushButton* AnalysisSummaryPanel::constraintsButton() const
{
    return m_constraintsButton;
}

QPushButton* AnalysisSummaryPanel::mpcsButton() const
{
    return m_mpcsButton;
}

QPushButton* AnalysisSummaryPanel::regionsButton() const
{
    return m_regionsButton;
}

void AnalysisSummaryPanel::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("AnalysisSummaryPanelClass");
    }
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(0, 0, 0, 0);
    QFrame* analysisParameterCard = new QFrame(this);
    analysisParameterCard->setObjectName("analysisParameterCard");
    QGridLayout* actionsLayout = new QGridLayout(analysisParameterCard);
    actionsLayout->setObjectName("actionsLayout");
    actionsLayout->setHorizontalSpacing(8);
    actionsLayout->setVerticalSpacing(8);
    actionsLayout->setContentsMargins(10, 10, 10, 10);
    m_stepsButton = new QPushButton(analysisParameterCard);
    m_stepsButton->setObjectName("stepsButton");
    m_stepsButton->setMinimumSize(QSize(116, 54));

    actionsLayout->addWidget(m_stepsButton, 0, 0, 1, 1);

    m_loadsButton = new QPushButton(analysisParameterCard);
    m_loadsButton->setObjectName("loadsButton");
    m_loadsButton->setMinimumSize(QSize(116, 54));

    actionsLayout->addWidget(m_loadsButton, 0, 1, 1, 1);

    m_constraintsButton = new QPushButton(analysisParameterCard);
    m_constraintsButton->setObjectName("constraintsButton");
    m_constraintsButton->setMinimumSize(QSize(116, 54));

    actionsLayout->addWidget(m_constraintsButton, 1, 0, 1, 1);

    m_mpcsButton = new QPushButton(analysisParameterCard);
    m_mpcsButton->setObjectName("mpcsButton");
    m_mpcsButton->setMinimumSize(QSize(116, 54));

    actionsLayout->addWidget(m_mpcsButton, 1, 1, 1, 1);

    m_regionsButton = new QPushButton(analysisParameterCard);
    m_regionsButton->setObjectName("regionsButton");
    m_regionsButton->setMinimumSize(QSize(116, 54));

    actionsLayout->addWidget(m_regionsButton, 2, 0, 1, 2);

    rootLayout->addWidget(analysisParameterCard);

    m_stepsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "分析步", nullptr));
    m_loadsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "荷载", nullptr));
    m_constraintsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "约束", nullptr));
    m_mpcsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "MPC", nullptr));
    m_regionsButton->setText(QCoreApplication::translate("AnalysisSummaryPanelClass", "计算区域", nullptr));
}

void AnalysisSummaryPanel::initializeAppearance()
{
    const QList<QPushButton*> bs = {m_stepsButton, m_loadsButton, m_constraintsButton, m_mpcsButton, m_regionsButton};
    for (auto* b : bs)
    {
        b->setIconSize(QSize(22, 22));
        QFont f = b->font();
        f.setPointSizeF(qMax(11.0, f.pointSizeF()));
        f.setWeight(QFont::DemiBold);
        b->setFont(f);
        b->setCursor(Qt::PointingHandCursor);
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }
}
