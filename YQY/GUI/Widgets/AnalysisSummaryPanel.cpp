#include "Widgets/AnalysisSummaryPanel.h"
#include "ui_AnalysisSummaryPanel.h"
#include <QPushButton>
AnalysisSummaryPanel::AnalysisSummaryPanel(QWidget* p) : QWidget(p), m_ui(new Ui::AnalysisSummaryPanelClass)
{
    m_ui->setupUi(this);
    const QList<QPushButton*> bs = {m_ui->stepsButton, m_ui->loadsButton, m_ui->constraintsButton};
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
AnalysisSummaryPanel::~AnalysisSummaryPanel()
{
    delete m_ui;
}
QPushButton* AnalysisSummaryPanel::stepsButton() const
{
    return m_ui->stepsButton;
}
QPushButton* AnalysisSummaryPanel::loadsButton() const
{
    return m_ui->loadsButton;
}
QPushButton* AnalysisSummaryPanel::constraintsButton() const
{
    return m_ui->constraintsButton;
}
