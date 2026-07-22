#include "Dialogs/SolveTaskManagerDialog.h"
#include "Widgets/TableAppearance.h"
#include "ui_SolveTaskManagerDialog.h"
#include <QDialogButtonBox>
#include <QHeaderView>
#include <QPushButton>
#include <QTableWidget>
SolveTaskManagerDialog::SolveTaskManagerDialog(QWidget* p) : QDialog(p), m_ui(new Ui::SolveTaskManagerDialogClass)
{
    m_ui->setupUi(this);
    setObjectName(QStringLiteral("solveTaskManagerDialog"));
    setAttribute(Qt::WA_DeleteOnClose);
    resize(920, 520);
    m_ui->hintLabel->setObjectName(QStringLiteral("solveTaskManagerHint"));
    m_ui->buttonBox->setObjectName(QStringLiteral("analysisManagerButtonBox"));
    m_ui->buttonBox->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
    m_ui->taskTable->verticalHeader()->setVisible(false);
    applyCenteredTableAppearance(m_ui->taskTable);
    m_ui->taskTable->verticalHeader()->setDefaultSectionSize(44);
    auto* h = m_ui->taskTable->horizontalHeader();
    h->setSectionResizeMode(0, QHeaderView::Fixed);
    h->setSectionResizeMode(1, QHeaderView::Stretch);
    for (int c = 2; c < 6; ++c)
        h->setSectionResizeMode(c, QHeaderView::Fixed);
    m_ui->taskTable->setColumnWidth(0, 106);
    m_ui->taskTable->setColumnWidth(2, 78);
    m_ui->taskTable->setColumnWidth(3, 230);
    m_ui->taskTable->setColumnWidth(4, 88);
    m_ui->taskTable->setColumnWidth(5, 90);
}
SolveTaskManagerDialog::~SolveTaskManagerDialog()
{
    delete m_ui;
}
QPushButton* SolveTaskManagerDialog::solveAllButton() const
{
    return m_ui->solveAllButton;
}
QPushButton* SolveTaskManagerDialog::restartAllButton() const
{
    return m_ui->restartAllButton;
}
QTableWidget* SolveTaskManagerDialog::taskTable() const
{
    return m_ui->taskTable;
}
