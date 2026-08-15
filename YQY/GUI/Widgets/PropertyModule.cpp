#include "Widgets/PropertyModule.h"
#include "Widgets/TableAppearance.h"
#include "ui_PropertyModule.h"
#include <QHeaderView>
#include <QTabBar>
#include <QTableWidget>
#include <QTreeWidget>
PropertyModule::PropertyModule(QWidget* p)
    : QWidget(p)
    , m_ui(new Ui::PropertyModuleClass)
{
    m_ui->setupUi(this);
    setObjectName(QStringLiteral("propertyPage"));
    m_ui->propertyTabs->tabBar()->setExpanding(false);
    m_ui->propertyTabs->tabBar()->setUsesScrollButtons(true);
    for (auto* t : {m_ui->materialTable, m_ui->sectionTable})
    {
        applyCenteredTableAppearance(t);
        t->setAlternatingRowColors(true);
        t->setSelectionBehavior(QAbstractItemView::SelectRows);
        t->setSelectionMode(QAbstractItemView::SingleSelection);
        t->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
        t->setShowGrid(false);
        t->setWordWrap(false);
        t->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
        t->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
        t->horizontalHeader()->setMinimumSectionSize(72);
        t->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
        t->verticalHeader()->setDefaultSectionSize(38);
        t->verticalHeader()->setVisible(false);
    }
    m_ui->materialTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_ui->sectionTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_ui->materialTable->setColumnWidth(0, 82);
    m_ui->materialTable->setColumnWidth(2, 82);
    m_ui->sectionTable->setColumnWidth(0, 82);
    m_ui->sectionTable->setColumnWidth(2, 82);
    for (int c = 3; c < 8; ++c)
    {
        m_ui->materialTable->setColumnWidth(c, 132);
        m_ui->sectionTable->setColumnWidth(c, 116);
    }
    m_ui->materialTree->setTextElideMode(Qt::ElideMiddle);
    m_ui->sectionTree->setTextElideMode(Qt::ElideMiddle);
}
PropertyModule::~PropertyModule()
{
    delete m_ui;
}
QTabWidget* PropertyModule::tabs() const
{
    return m_ui->propertyTabs;
}
QTableWidget* PropertyModule::materialTable() const
{
    return m_ui->materialTable;
}
QTableWidget* PropertyModule::sectionTable() const
{
    return m_ui->sectionTable;
}
QTreeWidget* PropertyModule::materialTree() const
{
    return m_ui->materialTree;
}
QTreeWidget* PropertyModule::sectionTree() const
{
    return m_ui->sectionTree;
}
QPushButton* PropertyModule::refreshButton() const
{
    return m_ui->refreshButton;
}
QPushButton* PropertyModule::applyButton() const
{
    return m_ui->applyButton;
}
