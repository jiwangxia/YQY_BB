#include "Widgets/PropertyModule.h"
#include <QVariant>
#include <QApplication>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>
#include "Widgets/TableAppearance.h"
#include <QTabBar>

PropertyModule::PropertyModule(QWidget* p)
    : QWidget(p)
{
    initializeUi();
    initializeAppearance();
}
PropertyModule::~PropertyModule() = default;
QTabWidget* PropertyModule::tabs() const
{
    return m_propertyTabs;
}
QTableWidget* PropertyModule::materialTable() const
{
    return m_materialTable;
}
QTableWidget* PropertyModule::sectionTable() const
{
    return m_sectionTable;
}
QTreeWidget* PropertyModule::materialTree() const
{
    return m_materialTree;
}
QTreeWidget* PropertyModule::sectionTree() const
{
    return m_sectionTree;
}

QTreeWidget* PropertyModule::springBehaviorTree() const
{
    return m_springBehaviorTree;
}

QPushButton* PropertyModule::refreshButton() const
{
    return m_refreshButton;
}
QPushButton* PropertyModule::applyButton() const
{
    return m_applyButton;
}

void PropertyModule::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("PropertyModuleClass");
    }
    m_rootLayout = new QVBoxLayout(this);
    m_rootLayout->setSpacing(8);
    m_rootLayout->setObjectName("rootLayout");
    m_rootLayout->setContentsMargins(8, 8, 8, 8);
    QLabel* descriptionLabel = new QLabel(this);
    descriptionLabel->setObjectName("descriptionLabel");
    descriptionLabel->setWordWrap(true);

    m_rootLayout->addWidget(descriptionLabel);

    m_propertyTabs = new QTabWidget(this);
    m_propertyTabs->setObjectName("propertyTabs");
    m_materialTree = new QTreeWidget();
    QTreeWidgetItem* __qtreewidgetitem = new QTreeWidgetItem();
    __qtreewidgetitem->setText(0, QString::fromUtf8("1"));
    m_materialTree->setHeaderItem(__qtreewidgetitem);
    m_materialTree->setObjectName("materialTree");
    m_materialTree->setHeaderHidden(true);
    m_materialTree->setAlternatingRowColors(true);
    m_materialTree->setUniformRowHeights(true);
    m_materialTree->setRootIsDecorated(true);
    m_materialTree->setIndentation(12);
    m_materialTree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_propertyTabs->addTab(m_materialTree, QString());
    m_sectionTree = new QTreeWidget();
    QTreeWidgetItem* __qtreewidgetitem1 = new QTreeWidgetItem();
    __qtreewidgetitem1->setText(0, QString::fromUtf8("1"));
    m_sectionTree->setHeaderItem(__qtreewidgetitem1);
    m_sectionTree->setObjectName("sectionTree");
    m_sectionTree->setHeaderHidden(true);
    m_sectionTree->setAlternatingRowColors(true);
    m_sectionTree->setUniformRowHeights(true);
    m_sectionTree->setRootIsDecorated(true);
    m_sectionTree->setIndentation(12);
    m_sectionTree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_propertyTabs->addTab(m_sectionTree, QString());
    m_springBehaviorTree = new QTreeWidget();
    QTreeWidgetItem* __qtreewidgetitem2 = new QTreeWidgetItem();
    __qtreewidgetitem2->setText(0, QString::fromUtf8("1"));
    m_springBehaviorTree->setHeaderItem(__qtreewidgetitem2);
    m_springBehaviorTree->setObjectName("springBehaviorTree");
    m_springBehaviorTree->setHeaderHidden(true);
    m_springBehaviorTree->setAlternatingRowColors(true);
    m_springBehaviorTree->setUniformRowHeights(true);
    m_springBehaviorTree->setRootIsDecorated(true);
    m_springBehaviorTree->setIndentation(12);
    m_springBehaviorTree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_propertyTabs->addTab(m_springBehaviorTree, QString());

    m_rootLayout->addWidget(m_propertyTabs);

    m_materialTable = new QTableWidget(this);
    if (m_materialTable->columnCount() < 8)
        m_materialTable->setColumnCount(8);
    QTableWidgetItem* __qtablewidgetitem = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
    QTableWidgetItem* __qtablewidgetitem1 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
    QTableWidgetItem* __qtablewidgetitem2 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
    QTableWidgetItem* __qtablewidgetitem3 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
    QTableWidgetItem* __qtablewidgetitem4 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
    QTableWidgetItem* __qtablewidgetitem5 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
    QTableWidgetItem* __qtablewidgetitem6 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(6, __qtablewidgetitem6);
    QTableWidgetItem* __qtablewidgetitem7 = new QTableWidgetItem();
    m_materialTable->setHorizontalHeaderItem(7, __qtablewidgetitem7);
    m_materialTable->setObjectName("materialTable");
    m_materialTable->setVisible(false);
    m_materialTable->setColumnCount(8);

    m_rootLayout->addWidget(m_materialTable);

    m_sectionTable = new QTableWidget(this);
    if (m_sectionTable->columnCount() < 8)
        m_sectionTable->setColumnCount(8);
    QTableWidgetItem* __qtablewidgetitem8 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(0, __qtablewidgetitem8);
    QTableWidgetItem* __qtablewidgetitem9 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(1, __qtablewidgetitem9);
    QTableWidgetItem* __qtablewidgetitem10 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(2, __qtablewidgetitem10);
    QTableWidgetItem* __qtablewidgetitem11 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(3, __qtablewidgetitem11);
    QTableWidgetItem* __qtablewidgetitem12 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(4, __qtablewidgetitem12);
    QTableWidgetItem* __qtablewidgetitem13 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(5, __qtablewidgetitem13);
    QTableWidgetItem* __qtablewidgetitem14 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(6, __qtablewidgetitem14);
    QTableWidgetItem* __qtablewidgetitem15 = new QTableWidgetItem();
    m_sectionTable->setHorizontalHeaderItem(7, __qtablewidgetitem15);
    m_sectionTable->setObjectName("sectionTable");
    m_sectionTable->setVisible(false);
    m_sectionTable->setColumnCount(8);

    m_rootLayout->addWidget(m_sectionTable);

    QVBoxLayout* buttonLayout = new QVBoxLayout();
    buttonLayout->setSpacing(6);
    buttonLayout->setObjectName("buttonLayout");
    m_refreshButton = new QPushButton(this);
    m_refreshButton->setObjectName("refreshButton");
    m_refreshButton->setMinimumSize(QSize(96, 0));

    buttonLayout->addWidget(m_refreshButton);

    m_applyButton = new QPushButton(this);
    m_applyButton->setObjectName("applyButton");
    m_applyButton->setMinimumSize(QSize(190, 0));
    m_applyButton->setVisible(false);

    buttonLayout->addWidget(m_applyButton);

    m_rootLayout->addLayout(buttonLayout);

    descriptionLabel->setText(QCoreApplication::translate(
        "PropertyModuleClass",
        "列出默认属性库和所有已加载模型的材料、截面、弹簧行为。双击条目可查看详情；材料与截面可直接修改。", nullptr));
    m_propertyTabs->setTabText(m_propertyTabs->indexOf(m_materialTree),
                               QCoreApplication::translate("PropertyModuleClass", "材料", nullptr));
    m_propertyTabs->setTabText(m_propertyTabs->indexOf(m_sectionTree),
                               QCoreApplication::translate("PropertyModuleClass", "截面", nullptr));
    m_propertyTabs->setTabText(m_propertyTabs->indexOf(m_springBehaviorTree),
                               QCoreApplication::translate("PropertyModuleClass", "弹簧行为", nullptr));
    QTableWidgetItem* ___qtablewidgetitem = m_materialTable->horizontalHeaderItem(0);
    ___qtablewidgetitem->setText(QCoreApplication::translate("PropertyModuleClass", "模型编号", nullptr));
    QTableWidgetItem* ___qtablewidgetitem1 = m_materialTable->horizontalHeaderItem(1);
    ___qtablewidgetitem1->setText(QCoreApplication::translate("PropertyModuleClass", "模型文件", nullptr));
    QTableWidgetItem* ___qtablewidgetitem2 = m_materialTable->horizontalHeaderItem(2);
    ___qtablewidgetitem2->setText(QCoreApplication::translate("PropertyModuleClass", "材料 ID", nullptr));
    QTableWidgetItem* ___qtablewidgetitem3 = m_materialTable->horizontalHeaderItem(3);
    ___qtablewidgetitem3->setText(QCoreApplication::translate("PropertyModuleClass", "E (Pa)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem4 = m_materialTable->horizontalHeaderItem(4);
    ___qtablewidgetitem4->setText(QCoreApplication::translate("PropertyModuleClass", "泊松比", nullptr));
    QTableWidgetItem* ___qtablewidgetitem5 = m_materialTable->horizontalHeaderItem(5);
    ___qtablewidgetitem5->setText(QCoreApplication::translate("PropertyModuleClass", "密度 (kg/m³)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem6 = m_materialTable->horizontalHeaderItem(6);
    ___qtablewidgetitem6->setText(QCoreApplication::translate("PropertyModuleClass", "许用应力 (Pa)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem7 = m_materialTable->horizontalHeaderItem(7);
    ___qtablewidgetitem7->setText(QCoreApplication::translate("PropertyModuleClass", "膨胀系数", nullptr));
    QTableWidgetItem* ___qtablewidgetitem8 = m_sectionTable->horizontalHeaderItem(0);
    ___qtablewidgetitem8->setText(QCoreApplication::translate("PropertyModuleClass", "模型编号", nullptr));
    QTableWidgetItem* ___qtablewidgetitem9 = m_sectionTable->horizontalHeaderItem(1);
    ___qtablewidgetitem9->setText(QCoreApplication::translate("PropertyModuleClass", "模型文件", nullptr));
    QTableWidgetItem* ___qtablewidgetitem10 = m_sectionTable->horizontalHeaderItem(2);
    ___qtablewidgetitem10->setText(QCoreApplication::translate("PropertyModuleClass", "截面 ID", nullptr));
    QTableWidgetItem* ___qtablewidgetitem11 = m_sectionTable->horizontalHeaderItem(3);
    ___qtablewidgetitem11->setText(QCoreApplication::translate("PropertyModuleClass", "类型", nullptr));
    QTableWidgetItem* ___qtablewidgetitem12 = m_sectionTable->horizontalHeaderItem(4);
    ___qtablewidgetitem12->setText(QCoreApplication::translate("PropertyModuleClass", "面积 (m²)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem13 = m_sectionTable->horizontalHeaderItem(5);
    ___qtablewidgetitem13->setText(QCoreApplication::translate("PropertyModuleClass", "半径 (m)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem14 = m_sectionTable->horizontalHeaderItem(6);
    ___qtablewidgetitem14->setText(QCoreApplication::translate("PropertyModuleClass", "宽度 (m)", nullptr));
    QTableWidgetItem* ___qtablewidgetitem15 = m_sectionTable->horizontalHeaderItem(7);
    ___qtablewidgetitem15->setText(QCoreApplication::translate("PropertyModuleClass", "高度 (m)", nullptr));
    m_refreshButton->setText(QCoreApplication::translate("PropertyModuleClass", "刷新", nullptr));
    m_applyButton->setText(QCoreApplication::translate("PropertyModuleClass", "应用修改到对应模型", nullptr));
}

void PropertyModule::initializeAppearance()
{
    setObjectName(QStringLiteral("propertyPage"));
    m_rootLayout->setStretch(1, 1);
    m_propertyTabs->tabBar()->setExpanding(false);
    m_propertyTabs->tabBar()->setUsesScrollButtons(true);
    for (auto* t : {m_materialTable, m_sectionTable})
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
    m_materialTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_sectionTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    m_materialTable->setColumnWidth(0, 82);
    m_materialTable->setColumnWidth(2, 82);
    m_sectionTable->setColumnWidth(0, 82);
    m_sectionTable->setColumnWidth(2, 82);
    for (int c = 3; c < 8; ++c)
    {
        m_materialTable->setColumnWidth(c, 132);
        m_sectionTable->setColumnWidth(c, 116);
    }
    m_materialTree->setTextElideMode(Qt::ElideMiddle);
    m_sectionTree->setTextElideMode(Qt::ElideMiddle);
}
