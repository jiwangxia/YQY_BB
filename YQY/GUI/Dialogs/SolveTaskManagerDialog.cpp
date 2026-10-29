#include "Dialogs/SolveTaskManagerDialog.h"
#include "Widgets/DialogSizing.h"
#include <QVariant>
#include <QAbstractButton>
#include <QApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSpacerItem>
#include <QTableWidget>
#include <QVBoxLayout>
#include "Utility/UiIcons.h"
#include "Widgets/TableAppearance.h"
#include <QStyledItemDelegate>

SolveTaskManagerDialog::SolveTaskManagerDialog(QWidget* p)
    : QDialog(p)
{
    initializeUi();
    setObjectName(QStringLiteral("solveTaskManagerDialog"));
    setWindowIcon(UiIcons::applicationIcon());
    setAttribute(Qt::WA_DeleteOnClose);
    resize(980, 520);
    DialogSizing::lockCurrentHeight(this);
    m_hintLabel->setObjectName(QStringLiteral("solveTaskManagerHint"));
    m_hintLabel->setProperty("secondaryText", true);
    m_solveAllButton->setProperty("primaryAction", true);
    m_buttonBox->setObjectName(QStringLiteral("analysisManagerButtonBox"));
    m_buttonBox->button(QDialogButtonBox::Close)->setText(QStringLiteral("关闭"));
    m_buttonBox->button(QDialogButtonBox::Close)->setAutoDefault(false);
    m_taskTable->verticalHeader()->setVisible(false);
    applyCenteredTableAppearance(m_taskTable);
    m_taskTable->setItemDelegateForColumn(1, new QStyledItemDelegate(m_taskTable));
    m_taskTable->verticalHeader()->setDefaultSectionSize(44);
    auto* h = m_taskTable->horizontalHeader();
    h->setSectionResizeMode(0, QHeaderView::Fixed);
    h->setSectionResizeMode(1, QHeaderView::Stretch);
    for (int c = 2; c < 6; ++c)
        h->setSectionResizeMode(c, QHeaderView::Fixed);
    m_taskTable->setColumnWidth(0, 106);
    m_taskTable->setColumnWidth(2, 78);
    m_taskTable->setColumnWidth(3, 230);
    m_taskTable->setColumnWidth(4, 88);
    m_taskTable->setColumnWidth(5, 90);
}
SolveTaskManagerDialog::~SolveTaskManagerDialog() = default;
QPushButton* SolveTaskManagerDialog::solveAllButton() const
{
    return m_solveAllButton;
}
QPushButton* SolveTaskManagerDialog::restartAllButton() const
{
    return m_restartAllButton;
}
QTableWidget* SolveTaskManagerDialog::taskTable() const
{
    return m_taskTable;
}

void SolveTaskManagerDialog::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("SolveTaskManagerDialogClass");
    }
    setMinimumSize(QSize(800, 380));
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setSpacing(12);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(16, 16, 16, 12);
    QHBoxLayout* summaryLayout = new QHBoxLayout();
    summaryLayout->setObjectName("summaryLayout");
    m_hintLabel = new QLabel(this);
    m_hintLabel->setObjectName("hintLabel");
    m_hintLabel->setWordWrap(true);

    summaryLayout->addWidget(m_hintLabel);

    rootLayout->addLayout(summaryLayout);

    m_taskTable = new QTableWidget(this);
    if (m_taskTable->columnCount() < 6)
        m_taskTable->setColumnCount(6);
    QTableWidgetItem* __qtablewidgetitem = new QTableWidgetItem();
    m_taskTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
    QTableWidgetItem* __qtablewidgetitem1 = new QTableWidgetItem();
    m_taskTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
    QTableWidgetItem* __qtablewidgetitem2 = new QTableWidgetItem();
    m_taskTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
    QTableWidgetItem* __qtablewidgetitem3 = new QTableWidgetItem();
    m_taskTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
    QTableWidgetItem* __qtablewidgetitem4 = new QTableWidgetItem();
    m_taskTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
    QTableWidgetItem* __qtablewidgetitem5 = new QTableWidgetItem();
    m_taskTable->setHorizontalHeaderItem(5, __qtablewidgetitem5);
    m_taskTable->setObjectName("taskTable");
    m_taskTable->setColumnCount(6);
    m_taskTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_taskTable->setSelectionMode(QAbstractItemView::SingleSelection);
    m_taskTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_taskTable->setAlternatingRowColors(true);
    m_taskTable->setShowGrid(false);
    m_taskTable->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);

    rootLayout->addWidget(m_taskTable);

    QHBoxLayout* actionLayout = new QHBoxLayout();
    actionLayout->setSpacing(8);
    actionLayout->setObjectName("actionLayout");
    m_solveAllButton = new QPushButton(this);
    m_solveAllButton->setObjectName("solveAllButton");
    m_solveAllButton->setMinimumSize(QSize(108, 0));
    m_solveAllButton->setAutoDefault(false);

    actionLayout->addWidget(m_solveAllButton);

    m_restartAllButton = new QPushButton(this);
    m_restartAllButton->setObjectName("restartAllButton");
    m_restartAllButton->setMinimumSize(QSize(108, 0));
    m_restartAllButton->setAutoDefault(false);

    actionLayout->addWidget(m_restartAllButton);

    QSpacerItem* summarySpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

    actionLayout->addItem(summarySpacer);

    m_buttonBox = new QDialogButtonBox(this);
    m_buttonBox->setObjectName("buttonBox");
    m_buttonBox->setStandardButtons(QDialogButtonBox::Close);

    actionLayout->addWidget(m_buttonBox);

    rootLayout->addLayout(actionLayout);

    QWidget::setTabOrder(m_taskTable, m_solveAllButton);
    QWidget::setTabOrder(m_solveAllButton, m_restartAllButton);

    setWindowTitle(QCoreApplication::translate("SolveTaskManagerDialogClass", "全部分析步求解", nullptr));
    m_hintLabel->setText(QCoreApplication::translate("SolveTaskManagerDialogClass",
                                                     "全部分析步算例，可在此直接运行、停止并查看完整进度。", nullptr));
    QTableWidgetItem* ___qtablewidgetitem = m_taskTable->horizontalHeaderItem(0);
    ___qtablewidgetitem->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "操作", nullptr));
    QTableWidgetItem* ___qtablewidgetitem1 = m_taskTable->horizontalHeaderItem(1);
    ___qtablewidgetitem1->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "算例名称", nullptr));
    QTableWidgetItem* ___qtablewidgetitem2 = m_taskTable->horizontalHeaderItem(2);
    ___qtablewidgetitem2->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "分析步", nullptr));
    QTableWidgetItem* ___qtablewidgetitem3 = m_taskTable->horizontalHeaderItem(3);
    ___qtablewidgetitem3->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "进度", nullptr));
    QTableWidgetItem* ___qtablewidgetitem4 = m_taskTable->horizontalHeaderItem(4);
    ___qtablewidgetitem4->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "结果", nullptr));
    QTableWidgetItem* ___qtablewidgetitem5 = m_taskTable->horizontalHeaderItem(5);
    ___qtablewidgetitem5->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "耗时", nullptr));
    m_solveAllButton->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "全部计算", nullptr));
#if QT_CONFIG(tooltip)
    m_solveAllButton->setToolTip(QCoreApplication::translate(
        "SolveTaskManagerDialogClass",
        "将所有待运行分析步加入队列；正在计算、已排队和已有结果的算例不会重复计算；单个分析步失败不会停止其他队列任务",
        nullptr));
#endif // QT_CONFIG(tooltip)
    m_restartAllButton->setText(QCoreApplication::translate("SolveTaskManagerDialogClass", "重新计算", nullptr));
#if QT_CONFIG(tooltip)
    m_restartAllButton->setToolTip(QCoreApplication::translate(
        "SolveTaskManagerDialogClass", "重新计算所有分析步；正在排队或计算的任务会先安全停止，再从头加入计算队列",
        nullptr));
#endif // QT_CONFIG(tooltip)

    QObject::connect(m_buttonBox, &QDialogButtonBox::rejected, this, qOverload<>(&QDialog::close));
}
