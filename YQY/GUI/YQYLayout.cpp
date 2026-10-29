#include "YQY.h"
#include "Widgets/NavigationButton.h"
#include "Widgets/ModelViewport.h"
#include "Widgets/WorkspaceSplitter.h"
#include "Widgets/SingleLineLabel.h"
#include "Widgets/ResultControlInputWidgets.h"
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSize>
#include <QSizePolicy>
#include <QSpacerItem>
#include <QSplitter>
#include <QStackedWidget>
#include <QString>
#include <QTabBar>
#include <QTabWidget>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QWidget>

void YQY::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("YQYClass");
    }
    resize(1500, 850);
    setMinimumSize(QSize(760, 520));
    QWidget* centralWidget = new QWidget(this);
    centralWidget->setObjectName("centralWidget");
    QVBoxLayout* rootLayout = new QVBoxLayout(centralWidget);
    rootLayout->setSpacing(0);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(0, 0, 0, 0);
    initializeHeader(centralWidget, rootLayout);

    QHBoxLayout* bodyLayout = new QHBoxLayout();
    bodyLayout->setSpacing(0);
    bodyLayout->setObjectName("bodyLayout");
    bodyLayout->setContentsMargins(0, 0, 0, 0);
    initializeNavigation(centralWidget, bodyLayout);

    m_mainSplitter = new QSplitter(centralWidget);
    m_mainSplitter->setObjectName("mainSplitter");
    m_mainSplitter->setOrientation(Qt::Horizontal);
    m_mainSplitter->setChildrenCollapsible(false);
    initializeProjectPanel();

    initializeWorkspacePanel();

    bodyLayout->addWidget(m_mainSplitter);

    rootLayout->addLayout(bodyLayout);

    initializeStatusBar(centralWidget, rootLayout);

    setCentralWidget(centralWidget);

    setWindowTitle(QCoreApplication::translate("YQYClass", "YQY CAE", nullptr));
    m_importButton->setText(QString());
    m_undoButton->setText(QString());
    m_selectModeButton->setText(QString());
    m_rotateModeButton->setText(QString());
    m_panModeButton->setText(QString());
    m_zoomModeButton->setText(QString());
    m_showNodesButton->setText(QString());
    m_showElementsButton->setText(QString());
    m_showSolidButton->setText(QString());
    m_showNodeIdsButton->setText(QString());
    m_showElementIdsButton->setText(QString());
    m_frontViewButton->setText(QString());
    m_backViewButton->setText(QString());
    m_leftViewButton->setText(QString());
    m_rightViewButton->setText(QString());
    m_topViewButton->setText(QString());
    m_bottomViewButton->setText(QString());
    m_fitViewButton->setText(QString());
    m_startSolveButton->setText(QString());

    m_moduleStack->setCurrentIndex(0);
    m_workspaceDisplayStack->setCurrentIndex(0);
    m_monitorTabs->setCurrentIndex(0);
}

void YQY::initializeHeader(QWidget* centralWidget, QVBoxLayout* rootLayout)
{
    QFrame* appHeader = new QFrame(centralWidget);
    appHeader->setObjectName("appHeader");
    appHeader->setMinimumSize(QSize(0, 58));
    appHeader->setMaximumSize(QSize(16777215, 58));
    QHBoxLayout* headerLayout = new QHBoxLayout(appHeader);
    headerLayout->setSpacing(12);
    headerLayout->setObjectName("headerLayout");
    headerLayout->setContentsMargins(18, -1, 18, -1);
    QLabel* logoLabel = new QLabel(appHeader);
    logoLabel->setObjectName("logoLabel");
    logoLabel->setMinimumSize(QSize(32, 32));
    logoLabel->setMaximumSize(QSize(32, 32));
    logoLabel->setAlignment(Qt::AlignCenter);

    headerLayout->addWidget(logoLabel);

    m_appTitleLabel = new QLabel(appHeader);
    m_appTitleLabel->setObjectName("appTitleLabel");

    headerLayout->addWidget(m_appTitleLabel);

    QSpacerItem* headerSpacer = new QSpacerItem(80, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

    headerLayout->addItem(headerSpacer);

    m_themeLabel = new QLabel(appHeader);
    m_themeLabel->setObjectName("themeLabel");

    headerLayout->addWidget(m_themeLabel);

    m_themeComboBox = new QComboBox(appHeader);
    m_themeComboBox->addItem(QString());
    m_themeComboBox->addItem(QString());
    m_themeComboBox->addItem(QString());
    m_themeComboBox->addItem(QString());
    m_themeComboBox->setObjectName("themeComboBox");
    m_themeComboBox->setMinimumSize(QSize(150, 34));

    headerLayout->addWidget(m_themeComboBox);

    rootLayout->addWidget(appHeader);

    logoLabel->setText(QCoreApplication::translate("YQYClass", "Y", nullptr));

    m_appTitleLabel->setText(QCoreApplication::translate("YQYClass", "YQY CAE", nullptr));
    m_themeLabel->setText(QCoreApplication::translate("YQYClass", "主题", nullptr));
    m_themeComboBox->setItemText(0, QCoreApplication::translate("YQYClass", "A · 黑色", nullptr));
    m_themeComboBox->setItemText(1, QCoreApplication::translate("YQYClass", "B · 紫色", nullptr));
    m_themeComboBox->setItemText(2, QCoreApplication::translate("YQYClass", "C · 绿色", nullptr));
    m_themeComboBox->setItemText(3, QCoreApplication::translate("YQYClass", "D · 浅色", nullptr));
}

void YQY::initializeNavigation(QWidget* centralWidget, QHBoxLayout* bodyLayout)
{
    m_navigationRail = new QFrame(centralWidget);
    m_navigationRail->setObjectName("navigationRail");
    m_navigationRail->setMinimumSize(QSize(84, 0));
    m_navigationRail->setMaximumSize(QSize(84, 16777215));
    QVBoxLayout* navigationLayout = new QVBoxLayout(m_navigationRail);
    navigationLayout->setSpacing(8);
    navigationLayout->setObjectName("navigationLayout");
    navigationLayout->setContentsMargins(8, 14, 8, 14);
    m_modelButton = new NavigationButton(m_navigationRail);
    m_modelButton->setObjectName("modelButton");
    m_modelButton->setMinimumSize(QSize(66, 58));
    m_modelButton->setCheckable(true);
    m_modelButton->setChecked(true);
    m_modelButton->setAutoExclusive(true);

    navigationLayout->addWidget(m_modelButton);

    m_propertyButton = new NavigationButton(m_navigationRail);
    m_propertyButton->setObjectName("propertyButton");
    m_propertyButton->setMinimumSize(QSize(66, 58));
    m_propertyButton->setCheckable(true);
    m_propertyButton->setAutoExclusive(true);

    navigationLayout->addWidget(m_propertyButton);

    m_analysisButton = new NavigationButton(m_navigationRail);
    m_analysisButton->setObjectName("analysisButton");
    m_analysisButton->setMinimumSize(QSize(66, 58));
    m_analysisButton->setCheckable(true);
    m_analysisButton->setAutoExclusive(true);

    navigationLayout->addWidget(m_analysisButton);

    m_solveButton = new NavigationButton(m_navigationRail);
    m_solveButton->setObjectName("solveButton");
    m_solveButton->setMinimumSize(QSize(66, 58));
    m_solveButton->setCheckable(true);
    m_solveButton->setAutoExclusive(true);

    navigationLayout->addWidget(m_solveButton);

    m_resultButton = new NavigationButton(m_navigationRail);
    m_resultButton->setObjectName("resultButton");
    m_resultButton->setMinimumSize(QSize(66, 58));
    m_resultButton->setCheckable(true);
    m_resultButton->setAutoExclusive(true);

    navigationLayout->addWidget(m_resultButton);

    QSpacerItem* navigationSpacer =
        new QSpacerItem(20, 160, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

    navigationLayout->addItem(navigationSpacer);

    m_settingsButton = new NavigationButton(m_navigationRail);
    m_settingsButton->setObjectName("settingsButton");
    m_settingsButton->setMinimumSize(QSize(66, 58));
    m_settingsButton->setCheckable(true);
    m_settingsButton->setAutoExclusive(true);

    navigationLayout->addWidget(m_settingsButton);

    bodyLayout->addWidget(m_navigationRail);

    m_modelButton->setText(QCoreApplication::translate("YQYClass", "模型", nullptr));
    m_propertyButton->setText(QCoreApplication::translate("YQYClass", "属性", nullptr));
    m_analysisButton->setText(QCoreApplication::translate("YQYClass", "分析", nullptr));
    m_solveButton->setText(QCoreApplication::translate("YQYClass", "求解", nullptr));
    m_resultButton->setText(QCoreApplication::translate("YQYClass", "结果", nullptr));
    m_settingsButton->setText(QCoreApplication::translate("YQYClass", "设置", nullptr));
}

void YQY::initializeModelPage()
{
    QWidget* modelModulePage = new QWidget();
    modelModulePage->setObjectName("modelModulePage");
    QVBoxLayout* modelModuleLayout = new QVBoxLayout(modelModulePage);
    modelModuleLayout->setSpacing(10);
    modelModuleLayout->setObjectName("modelModuleLayout");
    modelModuleLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* modelModuleHint = new QLabel(modelModulePage);
    modelModuleHint->setObjectName("modelModuleHint");
    modelModuleHint->setWordWrap(true);

    modelModuleLayout->addWidget(modelModuleHint);

    QFrame* modelSummaryCard = new QFrame(modelModulePage);
    modelSummaryCard->setObjectName("modelSummaryCard");
    QVBoxLayout* modelSummaryLayout = new QVBoxLayout(modelSummaryCard);
    modelSummaryLayout->setSpacing(8);
    modelSummaryLayout->setObjectName("modelSummaryLayout");
    modelSummaryLayout->setContentsMargins(12, 12, 12, 12);
    m_modelFileValue = new QLabel(modelSummaryCard);
    m_modelFileValue->setObjectName("modelFileValue");
    m_modelFileValue->setWordWrap(true);

    modelSummaryLayout->addWidget(m_modelFileValue);

    m_modelNodeValue = new QLabel(modelSummaryCard);
    m_modelNodeValue->setObjectName("modelNodeValue");

    modelSummaryLayout->addWidget(m_modelNodeValue);

    m_modelElementValue = new QLabel(modelSummaryCard);
    m_modelElementValue->setObjectName("modelElementValue");

    modelSummaryLayout->addWidget(m_modelElementValue);

    m_modelPropertyValue = new QLabel(modelSummaryCard);
    m_modelPropertyValue->setObjectName("modelPropertyValue");

    modelSummaryLayout->addWidget(m_modelPropertyValue);

    modelModuleLayout->addWidget(modelSummaryCard);

    m_modelDataTree = new QTreeWidget(modelModulePage);
    QTreeWidgetItem* __qtreewidgetitem = new QTreeWidgetItem();
    __qtreewidgetitem->setText(0, QString::fromUtf8("1"));
    m_modelDataTree->setHeaderItem(__qtreewidgetitem);
    m_modelDataTree->setObjectName("modelDataTree");
    m_modelDataTree->setHeaderHidden(true);
    m_modelDataTree->setIndentation(18);
    m_modelDataTree->setAnimated(true);

    modelModuleLayout->addWidget(m_modelDataTree);

    m_modelImportButton = new QPushButton(modelModulePage);
    m_modelImportButton->setObjectName("modelImportButton");

    modelModuleLayout->addWidget(m_modelImportButton);

    m_createConductorButton = new QPushButton(modelModulePage);
    m_createConductorButton->setObjectName("createConductorButton");

    modelModuleLayout->addWidget(m_createConductorButton);

    m_moduleStack->addWidget(modelModulePage);

    modelModuleHint->setText(QCoreApplication::translate("YQYClass", "模型数据与显示控制", nullptr));

    m_modelFileValue->setText(QCoreApplication::translate("YQYClass", "尚未加载模型", nullptr));
    m_modelNodeValue->setText(QCoreApplication::translate("YQYClass", "节点 --", nullptr));
    m_modelElementValue->setText(QCoreApplication::translate("YQYClass", "单元 --", nullptr));
    m_modelPropertyValue->setText(QCoreApplication::translate("YQYClass", "材料 -- · 截面 --", nullptr));
    m_modelImportButton->setText(QCoreApplication::translate("YQYClass", "导入模型", nullptr));
    m_createConductorButton->setText(QCoreApplication::translate("YQYClass", "创建导线", nullptr));
}

void YQY::initializeAnalysisPage()
{
    m_analysisModulePage = new QWidget();
    m_analysisModulePage->setObjectName("analysisModulePage");
    m_analysisModuleLayout = new QVBoxLayout(m_analysisModulePage);
    m_analysisModuleLayout->setSpacing(10);
    m_analysisModuleLayout->setObjectName("analysisModuleLayout");
    m_analysisModuleLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* analysisModuleHint = new QLabel(m_analysisModulePage);
    analysisModuleHint->setObjectName("analysisModuleHint");
    analysisModuleHint->setWordWrap(true);

    m_analysisModuleLayout->addWidget(analysisModuleHint);

    m_analysisTree = new QTreeWidget(m_analysisModulePage);
    QTreeWidgetItem* __qtreewidgetitem1 = new QTreeWidgetItem();
    __qtreewidgetitem1->setText(0, QString::fromUtf8("1"));
    m_analysisTree->setHeaderItem(__qtreewidgetitem1);
    m_analysisTree->setObjectName("analysisTree");
    m_analysisTree->setHeaderHidden(true);
    m_analysisTree->setAnimated(true);

    m_analysisModuleLayout->addWidget(m_analysisTree);

    m_moduleStack->addWidget(m_analysisModulePage);

    analysisModuleHint->setText(QCoreApplication::translate("YQYClass", "分析步、载荷与边界条件", nullptr));
}

void YQY::initializeSolvePage()
{
    m_solveModulePage = new QWidget();
    m_solveModulePage->setObjectName("solveModulePage");
    QVBoxLayout* solveModuleLayout = new QVBoxLayout(m_solveModulePage);
    solveModuleLayout->setSpacing(10);
    solveModuleLayout->setObjectName("solveModuleLayout");
    solveModuleLayout->setContentsMargins(0, 0, 0, 0);
    m_solveReadinessLabel = new QLabel(m_solveModulePage);
    m_solveReadinessLabel->setObjectName("solveReadinessLabel");
    m_solveReadinessLabel->setWordWrap(true);

    solveModuleLayout->addWidget(m_solveReadinessLabel);

    m_solveBatchActionsLayout = new QGridLayout();
    m_solveBatchActionsLayout->setSpacing(6);
    m_solveBatchActionsLayout->setObjectName("solveBatchActionsLayout");
    m_solveCasesButton = new QPushButton(m_solveModulePage);
    m_solveCasesButton->setObjectName("solveCasesButton");
    m_solveCasesButton->setMinimumSize(QSize(0, 54));

    m_solveBatchActionsLayout->addWidget(m_solveCasesButton, 0, 0, 1, 1);

    m_solveAllCasesButton = new QPushButton(m_solveModulePage);
    m_solveAllCasesButton->setObjectName("solveAllCasesButton");
    m_solveAllCasesButton->setMinimumSize(QSize(0, 54));
    m_solveAllCasesButton->setEnabled(false);

    m_solveBatchActionsLayout->addWidget(m_solveAllCasesButton, 0, 1, 1, 1);

    m_solveRestartAllCasesButton = new QPushButton(m_solveModulePage);
    m_solveRestartAllCasesButton->setObjectName("solveRestartAllCasesButton");
    m_solveRestartAllCasesButton->setMinimumSize(QSize(0, 54));
    m_solveRestartAllCasesButton->setEnabled(false);

    m_solveBatchActionsLayout->addWidget(m_solveRestartAllCasesButton, 0, 2, 1, 1);

    solveModuleLayout->addLayout(m_solveBatchActionsLayout);

    m_solveStepTree = new QTreeWidget(m_solveModulePage);
    QTreeWidgetItem* __qtreewidgetitem2 = new QTreeWidgetItem();
    __qtreewidgetitem2->setText(0, QString::fromUtf8("1"));
    m_solveStepTree->setHeaderItem(__qtreewidgetitem2);
    m_solveStepTree->setObjectName("solveStepTree");
    m_solveStepTree->setHeaderHidden(true);
    m_solveStepTree->setAnimated(true);

    solveModuleLayout->addWidget(m_solveStepTree);

    m_solvePageStartButton = new QPushButton(m_solveModulePage);
    m_solvePageStartButton->setObjectName("solvePageStartButton");
    m_solvePageStartButton->setEnabled(false);

    solveModuleLayout->addWidget(m_solvePageStartButton);

    QSpacerItem* solveModuleSpacer =
        new QSpacerItem(20, 80, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

    solveModuleLayout->addItem(solveModuleSpacer);

    m_moduleStack->addWidget(m_solveModulePage);

    m_solveReadinessLabel->setText(QCoreApplication::translate("YQYClass", "请先加载包含分析步的模型", nullptr));
    m_solveCasesButton->setText(QCoreApplication::translate("YQYClass", "全部分析步 · 0", nullptr));
    m_solveAllCasesButton->setText(QCoreApplication::translate("YQYClass", "全部计算 · 0", nullptr));
    m_solveRestartAllCasesButton->setText(QCoreApplication::translate("YQYClass", "重新计算 · 0", nullptr));
    m_solvePageStartButton->setText(QCoreApplication::translate("YQYClass", "运行全部分析步", nullptr));
}

void YQY::initializeResultPage()
{
    m_resultModulePage = new QWidget();
    m_resultModulePage->setObjectName("resultModulePage");
    m_resultModuleLayout = new QVBoxLayout(m_resultModulePage);
    m_resultModuleLayout->setSpacing(10);
    m_resultModuleLayout->setObjectName("resultModuleLayout");
    m_resultModuleLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* resultModuleHint = new QLabel(m_resultModulePage);
    resultModuleHint->setObjectName("resultModuleHint");
    resultModuleHint->setWordWrap(true);

    m_resultModuleLayout->addWidget(resultModuleHint);

    m_openH5Button = new QPushButton(m_resultModulePage);
    m_openH5Button->setObjectName("openH5Button");

    m_resultModuleLayout->addWidget(m_openH5Button);

    QHBoxLayout* resultCaseLayout = new QHBoxLayout();
    resultCaseLayout->setObjectName("resultCaseLayout");
    QLabel* resultCaseLabel = new QLabel(m_resultModulePage);
    resultCaseLabel->setObjectName("resultCaseLabel");

    resultCaseLayout->addWidget(resultCaseLabel);

    m_resultCaseCombo = new ResultSelectionComboBox(m_resultModulePage);
    m_resultCaseCombo->setObjectName("resultCaseCombo");
    m_resultCaseCombo->setEnabled(false);

    resultCaseLayout->addWidget(m_resultCaseCombo);

    m_resultModuleLayout->addLayout(resultCaseLayout);

    m_resultFileLabel = new SingleLineLabel(m_resultModulePage);
    m_resultFileLabel->setObjectName("resultFileLabel");

    m_resultModuleLayout->addWidget(m_resultFileLabel);

    m_resultTree = new QTreeWidget(m_resultModulePage);
    QTreeWidgetItem* __qtreewidgetitem3 = new QTreeWidgetItem();
    __qtreewidgetitem3->setText(0, QString::fromUtf8("1"));
    m_resultTree->setHeaderItem(__qtreewidgetitem3);
    m_resultTree->setObjectName("resultTree");
    m_resultTree->setHeaderHidden(true);
    m_resultTree->setAnimated(true);

    m_resultModuleLayout->addWidget(m_resultTree);

    m_moduleStack->addWidget(m_resultModulePage);

    resultModuleHint->setText(QCoreApplication::translate("YQYClass", "结果后处理显示", nullptr));
    resultCaseLabel->setText(QCoreApplication::translate("YQYClass", "结果算例", nullptr));

    m_openH5Button->setText(QCoreApplication::translate("YQYClass", "打开 H5 结果文件", nullptr));
    m_resultCaseCombo->setToolTip(
        QCoreApplication::translate("YQYClass", "切换当前模型下不同分析步的计算结果", nullptr));
    m_resultFileLabel->setText(QCoreApplication::translate("YQYClass", "尚未打开结果文件", nullptr));
}

void YQY::initializeSettingsPage()
{
    m_settingsModulePage = new QWidget();
    m_settingsModulePage->setObjectName("settingsModulePage");
    m_settingsModuleLayout = new QVBoxLayout(m_settingsModulePage);
    m_settingsModuleLayout->setSpacing(10);
    m_settingsModuleLayout->setObjectName("settingsModuleLayout");
    m_settingsModuleLayout->setContentsMargins(0, 0, 0, 0);
    QLabel* settingsModuleHint = new QLabel(m_settingsModulePage);
    settingsModuleHint->setObjectName("settingsModuleHint");
    settingsModuleHint->setWordWrap(true);

    m_settingsModuleLayout->addWidget(settingsModuleHint);

    QSpacerItem* settingsModuleSpacer =
        new QSpacerItem(20, 80, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

    m_settingsModuleLayout->addItem(settingsModuleSpacer);

    m_moduleStack->addWidget(m_settingsModulePage);

    settingsModuleHint->setText(
        QCoreApplication::translate("YQYClass", "界面设置保存在本机，主题可在窗口顶部切换", nullptr));
}

void YQY::initializeViewportToolbar()
{
    m_toolbarScrollArea = new QScrollArea(m_workspacePanel);
    m_toolbarScrollArea->setObjectName("toolbarScrollArea");
    m_toolbarScrollArea->setFrameShape(QFrame::NoFrame);
    m_toolbarScrollArea->setWidgetResizable(true);
    m_toolbarScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_toolbarScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_toolbarScrollArea->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    m_toolBarCard = new QFrame();
    m_toolBarCard->setObjectName("toolBarCard");
    m_toolBarLayout = new QHBoxLayout(m_toolBarCard);
    m_toolBarLayout->setSpacing(5);
    m_toolBarLayout->setObjectName("toolBarLayout");
    m_toolBarLayout->setContentsMargins(8, 6, 8, 6);
    m_importButton = new QPushButton(m_toolBarCard);
    m_importButton->setObjectName("importButton");

    m_toolBarLayout->addWidget(m_importButton);

    m_undoButton = new QPushButton(m_toolBarCard);
    m_undoButton->setObjectName("undoButton");

    m_toolBarLayout->addWidget(m_undoButton);

    m_selectModeButton = new QToolButton(m_toolBarCard);
    m_selectModeButton->setObjectName("selectModeButton");
    m_selectModeButton->setCheckable(true);
    m_selectModeButton->setChecked(true);

    m_toolBarLayout->addWidget(m_selectModeButton);

    m_rotateModeButton = new QToolButton(m_toolBarCard);
    m_rotateModeButton->setObjectName("rotateModeButton");
    m_rotateModeButton->setCheckable(true);

    m_toolBarLayout->addWidget(m_rotateModeButton);

    m_panModeButton = new QToolButton(m_toolBarCard);
    m_panModeButton->setObjectName("panModeButton");
    m_panModeButton->setCheckable(true);

    m_toolBarLayout->addWidget(m_panModeButton);

    m_zoomModeButton = new QToolButton(m_toolBarCard);
    m_zoomModeButton->setObjectName("zoomModeButton");
    m_zoomModeButton->setCheckable(true);

    m_toolBarLayout->addWidget(m_zoomModeButton);

    m_showNodesButton = new QToolButton(m_toolBarCard);
    m_showNodesButton->setObjectName("showNodesButton");
    m_showNodesButton->setCheckable(true);
    m_showNodesButton->setChecked(true);

    m_toolBarLayout->addWidget(m_showNodesButton);

    m_showElementsButton = new QToolButton(m_toolBarCard);
    m_showElementsButton->setObjectName("showElementsButton");
    m_showElementsButton->setCheckable(true);
    m_showElementsButton->setChecked(true);

    m_toolBarLayout->addWidget(m_showElementsButton);

    m_showSolidButton = new QToolButton(m_toolBarCard);
    m_showSolidButton->setObjectName("showSolidButton");
    m_showSolidButton->setCheckable(true);

    m_toolBarLayout->addWidget(m_showSolidButton);

    m_showNodeIdsButton = new QToolButton(m_toolBarCard);
    m_showNodeIdsButton->setObjectName("showNodeIdsButton");
    m_showNodeIdsButton->setCheckable(true);

    m_toolBarLayout->addWidget(m_showNodeIdsButton);

    m_showElementIdsButton = new QToolButton(m_toolBarCard);
    m_showElementIdsButton->setObjectName("showElementIdsButton");
    m_showElementIdsButton->setCheckable(true);

    m_toolBarLayout->addWidget(m_showElementIdsButton);

    m_frontViewButton = new QToolButton(m_toolBarCard);
    m_frontViewButton->setObjectName("frontViewButton");

    m_toolBarLayout->addWidget(m_frontViewButton);

    m_backViewButton = new QToolButton(m_toolBarCard);
    m_backViewButton->setObjectName("backViewButton");

    m_toolBarLayout->addWidget(m_backViewButton);

    m_leftViewButton = new QToolButton(m_toolBarCard);
    m_leftViewButton->setObjectName("leftViewButton");

    m_toolBarLayout->addWidget(m_leftViewButton);

    m_rightViewButton = new QToolButton(m_toolBarCard);
    m_rightViewButton->setObjectName("rightViewButton");

    m_toolBarLayout->addWidget(m_rightViewButton);

    m_topViewButton = new QToolButton(m_toolBarCard);
    m_topViewButton->setObjectName("topViewButton");

    m_toolBarLayout->addWidget(m_topViewButton);

    m_bottomViewButton = new QToolButton(m_toolBarCard);
    m_bottomViewButton->setObjectName("bottomViewButton");

    m_toolBarLayout->addWidget(m_bottomViewButton);

    m_fitViewButton = new QToolButton(m_toolBarCard);
    m_fitViewButton->setObjectName("fitViewButton");

    m_toolBarLayout->addWidget(m_fitViewButton);

    QSpacerItem* toolBarSpacer = new QSpacerItem(20, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

    m_toolBarLayout->addItem(toolBarSpacer);

    m_startSolveButton = new QPushButton(m_toolBarCard);
    m_startSolveButton->setObjectName("startSolveButton");

    m_toolBarLayout->addWidget(m_startSolveButton);

    m_toolbarScrollArea->setWidget(m_toolBarCard);

    m_workspaceLayout->addWidget(m_toolbarScrollArea);

    m_frontViewButton->setToolTip(QCoreApplication::translate("YQYClass", "前视图", nullptr));
    m_backViewButton->setToolTip(QCoreApplication::translate("YQYClass", "后视图", nullptr));
    m_leftViewButton->setToolTip(QCoreApplication::translate("YQYClass", "左视图", nullptr));
    m_rightViewButton->setToolTip(QCoreApplication::translate("YQYClass", "右视图", nullptr));
    m_topViewButton->setToolTip(QCoreApplication::translate("YQYClass", "顶视图", nullptr));
    m_bottomViewButton->setToolTip(QCoreApplication::translate("YQYClass", "底视图", nullptr));
    m_fitViewButton->setToolTip(QCoreApplication::translate("YQYClass", "适应窗口", nullptr));
}

void YQY::initializeMonitorPages()
{
    m_monitorTabs = new QTabWidget(m_workspaceVerticalSplitter);
    m_monitorTabs->setObjectName("monitorTabs");
    m_monitorTabs->setMinimumSize(QSize(0, 100));
    m_monitorPage = new QWidget();
    m_monitorPage->setObjectName("monitorPage");
    m_monitorLayout = new QVBoxLayout(m_monitorPage);
    m_monitorLayout->setSpacing(8);
    m_monitorLayout->setObjectName("monitorLayout");
    m_monitorLayout->setContentsMargins(12, 8, 12, 8);
    m_incrementLabel = new SingleLineLabel(m_monitorPage);
    m_incrementLabel->setObjectName("incrementLabel");

    m_monitorLayout->addWidget(m_incrementLabel);

    m_solveProgress = new QProgressBar(m_monitorPage);
    m_solveProgress->setObjectName("solveProgress");
    m_solveProgress->setValue(0);
    m_solveProgress->setTextVisible(true);

    m_monitorLayout->addWidget(m_solveProgress);

    QHBoxLayout* monitorStatusLayout = new QHBoxLayout();
    monitorStatusLayout->setSpacing(12);
    monitorStatusLayout->setObjectName("monitorStatusLayout");
    m_convergenceLabel = new SingleLineLabel(m_monitorPage);
    m_convergenceLabel->setObjectName("convergenceLabel");

    monitorStatusLayout->addWidget(m_convergenceLabel);

    m_stopButton = new QPushButton(m_monitorPage);
    m_stopButton->setObjectName("stopButton");

    monitorStatusLayout->addWidget(m_stopButton);

    monitorStatusLayout->setStretch(0, 1);

    m_monitorLayout->addLayout(monitorStatusLayout);

    m_monitorTabs->addTab(m_monitorPage, QString());
    m_logPage = new QWidget();
    m_logPage->setObjectName("logPage");
    QVBoxLayout* logLayout = new QVBoxLayout(m_logPage);
    logLayout->setSpacing(8);
    logLayout->setObjectName("logLayout");
    logLayout->setContentsMargins(12, 8, 12, 8);
    QHBoxLayout* logActionsLayout = new QHBoxLayout();
    logActionsLayout->setObjectName("logActionsLayout");
    QLabel* logHeadingLabel = new QLabel(m_logPage);
    logHeadingLabel->setObjectName("logHeadingLabel");

    logActionsLayout->addWidget(logHeadingLabel);

    QSpacerItem* logActionsSpacer =
        new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

    logActionsLayout->addItem(logActionsSpacer);

    m_logDetailsCheck = new QCheckBox(m_logPage);
    m_logDetailsCheck->setObjectName("logDetailsCheck");

    logActionsLayout->addWidget(m_logDetailsCheck);

    m_copyLogButton = new QPushButton(m_logPage);
    m_copyLogButton->setObjectName("copyLogButton");

    logActionsLayout->addWidget(m_copyLogButton);

    m_clearLogButton = new QPushButton(m_logPage);
    m_clearLogButton->setObjectName("clearLogButton");

    logActionsLayout->addWidget(m_clearLogButton);

    logLayout->addLayout(logActionsLayout);

    m_logEdit = new QPlainTextEdit(m_logPage);
    m_logEdit->setObjectName("logEdit");
    m_logEdit->setReadOnly(true);

    logLayout->addWidget(m_logEdit);

    m_monitorTabs->addTab(m_logPage, QString());
    m_workspaceVerticalSplitter->addWidget(m_monitorTabs);

    logHeadingLabel->setText(QCoreApplication::translate("YQYClass", "运行记录", nullptr));

    m_incrementLabel->setText(QCoreApplication::translate("YQYClass", "尚未开始求解", nullptr));
    m_convergenceLabel->setText(QCoreApplication::translate("YQYClass", "加载模型并配置分析步后可开始求解", nullptr));
    m_stopButton->setText(QCoreApplication::translate("YQYClass", "停止", nullptr));
    m_monitorTabs->setTabText(m_monitorTabs->indexOf(m_monitorPage),
                              QCoreApplication::translate("YQYClass", "求解监视器", nullptr));
    m_logDetailsCheck->setText(QCoreApplication::translate("YQYClass", "显示详细诊断", nullptr));
    m_copyLogButton->setText(QCoreApplication::translate("YQYClass", "复制", nullptr));
    m_clearLogButton->setText(QCoreApplication::translate("YQYClass", "清空", nullptr));
    m_monitorTabs->setTabText(m_monitorTabs->indexOf(m_logPage),
                              QCoreApplication::translate("YQYClass", "日志", nullptr));
}

void YQY::initializeStatusBar(QWidget* centralWidget, QVBoxLayout* rootLayout)
{
    QFrame* statusFrame = new QFrame(centralWidget);
    statusFrame->setObjectName("statusFrame");
    QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(statusFrame->sizePolicy().hasHeightForWidth());
    statusFrame->setSizePolicy(sizePolicy);
    statusFrame->setMinimumSize(QSize(0, 32));
    QHBoxLayout* statusLayout = new QHBoxLayout(statusFrame);
    statusLayout->setSpacing(10);
    statusLayout->setObjectName("statusLayout");
    statusLayout->setContentsMargins(16, 4, 16, 4);
    m_loadStatisticsLabel = new QLabel(statusFrame);
    m_loadStatisticsLabel->setObjectName("loadStatisticsLabel");
    m_loadStatisticsLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    statusLayout->addWidget(m_loadStatisticsLabel);

    QSpacerItem* statusSpacer = new QSpacerItem(80, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

    statusLayout->addItem(statusSpacer);

    m_readyLabel = new QLabel(statusFrame);
    m_readyLabel->setObjectName("readyLabel");
    m_readyLabel->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

    statusLayout->addWidget(m_readyLabel);

    rootLayout->addWidget(statusFrame);

    m_loadStatisticsLabel->setText(
        QCoreApplication::translate("YQYClass", "模型读取  总计 0  ·  成功 0  ·  失败 0", nullptr));
    m_readyLabel->setText(QCoreApplication::translate("YQYClass", "等待导入模型", nullptr));
}

void YQY::initializeProjectPanel()
{
    m_projectPanel = new QFrame(m_mainSplitter);
    m_projectPanel->setObjectName("projectPanel");
    m_projectPanel->setMinimumSize(QSize(320, 0));
    m_projectPanel->setMaximumSize(QSize(520, 16777215));
    QVBoxLayout* projectLayout = new QVBoxLayout(m_projectPanel);
    projectLayout->setSpacing(10);
    projectLayout->setObjectName("projectLayout");
    projectLayout->setContentsMargins(14, 14, 14, 14);
    m_projectTitleLabel = new QLabel(m_projectPanel);
    m_projectTitleLabel->setObjectName("projectTitleLabel");

    projectLayout->addWidget(m_projectTitleLabel);

    m_moduleStack = new QStackedWidget(m_projectPanel);
    m_moduleStack->setObjectName("moduleStack");
    initializeModelPage();

    initializeAnalysisPage();

    initializeSolvePage();

    initializeResultPage();

    initializeSettingsPage();

    projectLayout->addWidget(m_moduleStack);

    m_mainSplitter->addWidget(m_projectPanel);

    m_projectTitleLabel->setText(QCoreApplication::translate("YQYClass", "模型数据", nullptr));
}

void YQY::initializeWorkspacePanel()
{
    m_workspacePanel = new QFrame(m_mainSplitter);
    m_workspacePanel->setObjectName("workspacePanel");
    m_workspacePanel->setMinimumSize(QSize(560, 0));
    m_workspaceLayout = new QVBoxLayout(m_workspacePanel);
    m_workspaceLayout->setSpacing(10);
    m_workspaceLayout->setObjectName("workspaceLayout");
    m_workspaceLayout->setContentsMargins(12, 10, 12, 10);
    m_documentTabBar = new QTabBar(m_workspacePanel);
    m_documentTabBar->setObjectName("documentTabBar");
    m_documentTabBar->setCurrentIndex(-1);
    m_documentTabBar->setTabsClosable(true);
    m_documentTabBar->setMovable(true);
    m_documentTabBar->setDrawBase(false);
    m_documentTabBar->setExpanding(false);

    m_workspaceLayout->addWidget(m_documentTabBar);

    initializeViewportToolbar();

    m_workspaceVerticalSplitter = new WorkspaceSplitter(m_workspacePanel);
    m_workspaceVerticalSplitter->setObjectName("workspaceVerticalSplitter");
    m_workspaceVerticalSplitter->setOrientation(Qt::Vertical);
    m_workspaceVerticalSplitter->setChildrenCollapsible(false);
    m_workspaceDisplayStack = new QStackedWidget(m_workspaceVerticalSplitter);
    m_workspaceDisplayStack->setObjectName("workspaceDisplayStack");
    m_workspaceDisplayStack->setMinimumSize(QSize(280, 160));
    m_modelViewport = new ModelViewport();
    m_modelViewport->setObjectName("modelViewport");
    m_modelViewport->setMinimumSize(QSize(280, 160));
    m_workspaceDisplayStack->addWidget(m_modelViewport);
    m_workspaceVerticalSplitter->addWidget(m_workspaceDisplayStack);
    initializeMonitorPages();

    m_workspaceLayout->addWidget(m_workspaceVerticalSplitter);

    m_mainSplitter->addWidget(m_workspacePanel);
}
