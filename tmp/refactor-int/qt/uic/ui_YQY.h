/********************************************************************************
** Form generated from reading UI file 'YQY.ui'
**
** Created by: Qt User Interface Compiler version 6.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_YQY_H
#define UI_YQY_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTabBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "Widgets/ModelViewport.h"
#include "Widgets/NavigationButton.h"

QT_BEGIN_NAMESPACE

class Ui_YQYClass
{
public:
    QWidget *centralWidget;
    QVBoxLayout *rootLayout;
    QFrame *appHeader;
    QHBoxLayout *headerLayout;
    QLabel *logoLabel;
    QLabel *appTitleLabel;
    QSpacerItem *headerSpacer;
    QLabel *themeLabel;
    QComboBox *themeComboBox;
    QHBoxLayout *bodyLayout;
    QFrame *navigationRail;
    QVBoxLayout *navigationLayout;
    NavigationButton *modelButton;
    NavigationButton *propertyButton;
    NavigationButton *conductorButton;
    NavigationButton *analysisButton;
    NavigationButton *solveButton;
    NavigationButton *resultButton;
    QSpacerItem *navigationSpacer;
    NavigationButton *settingsButton;
    QSplitter *mainSplitter;
    QFrame *projectPanel;
    QVBoxLayout *projectLayout;
    QLabel *projectTitleLabel;
    QStackedWidget *moduleStack;
    QWidget *modelModulePage;
    QVBoxLayout *modelModuleLayout;
    QLabel *modelModuleHint;
    QFrame *modelSummaryCard;
    QVBoxLayout *modelSummaryLayout;
    QLabel *modelFileValue;
    QLabel *modelNodeValue;
    QLabel *modelElementValue;
    QLabel *modelPropertyValue;
    QTreeWidget *modelDataTree;
    QPushButton *modelImportButton;
    QPushButton *modelFitButton;
    QSpacerItem *modelModuleSpacer;
    QWidget *analysisModulePage;
    QVBoxLayout *analysisModuleLayout;
    QLabel *analysisModuleHint;
    QTreeWidget *analysisTree;
    QWidget *solveModulePage;
    QVBoxLayout *solveModuleLayout;
    QLabel *solveReadinessLabel;
    QHBoxLayout *solveBatchActionsLayout;
    QPushButton *solveCasesButton;
    QPushButton *solveAllCasesButton;
    QPushButton *solveRestartAllCasesButton;
    QTreeWidget *solveStepTree;
    QPushButton *solvePageStartButton;
    QSpacerItem *solveModuleSpacer;
    QWidget *resultModulePage;
    QVBoxLayout *resultModuleLayout;
    QLabel *resultModuleHint;
    QPushButton *openH5Button;
    QLabel *resultFileLabel;
    QTreeWidget *resultTree;
    QWidget *settingsModulePage;
    QVBoxLayout *settingsModuleLayout;
    QLabel *settingsModuleHint;
    QSpacerItem *settingsModuleSpacer;
    QFrame *workspacePanel;
    QVBoxLayout *workspaceLayout;
    QTabBar *documentTabBar;
    QScrollArea *toolbarScrollArea;
    QFrame *toolBarCard;
    QHBoxLayout *toolBarLayout;
    QPushButton *importButton;
    QPushButton *undoButton;
    QToolButton *selectModeButton;
    QToolButton *rotateModeButton;
    QToolButton *panModeButton;
    QToolButton *zoomModeButton;
    QToolButton *showNodesButton;
    QToolButton *showElementsButton;
    QToolButton *showSolidButton;
    QToolButton *showNodeIdsButton;
    QToolButton *showElementIdsButton;
    QToolButton *frontViewButton;
    QToolButton *backViewButton;
    QToolButton *leftViewButton;
    QToolButton *rightViewButton;
    QToolButton *topViewButton;
    QToolButton *bottomViewButton;
    QToolButton *fitViewButton;
    QSpacerItem *toolBarSpacer;
    QPushButton *startSolveButton;
    QSplitter *workspaceVerticalSplitter;
    ModelViewport *modelViewport;
    QTabWidget *monitorTabs;
    QWidget *monitorPage;
    QVBoxLayout *monitorLayout;
    QLabel *incrementLabel;
    QProgressBar *solveProgress;
    QHBoxLayout *monitorStatusLayout;
    QLabel *convergenceLabel;
    QSpacerItem *monitorSpacer;
    QPushButton *stopButton;
    QWidget *logPage;
    QVBoxLayout *logLayout;
    QPlainTextEdit *logEdit;
    QFrame *statusFrame;
    QHBoxLayout *statusLayout;
    QLabel *loadStatisticsLabel;
    QSpacerItem *statusSpacer;
    QLabel *readyLabel;

    void setupUi(QMainWindow *YQYClass)
    {
        if (YQYClass->objectName().isEmpty())
            YQYClass->setObjectName("YQYClass");
        YQYClass->resize(1500, 850);
        YQYClass->setMinimumSize(QSize(760, 520));
        centralWidget = new QWidget(YQYClass);
        centralWidget->setObjectName("centralWidget");
        rootLayout = new QVBoxLayout(centralWidget);
        rootLayout->setSpacing(0);
        rootLayout->setObjectName("rootLayout");
        rootLayout->setContentsMargins(0, 0, 0, 0);
        appHeader = new QFrame(centralWidget);
        appHeader->setObjectName("appHeader");
        appHeader->setMinimumSize(QSize(0, 58));
        appHeader->setMaximumSize(QSize(16777215, 58));
        headerLayout = new QHBoxLayout(appHeader);
        headerLayout->setSpacing(12);
        headerLayout->setObjectName("headerLayout");
        headerLayout->setContentsMargins(18, -1, 18, -1);
        logoLabel = new QLabel(appHeader);
        logoLabel->setObjectName("logoLabel");
        logoLabel->setMinimumSize(QSize(32, 32));
        logoLabel->setMaximumSize(QSize(32, 32));
        logoLabel->setAlignment(Qt::AlignCenter);

        headerLayout->addWidget(logoLabel);

        appTitleLabel = new QLabel(appHeader);
        appTitleLabel->setObjectName("appTitleLabel");

        headerLayout->addWidget(appTitleLabel);

        headerSpacer = new QSpacerItem(80, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        headerLayout->addItem(headerSpacer);

        themeLabel = new QLabel(appHeader);
        themeLabel->setObjectName("themeLabel");

        headerLayout->addWidget(themeLabel);

        themeComboBox = new QComboBox(appHeader);
        themeComboBox->addItem(QString());
        themeComboBox->addItem(QString());
        themeComboBox->addItem(QString());
        themeComboBox->addItem(QString());
        themeComboBox->setObjectName("themeComboBox");
        themeComboBox->setMinimumSize(QSize(150, 34));

        headerLayout->addWidget(themeComboBox);


        rootLayout->addWidget(appHeader);

        bodyLayout = new QHBoxLayout();
        bodyLayout->setSpacing(0);
        bodyLayout->setObjectName("bodyLayout");
        bodyLayout->setContentsMargins(0, 0, 0, 0);
        navigationRail = new QFrame(centralWidget);
        navigationRail->setObjectName("navigationRail");
        navigationRail->setMinimumSize(QSize(84, 0));
        navigationRail->setMaximumSize(QSize(84, 16777215));
        navigationLayout = new QVBoxLayout(navigationRail);
        navigationLayout->setSpacing(8);
        navigationLayout->setObjectName("navigationLayout");
        navigationLayout->setContentsMargins(8, 14, 8, 14);
        modelButton = new NavigationButton(navigationRail);
        modelButton->setObjectName("modelButton");
        modelButton->setMinimumSize(QSize(66, 58));
        modelButton->setCheckable(true);
        modelButton->setChecked(true);
        modelButton->setAutoExclusive(true);

        navigationLayout->addWidget(modelButton);

        propertyButton = new NavigationButton(navigationRail);
        propertyButton->setObjectName("propertyButton");
        propertyButton->setMinimumSize(QSize(66, 58));
        propertyButton->setCheckable(true);
        propertyButton->setAutoExclusive(true);

        navigationLayout->addWidget(propertyButton);

        conductorButton = new NavigationButton(navigationRail);
        conductorButton->setObjectName("conductorButton");
        conductorButton->setMinimumSize(QSize(66, 58));
        conductorButton->setCheckable(true);
        conductorButton->setAutoExclusive(true);

        navigationLayout->addWidget(conductorButton);

        analysisButton = new NavigationButton(navigationRail);
        analysisButton->setObjectName("analysisButton");
        analysisButton->setMinimumSize(QSize(66, 58));
        analysisButton->setCheckable(true);
        analysisButton->setAutoExclusive(true);

        navigationLayout->addWidget(analysisButton);

        solveButton = new NavigationButton(navigationRail);
        solveButton->setObjectName("solveButton");
        solveButton->setMinimumSize(QSize(66, 58));
        solveButton->setCheckable(true);
        solveButton->setAutoExclusive(true);

        navigationLayout->addWidget(solveButton);

        resultButton = new NavigationButton(navigationRail);
        resultButton->setObjectName("resultButton");
        resultButton->setMinimumSize(QSize(66, 58));
        resultButton->setCheckable(true);
        resultButton->setAutoExclusive(true);

        navigationLayout->addWidget(resultButton);

        navigationSpacer = new QSpacerItem(20, 160, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        navigationLayout->addItem(navigationSpacer);

        settingsButton = new NavigationButton(navigationRail);
        settingsButton->setObjectName("settingsButton");
        settingsButton->setMinimumSize(QSize(66, 58));
        settingsButton->setCheckable(true);
        settingsButton->setAutoExclusive(true);

        navigationLayout->addWidget(settingsButton);


        bodyLayout->addWidget(navigationRail);

        mainSplitter = new QSplitter(centralWidget);
        mainSplitter->setObjectName("mainSplitter");
        mainSplitter->setOrientation(Qt::Horizontal);
        mainSplitter->setChildrenCollapsible(false);
        projectPanel = new QFrame(mainSplitter);
        projectPanel->setObjectName("projectPanel");
        projectPanel->setMinimumSize(QSize(320, 0));
        projectPanel->setMaximumSize(QSize(520, 16777215));
        projectLayout = new QVBoxLayout(projectPanel);
        projectLayout->setSpacing(10);
        projectLayout->setObjectName("projectLayout");
        projectLayout->setContentsMargins(14, 14, 14, 14);
        projectTitleLabel = new QLabel(projectPanel);
        projectTitleLabel->setObjectName("projectTitleLabel");

        projectLayout->addWidget(projectTitleLabel);

        moduleStack = new QStackedWidget(projectPanel);
        moduleStack->setObjectName("moduleStack");
        modelModulePage = new QWidget();
        modelModulePage->setObjectName("modelModulePage");
        modelModuleLayout = new QVBoxLayout(modelModulePage);
        modelModuleLayout->setSpacing(10);
        modelModuleLayout->setObjectName("modelModuleLayout");
        modelModuleLayout->setContentsMargins(0, 0, 0, 0);
        modelModuleHint = new QLabel(modelModulePage);
        modelModuleHint->setObjectName("modelModuleHint");
        modelModuleHint->setWordWrap(true);

        modelModuleLayout->addWidget(modelModuleHint);

        modelSummaryCard = new QFrame(modelModulePage);
        modelSummaryCard->setObjectName("modelSummaryCard");
        modelSummaryLayout = new QVBoxLayout(modelSummaryCard);
        modelSummaryLayout->setSpacing(8);
        modelSummaryLayout->setObjectName("modelSummaryLayout");
        modelSummaryLayout->setContentsMargins(12, 12, 12, 12);
        modelFileValue = new QLabel(modelSummaryCard);
        modelFileValue->setObjectName("modelFileValue");
        modelFileValue->setWordWrap(true);

        modelSummaryLayout->addWidget(modelFileValue);

        modelNodeValue = new QLabel(modelSummaryCard);
        modelNodeValue->setObjectName("modelNodeValue");

        modelSummaryLayout->addWidget(modelNodeValue);

        modelElementValue = new QLabel(modelSummaryCard);
        modelElementValue->setObjectName("modelElementValue");

        modelSummaryLayout->addWidget(modelElementValue);

        modelPropertyValue = new QLabel(modelSummaryCard);
        modelPropertyValue->setObjectName("modelPropertyValue");

        modelSummaryLayout->addWidget(modelPropertyValue);


        modelModuleLayout->addWidget(modelSummaryCard);

        modelDataTree = new QTreeWidget(modelModulePage);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setText(0, QString::fromUtf8("1"));
        modelDataTree->setHeaderItem(__qtreewidgetitem);
        modelDataTree->setObjectName("modelDataTree");
        modelDataTree->setHeaderHidden(true);
        modelDataTree->setIndentation(18);
        modelDataTree->setAnimated(true);

        modelModuleLayout->addWidget(modelDataTree);

        modelImportButton = new QPushButton(modelModulePage);
        modelImportButton->setObjectName("modelImportButton");

        modelModuleLayout->addWidget(modelImportButton);

        modelFitButton = new QPushButton(modelModulePage);
        modelFitButton->setObjectName("modelFitButton");

        modelModuleLayout->addWidget(modelFitButton);

        modelModuleSpacer = new QSpacerItem(20, 80, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        modelModuleLayout->addItem(modelModuleSpacer);

        moduleStack->addWidget(modelModulePage);
        analysisModulePage = new QWidget();
        analysisModulePage->setObjectName("analysisModulePage");
        analysisModuleLayout = new QVBoxLayout(analysisModulePage);
        analysisModuleLayout->setSpacing(10);
        analysisModuleLayout->setObjectName("analysisModuleLayout");
        analysisModuleLayout->setContentsMargins(0, 0, 0, 0);
        analysisModuleHint = new QLabel(analysisModulePage);
        analysisModuleHint->setObjectName("analysisModuleHint");
        analysisModuleHint->setWordWrap(true);

        analysisModuleLayout->addWidget(analysisModuleHint);

        analysisTree = new QTreeWidget(analysisModulePage);
        QTreeWidgetItem *__qtreewidgetitem1 = new QTreeWidgetItem();
        __qtreewidgetitem1->setText(0, QString::fromUtf8("1"));
        analysisTree->setHeaderItem(__qtreewidgetitem1);
        analysisTree->setObjectName("analysisTree");
        analysisTree->setHeaderHidden(true);
        analysisTree->setAnimated(true);

        analysisModuleLayout->addWidget(analysisTree);

        moduleStack->addWidget(analysisModulePage);
        solveModulePage = new QWidget();
        solveModulePage->setObjectName("solveModulePage");
        solveModuleLayout = new QVBoxLayout(solveModulePage);
        solveModuleLayout->setSpacing(10);
        solveModuleLayout->setObjectName("solveModuleLayout");
        solveModuleLayout->setContentsMargins(0, 0, 0, 0);
        solveReadinessLabel = new QLabel(solveModulePage);
        solveReadinessLabel->setObjectName("solveReadinessLabel");
        solveReadinessLabel->setWordWrap(true);

        solveModuleLayout->addWidget(solveReadinessLabel);

        solveBatchActionsLayout = new QHBoxLayout();
        solveBatchActionsLayout->setSpacing(6);
        solveBatchActionsLayout->setObjectName("solveBatchActionsLayout");
        solveCasesButton = new QPushButton(solveModulePage);
        solveCasesButton->setObjectName("solveCasesButton");
        solveCasesButton->setMinimumSize(QSize(0, 54));

        solveBatchActionsLayout->addWidget(solveCasesButton);

        solveAllCasesButton = new QPushButton(solveModulePage);
        solveAllCasesButton->setObjectName("solveAllCasesButton");
        solveAllCasesButton->setMinimumSize(QSize(0, 54));
        solveAllCasesButton->setEnabled(false);

        solveBatchActionsLayout->addWidget(solveAllCasesButton);

        solveRestartAllCasesButton = new QPushButton(solveModulePage);
        solveRestartAllCasesButton->setObjectName("solveRestartAllCasesButton");
        solveRestartAllCasesButton->setMinimumSize(QSize(0, 54));
        solveRestartAllCasesButton->setEnabled(false);

        solveBatchActionsLayout->addWidget(solveRestartAllCasesButton);


        solveModuleLayout->addLayout(solveBatchActionsLayout);

        solveStepTree = new QTreeWidget(solveModulePage);
        QTreeWidgetItem *__qtreewidgetitem2 = new QTreeWidgetItem();
        __qtreewidgetitem2->setText(0, QString::fromUtf8("1"));
        solveStepTree->setHeaderItem(__qtreewidgetitem2);
        solveStepTree->setObjectName("solveStepTree");
        solveStepTree->setHeaderHidden(true);
        solveStepTree->setAnimated(true);

        solveModuleLayout->addWidget(solveStepTree);

        solvePageStartButton = new QPushButton(solveModulePage);
        solvePageStartButton->setObjectName("solvePageStartButton");
        solvePageStartButton->setEnabled(false);

        solveModuleLayout->addWidget(solvePageStartButton);

        solveModuleSpacer = new QSpacerItem(20, 80, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        solveModuleLayout->addItem(solveModuleSpacer);

        moduleStack->addWidget(solveModulePage);
        resultModulePage = new QWidget();
        resultModulePage->setObjectName("resultModulePage");
        resultModuleLayout = new QVBoxLayout(resultModulePage);
        resultModuleLayout->setSpacing(10);
        resultModuleLayout->setObjectName("resultModuleLayout");
        resultModuleLayout->setContentsMargins(0, 0, 0, 0);
        resultModuleHint = new QLabel(resultModulePage);
        resultModuleHint->setObjectName("resultModuleHint");
        resultModuleHint->setWordWrap(true);

        resultModuleLayout->addWidget(resultModuleHint);

        openH5Button = new QPushButton(resultModulePage);
        openH5Button->setObjectName("openH5Button");

        resultModuleLayout->addWidget(openH5Button);

        resultFileLabel = new QLabel(resultModulePage);
        resultFileLabel->setObjectName("resultFileLabel");
        resultFileLabel->setWordWrap(true);

        resultModuleLayout->addWidget(resultFileLabel);

        resultTree = new QTreeWidget(resultModulePage);
        QTreeWidgetItem *__qtreewidgetitem3 = new QTreeWidgetItem();
        __qtreewidgetitem3->setText(0, QString::fromUtf8("1"));
        resultTree->setHeaderItem(__qtreewidgetitem3);
        resultTree->setObjectName("resultTree");
        resultTree->setHeaderHidden(true);
        resultTree->setAnimated(true);

        resultModuleLayout->addWidget(resultTree);

        moduleStack->addWidget(resultModulePage);
        settingsModulePage = new QWidget();
        settingsModulePage->setObjectName("settingsModulePage");
        settingsModuleLayout = new QVBoxLayout(settingsModulePage);
        settingsModuleLayout->setSpacing(10);
        settingsModuleLayout->setObjectName("settingsModuleLayout");
        settingsModuleLayout->setContentsMargins(0, 0, 0, 0);
        settingsModuleHint = new QLabel(settingsModulePage);
        settingsModuleHint->setObjectName("settingsModuleHint");
        settingsModuleHint->setWordWrap(true);

        settingsModuleLayout->addWidget(settingsModuleHint);

        settingsModuleSpacer = new QSpacerItem(20, 80, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        settingsModuleLayout->addItem(settingsModuleSpacer);

        moduleStack->addWidget(settingsModulePage);

        projectLayout->addWidget(moduleStack);

        mainSplitter->addWidget(projectPanel);
        workspacePanel = new QFrame(mainSplitter);
        workspacePanel->setObjectName("workspacePanel");
        workspacePanel->setMinimumSize(QSize(560, 0));
        workspaceLayout = new QVBoxLayout(workspacePanel);
        workspaceLayout->setSpacing(10);
        workspaceLayout->setObjectName("workspaceLayout");
        workspaceLayout->setContentsMargins(12, 10, 12, 10);
        documentTabBar = new QTabBar(workspacePanel);
        documentTabBar->setObjectName("documentTabBar");
        documentTabBar->setCurrentIndex(-1);
        documentTabBar->setTabsClosable(true);
        documentTabBar->setMovable(true);
        documentTabBar->setDrawBase(false);
        documentTabBar->setExpanding(false);

        workspaceLayout->addWidget(documentTabBar);

        toolbarScrollArea = new QScrollArea(workspacePanel);
        toolbarScrollArea->setObjectName("toolbarScrollArea");
        toolbarScrollArea->setFrameShape(QFrame::NoFrame);
        toolbarScrollArea->setWidgetResizable(true);
        toolbarScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        toolbarScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        toolbarScrollArea->setAlignment(Qt::AlignLeft|Qt::AlignTop);
        toolBarCard = new QFrame();
        toolBarCard->setObjectName("toolBarCard");
        toolBarLayout = new QHBoxLayout(toolBarCard);
        toolBarLayout->setSpacing(5);
        toolBarLayout->setObjectName("toolBarLayout");
        toolBarLayout->setContentsMargins(8, 6, 8, 6);
        importButton = new QPushButton(toolBarCard);
        importButton->setObjectName("importButton");

        toolBarLayout->addWidget(importButton);

        undoButton = new QPushButton(toolBarCard);
        undoButton->setObjectName("undoButton");

        toolBarLayout->addWidget(undoButton);

        selectModeButton = new QToolButton(toolBarCard);
        selectModeButton->setObjectName("selectModeButton");
        selectModeButton->setCheckable(true);
        selectModeButton->setChecked(true);

        toolBarLayout->addWidget(selectModeButton);

        rotateModeButton = new QToolButton(toolBarCard);
        rotateModeButton->setObjectName("rotateModeButton");
        rotateModeButton->setCheckable(true);

        toolBarLayout->addWidget(rotateModeButton);

        panModeButton = new QToolButton(toolBarCard);
        panModeButton->setObjectName("panModeButton");
        panModeButton->setCheckable(true);

        toolBarLayout->addWidget(panModeButton);

        zoomModeButton = new QToolButton(toolBarCard);
        zoomModeButton->setObjectName("zoomModeButton");
        zoomModeButton->setCheckable(true);

        toolBarLayout->addWidget(zoomModeButton);

        showNodesButton = new QToolButton(toolBarCard);
        showNodesButton->setObjectName("showNodesButton");
        showNodesButton->setCheckable(true);
        showNodesButton->setChecked(true);

        toolBarLayout->addWidget(showNodesButton);

        showElementsButton = new QToolButton(toolBarCard);
        showElementsButton->setObjectName("showElementsButton");
        showElementsButton->setCheckable(true);
        showElementsButton->setChecked(true);

        toolBarLayout->addWidget(showElementsButton);

        showSolidButton = new QToolButton(toolBarCard);
        showSolidButton->setObjectName("showSolidButton");
        showSolidButton->setCheckable(true);

        toolBarLayout->addWidget(showSolidButton);

        showNodeIdsButton = new QToolButton(toolBarCard);
        showNodeIdsButton->setObjectName("showNodeIdsButton");
        showNodeIdsButton->setCheckable(true);

        toolBarLayout->addWidget(showNodeIdsButton);

        showElementIdsButton = new QToolButton(toolBarCard);
        showElementIdsButton->setObjectName("showElementIdsButton");
        showElementIdsButton->setCheckable(true);

        toolBarLayout->addWidget(showElementIdsButton);

        frontViewButton = new QToolButton(toolBarCard);
        frontViewButton->setObjectName("frontViewButton");

        toolBarLayout->addWidget(frontViewButton);

        backViewButton = new QToolButton(toolBarCard);
        backViewButton->setObjectName("backViewButton");

        toolBarLayout->addWidget(backViewButton);

        leftViewButton = new QToolButton(toolBarCard);
        leftViewButton->setObjectName("leftViewButton");

        toolBarLayout->addWidget(leftViewButton);

        rightViewButton = new QToolButton(toolBarCard);
        rightViewButton->setObjectName("rightViewButton");

        toolBarLayout->addWidget(rightViewButton);

        topViewButton = new QToolButton(toolBarCard);
        topViewButton->setObjectName("topViewButton");

        toolBarLayout->addWidget(topViewButton);

        bottomViewButton = new QToolButton(toolBarCard);
        bottomViewButton->setObjectName("bottomViewButton");

        toolBarLayout->addWidget(bottomViewButton);

        fitViewButton = new QToolButton(toolBarCard);
        fitViewButton->setObjectName("fitViewButton");

        toolBarLayout->addWidget(fitViewButton);

        toolBarSpacer = new QSpacerItem(20, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        toolBarLayout->addItem(toolBarSpacer);

        startSolveButton = new QPushButton(toolBarCard);
        startSolveButton->setObjectName("startSolveButton");

        toolBarLayout->addWidget(startSolveButton);

        toolbarScrollArea->setWidget(toolBarCard);

        workspaceLayout->addWidget(toolbarScrollArea);

        workspaceVerticalSplitter = new QSplitter(workspacePanel);
        workspaceVerticalSplitter->setObjectName("workspaceVerticalSplitter");
        workspaceVerticalSplitter->setOrientation(Qt::Vertical);
        workspaceVerticalSplitter->setChildrenCollapsible(false);
        modelViewport = new ModelViewport(workspaceVerticalSplitter);
        modelViewport->setObjectName("modelViewport");
        modelViewport->setMinimumSize(QSize(280, 160));
        workspaceVerticalSplitter->addWidget(modelViewport);
        monitorTabs = new QTabWidget(workspaceVerticalSplitter);
        monitorTabs->setObjectName("monitorTabs");
        monitorTabs->setMinimumSize(QSize(0, 100));
        monitorPage = new QWidget();
        monitorPage->setObjectName("monitorPage");
        monitorLayout = new QVBoxLayout(monitorPage);
        monitorLayout->setObjectName("monitorLayout");
        monitorLayout->setContentsMargins(12, 8, 12, 8);
        incrementLabel = new QLabel(monitorPage);
        incrementLabel->setObjectName("incrementLabel");

        monitorLayout->addWidget(incrementLabel);

        solveProgress = new QProgressBar(monitorPage);
        solveProgress->setObjectName("solveProgress");
        solveProgress->setValue(0);
        solveProgress->setTextVisible(true);

        monitorLayout->addWidget(solveProgress);

        monitorStatusLayout = new QHBoxLayout();
        monitorStatusLayout->setObjectName("monitorStatusLayout");
        convergenceLabel = new QLabel(monitorPage);
        convergenceLabel->setObjectName("convergenceLabel");

        monitorStatusLayout->addWidget(convergenceLabel);

        monitorSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        monitorStatusLayout->addItem(monitorSpacer);

        stopButton = new QPushButton(monitorPage);
        stopButton->setObjectName("stopButton");

        monitorStatusLayout->addWidget(stopButton);


        monitorLayout->addLayout(monitorStatusLayout);

        monitorTabs->addTab(monitorPage, QString());
        logPage = new QWidget();
        logPage->setObjectName("logPage");
        logLayout = new QVBoxLayout(logPage);
        logLayout->setObjectName("logLayout");
        logEdit = new QPlainTextEdit(logPage);
        logEdit->setObjectName("logEdit");
        logEdit->setReadOnly(true);

        logLayout->addWidget(logEdit);

        monitorTabs->addTab(logPage, QString());
        workspaceVerticalSplitter->addWidget(monitorTabs);

        workspaceLayout->addWidget(workspaceVerticalSplitter);

        mainSplitter->addWidget(workspacePanel);

        bodyLayout->addWidget(mainSplitter);


        rootLayout->addLayout(bodyLayout);

        statusFrame = new QFrame(centralWidget);
        statusFrame->setObjectName("statusFrame");
        statusFrame->setMinimumSize(QSize(0, 32));
        statusFrame->setMaximumSize(QSize(16777215, 32));
        statusLayout = new QHBoxLayout(statusFrame);
        statusLayout->setSpacing(10);
        statusLayout->setObjectName("statusLayout");
        statusLayout->setContentsMargins(16, -1, 16, -1);
        loadStatisticsLabel = new QLabel(statusFrame);
        loadStatisticsLabel->setObjectName("loadStatisticsLabel");

        statusLayout->addWidget(loadStatisticsLabel);

        statusSpacer = new QSpacerItem(80, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        statusLayout->addItem(statusSpacer);

        readyLabel = new QLabel(statusFrame);
        readyLabel->setObjectName("readyLabel");
        readyLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        statusLayout->addWidget(readyLabel);


        rootLayout->addWidget(statusFrame);

        YQYClass->setCentralWidget(centralWidget);

        retranslateUi(YQYClass);

        moduleStack->setCurrentIndex(0);
        monitorTabs->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(YQYClass);
    } // setupUi

    void retranslateUi(QMainWindow *YQYClass)
    {
        YQYClass->setWindowTitle(QCoreApplication::translate("YQYClass", "YQY CAE", nullptr));
        logoLabel->setText(QCoreApplication::translate("YQYClass", "Y", nullptr));
        appTitleLabel->setText(QCoreApplication::translate("YQYClass", "YQY CAE", nullptr));
        themeLabel->setText(QCoreApplication::translate("YQYClass", "\344\270\273\351\242\230", nullptr));
        themeComboBox->setItemText(0, QCoreApplication::translate("YQYClass", "A \302\267 \351\273\221\350\211\262", nullptr));
        themeComboBox->setItemText(1, QCoreApplication::translate("YQYClass", "B \302\267 \347\264\253\350\211\262", nullptr));
        themeComboBox->setItemText(2, QCoreApplication::translate("YQYClass", "C \302\267 \347\273\277\350\211\262", nullptr));
        themeComboBox->setItemText(3, QCoreApplication::translate("YQYClass", "D \302\267 \346\265\205\350\211\262", nullptr));

        modelButton->setText(QCoreApplication::translate("YQYClass", "\346\250\241\345\236\213", nullptr));
        propertyButton->setText(QCoreApplication::translate("YQYClass", "\345\261\236\346\200\247", nullptr));
        conductorButton->setText(QCoreApplication::translate("YQYClass", "\345\257\274\347\272\277", nullptr));
        analysisButton->setText(QCoreApplication::translate("YQYClass", "\345\210\206\346\236\220", nullptr));
        solveButton->setText(QCoreApplication::translate("YQYClass", "\346\261\202\350\247\243", nullptr));
        resultButton->setText(QCoreApplication::translate("YQYClass", "\347\273\223\346\236\234", nullptr));
        settingsButton->setText(QCoreApplication::translate("YQYClass", "\350\256\276\347\275\256", nullptr));
        projectTitleLabel->setText(QCoreApplication::translate("YQYClass", "\346\250\241\345\236\213\346\225\260\346\215\256", nullptr));
        modelModuleHint->setText(QCoreApplication::translate("YQYClass", "\346\250\241\345\236\213\346\225\260\346\215\256\344\270\216\346\230\276\347\244\272\346\216\247\345\210\266", nullptr));
        modelFileValue->setText(QCoreApplication::translate("YQYClass", "\345\260\232\346\234\252\345\212\240\350\275\275\346\250\241\345\236\213", nullptr));
        modelNodeValue->setText(QCoreApplication::translate("YQYClass", "\350\212\202\347\202\271 --", nullptr));
        modelElementValue->setText(QCoreApplication::translate("YQYClass", "\345\215\225\345\205\203 --", nullptr));
        modelPropertyValue->setText(QCoreApplication::translate("YQYClass", "\346\235\220\346\226\231 -- \302\267 \346\210\252\351\235\242 --", nullptr));
        modelImportButton->setText(QCoreApplication::translate("YQYClass", "\345\257\274\345\205\245\346\250\241\345\236\213", nullptr));
        modelFitButton->setText(QCoreApplication::translate("YQYClass", "\351\200\202\345\272\224\347\252\227\345\217\243", nullptr));
        analysisModuleHint->setText(QCoreApplication::translate("YQYClass", "\345\210\206\346\236\220\346\255\245\343\200\201\350\275\275\350\215\267\344\270\216\350\276\271\347\225\214\346\235\241\344\273\266", nullptr));
        solveReadinessLabel->setText(QCoreApplication::translate("YQYClass", "\350\257\267\345\205\210\345\212\240\350\275\275\345\214\205\345\220\253\345\210\206\346\236\220\346\255\245\347\232\204\346\250\241\345\236\213", nullptr));
        solveCasesButton->setText(QCoreApplication::translate("YQYClass", "\345\205\250\351\203\250\345\210\206\346\236\220\346\255\245 \302\267 0", nullptr));
        solveAllCasesButton->setText(QCoreApplication::translate("YQYClass", "\345\205\250\351\203\250\350\256\241\347\256\227 \302\267 0", nullptr));
        solveRestartAllCasesButton->setText(QCoreApplication::translate("YQYClass", "\351\207\215\346\226\260\350\256\241\347\256\227 \302\267 0", nullptr));
        solvePageStartButton->setText(QCoreApplication::translate("YQYClass", "\350\277\220\350\241\214\345\205\250\351\203\250\345\210\206\346\236\220\346\255\245", nullptr));
        resultModuleHint->setText(QCoreApplication::translate("YQYClass", "\347\273\223\346\236\234\345\220\216\345\244\204\347\220\206\346\230\276\347\244\272", nullptr));
        openH5Button->setText(QCoreApplication::translate("YQYClass", "\346\211\223\345\274\200 H5 \347\273\223\346\236\234\346\226\207\344\273\266", nullptr));
        resultFileLabel->setText(QCoreApplication::translate("YQYClass", "\345\260\232\346\234\252\346\211\223\345\274\200\347\273\223\346\236\234\346\226\207\344\273\266", nullptr));
        settingsModuleHint->setText(QCoreApplication::translate("YQYClass", "\347\225\214\351\235\242\350\256\276\347\275\256\344\277\235\345\255\230\345\234\250\346\234\254\346\234\272\357\274\214\344\270\273\351\242\230\345\217\257\345\234\250\347\252\227\345\217\243\351\241\266\351\203\250\345\210\207\346\215\242", nullptr));
        importButton->setText(QString());
        undoButton->setText(QString());
        selectModeButton->setText(QString());
        rotateModeButton->setText(QString());
        panModeButton->setText(QString());
        zoomModeButton->setText(QString());
        showNodesButton->setText(QString());
        showElementsButton->setText(QString());
        showSolidButton->setText(QString());
        showNodeIdsButton->setText(QString());
        showElementIdsButton->setText(QString());
        frontViewButton->setText(QString());
#if QT_CONFIG(tooltip)
        frontViewButton->setToolTip(QCoreApplication::translate("YQYClass", "\345\211\215\350\247\206\345\233\276", nullptr));
#endif // QT_CONFIG(tooltip)
        backViewButton->setText(QString());
#if QT_CONFIG(tooltip)
        backViewButton->setToolTip(QCoreApplication::translate("YQYClass", "\345\220\216\350\247\206\345\233\276", nullptr));
#endif // QT_CONFIG(tooltip)
        leftViewButton->setText(QString());
#if QT_CONFIG(tooltip)
        leftViewButton->setToolTip(QCoreApplication::translate("YQYClass", "\345\267\246\350\247\206\345\233\276", nullptr));
#endif // QT_CONFIG(tooltip)
        rightViewButton->setText(QString());
#if QT_CONFIG(tooltip)
        rightViewButton->setToolTip(QCoreApplication::translate("YQYClass", "\345\217\263\350\247\206\345\233\276", nullptr));
#endif // QT_CONFIG(tooltip)
        topViewButton->setText(QString());
#if QT_CONFIG(tooltip)
        topViewButton->setToolTip(QCoreApplication::translate("YQYClass", "\351\241\266\350\247\206\345\233\276", nullptr));
#endif // QT_CONFIG(tooltip)
        bottomViewButton->setText(QString());
#if QT_CONFIG(tooltip)
        bottomViewButton->setToolTip(QCoreApplication::translate("YQYClass", "\345\272\225\350\247\206\345\233\276", nullptr));
#endif // QT_CONFIG(tooltip)
        fitViewButton->setText(QString());
#if QT_CONFIG(tooltip)
        fitViewButton->setToolTip(QCoreApplication::translate("YQYClass", "\351\200\202\345\272\224\347\252\227\345\217\243", nullptr));
#endif // QT_CONFIG(tooltip)
        startSolveButton->setText(QString());
        incrementLabel->setText(QCoreApplication::translate("YQYClass", "\345\260\232\346\234\252\345\274\200\345\247\213\346\261\202\350\247\243", nullptr));
        convergenceLabel->setText(QCoreApplication::translate("YQYClass", "\345\212\240\350\275\275\346\250\241\345\236\213\345\271\266\351\205\215\347\275\256\345\210\206\346\236\220\346\255\245\345\220\216\345\217\257\345\274\200\345\247\213\346\261\202\350\247\243", nullptr));
        stopButton->setText(QCoreApplication::translate("YQYClass", "\345\201\234\346\255\242", nullptr));
        monitorTabs->setTabText(monitorTabs->indexOf(monitorPage), QCoreApplication::translate("YQYClass", "\346\261\202\350\247\243\347\233\221\350\247\206\345\231\250", nullptr));
        monitorTabs->setTabText(monitorTabs->indexOf(logPage), QCoreApplication::translate("YQYClass", "\346\227\245\345\277\227", nullptr));
        loadStatisticsLabel->setText(QCoreApplication::translate("YQYClass", "\346\250\241\345\236\213\350\257\273\345\217\226  \346\200\273\350\256\241 0  \302\267  \346\210\220\345\212\237 0  \302\267  \345\244\261\350\264\245 0", nullptr));
        readyLabel->setText(QCoreApplication::translate("YQYClass", "\347\255\211\345\276\205\345\257\274\345\205\245\346\250\241\345\236\213", nullptr));
    } // retranslateUi

};

namespace Ui {
    class YQYClass: public Ui_YQYClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_YQY_H
