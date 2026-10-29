#include "Widgets/SettingsPanel.h"
#include <QVariant>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QFrame>
#include <QLabel>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>
#include <QScrollArea>

SettingsPanel::SettingsPanel(QWidget* parent)
    : QWidget(parent)
{
    initializeUi();
    initializeAppearance();
}

SettingsPanel::~SettingsPanel() = default;

QSpinBox* SettingsPanel::concurrencySpin() const
{
    return m_concurrencySpin;
}

QSpinBox* SettingsPanel::assemblyThreadsSpin() const
{
    return m_assemblyThreadsSpin;
}

QCheckBox* SettingsPanel::gpuSolverCheckBox() const
{
    return m_gpuSolverCheckBox;
}

QComboBox* SettingsPanel::linearSolverModeCombo() const
{
    return m_linearSolverModeCombo;
}

QSpinBox* SettingsPanel::resultBatchFramesSpin() const
{
    return m_resultBatchFramesSpin;
}

QCheckBox* SettingsPanel::backgroundResultWriteCheckBox() const
{
    return m_backgroundResultWriteCheckBox;
}

QComboBox* SettingsPanel::nodeLabelModeCombo() const
{
    return m_nodeLabelModeCombo;
}

QLabel* SettingsPanel::statusLabel() const
{
    return m_statusLabel;
}

void SettingsPanel::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("SettingsPanelClass");
    }
    QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(this->sizePolicy().hasHeightForWidth());
    setSizePolicy(sizePolicy);
    m_rootLayout = new QVBoxLayout(this);
    m_rootLayout->setObjectName("rootLayout");
    m_rootLayout->setContentsMargins(0, 0, 0, 0);
    m_infoCard = new QFrame(this);
    m_infoCard->setObjectName("infoCard");
    m_cardLayout = new QVBoxLayout(m_infoCard);
    m_cardLayout->setSpacing(10);
    m_cardLayout->setObjectName("cardLayout");
    m_cardLayout->setContentsMargins(12, 12, 12, 12);
    m_titleLabel = new QLabel(m_infoCard);
    m_titleLabel->setObjectName("titleLabel");

    m_cardLayout->addWidget(m_titleLabel);

    m_descriptionLabel = new QLabel(m_infoCard);
    m_descriptionLabel->setObjectName("descriptionLabel");
    m_descriptionLabel->setWordWrap(true);

    m_cardLayout->addWidget(m_descriptionLabel);

    m_concurrencyForm = new QFormLayout();
    m_concurrencyForm->setObjectName("concurrencyForm");
    m_concurrencyForm->setHorizontalSpacing(12);
    m_concurrencyForm->setVerticalSpacing(8);
    QLabel* concurrencyLabel = new QLabel(m_infoCard);
    concurrencyLabel->setObjectName("concurrencyLabel");

    m_concurrencyForm->setWidget(0, QFormLayout::LabelRole, concurrencyLabel);

    m_concurrencySpin = new QSpinBox(m_infoCard);
    m_concurrencySpin->setObjectName("concurrencySpin");
    m_concurrencySpin->setMinimumSize(QSize(150, 0));
    m_concurrencySpin->setMinimum(1);
    m_concurrencySpin->setMaximum(12);
    m_concurrencySpin->setValue(5);

    m_concurrencyForm->setWidget(0, QFormLayout::FieldRole, m_concurrencySpin);

    QLabel* assemblyThreadsLabel = new QLabel(m_infoCard);
    assemblyThreadsLabel->setObjectName("assemblyThreadsLabel");

    m_concurrencyForm->setWidget(1, QFormLayout::LabelRole, assemblyThreadsLabel);

    m_assemblyThreadsSpin = new QSpinBox(m_infoCard);
    m_assemblyThreadsSpin->setObjectName("assemblyThreadsSpin");
    m_assemblyThreadsSpin->setMinimumSize(QSize(150, 0));
    m_assemblyThreadsSpin->setMinimum(1);
    m_assemblyThreadsSpin->setMaximum(12);
    m_assemblyThreadsSpin->setValue(1);

    m_concurrencyForm->setWidget(1, QFormLayout::FieldRole, m_assemblyThreadsSpin);

    QLabel* linearSolverModeLabel = new QLabel(m_infoCard);
    linearSolverModeLabel->setObjectName("linearSolverModeLabel");

    m_concurrencyForm->setWidget(2, QFormLayout::LabelRole, linearSolverModeLabel);

    m_linearSolverModeCombo = new QComboBox(m_infoCard);
    m_linearSolverModeCombo->addItem(QString());
    m_linearSolverModeCombo->addItem(QString());
    m_linearSolverModeCombo->addItem(QString());
    m_linearSolverModeCombo->addItem(QString());
    m_linearSolverModeCombo->addItem(QString());
    m_linearSolverModeCombo->addItem(QString());
    m_linearSolverModeCombo->setObjectName("linearSolverModeCombo");
    m_linearSolverModeCombo->setMinimumSize(QSize(150, 0));

    m_concurrencyForm->setWidget(2, QFormLayout::FieldRole, m_linearSolverModeCombo);

    QLabel* gpuSolverLabel = new QLabel(m_infoCard);
    gpuSolverLabel->setObjectName("gpuSolverLabel");

    m_concurrencyForm->setWidget(3, QFormLayout::LabelRole, gpuSolverLabel);

    m_gpuSolverCheckBox = new QCheckBox(m_infoCard);
    m_gpuSolverCheckBox->setObjectName("gpuSolverCheckBox");

    m_concurrencyForm->setWidget(3, QFormLayout::FieldRole, m_gpuSolverCheckBox);

    QLabel* resultBatchFramesLabel = new QLabel(m_infoCard);
    resultBatchFramesLabel->setObjectName("resultBatchFramesLabel");

    m_concurrencyForm->setWidget(4, QFormLayout::LabelRole, resultBatchFramesLabel);

    m_resultBatchFramesSpin = new QSpinBox(m_infoCard);
    m_resultBatchFramesSpin->setObjectName("resultBatchFramesSpin");
    m_resultBatchFramesSpin->setMinimumSize(QSize(150, 0));
    m_resultBatchFramesSpin->setMinimum(1);
    m_resultBatchFramesSpin->setMaximum(128);
    m_resultBatchFramesSpin->setValue(8);

    m_concurrencyForm->setWidget(4, QFormLayout::FieldRole, m_resultBatchFramesSpin);

    QLabel* backgroundResultWriteLabel = new QLabel(m_infoCard);
    backgroundResultWriteLabel->setObjectName("backgroundResultWriteLabel");

    m_concurrencyForm->setWidget(5, QFormLayout::LabelRole, backgroundResultWriteLabel);

    m_backgroundResultWriteCheckBox = new QCheckBox(m_infoCard);
    m_backgroundResultWriteCheckBox->setObjectName("backgroundResultWriteCheckBox");
    m_backgroundResultWriteCheckBox->setChecked(true);

    m_concurrencyForm->setWidget(5, QFormLayout::FieldRole, m_backgroundResultWriteCheckBox);

    QLabel* nodeLabelModeLabel = new QLabel(m_infoCard);
    nodeLabelModeLabel->setObjectName("nodeLabelModeLabel");

    m_concurrencyForm->setWidget(6, QFormLayout::LabelRole, nodeLabelModeLabel);

    m_nodeLabelModeCombo = new QComboBox(m_infoCard);
    m_nodeLabelModeCombo->addItem(QString());
    m_nodeLabelModeCombo->addItem(QString());
    m_nodeLabelModeCombo->setObjectName("nodeLabelModeCombo");
    m_nodeLabelModeCombo->setMinimumSize(QSize(150, 0));

    m_concurrencyForm->setWidget(6, QFormLayout::FieldRole, m_nodeLabelModeCombo);

    m_cardLayout->addLayout(m_concurrencyForm);

    m_statusLabel = new QLabel(m_infoCard);
    m_statusLabel->setObjectName("statusLabel");
    m_statusLabel->setWordWrap(true);

    m_cardLayout->addWidget(m_statusLabel);

    m_rootLayout->addWidget(m_infoCard);

    m_titleLabel->setText(QCoreApplication::translate("SettingsPanelClass", "求解与显示设置", nullptr));
    m_descriptionLabel->setText(QCoreApplication::translate(
        "SettingsPanelClass",
        "控制可以同时计算的分析步数量。超出上限的任务自动排队，建议保留 CPU 和内存资源给模型显示及其他模块。",
        nullptr));
    concurrencyLabel->setText(QCoreApplication::translate("SettingsPanelClass", "最大并发分析步", nullptr));
    m_concurrencySpin->setSuffix(QCoreApplication::translate("SettingsPanelClass", " 个分析步", nullptr));
    assemblyThreadsLabel->setText(QCoreApplication::translate("SettingsPanelClass", "单步装配线程", nullptr));
    m_assemblyThreadsSpin->setSuffix(QCoreApplication::translate("SettingsPanelClass", " 个线程", nullptr));
    linearSolverModeLabel->setText(QCoreApplication::translate("SettingsPanelClass", "线性求解方法", nullptr));
    m_linearSolverModeCombo->setItemText(0, QCoreApplication::translate("SettingsPanelClass", "自动选择", nullptr));
    m_linearSolverModeCombo->setItemText(
        1, QCoreApplication::translate("SettingsPanelClass", "Eigen LDLT（对称矩阵）", nullptr));
    m_linearSolverModeCombo->setItemText(
        2, QCoreApplication::translate("SettingsPanelClass", "Eigen LU（非对称矩阵）", nullptr));
    m_linearSolverModeCombo->setItemText(
        3, QCoreApplication::translate("SettingsPanelClass", "oneMKL PARDISO（对称 LDLT / 非对称 LU）", nullptr));
    m_linearSolverModeCombo->setItemText(
        4, QCoreApplication::translate("SettingsPanelClass", "CUDA BiCGSTAB（通用非对称迭代法）", nullptr));
    m_linearSolverModeCombo->setItemText(
        5, QCoreApplication::translate("SettingsPanelClass", "NVIDIA cuDSS（通用非对称直接法）", nullptr));

#if QT_CONFIG(tooltip)
    m_linearSolverModeCombo->setToolTip(QCoreApplication::translate(
        "SettingsPanelClass",
        "LDLT 仅用于对称矩阵；LU、BiCGSTAB 和通用矩阵 cuDSS 可处理非对称矩阵。PARDISO 会由分析流程在对称 LDLT 与非对称 "
        "LU 之间选择。自动模式失败时回退，手动模式不回退。",
        nullptr));
#endif // QT_CONFIG(tooltip)
    gpuSolverLabel->setText(QCoreApplication::translate("SettingsPanelClass", "GPU 稀疏求解", nullptr));
    m_gpuSolverCheckBox->setText(QCoreApplication::translate("SettingsPanelClass", "大模型自动 GPU", nullptr));
#if QT_CONFIG(tooltip)
    m_gpuSolverCheckBox->setToolTip(QCoreApplication::translate(
        "SettingsPanelClass",
        "仅影响自动模式。达到运行时 GPU 校准门槛后依次尝试 cuDSS 和 CUDA BiCGSTAB，失败后回退 CPU。", nullptr));
#endif // QT_CONFIG(tooltip)
    resultBatchFramesLabel->setText(QCoreApplication::translate("SettingsPanelClass", "结果批量写入", nullptr));
#if QT_CONFIG(tooltip)
    m_resultBatchFramesSpin->setToolTip(
        QCoreApplication::translate("SettingsPanelClass",
                                    "每累计指定数量的接受步结果后批量写入 HDF5。1 表示逐帧写入；大模型建议 "
                                    "4～8，小模型建议 8～32。修改后对新启动的任务生效。",
                                    nullptr));
#endif // QT_CONFIG(tooltip)
    m_resultBatchFramesSpin->setSuffix(QCoreApplication::translate("SettingsPanelClass", " 帧/批", nullptr));
    backgroundResultWriteLabel->setText(QCoreApplication::translate("SettingsPanelClass", "结果后台写入", nullptr));
    m_backgroundResultWriteCheckBox->setText(
        QCoreApplication::translate("SettingsPanelClass", "求解与写盘并行", nullptr));
#if QT_CONFIG(tooltip)
    m_backgroundResultWriteCheckBox->setToolTip(QCoreApplication::translate(
        "SettingsPanelClass",
        "使用单独线程按顺序写入 HDF5，求解线程最多提前两个批次；关闭后恢复同步写入。修改后对新启动的任务生效。",
        nullptr));
#endif // QT_CONFIG(tooltip)
    nodeLabelModeLabel->setText(QCoreApplication::translate("SettingsPanelClass", "节点编号显示", nullptr));
    m_nodeLabelModeCombo->setItemText(0, QCoreApplication::translate("SettingsPanelClass", "全部", nullptr));
    m_nodeLabelModeCombo->setItemText(1, QCoreApplication::translate("SettingsPanelClass", "自适应", nullptr));

    m_statusLabel->setText(QCoreApplication::translate("SettingsPanelClass", "正在检测本机可用线程……", nullptr));
}

void SettingsPanel::initializeAppearance()
{
    m_cardLayout->setAlignment(Qt::AlignTop);
    m_titleLabel->setObjectName(QStringLiteral("cardTitle"));
    m_descriptionLabel->setObjectName(QStringLiteral("mutedText"));
    m_statusLabel->setObjectName(QStringLiteral("mutedText"));
    m_concurrencyForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    m_concurrencyForm->setRowWrapPolicy(QFormLayout::WrapLongRows);
    m_concurrencyForm->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_concurrencyForm->setHorizontalSpacing(12);
    const QList<QWidget*> fields = {m_concurrencySpin, m_assemblyThreadsSpin, m_linearSolverModeCombo,
                                    m_resultBatchFramesSpin, m_nodeLabelModeCombo};
    for (auto* field : fields)
    {
        field->setMinimumWidth(0);
        field->setMinimumHeight(32);
        field->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        field->setFocusPolicy(Qt::StrongFocus);
    }
    for (auto* combo : {m_linearSolverModeCombo, m_nodeLabelModeCombo})
    {
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setMinimumContentsLength(combo == m_nodeLabelModeCombo ? 4 : 8);
        for (int index = 0; index < combo->count(); ++index)
            combo->setItemData(index, combo->itemText(index), Qt::ToolTipRole);
    }
    auto* scrollArea = new QScrollArea(this);
    scrollArea->setObjectName(QStringLiteral("settingsScrollArea"));
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_rootLayout->removeWidget(m_infoCard);
    scrollArea->setWidget(m_infoCard);
    m_rootLayout->addWidget(scrollArea);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
}
