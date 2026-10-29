#include "LoadEditorDialog.h"
#include "Widgets/CompactDoubleSpinBox.h"
#include "Widgets/DialogSizing.h"
#include "Utility/UiIcons.h"
#include "Utility/AnalysisDisplay.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Factory/LoadFactory.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Wind.h"
#include "LoadTimeFunctionPreview.h"
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QList>
#include <QMessageBox>
#include <QRect>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSize>
#include <QSizePolicy>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedLayout>
#include <QString>
#include <QStringLiteral>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <variant>
#include <vector>

class LoadEditorDialog::ParameterStack final : public QWidget
{
    class CurrentPageLayout final : public QStackedLayout
    {
    public:
        explicit CurrentPageLayout(QWidget* parent)
            : QStackedLayout(parent)
        {
        }

        QSize sizeHint() const override
        {
            return currentWidget() ? currentWidget()->sizeHint() : QSize();
        }

        QSize minimumSize() const override
        {
            const auto* page = currentWidget();
            return page ? page->minimumSizeHint().expandedTo(page->minimumSize()) : QSize();
        }

        bool hasHeightForWidth() const override
        {
            return currentWidget() && currentWidget()->hasHeightForWidth();
        }

        int heightForWidth(int width) const override
        {
            const auto* page = currentWidget();
            return page ? qMax(page->heightForWidth(width), minimumSize().height()) : -1;
        }
    };

public:
    explicit ParameterStack(QWidget* parent)
        : QWidget(parent)
        , m_layout(new CurrentPageLayout(this))
    {
        m_layout->setContentsMargins(0, 0, 0, 0);
        m_layout->setSizeConstraint(QLayout::SetNoConstraint);
        connect(m_layout, &QStackedLayout::currentChanged, this,
                [this]()
                {
                    m_layout->invalidate();
                    updateGeometry();
                });
    }

    void addWidget(QWidget* widget)
    {
        m_layout->addWidget(widget);
    }

    void setCurrentIndex(int index)
    {
        m_layout->setCurrentIndex(index);
    }

    QWidget* currentWidget() const
    {
        return m_layout->currentWidget();
    }

private:
    CurrentPageLayout* m_layout = nullptr;
};

LoadEditorDialog::LoadEditorDialog(int id, const std::shared_ptr<StructureData>& structure,
                                   const std::shared_ptr<LoadBase>& load, QWidget* parent)
    : AnalysisLoadEditor(parent)
    , m_id(id)
    , m_structure(structure)
    , m_original(load)
{
    initializeUi();
    setObjectName(QStringLiteral("analysisLoadEditorDialog"));
    setWindowTitle(load ? QStringLiteral("编辑荷载") : QStringLiteral("新增荷载"));
    setWindowIcon(UiIcons::applicationIcon());
    setModal(true);
    setMinimumWidth(640);
    m_name->setPlaceholderText(QStringLiteral("Load-%1").arg(id));
    m_step->addItem(QStringLiteral("初始 / 全局"), 0);
    for (const auto& [stepId, step] : structure->m_AnalysisStep)
        if (step)
            m_step->addItem(AnalysisDisplay::stepDisplayName(stepId, step), stepId);
    m_type->setItemData(0, static_cast<int>(EnumKeyword::LoadType::FORCE_NODE));
    m_type->setItemData(1, static_cast<int>(EnumKeyword::LoadType::FORCE_ELEMENT));
    m_type->setItemData(2, static_cast<int>(EnumKeyword::LoadType::FORCE_GRAVITY));
    m_type->setItemData(3, static_cast<int>(EnumKeyword::LoadType::FORCE_WIND));
    for (int value = static_cast<int>(EnumKeyword::Direction::X); value <= static_cast<int>(EnumKeyword::Direction::RZ);
         ++value)
    {
        const auto direction = static_cast<EnumKeyword::Direction>(value);
        const QString axis = direction == EnumKeyword::Direction::X    ? QStringLiteral("X")
                             : direction == EnumKeyword::Direction::Y  ? QStringLiteral("Y")
                             : direction == EnumKeyword::Direction::Z  ? QStringLiteral("Z")
                             : direction == EnumKeyword::Direction::RX ? QStringLiteral("X")
                             : direction == EnumKeyword::Direction::RY ? QStringLiteral("Y")
                                                                       : QStringLiteral("Z");
        const QString symbol = value >= static_cast<int>(EnumKeyword::Direction::RX)
                                   ? QStringLiteral("M%1").arg(axis.toLower())
                                   : QStringLiteral("F%1").arg(axis.toLower());
        m_direction->addItem(value >= static_cast<int>(EnumKeyword::Direction::RX)
                                 ? QStringLiteral("绕全局 %1 力矩  %2").arg(axis, symbol)
                                 : QStringLiteral("全局 %1 力  %2").arg(axis, symbol),
                             value);
    }
    m_directionStack = new ParameterStack(this);
    m_directionStack->addWidget(m_direction);
    auto* windDirectionWidget = new QWidget(m_directionStack);
    auto* windDirectionLayout = new QHBoxLayout(windDirectionWidget);
    windDirectionLayout->setContentsMargins(0, 0, 0, 0);
    for (int component = 0; component < 3; ++component)
    {
        m_windDirection[component] = new CompactDoubleSpinBox(windDirectionWidget);
        m_windDirection[component]->setObjectName(QStringLiteral("loadWindDirectionSpin%1").arg(component));
        m_windDirection[component]->setRange(-1.0e6, 1.0e6);
        m_windDirection[component]->setDecimals(6);
        m_windDirection[component]->setPrefix(QString(component == 0 ? "X " : component == 1 ? "Y " : "Z "));
        windDirectionLayout->addWidget(m_windDirection[component]);
    }
    // 导线 X、竖直 Z 坐标系以 -Y 为参考来流方向，这是旧版 TSSBN X/Y（竖直）/Z（风向）约定的
    // 右手映射，使正升力系数 Cl 指向 +Z。
    m_windDirection[1]->setValue(-1.0);
    m_directionStack->addWidget(windDirectionWidget);
    m_valueStack = new ParameterStack(this);
    m_valueStack->addWidget(m_value);
    m_windSpeed = new QComboBox(m_valueStack);
    m_windSpeed->setObjectName(QStringLiteral("loadWindSpeedCombo"));
    for (int speed : AeroManager::supportedWindSpeeds())
        m_windSpeed->addItem(QStringLiteral("%1 m/s").arg(speed), speed);
    m_valueStack->addWidget(m_windSpeed);
    m_rootLayout->setContentsMargins(18, 16, 18, 16);
    m_rootLayout->setSpacing(14);
    m_rootLayout->setSizeConstraint(QLayout::SetNoConstraint);

    auto* identity = new QWidget(this);
    auto* identityLayout = new QHBoxLayout(identity);
    identityLayout->setContentsMargins(0, 0, 0, 0);
    identityLayout->setSpacing(16);
    identityLayout->addWidget(createField(m_nameLabel, m_name), 3);
    identityLayout->addWidget(createField(m_stepLabel, m_step), 2);
    m_rootLayout->insertWidget(0, identity);

    auto* definition = new QGroupBox(QStringLiteral("荷载定义"), this);
    definition->setObjectName(QStringLiteral("loadDefinitionGroup"));
    auto* definitionLayout = new QGridLayout(definition);
    definitionLayout->setHorizontalSpacing(14);
    definitionLayout->setVerticalSpacing(12);
    definitionLayout->setColumnStretch(0, 1);
    definitionLayout->setColumnStretch(1, 1);
    definitionLayout->addWidget(createField(m_typeLabel, m_type), 0, 0);
    definitionLayout->addWidget(createField(m_valueLabel, m_valueStack), 0, 1);
    m_targetStack = new ParameterStack(definition);
    m_targetStack->addWidget(m_target);
    auto* wholeModel = new QLabel(QStringLiteral("整个模型"), m_targetStack);
    wholeModel->setMinimumHeight(34);
    wholeModel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    m_targetStack->addWidget(wholeModel);
    definitionLayout->addWidget(createField(m_targetLabel, m_targetStack), 1, 0, 1, 2);
    definitionLayout->addWidget(createField(m_directionLabel, m_directionStack), 2, 0, 1, 2);

    auto* timeGroup = new QGroupBox(QStringLiteral("作用时间"), this);
    timeGroup->setObjectName(QStringLiteral("loadTimeGroup"));
    auto* timeLayout = new QGridLayout(timeGroup);
    timeLayout->setHorizontalSpacing(14);
    timeLayout->setVerticalSpacing(10);
    timeLayout->setColumnStretch(0, 1);
    timeLayout->setColumnStretch(1, 1);
    m_startTime = createTimeSpinBox(timeGroup);
    m_startTime->setObjectName(QStringLiteral("loadStartTimeSpin"));
    m_endTime = createTimeSpinBox(timeGroup);
    m_endTime->setObjectName(QStringLiteral("loadEndTimeSpin"));
    timeLayout->addWidget(createField(new QLabel(QStringLiteral("开始时间")), m_startTime), 0, 0);
    timeLayout->addWidget(createField(new QLabel(QStringLiteral("结束时间")), m_endTime), 0, 1);
    m_continuous = new QCheckBox(QStringLiteral("持续至分析结束"), timeGroup);
    m_continuous->setObjectName(QStringLiteral("loadContinuousCheck"));
    m_continuous->setChecked(true);
    timeLayout->addWidget(m_continuous, 1, 0, 1, 2);

    auto* functionGroup = new QGroupBox(QStringLiteral("时间函数"), this);
    functionGroup->setObjectName(QStringLiteral("loadTimeFunctionGroup"));
    auto* functionLayout = new QVBoxLayout(functionGroup);
    functionLayout->setSpacing(10);
    m_function = new QComboBox(functionGroup);
    m_function->setObjectName(QStringLiteral("loadTimeFunctionCombo"));
    addFunctionType(TimeFunctionType::CONSTANT);
    addFunctionType(TimeFunctionType::SIN);
    addFunctionType(TimeFunctionType::COS);
    addFunctionType(TimeFunctionType::RAMP);
    addFunctionType(TimeFunctionType::EXPONENTIAL);
    addFunctionType(TimeFunctionType::TRIANGULAR);
    addFunctionType(TimeFunctionType::SQUARE);
    functionLayout->addWidget(m_function);
    m_functionParameters = new ParameterStack(functionGroup);
    m_functionParameters->setObjectName(QStringLiteral("loadTimeFunctionParameters"));
    m_functionParameters->addWidget(createConstantParametersPage());
    m_functionParameters->addWidget(createHarmonicParametersPage());
    m_functionParameters->addWidget(createRampParametersPage());
    m_functionParameters->addWidget(createExponentialParametersPage());
    m_functionParameters->addWidget(createTriangularParametersPage());
    m_functionParameters->addWidget(createSquareParametersPage());
    functionLayout->addWidget(m_functionParameters);

    auto* editorBody = new QWidget(this);
    auto* editorLayout = new QVBoxLayout(editorBody);
    editorLayout->setContentsMargins(0, 0, 0, 0);
    editorLayout->setSpacing(12);
    editorLayout->setAlignment(Qt::AlignTop);
    editorLayout->setSizeConstraint(QLayout::SetMinimumSize);
    for (auto* group : {definition, timeGroup, functionGroup})
    {
        group->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        group->layout()->setContentsMargins(8, 6, 8, 8);
        editorLayout->addWidget(group);
    }

    auto* previewGroup = new QGroupBox(QStringLiteral("时程预览"), this);
    previewGroup->setObjectName(QStringLiteral("loadPreviewGroup"));
    auto* previewLayout = new QVBoxLayout(previewGroup);
    previewLayout->setContentsMargins(8, 6, 8, 8);
    previewLayout->setSpacing(10);
    auto* probeLayout = new QHBoxLayout;
    probeLayout->addWidget(new QLabel(QStringLiteral("查看时刻"), previewGroup));
    m_previewTime = createTimeSpinBox(previewGroup);
    m_previewTime->setDecimals(6);
    m_previewTime->setFixedWidth(140);
    m_previewTime->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    probeLayout->addStretch();
    probeLayout->addWidget(m_previewTime);
    previewLayout->addLayout(probeLayout);
    m_previewValue = new QLabel(previewGroup);
    m_previewValue->setObjectName(QStringLiteral("analysisLoadProbeValue"));
    m_previewValue->setWordWrap(true);
    previewLayout->addWidget(m_previewValue);
    m_preview = new LoadTimeFunctionPreview(previewGroup);
    m_preview->setObjectName(QStringLiteral("analysisLoadPreview"));
    m_preview->setMinimumSize(0, 220);
    m_preview->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    previewLayout->addWidget(m_preview, 1);

    auto* expressionGroup = new QGroupBox(QStringLiteral("荷载表达式"), this);
    expressionGroup->setObjectName(QStringLiteral("loadExpressionGroup"));
    expressionGroup->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    auto* expressionLayout = new QVBoxLayout(expressionGroup);
    expressionLayout->setContentsMargins(8, 6, 8, 8);
    expressionLayout->setSpacing(8);
    m_typeSummary = new QLabel(expressionGroup);
    m_typeSummary->setObjectName(QStringLiteral("analysisLoadTypeSummary"));
    m_expression = new QLabel(expressionGroup);
    m_expression->setObjectName(QStringLiteral("analysisLoadExpression"));
    m_timeSummary = new QLabel(expressionGroup);
    m_timeSummary->setObjectName(QStringLiteral("analysisLoadTimeSummary"));
    for (auto* label : {m_typeSummary, m_expression, m_timeSummary})
    {
        label->setWordWrap(true);
        label->setTextFormat(Qt::PlainText);
        label->setMinimumWidth(0);
        label->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        expressionLayout->addWidget(label);
    }
    m_typeSummary->setProperty("secondaryText", true);
    m_timeSummary->setProperty("secondaryText", true);
    auto* previewBody = new QWidget(this);
    auto* previewBodyLayout = new QVBoxLayout(previewBody);
    previewBodyLayout->setContentsMargins(0, 0, 0, 0);
    previewBodyLayout->setSpacing(12);
    previewBodyLayout->setSizeConstraint(QLayout::SetMinimumSize);
    previewBodyLayout->addWidget(previewGroup, 1);
    previewBodyLayout->addWidget(expressionGroup);

    auto* content = new QSplitter(Qt::Horizontal, this);
    m_contentSplitter = content;
    content->setObjectName(QStringLiteral("loadEditorSplitter"));
    content->setChildrenCollapsible(false);
    content->setHandleWidth(14);
    content->setStyleSheet(QStringLiteral("QSplitter#loadEditorSplitter::handle { background: transparent; }"
                                          "QSplitter#loadEditorSplitter::handle:hover { background: palette(mid); "
                                          "margin: 0 5px; border-radius: 2px; }"));
    m_editorScroll = createFormScroll(editorBody, QStringLiteral("loadParametersScroll"));
    m_editorScroll->setMinimumWidth(360);
    m_editorScroll->setMaximumWidth(640);
    auto* previewScroll = createFormScroll(previewBody, QStringLiteral("loadPreviewScroll"));
    m_previewScroll = previewScroll;
    previewScroll->setMinimumWidth(300);
    content->addWidget(m_editorScroll);
    content->addWidget(previewScroll);
    content->setStretchFactor(0, 3);
    content->setStretchFactor(1, 2);
    content->setSizes({540, 384});
    m_rootLayout->insertWidget(1, content, 1);
    for (auto* combo : {m_type, m_step, m_direction, m_windSpeed, m_function})
    {
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setMinimumContentsLength(8);
        combo->setMinimumHeight(34);
    }
    for (auto* spin : m_windDirection)
    {
        spin->setMinimumWidth(80);
        spin->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    }
    auto* buttons = m_buttonBox;
    buttons->button(QDialogButtonBox::Ok)->setText(QStringLiteral("确定"));
    buttons->button(QDialogButtonBox::Ok)->setDefault(true);
    buttons->button(QDialogButtonBox::Ok)->setProperty("primaryAction", true);
    buttons->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    buttons->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    const QList<QWidget*> tabOrder{m_name,
                                   m_step,
                                   m_type,
                                   m_value,
                                   m_windSpeed,
                                   m_target,
                                   m_direction,
                                   m_windDirection[0],
                                   m_windDirection[1],
                                   m_windDirection[2],
                                   m_startTime,
                                   m_endTime,
                                   m_continuous,
                                   m_function,
                                   m_amplitude,
                                   m_frequency,
                                   m_phase,
                                   m_offset,
                                   m_rampT0,
                                   m_rampT1,
                                   m_decay,
                                   m_trianglePeriod,
                                   m_squarePeriod,
                                   m_dutyCycle,
                                   m_previewTime,
                                   buttons->button(QDialogButtonBox::Ok),
                                   buttons->button(QDialogButtonBox::Cancel)};
    for (qsizetype index = 1; index < tabOrder.size(); ++index)
        setTabOrder(tabOrder[index - 1], tabOrder[index]);
    connect(buttons, &QDialogButtonBox::accepted, this, &LoadEditorDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(m_type, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int)
            {
                updateTargetState();
            });
    connect(m_direction, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int)
            {
                updateTimeFunctionDisplay();
            });
    connect(m_continuous, &QCheckBox::toggled, this,
            [this](bool continuous)
            {
                m_endTime->setEnabled(!continuous);
                updateTimeFunctionDisplay();
            });
    connect(m_function, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int index)
            {
                m_functionParameters->setCurrentIndex(index == 2 ? 1 : index - (index > 2 ? 1 : 0));
                updateTimeFunctionDisplay();
            });
    connectTimeFunctionControls();
    connect(m_previewTime, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double)
            {
                updateTimeFunctionDisplay();
            });
    if (load)
    {
        m_name->setText(load->m_Name);
        int stepIndex = m_step->findData(load->m_StepId);
        if (stepIndex < 0)
        {
            m_step->addItem(QStringLiteral("Step-%1（缺失）").arg(load->m_StepId), load->m_StepId);
            stepIndex = m_step->count() - 1;
        }
        m_step->setCurrentIndex(stepIndex);
        m_type->setCurrentIndex(qMax(0, m_type->findData(static_cast<int>(load->m_LoadType))));
        m_direction->setCurrentIndex(qMax(0, m_direction->findData(static_cast<int>(load->m_Direction))));
        const int targetId = AnalysisDisplay::loadTargetId(load);
        if (targetId > 0)
            m_target->setValue(targetId);
        m_value->setValue(AnalysisDisplay::loadValue(load));
        if (const auto wind = std::dynamic_pointer_cast<Force_Wind>(load))
        {
            for (int component = 0; component < 3; ++component)
                m_windDirection[component]->setValue(wind->m_direction[component]);
            int windIndex = m_windSpeed->findData(wind->m_velocity);
            if (windIndex < 0)
            {
                m_windSpeed->addItem(QStringLiteral("%1 m/s（旧模型，不支持舞动）").arg(wind->m_velocity),
                                     wind->m_velocity);
                windIndex = m_windSpeed->count() - 1;
            }
            m_windSpeed->setCurrentIndex(windIndex);
        }
        m_startTime->setValue(load->m_StartTime);
        m_endTime->setValue(AnalysisDisplay::isContinuousLoad(load) ? suggestedEndTime() : load->m_EndTime);
        m_continuous->setChecked(AnalysisDisplay::isContinuousLoad(load));
        m_function->setCurrentIndex(qMax(0, m_function->findData(static_cast<int>(load->m_FunctionType))));
        m_amplitude->setValue(load->m_Amplitude);
        m_frequency->setValue(load->m_Frequency);
        m_phase->setValue(load->m_Phase);
        m_offset->setValue(load->m_Offset);
        m_rampT0->setValue(load->m_RampT0);
        m_rampT1->setValue(load->m_RampT1);
        m_decay->setValue(load->m_Decay);
        m_trianglePeriod->setValue(load->m_Period);
        m_squarePeriod->setValue(load->m_Period);
        m_dutyCycle->setValue(load->m_DutyCycle);
    }
    else
    {
        if (m_step->count() > 1)
            m_step->setCurrentIndex(1);
        if (!structure->m_Nodes.empty())
            m_target->setValue(structure->m_Nodes.begin()->first);
    }
    updateTargetState();
    m_endTime->setEnabled(!m_continuous->isChecked());
    updateTimeFunctionDisplay();
    resize(1000, 680);
    DialogSizing::lockCurrentHeight(this);
}

std::shared_ptr<LoadBase> LoadEditorDialog::value() const
{
    return m_result;
}

void LoadEditorDialog::accept()
{
    const auto type = static_cast<EnumKeyword::LoadType>(m_type->currentData().toInt());
    auto result = previewLoad();
    switch (type)
    {
        case EnumKeyword::LoadType::FORCE_NODE:
        {
            const auto node = m_structure->FindNode(m_target->value());
            if (!node)
            {
                QMessageBox::warning(this, QStringLiteral("无效目标"),
                                     QStringLiteral("节点 %1 不存在。").arg(m_target->value()));
                return;
            }
            std::static_pointer_cast<Force_Node>(result)->m_pNode = node;
            break;
        }
        case EnumKeyword::LoadType::FORCE_ELEMENT:
        {
            const auto element = m_structure->FindElement(m_target->value());
            if (!element)
            {
                QMessageBox::warning(this, QStringLiteral("无效目标"),
                                     QStringLiteral("单元 %1 不存在。").arg(m_target->value()));
                return;
            }
            std::static_pointer_cast<Force_Element>(result)->m_pElement = element;
            break;
        }
        case EnumKeyword::LoadType::FORCE_GRAVITY:
            break;
        case EnumKeyword::LoadType::FORCE_WIND:
        {
            const Eigen::Vector3d direction(m_windDirection[0]->value(), m_windDirection[1]->value(),
                                            m_windDirection[2]->value());
            if (!direction.allFinite() || direction.norm() <= 1.0e-12)
            {
                QMessageBox::warning(this, QStringLiteral("无效风向"), QStringLiteral("全局风向向量不能为零。"));
                return;
            }
            std::static_pointer_cast<Force_Wind>(result)->m_direction = direction.normalized();
            break;
        }
        default:
            return;
    }
    result->m_Id = m_id;
    result->m_Name = m_name->text().trimmed();
    result->m_StepId = m_step->currentData().toInt();
    QString timeFunctionError;
    if (!result->ValidateTimeFunction(timeFunctionError))
    {
        QMessageBox::warning(this, QStringLiteral("无效时间函数"), timeFunctionError);
        return;
    }
    const auto oldWind = std::dynamic_pointer_cast<Force_Wind>(m_original);
    const auto newWind = std::dynamic_pointer_cast<Force_Wind>(result);
    if (oldWind && newWind)
    {
        newWind->m_windDensity = oldWind->m_windDensity;
        newWind->m_EnableAerodynamicMoment = oldWind->m_EnableAerodynamicMoment;
    }
    m_result = result;
    QDialog::accept();
}

QWidget* LoadEditorDialog::createField(QLabel* label, QWidget* editor)
{
    auto* field = new QWidget(this);
    field->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto* fieldLayout = new QVBoxLayout(field);
    fieldLayout->setContentsMargins(0, 0, 0, 0);
    fieldLayout->setSpacing(6);
    label->setProperty("secondaryText", true);
    label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    fieldLayout->addWidget(label);
    editor->setMinimumWidth(0);
    editor->setMinimumHeight(34);
    editor->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    fieldLayout->addWidget(editor);
    return field;
}

void LoadEditorDialog::resizeEvent(QResizeEvent* event)
{
    QDialog::resizeEvent(event);
    if (!m_contentSplitter)
        return;
    const bool vertical = width() < 800;
    const auto orientation = vertical ? Qt::Vertical : Qt::Horizontal;
    if (m_contentSplitter->orientation() == orientation)
        return;
    m_editorScroll->setMinimumSize(vertical ? QSize(0, 90) : QSize(360, 0));
    m_editorScroll->setMaximumWidth(vertical ? QWIDGETSIZE_MAX : 640);
    m_previewScroll->setMinimumSize(vertical ? QSize(0, 90) : QSize(300, 0));
    m_contentSplitter->setOrientation(orientation);
    const int extent = vertical ? m_contentSplitter->height() : m_contentSplitter->width();
    m_contentSplitter->setSizes({extent * 3 / 5, extent * 2 / 5});
}

QScrollArea* LoadEditorDialog::createFormScroll(QWidget* body, const QString& name)
{
    auto* scroll = new QScrollArea(this);
    scroll->setObjectName(name);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setWidget(body);
    return scroll;
}

QDoubleSpinBox* LoadEditorDialog::createTimeSpinBox(QWidget* parent)
{
    auto* spin = new CompactDoubleSpinBox(parent);
    spin->setRange(0.0, 1.0e10);
    spin->setDecimals(8);
    spin->setSingleStep(0.01);
    spin->setMinimumSize(84, 34);
    spin->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    spin->setSuffix(QStringLiteral(" s"));
    return spin;
}

QDoubleSpinBox* LoadEditorDialog::createParameterSpinBox(QWidget* parent, double minimum, double maximum)
{
    auto* spin = new CompactDoubleSpinBox(parent);
    spin->setRange(minimum, maximum);
    spin->setDecimals(10);
    spin->setSingleStep(0.1);
    spin->setMinimumSize(84, 34);
    spin->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    return spin;
}

void LoadEditorDialog::addFunctionType(TimeFunctionType type)
{
    m_function->addItem(AnalysisDisplay::timeFunctionName(type), static_cast<int>(type));
}

QWidget* LoadEditorDialog::createConstantParametersPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    auto* hint = new QLabel(QStringLiteral("比例恒为 1，荷载等于基值。"), page);
    hint->setObjectName(QStringLiteral("analysisLoadHint"));
    hint->setWordWrap(true);
    layout->addWidget(hint);
    return page;
}

QWidget* LoadEditorDialog::createHarmonicParametersPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    m_amplitude = createParameterSpinBox(page);
    m_amplitude->setObjectName(QStringLiteral("loadAmplitudeSpin"));
    m_frequency = createParameterSpinBox(page, 0.0);
    m_frequency->setObjectName(QStringLiteral("loadFrequencySpin"));
    m_phase = createParameterSpinBox(page);
    m_phase->setObjectName(QStringLiteral("loadPhaseSpin"));
    m_offset = createParameterSpinBox(page);
    m_offset->setObjectName(QStringLiteral("loadOffsetSpin"));
    m_frequency->setSuffix(QStringLiteral(" Hz"));
    m_phase->setSuffix(QStringLiteral(" rad"));
    layout->addRow(QStringLiteral("系数"), m_amplitude);
    layout->addRow(QStringLiteral("频率"), m_frequency);
    layout->addRow(QStringLiteral("相位"), m_phase);
    layout->addRow(QStringLiteral("偏置"), m_offset);
    return page;
}

QWidget* LoadEditorDialog::createRampParametersPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    m_rampT0 = createTimeSpinBox(page);
    m_rampT0->setObjectName(QStringLiteral("loadRampStartSpin"));
    m_rampT1 = createTimeSpinBox(page);
    m_rampT1->setObjectName(QStringLiteral("loadRampEndSpin"));
    layout->addRow(QStringLiteral("局部起始时间"), m_rampT0);
    layout->addRow(QStringLiteral("局部结束时间"), m_rampT1);
    return page;
}

QWidget* LoadEditorDialog::createExponentialParametersPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    m_decay = createParameterSpinBox(page, 0.0);
    m_decay->setObjectName(QStringLiteral("loadDecaySpin"));
    layout->addRow(QStringLiteral("衰减系数"), m_decay);
    return page;
}

QWidget* LoadEditorDialog::createTriangularParametersPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    m_trianglePeriod = createTimeSpinBox(page);
    m_trianglePeriod->setObjectName(QStringLiteral("loadTrianglePeriodSpin"));
    m_trianglePeriod->setMinimum(1.0e-12);
    layout->addRow(QStringLiteral("周期"), m_trianglePeriod);
    return page;
}

QWidget* LoadEditorDialog::createSquareParametersPage()
{
    auto* page = new QWidget(this);
    auto* layout = new QFormLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    m_squarePeriod = createTimeSpinBox(page);
    m_squarePeriod->setObjectName(QStringLiteral("loadSquarePeriodSpin"));
    m_squarePeriod->setMinimum(1.0e-12);
    m_dutyCycle = createParameterSpinBox(page, 0.0, 1.0);
    m_dutyCycle->setObjectName(QStringLiteral("loadDutyCycleSpin"));
    m_dutyCycle->setValue(0.5);
    layout->addRow(QStringLiteral("周期"), m_squarePeriod);
    layout->addRow(QStringLiteral("占空比"), m_dutyCycle);
    return page;
}

double LoadEditorDialog::suggestedEndTime() const
{
    if (m_structure && m_step && m_step->currentData().toInt() > 0)
    {
        const auto found = m_structure->m_AnalysisStep.find(m_step->currentData().toInt());
        if (found != m_structure->m_AnalysisStep.end() && found->second && found->second->m_Time > 0.0)
            return found->second->m_Time;
    }
    return std::max(1.0, m_startTime ? m_startTime->value() + 1.0 : 1.0);
}

std::shared_ptr<LoadBase> LoadEditorDialog::previewLoad() const
{
    const auto type = static_cast<EnumKeyword::LoadType>(m_type->currentData().toInt());
    const double value =
        type == EnumKeyword::LoadType::FORCE_WIND ? m_windSpeed->currentData().toDouble() : m_value->value();
    auto result = LoadFactory::Create(type, value);
    if (!result)
        result = LoadFactory::Create(EnumKeyword::LoadType::FORCE_WIND, m_windSpeed->currentData().toDouble());
    result->m_Direction = type == EnumKeyword::LoadType::FORCE_WIND
                              ? EnumKeyword::Direction::UNKNOWN
                              : static_cast<EnumKeyword::Direction>(m_direction->currentData().toInt());
    result->m_StartTime = m_startTime->value();
    result->m_EndTime = m_continuous->isChecked() ? 1.0e10 : m_endTime->value();
    result->m_FunctionType = static_cast<TimeFunctionType>(m_function->currentData().toInt());
    result->m_Amplitude = m_amplitude->value();
    result->m_Frequency = m_frequency->value();
    result->m_Phase = m_phase->value();
    result->m_Offset = m_offset->value();
    result->m_RampT0 = m_rampT0->value();
    result->m_RampT1 = m_rampT1->value();
    result->m_Decay = m_decay->value();
    result->m_Period =
        result->m_FunctionType == TimeFunctionType::SQUARE ? m_squarePeriod->value() : m_trianglePeriod->value();
    result->m_DutyCycle = m_dutyCycle->value();
    return result;
}

void LoadEditorDialog::connectTimeFunctionControls()
{
    const auto update = [this](double)
    {
        updateTimeFunctionDisplay();
    };
    for (QDoubleSpinBox* control : {m_value, m_startTime, m_endTime, m_amplitude, m_frequency, m_phase, m_offset,
                                    m_rampT0, m_rampT1, m_decay, m_trianglePeriod, m_squarePeriod, m_dutyCycle})
    {
        connect(control, qOverload<double>(&QDoubleSpinBox::valueChanged), this, update);
    }
    connect(m_step, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int)
            {
                if (m_continuous->isChecked())
                    m_endTime->setValue(suggestedEndTime());
                updateTimeFunctionDisplay();
            });
    connect(m_windSpeed, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this](int)
            {
                updateTimeFunctionDisplay();
            });
}

void LoadEditorDialog::updateTimeFunctionDisplay()
{
    if (!m_preview || !m_function)
        return;
    const auto load = previewLoad();
    const auto function = load->m_FunctionType;
    m_functionParameters->setCurrentIndex(
        m_function->currentIndex() == 2 ? 1 : m_function->currentIndex() - (m_function->currentIndex() > 2 ? 1 : 0));
    m_typeSummary->setText(QStringLiteral("类型：%1 · %2 · %3")
                               .arg(AnalysisDisplay::loadPhysicalTypeName(load),
                                    AnalysisDisplay::loadComponentText(load),
                                    AnalysisDisplay::timeFunctionName(function)));
    const QString expression = AnalysisDisplay::loadExpression(load);
    m_expression->setText(expression);
    m_expression->setToolTip(expression);
    m_timeSummary->setText(QStringLiteral("有效区间：%1").arg(AnalysisDisplay::loadTimeWindowText(load)));
    const double previewEnd =
        m_continuous->isChecked() ? std::max(load->m_StartTime + 1.0e-9, suggestedEndTime()) : load->m_EndTime;
    {
        QSignalBlocker blocker(m_previewTime);
        m_previewTime->setRange(load->m_StartTime, previewEnd);
        if (m_previewTime->value() < load->m_StartTime || m_previewTime->value() > previewEnd)
            m_previewTime->setValue(load->m_StartTime);
    }
    m_preview->setDefinition(AnalysisDisplay::loadValue(load), function, load->m_StartTime, load->m_EndTime,
                             m_continuous->isChecked(), load->m_Amplitude, load->m_Frequency, load->m_Phase,
                             load->m_Offset, load->m_RampT0, load->m_RampT1, load->m_Decay, load->m_Period,
                             load->m_DutyCycle, suggestedEndTime(), m_previewTime->value());
    const QString unit = load->m_LoadType == EnumKeyword::LoadType::FORCE_GRAVITY ? QStringLiteral("m/s²")
                         : load->m_LoadType == EnumKeyword::LoadType::FORCE_WIND  ? QStringLiteral("m/s")
                         : static_cast<int>(load->m_Direction) >= static_cast<int>(EnumKeyword::Direction::RX)
                             ? QStringLiteral("N·m")
                             : QStringLiteral("N");
    const double probeValue = m_preview->valueAt(m_previewTime->value());
    m_previewValue->setText(QStringLiteral("%1(%2 s) = %3 %4")
                                .arg(AnalysisDisplay::loadVariableName(load),
                                     AnalysisDisplay::numberText(m_previewTime->value()),
                                     QString::number(probeValue, 'g', 6), unit));
    m_previewValue->setToolTip(QStringLiteral("%1(%2 s) = %3 %4")
                                   .arg(AnalysisDisplay::loadVariableName(load),
                                        AnalysisDisplay::numberText(m_previewTime->value()),
                                        AnalysisDisplay::numberText(probeValue), unit));
}

void LoadEditorDialog::updateTargetState()
{
    const auto type = static_cast<EnumKeyword::LoadType>(m_type->currentData().toInt());
    const bool hasTarget = type == EnumKeyword::LoadType::FORCE_NODE || type == EnumKeyword::LoadType::FORCE_ELEMENT;
    m_target->setEnabled(hasTarget);
    m_targetStack->setCurrentIndex(hasTarget ? 0 : 1);
    m_targetLabel->setText(hasTarget ? QStringLiteral("目标编号") : QStringLiteral("作用范围"));
    m_target->setToolTip(type == EnumKeyword::LoadType::FORCE_NODE      ? QStringLiteral("节点编号")
                         : type == EnumKeyword::LoadType::FORCE_ELEMENT ? QStringLiteral("单元编号")
                                                                        : QStringLiteral("该荷载作用于整个模型"));
    const bool wind = type == EnumKeyword::LoadType::FORCE_WIND;
    m_directionStack->setCurrentIndex(wind ? 1 : 0);
    m_directionLabel->setText(wind ? QStringLiteral("全局风向") : QStringLiteral("方向"));
    m_valueStack->setCurrentIndex(wind ? 1 : 0);
    const int direction = m_direction->currentData().toInt();
    const bool moment = direction >= static_cast<int>(EnumKeyword::Direction::RX);
    m_valueLabel->setText(wind                                           ? QStringLiteral("风速")
                          : type == EnumKeyword::LoadType::FORCE_GRAVITY ? QStringLiteral("重力加速度")
                                                                         : QStringLiteral("基值"));
    m_value->setSuffix(wind                                           ? QString()
                       : type == EnumKeyword::LoadType::FORCE_GRAVITY ? QStringLiteral(" m/s²")
                       : moment                                       ? QStringLiteral(" N·m")
                                                                      : QStringLiteral(" N"));
    updateTimeFunctionDisplay();
}

void LoadEditorDialog::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("LoadEditorDialogClass");
    }
    setMinimumSize(QSize(440, 0));
    m_rootLayout = new QVBoxLayout(this);
    m_rootLayout->setSpacing(14);
    m_rootLayout->setObjectName("rootLayout");
    m_rootLayout->setContentsMargins(18, 18, 18, 18);
    m_nameLabel = new QLabel(this);
    m_nameLabel->setObjectName("nameLabel");
    m_name = new QLineEdit(this);
    m_name->setObjectName("nameEdit");
    m_stepLabel = new QLabel(this);
    m_stepLabel->setObjectName("stepLabel");
    m_step = new QComboBox(this);
    m_step->setObjectName("stepCombo");
    m_typeLabel = new QLabel(this);
    m_typeLabel->setObjectName("typeLabel");
    m_type = new QComboBox(this);
    m_type->addItem(QString());
    m_type->addItem(QString());
    m_type->addItem(QString());
    m_type->addItem(QString());
    m_type->setObjectName("typeCombo");
    m_targetLabel = new QLabel(this);
    m_targetLabel->setObjectName("targetLabel");
    m_target = new QSpinBox(this);
    m_target->setObjectName("targetSpin");
    m_target->setMinimum(1);
    m_target->setMaximum(2147483647);
    m_directionLabel = new QLabel(this);
    m_directionLabel->setObjectName("directionLabel");
    m_direction = new QComboBox(this);
    m_direction->setObjectName("directionCombo");
    m_valueLabel = new QLabel(this);
    m_valueLabel->setObjectName("valueLabel");
    m_value = new CompactDoubleSpinBox(this);
    m_value->setObjectName("valueSpin");
    m_value->setMinimum(-1000000000000000.000000000000000);
    m_value->setMaximum(1000000000000000.000000000000000);
    m_value->setDecimals(10);
    m_buttonBox = new QDialogButtonBox(this);
    m_buttonBox->setObjectName("buttonBox");
    m_buttonBox->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Ok);

    m_rootLayout->addWidget(m_buttonBox);

    m_nameLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "名称", nullptr));
    m_stepLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "所属分析步", nullptr));
    m_typeLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "类型", nullptr));
    m_type->setItemText(0, QCoreApplication::translate("LoadEditorDialogClass", "节点力", nullptr));
    m_type->setItemText(1, QCoreApplication::translate("LoadEditorDialogClass", "单元荷载", nullptr));
    m_type->setItemText(2, QCoreApplication::translate("LoadEditorDialogClass", "重力", nullptr));
    m_type->setItemText(3, QCoreApplication::translate("LoadEditorDialogClass", "风荷载", nullptr));

    m_targetLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "目标编号", nullptr));
    m_directionLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "方向", nullptr));
    m_valueLabel->setText(QCoreApplication::translate("LoadEditorDialogClass", "数值", nullptr));
}
