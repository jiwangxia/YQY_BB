#include "AdaptiveTssbnSettingsEditor.h"
#include "Widgets/CompactDoubleSpinBox.h"
#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
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

AdaptiveTssbnSettingsEditor::AdaptiveTssbnSettingsEditor(QWidget* parent)
    : QGroupBox(QStringLiteral("自适应 TSSBN 参数"), parent)
{
    setObjectName(QStringLiteral("adaptiveTssbnSettingsGroup"));
    auto* contentLayout = new QVBoxLayout(this);
    contentLayout->setAlignment(Qt::AlignTop);
    auto* fieldGrid = new QGridLayout;
    fieldGrid->setContentsMargins(0, 0, 0, 0);
    fieldGrid->setHorizontalSpacing(12);
    fieldGrid->setVerticalSpacing(7);
    contentLayout->addLayout(fieldGrid);
    const std::array<QString, 3> sectionTitles{QStringLiteral("积分与步长"), QStringLiteral("误差控制"),
                                               QStringLiteral("步长调整与迭代")};
    const std::array<int, 3> sectionTitleRows{0, 3, 6};
    const std::array<int, 3> sectionFieldRows{1, 4, 7};
    for (std::size_t sectionIndex = 0; sectionIndex < sectionTitles.size(); ++sectionIndex)
    {
        auto* sectionTitle = new QLabel(sectionTitles[sectionIndex], this);
        QFont titleFont = sectionTitle->font();
        titleFont.setBold(true);
        sectionTitle->setFont(titleFont);
        fieldGrid->addWidget(sectionTitle, sectionTitleRows[sectionIndex], 0, 1, 3);
    }
    fieldGrid->setRowMinimumHeight(2, 0);
    const std::array<FieldSpec, 11> fields{
        {{QStringLiteral("高频半径"), QStringLiteral("高频谱半径 ρ∞"), &Settings::spectralRadiusInfinity, 0.0, 1.0, 6,
          0, 0, 0},
         {QStringLiteral("最小步长"), QStringLiteral("最小时间步"), &Settings::minimumTimeStep, 1.0e-12, 1.0e12, 12, 0,
          0, 1},
         {QStringLiteral("最大步长"), QStringLiteral("最大时间步"), &Settings::maximumTimeStep, 1.0e-12, 1.0e12, 12, 0,
          0, 2},
         {QStringLiteral("相对 LTE"), QStringLiteral("相对 LTE 容限"), &Settings::relativeTolerance, 1.0e-12, 1.0, 12,
          1, 0, 0},
         {QStringLiteral("位移绝对 LTE"), QStringLiteral("位移绝对 LTE 容限"), &Settings::absoluteTolerance, 1.0e-14,
          1.0, 14, 1, 0, 1},
         {QStringLiteral("速度绝对 LTE"), QStringLiteral("速度绝对 LTE 容限，单位为 m/s 或 rad/s"),
          &Settings::velocityAbsoluteTolerance, 1.0e-14, 1.0, 14, 1, 0, 2},
         {QStringLiteral("安全系数"), QStringLiteral("步长安全系数"), &Settings::safetyFactor, 0.01, 1.0, 6, 2, 0, 2},
         {QStringLiteral("缩步系数"), QStringLiteral("最小缩步系数"), &Settings::shrinkFactor, 0.01, 0.99, 6, 2, 0, 0},
         {QStringLiteral("增长系数"), QStringLiteral("最大增长系数"), &Settings::maximumGrowthFactor, 1.0, 100.0, 6, 2,
          0, 1},
         {QStringLiteral("牛顿总上限"), QStringLiteral("C1+C2 每步最大牛顿修正总数"),
          &Settings::maximumTotalNewtonIterations, 1.0, 1000.0, 0, 2, 1, 0},
         {QStringLiteral("拒绝上限"), QStringLiteral("最大连续拒绝次数"), &Settings::maximumRejectedAttempts, 1.0,
          1000.0, 0, 2, 1, 1}}};

    for (const FieldSpec& field : fields)
    {
        QAbstractSpinBox* editor = nullptr;
        if (std::holds_alternative<DoubleMember>(field.member))
        {
            auto* spin = new CompactDoubleSpinBox(this);
            spin->setRange(field.minimum, field.maximum);
            spin->setDecimals(field.decimals);
            spin->setKeyboardTracking(false);
            editor = spin;
            const auto member = std::get<DoubleMember>(field.member);
            if (member == &Settings::absoluteTolerance)
            {
                m_absoluteTolerance = spin;
                spin->setObjectName(QStringLiteral("tssbnAbsoluteToleranceSpin"));
            }
            else if (member == &Settings::velocityAbsoluteTolerance)
            {
                m_velocityAbsoluteTolerance = spin;
                spin->setObjectName(QStringLiteral("tssbnVelocityToleranceSpin"));
            }
        }
        else
        {
            auto* spin = new QSpinBox(this);
            spin->setRange(static_cast<int>(field.minimum), static_cast<int>(field.maximum));
            editor = spin;
        }
        editor->setToolTip(field.toolTip);
        editor->setMinimumWidth(140);
        editor->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        m_bindings.push_back({field.member, editor});
        auto* fieldWidget = new QWidget(this);
        fieldWidget->setMinimumWidth(140);
        fieldWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        auto* fieldLayout = new QVBoxLayout(fieldWidget);
        fieldLayout->setContentsMargins(0, 0, 0, 0);
        fieldLayout->setSpacing(6);
        auto* label = new QLabel(field.label, fieldWidget);
        label->setToolTip(field.toolTip);
        label->setProperty("secondaryText", true);
        label->setMinimumHeight(16);
        fieldLayout->addWidget(label);
        fieldLayout->addWidget(editor);
        fieldGrid->addWidget(fieldWidget, sectionFieldRows[field.section] + field.row, field.column);
    }
    m_errorNormMode = new QComboBox(this);
    m_errorNormMode->setObjectName(QStringLiteral("tssbnErrorNormCombo"));
    m_errorNormMode->addItem(QStringLiteral("位移、速度整体范数（两者取最大值）"),
                             static_cast<int>(SolverNameSpace::TssbnErrorNormMode::GlobalMax));
    m_errorNormMode->addItem(QStringLiteral("位移整体范数、速度逐自由度最大值"),
                             static_cast<int>(SolverNameSpace::TssbnErrorNormMode::DisplacementGlobalVelocityMax));
    m_errorNormMode->addItem(QStringLiteral("位移和速度逐自由度最大值"),
                             static_cast<int>(SolverNameSpace::TssbnErrorNormMode::ComponentwiseMax));
    m_errorNormMode->addItem(QStringLiteral("位移最大值、速度均方根"),
                             static_cast<int>(SolverNameSpace::TssbnErrorNormMode::DisplacementMaxVelocityRms));
    m_errorNormMode->addItem(QStringLiteral("旧版全局向量范数"),
                             static_cast<int>(SolverNameSpace::TssbnErrorNormMode::LegacyGlobal));
    m_errorNormMode->setToolTip(QStringLiteral("默认分别按位移、速度的整体向量范数判断误差，两项取最大值。"
                                               "整体尺度可能放宽局部小幅运动的误差；逐自由度模式可用于局部响应复核。"
                                               "速度均方根模式可用于舞动包络初筛。"));
    m_referenceIndependentError = new QCheckBox(QStringLiteral("逐自由度控制并恢复端点加速度"), this);
    m_referenceIndependentError->setObjectName(QStringLiteral("tssbnReferenceIndependentErrorCheck"));
    m_referenceIndependentError->setToolTip(
        QStringLiteral("开启时对位移、速度逐自由度设限，并恢复满足运动方程的端点加速度。"
                       "关闭时采用整体组合范数和原参考端点更新。两种模式均估计位移和速度误差。"));
    m_referenceIndependentError->setVisible(false);
    auto* errorControlRow = new QWidget(this);
    auto* errorControlLayout = new QHBoxLayout(errorControlRow);
    errorControlLayout->setContentsMargins(0, 0, 0, 0);
    errorControlLayout->setSpacing(12);
    errorControlLayout->addWidget(m_errorNormMode, 1);
    errorControlLayout->addWidget(m_referenceIndependentError);
    fieldGrid->addWidget(errorControlRow, 5, 0, 1, 3);
    connect(m_referenceIndependentError, &QCheckBox::toggled, this,
            [this](bool enabled)
            {
                if (!m_referenceMode)
                    return;
                if (!enabled)
                    m_referenceIndependentVelocityTolerance = m_velocityAbsoluteTolerance->value();
                applyReferenceSettings();
            });
    connect(m_absoluteTolerance, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double value)
            {
                if (m_referenceMode && !m_referenceIndependentError->isChecked())
                    m_velocityAbsoluteTolerance->setValue(value);
            });
    for (int column = 0; column < 3; ++column)
    {
        fieldGrid->setColumnMinimumWidth(column, 140);
        fieldGrid->setColumnStretch(column, 1);
    }
    setSettings(Settings{});
}

void AdaptiveTssbnSettingsEditor::setSettings(const Settings& settings)
{
    const QSignalBlocker independentBlocker(m_referenceIndependentError);
    m_referenceIndependentError->setChecked(settings.referenceIndependentErrorControl);
    m_referenceIndependentVelocityTolerance = settings.velocityAbsoluteTolerance;
    m_errorNormMode->setCurrentIndex(m_errorNormMode->findData(static_cast<int>(settings.errorNormMode)));
    for (const Binding& binding : m_bindings)
    {
        if (const auto member = std::get_if<DoubleMember>(&binding.member))
            static_cast<QDoubleSpinBox*>(binding.editor)->setValue(settings.*(*member));
        else
            static_cast<QSpinBox*>(binding.editor)->setValue(settings.*std::get<IntMember>(binding.member));
    }
    if (m_referenceMode)
    {
        m_regularErrorNormMode = settings.errorNormMode;
        m_regularVelocityTolerance = settings.velocityAbsoluteTolerance;
        applyReferenceSettings();
    }
}

void AdaptiveTssbnSettingsEditor::setReferenceMode(bool enabled)
{
    if (enabled == m_referenceMode)
        return;
    if (enabled)
    {
        m_regularErrorNormMode =
            static_cast<SolverNameSpace::TssbnErrorNormMode>(m_errorNormMode->currentData().toInt());
        m_regularVelocityTolerance = m_velocityAbsoluteTolerance->value();
    }
    if (m_referenceMode && m_referenceIndependentError->isChecked())
        m_referenceIndependentVelocityTolerance = m_velocityAbsoluteTolerance->value();
    m_referenceMode = enabled;
    setTitle(enabled ? QStringLiteral("参考 TSSBN 参数") : QStringLiteral("自适应 TSSBN 参数"));
    m_errorNormMode->setEnabled(!enabled);
    m_velocityAbsoluteTolerance->setReadOnly(enabled);
    m_referenceIndependentError->setVisible(enabled);
    if (enabled)
        applyReferenceSettings();
    else
    {
        m_errorNormMode->setCurrentIndex(m_errorNormMode->findData(static_cast<int>(m_regularErrorNormMode)));
        m_velocityAbsoluteTolerance->setValue(m_regularVelocityTolerance);
    }
}

AdaptiveTssbnSettingsEditor::Settings AdaptiveTssbnSettingsEditor::settings() const
{
    Settings value;
    value.referenceIndependentErrorControl = m_referenceMode && m_referenceIndependentError->isChecked();
    value.errorNormMode = static_cast<SolverNameSpace::TssbnErrorNormMode>(m_errorNormMode->currentData().toInt());
    for (const Binding& binding : m_bindings)
    {
        if (const auto member = std::get_if<DoubleMember>(&binding.member))
            value.*(*member) = static_cast<QDoubleSpinBox*>(binding.editor)->value();
        else
            value.*std::get<IntMember>(binding.member) = static_cast<QSpinBox*>(binding.editor)->value();
    }
    return value;
}

void AdaptiveTssbnSettingsEditor::commitEditors()
{
    for (const Binding& binding : m_bindings)
        binding.editor->interpretText();
}

void AdaptiveTssbnSettingsEditor::applyReferenceSettings()
{
    const bool independent = m_referenceIndependentError->isChecked();
    m_errorNormMode->setCurrentIndex(
        m_errorNormMode->findData(static_cast<int>(independent ? SolverNameSpace::TssbnErrorNormMode::ComponentwiseMax
                                                               : SolverNameSpace::TssbnErrorNormMode::LegacyGlobal)));
    m_velocityAbsoluteTolerance->setReadOnly(!independent);
    m_velocityAbsoluteTolerance->setValue(independent ? m_referenceIndependentVelocityTolerance
                                                      : m_absoluteTolerance->value());
}
