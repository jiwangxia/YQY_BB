#include "AdaptiveTimeStepSettingsEditor.h"
#include "Widgets/CompactDoubleSpinBox.h"
#include <QDoubleSpinBox>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
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

AdaptiveTimeStepSettingsEditor::AdaptiveTimeStepSettingsEditor(QWidget* parent)
    : QGroupBox(QStringLiteral("Newmark / RK 自适应步长参数"), parent)
{
    auto* contentLayout = new QVBoxLayout(this);
    contentLayout->setAlignment(Qt::AlignTop);
    auto* fieldGrid = new QGridLayout;
    fieldGrid->setContentsMargins(0, 0, 0, 0);
    fieldGrid->setHorizontalSpacing(12);
    fieldGrid->setVerticalSpacing(7);
    contentLayout->addLayout(fieldGrid);
    fieldGrid->addWidget(createSectionTitle(QStringLiteral("步长范围")), 0, 0);
    fieldGrid->addWidget(createSectionTitle(QStringLiteral("逐自由度最大误差")), 0, 1);
    fieldGrid->addWidget(createSectionTitle(QStringLiteral("步长调整")), 0, 2);
    auto* stepLayout = new QGridLayout;
    auto* errorLayout = new QGridLayout;
    auto* controlLayout = new QGridLayout;
    const std::array<QGridLayout*, 3> sectionLayouts{stepLayout, errorLayout, controlLayout};
    for (int column = 0; column < 3; ++column)
    {
        auto* sectionLayout = sectionLayouts[column];
        sectionLayout->setContentsMargins(0, 0, 0, 0);
        sectionLayout->setVerticalSpacing(7);
        sectionLayout->setColumnMinimumWidth(0, 140);
        sectionLayout->setColumnStretch(0, 1);
        fieldGrid->addLayout(sectionLayout, 1, column, Qt::AlignTop);
        fieldGrid->setColumnMinimumWidth(column, 140);
        fieldGrid->setColumnStretch(column, 1);
    }
    addDoubleField(stepLayout, 0, 0, QStringLiteral("最小步长"), 1.0e-12, 1.0e12, 12, m_minimumTimeStep);
    addDoubleField(stepLayout, 1, 0, QStringLiteral("最大步长"), 1.0e-12, 1.0e12, 12, m_maximumTimeStep);
    auto* stepNote = new QLabel(QStringLiteral("初始步长在左侧“步长 / 增量”中设置。"), this);
    stepNote->setProperty("secondaryText", true);
    stepNote->setWordWrap(true);
    stepLayout->addWidget(stepNote, 2, 0);

    addDoubleField(errorLayout, 0, 0, QStringLiteral("位移相对容差"), 1.0e-12, 1.0, 12, m_relativeTolerance);
    addDoubleField(errorLayout, 1, 0, QStringLiteral("位移绝对容差"), 1.0e-14, 1.0, 14, m_absoluteTolerance);
    addDoubleField(errorLayout, 2, 0, QStringLiteral("速度相对容差"), 1.0e-12, 1.0, 12, m_velocityRelativeTolerance);
    addDoubleField(errorLayout, 3, 0, QStringLiteral("速度绝对容差"), 1.0e-14, 1.0, 14, m_velocityAbsoluteTolerance);

    addDoubleField(controlLayout, 0, 0, QStringLiteral("安全系数"), 0.01, 1.0, 6, m_safetyFactor);
    addDoubleField(controlLayout, 1, 0, QStringLiteral("最小倍率"), 0.01, 0.99, 6, m_minimumScaleFactor);
    addDoubleField(controlLayout, 2, 0, QStringLiteral("最大增长倍率"), 1.0, 100.0, 6, m_maximumGrowthFactor);
    addDoubleField(controlLayout, 3, 0, QStringLiteral("求解失败倍率"), 0.01, 0.99, 6, m_solveFailureFactor);
    auto* rejectedField = new QWidget(this);
    rejectedField->setMinimumWidth(140);
    rejectedField->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto* rejectedLayout = new QVBoxLayout(rejectedField);
    rejectedLayout->setContentsMargins(0, 0, 0, 0);
    rejectedLayout->setSpacing(6);
    auto* rejectedLabel = new QLabel(QStringLiteral("连续拒绝上限"), rejectedField);
    rejectedLabel->setProperty("secondaryText", true);
    rejectedLabel->setMinimumHeight(16);
    rejectedLayout->addWidget(rejectedLabel);
    m_maximumRejectedAttempts = new QSpinBox(rejectedField);
    m_maximumRejectedAttempts->setRange(1, 1000);
    m_maximumRejectedAttempts->setMinimumWidth(140);
    m_maximumRejectedAttempts->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    rejectedLayout->addWidget(m_maximumRejectedAttempts);
    controlLayout->addWidget(rejectedField, 4, 0);
    setSettings(Settings{});
}

void AdaptiveTimeStepSettingsEditor::setSettings(const Settings& settings)
{
    m_minimumTimeStep->setValue(settings.minimumTimeStep);
    m_maximumTimeStep->setValue(settings.maximumTimeStep);
    m_relativeTolerance->setValue(settings.relativeTolerance);
    m_absoluteTolerance->setValue(settings.absoluteTolerance);
    m_velocityRelativeTolerance->setValue(settings.velocityRelativeTolerance);
    m_velocityAbsoluteTolerance->setValue(settings.velocityAbsoluteTolerance);
    m_safetyFactor->setValue(settings.safetyFactor);
    m_minimumScaleFactor->setValue(settings.minimumScaleFactor);
    m_maximumGrowthFactor->setValue(settings.maximumGrowthFactor);
    m_solveFailureFactor->setValue(settings.solveFailureFactor);
    m_maximumRejectedAttempts->setValue(settings.maximumRejectedAttempts);
}

AdaptiveTimeStepSettingsEditor::Settings AdaptiveTimeStepSettingsEditor::settings() const
{
    Settings value;
    value.minimumTimeStep = m_minimumTimeStep->value();
    value.maximumTimeStep = m_maximumTimeStep->value();
    value.relativeTolerance = m_relativeTolerance->value();
    value.absoluteTolerance = m_absoluteTolerance->value();
    value.velocityRelativeTolerance = m_velocityRelativeTolerance->value();
    value.velocityAbsoluteTolerance = m_velocityAbsoluteTolerance->value();
    value.safetyFactor = m_safetyFactor->value();
    value.minimumScaleFactor = m_minimumScaleFactor->value();
    value.maximumGrowthFactor = m_maximumGrowthFactor->value();
    value.solveFailureFactor = m_solveFailureFactor->value();
    value.maximumRejectedAttempts = m_maximumRejectedAttempts->value();
    return value;
}

void AdaptiveTimeStepSettingsEditor::commitEditors()
{
    for (QDoubleSpinBox* editor : m_doubleEditors)
        editor->interpretText();
    m_maximumRejectedAttempts->interpretText();
}

QLabel* AdaptiveTimeStepSettingsEditor::createSectionTitle(const QString& text)
{
    auto* label = new QLabel(text, this);
    QFont font = label->font();
    font.setBold(true);
    label->setFont(font);
    return label;
}

void AdaptiveTimeStepSettingsEditor::addDoubleField(QGridLayout* layout, int row, int column, const QString& text,
                                                    double minimum, double maximum, int decimals,
                                                    _OUT QDoubleSpinBox*& editor)
{
    auto* field = new QWidget(this);
    field->setMinimumWidth(140);
    field->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto* fieldLayout = new QVBoxLayout(field);
    fieldLayout->setContentsMargins(0, 0, 0, 0);
    fieldLayout->setSpacing(6);
    auto* label = new QLabel(text, field);
    label->setProperty("secondaryText", true);
    label->setMinimumHeight(16);
    fieldLayout->addWidget(label);
    auto* spin = new CompactDoubleSpinBox(field);
    spin->setRange(minimum, maximum);
    spin->setDecimals(decimals);
    spin->setKeyboardTracking(false);
    spin->setMinimumWidth(140);
    spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    fieldLayout->addWidget(spin);
    layout->addWidget(field, row, column);
    editor = spin;
    m_doubleEditors.push_back(spin);
}
