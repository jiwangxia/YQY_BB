#include "Widgets/SettingsPanel.h"

#include "ui_SettingsPanel.h"

#include <QLabel>
#include <QComboBox>
#include <QSpinBox>

SettingsPanel::SettingsPanel(QWidget* parent)
    : QWidget(parent)
    , m_ui(new Ui::SettingsPanelClass)
{
    m_ui->setupUi(this);
    m_ui->titleLabel->setObjectName(QStringLiteral("cardTitle"));
    m_ui->descriptionLabel->setObjectName(QStringLiteral("mutedText"));
    m_ui->statusLabel->setObjectName(QStringLiteral("mutedText"));
}

SettingsPanel::~SettingsPanel()
{
    delete m_ui;
}

QSpinBox* SettingsPanel::concurrencySpin() const
{
    return m_ui->concurrencySpin;
}

QComboBox* SettingsPanel::nodeLabelModeCombo() const
{
    return m_ui->nodeLabelModeCombo;
}

QLabel* SettingsPanel::statusLabel() const
{
    return m_ui->statusLabel;
}
