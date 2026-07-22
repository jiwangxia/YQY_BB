#pragma once

#include <QWidget>

class QLabel;
class QSpinBox;

namespace Ui { class SettingsPanelClass; }

class SettingsPanel final : public QWidget
{
public:
    explicit SettingsPanel(QWidget* parent = nullptr);
    ~SettingsPanel() override;

    QSpinBox* concurrencySpin() const;
    QLabel* statusLabel() const;

private:
    Ui::SettingsPanelClass* m_ui = nullptr;
};
