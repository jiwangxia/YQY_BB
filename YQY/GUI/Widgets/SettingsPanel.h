#pragma once

#include <QWidget>

class QLabel;
class QCheckBox;
class QComboBox;
class QSpinBox;

namespace Ui
{
class SettingsPanelClass;
}

class SettingsPanel final : public QWidget
{
public:
    explicit SettingsPanel(QWidget* parent = nullptr);
    ~SettingsPanel() override;

    QSpinBox* concurrencySpin() const;
    QSpinBox* assemblyThreadsSpin() const;
    QCheckBox* gpuSolverCheckBox() const;
    QComboBox* linearSolverModeCombo() const;
    QSpinBox* resultBatchFramesSpin() const;
    QCheckBox* backgroundResultWriteCheckBox() const;
    QComboBox* nodeLabelModeCombo() const;
    QLabel* statusLabel() const;

private:
    Ui::SettingsPanelClass* m_ui = nullptr;
};
