#pragma once
#include <QWidget>
class QPushButton;
namespace Ui
{
class AnalysisSummaryPanelClass;
}
class AnalysisSummaryPanel final : public QWidget
{
public:
    explicit AnalysisSummaryPanel(QWidget* parent = nullptr);
    ~AnalysisSummaryPanel() override;
    QPushButton* stepsButton() const;
    QPushButton* loadsButton() const;
    QPushButton* constraintsButton() const;
    QPushButton* regionsButton() const;

private:
    Ui::AnalysisSummaryPanelClass* m_ui = nullptr;
};
