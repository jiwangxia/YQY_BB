#pragma once
#include <QDialog>
class QPushButton; class QTableWidget; namespace Ui { class SolveTaskManagerDialogClass; }
class SolveTaskManagerDialog final : public QDialog
{
public:
    explicit SolveTaskManagerDialog(QWidget* parent=nullptr); ~SolveTaskManagerDialog() override;
    QPushButton* solveAllButton() const;
    QPushButton* restartAllButton() const;
    QTableWidget* taskTable() const;
private: Ui::SolveTaskManagerDialogClass* m_ui=nullptr;
};
