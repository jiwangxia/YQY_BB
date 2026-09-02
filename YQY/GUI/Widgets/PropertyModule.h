#pragma once
#include <QWidget>
class QTabWidget;
class QTableWidget;
class QTreeWidget;
class QPushButton;
namespace Ui
{
class PropertyModuleClass;
}
class PropertyModule final : public QWidget
{
public:
    explicit PropertyModule(QWidget* p = nullptr);
    ~PropertyModule() override;
    QTabWidget* tabs() const;
    QTableWidget* materialTable() const;
    QTableWidget* sectionTable() const;
    QTreeWidget* materialTree() const;
    QTreeWidget* sectionTree() const;
    QTreeWidget* springBehaviorTree() const;
    QPushButton* refreshButton() const;
    QPushButton* applyButton() const;

private:
    Ui::PropertyModuleClass* m_ui = nullptr;
};
