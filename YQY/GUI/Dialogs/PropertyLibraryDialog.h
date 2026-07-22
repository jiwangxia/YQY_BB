#pragma once
#include <QDialog>
class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QTableWidget;
class QTabWidget;
namespace Ui
{
class PropertyLibraryDialogClass;
}
class PropertyLibraryDialog final : public QDialog
{
public:
    explicit PropertyLibraryDialog(QWidget* p = nullptr);
    ~PropertyLibraryDialog() override;
    QTabWidget* tabs() const;
    QTableWidget* materialTable() const;
    QTableWidget* sectionTable() const;
    QCheckBox* syncCheck() const;
    QComboBox* targetMaterialCombo() const;
    QComboBox* targetSectionCombo() const;
    QDialogButtonBox* buttons() const;

private:
    Ui::PropertyLibraryDialogClass* m_ui = nullptr;
};
