#pragma once
#include <QDialog>
class QDialogButtonBox;
class QDoubleSpinBox;
namespace Ui
{
class PropertyItemEditorDialogClass;
}
class PropertyItemEditorDialog final : public QDialog
{
public:
    explicit PropertyItemEditorDialog(QWidget* p = nullptr);
    ~PropertyItemEditorDialog() override;
    void configureMaterial(const QString& source, int id, double young, double poisson, double density, double stress,
                           double expansion);
    void configureSection(const QString& source, int id, const QString& type, double area, bool rectangle,
                          double width = 0, double height = 0);
    QDoubleSpinBox* young() const;
    QDoubleSpinBox* poisson() const;
    QDoubleSpinBox* density() const;
    QDoubleSpinBox* stress() const;
    QDoubleSpinBox* expansion() const;
    QDoubleSpinBox* area() const;
    QDoubleSpinBox* width() const;
    QDoubleSpinBox* height() const;
    QDialogButtonBox* buttons() const;

private:
    void showMaterial(bool);
    Ui::PropertyItemEditorDialogClass* m_ui = nullptr;
};
