#include "Dialogs/PropertyItemEditorDialog.h"
#include "Widgets/DialogSizing.h"
#include "ui_PropertyItemEditorDialog.h"
#include <QDoubleSpinBox>
PropertyItemEditorDialog::PropertyItemEditorDialog(QWidget* p)
    : QDialog(p)
    , m_ui(new Ui::PropertyItemEditorDialogClass)
{
    m_ui->setupUi(this);
    DialogSizing::lockHeightToContents(this);
}
PropertyItemEditorDialog::~PropertyItemEditorDialog()
{
    delete m_ui;
}
static void number(QDoubleSpinBox* s, double value, double min, double max, int decimals)
{
    s->setRange(min, max);
    s->setDecimals(decimals);
    s->setValue(value);
    s->setKeyboardTracking(false);
}
void PropertyItemEditorDialog::showMaterial(bool material)
{
    const QList<QWidget*> materialWidgets = {
        m_ui->youngCaption, m_ui->youngSpin,     m_ui->poissonCaption, m_ui->poissonSpin,      m_ui->densityCaption,
        m_ui->densitySpin,  m_ui->stressCaption, m_ui->stressSpin,     m_ui->expansionCaption, m_ui->expansionSpin};
    for (auto* w : materialWidgets)
        w->setVisible(material);
    const QList<QWidget*> sectionWidgets = {m_ui->typeCaption,  m_ui->typeValue, m_ui->areaCaption,   m_ui->areaSpin,
                                            m_ui->widthCaption, m_ui->widthSpin, m_ui->heightCaption, m_ui->heightSpin};
    for (auto* w : sectionWidgets)
        w->setVisible(!material);
    m_ui->idCaption->setText(material ? QStringLiteral("材料 ID") : QStringLiteral("截面 ID"));
}
void PropertyItemEditorDialog::configureMaterial(const QString& s, int id, double y, double p, double d, double st,
                                                 double e)
{
    showMaterial(true);
    m_ui->sourceValue->setText(s);
    m_ui->idValue->setText(QString::number(id));
    number(m_ui->youngSpin, y, 1e-12, 1e15, 6);
    number(m_ui->poissonSpin, p, -.999999, .499999, 8);
    number(m_ui->densitySpin, d, 1e-12, 1e9, 8);
    number(m_ui->stressSpin, st, 0, 1e15, 6);
    number(m_ui->expansionSpin, e, 0, 1, 12);
}
void PropertyItemEditorDialog::configureSection(const QString& s, int id, const QString& t, double a, bool rectangle,
                                                double w, double h)
{
    showMaterial(false);
    m_ui->sourceValue->setText(s);
    m_ui->idValue->setText(QString::number(id));
    m_ui->typeValue->setText(t);
    number(m_ui->areaSpin, a, 1e-16, 1e6, 12);
    number(m_ui->widthSpin, w, 1e-12, 1e6, 10);
    number(m_ui->heightSpin, h, 1e-12, 1e6, 10);
    m_ui->areaSpin->setReadOnly(rectangle);
    m_ui->widthCaption->setVisible(rectangle);
    m_ui->widthSpin->setVisible(rectangle);
    m_ui->heightCaption->setVisible(rectangle);
    m_ui->heightSpin->setVisible(rectangle);
}
#define G(name)                                                                                                        \
    QDoubleSpinBox* PropertyItemEditorDialog::name() const                                                             \
    {                                                                                                                  \
        return m_ui->name##Spin;                                                                                       \
    }
G(young)
G(poisson)
G(density)
G(stress)
G(expansion)
G(area)
G(width)
G(height)
#undef G
    QDialogButtonBox* PropertyItemEditorDialog::buttons() const
{
    return m_ui->buttonBox;
}
