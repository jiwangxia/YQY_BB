#include "Dialogs/PropertyItemEditorDialog.h"
#include <QVariant>
#include <QAbstractButton>
#include <QApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QVBoxLayout>
#include "Widgets/CompactDoubleSpinBox.h"
#include "Utility/UiIcons.h"
#include "Widgets/DialogSizing.h"
#include <QDoubleSpinBox>
#include <QPushButton>

PropertyItemEditorDialog::PropertyItemEditorDialog(QWidget* p)
    : QDialog(p)
{
    initializeUi();
    setWindowIcon(UiIcons::applicationIcon());
    m_buttonBox->button(QDialogButtonBox::Save)->setText(QStringLiteral("保存"));
    m_buttonBox->button(QDialogButtonBox::Save)->setProperty("primaryAction", true);
    m_buttonBox->button(QDialogButtonBox::Save)->setDefault(true);
    m_buttonBox->button(QDialogButtonBox::Cancel)->setText(QStringLiteral("取消"));
    m_buttonBox->button(QDialogButtonBox::Cancel)->setAutoDefault(false);
    m_sourceValue->setProperty("secondaryText", true);
    auto sourcePolicy = m_sourceValue->sizePolicy();
    sourcePolicy.setHorizontalPolicy(QSizePolicy::Ignored);
    m_sourceValue->setSizePolicy(sourcePolicy);
    resize(540, 400);
    DialogSizing::fitCompactFormToScreen(this);
}
PropertyItemEditorDialog::~PropertyItemEditorDialog() = default;
static void number(QDoubleSpinBox* s, double value, double min, double max, int decimals)
{
    s->setRange(min, max);
    s->setDecimals(decimals);
    s->setValue(value);
    s->setKeyboardTracking(false);
}
void PropertyItemEditorDialog::showMaterial(bool material)
{
    const QList<QWidget*> materialWidgets = {m_youngCaption,     m_youngSpin,    m_poissonCaption, m_poissonSpin,
                                             m_densityCaption,   m_densitySpin,  m_stressCaption,  m_stressSpin,
                                             m_expansionCaption, m_expansionSpin};
    for (auto* w : materialWidgets)
        w->setVisible(material);
    const QList<QWidget*> sectionWidgets = {m_typeCaption,  m_typeValue, m_areaCaption,   m_areaSpin,
                                            m_widthCaption, m_widthSpin, m_heightCaption, m_heightSpin};
    for (auto* w : sectionWidgets)
        w->setVisible(!material);
    m_idCaption->setText(material ? QStringLiteral("材料 ID") : QStringLiteral("截面 ID"));
}
void PropertyItemEditorDialog::configureMaterial(const QString& s, int id, double y, double p, double d, double st,
                                                 double e)
{
    showMaterial(true);
    m_sourceValue->setText(s);
    m_sourceValue->setToolTip(s);
    m_idValue->setText(QString::number(id));
    number(m_youngSpin, y, 1e-12, 1e15, 6);
    number(m_poissonSpin, p, -.999999, .499999, 8);
    number(m_densitySpin, d, 1e-12, 1e9, 8);
    number(m_stressSpin, st, 0, 1e15, 6);
    number(m_expansionSpin, e, 0, 1, 12);
}
void PropertyItemEditorDialog::configureSection(const QString& s, int id, const QString& t, double a, bool rectangle,
                                                double w, double h)
{
    showMaterial(false);
    m_sourceValue->setText(s);
    m_sourceValue->setToolTip(s);
    m_idValue->setText(QString::number(id));
    m_typeValue->setText(t);
    number(m_areaSpin, a, 1e-16, 1e6, 12);
    number(m_widthSpin, w, 1e-12, 1e6, 10);
    number(m_heightSpin, h, 1e-12, 1e6, 10);
    m_areaSpin->setReadOnly(rectangle);
    m_areaSpin->setButtonSymbols(rectangle ? QAbstractSpinBox::NoButtons : QAbstractSpinBox::UpDownArrows);
    m_areaSpin->setToolTip(rectangle ? QStringLiteral("矩形截面面积由宽度和高度计算，保存后应用。") : QString());
    m_widthCaption->setVisible(rectangle);
    m_widthSpin->setVisible(rectangle);
    m_heightCaption->setVisible(rectangle);
    m_heightSpin->setVisible(rectangle);
}
QDoubleSpinBox* PropertyItemEditorDialog::young() const
{
    return m_youngSpin;
}

QDoubleSpinBox* PropertyItemEditorDialog::poisson() const
{
    return m_poissonSpin;
}

QDoubleSpinBox* PropertyItemEditorDialog::density() const
{
    return m_densitySpin;
}

QDoubleSpinBox* PropertyItemEditorDialog::stress() const
{
    return m_stressSpin;
}

QDoubleSpinBox* PropertyItemEditorDialog::expansion() const
{
    return m_expansionSpin;
}

QDoubleSpinBox* PropertyItemEditorDialog::area() const
{
    return m_areaSpin;
}

QDoubleSpinBox* PropertyItemEditorDialog::width() const
{
    return m_widthSpin;
}

QDoubleSpinBox* PropertyItemEditorDialog::height() const
{
    return m_heightSpin;
}
QDialogButtonBox* PropertyItemEditorDialog::buttons() const
{
    return m_buttonBox;
}

void PropertyItemEditorDialog::initializeUi()
{
    if (this->objectName().isEmpty())
    {
        setObjectName("PropertyItemEditorDialogClass");
    }
    setMinimumSize(QSize(460, 0));
    QVBoxLayout* rootLayout = new QVBoxLayout(this);
    rootLayout->setSpacing(16);
    rootLayout->setObjectName("rootLayout");
    rootLayout->setContentsMargins(20, 20, 20, 20);
    QFormLayout* formLayout = new QFormLayout();
    formLayout->setObjectName("formLayout");
    formLayout->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    formLayout->setRowWrapPolicy(QFormLayout::DontWrapRows);
    formLayout->setHorizontalSpacing(18);
    formLayout->setVerticalSpacing(10);
    QLabel* sourceCaption = new QLabel(this);
    sourceCaption->setObjectName("sourceCaption");

    formLayout->setWidget(0, QFormLayout::LabelRole, sourceCaption);

    m_sourceValue = new QLabel(this);
    m_sourceValue->setObjectName("sourceValue");
    m_sourceValue->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_sourceValue->setWordWrap(true);

    formLayout->setWidget(0, QFormLayout::FieldRole, m_sourceValue);

    m_idCaption = new QLabel(this);
    m_idCaption->setObjectName("idCaption");

    formLayout->setWidget(1, QFormLayout::LabelRole, m_idCaption);

    m_idValue = new QLabel(this);
    m_idValue->setObjectName("idValue");
    m_idValue->setTextInteractionFlags(Qt::TextSelectableByMouse);

    formLayout->setWidget(1, QFormLayout::FieldRole, m_idValue);

    m_typeCaption = new QLabel(this);
    m_typeCaption->setObjectName("typeCaption");

    formLayout->setWidget(2, QFormLayout::LabelRole, m_typeCaption);

    m_typeValue = new QLabel(this);
    m_typeValue->setObjectName("typeValue");
    m_typeValue->setTextInteractionFlags(Qt::TextSelectableByMouse);

    formLayout->setWidget(2, QFormLayout::FieldRole, m_typeValue);

    m_youngCaption = new QLabel(this);
    m_youngCaption->setObjectName("youngCaption");

    formLayout->setWidget(3, QFormLayout::LabelRole, m_youngCaption);

    m_youngSpin = new CompactDoubleSpinBox(this);
    m_youngSpin->setObjectName("youngSpin");
    m_youngSpin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    formLayout->setWidget(3, QFormLayout::FieldRole, m_youngSpin);

    m_poissonCaption = new QLabel(this);
    m_poissonCaption->setObjectName("poissonCaption");

    formLayout->setWidget(4, QFormLayout::LabelRole, m_poissonCaption);

    m_poissonSpin = new CompactDoubleSpinBox(this);
    m_poissonSpin->setObjectName("poissonSpin");
    m_poissonSpin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    formLayout->setWidget(4, QFormLayout::FieldRole, m_poissonSpin);

    m_densityCaption = new QLabel(this);
    m_densityCaption->setObjectName("densityCaption");

    formLayout->setWidget(5, QFormLayout::LabelRole, m_densityCaption);

    m_densitySpin = new CompactDoubleSpinBox(this);
    m_densitySpin->setObjectName("densitySpin");
    m_densitySpin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    formLayout->setWidget(5, QFormLayout::FieldRole, m_densitySpin);

    m_stressCaption = new QLabel(this);
    m_stressCaption->setObjectName("stressCaption");

    formLayout->setWidget(6, QFormLayout::LabelRole, m_stressCaption);

    m_stressSpin = new CompactDoubleSpinBox(this);
    m_stressSpin->setObjectName("stressSpin");
    m_stressSpin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    formLayout->setWidget(6, QFormLayout::FieldRole, m_stressSpin);

    m_expansionCaption = new QLabel(this);
    m_expansionCaption->setObjectName("expansionCaption");

    formLayout->setWidget(7, QFormLayout::LabelRole, m_expansionCaption);

    m_expansionSpin = new CompactDoubleSpinBox(this);
    m_expansionSpin->setObjectName("expansionSpin");
    m_expansionSpin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    formLayout->setWidget(7, QFormLayout::FieldRole, m_expansionSpin);

    m_areaCaption = new QLabel(this);
    m_areaCaption->setObjectName("areaCaption");

    formLayout->setWidget(8, QFormLayout::LabelRole, m_areaCaption);

    m_areaSpin = new CompactDoubleSpinBox(this);
    m_areaSpin->setObjectName("areaSpin");
    m_areaSpin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    formLayout->setWidget(8, QFormLayout::FieldRole, m_areaSpin);

    m_widthCaption = new QLabel(this);
    m_widthCaption->setObjectName("widthCaption");

    formLayout->setWidget(9, QFormLayout::LabelRole, m_widthCaption);

    m_widthSpin = new CompactDoubleSpinBox(this);
    m_widthSpin->setObjectName("widthSpin");
    m_widthSpin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    formLayout->setWidget(9, QFormLayout::FieldRole, m_widthSpin);

    m_heightCaption = new QLabel(this);
    m_heightCaption->setObjectName("heightCaption");

    formLayout->setWidget(10, QFormLayout::LabelRole, m_heightCaption);

    m_heightSpin = new CompactDoubleSpinBox(this);
    m_heightSpin->setObjectName("heightSpin");
    m_heightSpin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    formLayout->setWidget(10, QFormLayout::FieldRole, m_heightSpin);

    rootLayout->addLayout(formLayout);

    m_buttonBox = new QDialogButtonBox(this);
    m_buttonBox->setObjectName("buttonBox");
    m_buttonBox->setStandardButtons(QDialogButtonBox::Cancel | QDialogButtonBox::Save);

    rootLayout->addWidget(m_buttonBox);

#if QT_CONFIG(shortcut)
    m_youngCaption->setBuddy(m_youngSpin);
    m_poissonCaption->setBuddy(m_poissonSpin);
    m_densityCaption->setBuddy(m_densitySpin);
    m_stressCaption->setBuddy(m_stressSpin);
    m_expansionCaption->setBuddy(m_expansionSpin);
    m_areaCaption->setBuddy(m_areaSpin);
    m_widthCaption->setBuddy(m_widthSpin);
    m_heightCaption->setBuddy(m_heightSpin);
#endif // QT_CONFIG(shortcut)

    sourceCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "来源", nullptr));
    m_idCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "ID", nullptr));
    m_typeCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "截面类型", nullptr));
    m_youngCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "弹性模量 E (Pa)", nullptr));
    m_poissonCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "泊松比", nullptr));
    m_densityCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "密度 (kg/m³)", nullptr));
    m_stressCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "许用应力 (Pa)", nullptr));
    m_expansionCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "热膨胀系数", nullptr));
    m_areaCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "截面面积 (m²)", nullptr));
    m_widthCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "宽度 (m)", nullptr));
    m_heightCaption->setText(QCoreApplication::translate("PropertyItemEditorDialogClass", "高度 (m)", nullptr));
}
