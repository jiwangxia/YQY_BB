#include "Dialogs/PropertyLibraryDialog.h"
#include "Widgets/TableAppearance.h"
#include "ui_PropertyLibraryDialog.h"
#include <QHeaderView>
#include <QTableWidget>
PropertyLibraryDialog::PropertyLibraryDialog(QWidget* p)
    : QDialog(p)
    , m_ui(new Ui::PropertyLibraryDialogClass)
{
    m_ui->setupUi(this);
    resize(920, 560);
    for (auto* t : {m_ui->materialTable, m_ui->sectionTable})
    {
        applyCenteredTableAppearance(t);
        t->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
        t->setSelectionBehavior(QAbstractItemView::SelectRows);
        t->setSelectionMode(QAbstractItemView::SingleSelection);
        t->setAlternatingRowColors(true);
        t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        t->horizontalHeader()->setStretchLastSection(true);
    }
}
PropertyLibraryDialog::~PropertyLibraryDialog()
{
    delete m_ui;
}
QTabWidget* PropertyLibraryDialog::tabs() const
{
    return m_ui->tabs;
}
QTableWidget* PropertyLibraryDialog::materialTable() const
{
    return m_ui->materialTable;
}
QTableWidget* PropertyLibraryDialog::sectionTable() const
{
    return m_ui->sectionTable;
}
QCheckBox* PropertyLibraryDialog::syncCheck() const
{
    return m_ui->syncCheck;
}
QComboBox* PropertyLibraryDialog::targetMaterialCombo() const
{
    return m_ui->targetMaterialCombo;
}
QComboBox* PropertyLibraryDialog::targetSectionCombo() const
{
    return m_ui->targetSectionCombo;
}
QDialogButtonBox* PropertyLibraryDialog::buttons() const
{
    return m_ui->buttonBox;
}
