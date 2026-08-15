#pragma once

#include <QDialog>
#include <memory>

class StructureData;
class QTableWidget;

class ModelSetManagerDialog final : public QDialog
{
public:
    explicit ModelSetManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr);

private:
    void refreshTable(int preferredSetId = -1);
    void editSet(int setId = -1);
    void deleteSelectedSet();
    int selectedSetId() const;

    std::shared_ptr<StructureData> m_structure;
    QTableWidget* m_table = nullptr;
};
