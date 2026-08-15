#pragma once

#include <QDialog>
#include <memory>

class StructureData;
class QTableWidget;

class ComputeRegionManagerDialog final : public QDialog
{
public:
    explicit ComputeRegionManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr);

private:
    void refreshTable(int preferredRegionId = -1);
    void editRegion(int regionId = -1);
    void deleteSelectedRegion();
    void manageSets();
    int selectedRegionId() const;

    std::shared_ptr<StructureData> m_structure;
    QTableWidget* m_table = nullptr;
};
