#pragma once

#include <QDialog>
#include <memory>

class StructureData;
class QTableWidget;

// 编辑结构计算区域的对话框。
class ComputeRegionManagerDialog final : public QDialog
{
public:
    // 使用目标结构初始化区域列表。
    explicit ComputeRegionManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr);

private:
    void refreshTable(int preferredRegionId = -1); // 刷新区域表格并尝试保留选择
    void editRegion(int regionId = -1); // 编辑或新建一个区域
    void deleteSelectedRegion(); // 删除当前选中区域
    void manageSets(); // 打开模型集管理对话框
    int selectedRegionId() const; // 返回当前选中区域编号

    std::shared_ptr<StructureData> m_structure; // 被编辑的结构数据
    QTableWidget* m_table = nullptr; // 区域列表表格
};
