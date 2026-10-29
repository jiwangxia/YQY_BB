#pragma once

#include <QDialog>

#include <memory>

class StructureData;
class QTableWidget;

// 面向线路对象的计算区域开关；节点/单元集合仅作为 H5 内部实现细节。
class ComputeRegionManagerDialog final : public QDialog
{
public:
    explicit ComputeRegionManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr); // 创建计算区域管理窗口

private:
    void refreshTable(int preferredRegionId = -1);                                                                   // 刷新计算区域列表并恢复指定区域选择
    void createRegionFromCollections();                                                                              // 从线路集合创建计算区域
    void deleteSelectedRegion();                                                                                     // 删除当前选中的计算区域
    int selectedRegionId() const;                                                                                    // 读取当前选中计算区域的编号

    std::shared_ptr<StructureData> m_structure;
    QTableWidget* m_table = nullptr;
    bool m_refreshing = false;
};
