#pragma once

#include <QDialog>
#include <memory>

class StructureData;
class QTableWidget;

// 编辑节点集和单元集的对话框。
class ModelSetManagerDialog final : public QDialog
{
public:
    // 使用目标结构初始化模型集列表。
    explicit ModelSetManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr);

private:
    void refreshTable(int preferredSetId = -1); // 刷新模型集表格并尝试保留选择
    void editSet(int setId = -1); // 编辑或新建一个模型集
    void deleteSelectedSet(); // 删除当前选中模型集
    int selectedSetId() const; // 返回当前选中模型集编号

    std::shared_ptr<StructureData> m_structure; // 被编辑的结构数据
    QTableWidget* m_table = nullptr; // 模型集列表表格
};
