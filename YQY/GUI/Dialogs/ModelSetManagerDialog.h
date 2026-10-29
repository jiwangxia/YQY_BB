#pragma once

#include <QDialog>
#include <memory>

class StructureData;
class QTableWidget;

// 以“节点集 + 单元集”为一个逻辑线路集合进行管理，不向普通用户暴露实体 ID。
class ModelSetManagerDialog final : public QDialog
{
public:
    explicit ModelSetManagerDialog(const std::shared_ptr<StructureData>& structure, QWidget* parent = nullptr); // 创建模型线路集合管理窗口

private:
    void refreshTable(const QString& preferredName = {});                                                       // 刷新线路集合列表并恢复指定集合选择
    void createCombinedCollection();                                                                            // 创建配套的节点集和单元集
    void renameSelectedCollection();                                                                            // 重命名选中的线路集合
    void deleteSelectedCollection();                                                                            // 删除选中的线路集合
    int selectedNodeSetId() const;                                                                              // 读取当前集合的节点集编号
    int selectedElementSetId() const;                                                                           // 读取当前集合的单元集编号
    QString selectedCollectionName() const;                                                                     // 读取当前选中的线路集合名称

    std::shared_ptr<StructureData> m_structure;
    QTableWidget* m_table = nullptr;
};
