#pragma once

#include <QHash>
#include <QString>

// 维护模型与对应结果文件的工作区关联。
class WorkspaceController final
{
public:
    void associateResult(int modelId, const QString& resultFile); // 绑定模型与结果文件
    QString resultForModel(int modelId) const; // 查询模型关联的结果文件
    void removeModel(int modelId); // 删除一个模型的关联记录
    void clear(); // 清空全部关联记录

private:
    QHash<int, QString> m_resultFiles; // 模型编号到结果文件路径的映射
};
