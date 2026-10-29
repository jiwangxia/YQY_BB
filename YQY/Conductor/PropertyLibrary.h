#pragma once

#include <QString>
#include <QVector>

#include <memory>
#include "Base/EmptyOUT.h"

class Property;
class StructureData;

namespace Conductor
{
// 材料预设的一行数据。
struct MaterialPreset
{
    int id = 0;             // 预设编号
    QString category;       // 材料分类
    QString name;           // 材料名称
    double young = 0.0;     // 杨氏模量
    double poisson = 0.0;   // 泊松比
    double density = 0.0;   // 密度
    double maxStress = 0.0; // 许用应力
    double expansion = 0.0; // 线膨胀系数
};

// 截面预设的一行数据。
struct SectionPreset
{
    int id = 0;          // 预设编号
    QString category;    // 截面分类
    QString name;        // 截面名称
    double area = 0.0;   // 截面面积
    QString description; // 截面说明
};

// 产品目录中的每套间隔棒质量；材料牌号需与厂家资料单独核对。
struct SpacerPreset
{
    QString id;
    QString model;
    QString manufacturer;
    QString sourceUrl;
    QString materialName;
    QString materialNote;
    int bundleCount = 0;
    double mass = 0.0;    // 每套质量，kg
    double spacing = 0.0; // 相邻子导线中心距，m
    bool materialVerified = false;
};

// 管理导线建模使用的材料和截面预设。
class PropertyLibrary
{
public:
    bool load(_OUT QString& error);                      // 读取全部预设数据
    static const QVector<SpacerPreset>& spacerPresets(); // 不依赖加载状态的只读产品目录

    const QVector<MaterialPreset>& materials() const
    {
        return m_materials;
    }
    const QVector<SectionPreset>& sections() const
    {
        return m_sections;
    }
    bool isReady() const
    {
        return !m_materials.isEmpty() && !m_sections.isEmpty();
    }

    bool updateMaterial(int index, const MaterialPreset& value, _OUT QString& error); // 更新一个材料预设
    bool updateSection(int index, const SectionPreset& value, _OUT QString& error);   // 更新一个截面预设

    // 按预设创建可挂接到目标结构的属性对象。
    std::shared_ptr<Property> instantiateProperty(int materialIndex, int sectionIndex, _OUT StructureData& target,
                                                  _OUT QString& error) const;

private:
    bool loadBdf(const QString& filePath, _OUT QString& error);           // 读取 BDF 预设文件

    QVector<MaterialPreset> m_materials; // 已加载的材料预设
    QVector<SectionPreset> m_sections;   // 已加载的截面预设
};
}
