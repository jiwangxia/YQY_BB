#pragma once

#include <QString>
#include <QVector>

#include <memory>
#include "Base/EmptyOUT.h"

class Property;
class StructureData;

namespace Conductor
{
struct MaterialPreset
{
    int id = 0;
    QString category;
    QString name;
    double young = 0.0;
    double poisson = 0.0;
    double density = 0.0;
    double maxStress = 0.0;
    double expansion = 0.0;
};

struct SectionPreset
{
    int id = 0;
    QString category;
    QString name;
    double area = 0.0;
    QString description;
};

class PropertyLibrary
{
public:
    bool load(_OUT QString& error);

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

    bool updateMaterial(int index, const MaterialPreset& value, _OUT QString& error);
    bool updateSection(int index, const SectionPreset& value, _OUT QString& error);

    std::shared_ptr<Property> instantiateProperty(int materialIndex, int sectionIndex, _OUT StructureData& target,
                                                  _OUT QString& error) const;

private:
    bool loadBdf(const QString& filePath, _OUT QString& error);
    bool loadMaterials(const QString& resourcePath, _OUT QString& error);
    bool loadSections(const QString& resourcePath, _OUT QString& error);

    QVector<MaterialPreset> m_materials;
    QVector<SectionPreset> m_sections;
};
}
