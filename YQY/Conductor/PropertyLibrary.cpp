#include "PropertyLibrary.h"

#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Structure/StructureData.h"

#include <QFile>
#include <QCoreApplication>
#include <QDir>
#include <QRegularExpression>
#include <QTextStream>

namespace
{
QStringList dataRows(const QString& resourcePath, QString& error)
{
    QFile file(resourcePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QStringLiteral("无法读取内置属性库：%1").arg(resourcePath);
        return {};
    }

    QStringList rows;
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    while (!stream.atEnd())
    {
        const QString row = stream.readLine().trimmed();
        if (!row.isEmpty() && !row.startsWith(QLatin1Char('#')))
            rows.push_back(row);
    }
    return rows;
}
}

namespace Conductor
{
bool PropertyLibrary::load(QString& error)
{
    m_materials.clear();
    m_sections.clear();
    const QStringList candidates = {
        QDir::current().absoluteFilePath(QStringLiteral("YQY/Import/ImportFile/MaterialProperty.bdf")),
        QDir::current().absoluteFilePath(QStringLiteral("Import/ImportFile/MaterialProperty.bdf")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../../YQY/Import/ImportFile/MaterialProperty.bdf")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../../Import/ImportFile/MaterialProperty.bdf")),
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("../Import/ImportFile/MaterialProperty.bdf"))
    };
    QString filePath;
    for (const QString& candidate : candidates)
    {
        if (QFileInfo::exists(candidate))
        {
            filePath = QFileInfo(candidate).absoluteFilePath();
            break;
        }
    }
    if (filePath.isEmpty())
    {
        error = QStringLiteral("未找到默认属性文件 Import/ImportFile/MaterialProperty.bdf");
        return false;
    }
    if (!loadBdf(filePath, error))
    {
        m_materials.clear();
        m_sections.clear();
        return false;
    }
    return true;
}

bool PropertyLibrary::loadBdf(const QString& filePath, QString& error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QStringLiteral("无法读取默认属性文件：%1").arg(filePath);
        return false;
    }
    enum class Block { None, Material, Section } block = Block::None;
    QTextStream stream(&file);
    stream.setEncoding(QStringConverter::Utf8);
    int lineNumber = 0;
    while (!stream.atEnd())
    {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith(QStringLiteral("**")))
            continue;
        if (line.startsWith(QLatin1Char('*')))
        {
            const QString keyword = line.section(QLatin1Char(','), 0, 0).trimmed().toUpper();
            block = keyword == QStringLiteral("*MATERIAL") ? Block::Material
                : keyword == QStringLiteral("*SECTION") ? Block::Section : Block::None;
            continue;
        }
        const QStringList fields = line.split(QLatin1Char(','));
        if (block == Block::Material)
        {
            if (fields.size() != 8)
            {
                error = QStringLiteral("MaterialProperty.bdf 第 %1 行材料字段数量错误").arg(lineNumber);
                return false;
            }
            bool okId, okE, okNu, okDensity, okStress, okExpansion;
            MaterialPreset value;
            value.id = fields[0].trimmed().toInt(&okId);
            value.category = fields[1].trimmed();
            value.name = fields[2].trimmed();
            value.young = fields[3].trimmed().toDouble(&okE);
            value.poisson = fields[4].trimmed().toDouble(&okNu);
            value.density = fields[5].trimmed().toDouble(&okDensity);
            value.maxStress = fields[6].trimmed().toDouble(&okStress);
            value.expansion = fields[7].trimmed().toDouble(&okExpansion);
            if (!okId || !okE || !okNu || !okDensity || !okStress || !okExpansion ||
                value.id <= 0 || value.name.isEmpty() || value.young <= 0.0 || value.density <= 0.0)
            {
                error = QStringLiteral("MaterialProperty.bdf 第 %1 行材料数据无效").arg(lineNumber);
                return false;
            }
            m_materials.push_back(value);
        }
        else if (block == Block::Section)
        {
            if (fields.size() != 5)
            {
                error = QStringLiteral("MaterialProperty.bdf 第 %1 行截面字段数量错误").arg(lineNumber);
                return false;
            }
            bool okId, okArea;
            SectionPreset value;
            value.id = fields[0].trimmed().toInt(&okId);
            value.category = fields[1].trimmed();
            value.name = fields[2].trimmed();
            value.area = fields[3].trimmed().toDouble(&okArea);
            value.description = fields[4].trimmed();
            if (!okId || !okArea || value.id <= 0 || value.name.isEmpty() || value.area <= 0.0)
            {
                error = QStringLiteral("MaterialProperty.bdf 第 %1 行截面数据无效").arg(lineNumber);
                return false;
            }
            m_sections.push_back(value);
        }
    }
    if (m_materials.isEmpty() || m_sections.isEmpty())
    {
        error = QStringLiteral("MaterialProperty.bdf 必须同时包含 *MATERIAL 和 *SECTION 数据");
        return false;
    }
    return true;
}

bool PropertyLibrary::loadMaterials(const QString& resourcePath, QString& error)
{
    const QStringList rows = dataRows(resourcePath, error);
    if (rows.isEmpty())
        return false;

    for (const QString& row : rows)
    {
        const QStringList fields = row.split(QLatin1Char(','));
        if (fields.size() != 8)
        {
            error = QStringLiteral("材料库字段数量错误：%1").arg(row);
            return false;
        }
        bool okId = false, okE = false, okNu = false, okDensity = false, okStress = false, okAlpha = false;
        MaterialPreset item;
        item.id = fields[0].trimmed().toInt(&okId);
        item.category = fields[1].trimmed();
        item.name = fields[2].trimmed();
        item.young = fields[3].trimmed().toDouble(&okE);
        item.poisson = fields[4].trimmed().toDouble(&okNu);
        item.density = fields[5].trimmed().toDouble(&okDensity);
        item.maxStress = fields[6].trimmed().toDouble(&okStress);
        item.expansion = fields[7].trimmed().toDouble(&okAlpha);
        if (!okId || !okE || !okNu || !okDensity || !okStress || !okAlpha ||
            item.id <= 0 || item.name.isEmpty() || item.young <= 0.0 || item.density <= 0.0)
        {
            error = QStringLiteral("材料库数据无效：%1").arg(row);
            return false;
        }
        m_materials.push_back(item);
    }
    return !m_materials.isEmpty();
}

bool PropertyLibrary::loadSections(const QString& resourcePath, QString& error)
{
    const QStringList rows = dataRows(resourcePath, error);
    if (rows.isEmpty())
        return false;

    for (const QString& row : rows)
    {
        const QStringList fields = row.split(QLatin1Char(','));
        if (fields.size() != 5)
        {
            error = QStringLiteral("截面库字段数量错误：%1").arg(row);
            return false;
        }
        bool okId = false, okArea = false;
        SectionPreset item;
        item.id = fields[0].trimmed().toInt(&okId);
        item.category = fields[1].trimmed();
        item.name = fields[2].trimmed();
        item.area = fields[3].trimmed().toDouble(&okArea);
        item.description = fields[4].trimmed();
        if (!okId || !okArea || item.id <= 0 || item.name.isEmpty() || item.area <= 0.0)
        {
            error = QStringLiteral("截面库数据无效：%1").arg(row);
            return false;
        }
        m_sections.push_back(item);
    }
    return !m_sections.isEmpty();
}

bool PropertyLibrary::updateMaterial(int index, const MaterialPreset& value, QString& error)
{
    if (index < 0 || index >= m_materials.size() || value.young <= 0.0 ||
        value.poisson <= -1.0 || value.poisson >= 0.5 || value.density <= 0.0 ||
        value.maxStress < 0.0 || value.expansion < 0.0)
    {
        error = QStringLiteral("材料属性无效：E、密度必须大于 0，泊松比必须在 (-1, 0.5) 内");
        return false;
    }
    m_materials[index] = value;
    return true;
}

bool PropertyLibrary::updateSection(int index, const SectionPreset& value, QString& error)
{
    if (index < 0 || index >= m_sections.size() || value.area <= 0.0)
    {
        error = QStringLiteral("截面面积必须大于 0");
        return false;
    }
    m_sections[index] = value;
    return true;
}

std::shared_ptr<Property> PropertyLibrary::instantiateProperty(
    int materialIndex, int sectionIndex, StructureData& target, QString& error) const
{
    if (materialIndex < 0 || materialIndex >= m_materials.size() ||
        sectionIndex < 0 || sectionIndex >= m_sections.size())
    {
        error = QStringLiteral("请选择有效的材料和截面");
        return nullptr;
    }

    const MaterialPreset& sourceMaterial = m_materials.at(materialIndex);
    const SectionPreset& sourceSection = m_sections.at(sectionIndex);
    const int materialId = static_cast<int>(target.m_Material.size()) + 1;
    const int sectionId = static_cast<int>(target.m_Section.size()) + 1;

    auto material = std::make_shared<Material>();
    material->m_Id = materialId;
    material->m_Young = sourceMaterial.young;
    material->m_Poisson = sourceMaterial.poisson;
    material->m_Density = sourceMaterial.density;
    material->m_MaxStress = sourceMaterial.maxStress;
    material->m_Expansion = sourceMaterial.expansion;

    auto section = std::make_shared<SectionCircular>();
    section->m_Id = sectionId;
    section->m_Area = sourceSection.area;
    section->Calculate_Radius();

    target.m_Material.emplace(materialId, material);
    target.m_Section.emplace(sectionId, section);
    auto property = target.Create_Property(materialId, sectionId);
    if (!property)
    {
        target.m_Material.erase(materialId);
        target.m_Section.erase(sectionId);
        error = QStringLiteral("创建模型独立属性失败");
    }
    return property;
}
}
