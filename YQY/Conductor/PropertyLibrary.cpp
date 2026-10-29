#include "PropertyLibrary.h"

#include "DataStructure/Factory/SectionFactory.h"
#include "DataStructure/Material/Material.h"
#include "DataStructure/Property/Property.h"
#include "DataStructure/Structure/StructureData.h"

#include <QFile>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <limits>

namespace Conductor
{
const QVector<SpacerPreset>& PropertyLibrary::spacerPresets()
{
    static const QVector<SpacerPreset> presets = []()
    {
        const QString materialName = QStringLiteral("ZL102（参考）");
        const QString materialNote =
            QStringLiteral("厂家目录未公开确切材料牌号；ZL102仅为参考建模材料，不代表该型号的厂家材料认证。");
        const QString beili = QStringLiteral("浙江贝力");
        const QString twinSource = QStringLiteral("https://www.beiliele.com/fjz-double-split-damper-spacer-product/");
        const QString quadSource =
            QStringLiteral("https://www.beiliele.com/fjz4-four-split-conductor-damper-spacer-product/");
        const QString sixSource =
            QStringLiteral("https://www.beiliele.com/fjz6-six-split-conductor-single-phase-spacer-product/");
        const QString mcwadeSource = QStringLiteral("https://mcwadeproductions.co.za/wp-content/uploads/2023/10/"
                                                    "McWade-Powerlines-Catalogue-2021-2023-07-28.pdf");
        const QString sixMaterialNote =
            materialNote + QStringLiteral(" 原目录分裂圆直径750 mm；六分裂相邻子导线中心距换算为375 mm。");
        return QVector<SpacerPreset>{
            {QStringLiteral("beili-fjz-240-19"), QStringLiteral("FJZ-240/19"), beili, twinSource, materialName,
             materialNote, 2, 3.6, 0.4, false},
            {QStringLiteral("beili-fjz-245-19"), QStringLiteral("FJZ-245/19"), beili, twinSource, materialName,
             materialNote, 2, 3.96, 0.45, false},
            {QStringLiteral("beili-fjz-250-30"), QStringLiteral("FJZ-250/30"), beili, twinSource, materialName,
             materialNote, 2, 4.5, 0.5, false},
            {QStringLiteral("mcwade-856890-00"), QStringLiteral("856890-00"), QStringLiteral("McWade"), mcwadeSource,
             materialName, materialNote, 4, 5.0, 0.4, false},
            {QStringLiteral("mcwade-856889-00"), QStringLiteral("856889-00"), QStringLiteral("McWade"), mcwadeSource,
             materialName, materialNote, 4, 5.6, 0.45, false},
            {QStringLiteral("beili-45f-19"), QStringLiteral("FJZ4-45F/19"), beili, quadSource, materialName,
             materialNote, 4, 7.3, 0.45, false},
            {QStringLiteral("henvcon-jzf4-45400"), QStringLiteral("JZF4-45400"), QStringLiteral("恒威通"),
             QStringLiteral("https://cn.henvcon.com/products_details/141.html"), materialName, materialNote, 4, 7.5,
             0.45, false},
            {QStringLiteral("beili-50f-36"), QStringLiteral("FJZ4-50F/36"), beili, quadSource, materialName,
             materialNote, 4, 10.4, 0.5, false},
            {QStringLiteral("beili-fjz6-375-300"), QStringLiteral("FJZ6-375/300"), beili, sixSource, materialName,
             sixMaterialNote, 6, 11.3, 0.375, false},
            {QStringLiteral("beili-fjz6-375d-300"), QStringLiteral("FJZ6-375D/300"), beili, sixSource, materialName,
             sixMaterialNote, 6, 15.8, 0.375, false}};
    }();
    return presets;
}

static bool IsValidMaterialPreset(const MaterialPreset& value)
{
    return value.id > 0 && !value.name.trimmed().isEmpty() && std::isfinite(value.young) && value.young > 0.0 &&
           std::isfinite(value.poisson) && value.poisson > -1.0 && value.poisson < 0.5 &&
           std::isfinite(value.density) && value.density > 0.0 && std::isfinite(value.maxStress) &&
           value.maxStress >= 0.0 && std::isfinite(value.expansion) && value.expansion >= 0.0;
}

static bool IsValidSectionPreset(const SectionPreset& value)
{
    return value.id > 0 && !value.name.trimmed().isEmpty() && std::isfinite(value.area) && value.area > 0.0;
}

bool PropertyLibrary::load(_OUT QString& error)
{
    error.clear();
    m_materials.clear();
    m_sections.clear();
    const QStringList candidates = {
        QDir::current().absoluteFilePath(QStringLiteral("YQY/Import/ImportFile/MaterialProperty.bdf")),
        QDir::current().absoluteFilePath(QStringLiteral("Import/ImportFile/MaterialProperty.bdf")),
        QDir(QCoreApplication::applicationDirPath())
            .absoluteFilePath(QStringLiteral("../../YQY/Import/ImportFile/MaterialProperty.bdf")),
        QDir(QCoreApplication::applicationDirPath())
            .absoluteFilePath(QStringLiteral("../../Import/ImportFile/MaterialProperty.bdf")),
        QDir(QCoreApplication::applicationDirPath())
            .absoluteFilePath(QStringLiteral("../Import/ImportFile/MaterialProperty.bdf"))};
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

bool PropertyLibrary::loadBdf(const QString& filePath, _OUT QString& error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        error = QStringLiteral("无法读取默认属性文件：%1").arg(filePath);
        return false;
    }
    enum class Block
    {
        None,
        Material,
        Section
    } block = Block::None;
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
            block = keyword == QStringLiteral("*MATERIAL")  ? Block::Material
                    : keyword == QStringLiteral("*SECTION") ? Block::Section
                                                            : Block::None;
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
            if (!okId || !okE || !okNu || !okDensity || !okStress || !okExpansion || value.id <= 0 ||
                !IsValidMaterialPreset(value) ||
                std::any_of(m_materials.cbegin(), m_materials.cend(),
                            [&value](const MaterialPreset& existing) { return existing.id == value.id; }))
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
            if (!okId || !okArea || !IsValidSectionPreset(value) ||
                std::any_of(m_sections.cbegin(), m_sections.cend(),
                            [&value](const SectionPreset& existing) { return existing.id == value.id; }))
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

bool PropertyLibrary::updateMaterial(int index, const MaterialPreset& value, _OUT QString& error)
{
    error.clear();
    if (index < 0 || index >= m_materials.size() || !IsValidMaterialPreset(value))
    {
        error = QStringLiteral("材料属性无效：E、密度必须大于 0，泊松比必须在 (-1, 0.5) 内");
        return false;
    }
    m_materials[index] = value;
    return true;
}

bool PropertyLibrary::updateSection(int index, const SectionPreset& value, _OUT QString& error)
{
    error.clear();
    if (index < 0 || index >= m_sections.size() || !IsValidSectionPreset(value))
    {
        error = QStringLiteral("截面面积必须大于 0");
        return false;
    }
    m_sections[index] = value;
    return true;
}

std::shared_ptr<Property> PropertyLibrary::instantiateProperty(int materialIndex, int sectionIndex,
                                                               _OUT StructureData& target, _OUT QString& error) const
{
    error.clear();
    if (materialIndex < 0 || materialIndex >= m_materials.size() || sectionIndex < 0 ||
        sectionIndex >= m_sections.size())
    {
        error = QStringLiteral("请选择有效的材料和截面");
        return nullptr;
    }

    const MaterialPreset& sourceMaterial = m_materials.at(materialIndex);
    const SectionPreset& sourceSection = m_sections.at(sectionIndex);
    const auto nextId = [](const auto& values)
    {
        if (values.empty())
            return 1;
        const int maximum = values.rbegin()->first;
        return maximum == std::numeric_limits<int>::max() ? 0 : std::max(0, maximum) + 1;
    };
    const int materialId = nextId(target.m_Material);
    const int sectionId = nextId(target.m_Section);
    const int propertyId = nextId(target.m_Property);
    if (materialId <= 0 || sectionId <= 0 || propertyId <= 0 || !IsValidMaterialPreset(sourceMaterial) ||
        !IsValidSectionPreset(sourceSection))
    {
        error = QStringLiteral("创建模型独立属性失败");
        return nullptr;
    }

    auto material = std::make_shared<Material>();
    material->m_Id = materialId;
    material->m_Young = sourceMaterial.young;
    material->m_Poisson = sourceMaterial.poisson;
    material->m_Density = sourceMaterial.density;
    material->m_MaxStress = sourceMaterial.maxStress;
    material->m_Expansion = sourceMaterial.expansion;

    auto section = SectionFactory::CreateCircular(sectionId, sourceSection.area);

    auto property = std::make_shared<Property>();
    property->m_Id = propertyId;
    property->m_pMaterial = material;
    property->m_pSection = section;
    target.m_Material.emplace(materialId, std::move(material));
    target.m_Section.emplace(sectionId, std::move(section));
    target.m_Property.emplace(propertyId, property);
    return property;
}
}
