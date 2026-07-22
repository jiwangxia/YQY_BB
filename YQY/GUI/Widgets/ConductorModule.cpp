#include "Widgets/ConductorModule.h"
#include "ui_ConductorModule.h"
#include "Conductor/ConductorModelBuilder.h"
#include "Conductor/PropertyLibrary.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Outputter.h"
ConductorModule::ConductorModule(QWidget* p) : QWidget(p), m_ui(new Ui::ConductorModuleClass)
{
    m_ui->setupUi(this);
    setObjectName(QStringLiteral("conductorPage"));
    m_ui->scrollArea->setObjectName(QStringLiteral("conductorScrollArea"));
    m_ui->content->setObjectName(QStringLiteral("conductorContent"));
    m_ui->propertyGroup->setObjectName(QStringLiteral("conductorPropertyGroup"));
    m_ui->formGroup->setObjectName(QStringLiteral("conductorFormGroup"));
    m_ui->createButton->setObjectName(QStringLiteral("conductorCreateButton"));
    const QList<QWidget*> fields = {m_ui->nameEdit,    m_ui->materialCombo, m_ui->sectionCombo, m_ui->elementCombo,
                                    m_ui->bundleCombo, m_ui->spacingSpin,   m_ui->segmentsSpin, m_ui->stressSpin,
                                    m_ui->startX,      m_ui->startY,        m_ui->startZ,       m_ui->endX,
                                    m_ui->endY,        m_ui->endZ};
    for (auto* f : fields)
        f->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    for (auto* c : {m_ui->materialCombo, m_ui->sectionCombo, m_ui->elementCombo, m_ui->bundleCombo})
    {
        c->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        c->setMinimumContentsLength(8);
    }
}
ConductorModule::~ConductorModule()
{
    delete m_ui;
}

void ConductorModule::setPropertyLibrary(Conductor::PropertyLibrary* library)
{
    m_ui->materialCombo->clear();
    m_ui->sectionCombo->clear();
    m_ui->createButton->setEnabled(library && library->isReady());
    if (!library)
        return;

    for (const auto& material : library->materials())
    {
        const QString text = QStringLiteral("%1 · %2").arg(material.category, material.name);
        m_ui->materialCombo->addItem(text);
        m_ui->materialCombo->setItemData(m_ui->materialCombo->count() - 1, text, Qt::ToolTipRole);
    }
    for (const auto& section : library->sections())
    {
        const QString text = QStringLiteral("%1 · %2").arg(section.category, section.name);
        m_ui->sectionCombo->addItem(text);
        m_ui->sectionCombo->setItemData(m_ui->sectionCombo->count() - 1, text, Qt::ToolTipRole);
    }
    m_ui->elementCombo->setItemData(0, static_cast<int>(EnumKeyword::ElementType::T3D2));
    m_ui->elementCombo->setItemData(1, static_cast<int>(EnumKeyword::ElementType::CABLE));
    const QList<int> bundleCounts = {1, 2, 4, 6, 8};
    for (int index = 0; index < bundleCounts.size(); ++index)
        m_ui->bundleCombo->setItemData(index, bundleCounts.at(index));

    connect(m_ui->materialCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this, library](int index)
            {
                if (index < 0 || index >= library->materials().size())
                    return;
                const QString materialName = library->materials().at(index).name;
                for (int sectionIndex = 0; sectionIndex < library->sections().size(); ++sectionIndex)
                {
                    if (library->sections().at(sectionIndex).name.startsWith(materialName))
                    {
                        m_ui->sectionCombo->setCurrentIndex(sectionIndex);
                        break;
                    }
                }
            });
}

ConductorModule::BuildResult ConductorModule::buildModel(Conductor::PropertyLibrary& library,
                                                         const QString& generatedDirectory) const
{
    BuildResult build;
    if (!library.isReady())
    {
        build.error = QStringLiteral("启动属性库不可用。");
        return build;
    }

    const Vector3d start(m_ui->startX->value(), m_ui->startY->value(), m_ui->startZ->value());
    const Vector3d end(m_ui->endX->value(), m_ui->endY->value(), m_ui->endZ->value());
    if ((end - start).head<2>().norm() <= 1.0e-7)
    {
        build.error = QStringLiteral("左右挂点必须具有有效的水平档距。");
        return build;
    }

    build.structure = std::make_shared<StructureData>();
    auto property = library.instantiateProperty(m_ui->materialCombo->currentIndex(), m_ui->sectionCombo->currentIndex(),
                                                *build.structure, build.error);
    if (!property)
    {
        build.structure.reset();
        return build;
    }

    Conductor::LineBuildConfig config;
    config.start = start;
    config.end = end;
    config.property = property;
    config.elementType = static_cast<EnumKeyword::ElementType>(m_ui->elementCombo->currentData().toInt());
    config.conductor.nBundle = m_ui->bundleCombo->currentData().toInt();
    config.conductor.spacing = m_ui->spacingSpin->value();
    config.conductor.segments = m_ui->segmentsSpin->value();
    config.conductor.stress0 = m_ui->stressSpin->value() * 1.0e6;
    config.conductor.connecttype = Conductor::ConnectionMode::Parallel;

    Conductor::ConductorModelBuilder builder(build.structure);
    Conductor::LineBuildResult lineResult;
    std::string buildError;
    if (!builder.BuildLine(config, lineResult, buildError))
    {
        build.error = QString::fromStdString(buildError);
        build.structure.reset();
        return build;
    }

    if (m_ui->analysisCheck->isChecked())
    {
        std::vector<int> endpointIds;
        for (const auto& pair : lineResult.subConductors)
        {
            if (!pair.second.nodeIds.empty())
            {
                endpointIds.push_back(pair.second.nodeIds.front());
                endpointIds.push_back(pair.second.nodeIds.back());
            }
        }
        build.structure->Add_Constraint(endpointIds, {0, 1, 2}, {0.0, 0.0, 0.0});
        AnalysisStepConfig step;
        step.id = 1;
        step.type = EnumKeyword::StepType::STATIC;
        step.totalTime = 1.0;
        step.stepSize = 0.01;
        step.tolerance = 1.0e-5;
        step.maxIterations = 50;
        build.structure->AddAnalysisStep(step);
        build.structure->Add_Gravity(2, step.id);
    }

    QString fileName = m_ui->nameEdit->text().trimmed();
    if (fileName.isEmpty())
        fileName = QStringLiteral("导线模型");
    fileName.replace(QRegularExpression(QStringLiteral("[\\\\/:*?\"<>|]")), QStringLiteral("_"));
    if (!QDir().mkpath(generatedDirectory))
    {
        build.error = QStringLiteral("无法创建输出目录：%1").arg(generatedDirectory);
        build.structure.reset();
        return build;
    }
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
    build.filePath = QDir(generatedDirectory).absoluteFilePath(QStringLiteral("%1_%2.bdf").arg(fileName, timestamp));
    Outputter outputter;
    if (!outputter.SaveBdfModel(build.filePath, build.structure.get()))
    {
        build.error = QStringLiteral("模型保存失败：%1").arg(build.filePath);
        build.filePath.clear();
        build.structure.reset();
        return build;
    }

    build.nodeCount = lineResult.NodeCount();
    build.elementCount = lineResult.ElementCount();
    return build;
}
QPushButton* ConductorModule::viewLibraryButton() const
{
    return m_ui->viewLibraryButton;
}
QLineEdit* ConductorModule::nameEdit() const
{
    return m_ui->nameEdit;
}
QComboBox* ConductorModule::materialCombo() const
{
    return m_ui->materialCombo;
}
QComboBox* ConductorModule::sectionCombo() const
{
    return m_ui->sectionCombo;
}
QComboBox* ConductorModule::elementCombo() const
{
    return m_ui->elementCombo;
}
QDoubleSpinBox* ConductorModule::startX() const
{
    return m_ui->startX;
}
QDoubleSpinBox* ConductorModule::startY() const
{
    return m_ui->startY;
}
QDoubleSpinBox* ConductorModule::startZ() const
{
    return m_ui->startZ;
}
QDoubleSpinBox* ConductorModule::endX() const
{
    return m_ui->endX;
}
QDoubleSpinBox* ConductorModule::endY() const
{
    return m_ui->endY;
}
QDoubleSpinBox* ConductorModule::endZ() const
{
    return m_ui->endZ;
}
QComboBox* ConductorModule::bundleCombo() const
{
    return m_ui->bundleCombo;
}
QDoubleSpinBox* ConductorModule::spacingSpin() const
{
    return m_ui->spacingSpin;
}
QSpinBox* ConductorModule::segmentsSpin() const
{
    return m_ui->segmentsSpin;
}
QDoubleSpinBox* ConductorModule::stressSpin() const
{
    return m_ui->stressSpin;
}
QCheckBox* ConductorModule::analysisCheck() const
{
    return m_ui->analysisCheck;
}
QPushButton* ConductorModule::createButton() const
{
    return m_ui->createButton;
}
