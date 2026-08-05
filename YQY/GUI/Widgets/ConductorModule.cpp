#include "Widgets/ConductorModule.h"
#include "ui_ConductorModule.h"
#include "Conductor/ConductorModelBuilder.h"
#include "Conductor/PropertyLibrary.h"
#include "DataStructure/Structure/StructureData.h"
#include "Export/Outputter.h"

#include <QHeaderView>
#include <QTableWidgetItem>

#include <algorithm>
#include <map>
#include <set>
#include <vector>

ConductorModule::ConductorModule(QWidget* p) : QWidget(p), m_ui(new Ui::ConductorModuleClass)
{
    m_ui->setupUi(this);
    setObjectName(QStringLiteral("conductorPage"));
    m_ui->scrollArea->setObjectName(QStringLiteral("conductorScrollArea"));
    m_ui->content->setObjectName(QStringLiteral("conductorContent"));
    m_ui->propertyGroup->setObjectName(QStringLiteral("conductorPropertyGroup"));
    m_ui->spanDefinitionGroup->setObjectName(QStringLiteral("conductorSpanDefinitionGroup"));
    m_ui->formGroup->setObjectName(QStringLiteral("conductorFormGroup"));
    m_ui->spacerGroup->setObjectName(QStringLiteral("conductorSpacerGroup"));
    m_ui->createButton->setObjectName(QStringLiteral("conductorCreateButton"));
    const QList<QWidget*> fields = {m_ui->nameEdit, m_ui->materialCombo, m_ui->sectionCombo, m_ui->elementCombo,
                                     m_ui->bundleCombo, m_ui->spacingSpin, m_ui->segmentsSpin, m_ui->stressSpin,
                                     m_ui->endTopologyCombo, m_ui->dualSupportSpacingSpin,
                                     m_ui->modelModeCombo, m_ui->suspensionLengthSpin,
                                     m_ui->startX, m_ui->startY, m_ui->startZ, m_ui->endX, m_ui->endY, m_ui->endZ,
                                    m_ui->spacerLayoutCombo, m_ui->spacerStyleCombo, m_ui->spacerCountSpin, m_ui->spacerElementCombo,
                                    m_ui->spacerMaterialCombo, m_ui->spacerSectionCombo};
    for (auto* f : fields)
        f->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    for (auto* c : {m_ui->materialCombo, m_ui->sectionCombo, m_ui->elementCombo, m_ui->bundleCombo,
                    m_ui->endTopologyCombo,
                    m_ui->spacerLayoutCombo, m_ui->spacerStyleCombo, m_ui->spacerElementCombo,
                    m_ui->spacerMaterialCombo, m_ui->spacerSectionCombo})
    {
        c->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        c->setMinimumContentsLength(8);
    }

    m_ui->spacerLayoutCombo->setItemData(0, false);
    m_ui->spacerLayoutCombo->setItemData(1, true);
    m_ui->spacerStyleCombo->setItemData(0, static_cast<int>(Conductor::InnerSpacerStyle::OuterPolygon));
    m_ui->spacerStyleCombo->setItemData(1, static_cast<int>(Conductor::InnerSpacerStyle::CenterBraced));
    m_ui->spacerStyleCombo->setItemData(2, static_cast<int>(Conductor::InnerSpacerStyle::InnerPolygon));
    m_ui->spacerElementCombo->setItemData(0, static_cast<int>(EnumKeyword::ElementType::CR3D));
    m_ui->spacerElementCombo->setItemData(1, static_cast<int>(EnumKeyword::ElementType::T3D2));
    m_ui->endTopologyCombo->setItemData(0, static_cast<int>(Conductor::BundleEndTopology::SingleSupport));
    m_ui->endTopologyCombo->setItemData(1, static_cast<int>(Conductor::BundleEndTopology::DualSupportByGroup));
    m_ui->endTopologyCombo->setItemData(2, static_cast<int>(Conductor::BundleEndTopology::DirectWireSupports));
    initializeStationTable();

    connect(m_ui->modelModeCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]() { updateModelModeUi(); updateSpacerPreview(); });
    connect(m_ui->addStationButton, &QPushButton::clicked, this,
            [this]()
            {
                const int lastRow = m_ui->stationTable->rowCount() - 1;
                const int insertRow = std::max(1, lastRow);
                auto valueAt = [this](int row, int column)
                    {
                        bool ok = false;
                        const double value = m_ui->stationTable->item(row, column)->text().toDouble(&ok);
                        return ok ? value : 0.0;
                    };
                const int previousRow = std::max(0, insertRow - 1);
                const int nextRow = std::max(previousRow, lastRow);
                const Vector3d center(
                    (valueAt(previousRow, 2) + valueAt(nextRow, 2)) * 0.5,
                    (valueAt(previousRow, 3) + valueAt(nextRow, 3)) * 0.5,
                    (valueAt(previousRow, 4) + valueAt(nextRow, 4)) * 0.5);
                m_ui->stationTable->insertRow(insertRow);
                for (int column = 0; column < 5; ++column)
                    m_ui->stationTable->setItem(insertRow, column, new QTableWidgetItem());
                m_ui->stationTable->item(insertRow, 2)->setText(QString::number(center.x(), 'f', 3));
                m_ui->stationTable->item(insertRow, 3)->setText(QString::number(center.y(), 'f', 3));
                m_ui->stationTable->item(insertRow, 4)->setText(QString::number(center.z(), 'f', 3));
                refreshStationTypes();
                m_ui->stationTable->selectRow(insertRow);
                updateSpacerPreview();
            });
    connect(m_ui->removeStationButton, &QPushButton::clicked, this,
            [this]()
            {
                const int row = m_ui->stationTable->currentRow();
                if (m_ui->stationTable->rowCount() > 3 &&
                    row > 0 && row + 1 < m_ui->stationTable->rowCount())
                {
                    m_ui->stationTable->removeRow(row);
                    refreshStationTypes();
                    updateSpacerPreview();
                }
            });
    connect(m_ui->stationTable, &QTableWidget::itemChanged, this,
            [this]() { updateSpacerPreview(); });
    connect(m_ui->innerSpacerCheck, &QCheckBox::toggled, this,
            [this]() { updateSpacerUi(); });
    connect(m_ui->spacerLayoutCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]() { updateSpacerUi(); });
    connect(m_ui->spacerCountSpin, qOverload<int>(&QSpinBox::valueChanged), this,
            [this]() { updateSpacerPreview(); });
    connect(m_ui->endTopologyCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]() { updateEndTopologyUi(); });
    connect(m_ui->spacingSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double value)
            {
                if (!m_dualSupportSpacingCustomized)
                    m_ui->dualSupportSpacingSpin->setValue(value);
            });
    connect(m_ui->dualSupportSpacingSpin, &QDoubleSpinBox::editingFinished, this,
            [this]() { m_dualSupportSpacingCustomized = true; });
    connect(m_ui->bundleCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this]() { updateSpacerUi(); updateEndTopologyUi(); });
    for (auto* coordinate : {m_ui->startX, m_ui->startY, m_ui->endX, m_ui->endY})
    {
        connect(coordinate, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                [this]() { updateSpacerPreview(); });
    }
    updateSpacerUi();
    updateEndTopologyUi();
    updateModelModeUi();
}
ConductorModule::~ConductorModule()
{
    delete m_ui;
}

void ConductorModule::initializeStationTable()
{
    m_ui->stationTable->setRowCount(3);
    m_ui->stationTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_ui->stationTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    for (int column = 2; column < 5; ++column)
        m_ui->stationTable->horizontalHeader()->setSectionResizeMode(column, QHeaderView::Stretch);
    const Vector3d defaults[] = {
        Vector3d(0.0, 0.0, 0.0),
        Vector3d(150.0, 0.0, 0.0),
        Vector3d(300.0, 0.0, 0.0)
    };
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 5; ++column)
            m_ui->stationTable->setItem(row, column, new QTableWidgetItem());
        m_ui->stationTable->item(row, 2)->setText(QString::number(defaults[row].x(), 'f', 3));
        m_ui->stationTable->item(row, 3)->setText(QString::number(defaults[row].y(), 'f', 3));
        m_ui->stationTable->item(row, 4)->setText(QString::number(defaults[row].z(), 'f', 3));
    }
    refreshStationTypes();
}

void ConductorModule::updateModelModeUi()
{
    const bool multiSpan = m_ui->modelModeCombo->currentIndex() == 1;
    m_ui->multiSpanEditor->setVisible(multiSpan);
    m_ui->startLabel->setVisible(!multiSpan);
    m_ui->startCoordinates->setVisible(!multiSpan);
    m_ui->endLabel->setVisible(!multiSpan);
    m_ui->endCoordinates->setVisible(!multiSpan);
    m_ui->segmentsLabel->setText(
        multiSpan ? QStringLiteral("每档每根离散段数") : QStringLiteral("每根离散段数"));
    m_ui->analysisCheck->setToolTip(
        multiSpan
            ? QStringLiteral("约束首末耐张挂点和全部中间悬垂串上端，并创建静力分析步与重力")
            : QStringLiteral("创建静力分析步、重力和两端约束"));
}

void ConductorModule::refreshStationTypes()
{
    const int rowCount = m_ui->stationTable->rowCount();
    for (int row = 0; row < rowCount; ++row)
    {
        auto* sequenceItem = m_ui->stationTable->item(row, 0);
        auto* typeItem = m_ui->stationTable->item(row, 1);
        if (!sequenceItem)
        {
            sequenceItem = new QTableWidgetItem();
            m_ui->stationTable->setItem(row, 0, sequenceItem);
        }
        if (!typeItem)
        {
            typeItem = new QTableWidgetItem();
            m_ui->stationTable->setItem(row, 1, typeItem);
        }
        sequenceItem->setText(QString::number(row + 1));
        typeItem->setText(row == 0
            ? QStringLiteral("左耐张")
            : (row + 1 == rowCount ? QStringLiteral("右耐张") : QStringLiteral("H 型悬垂")));
        sequenceItem->setFlags(sequenceItem->flags() & ~Qt::ItemIsEditable);
        typeItem->setFlags(typeItem->flags() & ~Qt::ItemIsEditable);
    }
}

std::vector<Vector3d> ConductorModule::stationCenters(QString& error) const
{
    std::vector<Vector3d> centers;
    if (m_ui->stationTable->rowCount() < 3)
    {
        error = QStringLiteral("多档导线至少需要三个坐标点。");
        return centers;
    }
    centers.reserve(static_cast<std::size_t>(m_ui->stationTable->rowCount()));
    for (int row = 0; row < m_ui->stationTable->rowCount(); ++row)
    {
        double values[3] = {};
        for (int coordinate = 0; coordinate < 3; ++coordinate)
        {
            const auto* item = m_ui->stationTable->item(row, coordinate + 2);
            bool ok = false;
            values[coordinate] = item ? item->text().toDouble(&ok) : 0.0;
            if (!ok)
            {
                error = QStringLiteral("第 %1 个坐标点的 %2 值无效。")
                    .arg(row + 1)
                    .arg(QStringLiteral("XYZ").at(coordinate));
                centers.clear();
                return centers;
            }
        }
        centers.emplace_back(values[0], values[1], values[2]);
        if (row > 0 && (centers[row] - centers[row - 1]).head<2>().norm() <= 1.0e-7)
        {
            error = QStringLiteral("第 %1 档的水平档距过小。").arg(row);
            centers.clear();
            return centers;
        }
    }
    return centers;
}

void ConductorModule::updateSpacerUi()
{
    const bool multipleBundle = m_ui->bundleCombo->currentText().toInt() > 1;
    const bool enabled = m_ui->innerSpacerCheck->isChecked() && multipleBundle;
    const bool equalSpacing = m_ui->spacerLayoutCombo->currentData().toBool();
    for (auto* widget : {static_cast<QWidget*>(m_ui->spacerLayoutCombo),
                         static_cast<QWidget*>(m_ui->spacerStyleCombo),
                         static_cast<QWidget*>(m_ui->spacerElementCombo),
                         static_cast<QWidget*>(m_ui->spacerMaterialCombo),
                         static_cast<QWidget*>(m_ui->spacerSectionCombo)})
    {
        widget->setEnabled(enabled);
    }
    m_ui->spacerStyleCombo->setEnabled(enabled && m_ui->bundleCombo->currentText().toInt() > 2);
    m_ui->spacerCountLabel->setVisible(equalSpacing);
    m_ui->spacerCountSpin->setVisible(equalSpacing);
    m_ui->spacerCountSpin->setEnabled(enabled);
    updateSpacerPreview();
}

void ConductorModule::updateEndTopologyUi()
{
    // 属性库尚未注入时 bundleCombo 的 itemData 为空；界面默认值仍应
    // 根据显示的分裂数正确启用双挂点选项。
    const int bundleCount = m_ui->bundleCombo->currentText().toInt();
    const bool supportsAnyEndTopology = bundleCount > 1;
    const bool supportsDual = bundleCount > 2;
    m_ui->endTopologyCombo->setEnabled(supportsAnyEndTopology);
    if (!supportsAnyEndTopology)
        m_ui->endTopologyCombo->setCurrentIndex(0);
    else if (!supportsDual &&
        m_ui->endTopologyCombo->currentData().toInt() ==
            static_cast<int>(Conductor::BundleEndTopology::DualSupportByGroup))
        m_ui->endTopologyCombo->setCurrentIndex(0);
    const bool dual = supportsDual &&
        m_ui->endTopologyCombo->currentData().toInt() ==
            static_cast<int>(Conductor::BundleEndTopology::DualSupportByGroup);
    // 双挂点间距只对“分组双挂点”有意义；单挂点时完全隐藏，
    // 避免让用户误以为当前模型会采用该参数。
    m_ui->dualSupportSpacingLabel->setVisible(dual);
    m_ui->dualSupportSpacingSpin->setVisible(dual);
    m_ui->dualSupportSpacingLabel->setEnabled(dual);
    m_ui->dualSupportSpacingSpin->setEnabled(dual);
}

void ConductorModule::updateSpacerPreview()
{
    if (!m_ui->innerSpacerCheck->isChecked())
    {
        m_ui->spacerCountPreview->setText(QStringLiteral("不创建"));
        m_ui->spacerCountPreview->setToolTip(QString());
        return;
    }

    if (m_ui->bundleCombo->currentText().toInt() <= 1)
    {
        m_ui->spacerCountPreview->setText(QStringLiteral("单分裂导线不创建"));
        m_ui->spacerCountPreview->setToolTip(QString());
        return;
    }

    if (m_ui->spacerLayoutCombo->currentData().toBool())
    {
        const int spanCount = m_ui->modelModeCombo->currentIndex() == 1
            ? std::max(0, m_ui->stationTable->rowCount() - 1) : 1;
        m_ui->spacerCountPreview->setText(
            QStringLiteral("%1 个（%2 档等距布置）")
                .arg(m_ui->spacerCountSpin->value() * spanCount)
                .arg(spanCount));
        m_ui->spacerCountPreview->setToolTip(QStringLiteral("每一档均按指定数量等距布置"));
        return;
    }

    std::vector<double> spanLengths;
    if (m_ui->modelModeCombo->currentIndex() == 1)
    {
        QString coordinateError;
        const auto centers = stationCenters(coordinateError);
        if (centers.empty())
        {
            m_ui->spacerCountPreview->setText(QStringLiteral("坐标无效"));
            m_ui->spacerCountPreview->setToolTip(coordinateError);
            return;
        }
        for (std::size_t index = 0; index + 1 < centers.size(); ++index)
            spanLengths.push_back((centers[index + 1] - centers[index]).head<2>().norm());
    }
    else
    {
        const Vector2d span = Vector2d(
            m_ui->endX->value() - m_ui->startX->value(),
            m_ui->endY->value() - m_ui->startY->value());
        spanLengths.push_back(span.norm());
    }
    std::vector<double> positions;
    for (double spanLength : spanLengths)
    {
        const auto current =
            Conductor::ConductorModelBuilder::CalculateStandardInnerSpacerPositions(spanLength);
        positions.insert(positions.end(), current.begin(), current.end());
    }
    QStringList positionTexts;
    positionTexts.reserve(static_cast<qsizetype>(positions.size()));
    for (double position : positions)
        positionTexts.push_back(QString::number(position, 'f', 2));
    m_ui->spacerCountPreview->setText(
        QStringLiteral("%1 个（%2 档内置规则）")
            .arg(static_cast<qulonglong>(positions.size()))
            .arg(static_cast<qulonglong>(spanLengths.size())));
    m_ui->spacerCountPreview->setToolTip(
        positions.empty()
            ? QStringLiteral("当前水平档距无效")
            : QStringLiteral("距左挂点位置：%1 m").arg(positionTexts.join(QStringLiteral("、"))));
}

void ConductorModule::setPropertyLibrary(Conductor::PropertyLibrary* library)
{
    m_ui->materialCombo->clear();
    m_ui->sectionCombo->clear();
    m_ui->spacerMaterialCombo->clear();
    m_ui->spacerSectionCombo->clear();
    m_ui->createButton->setEnabled(library && library->isReady());
    if (!library)
        return;

    for (const auto& material : library->materials())
    {
        const QString text = QStringLiteral("%1 · %2").arg(material.category, material.name);
        m_ui->materialCombo->addItem(text);
        m_ui->materialCombo->setItemData(m_ui->materialCombo->count() - 1, text, Qt::ToolTipRole);
        m_ui->spacerMaterialCombo->addItem(text);
        m_ui->spacerMaterialCombo->setItemData(m_ui->spacerMaterialCombo->count() - 1, text, Qt::ToolTipRole);
    }
    for (const auto& section : library->sections())
    {
        const QString text = QStringLiteral("%1 · %2").arg(section.category, section.name);
        m_ui->sectionCombo->addItem(text);
        m_ui->sectionCombo->setItemData(m_ui->sectionCombo->count() - 1, text, Qt::ToolTipRole);
        m_ui->spacerSectionCombo->addItem(text);
        m_ui->spacerSectionCombo->setItemData(m_ui->spacerSectionCombo->count() - 1, text, Qt::ToolTipRole);
    }
    m_ui->elementCombo->setItemData(0, static_cast<int>(EnumKeyword::ElementType::T3D2));
    m_ui->elementCombo->setItemData(1, static_cast<int>(EnumKeyword::ElementType::CABLE));
    if (m_ui->elementCombo->count() < 3)
        m_ui->elementCombo->addItem(QStringLiteral("CR3D 共旋梁"));
    m_ui->elementCombo->setItemData(2, static_cast<int>(EnumKeyword::ElementType::CR3D));
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
    connect(m_ui->spacerMaterialCombo, qOverload<int>(&QComboBox::currentIndexChanged), this,
            [this, library](int index)
            {
                if (index < 0 || index >= library->materials().size())
                    return;
                const QString materialName = library->materials().at(index).name;
                for (int sectionIndex = 0; sectionIndex < library->sections().size(); ++sectionIndex)
                {
                    if (library->sections().at(sectionIndex).name.startsWith(materialName))
                    {
                        m_ui->spacerSectionCombo->setCurrentIndex(sectionIndex);
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

    const bool multiSpan = m_ui->modelModeCombo->currentIndex() == 1;
    std::vector<Vector3d> stations;
    Vector3d start(m_ui->startX->value(), m_ui->startY->value(), m_ui->startZ->value());
    Vector3d end(m_ui->endX->value(), m_ui->endY->value(), m_ui->endZ->value());
    if (multiSpan)
    {
        stations = stationCenters(build.error);
        if (stations.empty())
            return build;
        start = stations.front();
        end = stations.back();
    }
    else if ((end - start).head<2>().norm() <= 1.0e-7)
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
    config.endTopology = static_cast<Conductor::BundleEndTopology>(m_ui->endTopologyCombo->currentData().toInt());
    config.dualSupportSpacing = m_ui->dualSupportSpacingSpin->value();
    config.setNamePrefix = m_ui->nameEdit->text().trimmed();
    // A CR3D end fitting attached to a translation-only conductor forms an
    // independent rotational component with a zero-energy roll mode.
    config.endFittingElementType =
        config.elementType == EnumKeyword::ElementType::CR3D
        ? EnumKeyword::ElementType::CR3D
        : EnumKeyword::ElementType::T3D2;

    Conductor::ConductorModelBuilder builder(build.structure);
    Conductor::LineBuildResult lineResult;
    std::string buildError;
    Conductor::SpanConductorBuildConfig spanConfig;
    spanConfig.line = config;
    if (m_ui->innerSpacerCheck->isChecked() && config.conductor.nBundle > 1)
    {
        auto spacerProperty = library.instantiateProperty(
            m_ui->spacerMaterialCombo->currentIndex(),
            m_ui->spacerSectionCombo->currentIndex(),
            *build.structure,
            build.error);
        if (!spacerProperty)
        {
            build.structure.reset();
            return build;
        }
        spanConfig.useInnerSpacerLayout = true;
        spanConfig.innerSpacerLayout.useEqualSpacing = m_ui->spacerLayoutCombo->currentData().toBool();
        spanConfig.innerSpacerLayout.count = m_ui->spacerCountSpin->value();
        spanConfig.innerSpacerLayout.spacer.elementType =
            static_cast<EnumKeyword::ElementType>(m_ui->spacerElementCombo->currentData().toInt());
        spanConfig.innerSpacerLayout.spacer.property = spacerProperty;
        spanConfig.innerSpacerLayout.spacer.style =
            static_cast<Conductor::InnerSpacerStyle>(m_ui->spacerStyleCombo->currentData().toInt());
    }
    bool built = false;
    if (multiSpan)
    {
        Conductor::MultiSpanConductorBuildConfig multiConfig;
        multiConfig.span = spanConfig;
        multiConfig.stationCenters = stations;
        multiConfig.suspensionStringLength = m_ui->suspensionLengthSpin->value();
        multiConfig.suspensionElementType = EnumKeyword::ElementType::T3D2;
        multiConfig.suspensionProperty = property;
        built = builder.BuildMultiSpanConductor(multiConfig, lineResult, buildError);
    }
    else
    {
        built = builder.BuildSpanConductor(spanConfig, lineResult, buildError);
    }
    if (!built)
    {
        build.error = QString::fromStdString(buildError);
        build.structure.reset();
        return build;
    }

    if (m_ui->analysisCheck->isChecked())
    {
        std::set<int> endpointIds;
        if (!lineResult.leftTensionEnd.supportNodeIds.empty() ||
            !lineResult.rightTensionEnd.supportNodeIds.empty())
        {
            endpointIds.insert(lineResult.leftTensionEnd.supportNodeIds.begin(), lineResult.leftTensionEnd.supportNodeIds.end());
            endpointIds.insert(lineResult.rightTensionEnd.supportNodeIds.begin(), lineResult.rightTensionEnd.supportNodeIds.end());
        }
        else
        {
            for (const auto& pair : lineResult.subConductors)
            {
                if (!pair.second.nodeIds.empty())
                {
                    endpointIds.insert(pair.second.nodeIds.front());
                    endpointIds.insert(pair.second.nodeIds.back());
                }
            }
        }
        // Determine support constraints from every element actually attached
        // to the node; the node has not yet gone through AnalysisStep::Init_DOF.
        auto requiredNodeDofs = [&](int nodeId)
        {
            int required = 3;
            for (const auto& [elementId, element] : build.structure->m_Elements)
            {
                Q_UNUSED(elementId);
                if (!element)
                    continue;
                for (const auto& nodeReference : element->m_pNode)
                {
                    const auto node = nodeReference.lock();
                    if (node && node->m_Id == nodeId)
                    {
                        required = std::max(required, element->Get_NodeDOF());
                        break;
                    }
                }
            }
            return required;
        };
        auto addZeroConstraint = [&](int nodeId, int constrainedDofCount)
        {
            std::vector<int> directions;
            std::vector<double> values;
            directions.reserve(constrainedDofCount);
            values.assign(constrainedDofCount, 0.0);
            for (int direction = 0; direction < constrainedDofCount; ++direction)
                directions.push_back(direction);
            build.structure->Add_Constraint({ nodeId }, directions, values);
        };
        // A CR3D beam has three sectional rotation DOFs at each node. When a
        // beam-type spacer is connected only to truss/cable conductors, its
        // translations are carried by the conductors but its whole sectional
        // rotation field has no reference. This gauge mode makes the static
        // tangent matrix singular. Build connected beam groups and pin the
        // rotation at one deterministic node only when the group does not
        // already reach a rotationally constrained end support.
        std::map<int, int> beamParent;
        std::map<int, int> beamNodeDegree;
        const auto findBeamRoot = [&beamParent](int nodeId, const auto& self) -> int
        {
            const auto found = beamParent.find(nodeId);
            if (found == beamParent.cend() || found->second == nodeId)
                return nodeId;
            found->second = self(found->second, self);
            return found->second;
        };
        const auto joinBeamNodes = [&beamParent, &findBeamRoot](int firstId, int secondId)
        {
            beamParent.try_emplace(firstId, firstId);
            beamParent.try_emplace(secondId, secondId);
            const int firstRoot = findBeamRoot(firstId, findBeamRoot);
            const int secondRoot = findBeamRoot(secondId, findBeamRoot);
            if (firstRoot != secondRoot)
                beamParent[firstRoot] = secondRoot;
        };
        for (const auto& [elementId, element] : build.structure->m_Elements)
        {
            Q_UNUSED(elementId);
            if (!element || element->Get_NodeDOF() < 6 || element->m_pNode.size() != 2)
                continue;
            const auto first = element->m_pNode[0].lock();
            const auto second = element->m_pNode[1].lock();
            if (!first || !second)
                continue;
            joinBeamNodes(first->m_Id, second->m_Id);
            ++beamNodeDegree[first->m_Id];
            ++beamNodeDegree[second->m_Id];
        }
        std::map<int, std::vector<int>> beamComponents;
        for (const auto& [nodeId, parent] : beamParent)
        {
            Q_UNUSED(parent);
            beamComponents[findBeamRoot(nodeId, findBeamRoot)].push_back(nodeId);
        }
        std::set<int> rotationallySupportedEndpoints;
        for (int endpointId : endpointIds)
        {
            if (requiredNodeDofs(endpointId) >= 6)
                rotationallySupportedEndpoints.insert(endpointId);
        }
        for (auto& [root, nodeIds] : beamComponents)
        {
            Q_UNUSED(root);
            const bool hasRotationalSupport = std::any_of(
                nodeIds.cbegin(), nodeIds.cend(),
                [&rotationallySupportedEndpoints](int nodeId)
                {
                    return rotationallySupportedEndpoints.find(nodeId)
                        != rotationallySupportedEndpoints.cend();
                });
            if (hasRotationalSupport)
                continue;
            const int referenceNodeId = *std::max_element(
                nodeIds.cbegin(), nodeIds.cend(),
                [&beamNodeDegree](int lhs, int rhs)
                {
                    const int lhsDegree = beamNodeDegree[lhs];
                    const int rhsDegree = beamNodeDegree[rhs];
                    return lhsDegree != rhsDegree ? lhsDegree < rhsDegree : lhs > rhs;
                });
            build.structure->Add_Constraint(
                { referenceNodeId }, { 3, 4, 5 }, { 0.0, 0.0, 0.0 });
        }
        for (int endpointId : endpointIds)
            addZeroConstraint(endpointId, requiredNodeDofs(endpointId));
        // 中间 P 节点只连接 H 型悬垂串。当前悬垂串固定采用 T3D2，
        // 因此该节点只存在三个平动自由度，与线身采用索或梁无关。
        for (const auto& suspension : lineResult.suspensionPoints)
        {
            if (suspension.supportNodeId > 0)
                addZeroConstraint(suspension.supportNodeId, 3);
        }
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
    build.spacerCount = static_cast<int>(lineResult.innerSpacers.size());
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
QCheckBox* ConductorModule::innerSpacerCheck() const
{
    return m_ui->innerSpacerCheck;
}
QComboBox* ConductorModule::spacerLayoutCombo() const
{
    return m_ui->spacerLayoutCombo;
}
QSpinBox* ConductorModule::spacerCountSpin() const
{
    return m_ui->spacerCountSpin;
}
QComboBox* ConductorModule::spacerElementCombo() const
{
    return m_ui->spacerElementCombo;
}
QComboBox* ConductorModule::spacerMaterialCombo() const
{
    return m_ui->spacerMaterialCombo;
}
QComboBox* ConductorModule::spacerSectionCombo() const
{
    return m_ui->spacerSectionCombo;
}
QCheckBox* ConductorModule::analysisCheck() const
{
    return m_ui->analysisCheck;
}
QPushButton* ConductorModule::createButton() const
{
    return m_ui->createButton;
}
