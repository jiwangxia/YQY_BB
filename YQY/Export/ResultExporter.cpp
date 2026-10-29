#include "Export/ResultExporter.h"

#include "Export/Hdf5ModelIO.h"
#include <QSaveFile>
#include <QTextStream>
#include <QLocale>
#include <cmath>
#include <algorithm>

bool ResultExporter::exportEnergyBdf(const QString& outputFile, const std::vector<Hdf5EnergyHistorySample>& samples,
                                     unsigned int selectedMask) const
{
    const unsigned int masks[] = {EnergyResult::Kinetic, EnergyResult::Strain, EnergyResult::Gravity,
                                  EnergyResult::Mechanical};
    const char* columns[] = {"KINETIC", "STRAIN", "GRAVITY", "MECHANICAL"};
    unsigned int available = 0;
    for (const auto& sample : samples)
        available |= sample.energy.availableMask;
    selectedMask &= available;
    if (samples.empty() || selectedMask == 0)
        return false;
    QSaveFile file(outputFile);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    QTextStream stream(&file);
    stream.setLocale(QLocale::c());
    stream.setRealNumberPrecision(17);
    stream << "$ Energy history: TIME in s; energies in J; NA means unavailable\n";
    for (const char* column : {"TIME", "STEP", "INCREMENT", "LOAD_FACTOR"})
        stream << QString::fromLatin1(column).rightJustified(24) << ' ';
    for (int component = 0; component < 4; ++component)
        if (selectedMask & masks[component])
            stream << QString::fromLatin1(columns[component]).rightJustified(24) << ' ';
    stream << '\n';
    for (const auto& sample : samples)
    {
        stream << QString::number(sample.info.time, 'g', 17).rightJustified(24) << ' '
               << QString::number(sample.info.stepId).rightJustified(24) << ' '
               << QString::number(sample.info.increment).rightJustified(24) << ' '
               << QString::number(sample.info.loadFactor, 'g', 17).rightJustified(24) << ' ';
        const double values[] = {sample.energy.kinetic, sample.energy.strain, sample.energy.gravity,
                                 sample.energy.mechanical};
        for (int component = 0; component < 4; ++component)
        {
            if (!(selectedMask & masks[component]))
                continue;
            const QString value = (sample.energy.availableMask & masks[component]) && std::isfinite(values[component])
                                      ? QString::number(values[component], 'g', 17)
                                      : QStringLiteral("NA");
            stream << value.rightJustified(24) << ' ';
        }
        stream << '\n';
    }
    stream.flush();
    return stream.status() == QTextStream::Ok && file.commit();
}

bool ResultExporter::exportNodalHistoryBdf(const QString& outputFile, int nodeId,
                                           const std::vector<Hdf5NodalDisplacementHistorySample>& samples,
                                           const std::array<bool, 6>& selected) const
{
    if (samples.empty() || std::none_of(selected.begin(), selected.end(),
                                        [](bool value)
                                        {
                                            return value;
                                        }))
        return false;
    QSaveFile file(outputFile);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return false;
    QTextStream stream(&file);
    stream.setLocale(QLocale::c());
    const char* names[] = {"U1", "U2", "U3", "UR1", "UR2", "UR3"};
    stream << QStringLiteral("TIME").rightJustified(24) << ' ';
    for (std::size_t component = 0; component < selected.size(); ++component)
        if (selected[component])
            stream << QStringLiteral("N%1-%2").arg(nodeId).arg(QString::fromLatin1(names[component])).rightJustified(24)
                   << ' ';
    stream << '\n';
    for (const auto& sample : samples)
    {
        stream << QString::number(sample.time, 'g', 17).rightJustified(24) << ' ';
        for (std::size_t component = 0; component < selected.size(); ++component)
            if (selected[component])
                stream << QString::number(sample.displacement[component], 'g', 17).rightJustified(24) << ' ';
        stream << '\n';
    }
    stream.flush();
    return stream.status() == QTextStream::Ok && file.commit();
}

bool ResultExporter::exportBdf(const QString& sourceFile, const QString& outputFile, const std::vector<int>& nodeIds,
                               const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
                               const std::vector<int>& elementIds,
                               const std::vector<EnumKeyword::ElementResultType>& elementTypes) const
{
    Hdf5ModelIO modelIO;
    return modelIO.ExportBdfResultFromHdf5(sourceFile, outputFile, nodeIds, nodeTypes, elementIds, elementTypes);
}
