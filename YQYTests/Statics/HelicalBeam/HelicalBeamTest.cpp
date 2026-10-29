#include "Statics/HelicalBeam/HelicalBeamTest.h"
#include "Support/SolveModel.h"
#include "Support/TestPaths.h"

#include "DataStructure/Structure/StructureData.h"
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextStream>
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>

namespace
{
static QString BattiniFileHash(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file))
        return {};
    return QString::fromLatin1(hash.result().toHex());
}

static bool ReadBattiniReference(const QString& path, _OUT std::vector<std::pair<double, double>>& points,
                                 _OUT QString& error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("无法读取 Battini 图示参考：%1").arg(path);
        return false;
    }
    const QByteArray data = file.readAll().replace("\r\n", "\n");
    const QByteArray hash = QCryptographicHash::hash(data, QCryptographicHash::Sha256).toHex();
    if (hash != "9acca01231e4cc527e4a1859e94554a34fe23e060f699f7be5550884f410d5b4")
    {
        error = QStringLiteral("Battini 参考 CSV 的 LF 归一化 SHA256 不匹配。");
        return false;
    }
    const QStringList lines = QString::fromUtf8(data).split('\n', Qt::SkipEmptyParts);
    if (lines.isEmpty() || lines.front().trimmed() != QStringLiteral("pseudo_time,tip_uz_graphical"))
    {
        error = QStringLiteral("Battini 参考 CSV 表头不匹配。");
        return false;
    }
    points.clear();
    for (int index = 1; index < lines.size(); ++index)
    {
        const QStringList fields = lines[index].trimmed().split(',');
        bool timeOk = false;
        bool valueOk = false;
        const double time = fields.value(0).toDouble(&timeOk);
        const double value = fields.value(1).toDouble(&valueOk);
        if (fields.size() != 2 || !timeOk || !valueOk || !std::isfinite(time) || !std::isfinite(value) ||
            (!points.empty() && time <= points.back().first))
        {
            error = QStringLiteral("Battini 参考 CSV 第%1行无效。").arg(index + 1);
            return false;
        }
        points.emplace_back(time, value);
    }
    if (points.size() != 755 || points.front().first != 0.0 || std::abs(points.back().first - 6.0) > 1.0e-12)
    {
        error = QStringLiteral("Battini 参考 CSV 未覆盖完整的755点、0–6加载区间。");
        return false;
    }
    return true;
}
}

std::optional<int> VerificationCases::verifyBattiniHelicalBeam(const QStringList& arguments)
{
    const int argumentIndex = arguments.indexOf(QStringLiteral("--verify-battini-helical-beam"));
    if (argumentIndex < 0)
        return std::nullopt;

    try
    {
        const QString output = arguments.value(
            argumentIndex + 1,
            QDir(TestPaths::RepositoryRoot())
                .filePath(QStringLiteral("tmp/battini-helical-beam-%1")
                              .arg(QDateTime::currentDateTimeUtc().toString(QStringLiteral("yyyyMMdd-HHmmss-zzz")))));
        if (output.startsWith(QStringLiteral("--")) || !QDir().mkpath(output))
        {
            QTextStream(stderr) << "无法建立 Battini 输出目录：" << output << Qt::endl;
            return 2;
        }
        const QString relativeDirectory = QStringLiteral("Statics/HelicalBeam/");
        const QDir modelDirectory(
            QDir(TestPaths::TestRoot()).filePath(QStringLiteral("TestModels/") + relativeDirectory));
        const QString referencePath = modelDirectory.filePath(QStringLiteral("reference/figure7_4_tip_z.csv"));
        std::vector<std::pair<double, double>> reference;
        QString error;
        if (!ReadBattiniReference(referencePath, reference, error))
        {
            QTextStream(stderr) << error << Qt::endl;
            return 2;
        }
        const auto model = SolveModel::Read(relativeDirectory + QStringLiteral("HelicalBeam.bdf"), error);
        if (!model || model->m_Nodes.size() != 61 || model->m_Elements.size() != 60 ||
            model->m_AnalysisStep.size() != 1)
        {
            QTextStream(stderr) << "Battini 模型读取或实体数量检查失败：" << error << Qt::endl;
            return 3;
        }
        // 复用真实读取和区域求解；结果文件由公共辅助放在独立临时目录。
        if (!SolveModel::Run(model, error))
        {
            QTextStream(stderr) << "Battini 完整求解失败：" << error << Qt::endl;
            return 4;
        }
        const auto& frames = model->GetOutputter().GetDataSet();
        if (frames.size() != 3001)
        {
            QTextStream(stderr) << "Battini 结果帧数量错误，期望3001，实际" << frames.size() << Qt::endl;
            return 4;
        }
        const std::array<EnumKeyword::NodeResultType, 6> fixedTypes = {
            EnumKeyword::NodeResultType::U1,  EnumKeyword::NodeResultType::U2,  EnumKeyword::NodeResultType::U3,
            EnumKeyword::NodeResultType::UR1, EnumKeyword::NodeResultType::UR2, EnumKeyword::NodeResultType::UR3};
        std::vector<std::pair<double, double>> response;
        double maximumFixedDisplacement = 0.0;
        QFile responseFile(QDir(output).filePath(QStringLiteral("response.csv")));
        if (!responseFile.open(QIODevice::WriteOnly | QIODevice::Text))
            return 2;
        QTextStream responseStream(&responseFile);
        responseStream.setRealNumberPrecision(17);
        responseStream << "pseudo_time,applied_load_fraction,Fz,Mz,tip_ux,tip_uy,tip_uz\n";
        for (std::size_t index = 0; index < frames.size(); ++index)
        {
            const auto& frame = frames[index];
            if (std::abs(frame.GetTime() - 0.002 * index) > 1.0e-10 || frame.GetStepId() != 1 ||
                frame.GetNodeDatas().size() != 61 || frame.GetElementDatas().size() != 60 ||
                !frame.GetNodeDatas().contains(1) || !frame.GetNodeDatas().contains(61))
            {
                QTextStream(stderr) << "Battini 第" << index << "帧时间、实体或分析步不匹配。" << Qt::endl;
                return 4;
            }
            for (const auto type : fixedTypes)
                maximumFixedDisplacement = std::max(maximumFixedDisplacement, std::abs(frame.GetNodeData(1, type)));
            const double time = frame.GetTime();
            const double fraction = time / 6.0;
            const double uz = frame.GetNodeData(61, EnumKeyword::NodeResultType::U3);
            response.emplace_back(time, uz);
            responseStream << time << ',' << fraction << ',' << 30.0 * fraction << ','
                           << 120.0 * std::acos(-1.0) * fraction << ','
                           << frame.GetNodeData(61, EnumKeyword::NodeResultType::U1) << ','
                           << frame.GetNodeData(61, EnumKeyword::NodeResultType::U2) << ',' << uz << '\n';
        }
        responseStream.flush();
        if (responseStream.status() != QTextStream::Ok)
            return 2;
        responseFile.close();

        double maximumDifference = 0.0;
        double squaredDifferenceSum = 0.0;
        std::size_t lower = 0;
        for (const auto& [time, graphicalValue] : reference)
        {
            while (lower + 1 < response.size() - 1 && response[lower + 1].first < time)
                ++lower;
            const auto& first = response[lower];
            const auto& second = response[lower + 1];
            const double fraction = (time - first.first) / (second.first - first.first);
            const double interpolated = first.second + fraction * (second.second - first.second);
            const double difference = interpolated - graphicalValue;
            maximumDifference = std::max(maximumDifference, std::abs(difference));
            squaredDifferenceSum += difference * difference;
        }
        const double rmsDifference = std::sqrt(squaredDifferenceSum / reference.size());
        // 图线横向宽度约0.018；0.02只验收图示级吻合，不表示对作者原始数据的精度。
        const bool passed = maximumDifference <= 0.02 && maximumFixedDisplacement <= 1.0e-12;
        const QJsonObject summary = {
            {QStringLiteral("case_id"), QStringLiteral("Battini2002.HelicalBeam.StaticPhysicalBranch.EB60")},
            {QStringLiteral("passed"), passed},
            {QStringLiteral("read_complete"), true},
            {QStringLiteral("solve_complete"), true},
            {QStringLiteral("frames"), static_cast<int>(frames.size())},
            {QStringLiteral("completed_pseudo_time"), response.back().first},
            {QStringLiteral("reference_points"), static_cast<int>(reference.size())},
            {QStringLiteral("max_abs_tip_z_graphical_difference"), maximumDifference},
            {QStringLiteral("rms_tip_z_graphical_difference"), rmsDifference},
            {QStringLiteral("graphical_absolute_tolerance"), 0.02},
            {QStringLiteral("max_abs_clamped_displacement"), maximumFixedDisplacement},
            {QStringLiteral("input_sha256"),
             BattiniFileHash(modelDirectory.filePath(QStringLiteral("HelicalBeam.bdf")))},
            {QStringLiteral("executable_sha256"), BattiniFileHash(QCoreApplication::applicationFilePath())},
            {QStringLiteral("scope"),
             QStringLiteral("静力主响应的图示级验收；当前为EB共旋梁，非原t3dl或全部参数化对比。")}};
        QFile summaryFile(QDir(output).filePath(QStringLiteral("verification.json")));
        const QByteArray summaryData = QJsonDocument(summary).toJson(QJsonDocument::Indented);
        if (!summaryFile.open(QIODevice::WriteOnly) || summaryFile.write(summaryData) != summaryData.size() ||
            !summaryFile.flush())
            return 2;
        QTextStream(stdout) << "Battini helical beam: " << (passed ? "PASS" : "FAIL") << " frames=" << frames.size()
                            << " pseudo_time=" << response.back().first
                            << " max_graphical_difference=" << maximumDifference << " rms=" << rmsDifference
                            << " output=" << QDir(output).absolutePath() << Qt::endl;
        return passed ? 0 : 5;
    }
    catch (const std::exception& exception)
    {
        QTextStream(stderr) << "Battini 验证异常：" << exception.what() << Qt::endl;
        return 4;
    }
}
