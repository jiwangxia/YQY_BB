#include "AnalysisStep.h"

#include "Application/ApplicationPaths.h"
#include <barrier>
#include <mkl_service.h>
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementBase.h"
#include "DataStructure/Element/ElementBeam_CR.h"
#include "DataStructure/Element/ElementBeam_Simo.h"
#include "DataStructure/Element/ElementCable.h"
#include "DataStructure/Element/ElementTruss.h"
#include "DataStructure/Element/ElementSpringBase.h"
#include "DataStructure/Aerodynamics/AeroManager.h"
#include "DataStructure/Aerodynamics/BundleAeroMapper.h"
#include "DataStructure/Load/LoadAssembler.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Wind.h"
#include "Solver/Interface/ISolver.h"
#include "Solver/Dynamic/Integrators/SolverAdaptiveTSSBN.h"
#include "Solver/Dynamic/Damping/ConductorModalClassifier.h"
#include "Solver/Dynamic/Damping/LocalRayleighMatrix.h"
#include "Solver/Factory/SolverFactory.h"
#include "Solver/Static/SolverStatic.h"
#include "Solver/Assembly/AssemblySettings.h"
#include "Solver/Constraint/NonlinearMPC.h"
#include "Solver/Linear/LinearSystemSolver.h"
#include "Solver/Linear/Gpu/GpuSettings.h"
#include "Utility/CR.h"
#include "Utility/Logger/Logger.h"
#include <QBuffer>
#include <QCryptographicHash>
#include <QDataStream>
#include <QDebug>
#include <QFile>
#include <QSaveFile>
#include <QScopeGuard>
#include <QtConcurrent/QtConcurrentRun>
#include <array>
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <typeinfo>
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace
{
// oneMKL 的 local 设置只作用于调用 PARDISO 的当前区域任务线程；析构时恢复，
// 因而不会修改同一进程中其他独立分析任务的求解器线程策略。
class MklLocalThreadLimit final
{
public:
    explicit MklLocalThreadLimit(int threadCount)
        : m_enabled(threadCount > 0)
    {
        if (m_enabled)
            m_previousThreadCount = mkl_set_num_threads_local(threadCount);
    }

    ~MklLocalThreadLimit()
    {
        if (m_enabled)
            mkl_set_num_threads_local(m_previousThreadCount);
    }

    MklLocalThreadLimit(const MklLocalThreadLimit&) = delete;
    MklLocalThreadLimit& operator=(const MklLocalThreadLimit&) = delete;

private:
    int m_previousThreadCount = 1;
    bool m_enabled = false;
};

constexpr quint32 tssbnCheckpointMagic = 0x54534350; // TSCP
constexpr quint32 tssbnCheckpointVersion = 2;
constexpr qint64 maximumTssbnCheckpointBytes = 64LL * 1024LL * 1024LL;

struct TssbnNodeRecord
{
    int id = 0;
    std::vector<double> displacement;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d materialOmega = Eigen::Vector3d::Zero();
    Eigen::Vector3d materialAlpha = Eigen::Vector3d::Zero();
};

struct TssbnCableRecord
{
    int id = 0;
    std::array<double, 2> committedTwist{};
};

struct TssbnBeamRecord
{
    int id = 0;
    std::array<Eigen::Matrix3d, 2> rotations{};
};

struct TssbnCheckpointRecord
{
    SolverNameSpace::SolverAdaptiveTSSBN::CheckpointState controller;
    std::vector<TssbnNodeRecord> nodes;
    std::vector<TssbnCableRecord> cables;
    std::vector<TssbnBeamRecord> beams;
    Eigen::VectorXd mpcMultipliers;
};

void WriteCheckpointVector(QDataStream& stream, const std::vector<double>& values)
{
    for (double value : values)
        stream << value;
}

bool ReadCheckpointVector(QDataStream& stream, int count, _OUT std::vector<double>& values)
{
    values.resize(static_cast<std::size_t>(count));
    for (double& value : values)
    {
        stream >> value;
        if (!std::isfinite(value))
            return false;
    }
    return stream.status() == QDataStream::Ok;
}

// 验证恢复时仍使用同一静力平衡构型、拓扑、气动数据选择和阻尼算子。
QByteArray MakeTssbnModelSignature(const StructureData& structure, int fixedDofs, int freeDofs,
                                   const SolverNameSpace::StructuralDampingModel& damping,
                                    const QByteArray& caseSignature, GallopingProfileBindingMode profileBindingMode,
                                    const std::map<int, int>& profileBindings, bool referenceTssbn,
                                    const SolverNameSpace::AdaptiveTssbnSettings& tssbnSettings)
{
    QByteArray bytes;
    QDataStream stream(&bytes, QIODevice::WriteOnly);
    stream.setVersion(QDataStream::Qt_6_0);
    if (referenceTssbn)
        stream << QByteArray("Reference-TSSBN-Galloping-v2-table")
               << structure.m_AeroManager.isShapePreservingInterpolationEnabled();
    // 旧参考签名字节保持不变；独立控制策略及其实际速度容限必须拒绝跨策略恢复。
    if (referenceTssbn && tssbnSettings.referenceIndependentErrorControl)
        stream << QByteArray("Reference-TSSBN-independent-error-v1")
               << qint32(tssbnSettings.errorNormMode) << tssbnSettings.velocityAbsoluteTolerance;
    stream << QByteArray("TSSBN-qv-real-y3-iteration-change-v1");
    // 整体位移尺度改变接受步判据，不能沿用旧分量误差策略的检查点。
    if (!referenceTssbn &&
        tssbnSettings.errorNormMode == SolverNameSpace::TssbnErrorNormMode::DisplacementGlobalVelocityMax)
        stream << QByteArray("Adaptive-TSSBN-global-displacement-v1");
    // 整体速度尺度与旧位移整体模式分开，避免跨误差策略恢复。
    if (!referenceTssbn && tssbnSettings.errorNormMode == SolverNameSpace::TssbnErrorNormMode::GlobalMax)
        stream << QByteArray("Adaptive-TSSBN-global-max-v1");
    stream << caseSignature << qint32(fixedDofs) << qint32(freeDofs)
           << qint32(structure.m_Nodes.size()) << qint32(structure.m_Elements.size());
    for (const auto& [id, node] : structure.m_Nodes)
    {
        stream << qint32(id) << node->m_X << node->m_Y << node->m_Z << qint32(node->m_DOF.size());
        for (int dof : node->m_DOF)
            stream << qint32(dof);
        for (double value : node->m_Displacement)
            stream << value;
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                stream << node->m_Rg(row, column);
    }
    for (const auto& [id, element] : structure.m_Elements)
    {
        stream << qint32(id) << QString::fromLatin1(typeid(*element).name()) << element->L0
               << element->m_InitStress << qint32(element->m_WireId) << qint32(element->m_AeroProfileId)
               << qint32(element->m_pNode.size());
        for (const auto& nodeReference : element->m_pNode)
        {
            const auto node = nodeReference.lock();
            stream << qint32(node ? node->m_Id : -1);
        }
        if (const auto cable = std::dynamic_pointer_cast<ElementCable>(element))
            for (double twist : cable->GetCommittedSpatialTwist())
                stream << twist;
    }
    const auto& settings = damping.settings;
    // 旧显式瑞丽检查点保持兼容；自动分组方案另加版本及用户选择的参考阶次。
    stream << settings.enabled << qint32(settings.formulation) << settings.translationDampingRatio
           << settings.torsionDampingRatio << 3.0 << settings.useExplicitRayleighCoefficients
           << settings.translationMassCoefficient << settings.translationStiffnessCoefficient
           << settings.torsionMassCoefficient << settings.torsionStiffnessCoefficient;
    const auto& report = damping.report;
    stream << report.translationMassCoefficient << report.translationStiffnessCoefficient
           << report.torsionMassCoefficient << report.torsionStiffnessCoefficient;
    if (!report.localGroups.empty())
    {
        stream << QByteArray("YQY-ElementLocalRayleighGroups-v1") << qint32(report.localGroups.size());
        for (const auto& group : report.localGroups)
        {
            stream << group.label;
            for (double coefficient : group.coefficients)
                stream << coefficient;
        }
    }
    stream << qint32(damping.IsComputed() ? freeDofs : 0) << qint32(0);
    if (settings.enabled && !settings.useExplicitRayleighCoefficients)
    {
        stream << QByteArray("YQY-GroupedRayleigh-v1");
        for (int mode : settings.translationReferenceModes)
            stream << qint32(mode);
        for (int mode : settings.torsionReferenceModes)
            stream << qint32(mode);
    }
    // 保持旧自动模式的签名字节不变；显式模式必须把真正使用的列号纳入恢复校验。
    if (profileBindingMode == GallopingProfileBindingMode::ExplicitElementProfile)
    {
        stream << qint32(profileBindingMode) << qint32(profileBindings.size());
        for (const auto& [elementId, profileId] : profileBindings)
            stream << qint32(elementId) << qint32(profileId);
    }
    return stream.status() == QDataStream::Ok
               ? QCryptographicHash::hash(bytes, QCryptographicHash::Sha256)
               : QByteArray();
}

bool WriteTssbnCheckpoint(const QString& path, const QByteArray& signature,
                          const SolverNameSpace::SolverAdaptiveTSSBN::CheckpointState& controller,
                          const StructureData& structure, const Eigen::VectorXd& multipliers, _OUT QString& error)
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    stream.setVersion(QDataStream::Qt_6_0);
    stream << signature << controller.time << controller.nextTimeStep << controller.previousAcceptedError
           << controller.previousAcceptedTimeStep << controller.acceptedTimeStepSum
           << qint32(controller.acceptedStepCount) << qint32(controller.rejectedStepCount)
           << qint64(controller.totalStageNewtonIterations) << qint32(controller.maximumStageNewtonIterations)
           << qint32(structure.m_Nodes.size());
    for (const auto& [id, node] : structure.m_Nodes)
    {
        const int count = static_cast<int>(node->m_DOF.size());
        if (count < 1 || count > 6 || node->m_Displacement.size() != count || node->m_Velocity.size() != count ||
            node->m_Acceleration.size() != count || !node->m_Rg.allFinite() ||
            !node->m_OmegaMaterial.allFinite() || !node->m_AlphaMaterial.allFinite() ||
            !std::all_of(node->m_Displacement.cbegin(), node->m_Displacement.cend(),
                         [](double value) { return std::isfinite(value); }) ||
            !std::all_of(node->m_Velocity.cbegin(), node->m_Velocity.cend(),
                         [](double value) { return std::isfinite(value); }) ||
            !std::all_of(node->m_Acceleration.cbegin(), node->m_Acceleration.cend(),
                         [](double value) { return std::isfinite(value); }))
        {
            error = QStringLiteral("检查点节点 %1 状态尺寸或有限性无效").arg(id);
            return false;
        }
        stream << qint32(id) << qint32(count);
        WriteCheckpointVector(stream, node->m_Displacement);
        WriteCheckpointVector(stream, node->m_Velocity);
        WriteCheckpointVector(stream, node->m_Acceleration);
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                stream << node->m_Rg(row, column);
        for (int component = 0; component < 3; ++component)
            stream << node->m_OmegaMaterial[component] << node->m_AlphaMaterial[component];
    }
    qint32 cableCount = 0;
    for (const auto& [id, element] : structure.m_Elements)
        if (std::dynamic_pointer_cast<ElementCable>(element))
            ++cableCount;
    stream << cableCount;
    for (const auto& [id, element] : structure.m_Elements)
    {
        const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
        if (!cable)
            continue;
        const auto& twist = cable->GetCommittedSpatialTwist();
        if (!std::isfinite(twist[0]) || !std::isfinite(twist[1]))
        {
            error = QStringLiteral("检查点索单元 %1 已提交扭角无效").arg(id);
            return false;
        }
        stream << qint32(id) << twist[0] << twist[1];
    }
    qint32 beamCount = 0;
    for (const auto& [id, element] : structure.m_Elements)
        if (const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(element);
            beam && !beam->IsReferenceTssbnDynamicMode())
            ++beamCount;
    stream << beamCount;
    for (const auto& [id, element] : structure.m_Elements)
    {
        const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(element);
        if (!beam || beam->IsReferenceTssbnDynamicMode())
            continue;
        std::array<Matrix3d, 2> rotations;
        beam->GetCommittedGaussRotations(rotations);
        stream << qint32(id);
        for (const auto& rotation : rotations)
        {
            if (!rotation.allFinite() || (rotation.transpose() * rotation - Matrix3d::Identity()).norm() > 1.0e-8 ||
                std::abs(rotation.determinant() - 1.0) > 1.0e-8)
            {
                error = QStringLiteral("检查点梁单元 %1 已提交 Gauss 姿态无效").arg(id);
                return false;
            }
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    stream << rotation(row, column);
        }
    }
    if (!multipliers.allFinite())
    {
        error = QStringLiteral("检查点 MPC 乘子存在非有限数值");
        return false;
    }
    stream << qint32(multipliers.size());
    for (Eigen::Index index = 0; index < multipliers.size(); ++index)
        stream << multipliers[index];
    if (stream.status() != QDataStream::Ok || payload.size() > maximumTssbnCheckpointBytes)
    {
        error = QStringLiteral("检查点序列化失败或文件过大");
        return false;
    }

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly))
    {
        error = QStringLiteral("无法创建检查点 %1：%2").arg(path, file.errorString());
        return false;
    }
    QDataStream output(&file);
    output.setVersion(QDataStream::Qt_6_0);
    output << tssbnCheckpointMagic << tssbnCheckpointVersion << payload
           << QCryptographicHash::hash(payload, QCryptographicHash::Sha256);
    if (output.status() != QDataStream::Ok || !file.commit())
    {
        error = QStringLiteral("检查点原子提交失败：%1").arg(file.errorString());
        return false;
    }
    return true;
}

bool ReadTssbnCheckpoint(const QString& path, const QByteArray& expectedSignature, const StructureData& structure,
                         _OUT TssbnCheckpointRecord& record, _OUT QString& error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly) || file.size() > maximumTssbnCheckpointBytes)
    {
        error = QStringLiteral("检查点无法读取或超过 64 MiB：%1").arg(path);
        return false;
    }
    QDataStream envelope(&file);
    envelope.setVersion(QDataStream::Qt_6_0);
    quint32 magic = 0;
    quint32 version = 0;
    QByteArray payload;
    QByteArray digest;
    envelope >> magic >> version >> payload >> digest;
    if (envelope.status() != QDataStream::Ok || !file.atEnd() || magic != tssbnCheckpointMagic ||
        (version != 1 && version != tssbnCheckpointVersion) || digest.size() != 32 ||
        digest != QCryptographicHash::hash(payload, QCryptographicHash::Sha256))
    {
        error = QStringLiteral("检查点格式、版本或 SHA-256 完整性校验失败");
        return false;
    }

    QBuffer buffer(&payload);
    buffer.open(QIODevice::ReadOnly);
    QDataStream input(&buffer);
    input.setVersion(QDataStream::Qt_6_0);
    QByteArray signature;
    qint32 acceptedCount = 0;
    qint32 rejectedCount = 0;
    qint64 totalNewton = 0;
    qint32 maximumNewton = 0;
    qint32 nodeCount = 0;
    auto& controller = record.controller;
    input >> signature >> controller.time >> controller.nextTimeStep >> controller.previousAcceptedError
          >> controller.previousAcceptedTimeStep >> controller.acceptedTimeStepSum >> acceptedCount >> rejectedCount
          >> totalNewton >> maximumNewton >> nodeCount;
    controller.acceptedStepCount = acceptedCount;
    controller.rejectedStepCount = rejectedCount;
    controller.totalStageNewtonIterations = totalNewton;
    controller.maximumStageNewtonIterations = maximumNewton;
    if (signature != expectedSignature || input.status() != QDataStream::Ok ||
        nodeCount != static_cast<qint32>(structure.m_Nodes.size()))
    {
        error = QStringLiteral("检查点模型配置签名或节点数量不匹配");
        return false;
    }
    record.nodes.reserve(static_cast<std::size_t>(nodeCount));
    for (const auto& [expectedId, node] : structure.m_Nodes)
    {
        qint32 id = 0;
        qint32 count = 0;
        input >> id >> count;
        if (id != expectedId || count != node->m_DOF.size() || count < 1 || count > 6)
        {
            error = QStringLiteral("检查点节点拓扑与当前模型不匹配：%1").arg(expectedId);
            return false;
        }
        TssbnNodeRecord state;
        state.id = id;
        if (!ReadCheckpointVector(input, count, state.displacement) ||
            !ReadCheckpointVector(input, count, state.velocity) ||
            !ReadCheckpointVector(input, count, state.acceleration))
        {
            error = QStringLiteral("检查点节点 %1 运动状态无效").arg(id);
            return false;
        }
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                input >> state.rotation(row, column);
        for (int component = 0; component < 3; ++component)
            input >> state.materialOmega[component] >> state.materialAlpha[component];
        if (input.status() != QDataStream::Ok || !state.rotation.allFinite() ||
            !state.materialOmega.allFinite() || !state.materialAlpha.allFinite() ||
            (state.rotation.transpose() * state.rotation - Eigen::Matrix3d::Identity()).norm() > 1.0e-8 ||
            std::abs(state.rotation.determinant() - 1.0) > 1.0e-8)
        {
            error = QStringLiteral("检查点节点 %1 有限转动状态无效").arg(id);
            return false;
        }
        record.nodes.push_back(std::move(state));
    }
    qint32 cableCount = 0;
    input >> cableCount;
    const int expectedCableCount = static_cast<int>(std::count_if(
        structure.m_Elements.cbegin(), structure.m_Elements.cend(),
        [](const auto& entry) { return std::dynamic_pointer_cast<ElementCable>(entry.second) != nullptr; }));
    if (input.status() != QDataStream::Ok || cableCount != expectedCableCount)
    {
        error = QStringLiteral("检查点索单元数量与当前模型不匹配");
        return false;
    }
    record.cables.reserve(static_cast<std::size_t>(cableCount));
    for (const auto& [expectedId, element] : structure.m_Elements)
    {
        if (!std::dynamic_pointer_cast<ElementCable>(element))
            continue;
        qint32 id = 0;
        TssbnCableRecord cable;
        input >> id >> cable.committedTwist[0] >> cable.committedTwist[1];
        if (input.status() != QDataStream::Ok || id != expectedId ||
            !std::isfinite(cable.committedTwist[0]) || !std::isfinite(cable.committedTwist[1]))
        {
            error = QStringLiteral("检查点索单元 %1 已提交扭角无效").arg(expectedId);
            return false;
        }
        cable.id = id;
        record.cables.push_back(cable);
    }
    const int expectedBeamCount = static_cast<int>(std::count_if(
        structure.m_Elements.cbegin(), structure.m_Elements.cend(),
        [](const auto& entry)
        {
            const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(entry.second);
            return beam && !beam->IsReferenceTssbnDynamicMode();
        }));
    if (version == 1 && expectedBeamCount > 0)
    {
        error = QStringLiteral("旧版检查点缺少 CR 梁 Gauss 姿态历史，无法恢复正式梁动力学");
        return false;
    }
    if (version >= 2)
    {
        qint32 beamCount = 0;
        input >> beamCount;
        if (input.status() != QDataStream::Ok || beamCount != expectedBeamCount)
        {
            error = QStringLiteral("检查点梁单元数量与当前模型不匹配");
            return false;
        }
        for (const auto& [expectedId, element] : structure.m_Elements)
        {
            const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(element);
            if (!beam || beam->IsReferenceTssbnDynamicMode())
                continue;
            qint32 id = 0;
            input >> id;
            TssbnBeamRecord state;
            state.id = id;
            for (auto& rotation : state.rotations)
                for (int row = 0; row < 3; ++row)
                    for (int column = 0; column < 3; ++column)
                        input >> rotation(row, column);
            bool valid = input.status() == QDataStream::Ok && id == expectedId;
            for (const auto& rotation : state.rotations)
                valid = valid && rotation.allFinite() &&
                        (rotation.transpose() * rotation - Matrix3d::Identity()).norm() <= 1.0e-8 &&
                        std::abs(rotation.determinant() - 1.0) <= 1.0e-8;
            if (!valid)
            {
                error = QStringLiteral("检查点梁单元 %1 已提交 Gauss 姿态无效").arg(expectedId);
                return false;
            }
            record.beams.push_back(state);
        }
    }
    qint32 multiplierCount = 0;
    input >> multiplierCount;
    if (input.status() != QDataStream::Ok || multiplierCount < 0 || multiplierCount > 100000)
    {
        error = QStringLiteral("检查点 MPC 乘子数量无效");
        return false;
    }
    record.mpcMultipliers.resize(multiplierCount);
    for (Eigen::Index index = 0; index < record.mpcMultipliers.size(); ++index)
        input >> record.mpcMultipliers[index];
    if (input.status() != QDataStream::Ok || !buffer.atEnd() || !record.mpcMultipliers.allFinite() ||
        !std::isfinite(controller.time) || !std::isfinite(controller.nextTimeStep) ||
        !std::isfinite(controller.previousAcceptedError) || !std::isfinite(controller.previousAcceptedTimeStep) ||
        !std::isfinite(controller.acceptedTimeStepSum) || controller.time < 0.0 ||
        controller.nextTimeStep <= 0.0 || controller.previousAcceptedError <= 0.0 ||
        controller.previousAcceptedTimeStep <= 0.0 || controller.acceptedTimeStepSum <= 0.0 ||
        controller.acceptedStepCount < 1 || controller.rejectedStepCount < 0 ||
        controller.totalStageNewtonIterations < 0 || controller.maximumStageNewtonIterations < 0)
    {
        error = QStringLiteral("检查点控制器或 MPC 状态不完整");
        return false;
    }
    return true;
}

bool VerifyBuiltInQuadProfileConvention(const AeroManager& manager, const AeroCaseKey& key, QString& reason)
{
    const std::filesystem::path source = manager.getCaseSourceFile(key);
    if (source.empty())
    {
        reason = QStringLiteral("四列气动表没有来源路径");
        return false;
    }

    const QString directory = ApplicationPaths::aerodynamicDataDirectory();
    if (directory.isEmpty())
    {
        reason = QStringLiteral("找不到程序内置 Output_Data 气动目录");
        return false;
    }
    const std::filesystem::path builtInDirectory(directory.toStdWString());
    const std::filesystem::path expectedSource = builtInDirectory / AeroManager::buildChineseFileName(key);
    std::error_code fileError;
    if (!std::filesystem::equivalent(source, expectedSource, fileError) || fileError)
    {
        reason = QStringLiteral("四列气动表来源不是当前程序内置的 Output_Data/%1")
                     .arg(QString::fromStdWString(expectedSource.filename().wstring()));
        return false;
    }

    AeroManager reference;
    if (!reference.loadCase(builtInDirectory, key))
    {
        reason = QStringLiteral("无法重新读取程序内置四列气动表");
        return false;
    }
    const auto* actual = manager.findCaseModels(key);
    const auto* expected = reference.findCaseModels(key);
    if (!actual || !expected || actual->size() != 4 || expected->size() != 4)
    {
        reason = QStringLiteral("四列气动表的列数不完整");
        return false;
    }
    const auto sameSamples = [](const std::vector<double>& first, const std::vector<double>& second)
    {
        return first.size() == 73 && second.size() == 73 &&
               std::equal(first.cbegin(), first.cend(), second.cbegin(),
                          [](double left, double right) { return std::isfinite(left) && left == right; });
    };
    for (int profile = 0; profile < 4; ++profile)
    {
        const BladeModel& loaded = (*actual)[static_cast<std::size_t>(profile)];
        const BladeModel& original = (*expected)[static_cast<std::size_t>(profile)];
        if (!sameSamples(loaded.lift, original.lift) || !sameSamples(loaded.drag, original.drag) ||
            !sameSamples(loaded.moment, original.moment))
        {
            reason = QStringLiteral("内嵌四列气动系数与程序内置 Output_Data 的 P%1 原始 CSV 不一致")
                         .arg(profile + 1);
            return false;
        }
    }
    return true;
}
}

AeroCaseKey AnalysisStep::GetGallopingAeroCase(int bundleCount, const Force_Wind& wind) const
{
    return AeroCaseKey{bundleCount, static_cast<int>(std::llround(wind.m_velocity)), m_GallopingIceThickness};
}

bool AnalysisStep::ShouldAssembleGalloping(int bundleCount, const Force_Wind& wind) const
{
    return m_EnableGalloping && std::abs(wind.m_velocity - std::llround(wind.m_velocity)) <= 1.0e-9 &&
           AeroManager::isSupportedCase(GetGallopingAeroCase(bundleCount, wind));
}

bool AnalysisStep::SampleGallopingElementAero(int elementId, double time,
                                              _OUT LoadAssembler::GallopingElementAeroSample& sample) const
{
    if (!m_pData || !m_EnableGalloping)
        return false;
    const auto elementIt = m_pData->m_Elements.find(elementId);
    const auto bindingIt = m_gallopingProfileBindings.find(elementId);
    if (elementIt == m_pData->m_Elements.cend() || !elementIt->second ||
        bindingIt == m_gallopingProfileBindings.cend())
        return false;

    const Force_Wind* selectedWind = nullptr;
    double selectedScale = 0.0;
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        Q_UNUSED(loadId);
        const auto wind = std::dynamic_pointer_cast<Force_Wind>(load);
        if (!wind || !IsStepScopedDataActive(wind->m_StepId) ||
            (wind->m_StepId == m_Id && !wind->IsActive(time)))
            continue;
        selectedWind = wind.get();
        selectedScale = wind->m_StepId == m_Id ? wind->GetScaleFactor(time) : 1.0;
    }
    if (!selectedWind)
    {
        sample = {};
        return true;
    }

    sample = LoadAssembler::SampleGallopingElementAero(*selectedWind, *m_pData, *elementIt->second,
                                                       bindingIt->second, m_GallopingIceThickness,
                                                       m_GallopingInitialAttackDegrees, GetModelUpDirection(),
                                                       m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN);
    sample.appliedScale = selectedScale;
    sample.lineTorque *= selectedScale;
    sample.torsionalPowerPerLength *= selectedScale;
    sample.translationalPowerPerLength *= selectedScale;
    sample.rotationalPowerPerLength *= selectedScale;
    sample.totalPowerPerLength *= selectedScale;
    return true;
}

double AnalysisStep::SampleStructuralDampingPower()
{
    if (!m_pData || m_Type != EnumKeyword::StepType::DYNAMIC || !HasActiveLocalRayleighDamping())
        return 0.0;
    VectorXd dampingForce;
    return ComputeLocalRayleighForce(dampingForce);
}

bool AnalysisStep::IsStepScopedDataActive(int sourceStepId) const
{
    // 分析步 0 表示旧版全局作用域，在所有分析分支中生效。
    if (sourceStepId <= 0)
        return true;
    if (sourceStepId == m_Id)
        return true;

    if (m_Type == EnumKeyword::StepType::DYNAMIC && m_InitialStaticStepId > 0)
    {
        // 关联动力分析从选定的静力平衡状态开始，只继承该平衡状态中已经生效的定义。
        // 不能仅因其他动力分析步编号较小，就把同级分支的定义带入当前分支。
        return sourceStepId <= m_InitialStaticStepId;
    }

    // 静力分析以及未指定平衡分析步的独立动力分析继续采用既有顺序语义。
    return sourceStepId < m_Id;
}

void AnalysisStep::SetStructure(std::shared_ptr<StructureData> pStructure)
{
    InvalidateLocalRayleighCache();
    m_pStructure = pStructure;
}

bool AnalysisStep::PreloadGallopingDataForSnapshot(const std::shared_ptr<StructureData>& structure,
                                                   QString* errorMessage)
{
    SetStructure(structure);
    if (!PrepareData())
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("分析步未关联结构数据。");
        return false;
    }
    return PrepareGallopingData(errorMessage);
}

StructureData* AnalysisStep::GetStructure() const
{
    auto ptr = m_pStructure.lock();
    return ptr ? ptr.get() : nullptr;
}

bool AnalysisStep::PrepareData()
{
    m_pData = GetStructure();
    if (!m_pData)
    {
        qDebug().noquote() << QStringLiteral("Error: 分析步未关联结构数据");
        return false;
    }
    return true;
}

bool AnalysisStep::ValidateSolveConfiguration(_OUT QString& errorMessage) const
{
    errorMessage.clear();
    if (m_Type != EnumKeyword::StepType::STATIC && m_Type != EnumKeyword::StepType::DYNAMIC)
    {
        errorMessage = QStringLiteral("分析步类型无效。");
        return false;
    }
    if (!std::isfinite(m_Time) || m_Time <= 0.0)
    {
        errorMessage = QStringLiteral("分析总时间必须是大于 0 的有限数值。");
        return false;
    }
    if (!std::isfinite(m_StepSize) || m_StepSize <= 0.0)
    {
        errorMessage = QStringLiteral("时间步长必须是大于 0 的有限数值。");
        return false;
    }
    if (!std::isfinite(m_Tolerance) || m_Tolerance <= 0.0)
    {
        errorMessage = QStringLiteral("收敛容差必须是大于 0 的有限数值。");
        return false;
    }
    if (m_MaxIterations < 1)
    {
        errorMessage = QStringLiteral("最大迭代次数必须至少为 1。");
        return false;
    }
    if (m_Type == EnumKeyword::StepType::STATIC)
        return true;

    const auto executionPolicy = SolverNameSpace::SolverFactory::GetDynamicExecutionPolicy(m_DynamicSolverType);
    if (!executionPolicy.implemented)
    {
        if (m_DynamicSolverType == SolverNameSpace::SolverType::CentralDifference)
            errorMessage = QStringLiteral("中心差分动力积分器尚未实现，请选择当前可用的动力积分器。");
        else if (m_DynamicSolverType == SolverNameSpace::SolverType::HHT)
            errorMessage = QStringLiteral("HHT-α 动力积分器尚未实现，请选择当前可用的动力积分器。");
        else
            errorMessage = QStringLiteral("动力分析步的积分器类型无效。");
        return false;
    }

    if ((executionPolicy.requiresGalloping && !m_EnableGalloping) ||
        (executionPolicy.requiresStaticState && m_InitialStaticStepId <= 0))
    {
        errorMessage = QStringLiteral("参考 TSSBN 舞动需要启用舞动，并选择前置静力平衡步。");
        return false;
    }

    if (!executionPolicy.supportsNonlinearMpc && HasActiveNonlinearMPC())
    {
        errorMessage = QStringLiteral("当前动力积分器暂不支持非线性 MPC，请改用 Newmark 或自适应 TSSBN。");
        return false;
    }

    if (!executionPolicy.supportsPrescribedBoundaryMotion && !ValidateDynamicPrescribedConstraints(errorMessage))
        return false;

    if (m_DynamicSolverType == SolverNameSpace::SolverType::AdaptiveNewmark ||
        m_DynamicSolverType == SolverNameSpace::SolverType::AdaptiveRungeKutta4)
    {
        try
        {
            const SolverNameSpace::TimeStepController controller(m_AdaptiveTimeStep);
            Q_UNUSED(controller);
        }
        catch (const std::exception& error)
        {
            errorMessage = QStringLiteral("自适应时间步配置无效：%1").arg(QString::fromUtf8(error.what()));
            return false;
        }
    }
    return true;
}

bool AnalysisStep::ValidateDynamicPrescribedConstraints(_OUT QString& errorMessage) const
{
    if (!m_pData)
        return false;

    // 与 Init_DOF 一致，单元可在初始化时扩充节点自由度；继承状态的位移数组必须已完整。
    std::map<const Node*, qsizetype> effectiveDofs;
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        if (node)
            effectiveDofs.emplace(node.get(), node->m_DOF.size());
    }
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (!element)
            continue;
        for (const auto& reference : element->m_pNode)
        {
            const auto node = reference.lock();
            const auto found = effectiveDofs.find(node.get());
            if (found != effectiveDofs.end())
                found->second = std::max(found->second, static_cast<qsizetype>(element->Get_NodeDOF()));
        }
    }
    const auto staticStep = m_pData->m_AnalysisStep.find(m_InitialStaticStepId);
    const bool hasInheritedStaticState = m_initializeFromCurrentState && m_InitialStaticStepId > 0 &&
                                         staticStep != m_pData->m_AnalysisStep.end() && staticStep->second &&
                                         staticStep->second->m_Type == EnumKeyword::StepType::STATIC;
    for (const auto& [constraintId, constraint] : m_pData->m_Constraint)
    {
        if (!constraint || !IsStepScopedDataActive(constraint->m_StepId))
            continue;
        const auto node = constraint->m_pNode.lock();
        if (!node)
        {
            errorMessage = QStringLiteral("动力位移约束 %1 的节点引用无效。").arg(constraintId);
            return false;
        }
        const int direction = static_cast<int>(constraint->m_Direction);
        const auto dofs = effectiveDofs.find(node.get());
        if (direction < 0 || direction >= 6 || dofs == effectiveDofs.end() || direction >= dofs->second ||
            (m_initializeFromCurrentState && static_cast<std::size_t>(direction) >= node->m_Displacement.size()))
        {
            errorMessage = QStringLiteral("动力位移约束 %1 的方向无效。").arg(constraintId);
            return false;
        }

        const bool inheritedConstraint = hasInheritedStaticState && constraint->m_StepId <= m_InitialStaticStepId;
        // 历史静力时程已在找形阶段走完；动力阶段冻结该静力终点，不重放局部 t=0 的加载表。
        if (!inheritedConstraint && !constraint->IsConstantOverInterval(0.0, m_Time))
        {
            errorMessage =
                QStringLiteral("动力求解尚不支持随时间变化的规定位移/支座运动（规定边界 %1）。").arg(constraintId);
            return false;
        }
        const double referenceTime = inheritedConstraint ? staticStep->second->m_Time : 0.0;
        const double prescribedValue = constraint->m_Value == 0.0 ? 0.0 : constraint->GetValue(referenceTime, 1.0);
        const double currentValue =
            m_initializeFromCurrentState ? node->m_Displacement[static_cast<std::size_t>(direction)] : 0.0;
        const double tolerance = 64.0 * std::numeric_limits<double>::epsilon() *
                                 std::max({1.0, std::abs(currentValue), std::abs(prescribedValue)});
        if (!std::isfinite(currentValue) || !std::isfinite(prescribedValue) ||
            std::abs(currentValue - prescribedValue) > tolerance)
        {
            errorMessage = QStringLiteral("动力规定边界与初态不兼容（约束 %1）；"
                                          "当前动力求解尚不支持规定位移跳变/支座运动。")
                               .arg(constraintId);
            return false;
        }
    }
    return true;
}

void AnalysisStep::Init()
{
    InvalidateLocalRayleighCache();
    m_localRayleighCacheGroups.clear();
    m_localRayleighGroupLabels.clear();
    m_localRayleighElementGroups.clear();
    m_localRayleighInertiaGroups.clear();
    m_referenceNormSignedDofs.clear();
    ++m_referenceRhsStateVersion;
    m_referenceLoadStateVersion = 0;
    m_referenceOrderedResidual.resize(0);
    if (!PrepareData())
        return;
    const bool referenceStatic = m_Type == EnumKeyword::StepType::STATIC && m_referenceGravityEquilibrium;
    const bool referenceDynamic = m_Type == EnumKeyword::StepType::DYNAMIC &&
                                  m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (const auto cable = std::dynamic_pointer_cast<ElementCable>(element))
            cable->SetReferenceTssbnConstitutiveMode(referenceStatic || referenceDynamic, referenceStatic);
        if (const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(element))
        {
            beam->SetReferenceTssbnConstitutiveMode(referenceStatic || referenceDynamic, referenceStatic);
            beam->SetGaussHistoryDynamicMode(m_Type == EnumKeyword::StepType::DYNAMIC);
            if (m_Type != EnumKeyword::StepType::DYNAMIC || !m_initializeFromCurrentState)
                beam->ResetGaussRotationHistory();
        }
    }
    m_localRayleighDampingReady = false;
    Init_DOF();
    Init_Nodevector();
    if (referenceStatic || referenceDynamic)
    {
        QString reason;
        const bool mapped = PrepareReferenceVectorNormMapping(reason);
        if (m_DiagnosticTssbnErrorTrace)
            QTextStream(stdout) << "YQY_REFERENCE_LAYOUT mapped=" << mapped << " fixed=" << m_nFixed
                                << " free=" << m_nFree << " entries=" << m_referenceNormSignedDofs.size()
                                << " reason=" << reason << Qt::endl;
        Logger::Instance().Diagnostic(
            mapped ? QStringLiteral("参考向量范数：按完整自由度与 [ux,uz,-uy,theta] 顺序归约。")
                   : QStringLiteral("参考向量范数：使用原顺序，当前模型未匹配参考排列：%1。").arg(reason));
    }
}

bool AnalysisStep::PrepareReferenceVectorNormMapping(_OUT QString& reason)
{
    reason.clear();
    if (!m_pData || m_nFixed < 0 || m_nFree < 0 || m_nFixed + m_nFree == 0)
    {
        reason = QStringLiteral("自由度未初始化");
        return false;
    }
    if (!GetModelUpDirection().isApprox(Eigen::Vector3d::UnitZ(), 1.0e-12) || HasActiveNonlinearMPC())
    {
        reason = QStringLiteral("需要全局 Z 竖直方向且不含非线性 MPC");
        return false;
    }
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        Q_UNUSED(loadId);
        const auto wind = std::dynamic_pointer_cast<Force_Wind>(load);
        if (wind && IsStepScopedDataActive(wind->m_StepId) &&
            !wind->m_direction.normalized().isApprox(-Eigen::Vector3d::UnitY(), 1.0e-12))
        {
            reason = QStringLiteral("需要全局 -Y 风向");
            return false;
        }
    }
    for (const auto& [constraintId, constraint] : m_pData->m_Constraint)
    {
        Q_UNUSED(constraintId);
        if (constraint && IsStepScopedDataActive(constraint->m_StepId) &&
            (constraint->m_Value != 0.0 || constraint->HasTimePoints()))
        {
            reason = QStringLiteral("需要恒零位移约束");
            return false;
        }
    }

    const int totalDofs = m_nFixed + m_nFree;
    std::vector<int> signedDofs(static_cast<std::size_t>(totalDofs), 0);
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        if (!node || node->m_DOF.size() != 4)
        {
            reason = QStringLiteral("需要原生四自由度节点");
            return false;
        }
        int fixedCount = 0;
        for (const int dof : node->m_DOF)
        {
            if (dof < 0 || dof >= totalDofs)
            {
                reason = QStringLiteral("存在无效自由度编号");
                return false;
            }
            fixedCount += dof < m_nFixed;
        }
        if (fixedCount != 0 && fixedCount != 4)
        {
            reason = QStringLiteral("固定支座必须约束整节点四个自由度");
            return false;
        }
        // 正负号编码轴映射；加一保留零值作为未初始化标志。
        signedDofs[node->m_DOF[0]] = node->m_DOF[0] + 1;
        signedDofs[node->m_DOF[1]] = node->m_DOF[2] + 1;
        signedDofs[node->m_DOF[2]] = -(node->m_DOF[1] + 1);
        signedDofs[node->m_DOF[3]] = node->m_DOF[3] + 1;
    }
    if (std::find(signedDofs.cbegin(), signedDofs.cend(), 0) != signedDofs.cend())
    {
        reason = QStringLiteral("自由度布局不完整");
        return false;
    }
    m_referenceNormSignedDofs = std::move(signedDofs);
    return true;
}

void AnalysisStep::Init_DOF()
{
    InvalidateLocalRayleighCache();
    m_assemblyPattern = {};
    m_preparedPatternStamps.clear();
    m_elementBatchPlan = {};
    m_assemblyRangesByWorkerCount.clear();
    m_gpuAssemblyElementMask.clear();
    m_gpuTrussBatchElementIndices.clear();
    m_gpuTrussBatchElements.clear();
    m_gpuTrussBatchFirstNodes.clear();
    m_gpuTrussBatchSecondNodes.clear();
    m_gpuTrussBatchInput.clear();
    m_gpuTrussBatchAssemblyMaps.clear();
    m_gpuCableBatchSourceElementIndices.clear();
    m_gpuCableBatchElementIndices.clear();
    m_gpuCableBatchElements.clear();
    m_gpuCableBatchFirstNodes.clear();
    m_gpuCableBatchSecondNodes.clear();
    m_gpuCableBatchInput.clear();
    m_gpuCableBatchAssemblyMaps.clear();
    m_gpuSharedCableBatchSourceElementIndices.clear();
    m_gpuSharedCableBatchElementIndices.clear();
    m_gpuSharedCableBatchElements.clear();
    m_gpuSharedCableBatchFirstNodes.clear();
    m_gpuSharedCableBatchSecondNodes.clear();
    m_gpuSharedCableBatchInput.clear();
    m_gpuSharedCableBatchAssemblyMaps.clear();
    m_gpuBeamBatchElementIndices.clear();
    m_gpuBeamBatchElements.clear();
    m_gpuBeamBatchFirstNodes.clear();
    m_gpuBeamBatchSecondNodes.clear();
    m_gpuBeamBatchInput.clear();
    m_gpuBeamBatchAssemblyMaps.clear();
    // m_pData 已由 PrepareData() 准备好

    // 重置所有节点的DOF标记，确保重新计算
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto& pNode = nodePair.second;
        std::fill(pNode->m_DOF.begin(), pNode->m_DOF.end(), -1);
    }

    // 根据单元节点自由度初始化节点DOF数组

    for (auto& elements : m_pData->m_Elements)
    {
        auto pelement = elements.second;
        int NodeDOF = pelement->Get_NodeDOF();
        for (auto& nodes : pelement->m_pNode)
        {
            auto pNode = nodes.lock();
            if (pNode && pNode->m_DOF.size() < NodeDOF)
            {
                pNode->m_DOF.resize(NodeDOF, -1);
            }
        }
    }

    int iStart = 0;

    // 先处理约束自由度
    for (auto& constraints : m_pData->m_Constraint)
    {
        auto pConstrain = constraints.second;
        if (!pConstrain || !IsStepScopedDataActive(pConstrain->m_StepId))
            continue;
        auto pNode = pConstrain->m_pNode.lock();
        if (!pNode)
            continue;

        int iDirection = static_cast<int>(pConstrain->m_Direction);
        if (iDirection < 0 || iDirection >= pNode->m_DOF.size())
            continue;

        if (-1 == pNode->m_DOF[iDirection])
        {
            pNode->m_DOF[iDirection] = iStart++;
        }
    }

    m_nFixed = iStart;

    // 再处理自由自由度
    for (auto& nodes : m_pData->m_Nodes)
    {
        auto pNode = nodes.second;
        for (auto& dofValue : pNode->m_DOF)
        {
            if (-1 == dofValue)
                dofValue = iStart++;
        }
    }

    m_nFree = iStart - m_nFixed;
}

void AnalysisStep::Init_Nodevector()
{
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();
        if (m_initializeFromCurrentState)
        {
            // 静力找形后的位移和有限转动就是动力步的 t=0 构型。
            // 速度与加速度不从静力伪时间继承，默认从静止平衡状态启动。
            pNode->m_Displacement.resize(numDOF, 0.0);
            pNode->m_Displacement_n = pNode->m_Displacement;
            pNode->m_Rg_n = pNode->m_Rg;
        }
        else
        {
            // 独立求解从原始模型几何开始，避免重复计算继承上一次结果。
            pNode->m_Displacement.assign(numDOF, 0.0);
            pNode->m_Displacement_n.assign(numDOF, 0.0);
            pNode->m_Rg.setIdentity();
            pNode->m_Rg_n.setIdentity();
        }
        pNode->m_Velocity.assign(numDOF, 0.0);
        pNode->m_Velocity_n.assign(numDOF, 0.0);
        pNode->m_Acceleration.assign(numDOF, 0.0);
        pNode->m_Acceleration_n.assign(numDOF, 0.0);
        pNode->m_Force.assign(numDOF, 0.0);
        pNode->m_ReactionForce.assign(numDOF, 0.0);
        pNode->m_OmegaMaterial.setZero();
        pNode->m_AlphaMaterial.setZero();
        pNode->m_OmegaMaterial_n.setZero();
        pNode->m_AlphaMaterial_n.setZero();
        pNode->m_StepTranslation.setZero();
        pNode->m_StepScalarRotation = 0.0;
        pNode->m_StepRotation.setZero();
        pNode->m_TssbnImplicitStage = Node::TssbnImplicitStageState{};
    }
    m_initializeFromCurrentState = false;
}

bool AnalysisStep::ValidateGallopingConfiguration(QString* errorMessage) const
{
    if ((m_GallopingInitialVerticalPerturbationWaves != 1 &&
         m_GallopingInitialVerticalPerturbationWaves != 2) ||
        (m_GallopingInitialVerticalPerturbationWaves == 2 &&
         (!m_EnableGalloping || m_Type != EnumKeyword::StepType::DYNAMIC || m_InitialStaticStepId <= 0)))
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("竖向初始扰动的空间半波数必须为 1 或 2，且双半波只适用于继承静力平衡的舞动动力步。");
        return false;
    }
    if (!std::isfinite(m_GallopingInitialVerticalPerturbationMeters) ||
        (m_GallopingInitialVerticalPerturbationMeters != 0.0 &&
         (!m_EnableGalloping || m_Type != EnumKeyword::StepType::DYNAMIC || m_InitialStaticStepId <= 0)))
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("竖向初始扰动必须为有限值，且只适用于继承静力平衡的舞动动力步。");
        return false;
    }
    if (!m_EnableGalloping)
        return true;
    const Force_Wind* selectedWind = nullptr;
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        const auto wind = std::dynamic_pointer_cast<Force_Wind>(load);
        if (!wind || !IsStepScopedDataActive(wind->m_StepId))
            continue;
        if (selectedWind)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("舞动分析步只能继承一个风荷载，请合并或删除重复风荷载。");
            return false;
        }
        selectedWind = wind.get();
    }
    if (!selectedWind)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("启用舞动前必须给分析步配置风荷载。");
        return false;
    }
    if (!selectedWind->m_direction.allFinite() || selectedWind->m_direction.norm() <= 1.0e-12)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("舞动风荷载必须提供非零的全局风向向量。");
        return false;
    }
    if (!std::isfinite(m_GallopingInitialAttackDegrees))
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("初始风攻角必须是有限数值。");
        return false;
    }
    if (selectedWind->m_FunctionType != TimeFunctionType::CONSTANT &&
        selectedWind->m_FunctionType != TimeFunctionType::RAMP)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("当前离散气动参数只支持恒定风速，或在固定气动表下斜坡加载气动力。");
        return false;
    }
    if (m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN)
    {
        if (!std::isfinite(selectedWind->m_velocity) || selectedWind->m_velocity <= 0.0)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("参考 TSSBN 舞动的风速必须有限且大于 0。");
            return false;
        }
        const Eigen::Vector3d up = GetModelUpDirection().normalized();
        if (std::abs(selectedWind->m_direction.normalized().dot(up)) > 1.0e-6)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("参考 TSSBN 固定升阻力方向要求风向与模型竖向垂直。");
            return false;
        }
    }
    const double roundedSpeed = std::llround(selectedWind->m_velocity);
    if (std::abs(selectedWind->m_velocity - roundedSpeed) > 1.0e-9 ||
        !AeroManager::isSupportedWindSpeed(static_cast<int>(roundedSpeed)))
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("风速必须为 10、12、14 或 18 m/s。");
        return false;
    }
    return true;
}

bool AnalysisStep::PrepareGallopingData(QString* errorMessage)
{
    if (!m_EnableGalloping)
        return true;

    std::shared_ptr<Force_Wind> wind;
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        const auto candidate = std::dynamic_pointer_cast<Force_Wind>(load);
        if (candidate && IsStepScopedDataActive(candidate->m_StepId))
        {
            wind = candidate;
            break;
        }
    }
    if (!wind)
        return false;

    std::set<int> bundleCounts;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad())
            continue;
        if (element->m_AeroBundleCount <= 0 || element->m_WireId < 0)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("气动单元 %1 缺少分裂数或子导线编号。").arg(elementId);
            return false;
        }
        bundleCounts.insert(element->m_AeroBundleCount);
    }
    if (bundleCounts.empty())
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("舞动已启用，但模型没有读取到气动单元数据。");
        return false;
    }

    const std::filesystem::path dataDirectories[] = {
        std::filesystem::path(ApplicationPaths::aerodynamicDataDirectory().toStdWString()),
        std::filesystem::path("YQY/Import/Aero_Data/Output_Data"),
        std::filesystem::path("Import/Aero_Data/Output_Data")};
    for (int bundleCount : bundleCounts)
    {
        const AeroCaseKey key = GetGallopingAeroCase(bundleCount, *wind);
        if (!AeroManager::isSupportedCase(key))
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("当前模型包含 %1 分裂导线，但没有对应的气动参数工况。").arg(bundleCount);
            return false;
        }
        if (m_pData->m_AeroManager.hasCase(key))
            continue;

        bool loaded = false;
        for (const auto& directory : dataDirectories)
        {
            if (directory.empty())
                continue;
            if (m_pData->m_AeroManager.loadCase(directory, key))
            {
                loaded = true;
                break;
            }
        }
        if (!loaded)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("无法加载气动参数：%1 分裂，%2 m/s，覆冰 %3 mm。")
                                    .arg(key.bundleCount)
                                    .arg(key.windSpeed)
                                    .arg(key.iceThickness);
            return false;
        }
    }
    return true;
}

bool AnalysisStep::BindGallopingProfiles(QString* errorMessage)
{
    m_gallopingProfileBindings.clear();
    if (!m_EnableGalloping)
        return true;
    if (m_GallopingProfileBindingMode != GallopingProfileBindingMode::AutoGeometry &&
        m_GallopingProfileBindingMode != GallopingProfileBindingMode::ExplicitElementProfile)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("未知的舞动气动剖面绑定模式。");
        return false;
    }
    const bool explicitProfiles =
        m_GallopingProfileBindingMode == GallopingProfileBindingMode::ExplicitElementProfile;

    std::shared_ptr<Force_Wind> wind;
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        Q_UNUSED(loadId);
        const auto candidate = std::dynamic_pointer_cast<Force_Wind>(load);
        if (candidate && IsStepScopedDataActive(candidate->m_StepId))
        {
            wind = candidate;
            break;
        }
    }
    if (!wind)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("舞动气动剖面绑定需要有效的风荷载。");
        return false;
    }

    const Eigen::Vector3d modelUp = GetModelUpDirection();
    const Eigen::Vector3d windVelocity = wind->GetWindVelocityGlobal();
    const auto referencePosition = [](const Node& node)
    {
        return Eigen::Vector3d(node.m_X, node.m_Y, node.m_Z);
    };
    const auto currentPosition = [](const Node& node)
    {
        Eigen::Vector3d position(node.m_X, node.m_Y, node.m_Z);
        for (int component = 0; component < 3 && component < static_cast<int>(node.m_Displacement.size()); ++component)
            position[component] += node.m_Displacement[component];
        return position;
    };

    // AERO_PROFILE_ID 兼作气动参与标记。显式模式先检查全部导线单元，
    // 避免负编号被 HasAerodynamicLoad() 悄悄跳过。
    if (explicitProfiles)
    {
        for (const auto& [elementId, element] : m_pData->m_Elements)
        {
            if (!element || element->m_AeroBundleCount <= 0)
                continue;
            const int bundleCount = element->m_AeroBundleCount;
            if ((bundleCount != 1 && bundleCount != 4) || element->m_WireId < 0 ||
                element->m_WireId >= bundleCount || element->m_AeroProfileId < 0 ||
                element->m_AeroProfileId >= bundleCount || element->m_pNode.size() != 2)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("气动导线单元 %1 的分裂数、WIRE_ID、AERO_PROFILE_ID "
                                                   "或端节点数量无效。")
                                        .arg(elementId);
                return false;
            }
        }
    }

    struct WireBindingSummary
    {
        int elementCount = 0;
        int declaredProfile = -1;
        int boundProfile = -1;
        bool declaredMixed = false;
        bool boundMixed = false;
        double windProjectionSum = 0.0;
        bool hasMassFirstMoment = false;
        Eigen::Vector3d massFirstMoment = Eigen::Vector3d::Zero();
    };
    std::map<int, WireBindingSummary> quadWires;
    if (!windVelocity.allFinite() || windVelocity.norm() <= 1.0e-12)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("舞动气动剖面绑定需要有限且非零的风速向量。");
        return false;
    }
    const Eigen::Vector3d windUnit = windVelocity.normalized();
    Eigen::Vector3d projectionWindUnit = windUnit;

    std::array<int, 4> quadProfilesByWire{-1, -1, -1, -1};
    if (!explicitProfiles)
    {
        // 按初始几何注册四根物理子导线，前置静力风平衡的位移与扭转不改变 WIRE_ID 对应的实测 P 列。
        // 四根子导线网格不同时，仍在同一跨向位置插值。
        using WireSegment = std::array<Eigen::Vector3d, 2>;
        std::array<std::vector<WireSegment>, 4> segmentsByWire;
        Eigen::Vector3d referenceChord = Eigen::Vector3d::Zero();
        bool hasQuadElements = false;
        for (const auto& [elementId, element] : m_pData->m_Elements)
        {
            if (!element || !element->HasAerodynamicLoad() || element->m_AeroBundleCount != 4)
                continue;
            hasQuadElements = true;
            if (element->m_WireId < 0 || element->m_WireId >= 4 || element->m_pNode.size() != 2)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("四分裂自动绑定：气动单元 %1 的 WIRE_ID 或端节点数量无效。")
                                        .arg(elementId);
                return false;
            }
            const auto first = element->m_pNode[0].lock();
            const auto second = element->m_pNode[1].lock();
            if (!first || !second)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("四分裂自动绑定：气动单元 %1 的端节点无效。")
                                        .arg(elementId);
                return false;
            }
            const WireSegment segment{referencePosition(*first), referencePosition(*second)};
            if (referenceChord.squaredNorm() == 0.0)
                referenceChord = segment[1] - segment[0];
            segmentsByWire[element->m_WireId].push_back(segment);
        }
        if (hasQuadElements)
        {
            QString conventionError;
            if (!VerifyBuiltInQuadProfileConvention(m_pData->m_AeroManager, GetGallopingAeroCase(4, *wind),
                                                     conventionError))
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("四分裂自动绑定无法确认 P1–P4 风洞列方位：%1。"
                                                   "请使用按单元 AERO_PROFILE_ID 显式绑定并核对迎风、背风关系。")
                                        .arg(conventionError);
                return false;
            }
            for (int wire = 0; wire < 4; ++wire)
            {
                if (segmentsByWire[wire].empty())
                {
                    if (errorMessage)
                        *errorMessage = QStringLiteral("四分裂自动绑定需要 WIRE_ID=0..3 四根完整子导线；缺少 WIRE_ID=%1。")
                                            .arg(wire);
                    return false;
                }
            }
            const Eigen::Vector3d upUnit = modelUp.normalized();
            Eigen::Vector3d spanAxis = referenceChord - referenceChord.dot(upUnit) * upUnit;
            if (!spanAxis.allFinite() || spanAxis.norm() <= 1.0e-12)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("四分裂自动绑定的水平跨向无法确定。");
                return false;
            }
            spanAxis.normalize();
            double commonLower = -std::numeric_limits<double>::infinity();
            double commonUpper = std::numeric_limits<double>::infinity();
            for (const auto& wireSegments : segmentsByWire)
            {
                double wireLower = std::numeric_limits<double>::infinity();
                double wireUpper = -std::numeric_limits<double>::infinity();
                for (const WireSegment& segment : wireSegments)
                {
                    for (const Eigen::Vector3d& end : segment)
                    {
                        const double station = end.dot(spanAxis);
                        wireLower = std::min(wireLower, station);
                        wireUpper = std::max(wireUpper, station);
                    }
                }
                commonLower = std::max(commonLower, wireLower);
                commonUpper = std::min(commonUpper, wireUpper);
            }
            if (!std::isfinite(commonLower) || !std::isfinite(commonUpper) ||
                commonUpper - commonLower <= 1.0e-12)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("四分裂自动绑定：四根子导线没有共同跨段。");
                return false;
            }
            const double stationTolerance = 1.0e-9 * (commonUpper - commonLower);
            // P 标签随物理子导线绑定；截面方向沿跨变化时，不能共用一个全局 WIRE_ID -> P 映射。
            // 绑定前检查五个共同截面。
            constexpr std::array<double, 5> stationFractions{0.1, 0.3, 0.5, 0.7, 0.9};
            bool firstSection = true;
            for (double stationFraction : stationFractions)
            {
                const double station = commonLower + stationFraction * (commonUpper - commonLower);
                std::array<Eigen::Vector3d, 4> wireCenters;
                for (int wire = 0; wire < 4; ++wire)
                {
                    bool found = false;
                    for (const WireSegment& segment : segmentsByWire[wire])
                    {
                        const double firstStation = segment[0].dot(spanAxis);
                        const double secondStation = segment[1].dot(spanAxis);
                        if (station < std::min(firstStation, secondStation) - stationTolerance ||
                            station > std::max(firstStation, secondStation) + stationTolerance)
                            continue;
                        const double distance = secondStation - firstStation;
                        if (std::abs(distance) <= 1.0e-12)
                            continue;
                        const double fraction = std::clamp((station - firstStation) / distance, 0.0, 1.0);
                        const Eigen::Vector3d center = segment[0] + fraction * (segment[1] - segment[0]);
                        if (found && (center - wireCenters[wire]).norm() > 1.0e-6)
                        {
                            if (errorMessage)
                                *errorMessage = QStringLiteral("四分裂自动绑定：WIRE_ID=%1 在跨向位置 %2 有重叠且"
                                                               "不一致的子导线；请按独立跨段建模或显式绑定。")
                                                    .arg(wire)
                                                    .arg(stationFraction, 0, 'g', 3);
                            return false;
                        }
                        wireCenters[wire] = center;
                        found = true;
                    }
                    if (!found)
                    {
                        if (errorMessage)
                            *errorMessage = QStringLiteral("四分裂自动绑定：WIRE_ID=%1 在跨向位置 %2 没有连续单元。")
                                                .arg(wire)
                                                .arg(stationFraction, 0, 'g', 3);
                        return false;
                    }
                }
                try
                {
                    const auto sectionProfiles = BundleAeroMapper::ResolveQuadProfiles(
                        wireCenters, referenceChord, modelUp, windVelocity, m_GallopingInitialAttackDegrees);
                    if (firstSection)
                    {
                        quadProfilesByWire = sectionProfiles;
                        firstSection = false;
                    }
                    else if (sectionProfiles != quadProfilesByWire)
                    {
                        if (errorMessage)
                            *errorMessage = QStringLiteral("四分裂自动绑定：沿跨束截面方位发生变化，不能对全跨使用同一"
                                                           " P1–P4 绑定；请检查束扭转或使用显式绑定。");
                        return false;
                    }
                }
                catch (const std::exception& exception)
                {
                    if (errorMessage)
                        *errorMessage = QStringLiteral("四分裂自动绑定在跨向位置 %1 失败：%2")
                                            .arg(stationFraction, 0, 'g', 3)
                                            .arg(QString::fromUtf8(exception.what()));
                    return false;
                }
            }
        }
    }

    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        if (!element || !element->HasAerodynamicLoad() || element->m_pNode.size() != 2)
            continue;
        const auto first = element->m_pNode[0].lock();
        const auto second = element->m_pNode[1].lock();
        if (!first || !second)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("气动单元 %1 的端节点无效。").arg(elementId);
            return false;
        }

        if (element->m_AeroBundleCount <= 0 || element->m_WireId < 0)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("气动单元 %1 缺少分裂数或子导线编号。").arg(elementId);
            return false;
        }
        const int bundleCount = element->m_AeroBundleCount;
        const int wireId = element->m_WireId;
        int profileId = -1;
        try
        {
            if (explicitProfiles)
            {
                if ((bundleCount != 1 && bundleCount != 4) || wireId >= bundleCount ||
                    element->m_AeroProfileId >= bundleCount)
                    throw std::invalid_argument("Explicit element profile ID is outside the aerodynamic case");
                profileId = element->m_AeroProfileId;
                if (bundleCount == 4)
                    BundleAeroMapper::ValidateAxisConvention(currentPosition(*second) - currentPosition(*first),
                                                             modelUp);
            }
            else if (bundleCount == 4)
            {
                if (wireId >= 4 || quadProfilesByWire[wireId] < 0)
                    throw std::invalid_argument("Four-wire profile was not uniquely registered");
                BundleAeroMapper::ValidateAxisConvention(currentPosition(*second) - currentPosition(*first),
                                                         modelUp);
                profileId = quadProfilesByWire[wireId];
            }
            else
            {
                profileId = BundleAeroMapper::ResolveProfile(bundleCount, wireId,
                                                              currentPosition(*second) - currentPosition(*first),
                                                              modelUp, windVelocity);
            }
            m_gallopingProfileBindings.emplace(elementId, profileId);
        }
        catch (const std::exception& exception)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("无法为气动单元 %1 绑定气动剖面：%2")
                                    .arg(elementId)
                                    .arg(QString::fromUtf8(exception.what()));
            return false;
        }
        if (bundleCount == 4)
        {
            if (quadWires.empty())
            {
                const Eigen::Vector3d axis =
                    (currentPosition(*second) - currentPosition(*first)).normalized();
                const Eigen::Vector3d normalWind = windVelocity - windVelocity.dot(axis) * axis;
                if (normalWind.norm() > 1.0e-12)
                    projectionWindUnit = normalWind.normalized();
            }
            auto& summary = quadWires[wireId];
            if (summary.elementCount > 0)
            {
                summary.declaredMixed |= summary.declaredProfile != element->m_AeroProfileId;
                summary.boundMixed |= summary.boundProfile != profileId;
            }
            else
            {
                summary.declaredProfile = element->m_AeroProfileId;
                summary.boundProfile = profileId;
                const auto property = element->m_pProperty.lock();
                const auto section = property ? property->m_pSection.lock() : nullptr;
                if (section)
                {
                    summary.hasMassFirstMoment = true;
                    summary.massFirstMoment = section->m_MassFirstMomentPerLength;
                }
            }
            ++summary.elementCount;
            summary.windProjectionSum +=
                projectionWindUnit.dot(0.5 * (currentPosition(*first) + currentPosition(*second)));
            if (explicitProfiles && summary.boundMixed)
            {
                if (errorMessage)
                    *errorMessage = QStringLiteral("四分裂子导线 WIRE_ID=%1 沿跨使用了不同的 AERO_PROFILE_ID。")
                                        .arg(wireId);
                return false;
            }
        }
    }

    if (!quadWires.empty() && quadWires.size() != 4)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("四分裂气动计算要求 WIRE_ID=0..3 四根子导线齐全；当前只有 %1 根。")
                                .arg(quadWires.size());
        return false;
    }

    if (explicitProfiles && !quadWires.empty())
    {
        std::set<int> usedProfiles;
        for (const auto& [wireId, summary] : quadWires)
            usedProfiles.insert(summary.boundProfile);
        if (usedProfiles.size() != 4)
        {
            if (errorMessage)
                *errorMessage = QStringLiteral("显式四分裂绑定要求四根子导线分别使用 P1..P4，不能重复列号。");
            return false;
        }
    }

    if (!quadWires.empty())
    {
        const auto reportBinding = [](const QString& message)
        {
            QTextStream(stdout) << "YQY_AERO_BINDING " << message << Qt::endl;
            qInfo().noquote() << message;
        };
        std::vector<std::pair<double, int>> upstreamOrder;
        upstreamOrder.reserve(quadWires.size());
        for (const auto& [wireId, summary] : quadWires)
            upstreamOrder.emplace_back(summary.windProjectionSum / summary.elementCount, wireId);
        std::sort(upstreamOrder.begin(), upstreamOrder.end());
        std::map<int, int> upstreamRank;
        for (std::size_t rank = 0; rank < upstreamOrder.size(); ++rank)
            upstreamRank.emplace(upstreamOrder[rank].second, static_cast<int>(rank) + 1);

        const QString bindingCaption =
            QStringLiteral("四分裂气动绑定：step=%1，模式=%2，风速向量=(%3,%4,%5) m/s，"
                           "气动表=%6 m/s/%7 mm，初始攻角=%8°；迎风投影越小越靠上游。");
        reportBinding(bindingCaption
                                 .arg(m_Id)
                                 .arg(explicitProfiles ? QStringLiteral("显式单元编号") : QStringLiteral("风向几何自动"))
                                 .arg(windVelocity.x(), 0, 'g', 8)
                                 .arg(windVelocity.y(), 0, 'g', 8)
                                 .arg(windVelocity.z(), 0, 'g', 8)
                                 .arg(wind->m_velocity, 0, 'g', 8)
                                 .arg(m_GallopingIceThickness)
                                 .arg(m_GallopingInitialAttackDegrees, 0, 'g', 8));
        for (const auto& [wireId, summary] : quadWires)
        {
            const AeroCoefficients nominalCoefficients = m_pData->m_AeroManager.getCoefficients(
                GetGallopingAeroCase(4, *wind), summary.boundProfile, m_GallopingInitialAttackDegrees);
            const QString declared = summary.declaredMixed
                                         ? QStringLiteral("沿跨不一致")
                                         : QStringLiteral("%1(P%2)")
                                               .arg(summary.declaredProfile)
                                               .arg(summary.declaredProfile + 1);
            const QString bound = summary.boundMixed
                                      ? QStringLiteral("沿跨不一致")
                                      : QStringLiteral("%1(P%2)")
                                            .arg(summary.boundProfile)
                                            .arg(summary.boundProfile + 1);
            const QString profileCaption =
                QStringLiteral("  WIRE_ID=%1，索单元=%2，迎风投影=%3 m，上游排序=%4/%5，"
                               "AERO_PROFILE_ID=%6，实际剖面=%7，α0表值 CL/CD/CM=(%8,%9,%10)");
            reportBinding(profileCaption
                                     .arg(wireId)
                                     .arg(summary.elementCount)
                                     .arg(summary.windProjectionSum / summary.elementCount, 0, 'g', 8)
                                     .arg(upstreamRank.at(wireId))
                                     .arg(static_cast<int>(quadWires.size()))
                                     .arg(declared)
                                     .arg(bound)
                                     .arg(nominalCoefficients.lift, 0, 'g', 8)
                                     .arg(nominalCoefficients.drag, 0, 'g', 8)
                                     .arg(nominalCoefficients.moment, 0, 'g', 8));
            if (summary.hasMassFirstMoment)
            {
                const double momentNorm = summary.massFirstMoment.norm();
                Eigen::Vector3d direction = Eigen::Vector3d::Zero();
                if (momentNorm > 1.0e-12)
                    direction = summary.massFirstMoment / momentNorm;
                reportBinding(QStringLiteral("    首单元零扭转截面质量一次矩=(%1,%2,%3) kg，"
                                                   "截面局部偏心方向=(%4,%5,%6)，α0=%7°")
                                         .arg(summary.massFirstMoment.x(), 0, 'g', 8)
                                         .arg(summary.massFirstMoment.y(), 0, 'g', 8)
                                         .arg(summary.massFirstMoment.z(), 0, 'g', 8)
                                         .arg(direction.x(), 0, 'g', 6)
                                         .arg(direction.y(), 0, 'g', 6)
                                         .arg(direction.z(), 0, 'g', 6)
                                         .arg(m_GallopingInitialAttackDegrees, 0, 'g', 8));
            }
            else
            {
                reportBinding(QStringLiteral("    首单元截面质量一次矩不可用；α0=%1°")
                                  .arg(m_GallopingInitialAttackDegrees, 0, 'g', 8));
            }
        }
    }

    if (m_gallopingProfileBindings.empty())
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("舞动已启用，但没有找到可绑定的气动导线单元。");
        return false;
    }
    return true;
}

Eigen::Vector3d AnalysisStep::GetModelUpDirection() const
{
    if (!m_pData)
        return Eigen::Vector3d::UnitZ();
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        const auto gravity = std::dynamic_pointer_cast<Force_Gravity>(load);
        if (!gravity || !IsStepScopedDataActive(gravity->m_StepId))
            continue;
        const int component = static_cast<int>(gravity->m_Direction);
        if (component >= 0 && component < 3 && std::abs(gravity->m_g) > 1.0e-12)
        {
            Eigen::Vector3d gravityVector = Eigen::Vector3d::Zero();
            gravityVector[component] = gravity->m_g;
            return -gravityVector.normalized();
        }
    }
    return Eigen::Vector3d::UnitZ();
}

void AnalysisStep::Get_ElementLength()
{
    for (auto& ele : m_pData->m_Elements)
    {
        auto pElement = ele.second;
        pElement->Get_L0();
    }
}

void AnalysisStep::AssembleFreeFree(const std::vector<int>& dofs, const Eigen::MatrixXd& elementMatrix,
                                    _OUT std::vector<Tri>& freeFree)
{
    const int elementDofCount = static_cast<int>(dofs.size());

    for (int i = 0; i < elementDofCount; ++i)
    {
        const int globalRow = dofs[i];
        if (globalRow < m_nFixed || globalRow >= m_nFixed + m_nFree)
        {
            continue;
        }
        for (int j = 0; j < elementDofCount; ++j)
        {
            const int globalColumn = dofs[j];
            if (globalColumn >= m_nFixed && globalColumn < m_nFixed + m_nFree)
            {
                freeFree.emplace_back(globalRow - m_nFixed, globalColumn - m_nFixed, elementMatrix(i, j));
            }
        }
    }
}

void AnalysisStep::EnsureAssemblyPattern()
{
    if (m_assemblyPattern.valid)
        return;

    m_assemblyPattern = {};
    m_preparedPatternStamps.clear();
    m_elementBatchPlan = {};
    m_assemblyRangesByWorkerCount.clear();
    m_gpuAssemblyElementMask.clear();
    m_gpuSharedCableBatchSourceElementIndices.clear();
    std::vector<Tri> patternEntries;
    patternEntries.reserve((m_pData->m_Elements.size() + m_pData->m_RigidBodyInertias.size()) * 36);
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (!element)
            continue;
        ElementAssemblyMap assemblyMap;
        assemblyMap.element = element;
        element->GetDOFs(assemblyMap.dofs);
        const int dofCount = static_cast<int>(assemblyMap.dofs.size());
        m_assemblyPattern.matrixEntryCapacity += static_cast<std::size_t>(dofCount * dofCount);
        assemblyMap.valueIndices.assign(static_cast<std::size_t>(dofCount * dofCount), -1);
        for (int localColumn = 0; localColumn < dofCount; ++localColumn)
        {
            const int globalColumn = assemblyMap.dofs[localColumn] - m_nFixed;
            if (globalColumn < 0 || globalColumn >= m_nFree)
                continue;
            for (int localRow = 0; localRow < dofCount; ++localRow)
            {
                const int globalRow = assemblyMap.dofs[localRow] - m_nFixed;
                if (globalRow >= 0 && globalRow < m_nFree)
                    patternEntries.emplace_back(globalRow, globalColumn, 1.0);
            }
        }
        m_assemblyPattern.elements.push_back(std::move(assemblyMap));
    }
    for (const auto& [inertiaId, inertia] : m_pData->m_RigidBodyInertias)
    {
        Q_UNUSED(inertiaId);
        if (!inertia)
            continue;
        RigidBodyInertiaAssemblyMap assemblyMap;
        assemblyMap.inertia = inertia;
        inertia->GetDOFs(assemblyMap.dofs);
        const int dofCount = static_cast<int>(assemblyMap.dofs.size());
        m_assemblyPattern.matrixEntryCapacity += static_cast<std::size_t>(dofCount * dofCount);
        assemblyMap.valueIndices.assign(static_cast<std::size_t>(dofCount * dofCount), -1);
        for (int localColumn = 0; localColumn < dofCount; ++localColumn)
        {
            const int globalColumn = assemblyMap.dofs[localColumn] - m_nFixed;
            if (globalColumn < 0 || globalColumn >= m_nFree)
                continue;
            for (int localRow = 0; localRow < dofCount; ++localRow)
            {
                const int globalRow = assemblyMap.dofs[localRow] - m_nFixed;
                if (globalRow >= 0 && globalRow < m_nFree)
                    patternEntries.emplace_back(globalRow, globalColumn, 1.0);
            }
        }
        m_assemblyPattern.rigidBodyInertias.push_back(std::move(assemblyMap));
    }

    m_assemblyPattern.matrix.resize(m_nFree, m_nFree);
    m_assemblyPattern.matrix.setFromTriplets(patternEntries.begin(), patternEntries.end());
    m_assemblyPattern.matrix.makeCompressed();

    const auto* outer = m_assemblyPattern.matrix.outerIndexPtr();
    const auto* inner = m_assemblyPattern.matrix.innerIndexPtr();
    for (ElementAssemblyMap& assemblyMap : m_assemblyPattern.elements)
    {
        const int dofCount = static_cast<int>(assemblyMap.dofs.size());
        for (int localColumn = 0; localColumn < dofCount; ++localColumn)
        {
            const int globalColumn = assemblyMap.dofs[localColumn] - m_nFixed;
            if (globalColumn < 0 || globalColumn >= m_nFree)
                continue;
            const auto* first = inner + outer[globalColumn];
            const auto* last = inner + outer[globalColumn + 1];
            for (int localRow = 0; localRow < dofCount; ++localRow)
            {
                const int globalRow = assemblyMap.dofs[localRow] - m_nFixed;
                if (globalRow < 0 || globalRow >= m_nFree)
                    continue;
                const auto* position = std::lower_bound(first, last, globalRow);
                if (position != last && *position == globalRow)
                {
                    const std::size_t localIndex = static_cast<std::size_t>(localRow * dofCount + localColumn);
                    assemblyMap.valueIndices[localIndex] = position - inner;
                }
            }
        }
    }
    for (RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        const int dofCount = static_cast<int>(assemblyMap.dofs.size());
        for (int localColumn = 0; localColumn < dofCount; ++localColumn)
        {
            const int globalColumn = assemblyMap.dofs[localColumn] - m_nFixed;
            if (globalColumn < 0 || globalColumn >= m_nFree)
                continue;
            const auto* first = inner + outer[globalColumn];
            const auto* last = inner + outer[globalColumn + 1];
            for (int localRow = 0; localRow < dofCount; ++localRow)
            {
                const int globalRow = assemblyMap.dofs[localRow] - m_nFixed;
                if (globalRow < 0 || globalRow >= m_nFree)
                    continue;
                const auto* position = std::lower_bound(first, last, globalRow);
                if (position != last && *position == globalRow)
                {
                    const std::size_t localIndex = static_cast<std::size_t>(localRow * dofCount + localColumn);
                    assemblyMap.valueIndices[localIndex] = position - inner;
                }
            }
        }
    }
    std::fill(m_assemblyPattern.matrix.valuePtr(),
              m_assemblyPattern.matrix.valuePtr() + m_assemblyPattern.matrix.nonZeros(), 0.0);
    BuildAssemblyColorGroups();
    BuildElementBatchPlan();
    m_assemblyPattern.valid = true;
    ++m_sparsePatternVersion;
    if (m_sparsePatternVersion == 0)
        ++m_sparsePatternVersion;
}

void AnalysisStep::PreparePatternMatrix(_OUT SpMat& matrix) const
{
    const SpMat& pattern = m_assemblyPattern.matrix;
    const auto cached = m_preparedPatternStamps.find(&matrix);
    bool samePattern = false;
    if (cached != m_preparedPatternStamps.cend())
    {
        const PreparedPatternStamp& stamp = cached->second;
        samePattern = stamp.version == m_sparsePatternVersion && matrix.rows() == stamp.rows &&
                      matrix.cols() == stamp.columns && matrix.nonZeros() == stamp.nonZeros && matrix.isCompressed() &&
                      matrix.outerIndexPtr() == stamp.outerIndices && matrix.innerIndexPtr() == stamp.innerIndices &&
                      matrix.valuePtr() == stamp.values;
    }
    if (!samePattern)
    {
        samePattern = matrix.rows() == pattern.rows() && matrix.cols() == pattern.cols() &&
                      matrix.nonZeros() == pattern.nonZeros() && matrix.isCompressed();
        if (samePattern)
        {
            samePattern =
                std::equal(pattern.outerIndexPtr(), pattern.outerIndexPtr() + pattern.outerSize() + 1,
                           matrix.outerIndexPtr()) &&
                std::equal(pattern.innerIndexPtr(), pattern.innerIndexPtr() + pattern.nonZeros(),
                           matrix.innerIndexPtr());
        }
    }
    if (!samePattern)
        matrix = pattern;
    std::fill(matrix.valuePtr(), matrix.valuePtr() + matrix.nonZeros(), 0.0);
    m_preparedPatternStamps[&matrix] = {matrix.outerIndexPtr(), matrix.innerIndexPtr(), matrix.valuePtr(),
                                        matrix.rows(),          matrix.cols(),       matrix.nonZeros(),
                                        m_sparsePatternVersion};
}

void AnalysisStep::BuildAssemblyColorGroups()
{
    m_assemblyPattern.elementColorGroups.clear();
    const std::size_t elementCount = m_assemblyPattern.elements.size();
    if (elementCount == 0)
        return;
    if (m_nFree <= 0)
    {
        std::vector<std::size_t>& group = m_assemblyPattern.elementColorGroups.emplace_back();
        group.resize(elementCount);
        std::iota(group.begin(), group.end(), 0);
        return;
    }

    std::vector<std::vector<std::size_t>> elementsByDof(static_cast<std::size_t>(m_nFree));
    for (std::size_t elementIndex = 0; elementIndex < elementCount; ++elementIndex)
    {
        std::set<int> freeDofs;
        for (const int dof : m_assemblyPattern.elements[elementIndex].dofs)
        {
            const int freeDof = dof - m_nFixed;
            if (freeDof >= 0 && freeDof < m_nFree)
                freeDofs.insert(freeDof);
        }
        for (const int freeDof : freeDofs)
            elementsByDof[static_cast<std::size_t>(freeDof)].push_back(elementIndex);
    }

    std::vector<int> elementColors(elementCount, -1);
    std::vector<int> colorMarks;
    int mark = 0;
    for (std::size_t elementIndex = 0; elementIndex < elementCount; ++elementIndex)
    {
        ++mark;
        std::set<int> freeDofs;
        for (const int dof : m_assemblyPattern.elements[elementIndex].dofs)
        {
            const int freeDof = dof - m_nFixed;
            if (freeDof >= 0 && freeDof < m_nFree)
                freeDofs.insert(freeDof);
        }
        for (const int freeDof : freeDofs)
        {
            for (const std::size_t adjacent : elementsByDof[static_cast<std::size_t>(freeDof)])
            {
                if (adjacent >= elementIndex)
                    break;
                const int color = elementColors[adjacent];
                if (color >= 0)
                    colorMarks[static_cast<std::size_t>(color)] = mark;
            }
        }

        int color = 0;
        while (color < static_cast<int>(colorMarks.size()) && colorMarks[static_cast<std::size_t>(color)] == mark)
            ++color;
        if (color == static_cast<int>(colorMarks.size()))
        {
            colorMarks.push_back(0);
            m_assemblyPattern.elementColorGroups.emplace_back();
        }
        elementColors[elementIndex] = color;
        m_assemblyPattern.elementColorGroups[static_cast<std::size_t>(color)].push_back(elementIndex);
    }
}

void AnalysisStep::PrepareAssemblyWorkerBuffers(int workerCount, bool includeInertiaForce)
{
    const int appliedWorkerCount = std::max(1, workerCount);
    if (static_cast<int>(m_assemblyWorkerBuffers.size()) != appliedWorkerCount)
        m_assemblyWorkerBuffers.resize(static_cast<std::size_t>(appliedWorkerCount));

    const std::size_t valuesPerWorker =
        std::max<std::size_t>(1, (m_assemblyPattern.matrixEntryCapacity +
                                  static_cast<std::size_t>(appliedWorkerCount) - 1) /
                                     static_cast<std::size_t>(appliedWorkerCount));
    for (AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
    {
        buffer.matrixValues.clear();
        buffer.matrixValues.reserve(valuesPerWorker);
        if (!includeInertiaForce)
            continue;
        if (buffer.inertiaForce.size() != m_nFree)
            buffer.inertiaForce.resize(m_nFree);
        buffer.inertiaForce.setZero();
    }
}

void AnalysisStep::PrepareResidualWorkerBuffers(int workerCount)
{
    const int appliedWorkerCount = std::max(1, workerCount);
    if (static_cast<int>(m_assemblyWorkerBuffers.size()) != appliedWorkerCount)
        m_assemblyWorkerBuffers.resize(static_cast<std::size_t>(appliedWorkerCount));
    for (AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
    {
        if (buffer.inertiaForce.size() != m_nFree)
            buffer.inertiaForce.resize(m_nFree);
        buffer.inertiaForce.setZero();
    }
}

void AnalysisStep::PrepareDynamicAssemblyWorkerBuffers(int workerCount)
{
    const int appliedWorkerCount = std::max(1, workerCount);
    if (static_cast<int>(m_dynamicAssemblyWorkerBuffers.size()) != appliedWorkerCount)
        m_dynamicAssemblyWorkerBuffers.resize(static_cast<std::size_t>(appliedWorkerCount));

    const std::size_t expectedValueCount = m_assemblyPattern.matrixEntryCapacity;
    const std::size_t valuesPerWorker =
        std::max<std::size_t>(1, (expectedValueCount + static_cast<std::size_t>(appliedWorkerCount) - 1) /
                                     static_cast<std::size_t>(appliedWorkerCount));

    for (DynamicAssemblyWorkerBuffer& buffer : m_dynamicAssemblyWorkerBuffers)
    {
        buffer.massValues.clear();
        buffer.velocityValues.clear();
        buffer.configurationValues.clear();
        buffer.massValues.reserve(valuesPerWorker);
        buffer.velocityValues.reserve(valuesPerWorker);
        buffer.configurationValues.reserve(valuesPerWorker);
        if (buffer.inertiaForce.size() != m_nFree)
            buffer.inertiaForce.resize(m_nFree);
        buffer.inertiaForce.setZero();
    }
}

int AnalysisStep::SelectAssemblyWorkerCount() const
{
    constexpr std::size_t minimumAssemblyCostPerWorker = 50000;
    const std::size_t elementCount = m_assemblyPattern.elements.size();
    const std::size_t assemblyCost = m_assemblyPattern.matrixEntryCapacity;
    if (elementCount == 0 || assemblyCost < minimumAssemblyCostPerWorker)
        return 1;

    const int requestedThreads = m_assemblyThreadCountOverride > 0
                                     ? m_assemblyThreadCountOverride
                                     : SolverNameSpace::AssemblySettings::ThreadCount();
    const int workloadLimitedCount =
        std::max(1, static_cast<int>(assemblyCost / minimumAssemblyCostPerWorker));
    return std::max(1, std::min({requestedThreads, workloadLimitedCount, static_cast<int>(elementCount)}));
}

const std::vector<SolverNameSpace::ParallelWorkRange>& AnalysisStep::GetAssemblyRanges(int workerCount)
{
    const std::size_t elementCount = m_assemblyPattern.elements.size();
    const int appliedWorkerCount = std::max(1, std::min(workerCount, static_cast<int>(elementCount)));
    const auto cached = m_assemblyRangesByWorkerCount.find(appliedWorkerCount);
    if (cached != m_assemblyRangesByWorkerCount.end())
        return cached->second;

    QElapsedTimer timer;
    timer.start();
    std::vector<SolverNameSpace::ParallelWorkRange> ranges;
    if (elementCount == 0)
    {
        ranges.push_back({0, 0});
    }
    else if (appliedWorkerCount == 1)
    {
        ranges.push_back({0, elementCount});
    }
    else
    {
        std::vector<std::size_t> cumulativeCost(elementCount + 1, 0);
        for (std::size_t index = 0; index < elementCount; ++index)
        {
            const std::size_t dofCount = m_assemblyPattern.elements[index].dofs.size();
            cumulativeCost[index + 1] = cumulativeCost[index] + std::max<std::size_t>(1, dofCount * dofCount);
        }

        ranges.reserve(static_cast<std::size_t>(appliedWorkerCount));
        std::size_t first = 0;
        for (int worker = 0; worker < appliedWorkerCount; ++worker)
        {
            const int remainingWorkers = appliedWorkerCount - worker - 1;
            if (remainingWorkers == 0)
            {
                ranges.push_back({first, elementCount});
                break;
            }

            const std::size_t targetCost = cumulativeCost.back() * static_cast<std::size_t>(worker + 1) /
                                           static_cast<std::size_t>(appliedWorkerCount);
            const auto begin = cumulativeCost.cbegin() + static_cast<std::ptrdiff_t>(first + 1);
            const auto found = std::lower_bound(begin, cumulativeCost.cend(), targetCost);
            const std::size_t minimumLast = first + 1;
            const std::size_t maximumLast = elementCount - static_cast<std::size_t>(remainingWorkers);
            const std::size_t last = std::clamp(static_cast<std::size_t>(found - cumulativeCost.cbegin()),
                                                minimumLast, maximumLast);
            ranges.push_back({first, last});
            first = last;
        }
    }

    m_performanceStatistics.assemblyRangePlanMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.assemblyRangePlanBuildCount;
    return m_assemblyRangesByWorkerCount.emplace(appliedWorkerCount, std::move(ranges)).first->second;
}

bool AnalysisStep::ShouldUseColoredAssembly(int workerCount) const
{
    constexpr std::size_t minimumElementCount = 2048;
    constexpr std::size_t maximumColorCount = 8;
    return workerCount > 1 && m_assemblyPattern.elements.size() >= minimumElementCount &&
           !m_assemblyPattern.elementColorGroups.empty() &&
           m_assemblyPattern.elementColorGroups.size() <= maximumColorCount;
}

void AnalysisStep::PrepareReferenceCableEvaluationWorkspace()
{
    bool rebuild = m_referenceCableWorkspacePatternVersion != m_sparsePatternVersion ||
                   m_referenceCableEvaluationSlots.size() != m_assemblyPattern.elements.size();
    if (!rebuild)
    {
        // 属性关联变更时重新绑定，避免工作任务退回共享截面的可写 Calculate_I。
        for (std::size_t elementIndex : m_elementBatchPlan.cableElementIndices)
        {
            const ReferenceCableEvaluationSlot& slot = m_referenceCableEvaluationSlots[elementIndex];
            const auto property = slot.cable ? slot.cable->m_pProperty.lock() : nullptr;
            const auto section = property ? property->m_pSection.lock() : nullptr;
            if (!slot.cable || slot.cable.get() != m_assemblyPattern.elements[elementIndex].element.get() ||
                section != slot.section)
            {
                rebuild = true;
                break;
            }
        }
    }
    if (!rebuild)
    {
        return;
    }

    m_referenceCableWorkspacePatternVersion = 0;
    m_referenceCableEvaluationSlots.resize(m_assemblyPattern.elements.size());
    for (ReferenceCableEvaluationSlot& slot : m_referenceCableEvaluationSlots)
    {
        slot.cable.reset();
        slot.section.reset();
    }
    m_referenceCableSectionSnapshots.clear();
    std::map<const SectionBase*, std::size_t> sectionIndices;
    for (std::size_t elementIndex : m_elementBatchPlan.cableElementIndices)
    {
        ReferenceCableEvaluationSlot& slot = m_referenceCableEvaluationSlots[elementIndex];
        slot.cable = std::dynamic_pointer_cast<ElementCable>(m_assemblyPattern.elements[elementIndex].element);
        const auto property = slot.cable ? slot.cable->m_pProperty.lock() : nullptr;
        slot.section = property ? property->m_pSection.lock() : nullptr;
        if (!slot.cable || !slot.section)
        {
            throw std::runtime_error("Reference cable evaluation has incomplete property/section references");
        }
        const auto found = sectionIndices.find(slot.section.get());
        if (found != sectionIndices.end())
        {
            slot.sectionIndex = found->second;
            continue;
        }
        slot.sectionIndex = m_referenceCableSectionSnapshots.size();
        sectionIndices.emplace(slot.section.get(), slot.sectionIndex);
        m_referenceCableSectionSnapshots.push_back({slot.section, {}});
    }
    m_referenceCableWorkspacePatternVersion = m_sparsePatternVersion;
}

void AnalysisStep::ClearReferenceCableSectionSnapshots()
{
    for (ReferenceCableEvaluationSlot& slot : m_referenceCableEvaluationSlots)
    {
        if (slot.cable)
        {
            slot.cable->ClearPreparedSectionInertia();
        }
    }
}

bool AnalysisStep::TryAssembleReferenceCableEffectiveTangent(const SolverNameSpace::SystemEvaluationRequest& request,
                                                             _OUT SpMat& effectiveTangent)
{
    if (m_Type != EnumKeyword::StepType::DYNAMIC ||
        m_DynamicSolverType != SolverNameSpace::SolverType::ReferenceTSSBN)
    {
        return false;
    }
    const int requestedWorkers = m_assemblyThreadCountOverride > 0
                                     ? m_assemblyThreadCountOverride
                                     : SolverNameSpace::AssemblySettings::ThreadCount();
    const bool diagnosticOverride = m_DiagnosticReferenceParallelMinimumCableElements > 0;
    const std::size_t minimumCableElements = diagnosticOverride
                                                ? static_cast<std::size_t>(
                                                      m_DiagnosticReferenceParallelMinimumCableElements)
                                                : 128;
    const std::size_t cableCount = m_elementBatchPlan.cableElementIndices.size();
    if (cableCount < minimumCableElements || (!diagnosticOverride && requestedWorkers <= 1))
    {
        return false;
    }
    const int workerCount = std::max(1, std::min(requestedWorkers, static_cast<int>(cableCount)));

    try
    {
        QElapsedTimer preparationTimer;
        preparationTimer.start();
        PrepareReferenceCableEvaluationWorkspace();
        // 首版仅处理圆截面；其他截面的 Calculate_I 可能改变单元提前读取的面积。
        for (const ReferenceCableSectionSnapshot& snapshot : m_referenceCableSectionSnapshots)
        {
            if (!dynamic_cast<const SectionCircular*>(snapshot.section.get()))
            {
                return false;
            }
        }
        // 截面缓存只在主线程更新；求值期间所有索单元读取各自的只读惯性快照。
        for (ReferenceCableSectionSnapshot& snapshot : m_referenceCableSectionSnapshots)
        {
            if (snapshot.section->m_Area <= 0.0)
            {
                throw std::runtime_error("ElementCable section area must be positive");
            }
            snapshot.section->Calculate_I(snapshot.inertia[0], snapshot.inertia[1], snapshot.inertia[2]);
        }
        for (std::size_t elementIndex : m_elementBatchPlan.cableElementIndices)
        {
            ReferenceCableEvaluationSlot& slot = m_referenceCableEvaluationSlots[elementIndex];
            const std::array<double, 3>& inertia = m_referenceCableSectionSnapshots[slot.sectionIndex].inertia;
            slot.cable->SetPreparedSectionInertia(slot.section.get(), inertia[0], inertia[1], inertia[2]);
        }
        m_referenceCableWorkerExceptions.assign(static_cast<std::size_t>(workerCount), nullptr);
        m_referenceCableWorkerElementCounts.assign(static_cast<std::size_t>(workerCount), 0);
        m_referenceCableWorkerFutures.clear();
        m_referenceCableWorkerFutures.reserve(static_cast<std::size_t>(workerCount));
        m_performanceStatistics.referenceCablePreparationMilliseconds +=
            static_cast<double>(preparationTimer.nsecsElapsed()) / 1.0e6;

        const auto evaluateWorker = [this, &request, cableCount, workerCount](int worker)
        {
            const std::size_t workerIndex = static_cast<std::size_t>(worker);
            const std::size_t first = cableCount * workerIndex / static_cast<std::size_t>(workerCount);
            const std::size_t last = cableCount * (workerIndex + 1) / static_cast<std::size_t>(workerCount);
            try
            {
                for (std::size_t cableIndex = first; cableIndex < last; ++cableIndex)
                {
                    const std::size_t elementIndex = m_elementBatchPlan.cableElementIndices[cableIndex];
                    const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
                    ReferenceCableEvaluationSlot& slot = m_referenceCableEvaluationSlots[elementIndex];
                    slot.cable->Get_ke(slot.stiffness);
                    EvaluateDynamicElement(assemblyMap, slot.dynamicData);

                    ++m_referenceCableWorkerElementCounts[workerIndex];
                }
            }
            catch (...)
            {
                m_referenceCableWorkerExceptions[workerIndex] = std::current_exception();
            }
        };

        QElapsedTimer evaluationTimer;
        evaluationTimer.start();
        std::exception_ptr dispatchException;
        try
        {
            if (workerCount == 1)
            {
                evaluateWorker(0);
            }
            else
            {
                QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
                for (int worker = 0; worker < workerCount; ++worker)
                {
                    m_referenceCableWorkerFutures.push_back(QtConcurrent::run(&threadPool, evaluateWorker, worker));
                }
            }
        }
        catch (...)
        {
            dispatchException = std::current_exception();
        }
        // 即使任务提交或单个工作者失败，也必须先等待全部已提交任务结束再清快照。
        for (QFuture<void>& future : m_referenceCableWorkerFutures)
        {
            try
            {
                future.waitForFinished();
            }
            catch (...)
            {
                if (!dispatchException)
                {
                    dispatchException = std::current_exception();
                }
            }
        }
        m_performanceStatistics.referenceCableEvaluationMilliseconds +=
            static_cast<double>(evaluationTimer.nsecsElapsed()) / 1.0e6;
        ++m_performanceStatistics.referenceCableEvaluationCount;
        m_performanceStatistics.referenceCableWorkerCountMaximum =
            std::max(m_performanceStatistics.referenceCableWorkerCountMaximum, workerCount);
        for (std::size_t evaluations : m_referenceCableWorkerElementCounts)
        {
            m_performanceStatistics.referenceCableElementEvaluations += static_cast<long long>(evaluations);
        }
        ClearReferenceCableSectionSnapshots();
        if (dispatchException)
        {
            std::rethrow_exception(dispatchException);
        }
        for (const std::exception_ptr& workerException : m_referenceCableWorkerExceptions)
        {
            if (workerException)
            {
                std::rethrow_exception(workerException);
            }
        }

        QElapsedTimer mergeTimer;
        mergeTimer.start();
        PreparePatternMatrix(effectiveTangent);
        MatrixXd serialStiffness;
        DynamicElementData serialDynamicData;
        // 梁和其他单元在任务结束后求值，所有贡献仍按原单元顺序累加。
        for (std::size_t elementIndex = 0; elementIndex < m_assemblyPattern.elements.size(); ++elementIndex)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
            const ReferenceCableEvaluationSlot& slot = m_referenceCableEvaluationSlots[elementIndex];
            const MatrixXd* stiffness = &slot.stiffness;
            const DynamicElementData* dynamicData = &slot.dynamicData;
            if (!slot.cable)
            {
                assemblyMap.element->Get_ke(serialStiffness);
                EvaluateDynamicElement(assemblyMap, serialDynamicData);
                stiffness = &serialStiffness;
                dynamicData = &serialDynamicData;
            }
            AccumulateEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                            stiffness, *dynamicData, request.accelerationDerivative,
                                            request.velocityDerivative, effectiveTangent.valuePtr());
            CacheReferenceInertia(assemblyMap, dynamicData->inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, dynamicData->inertiaForce, m_dynamicInertiaForce);
        }
        DynamicElementData inertiaData;
        for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
        {
            EvaluateRigidBodyInertia(assemblyMap, inertiaData);
            AccumulateEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                            nullptr, inertiaData, request.accelerationDerivative,
                                            request.velocityDerivative, effectiveTangent.valuePtr());
            CacheReferenceInertia(assemblyMap, inertiaData.inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, inertiaData.inertiaForce, m_dynamicInertiaForce);
        }
        m_performanceStatistics.referenceCableMergeMilliseconds +=
            static_cast<double>(mergeTimer.nsecsElapsed()) / 1.0e6;
        return true;
    }
    catch (...)
    {
        ClearReferenceCableSectionSnapshots();
        throw;
    }
}

void AnalysisStep::AccumulateElementMatrix(const ElementAssemblyMap& assemblyMap, const Eigen::MatrixXd& elementMatrix,
                                           _OUT double* values) const
{
    const int dofCount = static_cast<int>(assemblyMap.dofs.size());
    AccumulateMatrixValues(assemblyMap.valueIndices, dofCount, elementMatrix, values);
}

void AnalysisStep::AccumulateMatrixValues(const std::vector<Eigen::Index>& valueIndices, int dofCount,
                                          const Eigen::MatrixXd& elementMatrix, _OUT double* values) const
{
    const bool referenceAssembly =
        m_referenceGravityEquilibrium || m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN;
    for (int localRow = 0; localRow < dofCount; ++localRow)
    {
        for (int localColumn = 0; localColumn < dofCount; ++localColumn)
        {
            const Eigen::Index valueIndex =
                valueIndices[static_cast<std::size_t>(localRow * dofCount + localColumn)];
            const double contribution = elementMatrix(localRow, localColumn);
            if (valueIndex >= 0 && (!referenceAssembly || !(std::abs(contribution) < 1.0e-14)))
                values[valueIndex] += contribution;
        }
    }
}

void AnalysisStep::AppendElementMatrixValues(const std::vector<Eigen::Index>& valueIndices, int dofCount,
                                             const Eigen::MatrixXd& elementMatrix,
                                             _OUT std::vector<IndexedAssemblyValue>& values) const
{
    const bool referenceAssembly =
        m_referenceGravityEquilibrium || m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN;
    for (int localRow = 0; localRow < dofCount; ++localRow)
    {
        for (int localColumn = 0; localColumn < dofCount; ++localColumn)
        {
            const Eigen::Index valueIndex =
                valueIndices[static_cast<std::size_t>(localRow * dofCount + localColumn)];
            const double contribution = elementMatrix(localRow, localColumn);
            if (valueIndex >= 0 && (!referenceAssembly || !(std::abs(contribution) < 1.0e-14)))
                values.push_back({valueIndex, contribution});
        }
    }
}

void AnalysisStep::AccumulateEffectiveMatrixValues(const std::vector<Eigen::Index>& valueIndices, int dofCount,
                                                   const Eigen::MatrixXd* stiffness,
                                                   const DynamicElementData& dynamicData,
                                                   double accelerationDerivative, double velocityDerivative,
                                                   _OUT double* values) const
{
    const bool referenceAssembly =
        m_referenceGravityEquilibrium || m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN;
    // 直接写入整体稀疏矩阵，避免为每个单元生成临时等效矩阵。
    for (int localRow = 0; localRow < dofCount; ++localRow)
    {
        for (int localColumn = 0; localColumn < dofCount; ++localColumn)
        {
            const Eigen::Index valueIndex =
                valueIndices[static_cast<std::size_t>(localRow * dofCount + localColumn)];
            if (valueIndex >= 0)
            {
                const double staticValue = stiffness ? (*stiffness)(localRow, localColumn) : 0.0;
                const double contribution = staticValue +
                                            accelerationDerivative * dynamicData.mass(localRow, localColumn) +
                                            velocityDerivative * dynamicData.velocityTangent(localRow, localColumn) +
                                            dynamicData.configurationTangent(localRow, localColumn);
                if (!referenceAssembly || !(std::abs(contribution) < 1.0e-14))
                    values[valueIndex] += contribution;
            }
        }
    }
}

void AnalysisStep::AppendEffectiveMatrixValues(const std::vector<Eigen::Index>& valueIndices, int dofCount,
                                               const Eigen::MatrixXd* stiffness,
                                               const DynamicElementData& dynamicData,
                                               double accelerationDerivative, double velocityDerivative,
                                               _OUT std::vector<IndexedAssemblyValue>& values) const
{
    const bool referenceAssembly =
        m_referenceGravityEquilibrium || m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN;
    for (int localRow = 0; localRow < dofCount; ++localRow)
    {
        for (int localColumn = 0; localColumn < dofCount; ++localColumn)
        {
            const Eigen::Index valueIndex =
                valueIndices[static_cast<std::size_t>(localRow * dofCount + localColumn)];
            if (valueIndex < 0)
                continue;
            const double staticValue = stiffness ? (*stiffness)(localRow, localColumn) : 0.0;
            const double contribution = staticValue + accelerationDerivative * dynamicData.mass(localRow, localColumn) +
                                        velocityDerivative * dynamicData.velocityTangent(localRow, localColumn) +
                                        dynamicData.configurationTangent(localRow, localColumn);
            if (!referenceAssembly || !(std::abs(contribution) < 1.0e-14))
                values.push_back({valueIndex, contribution});
        }
    }
}

void AnalysisStep::MergeAssemblyValues(const std::vector<IndexedAssemblyValue>& source, _OUT double* destination)
{
    for (const IndexedAssemblyValue& item : source)
        destination[item.index] += item.value;
}

void AnalysisStep::Assemble_AllLoads(_OUT VectorXd& F1, _OUT VectorXd& F2, _OUT double& Factor, double currentTime)
{
    F1.resize(m_nFixed);
    F1.setZero();
    F2.resize(m_nFree);
    F2.setZero();
    const bool referenceRhsOrder = UsesReferenceRhsOrder();
    if (referenceRhsOrder)
        m_referenceOrderedResidual.resize(0);

    const Force_Wind* gallopingWind = nullptr;
    double gallopingScale = 0.0;
    for (auto& Load : m_pData->m_Load)
    {
        auto pLoadBase = Load.second;
        if (!pLoadBase)
            continue;
        if (m_referenceGravityEquilibrium && pLoadBase->m_LoadType == EnumKeyword::LoadType::FORCE_WIND)
            continue;

        double loadScale = 0.0; // 每个荷载独立的缩放系数

        if (!IsStepScopedDataActive(pLoadBase->m_StepId))
        {
            continue;
        }
        if (pLoadBase->m_StepId != this->m_Id)
        {
            // 历史步的荷载：全额
            loadScale = 1.0;
        }
        else if (pLoadBase->m_StepId == this->m_Id)
        {
            // 当前步的荷载：检查时间窗口
            if (!pLoadBase->IsActive(currentTime))
            {
                continue; // 不在当前时间范围内，跳过
            }
            loadScale = Factor * pLoadBase->GetScaleFactor(currentTime);
        }
        const bool isGallopingWind = m_EnableGalloping && pLoadBase->m_LoadType == EnumKeyword::LoadType::FORCE_WIND;
        if (isGallopingWind)
        {
            gallopingWind = static_cast<const Force_Wind*>(pLoadBase.get());
            gallopingScale = loadScale;
            // AssembleGalloping 已包含完整平均阻力、升力和力矩；此处再调用 AssembleWind 会沿风向重复施加
            // qD 力，并使动力平衡发生两次偏移。
            continue;
        }
        LoadAssembler::Assemble(*pLoadBase, *m_pData, m_nFixed, loadScale, F1, F2);
    }
    if (referenceRhsOrder)
    {
        m_referenceBaseExternalForce.resize(m_nFixed + m_nFree);
        m_referenceBaseExternalForce.head(m_nFixed) = F1;
        m_referenceBaseExternalForce.tail(m_nFree) = F2;
        m_referenceAerodynamicForce.setZero(m_nFixed + m_nFree);
    }
    if (gallopingWind && gallopingScale != 0.0)
    {
        QElapsedTimer profileTimer;
        LoadAssembler::GallopingAssemblyTiming gallopingTiming;
        if (m_DiagnosticProfileGallopingForce)
            profileTimer.start();
        VectorXd referenceFixedAero;
        VectorXd referenceFreeAero;
        if (referenceRhsOrder)
        {
            referenceFixedAero.setZero(m_nFixed);
            referenceFreeAero.setZero(m_nFree);
        }
        LoadAssembler::AssembleGalloping(*gallopingWind, *m_pData, m_gallopingProfileBindings, m_GallopingIceThickness,
                                         m_GallopingInitialAttackDegrees, GetModelUpDirection(), m_nFixed,
                                         gallopingScale, referenceRhsOrder ? referenceFixedAero : F1,
                                         referenceRhsOrder ? referenceFreeAero : F2,
                                         m_DiagnosticProfileGallopingForce ? &gallopingTiming : nullptr,
                                         m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN);
        if (referenceRhsOrder)
        {
            m_referenceAerodynamicForce.head(m_nFixed) = referenceFixedAero;
            m_referenceAerodynamicForce.tail(m_nFree) = referenceFreeAero;
            F1 += referenceFixedAero;
            F2 += referenceFreeAero;
        }
        if (m_DiagnosticProfileGallopingForce)
        {
            m_performanceStatistics.gallopingForceAssemblyMilliseconds +=
                static_cast<double>(profileTimer.nsecsElapsed()) / 1.0e6;
            m_performanceStatistics.gallopingElementLoopMilliseconds += gallopingTiming.elementLoopMilliseconds;
            m_performanceStatistics.gallopingElementEvaluations += gallopingTiming.elementEvaluations;
            ++m_performanceStatistics.gallopingForceAssemblyCount;
        }
    }
    if (referenceRhsOrder)
    {
        m_referenceTotalExternalForce.resize(m_nFixed + m_nFree);
        m_referenceTotalExternalForce.head(m_nFixed) = F1;
        m_referenceTotalExternalForce.tail(m_nFree) = F2;
        m_referenceLoadStateVersion = m_referenceRhsStateVersion;
    }
}

void AnalysisStep::Updata_NodeData(const VectorXd& x1, const VectorXd& x2, const VectorXd& F1, const VectorXd* v2,
                                   const VectorXd* a2)
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        // 确保节点向量大小正确
        if (pNode->m_Displacement.size() < numDOF)
            pNode->m_Displacement.resize(numDOF, 0.0);
        if (pNode->m_Velocity.size() < numDOF)
            pNode->m_Velocity.resize(numDOF, 0.0);
        if (pNode->m_Acceleration.size() < numDOF)
            pNode->m_Acceleration.resize(numDOF, 0.0);
        if (pNode->m_Force.size() < numDOF)
            pNode->m_Force.resize(numDOF, 0.0);

        // 更新各自由度方向的数据
        for (int dofIdx = 0; dofIdx < numDOF; dofIdx++)
        {
            int dof = pNode->m_DOF[dofIdx];

            // 约束自由度：位移增量为0（除非是位移控制，那由调用者直接设置）
            if (dof < m_nFixed)
            {
                pNode->SetAdditiveDisplacement(dofIdx, x1[dof]);
                pNode->m_Force[dofIdx] = F1[dof];
            }
            // 自由自由度：从x2向量获取增量
            else if (dof < m_nFixed + m_nFree)
            {
                pNode->ApplyAdditiveIncrement(dofIdx, x2[dof - m_nFixed]);
                if (v2)
                {
                    pNode->m_Velocity[dofIdx] = (*v2)[dof - m_nFixed];
                }
                if (a2)
                {
                    pNode->m_Acceleration[dofIdx] = (*a2)[dof - m_nFixed];
                }
            }
        }
    }

    // 输出特定节点的位移（用于调试和监控）
    // 可以根据需要修改节点 ID
    int monitorNodeId = 11; // 监控节点 11
    auto it = m_pData->m_Nodes.find(monitorNodeId);
    if (it != m_pData->m_Nodes.end())
    {
        auto pNode = it->second;
        if (pNode->m_Displacement.size() >= 6)
        {
            qDebug().noquote() << QStringLiteral("      节点 %1: u1=%2, u2=%3, u3=%4, θ1=%5, θ2=%6, θ3=%7")
                                      .arg(monitorNodeId)
                                      .arg(pNode->m_Displacement[0], 8, 'e', 2)
                                      .arg(pNode->m_Displacement[1], 8, 'e', 2)
                                      .arg(pNode->m_Displacement[2], 8, 'e', 2)
                                      .arg(pNode->m_Displacement[3], 8, 'e', 2)
                                      .arg(pNode->m_Displacement[4], 8, 'e', 2)
                                      .arg(pNode->m_Displacement[5], 8, 'e', 2);
        }
    }
}

void AnalysisStep::Get_CurrentInforce(_OUT VectorXd& Inforce)
{
    for (auto& elementPair : m_pData->m_Elements)
    {
        auto pElement = elementPair.second;
        std::vector<int> elementDOFs;
        pElement->GetDOFs(elementDOFs);

        // 将单元内力累加到节点
        auto& nodeWeakPtrs = pElement->m_pNode;
        std::vector<int> localDofCounts;
        pElement->GetNodeLocalDOFCounts(localDofCounts);
        int localOffset = 0;

        for (auto nodeIdx = 0; nodeIdx < nodeWeakPtrs.size(); ++nodeIdx)
        {
            const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIdx)];
            auto pNode = nodeWeakPtrs[nodeIdx].lock();
            if (!pNode)
            {
                localOffset += localDofs;
                continue;
            }

            for (int dofIdx = 0; dofIdx < localDofs; ++dofIdx)
            {
                const int localIdx = localOffset + dofIdx;
                if ((localIdx < pElement->m_inforce.size()) && (dofIdx < pNode->m_Force.size()))
                {
                    pNode->m_Force[dofIdx] += pElement->m_inforce[localIdx];
                }
            }
            localOffset += localDofs;
        }

        // 累加自由自由度的内力用于收敛判断
        for (size_t dofIdx = 0; dofIdx < elementDOFs.size(); ++dofIdx)
        {
            int globalDOF = elementDOFs[dofIdx];
            if (globalDOF >= m_nFixed)
            {
                Inforce[globalDOF - m_nFixed] += pElement->m_inforce[dofIdx];
            }
        }
    }
}

bool AnalysisStep::Check_Rhs(const Eigen::VectorXd& Exteralforce, const Eigen::VectorXd& Inforce, Eigen::VectorXd& Rhs)
{
    Rhs = Exteralforce - Inforce;
    double RhsNorm = Rhs.norm();
    if (RhsNorm < m_Tolerance)
    {
        return true;
    }
    return false;
}

void AnalysisStep::Assemble_Constraint(VectorXd& x1, double currentTime, double factor)
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    x1.resize(m_nFixed);
    for (auto& constraintPair : m_pData->m_Constraint)
    {
        auto pConstraint = constraintPair.second;
        if (!pConstraint || !IsStepScopedDataActive(pConstraint->m_StepId))
            continue;
        auto pNode = pConstraint->m_pNode.lock();
        if (!pNode)
            continue;

        int iDirection = static_cast<int>(pConstraint->m_Direction);
        if (iDirection < 0 || iDirection >= pNode->m_DOF.size())
            continue;

        int dof = pNode->m_DOF[iDirection];
        if (dof >= 0 && dof < m_nFixed)
        {
            const double value = pConstraint->GetValue(currentTime, factor);
            x1[dof] = value;
            pNode->SetAdditiveDisplacement(iDirection, value);
            if (pNode->m_DOF.size() == 6 && iDirection >= 3)
            {
                // 六自由度梁单元以 m_Rg 为权威转动状态。施加转动必须与标量边界条件同步，否则指定 UR 只会
                // 出现在输出中，而单元和非线性 MPC 方程均无法感知。
                const Eigen::Vector3d rotationVector(pNode->m_Displacement[3], pNode->m_Displacement[4],
                                                     pNode->m_Displacement[5]);
                Eigen::Matrix3d imposedRotation;
                Utility::CR::Update_NodalRotation(rotationVector, Eigen::Matrix3d::Identity(), imposedRotation);
                pNode->m_Rg = imposedRotation;
            }
        }
    }
}

// ==========================================
// 求解方法实现
// ==========================================

bool AnalysisStep::Solve(bool persistHdf5)
{
    const auto resetInitialization = qScopeGuard(
        [this]()
        {
            m_initializeFromCurrentState = false;
        });
    const auto closeInterruptedResultStream = [this, persistHdf5]() {
        if (!persistHdf5 || !m_pData)
            return;
        try
        {
            m_pData->GetOutputter().EndHdf5ResultStream(false);
        }
        catch (...)
        {
            qCritical() << "分析异常后的 H5/HDF5 结果流清理失败";
        }
    };
    try
    {
        return SolveUnchecked(persistHdf5);
    }
    catch (const std::exception& error)
    {
        m_lastFailureReason = QStringLiteral("分析步发生未处理异常：%1").arg(QString::fromUtf8(error.what()));
        qCritical().noquote() << m_lastFailureReason;
        closeInterruptedResultStream();
        return false;
    }
    catch (...)
    {
        m_lastFailureReason = QStringLiteral("分析步发生未知异常");
        qCritical().noquote() << m_lastFailureReason;
        closeInterruptedResultStream();
        return false;
    }
}

bool AnalysisStep::SolveUnchecked(bool persistHdf5)
{
    m_performanceStatistics = {};
    m_dynamicReactionSupportChecked = false;
    m_dynamicReactionUnsupportedReason.clear();
    m_tssbnStoppedAtCheckpoint = false;
    m_tssbnAverageTimeStep = 0.0;
    m_tssbnAcceptedStepCount = 0;
    QElapsedTimer totalTimer;
    totalTimer.start();
    m_lastFailureReason.clear();
    m_lastProgressMessage.clear();
    MklLocalThreadLimit mklThreadLimit(m_pardisoThreadCountOverride);
    if (!PrepareData())
    {
        m_lastFailureReason = QStringLiteral("分析步未关联结构数据，无法启动求解。");
        return false;
    }
    QString configurationError;
    if (!ValidateSolveConfiguration(configurationError))
    {
        m_lastFailureReason = QStringLiteral("分析步配置错误：%1").arg(configurationError);
        qDebug().noquote() << QStringLiteral("Error:") << m_lastFailureReason;
        return false;
    }

    std::unique_ptr<SolverNameSpace::ISolver> solver;
    try
    {
        solver = SolverNameSpace::SolverFactory::Create(*this);
    }
    catch (const std::exception& error)
    {
        m_lastFailureReason = QStringLiteral("分析步求解器参数无效：%1").arg(QString::fromUtf8(error.what()));
        qDebug().noquote() << QStringLiteral("Error:") << m_lastFailureReason;
        return false;
    }
    if (!solver)
    {
        m_lastFailureReason = QStringLiteral("分析步没有可用的求解器。");
        qDebug().noquote() << QStringLiteral("Error:") << m_lastFailureReason;
        return false;
    }

    if (m_Type == EnumKeyword::StepType::STATIC && m_referenceGravityEquilibrium)
    {
        auto* staticSolver = dynamic_cast<SolverNameSpace::SolverStatic*>(solver.get());
        if (!staticSolver)
            return false;
        auto& parameters = staticSolver->GetParams();
        parameters.referenceEquilibrium = true;
        parameters.referenceTimeStep = m_StepSize;
        parameters.tol_dx = m_Tolerance;
    }

    if (m_Type == EnumKeyword::StepType::DYNAMIC &&
        m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN)
    {
        const auto* referenceSolver = dynamic_cast<const SolverNameSpace::SolverAdaptiveTSSBN*>(solver.get());
        const bool independentErrorControl = referenceSolver &&
                                             referenceSolver->GetParams().referenceIndependentErrorControl;
        const char* accelerationEstimator = !independentErrorControl ? "extrapolated"
                                            : HasActiveNonlinearMPC() ? "constrained-velocity-probe" : "true-y3";
        const QString errorDescription =
            independentErrorControl ? QStringLiteral("逐自由度误差范数、独立速度误差估计和接受端点加速度恢复；")
                                    : QStringLiteral("全局误差范数、外推嵌入加速度；");
        Logger::Instance().Diagnostic(QStringLiteral("参考 TSSBN 舞动：使用当前结构单元；固定风速动压、固定升阻力方向、"
                                                     "原气动表查表插值；每轮更新气动力且只装配一次；%1"
                                                     "迭代量反馈步长控制；索轴力保留正负值。"
                                                     "气动系数沿用原有 360° 周期插值。")
                                          .arg(errorDescription));
        if (Logger::DetailedOutputEnabled())
            QTextStream(stdout) << "YQY_CALCULATION_ROUTE solver=Reference-TSSBN-Galloping"
                                << " structure=current aero=table-interpolation force=current-single"
                                << " interpolation="
                                << (m_pData->m_AeroManager.isShapePreservingInterpolationEnabled() ? "periodic-shape"
                                                                                                   : "linear")
                                << " error=" << (independentErrorControl ? "componentwise-max" : "legacy-global")
                                << " y3=" << accelerationEstimator << " controller=reference"
                                << " cable_constitutive=signed-reference"
                                << " nominal_attack_deg=" << m_GallopingInitialAttackDegrees
                                << " ice_mm=" << m_GallopingIceThickness << Qt::endl;
    }

    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        if (!load)
        {
            m_lastFailureReason = QStringLiteral("荷载 %1 为空，无法求解。").arg(loadId);
            return false;
        }

        QString timeFunctionError;
        if (!load->ValidateTimeFunction(timeFunctionError))
        {
            m_lastFailureReason = QStringLiteral("荷载 %1 时程参数无效：%2").arg(loadId).arg(timeFunctionError);
            qDebug().noquote() << QStringLiteral("Error:") << m_lastFailureReason;
            return false;
        }
    }
    QString gallopingError;
    if (!ValidateGallopingConfiguration(&gallopingError))
    {
        m_lastFailureReason = QStringLiteral("舞动配置错误：%1").arg(gallopingError);
        qDebug().noquote() << m_lastFailureReason;
        return false;
    }
    if (!PrepareGallopingData(&gallopingError))
    {
        m_lastFailureReason = QStringLiteral("舞动气动参数错误：%1").arg(gallopingError);
        qDebug().noquote() << m_lastFailureReason;
        return false;
    }
    Init();
    if (!BindGallopingProfiles(&gallopingError))
    {
        m_lastFailureReason = QStringLiteral("舞动气动剖面绑定错误：%1").arg(gallopingError);
        qDebug().noquote() << m_lastFailureReason;
        return false;
    }

    // 初始化单元长度
    Get_ElementLength();

    if (m_Type == EnumKeyword::StepType::DYNAMIC && m_StructuralDamping.settings.enabled)
    {
        QString dampingError;
        if (!PrepareStructuralDamping(dampingError))
        {
            qDebug().noquote() << QStringLiteral("结构阻尼计算失败：") << dampingError;
            m_lastFailureReason =
                QStringLiteral(
                    "结构阻尼计算失败，动力分析未启动：%1。请编辑该动力分析步，关闭结构阻尼或调整阻尼设置后重新计算。")
                    .arg(dampingError);
            return false;
        }
        Logger::Instance().Diagnostic(m_StructuralDamping.report.summary);
        const auto& dampingSettings = m_StructuralDamping.settings;
        const auto& dampingReport = m_StructuralDamping.report;
        if (dampingSettings.useExplicitRayleighCoefficients)
            qInfo().noquote() << QStringLiteral("结构阻尼：局部 Rayleigh（指定系数）。");
        else
            qInfo().noquote() << QStringLiteral("结构阻尼：局部 Rayleigh，平动 %1%，扭转 %2%。")
                                     .arg(dampingSettings.translationDampingRatio * 100.0, 0, 'g', 4)
                                     .arg(dampingSettings.torsionDampingRatio * 100.0, 0, 'g', 4);
        QString dampingDetails;
        QTextStream dampingTrace(&dampingDetails);
        dampingTrace.setRealNumberPrecision(17);
        dampingTrace << "YQY_STRUCTURAL_DAMPING enabled=1 formulation="
                     << "local-rayleigh";
        if (!dampingSettings.useExplicitRayleighCoefficients)
            dampingTrace << " target_translation_ratio=" << dampingSettings.translationDampingRatio
                         << " target_torsion_ratio=" << dampingSettings.torsionDampingRatio;
        if (dampingReport.localGroups.empty())
            Logger::Instance().Diagnostic(QStringLiteral("Rayleigh 实际系数：平动质量 %1，平动刚度 %2；"
                                                         "扭转质量 %3，扭转刚度 %4。")
                                              .arg(dampingReport.translationMassCoefficient, 0, 'g', 17)
                                              .arg(dampingReport.translationStiffnessCoefficient, 0, 'g', 17)
                                              .arg(dampingReport.torsionMassCoefficient, 0, 'g', 17)
                                              .arg(dampingReport.torsionStiffnessCoefficient, 0, 'g', 17));
        else
        {
            for (const auto& group : dampingReport.localGroups)
                Logger::Instance().Diagnostic(QStringLiteral("局部 Rayleigh 组 %1：平动质量 %2，平动刚度 %3；"
                                                             "自扭质量 %4，自扭刚度 %5。")
                                                  .arg(group.label)
                                                  .arg(group.coefficients[0], 0, 'g', 17)
                                                  .arg(group.coefficients[1], 0, 'g', 17)
                                                  .arg(group.coefficients[2], 0, 'g', 17)
                                                  .arg(group.coefficients[3], 0, 'g', 17));
        }
        dampingTrace << " coefficients=" << (dampingSettings.useExplicitRayleighCoefficients ? "explicit" : "modal-fit")
                     << " y1=" << dampingReport.translationMassCoefficient
                     << " y2=" << dampingReport.translationStiffnessCoefficient
                     << " c1=" << dampingReport.torsionMassCoefficient
                     << " c2=" << dampingReport.torsionStiffnessCoefficient
                     << " local_group_count=" << dampingReport.localGroups.size();
        dampingTrace << " translation_frequency_1_hz=" << dampingReport.translationReferenceFrequency1Hz
                     << " translation_frequency_2_hz=" << dampingReport.translationReferenceFrequency2Hz
                     << " torsion_frequency_1_hz=" << dampingReport.torsionReferenceFrequency1Hz
                     << " torsion_frequency_2_hz=" << dampingReport.torsionReferenceFrequency2Hz;
        if (!dampingSettings.useExplicitRayleighCoefficients)
        {
            dampingTrace << " requested_modes=" << dampingReport.requestedModeCount
                         << " converged_modes=" << dampingReport.convergedModeCount
                         << " valid_modes=" << dampingReport.validModeCount
                         << " supplemental_modes=" << dampingReport.supplementalModeCount
                         << " degenerate_rotations=" << dampingReport.degenerateModeRotationCount
                         << " degenerate_rotations_rejected=" << dampingReport.degenerateModeRotationRejectedCount
                         << " max_coefficient_backcheck_error=" << dampingReport.maximumCoefficientBackCheckError
                         << " matrix_validation_passed=" << (dampingReport.matrixValidationPassed ? 1 : 0)
                         << " max_reference_projection_error=" << dampingReport.maximumReferenceProjectionError
                         << " reference_projection_tolerance=" << dampingReport.referenceProjectionTolerance
                         << " min_mass_normalized_damping_eigenvalue="
                         << dampingReport.minimumMassNormalizedDampingEigenvalue
                         << " dissipation_eigenvalue_tolerance=" << dampingReport.dissipationEigenvalueTolerance
                         << " damping_matrix_relative_asymmetry=" << dampingReport.dampingMatrixRelativeAsymmetry;
        }
        dampingTrace << Qt::endl;
        Logger::Instance().Diagnostic(dampingDetails.trimmed());
        if (m_DiagnosticTssbnErrorTrace)
        {
            QTextStream modeTrace(stdout);
            modeTrace.setRealNumberPrecision(17);
            for (const auto& mode : dampingReport.modeSpectrum)
            {
                modeTrace << "YQY_DAMPING_MODE order=" << mode.modeNumber << " frequency_hz=" << mode.frequencyHz
                          << " valid=" << (mode.valid ? 1 : 0)
                          << " family=" << (mode.torsion ? "torsion" : "translation")
                          << " torsion_fraction=" << mode.torsionFraction << " eigen_residual=" << mode.residual
                          << " projected_ratio=" << mode.projectedDampingRatio
                          << " target_ratio=" << mode.targetDampingRatio
                          << " is_reference=" << (mode.isReferenceMode ? 1 : 0) << Qt::endl;
            }
            for (const auto& mode : dampingReport.supplementalModeSpectrum)
            {
                modeTrace << "YQY_DAMPING_MODE order=" << mode.modeNumber << " frequency_hz=" << mode.frequencyHz
                          << " supplemental=1 valid=" << (mode.valid ? 1 : 0)
                          << " family=" << (mode.torsion ? "torsion" : "translation")
                          << " torsion_fraction=" << mode.torsionFraction << " eigen_residual=" << mode.residual
                          << " projected_ratio=" << mode.projectedDampingRatio
                          << " target_ratio=" << mode.targetDampingRatio
                          << " is_reference=" << (mode.isReferenceMode ? 1 : 0) << Qt::endl;
            }
        }
    }

    if (m_Type == EnumKeyword::StepType::DYNAMIC && m_initialDynamicStateCallback)
    {
        auto callback = std::move(m_initialDynamicStateCallback);
        m_initialDynamicStateCallback = {};
        QString perturbationError;
        if (!callback(perturbationError))
        {
            m_lastFailureReason = perturbationError.isEmpty()
                                      ? QStringLiteral("动力步初始扰动失败")
                                      : QStringLiteral("动力步初始扰动失败：%1").arg(perturbationError);
            qDebug().noquote() << m_lastFailureReason;
            return false;
        }
    }

    if (m_Type == EnumKeyword::StepType::DYNAMIC)
    {
        QString accelerationError;
        if (!InitializeConsistentDynamicAcceleration(0.0, accelerationError))
        {
            m_lastFailureReason = QStringLiteral("动力步初始加速度计算失败：%1").arg(accelerationError);
            qDebug().noquote() << QStringLiteral("Error:") << m_lastFailureReason;
            return false;
        }
    }

    if (m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN && Logger::DetailedOutputEnabled())
    {
        std::set<std::pair<int, int>> reportedWires;
        for (const auto& [elementId, element] : m_pData->m_Elements)
        {
            if (!element || !element->HasAerodynamicLoad() ||
                !reportedWires.emplace(element->m_AeroBundleCount, element->m_WireId).second)
                continue;
            LoadAssembler::GallopingElementAeroSample sample;
            if (!SampleGallopingElementAero(elementId, 0.0, sample))
                continue;
            QTextStream(stdout) << "YQY_REFERENCE_AERO_INITIAL element=" << elementId
                                << " wire=" << element->m_WireId << " attack_deg=" << sample.attackDegrees
                                << " wind_mps=" << sample.relativeSpeed << " scale=" << sample.appliedScale
                                << " cl=" << sample.liftCoefficient << " cd=" << sample.dragCoefficient
                                << " cm=" << sample.momentCoefficient << Qt::endl;
        }
    }

    bool resumingTssbn = false;
    if (m_tssbnCheckpointOptions)
    {
        const auto& options = *m_tssbnCheckpointOptions;
        auto* adaptiveTssbn = dynamic_cast<SolverNameSpace::SolverAdaptiveTSSBN*>(solver.get());
        if (!adaptiveTssbn || options.caseSignature.size() != 32 ||
            (options.savePath.isEmpty() && options.resumePath.isEmpty()) ||
            (!options.savePath.isEmpty() && options.savePath == options.resumePath) ||
            (!options.resumePath.isEmpty() && persistHdf5))
        {
            m_lastFailureReason = QStringLiteral("TSSBN 检查点仅支持单个自适应 TSSBN 动力步；恢复时不能写完整 HDF5");
            return false;
        }
        const QByteArray modelSignature =
            MakeTssbnModelSignature(*m_pData, m_nFixed, m_nFree, m_StructuralDamping, options.caseSignature,
                                    m_GallopingProfileBindingMode, m_gallopingProfileBindings,
                                    m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN,
                                    adaptiveTssbn->GetParams());
        if (modelSignature.size() != 32)
        {
            m_lastFailureReason = QStringLiteral("TSSBN 检查点模型签名计算失败");
            return false;
        }
        if (!options.resumePath.isEmpty())
        {
            TssbnCheckpointRecord checkpoint;
            QString checkpointError;
            if (!ReadTssbnCheckpoint(options.resumePath, modelSignature, *m_pData, checkpoint, checkpointError))
            {
                m_lastFailureReason = QStringLiteral("TSSBN 检查点恢复失败：%1").arg(checkpointError);
                return false;
            }
            if (checkpoint.controller.time >= m_Time)
            {
                m_lastFailureReason = QStringLiteral("TSSBN 检查点时间必须小于动力步总时间");
                return false;
            }
            for (const auto& state : checkpoint.nodes)
            {
                const auto& node = m_pData->m_Nodes.at(state.id);
                node->m_Displacement = state.displacement;
                node->m_Velocity = state.velocity;
                node->m_Acceleration = state.acceleration;
                node->m_Rg = state.rotation;
                node->m_OmegaMaterial = state.materialOmega;
                node->m_AlphaMaterial = state.materialAlpha;
                // 磁盘仅恢复接受端点；普通隐式阶段在下一次显式 Begin 时重新锚定。
                node->m_TssbnImplicitStage = Node::TssbnImplicitStageState{};
            }
            StoreCurrentNodeState();
            for (const auto& state : checkpoint.cables)
            {
                const auto cable = std::dynamic_pointer_cast<ElementCable>(m_pData->m_Elements.at(state.id));
                if (!cable || !cable->RestoreCommittedSpatialTwist(state.committedTwist) ||
                    cable->GetCommittedSpatialTwist() != state.committedTwist)
                {
                    m_lastFailureReason = QStringLiteral("TSSBN 索单元 %1 已提交扭角恢复校验失败").arg(state.id);
                    return false;
                }
            }
            for (const auto& state : checkpoint.beams)
            {
                const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(m_pData->m_Elements.at(state.id));
                if (!beam || !beam->RestoreCommittedGaussRotations(state.rotations))
                {
                    m_lastFailureReason = QStringLiteral("TSSBN 梁单元 %1 Gauss 姿态恢复校验失败").arg(state.id);
                    return false;
                }
            }
            m_mpcMultipliers = checkpoint.mpcMultipliers;
            m_dynamicInertiaForce = VectorXd::Zero(m_nFree);
            adaptiveTssbn->SetResumeState(checkpoint.controller);
            resumingTssbn = true;
            qDebug().noquote() << QStringLiteral("TSSBN 检查点已恢复：t=%1，下一 dt=%2，索单元=%3")
                                      .arg(checkpoint.controller.time, 0, 'g', 12)
                                      .arg(checkpoint.controller.nextTimeStep, 0, 'g', 12)
                                      .arg(checkpoint.cables.size());
            QTextStream(stdout) << "tssbn_checkpoint_resumed time=" << checkpoint.controller.time
                                << " next_dt=" << checkpoint.controller.nextTimeStep
                                << " cables=" << checkpoint.cables.size() << Qt::endl;
        }
        if (!options.savePath.isEmpty())
        {
            adaptiveTssbn->SetCheckpointWriter(
                options.targetTime, options.stopAfterSave,
                [this, path = options.savePath, modelSignature](
                    const SolverNameSpace::SolverAdaptiveTSSBN::CheckpointState& state, _OUT QString& error)
                {
                    if (!WriteTssbnCheckpoint(path, modelSignature, state, *m_pData, m_mpcMultipliers, error))
                        return false;
                    QTextStream(stdout) << "tssbn_checkpoint_saved time=" << state.time
                                        << " next_dt=" << state.nextTimeStep
                                        << " accepted=" << state.acceptedStepCount << " path=" << path << Qt::endl;
                    return true;
                });
        }
    }

    const bool dynamicAnalysis = (m_Type == EnumKeyword::StepType::DYNAMIC);
    m_pData->GetOutputter().SetResultContext(m_Id, static_cast<int>(m_Type));
    m_pData->GetOutputter().SetKeepFramesInMemory(!dynamicAnalysis || !m_pData->m_OutputControl.m_StreamResult);

    if (dynamicAnalysis && persistHdf5)
    {
        if (!m_pData->GetOutputter().BeginHdf5ResultStream(m_pData->m_OutputControl.m_Hdf5FileName, m_pData,
                                                           m_pData->m_OutputControl.m_SourceModelName))
        {
            qDebug().noquote() << QStringLiteral("Error: H5/HDF5 动力结果流式输出初始化失败");
            return false;
        }
    }

    // 每个分析结果都从 t=0 的收敛模型状态开始；动力分析直接写入已打开的 H5 流，静力分析保存在内存中，
    // 并纳入最终 H5 导出。
    if (!resumingTssbn)
        OnStepCompleted(0.0);

    // 执行求解
    bool solveOk = true;
    const bool solverSucceeded = solver->Solve(*this, m_Time);
    if (const auto* adaptiveTssbn = dynamic_cast<SolverNameSpace::SolverAdaptiveTSSBN*>(solver.get()))
    {
        m_tssbnStoppedAtCheckpoint = adaptiveTssbn->WasStoppedAtCheckpoint();
        m_tssbnAverageTimeStep = adaptiveTssbn->GetAverageTimeStep();
        m_tssbnAcceptedStepCount = adaptiveTssbn->GetAcceptedStepCount();
    }
    if (!solverSucceeded)
    {
        qDebug().noquote() << QStringLiteral("求解失败: %1").arg(solver->GetName());
        m_lastFailureReason = m_lastProgressMessage.isEmpty()
                                  ? QStringLiteral("%1 求解失败，请检查约束、荷载和模型刚度。").arg(solver->GetName())
                                  : m_lastProgressMessage;
        solveOk = false;
    }
    const bool solveCompleted = solveOk;

    if (dynamicAnalysis && persistHdf5)
    {
        const bool hdf5Finalized = m_pData->GetOutputter().EndHdf5ResultStream(solveOk);
        if (!hdf5Finalized)
        {
            qDebug().noquote() << QStringLiteral("Error: H5/HDF5 动力结果最终落盘失败");
            solveOk = false;
        }
    }
    else if (!dynamicAnalysis && persistHdf5)
    {
        bool hdf5Saved = false;
        if (m_pData->GetOutputter().GetFrameCount() == 0)
        {
            qDebug().noquote() << QStringLiteral("Error: 静力分析没有可写入 H5 的结果帧");
            solveOk = false;
        }
        else
        {
            hdf5Saved = m_pData->GetOutputter().SaveHdf5File(m_pData->m_OutputControl.m_Hdf5FileName, m_pData,
                                                             m_pData->m_OutputControl.m_SourceModelName, solveOk);
            if (!hdf5Saved)
            {
                qDebug().noquote() << QStringLiteral("Error: H5/HDF5 结果文件保存失败");
                solveOk = false;
            }
        }
        if (!solveCompleted && hdf5Saved)
        {
            qDebug().noquote() << QStringLiteral("静力求解未完成，已保存此前收敛的 %1 帧部分结果")
                                      .arg(m_pData->GetOutputter().GetFrameCount());
        }
    }
    else if (!solveOk)
    {
        qDebug().noquote() << QStringLiteral("求解失败，跳过结果文件保存");
    }

    m_performanceStatistics.totalMilliseconds = static_cast<double>(totalTimer.nsecsElapsed()) / 1.0e6;
    return solveOk;
}

bool AnalysisStep::InitializeConsistentDynamicAcceleration(double time, QString& errorMessage)
{
    if (!m_pData || m_Type != EnumKeyword::StepType::DYNAMIC || m_nFree <= 0)
    {
        errorMessage = QStringLiteral("动力模型或自由度尚未初始化");
        return false;
    }

    VectorXd displacement;
    VectorXd velocity;
    VectorXd ignoredAcceleration;
    Get_CurrentStepState(displacement, velocity, ignoredAcceleration);
    SetTrialKinematics(velocity, VectorXd::Zero(m_nFree));

    SpMat mass;
    RefreshInertiaForce();
    AssembleMassMatrix(mass);
    UpdateInternalForce();

    VectorXd constrainedForce;
    VectorXd externalForce;
    VectorXd residual;
    ComputeExternalForce(time, 1.0, constrainedForce, externalForce);
    ComputeResidual(externalForce, residual);
    if (mass.rows() != m_nFree || mass.cols() != m_nFree || residual.size() != m_nFree || !residual.allFinite())
    {
        errorMessage = QStringLiteral("质量矩阵或初始残差维数无效");
        return false;
    }

    SolverNameSpace::NonlinearMPCData constraints;
    if (!AssembleNonlinearMPC(constraints))
    {
        errorMessage = QStringLiteral("无法组装初始状态的 MPC 约束");
        return false;
    }
    if (!constraints.Empty() && constraints.value.norm() > 1.0e-9)
    {
        errorMessage =
            QStringLiteral("初始构型不满足 MPC 位置约束，残差为 %1").arg(constraints.value.norm(), 0, 'g', 8);
        return false;
    }

    if (!constraints.Empty())
    {
        const double velocityScale = std::max(1.0, velocity.norm());
        const double velocityConstraintNorm = (constraints.jacobian * velocity).norm();
        if (!std::isfinite(velocityConstraintNorm) || velocityConstraintNorm > 1.0e-8 * velocityScale)
        {
            errorMessage = QStringLiteral("初始速度不满足 MPC 速度约束，残差为 %1")
                               .arg(velocityConstraintNorm, 0, 'g', 8);
            return false;
        }
    }

    SolverNameSpace::LinearSystemSolver solver;
    VectorXd acceleration;
    VectorXd multipliers;
    QString recoveryError;
    if (!SolverNameSpace::NonlinearMPC::RecoverConsistentAcceleration(
            mass, residual, velocity, VectorXd::Zero(m_nFree), constraints, solver, acceleration, multipliers,
            recoveryError))
    {
        errorMessage = QStringLiteral("初始加速度恢复失败：%1").arg(recoveryError);
        return false;
    }
    SetNonlinearMPCMultipliers(multipliers);
    SetTrialKinematics(velocity, acceleration);
    CommitState();
    return true;
}

void AnalysisStep::SetRuntimeCallbacks(ProgressCallback progressCallback, CancelCallback cancelCallback)
{
    m_progressCallback = std::move(progressCallback);
    m_cancelCallback = std::move(cancelCallback);
}

void AnalysisStep::ClearRuntimeCallbacks()
{
    m_progressCallback = {};
    m_cancelCallback = {};
}

bool AnalysisStep::IsCancellationRequested() const
{
    return m_cancelCallback && m_cancelCallback();
}

void AnalysisStep::ReportProgress(double progress, const QString& message)
{
    if (!message.isEmpty())
        m_lastProgressMessage = message;
    if (m_progressCallback)
        m_progressCallback(std::clamp(progress, 0.0, 1.0), message);
}

// ==========================================
// 新增辅助函数实现
// ==========================================

void AnalysisStep::CalculateReactions(_OUT VectorXd& F1)
{
    // 迭代已收敛，节点里的 m_Force 是结构处于平衡状态下的真实总内力
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        // 确保数组大小足够
        if (pNode->m_ReactionForce.size() < numDOF)
            pNode->m_ReactionForce.resize(numDOF, 0.0);

        for (int i = 0; i < numDOF; ++i)
        {
            int dof = pNode->m_DOF[i];

            if (dof >= 0 && dof < m_nFixed)
            {
                // 约束自由度：计算真实反力
                // F1 是之前 ComputeExternalForce 存入的约束节点外荷载
                double extForce = (dof < F1.size()) ? F1[dof] : 0.0;

                // 公式：反力 = 内力 - 外力
                pNode->m_ReactionForce[i] = pNode->m_Force[i] - extForce;
            }
            else
            {
                // 自由自由度：没有支座反力
                pNode->m_ReactionForce[i] = 0.0;
            }
        }
    }
    // 非线性 MPC 的约束反力由收敛乘子恢复。
    int multiplierOffset = 0;
    for (const auto& [id, mpc] : m_pData->m_MPCConstraints)
    {
        if (!mpc || !IsStepScopedDataActive(mpc->m_StepId))
            continue;
        SolverNameSpace::NonlinearMPCData contribution;
        if (!mpc->Evaluate(m_nFixed, m_nFree, contribution))
            continue;
        const int count = static_cast<int>(contribution.value.size());
        if (multiplierOffset + count > m_mpcMultipliers.size())
            break;
        mpc->AccumulateReactions(m_nFixed, m_mpcMultipliers.segment(multiplierOffset, count));
        multiplierOffset += count;
    }
}

void AnalysisStep::RecoverDynamicReactions(double time)
{
    // 只恢复状态提交后仍可准确重建的固定自由度反力。
    // 含自由转动的梁和固定端 MPC 的约束力在提交后无法保证与接受态一致。
    if (!m_dynamicReactionSupportChecked)
    {
        m_dynamicReactionSupportChecked = true;
        std::set<int> prescribedRotatingDofs;
        for (const auto& [constraintId, constraint] : m_pData->m_Constraint)
        {
            Q_UNUSED(constraintId);
            if (!constraint || !IsStepScopedDataActive(constraint->m_StepId) || constraint->m_Value == 0.0)
                continue;
            const int direction = static_cast<int>(constraint->m_Direction);
            const auto node = constraint->m_pNode.lock();
            if (node && node->m_DOF.size() == 6 && direction >= 3 && direction < 6)
                prescribedRotatingDofs.insert(node->m_DOF[direction]);
        }
        std::vector<int> dofs;
        for (const auto& [elementId, element] : m_pData->m_Elements)
        {
            if (!element)
                continue;
            element->GetDOFs(dofs);
            const bool hasFixedDof = std::any_of(dofs.cbegin(), dofs.cend(),
                                                 [this](int dof)
                                                 {
                                                     return dof >= 0 && dof < m_nFixed;
                                                 });
            // CR 梁全部转动自由度恒零固定时没有转动增量，接受态的平动惯性行可直接恢复。
            const auto hasUnsupportedRotation = [this, &prescribedRotatingDofs](int dof)
            {
                return (dof >= m_nFixed && dof < m_nFixed + m_nFree) || prescribedRotatingDofs.count(dof) != 0;
            };
            const bool hasUnsupportedCrRotation =
                dynamic_cast<const ElementBeam_CR*>(element.get()) &&
                (dofs.size() != 12 || std::any_of(dofs.cbegin() + 3, dofs.cbegin() + 6, hasUnsupportedRotation) ||
                 std::any_of(dofs.cbegin() + 9, dofs.cend(), hasUnsupportedRotation));
            if (hasFixedDof && (hasUnsupportedCrRotation || dynamic_cast<const ElementBeam_Simo*>(element.get())))
            {
                m_dynamicReactionUnsupportedReason = QStringLiteral("固定自由度连接有限转动梁单元 %1").arg(elementId);
                break;
            }
        }
        if (m_dynamicReactionUnsupportedReason.isEmpty())
        {
            for (const auto& [mpcId, mpc] : m_pData->m_MPCConstraints)
            {
                if (!mpc || !IsStepScopedDataActive(mpc->m_StepId))
                    continue;
                bool touchesFixedNode = false;
                for (int nodeId : mpc->GetNodeIds())
                {
                    const auto found = m_pData->m_Nodes.find(nodeId);
                    if (found == m_pData->m_Nodes.end() || !found->second)
                        continue;
                    const auto& nodeDofs = found->second->m_DOF;
                    touchesFixedNode = std::any_of(nodeDofs.cbegin(), nodeDofs.cend(),
                                                   [this](int dof) { return dof >= 0 && dof < m_nFixed; });
                    if (touchesFixedNode)
                        break;
                }
                if (touchesFixedNode)
                {
                    m_dynamicReactionUnsupportedReason =
                        QStringLiteral("活跃 MPC %1 连接固定节点").arg(mpcId);
                    break;
                }
            }
        }
        if (!m_dynamicReactionUnsupportedReason.isEmpty())
            qWarning().noquote() << QStringLiteral("动力支座反力暂不恢复：%1；输出维持零值。")
                                        .arg(m_dynamicReactionUnsupportedReason);
    }
    if (!m_dynamicReactionUnsupportedReason.isEmpty())
        return;

    VectorXd fixedExternalForce;
    VectorXd unusedFreeExternalForce;
    ComputeExternalForce(time, 1.0, fixedExternalForce, unusedFreeExternalForce);
    VectorXd fixedInternalForce = VectorXd::Zero(m_nFixed);
    MatrixXd unusedStiffness;
    VectorXd elementInertia;
    std::vector<int> dofs;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (!element)
            continue;
        element->GetDOFs(dofs);
        const bool hasFixedDof = std::any_of(dofs.cbegin(), dofs.cend(),
                                             [this](int dof) { return dof >= 0 && dof < m_nFixed; });
        if (!hasFixedDof)
            continue;
        element->Get_ke(unusedStiffness);
        element->Get_InertiaForce(elementInertia);
        if (element->m_inforce.size() != static_cast<int>(dofs.size()) ||
            elementInertia.size() != static_cast<int>(dofs.size()))
            throw std::runtime_error("Dynamic reaction element force size mismatch");
        VectorXd elementForce = element->m_inforce + elementInertia;

        for (int localDof = 0; localDof < static_cast<int>(dofs.size()); ++localDof)
        {
            const int globalDof = dofs[localDof];
            if (globalDof >= 0 && globalDof < m_nFixed)
                fixedInternalForce[globalDof] += elementForce[localDof];
        }
    }
    for (const auto& [inertiaId, inertia] : m_pData->m_RigidBodyInertias)
    {
        Q_UNUSED(inertiaId);
        if (!inertia)
            continue;
        inertia->GetDOFs(dofs);
        const bool hasFixedDof = std::any_of(dofs.cbegin(), dofs.cend(),
                                             [this](int dof) { return dof >= 0 && dof < m_nFixed; });
        if (!hasFixedDof)
            continue;
        MatrixXd mass;
        MatrixXd velocityTangent;
        MatrixXd configurationTangent;
        inertia->GetDynamicContributions(mass, elementInertia, velocityTangent, configurationTangent);
        if (elementInertia.size() != static_cast<int>(dofs.size()))
            throw std::runtime_error("Dynamic reaction rigid inertia size mismatch");
        for (int localDof = 0; localDof < static_cast<int>(dofs.size()); ++localDof)
        {
            const int globalDof = dofs[localDof];
            if (globalDof >= 0 && globalDof < m_nFixed)
                fixedInternalForce[globalDof] += elementInertia[localDof];
        }
    }
    if (HasActiveLocalRayleighDamping())
    {
        VectorXd fixedDampingForce;
        ComputeLocalRayleighFixedForce(fixedDampingForce);
        fixedInternalForce += fixedDampingForce;
    }
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        if (!node)
            continue;
        node->m_ReactionForce.assign(node->m_DOF.size(), 0.0);
        for (int localDof = 0; localDof < node->m_DOF.size(); ++localDof)
        {
            const int globalDof = node->m_DOF[localDof];
            if (globalDof >= 0 && globalDof < m_nFixed)
                node->m_ReactionForce[localDof] = fixedInternalForce[globalDof] - fixedExternalForce[globalDof];
        }
    }
}

void AnalysisStep::Get_CurrentStepState(VectorXd& U, VectorXd& V, VectorXd& A) const
{
    // 调整向量大小
    if (U.size() != m_nFree)
        U.resize(m_nFree);
    if (V.size() != m_nFree)
        V.resize(m_nFree);
    if (A.size() != m_nFree)
        A.resize(m_nFree);

    U.setZero();
    V.setZero();
    A.setZero();

    // 遍历节点提取自由自由度数据
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        for (int i = 0; i < numDOF; ++i)
        {
            int globalDof = pNode->m_DOF[i];

            // 仅提取自由自由度 (>= m_nFixed)
            if (globalDof >= m_nFixed && globalDof < (m_nFixed + m_nFree))
            {
                int idx = globalDof - m_nFixed;

                if (i < pNode->m_Displacement.size())
                    U[idx] = pNode->m_Displacement[i];
                if (i < pNode->m_Velocity.size())
                    V[idx] = pNode->m_Velocity[i];
                if (i < pNode->m_Acceleration.size())
                    A[idx] = pNode->m_Acceleration[i];
            }
        }
    }
}

// ==========================================
// IAnalysisModel 接口实现
// ==========================================

bool AnalysisStep::ShouldPreferGpuLinearSolver() const
{
    if (!m_pData)
        return true;
    int cableCount = 0;
    bool containsOtherElement = false;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (std::dynamic_pointer_cast<ElementCable>(element))
            ++cableCount;
        else
            containsOtherElement = true;
    }
    return containsOtherElement || cableCount >= SolverNameSpace::GpuSettings::MinimumGpuCableElements();
}

std::uint64_t AnalysisStep::GetSparsePatternVersion() const
{
    return m_sparsePatternVersion;
}

bool AnalysisStep::HasStateDependentMass() const
{
    const StructureData* data = m_pData ? m_pData : GetStructure();
    if (!data || !data->m_RigidBodyInertias.empty())
        return true;
    for (const auto& [elementId, element] : data->m_Elements)
    {
        Q_UNUSED(elementId);
        if (std::dynamic_pointer_cast<ElementTruss>(element))
            continue;
        const auto cable = std::dynamic_pointer_cast<ElementCable>(element);
        if (!cable)
            return true;
        std::vector<int> localDofCounts;
        cable->GetNodeLocalDOFCounts(localDofCounts);
        if (std::any_of(localDofCounts.cbegin(), localDofCounts.cend(), [](int dofCount) { return dofCount != 4; }))
            return true;
        const auto property = cable->m_pProperty.lock();
        const auto section = property ? property->m_pSection.lock() : nullptr;
        // 每节点仅四个 DOF 时，偏心截面的平动/扭转质量耦合仍随试探扭转变化。
        // 因此显式 RK 各阶段必须重新装配并分解质量矩阵。
        if (!section || section->m_MassFirstMomentPerLength.squaredNorm() > 0.0)
            return true;
    }
    return false;
}

QString AnalysisStep::DescribeFreeDof(int freeDof) const
{
    if (!m_pData || freeDof < 0 || freeDof >= m_nFree)
        return QStringLiteral("free_dof=%1").arg(freeDof);
    const int globalDof = m_nFixed + freeDof;
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node)
            continue;
        for (int direction = 0; direction < node->m_DOF.size(); ++direction)
            if (node->m_DOF[direction] == globalDof)
                return QStringLiteral("free_dof=%1 node=%2 direction=%3").arg(freeDof).arg(nodeId).arg(direction);
    }
    return QStringLiteral("free_dof=%1 global_dof=%2").arg(freeDof).arg(globalDof);
}

std::vector<bool> AnalysisStep::GetRotationalFreeDofs() const
{
    std::vector<bool> torsionDofs(static_cast<std::size_t>(m_nFree), false);
    if (!m_pData)
        return torsionDofs;
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        if (!node)
            continue;
        for (int component = 3; component < node->m_DOF.size(); ++component)
        {
            const int freeDof = node->m_DOF[component] - m_nFixed;
            if (freeDof >= 0 && freeDof < m_nFree)
                torsionDofs[static_cast<std::size_t>(freeDof)] = true;
        }
    }
    return torsionDofs;
}

bool AnalysisStep::PrepareStructuralDamping(_OUT QString& errorMessage)
{
    QElapsedTimer calibrationTimer;
    calibrationTimer.start();
    InvalidateLocalRayleighCache();
    m_localRayleighCacheGroups.clear();
    m_localRayleighGroupLabels.clear();
    m_localRayleighElementGroups.clear();
    m_localRayleighInertiaGroups.clear();
    m_StructuralDamping.Reset();
    m_localRayleighDampingReady = false;
    const SolverNameSpace::StructuralDampingSettings& settings = m_StructuralDamping.settings;
    if (settings.useExplicitRayleighCoefficients)
    {
        const std::array<double, 4> coefficients{settings.translationMassCoefficient,
                                                 settings.translationStiffnessCoefficient,
                                                 settings.torsionMassCoefficient, settings.torsionStiffnessCoefficient};
        if (!std::all_of(coefficients.cbegin(), coefficients.cend(),
                         [](double coefficient)
                         {
                             return std::isfinite(coefficient) && coefficient >= 0.0;
                         }))
        {
            errorMessage = QStringLiteral("局部 Rayleigh 阻尼系数必须是非负有限数值。");
            return false;
        }
        auto& report = m_StructuralDamping.report;
        report.originalDofCount = m_nFree;
        report.translationMassCoefficient = coefficients[0];
        report.translationStiffnessCoefficient = coefficients[1];
        report.torsionMassCoefficient = coefficients[2];
        report.torsionStiffnessCoefficient = coefficients[3];
        report.summary = QStringLiteral("局部 Rayleigh 系数已设置。");
        m_localRayleighDampingReady = true;
        return true;
    }
    SpMat stiffness;
    SpMat mass;
    Assemble_Matrix(stiffness, false);
    AssembleMassMatrix(mass);
    SpMat modalTransformation(m_nFree, m_nFree);
    modalTransformation.setIdentity();
    SolverNameSpace::NonlinearMPCData constraints;
    if (!AssembleNonlinearMPC(constraints))
    {
        errorMessage = QStringLiteral("结构阻尼无法组装当前平衡构型的 MPC 约束。");
        return false;
    }
    if (!constraints.Empty())
    {
        SolverNameSpace::NonlinearMPCReduction reduction;
        if (!SolverNameSpace::NonlinearMPC::Reduce(stiffness, VectorXd::Zero(m_nFree), constraints, reduction))
        {
            errorMessage = QStringLiteral("结构阻尼无法在当前平衡构型上约化 MPC 自由度。");
            return false;
        }
        modalTransformation = reduction.masterTransformation;
    }
    SolverNameSpace::ConductorModalMetrics modalMetrics;
    if (!SolverNameSpace::ConductorModalClassifier::Build(*m_pData, m_nFixed, m_nFree, modalMetrics, errorMessage,
                                                          [this](int stepId)
                                                          {
                                                              return IsStepScopedDataActive(stepId);
                                                          }))
        return false;
    std::function<double(const VectorXd&)> classifyMode;
    SpMat torsionMetric;
    if (modalMetrics.conductorCount > 0)
    {
        classifyMode = [&modalMetrics, &mass](const VectorXd& shape)
        {
            return modalMetrics.TorsionFraction(shape, mass);
        };
        torsionMetric = modalMetrics.bundleTorsion + modalMetrics.axialTwist;
    }
    std::array<SpMat, 4> localDampingBasis;
    for (int component = 0; component < 4; ++component)
    {
        std::array<double, 4> coefficients{};
        coefficients[component] = 1.0;
        BuildLocalRayleighDamping(coefficients, localDampingBasis[component]);
    }
    std::vector<SolverNameSpace::LocalRayleighGroupBasis> groupedBasis;
    if (modalMetrics.bundleSpanCount == 1 && modalMetrics.skippedBundleCount == 0 &&
        !BuildLocalRayleighGroupBasis(groupedBasis, errorMessage))
        return false;
    if (!m_StructuralDamping.Calculate(stiffness, mass, localDampingBasis, GetRotationalFreeDofs(), modalTransformation,
                                       errorMessage, classifyMode, torsionMetric, groupedBasis))
        return false;
    m_localRayleighDampingReady = true;
    SpMat damping;
    AssembleLocalRayleighDamping(damping);

    if (!m_StructuralDamping.ValidateRayleighMatrix(mass, damping, modalTransformation, errorMessage))
    {
        m_localRayleighDampingReady = false;
        return false;
    }
    m_StructuralDamping.report.summary += QStringLiteral("\n") + modalMetrics.summary;
    QString calibrationDetails;
    QTextStream calibrationTrace(&calibrationDetails);
    calibrationTrace << "YQY_DAMPING_INITIALIZATION calibration_basis=equilibrium-current"
                     << " elapsed_ms=" << calibrationTimer.elapsed() << Qt::endl;
    Logger::Instance().Diagnostic(calibrationDetails.trimmed());
    return true;
}

bool AnalysisStep::HasActiveLocalRayleighDamping() const
{
    if (!m_localRayleighDampingReady)
        return false;
    const auto& groups = m_StructuralDamping.report.localGroups;
    if (!groups.empty())
    {
        for (const auto& group : groups)
            for (double coefficient : group.coefficients)
                if (coefficient != 0.0)
                    return true;
        return false;
    }
    const auto coefficients = GetLocalRayleighCoefficients();
    return std::any_of(coefficients.cbegin(), coefficients.cend(),
                       [](double value)
                       {
                           return value != 0.0;
                       });
}

std::array<double, 4> AnalysisStep::GetLocalRayleighCoefficients() const
{
    const auto& report = m_StructuralDamping.report;
    return {report.translationMassCoefficient, report.translationStiffnessCoefficient, report.torsionMassCoefficient,
            report.torsionStiffnessCoefficient};
}

std::array<double, 4> AnalysisStep::GetLocalRayleighGroupCoefficients(std::size_t groupIndex) const
{
    const auto& groups = m_StructuralDamping.report.localGroups;
    if (groupIndex >= groups.size() || groupIndex >= m_localRayleighGroupLabels.size() ||
        groups[groupIndex].label != m_localRayleighGroupLabels[groupIndex])
        throw std::logic_error("局部 Rayleigh 单元分组与标定报告不一致");
    return groups[groupIndex].coefficients;
}

std::array<double, 4> AnalysisStep::GetElementRayleighCoefficients(const ElementBase& element) const
{
    const auto group = m_localRayleighElementGroups.find(element.m_Id);
    if (!m_StructuralDamping.report.localGroups.empty() && group != m_localRayleighElementGroups.cend())
        return GetLocalRayleighGroupCoefficients(group->second);
    return GetLocalRayleighCoefficients();
}

std::array<double, 4> AnalysisStep::GetRigidBodyRayleighCoefficients(const RigidBodyInertia& inertia) const
{
    const auto group = m_localRayleighInertiaGroups.find(inertia.m_Id);
    if (!m_StructuralDamping.report.localGroups.empty() && group != m_localRayleighInertiaGroups.cend())
        return GetLocalRayleighGroupCoefficients(group->second);
    return GetLocalRayleighCoefficients();
}

QString AnalysisStep::LocalRayleighElementGroupLabel(const ElementBase& element, int zone) const
{
    QString type;
    if (dynamic_cast<const ElementCable*>(&element))
        type = QStringLiteral("Cable");
    else if (dynamic_cast<const ElementBeam_CR*>(&element))
        type = QStringLiteral("BeamCR");
    else if (dynamic_cast<const ElementBeam_Simo*>(&element))
        type = QStringLiteral("BeamSimo");
    else if (dynamic_cast<const ElementTruss*>(&element))
        type = QStringLiteral("Truss");
    else
        type = QString::fromLatin1(typeid(element).name());
    const auto property = element.m_pProperty.lock();
    return QStringLiteral("%1/property=%2/role=%3/zone=%4")
        .arg(type)
        .arg(property ? property->m_Id : -1)
        .arg(static_cast<int>(element.m_Role))
        .arg(zone + 1);
}

std::size_t AnalysisStep::AddLocalRayleighGroup(const QString& label,
                                                _OUT std::vector<SolverNameSpace::LocalRayleighGroupBasis>& groups)
{
    const auto found = std::find(m_localRayleighGroupLabels.cbegin(), m_localRayleighGroupLabels.cend(), label);
    if (found != m_localRayleighGroupLabels.cend())
        return static_cast<std::size_t>(found - m_localRayleighGroupLabels.cbegin());
    const std::size_t index = groups.size();
    groups.emplace_back();
    groups.back().label = label;
    for (SpMat& matrix : groups.back().basis)
    {
        matrix = m_assemblyPattern.matrix;
        std::fill(matrix.valuePtr(), matrix.valuePtr() + matrix.nonZeros(), 0.0);
    }
    m_localRayleighGroupLabels.push_back(label);
    return index;
}

bool AnalysisStep::BuildLocalRayleighGroupBasis(_OUT std::vector<SolverNameSpace::LocalRayleighGroupBasis>& groups,
                                                _OUT QString& errorMessage)
{
    EnsureAssemblyPattern();
    std::map<int, Eigen::Vector3d> referenceNodes;
    std::set<int> wireIds;
    int bundleCount = 0;
    for (const ElementAssemblyMap& map : m_assemblyPattern.elements)
    {
        const auto* cable = dynamic_cast<const ElementCable*>(map.element.get());
        if (!cable)
            continue;
        if (cable->m_Role != ElementRole::Conductor || cable->m_WireId < 0 || cable->m_AeroBundleCount <= 1)
            return true;
        if (bundleCount != 0 && bundleCount != cable->m_AeroBundleCount)
            return true;
        bundleCount = cable->m_AeroBundleCount;
        wireIds.insert(cable->m_WireId);
        for (const auto& weakNode : cable->m_pNode)
        {
            const auto node = weakNode.lock();
            if (!node)
            {
                errorMessage = QStringLiteral("局部 Rayleigh 分组的导线节点引用无效。");
                return false;
            }
            referenceNodes[node->m_Id] = Eigen::Vector3d(node->m_X, node->m_Y, node->m_Z);
        }
    }
    if (bundleCount <= 1 || wireIds.size() != static_cast<std::size_t>(bundleCount) || referenceNodes.size() < 4)
        return true;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto& [id, position] : referenceNodes)
    {
        Q_UNUSED(id);
        if (!position.allFinite())
        {
            errorMessage = QStringLiteral("局部 Rayleigh 分组的参考坐标包含非有限值。");
            return false;
        }
        center += position;
    }
    center /= static_cast<double>(referenceNodes.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& [id, position] : referenceNodes)
    {
        Q_UNUSED(id);
        const Eigen::Vector3d relative = position - center;
        covariance.noalias() += relative * relative.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
    {
        errorMessage = QStringLiteral("局部 Rayleigh 分组无法确定参考导线跨的主轴。");
        return false;
    }
    if (solver.eigenvalues()[2] <= 1.0e-16 || solver.eigenvalues()[2] < 4.0 * solver.eigenvalues()[1])
        return true;
    const Eigen::Vector3d axis = solver.eigenvectors().col(2);
    double lower = std::numeric_limits<double>::infinity();
    double upper = -std::numeric_limits<double>::infinity();
    for (const auto& [id, position] : referenceNodes)
    {
        Q_UNUSED(id);
        const double coordinate = axis.dot(position - center);
        lower = std::min(lower, coordinate);
        upper = std::max(upper, coordinate);
    }
    const double length = upper - lower;
    if (!std::isfinite(length) || length <= 1.0e-10)
        return true;
    // 分组使用参考主轴上的对称位置，动力过程中成员和系数保持固定。
    constexpr int zoneCount = 8;
    const auto zoneOf = [&axis, &center, lower, upper, length](const Eigen::Vector3d& position)
    {
        if (!position.allFinite())
            throw std::runtime_error("局部 Rayleigh 分组的参考位置包含非有限值");
        const double coordinate = axis.dot(position - center);
        const double symmetricPosition = 2.0 * std::min(coordinate - lower, upper - coordinate) / length;
        return std::clamp(static_cast<int>(std::floor(symmetricPosition * zoneCount + 1.0e-10)), 0, zoneCount - 1);
    };
    try
    {
        groups.reserve(m_assemblyPattern.elements.size() + m_assemblyPattern.rigidBodyInertias.size());
        MatrixXd stiffness;
        MatrixXd mass;
        MatrixXd damping;
        for (const ElementAssemblyMap& map : m_assemblyPattern.elements)
        {
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            if (map.element->m_pNode.empty())
                throw std::runtime_error("局部 Rayleigh 分组的单元没有节点");
            for (const auto& weakNode : map.element->m_pNode)
            {
                const auto node = weakNode.lock();
                if (!node)
                    throw std::runtime_error("局部 Rayleigh 分组的单元节点引用无效");
                position += Eigen::Vector3d(node->m_X, node->m_Y, node->m_Z);
            }
            position /= static_cast<double>(map.element->m_pNode.size());
            const std::size_t group =
                AddLocalRayleighGroup(LocalRayleighElementGroupLabel(*map.element, zoneOf(position)), groups);
            m_localRayleighElementGroups[map.element->m_Id] = group;
            map.element->Get_ke(stiffness);
            map.element->Get_me_Physical(mass);
            for (int component = 0; component < 4; ++component)
            {
                std::array<double, 4> coefficients{};
                coefficients[component] = 1.0;
                map.element->GetRayleighDamping(coefficients, stiffness, mass, damping);
                AccumulateElementMatrix(map, damping, groups[group].basis[component].valuePtr());
            }
        }
        for (const RigidBodyInertiaAssemblyMap& map : m_assemblyPattern.rigidBodyInertias)
        {
            const auto node = map.inertia->m_pNode.lock();
            if (!node)
                throw std::runtime_error("局部 Rayleigh 分组的刚体惯性节点引用无效");
            const Eigen::Vector3d position(node->m_X, node->m_Y, node->m_Z);
            const std::size_t group =
                AddLocalRayleighGroup(QStringLiteral("RigidBodyInertia/zone=%1").arg(zoneOf(position) + 1), groups);
            m_localRayleighInertiaGroups[map.inertia->m_Id] = group;
            BuildRigidBodyRayleighDamping(*map.inertia, {1.0, 0.0, 0.0, 0.0}, damping);
            AccumulateMatrixValues(map.valueIndices, static_cast<int>(map.dofs.size()), damping,
                                   groups[group].basis[0].valuePtr());
        }
        return true;
    }
    catch (const std::exception& exception)
    {
        groups.clear();
        m_localRayleighGroupLabels.clear();
        m_localRayleighElementGroups.clear();
        m_localRayleighInertiaGroups.clear();
        errorMessage = QStringLiteral("局部 Rayleigh 分组基矩阵组装失败：%1").arg(QString::fromUtf8(exception.what()));
        return false;
    }
}

void AnalysisStep::BuildRigidBodyRayleighDamping(const RigidBodyInertia& inertia,
                                                 const std::array<double, 4>& coefficients,
                                                 _OUT MatrixXd& damping) const
{
    MatrixXd mass;
    inertia.GetPhysicalMass(mass);
    const MatrixXd stiffness = MatrixXd::Zero(mass.rows(), mass.cols());
    SolverNameSpace::BuildProportionalRayleighMatrix(stiffness, mass, coefficients[0], 0.0, damping);
}

void AnalysisStep::BuildLocalRayleighDamping(const std::array<double, 4>& coefficients, _OUT SpMat& damping,
                                             bool useGroupCoefficients)
{
    EnsureAssemblyPattern();
    PreparePatternMatrix(damping);
    MatrixXd elementDamping;
    for (const ElementAssemblyMap& assemblyMap : m_assemblyPattern.elements)
    {
        const auto localCoefficients =
            useGroupCoefficients ? GetElementRayleighCoefficients(*assemblyMap.element) : coefficients;
        assemblyMap.element->GetRayleighDamping(localCoefficients, elementDamping);
        AccumulateElementMatrix(assemblyMap, elementDamping, damping.valuePtr());
    }
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        const auto localCoefficients =
            useGroupCoefficients ? GetRigidBodyRayleighCoefficients(*assemblyMap.inertia) : coefficients;
        BuildRigidBodyRayleighDamping(*assemblyMap.inertia, localCoefficients, elementDamping);
        AccumulateMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()), elementDamping,
                               damping.valuePtr());
    }
}

const SpMat& AnalysisStep::GetLocalRayleighDamping()
{
    const auto coefficients = GetLocalRayleighCoefficients();
    const auto& groups = m_StructuralDamping.report.localGroups;
    const bool sameGroups = groups.size() == m_localRayleighCacheGroups.size() &&
                            std::equal(groups.cbegin(), groups.cend(), m_localRayleighCacheGroups.cbegin(),
                                       [](const auto& left, const auto& right)
                                       {
                                           return left.label == right.label && left.coefficients == right.coefficients;
                                       });
    if (!m_localRayleighCacheValid || coefficients != m_localRayleighCacheCoefficients || !sameGroups)
    {
        BuildLocalRayleighDamping(coefficients, m_localRayleighAssembledCache, true);
        m_localRayleighCacheCoefficients = coefficients;
        m_localRayleighCacheGroups = groups;
        m_localRayleighCacheValid = true;
    }
    return m_localRayleighAssembledCache;
}

double AnalysisStep::ComputeLocalRayleighForce(_OUT VectorXd& force)
{
    const SpMat& damping = GetLocalRayleighDamping();
    const VectorXd velocity = GetCurrentVelocity();
    force = damping * velocity;
    const double power = velocity.dot(force);
    const double scale = velocity.norm() * force.norm();
    if (!force.allFinite() || !std::isfinite(power) || !std::isfinite(scale))
        throw std::runtime_error("Current structural damping force or dissipation is non-finite");
    const double tolerance = 1.0e-10 * std::max(1.0, scale);
    if (power < -tolerance)
    {
        const QString errorMessage =
            QStringLiteral("当前结构阻尼产生负耗散：v^T C v=%1，容许误差=%2；请检查当前切线刚度。")
                .arg(power, 0, 'g', 12)
                .arg(tolerance, 0, 'g', 12);
        throw std::runtime_error(errorMessage.toStdString());
    }
    return power;
}

void AnalysisStep::ComputeLocalRayleighFixedForce(_OUT VectorXd& fixedForce) const
{
    fixedForce = VectorXd::Zero(m_nFixed);
    if (!HasActiveLocalRayleighDamping() || m_nFixed == 0)
        return;
    const VectorXd freeVelocity = GetCurrentVelocity();
    const auto accumulate = [this, &freeVelocity, &fixedForce](const std::vector<int>& dofs, const MatrixXd& damping)
    {
        VectorXd velocity = VectorXd::Zero(static_cast<int>(dofs.size()));
        for (int column = 0; column < velocity.size(); ++column)
        {
            const int freeDof = dofs[column] - m_nFixed;
            if (freeDof >= 0 && freeDof < m_nFree)
                velocity[column] = freeVelocity[freeDof];
        }
        const VectorXd force = damping * velocity;
        for (int row = 0; row < force.size(); ++row)
            if (dofs[row] >= 0 && dofs[row] < m_nFixed)
                fixedForce[dofs[row]] += force[row];
    };
    std::vector<int> dofs;
    MatrixXd damping;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (!element)
            continue;
        element->GetDOFs(dofs);
        if (std::none_of(dofs.cbegin(), dofs.cend(),
                         [this](int dof)
                         {
                             return dof >= 0 && dof < m_nFixed;
                         }))
            continue;
        element->GetRayleighDamping(GetElementRayleighCoefficients(*element), damping);
        accumulate(dofs, damping);
    }
    for (const auto& [inertiaId, inertia] : m_pData->m_RigidBodyInertias)
    {
        Q_UNUSED(inertiaId);
        if (!inertia)
            continue;
        inertia->GetDOFs(dofs);
        if (std::none_of(dofs.cbegin(), dofs.cend(),
                         [this](int dof)
                         {
                             return dof >= 0 && dof < m_nFixed;
                         }))
            continue;
        BuildRigidBodyRayleighDamping(*inertia, GetRigidBodyRayleighCoefficients(*inertia), damping);
        accumulate(dofs, damping);
    }
}

void AnalysisStep::AddLocalRayleighTangent(double velocityDerivative, _OUT SpMat& tangent)
{
    if (!HasActiveLocalRayleighDamping() || velocityDerivative == 0.0)
        return;
    // 采用 C(q) * dv/dq 的拟牛顿切线，残差中的 C 在每次构型更新后重建。
    tangent += velocityDerivative * GetLocalRayleighDamping();
    tangent.makeCompressed();
    m_gpuTrussCscValuesReady = false;
    m_gpuCableCscValuesReady = false;
    m_gpuBeamCscValuesReady = false;
    m_gpuMixedCscValuesReady = false;
}

void AnalysisStep::InvalidateLocalRayleighCache()
{
    m_localRayleighCacheValid = false;
}

bool AnalysisStep::UsesReferenceRhsOrder() const
{
    return m_Type == EnumKeyword::StepType::DYNAMIC &&
           m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN &&
           !m_referenceNormSignedDofs.empty();
}

void AnalysisStep::InvalidateReferenceRhsCache()
{
    if (!UsesReferenceRhsOrder())
        return;
    ++m_referenceRhsStateVersion;
    m_referenceOrderedResidual.resize(0);
}

void AnalysisStep::CacheReferenceInertia(const ElementAssemblyMap& assemblyMap,
                                         const Eigen::VectorXd& inertiaForce) const
{
    if (!UsesReferenceRhsOrder())
        return;
    assemblyMap.referenceInertiaForce = inertiaForce;
    assemblyMap.referenceInertiaStateVersion = m_referenceRhsStateVersion;
}

void AnalysisStep::CacheReferenceInertia(const RigidBodyInertiaAssemblyMap& assemblyMap,
                                         const Eigen::VectorXd& inertiaForce) const
{
    if (!UsesReferenceRhsOrder())
        return;
    assemblyMap.referenceInertiaForce = inertiaForce;
    assemblyMap.referenceInertiaStateVersion = m_referenceRhsStateVersion;
}

void AnalysisStep::AssembleLocalRayleighDamping(_OUT SpMat& damping)
{
    if (!HasActiveLocalRayleighDamping())
    {
        EnsureAssemblyPattern();
        PreparePatternMatrix(damping);
        return;
    }
    damping = GetLocalRayleighDamping();
}

void AnalysisStep::ApplyIncrement(const SolverNameSpace::Vec& dx)
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    // 将位移增量应用到节点
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        if (numDOF == 6) // 针对 6 自由度空间节点
        {
            for (int dofIdx = 0; dofIdx < 3; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    int idx = dof - m_nFixed;
                    pNode->ApplyAdditiveIncrement(dofIdx, dx[idx]);
                }
            }

            // 提取本次迭代产生的微小旋转增量 delta_theta
            Eigen::Vector3d delta_theta = Eigen::Vector3d::Zero();
            for (int dofIdx = 3; dofIdx < 6; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    int idx = dof - m_nFixed;
                    delta_theta(dofIdx - 3) = dx[idx];
                }
            }

            Eigen::Matrix3d R_temp;
            Utility::CR::Update_NodalRotation(delta_theta, pNode->m_Rg, R_temp);
            pNode->m_Rg = R_temp;
            Eigen::Vector3d theta;
            Utility::CR::Extract_RotationVector(R_temp, theta);
            pNode->m_Displacement[3] = theta(0);
            pNode->m_Displacement[4] = theta(1);
            pNode->m_Displacement[5] = theta(2);
        }
        else // 对于非 6 自由度的普通节点（保留你原有的逻辑）
        {

            for (int dofIdx = 0; dofIdx < numDOF; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    int idx = dof - m_nFixed;
                    pNode->ApplyAdditiveIncrement(dofIdx, dx[idx]);
                }
            }
        }
    }
}

void AnalysisStep::BeginDynamicStep(double dt, double beta, double gamma)
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        std::array<bool, 3> translationActive{false, false, false};
        std::array<bool, 3> rotationActive{false, false, false};

        for (int i = 0; i < pNode->m_DOF.size() && i < 6; ++i)
        {
            const int dof = pNode->m_DOF[i];
            const bool isFree = dof >= m_nFixed && dof < m_nFixed + m_nFree;
            if (i < 3)
                translationActive[i] = isFree;
            else
                rotationActive[i - 3] = isFree;
        }

        pNode->BeginNewmarkStep(dt, beta, gamma, translationActive, rotationActive);
    }
}

void AnalysisStep::ApplyDynamicCorrection(const SolverNameSpace::Vec& dx, double a0, double a1)
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        Eigen::Vector3d deltaTranslation = Eigen::Vector3d::Zero();
        Eigen::Vector3d deltaRotation = Eigen::Vector3d::Zero();

        for (int i = 0; i < pNode->m_DOF.size() && i < 6; ++i)
        {
            const int dof = pNode->m_DOF[i];
            if (dof < m_nFixed || dof >= m_nFixed + m_nFree)
                continue;

            const double correction = dx[dof - m_nFixed];
            if (i < 3)
                deltaTranslation(i) = correction;
            else
                deltaRotation(i - 3) = correction;
        }

        pNode->ApplyNewmarkCorrection(deltaTranslation, deltaRotation, a0, a1);
    }
}

void AnalysisStep::RollbackDynamicStep()
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    for (auto& nodePair : m_pData->m_Nodes)
    {
        nodePair.second->RollbackNewmarkStep();
    }
    for (auto& elementPair : m_pData->m_Elements)
    {
        if (elementPair.second)
            elementPair.second->RestoreState();
    }
}

void AnalysisStep::SetTrialKinematics(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a)
{
    SetTrialKinematicsImpl(v, a, false);
}

void AnalysisStep::SetTssbnTrialKinematics(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a)
{
    SetTrialKinematicsImpl(v, a, true);
}

void AnalysisStep::SetTrialKinematicsImpl(const SolverNameSpace::Vec& v, const SolverNameSpace::Vec& a,
                                          bool preserveStepRotation)
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    // 将速度和加速度设置到节点
    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        // 确保向量大小足够
        if (pNode->m_Velocity.size() < numDOF)
            pNode->m_Velocity.resize(numDOF, 0.0);
        if (pNode->m_Acceleration.size() < numDOF)
            pNode->m_Acceleration.resize(numDOF, 0.0);

        for (int dofIdx = 0; dofIdx < numDOF; ++dofIdx)
        {
            int dof = pNode->m_DOF[dofIdx];

            if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
            {
                int idx = dof - m_nFixed;
                pNode->m_Velocity[dofIdx] = v[idx];
                pNode->m_Acceleration[dofIdx] = a[idx];
            }
        }

        if (numDOF >= 6)
        {
            Eigen::Vector3d spatialVelocity = Eigen::Vector3d::Zero();
            Eigen::Vector3d spatialAcceleration = Eigen::Vector3d::Zero();
            for (int component = 0; component < 3; ++component)
            {
                spatialVelocity[component] = pNode->m_Velocity[component + 3];
                spatialAcceleration[component] = pNode->m_Acceleration[component + 3];
            }
            pNode->m_OmegaMaterial = pNode->m_Rg.transpose() * spatialVelocity;
            pNode->m_AlphaMaterial = pNode->m_Rg.transpose() * spatialAcceleration;
            if (!preserveStepRotation)
                Utility::CR::Extract_RotationVector(pNode->m_Rg * pNode->m_Rg_n.transpose(), pNode->m_StepRotation);
        }
    }
}

void AnalysisStep::SetTssbnStageKinematics(int stageIndex, double timeStep, double firstStageTime,
                                           double secondStageTime, double secondStageDiagonalFraction,
                                           _OUT SolverNameSpace::Vec& velocity, _OUT SolverNameSpace::Vec& acceleration,
                                           bool useStoredIncrement)
{
    SetTrialKinematicsImpl(velocity, acceleration, useStoredIncrement);
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node || node->m_DOF.size() < 6)
            continue;
        Eigen::Vector3d angularVelocity;
        Eigen::Vector3d angularAcceleration;
        node->SetTssbnStageKinematics(stageIndex, timeStep, firstStageTime, secondStageTime,
                                      secondStageDiagonalFraction, angularVelocity, angularAcceleration,
                                      useStoredIncrement);
        for (int component = 0; component < 3; ++component)
        {
            const int globalDof = node->m_DOF[component + 3];
            if (globalDof < m_nFixed || globalDof >= m_nFixed + m_nFree)
            {
                continue;
            }
            const int freeDof = globalDof - m_nFixed;
            velocity[freeDof] = angularVelocity[component];
            acceleration[freeDof] = angularAcceleration[component];
        }
    }
}

void AnalysisStep::BeginTssbnImplicitStage(int stageIndex, double timeStep, double firstStageTime,
                                           double secondStageTime, double secondStageDiagonalFraction)
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    for (auto& [nodeId, node] : m_pData->m_Nodes)
        if (node && node->m_DOF.size() >= 6)
            node->BeginTssbnImplicitStage(stageIndex, timeStep, firstStageTime, secondStageTime,
                                          secondStageDiagonalFraction);
}

void AnalysisStep::ApplyTssbnImplicitStageCorrection(const SolverNameSpace::Vec& increment)
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node)
            continue;
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
        for (int component = 0; component < node->m_DOF.size() && component < 6; ++component)
        {
            const int dof = node->m_DOF[component];
            if (dof < m_nFixed || dof >= m_nFixed + m_nFree)
                continue;
            if (component < 3)
                translation[component] = increment[dof - m_nFixed];
            else
                rotation[component - 3] = increment[dof - m_nFixed];
        }
        node->ApplyTssbnImplicitStageCorrection(translation, rotation);
    }
}

bool AnalysisStep::NormalizeTssbnAcceptedRotations()
{
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
        if (!node || !node->IsTssbnAcceptedRotationValid())
            return false;
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
        if (!node->NormalizeTssbnAcceptedRotation())
            return false;
    return true;
}

void AnalysisStep::CorrectTssbnExplicitLastStage(double timeStep, double lastStageFirstCoefficient,
                                                 double lastStageSecondCoefficient,
                                                 _OUT SolverNameSpace::Vec& stepIncrement,
                                                 _OUT SolverNameSpace::Vec& velocity)
{
    InvalidateLocalRayleighCache();
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node || node->m_DOF.size() < 6)
            continue;
        const Node::TssbnExplicitRotationState rotation =
            node->IntegrateTssbnExplicitLastStage(timeStep, lastStageFirstCoefficient, lastStageSecondCoefficient);
        for (int component = 0; component < 3; ++component)
        {
            const int globalDof = node->m_DOF[component + 3];
            if (globalDof < m_nFixed || globalDof >= m_nFixed + m_nFree)
                continue;
            const int freeDof = globalDof - m_nFixed;
            stepIncrement[freeDof] = rotation.spatialIncrement[component];
            velocity[freeDof] = rotation.spatialVelocity[component];
        }
    }
}

void AnalysisStep::CorrectTssbnStepStates(
    double timeStep, double firstStageTime, double secondStageTime, double lastStageTime, double baseFirstWeight,
    double embeddedFirstWeight, double embeddedSecondWeight, double embeddedLastWeight,
    double lastStageFirstCoefficient, double lastStageSecondCoefficient, _OUT SolverNameSpace::Vec& baseIncrement,
    _OUT SolverNameSpace::Vec& baseVelocity, _OUT SolverNameSpace::Vec& embeddedIncrement,
    _OUT SolverNameSpace::Vec& embeddedVelocity, _OUT SolverNameSpace::Vec& acceptedAcceleration)
{
    InvalidateLocalRayleighCache();
    const double stageSeparation = secondStageTime - firstStageTime;
    const double extrapolation = (lastStageTime - secondStageTime) / stageSeparation;
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node || node->m_DOF.size() < 6)
            continue;

        const Node::TssbnRotationState rotation = node->IntegrateTssbnRotation(
            timeStep, extrapolation, baseFirstWeight, embeddedFirstWeight, embeddedSecondWeight, embeddedLastWeight,
            lastStageFirstCoefficient, lastStageSecondCoefficient);

        for (int component = 0; component < 3; ++component)
        {
            const int globalDof = node->m_DOF[component + 3];
            if (globalDof < m_nFixed || globalDof >= m_nFixed + m_nFree)
            {
                continue;
            }
            const int freeDof = globalDof - m_nFixed;
            baseIncrement[freeDof] = rotation.baseSpatialIncrement[component];
            embeddedIncrement[freeDof] = rotation.embeddedSpatialIncrement[component];
            baseVelocity[freeDof] = rotation.baseSpatialVelocity[component];
            embeddedVelocity[freeDof] = rotation.embeddedSpatialVelocity[component];
            acceptedAcceleration[freeDof] = rotation.acceptedSpatialAcceleration[component];
        }
    }
}

bool AnalysisStep::CorrectReferenceTssbnErrorDisplacements(
    double timeStep, double baseFirstWeight, double embeddedFirstWeight, double embeddedSecondWeight,
    double embeddedLastWeight, double lastStageFirstCoefficient, double lastStageSecondCoefficient,
    _OUT SolverNameSpace::Vec& stepStartDisplacement, _OUT SolverNameSpace::Vec& baseDisplacement,
    _OUT SolverNameSpace::Vec& embeddedDisplacement) const
{
    if (!m_pData || stepStartDisplacement.size() != m_nFree || baseDisplacement.size() != m_nFree ||
        embeddedDisplacement.size() != m_nFree)
        return false;
    bool corrected = false;
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        if (!node || node->m_DOF.size() != 6)
            continue;
        const Node::TssbnReferenceErrorState rotation = node->GetReferenceTssbnErrorState(
            timeStep, baseFirstWeight, embeddedFirstWeight, embeddedSecondWeight, embeddedLastWeight,
            lastStageFirstCoefficient, lastStageSecondCoefficient);
        for (int component = 0; component < 3; ++component)
        {
            const int freeDof = node->m_DOF[component + 3] - m_nFixed;
            if (freeDof < 0 || freeDof >= m_nFree)
                continue;
            stepStartDisplacement[freeDof] = rotation.stepStartRotationVector[component];
            baseDisplacement[freeDof] = rotation.baseMaterialIncrement[component];
            embeddedDisplacement[freeDof] = rotation.embeddedMaterialIncrement[component];
            corrected = true;
        }
    }
    return corrected;
}

void AnalysisStep::CorrectTssbnEmbeddedVelocity(double timeStep, double embeddedFirstWeight,
                                                double embeddedSecondWeight, double embeddedLastWeight,
                                                double lastStageFirstCoefficient, double lastStageSecondCoefficient,
    const SolverNameSpace::Vec& explicitLastStageAcceleration, _OUT SolverNameSpace::Vec& embeddedVelocity)
{
    InvalidateLocalRayleighCache();
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node || node->m_DOF.size() < 6)
            continue;

        Eigen::Vector3d explicitSpatialAcceleration = Eigen::Vector3d::Zero();
        for (int component = 0; component < 3; ++component)
        {
            const int globalDof = node->m_DOF[component + 3];
            if (globalDof >= m_nFixed && globalDof < m_nFixed + m_nFree)
                explicitSpatialAcceleration[component] = explicitLastStageAcceleration[globalDof - m_nFixed];
            else if (component + 3 < static_cast<int>(node->m_Acceleration.size()))
                explicitSpatialAcceleration[component] = node->m_Acceleration[component + 3];
        }
        const Eigen::Vector3d correctedVelocity = node->IntegrateTssbnEmbeddedVelocity(
            timeStep, embeddedFirstWeight, embeddedSecondWeight, embeddedLastWeight, lastStageFirstCoefficient,
            lastStageSecondCoefficient, explicitSpatialAcceleration);
        for (int component = 0; component < 3; ++component)
        {
            const int globalDof = node->m_DOF[component + 3];
            if (globalDof < m_nFixed || globalDof >= m_nFixed + m_nFree)
                continue;
            embeddedVelocity[globalDof - m_nFixed] = correctedVelocity[component];
        }
    }
}

void AnalysisStep::GetState(SolverNameSpace::Vec& u, SolverNameSpace::Vec& v, SolverNameSpace::Vec& a) const
{
    // 直接调用现有的 Get_CurrentStepState
    Get_CurrentStepState(u, v, a);
}

void AnalysisStep::GetFiniteRotationStates(_OUT std::vector<SolverNameSpace::FiniteRotationState>& states) const
{
    states.clear();
    if (!m_pData)
        return;
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        if (!node || node->m_DOF.size() != 6)
            continue;
        SolverNameSpace::FiniteRotationState state;
        state.rotation = node->m_Rg;
        for (int component = 0; component < 3; ++component)
        {
            const int freeDof = node->m_DOF[component + 3] - m_nFixed;
            if (freeDof >= 0 && freeDof < m_nFree)
                state.freeDofs[component] = freeDof;
        }
        states.push_back(state);
    }
}

void AnalysisStep::GetAuxiliaryFiniteRotationStates(_OUT std::vector<Eigen::Matrix3d>& states) const
{
    states.clear();
    if (!m_pData || m_Type != EnumKeyword::StepType::DYNAMIC)
        return;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(element);
        if (!beam || beam->IsReferenceTssbnDynamicMode())
            continue;
        std::array<Matrix3d, 2> rotations;
        beam->GetCurrentGaussRotations(rotations);
        states.insert(states.end(), rotations.begin(), rotations.end());
    }
}

void AnalysisStep::Assemble_Matrix(SpMat& Keff, bool isDynamic)
{
    Q_UNUSED(isDynamic);
    EnsureAssemblyPattern();
    const std::size_t elementCount = m_assemblyPattern.elements.size();
    const int workerCount = SelectAssemblyWorkerCount();
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Stiffness)};
    if (workerCount > 1 && !ShouldUseColoredAssembly(workerCount))
    {
        const auto& ranges = GetAssemblyRanges(workerCount);
        PrepareAssemblyWorkerBuffers(static_cast<int>(ranges.size()), false);
        const auto assembleRange = [this](int worker, std::size_t first, std::size_t last)
        {
            AssemblyWorkerBuffer& buffer = m_assemblyWorkerBuffers[static_cast<std::size_t>(worker)];
            MatrixXd stiffness;
            for (std::size_t index = first; index < last; ++index)
            {
                const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[index];
                assemblyMap.element->Get_ke(stiffness);
                AppendElementMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                          stiffness, buffer.matrixValues);
            }
        };

        QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
        m_parallelAssemblyEngine.ExecuteRanges(
            request, ranges, threadPool,
            [&assembleRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
            { assembleRange(static_cast<int>(worker), range.first, range.last); });

        PreparePatternMatrix(m_Keff22);
        for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
            MergeAssemblyValues(buffer.matrixValues, m_Keff22.valuePtr());
        Keff = m_Keff22;
        return;
    }

    PreparePatternMatrix(m_Keff22);
    const auto assembleElement = [this, values = m_Keff22.valuePtr()](std::size_t elementIndex)
    {
        const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
        MatrixXd stiffness;
        assemblyMap.element->Get_ke(stiffness);
        AccumulateElementMatrix(assemblyMap, stiffness, values);
    };

    if (workerCount <= 1)
    {
        for (std::size_t elementIndex = 0; elementIndex < elementCount; ++elementIndex)
            assembleElement(elementIndex);
    }
    else
    {
        QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
        std::barrier synchronization(workerCount);
        m_parallelAssemblyEngine.ExecuteWorkers(
            request, static_cast<std::size_t>(workerCount), threadPool,
            [this, &assembleElement, &synchronization, workerCount](std::size_t workerIndex)
            {
                const int worker = static_cast<int>(workerIndex);
                for (const std::vector<std::size_t>& colorGroup : m_assemblyPattern.elementColorGroups)
                {
                    const std::size_t first = colorGroup.size() * static_cast<std::size_t>(worker) /
                                              static_cast<std::size_t>(workerCount);
                    const std::size_t last = colorGroup.size() * static_cast<std::size_t>(worker + 1) /
                                             static_cast<std::size_t>(workerCount);
                    for (std::size_t index = first; index < last; ++index)
                        assembleElement(colorGroup[index]);
                    synchronization.arrive_and_wait();
                }
            });
    }
    Keff = m_Keff22;
}

void AnalysisStep::EvaluateSystem(const SolverNameSpace::SystemEvaluationRequest& request,
                                  SolverNameSpace::SystemEvaluationResult& result)
{
    InvalidateLocalRayleighCache();
    QElapsedTimer timer;
    timer.start();
    const AnalysisPerformanceStatistics before = m_performanceStatistics;
    SolverNameSpace::SystemEvaluationRequest evaluatedRequest = request;
    const bool localDampingRequested = m_localRayleighDampingReady &&
                                       request.Requires(SolverNameSpace::SystemContribution::Damping);
    if (localDampingRequested)
        evaluatedRequest.contributions &= ~SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Damping);
    const auto explicitCpuContributions = SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ResidualState) |
                                          SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ExternalForce) |
                                          SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Mass);
    if (evaluatedRequest.contributions == explicitCpuContributions && !SolverNameSpace::GpuSettings::IsEnabled())
    {
        // Y3 与接受端点同一试探态同时需要 M 和惯性力。索单元惯性力本身已经
        // 计算一致质量矩阵；在此合并两次 CPU 遍历，保持外载先于残差的原有顺序。
        result = {};
        ComputeExternalForce(evaluatedRequest.time, evaluatedRequest.loadFactor, result.constrainedExternalForce,
                             result.externalForce);
        RefreshCpuResidualStateAndMass(result.mass);
        GetDynamicInertiaForce(result.inertiaForce);
        result.sparsePatternVersion = GetSparsePatternVersion();
    }

    else
        SolverNameSpace::IAnalysisModel::EvaluateSystem(evaluatedRequest, result);
    if (localDampingRequested)
        AssembleLocalRayleighDamping(result.damping);
    if (request.Requires(SolverNameSpace::SystemContribution::EffectiveTangent) && m_gpuMixedCscValuesReady)
    {
        result.gpuCscValues = m_gpuMixedCscSum.Values();
        result.gpuCscValueCount = m_gpuMixedCscSum.ValueCount();
    }
    else if (request.Requires(SolverNameSpace::SystemContribution::EffectiveTangent) && m_gpuTrussCscValuesReady)
    {
        result.gpuCscValues = m_gpuTrussBatchEvaluator.DeviceMatrixValues();
        result.gpuCscValueCount = m_gpuTrussBatchEvaluator.DeviceMatrixValueCount();
    }
    else if (request.Requires(SolverNameSpace::SystemContribution::EffectiveTangent) && m_gpuBeamCscValuesReady)
    {
        result.gpuCscValues = m_gpuBeamBatchEvaluator.DeviceMatrixValues();
        result.gpuCscValueCount = m_gpuBeamBatchEvaluator.DeviceMatrixValueCount();
    }
    else if (request.Requires(SolverNameSpace::SystemContribution::EffectiveTangent) && m_gpuCableCscValuesReady)
    {
        result.gpuCscValues = m_gpuCableBatchEvaluator.DeviceMatrixValues();
        result.gpuCscValueCount = m_gpuCableBatchEvaluator.DeviceMatrixValueCount();
    }
    const double inclusiveMilliseconds = static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    const double childMilliseconds =
        m_performanceStatistics.effectiveTangentMilliseconds - before.effectiveTangentMilliseconds +
        m_performanceStatistics.dynamicSystemMilliseconds - before.dynamicSystemMilliseconds +
        m_performanceStatistics.massMatrixMilliseconds - before.massMatrixMilliseconds +
        m_performanceStatistics.inertiaForceMilliseconds - before.inertiaForceMilliseconds +
        m_performanceStatistics.residualStateMilliseconds - before.residualStateMilliseconds +
        m_performanceStatistics.externalTangentMilliseconds - before.externalTangentMilliseconds;
    m_performanceStatistics.systemEvaluationMilliseconds += inclusiveMilliseconds;
    m_performanceStatistics.systemEvaluationExclusiveMilliseconds +=
        std::max(0.0, inclusiveMilliseconds - childMilliseconds);
    ++m_performanceStatistics.systemEvaluationCount;
}

void AnalysisStep::AssembleDynamicSystem(SpMat& mass, SpMat& gyroscopic, SpMat& centrifugal)
{
    QElapsedTimer timer;
    timer.start();
    EnsureAssemblyPattern();
    m_gpuCableCscValuesReady = false;
    m_gpuBeamCscValuesReady = false;
    const int workerCount = SelectAssemblyWorkerCount();
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Mass) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InertiaForce) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::VelocityTangent) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ConfigurationTangent)};
    m_dynamicInertiaForce = VectorXd::Zero(m_nFree);
    const auto& ranges = GetAssemblyRanges(workerCount);
    PrepareDynamicAssemblyWorkerBuffers(static_cast<int>(ranges.size()));
    const auto assembleRange = [this](int worker, std::size_t first, std::size_t last)
    {
        DynamicAssemblyWorkerBuffer& buffer = m_dynamicAssemblyWorkerBuffers[static_cast<std::size_t>(worker)];
        DynamicElementData elementData;
        for (std::size_t index = first; index < last; ++index)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[index];
            EvaluateDynamicElement(assemblyMap, elementData);
            const int dofCount = static_cast<int>(assemblyMap.dofs.size());
            AppendElementMatrixValues(assemblyMap.valueIndices, dofCount, elementData.mass, buffer.massValues);
            AppendElementMatrixValues(assemblyMap.valueIndices, dofCount, elementData.velocityTangent,
                                      buffer.velocityValues);
            AppendElementMatrixValues(assemblyMap.valueIndices, dofCount, elementData.configurationTangent,
                                      buffer.configurationValues);
            CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, buffer.inertiaForce);
        }
    };
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    m_parallelAssemblyEngine.ExecuteRanges(
        request, ranges, threadPool,
        [&assembleRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        { assembleRange(static_cast<int>(worker), range.first, range.last); });

    PreparePatternMatrix(mass);
    PreparePatternMatrix(gyroscopic);
    PreparePatternMatrix(centrifugal);
    for (const DynamicAssemblyWorkerBuffer& buffer : m_dynamicAssemblyWorkerBuffers)
    {
        MergeAssemblyValues(buffer.massValues, mass.valuePtr());
        MergeAssemblyValues(buffer.velocityValues, gyroscopic.valuePtr());
        MergeAssemblyValues(buffer.configurationValues, centrifugal.valuePtr());
        m_dynamicInertiaForce += buffer.inertiaForce;
    }

    DynamicElementData elementData;
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        EvaluateRigidBodyInertia(assemblyMap, elementData);
        const int dofCount = static_cast<int>(assemblyMap.dofs.size());
        AccumulateMatrixValues(assemblyMap.valueIndices, dofCount, elementData.mass, mass.valuePtr());
        AccumulateMatrixValues(assemblyMap.valueIndices, dofCount, elementData.velocityTangent,
                               gyroscopic.valuePtr());
        AccumulateMatrixValues(assemblyMap.valueIndices, dofCount, elementData.configurationTangent,
                               centrifugal.valuePtr());
        CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
        AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, m_dynamicInertiaForce);
    }
    m_performanceStatistics.dynamicSystemMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.dynamicSystemCount;
}

void AnalysisStep::AssembleMassMatrix(SpMat& mass)
{
    QElapsedTimer timer;
    timer.start();
    EnsureAssemblyPattern();
    const int workerCount = SelectAssemblyWorkerCount();
    const auto& ranges = GetAssemblyRanges(workerCount);
    PrepareAssemblyWorkerBuffers(static_cast<int>(ranges.size()), false);
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Mass)};
    const auto assembleRange = [this](int worker, std::size_t first, std::size_t last)
    {
        AssemblyWorkerBuffer& buffer = m_assemblyWorkerBuffers[static_cast<std::size_t>(worker)];
        MatrixXd elementMass;
        for (std::size_t index = first; index < last; ++index)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[index];
            assemblyMap.element->Get_me_Physical(elementMass);
            AppendElementMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                      elementMass, buffer.matrixValues);
        }
    };
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    m_parallelAssemblyEngine.ExecuteRanges(
        request, ranges, threadPool,
        [&assembleRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        { assembleRange(static_cast<int>(worker), range.first, range.last); });

    PreparePatternMatrix(mass);
    for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
        MergeAssemblyValues(buffer.matrixValues, mass.valuePtr());

    DynamicElementData elementData;
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        assemblyMap.inertia->GetPhysicalMass(elementData.mass);
        AccumulateMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()), elementData.mass,
                               mass.valuePtr());
    }
    m_performanceStatistics.massMatrixMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.massMatrixCount;
}

void AnalysisStep::AssembleEffectiveDynamicSystem(double accelerationDerivative, double velocityDerivative,
                                                  SpMat& effectiveDynamicTangent)
{
    AssembleEffectiveDynamicSystemImpl(accelerationDerivative, velocityDerivative, effectiveDynamicTangent);
    AddLocalRayleighTangent(velocityDerivative, effectiveDynamicTangent);
}

void AnalysisStep::AssembleEffectiveDynamicSystemImpl(double accelerationDerivative, double velocityDerivative,
                                                      SpMat& effectiveDynamicTangent)
{
    EnsureAssemblyPattern();
    const int workerCount = SelectAssemblyWorkerCount();
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Mass) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InertiaForce) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::VelocityTangent) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ConfigurationTangent),
        0.0,
        1.0,
        accelerationDerivative,
        velocityDerivative};
    m_dynamicInertiaForce = VectorXd::Zero(m_nFree);

    const auto assembleRange = [this, accelerationDerivative, velocityDerivative](int worker, std::size_t first,
                                                                                    std::size_t last)
    {
        AssemblyWorkerBuffer& buffer = m_assemblyWorkerBuffers[static_cast<std::size_t>(worker)];
        DynamicElementData elementData;
        for (std::size_t index = first; index < last; ++index)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[index];
            EvaluateDynamicElement(assemblyMap, elementData);
            AppendEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()), nullptr,
                                        elementData, accelerationDerivative, velocityDerivative,
                                        buffer.matrixValues);
            CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, buffer.inertiaForce);
        }
    };

    const auto& ranges = GetAssemblyRanges(workerCount);
    PrepareAssemblyWorkerBuffers(static_cast<int>(ranges.size()), true);
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    m_parallelAssemblyEngine.ExecuteRanges(
        request, ranges, threadPool,
        [&assembleRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        { assembleRange(static_cast<int>(worker), range.first, range.last); });

    PreparePatternMatrix(effectiveDynamicTangent);
    for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
    {
        MergeAssemblyValues(buffer.matrixValues, effectiveDynamicTangent.valuePtr());
        m_dynamicInertiaForce += buffer.inertiaForce;
    }

    DynamicElementData elementData;
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        EvaluateRigidBodyInertia(assemblyMap, elementData);
        AccumulateEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()), nullptr,
                                        elementData, accelerationDerivative, velocityDerivative,
                                        effectiveDynamicTangent.valuePtr());
        CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
        AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, m_dynamicInertiaForce);
    }
}

void AnalysisStep::BuildElementBatchPlan()
{
    QElapsedTimer timer;
    timer.start();
    m_elementBatchPlan = {};
    m_elementBatchPlan.trussElementIndices.reserve(m_assemblyPattern.elements.size());
    m_elementBatchPlan.cableElementIndices.reserve(m_assemblyPattern.elements.size());
    m_elementBatchPlan.beamElementIndices.reserve(m_assemblyPattern.elements.size());
    m_elementBatchPlan.otherElementIndices.reserve(m_assemblyPattern.elements.size());
    for (std::size_t elementIndex = 0; elementIndex < m_assemblyPattern.elements.size(); ++elementIndex)
    {
        const std::shared_ptr<ElementBase>& element = m_assemblyPattern.elements[elementIndex].element;
        if (std::dynamic_pointer_cast<ElementTruss>(element))
            m_elementBatchPlan.trussElementIndices.push_back(elementIndex);
        else if (std::dynamic_pointer_cast<ElementCable>(element))
            m_elementBatchPlan.cableElementIndices.push_back(elementIndex);
        else if (std::dynamic_pointer_cast<ElementBeam_CR>(element))
            m_elementBatchPlan.beamElementIndices.push_back(elementIndex);
        else
            m_elementBatchPlan.otherElementIndices.push_back(elementIndex);
    }
    m_performanceStatistics.elementBatchPlanMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.elementBatchPlanBuildCount;
}

bool AnalysisStep::TryAssembleGpuTrussEffectiveTangent(const ElementBatchPlan& batchPlan, double accelerationDerivative,
                                                       bool matrixPrepared, bool copyMatrix,
                                                       std::vector<bool>& gpuElements,
                                                       _OUT SpMat& effectiveTangent)
{
    if (!SolverNameSpace::GpuSettings::IsEnabled() ||
        batchPlan.trussElementIndices.size() < SolverNameSpace::GpuSettings::MinimumGpuTrussElements())
        return false;

    const bool cacheMatches = m_gpuTrussBatchElementIndices == batchPlan.trussElementIndices &&
                              m_gpuTrussBatchInput.size() == batchPlan.trussElementIndices.size();
    if (!cacheMatches)
    {
        m_gpuTrussBatchElementIndices = batchPlan.trussElementIndices;
        m_gpuTrussBatchElements.clear();
        m_gpuTrussBatchFirstNodes.clear();
        m_gpuTrussBatchSecondNodes.clear();
        m_gpuTrussBatchInput.clear();
        m_gpuTrussBatchAssemblyMaps.clear();
        m_gpuTrussBatchElements.reserve(batchPlan.trussElementIndices.size());
        m_gpuTrussBatchFirstNodes.reserve(batchPlan.trussElementIndices.size());
        m_gpuTrussBatchSecondNodes.reserve(batchPlan.trussElementIndices.size());
        m_gpuTrussBatchInput.reserve(batchPlan.trussElementIndices.size());
        m_gpuTrussBatchAssemblyMaps.reserve(batchPlan.trussElementIndices.size());
        for (const std::size_t elementIndex : batchPlan.trussElementIndices)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
            const auto truss = std::dynamic_pointer_cast<ElementTruss>(assemblyMap.element);
            if (!truss || assemblyMap.dofs.size() != 6 || truss->m_pNode.size() != 2 || truss->L0 <= 1.0e-12)
                return false;
            const auto firstNode = truss->m_pNode[0].lock();
            const auto secondNode = truss->m_pNode[1].lock();
            const auto property = truss->m_pProperty.lock();
            const auto material = property ? property->m_pMaterial.lock() : nullptr;
            const auto section = property ? property->m_pSection.lock() : nullptr;
            if (!firstNode || !secondNode || !material || !section || material->m_Young <= 0.0 ||
                section->m_Area <= 0.0 || material->m_Density < 0.0)
                return false;

            SolverNameSpace::GpuTrussElementInput elementInput;
            elementInput.initialPosition[0] = firstNode->m_X;
            elementInput.initialPosition[1] = firstNode->m_Y;
            elementInput.initialPosition[2] = firstNode->m_Z;
            elementInput.initialPosition[3] = secondNode->m_X;
            elementInput.initialPosition[4] = secondNode->m_Y;
            elementInput.initialPosition[5] = secondNode->m_Z;
            elementInput.young = material->m_Young;
            elementInput.area = section->m_Area;
            elementInput.density = material->m_Density;
            elementInput.initialLength = truss->L0;
            elementInput.initialStress = truss->m_InitStress;
            elementInput.useEngineeringStrain = truss->m_UseEngineeringStrain ? 1 : 0;
            SolverNameSpace::GpuTrussAssemblyMap gpuAssemblyMap;
            for (int localDof = 0; localDof < 6; ++localDof)
            {
                const int freeDof = assemblyMap.dofs[localDof] - m_nFixed;
                gpuAssemblyMap.freeDofs[localDof] = freeDof >= 0 && freeDof < m_nFree ? freeDof : -1;
            }
            for (int valueIndex = 0; valueIndex < 36; ++valueIndex)
            {
                const Eigen::Index matrixValueIndex = assemblyMap.valueIndices[static_cast<std::size_t>(valueIndex)];
                if (matrixValueIndex > std::numeric_limits<int>::max())
                    return false;
                gpuAssemblyMap.matrixValueIndices[valueIndex] = static_cast<int>(matrixValueIndex);
            }
            m_gpuTrussBatchElements.push_back(truss);
            m_gpuTrussBatchFirstNodes.push_back(firstNode);
            m_gpuTrussBatchSecondNodes.push_back(secondNode);
            m_gpuTrussBatchInput.push_back(elementInput);
            m_gpuTrussBatchAssemblyMaps.push_back(gpuAssemblyMap);
        }
    }

    for (std::size_t elementIndex = 0; elementIndex < m_gpuTrussBatchInput.size(); ++elementIndex)
    {
        const auto& firstNode = m_gpuTrussBatchFirstNodes[elementIndex];
        const auto& secondNode = m_gpuTrussBatchSecondNodes[elementIndex];
        if (!firstNode || !secondNode || firstNode->m_Displacement.size() < 3 ||
            secondNode->m_Displacement.size() < 3 ||
            firstNode->m_Acceleration.size() < 3 || secondNode->m_Acceleration.size() < 3)
            return false;
        SolverNameSpace::GpuTrussElementInput& elementInput = m_gpuTrussBatchInput[elementIndex];
        for (int component = 0; component < 3; ++component)
        {
            elementInput.displacement[component] = firstNode->m_Displacement[component];
            elementInput.displacement[component + 3] = secondNode->m_Displacement[component];
            elementInput.acceleration[component] = firstNode->m_Acceleration[component];
            elementInput.acceleration[component + 3] = secondNode->m_Acceleration[component];
        }
    }

    const Eigen::Index matrixValueCount = m_assemblyPattern.matrix.nonZeros();
    if (matrixValueCount > std::numeric_limits<int>::max())
        return false;
    const bool evaluated = copyMatrix
                               ? m_gpuTrussBatchEvaluator.EvaluateAndAssemble(
                                     m_gpuTrussBatchInput, m_gpuTrussBatchAssemblyMaps, accelerationDerivative,
                                     static_cast<int>(matrixValueCount), m_nFree)
                               : m_gpuTrussBatchEvaluator.EvaluateResidualAndAssemble(
                                     m_gpuTrussBatchInput, m_gpuTrussBatchAssemblyMaps,
                                     static_cast<int>(matrixValueCount), m_nFree);
    if (!evaluated)
        return false;

    VectorXd gpuInertiaForce = VectorXd::Zero(m_nFree);
    if (!copyMatrix)
    {
        if (!m_gpuTrussBatchEvaluator.CopyAssembledInertia(gpuInertiaForce.data(), m_nFree))
            return false;
    }
    else if (matrixPrepared)
    {
        m_gpuMatrixValueScratch.resize(static_cast<std::size_t>(matrixValueCount));
        if (!m_gpuTrussBatchEvaluator.CopyAssembledData(
                m_gpuMatrixValueScratch.data(), static_cast<int>(matrixValueCount), gpuInertiaForce.data(), m_nFree))
            return false;
        for (Eigen::Index valueIndex = 0; valueIndex < matrixValueCount; ++valueIndex)
            effectiveTangent.valuePtr()[valueIndex] += m_gpuMatrixValueScratch[static_cast<std::size_t>(valueIndex)];
    }
    else
    {
        PreparePatternMatrix(effectiveTangent);
        if (!m_gpuTrussBatchEvaluator.CopyAssembledData(effectiveTangent.valuePtr(), static_cast<int>(matrixValueCount),
                                                        gpuInertiaForce.data(), m_nFree))
            return false;
    }
    const SolverNameSpace::GpuTrussElementState* state = m_gpuTrussBatchEvaluator.StateData();
    if (!state)
        return false;
    m_dynamicInertiaForce += gpuInertiaForce;
    for (std::size_t elementIndex = 0; elementIndex < m_gpuTrussBatchInput.size(); ++elementIndex)
    {
        const SolverNameSpace::GpuTrussElementState& elementState = state[elementIndex];
        m_gpuTrussBatchElements[elementIndex]->m_Stress = elementState.stress;
        m_gpuTrussBatchElements[elementIndex]->L = elementState.currentLength;
        m_gpuTrussBatchElements[elementIndex]->m_inforce =
            Eigen::Map<const Eigen::VectorXd>(elementState.internalForce, 6);
    }

    for (const std::size_t elementIndex : batchPlan.trussElementIndices)
        gpuElements[elementIndex] = true;

    const SolverNameSpace::GpuTrussBatchTiming& timing = m_gpuTrussBatchEvaluator.LastTiming();
    m_performanceStatistics.gpuTrussUploadMilliseconds += timing.uploadMilliseconds;
    m_performanceStatistics.gpuTrussKernelMilliseconds += timing.kernelMilliseconds;
    m_performanceStatistics.gpuTrussDownloadMilliseconds +=
        timing.downloadMilliseconds + timing.assemblyDownloadMilliseconds;
    ++m_performanceStatistics.gpuTrussBatchCount;
    m_gpuTrussCscValuesComplete = m_gpuTrussBatchInput.size() == m_assemblyPattern.elements.size() &&
                                  m_assemblyPattern.rigidBodyInertias.empty();
    return true;
}

bool AnalysisStep::TryAssembleGpuCableEffectiveTangent(const ElementBatchPlan& batchPlan, double accelerationDerivative,
                                                       bool matrixPrepared, bool copyMatrix,
                                                       std::vector<bool>& gpuElements,
                                                       _OUT SpMat& effectiveTangent)
{
    if (m_referenceGravityEquilibrium || m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN)
        return false;
    if (!SolverNameSpace::GpuSettings::IsEnabled())
        return false;

    const bool cacheMatches = m_gpuCableBatchSourceElementIndices == batchPlan.cableElementIndices;
    if (!cacheMatches)
    {
        m_gpuCableBatchSourceElementIndices = batchPlan.cableElementIndices;
        m_gpuCableBatchElementIndices.clear();
        m_gpuCableBatchElements.clear();
        m_gpuCableBatchFirstNodes.clear();
        m_gpuCableBatchSecondNodes.clear();
        m_gpuCableBatchInput.clear();
        m_gpuCableBatchAssemblyMaps.clear();
        m_gpuCableBatchElementIndices.reserve(batchPlan.cableElementIndices.size());
        m_gpuCableBatchElements.reserve(batchPlan.cableElementIndices.size());
        m_gpuCableBatchFirstNodes.reserve(batchPlan.cableElementIndices.size());
        m_gpuCableBatchSecondNodes.reserve(batchPlan.cableElementIndices.size());
        m_gpuCableBatchInput.reserve(batchPlan.cableElementIndices.size());
        m_gpuCableBatchAssemblyMaps.reserve(batchPlan.cableElementIndices.size());
        ++m_performanceStatistics.gpuCableTopologyBuildCount;
        for (const std::size_t elementIndex : batchPlan.cableElementIndices)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
            const auto cable = std::dynamic_pointer_cast<ElementCable>(assemblyMap.element);
            if (!cable || assemblyMap.dofs.size() != 8 || cable->m_pNode.size() != 2)
                continue;

            const auto firstNode = cable->m_pNode[0].lock();
            const auto secondNode = cable->m_pNode[1].lock();
            const auto property = cable->m_pProperty.lock();
            const auto material = property ? property->m_pMaterial.lock() : nullptr;
            const auto section = property ? property->m_pSection.lock() : nullptr;
            if (!firstNode || !secondNode || !material || !section || firstNode->m_DOF.size() != 4 ||
                secondNode->m_DOF.size() != 4 || material->m_Young <= 0.0 || section->m_Area <= 0.0 ||
                material->m_Density < 0.0 || 1.0 + material->m_Poisson <= 1.0e-12)
                continue;
            double iy = 0.0;
            double iz = 0.0;
            double polarMoment = 0.0;
            section->Calculate_I(iy, iz, polarMoment);
            const double massPolarMoment = polarMoment;
            if (polarMoment <= 0.0 && section->m_Radius > 0.0)
                polarMoment = 0.5 * section->m_Area * section->m_Radius * section->m_Radius;
            if (polarMoment < 0.0)
                continue;
            // CUDA 内核用 density * polarMoment 计算转动质量，未包含偏心平动/扭转耦合。
            // 兼容的裸索仍可使用 CUDA，不兼容截面由 CPU 装配。
            const double gpuRotaryMass = material->m_Density * polarMoment;
            const double cpuRotaryMass = section->m_MassInertiaPerLength.x() > 0.0
                                             ? section->m_MassInertiaPerLength.x()
                                             : material->m_Density * massPolarMoment;
            if (section->m_MassFirstMomentPerLength.squaredNorm() > 0.0 ||
                std::abs(cpuRotaryMass - gpuRotaryMass) >
                    1.0e-10 * std::max({1.0e-12, std::abs(cpuRotaryMass), std::abs(gpuRotaryMass)}))
                continue;

            SolverNameSpace::GpuCableElementInput elementInput;
            const double firstPosition[] = {firstNode->m_X, firstNode->m_Y, firstNode->m_Z};
            const double secondPosition[] = {secondNode->m_X, secondNode->m_Y, secondNode->m_Z};
            double initialLengthSquared = 0.0;
            for (int component = 0; component < 3; ++component)
            {
                elementInput.initialPosition[component] = firstPosition[component];
                elementInput.initialPosition[component + 3] = secondPosition[component];
                const double initialChord = secondPosition[component] - firstPosition[component];
                initialLengthSquared += initialChord * initialChord;
            }
            elementInput.young = material->m_Young;
            elementInput.poisson = material->m_Poisson;
            elementInput.area = section->m_Area;
            elementInput.polarMoment = polarMoment;
            elementInput.density = material->m_Density;
            elementInput.initialLength = std::sqrt(initialLengthSquared);
            elementInput.initialStress = cable->m_InitStress;
            if (elementInput.initialLength <= 1.0e-12)
                continue;

            SolverNameSpace::GpuCableAssemblyMap gpuAssemblyMap;
            for (int localDof = 0; localDof < 8; ++localDof)
            {
                const int freeDof = assemblyMap.dofs[localDof] - m_nFixed;
                gpuAssemblyMap.freeDofs[localDof] = freeDof >= 0 && freeDof < m_nFree ? freeDof : -1;
            }
            for (int valueIndex = 0; valueIndex < 64; ++valueIndex)
            {
                const Eigen::Index matrixValueIndex = assemblyMap.valueIndices[static_cast<std::size_t>(valueIndex)];
                if (matrixValueIndex > std::numeric_limits<int>::max())
                    return false;
                gpuAssemblyMap.matrixValueIndices[valueIndex] = static_cast<int>(matrixValueIndex);
            }
            m_gpuCableBatchElementIndices.push_back(elementIndex);
            m_gpuCableBatchElements.push_back(cable);
            m_gpuCableBatchFirstNodes.push_back(firstNode);
            m_gpuCableBatchSecondNodes.push_back(secondNode);
            m_gpuCableBatchInput.push_back(elementInput);
            m_gpuCableBatchAssemblyMaps.push_back(gpuAssemblyMap);
        }
    }

    if (m_gpuCableBatchInput.size() < SolverNameSpace::GpuSettings::MinimumGpuCableElements())
        return false;

    for (std::size_t elementIndex = 0; elementIndex < m_gpuCableBatchInput.size(); ++elementIndex)
    {
        const auto& firstNode = m_gpuCableBatchFirstNodes[elementIndex];
        const auto& secondNode = m_gpuCableBatchSecondNodes[elementIndex];
        if (!firstNode || !secondNode || firstNode->m_Displacement.size() < 4 ||
            secondNode->m_Displacement.size() < 4 || firstNode->m_Acceleration.size() < 4 ||
            secondNode->m_Acceleration.size() < 4)
            return false;
        SolverNameSpace::GpuCableElementInput& elementInput = m_gpuCableBatchInput[elementIndex];
        for (int component = 0; component < 3; ++component)
        {
            elementInput.displacement[component] = firstNode->m_Displacement[component];
            elementInput.displacement[component + 4] = secondNode->m_Displacement[component];
            elementInput.acceleration[component] = firstNode->m_Acceleration[component];
            elementInput.acceleration[component + 4] = secondNode->m_Acceleration[component];
        }
        elementInput.displacement[3] = firstNode->m_Displacement[3];
        elementInput.displacement[7] = secondNode->m_Displacement[3];
        elementInput.acceleration[3] = firstNode->m_Acceleration[3];
        elementInput.acceleration[7] = secondNode->m_Acceleration[3];
    }

    const Eigen::Index matrixValueCount = m_assemblyPattern.matrix.nonZeros();
    if (matrixValueCount > std::numeric_limits<int>::max())
        return false;
    const bool evaluated = copyMatrix
                               ? m_gpuCableBatchEvaluator.EvaluateAndAssemble(
                                     m_gpuCableBatchInput, m_gpuCableBatchAssemblyMaps, accelerationDerivative,
                                     static_cast<int>(matrixValueCount), m_nFree)
                               : m_gpuCableBatchEvaluator.EvaluateResidualAndAssemble(
                                     m_gpuCableBatchInput, m_gpuCableBatchAssemblyMaps,
                                     static_cast<int>(matrixValueCount), m_nFree);
    if (!evaluated)
        return false;

    VectorXd gpuInertiaForce = VectorXd::Zero(m_nFree);
    if (!copyMatrix)
    {
        if (!m_gpuCableBatchEvaluator.CopyAssembledInertia(gpuInertiaForce.data(), m_nFree))
            return false;
    }
    else
    {
        if (!matrixPrepared)
            PreparePatternMatrix(effectiveTangent);
        if (!m_gpuCableBatchEvaluator.CopyAssembledData(effectiveTangent.valuePtr(),
                                                        static_cast<int>(matrixValueCount), gpuInertiaForce.data(),
                                                        m_nFree))
            return false;
    }
    const SolverNameSpace::GpuCableElementState* state = m_gpuCableBatchEvaluator.StateData();
    if (!state)
        return false;
    m_dynamicInertiaForce += gpuInertiaForce;
    for (std::size_t elementIndex = 0; elementIndex < m_gpuCableBatchInput.size(); ++elementIndex)
    {
        const SolverNameSpace::GpuCableElementState& elementState = state[elementIndex];
        m_gpuCableBatchElements[elementIndex]->m_Stress = elementState.stress;
        m_gpuCableBatchElements[elementIndex]->L = elementState.currentLength;
        m_gpuCableBatchElements[elementIndex]->m_inforce =
            Eigen::Map<const Eigen::VectorXd>(elementState.internalForce, 8);
    }

    for (const std::size_t elementIndex : m_gpuCableBatchElementIndices)
        gpuElements[elementIndex] = true;

    const SolverNameSpace::GpuCableBatchTiming& timing = m_gpuCableBatchEvaluator.LastTiming();
    m_performanceStatistics.gpuCableUploadMilliseconds += timing.uploadMilliseconds;
    m_performanceStatistics.gpuCableKernelMilliseconds += timing.kernelMilliseconds;
    m_performanceStatistics.gpuCableDownloadMilliseconds += timing.downloadMilliseconds;
    m_performanceStatistics.gpuCableAssemblyDownloadMilliseconds += timing.assemblyDownloadMilliseconds;
    ++m_performanceStatistics.gpuCableBatchCount;
    m_gpuCableCscValuesComplete =
        m_gpuCableBatchInput.size() == m_assemblyPattern.elements.size() &&
        m_assemblyPattern.rigidBodyInertias.empty();
    return true;
}

bool AnalysisStep::TryAssembleGpuSharedCableEffectiveTangent(const ElementBatchPlan& batchPlan,
                                                             double accelerationDerivative, bool matrixPrepared,
                                                             bool copyMatrix, _OUT std::vector<bool>& gpuElements,
                                                             _OUT SpMat& effectiveTangent)
{
    if (m_referenceGravityEquilibrium || m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN)
        return false;
    if (!SolverNameSpace::GpuSettings::IsEnabled())
        return false;

    if (m_gpuSharedCableBatchSourceElementIndices != batchPlan.cableElementIndices)
    {
        m_gpuSharedCableBatchSourceElementIndices = batchPlan.cableElementIndices;
        m_gpuSharedCableBatchElementIndices.clear();
        m_gpuSharedCableBatchElements.clear();
        m_gpuSharedCableBatchFirstNodes.clear();
        m_gpuSharedCableBatchSecondNodes.clear();
        m_gpuSharedCableBatchInput.clear();
        m_gpuSharedCableBatchAssemblyMaps.clear();
        for (const std::size_t elementIndex : batchPlan.cableElementIndices)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
            const auto cable = std::dynamic_pointer_cast<ElementCable>(assemblyMap.element);
            const int localDofs = static_cast<int>(assemblyMap.dofs.size());
            if (!cable || (localDofs != 10 && localDofs != 12) || cable->m_pNode.size() != 2)
                continue;
            const auto firstNode = cable->m_pNode[0].lock();
            const auto secondNode = cable->m_pNode[1].lock();
            const auto property = cable->m_pProperty.lock();
            const auto material = property ? property->m_pMaterial.lock() : nullptr;
            const auto section = property ? property->m_pSection.lock() : nullptr;
            if (!firstNode || !secondNode || !material || !section || material->m_Young <= 0.0 ||
                section->m_Area <= 0.0 || material->m_Density < 0.0 || 1.0 + material->m_Poisson <= 1.0e-12)
                continue;
            const int firstDofs = static_cast<int>(firstNode->m_DOF.size());
            const int secondDofs = static_cast<int>(secondNode->m_DOF.size());
            if ((firstDofs != 4 && firstDofs != 6) || (secondDofs != 4 && secondDofs != 6) ||
                firstDofs + secondDofs != localDofs)
                continue;

            double iy = 0.0;
            double iz = 0.0;
            double polarMoment = 0.0;
            section->Calculate_I(iy, iz, polarMoment);
            const double massPolarMoment = polarMoment;
            if (polarMoment <= 0.0 && section->m_Radius > 0.0)
                polarMoment = 0.5 * section->m_Area * section->m_Radius * section->m_Radius;
            if (polarMoment < 0.0)
                continue;
            const double gpuRotaryMass = material->m_Density * polarMoment;
            const double cpuRotaryMass = section->m_MassInertiaPerLength.x() > 0.0
                                             ? section->m_MassInertiaPerLength.x()
                                             : material->m_Density * massPolarMoment;
            if (section->m_MassFirstMomentPerLength.squaredNorm() > 0.0 ||
                std::abs(cpuRotaryMass - gpuRotaryMass) >
                    1.0e-10 * std::max({1.0e-12, std::abs(cpuRotaryMass), std::abs(gpuRotaryMass)}))
                continue;

            SolverNameSpace::GpuSharedCableElementInput elementInput;
            elementInput.initialPosition[0] = firstNode->m_X;
            elementInput.initialPosition[1] = firstNode->m_Y;
            elementInput.initialPosition[2] = firstNode->m_Z;
            elementInput.initialPosition[3] = secondNode->m_X;
            elementInput.initialPosition[4] = secondNode->m_Y;
            elementInput.initialPosition[5] = secondNode->m_Z;
            elementInput.young = material->m_Young;
            elementInput.poisson = material->m_Poisson;
            elementInput.area = section->m_Area;
            elementInput.polarMoment = polarMoment;
            elementInput.density = material->m_Density;
            const Eigen::Vector3d initialChord(elementInput.initialPosition[3] - elementInput.initialPosition[0],
                                               elementInput.initialPosition[4] - elementInput.initialPosition[1],
                                               elementInput.initialPosition[5] - elementInput.initialPosition[2]);
            elementInput.initialLength = initialChord.norm();
            elementInput.initialStress = cable->m_InitStress;
            elementInput.firstNodeDofs = firstDofs;
            elementInput.secondNodeDofs = secondDofs;
            if (elementInput.initialLength <= 1.0e-12)
                continue;

            SolverNameSpace::GpuSharedCableAssemblyMap gpuAssemblyMap;
            std::fill(std::begin(gpuAssemblyMap.matrixValueIndices), std::end(gpuAssemblyMap.matrixValueIndices), -1);
            std::fill(std::begin(gpuAssemblyMap.freeDofs), std::end(gpuAssemblyMap.freeDofs), -1);
            for (int localDof = 0; localDof < localDofs; ++localDof)
            {
                const int freeDof = assemblyMap.dofs[localDof] - m_nFixed;
                gpuAssemblyMap.freeDofs[localDof] = freeDof >= 0 && freeDof < m_nFree ? freeDof : -1;
            }
            for (int row = 0; row < localDofs; ++row)
                for (int column = 0; column < localDofs; ++column)
                {
                    const Eigen::Index index =
                        assemblyMap.valueIndices[static_cast<std::size_t>(row * localDofs + column)];
                    if (index > std::numeric_limits<int>::max())
                        return false;
                    gpuAssemblyMap.matrixValueIndices[row * 12 + column] = static_cast<int>(index);
                }
            m_gpuSharedCableBatchElementIndices.push_back(elementIndex);
            m_gpuSharedCableBatchElements.push_back(cable);
            m_gpuSharedCableBatchFirstNodes.push_back(firstNode);
            m_gpuSharedCableBatchSecondNodes.push_back(secondNode);
            m_gpuSharedCableBatchInput.push_back(elementInput);
            m_gpuSharedCableBatchAssemblyMaps.push_back(gpuAssemblyMap);
        }
    }

    if (m_gpuSharedCableBatchInput.size() < SolverNameSpace::GpuSettings::MinimumGpuSharedCableElements())
        return false;
    for (std::size_t index = 0; index < m_gpuSharedCableBatchInput.size(); ++index)
    {
        const auto& firstNode = m_gpuSharedCableBatchFirstNodes[index];
        const auto& secondNode = m_gpuSharedCableBatchSecondNodes[index];
        SolverNameSpace::GpuSharedCableElementInput& input = m_gpuSharedCableBatchInput[index];
        if (!firstNode || !secondNode || firstNode->m_Displacement.size() < input.firstNodeDofs ||
            secondNode->m_Displacement.size() < input.secondNodeDofs ||
            firstNode->m_Acceleration.size() < input.firstNodeDofs ||
            secondNode->m_Acceleration.size() < input.secondNodeDofs)
            return false;
        for (int component = 0; component < 3; ++component)
        {
            input.displacement[component] = firstNode->m_Displacement[component];
            input.displacement[component + 3] = secondNode->m_Displacement[component];
        }
        for (int component = 0; component < input.firstNodeDofs; ++component)
            input.acceleration[component] = firstNode->m_Acceleration[component];
        for (int component = 0; component < input.secondNodeDofs; ++component)
            input.acceleration[input.firstNodeDofs + component] = secondNode->m_Acceleration[component];
        input.twist[0] = m_gpuSharedCableBatchElements[index]->GetNodalTwist(0);
        input.twist[1] = m_gpuSharedCableBatchElements[index]->GetNodalTwist(1);
        const Eigen::Vector3d chord(
            input.initialPosition[3] + input.displacement[3] - input.initialPosition[0] - input.displacement[0],
            input.initialPosition[4] + input.displacement[4] - input.initialPosition[1] - input.displacement[1],
            input.initialPosition[5] + input.displacement[5] - input.initialPosition[2] - input.displacement[2]);
        const Eigen::Vector3d torsionAxis = Utility::CR::CanonicalAxis(chord);
        for (int component = 0; component < 3; ++component)
            input.torsionAxis[component] = torsionAxis[component];
    }

    const Eigen::Index matrixValueCount = m_assemblyPattern.matrix.nonZeros();
    if (matrixValueCount > std::numeric_limits<int>::max())
        return false;
    if (!m_gpuSharedCableBatchEvaluator.EvaluateAndAssemble(m_gpuSharedCableBatchInput,
                                                            m_gpuSharedCableBatchAssemblyMaps, accelerationDerivative,
                                                            static_cast<int>(matrixValueCount), m_nFree, copyMatrix))
        return false;
    VectorXd inertiaForce = VectorXd::Zero(m_nFree);
    if (!copyMatrix)
    {
        if (!m_gpuSharedCableBatchEvaluator.CopyAssembledInertia(inertiaForce.data(), m_nFree))
            return false;
    }
    else if (matrixPrepared)
    {
        m_gpuMatrixValueScratch.resize(static_cast<std::size_t>(matrixValueCount));
        if (!m_gpuSharedCableBatchEvaluator.CopyAssembledData(
                m_gpuMatrixValueScratch.data(), static_cast<int>(matrixValueCount), inertiaForce.data(), m_nFree))
            return false;
        for (Eigen::Index index = 0; index < matrixValueCount; ++index)
            effectiveTangent.valuePtr()[index] += m_gpuMatrixValueScratch[static_cast<std::size_t>(index)];
    }
    else
    {
        PreparePatternMatrix(effectiveTangent);
        if (!m_gpuSharedCableBatchEvaluator.CopyAssembledData(
                effectiveTangent.valuePtr(), static_cast<int>(matrixValueCount), inertiaForce.data(), m_nFree))
            return false;
    }
    const SolverNameSpace::GpuSharedCableElementState* state = m_gpuSharedCableBatchEvaluator.StateData();
    if (!state)
        return false;
    m_dynamicInertiaForce += inertiaForce;
    for (std::size_t index = 0; index < m_gpuSharedCableBatchInput.size(); ++index)
    {
        const int localDofs =
            m_gpuSharedCableBatchInput[index].firstNodeDofs + m_gpuSharedCableBatchInput[index].secondNodeDofs;
        m_gpuSharedCableBatchElements[index]->m_Stress = state[index].stress;
        m_gpuSharedCableBatchElements[index]->L = state[index].currentLength;
        m_gpuSharedCableBatchElements[index]->m_inforce =
            Eigen::Map<const Eigen::VectorXd>(state[index].internalForce, localDofs);
        gpuElements[m_gpuSharedCableBatchElementIndices[index]] = true;
    }
    ++m_performanceStatistics.gpuCableBatchCount;
    return true;
}

bool AnalysisStep::TryAssembleGpuBeamEffectiveTangent(const ElementBatchPlan& batchPlan, double accelerationDerivative,
                                                      double velocityDerivative, bool matrixPrepared, bool copyMatrix,
                                                      _OUT std::vector<bool>& gpuElements, _OUT SpMat& effectiveTangent)
{
    if (!SolverNameSpace::GpuSettings::IsEnabled() ||
        batchPlan.beamElementIndices.size() < SolverNameSpace::GpuSettings::MinimumGpuBeamElements())
        return false;

    const bool cacheMatches = m_gpuBeamBatchElementIndices == batchPlan.beamElementIndices &&
                              m_gpuBeamBatchInput.size() == batchPlan.beamElementIndices.size();
    if (!cacheMatches)
    {
        m_gpuBeamBatchElementIndices = batchPlan.beamElementIndices;
        m_gpuBeamBatchElements.clear();
        m_gpuBeamBatchFirstNodes.clear();
        m_gpuBeamBatchSecondNodes.clear();
        m_gpuBeamBatchInput.clear();
        m_gpuBeamBatchAssemblyMaps.clear();
        m_gpuBeamBatchElements.reserve(batchPlan.beamElementIndices.size());
        m_gpuBeamBatchFirstNodes.reserve(batchPlan.beamElementIndices.size());
        m_gpuBeamBatchSecondNodes.reserve(batchPlan.beamElementIndices.size());
        m_gpuBeamBatchInput.reserve(batchPlan.beamElementIndices.size());
        m_gpuBeamBatchAssemblyMaps.reserve(batchPlan.beamElementIndices.size());
        for (const std::size_t elementIndex : batchPlan.beamElementIndices)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
            const auto beam = std::dynamic_pointer_cast<ElementBeam_CR>(assemblyMap.element);
            if (!beam || assemblyMap.dofs.size() != 12 || beam->m_pNode.size() != 2 || beam->L0 <= 1.0e-12)
                return false;
            const auto firstNode = beam->m_pNode[0].lock();
            const auto secondNode = beam->m_pNode[1].lock();
            const auto property = beam->m_pProperty.lock();
            const auto material = property ? property->m_pMaterial.lock() : nullptr;
            const auto section = property ? property->m_pSection.lock() : nullptr;
            if (!firstNode || !secondNode || !material || !section || material->m_Young <= 0.0 ||
                section->m_Area <= 0.0 || material->m_Density < 0.0 || 1.0 + material->m_Poisson <= 1.0e-12)
                return false;

            double inertiaY = 0.0;
            double inertiaZ = 0.0;
            double torsionConstant = 0.0;
            section->Calculate_I(inertiaY, inertiaZ, torsionConstant);
            if (inertiaY < 0.0 || inertiaZ < 0.0 || torsionConstant < 0.0)
                return false;

            SolverNameSpace::GpuBeamElementInput elementInput;
            elementInput.initialPosition[0] = firstNode->m_X;
            elementInput.initialPosition[1] = firstNode->m_Y;
            elementInput.initialPosition[2] = firstNode->m_Z;
            elementInput.initialPosition[3] = secondNode->m_X;
            elementInput.initialPosition[4] = secondNode->m_Y;
            elementInput.initialPosition[5] = secondNode->m_Z;
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    elementInput.initialRotation[row * 3 + column] = beam->R0(row, column);
            elementInput.young = material->m_Young;
            elementInput.poisson = material->m_Poisson;
            elementInput.area = section->m_Area;
            elementInput.inertiaY = inertiaY;
            elementInput.inertiaZ = inertiaZ;
            elementInput.torsionConstant = torsionConstant;
            elementInput.density = material->m_Density;
            elementInput.initialLength = beam->L0;
            elementInput.initialStress = beam->m_InitStress;
            elementInput.rotaryInertia[0] = material->m_Density * (inertiaY + inertiaZ);
            elementInput.rotaryInertia[1] = material->m_Density * inertiaY;
            elementInput.rotaryInertia[2] = material->m_Density * inertiaZ;
            if (section->m_MassInertiaPerLength.allFinite() &&
                (section->m_MassInertiaPerLength.array() >= 0.0).all() &&
                (section->m_MassInertiaPerLength.array() > 0.0).any())
                for (int component = 0; component < 3; ++component)
                    elementInput.rotaryInertia[component] = section->m_MassInertiaPerLength[component];

            SolverNameSpace::GpuBeamAssemblyMap gpuAssemblyMap;
            for (int localDof = 0; localDof < 12; ++localDof)
            {
                const int freeDof = assemblyMap.dofs[localDof] - m_nFixed;
                gpuAssemblyMap.freeDofs[localDof] = freeDof >= 0 && freeDof < m_nFree ? freeDof : -1;
            }
            for (int valueIndex = 0; valueIndex < 144; ++valueIndex)
            {
                const Eigen::Index matrixValueIndex = assemblyMap.valueIndices[static_cast<std::size_t>(valueIndex)];
                if (matrixValueIndex > std::numeric_limits<int>::max())
                    return false;
                gpuAssemblyMap.matrixValueIndices[valueIndex] = static_cast<int>(matrixValueIndex);
            }
            m_gpuBeamBatchElements.push_back(beam);
            m_gpuBeamBatchFirstNodes.push_back(firstNode);
            m_gpuBeamBatchSecondNodes.push_back(secondNode);
            m_gpuBeamBatchInput.push_back(elementInput);
            m_gpuBeamBatchAssemblyMaps.push_back(gpuAssemblyMap);
        }
    }

    for (std::size_t elementIndex = 0; elementIndex < m_gpuBeamBatchInput.size(); ++elementIndex)
    {
        const auto& firstNode = m_gpuBeamBatchFirstNodes[elementIndex];
        const auto& secondNode = m_gpuBeamBatchSecondNodes[elementIndex];
        if (!firstNode || !secondNode || firstNode->m_Displacement.size() < 6 ||
            secondNode->m_Displacement.size() < 6 || firstNode->m_Velocity.size() < 6 ||
            secondNode->m_Velocity.size() < 6 || firstNode->m_Acceleration.size() < 6 ||
            secondNode->m_Acceleration.size() < 6)
            return false;
        SolverNameSpace::GpuBeamElementInput& elementInput = m_gpuBeamBatchInput[elementIndex];
        std::array<Matrix3d, 2> gaussRotations;
        m_gpuBeamBatchElements[elementIndex]->GetCommittedGaussRotations(gaussRotations);
        elementInput.hasGaussRotationHistory = true;
        for (int gaussIndex = 0; gaussIndex < 2; ++gaussIndex)
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    elementInput.previousGaussRotation[9 * gaussIndex + 3 * row + column] =
                        gaussRotations[gaussIndex](row, column);
        const std::shared_ptr<Node> nodes[2] = {firstNode, secondNode};
        for (int nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            const Node& node = *nodes[nodeIndex];
            for (int component = 0; component < 3; ++component)
            {
                elementInput.displacement[nodeIndex * 3 + component] = node.m_Displacement[component];
                elementInput.stepRotation[nodeIndex * 3 + component] = node.m_StepRotation[component];
            }
            for (int localDof = 0; localDof < 6; ++localDof)
            {
                elementInput.velocity[nodeIndex * 6 + localDof] = node.m_Velocity[localDof];
                elementInput.acceleration[nodeIndex * 6 + localDof] = node.m_Acceleration[localDof];
            }
            for (int row = 0; row < 3; ++row)
            {
                for (int column = 0; column < 3; ++column)
                {
                    const int valueIndex = nodeIndex * 9 + row * 3 + column;
                    elementInput.currentRotation[valueIndex] = node.m_Rg(row, column);
                    elementInput.previousRotation[valueIndex] = node.m_Rg_n(row, column);
                }
            }
        }
    }

    const Eigen::Index matrixValueCount = m_assemblyPattern.matrix.nonZeros();
    if (matrixValueCount > std::numeric_limits<int>::max())
        return false;
    const bool evaluated = copyMatrix
                               ? m_gpuBeamBatchEvaluator.EvaluateAndAssemble(
                                     m_gpuBeamBatchInput, m_gpuBeamBatchAssemblyMaps, accelerationDerivative,
                                     velocityDerivative, static_cast<int>(matrixValueCount), m_nFree)
                               : m_gpuBeamBatchEvaluator.EvaluateResidualAndAssemble(
                                     m_gpuBeamBatchInput, m_gpuBeamBatchAssemblyMaps,
                                     static_cast<int>(matrixValueCount), m_nFree);
    if (!evaluated)
        return false;
    VectorXd gpuInertiaForce = VectorXd::Zero(m_nFree);
    if (!copyMatrix)
    {
        if (!m_gpuBeamBatchEvaluator.CopyAssembledInertia(gpuInertiaForce.data(), m_nFree))
            return false;
    }
    else if (matrixPrepared)
    {
        m_gpuMatrixValueScratch.resize(static_cast<std::size_t>(matrixValueCount));
        if (!m_gpuBeamBatchEvaluator.CopyAssembledData(
                m_gpuMatrixValueScratch.data(), static_cast<int>(matrixValueCount), gpuInertiaForce.data(), m_nFree))
            return false;
        for (Eigen::Index valueIndex = 0; valueIndex < matrixValueCount; ++valueIndex)
            effectiveTangent.valuePtr()[valueIndex] += m_gpuMatrixValueScratch[static_cast<std::size_t>(valueIndex)];
    }
    else
    {
        PreparePatternMatrix(effectiveTangent);
        if (!m_gpuBeamBatchEvaluator.CopyAssembledData(effectiveTangent.valuePtr(), static_cast<int>(matrixValueCount),
                                                       gpuInertiaForce.data(), m_nFree))
            return false;
    }
    m_dynamicInertiaForce += gpuInertiaForce;
    const SolverNameSpace::GpuBeamElementState* state = m_gpuBeamBatchEvaluator.StateData();
    if (!state)
        return false;
    for (std::size_t elementIndex = 0; elementIndex < m_gpuBeamBatchElements.size(); ++elementIndex)
    {
        const SolverNameSpace::GpuBeamElementState& elementState = state[elementIndex];
        m_gpuBeamBatchElements[elementIndex]->m_Stress = elementState.stress;
        m_gpuBeamBatchElements[elementIndex]->L = elementState.currentLength;
        m_gpuBeamBatchElements[elementIndex]->m_inforce =
            Eigen::Map<const Eigen::VectorXd>(elementState.internalForce, 12);
    }
    for (const std::size_t elementIndex : batchPlan.beamElementIndices)
        gpuElements[elementIndex] = true;

    const SolverNameSpace::GpuBeamBatchTiming& timing = m_gpuBeamBatchEvaluator.LastTiming();
    m_performanceStatistics.gpuBeamUploadMilliseconds += timing.uploadMilliseconds;
    m_performanceStatistics.gpuBeamKernelMilliseconds += timing.kernelMilliseconds;
    m_performanceStatistics.gpuBeamDownloadMilliseconds += timing.downloadMilliseconds;
    m_performanceStatistics.gpuBeamAssemblyDownloadMilliseconds += timing.assemblyDownloadMilliseconds;
    ++m_performanceStatistics.gpuBeamBatchCount;
    m_gpuBeamCscValuesComplete = m_gpuBeamBatchInput.size() == m_assemblyPattern.elements.size() &&
                                 m_assemblyPattern.rigidBodyInertias.empty();
    return true;
}

void AnalysisStep::AssembleCpuEffectiveTangentRemainder(const std::vector<bool>& gpuElements,
                                                        double accelerationDerivative, double velocityDerivative,
                                                        _OUT SpMat& effectiveTangent)
{
    const int workerCount = SelectAssemblyWorkerCount();
    const auto& ranges = GetAssemblyRanges(workerCount);
    PrepareAssemblyWorkerBuffers(static_cast<int>(ranges.size()), true);
    const auto assembleRemainingRange = [this, &gpuElements, accelerationDerivative,
                                         velocityDerivative](int worker, std::size_t first, std::size_t last)
    {
        AssemblyWorkerBuffer& buffer = m_assemblyWorkerBuffers[static_cast<std::size_t>(worker)];
        MatrixXd stiffness;
        DynamicElementData elementData;
        for (std::size_t elementIndex = first; elementIndex < last; ++elementIndex)
        {
            if (gpuElements[elementIndex])
                continue;
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
            ElementBase& element = *assemblyMap.element;
            element.Get_ke(stiffness);
            EvaluateDynamicElement(assemblyMap, elementData);
            AppendEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                        &stiffness, elementData, accelerationDerivative, velocityDerivative,
                                        buffer.matrixValues);
            CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, buffer.inertiaForce);
        }
    };
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Stiffness)};
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    m_parallelAssemblyEngine.ExecuteRanges(
        request, ranges, threadPool,
        [&assembleRemainingRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        {
            assembleRemainingRange(static_cast<int>(worker), range.first, range.last);
        });

    double* values = effectiveTangent.valuePtr();
    for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
    {
        MergeAssemblyValues(buffer.matrixValues, values);
        m_dynamicInertiaForce += buffer.inertiaForce;
    }

    DynamicElementData elementData;
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        EvaluateRigidBodyInertia(assemblyMap, elementData);
        AccumulateEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()), nullptr,
                                        elementData, accelerationDerivative, velocityDerivative, values);
        CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
        AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, m_dynamicInertiaForce);
    }
}

bool AnalysisStep::TryAssembleAcceleratedEffectiveTangent(const ElementBatchPlan& batchPlan,
                                                          double accelerationDerivative, double velocityDerivative,
                                                          _OUT SpMat& effectiveTangent)
{
    // 参考线路使用独立单元方程，通用 GPU 批量核不能代替该方程。
    if ((m_Type == EnumKeyword::StepType::DYNAMIC &&
         m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN) ||
        !SolverNameSpace::GpuSettings::IsEnabled())
        return false;
    const bool hasEligibleBatch =
        batchPlan.trussElementIndices.size() >= SolverNameSpace::GpuSettings::MinimumGpuTrussElements() ||
        batchPlan.cableElementIndices.size() >= SolverNameSpace::GpuSettings::MinimumGpuCableElements() ||
        batchPlan.beamElementIndices.size() >= SolverNameSpace::GpuSettings::MinimumGpuBeamElements();
    if (!hasEligibleBatch)
    {
        SolverNameSpace::GpuSettings::RecordSkipped(
            QStringLiteral("单元批次低于 GPU 阈值（桁架 %1/%2、索 %3/%4、梁 %5/%6），已使用 CPU")
                .arg(batchPlan.trussElementIndices.size())
                .arg(SolverNameSpace::GpuSettings::MinimumGpuTrussElements())
                .arg(batchPlan.cableElementIndices.size())
                .arg(SolverNameSpace::GpuSettings::MinimumGpuCableElements())
                .arg(batchPlan.beamElementIndices.size())
                .arg(SolverNameSpace::GpuSettings::MinimumGpuBeamElements()));
        return false;
    }

    if (m_gpuAdaptivePatternVersion != m_sparsePatternVersion)
    {
        m_gpuAdaptivePatternVersion = m_sparsePatternVersion;
        m_gpuAdaptiveWarmupCount = 0;
        m_gpuAdaptiveUseCpu = false;
    }
    if (m_gpuAdaptiveUseCpu)
        return false;

    // 标记缓冲区随装配模式复用，避免每次牛顿迭代重新申请内存。
    m_gpuAssemblyElementMask.assign(m_assemblyPattern.elements.size(), false);
    std::vector<bool>& gpuElements = m_gpuAssemblyElementMask;
    const bool dynamicCalibration = accelerationDerivative > 0.0;
    const bool compareBackends = dynamicCalibration && m_gpuAdaptiveWarmupCount == 1;
    std::vector<double> cpuReferenceValues;
    VectorXd cpuReferenceInertia;
    double cpuMilliseconds = 0.0;
    if (compareBackends)
    {
        // 首次 GPU 调用用于建立缓存；第二次用同一试探状态实测 CPU，避免按固定规模猜测收益。
        QElapsedTimer cpuTimer;
        cpuTimer.start();
        PreparePatternMatrix(effectiveTangent);
        AssembleCpuEffectiveTangentRemainder(gpuElements, accelerationDerivative, velocityDerivative, effectiveTangent);
        cpuMilliseconds = static_cast<double>(cpuTimer.nsecsElapsed()) / 1.0e6;
        cpuReferenceValues.assign(effectiveTangent.valuePtr(),
                                  effectiveTangent.valuePtr() + effectiveTangent.nonZeros());
        cpuReferenceInertia = m_dynamicInertiaForce;
        m_dynamicInertiaForce.setZero();
    }
    QElapsedTimer gpuTimer;
    gpuTimer.start();
    bool matrixPrepared = false;
    bool cableUsed = false;
    bool sharedCableUsed = false;
    bool trussUsed = false;
    bool beamUsed = false;

    // 索批次可直接保留整体 CSC 数值，先写入整体矩阵；其后可继续叠加其他单元批次。
    if (TryAssembleGpuCableEffectiveTangent(batchPlan, accelerationDerivative, matrixPrepared, true, gpuElements,
                                            effectiveTangent))
    {
        matrixPrepared = true;
        cableUsed = true;
    }
    if (TryAssembleGpuSharedCableEffectiveTangent(batchPlan, accelerationDerivative, matrixPrepared, true, gpuElements,
                                                  effectiveTangent))
    {
        matrixPrepared = true;
        sharedCableUsed = true;
    }
    if (TryAssembleGpuTrussEffectiveTangent(batchPlan, accelerationDerivative, matrixPrepared, true, gpuElements,
                                            effectiveTangent))
    {
        matrixPrepared = true;
        trussUsed = true;
    }
    if (TryAssembleGpuBeamEffectiveTangent(batchPlan, accelerationDerivative, velocityDerivative, matrixPrepared, true,
                                           gpuElements, effectiveTangent))
    {
        matrixPrepared = true;
        beamUsed = true;
    }
    if (!matrixPrepared)
    {
        SolverNameSpace::GpuSettings::RecordFallback(QStringLiteral("GPU 单元批装配未成功，已回退 CPU"));
        // 同一稀疏模式下避免每次迭代重复进行失败的 GPU 尝试和 CPU 对照。
        m_gpuAdaptiveUseCpu = true;
        m_dynamicInertiaForce.setZero();
        return false;
    }

    SolverNameSpace::GpuSettings::RecordGpuUse(QStringLiteral("GPU 单元批装配已实际执行"));
    ++m_performanceStatistics.gpuEffectiveTangentCount;

    const bool allElementsOnGpu = std::all_of(gpuElements.cbegin(), gpuElements.cend(),
                                              [](bool assembledOnGpu)
                                              {
                                                  return assembledOnGpu;
                                              });
    const int gpuBatchCount = static_cast<int>(cableUsed) + static_cast<int>(sharedCableUsed) +
                              static_cast<int>(trussUsed) + static_cast<int>(beamUsed);
    if (m_assemblyPattern.rigidBodyInertias.empty() && (gpuBatchCount > 1 || !allElementsOnGpu || sharedCableUsed))
    {
        const int valueCount = static_cast<int>(effectiveTangent.nonZeros());
        m_gpuMixedCscValuesReady =
            m_gpuMixedCscSum.Combine(trussUsed ? m_gpuTrussBatchEvaluator.DeviceMatrixValues() : nullptr,
                                     cableUsed ? m_gpuCableBatchEvaluator.DeviceMatrixValues() : nullptr,
                                     beamUsed ? m_gpuBeamBatchEvaluator.DeviceMatrixValues() : nullptr, valueCount,
                                     sharedCableUsed ? m_gpuSharedCableBatchEvaluator.DeviceMatrixValues() : nullptr);
    }
    if (!allElementsOnGpu || !m_assemblyPattern.rigidBodyInertias.empty())
    {
        // 未达到 GPU 门槛及其他暂未支持 GPU 的单元使用同一并行回退路径。
        AssembleCpuEffectiveTangentRemainder(gpuElements, accelerationDerivative, velocityDerivative, effectiveTangent);
    }
    if (m_gpuMixedCscValuesReady && !allElementsOnGpu)
    {
        std::size_t additionCount = 0;
        for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
            additionCount += buffer.matrixValues.size();
        std::vector<SolverNameSpace::DeviceCscValueAddition> additions;
        additions.reserve(additionCount);
        for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
            for (const IndexedAssemblyValue& item : buffer.matrixValues)
                if (item.value != 0.0)
                    additions.push_back({static_cast<int>(item.index), item.value});
        m_gpuMixedCscValuesReady = m_gpuMixedCscSum.AddHostValues(additions);
    }
    const double gpuMilliseconds = static_cast<double>(gpuTimer.nsecsElapsed()) / 1.0e6;
    if (compareBackends)
    {
        const Eigen::Map<const VectorXd> cpuValues(cpuReferenceValues.data(),
                                                   static_cast<Eigen::Index>(cpuReferenceValues.size()));
        const Eigen::Map<const VectorXd> gpuValues(effectiveTangent.valuePtr(), effectiveTangent.nonZeros());
        const double matrixError = (gpuValues - cpuValues).norm() / std::max(1.0, cpuValues.norm());
        const double entryError = ((gpuValues - cpuValues).array().abs() / cpuValues.array().abs().max(1.0)).maxCoeff();
        const double inertiaError =
            (m_dynamicInertiaForce - cpuReferenceInertia).norm() / std::max(1.0, cpuReferenceInertia.norm());
        const bool inaccurate = !std::isfinite(matrixError) || !std::isfinite(entryError) ||
                                !std::isfinite(inertiaError) || matrixError > 1.0e-8 || entryError > 1.0e-6 ||
                                inertiaError > 1.0e-8;
        m_gpuAdaptiveUseCpu = inaccurate || gpuMilliseconds >= 0.9 * cpuMilliseconds;
        if (m_gpuAdaptiveUseCpu)
        {
            SolverNameSpace::GpuSettings::RecordFallback(
                inaccurate ? QStringLiteral("模型实测 GPU/CPU 数值偏差超限，后续装配使用 CPU")
                           : QStringLiteral("模型实测 GPU 装配 %1 ms、CPU 装配 %2 ms，后续使用 CPU")
                                 .arg(gpuMilliseconds, 0, 'f', 2)
                                 .arg(cpuMilliseconds, 0, 'f', 2));
            m_gpuTrussCscValuesComplete = false;
            m_gpuCableCscValuesComplete = false;
            m_gpuBeamCscValuesComplete = false;
            m_gpuMixedCscValuesReady = false;
            m_dynamicInertiaForce.setZero();
            return false;
        }
    }
    if (dynamicCalibration && m_gpuAdaptiveWarmupCount < 2)
        ++m_gpuAdaptiveWarmupCount;
    return true;
}

void AnalysisStep::RefreshCpuResidualStateRemainder(const std::vector<bool>& gpuElements)
{
    const int workerCount = SelectAssemblyWorkerCount();
    const auto& ranges = GetAssemblyRanges(workerCount);
    PrepareResidualWorkerBuffers(static_cast<int>(ranges.size()));
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InternalForce) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InertiaForce)};
    const auto refreshRange = [this, &gpuElements](int worker, std::size_t first, std::size_t last)
    {
        AssemblyWorkerBuffer& buffer = m_assemblyWorkerBuffers[static_cast<std::size_t>(worker)];
        MatrixXd unusedStiffness;
        MatrixXd elementMass;
        VectorXd inertiaForce;
        for (std::size_t index = first; index < last; ++index)
        {
            if (gpuElements[index])
                continue;
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[index];
            assemblyMap.element->Get_ke(unusedStiffness);
            assemblyMap.element->Get_InertiaForce(inertiaForce);
            CacheReferenceInertia(assemblyMap, inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, inertiaForce, buffer.inertiaForce);
        }
    };
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    m_parallelAssemblyEngine.ExecuteRanges(
        request, ranges, threadPool,
        [&refreshRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        { refreshRange(static_cast<int>(worker), range.first, range.last); });
    for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
        m_dynamicInertiaForce += buffer.inertiaForce;

    DynamicElementData inertiaData;
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        EvaluateRigidBodyInertia(assemblyMap, inertiaData);
        CacheReferenceInertia(assemblyMap, inertiaData.inertiaForce);
        AccumulateInertiaForce(assemblyMap.dofs, inertiaData.inertiaForce, m_dynamicInertiaForce);
    }
}

void AnalysisStep::RefreshCpuResidualStateAndMass(_OUT SpMat& mass)
{
    QElapsedTimer timer;
    timer.start();
    EnsureAssemblyPattern();
    const int workerCount = SelectAssemblyWorkerCount();
    const auto& ranges = GetAssemblyRanges(workerCount);
    PrepareAssemblyWorkerBuffers(static_cast<int>(ranges.size()), true);
    m_dynamicInertiaForce = VectorXd::Zero(m_nFree);
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InternalForce) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InertiaForce) |
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Mass)};
    const auto assembleRange = [this](int worker, std::size_t first, std::size_t last)
    {
        AssemblyWorkerBuffer& buffer = m_assemblyWorkerBuffers[static_cast<std::size_t>(worker)];
        MatrixXd unusedStiffness;
        MatrixXd elementMass;
        VectorXd inertiaForce;
        for (std::size_t index = first; index < last; ++index)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[index];
            assemblyMap.element->Get_ke(unusedStiffness);
            if (auto* cable = dynamic_cast<ElementCable*>(assemblyMap.element.get()))
                cable->GetMassAndInertiaForce(elementMass, inertiaForce);
            else
            {
                assemblyMap.element->Get_InertiaForce(inertiaForce);
                assemblyMap.element->Get_me_Physical(elementMass);
            }
            AppendElementMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                      elementMass, buffer.matrixValues);
            CacheReferenceInertia(assemblyMap, inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, inertiaForce, buffer.inertiaForce);
        }
    };
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    m_parallelAssemblyEngine.ExecuteRanges(
        request, ranges, threadPool,
        [&assembleRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        { assembleRange(static_cast<int>(worker), range.first, range.last); });

    PreparePatternMatrix(mass);
    for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
    {
        MergeAssemblyValues(buffer.matrixValues, mass.valuePtr());
        m_dynamicInertiaForce += buffer.inertiaForce;
    }

    DynamicElementData elementData;
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        EvaluateRigidBodyInertia(assemblyMap, elementData);
        assemblyMap.inertia->GetPhysicalMass(elementData.mass);
        AccumulateMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()), elementData.mass,
                               mass.valuePtr());
        CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
        AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, m_dynamicInertiaForce);
    }
    m_performanceStatistics.residualStateMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.residualStateCount;
}

bool AnalysisStep::TryRefreshAcceleratedResidualState(const ElementBatchPlan& batchPlan)
{
    if ((m_Type == EnumKeyword::StepType::DYNAMIC &&
         m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN) ||
        !SolverNameSpace::GpuSettings::IsEnabled())
        return false;
    const bool hasEligibleBatch =
        batchPlan.trussElementIndices.size() >= SolverNameSpace::GpuSettings::MinimumGpuTrussElements() ||
        batchPlan.cableElementIndices.size() >= SolverNameSpace::GpuSettings::MinimumGpuCableElements() ||
        batchPlan.beamElementIndices.size() >= SolverNameSpace::GpuSettings::MinimumGpuBeamElements();
    if (!hasEligibleBatch)
        return false;

    m_gpuAssemblyElementMask.assign(m_assemblyPattern.elements.size(), false);
    std::vector<bool>& gpuElements = m_gpuAssemblyElementMask;
    SpMat unusedMatrix;
    bool usedGpu = false;
    usedGpu = TryAssembleGpuCableEffectiveTangent(batchPlan, 0.0, false, false, gpuElements, unusedMatrix) || usedGpu;
    usedGpu = TryAssembleGpuSharedCableEffectiveTangent(batchPlan, 0.0, false, false, gpuElements, unusedMatrix) ||
              usedGpu;
    usedGpu = TryAssembleGpuTrussEffectiveTangent(batchPlan, 0.0, false, false, gpuElements, unusedMatrix) || usedGpu;
    usedGpu =
        TryAssembleGpuBeamEffectiveTangent(batchPlan, 0.0, 0.0, false, false, gpuElements, unusedMatrix) || usedGpu;
    if (!usedGpu)
    {
        SolverNameSpace::GpuSettings::RecordFallback(QStringLiteral("GPU 残差批计算未成功，已回退 CPU"));
        return false;
    }

    SolverNameSpace::GpuSettings::RecordGpuUse(QStringLiteral("GPU 残差内力与惯性力批计算已实际执行"));
    ++m_performanceStatistics.gpuResidualStateCount;
    const bool allElementsOnGpu = std::all_of(gpuElements.cbegin(), gpuElements.cend(),
                                              [](bool evaluatedOnGpu) { return evaluatedOnGpu; });
    if (!allElementsOnGpu || !m_assemblyPattern.rigidBodyInertias.empty())
        RefreshCpuResidualStateRemainder(gpuElements);
    return true;
}

void AnalysisStep::AssembleEffectiveTangent(double accelerationDerivative, double velocityDerivative,
                                            SpMat& effectiveTangent)
{
    AssembleEffectiveTangentImpl(accelerationDerivative, velocityDerivative, effectiveTangent);
    AddLocalRayleighTangent(velocityDerivative, effectiveTangent);
}

void AnalysisStep::AssembleEffectiveTangentImpl(double accelerationDerivative, double velocityDerivative,
                                                SpMat& effectiveTangent)
{
    InvalidateLocalRayleighCache();
    QElapsedTimer timer;
    timer.start();
    EnsureAssemblyPattern();
    m_gpuTrussCscValuesReady = false;
    m_gpuTrussCscValuesComplete = false;
    m_gpuCableCscValuesReady = false;
    m_gpuCableCscValuesComplete = false;
    m_gpuBeamCscValuesReady = false;
    m_gpuBeamCscValuesComplete = false;
    m_gpuMixedCscValuesReady = false;
    const int workerCount = SelectAssemblyWorkerCount();
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Stiffness) |
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::Mass) |
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InertiaForce) |
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::VelocityTangent) |
            SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::ConfigurationTangent),
        0.0, 1.0, accelerationDerivative, velocityDerivative};
    m_dynamicInertiaForce = VectorXd::Zero(m_nFree);
    if (TryAssembleAcceleratedEffectiveTangent(m_elementBatchPlan, accelerationDerivative, velocityDerivative,
                                               effectiveTangent))
    {
        m_gpuTrussCscValuesReady = m_gpuTrussCscValuesComplete;
        m_gpuCableCscValuesReady = m_gpuCableCscValuesComplete;
        m_gpuBeamCscValuesReady = m_gpuBeamCscValuesComplete;
        m_performanceStatistics.effectiveTangentMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
        ++m_performanceStatistics.effectiveTangentCount;
        return;
    }

    if (TryAssembleReferenceCableEffectiveTangent(request, effectiveTangent))
    {
        m_performanceStatistics.effectiveTangentMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
        ++m_performanceStatistics.effectiveTangentCount;
        return;
    }
    if (workerCount > 1 && !ShouldUseColoredAssembly(workerCount))
    {
        const auto& ranges = GetAssemblyRanges(workerCount);
        PrepareAssemblyWorkerBuffers(static_cast<int>(ranges.size()), true);
        const auto assembleRange =
            [this, accelerationDerivative, velocityDerivative](int worker, std::size_t first, std::size_t last)
        {
            AssemblyWorkerBuffer& buffer = m_assemblyWorkerBuffers[static_cast<std::size_t>(worker)];
            MatrixXd stiffness;
            DynamicElementData elementData;
            for (std::size_t index = first; index < last; ++index)
            {
                const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[index];
                ElementBase& element = *assemblyMap.element;
                element.Get_ke(stiffness);
                EvaluateDynamicElement(assemblyMap, elementData);

                AppendEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                            &stiffness, elementData, accelerationDerivative, velocityDerivative,
                                            buffer.matrixValues);
                CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
                AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, buffer.inertiaForce);
            }
        };

        QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
        m_parallelAssemblyEngine.ExecuteRanges(
            request, ranges, threadPool,
            [&assembleRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
            { assembleRange(static_cast<int>(worker), range.first, range.last); });

        PreparePatternMatrix(effectiveTangent);
        for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
        {
            MergeAssemblyValues(buffer.matrixValues, effectiveTangent.valuePtr());
            m_dynamicInertiaForce += buffer.inertiaForce;
        }
        DynamicElementData inertiaData;
        for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
        {
            EvaluateRigidBodyInertia(assemblyMap, inertiaData);
            AccumulateEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                            nullptr, inertiaData, accelerationDerivative, velocityDerivative,
                                            effectiveTangent.valuePtr());
            CacheReferenceInertia(assemblyMap, inertiaData.inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, inertiaData.inertiaForce, m_dynamicInertiaForce);
        }
        m_performanceStatistics.effectiveTangentMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
        ++m_performanceStatistics.effectiveTangentCount;
        return;
    }

    PreparePatternMatrix(effectiveTangent);
    const auto assembleElement =
        [this, accelerationDerivative, velocityDerivative, values = effectiveTangent.valuePtr()](
            std::size_t elementIndex, _OUT MatrixXd& stiffness, _OUT DynamicElementData& elementData)
    {
        const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
        ElementBase& element = *assemblyMap.element;
        element.Get_ke(stiffness);
        EvaluateDynamicElement(assemblyMap, elementData);

        AccumulateEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()),
                                        &stiffness, elementData, accelerationDerivative, velocityDerivative, values);
        CacheReferenceInertia(assemblyMap, elementData.inertiaForce);
        AccumulateInertiaForce(assemblyMap.dofs, elementData.inertiaForce, m_dynamicInertiaForce);
    };

    if (workerCount <= 1)
    {
        // 每个工作线程复用单元缓冲，避免逐单元申请和释放相同尺寸的矩阵。
        MatrixXd stiffness;
        DynamicElementData elementData;
        for (std::size_t elementIndex = 0; elementIndex < m_assemblyPattern.elements.size(); ++elementIndex)
            assembleElement(elementIndex, stiffness, elementData);
    }
    else
    {
        QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
        std::barrier synchronization(workerCount);
        m_parallelAssemblyEngine.ExecuteWorkers(
            request, static_cast<std::size_t>(workerCount), threadPool,
            [this, &assembleElement, &synchronization, workerCount](std::size_t workerIndex)
            {
                const int worker = static_cast<int>(workerIndex);
                MatrixXd stiffness;
                DynamicElementData elementData;
                for (const std::vector<std::size_t>& colorGroup : m_assemblyPattern.elementColorGroups)
                {
                    const std::size_t first = colorGroup.size() * static_cast<std::size_t>(worker) /
                                              static_cast<std::size_t>(workerCount);
                    const std::size_t last = colorGroup.size() * static_cast<std::size_t>(worker + 1) /
                                             static_cast<std::size_t>(workerCount);
                    for (std::size_t index = first; index < last; ++index)
                        assembleElement(colorGroup[index], stiffness, elementData);
                    synchronization.arrive_and_wait();
                }
            });
    }
    DynamicElementData inertiaData;
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        EvaluateRigidBodyInertia(assemblyMap, inertiaData);
        AccumulateEffectiveMatrixValues(assemblyMap.valueIndices, static_cast<int>(assemblyMap.dofs.size()), nullptr,
                                        inertiaData, accelerationDerivative, velocityDerivative,
                                        effectiveTangent.valuePtr());
        CacheReferenceInertia(assemblyMap, inertiaData.inertiaForce);
        AccumulateInertiaForce(assemblyMap.dofs, inertiaData.inertiaForce, m_dynamicInertiaForce);
    }
    m_performanceStatistics.effectiveTangentMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.effectiveTangentCount;
}

void AnalysisStep::UpdateInternalForce()
{
    InvalidateLocalRayleighCache();
    EnsureAssemblyPattern();
    const int workerCount = SelectAssemblyWorkerCount();
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InternalForce)};
    const auto& ranges = GetAssemblyRanges(workerCount);
    const auto refreshRange = [this](std::size_t first, std::size_t last)
    {
        MatrixXd unusedStiffness;
        for (std::size_t index = first; index < last; ++index)
            m_assemblyPattern.elements[index].element->Get_ke(unusedStiffness);
    };

    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    m_parallelAssemblyEngine.ExecuteRanges(
        request, ranges, threadPool,
        [&refreshRange](std::size_t, const SolverNameSpace::ParallelWorkRange& range)
        { refreshRange(range.first, range.last); });
}

void AnalysisStep::RefreshResidualState()
{
    RefreshResidualStateImpl();
}

void AnalysisStep::RefreshResidualStateImpl()
{
    InvalidateLocalRayleighCache();
    QElapsedTimer timer;
    timer.start();
    EnsureAssemblyPattern();
    m_dynamicInertiaForce = VectorXd::Zero(m_nFree);
    if (!TryRefreshAcceleratedResidualState(m_elementBatchPlan))
        RefreshCpuResidualStateRemainder(std::vector<bool>(m_assemblyPattern.elements.size(), false));
    m_performanceStatistics.residualStateMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.residualStateCount;
}

void AnalysisStep::RefreshInertiaForce()
{
    InvalidateLocalRayleighCache();
    QElapsedTimer timer;
    timer.start();
    EnsureAssemblyPattern();
    const int workerCount = SelectAssemblyWorkerCount();
    const auto& ranges = GetAssemblyRanges(workerCount);
    PrepareResidualWorkerBuffers(static_cast<int>(ranges.size()));
    m_dynamicInertiaForce = VectorXd::Zero(m_nFree);
    const SolverNameSpace::SystemEvaluationRequest request{
        SolverNameSpace::ToMask(SolverNameSpace::SystemContribution::InertiaForce)};
    const auto refreshRange = [this](int worker, std::size_t first, std::size_t last)
    {
        AssemblyWorkerBuffer& buffer = m_assemblyWorkerBuffers[static_cast<std::size_t>(worker)];
        VectorXd inertiaForce;
        for (std::size_t index = first; index < last; ++index)
        {
            const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[index];
            assemblyMap.element->Get_InertiaForce(inertiaForce);
            CacheReferenceInertia(assemblyMap, inertiaForce);
            AccumulateInertiaForce(assemblyMap.dofs, inertiaForce, buffer.inertiaForce);
        }
    };
    QThreadPool& threadPool = SolverNameSpace::AssemblySettings::ThreadPool();
    m_parallelAssemblyEngine.ExecuteRanges(
        request, ranges, threadPool,
        [&refreshRange](std::size_t worker, const SolverNameSpace::ParallelWorkRange& range)
        { refreshRange(static_cast<int>(worker), range.first, range.last); });
    for (const AssemblyWorkerBuffer& buffer : m_assemblyWorkerBuffers)
        m_dynamicInertiaForce += buffer.inertiaForce;

    DynamicElementData inertiaData;
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        EvaluateRigidBodyInertia(assemblyMap, inertiaData);
        CacheReferenceInertia(assemblyMap, inertiaData.inertiaForce);
        AccumulateInertiaForce(assemblyMap.dofs, inertiaData.inertiaForce, m_dynamicInertiaForce);
    }
    m_performanceStatistics.inertiaForceMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.inertiaForceCount;
}

void AnalysisStep::GetDynamicInertiaForce(SolverNameSpace::Vec& inertiaForce) const
{
    if (m_dynamicInertiaForce.size() == m_nFree)
        inertiaForce = m_dynamicInertiaForce;
    else
        inertiaForce = SolverNameSpace::Vec::Zero(m_nFree);
}

bool AnalysisStep::SaveTrialState()
{
    if (!m_pData)
        return false;
    m_trialNodeStates.clear();
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
        if (node)
            m_trialNodeStates.emplace(nodeId, std::make_shared<Node>(*node));
    return m_trialNodeStates.size() == m_pData->m_Nodes.size();
}

bool AnalysisStep::RestoreTrialState()
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    if (!m_pData || m_trialNodeStates.size() != m_pData->m_Nodes.size())
        return false;
    for (const auto& [nodeId, state] : m_trialNodeStates)
    {
        const auto found = m_pData->m_Nodes.find(nodeId);
        if (found == m_pData->m_Nodes.end() || !found->second)
            return false;
        if (!state)
            return false;
        *found->second = *state;
    }
    return true;
}

bool AnalysisStep::PushStateCheckpoint()
{
    if (!m_pData || m_stateCheckpoint.has_value())
        return false;

    StateCheckpoint checkpoint;
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        if (!node)
            return false;
        checkpoint.nodes.emplace(nodeId, std::make_shared<Node>(*node));
    }
    if (checkpoint.nodes.size() != m_pData->m_Nodes.size())
        return false;

    checkpoint.mpcMultipliers = m_mpcMultipliers;
    std::vector<std::shared_ptr<ElementBase>> checkpointedElements;
    checkpointedElements.reserve(m_pData->m_Elements.size());
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (!element)
            continue;
        if (!element->PushStateCheckpoint())
        {
            for (const auto& checkpointedElement : checkpointedElements)
                checkpointedElement->DiscardStateCheckpoint();
            return false;
        }
        checkpointedElements.push_back(element);
    }
    m_stateCheckpoint = std::move(checkpoint);
    return true;
}

bool AnalysisStep::RestoreStateCheckpoint()
{
    InvalidateReferenceRhsCache();
    InvalidateLocalRayleighCache();
    if (!m_pData || !m_stateCheckpoint.has_value())
        return false;

    const StateCheckpoint& checkpoint = *m_stateCheckpoint;
    if (checkpoint.nodes.size() != m_pData->m_Nodes.size())
        return false;
    for (const auto& [nodeId, state] : checkpoint.nodes)
    {
        const auto found = m_pData->m_Nodes.find(nodeId);
        if (found == m_pData->m_Nodes.end() || !found->second || !state)
            return false;
        *found->second = *state;
    }
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (element && !element->RestoreStateCheckpoint())
            return false;
    }
    m_mpcMultipliers = checkpoint.mpcMultipliers;
    return true;
}

void AnalysisStep::DiscardStateCheckpoint()
{
    if (m_pData && m_stateCheckpoint.has_value())
    {
        for (const auto& [elementId, element] : m_pData->m_Elements)
        {
            Q_UNUSED(elementId);
            if (element)
                element->DiscardStateCheckpoint();
        }
    }
    m_stateCheckpoint.reset();
}

void AnalysisStep::AssembleExternalLoadTangent(double time, double loadFactor, double velocityDerivative,
                                               SpMat& externalTangent)
{
    QElapsedTimer timer;
    timer.start();
    const auto recordDuration = [this, &timer]
    {
        m_performanceStatistics.externalTangentMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
        ++m_performanceStatistics.externalTangentCount;
    };
    externalTangent.resize(m_nFree, m_nFree);
    externalTangent.setZero();
    if (!m_pData || (!m_EnableGalloping && !m_referenceGravityEquilibrium))
    {
        recordDuration();
        return;
    }

    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        Q_UNUSED(loadId);
        if (!load || load->m_LoadType != EnumKeyword::LoadType::FORCE_GRAVITY ||
            !IsStepScopedDataActive(load->m_StepId))
            continue;
        double scale = 1.0;
        if (load->m_StepId == m_Id)
        {
            if (!load->IsActive(time))
                continue;
            scale = loadFactor * load->GetScaleFactor(time);
        }
        SpMat gravityTangent;
        LoadAssembler::AssembleEccentricGravityTangent(*static_cast<const Force_Gravity*>(load.get()), *m_pData,
                                                       m_nFixed, m_nFree, scale, gravityTangent);
        externalTangent += gravityTangent;
        // 普通非索单元的分布重力按当前弦线长度与方向求等效节点力；
        // 其对节点平移的导数也必须进入 Newton 与动力线性化矩阵。
        LoadAssembler::AssembleDeformedGravityTangent(*static_cast<const Force_Gravity*>(load.get()), *m_pData,
                                                      m_nFixed, m_nFree, scale, gravityTangent);
        externalTangent += gravityTangent;
    }

    if (m_referenceGravityEquilibrium)
    {
        recordDuration();
        return;
    }

    const Force_Wind* gallopingWind = nullptr;
    double gallopingScale = 0.0;
    for (const auto& [loadId, load] : m_pData->m_Load)
    {
        Q_UNUSED(loadId);
        if (!load || load->m_LoadType != EnumKeyword::LoadType::FORCE_WIND || !IsStepScopedDataActive(load->m_StepId))
            continue;
        double scale = 1.0;
        if (load->m_StepId == m_Id)
        {
            if (!load->IsActive(time))
                continue;
            scale = loadFactor * load->GetScaleFactor(time);
        }
        gallopingWind = static_cast<const Force_Wind*>(load.get());
        gallopingScale = scale;
    }
    if (!gallopingWind || gallopingScale == 0.0)
    {
        recordDuration();
        return;
    }

    SpMat aerodynamicTangent;
    LoadAssembler::AssembleGallopingTangent(*gallopingWind, *m_pData, m_gallopingProfileBindings,
                                             m_GallopingIceThickness, m_GallopingInitialAttackDegrees,
                                             GetModelUpDirection(), m_nFixed, m_nFree, gallopingScale,
                                             velocityDerivative, aerodynamicTangent,
                                             m_DynamicSolverType == SolverNameSpace::SolverType::ReferenceTSSBN);
    externalTangent += aerodynamicTangent;
    recordDuration();
}

void AnalysisStep::EvaluateDynamicElement(const ElementAssemblyMap& assemblyMap, _OUT DynamicElementData& result)
{
    const int dofCount = static_cast<int>(assemblyMap.dofs.size());
    result.mass.setZero(dofCount, dofCount);
    result.inertiaForce.setZero(dofCount);
    result.velocityTangent.setZero(dofCount, dofCount);
    result.configurationTangent.setZero(dofCount, dofCount);
    assemblyMap.element->GetDynamicContributions(result.mass, result.inertiaForce, result.velocityTangent,
                                                 result.configurationTangent);
}

void AnalysisStep::EvaluateRigidBodyInertia(const RigidBodyInertiaAssemblyMap& assemblyMap,
                                            _OUT DynamicElementData& result)
{
    assemblyMap.inertia->GetDynamicContributions(result.mass, result.inertiaForce, result.velocityTangent,
                                                 result.configurationTangent);
}

void AnalysisStep::AccumulateInertiaForce(const std::vector<int>& dofs, const Eigen::VectorXd& elementForce,
                                          Eigen::VectorXd& globalForce) const
{
    for (int localDof = 0; localDof < static_cast<int>(dofs.size()); ++localDof)
    {
        const int freeDof = dofs[localDof] - m_nFixed;
        if (freeDof >= 0 && freeDof < m_nFree)
            globalForce[freeDof] += elementForce[localDof];
    }
}

bool AnalysisStep::AssembleNonlinearMPC(SolverNameSpace::NonlinearMPCData& constraints)
{
    constraints.Clear();
    if (!m_pData || m_pData->m_MPCConstraints.empty())
        return true;

    std::vector<SolverNameSpace::NonlinearMPCData> contributions;
    int equationCount = 0;
    for (const auto& [id, mpc] : m_pData->m_MPCConstraints)
    {
        if (!mpc || !IsStepScopedDataActive(mpc->m_StepId))
            continue;
        SolverNameSpace::NonlinearMPCData contribution;
        if (!mpc->Evaluate(m_nFixed, m_nFree, contribution))
        {
            qDebug().noquote() << QStringLiteral("Error: MPC %1在当前状态无法组装，"
                                                 "请检查从自由度和约束Jacobian。")
                                      .arg(id);
            return false;
        }
        equationCount += static_cast<int>(contribution.value.size());
        contributions.push_back(std::move(contribution));
    }
    if (equationCount == 0)
        return true;

    constraints.value = Eigen::VectorXd::Zero(equationCount);
    constraints.jacobian = Eigen::MatrixXd::Zero(equationCount, m_nFree);
    constraints.hessians.resize(equationCount);
    constraints.hessianEntries.resize(equationCount);
    constraints.slaveDofs.reserve(equationCount);

    int row = 0;
    for (auto& contribution : contributions)
    {
        const int rows = static_cast<int>(contribution.value.size());
        constraints.value.segment(row, rows) = contribution.value;
        constraints.jacobian.middleRows(row, rows) = contribution.jacobian;
        if (static_cast<int>(contribution.hessians.size()) == rows)
            for (int i = 0; i < rows; ++i)
                constraints.hessians[row + i] = std::move(contribution.hessians[i]);
        if (static_cast<int>(contribution.hessianEntries.size()) == rows)
            for (int i = 0; i < rows; ++i)
                constraints.hessianEntries[row + i] = std::move(contribution.hessianEntries[i]);
        constraints.slaveDofs.insert(constraints.slaveDofs.end(), contribution.slaveDofs.begin(),
                                     contribution.slaveDofs.end());
        row += rows;
    }
    return constraints.IsValid(m_nFree);
}

bool AnalysisStep::HasActiveNonlinearMPC() const
{
    if (!m_pData)
        return false;
    return std::any_of(m_pData->m_MPCConstraints.cbegin(), m_pData->m_MPCConstraints.cend(),
                       [this](const auto& entry)
                       {
                           return entry.second && IsStepScopedDataActive(entry.second->m_StepId);
                       });
}

void AnalysisStep::SetNonlinearMPCMultipliers(const SolverNameSpace::Vec& multipliers)
{
    m_mpcMultipliers = multipliers;
}

void AnalysisStep::ComputeExternalForce(double time, double loadFactor, VectorXd& F1, VectorXd& F2)
{
    double factor = loadFactor;
    Assemble_AllLoads(F1, F2, factor, time);
}

bool AnalysisStep::ComputeReferenceOrderedResidual(const SolverNameSpace::Vec& externalForce,
                                                   _OUT SolverNameSpace::Vec& residual)
{
    if (!UsesReferenceRhsOrder())
        return false;
    m_referenceOrderedResidual.resize(0);
    const int totalDofs = m_nFixed + m_nFree;
    if (m_referenceLoadStateVersion != m_referenceRhsStateVersion || externalForce.size() != m_nFree ||
        m_referenceTotalExternalForce.size() != totalDofs ||
        !(externalForce.array() == m_referenceTotalExternalForce.tail(m_nFree).array()).all())
        return false;

    EnsureAssemblyPattern();
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        std::fill(node->m_Force.begin(), node->m_Force.end(), 0.0);
    }
    VectorXd orderedInternal = VectorXd::Zero(totalDofs);
    VectorXd dynamicForce;
    std::vector<int> localDofCounts;
    for (std::size_t elementIndex = 0; elementIndex < m_assemblyPattern.elements.size(); ++elementIndex)
    {
        const ElementAssemblyMap& assemblyMap = m_assemblyPattern.elements[elementIndex];
        const int dofCount = static_cast<int>(assemblyMap.dofs.size());
        if (assemblyMap.referenceInertiaStateVersion != m_referenceRhsStateVersion)
        {
            // CPU 刷新入口均已缓存局部惯性；未提供局部量的加速入口按当前状态补取。
            assemblyMap.element->Get_InertiaForce(dynamicForce);
            CacheReferenceInertia(assemblyMap, dynamicForce);
        }
        dynamicForce = assemblyMap.referenceInertiaForce;
        if (dynamicForce.size() != dofCount || assemblyMap.element->m_inforce.size() != dofCount)
            throw std::runtime_error("Reference ordered element force size mismatch");
        // 与参考残差共用单元遍历，保留节点结构内力的原累加顺序。
        assemblyMap.element->GetNodeLocalDOFCounts(localDofCounts);
        int localOffset = 0;
        for (int nodeIndex = 0; nodeIndex < assemblyMap.element->m_pNode.size(); ++nodeIndex)
        {
            const int localDofs = localDofCounts[static_cast<std::size_t>(nodeIndex)];
            const auto node = assemblyMap.element->m_pNode[nodeIndex].lock();
            if (!node)
            {
                localOffset += localDofs;
                continue;
            }
            const int nodeDofs = std::min(localDofs, static_cast<int>(node->m_Force.size()));
            for (int dofIndex = 0; dofIndex < nodeDofs; ++dofIndex)
                node->m_Force[dofIndex] += assemblyMap.element->m_inforce[localOffset + dofIndex];
            localOffset += localDofs;
        }

        for (int localDof = 0; localDof < dofCount; ++localDof)
        {
            const int globalDof = assemblyMap.dofs[localDof];
            if (globalDof < 0 || globalDof >= totalDofs)
                continue;
            orderedInternal[globalDof] += assemblyMap.element->m_inforce[localDof];
            orderedInternal[globalDof] += dynamicForce[localDof];
        }
    }
    for (const RigidBodyInertiaAssemblyMap& assemblyMap : m_assemblyPattern.rigidBodyInertias)
    {
        if (!assemblyMap.inertia)
            continue;
        if (assemblyMap.referenceInertiaStateVersion != m_referenceRhsStateVersion)
        {
            DynamicElementData inertiaData;
            EvaluateRigidBodyInertia(assemblyMap, inertiaData);
            CacheReferenceInertia(assemblyMap, inertiaData.inertiaForce);
        }
        for (int localDof = 0; localDof < static_cast<int>(assemblyMap.dofs.size()); ++localDof)
        {
            const int globalDof = assemblyMap.dofs[localDof];
            if (globalDof >= 0 && globalDof < totalDofs)
                orderedInternal[globalDof] += assemblyMap.referenceInertiaForce[localDof];
        }
    }

    if (HasActiveLocalRayleighDamping())
    {
        VectorXd dampingForce;
        ComputeLocalRayleighForce(dampingForce);
        orderedInternal.tail(m_nFree) += dampingForce;
        VectorXd fixedDampingForce;
        ComputeLocalRayleighFixedForce(fixedDampingForce);
        orderedInternal.head(m_nFixed) += fixedDampingForce;
    }
    m_referenceOrderedResidual = m_referenceBaseExternalForce - orderedInternal;
    m_referenceOrderedResidual += m_referenceAerodynamicForce;
    residual = m_referenceOrderedResidual.tail(m_nFree);
    return true;
}

void AnalysisStep::ComputeResidual(const SolverNameSpace::Vec& F_ext, SolverNameSpace::Vec& R)
{
    QElapsedTimer timer;
    timer.start();
    if (ComputeReferenceOrderedResidual(F_ext, R))
    {
        m_performanceStatistics.residualMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
        ++m_performanceStatistics.residualCount;
        return;
    }
    VectorXd internalForce = VectorXd::Zero(m_nFree);
    for (auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        std::fill(node->m_Force.begin(), node->m_Force.end(), 0.0);
    }
    Get_CurrentInforce(internalForce);
    if (m_Type == EnumKeyword::StepType::DYNAMIC)
    {
        if (m_dynamicInertiaForce.size() == m_nFree)
            internalForce += m_dynamicInertiaForce;
        if (HasActiveLocalRayleighDamping())
        {
            VectorXd dampingForce;
            ComputeLocalRayleighForce(dampingForce);
            internalForce += dampingForce;
        }
    }
    R = F_ext - internalForce;
    m_performanceStatistics.residualMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.residualCount;
}

bool AnalysisStep::GetReferenceLinearSolvePermutation(_OUT std::vector<int>& signedSource) const
{
    signedSource.clear();
    const int totalDofs = m_nFixed + m_nFree;
    if (m_referenceNormSignedDofs.size() != static_cast<std::size_t>(totalDofs) || m_nFree <= 0)
        return false;
    signedSource.reserve(static_cast<std::size_t>(m_nFree));
    for (int targetDof = m_nFixed; targetDof < totalDofs; ++targetDof)
    {
        const int source = m_referenceNormSignedDofs[static_cast<std::size_t>(targetDof)];
        const int freeSource = std::abs(source) - m_nFixed;
        if (freeSource <= 0 || freeSource > m_nFree)
        {
            signedSource.clear();
            return false;
        }
        signedSource.push_back(source < 0 ? -freeSource : freeSource);
    }
    return true;
}

double AnalysisStep::GetReferenceVectorNorm(const SolverNameSpace::Vec& vector) const
{
    const int totalDofs = m_nFixed + m_nFree;
    if (m_referenceNormSignedDofs.size() != static_cast<std::size_t>(totalDofs) ||
        (vector.size() != m_nFree && vector.size() != totalDofs))
        return vector.norm();
    const bool freeOnly = vector.size() == m_nFree;
    SolverNameSpace::Vec referenceVector = SolverNameSpace::Vec::Zero(totalDofs);
    for (int targetDof = 0; targetDof < totalDofs; ++targetDof)
    {
        const int signedSource = m_referenceNormSignedDofs[static_cast<std::size_t>(targetDof)];
        const int sourceDof = std::abs(signedSource) - 1;
        if (freeOnly && sourceDof < m_nFixed)
            continue;
        const double value = vector[freeOnly ? sourceDof - m_nFixed : sourceDof];
        referenceVector[targetDof] = signedSource < 0 ? -value : value;
    }
    return referenceVector.norm();
}

void AnalysisStep::GetReferenceConvergenceResidual(const SolverNameSpace::Vec& freeResidual,
                                                   const SolverNameSpace::Vec& constrainedExternalForce,
                                                   _OUT SolverNameSpace::Vec& residual) const
{
    if (!m_pData || freeResidual.size() != m_nFree || constrainedExternalForce.size() != m_nFixed)
        throw std::runtime_error("Reference convergence residual size mismatch");
    if (UsesReferenceRhsOrder() && m_referenceOrderedResidual.size() == m_nFixed + m_nFree &&
        m_referenceLoadStateVersion == m_referenceRhsStateVersion &&
        (constrainedExternalForce.array() == m_referenceTotalExternalForce.head(m_nFixed).array()).all())
    {
        residual = m_referenceOrderedResidual;
        residual.tail(m_nFree) = freeResidual;
        return;
    }
    residual.resize(m_nFixed + m_nFree);
    residual.head(m_nFixed) = constrainedExternalForce;
    residual.tail(m_nFree) = freeResidual;
    for (const auto& [nodeId, node] : m_pData->m_Nodes)
    {
        Q_UNUSED(nodeId);
        if (!node)
            continue;
        for (int localDof = 0; localDof < static_cast<int>(node->m_DOF.size()); ++localDof)
        {
            const int globalDof = node->m_DOF[localDof];
            if (globalDof >= 0 && globalDof < m_nFixed && localDof < static_cast<int>(node->m_Force.size()))
                residual[globalDof] -= node->m_Force[localDof];
        }
    }
    if (m_Type != EnumKeyword::StepType::DYNAMIC)
        return;

    if (HasActiveLocalRayleighDamping())
    {
        VectorXd fixedDampingForce;
        ComputeLocalRayleighFixedForce(fixedDampingForce);
        residual.head(m_nFixed) -= fixedDampingForce;
    }
    std::vector<int> dofs;
    VectorXd inertiaForce;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        Q_UNUSED(elementId);
        if (!element)
            continue;
        element->GetDOFs(dofs);
        const bool hasFixedDof = std::any_of(dofs.cbegin(), dofs.cend(),
                                             [this](int dof) { return dof >= 0 && dof < m_nFixed; });
        if (!hasFixedDof)
            continue;
        element->Get_InertiaForce(inertiaForce);
        if (inertiaForce.size() != static_cast<int>(dofs.size()))
            throw std::runtime_error("Reference convergence inertia size mismatch");

        for (int localDof = 0; localDof < static_cast<int>(dofs.size()); ++localDof)
        {
            const int globalDof = dofs[localDof];
            if (globalDof >= 0 && globalDof < m_nFixed)
                residual[globalDof] -= inertiaForce[localDof];
        }
    }
    for (const auto& [inertiaId, inertia] : m_pData->m_RigidBodyInertias)
    {
        Q_UNUSED(inertiaId);
        if (!inertia)
            continue;
        inertia->GetDOFs(dofs);
        MatrixXd mass;
        MatrixXd velocityTangent;
        MatrixXd configurationTangent;
        inertia->GetDynamicContributions(mass, inertiaForce, velocityTangent, configurationTangent);
        for (int localDof = 0; localDof < static_cast<int>(dofs.size()); ++localDof)
        {
            const int globalDof = dofs[localDof];
            if (globalDof >= 0 && globalDof < m_nFixed)
                residual[globalDof] -= inertiaForce[localDof];
        }
    }
}

void AnalysisStep::ComputeStaticResidual(const SolverNameSpace::Vec& F_ext, SolverNameSpace::Vec& R)
{
    InvalidateLocalRayleighCache();
    VectorXd internalForce = VectorXd::Zero(m_nFree);
    for (auto& nodePair : m_pData->m_Nodes)
        std::fill(nodePair.second->m_Force.begin(), nodePair.second->m_Force.end(), 0.0);
    Get_CurrentInforce(internalForce);
    R = F_ext - internalForce;
}

bool AnalysisStep::EvaluateEnergy(double time, _OUT EnergyResult& energy)
{
    energy = EnergyResult{};
    if (!m_pData || !std::isfinite(time))
        return false;
    energy.availableMask = EnergyResult::Kinetic | EnergyResult::Strain | EnergyResult::Gravity;
    for (const auto& [id, element] : m_pData->m_Elements)
    {
        Q_UNUSED(id);
        double value = 0.0;
        if (element && element->TryGetStrainEnergy(value) && std::isfinite(value))
            energy.strain += value;
        else
            energy.availableMask &= ~EnergyResult::Strain;
        if (m_Type != EnumKeyword::StepType::DYNAMIC)
            continue;
        if (element && element->TryGetKineticEnergy(value) && std::isfinite(value))
            energy.kinetic += value;
        else
            energy.availableMask &= ~EnergyResult::Kinetic;
    }
    if (m_Type == EnumKeyword::StepType::DYNAMIC)
    {
        for (const auto& [id, inertia] : m_pData->m_RigidBodyInertias)
        {
            Q_UNUSED(id);
            const auto node = inertia ? inertia->m_pNode.lock() : nullptr;
            if (!node || node->m_Velocity.size() < 6)
            {
                energy.availableMask &= ~EnergyResult::Kinetic;
                continue;
            }
            const Eigen::Vector3d velocity(node->m_Velocity[0], node->m_Velocity[1], node->m_Velocity[2]);
            const Eigen::Vector3d angularVelocity(node->m_Velocity[3], node->m_Velocity[4], node->m_Velocity[5]);
            const Eigen::Vector3d materialSpin = node->m_Rg.transpose() * angularVelocity;
            energy.kinetic += 0.5 * (inertia->m_Mass * velocity.squaredNorm() +
                                      materialSpin.dot(inertia->m_RotaryInertia * materialSpin));
        }
    }

    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
    Eigen::Vector3d eccentricGravity = Eigen::Vector3d::Zero();
    const double factor = m_Type == EnumKeyword::StepType::STATIC && m_Time > 0.0
                              ? std::clamp(time / m_Time, 0.0, 1.0)
                              : 1.0;
    for (const auto& [id, load] : m_pData->m_Load)
    {
        Q_UNUSED(id);
        const auto* gravityLoad = dynamic_cast<const Force_Gravity*>(load.get());
        if (!gravityLoad || !IsStepScopedDataActive(gravityLoad->m_StepId))
            continue;
        const int component = static_cast<int>(gravityLoad->m_Direction);
        if (component < 0 || component >= 3)
        {
            energy.availableMask &= ~EnergyResult::Gravity;
            continue;
        }
        const double scale = gravityLoad->m_StepId == m_Id ? factor * gravityLoad->GetScaleFactor(time) : 1.0;
        gravity[component] += gravityLoad->m_g * scale;
        if (!gravityLoad->m_DiagnosticIgnoreEccentricMoment)
            eccentricGravity[component] += gravityLoad->m_g * scale;
    }
    if (!LoadAssembler::EvaluateGravityPotential(*m_pData, gravity, eccentricGravity, energy.gravity))
        energy.availableMask &= ~EnergyResult::Gravity;
    energy.FinalizeTotals();
    return energy.availableMask == EnergyResult::All;
}

void AnalysisStep::OnStepCompleted(double time)
{
    if (!m_pData)
        return;

    QElapsedTimer timer;
    timer.start();
    if (m_Type == EnumKeyword::StepType::DYNAMIC)
        RecoverDynamicReactions(time);
    EnergyResult energy;
    if (m_pData->GetOutputter().HasResultSink())
        EvaluateEnergy(time, energy);
    m_pData->GetOutputter().SaveDataFromNodes(time, m_pData, energy);

    m_performanceStatistics.outputMilliseconds += static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    ++m_performanceStatistics.outputCount;

    if (m_Type != EnumKeyword::StepType::STATIC)
        return;

    QStringList springResults;
    for (const auto& [elementId, element] : m_pData->m_Elements)
    {
        const auto* spring = dynamic_cast<const ElementSpringBase*>(element.get());
        if (!spring)
            continue;

        springResults.append(QStringLiteral("EID=%1  REL_DISP=%2  AXIAL=%3")
                                 .arg(elementId, 6)
                                 .arg(spring->m_CurrentRelativeDisplacement, 20, 'g', 12)
                                 .arg(spring->m_CurrentForce, 20, 'g', 12));
    }
    if (!springResults.isEmpty())
    {
        Logger::Instance().Diagnostic(QStringLiteral("[弹簧结果]  TIME=%1  %2")
                                          .arg(time, 20, 'g', 12)
                                          .arg(springResults.join(QStringLiteral("  "))));
    }
}

void AnalysisStep::RecordStepIterations(double time, int iterations)
{
    if (m_pData)
        m_pData->GetOutputter().RecordSolverIteration(time, iterations);
}

void AnalysisStep::CommitState()
{
    InvalidateLocalRayleighCache();
    if (!m_pData)
    {
        return;
    }

    for (auto& elementPair : m_pData->m_Elements)
    {
        elementPair.second->CommitState();
    }
    StoreCurrentNodeState();
}

VectorXd AnalysisStep::GetCurrentVelocity() const
{
    VectorXd v(m_nFree);
    v.setZero();

    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        for (int i = 0; i < numDOF; ++i)
        {
            int dof = pNode->m_DOF[i];
            if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
            {
                int idx = dof - m_nFixed;
                if (i < pNode->m_Velocity.size())
                    v[idx] = pNode->m_Velocity[i];
            }
        }
    }
    return v;
}

VectorXd AnalysisStep::GetCurrentAcceleration() const
{
    VectorXd a(m_nFree);
    a.setZero();

    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        for (int i = 0; i < numDOF; ++i)
        {
            int dof = pNode->m_DOF[i];
            if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
            {
                int idx = dof - m_nFixed;
                if (i < pNode->m_Acceleration.size())
                    a[idx] = pNode->m_Acceleration[i];
            }
        }
    }
    return a;
}

void AnalysisStep::StoreCurrentNodeState()
{
    InvalidateLocalRayleighCache();
    if (!m_pData)
    {
        return;
    }

    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        pNode->m_Displacement_n = pNode->m_Displacement;
        pNode->m_Velocity_n = pNode->m_Velocity;
        pNode->m_Acceleration_n = pNode->m_Acceleration;
        pNode->m_Rg_n = pNode->m_Rg;
        pNode->m_OmegaMaterial_n = pNode->m_OmegaMaterial;
        pNode->m_AlphaMaterial_n = pNode->m_AlphaMaterial;
        pNode->m_StepTranslation.setZero();
        pNode->m_StepScalarRotation = 0.0;
        pNode->m_StepRotation.setZero();
        pNode->m_TssbnImplicitStage = Node::TssbnImplicitStageState{};
    }
}

void AnalysisStep::BackupStepState()
{
    InvalidateLocalRayleighCache();
    StoreCurrentNodeState();
    for (auto& elementPair : m_pData->m_Elements)
    {
        if (elementPair.second)
            elementPair.second->BackupState();
    }
}

void AnalysisStep::GetStepIncrement(SolverNameSpace::Vec& dx_step) const
{
    dx_step.resize(m_nFree);
    dx_step.setZero();

    for (auto& nodePair : m_pData->m_Nodes)
    {
        auto pNode = nodePair.second;
        int numDOF = pNode->m_DOF.size();

        if (numDOF == 6)
        {
            // 独立保存小步平移，不能从两个大的累计位移相减恢复。
            for (int dofIdx = 0; dofIdx < 3; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    dx_step[dof - m_nFixed] = pNode->m_StepTranslation[dofIdx];
                }
            }

            // 2. 转动自由度：必须通过矩阵相对变换求出步内增量！
            // 由于你在 ApplyIncrement 中使用的是左乘 R_new = delta_R * R_old
            // 所以当前的增量旋转矩阵 R_step = R_curr * R_n^T
            Eigen::Matrix3d R_step = pNode->m_Rg * pNode->m_Rg_n.transpose();
            Eigen::Vector3d delta_theta_step;
            Utility::CR::Extract_RotationVector(R_step, delta_theta_step);

            for (int dofIdx = 3; dofIdx < 6; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    // 填入真实的 SO(3) 空间角位移增量
                    dx_step[dof - m_nFixed] = delta_theta_step(dofIdx - 3);
                }
            }
        }
        else // 对于非 6 自由度节点
        {
            for (int dofIdx = 0; dofIdx < numDOF; ++dofIdx)
            {
                int dof = pNode->m_DOF[dofIdx];
                if (dof >= m_nFixed && dof < m_nFixed + m_nFree)
                {
                    if (dofIdx < 3)
                        dx_step[dof - m_nFixed] = pNode->m_StepTranslation[dofIdx];
                    else if (numDOF == 4 && dofIdx == 3)
                        dx_step[dof - m_nFixed] = pNode->m_StepScalarRotation;
                    else
                        dx_step[dof - m_nFixed] = pNode->m_Displacement[dofIdx] - pNode->m_Displacement_n[dofIdx];
                }
            }
        }
    }
}
