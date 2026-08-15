#include "Hdf5ModelIO.h"

#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Node/Node.h"
#include "DataStructure/Aerodynamics/AeroManager.h"
#include "Export/Outputter.h"
#include "Utility/CR.h"

#ifndef H5_BUILT_AS_DYNAMIC_LIB
#define H5_BUILT_AS_DYNAMIC_LIB
#endif
#include <vtk_hdf5.h>

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QSaveFile>
#include <QFileInfo>
#include <QMutex>
#include <QHash>
#include <QTextStream>
#include <QUuid>

#ifdef Q_OS_WIN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <vector>

namespace
{
// VTK 自带的 HDF5 未启用 H5_HAVE_THREADSAFE。求解任务可能并发运行，因此进入 HDF5 C API 时必须串行化。
QRecursiveMutex g_hdf5ApiMutex;

constexpr int kModelDomainId = 1;
constexpr int kDynamicStateFormatVersion = 1;
constexpr int kResultFormatVersion = 2;
constexpr size_t kEntityNameSize = 256;

bool ReadIntAttribute(hid_t object, const char* name, int& value);

QString MakeAsciiTempHdf5FileName()
{
    return QDir::temp().filePath("yqy_h5_" + QUuid::createUuid().toString(QUuid::Id128) + ".tmp");
}

bool IsAsciiPath(const QString& path)
{
    for (const QChar value : path)
    {
        if (value.unicode() > 0x7f)
            return false;
    }
    return true;
}

QString MakeAsciiTempHdf5FileNameNear(const QString& targetFileName)
{
    const QFileInfo targetInfo(targetFileName);
    const QString directory = targetInfo.absolutePath();
    if (IsAsciiPath(directory))
    {
        return QDir(directory).filePath(".yqy_h5_" + QUuid::createUuid().toString(QUuid::Id128) + ".tmp");
    }
    return MakeAsciiTempHdf5FileName();
}

QByteArray ToHdf5Path(const QString& fileName)
{
    return QDir::toNativeSeparators(fileName).toUtf8();
}

bool LinkExistsQuietly(hid_t file, const char* path)
{
    htri_t exists = -1;
    H5E_BEGIN_TRY
    {
        exists = H5Lexists(file, path, H5P_DEFAULT);
    }
    H5E_END_TRY;
    return exists > 0;
}

bool HasRequiredInputSchema(hid_t file)
{
    static constexpr const char* requiredPaths[] = {"/YQY/INPUT/NODE/GRID",       "/YQY/INPUT/MATERIAL/MAT",
                                                    "/YQY/INPUT/SECTION/SECTION", "/YQY/INPUT/PROPERTY/PROPERTY",
                                                    "/YQY/INPUT/ELEMENT/ELEMENT", "/YQY/INPUT/CONSTRAINT/SPC",
                                                    "/YQY/INPUT/LOAD/LOAD",       "/YQY/INPUT/ANALYSIS_STEP/STEP"};

    for (const char* path : requiredPaths)
    {
        if (!LinkExistsQuietly(file, path))
        {
            return false;
        }
    }
    return true;
}

bool HasRequiredResultSchema(hid_t file)
{
    static constexpr const char* requiredPaths[] = {"/YQY/RESULT/DOMAINS",
                                                    "/YQY/RESULT/NODAL/DISPLACEMENT",
                                                    "/YQY/RESULT/NODAL/CURRENT_COORDINATE",
                                                    "/YQY/RESULT/NODAL/VELOCITY",
                                                    "/YQY/RESULT/NODAL/ACCELERATION",
                                                    "/YQY/RESULT/NODAL/REACTION_FORCE",
                                                    "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE",
                                                    "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE",
                                                    "/YQY/RESULT/ELEMENTAL/CABLE_FORCE",
                                                    "/YQY/RESULT/ELEMENTAL/STRESS",
                                                    "/YQY/RESULT/ELEMENTAL/STRAIN",
                                                    "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT",
                                                    "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE",
                                                    "/INDEX/YQY/RESULT/NODAL/VELOCITY",
                                                    "/INDEX/YQY/RESULT/NODAL/ACCELERATION",
                                                    "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE",
                                                    "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE",
                                                    "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE",
                                                    "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE",
                                                    "/INDEX/YQY/RESULT/ELEMENTAL/STRESS",
                                                    "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN"};

    for (const char* path : requiredPaths)
    {
        if (LinkExistsQuietly(file, path))
            continue;
        return false;
    }
    int stateVersion = 0;
    int resultVersion = 0;
    const bool versionOk = ReadIntAttribute(file, "DYNAMIC_STATE_FORMAT_VERSION", stateVersion) &&
                           stateVersion >= kDynamicStateFormatVersion &&
                           ReadIntAttribute(file, "RESULT_FORMAT_VERSION", resultVersion) &&
                           resultVersion >= kResultFormatVersion;
    if (!versionOk)
        return false;

    static constexpr const char* requiredRangeAttributes[] = {"RESULT_RANGE_DISPLACEMENT_MAGNITUDE_VALID",
                                                              "RESULT_RANGE_DISPLACEMENT_X_VALID",
                                                              "RESULT_RANGE_DISPLACEMENT_Y_VALID",
                                                              "RESULT_RANGE_DISPLACEMENT_Z_VALID",
                                                              "RESULT_RANGE_AXIAL_FORCE_VALID",
                                                              "RESULT_RANGE_STRESS_VALID",
                                                              "RESULT_RANGE_STRAIN_VALID"};
    for (const char* attribute : requiredRangeAttributes)
    {
        if (H5Aexists(file, attribute) <= 0)
            return false;
    }
    return true;
}

bool MoveTempFileToTarget(const QString& tempFileName, const QString& targetFileName)
{
    if (tempFileName == targetFileName)
    {
        return true;
    }

    if (QFile::exists(targetFileName) && !QFile::remove(targetFileName))
    {
        qDebug() << "Failed to remove old HDF5 file:" << targetFileName;
        return false;
    }

    if (!QFile::rename(tempFileName, targetFileName))
    {
        qDebug() << "Failed to rename HDF5 temp file:" << tempFileName << "->" << targetFileName;
        return false;
    }

    return true;
}

QString CopyHdf5FileToAsciiTemp(const QString& sourceFileName)
{
    // HDF5 通过窄 UTF-8 路径打开。若只有结果文件名包含中文，则通过临时 ASCII 硬链接访问同一 NTFS 文件。
    // 该操作复杂度为 O(1)，可避免在检查、打开和导出时复制数 GB 的结果文件。
    const QFileInfo sourceInfo(sourceFileName);
    if (sourceInfo.exists() && IsAsciiPath(sourceInfo.absolutePath()))
    {
        const QString linkFileName =
            QDir(sourceInfo.absolutePath())
                .filePath(".yqy_h5_read_" + QUuid::createUuid().toString(QUuid::Id128) + ".tmp");
#ifdef Q_OS_WIN
        const std::wstring nativeLink = QDir::toNativeSeparators(linkFileName).toStdWString();
        const std::wstring nativeSource = QDir::toNativeSeparators(sourceInfo.absoluteFilePath()).toStdWString();
        if (CreateHardLinkW(nativeLink.c_str(), nativeSource.c_str(), nullptr) != FALSE)
            return linkFileName;
#endif
    }

    const QString tempFileName = MakeAsciiTempHdf5FileName();
    if (!QFile::copy(sourceFileName, tempFileName))
    {
        return QString();
    }
    return tempFileName;
}

struct H5Handle
{
    hid_t id = -1;
    herr_t (*closeFunc)(hid_t) = nullptr;

    H5Handle() = default;
    H5Handle(hid_t value, herr_t (*func)(hid_t))
        : id(value)
        , closeFunc(func)
    {
    }
    ~H5Handle()
    {
        reset();
    }

    H5Handle(const H5Handle&) = delete;
    H5Handle& operator=(const H5Handle&) = delete;

    H5Handle(H5Handle&& other) noexcept
        : id(other.id)
        , closeFunc(other.closeFunc)
    {
        other.id = -1;
        other.closeFunc = nullptr;
    }

    H5Handle& operator=(H5Handle&& other) noexcept
    {
        if (this != &other)
        {
            reset();
            id = other.id;
            closeFunc = other.closeFunc;
            other.id = -1;
            other.closeFunc = nullptr;
        }
        return *this;
    }

    void reset(hid_t value = -1, herr_t (*func)(hid_t) = nullptr)
    {
        if (id >= 0 && closeFunc)
        {
            closeFunc(id);
        }
        id = value;
        closeFunc = func;
    }

    bool valid() const
    {
        return id >= 0;
    }
    operator hid_t() const
    {
        return id;
    }
};

struct ExtendableDataset
{
    H5Handle handle;
    hsize_t size = 0;
};

struct StreamDatasetPair
{
    ExtendableDataset data;
    ExtendableDataset index;
};

class H5ReadErrorScope
{
public:
    H5ReadErrorScope()
    {
        if (H5Eget_auto2(H5E_DEFAULT, &m_previousHandler, &m_previousClientData) >= 0)
        {
            H5Eset_auto2(H5E_DEFAULT, nullptr, nullptr);
            m_restore = true;
        }
    }

    ~H5ReadErrorScope()
    {
        if (m_restore)
            H5Eset_auto2(H5E_DEFAULT, m_previousHandler, m_previousClientData);
    }

    H5ReadErrorScope(const H5ReadErrorScope&) = delete;
    H5ReadErrorScope& operator=(const H5ReadErrorScope&) = delete;

private:
    H5E_auto2_t m_previousHandler = nullptr;
    void* m_previousClientData = nullptr;
    bool m_restore = false;
};

bool ReportHdf5FormatError()
{
    qWarning().noquote() << QStringLiteral("H5格式错误");
    return false;
}

struct GridRecord
{
    int id = 0;
    double x[3] = {};
    int dofCount = 0;
    int domainId = kModelDomainId;
};

struct MaterialRecord
{
    int id = 0;
    double young = 0.0;
    double poisson = 0.0;
    double density = 0.0;
    double maxStress = 0.0;
    double expansion = 0.0;
    int domainId = kModelDomainId;
};

struct SectionRecord
{
    int id = 0;
    int type = 0;
    double area = 0.0;
    double radius = 0.0;
    double width = 0.0;
    double height = 0.0;
    double iy = 0.0;
    double iz = 0.0;
    double j = 0.0;
    int domainId = kModelDomainId;
};

struct PropertyRecord
{
    int id = 0;
    int materialId = 0;
    int sectionId = 0;
    int domainId = kModelDomainId;
};

struct ElementRecord
{
    int id = 0;
    int type = 0;
    int propertyId = 0;
    int nodeIds[2] = {};
    double q0[3] = {};
    double initStress = 0.0;
    int role = static_cast<int>(ElementRole::Generic);
    int wireId = -1;
    int aeroBundleCount = 0;
    int aeroProfileId = -1;
    int domainId = kModelDomainId;
};

struct ConstraintRecord
{
    int id = 0;
    int nodeId = 0;
    int direction = 0;
    double value = 0.0;
    int domainId = kModelDomainId;
};

struct MPCRecord
{
    int id = 0;
    int relationType = 0;
    int masterNodeId = 0;
    int slaveNodeId = 0;
    int slaveDirectionMask = 0;
    int stepId = 0;
    double parameters[3] = {};
    int domainId = kModelDomainId;
};

struct RigidBodyInertiaRecord
{
    int id = 0;
    int nodeId = 0;
    double mass = 0.0;
    double inertia[6] = {};
    int domainId = kModelDomainId;
};

struct LoadRecord
{
    int id = 0;
    int type = 0;
    int targetId = 0;
    int direction = 0;
    int stepId = 0;
    int functionType = 0;
    double value = 0.0;
    double startTime = 0.0;
    double endTime = 0.0;
    double params[8] = {};
    int domainId = kModelDomainId;
};

struct WindDirectionRecord
{
    int loadId = 0;
    double direction[3] = {0.0, 1.0, 0.0};
    int domainId = kModelDomainId;
};

struct StepRecord
{
    int id = 0;
    int type = 0;
    int isDynamic = 0;
    int dynamicSolverType = 0;
    double time = 0.0;
    double stepSize = 0.0;
    double tolerance = 0.0;
    int maxIterations = 0;
    int domainId = kModelDomainId;
};

struct ModelSetRecord
{
    int id = 0;
    int type = 0;
    char name[kEntityNameSize] = {};
    int domainId = kModelDomainId;
};

struct ModelSetMemberRecord
{
    int setId = 0;
    int entityId = 0;
    int domainId = kModelDomainId;
};

struct ComputeRegionRecord
{
    int id = 0;
    int enabled = 1;
    int autoGenerated = 0;
    char name[kEntityNameSize] = {};
    int domainId = kModelDomainId;
};

struct RegionLinkRecord
{
    int regionId = 0;
    int targetId = 0;
    int domainId = kModelDomainId;
};

struct StepMetadataRecord
{
    int stepId = 0;
    int regionScope = 0;
    int initialStaticStepId = 0;
    char name[kEntityNameSize] = {};
    int domainId = kModelDomainId;
};

struct StepGallopingRecord
{
    int stepId = 0;
    int enabled = 0;
    int iceThickness = 12;
    double initialAttackDegrees = 45.0;
    int aerodynamicTangentMode = -1;
    int domainId = kModelDomainId;
};

struct StepTssbnRecord
{
    int stepId = 0;
    double spectralRadiusInfinity = 0.0;
    double minimumTimeStep = 1.0e-6;
    double maximumTimeStep = 0.5;
    double relativeTolerance = 1.0e-3;
    double absoluteTolerance = 1.0e-6;
    double safetyFactor = 0.9;
    double shrinkFactor = 0.8;
    double maximumGrowthFactor = 3.0;
    int targetNewtonIterations = 8;
    int maximumTotalNewtonIterations = 32;
    double derivativeGain = 0.1;
    double minimumDerivativeFactor = 0.5;
    double maximumDerivativeFactor = 1.5;
    int maximumRejectedAttempts = 24;
    int domainId = kModelDomainId;
};

struct StepStructuralDampingRecord
{
    int stepId = 0;
    int enabled = 0;
    double translationDampingRatio = 0.005;
    double torsionDampingRatio = 0.038;
    double maximumFrequencyHz = 3.0;
    int domainId = kModelDomainId;
};

struct StepRegionLinkRecord
{
    int stepId = 0;
    int regionId = 0;
    int domainId = kModelDomainId;
};

struct AeroCaseRecord
{
    int id = 0;
    int bundleCount = 0;
    int windSpeed = 0;
    int iceThickness = 0;
    int modelCount = 0;
    int dataSize = 0;
    double startAngle = 0.0;
    double angleStep = 0.0;
    char sourceFile[260] = {};
    char sourcePath[520] = {};
    int domainId = kModelDomainId;
};

struct AeroCoefficientRecord
{
    int caseId = 0;
    int modelIndex = 0;
    int angleIndex = 0;
    double angle = 0.0;
    double lift = 0.0;
    double drag = 0.0;
    double moment = 0.0;
    int domainId = kModelDomainId;
};

struct DomainRecord
{
    int id = 0;
    int stepId = 0;
    int increment = 0;
    int analysis = 0;
    double time = 0.0;
    double loadFactor = 1.0;
};

struct SolverIterationH5Record
{
    int stepId = 0;
    int analysisType = 0;
    double time = 0.0;
    int iterations = 0;
};

struct IndexRecord
{
    int domainId = 0;
    long long position = 0;
    long long length = 0;
};

struct NodalRecord
{
    int id = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    int domainId = 0;
};

struct ElementForceRecord
{
    int id = 0;
    double axial = 0.0;
    double shearY = 0.0;
    double shearZ = 0.0;
    double torque = 0.0;
    double momentY = 0.0;
    double momentZ = 0.0;
    int domainId = 0;
};

// 仅含轴力的桁架结果使用紧凑记录；与梁内力分开可避免每一结果帧为每个桁架存储五个恒为零的分量。
struct TrussForceRecord
{
    int id = 0;
    int domainId = 0;
    double axial = 0.0;
};

// 索结果预留扭矩以支持扭转公式，但剪力和弯矩仍不适用。
struct CableForceRecord
{
    int id = 0;
    int domainId = 0;
    double axial = 0.0;
    double torque = 0.0;
};

struct ElementStressRecord
{
    int id = 0;
    double initStress = 0.0;
    double currentStress = 0.0;
    double deltaStress = 0.0;
    int domainId = 0;
};

struct ElementStrainRecord
{
    int id = 0;
    double strain = 0.0;
    int domainId = 0;
};

int ToInt(EnumKeyword::ElementType value)
{
    return static_cast<int>(value);
}

int ToInt(EnumKeyword::SectionType value)
{
    return static_cast<int>(value);
}

int ToInt(EnumKeyword::LoadType value)
{
    return static_cast<int>(value);
}

int ToInt(EnumKeyword::StepType value)
{
    return static_cast<int>(value);
}

int ToInt(EnumKeyword::Direction value)
{
    return static_cast<int>(value);
}

int ToInt(TimeFunctionType value)
{
    return static_cast<int>(value);
}

bool CreateGroupRecursive(hid_t file, const char* path)
{
    QString normalized = QString::fromLatin1(path);
    if (normalized.isEmpty() || normalized == "/")
    {
        return true;
    }

    if (normalized.startsWith('/'))
    {
        normalized.remove(0, 1);
    }

    QString currentPath;
    const QStringList parts = normalized.split('/', Qt::SkipEmptyParts);
    for (const QString& part : parts)
    {
        currentPath += "/" + part;
        const QByteArray name = currentPath.toUtf8();
        htri_t exists = H5Lexists(file, name.constData(), H5P_DEFAULT);
        if (exists > 0)
        {
            continue;
        }

        H5Handle group(H5Gcreate2(file, name.constData(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT), H5Gclose);
        if (!group.valid())
        {
            return false;
        }
    }
    return true;
}

bool WriteStringAttribute(hid_t object, const char* name, const QString& value)
{
    const QByteArray bytes = value.toUtf8();
    const size_t size = static_cast<size_t>(bytes.size() + 1);

    H5Handle type(H5Tcopy(H5T_C_S1), H5Tclose);
    if (!type.valid() || H5Tset_size(type, size) < 0 || H5Tset_strpad(type, H5T_STR_NULLTERM) < 0 ||
        H5Tset_cset(type, H5T_CSET_UTF8) < 0)
    {
        return false;
    }

    H5Handle space(H5Screate(H5S_SCALAR), H5Sclose);
    if (!space.valid())
    {
        return false;
    }

    if (H5Aexists(object, name) > 0)
    {
        H5Adelete(object, name);
    }

    H5Handle attr(H5Acreate2(object, name, type, space, H5P_DEFAULT, H5P_DEFAULT), H5Aclose);
    if (!attr.valid())
    {
        return false;
    }

    return H5Awrite(attr, type, bytes.constData()) >= 0;
}

bool WriteIntAttribute(hid_t object, const char* name, int value)
{
    H5Handle space(H5Screate(H5S_SCALAR), H5Sclose);
    if (!space.valid())
    {
        return false;
    }

    if (H5Aexists(object, name) > 0)
    {
        H5Adelete(object, name);
    }

    H5Handle attr(H5Acreate2(object, name, H5T_NATIVE_INT, space, H5P_DEFAULT, H5P_DEFAULT), H5Aclose);
    if (!attr.valid())
    {
        return false;
    }

    return H5Awrite(attr, H5T_NATIVE_INT, &value) >= 0;
}

bool ReadIntAttribute(hid_t object, const char* name, int& value)
{
    if (H5Aexists(object, name) <= 0)
        return false;
    H5Handle attr(H5Aopen(object, name, H5P_DEFAULT), H5Aclose);
    return attr.valid() && H5Aread(attr, H5T_NATIVE_INT, &value) >= 0;
}

bool WriteDoubleAttribute(hid_t object, const char* name, double value)
{
    H5Handle space(H5Screate(H5S_SCALAR), H5Sclose);
    if (!space.valid())
        return false;
    if (H5Aexists(object, name) > 0)
        H5Adelete(object, name);
    H5Handle attr(H5Acreate2(object, name, H5T_NATIVE_DOUBLE, space, H5P_DEFAULT, H5P_DEFAULT), H5Aclose);
    return attr.valid() && H5Awrite(attr, H5T_NATIVE_DOUBLE, &value) >= 0;
}

bool ReadDoubleAttribute(hid_t object, const char* name, double& value)
{
    if (H5Aexists(object, name) <= 0)
        return false;
    H5Handle attr(H5Aopen(object, name, H5P_DEFAULT), H5Aclose);
    return attr.valid() && H5Aread(attr, H5T_NATIVE_DOUBLE, &value) >= 0;
}

void IncludeResultRangeValue(Hdf5ResultRange& range, double value)
{
    if (!std::isfinite(value))
        return;
    if (!range.valid)
    {
        range.minimum = value;
        range.maximum = value;
        range.valid = true;
        return;
    }
    range.minimum = std::min(range.minimum, value);
    range.maximum = std::max(range.maximum, value);
}

bool WriteResultRangeAttributes(hid_t file, const Hdf5ResultRanges& ranges)
{
    const auto writeRange = [file](const char* prefix, const Hdf5ResultRange& range)
    {
        const QByteArray validName = QByteArray("RESULT_RANGE_") + prefix + "_VALID";
        const QByteArray minimumName = QByteArray("RESULT_RANGE_") + prefix + "_MIN";
        const QByteArray maximumName = QByteArray("RESULT_RANGE_") + prefix + "_MAX";
        return WriteIntAttribute(file, validName.constData(), range.valid ? 1 : 0) &&
               (!range.valid || (WriteDoubleAttribute(file, minimumName.constData(), range.minimum) &&
                                 WriteDoubleAttribute(file, maximumName.constData(), range.maximum)));
    };
    return writeRange("DISPLACEMENT_MAGNITUDE", ranges.displacementMagnitude) &&
           writeRange("DISPLACEMENT_X", ranges.displacementX) && writeRange("DISPLACEMENT_Y", ranges.displacementY) &&
           writeRange("DISPLACEMENT_Z", ranges.displacementZ) && writeRange("AXIAL_FORCE", ranges.axialForce) &&
           writeRange("STRESS", ranges.stress) && writeRange("STRAIN", ranges.strain);
}

bool InsertArray(hid_t type, const char* name, size_t offset, hid_t baseType, hsize_t count)
{
    H5Handle arrayType(H5Tarray_create2(baseType, 1, &count), H5Tclose);
    if (!arrayType.valid())
    {
        return false;
    }
    return H5Tinsert(type, name, offset, arrayType) >= 0;
}

bool InsertFixedString(hid_t type, const char* name, size_t offset, size_t size)
{
    H5Handle stringType(H5Tcopy(H5T_C_S1), H5Tclose);
    if (!stringType.valid() || H5Tset_size(stringType, size) < 0 || H5Tset_strpad(stringType, H5T_STR_NULLTERM) < 0 ||
        H5Tset_cset(stringType, H5T_CSET_UTF8) < 0)
    {
        return false;
    }
    return H5Tinsert(type, name, offset, stringType) >= 0;
}

void CopyUtf8String(char* target, size_t targetSize, const QString& value)
{
    if (!target || targetSize == 0)
    {
        return;
    }

    std::memset(target, 0, targetSize);
    const QByteArray bytes = value.toUtf8();
    const size_t copySize = std::min(static_cast<size_t>(bytes.size()), targetSize - 1);
    if (copySize > 0)
    {
        std::memcpy(target, bytes.constData(), copySize);
    }
}

QString ReadUtf8String(const char* source, size_t sourceSize)
{
    if (!source || sourceSize == 0)
    {
        return {};
    }
    QByteArray bytes(source, static_cast<qsizetype>(sourceSize));
    const qsizetype terminator = bytes.indexOf('\0');
    if (terminator >= 0)
    {
        bytes.truncate(terminator);
    }
    return QString::fromUtf8(bytes);
}

QString PathToQString(const std::filesystem::path& path)
{
#ifdef _WIN32
    return QString::fromStdWString(path.wstring());
#else
    return QString::fromStdString(path.string());
#endif
}

template <typename Record>
bool WriteDataset(hid_t file, const char* path, hid_t memoryType, const std::vector<Record>& records)
{
    const QString datasetPath = QString::fromLatin1(path);
    const int lastSlash = datasetPath.lastIndexOf('/');
    if (lastSlash > 0)
    {
        const QByteArray groupPath = datasetPath.left(lastSlash).toUtf8();
        if (!CreateGroupRecursive(file, groupPath.constData()))
        {
            return false;
        }
    }

    const hsize_t dims[1] = {static_cast<hsize_t>(records.size())};
    H5Handle space(H5Screate_simple(1, dims, nullptr), H5Sclose);
    if (!space.valid())
    {
        return false;
    }

    if (H5Lexists(file, path, H5P_DEFAULT) > 0)
    {
        H5Ldelete(file, path, H5P_DEFAULT);
    }

    H5Handle dataset(H5Dcreate2(file, path, memoryType, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT), H5Dclose);
    if (!dataset.valid())
    {
        return false;
    }

    if (records.empty())
    {
        return true;
    }
    return H5Dwrite(dataset, memoryType, H5S_ALL, H5S_ALL, H5P_DEFAULT, records.data()) >= 0;
}

H5Handle CreateExtendableDataset(hid_t file, const char* path, hid_t memoryType)
{
    const QString datasetPath = QString::fromLatin1(path);
    const int lastSlash = datasetPath.lastIndexOf('/');
    if (lastSlash > 0)
    {
        const QByteArray groupPath = datasetPath.left(lastSlash).toUtf8();
        if (!CreateGroupRecursive(file, groupPath.constData()))
        {
            return {};
        }
    }

    if (H5Lexists(file, path, H5P_DEFAULT) > 0)
    {
        H5Ldelete(file, path, H5P_DEFAULT);
    }

    const hsize_t dims[1] = {0};
    const hsize_t maxDims[1] = {H5S_UNLIMITED};
    H5Handle space(H5Screate_simple(1, dims, maxDims), H5Sclose);
    if (!space.valid())
    {
        return {};
    }

    H5Handle plist(H5Pcreate(H5P_DATASET_CREATE), H5Pclose);
    if (!plist.valid())
    {
        return {};
    }

    const hsize_t chunkDims[1] = {1024};
    if (H5Pset_chunk(plist, 1, chunkDims) < 0)
    {
        return {};
    }

    return H5Handle(H5Dcreate2(file, path, memoryType, space, H5P_DEFAULT, plist, H5P_DEFAULT), H5Dclose);
}

template <typename Record>
bool AppendDataset(_OUT ExtendableDataset& dataset, hid_t memoryType, const std::vector<Record>& records,
                   _OUT long long& position)
{
    if (!dataset.handle.valid())
        return false;
    position = static_cast<long long>(dataset.size);
    if (records.empty())
        return true;

    const hsize_t newSize = dataset.size + static_cast<hsize_t>(records.size());
    const hsize_t newDims[1] = {newSize};
    if (H5Dset_extent(dataset.handle, newDims) < 0)
    {
        return false;
    }

    H5Handle fileSpace(H5Dget_space(dataset.handle), H5Sclose);
    if (!fileSpace.valid())
    {
        return false;
    }

    const hsize_t start[1] = {dataset.size};
    const hsize_t count[1] = {static_cast<hsize_t>(records.size())};
    if (H5Sselect_hyperslab(fileSpace, H5S_SELECT_SET, start, nullptr, count, nullptr) < 0)
    {
        return false;
    }

    H5Handle memSpace(H5Screate_simple(1, count, nullptr), H5Sclose);
    if (!memSpace.valid())
    {
        return false;
    }

    if (H5Dwrite(dataset.handle, memoryType, memSpace, fileSpace, H5P_DEFAULT, records.data()) < 0)
        return false;
    dataset.size = newSize;
    return true;
}

H5Handle CreateGridType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(GridRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(GridRecord, id), H5T_NATIVE_INT);
    InsertArray(type, "X", HOFFSET(GridRecord, x), H5T_NATIVE_DOUBLE, 3);
    H5Tinsert(type, "DOF_COUNT", HOFFSET(GridRecord, dofCount), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(GridRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateMaterialType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(MaterialRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(MaterialRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "E", HOFFSET(MaterialRecord, young), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "NU", HOFFSET(MaterialRecord, poisson), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "RHO", HOFFSET(MaterialRecord, density), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MAX_STRESS", HOFFSET(MaterialRecord, maxStress), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "EXPANSION", HOFFSET(MaterialRecord, expansion), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(MaterialRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateSectionType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(SectionRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(SectionRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "TYPE", HOFFSET(SectionRecord, type), H5T_NATIVE_INT);
    H5Tinsert(type, "AREA", HOFFSET(SectionRecord, area), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "RADIUS", HOFFSET(SectionRecord, radius), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "WIDTH", HOFFSET(SectionRecord, width), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "HEIGHT", HOFFSET(SectionRecord, height), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "IY", HOFFSET(SectionRecord, iy), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "IZ", HOFFSET(SectionRecord, iz), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "J", HOFFSET(SectionRecord, j), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(SectionRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreatePropertyType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(PropertyRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(PropertyRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "MID", HOFFSET(PropertyRecord, materialId), H5T_NATIVE_INT);
    H5Tinsert(type, "SID", HOFFSET(PropertyRecord, sectionId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(PropertyRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateElementType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ElementRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "EID", HOFFSET(ElementRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "TYPE", HOFFSET(ElementRecord, type), H5T_NATIVE_INT);
    H5Tinsert(type, "PID", HOFFSET(ElementRecord, propertyId), H5T_NATIVE_INT);
    InsertArray(type, "G", HOFFSET(ElementRecord, nodeIds), H5T_NATIVE_INT, 2);
    InsertArray(type, "Q0", HOFFSET(ElementRecord, q0), H5T_NATIVE_DOUBLE, 3);
    H5Tinsert(type, "INIT_STRESS", HOFFSET(ElementRecord, initStress), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "ROLE", HOFFSET(ElementRecord, role), H5T_NATIVE_INT);
    H5Tinsert(type, "WIRE_ID", HOFFSET(ElementRecord, wireId), H5T_NATIVE_INT);
    H5Tinsert(type, "AERO_BUNDLE_COUNT", HOFFSET(ElementRecord, aeroBundleCount), H5T_NATIVE_INT);
    H5Tinsert(type, "AERO_PROFILE_ID", HOFFSET(ElementRecord, aeroProfileId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ElementRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateConstraintType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ConstraintRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(ConstraintRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "G", HOFFSET(ConstraintRecord, nodeId), H5T_NATIVE_INT);
    H5Tinsert(type, "C", HOFFSET(ConstraintRecord, direction), H5T_NATIVE_INT);
    H5Tinsert(type, "D", HOFFSET(ConstraintRecord, value), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ConstraintRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateMPCType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(MPCRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(MPCRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "RELATION_TYPE", HOFFSET(MPCRecord, relationType), H5T_NATIVE_INT);
    H5Tinsert(type, "MASTER_G", HOFFSET(MPCRecord, masterNodeId), H5T_NATIVE_INT);
    H5Tinsert(type, "SLAVE_G", HOFFSET(MPCRecord, slaveNodeId), H5T_NATIVE_INT);
    H5Tinsert(type, "SLAVE_DIRECTION_MASK", HOFFSET(MPCRecord, slaveDirectionMask), H5T_NATIVE_INT);
    H5Tinsert(type, "STEP_ID", HOFFSET(MPCRecord, stepId), H5T_NATIVE_INT);
    InsertArray(type, "PARAMETERS", HOFFSET(MPCRecord, parameters), H5T_NATIVE_DOUBLE, 3);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(MPCRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateRigidBodyInertiaType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(RigidBodyInertiaRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(RigidBodyInertiaRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "G", HOFFSET(RigidBodyInertiaRecord, nodeId), H5T_NATIVE_INT);
    H5Tinsert(type, "MASS", HOFFSET(RigidBodyInertiaRecord, mass), H5T_NATIVE_DOUBLE);
    InsertArray(type, "INERTIA", HOFFSET(RigidBodyInertiaRecord, inertia), H5T_NATIVE_DOUBLE, 6);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(RigidBodyInertiaRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateLoadType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(LoadRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(LoadRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "TYPE", HOFFSET(LoadRecord, type), H5T_NATIVE_INT);
    H5Tinsert(type, "TARGET_ID", HOFFSET(LoadRecord, targetId), H5T_NATIVE_INT);
    H5Tinsert(type, "DIRECTION", HOFFSET(LoadRecord, direction), H5T_NATIVE_INT);
    H5Tinsert(type, "STEP_ID", HOFFSET(LoadRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "FUNCTION_TYPE", HOFFSET(LoadRecord, functionType), H5T_NATIVE_INT);
    H5Tinsert(type, "VALUE", HOFFSET(LoadRecord, value), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "START_TIME", HOFFSET(LoadRecord, startTime), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "END_TIME", HOFFSET(LoadRecord, endTime), H5T_NATIVE_DOUBLE);
    InsertArray(type, "PARAMS", HOFFSET(LoadRecord, params), H5T_NATIVE_DOUBLE, 8);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(LoadRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateWindDirectionType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(WindDirectionRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "LOAD_ID", HOFFSET(WindDirectionRecord, loadId), H5T_NATIVE_INT);
    InsertArray(type, "DIRECTION", HOFFSET(WindDirectionRecord, direction), H5T_NATIVE_DOUBLE, 3);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(WindDirectionRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateStepType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(StepRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "TYPE", HOFFSET(StepRecord, type), H5T_NATIVE_INT);
    H5Tinsert(type, "IS_DYNAMIC", HOFFSET(StepRecord, isDynamic), H5T_NATIVE_INT);
    H5Tinsert(type, "DYNAMIC_SOLVER_TYPE", HOFFSET(StepRecord, dynamicSolverType), H5T_NATIVE_INT);
    H5Tinsert(type, "TIME", HOFFSET(StepRecord, time), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "STEP_SIZE", HOFFSET(StepRecord, stepSize), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "TOLERANCE", HOFFSET(StepRecord, tolerance), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MAX_ITERATIONS", HOFFSET(StepRecord, maxIterations), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(StepRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateModelSetType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ModelSetRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(ModelSetRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "TYPE", HOFFSET(ModelSetRecord, type), H5T_NATIVE_INT);
    InsertFixedString(type, "NAME", HOFFSET(ModelSetRecord, name), sizeof(ModelSetRecord::name));
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ModelSetRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateModelSetMemberType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ModelSetMemberRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "SET_ID", HOFFSET(ModelSetMemberRecord, setId), H5T_NATIVE_INT);
    H5Tinsert(type, "ENTITY_ID", HOFFSET(ModelSetMemberRecord, entityId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ModelSetMemberRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateComputeRegionType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ComputeRegionRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(ComputeRegionRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "ENABLED", HOFFSET(ComputeRegionRecord, enabled), H5T_NATIVE_INT);
    H5Tinsert(type, "AUTO_GENERATED", HOFFSET(ComputeRegionRecord, autoGenerated), H5T_NATIVE_INT);
    InsertFixedString(type, "NAME", HOFFSET(ComputeRegionRecord, name), sizeof(ComputeRegionRecord::name));
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ComputeRegionRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateRegionLinkType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(RegionLinkRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "REGION_ID", HOFFSET(RegionLinkRecord, regionId), H5T_NATIVE_INT);
    H5Tinsert(type, "TARGET_ID", HOFFSET(RegionLinkRecord, targetId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(RegionLinkRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateStepMetadataType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepMetadataRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "STEP_ID", HOFFSET(StepMetadataRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "REGION_SCOPE", HOFFSET(StepMetadataRecord, regionScope), H5T_NATIVE_INT);
    H5Tinsert(type, "INITIAL_STATIC_STEP_ID", HOFFSET(StepMetadataRecord, initialStaticStepId), H5T_NATIVE_INT);
    InsertFixedString(type, "NAME", HOFFSET(StepMetadataRecord, name), sizeof(StepMetadataRecord::name));
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(StepMetadataRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateStepGallopingType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepGallopingRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "STEP_ID", HOFFSET(StepGallopingRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "ENABLED", HOFFSET(StepGallopingRecord, enabled), H5T_NATIVE_INT);
    H5Tinsert(type, "ICE_THICKNESS", HOFFSET(StepGallopingRecord, iceThickness), H5T_NATIVE_INT);
    H5Tinsert(type, "INITIAL_ATTACK_DEGREES", HOFFSET(StepGallopingRecord, initialAttackDegrees), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "AERODYNAMIC_TANGENT_MODE", HOFFSET(StepGallopingRecord, aerodynamicTangentMode), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(StepGallopingRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateStepTssbnType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepTssbnRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "STEP_ID", HOFFSET(StepTssbnRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "SPECTRAL_RADIUS_INFINITY", HOFFSET(StepTssbnRecord, spectralRadiusInfinity), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MINIMUM_TIME_STEP", HOFFSET(StepTssbnRecord, minimumTimeStep), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MAXIMUM_TIME_STEP", HOFFSET(StepTssbnRecord, maximumTimeStep), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "RELATIVE_TOLERANCE", HOFFSET(StepTssbnRecord, relativeTolerance), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "ABSOLUTE_TOLERANCE", HOFFSET(StepTssbnRecord, absoluteTolerance), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "SAFETY_FACTOR", HOFFSET(StepTssbnRecord, safetyFactor), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "SHRINK_FACTOR", HOFFSET(StepTssbnRecord, shrinkFactor), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MAXIMUM_GROWTH_FACTOR", HOFFSET(StepTssbnRecord, maximumGrowthFactor), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "TARGET_NEWTON_ITERATIONS", HOFFSET(StepTssbnRecord, targetNewtonIterations), H5T_NATIVE_INT);
    H5Tinsert(type, "MAXIMUM_TOTAL_NEWTON_ITERATIONS", HOFFSET(StepTssbnRecord, maximumTotalNewtonIterations),
              H5T_NATIVE_INT);
    H5Tinsert(type, "DERIVATIVE_GAIN", HOFFSET(StepTssbnRecord, derivativeGain), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MINIMUM_DERIVATIVE_FACTOR", HOFFSET(StepTssbnRecord, minimumDerivativeFactor), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MAXIMUM_DERIVATIVE_FACTOR", HOFFSET(StepTssbnRecord, maximumDerivativeFactor), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MAXIMUM_REJECTED_ATTEMPTS", HOFFSET(StepTssbnRecord, maximumRejectedAttempts), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(StepTssbnRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateStepStructuralDampingType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepStructuralDampingRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "STEP_ID", HOFFSET(StepStructuralDampingRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "ENABLED", HOFFSET(StepStructuralDampingRecord, enabled), H5T_NATIVE_INT);
    H5Tinsert(type, "TRANSLATION_DAMPING_RATIO", HOFFSET(StepStructuralDampingRecord, translationDampingRatio),
              H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "TORSION_DAMPING_RATIO", HOFFSET(StepStructuralDampingRecord, torsionDampingRatio),
              H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MAXIMUM_FREQUENCY_HZ", HOFFSET(StepStructuralDampingRecord, maximumFrequencyHz),
              H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(StepStructuralDampingRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateStepRegionLinkType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepRegionLinkRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "STEP_ID", HOFFSET(StepRegionLinkRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "REGION_ID", HOFFSET(StepRegionLinkRecord, regionId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(StepRegionLinkRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateAeroCaseType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(AeroCaseRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(AeroCaseRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "BUNDLE_COUNT", HOFFSET(AeroCaseRecord, bundleCount), H5T_NATIVE_INT);
    H5Tinsert(type, "WIND_SPEED", HOFFSET(AeroCaseRecord, windSpeed), H5T_NATIVE_INT);
    H5Tinsert(type, "ICE_THICKNESS", HOFFSET(AeroCaseRecord, iceThickness), H5T_NATIVE_INT);
    H5Tinsert(type, "MODEL_COUNT", HOFFSET(AeroCaseRecord, modelCount), H5T_NATIVE_INT);
    H5Tinsert(type, "DATA_SIZE", HOFFSET(AeroCaseRecord, dataSize), H5T_NATIVE_INT);
    H5Tinsert(type, "START_ANGLE", HOFFSET(AeroCaseRecord, startAngle), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "ANGLE_STEP", HOFFSET(AeroCaseRecord, angleStep), H5T_NATIVE_DOUBLE);
    InsertFixedString(type, "SOURCE_FILE", HOFFSET(AeroCaseRecord, sourceFile), sizeof(AeroCaseRecord::sourceFile));
    InsertFixedString(type, "SOURCE_PATH", HOFFSET(AeroCaseRecord, sourcePath), sizeof(AeroCaseRecord::sourcePath));
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(AeroCaseRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateAeroCoefficientType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(AeroCoefficientRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "CASE_ID", HOFFSET(AeroCoefficientRecord, caseId), H5T_NATIVE_INT);
    H5Tinsert(type, "MODEL_INDEX", HOFFSET(AeroCoefficientRecord, modelIndex), H5T_NATIVE_INT);
    H5Tinsert(type, "ANGLE_INDEX", HOFFSET(AeroCoefficientRecord, angleIndex), H5T_NATIVE_INT);
    H5Tinsert(type, "ANGLE", HOFFSET(AeroCoefficientRecord, angle), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "CL", HOFFSET(AeroCoefficientRecord, lift), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "CD", HOFFSET(AeroCoefficientRecord, drag), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "CM", HOFFSET(AeroCoefficientRecord, moment), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(AeroCoefficientRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateDomainType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(DomainRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(DomainRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "STEP_ID", HOFFSET(DomainRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "INCREMENT", HOFFSET(DomainRecord, increment), H5T_NATIVE_INT);
    H5Tinsert(type, "ANALYSIS", HOFFSET(DomainRecord, analysis), H5T_NATIVE_INT);
    H5Tinsert(type, "TIME", HOFFSET(DomainRecord, time), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "LOAD_FACTOR", HOFFSET(DomainRecord, loadFactor), H5T_NATIVE_DOUBLE);
    return type;
}

H5Handle CreateSolverIterationType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(SolverIterationH5Record)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "STEP_ID", HOFFSET(SolverIterationH5Record, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "ANALYSIS_TYPE", HOFFSET(SolverIterationH5Record, analysisType), H5T_NATIVE_INT);
    H5Tinsert(type, "TIME", HOFFSET(SolverIterationH5Record, time), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "ITERATIONS", HOFFSET(SolverIterationH5Record, iterations), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateIndexType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(IndexRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(IndexRecord, domainId), H5T_NATIVE_INT);
    H5Tinsert(type, "POSITION", HOFFSET(IndexRecord, position), H5T_NATIVE_LLONG);
    H5Tinsert(type, "LENGTH", HOFFSET(IndexRecord, length), H5T_NATIVE_LLONG);
    return type;
}

H5Handle CreateNodalType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(NodalRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "ID", HOFFSET(NodalRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "X", HOFFSET(NodalRecord, x), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "Y", HOFFSET(NodalRecord, y), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "Z", HOFFSET(NodalRecord, z), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "RX", HOFFSET(NodalRecord, rx), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "RY", HOFFSET(NodalRecord, ry), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "RZ", HOFFSET(NodalRecord, rz), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(NodalRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateElementForceType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ElementForceRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "EID", HOFFSET(ElementForceRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "AXIAL", HOFFSET(ElementForceRecord, axial), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "SHEARY", HOFFSET(ElementForceRecord, shearY), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "SHEARZ", HOFFSET(ElementForceRecord, shearZ), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "TORQUE", HOFFSET(ElementForceRecord, torque), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MY", HOFFSET(ElementForceRecord, momentY), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "MZ", HOFFSET(ElementForceRecord, momentZ), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ElementForceRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateTrussForceType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(TrussForceRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "EID", HOFFSET(TrussForceRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(TrussForceRecord, domainId), H5T_NATIVE_INT);
    H5Tinsert(type, "AXIAL", HOFFSET(TrussForceRecord, axial), H5T_NATIVE_DOUBLE);
    return type;
}

H5Handle CreateCableForceType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(CableForceRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "EID", HOFFSET(CableForceRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(CableForceRecord, domainId), H5T_NATIVE_INT);
    H5Tinsert(type, "AXIAL", HOFFSET(CableForceRecord, axial), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "TORQUE", HOFFSET(CableForceRecord, torque), H5T_NATIVE_DOUBLE);
    return type;
}

H5Handle CreateElementStressType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ElementStressRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "EID", HOFFSET(ElementStressRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "S0", HOFFSET(ElementStressRecord, initStress), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "S", HOFFSET(ElementStressRecord, currentStress), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DS", HOFFSET(ElementStressRecord, deltaStress), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ElementStressRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateElementStrainType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ElementStrainRecord)), H5Tclose);
    if (!type.valid())
        return {};
    H5Tinsert(type, "EID", HOFFSET(ElementStrainRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "STRAIN", HOFFSET(ElementStrainRecord, strain), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ElementStrainRecord, domainId), H5T_NATIVE_INT);
    return type;
}

EnumKeyword::ElementType GetElementType(const std::shared_ptr<ElementBase>& pElement)
{
    if (std::dynamic_pointer_cast<ElementTruss>(pElement))
        return EnumKeyword::ElementType::T3D2;
    if (std::dynamic_pointer_cast<ElementCable>(pElement))
        return EnumKeyword::ElementType::CABLE;
    if (std::dynamic_pointer_cast<ElementBeam_CR>(pElement))
        return EnumKeyword::ElementType::CR3D;
    return EnumKeyword::ElementType::UNKNOWN;
}

QHash<int, EnumKeyword::ElementType> BuildElementTypeMap(const StructureData* pData)
{
    QHash<int, EnumKeyword::ElementType> result;
    if (!pData)
        return result;
    for (const auto& [elementId, element] : pData->m_Elements)
        result.insert(elementId, GetElementType(element));
    return result;
}

EnumKeyword::SectionType GetSectionType(const std::shared_ptr<SectionBase>& pSection)
{
    if (std::dynamic_pointer_cast<SectionCircular>(pSection))
        return EnumKeyword::SectionType::CIRCULAR;
    if (std::dynamic_pointer_cast<SectionRectangle>(pSection))
        return EnumKeyword::SectionType::RECTANGULAR;
    return EnumKeyword::SectionType::UNKNOWN;
}

std::vector<GridRecord> BuildGridRecords(const StructureData* pData)
{
    std::vector<GridRecord> records;
    records.reserve(pData->m_Nodes.size());
    for (const auto& pair : pData->m_Nodes)
    {
        const auto& pNode = pair.second;
        if (!pNode)
            continue;

        GridRecord record;
        record.id = pNode->m_Id;
        record.x[0] = pNode->m_X;
        record.x[1] = pNode->m_Y;
        record.x[2] = pNode->m_Z;
        record.dofCount = static_cast<int>(pNode->m_DOF.size());
        records.push_back(record);
    }
    return records;
}

std::vector<MaterialRecord> BuildMaterialRecords(const StructureData* pData)
{
    std::vector<MaterialRecord> records;
    records.reserve(pData->m_Material.size());
    for (const auto& pair : pData->m_Material)
    {
        const auto& pMaterial = pair.second;
        if (!pMaterial)
            continue;

        MaterialRecord record;
        record.id = pMaterial->m_Id;
        record.young = pMaterial->m_Young;
        record.poisson = pMaterial->m_Poisson;
        record.density = pMaterial->m_Density;
        // 为兼容既有 HDF5 格式，字段名仍为 MAX_STRESS。
        record.maxStress = pMaterial->m_MaxStress;
        record.expansion = pMaterial->m_Expansion;
        records.push_back(record);
    }
    return records;
}

std::vector<SectionRecord> BuildSectionRecords(const StructureData* pData)
{
    std::vector<SectionRecord> records;
    records.reserve(pData->m_Section.size());
    for (const auto& pair : pData->m_Section)
    {
        const auto& pSection = pair.second;
        if (!pSection)
            continue;

        SectionRecord record;
        record.id = pSection->m_Id;
        record.type = ToInt(GetSectionType(pSection));
        record.area = pSection->m_Area;
        record.radius = pSection->m_Radius;
        record.iy = pSection->Io;
        record.iz = pSection->Irr;
        record.j = 0.0;

        if (auto pRect = std::dynamic_pointer_cast<SectionRectangle>(pSection))
        {
            record.width = pRect->m_Width;
            record.height = pRect->m_Height;
        }

        records.push_back(record);
    }
    return records;
}

std::vector<PropertyRecord> BuildPropertyRecords(const StructureData* pData)
{
    std::vector<PropertyRecord> records;
    records.reserve(pData->m_Property.size());
    for (const auto& pair : pData->m_Property)
    {
        const auto& pProperty = pair.second;
        if (!pProperty)
            continue;

        PropertyRecord record;
        record.id = pProperty->m_Id;
        if (auto pMaterial = pProperty->m_pMaterial.lock())
            record.materialId = pMaterial->m_Id;
        if (auto pSection = pProperty->m_pSection.lock())
            record.sectionId = pSection->m_Id;
        records.push_back(record);
    }
    return records;
}

std::vector<ElementRecord> BuildElementRecords(const StructureData* pData)
{
    std::vector<ElementRecord> records;
    records.reserve(pData->m_Elements.size());
    for (const auto& pair : pData->m_Elements)
    {
        const auto& pElement = pair.second;
        if (!pElement)
            continue;

        ElementRecord record;
        record.id = pElement->m_Id;
        record.type = ToInt(GetElementType(pElement));
        record.initStress = pElement->m_InitStress;
        record.role = static_cast<int>(pElement->m_Role);
        record.wireId = pElement->m_WireId;
        record.aeroBundleCount = pElement->m_AeroBundleCount;
        record.aeroProfileId = pElement->m_AeroProfileId;
        if (auto pProperty = pElement->m_pProperty.lock())
            record.propertyId = pProperty->m_Id;
        if (pElement->m_pNode.size() > 0)
        {
            if (auto pNode = pElement->m_pNode[0].lock())
                record.nodeIds[0] = pNode->m_Id;
        }
        if (pElement->m_pNode.size() > 1)
        {
            if (auto pNode = pElement->m_pNode[1].lock())
                record.nodeIds[1] = pNode->m_Id;
        }
        if (auto pBeam = std::dynamic_pointer_cast<ElementBeam_CR>(pElement))
        {
            record.q0[0] = pBeam->q0.x();
            record.q0[1] = pBeam->q0.y();
            record.q0[2] = pBeam->q0.z();
        }
        records.push_back(record);
    }
    return records;
}

std::vector<ConstraintRecord> BuildConstraintRecords(const StructureData* pData)
{
    std::vector<ConstraintRecord> records;
    records.reserve(pData->m_Constraint.size());
    for (const auto& pair : pData->m_Constraint)
    {
        const auto& pConstraint = pair.second;
        if (!pConstraint)
            continue;

        ConstraintRecord record;
        record.id = pConstraint->m_Id;
        if (auto pNode = pConstraint->m_pNode.lock())
            record.nodeId = pNode->m_Id;
        record.direction = ToInt(pConstraint->m_Direction);
        record.value = pConstraint->m_Value;
        records.push_back(record);
    }
    return records;
}

std::vector<MPCRecord> BuildMPCRecords(const StructureData* pData)
{
    std::vector<MPCRecord> records;
    records.reserve(pData->m_MPCConstraints.size());
    for (const auto& [id, mpc] : pData->m_MPCConstraints)
    {
        if (!mpc)
            continue;
        const std::vector<int> nodeIds = mpc->GetNodeIds();
        if (nodeIds.size() != 2)
            continue;
        MPCRecord record;
        record.id = id;
        record.masterNodeId = nodeIds[0];
        record.slaveNodeId = nodeIds[1];
        record.stepId = mpc->m_StepId;
        for (const int direction : mpc->m_SlaveDirections)
            if (direction >= 0 && direction < 6)
                record.slaveDirectionMask |= (1 << direction);
        if (const auto distance = std::dynamic_pointer_cast<DistanceMPCConstraint>(mpc))
        {
            record.relationType = 0;
            record.parameters[0] = distance->m_Length;
        }
        else if (const auto rigid = std::dynamic_pointer_cast<RigidOffsetMPCConstraint>(mpc))
        {
            record.relationType = 1;
            record.parameters[0] = rigid->m_Offset.x();
            record.parameters[1] = rigid->m_Offset.y();
            record.parameters[2] = rigid->m_Offset.z();
        }
        else if (std::dynamic_pointer_cast<PlanarShearReleaseMPCConstraint>(mpc))
        {
            record.relationType = 2;
        }
        else if (std::dynamic_pointer_cast<TranslationalTieMPCConstraint>(mpc))
        {
            record.relationType = 3;
        }
        else if (const auto twist = std::dynamic_pointer_cast<AxialTwistTieMPCConstraint>(mpc))
        {
            record.relationType = 4;
            record.parameters[0] = twist->m_Axis.x();
            record.parameters[1] = twist->m_Axis.y();
            record.parameters[2] = twist->m_Axis.z();
        }
        else
        {
            continue;
        }
        records.push_back(record);
    }
    return records;
}

std::vector<RigidBodyInertiaRecord> BuildRigidBodyInertiaRecords(const StructureData* pData)
{
    std::vector<RigidBodyInertiaRecord> records;
    records.reserve(pData->m_RigidBodyInertias.size());
    for (const auto& [id, inertia] : pData->m_RigidBodyInertias)
    {
        const auto node = inertia ? inertia->m_pNode.lock() : nullptr;
        if (!node)
            continue;
        RigidBodyInertiaRecord record;
        record.id = id;
        record.nodeId = node->m_Id;
        record.mass = inertia->m_Mass;
        record.inertia[0] = inertia->m_RotaryInertia(0, 0);
        record.inertia[1] = inertia->m_RotaryInertia(1, 1);
        record.inertia[2] = inertia->m_RotaryInertia(2, 2);
        record.inertia[3] = inertia->m_RotaryInertia(0, 1);
        record.inertia[4] = inertia->m_RotaryInertia(0, 2);
        record.inertia[5] = inertia->m_RotaryInertia(1, 2);
        records.push_back(record);
    }
    return records;
}

std::vector<LoadRecord> BuildLoadRecords(const StructureData* pData)
{
    std::vector<LoadRecord> records;
    records.reserve(pData->m_Load.size());
    for (const auto& pair : pData->m_Load)
    {
        const auto& pLoad = pair.second;
        if (!pLoad)
            continue;

        LoadRecord record;
        record.id = pLoad->m_Id;
        record.type = ToInt(pLoad->m_LoadType);
        record.direction = ToInt(pLoad->m_Direction);
        record.stepId = pLoad->m_StepId;
        record.functionType = ToInt(pLoad->m_FunctionType);
        record.startTime = pLoad->m_StartTime;
        record.endTime = pLoad->m_EndTime;
        record.params[0] = pLoad->m_Amplitude;
        record.params[1] = pLoad->m_Frequency;
        record.params[2] = pLoad->m_Phase;
        record.params[3] = pLoad->m_Offset;
        record.params[4] = pLoad->m_RampT0;
        record.params[5] = pLoad->m_RampT1;
        record.params[6] = pLoad->m_Decay;
        record.params[7] = pLoad->m_Period;

        if (auto pNodeLoad = std::dynamic_pointer_cast<Force_Node>(pLoad))
        {
            if (auto pNode = pNodeLoad->m_pNode.lock())
                record.targetId = pNode->m_Id;
            record.value = pNodeLoad->m_Value;
        }
        else if (auto pElementLoad = std::dynamic_pointer_cast<Force_Element>(pLoad))
        {
            if (auto pElement = pElementLoad->m_pElement.lock())
                record.targetId = pElement->m_Id;
            record.value = pElementLoad->m_Value;
        }
        else if (auto pGravity = std::dynamic_pointer_cast<Force_Gravity>(pLoad))
        {
            record.value = pGravity->m_g;
        }
        else if (auto pWind = std::dynamic_pointer_cast<Force_Wind>(pLoad))
        {
            record.value = pWind->m_velocity;
        }

        records.push_back(record);
    }
    return records;
}

std::vector<WindDirectionRecord> BuildWindDirectionRecords(const StructureData* pData)
{
    std::vector<WindDirectionRecord> records;
    for (const auto& [loadId, load] : pData->m_Load)
    {
        const auto wind = std::dynamic_pointer_cast<Force_Wind>(load);
        if (!wind)
            continue;
        WindDirectionRecord record;
        record.loadId = loadId;
        for (int component = 0; component < 3; ++component)
            record.direction[component] = wind->m_direction[component];
        records.push_back(record);
    }
    return records;
}

std::vector<StepRecord> BuildStepRecords(const StructureData* pData)
{
    std::vector<StepRecord> records;
    records.reserve(pData->m_AnalysisStep.size());
    for (const auto& pair : pData->m_AnalysisStep)
    {
        const auto& pStep = pair.second;
        if (!pStep)
            continue;

        StepRecord record;
        record.id = pStep->m_Id;
        record.type = ToInt(pStep->m_Type);
        record.isDynamic = pStep->isDynamic ? 1 : 0;
        record.dynamicSolverType = static_cast<int>(pStep->m_DynamicSolverType);
        record.time = pStep->m_Time;
        record.stepSize = pStep->m_StepSize;
        record.tolerance = pStep->m_Tolerance;
        record.maxIterations = pStep->m_MaxIterations;
        records.push_back(record);
    }
    return records;
}

void BuildModelSetRecords(const StructureData* pData, std::vector<ModelSetRecord>& sets,
                          std::vector<ModelSetMemberRecord>& members)
{
    for (const auto& [setId, modelSet] : pData->m_ModelSets)
    {
        if (!modelSet)
        {
            continue;
        }
        ModelSetRecord setRecord;
        setRecord.id = setId;
        setRecord.type = static_cast<int>(modelSet->m_Type);
        CopyUtf8String(setRecord.name, sizeof(setRecord.name), modelSet->m_Name);
        sets.push_back(setRecord);

        for (int entityId : modelSet->m_Ids)
        {
            ModelSetMemberRecord member;
            member.setId = setId;
            member.entityId = entityId;
            members.push_back(member);
        }
    }
}

void BuildComputeRegionRecords(const StructureData* pData, std::vector<ComputeRegionRecord>& regions,
                               std::vector<RegionLinkRecord>& setLinks, std::vector<RegionLinkRecord>& directNodes,
                               std::vector<RegionLinkRecord>& directElements)
{
    for (const auto& [regionId, region] : pData->m_ComputeRegions)
    {
        if (!region)
        {
            continue;
        }
        ComputeRegionRecord regionRecord;
        regionRecord.id = regionId;
        regionRecord.enabled = region->m_Enabled ? 1 : 0;
        regionRecord.autoGenerated = region->m_AutoGenerated ? 1 : 0;
        CopyUtf8String(regionRecord.name, sizeof(regionRecord.name), region->m_Name);
        regions.push_back(regionRecord);

        for (int setId : region->m_SourceSetIds)
        {
            setLinks.push_back({regionId, setId});
        }
        for (int nodeId : region->m_DirectNodeIds)
        {
            directNodes.push_back({regionId, nodeId});
        }
        for (int elementId : region->m_DirectElementIds)
        {
            directElements.push_back({regionId, elementId});
        }
    }
}

void BuildStepMetadataRecords(const StructureData* pData, std::vector<StepMetadataRecord>& metadata,
                              std::vector<StepRegionLinkRecord>& regionLinks)
{
    for (const auto& [stepId, step] : pData->m_AnalysisStep)
    {
        if (!step)
        {
            continue;
        }
        StepMetadataRecord record;
        record.stepId = stepId;
        record.regionScope = static_cast<int>(step->m_RegionScope);
        record.initialStaticStepId = step->m_InitialStaticStepId;
        CopyUtf8String(record.name, sizeof(record.name), step->m_Name);
        metadata.push_back(record);

        for (int regionId : step->m_ComputeRegionIds)
        {
            regionLinks.push_back({stepId, regionId});
        }
    }
}

std::vector<StepGallopingRecord> BuildStepGallopingRecords(const StructureData* pData)
{
    std::vector<StepGallopingRecord> records;
    records.reserve(pData->m_AnalysisStep.size());
    for (const auto& [stepId, step] : pData->m_AnalysisStep)
    {
        if (!step)
            continue;
        StepGallopingRecord record;
        record.stepId = stepId;
        record.enabled = step->m_EnableGalloping ? 1 : 0;
        record.iceThickness = step->m_GallopingIceThickness;
        record.initialAttackDegrees = step->m_GallopingInitialAttackDegrees;
        record.aerodynamicTangentMode = static_cast<int>(step->m_GallopingAerodynamicTangentMode);
        records.push_back(record);
    }
    return records;
}

std::vector<StepTssbnRecord> BuildStepTssbnRecords(const StructureData* pData)
{
    std::vector<StepTssbnRecord> records;
    records.reserve(pData->m_AnalysisStep.size());
    for (const auto& [stepId, step] : pData->m_AnalysisStep)
    {
        if (!step)
            continue;
        const auto& settings = step->m_AdaptiveTssbn;
        StepTssbnRecord record;
        record.stepId = stepId;
        record.spectralRadiusInfinity = settings.spectralRadiusInfinity;
        record.minimumTimeStep = settings.minimumTimeStep;
        record.maximumTimeStep = settings.maximumTimeStep;
        record.relativeTolerance = settings.relativeTolerance;
        record.absoluteTolerance = settings.absoluteTolerance;
        record.safetyFactor = settings.safetyFactor;
        record.shrinkFactor = settings.shrinkFactor;
        record.maximumGrowthFactor = settings.maximumGrowthFactor;
        record.targetNewtonIterations = settings.targetNewtonIterations;
        record.maximumTotalNewtonIterations = settings.maximumTotalNewtonIterations;
        record.derivativeGain = settings.derivativeGain;
        record.minimumDerivativeFactor = settings.minimumDerivativeFactor;
        record.maximumDerivativeFactor = settings.maximumDerivativeFactor;
        record.maximumRejectedAttempts = settings.maximumRejectedAttempts;
        records.push_back(record);
    }
    return records;
}

std::vector<StepStructuralDampingRecord> BuildStepStructuralDampingRecords(const StructureData* pData)
{
    std::vector<StepStructuralDampingRecord> records;
    records.reserve(pData->m_AnalysisStep.size());
    for (const auto& [stepId, step] : pData->m_AnalysisStep)
    {
        if (!step)
            continue;
        const auto& settings = step->m_StructuralDamping.settings;
        StepStructuralDampingRecord record;
        record.stepId = stepId;
        record.enabled = settings.enabled ? 1 : 0;
        record.translationDampingRatio = settings.translationDampingRatio;
        record.torsionDampingRatio = settings.torsionDampingRatio;
        record.maximumFrequencyHz = settings.maximumFrequencyHz;
        records.push_back(record);
    }
    return records;
}

int GetAeroDataSize(const std::vector<BladeModel>& models)
{
    if (models.empty())
    {
        return 0;
    }
    return static_cast<int>(models.front().lift.size());
}

void AppendAeroRecords(int caseId, const AeroCaseKey& key, const std::vector<BladeModel>& models,
                       const std::filesystem::path& sourceFile, const AeroManager& manager,
                       std::vector<AeroCaseRecord>& caseRecords, std::vector<AeroCoefficientRecord>& coefficientRecords)
{
    AeroCaseRecord caseRecord;
    caseRecord.id = caseId;
    caseRecord.bundleCount = key.bundleCount;
    caseRecord.windSpeed = key.windSpeed;
    caseRecord.iceThickness = key.iceThickness;
    caseRecord.modelCount = static_cast<int>(models.size());
    caseRecord.dataSize = GetAeroDataSize(models);
    caseRecord.startAngle = manager.getStartAngle();
    caseRecord.angleStep = manager.getAngleStep();

    const QString sourcePath = PathToQString(sourceFile);
    CopyUtf8String(caseRecord.sourceFile, sizeof(caseRecord.sourceFile), QFileInfo(sourcePath).fileName());
    CopyUtf8String(caseRecord.sourcePath, sizeof(caseRecord.sourcePath), sourcePath);
    caseRecords.push_back(caseRecord);

    for (int modelIndex = 0; modelIndex < static_cast<int>(models.size()); ++modelIndex)
    {
        const BladeModel& model = models[modelIndex];
        const size_t dataSize = std::min({model.lift.size(), model.drag.size(), model.moment.size()});
        for (size_t angleIndex = 0; angleIndex < dataSize; ++angleIndex)
        {
            AeroCoefficientRecord coefficient;
            coefficient.caseId = caseId;
            coefficient.modelIndex = modelIndex;
            coefficient.angleIndex = static_cast<int>(angleIndex);
            coefficient.angle = manager.getStartAngle() + manager.getAngleStep() * static_cast<double>(angleIndex);
            coefficient.lift = model.lift[angleIndex];
            coefficient.drag = model.drag[angleIndex];
            coefficient.moment = model.moment[angleIndex];
            coefficientRecords.push_back(coefficient);
        }
    }
}

void BuildAeroRecords(const StructureData* pData, std::vector<AeroCaseRecord>& caseRecords,
                      std::vector<AeroCoefficientRecord>& coefficientRecords)
{
    const AeroManager& manager = pData->m_AeroManager;
    int caseId = 1;

    for (const auto& pair : manager.getCaseModels())
    {
        AppendAeroRecords(caseId, pair.first, pair.second, manager.getCaseSourceFile(pair.first), manager, caseRecords,
                          coefficientRecords);
        ++caseId;
    }

    if (caseRecords.empty() && !manager.getModels().empty())
    {
        AeroCaseKey currentKey;
        AppendAeroRecords(caseId, currentKey, manager.getModels(), manager.getCurrentSourceFile(), manager, caseRecords,
                          coefficientRecords);
    }
}

void AppendFrameIndex(std::vector<IndexRecord>& indexRecords, int domainId, long long position, long long length)
{
    IndexRecord record;
    record.domainId = domainId;
    record.position = position;
    record.length = length;
    indexRecords.push_back(record);
}

bool WriteInputData(hid_t file, const StructureData* pData)
{
    H5Handle gridType = CreateGridType();
    H5Handle materialType = CreateMaterialType();
    H5Handle sectionType = CreateSectionType();
    H5Handle propertyType = CreatePropertyType();
    H5Handle elementType = CreateElementType();
    H5Handle constraintType = CreateConstraintType();
    H5Handle mpcType = CreateMPCType();
    H5Handle rigidBodyInertiaType = CreateRigidBodyInertiaType();
    H5Handle loadType = CreateLoadType();
    H5Handle windDirectionType = CreateWindDirectionType();
    H5Handle stepType = CreateStepType();
    H5Handle modelSetType = CreateModelSetType();
    H5Handle modelSetMemberType = CreateModelSetMemberType();
    H5Handle computeRegionType = CreateComputeRegionType();
    H5Handle regionLinkType = CreateRegionLinkType();
    H5Handle stepMetadataType = CreateStepMetadataType();
    H5Handle stepGallopingType = CreateStepGallopingType();
    H5Handle stepTssbnType = CreateStepTssbnType();
    H5Handle stepStructuralDampingType = CreateStepStructuralDampingType();
    H5Handle stepRegionLinkType = CreateStepRegionLinkType();
    H5Handle aeroCaseType = CreateAeroCaseType();
    H5Handle aeroCoefficientType = CreateAeroCoefficientType();
    std::vector<ModelSetRecord> modelSets;
    std::vector<ModelSetMemberRecord> modelSetMembers;
    std::vector<ComputeRegionRecord> computeRegions;
    std::vector<RegionLinkRecord> regionSetLinks;
    std::vector<RegionLinkRecord> regionDirectNodes;
    std::vector<RegionLinkRecord> regionDirectElements;
    std::vector<StepMetadataRecord> stepMetadata;
    std::vector<StepRegionLinkRecord> stepRegionLinks;
    std::vector<AeroCaseRecord> aeroCases;
    std::vector<AeroCoefficientRecord> aeroCoefficients;
    BuildModelSetRecords(pData, modelSets, modelSetMembers);
    BuildComputeRegionRecords(pData, computeRegions, regionSetLinks, regionDirectNodes, regionDirectElements);
    BuildStepMetadataRecords(pData, stepMetadata, stepRegionLinks);
    BuildAeroRecords(pData, aeroCases, aeroCoefficients);

    return gridType.valid() && materialType.valid() && sectionType.valid() && propertyType.valid() &&
           elementType.valid() && constraintType.valid() && mpcType.valid() && rigidBodyInertiaType.valid() &&
           loadType.valid() &&
           windDirectionType.valid() && stepType.valid() && modelSetType.valid() && modelSetMemberType.valid() &&
           computeRegionType.valid() && regionLinkType.valid() && stepMetadataType.valid() &&
           stepGallopingType.valid() && stepTssbnType.valid() && stepStructuralDampingType.valid() &&
           stepRegionLinkType.valid() && aeroCaseType.valid() && aeroCoefficientType.valid() &&
           WriteDataset(file, "/YQY/INPUT/NODE/GRID", gridType, BuildGridRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/MATERIAL/MAT", materialType, BuildMaterialRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/SECTION/SECTION", sectionType, BuildSectionRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/PROPERTY/PROPERTY", propertyType, BuildPropertyRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/ELEMENT/ELEMENT", elementType, BuildElementRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/CONSTRAINT/SPC", constraintType, BuildConstraintRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/CONSTRAINT/MPC", mpcType, BuildMPCRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/INERTIA/RIGID_BODY", rigidBodyInertiaType,
                        BuildRigidBodyInertiaRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/LOAD/LOAD", loadType, BuildLoadRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/LOAD/WIND_DIRECTION", windDirectionType, BuildWindDirectionRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/STEP", stepType, BuildStepRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/METADATA", stepMetadataType, stepMetadata) &&
           WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/GALLOPING", stepGallopingType,
                        BuildStepGallopingRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/TSSBN", stepTssbnType, BuildStepTssbnRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/STRUCTURAL_DAMPING", stepStructuralDampingType,
                        BuildStepStructuralDampingRecords(pData)) &&
           WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/REGION_LINK", stepRegionLinkType, stepRegionLinks) &&
           WriteDataset(file, "/YQY/INPUT/MODEL_SET/SET", modelSetType, modelSets) &&
           WriteDataset(file, "/YQY/INPUT/MODEL_SET/MEMBER", modelSetMemberType, modelSetMembers) &&
           WriteDataset(file, "/YQY/INPUT/COMPUTE_REGION/REGION", computeRegionType, computeRegions) &&
           WriteDataset(file, "/YQY/INPUT/COMPUTE_REGION/SET_LINK", regionLinkType, regionSetLinks) &&
           WriteDataset(file, "/YQY/INPUT/COMPUTE_REGION/DIRECT_NODE", regionLinkType, regionDirectNodes) &&
           WriteDataset(file, "/YQY/INPUT/COMPUTE_REGION/DIRECT_ELEMENT", regionLinkType, regionDirectElements) &&
           WriteDataset(file, "/YQY/INPUT/AERO/CASE", aeroCaseType, aeroCases) &&
           WriteDataset(file, "/YQY/INPUT/AERO/COEFFICIENT", aeroCoefficientType, aeroCoefficients);
}

bool WriteResultData(hid_t file, const StructureData* pData)
{
    const auto& frames = pData->GetOutputter().GetDataSet();
    const auto& iterationHistory = pData->GetOutputter().GetSolverIterationRecords();
    const auto elementTypes = BuildElementTypeMap(pData);

    std::vector<DomainRecord> domains;
    std::vector<NodalRecord> displacements;
    std::vector<NodalRecord> currentCoordinates;
    std::vector<NodalRecord> velocities;
    std::vector<NodalRecord> accelerations;
    std::vector<NodalRecord> reactions;
    std::vector<ElementForceRecord> elementForces;
    std::vector<TrussForceRecord> trussForces;
    std::vector<CableForceRecord> cableForces;
    std::vector<ElementStressRecord> elementStresses;
    std::vector<ElementStrainRecord> elementStrains;
    std::vector<SolverIterationH5Record> iterations;
    iterations.reserve(iterationHistory.size());
    for (const SolverIterationRecord& record : iterationHistory)
        iterations.push_back({record.stepId, record.analysisType, record.time, record.iterations});

    std::vector<IndexRecord> displacementIndex;
    std::vector<IndexRecord> currentCoordinateIndex;
    std::vector<IndexRecord> velocityIndex;
    std::vector<IndexRecord> accelerationIndex;
    std::vector<IndexRecord> reactionIndex;
    std::vector<IndexRecord> elementForceIndex;
    std::vector<IndexRecord> trussForceIndex;
    std::vector<IndexRecord> cableForceIndex;
    std::vector<IndexRecord> elementStressIndex;
    std::vector<IndexRecord> elementStrainIndex;

    domains.reserve(frames.size());
    int domainId = 1;
    for (const DataFrame& frame : frames)
    {
        const bool storesDynamicKinematics =
            frame.GetAnalysisType() == static_cast<int>(EnumKeyword::StepType::DYNAMIC);

        DomainRecord domain;
        domain.id = domainId;
        domain.stepId = frame.GetStepId();
        domain.increment = frame.GetIncrement();
        domain.analysis = frame.GetAnalysisType();
        domain.time = frame.GetTime();
        domains.push_back(domain);

        const long long displacementPosition = static_cast<long long>(displacements.size());
        const long long currentCoordinatePosition = static_cast<long long>(currentCoordinates.size());
        const long long velocityPosition = static_cast<long long>(velocities.size());
        const long long accelerationPosition = static_cast<long long>(accelerations.size());
        const long long reactionPosition = static_cast<long long>(reactions.size());
        for (const auto& pair : frame.GetNodeDatas())
        {
            const int nodeId = pair.first;
            const NodeData& nodeData = pair.second;

            NodalRecord displacement;
            displacement.id = nodeId;
            displacement.x = nodeData.GetValue(EnumKeyword::NodeResultType::U1);
            displacement.y = nodeData.GetValue(EnumKeyword::NodeResultType::U2);
            displacement.z = nodeData.GetValue(EnumKeyword::NodeResultType::U3);
            displacement.rx = nodeData.GetValue(EnumKeyword::NodeResultType::UR1);
            displacement.ry = nodeData.GetValue(EnumKeyword::NodeResultType::UR2);
            displacement.rz = nodeData.GetValue(EnumKeyword::NodeResultType::UR3);
            displacement.domainId = domainId;
            displacements.push_back(displacement);

            NodalRecord currentCoordinate;
            currentCoordinate.id = nodeId;
            currentCoordinate.x = nodeData.GetValue(EnumKeyword::NodeResultType::CX);
            currentCoordinate.y = nodeData.GetValue(EnumKeyword::NodeResultType::CY);
            currentCoordinate.z = nodeData.GetValue(EnumKeyword::NodeResultType::CZ);
            currentCoordinate.domainId = domainId;
            currentCoordinates.push_back(currentCoordinate);

            if (storesDynamicKinematics)
            {
                NodalRecord velocity;
                velocity.id = nodeId;
                velocity.x = nodeData.GetVelocityComponent(0);
                velocity.y = nodeData.GetVelocityComponent(1);
                velocity.z = nodeData.GetVelocityComponent(2);
                velocity.rx = nodeData.GetVelocityComponent(3);
                velocity.ry = nodeData.GetVelocityComponent(4);
                velocity.rz = nodeData.GetVelocityComponent(5);
                velocity.domainId = domainId;
                velocities.push_back(velocity);

                NodalRecord acceleration;
                acceleration.id = nodeId;
                acceleration.x = nodeData.GetAccelerationComponent(0);
                acceleration.y = nodeData.GetAccelerationComponent(1);
                acceleration.z = nodeData.GetAccelerationComponent(2);
                acceleration.rx = nodeData.GetAccelerationComponent(3);
                acceleration.ry = nodeData.GetAccelerationComponent(4);
                acceleration.rz = nodeData.GetAccelerationComponent(5);
                acceleration.domainId = domainId;
                accelerations.push_back(acceleration);
            }

            NodalRecord reaction;
            reaction.id = nodeId;
            reaction.x = nodeData.GetValue(EnumKeyword::NodeResultType::R1);
            reaction.y = nodeData.GetValue(EnumKeyword::NodeResultType::R2);
            reaction.z = nodeData.GetValue(EnumKeyword::NodeResultType::R3);
            reaction.domainId = domainId;
            reactions.push_back(reaction);
        }

        const long long nodeLength = static_cast<long long>(frame.GetNodeDatas().size());
        const long long dynamicNodeLength = storesDynamicKinematics ? nodeLength : 0;
        AppendFrameIndex(displacementIndex, domainId, displacementPosition, nodeLength);
        AppendFrameIndex(currentCoordinateIndex, domainId, currentCoordinatePosition, nodeLength);
        AppendFrameIndex(velocityIndex, domainId, velocityPosition, dynamicNodeLength);
        AppendFrameIndex(accelerationIndex, domainId, accelerationPosition, dynamicNodeLength);
        AppendFrameIndex(reactionIndex, domainId, reactionPosition, nodeLength);

        const long long elementPosition = static_cast<long long>(elementForces.size());
        const long long trussPosition = static_cast<long long>(trussForces.size());
        const long long cablePosition = static_cast<long long>(cableForces.size());
        const long long stressPosition = static_cast<long long>(elementStresses.size());
        const long long strainPosition = static_cast<long long>(elementStrains.size());
        long long elementLength = 0;
        long long trussLength = 0;
        long long cableLength = 0;
        for (const auto& pair : frame.GetElementDatas())
        {
            const int elementId = pair.first;
            const ElementData& elementData = pair.second;
            const auto elementType = elementTypes.value(elementId, EnumKeyword::ElementType::UNKNOWN);
            switch (elementType)
            {
                case EnumKeyword::ElementType::T3D2:
                {
                    TrussForceRecord force;
                    force.id = elementId;
                    force.domainId = domainId;
                    force.axial = elementData.GetValue(EnumKeyword::ElementResultType::AxialForce);
                    trussForces.push_back(force);
                    ++trussLength;
                    break;
                }
                case EnumKeyword::ElementType::CABLE:
                {
                    CableForceRecord force;
                    force.id = elementId;
                    force.domainId = domainId;
                    force.axial = elementData.GetValue(EnumKeyword::ElementResultType::AxialForce);
                    force.torque = elementData.GetValue(EnumKeyword::ElementResultType::Torque);
                    cableForces.push_back(force);
                    ++cableLength;
                    break;
                }
                default:
                {
                    ElementForceRecord force;
                    force.id = elementId;
                    force.axial = elementData.GetValue(EnumKeyword::ElementResultType::AxialForce);
                    force.shearY = elementData.GetValue(EnumKeyword::ElementResultType::ShearY);
                    force.shearZ = elementData.GetValue(EnumKeyword::ElementResultType::ShearZ);
                    force.torque = elementData.GetValue(EnumKeyword::ElementResultType::Torque);
                    force.momentY = elementData.GetValue(EnumKeyword::ElementResultType::MomentY);
                    force.momentZ = elementData.GetValue(EnumKeyword::ElementResultType::MomentZ);
                    force.domainId = domainId;
                    elementForces.push_back(force);
                    ++elementLength;
                    break;
                }
            }

            ElementStressRecord stress;
            stress.id = elementId;
            stress.initStress = elementData.GetValue(EnumKeyword::ElementResultType::InitStress);
            stress.currentStress = elementData.GetValue(EnumKeyword::ElementResultType::CurrentStress);
            stress.deltaStress = elementData.GetValue(EnumKeyword::ElementResultType::DeltaStress);
            stress.domainId = domainId;
            elementStresses.push_back(stress);

            ElementStrainRecord strain;
            strain.id = elementId;
            strain.strain = elementData.GetValue(EnumKeyword::ElementResultType::Strain);
            strain.domainId = domainId;
            elementStrains.push_back(strain);
        }

        AppendFrameIndex(elementForceIndex, domainId, elementPosition, elementLength);
        AppendFrameIndex(trussForceIndex, domainId, trussPosition, trussLength);
        AppendFrameIndex(cableForceIndex, domainId, cablePosition, cableLength);
        const long long resultElementLength = static_cast<long long>(frame.GetElementDatas().size());
        AppendFrameIndex(elementStressIndex, domainId, stressPosition, resultElementLength);
        AppendFrameIndex(elementStrainIndex, domainId, strainPosition, resultElementLength);

        ++domainId;
    }

    Hdf5ResultRanges resultRanges;
    for (const NodalRecord& value : displacements)
    {
        IncludeResultRangeValue(resultRanges.displacementMagnitude,
                                std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z));
        IncludeResultRangeValue(resultRanges.displacementX, value.x);
        IncludeResultRangeValue(resultRanges.displacementY, value.y);
        IncludeResultRangeValue(resultRanges.displacementZ, value.z);
    }
    for (const ElementForceRecord& value : elementForces)
        IncludeResultRangeValue(resultRanges.axialForce, value.axial);
    for (const TrussForceRecord& value : trussForces)
        IncludeResultRangeValue(resultRanges.axialForce, value.axial);
    for (const CableForceRecord& value : cableForces)
        IncludeResultRangeValue(resultRanges.axialForce, value.axial);
    for (const ElementStressRecord& value : elementStresses)
        IncludeResultRangeValue(resultRanges.stress, value.currentStress);
    for (const ElementStrainRecord& value : elementStrains)
        IncludeResultRangeValue(resultRanges.strain, value.strain);

    H5Handle domainType = CreateDomainType();
    H5Handle iterationType = CreateSolverIterationType();
    H5Handle indexType = CreateIndexType();
    H5Handle nodalType = CreateNodalType();
    H5Handle elementForceType = CreateElementForceType();
    H5Handle trussForceType = CreateTrussForceType();
    H5Handle cableForceType = CreateCableForceType();
    H5Handle elementStressType = CreateElementStressType();
    H5Handle elementStrainType = CreateElementStrainType();

    return domainType.valid() && iterationType.valid() && indexType.valid() && nodalType.valid() &&
           elementForceType.valid() && trussForceType.valid() && cableForceType.valid() && elementStressType.valid() &&
           elementStrainType.valid() && WriteResultRangeAttributes(file, resultRanges) &&
           WriteDataset(file, "/YQY/RESULT/DOMAINS", domainType, domains) &&
           WriteDataset(file, "/YQY/RESULT/SOLVER_ITERATION", iterationType, iterations) &&
           WriteDataset(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, displacements) &&
           WriteDataset(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType, currentCoordinates) &&
           WriteDataset(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType, velocities) &&
           WriteDataset(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType, accelerations) &&
           WriteDataset(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType, reactions) &&
           WriteDataset(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType, elementForces) &&
           WriteDataset(file, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", trussForceType, trussForces) &&
           WriteDataset(file, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE", cableForceType, cableForces) &&
           WriteDataset(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType, elementStresses) &&
           WriteDataset(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType, elementStrains) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType, displacementIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType, currentCoordinateIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType, velocityIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType, accelerationIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType, reactionIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType, elementForceIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", indexType, trussForceIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE", indexType, cableForceIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType, elementStressIndex) &&
           WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType, elementStrainIndex);
}
}

class Hdf5ModelIO::Impl
{
public:
    H5Handle file;
    QString targetFileName;
    QString tempFileName;
    H5Handle resultFile;
    QString resultTempFileName;
    QString resultSourceFileName;
    std::vector<DomainRecord> resultDomains;
    QHash<int, EnumKeyword::ElementType> elementTypes;
    QHash<int, IndexRecord> displacementIndex;
    QHash<int, IndexRecord> currentCoordinateIndex;
    QHash<int, IndexRecord> velocityIndex;
    QHash<int, IndexRecord> accelerationIndex;
    QHash<int, IndexRecord> elementForceIndex;
    QHash<int, IndexRecord> trussForceIndex;
    QHash<int, IndexRecord> cableForceIndex;
    QHash<int, IndexRecord> stressIndex;
    QHash<int, IndexRecord> strainIndex;
    Hdf5ResultRanges streamRanges;
    H5Handle domainType;
    H5Handle iterationType;
    H5Handle indexType;
    H5Handle nodalType;
    H5Handle elementForceType;
    H5Handle trussForceType;
    H5Handle cableForceType;
    H5Handle elementStressType;
    H5Handle elementStrainType;
    ExtendableDataset domainsDataset;
    ExtendableDataset iterationsDataset;
    StreamDatasetPair displacementDataset;
    StreamDatasetPair currentCoordinateDataset;
    StreamDatasetPair velocityDataset;
    StreamDatasetPair accelerationDataset;
    StreamDatasetPair reactionDataset;
    StreamDatasetPair elementForceDataset;
    StreamDatasetPair trussForceDataset;
    StreamDatasetPair cableForceDataset;
    StreamDatasetPair stressDataset;
    StreamDatasetPair strainDataset;
    std::vector<DomainRecord> pendingDomains;
    std::vector<NodalRecord> pendingDisplacements;
    std::vector<NodalRecord> pendingCurrentCoordinates;
    std::vector<NodalRecord> pendingVelocities;
    std::vector<NodalRecord> pendingAccelerations;
    std::vector<NodalRecord> pendingReactions;
    std::vector<ElementForceRecord> pendingElementForces;
    std::vector<TrussForceRecord> pendingTrussForces;
    std::vector<CableForceRecord> pendingCableForces;
    std::vector<ElementStressRecord> pendingElementStresses;
    std::vector<ElementStrainRecord> pendingElementStrains;
    std::vector<IndexRecord> pendingDisplacementIndices;
    std::vector<IndexRecord> pendingCurrentCoordinateIndices;
    std::vector<IndexRecord> pendingVelocityIndices;
    std::vector<IndexRecord> pendingAccelerationIndices;
    std::vector<IndexRecord> pendingReactionIndices;
    std::vector<IndexRecord> pendingElementForceIndices;
    std::vector<IndexRecord> pendingTrussForceIndices;
    std::vector<IndexRecord> pendingCableForceIndices;
    std::vector<IndexRecord> pendingStressIndices;
    std::vector<IndexRecord> pendingStrainIndices;

    ~Impl()
    {
        CloseResult();
        ResetStreamResources();
        file.reset();
        if (!tempFileName.isEmpty() && tempFileName != targetFileName)
            QFile::remove(tempFileName);
    }

    void ResetStreamResources()
    {
        domainsDataset = {};
        iterationsDataset = {};
        displacementDataset = {};
        currentCoordinateDataset = {};
        velocityDataset = {};
        accelerationDataset = {};
        reactionDataset = {};
        elementForceDataset = {};
        trussForceDataset = {};
        cableForceDataset = {};
        stressDataset = {};
        strainDataset = {};
        domainType.reset();
        iterationType.reset();
        indexType.reset();
        nodalType.reset();
        elementForceType.reset();
        trussForceType.reset();
        cableForceType.reset();
        elementStressType.reset();
        elementStrainType.reset();
    }

    void CloseResult()
    {
        resultFile.reset();
        if (!resultTempFileName.isEmpty())
            QFile::remove(resultTempFileName);
        resultTempFileName.clear();
        resultSourceFileName.clear();
        resultDomains.clear();
        elementTypes.clear();
        displacementIndex.clear();
        currentCoordinateIndex.clear();
        velocityIndex.clear();
        accelerationIndex.clear();
        elementForceIndex.clear();
        trussForceIndex.clear();
        cableForceIndex.clear();
        stressIndex.clear();
        strainIndex.clear();
    }

    bool Begin(const QString& fileName, const StructureData* pData, const QString& sourceModelName)
    {
        if (!pData || fileName.isEmpty())
        {
            return false;
        }

        const QFileInfo fileInfo(fileName);
        if (!fileInfo.absoluteDir().exists() && !QDir().mkpath(fileInfo.absolutePath()))
        {
            qDebug() << "Failed to create HDF5 output directory:" << fileInfo.absolutePath();
            return false;
        }

        targetFileName = fileInfo.absoluteFilePath();
        // 流式文件保留在目标卷，结束时可执行同卷原子重命名，避免数 GB 的跨卷复制。
        tempFileName = MakeAsciiTempHdf5FileNameNear(targetFileName);
        elementTypes = BuildElementTypeMap(pData);
        streamRanges = {};

        const QByteArray path = ToHdf5Path(tempFileName);
        file.reset(H5Fcreate(path.constData(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT), H5Fclose);
        if (!file.valid())
        {
            qDebug() << "Failed to create HDF5 file:" << fileName;
            return false;
        }

        if (!CreateGroupRecursive(file, "/YQY") || !CreateGroupRecursive(file, "/INDEX/YQY"))
        {
            return false;
        }

        const bool attrOk =
            WriteStringAttribute(file, "FORMAT", "YQY_H5") && WriteStringAttribute(file, "FILE_KIND", "MODEL_RESULT") &&
            WriteIntAttribute(file, "RESULT_COMPLETE", 0) &&
            WriteIntAttribute(file, "DYNAMIC_STATE_FORMAT_VERSION", kDynamicStateFormatVersion) &&
            WriteIntAttribute(file, "RESULT_FORMAT_VERSION", kResultFormatVersion) &&
            WriteStringAttribute(file, "CREATED_TIME", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")) &&
            WriteStringAttribute(file, "PROGRAM", "YQY_CAE") &&
            WriteStringAttribute(file, "SOURCE_MODEL", sourceModelName);

        const bool initialized = attrOk && WriteInputData(file, pData) &&
                                 CreateStreamDatasets()
                                 // 使用初始无效范围标记中断的流式结果，正常结束时覆盖这些范围。
                                 && WriteResultRangeAttributes(file, streamRanges);
        if (initialized)
            H5Fflush(file, H5F_SCOPE_GLOBAL);
        else
        {
            ResetStreamResources();
            file.reset();
            QFile::remove(tempFileName);
            tempFileName.clear();
            targetFileName.clear();
        }
        return initialized;
    }

    bool CreateStreamDatasets()
    {
        ResetStreamResources();
        domainType = CreateDomainType();
        iterationType = CreateSolverIterationType();
        indexType = CreateIndexType();
        nodalType = CreateNodalType();
        elementForceType = CreateElementForceType();
        trussForceType = CreateTrussForceType();
        cableForceType = CreateCableForceType();
        elementStressType = CreateElementStressType();
        elementStrainType = CreateElementStrainType();
        if (!domainType.valid() || !iterationType.valid() || !indexType.valid() || !nodalType.valid() ||
            !elementForceType.valid() || !trussForceType.valid() || !cableForceType.valid() ||
            !elementStressType.valid() || !elementStrainType.valid())
            return false;

        domainsDataset.handle = CreateExtendableDataset(file, "/YQY/RESULT/DOMAINS", domainType);
        iterationsDataset.handle = CreateExtendableDataset(file, "/YQY/RESULT/SOLVER_ITERATION", iterationType);
        displacementDataset.data.handle = CreateExtendableDataset(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType);
        currentCoordinateDataset.data.handle =
            CreateExtendableDataset(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType);
        velocityDataset.data.handle = CreateExtendableDataset(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType);
        accelerationDataset.data.handle = CreateExtendableDataset(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType);
        reactionDataset.data.handle = CreateExtendableDataset(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType);
        elementForceDataset.data.handle =
            CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType);
        trussForceDataset.data.handle =
            CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", trussForceType);
        cableForceDataset.data.handle =
            CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE", cableForceType);
        stressDataset.data.handle = CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType);
        strainDataset.data.handle = CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType);
        displacementDataset.index.handle =
            CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType);
        currentCoordinateDataset.index.handle =
            CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType);
        velocityDataset.index.handle = CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType);
        accelerationDataset.index.handle =
            CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType);
        reactionDataset.index.handle =
            CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType);
        elementForceDataset.index.handle =
            CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType);
        trussForceDataset.index.handle =
            CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", indexType);
        cableForceDataset.index.handle =
            CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE", indexType);
        stressDataset.index.handle = CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType);
        strainDataset.index.handle = CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType);

        return domainsDataset.handle.valid() && iterationsDataset.handle.valid() &&
               displacementDataset.data.handle.valid() && currentCoordinateDataset.data.handle.valid() &&
               velocityDataset.data.handle.valid() && accelerationDataset.data.handle.valid() &&
               reactionDataset.data.handle.valid() && elementForceDataset.data.handle.valid() &&
               trussForceDataset.data.handle.valid() && cableForceDataset.data.handle.valid() &&
               stressDataset.data.handle.valid() && strainDataset.data.handle.valid() &&
               displacementDataset.index.handle.valid() && currentCoordinateDataset.index.handle.valid() &&
               velocityDataset.index.handle.valid() && accelerationDataset.index.handle.valid() &&
               reactionDataset.index.handle.valid() && elementForceDataset.index.handle.valid() &&
               trussForceDataset.index.handle.valid() && cableForceDataset.index.handle.valid() &&
               stressDataset.index.handle.valid() && strainDataset.index.handle.valid();
    }

    bool QueueFrame(int domainId, int stepId, int increment, int analysis, double time, const DataFrame& frame)
    {
        if (!file.valid())
        {
            return false;
        }

        DomainRecord domain;
        domain.id = domainId;
        domain.stepId = stepId;
        domain.increment = increment;
        domain.analysis = analysis;
        domain.time = time;

        pendingDomains.push_back(domain);

        const bool storesDynamicKinematics = analysis == static_cast<int>(EnumKeyword::StepType::DYNAMIC);

        std::vector<NodalRecord> displacements;
        std::vector<NodalRecord> currentCoordinates;
        std::vector<NodalRecord> velocities;
        std::vector<NodalRecord> accelerations;
        std::vector<NodalRecord> reactions;
        displacements.reserve(frame.GetNodeDatas().size());
        currentCoordinates.reserve(frame.GetNodeDatas().size());
        velocities.reserve(frame.GetNodeDatas().size());
        accelerations.reserve(frame.GetNodeDatas().size());
        reactions.reserve(frame.GetNodeDatas().size());

        for (const auto& pair : frame.GetNodeDatas())
        {
            const int nodeId = pair.first;

            NodalRecord displacement;
            displacement.id = nodeId;
            displacement.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U1);
            displacement.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U2);
            displacement.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::U3);
            displacement.rx = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::UR1);
            displacement.ry = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::UR2);
            displacement.rz = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::UR3);
            displacement.domainId = domainId;
            displacements.push_back(displacement);

            NodalRecord currentCoordinate;
            currentCoordinate.id = nodeId;
            currentCoordinate.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::CX);
            currentCoordinate.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::CY);
            currentCoordinate.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::CZ);
            currentCoordinate.domainId = domainId;
            currentCoordinates.push_back(currentCoordinate);

            if (storesDynamicKinematics)
            {
                NodalRecord velocity;
                velocity.id = nodeId;
                velocity.x = pair.second.GetVelocityComponent(0);
                velocity.y = pair.second.GetVelocityComponent(1);
                velocity.z = pair.second.GetVelocityComponent(2);
                velocity.rx = pair.second.GetVelocityComponent(3);
                velocity.ry = pair.second.GetVelocityComponent(4);
                velocity.rz = pair.second.GetVelocityComponent(5);
                velocity.domainId = domainId;
                velocities.push_back(velocity);

                NodalRecord acceleration;
                acceleration.id = nodeId;
                acceleration.x = pair.second.GetAccelerationComponent(0);
                acceleration.y = pair.second.GetAccelerationComponent(1);
                acceleration.z = pair.second.GetAccelerationComponent(2);
                acceleration.rx = pair.second.GetAccelerationComponent(3);
                acceleration.ry = pair.second.GetAccelerationComponent(4);
                acceleration.rz = pair.second.GetAccelerationComponent(5);
                acceleration.domainId = domainId;
                accelerations.push_back(acceleration);
            }

            NodalRecord reaction;
            reaction.id = nodeId;
            reaction.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R1);
            reaction.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R2);
            reaction.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R3);
            reaction.domainId = domainId;
            reactions.push_back(reaction);
        }

        std::vector<ElementForceRecord> elementForces;
        std::vector<TrussForceRecord> trussForces;
        std::vector<CableForceRecord> cableForces;
        std::vector<ElementStressRecord> elementStresses;
        std::vector<ElementStrainRecord> elementStrains;
        elementForces.reserve(frame.GetElementDatas().size());
        elementStresses.reserve(frame.GetElementDatas().size());
        elementStrains.reserve(frame.GetElementDatas().size());

        for (const auto& pair : frame.GetElementDatas())
        {
            const int elementId = pair.first;
            const auto elementType = elementTypes.value(elementId, EnumKeyword::ElementType::UNKNOWN);
            switch (elementType)
            {
                case EnumKeyword::ElementType::T3D2:
                {
                    TrussForceRecord force;
                    force.id = elementId;
                    force.domainId = domainId;
                    force.axial = frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce);
                    trussForces.push_back(force);
                    break;
                }
                case EnumKeyword::ElementType::CABLE:
                {
                    CableForceRecord force;
                    force.id = elementId;
                    force.domainId = domainId;
                    force.axial = frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce);
                    force.torque = frame.GetElementData(elementId, EnumKeyword::ElementResultType::Torque);
                    cableForces.push_back(force);
                    break;
                }
                default:
                {
                    ElementForceRecord force;
                    force.id = elementId;
                    force.axial = frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce);
                    force.shearY = frame.GetElementData(elementId, EnumKeyword::ElementResultType::ShearY);
                    force.shearZ = frame.GetElementData(elementId, EnumKeyword::ElementResultType::ShearZ);
                    force.torque = frame.GetElementData(elementId, EnumKeyword::ElementResultType::Torque);
                    force.momentY = frame.GetElementData(elementId, EnumKeyword::ElementResultType::MomentY);
                    force.momentZ = frame.GetElementData(elementId, EnumKeyword::ElementResultType::MomentZ);
                    force.domainId = domainId;
                    elementForces.push_back(force);
                    break;
                }
            }

            ElementStressRecord stress;
            stress.id = elementId;
            stress.initStress = frame.GetElementData(elementId, EnumKeyword::ElementResultType::InitStress);
            stress.currentStress = frame.GetElementData(elementId, EnumKeyword::ElementResultType::CurrentStress);
            stress.deltaStress = frame.GetElementData(elementId, EnumKeyword::ElementResultType::DeltaStress);
            stress.domainId = domainId;
            elementStresses.push_back(stress);

            ElementStrainRecord strain;
            strain.id = elementId;
            strain.strain = frame.GetElementData(elementId, EnumKeyword::ElementResultType::Strain);
            strain.domainId = domainId;
            elementStrains.push_back(strain);
        }

        // 帧数据位于 CPU 缓存时同步累计可视化范围。打开完整结果时只需读取 21 个标量属性，
        // 不必扫描数 GB 文件中的每条记录。
        for (const NodalRecord& value : displacements)
        {
            IncludeResultRangeValue(streamRanges.displacementMagnitude,
                                    std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z));
            IncludeResultRangeValue(streamRanges.displacementX, value.x);
            IncludeResultRangeValue(streamRanges.displacementY, value.y);
            IncludeResultRangeValue(streamRanges.displacementZ, value.z);
        }
        for (const ElementForceRecord& value : elementForces)
            IncludeResultRangeValue(streamRanges.axialForce, value.axial);
        for (const TrussForceRecord& value : trussForces)
            IncludeResultRangeValue(streamRanges.axialForce, value.axial);
        for (const CableForceRecord& value : cableForces)
            IncludeResultRangeValue(streamRanges.axialForce, value.axial);
        for (const ElementStressRecord& value : elementStresses)
            IncludeResultRangeValue(streamRanges.stress, value.currentStress);
        for (const ElementStrainRecord& value : elementStrains)
            IncludeResultRangeValue(streamRanges.strain, value.strain);

        QueueRecordsWithIndex(displacementDataset, domainId, displacements, pendingDisplacements,
                              pendingDisplacementIndices);
        QueueRecordsWithIndex(currentCoordinateDataset, domainId, currentCoordinates, pendingCurrentCoordinates,
                              pendingCurrentCoordinateIndices);
        QueueRecordsWithIndex(velocityDataset, domainId, velocities, pendingVelocities, pendingVelocityIndices);
        QueueRecordsWithIndex(accelerationDataset, domainId, accelerations, pendingAccelerations,
                              pendingAccelerationIndices);
        QueueRecordsWithIndex(reactionDataset, domainId, reactions, pendingReactions, pendingReactionIndices);
        QueueRecordsWithIndex(elementForceDataset, domainId, elementForces, pendingElementForces,
                              pendingElementForceIndices);
        QueueRecordsWithIndex(trussForceDataset, domainId, trussForces, pendingTrussForces, pendingTrussForceIndices);
        QueueRecordsWithIndex(cableForceDataset, domainId, cableForces, pendingCableForces, pendingCableForceIndices);
        QueueRecordsWithIndex(stressDataset, domainId, elementStresses, pendingElementStresses, pendingStressIndices);
        QueueRecordsWithIndex(strainDataset, domainId, elementStrains, pendingElementStrains, pendingStrainIndices);
        return true;
    }

    template <typename Record>
    void QueueRecordsWithIndex(const StreamDatasetPair& datasets, int domainId, const std::vector<Record>& records,
                               _OUT std::vector<Record>& pendingRecords, _OUT std::vector<IndexRecord>& pendingIndices)
    {
        IndexRecord index;
        index.domainId = domainId;
        index.position = static_cast<long long>(datasets.data.size + pendingRecords.size());
        index.length = static_cast<long long>(records.size());
        pendingIndices.push_back(index);
        pendingRecords.insert(pendingRecords.end(), records.cbegin(), records.cend());
    }

    void ClearPendingFrames()
    {
        pendingDomains.clear();
        pendingDisplacements.clear();
        pendingCurrentCoordinates.clear();
        pendingVelocities.clear();
        pendingAccelerations.clear();
        pendingReactions.clear();
        pendingElementForces.clear();
        pendingTrussForces.clear();
        pendingCableForces.clear();
        pendingElementStresses.clear();
        pendingElementStrains.clear();
        pendingDisplacementIndices.clear();
        pendingCurrentCoordinateIndices.clear();
        pendingVelocityIndices.clear();
        pendingAccelerationIndices.clear();
        pendingReactionIndices.clear();
        pendingElementForceIndices.clear();
        pendingTrussForceIndices.clear();
        pendingCableForceIndices.clear();
        pendingStressIndices.clear();
        pendingStrainIndices.clear();
    }

    bool FlushPendingFrames()
    {
        long long position = 0;
        const bool written = AppendDataset(domainsDataset, domainType, pendingDomains, position) &&
                             AppendDataset(displacementDataset.data, nodalType, pendingDisplacements, position) &&
                             AppendDataset(displacementDataset.index, indexType, pendingDisplacementIndices,
                                           position) &&
                             AppendDataset(currentCoordinateDataset.data, nodalType, pendingCurrentCoordinates,
                                           position) &&
                             AppendDataset(currentCoordinateDataset.index, indexType, pendingCurrentCoordinateIndices,
                                           position) &&
                             AppendDataset(velocityDataset.data, nodalType, pendingVelocities, position) &&
                             AppendDataset(velocityDataset.index, indexType, pendingVelocityIndices, position) &&
                             AppendDataset(accelerationDataset.data, nodalType, pendingAccelerations, position) &&
                             AppendDataset(accelerationDataset.index, indexType, pendingAccelerationIndices,
                                           position) &&
                             AppendDataset(reactionDataset.data, nodalType, pendingReactions, position) &&
                             AppendDataset(reactionDataset.index, indexType, pendingReactionIndices, position) &&
                             AppendDataset(elementForceDataset.data, elementForceType, pendingElementForces,
                                           position) &&
                             AppendDataset(elementForceDataset.index, indexType, pendingElementForceIndices,
                                           position) &&
                             AppendDataset(trussForceDataset.data, trussForceType, pendingTrussForces, position) &&
                             AppendDataset(trussForceDataset.index, indexType, pendingTrussForceIndices, position) &&
                             AppendDataset(cableForceDataset.data, cableForceType, pendingCableForces, position) &&
                             AppendDataset(cableForceDataset.index, indexType, pendingCableForceIndices, position) &&
                             AppendDataset(stressDataset.data, elementStressType, pendingElementStresses, position) &&
                             AppendDataset(stressDataset.index, indexType, pendingStressIndices, position) &&
                             AppendDataset(strainDataset.data, elementStrainType, pendingElementStrains, position) &&
                             AppendDataset(strainDataset.index, indexType, pendingStrainIndices, position);
        ClearPendingFrames();
        return written;
    }

    bool WriteFrames(int firstDomainId, const std::vector<DataFrame>& frames)
    {
        ClearPendingFrames();
        for (int frameIndex = 0; frameIndex < static_cast<int>(frames.size()); ++frameIndex)
        {
            const DataFrame& frame = frames[static_cast<std::size_t>(frameIndex)];
            if (!QueueFrame(firstDomainId + frameIndex, frame.GetStepId(), frame.GetIncrement(),
                            frame.GetAnalysisType(), frame.GetTime(), frame))
            {
                ClearPendingFrames();
                return false;
            }
        }
        return FlushPendingFrames();
    }

    bool WriteSolverIterationHistory(const std::vector<SolverIterationRecord>& records)
    {
        if (!file.valid())
            return false;
        std::vector<SolverIterationH5Record> h5Records;
        h5Records.reserve(records.size());
        for (const SolverIterationRecord& record : records)
            h5Records.push_back({record.stepId, record.analysisType, record.time, record.iterations});
        long long position = 0;
        return AppendDataset(iterationsDataset, iterationType, h5Records, position);
    }

    bool End(bool resultComplete)
    {
        bool finalized = true;
        if (file.valid())
        {
            finalized = WriteIntAttribute(file, "RESULT_COMPLETE", resultComplete ? 1 : 0) &&
                        WriteResultRangeAttributes(file, streamRanges) && H5Fflush(file, H5F_SCOPE_GLOBAL) >= 0;
        }
        ResetStreamResources();
        file.reset();

        if (!tempFileName.isEmpty() && !targetFileName.isEmpty())
        {
            finalized = MoveTempFileToTarget(tempFileName, targetFileName) && finalized;
        }
        else
        {
            finalized = false;
        }
        tempFileName.clear();
        targetFileName.clear();
        return finalized;
    }
};

Hdf5ModelIO::Hdf5ModelIO()
    : m_impl(std::make_unique<Impl>())
{
}

Hdf5ModelIO::~Hdf5ModelIO()
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    m_impl.reset();
}

bool Hdf5ModelIO::ExportHdf5(const QString& fileName, const StructureData* pData) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    return ExportHdf5(fileName, pData, QString());
}

bool Hdf5ModelIO::ExportHdf5(const QString& fileName, const StructureData* pData, const QString& sourceModelName,
                             bool resultComplete) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    if (!pData || fileName.isEmpty())
    {
        return false;
    }

    const QFileInfo fileInfo(fileName);
    if (!fileInfo.absoluteDir().exists() && !QDir().mkpath(fileInfo.absolutePath()))
    {
        qDebug() << "Failed to create HDF5 output directory:" << fileInfo.absolutePath();
        return false;
    }

    const QString targetFileName = fileInfo.absoluteFilePath();
    const QString tempFileName = MakeAsciiTempHdf5FileNameNear(targetFileName);
    const QByteArray path = ToHdf5Path(tempFileName);
    H5Handle file(H5Fcreate(path.constData(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT), H5Fclose);
    if (!file.valid())
    {
        qDebug() << "创建 HDF5 文件失败:" << fileName;
        return false;
    }

    if (!CreateGroupRecursive(file, "/YQY") || !CreateGroupRecursive(file, "/INDEX/YQY"))
    {
        return false;
    }

    const bool attrOk =
        WriteStringAttribute(file, "FORMAT", "YQY_H5") && WriteStringAttribute(file, "FILE_KIND", "MODEL_RESULT") &&
        WriteIntAttribute(file, "RESULT_COMPLETE", resultComplete ? 1 : 0) &&
        WriteIntAttribute(file, "DYNAMIC_STATE_FORMAT_VERSION", kDynamicStateFormatVersion) &&
        WriteIntAttribute(file, "RESULT_FORMAT_VERSION", kResultFormatVersion) &&
        WriteStringAttribute(file, "CREATED_TIME", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")) &&
        WriteStringAttribute(file, "PROGRAM", "YQY_CAE") && WriteStringAttribute(file, "SOURCE_MODEL", sourceModelName);

    const bool dataOk = WriteInputData(file, pData) && WriteResultData(file, pData);
    if (!attrOk || !dataOk)
    {
        file.reset();
        QFile::remove(tempFileName);
        qDebug() << "写入HDF5数据失败:" << fileName;
        return false;
    }

    qDebug().noquote() << QStringLiteral("H5/HDF5 文件已输出至") << fileName;
    file.reset();
    return MoveTempFileToTarget(tempFileName, targetFileName);
}

bool Hdf5ModelIO::ExportModelHdf5(const QString& fileName, const StructureData* pData,
                                  const QString& sourceModelName) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    if (!pData || fileName.isEmpty())
    {
        return false;
    }

    const QFileInfo fileInfo(fileName);
    if (!fileInfo.absoluteDir().exists() && !QDir().mkpath(fileInfo.absolutePath()))
    {
        return false;
    }
    const QString targetFileName = fileInfo.absoluteFilePath();
    const QString tempFileName = MakeAsciiTempHdf5FileNameNear(targetFileName);
    const QByteArray path = ToHdf5Path(tempFileName);
    H5Handle file(H5Fcreate(path.constData(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT), H5Fclose);
    if (!file.valid() || !CreateGroupRecursive(file, "/YQY"))
    {
        QFile::remove(tempFileName);
        return false;
    }

    const bool written =
        WriteStringAttribute(file, "FORMAT", "YQY_H5") && WriteStringAttribute(file, "FILE_KIND", "MODEL") &&
        WriteIntAttribute(file, "RESULT_COMPLETE", 0) &&
        WriteStringAttribute(file, "CREATED_TIME", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss")) &&
        WriteStringAttribute(file, "PROGRAM", "YQY_CAE") &&
        WriteStringAttribute(file, "SOURCE_MODEL", sourceModelName) && WriteInputData(file, pData);
    file.reset();
    if (!written)
    {
        QFile::remove(tempFileName);
        return false;
    }
    return MoveTempFileToTarget(tempFileName, targetFileName);
}

bool Hdf5ModelIO::BeginResultStream(const QString& fileName, const StructureData* pData, const QString& sourceModelName)
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    return m_impl->Begin(fileName, pData, sourceModelName);
}

bool Hdf5ModelIO::WriteResultFrame(int domainId, int stepId, int increment, int analysis, double time,
                                   const DataFrame& frame)
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    if (!m_impl->QueueFrame(domainId, stepId, increment, analysis, time, frame))
        return false;
    return m_impl->FlushPendingFrames();
}

bool Hdf5ModelIO::WriteResultFrames(int firstDomainId, const std::vector<DataFrame>& frames)
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    return m_impl->WriteFrames(firstDomainId, frames);
}

bool Hdf5ModelIO::WriteSolverIterationHistory(const std::vector<SolverIterationRecord>& records)
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    return m_impl->WriteSolverIterationHistory(records);
}

bool Hdf5ModelIO::EndResultStream(bool resultComplete)
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    return m_impl->End(resultComplete);
}

namespace
{
template <typename Record>
bool ReadDatasetAll(hid_t file, const char* path, hid_t memoryType, std::vector<Record>& records)
{
    H5Handle dataset(H5Dopen2(file, path, H5P_DEFAULT), H5Dclose);
    if (!dataset.valid())
    {
        return false;
    }

    H5Handle space(H5Dget_space(dataset), H5Sclose);
    if (!space.valid())
    {
        return false;
    }

    hsize_t dims[1] = {0};
    if (H5Sget_simple_extent_dims(space, dims, nullptr) < 0)
    {
        return false;
    }

    records.resize(static_cast<size_t>(dims[0]));
    if (records.empty())
    {
        return true;
    }

    return H5Dread(dataset, memoryType, H5S_ALL, H5S_ALL, H5P_DEFAULT, records.data()) >= 0;
}

template <typename Record>
bool ReadOptionalDatasetAll(hid_t file, const char* path, hid_t memoryType, std::vector<Record>& records)
{
    records.clear();
    return !LinkExistsQuietly(file, path) || ReadDatasetAll(file, path, memoryType, records);
}

bool ReadInputData(hid_t file, StructureData* data)
{
    if (!data)
        return false;

    H5Handle gridType = CreateGridType();
    H5Handle materialType = CreateMaterialType();
    H5Handle sectionType = CreateSectionType();
    H5Handle propertyType = CreatePropertyType();
    H5Handle elementType = CreateElementType();
    H5Handle constraintType = CreateConstraintType();
    H5Handle mpcType = CreateMPCType();
    H5Handle rigidBodyInertiaType = CreateRigidBodyInertiaType();
    H5Handle loadType = CreateLoadType();
    H5Handle windDirectionType = CreateWindDirectionType();
    H5Handle stepType = CreateStepType();
    H5Handle modelSetType = CreateModelSetType();
    H5Handle modelSetMemberType = CreateModelSetMemberType();
    H5Handle computeRegionType = CreateComputeRegionType();
    H5Handle regionLinkType = CreateRegionLinkType();
    H5Handle stepMetadataType = CreateStepMetadataType();
    H5Handle stepGallopingType = CreateStepGallopingType();
    H5Handle stepTssbnType = CreateStepTssbnType();
    H5Handle stepStructuralDampingType = CreateStepStructuralDampingType();
    H5Handle stepRegionLinkType = CreateStepRegionLinkType();
    H5Handle aeroCaseType = CreateAeroCaseType();
    H5Handle aeroCoefficientType = CreateAeroCoefficientType();
    if (!gridType.valid() || !materialType.valid() || !sectionType.valid() || !propertyType.valid() ||
        !elementType.valid() || !constraintType.valid() || !mpcType.valid() || !rigidBodyInertiaType.valid() ||
        !loadType.valid() ||
        !windDirectionType.valid() || !stepType.valid() || !modelSetType.valid() || !modelSetMemberType.valid() ||
        !computeRegionType.valid() || !regionLinkType.valid() || !stepMetadataType.valid() ||
        !stepGallopingType.valid() || !stepTssbnType.valid() || !stepStructuralDampingType.valid() ||
        !stepRegionLinkType.valid() || !aeroCaseType.valid() || !aeroCoefficientType.valid())
    {
        return false;
    }

    std::vector<GridRecord> grids;
    std::vector<MaterialRecord> materials;
    std::vector<SectionRecord> sections;
    std::vector<PropertyRecord> properties;
    std::vector<ElementRecord> elements;
    std::vector<ConstraintRecord> constraints;
    std::vector<MPCRecord> mpcs;
    std::vector<RigidBodyInertiaRecord> rigidBodyInertias;
    std::vector<LoadRecord> loads;
    std::vector<WindDirectionRecord> windDirections;
    std::vector<StepRecord> steps;
    std::vector<ModelSetRecord> modelSets;
    std::vector<ModelSetMemberRecord> modelSetMembers;
    std::vector<ComputeRegionRecord> computeRegions;
    std::vector<RegionLinkRecord> regionSetLinks;
    std::vector<RegionLinkRecord> regionDirectNodes;
    std::vector<RegionLinkRecord> regionDirectElements;
    std::vector<StepMetadataRecord> stepMetadata;
    std::vector<StepGallopingRecord> stepGalloping;
    std::vector<StepTssbnRecord> stepTssbn;
    std::vector<StepStructuralDampingRecord> stepStructuralDamping;
    std::vector<StepRegionLinkRecord> stepRegionLinks;
    std::vector<AeroCaseRecord> aeroCases;
    std::vector<AeroCoefficientRecord> aeroCoefficients;
    if (!ReadDatasetAll(file, "/YQY/INPUT/NODE/GRID", gridType, grids) ||
        !ReadDatasetAll(file, "/YQY/INPUT/MATERIAL/MAT", materialType, materials) ||
        !ReadDatasetAll(file, "/YQY/INPUT/SECTION/SECTION", sectionType, sections) ||
        !ReadDatasetAll(file, "/YQY/INPUT/PROPERTY/PROPERTY", propertyType, properties) ||
        !ReadDatasetAll(file, "/YQY/INPUT/ELEMENT/ELEMENT", elementType, elements) ||
        !ReadDatasetAll(file, "/YQY/INPUT/CONSTRAINT/SPC", constraintType, constraints) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/CONSTRAINT/MPC", mpcType, mpcs) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/INERTIA/RIGID_BODY", rigidBodyInertiaType,
                                rigidBodyInertias) ||
        !ReadDatasetAll(file, "/YQY/INPUT/LOAD/LOAD", loadType, loads) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/LOAD/WIND_DIRECTION", windDirectionType, windDirections) ||
        !ReadDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/STEP", stepType, steps) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/METADATA", stepMetadataType, stepMetadata) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/GALLOPING", stepGallopingType, stepGalloping) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/TSSBN", stepTssbnType, stepTssbn) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/STRUCTURAL_DAMPING", stepStructuralDampingType,
                                stepStructuralDamping) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/REGION_LINK", stepRegionLinkType, stepRegionLinks) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/AERO/CASE", aeroCaseType, aeroCases) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/AERO/COEFFICIENT", aeroCoefficientType, aeroCoefficients) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/MODEL_SET/SET", modelSetType, modelSets) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/MODEL_SET/MEMBER", modelSetMemberType, modelSetMembers) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/COMPUTE_REGION/REGION", computeRegionType, computeRegions) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/COMPUTE_REGION/SET_LINK", regionLinkType, regionSetLinks) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/COMPUTE_REGION/DIRECT_NODE", regionLinkType, regionDirectNodes) ||
        !ReadOptionalDatasetAll(file, "/YQY/INPUT/COMPUTE_REGION/DIRECT_ELEMENT", regionLinkType,
                                regionDirectElements) ||
        grids.empty())
    {
        return false;
    }

    auto restored = std::make_unique<StructureData>();
    for (const GridRecord& record : grids)
    {
        if (record.id <= 0 || restored->m_Nodes.find(record.id) != restored->m_Nodes.end())
            return false;
        auto node = std::make_shared<Node>();
        node->m_Id = record.id;
        node->m_X = record.x[0];
        node->m_Y = record.x[1];
        node->m_Z = record.x[2];
        node->SetNumDOFs(std::max(3, record.dofCount));
        restored->m_Nodes.emplace(record.id, std::move(node));
    }

    for (const MaterialRecord& record : materials)
    {
        auto material = std::make_shared<Material>();
        material->m_Id = record.id;
        material->m_Young = record.young;
        material->m_Poisson = record.poisson;
        material->m_Density = record.density;
        material->m_MaxStress = record.maxStress;
        material->m_Expansion = record.expansion;
        restored->m_Material[record.id] = std::move(material);
    }

    for (const SectionRecord& record : sections)
    {
        std::shared_ptr<SectionBase> section;
        const auto storedType = static_cast<EnumKeyword::SectionType>(record.type);
        if (storedType == EnumKeyword::SectionType::RECTANGULAR ||
            (storedType == EnumKeyword::SectionType::UNKNOWN && record.width > 0.0 && record.height > 0.0))
        {
            auto rectangle = std::make_shared<SectionRectangle>();
            rectangle->m_Width = record.width;
            rectangle->m_Height = record.height;
            section = std::move(rectangle);
        }
        else
        {
            section = std::make_shared<SectionCircular>();
        }
        section->m_Id = record.id;
        section->m_Area = record.area;
        section->m_Radius = record.radius;
        section->Io = record.iy;
        section->Irr = record.iz;
        restored->m_Section[record.id] = std::move(section);
    }

    for (const PropertyRecord& record : properties)
    {
        auto property = std::make_shared<Property>();
        property->m_Id = record.id;
        if (record.materialId > 0)
        {
            const auto material = restored->m_Material.find(record.materialId);
            if (material == restored->m_Material.end())
                return false;
            property->m_pMaterial = material->second;
        }
        if (record.sectionId > 0)
        {
            const auto section = restored->m_Section.find(record.sectionId);
            if (section == restored->m_Section.end())
                return false;
            property->m_pSection = section->second;
        }
        restored->m_Property[record.id] = std::move(property);
    }

    for (const ElementRecord& record : elements)
    {
        std::shared_ptr<ElementBase> element;
        switch (static_cast<EnumKeyword::ElementType>(record.type))
        {
            case EnumKeyword::ElementType::T3D2:
                element = std::make_shared<ElementTruss>();
                break;
            case EnumKeyword::ElementType::CABLE:
                element = std::make_shared<ElementCable>();
                break;
            case EnumKeyword::ElementType::CR3D:
            {
                auto beam = std::make_shared<ElementBeam_CR>();
                beam->q0 = Eigen::Vector3d(record.q0[0], record.q0[1], record.q0[2]);
                element = std::move(beam);
                break;
            }
            default:
                element = std::make_shared<ElementTruss>();
                break;
        }

        element->m_Id = record.id;
        element->m_InitStress = record.initStress;
        element->m_Role = static_cast<ElementRole>(record.role);
        element->m_WireId = record.wireId;
        element->m_AeroBundleCount = record.aeroBundleCount;
        element->m_AeroProfileId = record.aeroProfileId;
        if (element->m_pNode.size() < 2)
        {
            element->m_pNode.resize(2);
        }
        for (size_t nodeIndex = 0; nodeIndex < 2; ++nodeIndex)
        {
            const int nodeId = record.nodeIds[nodeIndex];
            const auto node = restored->m_Nodes.find(nodeId);
            if (node == restored->m_Nodes.end())
                return false;
            element->m_pNode[nodeIndex] = node->second;
        }
        if (record.propertyId > 0)
        {
            const auto property = restored->m_Property.find(record.propertyId);
            if (property == restored->m_Property.end())
                return false;
            element->m_pProperty = property->second;
        }
        const auto firstNode = element->m_pNode[0].lock();
        const auto secondNode = element->m_pNode[1].lock();
        const double dx = secondNode->m_X - firstNode->m_X;
        const double dy = secondNode->m_Y - firstNode->m_Y;
        const double dz = secondNode->m_Z - firstNode->m_Z;
        element->L0 = std::sqrt(dx * dx + dy * dy + dz * dz);
        element->L = element->L0;
        restored->m_Elements[record.id] = std::move(element);
    }

    for (const ModelSetRecord& record : modelSets)
    {
        if (record.id <= 0 || restored->m_ModelSets.find(record.id) != restored->m_ModelSets.cend() ||
            (record.type != static_cast<int>(ModelSetType::Node) &&
             record.type != static_cast<int>(ModelSetType::Element)))
        {
            return false;
        }
        auto modelSet = std::make_shared<ModelSet>(static_cast<ModelSetType>(record.type));
        modelSet->m_Id = record.id;
        modelSet->m_Name = ReadUtf8String(record.name, sizeof(record.name));
        restored->m_ModelSets.emplace(record.id, std::move(modelSet));
    }
    for (const ModelSetMemberRecord& record : modelSetMembers)
    {
        const auto setIt = restored->m_ModelSets.find(record.setId);
        if (setIt == restored->m_ModelSets.cend() || !setIt->second || record.entityId <= 0)
        {
            return false;
        }
        const bool entityExists = setIt->second->m_Type == ModelSetType::Node
                                      ? restored->m_Nodes.find(record.entityId) != restored->m_Nodes.cend()
                                      : restored->m_Elements.find(record.entityId) != restored->m_Elements.cend();
        if (!entityExists)
        {
            return false;
        }
        setIt->second->m_Ids.insert(record.entityId);
    }
    for (const auto& [setId, modelSet] : restored->m_ModelSets)
    {
        Q_UNUSED(setId);
        if (!modelSet || modelSet->m_Ids.empty())
        {
            return false;
        }
    }

    for (const ComputeRegionRecord& record : computeRegions)
    {
        if (record.id <= 0 || restored->m_ComputeRegions.find(record.id) != restored->m_ComputeRegions.cend())
        {
            return false;
        }
        auto region = std::make_shared<ComputeRegion>();
        region->m_Id = record.id;
        region->m_Name = ReadUtf8String(record.name, sizeof(record.name));
        region->m_Enabled = record.enabled != 0;
        region->m_AutoGenerated = record.autoGenerated != 0;
        restored->m_ComputeRegions.emplace(record.id, std::move(region));
    }
    const auto findRegion = [&restored](int regionId) -> std::shared_ptr<ComputeRegion>
    {
        const auto regionIt = restored->m_ComputeRegions.find(regionId);
        return regionIt != restored->m_ComputeRegions.cend() ? regionIt->second : nullptr;
    };
    for (const RegionLinkRecord& record : regionSetLinks)
    {
        const auto region = findRegion(record.regionId);
        if (!region || restored->m_ModelSets.find(record.targetId) == restored->m_ModelSets.cend())
        {
            return false;
        }
        region->m_SourceSetIds.insert(record.targetId);
    }
    for (const RegionLinkRecord& record : regionDirectNodes)
    {
        const auto region = findRegion(record.regionId);
        if (!region || restored->m_Nodes.find(record.targetId) == restored->m_Nodes.cend())
        {
            return false;
        }
        region->m_DirectNodeIds.insert(record.targetId);
    }
    for (const RegionLinkRecord& record : regionDirectElements)
    {
        const auto region = findRegion(record.regionId);
        if (!region || restored->m_Elements.find(record.targetId) == restored->m_Elements.cend())
        {
            return false;
        }
        region->m_DirectElementIds.insert(record.targetId);
    }

    for (const ConstraintRecord& record : constraints)
    {
        const auto node = restored->m_Nodes.find(record.nodeId);
        if (node == restored->m_Nodes.end())
            return false;
        auto constraint = std::make_shared<Constraint>();
        constraint->m_Id = record.id;
        constraint->m_pNode = node->second;
        constraint->m_Direction = static_cast<EnumKeyword::Direction>(record.direction);
        constraint->m_Value = record.value;
        restored->m_Constraint[record.id] = std::move(constraint);
    }

    for (const RigidBodyInertiaRecord& record : rigidBodyInertias)
    {
        const auto node = restored->m_Nodes.find(record.nodeId);
        if (record.id <= 0 || node == restored->m_Nodes.end() ||
            restored->m_RigidBodyInertias.find(record.id) != restored->m_RigidBodyInertias.end())
        {
            return false;
        }
        auto inertia = std::make_shared<RigidBodyInertia>();
        inertia->m_Id = record.id;
        inertia->m_pNode = node->second;
        inertia->m_Mass = record.mass;
        inertia->m_RotaryInertia << record.inertia[0], record.inertia[3], record.inertia[4], record.inertia[3],
            record.inertia[1], record.inertia[5], record.inertia[4], record.inertia[5], record.inertia[2];
        node->second->SetNumDOFs(6);
        if (!inertia->IsValid())
            return false;
        restored->m_RigidBodyInertias.emplace(record.id, std::move(inertia));
    }

    for (const MPCRecord& record : mpcs)
    {
        const auto master = restored->m_Nodes.find(record.masterNodeId);
        const auto slave = restored->m_Nodes.find(record.slaveNodeId);
        if (record.id <= 0 || master == restored->m_Nodes.cend() || slave == restored->m_Nodes.cend() ||
            master == slave || restored->m_MPCConstraints.find(record.id) != restored->m_MPCConstraints.cend())
        {
            return false;
        }

        std::shared_ptr<NonlinearMPCConstraint> mpc;
        if (record.relationType == 0)
        {
            auto distance = std::make_shared<DistanceMPCConstraint>();
            distance->m_pNodeA = slave->second;
            distance->m_pNodeB = master->second;
            distance->m_pSlaveNode = slave->second;
            distance->m_Length = record.parameters[0];
            for (int direction = 0; direction < 6; ++direction)
            {
                if ((record.slaveDirectionMask & (1 << direction)) != 0)
                {
                    distance->m_SlaveDirection = direction;
                    break;
                }
            }
            mpc = std::move(distance);
        }
        else if (record.relationType == 1)
        {
            auto rigid = std::make_shared<RigidOffsetMPCConstraint>();
            rigid->m_pMasterNode = master->second;
            rigid->m_pSlaveNode = slave->second;
            rigid->m_Offset = Eigen::Vector3d(record.parameters[0], record.parameters[1], record.parameters[2]);
            mpc = std::move(rigid);
        }
        else if (record.relationType == 2)
        {
            auto release = std::make_shared<PlanarShearReleaseMPCConstraint>();
            release->m_pMasterNode = master->second;
            release->m_pSlaveNode = slave->second;
            mpc = std::move(release);
        }
        else if (record.relationType == 3)
        {
            auto tie = std::make_shared<TranslationalTieMPCConstraint>();
            tie->m_pMasterNode = master->second;
            tie->m_pSlaveNode = slave->second;
            mpc = std::move(tie);
        }
        else if (record.relationType == 4)
        {
            auto twist = std::make_shared<AxialTwistTieMPCConstraint>();
            twist->m_pMasterNode = master->second;
            twist->m_pSlaveNode = slave->second;
            twist->m_Axis = Eigen::Vector3d(record.parameters[0], record.parameters[1], record.parameters[2]);
            twist->m_SlaveDirection = 3;
            mpc = std::move(twist);
        }
        else
        {
            return false;
        }

        mpc->m_Id = record.id;
        mpc->m_StepId = record.stepId;
        for (int direction = 0; direction < 6; ++direction)
            if ((record.slaveDirectionMask & (1 << direction)) != 0)
                mpc->m_SlaveDirections.push_back(direction);
        if (mpc->m_SlaveDirections.empty())
            return false;
        restored->m_MPCConstraints[record.id] = std::move(mpc);
    }

    for (const LoadRecord& record : loads)
    {
        std::shared_ptr<LoadBase> load;
        switch (static_cast<EnumKeyword::LoadType>(record.type))
        {
            case EnumKeyword::LoadType::FORCE_NODE:
            {
                const auto node = restored->m_Nodes.find(record.targetId);
                if (node == restored->m_Nodes.end())
                    return false;
                auto nodeLoad = std::make_shared<Force_Node>();
                nodeLoad->m_pNode = node->second;
                nodeLoad->m_Value = record.value;
                load = std::move(nodeLoad);
                break;
            }
            case EnumKeyword::LoadType::FORCE_ELEMENT:
            {
                const auto element = restored->m_Elements.find(record.targetId);
                if (element == restored->m_Elements.end())
                    return false;
                auto elementLoad = std::make_shared<Force_Element>();
                elementLoad->m_pElement = element->second;
                elementLoad->m_Value = record.value;
                load = std::move(elementLoad);
                break;
            }
            case EnumKeyword::LoadType::FORCE_GRAVITY:
            {
                auto gravity = std::make_shared<Force_Gravity>();
                gravity->m_g = record.value;
                load = std::move(gravity);
                break;
            }
            case EnumKeyword::LoadType::FORCE_WIND:
            {
                auto wind = std::make_shared<Force_Wind>();
                wind->m_velocity = record.value;
                load = std::move(wind);
                break;
            }
            default:
                continue;
        }

        load->m_Id = record.id;
        load->m_Direction = static_cast<EnumKeyword::Direction>(record.direction);
        if (const auto wind = std::dynamic_pointer_cast<Force_Wind>(load))
        {
            const int component = record.direction;
            if (component >= 0 && component < 3)
            {
                wind->m_direction.setZero();
                wind->m_direction[component] = 1.0;
            }
        }
        load->m_StepId = record.stepId;
        load->m_FunctionType = static_cast<TimeFunctionType>(record.functionType);
        load->m_StartTime = record.startTime;
        load->m_EndTime = record.endTime;
        load->m_Amplitude = record.params[0];
        load->m_Frequency = record.params[1];
        load->m_Phase = record.params[2];
        load->m_Offset = record.params[3];
        load->m_RampT0 = record.params[4];
        load->m_RampT1 = record.params[5];
        load->m_Decay = record.params[6];
        load->m_Period = record.params[7];
        restored->m_Load[record.id] = std::move(load);
    }
    for (const WindDirectionRecord& record : windDirections)
    {
        const auto found = restored->m_Load.find(record.loadId);
        const auto wind =
            found == restored->m_Load.cend() ? nullptr : std::dynamic_pointer_cast<Force_Wind>(found->second);
        const Eigen::Vector3d direction(record.direction[0], record.direction[1], record.direction[2]);
        if (!wind || !direction.allFinite() || direction.norm() <= 1.0e-12)
            return false;
        wind->m_direction = direction.normalized();
        wind->m_Direction = EnumKeyword::Direction::UNKNOWN;
    }

    for (const StepRecord& record : steps)
    {
        AnalysisStepConfig config;
        config.id = record.id;
        config.name = QStringLiteral("Step-%1").arg(record.id);
        config.type = static_cast<EnumKeyword::StepType>(record.type);
        config.totalTime = record.time;
        config.stepSize = record.stepSize;
        config.tolerance = record.tolerance;
        config.maxIterations = record.maxIterations;
        constexpr int legacyTssbnSolverType = 2;
        config.dynamicSolverType = record.dynamicSolverType == legacyTssbnSolverType
                                       ? SolverNameSpace::SolverType::AdaptiveTSSBN
                                       : static_cast<SolverNameSpace::SolverType>(record.dynamicSolverType);
        restored->AddAnalysisStep(config);
    }

    for (const StepMetadataRecord& record : stepMetadata)
    {
        const auto stepIt = restored->m_AnalysisStep.find(record.stepId);
        if (stepIt == restored->m_AnalysisStep.cend() || !stepIt->second ||
            (record.regionScope != static_cast<int>(AnalysisRegionScope::AllEnabledRegions) &&
             record.regionScope != static_cast<int>(AnalysisRegionScope::SelectedRegions)))
        {
            return false;
        }
        stepIt->second->m_Name = ReadUtf8String(record.name, sizeof(record.name));
        stepIt->second->m_RegionScope = static_cast<AnalysisRegionScope>(record.regionScope);
        stepIt->second->m_InitialStaticStepId = stepIt->second->isDynamic ? record.initialStaticStepId : 0;
    }
    for (const auto& [stepId, step] : restored->m_AnalysisStep)
    {
        if (!step || step->m_InitialStaticStepId <= 0)
            continue;
        const auto staticIt = restored->m_AnalysisStep.find(step->m_InitialStaticStepId);
        if (!step->isDynamic || staticIt == restored->m_AnalysisStep.cend() || !staticIt->second ||
            staticIt->second->m_Type != EnumKeyword::StepType::STATIC || staticIt->first >= stepId)
            return false;
    }
    for (const StepGallopingRecord& record : stepGalloping)
    {
        const auto stepIt = restored->m_AnalysisStep.find(record.stepId);
        if (stepIt == restored->m_AnalysisStep.cend() || !stepIt->second ||
            !AeroManager::isSupportedIceThickness(record.iceThickness))
        {
            return false;
        }
        stepIt->second->m_GallopingIceThickness = record.iceThickness;
        stepIt->second->m_GallopingInitialAttackDegrees =
            std::isfinite(record.initialAttackDegrees) ? record.initialAttackDegrees : 45.0;
        stepIt->second->m_EnableGalloping = stepIt->second->isDynamic && record.enabled != 0;
        if (record.aerodynamicTangentMode < static_cast<int>(SolverNameSpace::AerodynamicTangentMode::Disabled) ||
            record.aerodynamicTangentMode >
                static_cast<int>(SolverNameSpace::AerodynamicTangentMode::EveryNewtonIteration))
        {
            return false;
        }
        stepIt->second->m_GallopingAerodynamicTangentMode =
            static_cast<SolverNameSpace::AerodynamicTangentMode>(record.aerodynamicTangentMode);
    }
    for (const StepTssbnRecord& record : stepTssbn)
    {
        const auto stepIt = restored->m_AnalysisStep.find(record.stepId);
        if (stepIt == restored->m_AnalysisStep.cend() || !stepIt->second ||
            !std::isfinite(record.spectralRadiusInfinity) || record.spectralRadiusInfinity < 0.0 ||
            record.spectralRadiusInfinity > 1.0 || !(record.minimumTimeStep > 0.0) ||
            !(record.maximumTimeStep >= record.minimumTimeStep) || !(record.relativeTolerance > 0.0) ||
            !(record.absoluteTolerance > 0.0) || !(record.safetyFactor > 0.0) || record.safetyFactor > 1.0 ||
            !(record.shrinkFactor > 0.0) || record.shrinkFactor >= 1.0 || record.maximumGrowthFactor < 1.0 ||
            record.targetNewtonIterations < 1 || record.maximumTotalNewtonIterations < 1 ||
            record.derivativeGain < 0.0 || !(record.minimumDerivativeFactor > 0.0) ||
            record.minimumDerivativeFactor > record.maximumDerivativeFactor || record.maximumRejectedAttempts < 1)
        {
            return false;
        }
        auto& settings = stepIt->second->m_AdaptiveTssbn;
        settings.spectralRadiusInfinity = record.spectralRadiusInfinity;
        settings.minimumTimeStep = record.minimumTimeStep;
        settings.maximumTimeStep = record.maximumTimeStep;
        settings.relativeTolerance = record.relativeTolerance;
        settings.absoluteTolerance = record.absoluteTolerance;
        settings.safetyFactor = record.safetyFactor;
        settings.shrinkFactor = record.shrinkFactor;
        settings.maximumGrowthFactor = record.maximumGrowthFactor;
        settings.targetNewtonIterations = record.targetNewtonIterations;
        settings.maximumTotalNewtonIterations = record.maximumTotalNewtonIterations;
        settings.derivativeGain = record.derivativeGain;
        settings.minimumDerivativeFactor = record.minimumDerivativeFactor;
        settings.maximumDerivativeFactor = record.maximumDerivativeFactor;
        settings.maximumRejectedAttempts = record.maximumRejectedAttempts;
    }
    for (const StepStructuralDampingRecord& record : stepStructuralDamping)
    {
        const auto stepIt = restored->m_AnalysisStep.find(record.stepId);
        if (stepIt == restored->m_AnalysisStep.cend() || !stepIt->second ||
            !std::isfinite(record.translationDampingRatio) || record.translationDampingRatio < 0.0 ||
            record.translationDampingRatio > 1.0 || !std::isfinite(record.torsionDampingRatio) ||
            record.torsionDampingRatio < 0.0 || record.torsionDampingRatio > 1.0 ||
            !std::isfinite(record.maximumFrequencyHz) || record.maximumFrequencyHz <= 0.0)
        {
            return false;
        }
        auto& settings = stepIt->second->m_StructuralDamping.settings;
        settings.enabled = stepIt->second->isDynamic && record.enabled != 0;
        settings.translationDampingRatio = record.translationDampingRatio;
        settings.torsionDampingRatio = record.torsionDampingRatio;
        settings.maximumFrequencyHz = record.maximumFrequencyHz;
        stepIt->second->m_StructuralDamping.Reset();
    }
    for (const StepRegionLinkRecord& record : stepRegionLinks)
    {
        const auto stepIt = restored->m_AnalysisStep.find(record.stepId);
        if (stepIt == restored->m_AnalysisStep.cend() || !stepIt->second ||
            restored->m_ComputeRegions.find(record.regionId) == restored->m_ComputeRegions.cend())
        {
            return false;
        }
        stepIt->second->m_ComputeRegionIds.insert(record.regionId);
    }
    if (!restored->m_ComputeRegions.empty())
    {
        QString regionError;
        if (!restored->RebuildAndMergeComputeRegions(&regionError))
        {
            qWarning().noquote() << QStringLiteral("H5计算区域恢复失败：") << regionError;
            return false;
        }
    }

    std::map<int, std::pair<AeroCaseRecord, std::vector<BladeModel>>> restoredAeroCases;
    for (const AeroCaseRecord& record : aeroCases)
    {
        const AeroCaseKey key{record.bundleCount, record.windSpeed, record.iceThickness};
        if (record.id <= 0 || record.modelCount <= 0 || record.dataSize <= 0 || !AeroManager::isSupportedCase(key) ||
            std::abs(record.startAngle) > 1.0e-12 || std::abs(record.angleStep - 5.0) > 1.0e-12)
        {
            return false;
        }
        std::vector<BladeModel> models(static_cast<std::size_t>(record.modelCount));
        for (BladeModel& model : models)
        {
            model.lift.resize(static_cast<std::size_t>(record.dataSize));
            model.drag.resize(static_cast<std::size_t>(record.dataSize));
            model.moment.resize(static_cast<std::size_t>(record.dataSize));
        }
        if (!restoredAeroCases.emplace(record.id, std::make_pair(record, std::move(models))).second)
        {
            return false;
        }
    }
    std::size_t expectedAeroCoefficientCount = 0;
    for (const auto& [caseId, stored] : restoredAeroCases)
    {
        expectedAeroCoefficientCount +=
            static_cast<std::size_t>(stored.first.modelCount) * static_cast<std::size_t>(stored.first.dataSize);
    }
    if (expectedAeroCoefficientCount != aeroCoefficients.size())
        return false;
    for (const AeroCoefficientRecord& coefficient : aeroCoefficients)
    {
        const auto found = restoredAeroCases.find(coefficient.caseId);
        if (found == restoredAeroCases.end() || coefficient.modelIndex < 0 ||
            coefficient.modelIndex >= found->second.first.modelCount || coefficient.angleIndex < 0 ||
            coefficient.angleIndex >= found->second.first.dataSize)
        {
            return false;
        }
        BladeModel& model = found->second.second[static_cast<std::size_t>(coefficient.modelIndex)];
        const auto angleIndex = static_cast<std::size_t>(coefficient.angleIndex);
        model.lift[angleIndex] = coefficient.lift;
        model.drag[angleIndex] = coefficient.drag;
        model.moment[angleIndex] = coefficient.moment;
    }
    for (auto& [caseId, stored] : restoredAeroCases)
    {
        const AeroCaseRecord& record = stored.first;
        const QString sourcePath = ReadUtf8String(record.sourcePath, sizeof(record.sourcePath));
        restored->m_AeroManager.setCaseData(AeroCaseKey{record.bundleCount, record.windSpeed, record.iceThickness},
                                            std::move(stored.second), std::filesystem::path(sourcePath.toStdWString()));
    }

    data->Clear();
    data->m_Nodes = std::move(restored->m_Nodes);
    data->m_Elements = std::move(restored->m_Elements);
    data->m_Material = std::move(restored->m_Material);
    data->m_Section = std::move(restored->m_Section);
    data->m_Property = std::move(restored->m_Property);
    data->m_Constraint = std::move(restored->m_Constraint);
    data->m_MPCConstraints = std::move(restored->m_MPCConstraints);
    data->m_RigidBodyInertias = std::move(restored->m_RigidBodyInertias);
    data->m_Load = std::move(restored->m_Load);
    data->m_AnalysisStep = std::move(restored->m_AnalysisStep);
    data->m_ModelSets = std::move(restored->m_ModelSets);
    data->m_ComputeRegions = std::move(restored->m_ComputeRegions);
    data->m_AeroManager = std::move(restored->m_AeroManager);
    return true;
}

template <typename Record>
bool ReadDatasetBlock(hid_t file, const char* path, hid_t memoryType, long long position, long long length,
                      std::vector<Record>& records)
{
    records.clear();
    if (length <= 0)
    {
        return true;
    }

    H5Handle dataset(H5Dopen2(file, path, H5P_DEFAULT), H5Dclose);
    if (!dataset.valid())
    {
        return false;
    }

    H5Handle fileSpace(H5Dget_space(dataset), H5Sclose);
    if (!fileSpace.valid())
    {
        return false;
    }

    const hsize_t start[1] = {static_cast<hsize_t>(position)};
    const hsize_t count[1] = {static_cast<hsize_t>(length)};
    if (H5Sselect_hyperslab(fileSpace, H5S_SELECT_SET, start, nullptr, count, nullptr) < 0)
    {
        return false;
    }

    H5Handle memSpace(H5Screate_simple(1, count, nullptr), H5Sclose);
    if (!memSpace.valid())
    {
        return false;
    }

    records.resize(static_cast<size_t>(length));
    return H5Dread(dataset, memoryType, memSpace, fileSpace, H5P_DEFAULT, records.data()) >= 0;
}

QString NodeTypeName(EnumKeyword::NodeResultType type)
{
    switch (type)
    {
        case EnumKeyword::NodeResultType::U1:
            return "U1";
        case EnumKeyword::NodeResultType::U2:
            return "U2";
        case EnumKeyword::NodeResultType::U3:
            return "U3";
        case EnumKeyword::NodeResultType::MagnitudeU:
            return "MAG";
        case EnumKeyword::NodeResultType::CX:
            return "CX";
        case EnumKeyword::NodeResultType::CY:
            return "CY";
        case EnumKeyword::NodeResultType::CZ:
            return "CZ";
        case EnumKeyword::NodeResultType::V1:
            return "V1";
        case EnumKeyword::NodeResultType::V2:
            return "V2";
        case EnumKeyword::NodeResultType::V3:
            return "V3";
        case EnumKeyword::NodeResultType::A1:
            return "A1";
        case EnumKeyword::NodeResultType::A2:
            return "A2";
        case EnumKeyword::NodeResultType::A3:
            return "A3";
        case EnumKeyword::NodeResultType::UR1:
            return "UR1";
        case EnumKeyword::NodeResultType::UR2:
            return "UR2";
        case EnumKeyword::NodeResultType::UR3:
            return "UR3";
        case EnumKeyword::NodeResultType::R1:
            return "R1";
        case EnumKeyword::NodeResultType::R2:
            return "R2";
        case EnumKeyword::NodeResultType::R3:
            return "R3";
        default:
            return "UNKNOWN";
    }
}

QString ElementTypeName(EnumKeyword::ElementResultType type)
{
    switch (type)
    {
        case EnumKeyword::ElementResultType::AxialForce:
            return "AXIAL";
        case EnumKeyword::ElementResultType::ShearY:
            return "SHEARY";
        case EnumKeyword::ElementResultType::ShearZ:
            return "SHEARZ";
        case EnumKeyword::ElementResultType::Torque:
            return "TORQUE";
        case EnumKeyword::ElementResultType::MomentY:
            return "MY";
        case EnumKeyword::ElementResultType::MomentZ:
            return "MZ";
        case EnumKeyword::ElementResultType::Strain:
            return "STRAIN";
        case EnumKeyword::ElementResultType::InitStress:
            return "S0";
        case EnumKeyword::ElementResultType::CurrentStress:
            return "S";
        case EnumKeyword::ElementResultType::DeltaStress:
            return "DS";
        default:
            return "UNKNOWN";
    }
}

QString FormatBdfValue(double value)
{
    return QString::number(value, 'E', 8).rightJustified(16, ' ');
}

QString FormatBdfTime(double time)
{
    QString value = QString::number(time, 'f', 12);
    while (value.contains('.') && value.endsWith('0'))
        value.chop(1);
    if (value.endsWith('.'))
        value.chop(1);
    return value.rightJustified(16, ' ');
}

double FindNodalValue(int nodeId, EnumKeyword::NodeResultType type, const QHash<int, NodalRecord>& displacements,
                      const QHash<int, NodalRecord>& currentCoordinates, const QHash<int, NodalRecord>& velocities,
                      const QHash<int, NodalRecord>& accelerations, const QHash<int, NodalRecord>& reactions)
{
    const NodalRecord zero;
    switch (type)
    {
        case EnumKeyword::NodeResultType::U1:
            return displacements.value(nodeId, zero).x;
        case EnumKeyword::NodeResultType::U2:
            return displacements.value(nodeId, zero).y;
        case EnumKeyword::NodeResultType::U3:
            return displacements.value(nodeId, zero).z;
        case EnumKeyword::NodeResultType::MagnitudeU:
        {
            const auto record = displacements.value(nodeId, zero);
            return std::sqrt(record.x * record.x + record.y * record.y + record.z * record.z);
        }
        case EnumKeyword::NodeResultType::CX:
            return currentCoordinates.value(nodeId, zero).x;
        case EnumKeyword::NodeResultType::CY:
            return currentCoordinates.value(nodeId, zero).y;
        case EnumKeyword::NodeResultType::CZ:
            return currentCoordinates.value(nodeId, zero).z;
        case EnumKeyword::NodeResultType::V1:
            return velocities.value(nodeId, zero).x;
        case EnumKeyword::NodeResultType::V2:
            return velocities.value(nodeId, zero).y;
        case EnumKeyword::NodeResultType::V3:
            return velocities.value(nodeId, zero).z;
        case EnumKeyword::NodeResultType::A1:
            return accelerations.value(nodeId, zero).x;
        case EnumKeyword::NodeResultType::A2:
            return accelerations.value(nodeId, zero).y;
        case EnumKeyword::NodeResultType::A3:
            return accelerations.value(nodeId, zero).z;
        case EnumKeyword::NodeResultType::UR1:
            return displacements.value(nodeId, zero).rx;
        case EnumKeyword::NodeResultType::UR2:
            return displacements.value(nodeId, zero).ry;
        case EnumKeyword::NodeResultType::UR3:
            return displacements.value(nodeId, zero).rz;
        case EnumKeyword::NodeResultType::R1:
            return reactions.value(nodeId, zero).x;
        case EnumKeyword::NodeResultType::R2:
            return reactions.value(nodeId, zero).y;
        case EnumKeyword::NodeResultType::R3:
            return reactions.value(nodeId, zero).z;
        default:
            return 0.0;
    }
}

double FindElementValue(int elementId, EnumKeyword::ElementResultType type,
                        const QHash<int, ElementForceRecord>& forces, const QHash<int, ElementStressRecord>& stresses,
                        const QHash<int, ElementStrainRecord>& strains)
{
    const ElementForceRecord zeroForce;
    const ElementStressRecord zeroStress;
    const ElementStrainRecord zeroStrain;

    switch (type)
    {
        case EnumKeyword::ElementResultType::AxialForce:
            return forces.value(elementId, zeroForce).axial;
        case EnumKeyword::ElementResultType::ShearY:
            return forces.value(elementId, zeroForce).shearY;
        case EnumKeyword::ElementResultType::ShearZ:
            return forces.value(elementId, zeroForce).shearZ;
        case EnumKeyword::ElementResultType::Torque:
            return forces.value(elementId, zeroForce).torque;
        case EnumKeyword::ElementResultType::MomentY:
            return forces.value(elementId, zeroForce).momentY;
        case EnumKeyword::ElementResultType::MomentZ:
            return forces.value(elementId, zeroForce).momentZ;
        case EnumKeyword::ElementResultType::Strain:
            return strains.value(elementId, zeroStrain).strain;
        case EnumKeyword::ElementResultType::InitStress:
            return stresses.value(elementId, zeroStress).initStress;
        case EnumKeyword::ElementResultType::CurrentStress:
            return stresses.value(elementId, zeroStress).currentStress;
        case EnumKeyword::ElementResultType::DeltaStress:
            return stresses.value(elementId, zeroStress).deltaStress;
        default:
            return 0.0;
    }
}

template <typename Record> QHash<int, Record> BuildRecordMap(const std::vector<Record>& records)
{
    QHash<int, Record> result;
    for (const auto& record : records)
    {
        result.insert(record.id, record);
    }
    return result;
}

QHash<int, IndexRecord> BuildIndexMap(const std::vector<IndexRecord>& records)
{
    QHash<int, IndexRecord> result;
    for (const auto& record : records)
    {
        result.insert(record.domainId, record);
    }
    return result;
}
}

bool Hdf5ModelIO::ExportBdfResultFromHdf5(const QString& hdf5FileName, const QString& bdfFileName,
                                          const std::vector<int>& nodeIds,
                                          const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
                                          const std::vector<int>& elementIds,
                                          const std::vector<EnumKeyword::ElementResultType>& elementTypes) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    H5ReadErrorScope errorScope;
    const QString tempHdf5FileName = CopyHdf5FileToAsciiTemp(QFileInfo(hdf5FileName).absoluteFilePath());
    if (tempHdf5FileName.isEmpty())
        return ReportHdf5FormatError();

    const QByteArray path = ToHdf5Path(tempHdf5FileName);
    H5Handle file(H5Fopen(path.constData(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose);
    if (!file.valid())
    {
        QFile::remove(tempHdf5FileName);
        return ReportHdf5FormatError();
    }
    if (!HasRequiredResultSchema(file))
    {
        file.reset();
        QFile::remove(tempHdf5FileName);
        return ReportHdf5FormatError();
    }
    QSaveFile outFile(bdfFileName);
    if (!outFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open BDF result file:" << bdfFileName;
        return false;
    }

    H5Handle domainType = CreateDomainType();
    H5Handle indexType = CreateIndexType();
    H5Handle nodalType = CreateNodalType();
    H5Handle elementForceType = CreateElementForceType();
    H5Handle trussForceType = CreateTrussForceType();
    H5Handle cableForceType = CreateCableForceType();
    H5Handle elementStressType = CreateElementStressType();
    H5Handle elementStrainType = CreateElementStrainType();
    if (!domainType.valid() || !indexType.valid() || !nodalType.valid() || !elementForceType.valid() ||
        !trussForceType.valid() || !cableForceType.valid() || !elementStressType.valid() || !elementStrainType.valid())
    {
        return ReportHdf5FormatError();
    }

    std::vector<DomainRecord> domains;
    std::vector<IndexRecord> displacementIndexRecords;
    std::vector<IndexRecord> currentCoordinateIndexRecords;
    std::vector<IndexRecord> velocityIndexRecords;
    std::vector<IndexRecord> accelerationIndexRecords;
    std::vector<IndexRecord> reactionIndexRecords;
    std::vector<IndexRecord> elementForceIndexRecords;
    std::vector<IndexRecord> trussForceIndexRecords;
    std::vector<IndexRecord> cableForceIndexRecords;
    std::vector<IndexRecord> elementStressIndexRecords;
    std::vector<IndexRecord> elementStrainIndexRecords;

    if (!ReadDatasetAll(file, "/YQY/RESULT/DOMAINS", domainType, domains) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType, displacementIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType, currentCoordinateIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType, velocityIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType, accelerationIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType, reactionIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType, elementForceIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", indexType, trussForceIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE", indexType, cableForceIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType, elementStressIndexRecords) ||
        !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType, elementStrainIndexRecords))
    {
        return ReportHdf5FormatError();
    }

    std::sort(domains.begin(), domains.end(),
              [](const DomainRecord& lhs, const DomainRecord& rhs)
              {
                  return lhs.id < rhs.id;
              });

    const QHash<int, IndexRecord> displacementIndex = BuildIndexMap(displacementIndexRecords);
    const QHash<int, IndexRecord> currentCoordinateIndex = BuildIndexMap(currentCoordinateIndexRecords);
    const QHash<int, IndexRecord> velocityIndex = BuildIndexMap(velocityIndexRecords);
    const QHash<int, IndexRecord> accelerationIndex = BuildIndexMap(accelerationIndexRecords);
    const QHash<int, IndexRecord> reactionIndex = BuildIndexMap(reactionIndexRecords);
    const QHash<int, IndexRecord> elementForceIndex = BuildIndexMap(elementForceIndexRecords);
    const QHash<int, IndexRecord> trussForceIndex = BuildIndexMap(trussForceIndexRecords);
    const QHash<int, IndexRecord> cableForceIndex = BuildIndexMap(cableForceIndexRecords);
    const QHash<int, IndexRecord> elementStressIndex = BuildIndexMap(elementStressIndexRecords);
    const QHash<int, IndexRecord> elementStrainIndex = BuildIndexMap(elementStrainIndexRecords);

    const auto hasNodeType = [&nodeTypes](std::initializer_list<EnumKeyword::NodeResultType> candidates)
    {
        return std::any_of(nodeTypes.cbegin(), nodeTypes.cend(),
                           [&candidates](auto value)
                           {
                               return std::find(candidates.begin(), candidates.end(), value) != candidates.end();
                           });
    };
    const auto hasElementType = [&elementTypes](std::initializer_list<EnumKeyword::ElementResultType> candidates)
    {
        return std::any_of(elementTypes.cbegin(), elementTypes.cend(),
                           [&candidates](auto value)
                           {
                               return std::find(candidates.begin(), candidates.end(), value) != candidates.end();
                           });
    };
    const bool needDisplacement =
        !nodeIds.empty() &&
        hasNodeType({EnumKeyword::NodeResultType::U1, EnumKeyword::NodeResultType::U2, EnumKeyword::NodeResultType::U3,
                     EnumKeyword::NodeResultType::MagnitudeU, EnumKeyword::NodeResultType::UR1,
                     EnumKeyword::NodeResultType::UR2, EnumKeyword::NodeResultType::UR3});
    const bool needCurrentCoordinate =
        !nodeIds.empty() && hasNodeType({EnumKeyword::NodeResultType::CX, EnumKeyword::NodeResultType::CY,
                                         EnumKeyword::NodeResultType::CZ});
    const bool needVelocity =
        !nodeIds.empty() && hasNodeType({EnumKeyword::NodeResultType::V1, EnumKeyword::NodeResultType::V2,
                                         EnumKeyword::NodeResultType::V3});
    const bool needAcceleration =
        !nodeIds.empty() && hasNodeType({EnumKeyword::NodeResultType::A1, EnumKeyword::NodeResultType::A2,
                                         EnumKeyword::NodeResultType::A3});
    const bool needReaction =
        !nodeIds.empty() && hasNodeType({EnumKeyword::NodeResultType::R1, EnumKeyword::NodeResultType::R2,
                                         EnumKeyword::NodeResultType::R3});
    const bool needElementForce =
        !elementIds.empty() &&
        hasElementType({EnumKeyword::ElementResultType::AxialForce, EnumKeyword::ElementResultType::ShearY,
                        EnumKeyword::ElementResultType::ShearZ, EnumKeyword::ElementResultType::Torque,
                        EnumKeyword::ElementResultType::MomentY, EnumKeyword::ElementResultType::MomentZ});
    const bool needElementStress = !elementIds.empty() && hasElementType({EnumKeyword::ElementResultType::InitStress,
                                                                          EnumKeyword::ElementResultType::CurrentStress,
                                                                          EnumKeyword::ElementResultType::DeltaStress});
    const bool needElementStrain = !elementIds.empty() && hasElementType({EnumKeyword::ElementResultType::Strain});

    QTextStream stream(&outFile);
    stream.setEncoding(QStringConverter::Utf8);
    stream << QString("TIME").rightJustified(16, ' ');
    for (int nodeId : nodeIds)
    {
        for (auto type : nodeTypes)
        {
            stream << QString("N%1-%2").arg(nodeId).arg(NodeTypeName(type)).right(16).rightJustified(16, ' ');
        }
    }
    for (int elementId : elementIds)
    {
        for (auto type : elementTypes)
        {
            stream << QString("E%1-%2").arg(elementId).arg(ElementTypeName(type)).right(16).rightJustified(16, ' ');
        }
    }
    stream << "\n";

    for (const DomainRecord& domain : domains)
    {
        std::vector<NodalRecord> displacementRecords;
        std::vector<NodalRecord> currentCoordinateRecords;
        std::vector<NodalRecord> velocityRecords;
        std::vector<NodalRecord> accelerationRecords;
        std::vector<NodalRecord> reactionRecords;
        std::vector<ElementForceRecord> forceRecords;
        std::vector<TrussForceRecord> trussForceRecords;
        std::vector<CableForceRecord> cableForceRecords;
        std::vector<ElementStressRecord> stressRecords;
        std::vector<ElementStrainRecord> strainRecords;

        const IndexRecord displacementInfo = displacementIndex.value(domain.id);
        const IndexRecord currentCoordinateInfo = currentCoordinateIndex.value(domain.id);
        const IndexRecord velocityInfo = velocityIndex.value(domain.id);
        const IndexRecord accelerationInfo = accelerationIndex.value(domain.id);
        const IndexRecord reactionInfo = reactionIndex.value(domain.id);
        const IndexRecord forceInfo = elementForceIndex.value(domain.id);
        const IndexRecord trussForceInfo = trussForceIndex.value(domain.id);
        const IndexRecord cableForceInfo = cableForceIndex.value(domain.id);
        const IndexRecord stressInfo = elementStressIndex.value(domain.id);
        const IndexRecord strainInfo = elementStrainIndex.value(domain.id);

        if ((needDisplacement &&
             !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, displacementInfo.position,
                               displacementInfo.length, displacementRecords)) ||
            (needCurrentCoordinate &&
             !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType, currentCoordinateInfo.position,
                               currentCoordinateInfo.length, currentCoordinateRecords)) ||
            (needVelocity && !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType, velocityInfo.position,
                                               velocityInfo.length, velocityRecords)) ||
            (needAcceleration &&
             !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType, accelerationInfo.position,
                               accelerationInfo.length, accelerationRecords)) ||
            (needReaction && !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType,
                                               reactionInfo.position, reactionInfo.length, reactionRecords)) ||
            (needElementForce && !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType,
                                                   forceInfo.position, forceInfo.length, forceRecords)) ||
            (needElementForce &&
             !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", trussForceType, trussForceInfo.position,
                               trussForceInfo.length, trussForceRecords)) ||
            (needElementForce &&
             !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE", cableForceType, cableForceInfo.position,
                               cableForceInfo.length, cableForceRecords)) ||
            (needElementStress && !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType,
                                                    stressInfo.position, stressInfo.length, stressRecords)) ||
            (needElementStrain && !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType,
                                                    strainInfo.position, strainInfo.length, strainRecords)))
        {
            return ReportHdf5FormatError();
        }

        const QHash<int, NodalRecord> displacements = BuildRecordMap(displacementRecords);
        const QHash<int, NodalRecord> currentCoordinates = BuildRecordMap(currentCoordinateRecords);
        const QHash<int, NodalRecord> velocities = BuildRecordMap(velocityRecords);
        const QHash<int, NodalRecord> accelerations = BuildRecordMap(accelerationRecords);
        const QHash<int, NodalRecord> reactions = BuildRecordMap(reactionRecords);
        QHash<int, ElementForceRecord> forces = BuildRecordMap(forceRecords);
        for (const TrussForceRecord& value : trussForceRecords)
        {
            ElementForceRecord force;
            force.id = value.id;
            force.domainId = value.domainId;
            force.axial = value.axial;
            forces.insert(force.id, force);
        }
        for (const CableForceRecord& value : cableForceRecords)
        {
            ElementForceRecord force;
            force.id = value.id;
            force.domainId = value.domainId;
            force.axial = value.axial;
            force.torque = value.torque;
            forces.insert(force.id, force);
        }
        const QHash<int, ElementStressRecord> stresses = BuildRecordMap(stressRecords);
        const QHash<int, ElementStrainRecord> strains = BuildRecordMap(strainRecords);

        stream << FormatBdfTime(domain.time);
        for (int nodeId : nodeIds)
        {
            for (auto type : nodeTypes)
            {
                stream << FormatBdfValue(FindNodalValue(nodeId, type, displacements, currentCoordinates, velocities,
                                                        accelerations, reactions));
            }
        }
        for (int elementId : elementIds)
        {
            for (auto type : elementTypes)
            {
                stream << FormatBdfValue(FindElementValue(elementId, type, forces, stresses, strains));
            }
        }
        stream << "\n";
    }

    stream.flush();
    if (stream.status() != QTextStream::Ok || !outFile.commit())
    {
        file.reset();
        QFile::remove(tempHdf5FileName);
        return false;
    }
    qDebug().noquote() << QStringLiteral("H5/HDF5 结果已转换输出至") << bdfFileName;
    file.reset();
    QFile::remove(tempHdf5FileName);
    return true;
}

bool Hdf5ModelIO::ImportHdf5(const QString& fileName, StructureData* pData) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    H5ReadErrorScope errorScope;
    if (fileName.isEmpty() || !pData)
    {
        return false;
    }

    const QString tempHdf5FileName = CopyHdf5FileToAsciiTemp(QFileInfo(fileName).absoluteFilePath());
    if (tempHdf5FileName.isEmpty())
        return ReportHdf5FormatError();

    const QByteArray path = ToHdf5Path(tempHdf5FileName);
    H5Handle file(H5Fopen(path.constData(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose);
    if (!file.valid())
    {
        QFile::remove(tempHdf5FileName);
        return ReportHdf5FormatError();
    }
    if (!HasRequiredInputSchema(file))
    {
        file.reset();
        QFile::remove(tempHdf5FileName);
        return ReportHdf5FormatError();
    }
    const bool restored = ReadInputData(file, pData);
    file.reset();
    QFile::remove(tempHdf5FileName);
    return restored || ReportHdf5FormatError();
}

bool Hdf5ModelIO::InspectHdf5(const QString& fileName, Hdf5ResultSummary& summary) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    H5ReadErrorScope errorScope;
    summary = {};
    if (fileName.isEmpty())
        return false;

    const QString tempFileName = CopyHdf5FileToAsciiTemp(QFileInfo(fileName).absoluteFilePath());
    if (tempFileName.isEmpty())
        return ReportHdf5FormatError();

    const QByteArray path = ToHdf5Path(tempFileName);
    H5Handle file(H5Fopen(path.constData(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose);
    if (!file.valid())
    {
        QFile::remove(tempFileName);
        return ReportHdf5FormatError();
    }
    const bool hasModelSchema = HasRequiredInputSchema(file);
    const bool hasResultSchema = HasRequiredResultSchema(file);
    if (!hasModelSchema && !hasResultSchema)
    {
        file.reset();
        QFile::remove(tempFileName);
        return ReportHdf5FormatError();
    }
    const auto datasetLength = [](hid_t fileId, const char* datasetPath) -> qint64
    {
        H5Handle dataset(H5Dopen2(fileId, datasetPath, H5P_DEFAULT), H5Dclose);
        if (!dataset.valid())
            return 0;
        H5Handle space(H5Dget_space(dataset), H5Sclose);
        if (!space.valid())
            return 0;
        hsize_t dimensions[1] = {0};
        return H5Sget_simple_extent_dims(space, dimensions, nullptr) >= 0 ? static_cast<qint64>(dimensions[0]) : 0;
    };

    summary.hasModel = hasModelSchema;
    summary.hasResult = hasResultSchema;
    summary.frameCount = datasetLength(file, "/YQY/RESULT/DOMAINS");
    summary.displacementRecordCount = datasetLength(file, "/YQY/RESULT/NODAL/DISPLACEMENT");
    summary.stressRecordCount = datasetLength(file, "/YQY/RESULT/ELEMENTAL/STRESS");
    summary.strainRecordCount = datasetLength(file, "/YQY/RESULT/ELEMENTAL/STRAIN");
    int resultComplete = 1;
    if (ReadIntAttribute(file, "RESULT_COMPLETE", resultComplete))
        summary.partialResult = summary.frameCount > 0 && resultComplete == 0;

    file.reset();
    QFile::remove(tempFileName);
    return summary.hasModel || summary.hasResult;
}

bool Hdf5ModelIO::OpenResultFile(const QString& fileName, std::vector<Hdf5ResultFrameInfo>& frames)
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    H5ReadErrorScope errorScope;
    frames.clear();
    m_impl->CloseResult();
    if (fileName.trimmed().isEmpty())
        return false;

    m_impl->resultTempFileName = CopyHdf5FileToAsciiTemp(QFileInfo(fileName).absoluteFilePath());
    if (m_impl->resultTempFileName.isEmpty())
        return ReportHdf5FormatError();

    const QByteArray path = ToHdf5Path(m_impl->resultTempFileName);
    m_impl->resultFile.reset(H5Fopen(path.constData(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose);
    if (!m_impl->resultFile.valid())
    {
        m_impl->CloseResult();
        return ReportHdf5FormatError();
    }
    if (!HasRequiredResultSchema(m_impl->resultFile))
    {
        m_impl->CloseResult();
        return ReportHdf5FormatError();
    }
    H5Handle domainType = CreateDomainType();
    H5Handle indexType = CreateIndexType();
    std::vector<IndexRecord> displacement;
    std::vector<IndexRecord> currentCoordinates;
    std::vector<IndexRecord> velocities;
    std::vector<IndexRecord> accelerations;
    std::vector<IndexRecord> forces;
    std::vector<IndexRecord> trussForces;
    std::vector<IndexRecord> cableForces;
    std::vector<IndexRecord> stresses;
    std::vector<IndexRecord> strains;
    const bool ok =
        domainType.valid() && indexType.valid() &&
        ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/DOMAINS", domainType, m_impl->resultDomains) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType, displacement) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType,
                       currentCoordinates) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType, velocities) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType, accelerations) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType, forces) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", indexType, trussForces) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE", indexType, cableForces) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType, stresses) &&
        ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType, strains);
    if (!ok || m_impl->resultDomains.empty())
    {
        m_impl->CloseResult();
        return ReportHdf5FormatError();
    }

    std::sort(m_impl->resultDomains.begin(), m_impl->resultDomains.end(),
              [](const DomainRecord& lhs, const DomainRecord& rhs)
              {
                  return lhs.id < rhs.id;
              });
    m_impl->displacementIndex = BuildIndexMap(displacement);
    m_impl->currentCoordinateIndex = BuildIndexMap(currentCoordinates);
    m_impl->velocityIndex = BuildIndexMap(velocities);
    m_impl->accelerationIndex = BuildIndexMap(accelerations);
    m_impl->elementForceIndex = BuildIndexMap(forces);
    m_impl->trussForceIndex = BuildIndexMap(trussForces);
    m_impl->cableForceIndex = BuildIndexMap(cableForces);
    m_impl->stressIndex = BuildIndexMap(stresses);
    m_impl->strainIndex = BuildIndexMap(strains);
    m_impl->resultSourceFileName = QFileInfo(fileName).absoluteFilePath();

    frames.reserve(m_impl->resultDomains.size());
    for (const DomainRecord& domain : m_impl->resultDomains)
    {
        Hdf5ResultFrameInfo info;
        info.domainId = domain.id;
        info.stepId = domain.stepId;
        info.increment = domain.increment;
        info.analysis = domain.analysis;
        info.time = domain.time;
        info.loadFactor = domain.loadFactor;
        frames.push_back(info);
    }
    return true;
}

bool Hdf5ModelIO::ReadResultRanges(Hdf5ResultRanges& ranges) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    H5ReadErrorScope errorScope;
    ranges = {};
    if (!m_impl->resultFile.valid())
        return false;

    const auto readCachedRange = [this](const char* prefix, Hdf5ResultRange& range)
    {
        const QByteArray validName = QByteArray("RESULT_RANGE_") + prefix + "_VALID";
        const QByteArray minimumName = QByteArray("RESULT_RANGE_") + prefix + "_MIN";
        const QByteArray maximumName = QByteArray("RESULT_RANGE_") + prefix + "_MAX";
        int valid = 0;
        if (!ReadIntAttribute(m_impl->resultFile, validName.constData(), valid))
            return false;
        range.valid = valid != 0;
        return !range.valid || (ReadDoubleAttribute(m_impl->resultFile, minimumName.constData(), range.minimum) &&
                                ReadDoubleAttribute(m_impl->resultFile, maximumName.constData(), range.maximum));
    };
    const bool readOk = readCachedRange("DISPLACEMENT_MAGNITUDE", ranges.displacementMagnitude) &&
                        readCachedRange("DISPLACEMENT_X", ranges.displacementX) &&
                        readCachedRange("DISPLACEMENT_Y", ranges.displacementY) &&
                        readCachedRange("DISPLACEMENT_Z", ranges.displacementZ) &&
                        readCachedRange("AXIAL_FORCE", ranges.axialForce) && readCachedRange("STRESS", ranges.stress) &&
                        readCachedRange("STRAIN", ranges.strain);
    return readOk &&
           (ranges.displacementMagnitude.valid || ranges.displacementX.valid || ranges.displacementY.valid ||
            ranges.displacementZ.valid || ranges.axialForce.valid || ranges.stress.valid || ranges.strain.valid);
}

bool Hdf5ModelIO::ReadResultFrame(int frameIndex, Hdf5ResultFrame& frame) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    H5ReadErrorScope errorScope;
    frame = {};
    if (!m_impl->resultFile.valid() || frameIndex < 0 || frameIndex >= static_cast<int>(m_impl->resultDomains.size()))
        return false;

    const DomainRecord& domain = m_impl->resultDomains[static_cast<std::size_t>(frameIndex)];
    const IndexRecord displacementInfo = m_impl->displacementIndex.value(domain.id);
    const IndexRecord coordinateInfo = m_impl->currentCoordinateIndex.value(domain.id);
    const IndexRecord velocityInfo = m_impl->velocityIndex.value(domain.id);
    const IndexRecord accelerationInfo = m_impl->accelerationIndex.value(domain.id);
    const IndexRecord forceInfo = m_impl->elementForceIndex.value(domain.id);
    const IndexRecord trussForceInfo = m_impl->trussForceIndex.value(domain.id);
    const IndexRecord cableForceInfo = m_impl->cableForceIndex.value(domain.id);
    const IndexRecord stressInfo = m_impl->stressIndex.value(domain.id);
    const IndexRecord strainInfo = m_impl->strainIndex.value(domain.id);

    H5Handle nodalType = CreateNodalType();
    H5Handle forceType = CreateElementForceType();
    H5Handle trussForceType = CreateTrussForceType();
    H5Handle cableForceType = CreateCableForceType();
    H5Handle stressType = CreateElementStressType();
    H5Handle strainType = CreateElementStrainType();
    std::vector<NodalRecord> displacements;
    std::vector<NodalRecord> coordinates;
    std::vector<NodalRecord> velocities;
    std::vector<NodalRecord> accelerations;
    std::vector<ElementForceRecord> forces;
    std::vector<TrussForceRecord> trussForces;
    std::vector<CableForceRecord> cableForces;
    std::vector<ElementStressRecord> stresses;
    std::vector<ElementStrainRecord> strains;
    if (!nodalType.valid() || !forceType.valid() || !trussForceType.valid() || !cableForceType.valid() ||
        !stressType.valid() || !strainType.valid() ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, displacementInfo.position,
                          displacementInfo.length, displacements) ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType,
                          coordinateInfo.position, coordinateInfo.length, coordinates) ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/NODAL/VELOCITY", nodalType, velocityInfo.position,
                          velocityInfo.length, velocities) ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/NODAL/ACCELERATION", nodalType, accelerationInfo.position,
                          accelerationInfo.length, accelerations) ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", forceType, forceInfo.position,
                          forceInfo.length, forces) ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", trussForceType,
                          trussForceInfo.position, trussForceInfo.length, trussForces) ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE", cableForceType,
                          cableForceInfo.position, cableForceInfo.length, cableForces) ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/STRESS", stressType, stressInfo.position,
                          stressInfo.length, stresses) ||
        !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/STRAIN", strainType, strainInfo.position,
                          strainInfo.length, strains))
        return ReportHdf5FormatError();

    frame.info = {domain.id, domain.stepId, domain.increment, domain.analysis, domain.time, domain.loadFactor};
    const QHash<int, NodalRecord> coordinateMap = BuildRecordMap(coordinates);
    const QHash<int, NodalRecord> velocityMap = BuildRecordMap(velocities);
    const QHash<int, NodalRecord> accelerationMap = BuildRecordMap(accelerations);
    frame.nodes.reserve(displacements.size());
    for (const NodalRecord& value : displacements)
    {
        Hdf5NodalResult result;
        result.id = value.id;
        result.displacement[0] = value.x;
        result.displacement[1] = value.y;
        result.displacement[2] = value.z;
        result.displacement[3] = value.rx;
        result.displacement[4] = value.ry;
        result.displacement[5] = value.rz;
        const NodalRecord coordinate = coordinateMap.value(value.id);
        result.currentCoordinate[0] = coordinate.x;
        result.currentCoordinate[1] = coordinate.y;
        result.currentCoordinate[2] = coordinate.z;
        const NodalRecord velocity = velocityMap.value(value.id);
        result.velocity[0] = velocity.x;
        result.velocity[1] = velocity.y;
        result.velocity[2] = velocity.z;
        result.velocity[3] = velocity.rx;
        result.velocity[4] = velocity.ry;
        result.velocity[5] = velocity.rz;
        const NodalRecord acceleration = accelerationMap.value(value.id);
        result.acceleration[0] = acceleration.x;
        result.acceleration[1] = acceleration.y;
        result.acceleration[2] = acceleration.z;
        result.acceleration[3] = acceleration.rx;
        result.acceleration[4] = acceleration.ry;
        result.acceleration[5] = acceleration.rz;
        frame.nodes.push_back(result);
    }

    QHash<int, Hdf5ElementResult> elementMap;
    for (const ElementForceRecord& value : forces)
    {
        auto& result = elementMap[value.id];
        result.id = value.id;
        result.axialForce = value.axial;
        result.shearY = value.shearY;
        result.shearZ = value.shearZ;
        result.torque = value.torque;
        result.momentY = value.momentY;
        result.momentZ = value.momentZ;
    }
    for (const TrussForceRecord& value : trussForces)
    {
        auto& result = elementMap[value.id];
        result.id = value.id;
        result.axialForce = value.axial;
    }
    for (const CableForceRecord& value : cableForces)
    {
        auto& result = elementMap[value.id];
        result.id = value.id;
        result.axialForce = value.axial;
        result.torque = value.torque;
    }
    for (const ElementStressRecord& value : stresses)
    {
        auto& result = elementMap[value.id];
        result.id = value.id;
        result.initStress = value.initStress;
        result.currentStress = value.currentStress;
        result.deltaStress = value.deltaStress;
    }
    for (const ElementStrainRecord& value : strains)
    {
        auto& result = elementMap[value.id];
        result.id = value.id;
        result.strain = value.strain;
    }
    frame.elements.reserve(static_cast<std::size_t>(elementMap.size()));
    for (auto it = elementMap.cbegin(); it != elementMap.cend(); ++it)
        frame.elements.push_back(it.value());
    return true;
}

bool Hdf5ModelIO::RestoreLastDynamicState(const QString& fileName, StructureData* pData, double* time, int* stepId)
{
    if (!pData)
        return false;

    std::vector<Hdf5ResultFrameInfo> frames;
    if (!OpenResultFile(fileName, frames))
        return false;

    int dynamicFrameIndex = -1;
    for (int index = static_cast<int>(frames.size()) - 1; index >= 0; --index)
    {
        if (frames[static_cast<std::size_t>(index)].analysis == static_cast<int>(EnumKeyword::StepType::DYNAMIC))
        {
            dynamicFrameIndex = index;
            break;
        }
    }
    if (dynamicFrameIndex < 0)
        return false;

    Hdf5ResultFrame frame;
    if (!ReadResultFrame(dynamicFrameIndex, frame))
        return false;

    int restoredCount = 0;
    for (const Hdf5NodalResult& result : frame.nodes)
    {
        const auto nodeIt = pData->m_Nodes.find(result.id);
        if (nodeIt == pData->m_Nodes.end() || !nodeIt->second)
            continue;

        const auto& node = nodeIt->second;
        const int dofCount = std::max(3, static_cast<int>(node->m_DOF.size()));
        node->m_Displacement.assign(dofCount, 0.0);
        node->m_Velocity.assign(dofCount, 0.0);
        node->m_Acceleration.assign(dofCount, 0.0);
        for (int component = 0; component < dofCount && component < 6; ++component)
        {
            node->m_Displacement[component] = result.displacement[component];
            node->m_Velocity[component] = result.velocity[component];
            node->m_Acceleration[component] = result.acceleration[component];
        }

        if (dofCount >= 6)
        {
            const Eigen::Vector3d rotation(result.displacement[3], result.displacement[4], result.displacement[5]);
            Utility::CR::Calculate_RotationMatrix(rotation, node->m_Rg);
            const Eigen::Vector3d spatialOmega(result.velocity[3], result.velocity[4], result.velocity[5]);
            const Eigen::Vector3d spatialAlpha(result.acceleration[3], result.acceleration[4], result.acceleration[5]);
            node->m_OmegaMaterial = node->m_Rg.transpose() * spatialOmega;
            node->m_AlphaMaterial = node->m_Rg.transpose() * spatialAlpha;
        }

        node->m_Displacement_n = node->m_Displacement;
        node->m_Velocity_n = node->m_Velocity;
        node->m_Acceleration_n = node->m_Acceleration;
        node->m_Rg_n = node->m_Rg;
        node->m_OmegaMaterial_n = node->m_OmegaMaterial;
        node->m_AlphaMaterial_n = node->m_AlphaMaterial;
        node->m_StepRotation.setZero();
        ++restoredCount;
    }

    if (restoredCount != static_cast<int>(pData->m_Nodes.size()))
        return false;
    if (time)
        *time = frame.info.time;
    if (stepId)
        *stepId = frame.info.stepId;
    return true;
}

bool Hdf5ModelIO::ReadSolverIterationHistory(std::vector<SolverIterationRecord>& records) const
{
    records.clear();
    if (!m_impl->resultFile.valid())
        return false;
    H5Handle type = CreateSolverIterationType();
    std::vector<SolverIterationH5Record> h5Records;
    if (!type.valid() || !ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/SOLVER_ITERATION", type, h5Records))
    {
        return false;
    }
    records.reserve(h5Records.size());
    for (const SolverIterationH5Record& record : h5Records)
        records.push_back({record.stepId, record.analysisType, record.time, record.iterations});
    return true;
}

void Hdf5ModelIO::CloseResultFile()
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    m_impl->CloseResult();
}
