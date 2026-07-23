#include "Hdf5ModelIO.h"

#include "DataStructure/Structure/StructureData.h"

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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <vector>

namespace
{
// VTK bundled HDF5 is built without H5_HAVE_THREADSAFE.  Solver jobs may run
// concurrently, so every entry into the HDF5 C API must be serialized.
QRecursiveMutex g_hdf5ApiMutex;

constexpr int kModelDomainId = 1;
constexpr int kSchemaVersion = 3;
constexpr size_t kEntityNameSize = 256;

QString MakeAsciiTempHdf5FileName()
{
    return QDir::temp().filePath("yqy_h5_" + QUuid::createUuid().toString(QUuid::Id128) + ".h5");
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
    static constexpr const char* requiredPaths[] = {
        "/YQY/INPUT/NODE/GRID",
        "/YQY/INPUT/MATERIAL/MAT",
        "/YQY/INPUT/SECTION/SECTION",
        "/YQY/INPUT/PROPERTY/PROPERTY",
        "/YQY/INPUT/ELEMENT/ELEMENT",
        "/YQY/INPUT/CONSTRAINT/SPC",
        "/YQY/INPUT/LOAD/LOAD",
        "/YQY/INPUT/ANALYSIS_STEP/STEP"
    };

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
    static constexpr const char* requiredPaths[] = {
        "/YQY/RESULT/DOMAINS",
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
        "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN"
    };

    for (const char* path : requiredPaths)
    {
        if (LinkExistsQuietly(file, path))
            continue;
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
    herr_t(*closeFunc)(hid_t) = nullptr;

    H5Handle() = default;
        H5Handle(hid_t value, herr_t(*func)(hid_t)) : id(value), closeFunc(func)
        {
        }
        ~H5Handle()
        {
            reset();
        }

    H5Handle(const H5Handle&) = delete;
    H5Handle& operator=(const H5Handle&) = delete;

    H5Handle(H5Handle&& other) noexcept
        : id(other.id), closeFunc(other.closeFunc)
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

    void reset(hid_t value = -1, herr_t(*func)(hid_t) = nullptr)
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
    char name[kEntityNameSize] = {};
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

// Axial-only truss results deliberately use a compact record.  Keeping these
// separate from beam forces avoids storing five permanently-zero components
// for every truss in every result frame.
struct TrussForceRecord
{
    int id = 0;
    int domainId = 0;
    double axial = 0.0;
};

// Cable results reserve torque because the cable formulation is expected to
// support torsion, while shear forces and bending moments remain inapplicable.
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
    if (!type.valid()
        || H5Tset_size(type, size) < 0
        || H5Tset_strpad(type, H5T_STR_NULLTERM) < 0
        || H5Tset_cset(type, H5T_CSET_UTF8) < 0)
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
    if (!stringType.valid()
        || H5Tset_size(stringType, size) < 0
        || H5Tset_strpad(stringType, H5T_STR_NULLTERM) < 0
        || H5Tset_cset(stringType, H5T_CSET_UTF8) < 0)
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

    const hsize_t dims[1] = { static_cast<hsize_t>(records.size()) };
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

bool CreateExtendableDataset(hid_t file, const char* path, hid_t memoryType)
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

    if (H5Lexists(file, path, H5P_DEFAULT) > 0)
    {
        H5Ldelete(file, path, H5P_DEFAULT);
    }

    const hsize_t dims[1] = { 0 };
    const hsize_t maxDims[1] = { H5S_UNLIMITED };
    H5Handle space(H5Screate_simple(1, dims, maxDims), H5Sclose);
    if (!space.valid())
    {
        return false;
    }

    H5Handle plist(H5Pcreate(H5P_DATASET_CREATE), H5Pclose);
    if (!plist.valid())
    {
        return false;
    }

    const hsize_t chunkDims[1] = { 1024 };
    if (H5Pset_chunk(plist, 1, chunkDims) < 0)
    {
        return false;
    }

    H5Handle dataset(H5Dcreate2(file, path, memoryType, space, H5P_DEFAULT, plist, H5P_DEFAULT), H5Dclose);
    return dataset.valid();
}

template <typename Record>
bool AppendDataset(hid_t file, const char* path, hid_t memoryType, const std::vector<Record>& records, long long& position)
{
    H5Handle dataset(H5Dopen2(file, path, H5P_DEFAULT), H5Dclose);
    if (!dataset.valid())
    {
        return false;
    }

    H5Handle oldSpace(H5Dget_space(dataset), H5Sclose);
    if (!oldSpace.valid())
    {
        return false;
    }

    hsize_t oldDims[1] = { 0 };
    if (H5Sget_simple_extent_dims(oldSpace, oldDims, nullptr) < 0)
    {
        return false;
    }

    position = static_cast<long long>(oldDims[0]);
    if (records.empty())
    {
        return true;
    }

    const hsize_t newDims[1] = { oldDims[0] + static_cast<hsize_t>(records.size()) };
    if (H5Dset_extent(dataset, newDims) < 0)
    {
        return false;
    }

    H5Handle fileSpace(H5Dget_space(dataset), H5Sclose);
    if (!fileSpace.valid())
    {
        return false;
    }

    const hsize_t start[1] = { oldDims[0] };
    const hsize_t count[1] = { static_cast<hsize_t>(records.size()) };
    if (H5Sselect_hyperslab(fileSpace, H5S_SELECT_SET, start, nullptr, count, nullptr) < 0)
    {
        return false;
    }

    H5Handle memSpace(H5Screate_simple(1, count, nullptr), H5Sclose);
    if (!memSpace.valid())
    {
        return false;
    }

    return H5Dwrite(dataset, memoryType, memSpace, fileSpace, H5P_DEFAULT, records.data()) >= 0;
}

H5Handle CreateGridType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(GridRecord)), H5Tclose);
    if (!type.valid()) return {};
    H5Tinsert(type, "ID", HOFFSET(GridRecord, id), H5T_NATIVE_INT);
    InsertArray(type, "X", HOFFSET(GridRecord, x), H5T_NATIVE_DOUBLE, 3);
    H5Tinsert(type, "DOF_COUNT", HOFFSET(GridRecord, dofCount), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(GridRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateMaterialType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(MaterialRecord)), H5Tclose);
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
    H5Tinsert(type, "ID", HOFFSET(PropertyRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "MID", HOFFSET(PropertyRecord, materialId), H5T_NATIVE_INT);
    H5Tinsert(type, "SID", HOFFSET(PropertyRecord, sectionId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(PropertyRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateElementType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ElementRecord)), H5Tclose);
    if (!type.valid()) return {};
    H5Tinsert(type, "EID", HOFFSET(ElementRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "TYPE", HOFFSET(ElementRecord, type), H5T_NATIVE_INT);
    H5Tinsert(type, "PID", HOFFSET(ElementRecord, propertyId), H5T_NATIVE_INT);
    InsertArray(type, "G", HOFFSET(ElementRecord, nodeIds), H5T_NATIVE_INT, 2);
    InsertArray(type, "Q0", HOFFSET(ElementRecord, q0), H5T_NATIVE_DOUBLE, 3);
    H5Tinsert(type, "INIT_STRESS", HOFFSET(ElementRecord, initStress), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "ROLE", HOFFSET(ElementRecord, role), H5T_NATIVE_INT);
    H5Tinsert(type, "WIRE_ID", HOFFSET(ElementRecord, wireId), H5T_NATIVE_INT);
    H5Tinsert(type, "AERO_PROFILE_ID", HOFFSET(ElementRecord, aeroProfileId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ElementRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateConstraintType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ConstraintRecord)), H5Tclose);
    if (!type.valid()) return {};
    H5Tinsert(type, "ID", HOFFSET(ConstraintRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "G", HOFFSET(ConstraintRecord, nodeId), H5T_NATIVE_INT);
    H5Tinsert(type, "C", HOFFSET(ConstraintRecord, direction), H5T_NATIVE_INT);
    H5Tinsert(type, "D", HOFFSET(ConstraintRecord, value), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ConstraintRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateLoadType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(LoadRecord)), H5Tclose);
    if (!type.valid()) return {};
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

H5Handle CreateStepType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepRecord)), H5Tclose);
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
    H5Tinsert(type, "ID", HOFFSET(ModelSetRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "TYPE", HOFFSET(ModelSetRecord, type), H5T_NATIVE_INT);
    InsertFixedString(type, "NAME", HOFFSET(ModelSetRecord, name), sizeof(ModelSetRecord::name));
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ModelSetRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateModelSetMemberType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ModelSetMemberRecord)), H5Tclose);
    if (!type.valid()) return {};
    H5Tinsert(type, "SET_ID", HOFFSET(ModelSetMemberRecord, setId), H5T_NATIVE_INT);
    H5Tinsert(type, "ENTITY_ID", HOFFSET(ModelSetMemberRecord, entityId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ModelSetMemberRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateComputeRegionType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ComputeRegionRecord)), H5Tclose);
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
    H5Tinsert(type, "REGION_ID", HOFFSET(RegionLinkRecord, regionId), H5T_NATIVE_INT);
    H5Tinsert(type, "TARGET_ID", HOFFSET(RegionLinkRecord, targetId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(RegionLinkRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateStepMetadataType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepMetadataRecord)), H5Tclose);
    if (!type.valid()) return {};
    H5Tinsert(type, "STEP_ID", HOFFSET(StepMetadataRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "REGION_SCOPE", HOFFSET(StepMetadataRecord, regionScope), H5T_NATIVE_INT);
    InsertFixedString(type, "NAME", HOFFSET(StepMetadataRecord, name), sizeof(StepMetadataRecord::name));
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(StepMetadataRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateStepRegionLinkType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(StepRegionLinkRecord)), H5Tclose);
    if (!type.valid()) return {};
    H5Tinsert(type, "STEP_ID", HOFFSET(StepRegionLinkRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "REGION_ID", HOFFSET(StepRegionLinkRecord, regionId), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(StepRegionLinkRecord, domainId), H5T_NATIVE_INT);
    return type;
}

H5Handle CreateAeroCaseType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(AeroCaseRecord)), H5Tclose);
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
    H5Tinsert(type, "ID", HOFFSET(DomainRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "STEP_ID", HOFFSET(DomainRecord, stepId), H5T_NATIVE_INT);
    H5Tinsert(type, "INCREMENT", HOFFSET(DomainRecord, increment), H5T_NATIVE_INT);
    H5Tinsert(type, "ANALYSIS", HOFFSET(DomainRecord, analysis), H5T_NATIVE_INT);
    H5Tinsert(type, "TIME", HOFFSET(DomainRecord, time), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "LOAD_FACTOR", HOFFSET(DomainRecord, loadFactor), H5T_NATIVE_DOUBLE);
    return type;
}

H5Handle CreateIndexType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(IndexRecord)), H5Tclose);
    if (!type.valid()) return {};
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(IndexRecord, domainId), H5T_NATIVE_INT);
    H5Tinsert(type, "POSITION", HOFFSET(IndexRecord, position), H5T_NATIVE_LLONG);
    H5Tinsert(type, "LENGTH", HOFFSET(IndexRecord, length), H5T_NATIVE_LLONG);
    return type;
}

H5Handle CreateNodalType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(NodalRecord)), H5Tclose);
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
    H5Tinsert(type, "EID", HOFFSET(TrussForceRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(TrussForceRecord, domainId), H5T_NATIVE_INT);
    H5Tinsert(type, "AXIAL", HOFFSET(TrussForceRecord, axial), H5T_NATIVE_DOUBLE);
    return type;
}

H5Handle CreateCableForceType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(CableForceRecord)), H5Tclose);
    if (!type.valid()) return {};
    H5Tinsert(type, "EID", HOFFSET(CableForceRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(CableForceRecord, domainId), H5T_NATIVE_INT);
    H5Tinsert(type, "AXIAL", HOFFSET(CableForceRecord, axial), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "TORQUE", HOFFSET(CableForceRecord, torque), H5T_NATIVE_DOUBLE);
    return type;
}

H5Handle CreateElementStressType()
{
    H5Handle type(H5Tcreate(H5T_COMPOUND, sizeof(ElementStressRecord)), H5Tclose);
    if (!type.valid()) return {};
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
    if (!type.valid()) return {};
    H5Tinsert(type, "EID", HOFFSET(ElementStrainRecord, id), H5T_NATIVE_INT);
    H5Tinsert(type, "STRAIN", HOFFSET(ElementStrainRecord, strain), H5T_NATIVE_DOUBLE);
    H5Tinsert(type, "DOMAIN_ID", HOFFSET(ElementStrainRecord, domainId), H5T_NATIVE_INT);
    return type;
}

EnumKeyword::ElementType GetElementType(const std::shared_ptr<ElementBase>& pElement)
{
    if (std::dynamic_pointer_cast<ElementTruss>(pElement)) return EnumKeyword::ElementType::T3D2;
    if (std::dynamic_pointer_cast<ElementCable>(pElement)) return EnumKeyword::ElementType::CABLE;
    if (std::dynamic_pointer_cast<ElementBeam_CR>(pElement)) return EnumKeyword::ElementType::CR3D;
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
    if (std::dynamic_pointer_cast<SectionCircular>(pSection)) return EnumKeyword::SectionType::CIRCULAR;
    if (std::dynamic_pointer_cast<SectionRectangle>(pSection)) return EnumKeyword::SectionType::RECTANGULAR;
    return EnumKeyword::SectionType::UNKNOWN;
}

std::vector<GridRecord> BuildGridRecords(const StructureData* pData)
{
    std::vector<GridRecord> records;
    records.reserve(pData->m_Nodes.size());
    for (const auto& pair : pData->m_Nodes)
    {
        const auto& pNode = pair.second;
        if (!pNode) continue;

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
        if (!pMaterial) continue;

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
        if (!pSection) continue;

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
        if (!pProperty) continue;

        PropertyRecord record;
        record.id = pProperty->m_Id;
        if (auto pMaterial = pProperty->m_pMaterial.lock()) record.materialId = pMaterial->m_Id;
        if (auto pSection = pProperty->m_pSection.lock()) record.sectionId = pSection->m_Id;
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
        if (!pElement) continue;

        ElementRecord record;
        record.id = pElement->m_Id;
        record.type = ToInt(GetElementType(pElement));
        record.initStress = pElement->m_InitStress;
        record.role = static_cast<int>(pElement->m_Role);
        record.wireId = pElement->m_WireId;
        record.aeroProfileId = pElement->m_AeroProfileId;
        if (auto pProperty = pElement->m_pProperty.lock()) record.propertyId = pProperty->m_Id;
        if (pElement->m_pNode.size() > 0)
        {
            if (auto pNode = pElement->m_pNode[0].lock()) record.nodeIds[0] = pNode->m_Id;
        }
        if (pElement->m_pNode.size() > 1)
        {
            if (auto pNode = pElement->m_pNode[1].lock()) record.nodeIds[1] = pNode->m_Id;
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
        if (!pConstraint) continue;

        ConstraintRecord record;
        record.id = pConstraint->m_Id;
        if (auto pNode = pConstraint->m_pNode.lock()) record.nodeId = pNode->m_Id;
        record.direction = ToInt(pConstraint->m_Direction);
        record.value = pConstraint->m_Value;
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
        if (!pLoad) continue;

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
            if (auto pNode = pNodeLoad->m_pNode.lock()) record.targetId = pNode->m_Id;
            record.value = pNodeLoad->m_Value;
        }
        else if (auto pElementLoad = std::dynamic_pointer_cast<Force_Element>(pLoad))
        {
            if (auto pElement = pElementLoad->m_pElement.lock()) record.targetId = pElement->m_Id;
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

std::vector<StepRecord> BuildStepRecords(const StructureData* pData)
{
    std::vector<StepRecord> records;
    records.reserve(pData->m_AnalysisStep.size());
    for (const auto& pair : pData->m_AnalysisStep)
    {
        const auto& pStep = pair.second;
        if (!pStep) continue;

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
        CopyUtf8String(record.name, sizeof(record.name), step->m_Name);
        metadata.push_back(record);

        for (int regionId : step->m_ComputeRegionIds)
        {
            regionLinks.push_back({stepId, regionId});
        }
    }
}

int GetAeroDataSize(const std::vector<BladeModel>& models)
{
    if (models.empty())
    {
        return 0;
    }
    return static_cast<int>(models.front().lift.size());
}

void AppendAeroRecords(int caseId,
    const AeroCaseKey& key,
    const std::vector<BladeModel>& models,
    const std::filesystem::path& sourceFile,
    const AeroManager& manager,
    std::vector<AeroCaseRecord>& caseRecords,
    std::vector<AeroCoefficientRecord>& coefficientRecords)
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
        const size_t dataSize = std::min({ model.lift.size(), model.drag.size(), model.moment.size() });
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

void BuildAeroRecords(const StructureData* pData,
    std::vector<AeroCaseRecord>& caseRecords,
    std::vector<AeroCoefficientRecord>& coefficientRecords)
{
    const AeroManager& manager = pData->m_AeroManager;
    int caseId = 1;

    for (const auto& pair : manager.getCaseModels())
    {
        AppendAeroRecords(caseId,
            pair.first,
            pair.second,
            manager.getCaseSourceFile(pair.first),
            manager,
            caseRecords,
            coefficientRecords);
        ++caseId;
    }

    if (caseRecords.empty() && !manager.getModels().empty())
    {
        AeroCaseKey currentKey;
        AppendAeroRecords(caseId,
            currentKey,
            manager.getModels(),
            manager.getCurrentSourceFile(),
            manager,
            caseRecords,
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
    H5Handle loadType = CreateLoadType();
    H5Handle stepType = CreateStepType();
    H5Handle modelSetType = CreateModelSetType();
    H5Handle modelSetMemberType = CreateModelSetMemberType();
    H5Handle computeRegionType = CreateComputeRegionType();
    H5Handle regionLinkType = CreateRegionLinkType();
    H5Handle stepMetadataType = CreateStepMetadataType();
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
    BuildComputeRegionRecords(pData, computeRegions, regionSetLinks,
        regionDirectNodes, regionDirectElements);
    BuildStepMetadataRecords(pData, stepMetadata, stepRegionLinks);
    BuildAeroRecords(pData, aeroCases, aeroCoefficients);

    return gridType.valid()
        && materialType.valid()
        && sectionType.valid()
        && propertyType.valid()
        && elementType.valid()
        && constraintType.valid()
        && loadType.valid()
        && stepType.valid()
        && modelSetType.valid()
        && modelSetMemberType.valid()
        && computeRegionType.valid()
        && regionLinkType.valid()
        && stepMetadataType.valid()
        && stepRegionLinkType.valid()
        && aeroCaseType.valid()
        && aeroCoefficientType.valid()
        && WriteDataset(file, "/YQY/INPUT/NODE/GRID", gridType, BuildGridRecords(pData))
        && WriteDataset(file, "/YQY/INPUT/MATERIAL/MAT", materialType, BuildMaterialRecords(pData))
        && WriteDataset(file, "/YQY/INPUT/SECTION/SECTION", sectionType, BuildSectionRecords(pData))
        && WriteDataset(file, "/YQY/INPUT/PROPERTY/PROPERTY", propertyType, BuildPropertyRecords(pData))
        && WriteDataset(file, "/YQY/INPUT/ELEMENT/ELEMENT", elementType, BuildElementRecords(pData))
        && WriteDataset(file, "/YQY/INPUT/CONSTRAINT/SPC", constraintType, BuildConstraintRecords(pData))
        && WriteDataset(file, "/YQY/INPUT/LOAD/LOAD", loadType, BuildLoadRecords(pData))
        && WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/STEP", stepType, BuildStepRecords(pData))
        && WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/METADATA", stepMetadataType, stepMetadata)
        && WriteDataset(file, "/YQY/INPUT/ANALYSIS_STEP/REGION_LINK", stepRegionLinkType, stepRegionLinks)
        && WriteDataset(file, "/YQY/INPUT/MODEL_SET/SET", modelSetType, modelSets)
        && WriteDataset(file, "/YQY/INPUT/MODEL_SET/MEMBER", modelSetMemberType, modelSetMembers)
        && WriteDataset(file, "/YQY/INPUT/COMPUTE_REGION/REGION", computeRegionType, computeRegions)
        && WriteDataset(file, "/YQY/INPUT/COMPUTE_REGION/SET_LINK", regionLinkType, regionSetLinks)
        && WriteDataset(file, "/YQY/INPUT/COMPUTE_REGION/DIRECT_NODE", regionLinkType, regionDirectNodes)
        && WriteDataset(file, "/YQY/INPUT/COMPUTE_REGION/DIRECT_ELEMENT", regionLinkType, regionDirectElements)
        && WriteDataset(file, "/YQY/INPUT/AERO/CASE", aeroCaseType, aeroCases)
        && WriteDataset(file, "/YQY/INPUT/AERO/COEFFICIENT", aeroCoefficientType, aeroCoefficients);
}

bool WriteResultData(hid_t file, const StructureData* pData)
{
    const auto& frames = pData->GetOutputter().GetDataSet();
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
        const bool storesDynamicKinematics = frame.GetAnalysisType()
            == static_cast<int>(EnumKeyword::StepType::DYNAMIC);

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
                velocity.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::V1);
                velocity.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::V2);
                velocity.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::V3);
                velocity.domainId = domainId;
                velocities.push_back(velocity);

                NodalRecord acceleration;
                acceleration.id = nodeId;
                acceleration.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::A1);
                acceleration.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::A2);
                acceleration.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::A3);
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
            const auto elementType = elementTypes.value(elementId, EnumKeyword::ElementType::UNKNOWN);
            if (elementType == EnumKeyword::ElementType::T3D2)
            {
                TrussForceRecord force;
                force.id = elementId;
                force.domainId = domainId;
                force.axial = frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce);
                trussForces.push_back(force);
                ++trussLength;
            }
            else if (elementType == EnumKeyword::ElementType::CABLE)
            {
                CableForceRecord force;
                force.id = elementId;
                force.domainId = domainId;
                force.axial = frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce);
                force.torque = frame.GetElementData(elementId, EnumKeyword::ElementResultType::Torque);
                cableForces.push_back(force);
                ++cableLength;
            }
            else
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
                ++elementLength;
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

        AppendFrameIndex(elementForceIndex, domainId, elementPosition, elementLength);
        AppendFrameIndex(trussForceIndex, domainId, trussPosition, trussLength);
        AppendFrameIndex(cableForceIndex, domainId, cablePosition, cableLength);
        const long long resultElementLength = static_cast<long long>(frame.GetElementDatas().size());
        AppendFrameIndex(elementStressIndex, domainId, stressPosition, resultElementLength);
        AppendFrameIndex(elementStrainIndex, domainId, strainPosition, resultElementLength);

        ++domainId;
    }

    H5Handle domainType = CreateDomainType();
    H5Handle indexType = CreateIndexType();
    H5Handle nodalType = CreateNodalType();
    H5Handle elementForceType = CreateElementForceType();
    H5Handle trussForceType = CreateTrussForceType();
    H5Handle cableForceType = CreateCableForceType();
    H5Handle elementStressType = CreateElementStressType();
    H5Handle elementStrainType = CreateElementStrainType();

    return domainType.valid()
        && indexType.valid()
        && nodalType.valid()
        && elementForceType.valid()
        && trussForceType.valid()
        && cableForceType.valid()
        && elementStressType.valid()
        && elementStrainType.valid()
        && WriteDataset(file, "/YQY/RESULT/DOMAINS", domainType, domains)
        && WriteDataset(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, displacements)
        && WriteDataset(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType, currentCoordinates)
        && WriteDataset(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType, velocities)
        && WriteDataset(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType, accelerations)
        && WriteDataset(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType, reactions)
        && WriteDataset(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType, elementForces)
        && WriteDataset(file, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", trussForceType, trussForces)
        && WriteDataset(file, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE", cableForceType, cableForces)
        && WriteDataset(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType, elementStresses)
        && WriteDataset(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType, elementStrains)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType, displacementIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType, currentCoordinateIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType, velocityIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType, accelerationIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType, reactionIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType, elementForceIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", indexType, trussForceIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE", indexType, cableForceIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType, elementStressIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType, elementStrainIndex);
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
    QHash<int, IndexRecord> elementForceIndex;
    QHash<int, IndexRecord> trussForceIndex;
    QHash<int, IndexRecord> cableForceIndex;
    QHash<int, IndexRecord> stressIndex;
    QHash<int, IndexRecord> strainIndex;

    ~Impl()
    {
        CloseResult();
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
        tempFileName = MakeAsciiTempHdf5FileName();
        elementTypes = BuildElementTypeMap(pData);

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

        const bool attrOk = WriteStringAttribute(file, "FORMAT", "YQY_H5")
            && WriteIntAttribute(file, "SCHEMA_VERSION", kSchemaVersion)
            && WriteStringAttribute(file, "FILE_KIND", "MODEL_RESULT")
            && WriteIntAttribute(file, "RESULT_COMPLETE", 0)
            && WriteStringAttribute(file, "CREATED_TIME", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
            && WriteStringAttribute(file, "PROGRAM", "YQY_CAE")
            && WriteStringAttribute(file, "SOURCE_MODEL", sourceModelName);

        return attrOk && WriteInputData(file, pData) && CreateStreamDatasets();
    }

    bool CreateStreamDatasets()
    {
        H5Handle domainType = CreateDomainType();
        H5Handle indexType = CreateIndexType();
        H5Handle nodalType = CreateNodalType();
        H5Handle elementForceType = CreateElementForceType();
        H5Handle trussForceType = CreateTrussForceType();
        H5Handle cableForceType = CreateCableForceType();
        H5Handle elementStressType = CreateElementStressType();
        H5Handle elementStrainType = CreateElementStrainType();

        return domainType.valid()
            && indexType.valid()
            && nodalType.valid()
            && elementForceType.valid()
            && trussForceType.valid()
            && cableForceType.valid()
            && elementStressType.valid()
            && elementStrainType.valid()
            && CreateExtendableDataset(file, "/YQY/RESULT/DOMAINS", domainType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType)
            && CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", trussForceType)
            && CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE", cableForceType)
            && CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType)
            && CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType);
    }

    bool WriteFrame(int domainId, int stepId, int increment, int analysis, double time, const DataFrame& frame)
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

        std::vector<DomainRecord> domains{ domain };
        H5Handle domainType = CreateDomainType();
        long long domainPosition = 0;
        if (!domainType.valid() || !AppendDataset(file, "/YQY/RESULT/DOMAINS", domainType, domains, domainPosition))
        {
            return false;
        }

        const bool storesDynamicKinematics = analysis
            == static_cast<int>(EnumKeyword::StepType::DYNAMIC);

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
                velocity.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::V1);
                velocity.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::V2);
                velocity.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::V3);
                velocity.domainId = domainId;
                velocities.push_back(velocity);

                NodalRecord acceleration;
                acceleration.id = nodeId;
                acceleration.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::A1);
                acceleration.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::A2);
                acceleration.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::A3);
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
            if (elementType == EnumKeyword::ElementType::T3D2)
            {
                TrussForceRecord force;
                force.id = elementId;
                force.domainId = domainId;
                force.axial = frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce);
                trussForces.push_back(force);
            }
            else if (elementType == EnumKeyword::ElementType::CABLE)
            {
                CableForceRecord force;
                force.id = elementId;
                force.domainId = domainId;
                force.axial = frame.GetElementData(elementId, EnumKeyword::ElementResultType::AxialForce);
                force.torque = frame.GetElementData(elementId, EnumKeyword::ElementResultType::Torque);
                cableForces.push_back(force);
            }
            else
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

        H5Handle indexType = CreateIndexType();
        H5Handle nodalType = CreateNodalType();
        H5Handle elementForceType = CreateElementForceType();
        H5Handle trussForceType = CreateTrussForceType();
        H5Handle cableForceType = CreateCableForceType();
        H5Handle elementStressType = CreateElementStressType();
        H5Handle elementStrainType = CreateElementStrainType();
        if (!indexType.valid() || !nodalType.valid() || !elementForceType.valid()
            || !trussForceType.valid() || !cableForceType.valid()
            || !elementStressType.valid() || !elementStrainType.valid())
        {
            return false;
        }

        return AppendRecordsWithIndex("/YQY/RESULT/NODAL/DISPLACEMENT", "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, indexType, domainId, displacements)
            && AppendRecordsWithIndex("/YQY/RESULT/NODAL/CURRENT_COORDINATE", "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType, indexType, domainId, currentCoordinates)
            && AppendRecordsWithIndex("/YQY/RESULT/NODAL/VELOCITY", "/INDEX/YQY/RESULT/NODAL/VELOCITY", nodalType, indexType, domainId, velocities)
            && AppendRecordsWithIndex("/YQY/RESULT/NODAL/ACCELERATION", "/INDEX/YQY/RESULT/NODAL/ACCELERATION", nodalType, indexType, domainId, accelerations)
            && AppendRecordsWithIndex("/YQY/RESULT/NODAL/REACTION_FORCE", "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", nodalType, indexType, domainId, reactions)
            && AppendRecordsWithIndex("/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType, indexType, domainId, elementForces)
            && AppendRecordsWithIndex("/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", trussForceType, indexType, domainId, trussForces)
            && AppendRecordsWithIndex("/YQY/RESULT/ELEMENTAL/CABLE_FORCE", "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE", cableForceType, indexType, domainId, cableForces)
            && AppendRecordsWithIndex("/YQY/RESULT/ELEMENTAL/STRESS", "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", elementStressType, indexType, domainId, elementStresses)
            && AppendRecordsWithIndex("/YQY/RESULT/ELEMENTAL/STRAIN", "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType, indexType, domainId, elementStrains);
    }

    template <typename Record>
    bool AppendRecordsWithIndex(const char* dataPath, const char* indexPath, hid_t dataType, hid_t indexType,
        int domainId, const std::vector<Record>& records)
    {
        long long position = 0;
        if (!AppendDataset(file, dataPath, dataType, records, position))
        {
            return false;
        }

        IndexRecord index;
        index.domainId = domainId;
        index.position = position;
        index.length = static_cast<long long>(records.size());

        std::vector<IndexRecord> indices{ index };
        long long indexPosition = 0;
        return AppendDataset(file, indexPath, indexType, indices, indexPosition);
    }

    void End(bool resultComplete)
    {
        if (file.valid())
        {
            WriteIntAttribute(file, "RESULT_COMPLETE", resultComplete ? 1 : 0);
            H5Fflush(file, H5F_SCOPE_GLOBAL);
        }
        file.reset();

        if (!tempFileName.isEmpty() && !targetFileName.isEmpty())
        {
            MoveTempFileToTarget(tempFileName, targetFileName);
        }
        tempFileName.clear();
        targetFileName.clear();
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

bool Hdf5ModelIO::ExportHdf5(const QString& fileName, const StructureData* pData,
    const QString& sourceModelName, bool resultComplete) const
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
    const QString tempFileName = MakeAsciiTempHdf5FileName();
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

    const bool attrOk = WriteStringAttribute(file, "FORMAT", "YQY_H5")
        && WriteIntAttribute(file, "SCHEMA_VERSION", kSchemaVersion)
        && WriteStringAttribute(file, "FILE_KIND", "MODEL_RESULT")
        && WriteIntAttribute(file, "RESULT_COMPLETE", resultComplete ? 1 : 0)
        && WriteStringAttribute(file, "CREATED_TIME", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
        && WriteStringAttribute(file, "PROGRAM", "YQY_CAE")
        && WriteStringAttribute(file, "SOURCE_MODEL", sourceModelName);

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
    const QString tempFileName = MakeAsciiTempHdf5FileName();
    const QByteArray path = ToHdf5Path(tempFileName);
    H5Handle file(H5Fcreate(path.constData(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT), H5Fclose);
    if (!file.valid() || !CreateGroupRecursive(file, "/YQY"))
    {
        QFile::remove(tempFileName);
        return false;
    }

    const bool written = WriteStringAttribute(file, "FORMAT", "YQY_H5")
        && WriteIntAttribute(file, "SCHEMA_VERSION", kSchemaVersion)
        && WriteStringAttribute(file, "FILE_KIND", "MODEL")
        && WriteIntAttribute(file, "RESULT_COMPLETE", 0)
        && WriteStringAttribute(file, "CREATED_TIME", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
        && WriteStringAttribute(file, "PROGRAM", "YQY_CAE")
        && WriteStringAttribute(file, "SOURCE_MODEL", sourceModelName)
        && WriteInputData(file, pData);
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

bool Hdf5ModelIO::WriteResultFrame(int domainId, int stepId, int increment, int analysis, double time, const DataFrame& frame)
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    return m_impl->WriteFrame(domainId, stepId, increment, analysis, time, frame);
}

void Hdf5ModelIO::EndResultStream(bool resultComplete)
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    m_impl->End(resultComplete);
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

    hsize_t dims[1] = { 0 };
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
    H5Handle loadType = CreateLoadType();
    H5Handle stepType = CreateStepType();
    H5Handle modelSetType = CreateModelSetType();
    H5Handle modelSetMemberType = CreateModelSetMemberType();
    H5Handle computeRegionType = CreateComputeRegionType();
    H5Handle regionLinkType = CreateRegionLinkType();
    H5Handle stepMetadataType = CreateStepMetadataType();
    H5Handle stepRegionLinkType = CreateStepRegionLinkType();
    if (!gridType.valid() || !materialType.valid() || !sectionType.valid() || !propertyType.valid()
        || !elementType.valid() || !constraintType.valid() || !loadType.valid() || !stepType.valid()
        || !modelSetType.valid() || !modelSetMemberType.valid() || !computeRegionType.valid()
        || !regionLinkType.valid() || !stepMetadataType.valid() || !stepRegionLinkType.valid())
    {
        return false;
    }

    std::vector<GridRecord> grids;
    std::vector<MaterialRecord> materials;
    std::vector<SectionRecord> sections;
    std::vector<PropertyRecord> properties;
    std::vector<ElementRecord> elements;
    std::vector<ConstraintRecord> constraints;
    std::vector<LoadRecord> loads;
    std::vector<StepRecord> steps;
    std::vector<ModelSetRecord> modelSets;
    std::vector<ModelSetMemberRecord> modelSetMembers;
    std::vector<ComputeRegionRecord> computeRegions;
    std::vector<RegionLinkRecord> regionSetLinks;
    std::vector<RegionLinkRecord> regionDirectNodes;
    std::vector<RegionLinkRecord> regionDirectElements;
    std::vector<StepMetadataRecord> stepMetadata;
    std::vector<StepRegionLinkRecord> stepRegionLinks;
    if (!ReadDatasetAll(file, "/YQY/INPUT/NODE/GRID", gridType, grids)
        || !ReadDatasetAll(file, "/YQY/INPUT/MATERIAL/MAT", materialType, materials)
        || !ReadDatasetAll(file, "/YQY/INPUT/SECTION/SECTION", sectionType, sections)
        || !ReadDatasetAll(file, "/YQY/INPUT/PROPERTY/PROPERTY", propertyType, properties)
        || !ReadDatasetAll(file, "/YQY/INPUT/ELEMENT/ELEMENT", elementType, elements)
        || !ReadDatasetAll(file, "/YQY/INPUT/CONSTRAINT/SPC", constraintType, constraints)
        || !ReadDatasetAll(file, "/YQY/INPUT/LOAD/LOAD", loadType, loads)
        || !ReadDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/STEP", stepType, steps)
        || !ReadOptionalDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/METADATA", stepMetadataType, stepMetadata)
        || !ReadOptionalDatasetAll(file, "/YQY/INPUT/ANALYSIS_STEP/REGION_LINK", stepRegionLinkType, stepRegionLinks)
        || !ReadOptionalDatasetAll(file, "/YQY/INPUT/MODEL_SET/SET", modelSetType, modelSets)
        || !ReadOptionalDatasetAll(file, "/YQY/INPUT/MODEL_SET/MEMBER", modelSetMemberType, modelSetMembers)
        || !ReadOptionalDatasetAll(file, "/YQY/INPUT/COMPUTE_REGION/REGION", computeRegionType, computeRegions)
        || !ReadOptionalDatasetAll(file, "/YQY/INPUT/COMPUTE_REGION/SET_LINK", regionLinkType, regionSetLinks)
        || !ReadOptionalDatasetAll(file, "/YQY/INPUT/COMPUTE_REGION/DIRECT_NODE", regionLinkType, regionDirectNodes)
        || !ReadOptionalDatasetAll(file, "/YQY/INPUT/COMPUTE_REGION/DIRECT_ELEMENT", regionLinkType, regionDirectElements)
        || grids.empty())
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
        if (storedType == EnumKeyword::SectionType::RECTANGULAR
            || (storedType == EnumKeyword::SectionType::UNKNOWN && record.width > 0.0 && record.height > 0.0))
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
        if (record.id <= 0 || restored->m_ModelSets.find(record.id) != restored->m_ModelSets.cend()
            || (record.type != static_cast<int>(ModelSetType::Node)
                && record.type != static_cast<int>(ModelSetType::Element)))
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
        config.dynamicSolverType = static_cast<SolverNameSpace::SolverType>(record.dynamicSolverType);
        restored->AddAnalysisStep(config);
    }

    for (const StepMetadataRecord& record : stepMetadata)
    {
        const auto stepIt = restored->m_AnalysisStep.find(record.stepId);
        if (stepIt == restored->m_AnalysisStep.cend() || !stepIt->second
            || (record.regionScope != static_cast<int>(AnalysisRegionScope::AllEnabledRegions)
                && record.regionScope != static_cast<int>(AnalysisRegionScope::SelectedRegions)))
        {
            return false;
        }
        stepIt->second->m_Name = ReadUtf8String(record.name, sizeof(record.name));
        stepIt->second->m_RegionScope = static_cast<AnalysisRegionScope>(record.regionScope);
    }
    for (const StepRegionLinkRecord& record : stepRegionLinks)
    {
        const auto stepIt = restored->m_AnalysisStep.find(record.stepId);
        if (stepIt == restored->m_AnalysisStep.cend() || !stepIt->second
            || restored->m_ComputeRegions.find(record.regionId) == restored->m_ComputeRegions.cend())
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

    data->Clear();
    data->m_Nodes = std::move(restored->m_Nodes);
    data->m_Elements = std::move(restored->m_Elements);
    data->m_Material = std::move(restored->m_Material);
    data->m_Section = std::move(restored->m_Section);
    data->m_Property = std::move(restored->m_Property);
    data->m_Constraint = std::move(restored->m_Constraint);
    data->m_Load = std::move(restored->m_Load);
    data->m_AnalysisStep = std::move(restored->m_AnalysisStep);
    data->m_ModelSets = std::move(restored->m_ModelSets);
    data->m_ComputeRegions = std::move(restored->m_ComputeRegions);
    return true;
}

template <typename Record>
bool ReadDatasetBlock(hid_t file, const char* path, hid_t memoryType, long long position, long long length, std::vector<Record>& records)
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

    const hsize_t start[1] = { static_cast<hsize_t>(position) };
    const hsize_t count[1] = { static_cast<hsize_t>(length) };
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
    case EnumKeyword::NodeResultType::U1: return "U1";
    case EnumKeyword::NodeResultType::U2: return "U2";
    case EnumKeyword::NodeResultType::U3: return "U3";
    case EnumKeyword::NodeResultType::MagnitudeU: return "MAG";
    case EnumKeyword::NodeResultType::CX: return "CX";
    case EnumKeyword::NodeResultType::CY: return "CY";
    case EnumKeyword::NodeResultType::CZ: return "CZ";
    case EnumKeyword::NodeResultType::V1: return "V1";
    case EnumKeyword::NodeResultType::V2: return "V2";
    case EnumKeyword::NodeResultType::V3: return "V3";
    case EnumKeyword::NodeResultType::A1: return "A1";
    case EnumKeyword::NodeResultType::A2: return "A2";
    case EnumKeyword::NodeResultType::A3: return "A3";
    case EnumKeyword::NodeResultType::UR1: return "UR1";
    case EnumKeyword::NodeResultType::UR2: return "UR2";
    case EnumKeyword::NodeResultType::UR3: return "UR3";
    case EnumKeyword::NodeResultType::R1: return "R1";
    case EnumKeyword::NodeResultType::R2: return "R2";
    case EnumKeyword::NodeResultType::R3: return "R3";
    default: return "UNKNOWN";
    }
}

QString ElementTypeName(EnumKeyword::ElementResultType type)
{
    switch (type)
    {
    case EnumKeyword::ElementResultType::AxialForce: return "AXIAL";
    case EnumKeyword::ElementResultType::ShearY: return "SHEARY";
    case EnumKeyword::ElementResultType::ShearZ: return "SHEARZ";
    case EnumKeyword::ElementResultType::Torque: return "TORQUE";
    case EnumKeyword::ElementResultType::MomentY: return "MY";
    case EnumKeyword::ElementResultType::MomentZ: return "MZ";
    case EnumKeyword::ElementResultType::Strain: return "STRAIN";
    case EnumKeyword::ElementResultType::InitStress: return "S0";
    case EnumKeyword::ElementResultType::CurrentStress: return "S";
    case EnumKeyword::ElementResultType::DeltaStress: return "DS";
    default: return "UNKNOWN";
    }
}

QString FormatBdfValue(double value)
{
    return QString::number(value, 'E', 8).rightJustified(16, ' ');
}

double FindNodalValue(int nodeId, EnumKeyword::NodeResultType type,
    const QHash<int, NodalRecord>& displacements,
    const QHash<int, NodalRecord>& currentCoordinates,
    const QHash<int, NodalRecord>& velocities,
    const QHash<int, NodalRecord>& accelerations,
    const QHash<int, NodalRecord>& reactions)
{
    const NodalRecord zero;
    switch (type)
    {
    case EnumKeyword::NodeResultType::U1: return displacements.value(nodeId, zero).x;
    case EnumKeyword::NodeResultType::U2: return displacements.value(nodeId, zero).y;
    case EnumKeyword::NodeResultType::U3: return displacements.value(nodeId, zero).z;
    case EnumKeyword::NodeResultType::MagnitudeU:
    {
        const auto record = displacements.value(nodeId, zero);
        return std::sqrt(record.x * record.x + record.y * record.y + record.z * record.z);
    }
    case EnumKeyword::NodeResultType::CX: return currentCoordinates.value(nodeId, zero).x;
    case EnumKeyword::NodeResultType::CY: return currentCoordinates.value(nodeId, zero).y;
    case EnumKeyword::NodeResultType::CZ: return currentCoordinates.value(nodeId, zero).z;
    case EnumKeyword::NodeResultType::V1: return velocities.value(nodeId, zero).x;
    case EnumKeyword::NodeResultType::V2: return velocities.value(nodeId, zero).y;
    case EnumKeyword::NodeResultType::V3: return velocities.value(nodeId, zero).z;
    case EnumKeyword::NodeResultType::A1: return accelerations.value(nodeId, zero).x;
    case EnumKeyword::NodeResultType::A2: return accelerations.value(nodeId, zero).y;
    case EnumKeyword::NodeResultType::A3: return accelerations.value(nodeId, zero).z;
    case EnumKeyword::NodeResultType::UR1: return displacements.value(nodeId, zero).rx;
    case EnumKeyword::NodeResultType::UR2: return displacements.value(nodeId, zero).ry;
    case EnumKeyword::NodeResultType::UR3: return displacements.value(nodeId, zero).rz;
    case EnumKeyword::NodeResultType::R1: return reactions.value(nodeId, zero).x;
    case EnumKeyword::NodeResultType::R2: return reactions.value(nodeId, zero).y;
    case EnumKeyword::NodeResultType::R3: return reactions.value(nodeId, zero).z;
    default: return 0.0;
    }
}

double FindElementValue(int elementId, EnumKeyword::ElementResultType type,
    const QHash<int, ElementForceRecord>& forces,
    const QHash<int, ElementStressRecord>& stresses,
    const QHash<int, ElementStrainRecord>& strains)
{
    const ElementForceRecord zeroForce;
    const ElementStressRecord zeroStress;
    const ElementStrainRecord zeroStrain;

    switch (type)
    {
    case EnumKeyword::ElementResultType::AxialForce: return forces.value(elementId, zeroForce).axial;
    case EnumKeyword::ElementResultType::ShearY: return forces.value(elementId, zeroForce).shearY;
    case EnumKeyword::ElementResultType::ShearZ: return forces.value(elementId, zeroForce).shearZ;
    case EnumKeyword::ElementResultType::Torque: return forces.value(elementId, zeroForce).torque;
    case EnumKeyword::ElementResultType::MomentY: return forces.value(elementId, zeroForce).momentY;
    case EnumKeyword::ElementResultType::MomentZ: return forces.value(elementId, zeroForce).momentZ;
    case EnumKeyword::ElementResultType::Strain: return strains.value(elementId, zeroStrain).strain;
    case EnumKeyword::ElementResultType::InitStress: return stresses.value(elementId, zeroStress).initStress;
    case EnumKeyword::ElementResultType::CurrentStress: return stresses.value(elementId, zeroStress).currentStress;
    case EnumKeyword::ElementResultType::DeltaStress: return stresses.value(elementId, zeroStress).deltaStress;
    default: return 0.0;
    }
}

template <typename Record>
QHash<int, Record> BuildRecordMap(const std::vector<Record>& records)
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

bool Hdf5ModelIO::ExportBdfResultFromHdf5(const QString& hdf5FileName,
    const QString& bdfFileName,
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
    if (!domainType.valid() || !indexType.valid() || !nodalType.valid()
        || !elementForceType.valid() || !trussForceType.valid() || !cableForceType.valid()
        || !elementStressType.valid() || !elementStrainType.valid())
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

    if (!ReadDatasetAll(file, "/YQY/RESULT/DOMAINS", domainType, domains)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType, displacementIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType, currentCoordinateIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType, velocityIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType, accelerationIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType, reactionIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType, elementForceIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", indexType, trussForceIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE", indexType, cableForceIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType, elementStressIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType, elementStrainIndexRecords))
    {
        return ReportHdf5FormatError();
    }

    std::sort(domains.begin(), domains.end(), [](const DomainRecord& lhs, const DomainRecord& rhs) {
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

        if (!ReadDatasetBlock(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, displacementInfo.position, displacementInfo.length, displacementRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType, currentCoordinateInfo.position, currentCoordinateInfo.length, currentCoordinateRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType, velocityInfo.position, velocityInfo.length, velocityRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType, accelerationInfo.position, accelerationInfo.length, accelerationRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType, reactionInfo.position, reactionInfo.length, reactionRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType, forceInfo.position, forceInfo.length, forceRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE", trussForceType, trussForceInfo.position, trussForceInfo.length, trussForceRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE", cableForceType, cableForceInfo.position, cableForceInfo.length, cableForceRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType, stressInfo.position, stressInfo.length, stressRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType, strainInfo.position, strainInfo.length, strainRecords))
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

        stream << FormatBdfValue(domain.time);
        for (int nodeId : nodeIds)
        {
            for (auto type : nodeTypes)
            {
                stream << FormatBdfValue(FindNodalValue(nodeId, type, displacements, currentCoordinates, velocities, accelerations, reactions));
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
    const auto datasetLength = [](hid_t fileId, const char* datasetPath) -> qint64 {
        H5Handle dataset(H5Dopen2(fileId, datasetPath, H5P_DEFAULT), H5Dclose);
        if (!dataset.valid())
            return 0;
        H5Handle space(H5Dget_space(dataset), H5Sclose);
        if (!space.valid())
            return 0;
        hsize_t dimensions[1] = { 0 };
        return H5Sget_simple_extent_dims(space, dimensions, nullptr) >= 0
            ? static_cast<qint64>(dimensions[0]) : 0;
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

bool Hdf5ModelIO::OpenResultFile(const QString& fileName,
    std::vector<Hdf5ResultFrameInfo>& frames)
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
    std::vector<IndexRecord> forces;
    std::vector<IndexRecord> trussForces;
    std::vector<IndexRecord> cableForces;
    std::vector<IndexRecord> stresses;
    std::vector<IndexRecord> strains;
    const bool ok = domainType.valid() && indexType.valid()
        && ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/DOMAINS", domainType,
            m_impl->resultDomains)
        && ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT",
            indexType, displacement)
        && ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE",
            indexType, currentCoordinates)
        && ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE",
            indexType, forces)
        && ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/TRUSS_FORCE",
            indexType, trussForces)
        && ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/CABLE_FORCE",
            indexType, cableForces)
        && ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS",
            indexType, stresses)
        && ReadDatasetAll(m_impl->resultFile, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN",
            indexType, strains);
    if (!ok || m_impl->resultDomains.empty())
    {
        m_impl->CloseResult();
        return ReportHdf5FormatError();
    }

    std::sort(m_impl->resultDomains.begin(), m_impl->resultDomains.end(),
        [](const DomainRecord& lhs, const DomainRecord& rhs) { return lhs.id < rhs.id; });
    m_impl->displacementIndex = BuildIndexMap(displacement);
    m_impl->currentCoordinateIndex = BuildIndexMap(currentCoordinates);
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

    const auto includeValue = [](Hdf5ResultRange& range, double value)
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
    };

    H5Handle nodalType = CreateNodalType();
    std::vector<NodalRecord> displacements;
    if (!nodalType.valid()
        || !ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, displacements))
    {
        return ReportHdf5FormatError();
    }
    for (const NodalRecord& value : displacements)
    {
        includeValue(ranges.displacementMagnitude,
            std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z));
        includeValue(ranges.displacementX, value.x);
        includeValue(ranges.displacementY, value.y);
        includeValue(ranges.displacementZ, value.z);
    }
    displacements.clear();
    displacements.shrink_to_fit();

    H5Handle forceType = CreateElementForceType();
    H5Handle trussForceType = CreateTrussForceType();
    H5Handle cableForceType = CreateCableForceType();
    std::vector<ElementForceRecord> forces;
    std::vector<TrussForceRecord> trussForces;
    std::vector<CableForceRecord> cableForces;
    if (!forceType.valid() || !trussForceType.valid() || !cableForceType.valid()
        || !ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", forceType, forces)
        || !ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE",
            trussForceType, trussForces)
        || !ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE",
            cableForceType, cableForces))
    {
        return ReportHdf5FormatError();
    }
    for (const ElementForceRecord& value : forces)
        includeValue(ranges.axialForce, value.axial);
    for (const TrussForceRecord& value : trussForces)
        includeValue(ranges.axialForce, value.axial);
    for (const CableForceRecord& value : cableForces)
        includeValue(ranges.axialForce, value.axial);
    forces.clear();
    forces.shrink_to_fit();

    H5Handle stressType = CreateElementStressType();
    std::vector<ElementStressRecord> stresses;
    if (!stressType.valid()
        || !ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/STRESS", stressType, stresses))
    {
        return ReportHdf5FormatError();
    }
    for (const ElementStressRecord& value : stresses)
        includeValue(ranges.stress, value.currentStress);
    stresses.clear();
    stresses.shrink_to_fit();

    H5Handle strainType = CreateElementStrainType();
    std::vector<ElementStrainRecord> strains;
    if (!strainType.valid()
        || !ReadDatasetAll(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/STRAIN", strainType, strains))
    {
        return ReportHdf5FormatError();
    }
    for (const ElementStrainRecord& value : strains)
        includeValue(ranges.strain, value.strain);

    return ranges.displacementMagnitude.valid || ranges.displacementX.valid || ranges.displacementY.valid
        || ranges.displacementZ.valid || ranges.axialForce.valid || ranges.stress.valid || ranges.strain.valid;
}

bool Hdf5ModelIO::ReadResultFrame(int frameIndex, Hdf5ResultFrame& frame) const
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    H5ReadErrorScope errorScope;
    frame = {};
    if (!m_impl->resultFile.valid() || frameIndex < 0
        || frameIndex >= static_cast<int>(m_impl->resultDomains.size()))
        return false;

    const DomainRecord& domain = m_impl->resultDomains[static_cast<std::size_t>(frameIndex)];
    const IndexRecord displacementInfo = m_impl->displacementIndex.value(domain.id);
    const IndexRecord coordinateInfo = m_impl->currentCoordinateIndex.value(domain.id);
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
    std::vector<ElementForceRecord> forces;
    std::vector<TrussForceRecord> trussForces;
    std::vector<CableForceRecord> cableForces;
    std::vector<ElementStressRecord> stresses;
    std::vector<ElementStrainRecord> strains;
    if (!nodalType.valid() || !forceType.valid() || !trussForceType.valid() || !cableForceType.valid()
        || !stressType.valid() || !strainType.valid()
        || !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/NODAL/DISPLACEMENT",
            nodalType, displacementInfo.position, displacementInfo.length, displacements)
        || !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/NODAL/CURRENT_COORDINATE",
            nodalType, coordinateInfo.position, coordinateInfo.length, coordinates)
        || !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE",
            forceType, forceInfo.position, forceInfo.length, forces)
        || !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/TRUSS_FORCE",
            trussForceType, trussForceInfo.position, trussForceInfo.length, trussForces)
        || !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/CABLE_FORCE",
            cableForceType, cableForceInfo.position, cableForceInfo.length, cableForces)
        || !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/STRESS",
            stressType, stressInfo.position, stressInfo.length, stresses)
        || !ReadDatasetBlock(m_impl->resultFile, "/YQY/RESULT/ELEMENTAL/STRAIN",
            strainType, strainInfo.position, strainInfo.length, strains))
        return ReportHdf5FormatError();

    frame.info = { domain.id, domain.stepId, domain.increment, domain.analysis,
        domain.time, domain.loadFactor };
    const QHash<int, NodalRecord> coordinateMap = BuildRecordMap(coordinates);
    frame.nodes.reserve(displacements.size());
    for (const NodalRecord& value : displacements)
    {
        Hdf5NodalResult result;
        result.id = value.id;
        result.displacement[0] = value.x;
        result.displacement[1] = value.y;
        result.displacement[2] = value.z;
        const NodalRecord coordinate = coordinateMap.value(value.id);
        result.currentCoordinate[0] = coordinate.x;
        result.currentCoordinate[1] = coordinate.y;
        result.currentCoordinate[2] = coordinate.z;
        frame.nodes.push_back(result);
    }

    QHash<int, Hdf5ElementResult> elementMap;
    for (const ElementForceRecord& value : forces)
    {
        auto& result = elementMap[value.id];
        result.id = value.id;
        result.axialForce = value.axial;
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
    }
    for (const ElementStressRecord& value : stresses)
    {
        auto& result = elementMap[value.id];
        result.id = value.id;
        result.currentStress = value.currentStress;
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

void Hdf5ModelIO::CloseResultFile()
{
    QMutexLocker locker(&g_hdf5ApiMutex);
    m_impl->CloseResult();
}
