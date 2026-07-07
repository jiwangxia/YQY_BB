#include "Hdf5ResultIO.h"

#include "DataStructure/Structure/StructureData.h"

#ifndef H5_BUILT_AS_DYNAMIC_LIB
#define H5_BUILT_AS_DYNAMIC_LIB
#endif
#include <vtk_hdf5.h>

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QHash>
#include <QTextStream>
#include <QUuid>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <memory>
#include <vector>

namespace
{
constexpr int kModelDomainId = 1;

QString MakeAsciiTempHdf5FileName()
{
    return QDir::temp().filePath("yqy_h5_" + QUuid::createUuid().toString(QUuid::Id128) + ".h5");
}

QByteArray ToHdf5Path(const QString& fileName)
{
    return QDir::toNativeSeparators(fileName).toUtf8();
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
    H5Handle(hid_t value, herr_t(*func)(hid_t)) : id(value), closeFunc(func) {}
    ~H5Handle() { reset(); }

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

    bool valid() const { return id >= 0; }
    operator hid_t() const { return id; }
};

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
    H5Handle aeroCaseType = CreateAeroCaseType();
    H5Handle aeroCoefficientType = CreateAeroCoefficientType();
    std::vector<AeroCaseRecord> aeroCases;
    std::vector<AeroCoefficientRecord> aeroCoefficients;
    BuildAeroRecords(pData, aeroCases, aeroCoefficients);

    return gridType.valid()
        && materialType.valid()
        && sectionType.valid()
        && propertyType.valid()
        && elementType.valid()
        && constraintType.valid()
        && loadType.valid()
        && stepType.valid()
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
        && WriteDataset(file, "/YQY/INPUT/AERO/CASE", aeroCaseType, aeroCases)
        && WriteDataset(file, "/YQY/INPUT/AERO/COEFFICIENT", aeroCoefficientType, aeroCoefficients);
}

bool WriteResultData(hid_t file, const StructureData* pData)
{
    const auto& frames = pData->GetOutputter().GetDataSet();

    std::vector<DomainRecord> domains;
    std::vector<NodalRecord> displacements;
    std::vector<NodalRecord> currentCoordinates;
    std::vector<NodalRecord> velocities;
    std::vector<NodalRecord> accelerations;
    std::vector<NodalRecord> reactions;
    std::vector<ElementForceRecord> elementForces;
    std::vector<ElementStressRecord> elementStresses;
    std::vector<ElementStrainRecord> elementStrains;

    std::vector<IndexRecord> displacementIndex;
    std::vector<IndexRecord> currentCoordinateIndex;
    std::vector<IndexRecord> velocityIndex;
    std::vector<IndexRecord> accelerationIndex;
    std::vector<IndexRecord> reactionIndex;
    std::vector<IndexRecord> elementForceIndex;
    std::vector<IndexRecord> elementStressIndex;
    std::vector<IndexRecord> elementStrainIndex;

    domains.reserve(frames.size());
    int domainId = 1;
    int increment = 0;
    for (const DataFrame& frame : frames)
    {
        DomainRecord domain;
        domain.id = domainId;
        domain.increment = increment;
        domain.time = frame.GetTime();
        domains.push_back(domain);

        const long long nodePosition = static_cast<long long>(displacements.size());
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

            NodalRecord reaction;
            reaction.id = nodeId;
            reaction.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R1);
            reaction.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R2);
            reaction.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R3);
            reaction.domainId = domainId;
            reactions.push_back(reaction);
        }

        const long long nodeLength = static_cast<long long>(frame.GetNodeDatas().size());
        AppendFrameIndex(displacementIndex, domainId, nodePosition, nodeLength);
        AppendFrameIndex(currentCoordinateIndex, domainId, nodePosition, nodeLength);
        AppendFrameIndex(velocityIndex, domainId, nodePosition, nodeLength);
        AppendFrameIndex(accelerationIndex, domainId, nodePosition, nodeLength);
        AppendFrameIndex(reactionIndex, domainId, nodePosition, nodeLength);

        const long long elementPosition = static_cast<long long>(elementForces.size());
        for (const auto& pair : frame.GetElementDatas())
        {
            const int elementId = pair.first;

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

        const long long elementLength = static_cast<long long>(frame.GetElementDatas().size());
        AppendFrameIndex(elementForceIndex, domainId, elementPosition, elementLength);
        AppendFrameIndex(elementStressIndex, domainId, elementPosition, elementLength);
        AppendFrameIndex(elementStrainIndex, domainId, elementPosition, elementLength);

        ++domainId;
        ++increment;
    }

    H5Handle domainType = CreateDomainType();
    H5Handle indexType = CreateIndexType();
    H5Handle nodalType = CreateNodalType();
    H5Handle elementForceType = CreateElementForceType();
    H5Handle elementStressType = CreateElementStressType();
    H5Handle elementStrainType = CreateElementStrainType();

    return domainType.valid()
        && indexType.valid()
        && nodalType.valid()
        && elementForceType.valid()
        && elementStressType.valid()
        && elementStrainType.valid()
        && WriteDataset(file, "/YQY/RESULT/DOMAINS", domainType, domains)
        && WriteDataset(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, displacements)
        && WriteDataset(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType, currentCoordinates)
        && WriteDataset(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType, velocities)
        && WriteDataset(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType, accelerations)
        && WriteDataset(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType, reactions)
        && WriteDataset(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType, elementForces)
        && WriteDataset(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType, elementStresses)
        && WriteDataset(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType, elementStrains)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType, displacementIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType, currentCoordinateIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType, velocityIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType, accelerationIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType, reactionIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType, elementForceIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType, elementStressIndex)
        && WriteDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType, elementStrainIndex);
}
}

class Hdf5ResultIO::Impl
{
public:
    H5Handle file;
    QString targetFileName;
    QString tempFileName;

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
            && WriteIntAttribute(file, "VERSION", 1)
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
        H5Handle elementStressType = CreateElementStressType();
        H5Handle elementStrainType = CreateElementStrainType();

        return domainType.valid()
            && indexType.valid()
            && nodalType.valid()
            && elementForceType.valid()
            && elementStressType.valid()
            && elementStrainType.valid()
            && CreateExtendableDataset(file, "/YQY/RESULT/DOMAINS", domainType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType)
            && CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType)
            && CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType)
            && CreateExtendableDataset(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType)
            && CreateExtendableDataset(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType)
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

            NodalRecord reaction;
            reaction.id = nodeId;
            reaction.x = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R1);
            reaction.y = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R2);
            reaction.z = frame.GetNodeData(nodeId, EnumKeyword::NodeResultType::R3);
            reaction.domainId = domainId;
            reactions.push_back(reaction);
        }

        std::vector<ElementForceRecord> elementForces;
        std::vector<ElementStressRecord> elementStresses;
        std::vector<ElementStrainRecord> elementStrains;
        elementForces.reserve(frame.GetElementDatas().size());
        elementStresses.reserve(frame.GetElementDatas().size());
        elementStrains.reserve(frame.GetElementDatas().size());

        for (const auto& pair : frame.GetElementDatas())
        {
            const int elementId = pair.first;

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
        H5Handle elementStressType = CreateElementStressType();
        H5Handle elementStrainType = CreateElementStrainType();
        if (!indexType.valid() || !nodalType.valid() || !elementForceType.valid()
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

    void End()
    {
        if (file.valid())
        {
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

Hdf5ResultIO::Hdf5ResultIO()
    : m_impl(std::make_unique<Impl>())
{
}

Hdf5ResultIO::~Hdf5ResultIO() = default;

bool Hdf5ResultIO::ExportHdf5(const QString& fileName, const StructureData* pData) const
{
    return ExportHdf5(fileName, pData, QString());
}

bool Hdf5ResultIO::ExportHdf5(const QString& fileName, const StructureData* pData, const QString& sourceModelName) const
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

    const QString targetFileName = fileInfo.absoluteFilePath();
    const QString tempFileName = MakeAsciiTempHdf5FileName();
    const QByteArray path = ToHdf5Path(tempFileName);
    H5Handle file(H5Fcreate(path.constData(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT), H5Fclose);
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
        && WriteIntAttribute(file, "VERSION", 1)
        && WriteStringAttribute(file, "CREATED_TIME", QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
        && WriteStringAttribute(file, "PROGRAM", "YQY_CAE")
        && WriteStringAttribute(file, "SOURCE_MODEL", sourceModelName);

    const bool dataOk = WriteInputData(file, pData) && WriteResultData(file, pData);
    if (!attrOk || !dataOk)
    {
        file.reset();
        QFile::remove(tempFileName);
        qDebug() << "Failed to write HDF5 data:" << fileName;
        return false;
    }

    qDebug().noquote() << QStringLiteral("H5/HDF5 文件已输出至") << fileName;
    file.reset();
    return MoveTempFileToTarget(tempFileName, targetFileName);
}

bool Hdf5ResultIO::BeginResultStream(const QString& fileName, const StructureData* pData, const QString& sourceModelName)
{
    return m_impl->Begin(fileName, pData, sourceModelName);
}

bool Hdf5ResultIO::WriteResultFrame(int domainId, int stepId, int increment, int analysis, double time, const DataFrame& frame)
{
    return m_impl->WriteFrame(domainId, stepId, increment, analysis, time, frame);
}

void Hdf5ResultIO::EndResultStream()
{
    m_impl->End();
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

bool Hdf5ResultIO::ExportBdfResultFromHdf5(const QString& hdf5FileName,
    const QString& bdfFileName,
    const std::vector<int>& nodeIds,
    const std::vector<EnumKeyword::NodeResultType>& nodeTypes,
    const std::vector<int>& elementIds,
    const std::vector<EnumKeyword::ElementResultType>& elementTypes) const
{
    const QString tempHdf5FileName = CopyHdf5FileToAsciiTemp(QFileInfo(hdf5FileName).absoluteFilePath());
    if (tempHdf5FileName.isEmpty())
    {
        qDebug() << "Failed to copy HDF5 file to temp path:" << hdf5FileName;
        return false;
    }

    const QByteArray path = ToHdf5Path(tempHdf5FileName);
    H5Handle file(H5Fopen(path.constData(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose);
    if (!file.valid())
    {
        QFile::remove(tempHdf5FileName);
        qDebug() << "Failed to open HDF5 file:" << hdf5FileName;
        return false;
    }

    QFile outFile(bdfFileName);
    if (!outFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open BDF result file:" << bdfFileName;
        return false;
    }

    H5Handle domainType = CreateDomainType();
    H5Handle indexType = CreateIndexType();
    H5Handle nodalType = CreateNodalType();
    H5Handle elementForceType = CreateElementForceType();
    H5Handle elementStressType = CreateElementStressType();
    H5Handle elementStrainType = CreateElementStrainType();
    if (!domainType.valid() || !indexType.valid() || !nodalType.valid()
        || !elementForceType.valid() || !elementStressType.valid() || !elementStrainType.valid())
    {
        return false;
    }

    std::vector<DomainRecord> domains;
    std::vector<IndexRecord> displacementIndexRecords;
    std::vector<IndexRecord> currentCoordinateIndexRecords;
    std::vector<IndexRecord> velocityIndexRecords;
    std::vector<IndexRecord> accelerationIndexRecords;
    std::vector<IndexRecord> reactionIndexRecords;
    std::vector<IndexRecord> elementForceIndexRecords;
    std::vector<IndexRecord> elementStressIndexRecords;
    std::vector<IndexRecord> elementStrainIndexRecords;

    if (!ReadDatasetAll(file, "/YQY/RESULT/DOMAINS", domainType, domains)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/DISPLACEMENT", indexType, displacementIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/CURRENT_COORDINATE", indexType, currentCoordinateIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/VELOCITY", indexType, velocityIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/ACCELERATION", indexType, accelerationIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/NODAL/REACTION_FORCE", indexType, reactionIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", indexType, elementForceIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRESS", indexType, elementStressIndexRecords)
        || !ReadDatasetAll(file, "/INDEX/YQY/RESULT/ELEMENTAL/STRAIN", indexType, elementStrainIndexRecords))
    {
        return false;
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
        std::vector<ElementStressRecord> stressRecords;
        std::vector<ElementStrainRecord> strainRecords;

        const IndexRecord displacementInfo = displacementIndex.value(domain.id);
        const IndexRecord currentCoordinateInfo = currentCoordinateIndex.value(domain.id);
        const IndexRecord velocityInfo = velocityIndex.value(domain.id);
        const IndexRecord accelerationInfo = accelerationIndex.value(domain.id);
        const IndexRecord reactionInfo = reactionIndex.value(domain.id);
        const IndexRecord forceInfo = elementForceIndex.value(domain.id);
        const IndexRecord stressInfo = elementStressIndex.value(domain.id);
        const IndexRecord strainInfo = elementStrainIndex.value(domain.id);

        if (!ReadDatasetBlock(file, "/YQY/RESULT/NODAL/DISPLACEMENT", nodalType, displacementInfo.position, displacementInfo.length, displacementRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/CURRENT_COORDINATE", nodalType, currentCoordinateInfo.position, currentCoordinateInfo.length, currentCoordinateRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/VELOCITY", nodalType, velocityInfo.position, velocityInfo.length, velocityRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/ACCELERATION", nodalType, accelerationInfo.position, accelerationInfo.length, accelerationRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/NODAL/REACTION_FORCE", nodalType, reactionInfo.position, reactionInfo.length, reactionRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/ELEMENT_FORCE", elementForceType, forceInfo.position, forceInfo.length, forceRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/STRESS", elementStressType, stressInfo.position, stressInfo.length, stressRecords)
            || !ReadDatasetBlock(file, "/YQY/RESULT/ELEMENTAL/STRAIN", elementStrainType, strainInfo.position, strainInfo.length, strainRecords))
        {
            return false;
        }

        const QHash<int, NodalRecord> displacements = BuildRecordMap(displacementRecords);
        const QHash<int, NodalRecord> currentCoordinates = BuildRecordMap(currentCoordinateRecords);
        const QHash<int, NodalRecord> velocities = BuildRecordMap(velocityRecords);
        const QHash<int, NodalRecord> accelerations = BuildRecordMap(accelerationRecords);
        const QHash<int, NodalRecord> reactions = BuildRecordMap(reactionRecords);
        const QHash<int, ElementForceRecord> forces = BuildRecordMap(forceRecords);
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

    qDebug().noquote() << QStringLiteral("H5/HDF5 结果已转换输出至") << bdfFileName;
    file.reset();
    QFile::remove(tempHdf5FileName);
    return true;
}

bool Hdf5ResultIO::ImportHdf5(const QString& fileName, StructureData* pData) const
{
    Q_UNUSED(pData);

    if (fileName.isEmpty())
    {
        return false;
    }

    const QString tempHdf5FileName = CopyHdf5FileToAsciiTemp(QFileInfo(fileName).absoluteFilePath());
    if (tempHdf5FileName.isEmpty())
    {
        qDebug() << "Failed to copy HDF5 file to temp path:" << fileName;
        return false;
    }

    const QByteArray path = ToHdf5Path(tempHdf5FileName);
    H5Handle file(H5Fopen(path.constData(), H5F_ACC_RDONLY, H5P_DEFAULT), H5Fclose);
    if (!file.valid())
    {
        QFile::remove(tempHdf5FileName);
        qDebug() << "Failed to open HDF5 file:" << fileName;
        return false;
    }

    qDebug().noquote() << QStringLiteral("H5/HDF5 文件读取接口已预留，模型对象恢复将在后续实现。");
    file.reset();
    QFile::remove(tempHdf5FileName);
    return false;
}
