#pragma once
#include <vector>

#include <string>
#include <filesystem>
#include <map>

// ============ 控制台颜色支持 ============
const int COLOR_GREEN = 10;
const int COLOR_RED = 12;
const int COLOR_WHITE = 7;

// 颜色函数声明
void enableConsoleColor();
void setColor(int color);
void resetColor();
void ConsoleOut(const std::wstring& text);
void ConsoleError(const std::wstring& text);

// ============ 枚举类型 ============
enum CoefType
{
    LIFT = 0,
    DRAG = 1,
    MOMENT = 2
};

// ============ 数据结构 ============
struct BladeModel
{
    std::vector<double> lift;
    std::vector<double> drag;
    std::vector<double> moment;
};

struct AeroCoefficients
{
    double lift = 0.0;
    double drag = 0.0;
    double moment = 0.0;
};

struct AeroCaseKey
{
    int bundleCount = 1;     // 分裂数
    int windSpeed = 0;       // 风速，单位 m/s
    int iceThickness = 0;    // 覆冰厚度，单位 mm

    bool operator<(const AeroCaseKey& other) const
    {
        if (bundleCount != other.bundleCount) return bundleCount < other.bundleCount;
        if (windSpeed != other.windSpeed) return windSpeed < other.windSpeed;
        return iceThickness < other.iceThickness;
    }

    bool operator==(const AeroCaseKey& other) const
    {
        return bundleCount == other.bundleCount
            && windSpeed == other.windSpeed
            && iceThickness == other.iceThickness;
    }
};

// ============ 气动数据管理器类 ============
class AeroManager
{
private:
    std::vector<BladeModel> models;
    std::map<AeroCaseKey, std::vector<BladeModel>> caseModels;
    std::filesystem::path currentSourceFile;
    std::map<AeroCaseKey, std::filesystem::path> caseSourceFiles;
    const double START_VAL = 0.0;
    const double STEP = 5.0;

    // 私有辅助函数
    std::vector<std::string> split(const std::string& s, char delimiter) const;
    bool loadModelsFromCSV(const std::filesystem::path& filepath, std::vector<BladeModel>& outModels) const;
    double interpolate(const std::vector<double>& yValues, double inputX) const;
    double parseDouble(const std::string& str) const;
    bool removeUTF8BOM(std::string& line) const;
    double getDataFromModels(const std::vector<BladeModel>& data, int modelIdx, CoefType type, double inputAngle) const;

public:
    AeroManager() = default;
    AeroManager(const AeroManager& other)
        : models(other.models), caseModels(other.caseModels),
          currentSourceFile(other.currentSourceFile), caseSourceFiles(other.caseSourceFiles)
    {
    }
    AeroManager& operator=(const AeroManager& other)
    {
        if (this != &other)
        {
            models = other.models;
            caseModels = other.caseModels;
            currentSourceFile = other.currentSourceFile;
            caseSourceFiles = other.caseSourceFiles;
        }
        return *this;
    }

    static std::filesystem::path buildChineseFileName(const AeroCaseKey& key);
    static std::filesystem::path buildLegacyFileName(const AeroCaseKey& key);
    static const std::vector<int>& supportedBundleCounts();
    static const std::vector<int>& supportedWindSpeeds();
    static const std::vector<int>& supportedIceThicknesses();
    static bool isSupportedWindSpeed(int windSpeed);
    static bool isSupportedIceThickness(int iceThickness);
    static bool isSupportedCase(const AeroCaseKey& key);
    static double normalizeAngleDegrees(double angleDegrees);

    // 加载CSV数据
    bool loadCSV(const std::filesystem::path& filepath);

    // 按分裂数、风速、覆冰厚度加载并缓存气动参数
    bool loadCase(const std::filesystem::path& dataDir, const AeroCaseKey& key);
    bool loadAllCases(const std::filesystem::path& dataDir);
    bool hasCase(const AeroCaseKey& key) const;
    int getLoadedCaseCount() const;

    // 获取插值数据
    double getData(int modelIdx, CoefType type, double inputAngle) const;
    double getData(const AeroCaseKey& key, int modelIdx, CoefType type, double inputAngle) const;
    const std::vector<BladeModel>* findCaseModels(const AeroCaseKey& key) const;
    AeroCoefficients getCoefficients(
        const std::vector<BladeModel>& caseData, int modelIdx, double inputAngle) const;
    AeroCoefficients getCoefficients(
        const AeroCaseKey& key, int modelIdx, double inputAngle) const;
    void setCaseData(const AeroCaseKey& key, std::vector<BladeModel> caseData,
        const std::filesystem::path& sourceFile = {});

    // 获取模型信息
    int getModelCount() const;
    int getDataSize(int modelIdx) const;
    double getStartAngle() const;
    double getAngleStep() const;
    const std::vector<BladeModel>& getModels() const;
    const std::map<AeroCaseKey, std::vector<BladeModel>>& getCaseModels() const;
    const std::filesystem::path& getCurrentSourceFile() const;
    std::filesystem::path getCaseSourceFile(const AeroCaseKey& key) const;

    // 导出数据
    bool exportToCSV(const std::filesystem::path& filepath, int precision = 10) const;

    // 验证功能
    bool ValiDateAllCSV(const std::filesystem::path& outputDir, const std::filesystem::path& standardDir, double tolerance = 0.000001) const;
};
