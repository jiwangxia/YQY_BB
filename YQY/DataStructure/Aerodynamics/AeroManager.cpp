#include "AeroManager.h"

#include "Utility/Logger/Logger.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

namespace
{
std::string WideToUtf8(const std::wstring& text)
{
    if (text.empty())
    {
        return {};
    }

    const int size =
        WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
    std::string result(size, '\0');
    WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), result.data(), size, nullptr, nullptr);
    return result;
}

void WriteConsoleText(DWORD handleId, FILE* fallbackStream, const std::wstring& text)
{
    HANDLE handle = GetStdHandle(handleId);
    DWORD mode = 0;
    if (handle != INVALID_HANDLE_VALUE && GetConsoleMode(handle, &mode))
    {
        DWORD written = 0;
        WriteConsoleW(handle, text.c_str(), static_cast<DWORD>(text.size()), &written, nullptr);
        return;
    }

    const std::string utf8 = WideToUtf8(text);
    std::fwrite(utf8.data(), 1, utf8.size(), fallbackStream);
    std::fflush(fallbackStream);
}

double MaximumPeriodicEndpointGap(const std::vector<BladeModel>& models)
{
    double maximumGap = 0.0;
    for (const BladeModel& model : models)
    {
        for (const std::vector<double>* values : {&model.lift, &model.drag, &model.moment})
            if (values->size() == 73)
                maximumGap = std::max(maximumGap, std::abs(values->front() - values->back()));
    }
    return maximumGap;
}

double PeriodicSample(const std::vector<double>& values, std::size_t index)
{
    // 模型和导出的 H5 保留实测 360 度样本；
    // 仅在插值时使用 0 度样本闭合最后一个区间。
    return values.size() == 73 && index == 72 ? values.front() : values[index];
}
}

static double ShapePreservingSlope(double leftChord, double rightChord)
{
    if (leftChord * rightChord <= 0.0)
        return 0.0;
    return 2.0 * leftChord * rightChord / (leftChord + rightChord);
}

static double InterpolatePeriodicShapePreserving(const std::vector<double>& values, double angleDegrees)
{
    constexpr double step = 5.0;
    const std::size_t period = values.size() - 1;
    const double exactPosition = angleDegrees / step;
    const std::size_t first = static_cast<std::size_t>(std::floor(exactPosition)) % period;
    const std::size_t second = (first + 1) % period;
    const std::size_t previous = (first + period - 1) % period;
    const std::size_t following = (second + 1) % period;
    const double ratio = exactPosition - std::floor(exactPosition);
    const double leftChord = (values[first] - values[previous]) / step;
    const double middleChord = (values[second] - values[first]) / step;
    const double rightChord = (values[following] - values[second]) / step;
    const double firstSlope = ShapePreservingSlope(leftChord, middleChord);
    const double secondSlope = ShapePreservingSlope(middleChord, rightChord);
    const double ratioSquared = ratio * ratio;
    const double ratioCubed = ratioSquared * ratio;
    return (2.0 * ratioCubed - 3.0 * ratioSquared + 1.0) * values[first] +
           (ratioCubed - 2.0 * ratioSquared + ratio) * step * firstSlope +
           (-2.0 * ratioCubed + 3.0 * ratioSquared) * values[second] +
           (ratioCubed - ratioSquared) * step * secondSlope;
}

static double InterpolatePeriodicShapePreservingSlope(const std::vector<double>& values, double angleDegrees)
{
    constexpr double step = 5.0;
    const std::size_t period = values.size() - 1;
    const double exactPosition = angleDegrees / step;
    const std::size_t first = static_cast<std::size_t>(std::floor(exactPosition)) % period;
    const std::size_t second = (first + 1) % period;
    const std::size_t previous = (first + period - 1) % period;
    const std::size_t following = (second + 1) % period;
    const double ratio = exactPosition - std::floor(exactPosition);
    const double leftChord = (values[first] - values[previous]) / step;
    const double middleChord = (values[second] - values[first]) / step;
    const double rightChord = (values[following] - values[second]) / step;
    const double firstSlope = ShapePreservingSlope(leftChord, middleChord);
    const double secondSlope = ShapePreservingSlope(middleChord, rightChord);
    const double ratioSquared = ratio * ratio;
    return ((6.0 * ratioSquared - 6.0 * ratio) * values[first] +
            (3.0 * ratioSquared - 4.0 * ratio + 1.0) * step * firstSlope +
            (-6.0 * ratioSquared + 6.0 * ratio) * values[second] +
            (3.0 * ratioSquared - 2.0 * ratio) * step * secondSlope) /
           step;
}

void ConsoleOut(const std::wstring& text)
{
    WriteConsoleText(STD_OUTPUT_HANDLE, stdout, text);
}

void ConsoleError(const std::wstring& text)
{
    WriteConsoleText(STD_ERROR_HANDLE, stderr, text);
}

// ============ AeroManager 私有函数实现 ============

std::vector<std::string> AeroManager::split(const std::string& s, char delimiter) const
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);

    while (std::getline(tokenStream, token, delimiter))
    {
        size_t first = token.find_first_not_of(" \t\r\n");
        if (std::string::npos == first)
        {
            tokens.push_back("");
            continue;
        }
        size_t last = token.find_last_not_of(" \t\r\n");
        tokens.push_back(token.substr(first, (last - first + 1)));
    }

    if (!s.empty() && s.back() == delimiter)
    {
        tokens.push_back("");
    }

    return tokens;
}

bool AeroManager::loadModelsFromCSV(const std::filesystem::path& filepath, std::vector<BladeModel>& outModels) const
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        ConsoleError(L"Error: Cannot open file " + filepath.wstring() + L"\n");
        return false;
    }

    std::string line;

    if (!std::getline(file, line))
    {
        ConsoleError(L"Error: File is empty " + filepath.wstring() + L"\n");
        return false;
    }

    removeUTF8BOM(line);

    std::vector<std::string> headers = split(line, ',');
    size_t totalCols = headers.size();

    if (totalCols == 0)
    {
        ConsoleError(L"Error: CSV header is empty " + filepath.wstring() + L"\n");
        return false;
    }

    size_t numModels = 0;
    int startColIndex = 0;

    if (totalCols % 3 == 0)
    {
        numModels = totalCols / 3;
        startColIndex = 0;
    }
    else if (totalCols > 1 && (totalCols - 1) % 3 == 0)
    {
        numModels = (totalCols - 1) / 3;
        startColIndex = 1;
    }
    else
    {
        ConsoleError(L"Error: Invalid CSV format, cols=" + std::to_wstring(totalCols) + L" file=" + filepath.wstring() +
                     L"\n");
        return false;
    }
    for (size_t model = 0; model < numModels; ++model)
    {
        const size_t base = static_cast<size_t>(startColIndex) + 3 * model;
        const std::string suffix = std::to_string(model + 1);
        if (headers[base] != "CL" + suffix || headers[base + 1] != "CD" + suffix || headers[base + 2] != "CM" + suffix)
        {
            ConsoleError(L"Error: Aerodynamic columns must be ordered CL/CD/CM at model " + std::to_wstring(model + 1) +
                         L" file=" + filepath.wstring() + L"\n");
            return false;
        }
    }

    std::vector<BladeModel> parsedModels(numModels);

    int rowCount = 0;
    int lineNumber = 1;
    const auto trim = [](const std::string& value)
    {
        const size_t first = value.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
            return std::string{};
        const size_t last = value.find_last_not_of(" \t\r\n");
        return value.substr(first, last - first + 1);
    };
    const auto tryParseFinite = [&trim](const std::string& value, double& parsed)
    {
        const std::string cleaned = trim(value);
        if (cleaned.empty() || cleaned == "--" || cleaned == "N/A" || cleaned == "NA")
            return false;
        try
        {
            size_t consumed = 0;
            parsed = std::stod(cleaned, &consumed);
            return consumed == cleaned.size() && std::isfinite(parsed);
        }
        catch (...)
        {
            return false;
        }
    };
    while (std::getline(file, line))
    {
        lineNumber++;
        if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos)
        {
            continue;
        }

        std::vector<std::string> vals = split(line, ',');

        if (vals.size() != totalCols)
        {
            ConsoleError(L"Warning: Skip line " + std::to_wstring(lineNumber) + L", expected cols=" +
                         std::to_wstring(totalCols) + L", actual cols=" + std::to_wstring(vals.size()) + L"\n");
            continue;
        }

        bool allCoefficientCellsEmpty = true;
        for (size_t column = static_cast<size_t>(startColIndex); column < vals.size(); ++column)
        {
            if (!trim(vals[column]).empty())
            {
                allCoefficientCellsEmpty = false;
                break;
            }
        }
        // 部分源表格包含以 ",," 结尾的空行，这些行不是气动样本，不能转换为人为零值数据。
        if (allCoefficientCellsEmpty)
            continue;

        if (startColIndex == 1)
        {
            double angle = 0.0;
            if (!tryParseFinite(vals[0], angle) || std::abs(angle - rowCount * STEP) > 1.0e-9)
            {
                ConsoleError(L"Error: Invalid aerodynamic angle at line " + std::to_wstring(lineNumber) + L" file=" +
                             filepath.wstring() + L"\n");
                return false;
            }
        }

        for (size_t i = 0; i < numModels; i++)
        {
            size_t base = startColIndex + (i * 3);
            double lift = 0.0;
            double drag = 0.0;
            double moment = 0.0;
            if (!tryParseFinite(vals[base + 0], lift) || !tryParseFinite(vals[base + 1], drag) ||
                !tryParseFinite(vals[base + 2], moment))
            {
                ConsoleError(L"Error: Missing or invalid aerodynamic coefficient at line " +
                             std::to_wstring(lineNumber) + L" file=" + filepath.wstring() + L"\n");
                return false;
            }
            parsedModels[i].lift.push_back(lift);
            parsedModels[i].drag.push_back(drag);
            parsedModels[i].moment.push_back(moment);
        }
        rowCount++;
    }

    constexpr int expectedRows = 73; // 0..360 度，包含两端，间隔 5 度
    if (rowCount != expectedRows)
    {
        ConsoleError(L"Error: Expected " + std::to_wstring(expectedRows) + L" aerodynamic rows, got " +
                     std::to_wstring(rowCount) + L" file=" + filepath.wstring() + L"\n");
        return false;
    }

    const double endpointGap = MaximumPeriodicEndpointGap(parsedModels);
    if (endpointGap > 1.0e-3)
    {
        std::wostringstream warning;
        warning << L"Warning: Aerodynamic 0/360-degree samples differ by up to " << std::setprecision(8) << endpointGap
                << L" in " << (Logger::DetailedOutputEnabled() ? filepath.wstring() : filepath.filename().wstring())
                << (Logger::DetailedOutputEnabled()
                        ? L"; raw samples are preserved, while interpolation closes 355 to 0 degrees.\n"
                        : L"; using periodic interpolation.\n");
        ConsoleError(warning.str());
    }
    outModels = std::move(parsedModels);
    return true;
}

double AeroManager::interpolate(const std::vector<double>& yValues, double inputX) const
{
    if (yValues.empty())
    {
        return 0.0;
    }

    double exactPos = (inputX - START_VAL) / STEP;
    int idx = static_cast<int>(std::floor(exactPos));
    double ratio = exactPos - idx;

    if (idx < 0)
    {
        return yValues.front();
    }
    if (idx >= static_cast<int>(yValues.size()) - 1)
    {
        return yValues.back();
    }

    return yValues[idx] + (PeriodicSample(yValues, static_cast<std::size_t>(idx + 1)) - yValues[idx]) * ratio;
}

double AeroManager::parseDouble(const std::string& str) const
{
    std::string trimmed = str;
    size_t first = trimmed.find_first_not_of(" \t\r\n");

    if (first == std::string::npos)
    {
        return 0.0;
    }

    size_t last = trimmed.find_last_not_of(" \t\r\n");
    trimmed = trimmed.substr(first, (last - first + 1));

    if (trimmed.empty() || trimmed == "--" || trimmed == "N/A" || trimmed == "NA")
    {
        return 0.0;
    }

    try
    {
        return std::stod(trimmed);
    }
    catch (...)
    {
        return 0.0;
    }
}

bool AeroManager::removeUTF8BOM(_OUT std::string& line) const
{
    if (line.size() >= 3 && (unsigned char)line[0] == 0xEF && (unsigned char)line[1] == 0xBB &&
        (unsigned char)line[2] == 0xBF)
    {
        line = line.substr(3);
        return true;
    }
    return false;
}

double AeroManager::getDataFromModels(const std::vector<BladeModel>& data, int modelIdx, CoefType type,
                                      double inputAngle) const
{
    if (modelIdx < 0 || modelIdx >= static_cast<int>(data.size()))
    {
        std::cerr << "Error: Model index out of range (" << modelIdx << ")\n";
        return 0.0;
    }

    const std::vector<double>* vec = nullptr;
    switch (type)
    {
        case LIFT:
            vec = &data[modelIdx].lift;
            break;
        case DRAG:
            vec = &data[modelIdx].drag;
            break;
        case MOMENT:
            vec = &data[modelIdx].moment;
            break;
        default:
            return 0.0;
    }

    if (vec->empty())
        return 0.0;

    const double angle = normalizeAngleDegrees(inputAngle);

    size_t n = vec->size();
    double maxAngle = (n - 1) * STEP;

    if (angle <= maxAngle)
    {
        return interpolate(*vec, angle);
    }

    double y_first = vec->front();
    double y_last = vec->back();
    double t = (angle - maxAngle) / (360.0 - maxAngle);
    return y_last + (y_first - y_last) * t;
}

// ============ AeroManager 公共函数实现 ============
std::filesystem::path AeroManager::buildChineseFileName(const AeroCaseKey& key)
{
    return std::filesystem::path(std::to_wstring(key.bundleCount) + L"分裂-风速" + std::to_wstring(key.windSpeed) +
                                 L"-厚度" + std::to_wstring(key.iceThickness) + L".csv");
}

std::filesystem::path AeroManager::buildLegacyFileName(const AeroCaseKey& key)
{
    return std::to_string(key.bundleCount) + "-" + std::to_string(key.windSpeed) + "ms-" +
           std::to_string(key.iceThickness) + "mm.csv";
}

const std::vector<int>& AeroManager::supportedBundleCounts()
{
    static const std::vector<int> values{1, 4};
    return values;
}

const std::vector<int>& AeroManager::supportedWindSpeeds()
{
    static const std::vector<int> values{10, 12, 14, 18};
    return values;
}

const std::vector<int>& AeroManager::supportedIceThicknesses()
{
    static const std::vector<int> values{12, 18, 20, 25, 28};
    return values;
}

bool AeroManager::isSupportedCase(const AeroCaseKey& key)
{
    return std::find(supportedBundleCounts().cbegin(), supportedBundleCounts().cend(), key.bundleCount) !=
               supportedBundleCounts().cend() &&
           isSupportedWindSpeed(key.windSpeed) && isSupportedIceThickness(key.iceThickness);
}

bool AeroManager::isSupportedWindSpeed(int windSpeed)
{
    return std::find(supportedWindSpeeds().cbegin(), supportedWindSpeeds().cend(), windSpeed) !=
           supportedWindSpeeds().cend();
}

bool AeroManager::isSupportedIceThickness(int iceThickness)
{
    return std::find(supportedIceThicknesses().cbegin(), supportedIceThicknesses().cend(), iceThickness) !=
           supportedIceThicknesses().cend();
}

double AeroManager::normalizeAngleDegrees(double angleDegrees)
{
    if (!std::isfinite(angleDegrees))
        return 0.0;
    double normalized = std::fmod(angleDegrees, 360.0);
    if (normalized < 0.0)
        normalized += 360.0;
    // 极小负角加 360 后可能舍入为 360，仍须闭合到零度样本。
    return normalized >= 360.0 ? 0.0 : normalized;
}

void AeroManager::setShapePreservingInterpolation(bool enabled)
{
    shapePreservingInterpolation = enabled;
}

bool AeroManager::loadCSV(const std::filesystem::path& filepath)
{
    std::vector<BladeModel> parsedModels;
    if (!loadModelsFromCSV(filepath, parsedModels))
    {
        return false;
    }

    models = std::move(parsedModels);
    currentSourceFile = filepath;
    return true;
}

bool AeroManager::loadCase(const std::filesystem::path& dataDir, const AeroCaseKey& key)
{
    const auto cached = caseModels.find(key);
    if (cached != caseModels.end())
    {
        models = cached->second;
        currentSourceFile = getCaseSourceFile(key);
        return true;
    }

    const std::filesystem::path chinesePath = dataDir / buildChineseFileName(key);
    const std::filesystem::path legacyPath = dataDir / buildLegacyFileName(key);

    std::filesystem::path selectedPath;
    if (std::filesystem::exists(chinesePath))
    {
        selectedPath = chinesePath;
    }
    else if (std::filesystem::exists(legacyPath))
    {
        selectedPath = legacyPath;
    }
    else
    {
        ConsoleError(L"Error: Aero CSV not found. Tried " + chinesePath.wstring() + L" and " + legacyPath.wstring() +
                     L"\n");
        return false;
    }

    std::vector<BladeModel> parsedModels;
    if (!loadModelsFromCSV(selectedPath, parsedModels))
    {
        return false;
    }
    if (parsedModels.size() != static_cast<size_t>(key.bundleCount))
    {
        ConsoleError(L"Error: Aero profile count does not match bundle count. expected=" +
                     std::to_wstring(key.bundleCount) + L", actual=" + std::to_wstring(parsedModels.size()) +
                     L" file=" + selectedPath.wstring() + L"\n");
        return false;
    }

    models = parsedModels;
    currentSourceFile = selectedPath;
    caseModels[key] = std::move(parsedModels);
    caseSourceFiles[key] = selectedPath;
    return true;
}

bool AeroManager::loadAllCases(const std::filesystem::path& dataDir)
{
    bool allLoaded = true;
    for (int bundleCount : supportedBundleCounts())
    {
        for (int windSpeed : supportedWindSpeeds())
        {
            for (int iceThickness : supportedIceThicknesses())
            {
                if (!loadCase(dataDir, AeroCaseKey{bundleCount, windSpeed, iceThickness}))
                {
                    allLoaded = false;
                }
            }
        }
    }
    return allLoaded;
}

bool AeroManager::hasCase(const AeroCaseKey& key) const
{
    return caseModels.find(key) != caseModels.end();
}

int AeroManager::getLoadedCaseCount() const
{
    return static_cast<int>(caseModels.size());
}

double AeroManager::getData(int modelIdx, CoefType type, double inputAngle) const
{
    return getDataFromModels(models, modelIdx, type, inputAngle);
}

double AeroManager::getData(const AeroCaseKey& key, int modelIdx, CoefType type, double inputAngle) const
{
    const auto it = caseModels.find(key);
    if (it == caseModels.end())
    {
        std::cerr << "Error: Aero case not loaded (bundle=" << key.bundleCount << ", wind=" << key.windSpeed
                  << ", ice=" << key.iceThickness << ")\n";
        return 0.0;
    }

    return getDataFromModels(it->second, modelIdx, type, inputAngle);
}

const std::vector<BladeModel>* AeroManager::findCaseModels(const AeroCaseKey& key) const
{
    const auto found = caseModels.find(key);
    return found == caseModels.cend() ? nullptr : &found->second;
}

AeroCoefficients AeroManager::getCoefficients(const std::vector<BladeModel>& caseData, int modelIdx,
                                              double inputAngle) const
{
    AeroCoefficients result;
    if (modelIdx < 0 || modelIdx >= static_cast<int>(caseData.size()))
        return result;

    const BladeModel& model = caseData[modelIdx];
    if (model.lift.empty() || model.drag.empty() || model.moment.empty())
        return result;

    const double angle = normalizeAngleDegrees(inputAngle);

    const std::size_t dataSize = std::min({model.lift.size(), model.drag.size(), model.moment.size()});
    const double maxAngle = (dataSize - 1) * STEP;
    if (shapePreservingInterpolation && dataSize == 73)
    {
        result.lift = InterpolatePeriodicShapePreserving(model.lift, angle);
        result.drag = InterpolatePeriodicShapePreserving(model.drag, angle);
        result.moment = InterpolatePeriodicShapePreserving(model.moment, angle);
        return result;
    }
    if (angle <= maxAngle)
    {
        const double exactPosition = (angle - START_VAL) / STEP;
        const std::size_t firstIndex = std::min(static_cast<std::size_t>(std::floor(exactPosition)), dataSize - 1);
        const std::size_t secondIndex = std::min(firstIndex + 1, dataSize - 1);
        const double ratio = secondIndex == firstIndex ? 0.0 : exactPosition - firstIndex;
        const auto evaluate = [firstIndex, secondIndex, ratio](const std::vector<double>& values)
        {
            return values[firstIndex] + (PeriodicSample(values, secondIndex) - values[firstIndex]) * ratio;
        };
        result.lift = evaluate(model.lift);
        result.drag = evaluate(model.drag);
        result.moment = evaluate(model.moment);
    }
    else
    {
        const double ratio = (angle - maxAngle) / (360.0 - maxAngle);
        const auto evaluate = [dataSize, ratio](const std::vector<double>& values)
        {
            return values[dataSize - 1] + (values.front() - values[dataSize - 1]) * ratio;
        };
        result.lift = evaluate(model.lift);
        result.drag = evaluate(model.drag);
        result.moment = evaluate(model.moment);
    }
    return result;
}

AeroCoefficients AeroManager::getCoefficientSlopes(const std::vector<BladeModel>& caseData, int modelIdx,
                                                   double inputAngle) const
{
    AeroCoefficients result;
    if (modelIdx < 0 || modelIdx >= static_cast<int>(caseData.size()))
        return result;

    const BladeModel& model = caseData[modelIdx];
    if (model.lift.empty() || model.drag.empty() || model.moment.empty())
        return result;

    const double angle = normalizeAngleDegrees(inputAngle);
    const std::size_t dataSize = std::min({model.lift.size(), model.drag.size(), model.moment.size()});
    const double maxAngle = (dataSize - 1) * STEP;
    const double degreesPerRadian = 180.0 / std::acos(-1.0);
    if (shapePreservingInterpolation && dataSize == 73)
    {
        result.lift = degreesPerRadian * InterpolatePeriodicShapePreservingSlope(model.lift, angle);
        result.drag = degreesPerRadian * InterpolatePeriodicShapePreservingSlope(model.drag, angle);
        result.moment = degreesPerRadian * InterpolatePeriodicShapePreservingSlope(model.moment, angle);
        return result;
    }

    std::size_t first = 0;
    std::size_t second = 0;
    double width = STEP;
    if (angle <= maxAngle)
    {
        first = std::min(static_cast<std::size_t>(std::floor((angle - START_VAL) / STEP)), dataSize - 1);
        second = std::min(first + 1, dataSize - 1);
    }
    else
    {
        first = dataSize - 1;
        width = 360.0 - maxAngle;
    }

    const auto slope = [first, second, width](const std::vector<double>& values)
    {
        return (PeriodicSample(values, second) - values[first]) / width;
    };
    if (angle > maxAngle)
    {
        result.lift = degreesPerRadian * (model.lift.front() - model.lift.back()) / width;
        result.drag = degreesPerRadian * (model.drag.front() - model.drag.back()) / width;
        result.moment = degreesPerRadian * (model.moment.front() - model.moment.back()) / width;
    }
    else
    {
        result.lift = degreesPerRadian * slope(model.lift);
        result.drag = degreesPerRadian * slope(model.drag);
        result.moment = degreesPerRadian * slope(model.moment);
    }
    return result;
}

AeroCoefficients AeroManager::getCoefficients(const AeroCaseKey& key, int modelIdx, double inputAngle) const
{
    const auto* caseData = findCaseModels(key);
    return caseData ? getCoefficients(*caseData, modelIdx, inputAngle) : AeroCoefficients{};
}

void AeroManager::setCaseData(const AeroCaseKey& key, std::vector<BladeModel> caseData,
                              const std::filesystem::path& sourceFile)
{
    const double endpointGap = MaximumPeriodicEndpointGap(caseData);
    if (endpointGap > 1.0e-3)
    {
        std::wostringstream warning;
        warning << L"Warning: Aerodynamic 0/360-degree samples differ by up to " << std::setprecision(8) << endpointGap
                << L" in case " << key.bundleCount << L"/" << key.windSpeed << L"/" << key.iceThickness
                << (Logger::DetailedOutputEnabled()
                        ? L"; raw samples are preserved, while interpolation closes 355 to 0 degrees.\n"
                        : L"; using periodic interpolation.\n");
        ConsoleError(warning.str());
    }
    models = caseData;
    currentSourceFile = sourceFile;
    caseModels[key] = std::move(caseData);
    caseSourceFiles[key] = sourceFile;
}

int AeroManager::getModelCount() const
{
    return static_cast<int>(models.size());
}

int AeroManager::getDataSize(int modelIdx) const
{
    if (modelIdx < 0 || modelIdx >= static_cast<int>(models.size()))
    {
        return 0;
    }
    return static_cast<int>(models[modelIdx].lift.size());
}

double AeroManager::getStartAngle() const
{
    return START_VAL;
}

double AeroManager::getAngleStep() const
{
    return STEP;
}

const std::vector<BladeModel>& AeroManager::getModels() const
{
    return models;
}

const std::map<AeroCaseKey, std::vector<BladeModel>>& AeroManager::getCaseModels() const
{
    return caseModels;
}

const std::filesystem::path& AeroManager::getCurrentSourceFile() const
{
    return currentSourceFile;
}

std::filesystem::path AeroManager::getCaseSourceFile(const AeroCaseKey& key) const
{
    const auto it = caseSourceFiles.find(key);
    if (it == caseSourceFiles.end())
    {
        return {};
    }
    return it->second;
}

bool AeroManager::exportToCSV(const std::filesystem::path& filepath, int precision) const
{
    if (filepath.has_parent_path())
    {
        std::filesystem::create_directories(filepath.parent_path());
    }

    std::ofstream outFile(filepath);
    if (!outFile.is_open())
    {
        ConsoleError(L"Error: Cannot create file " + filepath.wstring() + L"\n");
        return false;
    }

    outFile << std::fixed << std::setprecision(precision);

    for (size_t i = 0; i < models.size(); i++)
    {
        if (i > 0)
            outFile << ",";
        outFile << "CL" << (i + 1) << ",CD" << (i + 1) << ",CM" << (i + 1);
    }
    outFile << "\n";

    if (!models.empty())
    {
        size_t numRows = models[0].lift.size();

        for (size_t row = 0; row < numRows; row++)
        {
            for (size_t modelIdx = 0; modelIdx < models.size(); modelIdx++)
            {
                if (modelIdx > 0)
                    outFile << ",";
                outFile << models[modelIdx].lift[row] << "," << models[modelIdx].drag[row] << ","
                        << models[modelIdx].moment[row];
            }
            outFile << "\n";
        }
    }

    outFile.close();
    ConsoleOut(L"CSV exported to: " + filepath.wstring() + L"\n");
    return true;
}
