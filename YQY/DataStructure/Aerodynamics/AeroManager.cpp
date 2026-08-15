#include "AeroManager.h"

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

void EnforcePeriodicEndpoint(std::vector<BladeModel>& models)
{
    for (const BladeModel& model : models)
        if (model.lift.size() != 73 || model.drag.size() != 73 || model.moment.size() != 73)
            return;

    // 0 度与 360 度是同一周期点。保留原始文件，仅在读取后合并重复端点。
    for (BladeModel& model : models)
    {
        model.lift[72] = model.lift[0];
        model.drag[72] = model.drag[0];
        model.moment[72] = model.moment[0];
    }
}
}

// ============ 控制台颜色函数实现 ============
void enableConsoleColor()
{
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    GetConsoleMode(hOut, &dwMode);
    SetConsoleMode(hOut, dwMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
}

void setColor(int color)
{
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), color);
}

void resetColor()
{
    SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), COLOR_WHITE);
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

    constexpr int expectedRows = 73; // 0..360 degrees, inclusive, at 5-degree spacing
    if (rowCount != expectedRows)
    {
        ConsoleError(L"Error: Expected " + std::to_wstring(expectedRows) + L" aerodynamic rows, got " +
                     std::to_wstring(rowCount) + L" file=" + filepath.wstring() + L"\n");
        return false;
    }

    EnforcePeriodicEndpoint(parsedModels);
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

    return yValues[idx] + (yValues[idx + 1] - yValues[idx]) * ratio;
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
    return normalized;
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
    if (angle <= maxAngle)
    {
        const double exactPosition = (angle - START_VAL) / STEP;
        const std::size_t firstIndex = std::min(static_cast<std::size_t>(std::floor(exactPosition)), dataSize - 1);
        const std::size_t secondIndex = std::min(firstIndex + 1, dataSize - 1);
        const double ratio = secondIndex == firstIndex ? 0.0 : exactPosition - firstIndex;
        const auto evaluate = [firstIndex, secondIndex, ratio](const std::vector<double>& values)
        {
            return values[firstIndex] + (values[secondIndex] - values[firstIndex]) * ratio;
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

AeroCoefficients AeroManager::getCoefficients(const AeroCaseKey& key, int modelIdx, double inputAngle) const
{
    const auto* caseData = findCaseModels(key);
    return caseData ? getCoefficients(*caseData, modelIdx, inputAngle) : AeroCoefficients{};
}

void AeroManager::setCaseData(const AeroCaseKey& key, std::vector<BladeModel> caseData,
                              const std::filesystem::path& sourceFile)
{
    EnforcePeriodicEndpoint(caseData);
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

bool AeroManager::ValiDateAllCSV(const std::filesystem::path& outputDir, const std::filesystem::path& standardDir,
                                 double tolerance) const
{
    namespace fs = std::filesystem;

    std::cout << "\n========================================\n";
    std::cout << "             CSV Validation\n";
    std::cout << "========================================\n";
    ConsoleOut(L"Output folder:   " + outputDir.wstring() + L"\n");
    ConsoleOut(L"Standard folder: " + standardDir.wstring() + L"\n");
    std::cout << "========================================\n\n";

    if (!fs::exists(outputDir))
    {
        ConsoleError(L"Error: Output folder not found " + outputDir.wstring() + L"\n");
        return false;
    }

    if (!fs::exists(standardDir))
    {
        ConsoleError(L"Error: Standard folder not found " + standardDir.wstring() + L"\n");
        return false;
    }

    std::vector<std::filesystem::path> csvFiles;
    for (const auto& entry : fs::directory_iterator(outputDir))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".csv")
        {
            csvFiles.push_back(entry.path().filename());
        }
    }

    if (csvFiles.empty())
    {
        std::cout << "Warning: No CSV files found in output folder\n";
        return true;
    }

    std::cout << "Found " << csvFiles.size() << " CSV files to validate\n\n";

    int passCount = 0;
    int failCount = 0;
    std::vector<std::wstring> failedFiles;

    for (size_t i = 0; i < csvFiles.size(); i++)
    {
        const std::filesystem::path& filename = csvFiles[i];
        std::filesystem::path outputFile = outputDir / filename;
        std::filesystem::path standardFile = standardDir / filename;

        ConsoleOut(L"[" + std::to_wstring(i + 1) + L"/" + std::to_wstring(csvFiles.size()) + L"] Validating: " +
                   filename.wstring() + L"\n");

        if (!fs::exists(standardFile))
        {
            setColor(COLOR_RED);
            std::cerr << "  Warning";
            resetColor();
            std::cerr << ": Standard file not found, skipping\n\n";
            failCount++;
            failedFiles.push_back(filename.wstring() + L" (missing standard)");
            continue;
        }

        std::ifstream outF(outputFile);
        std::ifstream stdF(standardFile);
        bool hasError = false;

        if (outF.is_open() && stdF.is_open())
        {
            std::string outLine, stdLine;
            int lineNum = 0;

            while (std::getline(outF, outLine) && std::getline(stdF, stdLine))
            {
                lineNum++;
                if (lineNum == 1)
                {
                    removeUTF8BOM(outLine);
                    removeUTF8BOM(stdLine);
                    continue;
                }

                std::vector<std::string> outVals = split(outLine, ',');
                std::vector<std::string> stdVals = split(stdLine, ',');

                if (outVals.size() != stdVals.size())
                {
                    hasError = true;
                    break;
                }

                for (size_t col = 0; col < outVals.size(); col++)
                {
                    double outVal = parseDouble(outVals[col]);
                    double stdVal = parseDouble(stdVals[col]);
                    if (std::abs(outVal - stdVal) > tolerance)
                    {
                        hasError = true;
                        break;
                    }
                }
                if (hasError)
                    break;
            }

            outF.close();
            stdF.close();
        }
        else
        {
            hasError = true;
        }

        if (!hasError)
        {
            setColor(COLOR_GREEN);
            std::cout << "  PASSED";
            resetColor();
            std::cout << " - Files are identical\n\n";
            passCount++;
        }
        else
        {
            setColor(COLOR_RED);
            std::cout << "  FAILED";
            resetColor();
            std::cout << " - Differences found\n\n";
            failCount++;
            failedFiles.push_back(filename.wstring());
        }
    }

    std::cout << "Total files: " << csvFiles.size() << "\n";
    std::cout << "Passed: " << passCount << "\n";
    std::cout << "Failed: " << failCount << "\n";

    if (!failedFiles.empty())
    {
        std::cout << "\nFailed files:\n";
        for (const auto& f : failedFiles)
        {
            ConsoleOut(L"  - " + f + L"\n");
        }
    }

    std::cout << "\nFinal result: ";
    if (failCount == 0)
    {
        setColor(COLOR_GREEN);
        std::cout << "ALL PASSED!\n";
        resetColor();
    }
    else
    {
        setColor(COLOR_RED);
        std::cout << "SOME FAILED\n";
        resetColor();
    }
    std::cout << "========================================\n\n";

    return failCount == 0;
}
