#include "AeroManager.h"

#include <windows.h>
#include <cstdio>

namespace
{
std::string WideToUtf8(const std::wstring& text)
{
    if (text.empty())
    {
        return {};
    }

    const int size = WideCharToMultiByte(CP_UTF8, 0, text.c_str(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
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
        ConsoleError(L"Error: Invalid CSV format, cols=" + std::to_wstring(totalCols)
            + L" file=" + filepath.wstring() + L"\n");
        return false;
    }

    std::vector<BladeModel> parsedModels(numModels);

    int rowCount = 0;
    int lineNumber = 1;
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
            ConsoleError(L"Warning: Skip line " + std::to_wstring(lineNumber)
                + L", expected cols=" + std::to_wstring(totalCols)
                + L", actual cols=" + std::to_wstring(vals.size()) + L"\n");
            continue;
        }

        for (size_t i = 0; i < numModels; i++)
        {
            size_t base = startColIndex + (i * 3);

            parsedModels[i].lift.push_back(parseDouble(vals[base + 0]));
            parsedModels[i].drag.push_back(parseDouble(vals[base + 1]));
            parsedModels[i].moment.push_back(parseDouble(vals[base + 2]));
        }
        rowCount++;
    }

    if (rowCount == 0)
    {
        ConsoleError(L"Error: No valid data " + filepath.wstring() + L"\n");
        return false;
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

bool AeroManager::removeUTF8BOM(std::string& line) const
{
    if (line.size() >= 3 &&
        (unsigned char)line[0] == 0xEF &&
        (unsigned char)line[1] == 0xBB &&
        (unsigned char)line[2] == 0xBF)
    {
        line = line.substr(3);
        return true;
    }
    return false;
}

double AeroManager::getDataFromModels(const std::vector<BladeModel>& data, int modelIdx, CoefType type, double inputAngle) const
{
    if (modelIdx < 0 || modelIdx >= static_cast<int>(data.size()))
    {
        std::cerr << "Error: Model index out of range (" << modelIdx << ")\n";
        return 0.0;
    }

    const std::vector<double>* vec = nullptr;
    switch (type)
    {
    case LIFT:   vec = &data[modelIdx].lift;   break;
    case DRAG:   vec = &data[modelIdx].drag;   break;
    case MOMENT: vec = &data[modelIdx].moment; break;
    default:     return 0.0;
    }

    if (vec->empty()) return 0.0;

    double angle = std::fmod(inputAngle, 360.0);
    if (angle < 0) angle += 360.0;

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
    return std::filesystem::path(
        std::to_wstring(key.bundleCount) + L"分裂-风速"
        + std::to_wstring(key.windSpeed) + L"-厚度"
        + std::to_wstring(key.iceThickness) + L".csv");
}

std::filesystem::path AeroManager::buildLegacyFileName(const AeroCaseKey& key)
{
    return std::to_string(key.bundleCount) + "-"
        + std::to_string(key.windSpeed) + "ms-"
        + std::to_string(key.iceThickness) + "mm.csv";
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
        ConsoleError(L"Error: Aero CSV not found. Tried "
            + chinesePath.wstring() + L" and " + legacyPath.wstring() + L"\n");
        return false;
    }

    std::vector<BladeModel> parsedModels;
    if (!loadModelsFromCSV(selectedPath, parsedModels))
    {
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
    static const int bundleCounts[] = { 1, 4 };
    static const int windSpeeds[] = { 10, 12, 14, 18 };
    static const int iceThicknesses[] = { 12, 18, 20, 25, 28 };

    bool allLoaded = true;
    for (int bundleCount : bundleCounts)
    {
        for (int windSpeed : windSpeeds)
        {
            for (int iceThickness : iceThicknesses)
            {
                if (!loadCase(dataDir, AeroCaseKey{ bundleCount, windSpeed, iceThickness }))
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
        std::cerr << "Error: Aero case not loaded (bundle=" << key.bundleCount
            << ", wind=" << key.windSpeed
            << ", ice=" << key.iceThickness << ")\n";
        return 0.0;
    }

    return getDataFromModels(it->second, modelIdx, type, inputAngle);
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
        if (i > 0) outFile << ",";
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
                if (modelIdx > 0) outFile << ",";
                outFile << models[modelIdx].lift[row] << ","
                    << models[modelIdx].drag[row] << ","
                    << models[modelIdx].moment[row];
            }
            outFile << "\n";
        }
    }

    outFile.close();
    ConsoleOut(L"CSV exported to: " + filepath.wstring() + L"\n");
    return true;
}

bool AeroManager::ValiDateAllCSV(const std::filesystem::path& outputDir, const std::filesystem::path& standardDir, double tolerance) const
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

        ConsoleOut(L"[" + std::to_wstring(i + 1) + L"/" + std::to_wstring(csvFiles.size())
            + L"] Validating: " + filename.wstring() + L"\n");

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
                if (hasError) break;
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
