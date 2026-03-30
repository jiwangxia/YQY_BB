#include "AeroManager.h"

// ============ 控制台颜色函数实现 ============
void enableConsoleColor()
{
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
            continue;
        }
        size_t last = token.find_last_not_of(" \t\r\n");
        tokens.push_back(token.substr(first, (last - first + 1)));
    }

    return tokens;
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

// ============ AeroManager 公共函数实现 ============

bool AeroManager::loadCSV(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "Error: Cannot open file " << filepath << std::endl;
        return false;
    }

    std::string line;

    if (!std::getline(file, line))
    {
        std::cerr << "Error: File is empty" << std::endl;
        return false;
    }

    removeUTF8BOM(line);

    std::vector<std::string> headers = split(line, ',');
    size_t totalCols = headers.size();

    size_t numModels = 0;
    int startColIndex = 0;

    if (totalCols % 3 == 0)
    {
        numModels = totalCols / 3;
        startColIndex = 0;
    }
    else if ((totalCols - 1) % 3 == 0)
    {
        numModels = (totalCols - 1) / 3;
        startColIndex = 1;
    }
    else
    {
        std::cerr << "Error: Invalid CSV format, cols=" << totalCols << std::endl;
        return false;
    }

    models.clear();
    models.resize(numModels);

    int rowCount = 0;
    while (std::getline(file, line))
    {
        if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos)
        {
            continue;
        }

        std::vector<std::string> vals = split(line, ',');

        if (vals.size() != totalCols)
        {
            continue;
        }

        try
        {
            for (size_t i = 0; i < numModels; i++)
            {
                size_t base = startColIndex + (i * 3);

                models[i].lift.push_back(parseDouble(vals[base + 0]));
                models[i].drag.push_back(parseDouble(vals[base + 1]));
                models[i].moment.push_back(parseDouble(vals[base + 2]));
            }
            rowCount++;
        }
        catch (...)
        {
        }
    }

    if (rowCount == 0)
    {
        std::cerr << "Error: No valid data" << std::endl;
        return false;
    }

    return true;
}

double AeroManager::getData(int modelIdx, CoefType type, double inputAngle) const
{
    if (modelIdx < 0 || modelIdx >= static_cast<int>(models.size()))
    {
        std::cerr << "Error: Model index out of range (" << modelIdx << ")\n";
        return 0.0;
    }

    const std::vector<double>* vec = nullptr;
    switch (type)
    {
    case LIFT:   vec = &models[modelIdx].lift;   break;
    case DRAG:   vec = &models[modelIdx].drag;   break;
    case MOMENT: vec = &models[modelIdx].moment; break;
    default:     return 0.0;
    }

    if (vec->empty()) return 0.0;

    // 1. 将角度归一化到 [0, 360)
    double angle = std::fmod(inputAngle, 360.0);
    if (angle < 0) angle += 360.0;

    size_t n = vec->size();
    double maxAngle = (n - 1) * STEP;   // 数据覆盖的最大角度（本例中为355°）

    // 2. 如果归一化角度在数据范围内，直接插值
    if (angle <= maxAngle) 
    {
        return interpolate(*vec, angle);
    }
    // 3. 否则角度在 (maxAngle, 360)，利用周期性用 maxAngle 和 0° 插值
    else {
        double y_first = vec->front();   // 0° 对应的值
        double y_last = vec->back();    // maxAngle 对应的值
        double t = (angle - maxAngle) / (360.0 - maxAngle); // 归一化位置
        return y_last + (y_first - y_last) * t;
    }
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

bool AeroManager::exportToCSV(const std::string& filepath, int precision) const
{
    std::ofstream outFile(filepath);
    if (!outFile.is_open())
    {
        std::cerr << "Error: Cannot create file " << filepath << std::endl;
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
    std::cout << "CSV exported to: " << filepath << std::endl;
    return true;
}

bool AeroManager::ValiDateAllCSV(const std::string& outputDir, const std::string& standardDir, double tolerance) const
{
    namespace fs = std::filesystem;

    std::cout << "\n========================================\n";
    std::cout << "             CSV Validation\n";
    std::cout << "========================================\n";
    std::cout << "Output folder:   " << outputDir << "\n";
    std::cout << "Standard folder: " << standardDir << "\n";
    std::cout << "========================================\n\n";

    if (!fs::exists(outputDir))
    {
        std::cerr << "Error: Output folder not found " << outputDir << std::endl;
        return false;
    }

    if (!fs::exists(standardDir))
    {
        std::cerr << "Error: Standard folder not found " << standardDir << std::endl;
        return false;
    }

    std::vector<std::string> csvFiles;
    for (const auto& entry : fs::directory_iterator(outputDir))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".csv")
        {
            csvFiles.push_back(entry.path().filename().string());
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
    std::vector<std::string> failedFiles;

    for (size_t i = 0; i < csvFiles.size(); i++)
    {
        const std::string& filename = csvFiles[i];
        std::string outputFile = outputDir + "/" + filename;
        std::string standardFile = standardDir + "/" + filename;

        std::cout << "[" << (i + 1) << "/" << csvFiles.size() << "] Validating: " << filename << "\n";

        if (!fs::exists(standardFile))
        {
            setColor(COLOR_RED);
            std::cerr << "  Warning";
            resetColor();
            std::cerr << ": Standard file not found, skipping\n\n";
            failCount++;
            failedFiles.push_back(filename + " (missing standard)");
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
            failedFiles.push_back(filename);
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
            std::cout << "  - " << f << "\n";
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
