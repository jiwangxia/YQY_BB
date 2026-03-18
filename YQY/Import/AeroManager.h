#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <filesystem>
#include <windows.h>

// ============ 控制台颜色支持 ============
const int COLOR_GREEN = 10;
const int COLOR_RED = 12;
const int COLOR_WHITE = 7;

// 颜色函数声明
void enableConsoleColor();
void setColor(int color);
void resetColor();

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

// ============ 气动数据管理器类 ============
class AeroManager
{
private:
    std::vector<BladeModel> models;
    const double START_VAL = 0.0;
    const double STEP = 5.0;

    // 私有辅助函数
    std::vector<std::string> split(const std::string& s, char delimiter) const;
    double interpolate(const std::vector<double>& yValues, double inputX) const;
    double parseDouble(const std::string& str) const;
    bool removeUTF8BOM(std::string& line) const;

public:
    AeroManager() = default;

    // 加载CSV数据
    bool loadCSV(const std::string& filepath);

    // 获取插值数据
    double getData(int modelIdx, CoefType type, double inputAngle) const;

    // 获取模型信息
    int getModelCount() const;
    int getDataSize(int modelIdx) const;

    // 导出数据
    bool exportToCSV(const std::string& filepath, int precision = 10) const;

    // 验证功能
    bool ValiDateAllCSV(const std::string& outputDir, const std::string& standardDir, double tolerance = 0.000001) const;
};