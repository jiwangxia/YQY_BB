#include "../Import/AeroManager.h"

#include <cassert>
#include <filesystem>
#include <fstream>

namespace
{
std::filesystem::path MakeTempDir()
{
    auto dir = std::filesystem::temp_directory_path() / "yqy_aero_manager_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

void WriteCsv(const std::filesystem::path& path)
{
    std::ofstream file(path);
    file << "CL1,CD1,CM1\n";
    file << "1.0,2.0,3.0\n";
    file << "2.0,4.0,6.0\n";
}
}

int main()
{
    const auto dir = MakeTempDir();
    const AeroCaseKey key{ 1, 14, 12 };

    const auto legacyPath = dir / "1-14ms-12mm.csv";
    WriteCsv(legacyPath);

    AeroManager manager;
    assert(AeroManager::buildChineseFileName(key) == std::filesystem::path(L"1分裂-风速14-厚度12.csv"));
    assert(AeroManager::buildLegacyFileName(key) == std::filesystem::path("1-14ms-12mm.csv"));
    assert(manager.loadCase(dir, key));
    assert(manager.hasCase(key));
    assert(manager.getLoadedCaseCount() == 1);
    assert(manager.getData(key, 0, LIFT, 2.5) == 1.5);
    assert(manager.exportToCSV(dir / L"导出.csv", 6));

    const auto chinesePath = dir / AeroManager::buildChineseFileName(key);
    std::filesystem::rename(legacyPath, chinesePath);

    AeroManager chineseManager;
    assert(chineseManager.loadCase(dir, key));
    assert(chineseManager.hasCase(key));
    assert(chineseManager.getLoadedCaseCount() == 1);

    std::filesystem::remove_all(dir);

    AeroManager repositoryData;
    assert(repositoryData.loadAllCases(std::filesystem::path("YQY/Import/Aero_Data/Input_Data")));
    assert(repositoryData.getLoadedCaseCount() == 40);
    assert(repositoryData.hasCase(AeroCaseKey{ 4, 18, 28 }));

    return 0;
}
