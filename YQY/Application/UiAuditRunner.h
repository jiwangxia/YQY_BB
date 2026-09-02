#pragma once

#include <optional>

class QApplication;
class YQY;

// 解析并执行界面自动巡检参数。
class UiAuditRunner final
{
public:
    explicit UiAuditRunner(const QStringList& arguments); // 解析巡检命令行参数

    bool isValid() const
    {
        return m_valid;
    }
    int errorCode() const
    {
        return m_errorCode;
    }
    std::optional<int> run(QApplication& application, YQY& window) const; // 执行指定巡检项

private:
    enum class Kind // 巡检功能类型
    {
        None,
        SolveTasks,
        NodeExport,
        Postprocess,
        AnalysisManager
    };

    Kind m_kind = Kind::None;    // 待执行的巡检项
    bool m_valid = true;          // 参数是否合法
    int m_errorCode = 1;          // 参数错误时的返回码
    QString m_outputFile;         // 巡检输出文件
    QString m_modelFile;          // 待打开的模型文件
    QString m_resultFile;         // 待打开的结果文件
    int m_analysisManager = 0;    // 分析管理器目标页面
};
