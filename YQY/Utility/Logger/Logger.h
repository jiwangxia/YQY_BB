#pragma once

#include <QElapsedTimer>
#include <QFile>
#include <QMutex>
#include <QString>
#include <QtGlobal>

// 线程安全的文件与终端日志记录器。
class Logger
{
public:
    enum class Level // 日志严重级别
    {
        Debug,
        Info,
        Success,
        Warning,
        Error
    };

    static Logger& Instance(); // 返回全局日志实例

    static void InitializeConsoleEncoding(); // 初始化 Windows 控制台 UTF-8 输出

    bool Start(const QString& modelFileName, int maxBackupCount = 20); // 创建本次运行的日志文件
    void Stop(bool success, const QString& summary = QString()); // 写入运行结论并关闭日志

    void Debug(const QString& message); // 记录调试信息
    void Info(const QString& message); // 记录普通信息
    void InfoToFile(const QString& message); // 仅记录到文件
    void Success(const QString& message); // 记录成功信息
    void Warning(const QString& message); // 记录警告信息
    void Error(const QString& message); // 记录错误信息

    QString LogFilePath() const; // 返回当前日志文件路径
    bool IsActive() const; // 返回日志系统是否已启动

private:
    Logger() = default;
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void Write(Level level, const QString& message); // 写入文件和终端
    void WriteRawLine(const QString& line); // 将已格式化文本写入文件
    void WriteTerminalLine(Level level, const QString& message); // 将日志写入终端
    void RotateLogFiles(const QString& filePath, int maxBackupCount); // 轮换历史日志
    QString BuildLogFilePath(const QString& modelFileName) const; // 生成日志路径
    QString SanitizeFileName(const QString& fileName) const; // 过滤文件名非法字符
    QString LevelName(Level level) const; // 返回级别显示文本

    void InstallMessageHandler(); // 接管 Qt 日志回调
    void RestoreMessageHandler(); // 恢复原 Qt 日志回调
    void WriteQtMessage(QtMsgType type, const QMessageLogContext& context, const QString& message); // 转存 Qt 日志

    static void MessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& message); // Qt 回调入口

private:
    mutable QMutex m_mutex;                     // 保护日志状态和文件
    QFile m_file;                               // 当前日志文件
    QString m_logFilePath;                      // 当前日志文件路径
    QString m_modelFileName;                    // 当前模型文件名
    QElapsedTimer m_elapsedTimer;               // 本次运行耗时计时器
    bool m_active = false;                      // 日志文件是否已打开
    bool m_handlerInstalled = false;            // Qt 回调是否已接管
    QtMessageHandler m_previousHandler = nullptr; // 原 Qt 日志回调
};
