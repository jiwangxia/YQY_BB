#include "Logger.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QMutexLocker>
#include <QRegularExpression>
#include <QTextStream>

#ifdef Q_OS_WIN
#include <Windows.h>
#endif

namespace
{
void WriteNativeConsole(const QString& message, bool errorStream)
{
#ifdef Q_OS_WIN
    HANDLE handle = GetStdHandle(errorStream ? STD_ERROR_HANDLE : STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    const QString line = message + QStringLiteral("\n");
    if (handle != INVALID_HANDLE_VALUE && GetConsoleMode(handle, &mode))
    {
        DWORD written = 0;
        WriteConsoleW(handle, reinterpret_cast<LPCWSTR>(line.utf16()), static_cast<DWORD>(line.size()), &written, nullptr);
        return;
    }
#endif

    QTextStream stream(errorStream ? stderr : stdout);
    stream.setEncoding(QStringConverter::Utf8);
    stream << message << Qt::endl;
    stream.flush();
}
}

Logger& Logger::Instance()
{
    static Logger logger;
    return logger;
}

void Logger::InitializeConsoleEncoding()
{
#ifdef Q_OS_WIN
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);

    HANDLE outputHandle = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD outputMode = 0;
    if (outputHandle != INVALID_HANDLE_VALUE && GetConsoleMode(outputHandle, &outputMode))
    {
        SetConsoleMode(outputHandle, outputMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    }
#endif
}

Logger::~Logger()
{
    Stop(false, QStringLiteral("日志系统析构时仍处于开启状态"));
}

bool Logger::Start(const QString& modelFileName, int maxBackupCount)
{
    QMutexLocker locker(&m_mutex);

    if (m_active)
    {
        WriteRawLine(QStringLiteral("============================================================"));
        WriteRawLine(QStringLiteral("运行结果: 失败"));
        WriteRawLine(QStringLiteral("运行摘要: 新的日志启动前自动关闭上一条日志"));
        WriteRawLine(QStringLiteral("结束时间: %1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")));
        WriteRawLine(QStringLiteral("总耗时: %1 ms").arg(m_elapsedTimer.elapsed()));
        WriteRawLine(QStringLiteral("============================================================"));
        m_file.flush();
        m_file.close();
        m_active = false;
        RestoreMessageHandler();
    }

    m_modelFileName = modelFileName;
    m_logFilePath = BuildLogFilePath(modelFileName);
    RotateLogFiles(m_logFilePath, maxBackupCount);

    m_file.setFileName(m_logFilePath);
    if (!m_file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        WriteNativeConsole(QStringLiteral("Logger: cannot open log file: %1").arg(m_logFilePath), true);
        return false;
    }

    m_active = true;
    m_elapsedTimer.start();
    InstallMessageHandler();

    WriteRawLine(QStringLiteral("============================================================"));
    WriteRawLine(QStringLiteral("YQY CAE 运行日志"));
    WriteRawLine(QStringLiteral("开始时间: %1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")));
    WriteRawLine(QStringLiteral("模型文件: %1").arg(modelFileName));
    WriteRawLine(QStringLiteral("日志文件: %1").arg(m_logFilePath));
    WriteRawLine(QStringLiteral("旧日志规则: 最新日志为 *.log，历史日志依次为 *.log.1、*.log.2 ..."));
    WriteRawLine(QStringLiteral("============================================================"));
    return true;
}

void Logger::Stop(bool success, const QString& summary)
{
    QMutexLocker locker(&m_mutex);

    if (!m_active)
    {
        return;
    }

    WriteRawLine(QStringLiteral("============================================================"));
    WriteRawLine(QStringLiteral("运行结果: %1").arg(success ? QStringLiteral("成功") : QStringLiteral("失败")));
    if (!summary.isEmpty())
    {
        WriteRawLine(QStringLiteral("运行摘要: %1").arg(summary));
    }
    WriteRawLine(QStringLiteral("结束时间: %1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz")));
    WriteRawLine(QStringLiteral("总耗时: %1 ms").arg(m_elapsedTimer.elapsed()));
    WriteRawLine(QStringLiteral("============================================================"));

    m_file.flush();
    m_file.close();
    m_active = false;
    RestoreMessageHandler();
}

void Logger::Debug(const QString& message)
{
    Write(Level::Debug, message);
}

void Logger::Info(const QString& message)
{
    Write(Level::Info, message);
}

void Logger::Success(const QString& message)
{
    Write(Level::Success, message);
}

void Logger::Warning(const QString& message)
{
    Write(Level::Warning, message);
}

void Logger::Error(const QString& message)
{
    Write(Level::Error, message);
}

QString Logger::LogFilePath() const
{
    QMutexLocker locker(&m_mutex);
    return m_logFilePath;
}

bool Logger::IsActive() const
{
    QMutexLocker locker(&m_mutex);
    return m_active;
}

void Logger::Write(Level level, const QString& message)
{
    {
        QMutexLocker locker(&m_mutex);
        if (!m_active)
        {
            return;
        }

        const QString line = QStringLiteral("[%1] [%2] %3")
            .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"))
            .arg(LevelName(level))
            .arg(message);
        WriteRawLine(line);
    }

    WriteTerminalLine(level, message);
}

void Logger::WriteRawLine(const QString& line)
{
    if (!m_file.isOpen())
    {
        return;
    }

    QTextStream stream(&m_file);
    stream.setEncoding(QStringConverter::Utf8);
    stream << line << Qt::endl;
    stream.flush();
}

void Logger::WriteTerminalLine(Level level, const QString& message)
{
    const QString line = QStringLiteral("[%1] [%2] %3")
        .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"))
        .arg(LevelName(level))
        .arg(message);

    WriteNativeConsole(line, level == Level::Error);
}

void Logger::RotateLogFiles(const QString& filePath, int maxBackupCount)
{
    if (maxBackupCount < 0)
    {
        maxBackupCount = 0;
    }

    if (maxBackupCount == 0)
    {
        QFile::remove(filePath);
        return;
    }

    QFile::remove(QStringLiteral("%1.%2").arg(filePath).arg(maxBackupCount));
    for (int i = maxBackupCount - 1; i >= 1; --i)
    {
        const QString oldName = QStringLiteral("%1.%2").arg(filePath).arg(i);
        const QString newName = QStringLiteral("%1.%2").arg(filePath).arg(i + 1);
        if (QFile::exists(oldName))
        {
            QFile::remove(newName);
            QFile::rename(oldName, newName);
        }
    }

    if (QFile::exists(filePath))
    {
        const QString backupName = QStringLiteral("%1.1").arg(filePath);
        QFile::remove(backupName);
        QFile::rename(filePath, backupName);
    }
}

QString Logger::BuildLogFilePath(const QString& modelFileName) const
{
    QDir logDir(QDir::current().filePath("Utility/Logger/Log"));
    if (!logDir.exists())
    {
        QDir().mkpath(logDir.absolutePath());
    }

    QString baseName = QFileInfo(modelFileName).completeBaseName();
    if (baseName.isEmpty())
    {
        baseName = QStringLiteral("YQY");
    }

    return logDir.filePath(SanitizeFileName(baseName) + QStringLiteral(".log"));
}

QString Logger::SanitizeFileName(const QString& fileName) const
{
    QString result = fileName.trimmed();
    result.replace(QRegularExpression(QStringLiteral("[<>:\"/\\\\|?*\\x00-\\x1F]")), QStringLiteral("_"));
    result.replace(QRegularExpression(QStringLiteral("\\s+")), QStringLiteral("_"));
    result.replace(QRegularExpression(QStringLiteral("_+")), QStringLiteral("_"));
    if (result.isEmpty())
    {
        result = QStringLiteral("YQY");
    }
    return result;
}

QString Logger::LevelName(Level level) const
{
    switch (level)
    {
    case Level::Debug:   return QStringLiteral("DEBUG");
    case Level::Info:    return QStringLiteral("INFO");
    case Level::Success: return QStringLiteral("SUCCESS");
    case Level::Warning: return QStringLiteral("WARNING");
    case Level::Error:   return QStringLiteral("ERROR");
    default:             return QStringLiteral("INFO");
    }
}

void Logger::InstallMessageHandler()
{
    if (m_handlerInstalled)
    {
        return;
    }

    m_previousHandler = qInstallMessageHandler(Logger::MessageHandler);
    m_handlerInstalled = true;
}

void Logger::RestoreMessageHandler()
{
    if (!m_handlerInstalled)
    {
        return;
    }

    qInstallMessageHandler(m_previousHandler);
    m_previousHandler = nullptr;
    m_handlerInstalled = false;
}

void Logger::WriteQtMessage(QtMsgType type, const QMessageLogContext& context, const QString& message)
{
    Q_UNUSED(context);

    Level level = Level::Info;
    switch (type)
    {
    case QtDebugMsg:
        level = Level::Info;
        break;
    case QtInfoMsg:
        level = Level::Info;
        break;
    case QtWarningMsg:
        level = Level::Warning;
        break;
    case QtCriticalMsg:
    case QtFatalMsg:
        level = Level::Error;
        break;
    }

    {
        QMutexLocker locker(&m_mutex);
        if (m_active)
        {
            const QString line = QStringLiteral("[%1] [%2] %3")
                .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"))
                .arg(LevelName(level))
                .arg(message);
            WriteRawLine(line);
        }
    }

    WriteNativeConsole(message, type == QtCriticalMsg || type == QtFatalMsg);
}

void Logger::MessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& message)
{
    Logger::Instance().WriteQtMessage(type, context, message);
}
