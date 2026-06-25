#pragma once

#include <QElapsedTimer>
#include <QFile>
#include <QMutex>
#include <QString>
#include <QtGlobal>

class Logger
{
public:
    enum class Level
    {
        Debug,
        Info,
        Success,
        Warning,
        Error
    };

    static Logger& Instance();

    bool Start(const QString& modelFileName, int maxBackupCount = 20);
    void Stop(bool success, const QString& summary = QString());

    void Debug(const QString& message);
    void Info(const QString& message);
    void Success(const QString& message);
    void Warning(const QString& message);
    void Error(const QString& message);

    QString LogFilePath() const;
    bool IsActive() const;

private:
    Logger() = default;
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void Write(Level level, const QString& message);
    void WriteRawLine(const QString& line);
    void WriteTerminalLine(Level level, const QString& message);
    void RotateLogFiles(const QString& filePath, int maxBackupCount);
    QString BuildLogFilePath(const QString& modelFileName) const;
    QString SanitizeFileName(const QString& fileName) const;
    QString LevelName(Level level) const;

    void InstallMessageHandler();
    void RestoreMessageHandler();
    void WriteQtMessage(QtMsgType type, const QMessageLogContext& context, const QString& message);

    static void MessageHandler(QtMsgType type, const QMessageLogContext& context, const QString& message);

private:
    mutable QMutex m_mutex;
    QFile m_file;
    QString m_logFilePath;
    QString m_modelFileName;
    QElapsedTimer m_elapsedTimer;
    bool m_active = false;
    bool m_handlerInstalled = false;
    QtMessageHandler m_previousHandler = nullptr;
};
