#pragma once

class QApplication;
class YQY;

class ApplicationBootstrap final
{
public:
    static void prepareEnvironment();
    static void configureApplication(QApplication& application);
    static void prepareMainWindow(YQY& window);
};
