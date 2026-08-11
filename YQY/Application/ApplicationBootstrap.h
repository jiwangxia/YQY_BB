#pragma once

class QApplication;
class YQY;

class ApplicationBootstrap final
{
public:
    static void prepareEnvironment();//初始化QT/VTK显示环境
    static void configureApplication(QApplication& application);//配置应用程序名称、图标、样式和字体
    static void prepareMainWindow(YQY& window);//设置主窗口的初始大小和位置
};
