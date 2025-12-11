#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_YQY.h"

class YQY : public QMainWindow
{
    Q_OBJECT

public:
    YQY(QWidget *parent = nullptr);
    ~YQY();

private:
    Ui::YQYClass ui;
};

