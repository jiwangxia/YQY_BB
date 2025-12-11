#include "GUI/YQY.h"
#include <QtWidgets/QApplication>
#include <Eigen/Dense>

#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    YQY window;
    window.show();
    return app.exec();

}
