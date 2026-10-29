#pragma once

#include <array>

class vtkRenderer;

// VTK 相机修改统一支持回退；屏幕与世界坐标转换失败或相机基底退化时，恢复上一个有效视图。
class ViewportCameraController final
{
public:
    explicit ViewportCameraController(vtkRenderer& renderer);                                        // 绑定视口渲染器并管理相机操作

    bool resetToVisibleBounds();                                                                     // 调整相机使全部可见几何适合视口
    bool fitToBounds(const double bounds[6]);                                                        // 调整相机以容纳指定包围盒
    bool setViewPose(const std::array<double, 3>& focalPoint, double distance,
                     const std::array<double, 3>& centerToCameraDirection,
                     const std::array<double, 3>& requestedViewUp);                                  // 根据焦点、距离和方向设置相机姿态

    bool displayToFocalPlaneWorld(double displayX, double displayY, double world[3]) const;          // 将显示坐标转换到相机焦平面的世界坐标
    bool panByDisplayDelta(int deltaX, int deltaY);                                                  // 按显示坐标位移平移相机
    bool zoomAtDisplayPoint(double displayX, double displayY, double factor);                        // 以指定显示坐标为基点缩放相机
    bool zoomAtWorldPoint(const double world[3], double factor);                                     // 以指定世界坐标为基点缩放相机
    bool zoomToDisplayRect(double minimumX, double minimumY, double maximumX, double maximumY,
                           std::array<double, 3>& newFocalPoint);                                    // 缩放到指定显示矩形并返回新焦点
    bool rotateAroundCenter(const double center[3], double azimuthDegrees, double elevationDegrees); // 围绕指定中心旋转相机

private:
    struct CameraState
    {
        std::array<double, 3> position{};
        std::array<double, 3> focalPoint{};
        std::array<double, 3> viewUp{};
        double parallelScale = 1.0;
        bool parallelProjection = true;
    };

    bool capture(CameraState& state) const;                                                          // 保存当前相机状态供校验和回退
    bool restore(const CameraState& state) const;                                                    // 将相机恢复到指定状态
    bool isValid(const CameraState& state) const;                                                    // 检查相机位置、方向和投影参数是否有效
    bool finishOrRollback(const CameraState& previous) const;                                        // 校验修改后的相机并在无效时回退
    bool moveCameraBy(const double offset[3]) const;                                                 // 按世界坐标偏移同步移动相机和焦点

    vtkRenderer& m_renderer;
};
