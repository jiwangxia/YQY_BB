#include "Widgets/ViewportCameraController.h"

#include <vtkCamera.h>
#include <vtkRenderer.h>

#include <algorithm>
#include <cmath>

namespace
{
constexpr double MinimumScale = 1.0e-12;
constexpr double MaximumScale = 1.0e18;
constexpr double MinimumVectorLength = 1.0e-10;

bool finiteVector(const double value[3])
{
    return value && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

double vectorLength(const double value[3])
{
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

bool normalize(double value[3])
{
    const double length = vectorLength(value);
    if (!std::isfinite(length) || length <= MinimumVectorLength)
        return false;
    value[0] /= length;
    value[1] /= length;
    value[2] /= length;
    return true;
}

bool rotateVector(const double input[3], const double rawAxis[3], double degrees, double output[3])
{
    double axis[3] = {rawAxis[0], rawAxis[1], rawAxis[2]};
    if (!finiteVector(input) || !finiteVector(axis) || !std::isfinite(degrees) || !normalize(axis))
        return false;
    const double radians = degrees * 3.14159265358979323846 / 180.0;
    const double cosine = std::cos(radians);
    const double sine = std::sin(radians);
    const double axisDot = axis[0] * input[0] + axis[1] * input[1] + axis[2] * input[2];
    const double crossValue[3] = {axis[1] * input[2] - axis[2] * input[1],
                                  axis[2] * input[0] - axis[0] * input[2],
                                  axis[0] * input[1] - axis[1] * input[0]};
    output[0] = input[0] * cosine + crossValue[0] * sine + axis[0] * axisDot * (1.0 - cosine);
    output[1] = input[1] * cosine + crossValue[1] * sine + axis[1] * axisDot * (1.0 - cosine);
    output[2] = input[2] * cosine + crossValue[2] * sine + axis[2] * axisDot * (1.0 - cosine);
    return finiteVector(output);
}
}

ViewportCameraController::ViewportCameraController(vtkRenderer& renderer)
    : m_renderer(renderer)
{
}

bool ViewportCameraController::capture(CameraState& state) const
{
    vtkCamera* camera = m_renderer.GetActiveCamera();
    if (!camera)
        return false;
    camera->GetPosition(state.position.data());
    camera->GetFocalPoint(state.focalPoint.data());
    camera->GetViewUp(state.viewUp.data());
    state.parallelScale = camera->GetParallelScale();
    state.parallelProjection = camera->GetParallelProjection() != 0;
    return isValid(state);
}

bool ViewportCameraController::restore(const CameraState& state) const
{
    if (!isValid(state))
        return false;
    vtkCamera* camera = m_renderer.GetActiveCamera();
    if (!camera)
        return false;
    camera->SetPosition(state.position.data());
    camera->SetFocalPoint(state.focalPoint.data());
    camera->SetViewUp(state.viewUp.data());
    camera->SetParallelScale(state.parallelScale);
    camera->SetParallelProjection(state.parallelProjection ? 1 : 0);
    m_renderer.ResetCameraClippingRange();
    return true;
}

bool ViewportCameraController::isValid(const CameraState& state) const
{
    if (!finiteVector(state.position.data()) || !finiteVector(state.focalPoint.data()) ||
        !finiteVector(state.viewUp.data()) || !std::isfinite(state.parallelScale) ||
        state.parallelScale < MinimumScale || state.parallelScale > MaximumScale)
    {
        return false;
    }
    const double view[3] = {state.focalPoint[0] - state.position[0], state.focalPoint[1] - state.position[1],
                            state.focalPoint[2] - state.position[2]};
    const double cross[3] = {view[1] * state.viewUp[2] - view[2] * state.viewUp[1],
                             view[2] * state.viewUp[0] - view[0] * state.viewUp[2],
                             view[0] * state.viewUp[1] - view[1] * state.viewUp[0]};
    return vectorLength(view) > MinimumVectorLength && vectorLength(state.viewUp.data()) > MinimumVectorLength &&
           vectorLength(cross) > MinimumVectorLength;
}

bool ViewportCameraController::finishOrRollback(const CameraState& previous) const
{
    CameraState candidate;
    if (!capture(candidate))
    {
        restore(previous);
        return false;
    }
    m_renderer.ResetCameraClippingRange();
    return true;
}

bool ViewportCameraController::moveCameraBy(const double offset[3]) const
{
    if (!finiteVector(offset))
        return false;
    vtkCamera* camera = m_renderer.GetActiveCamera();
    if (!camera)
        return false;
    double position[3] = {};
    double focal[3] = {};
    camera->GetPosition(position);
    camera->GetFocalPoint(focal);
    camera->SetPosition(position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]);
    camera->SetFocalPoint(focal[0] + offset[0], focal[1] + offset[1], focal[2] + offset[2]);
    return true;
}

bool ViewportCameraController::resetToVisibleBounds()
{
    CameraState previous;
    if (!capture(previous))
        return false;
    vtkCamera* camera = m_renderer.GetActiveCamera();
    camera->ParallelProjectionOn();
    camera->SetFocalPoint(0.0, 0.0, 0.0);
    camera->SetPosition(0.0, -1.0, 0.0);
    camera->SetViewUp(0.0, 0.0, 1.0);
    m_renderer.ResetCamera();
    return finishOrRollback(previous);
}

bool ViewportCameraController::fitToBounds(const double bounds[6])
{
    if (!bounds || !std::isfinite(bounds[0]) || !std::isfinite(bounds[1]) || !std::isfinite(bounds[2]) ||
        !std::isfinite(bounds[3]) || !std::isfinite(bounds[4]) || !std::isfinite(bounds[5]) ||
        bounds[0] > bounds[1] || bounds[2] > bounds[3] || bounds[4] > bounds[5])
    {
        return false;
    }
    CameraState previous;
    if (!capture(previous))
        return false;
    m_renderer.ResetCamera(bounds);
    return finishOrRollback(previous);
}

bool ViewportCameraController::setViewPose(const std::array<double, 3>& focalPoint, double distance,
                                           const std::array<double, 3>& centerToCameraDirection,
                                           const std::array<double, 3>& requestedViewUp)
{
    double direction[3] = {centerToCameraDirection[0], centerToCameraDirection[1], centerToCameraDirection[2]};
    if (!finiteVector(focalPoint.data()) || !finiteVector(requestedViewUp.data()) || !std::isfinite(distance) ||
        distance <= MinimumVectorLength || !normalize(direction))
    {
        return false;
    }
    CameraState previous;
    if (!capture(previous))
        return false;
    vtkCamera* camera = m_renderer.GetActiveCamera();
    camera->ParallelProjectionOn();
    camera->SetFocalPoint(focalPoint.data());
    camera->SetPosition(focalPoint[0] + direction[0] * distance, focalPoint[1] + direction[1] * distance,
                        focalPoint[2] + direction[2] * distance);
    camera->SetViewUp(requestedViewUp.data());
    camera->OrthogonalizeViewUp();
    return finishOrRollback(previous);
}

bool ViewportCameraController::displayToFocalPlaneWorld(double displayX, double displayY, double world[3]) const
{
    if (!world || !std::isfinite(displayX) || !std::isfinite(displayY))
        return false;
    vtkCamera* camera = m_renderer.GetActiveCamera();
    if (!camera)
        return false;
    double focal[3] = {};
    camera->GetFocalPoint(focal);
    m_renderer.SetWorldPoint(focal[0], focal[1], focal[2], 1.0);
    m_renderer.WorldToDisplay();
    const double* focalDisplay = m_renderer.GetDisplayPoint();
    if (!focalDisplay || !std::isfinite(focalDisplay[2]))
        return false;
    m_renderer.SetDisplayPoint(displayX, displayY, focalDisplay[2]);
    m_renderer.DisplayToWorld();
    const double* homogeneous = m_renderer.GetWorldPoint();
    if (!homogeneous || !std::isfinite(homogeneous[3]) || std::abs(homogeneous[3]) <= MinimumVectorLength)
        return false;
    world[0] = homogeneous[0] / homogeneous[3];
    world[1] = homogeneous[1] / homogeneous[3];
    world[2] = homogeneous[2] / homogeneous[3];
    return finiteVector(world);
}

bool ViewportCameraController::panByDisplayDelta(int deltaX, int deltaY)
{
    if (deltaX == 0 && deltaY == 0)
        return false;
    const int* size = m_renderer.GetSize();
    if (!size || size[0] <= 0 || size[1] <= 0)
        return false;
    CameraState previous;
    if (!capture(previous))
        return false;
    const double centerX = static_cast<double>(size[0]) * 0.5;
    const double centerY = static_cast<double>(size[1]) * 0.5;
    double before[3] = {};
    double after[3] = {};
    if (!displayToFocalPlaneWorld(centerX, centerY, before) ||
        !displayToFocalPlaneWorld(centerX + deltaX, centerY + deltaY, after))
    {
        return false;
    }
    const double offset[3] = {before[0] - after[0], before[1] - after[1], before[2] - after[2]};
    return moveCameraBy(offset) && finishOrRollback(previous);
}

bool ViewportCameraController::zoomAtDisplayPoint(double displayX, double displayY, double factor)
{
    if (!std::isfinite(factor) || factor <= 0.0)
        return false;
    CameraState previous;
    if (!capture(previous))
        return false;
    double before[3] = {};
    if (!displayToFocalPlaneWorld(displayX, displayY, before))
        return false;
    vtkCamera* camera = m_renderer.GetActiveCamera();
    camera->ParallelProjectionOn();
    camera->SetParallelScale(std::clamp(previous.parallelScale / factor, MinimumScale, MaximumScale));
    double after[3] = {};
    if (!displayToFocalPlaneWorld(displayX, displayY, after))
    {
        restore(previous);
        return false;
    }
    const double offset[3] = {before[0] - after[0], before[1] - after[1], before[2] - after[2]};
    return moveCameraBy(offset) && finishOrRollback(previous);
}

bool ViewportCameraController::zoomAtWorldPoint(const double world[3], double factor)
{
    if (!finiteVector(world))
        return false;
    m_renderer.SetWorldPoint(world[0], world[1], world[2], 1.0);
    m_renderer.WorldToDisplay();
    const double* display = m_renderer.GetDisplayPoint();
    return display && std::isfinite(display[0]) && std::isfinite(display[1]) &&
           zoomAtDisplayPoint(display[0], display[1], factor);
}

bool ViewportCameraController::zoomToDisplayRect(double minimumX, double minimumY, double maximumX, double maximumY,
                                                  std::array<double, 3>& newFocalPoint)
{
    const int* size = m_renderer.GetSize();
    if (!size || size[0] <= 0 || size[1] <= 0)
        return false;
    const double width = std::abs(maximumX - minimumX);
    const double height = std::abs(maximumY - minimumY);
    if (!std::isfinite(width) || !std::isfinite(height) || width < 5.0 || height < 5.0)
        return false;
    const double ratio = std::max(width / static_cast<double>(size[0]), height / static_cast<double>(size[1]));
    if (!std::isfinite(ratio) || ratio <= 0.0 || ratio >= 1.0)
        return false;
    CameraState previous;
    if (!capture(previous))
        return false;
    double target[3] = {};
    if (!displayToFocalPlaneWorld((minimumX + maximumX) * 0.5, (minimumY + maximumY) * 0.5, target))
        return false;
    vtkCamera* camera = m_renderer.GetActiveCamera();
    camera->ParallelProjectionOn();
    camera->SetParallelScale(std::clamp(previous.parallelScale * ratio, MinimumScale, MaximumScale));
    double position[3] = {};
    double focal[3] = {};
    camera->GetPosition(position);
    camera->GetFocalPoint(focal);
    camera->SetPosition(position[0] + target[0] - focal[0], position[1] + target[1] - focal[1],
                        position[2] + target[2] - focal[2]);
    camera->SetFocalPoint(target);
    if (!finishOrRollback(previous))
        return false;
    newFocalPoint = {target[0], target[1], target[2]};
    return true;
}

bool ViewportCameraController::rotateAroundCenter(const double center[3], double azimuthDegrees, double elevationDegrees)
{
    if (!finiteVector(center) || !std::isfinite(azimuthDegrees) || !std::isfinite(elevationDegrees))
        return false;
    CameraState previous;
    if (!capture(previous))
        return false;
    double positionOffset[3] = {previous.position[0] - center[0], previous.position[1] - center[1],
                                previous.position[2] - center[2]};
    double focalOffset[3] = {previous.focalPoint[0] - center[0], previous.focalPoint[1] - center[1],
                             previous.focalPoint[2] - center[2]};
    double viewUp[3] = {previous.viewUp[0], previous.viewUp[1], previous.viewUp[2]};
    if (!rotateVector(positionOffset, viewUp, azimuthDegrees, positionOffset) ||
        !rotateVector(focalOffset, viewUp, azimuthDegrees, focalOffset))
    {
        return false;
    }
    const double projection[3] = {focalOffset[0] - positionOffset[0], focalOffset[1] - positionOffset[1],
                                  focalOffset[2] - positionOffset[2]};
    double right[3] = {projection[1] * viewUp[2] - projection[2] * viewUp[1],
                       projection[2] * viewUp[0] - projection[0] * viewUp[2],
                       projection[0] * viewUp[1] - projection[1] * viewUp[0]};
    if (!normalize(right) || !rotateVector(positionOffset, right, elevationDegrees, positionOffset) ||
        !rotateVector(focalOffset, right, elevationDegrees, focalOffset) ||
        !rotateVector(viewUp, right, elevationDegrees, viewUp))
    {
        return false;
    }
    vtkCamera* camera = m_renderer.GetActiveCamera();
    camera->SetPosition(center[0] + positionOffset[0], center[1] + positionOffset[1], center[2] + positionOffset[2]);
    camera->SetFocalPoint(center[0] + focalOffset[0], center[1] + focalOffset[1], center[2] + focalOffset[2]);
    camera->SetViewUp(viewUp);
    camera->OrthogonalizeViewUp();
    return finishOrRollback(previous);
}
