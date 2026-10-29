#include "Widgets/ModelViewport.h"

#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Element/ElementSpring1.h"
#include "DataStructure/Element/ElementSpring2.h"
#include "DataStructure/Element/ElementSpringA.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Section/SectionRectangle.h"
#include "DataStructure/Section/Section_Ice.h"
#include "Export/Hdf5ResultData.h"

#include <QLabel>
#include <QActionGroup>
#include <QContextMenuEvent>
#include <QIcon>
#include <QKeyEvent>
#include <QLineF>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QResizeEvent>
#include <QRubberBand>
#include <QTimer>
#include <QWheelEvent>

#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkHardwarePicker.h>
#include <vtkIntArray.h>
#include <vtkDoubleArray.h>
#include <vtkDataObject.h>
#include <vtkHardwareSelector.h>
#include <vtkInformation.h>
#include <vtkIdTypeArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkIdList.h>
#include <vtkLabeledDataMapper.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkProp.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkObjectFactory.h>
#include <vtkTextProperty.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>

#include <unordered_map>
#include <unordered_set>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>

namespace
{
class ModeInteractorStyle final : public vtkInteractorStyleTrackballCamera
{
public:
    static ModeInteractorStyle* New();
    vtkTypeMacro(ModeInteractorStyle, vtkInteractorStyleTrackballCamera);

    void SetInteractionMode(ModelViewport::InteractionMode mode)
    {
        if (this->State != VTKIS_NONE)
            this->StopState();
        m_rotating = false;
        m_panning = false;
        m_zooming = false;
        m_mode = mode;
    }

    void SetRotationCenterProvider(std::function<bool(double*)> provider)
    {
        m_rotationCenterProvider = std::move(provider);
    }

    void SetCameraChangedCallback(std::function<void()> callback)
    {
        m_cameraChanged = std::move(callback);
    }

    void SetPanFinishedCallback(std::function<void()> callback)
    {
        m_panFinished = std::move(callback);
    }

    void SetPanByDisplayDeltaCallback(std::function<bool(int, int)> callback)
    {
        m_panByDisplayDelta = std::move(callback);
    }

    void SetZoomAtRotationCenterCallback(std::function<bool(double)> callback)
    {
        m_zoomAtRotationCenter = std::move(callback);
    }

    void SetRotateAroundCenterCallback(std::function<bool(const double*, double, double)> callback)
    {
        m_rotateAroundCenter = std::move(callback);
    }

    void OnLeftButtonDown() override
    {
        switch (m_mode)
        {
            case ModelViewport::InteractionMode::Rotate:
                if (this->Interactor)
                {
                    const int* position = this->Interactor->GetEventPosition();
                    this->FindPokedRenderer(position[0], position[1]);
                    m_lastPosition[0] = position[0];
                    m_lastPosition[1] = position[1];
                    m_rotating = true;
                }
                break;
            case ModelViewport::InteractionMode::Pan:
                if (this->Interactor)
                {
                    const int* position = this->Interactor->GetEventPosition();
                    this->FindPokedRenderer(position[0], position[1]);
                    m_lastPosition[0] = position[0];
                    m_lastPosition[1] = position[1];
                    m_panning = this->CurrentRenderer != nullptr;
                }
                break;
            case ModelViewport::InteractionMode::Zoom:
                if (this->Interactor)
                {
                    const int* position = this->Interactor->GetEventPosition();
                    this->FindPokedRenderer(position[0], position[1]);
                    m_lastPosition[0] = position[0];
                    m_lastPosition[1] = position[1];
                    m_zooming = this->CurrentRenderer != nullptr;
                }
                break;
            case ModelViewport::InteractionMode::ZoomToRect:
                break;
            case ModelViewport::InteractionMode::Select:
                break;
        }
    }

    void OnLeftButtonUp() override
    {
        switch (m_mode)
        {
            case ModelViewport::InteractionMode::Rotate:
                m_rotating = false;
                break;
            case ModelViewport::InteractionMode::Pan:
            {
                const bool wasPanning = m_panning;
                m_panning = false;
                if (wasPanning && m_panFinished)
                    m_panFinished();
                break;
            }
            case ModelViewport::InteractionMode::Zoom:
                m_zooming = false;
                break;
            case ModelViewport::InteractionMode::ZoomToRect:
                break;
            case ModelViewport::InteractionMode::Select:
                break;
        }
    }

    void OnMouseMove() override
    {
        if (m_mode == ModelViewport::InteractionMode::Rotate && m_rotating && this->Interactor && this->CurrentRenderer)
        {
            const int* position = this->Interactor->GetEventPosition();
            const int dx = position[0] - m_lastPosition[0];
            const int dy = position[1] - m_lastPosition[1];
            m_lastPosition[0] = position[0];
            m_lastPosition[1] = position[1];
            if (dx == 0 && dy == 0)
                return;

            double center[3] = {0.0, 0.0, 0.0};
            if (!m_rotationCenterProvider || !m_rotationCenterProvider(center) || !m_rotateAroundCenter)
                return;
            const double azimuth = -static_cast<double>(dx) * 0.25;
            const double elevation = static_cast<double>(dy) * 0.25;
            if (!m_rotateAroundCenter(center, azimuth, elevation))
                return;
            if (m_cameraChanged)
                m_cameraChanged();
            this->Interactor->Render();
            return;
        }

        if (m_mode == ModelViewport::InteractionMode::Pan && m_panning && this->Interactor && this->CurrentRenderer)
        {
            const int* position = this->Interactor->GetEventPosition();
            const int dx = position[0] - m_lastPosition[0];
            const int dy = position[1] - m_lastPosition[1];
            m_lastPosition[0] = position[0];
            m_lastPosition[1] = position[1];
            if ((dx != 0 || dy != 0) && m_panByDisplayDelta && m_panByDisplayDelta(dx, dy))
            {
                if (m_cameraChanged)
                    m_cameraChanged();
                this->Interactor->Render();
            }
            return;
        }

        if (m_mode == ModelViewport::InteractionMode::Zoom && m_zooming && this->Interactor && this->CurrentRenderer)
        {
            const int* position = this->Interactor->GetEventPosition();
            const int dy = position[1] - m_lastPosition[1];
            m_lastPosition[0] = position[0];
            m_lastPosition[1] = position[1];
            if (dy != 0)
            {
                const double factor = std::pow(1.01, -static_cast<double>(dy));
                if (m_zoomAtRotationCenter && m_zoomAtRotationCenter(factor))
                {
                    if (m_cameraChanged)
                        m_cameraChanged();
                    this->Interactor->Render();
                }
            }
            return;
        }

        if (m_mode == ModelViewport::InteractionMode::Select)
            vtkInteractorStyleTrackballCamera::OnMouseMove();
    }

    void OnRightButtonDown() override
    {
    }
    void OnRightButtonUp() override
    {
    }

    void OnChar() override
    {
    }

private:
    ModelViewport::InteractionMode m_mode = ModelViewport::InteractionMode::Select;
    bool m_rotating = false;
    bool m_panning = false;
    bool m_zooming = false;
    int m_lastPosition[2] = {0, 0};
    std::function<bool(double*)> m_rotationCenterProvider;
    std::function<void()> m_cameraChanged;
    std::function<void()> m_panFinished;
    std::function<bool(int, int)> m_panByDisplayDelta;
    std::function<bool(double)> m_zoomAtRotationCenter;
    std::function<bool(const double*, double, double)> m_rotateAroundCenter;
};

vtkStandardNewMacro(ModeInteractorStyle);

struct RenderColors
{
    double background[3];
    double backgroundSecond[3];
    double element[3];
    double node[3];
    double solid[3];
    bool gradientBackground;
};

RenderColors colorsForTheme(int themeIndex)
{
    if (themeIndex == 1)
        return {{0.063, 0.035, 0.106}, {0.063, 0.035, 0.106}, {0.50, 0.38, 1.00},
                {0.94, 0.36, 1.00},    {0.55, 0.40, 0.95},    false};
    if (themeIndex == 2)
        return {{0.051, 0.145, 0.169}, {0.051, 0.145, 0.169}, {0.27, 0.66, 1.00},
                {0.33, 0.88, 0.76},    {0.20, 0.63, 0.72},    false};
    if (themeIndex == 3)
    {
        // FEM 的 VTK 背景采用 #FFFFFF 到 #EEF6FF 渐变；深色几何和蓝色节点保证两端都有足够对比度。
        return {{1.0, 1.0, 1.0},       {0.933, 0.965, 1.0}, {0.114, 0.169, 0.227},
                {0.184, 0.502, 0.929}, {0.31, 0.52, 0.72},  true};
    }
    return {{0.043, 0.075, 0.106}, {0.043, 0.075, 0.106}, {0.31, 0.55, 1.00},
            {0.19, 0.78, 0.85},    {0.28, 0.48, 0.68},    false};
}

using SolidVector3 = std::array<double, 3>;

SolidVector3 subtract(const SolidVector3& first, const SolidVector3& second)
{
    return {first[0] - second[0], first[1] - second[1], first[2] - second[2]};
}

double dot(const SolidVector3& first, const SolidVector3& second)
{
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
}

SolidVector3 cross(const SolidVector3& first, const SolidVector3& second)
{
    return {first[1] * second[2] - first[2] * second[1], first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0]};
}

bool normalize(SolidVector3& value)
{
    const double length = std::sqrt(dot(value, value));
    if (!std::isfinite(length) || length <= 1.0e-12)
        return false;
    for (double& component : value)
        component /= length;
    return true;
}

struct SectionPoint
{
    double y = 0.0;
    double z = 0.0;
};

std::vector<SectionPoint> buildSectionContour(const std::shared_ptr<SectionBase>& section)
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double displayScale = 2.5;
    constexpr double epsilon = 1.0e-12;
    std::vector<SectionPoint> contour;
    if (!section)
        return contour;

    if (const auto rectangle = std::dynamic_pointer_cast<SectionRectangle>(section))
    {
        const double halfWidth = 0.5 * rectangle->m_Width * displayScale;
        const double halfHeight = 0.5 * rectangle->m_Height * displayScale;
        if (std::isfinite(halfWidth) && std::isfinite(halfHeight) && halfWidth > epsilon && halfHeight > epsilon)
        {
            return {{halfWidth, halfHeight},
                    {-halfWidth, halfHeight},
                    {-halfWidth, -halfHeight},
                    {halfWidth, -halfHeight}};
        }
        return contour;
    }

    const auto ice = std::dynamic_pointer_cast<Section_Ice>(section);
    double radius = section->m_Radius;
    if ((!std::isfinite(radius) || radius <= epsilon) && std::isfinite(section->m_Area) && section->m_Area > epsilon)
    {
        radius = std::sqrt(section->m_Area / pi);
    }
    if (!std::isfinite(radius) || radius <= epsilon)
        return contour;

    const int segmentCount = ice ? 32 : 16;
    contour.reserve(segmentCount);
    for (int index = 0; index < segmentCount; ++index)
    {
        const double angle = 2.0 * pi * static_cast<double>(index) / static_cast<double>(segmentCount);
        double outerRadius = radius;
        if (ice && ice->m_IceThickness > 0.0 && ice->m_IceHalfAngle > epsilon)
        {
            double delta = std::fmod(angle - ice->m_IceAngle + pi, 2.0 * pi);
            if (delta < 0.0)
                delta += 2.0 * pi;
            delta -= pi;
            if (std::abs(delta) <= ice->m_IceHalfAngle)
            {
                const double ratio = delta / ice->m_IceHalfAngle;
                outerRadius +=
                    ice->m_IceThickness * std::pow(std::max(0.0, std::cos(ratio * pi * 0.5)), ice->m_IceShapeFactor);
            }
        }
        outerRadius *= displayScale;
        contour.push_back({outerRadius * std::cos(angle), outerRadius * std::sin(angle)});
    }
    return contour;
}
}

ModelViewport::ModelViewport(QWidget* parent)
    : QVTKOpenGLNativeWidget(parent)
    , m_renderWindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New())
    , m_renderer(vtkSmartPointer<vtkRenderer>::New())
    , m_points(vtkSmartPointer<vtkPoints>::New())
    , m_elementData(vtkSmartPointer<vtkPolyData>::New())
    , m_nodeData(vtkSmartPointer<vtkPolyData>::New())
    , m_nodeLabelData(vtkSmartPointer<vtkPolyData>::New())
    , m_elementLabelData(vtkSmartPointer<vtkPolyData>::New())
    , m_solidData(vtkSmartPointer<vtkPolyData>::New())
    , m_selectionData(vtkSmartPointer<vtkPolyData>::New())
    , m_solidSelectionData(vtkSmartPointer<vtkPolyData>::New())
    , m_originalElementData(vtkSmartPointer<vtkPolyData>::New())
    , m_elementMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_nodeMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_selectionMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_solidSelectionMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_solidMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_originalElementMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_nodeLabelMapper(vtkSmartPointer<vtkLabeledDataMapper>::New())
    , m_elementLabelMapper(vtkSmartPointer<vtkLabeledDataMapper>::New())
    , m_elementActor(vtkSmartPointer<vtkActor>::New())
    , m_nodeActor(vtkSmartPointer<vtkActor>::New())
    , m_selectionActor(vtkSmartPointer<vtkActor>::New())
    , m_solidSelectionActor(vtkSmartPointer<vtkActor>::New())
    , m_solidActor(vtkSmartPointer<vtkActor>::New())
    , m_originalElementActor(vtkSmartPointer<vtkActor>::New())
    , m_nodeLabelActor(vtkSmartPointer<vtkActor2D>::New())
    , m_elementLabelActor(vtkSmartPointer<vtkActor2D>::New())
    , m_hardwarePicker(vtkSmartPointer<vtkHardwarePicker>::New())
    , m_cellPicker(vtkSmartPointer<vtkCellPicker>::New())
    , m_resultLookupTable(vtkSmartPointer<vtkLookupTable>::New())
    , m_resultScalarBar(vtkSmartPointer<vtkScalarBarActor>::New())
{
    setObjectName(QStringLiteral("modelViewport"));
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setFocusPolicy(Qt::StrongFocus);

    setRenderWindow(m_renderWindow);
    m_renderWindow->AddRenderer(m_renderer);
    // VTK 在硬件拾取期间会临时关闭多重采样，常规绘制可以保留抗锯齿。
    m_renderWindow->SetMultiSamples(4);
    m_renderer->UseFXAAOn();
    m_renderWindow->GetInteractor()->SetDesiredUpdateRate(30.0);
    m_renderWindow->GetInteractor()->SetStillUpdateRate(5.0);
    m_cameraController = std::make_unique<ViewportCameraController>(*m_renderer);

    auto interactionStyle = vtkSmartPointer<ModeInteractorStyle>::New();
    interactionStyle->SetDefaultRenderer(m_renderer);
    interactionStyle->SetMotionFactor(8.0);
    interactionStyle->SetRotationCenterProvider(
        [this](double* center)
        {
            return ensureRotationCenter(center);
        });
    interactionStyle->SetCameraChangedCallback(
        [this]()
        {
            updateRotationCenterIndicator();
            updateAdaptiveLabels();
        });
    interactionStyle->SetPanFinishedCallback(
        [this]()
        {
            updateRotationCenterToViewportCenter(true);
        });
    interactionStyle->SetPanByDisplayDeltaCallback(
        [this](int deltaX, int deltaY)
        {
            return panByDisplayDelta(deltaX, deltaY);
        });
    interactionStyle->SetZoomAtRotationCenterCallback(
        [this](double factor)
        {
            return zoomAtRotationCenter(factor);
        });
    interactionStyle->SetRotateAroundCenterCallback(
        [this](const double* center, double azimuth, double elevation)
        {
            return m_cameraController && m_cameraController->rotateAroundCenter(center, azimuth, elevation);
        });
    m_interactorStyle = interactionStyle;
    m_renderWindow->GetInteractor()->SetInteractorStyle(interactionStyle);

    m_points->SetDataTypeToDouble();
    m_elementData->SetPoints(m_points);
    m_nodeData->SetPoints(m_points);
    m_elementMapper->SetInputData(m_elementData);
    m_elementMapper->ScalarVisibilityOff();
    m_nodeMapper->SetInputData(m_nodeData);
    m_nodeMapper->ScalarVisibilityOff();
    m_nodeLabelMapper->SetInputData(m_nodeLabelData);
    m_nodeLabelMapper->SetLabelModeToLabelScalars();
    m_nodeLabelMapper->SetLabelFormat("%d");
    m_nodeLabelMapper->GetLabelTextProperty()->SetFontSize(13);
    m_nodeLabelMapper->GetLabelTextProperty()->BoldOff();
    m_elementLabelMapper->SetInputData(m_elementLabelData);
    m_elementLabelMapper->SetLabelModeToLabelScalars();
    m_elementLabelMapper->SetLabelFormat("%d");
    m_elementLabelMapper->GetLabelTextProperty()->SetFontSize(13);
    m_elementLabelMapper->GetLabelTextProperty()->BoldOff();

    m_elementActor->SetMapper(m_elementMapper);
    m_elementActor->GetProperty()->SetLineWidth(3.0);
    m_originalElementMapper->SetInputData(m_originalElementData);
    m_originalElementMapper->ScalarVisibilityOff();
    m_originalElementActor->SetMapper(m_originalElementMapper);
    m_originalElementActor->GetProperty()->SetColor(0.55, 0.58, 0.62);
    m_originalElementActor->GetProperty()->SetOpacity(0.45);
    m_originalElementActor->GetProperty()->SetLineWidth(1.0);
    m_originalElementActor->SetVisibility(false);
    m_originalElementActor->PickableOff();
    m_solidMapper->SetInputData(m_solidData);
    m_solidMapper->ScalarVisibilityOff();
    m_solidActor->SetMapper(m_solidMapper);
    m_solidActor->SetVisibility(false);
    m_solidActor->PickableOff();
    m_solidActor->GetProperty()->SetInterpolationToPhong();
    m_nodeActor->SetMapper(m_nodeMapper);
    m_nodeActor->GetProperty()->SetPointSize(static_cast<float>(m_nodeSize));
    m_nodeActor->GetProperty()->RenderPointsAsSpheresOn();
    m_nodeLabelActor->SetMapper(m_nodeLabelMapper);
    m_nodeLabelActor->SetVisibility(false);
    m_nodeLabelActor->PickableOff();
    m_elementLabelActor->SetMapper(m_elementLabelMapper);
    m_elementLabelActor->SetVisibility(false);
    m_elementLabelActor->PickableOff();

    m_selectionData->SetPoints(m_points);
    m_selectionMapper->SetInputData(m_selectionData);
    m_selectionMapper->ScalarVisibilityOff();
    m_selectionActor->SetMapper(m_selectionMapper);
    m_selectionActor->GetProperty()->SetColor(1.0, 0.62, 0.12);
    m_selectionActor->GetProperty()->SetLineWidth(5.0);
    m_selectionActor->GetProperty()->SetPointSize(12.0);
    m_selectionActor->GetProperty()->RenderPointsAsSpheresOn();
    m_selectionActor->SetVisibility(false);
    m_selectionActor->PickableOff();

    m_solidSelectionMapper->SetInputData(m_solidSelectionData);
    m_solidSelectionMapper->ScalarVisibilityOff();
    m_solidSelectionMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(0.0, -8.0);
    m_solidSelectionActor->SetMapper(m_solidSelectionMapper);
    m_solidSelectionActor->GetProperty()->SetColor(1.0, 0.62, 0.12);
    m_solidSelectionActor->GetProperty()->SetOpacity(0.92);
    m_solidSelectionActor->GetProperty()->LightingOff();
    m_solidSelectionActor->GetProperty()->EdgeVisibilityOn();
    m_solidSelectionActor->GetProperty()->SetEdgeColor(1.0, 0.84, 0.28);
    m_solidSelectionActor->SetVisibility(false);
    m_solidSelectionActor->PickableOff();

    m_renderer->AddActor(m_elementActor);
    m_renderer->AddActor(m_originalElementActor);
    m_renderer->AddActor(m_solidActor);
    m_renderer->AddActor(m_nodeActor);
    m_renderer->AddActor(m_selectionActor);
    m_renderer->AddActor(m_solidSelectionActor);
    m_renderer->AddActor2D(m_nodeLabelActor);
    m_renderer->AddActor2D(m_elementLabelActor);
    m_resultLookupTable->SetHueRange(0.667, 0.0);
    m_resultLookupTable->SetSaturationRange(1.0, 1.0);
    m_resultLookupTable->SetValueRange(1.0, 1.0);
    m_resultLookupTable->SetNumberOfTableValues(1024);
    m_resultLookupTable->Build();
    m_elementMapper->InterpolateScalarsBeforeMappingOn();
    m_nodeMapper->InterpolateScalarsBeforeMappingOn();
    m_resultScalarBar->SetLookupTable(m_resultLookupTable);
    m_resultScalarBar->SetNumberOfLabels(7);
    m_resultScalarBar->SetLabelFormat("%.5g");
    m_resultScalarBar->SetBarRatio(0.20);
    m_resultScalarBar->UnconstrainedFontSizeOn();
    m_resultScalarBar->GetTitleTextProperty()->SetFontFamilyToArial();
    m_resultScalarBar->GetTitleTextProperty()->SetFontSize(18);
    m_resultScalarBar->GetTitleTextProperty()->BoldOff();
    m_resultScalarBar->GetTitleTextProperty()->ItalicOff();
    m_resultScalarBar->GetTitleTextProperty()->ShadowOff();
    m_resultScalarBar->GetLabelTextProperty()->SetFontFamilyToArial();
    m_resultScalarBar->GetLabelTextProperty()->SetFontSize(14);
    m_resultScalarBar->GetLabelTextProperty()->BoldOff();
    m_resultScalarBar->GetLabelTextProperty()->ItalicOff();
    m_resultScalarBar->GetLabelTextProperty()->ShadowOff();
    m_resultScalarBar->SetMaximumWidthInPixels(112);
    m_resultScalarBar->SetWidth(0.085);
    m_resultScalarBar->SetHeight(0.68);
    m_resultScalarBar->SetPosition(0.90, 0.15);
    m_resultScalarBar->SetVisibility(false);
    m_resultScalarBar->PickableOff();
    m_renderer->AddActor2D(m_resultScalarBar);
    m_renderer->GetActiveCamera()->ParallelProjectionOn();

    vtkNew<vtkAxesActor> axes;
    axes->SetTotalLength(1.0, 1.0, 1.0);
    axes->SetShaftTypeToCylinder();
    axes->SetCylinderRadius(0.04);
    axes->SetConeRadius(0.3);
    m_axesWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_axesWidget->SetOrientationMarker(axes);
    m_axesWidget->SetInteractor(m_renderWindow->GetInteractor());
    m_axesWidget->SetViewport(0.0, 0.0, 0.16, 0.16);
    m_axesWidget->SetEnabled(true);
    m_axesWidget->InteractiveOff();
    m_emptyStateIconLabel = new QLabel(this);
    m_emptyStateIconLabel->setObjectName(QStringLiteral("viewportEmptyIcon"));
    m_emptyStateIconLabel->setAlignment(Qt::AlignCenter);
    m_emptyStateIconLabel->setScaledContents(true);
    m_emptyStateIconLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_emptyStateLabel = new QLabel(QStringLiteral("尚未加载模型\n\n导入模型后，真实几何与计算结果将在此处显示"), this);
    m_emptyStateLabel->setObjectName(QStringLiteral("viewportEmptyLabel"));
    m_emptyStateLabel->setAlignment(Qt::AlignCenter);
    m_emptyStateLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_emptyStateLabel->setWordWrap(true);
    m_rotationCenterIndicator = new QLabel(this);
    m_rotationCenterIndicator->setObjectName(QStringLiteral("rotationCenterIndicator"));
    m_rotationCenterIndicator->setFixedSize(28, 28);
    m_rotationCenterIndicator->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_rotationCenterIndicator->hide();
    setMouseTracking(true);
    m_selectionRubberBand = new QRubberBand(QRubberBand::Rectangle, this);
    m_selectionRubberBand->setStyleSheet(
        QStringLiteral("border: 1px dashed #32C6D8; background-color: rgba(50, 198, 216, 36);"));
    m_selectionRubberBand->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_selectionRubberBand->hide();
    updateThemeColors();
    updateEmptyStateGeometry();
}

void ModelViewport::displayModel(const std::shared_ptr<StructureData>& structure, bool resetCamera)
{
    if (!structure || structure->Nodes().empty())
    {
        clearModel();
        return;
    }

    const bool modelChanged = m_structure.get() != structure.get();
    m_structure = structure;
    if (modelChanged)
    {
        m_rotationCenterValid = false;
        m_rotationCenterSnapped = false;
    }
    m_pointNodeIds.clear();
    m_cellElementIds.clear();
    m_solidCellElementIds.clear();
    m_elementLabelPositions.clear();
    m_originalNodeCoordinates.clear();
    // 不同模型共用 VTK 数据对象；装载新几何前清除旧标量和映射器着色状态，
    // 避免沿用上一模型的结果颜色。
    m_elementData->GetPointData()->SetScalars(nullptr);
    m_elementData->GetCellData()->SetScalars(nullptr);
    m_nodeData->GetPointData()->SetScalars(nullptr);
    m_elementMapper->ScalarVisibilityOff();
    m_nodeMapper->ScalarVisibilityOff();
    clearResultScalarRange();
    m_activeDeformationScale = 1.0;
    m_resultDisplayActive = false;
    m_originalElementActor->SetVisibility(false);
    m_resultScalarBar->SetVisibility(false);

    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    points->Allocate(static_cast<vtkIdType>(structure->Nodes().size()));
    std::unordered_map<const Node*, vtkIdType> pointIds;
    pointIds.reserve(structure->Nodes().size());
    m_nodePointIds.clear();
    m_springPointReferences.clear();
    m_nodePointIds.reserve(structure->Nodes().size());
    for (const auto& [nodeId, node] : structure->Nodes())
    {
        if (!node)
            continue;
        const vtkIdType pointId = points->InsertNextPoint(node->m_X, node->m_Y, node->m_Z);
        pointIds.emplace(node.get(), pointId);
        m_nodePointIds.emplace(nodeId, pointId);
        m_originalNodeCoordinates.emplace(nodeId, std::array<double, 3>{node->m_X, node->m_Y, node->m_Z});
        m_pointNodeIds.push_back(nodeId);
    }

    vtkNew<vtkPoints> nodePoints;
    nodePoints->DeepCopy(points);

    double bounds[6] = {};
    points->GetBounds(bounds);
    const double diagonal = std::sqrt((bounds[1] - bounds[0]) * (bounds[1] - bounds[0]) +
                                      (bounds[3] - bounds[2]) * (bounds[3] - bounds[2]) +
                                      (bounds[5] - bounds[4]) * (bounds[5] - bounds[4]));
    const double groundSpringLength = std::max(diagonal * 0.08, 0.08);

    vtkNew<vtkCellArray> lines;
    m_elementLabelPositions.reserve(structure->Elements().size());
    lines->AllocateEstimate(static_cast<vtkIdType>(structure->Elements().size()), 2);
    const auto addSpringPolyline =
        [&points, &lines, groundSpringLength, this](int elementId, const std::array<double, 3>& first,
                                                    const std::array<double, 3>& second, int firstNodeId,
                                                    int secondNodeId, int springAxis)
    {
        std::array<double, 3> direction = {second[0] - first[0], second[1] - first[1], second[2] - first[2]};
        double length =
            std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
        const bool coincidentNodes = length <= 1.0e-12;
        if (coincidentNodes)
        {
            // 零长度弹簧在共同节点旁绘制符号，真实端点与节点坐标保持不变。
            direction = {0.0, 0.0, 0.0};
            direction[static_cast<std::size_t>(std::clamp(springAxis, 0, 2))] = 1.0;
            length = groundSpringLength;
        }
        else
        {
            for (double& value : direction)
                value /= length;
        }

        std::array<double, 3> reference =
            std::abs(direction[2]) < 0.8 ? std::array<double, 3>{0.0, 0.0, 1.0} : std::array<double, 3>{0.0, 1.0, 0.0};
        std::array<double, 3> normal = {direction[1] * reference[2] - direction[2] * reference[1],
                                        direction[2] * reference[0] - direction[0] * reference[2],
                                        direction[0] * reference[1] - direction[1] * reference[0]};
        const double normalLength = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
        for (double& value : normal)
            value /= normalLength;

        const std::array<double, 3> binormal = {direction[1] * normal[2] - direction[2] * normal[1],
                                                direction[2] * normal[0] - direction[0] * normal[2],
                                                direction[0] * normal[1] - direction[1] * normal[0]};
        constexpr int coilTurns = 3;
        constexpr int segmentsPerTurn = 48;
        constexpr int coilSegments = coilTurns * segmentsPerTurn;
        constexpr double coilStart = 0.12;
        constexpr double coilLength = 0.76;
        constexpr double twoPi = 6.28318530717958647692;
        const double coilRadius = length * 0.08;
        const double symbolOffset = coincidentNodes ? length * 0.28 : 0.0;
        std::vector<vtkIdType> ids;
        ids.reserve(coilSegments + 3);
        const auto addPoint = [&points, &ids, this, firstNodeId, secondNodeId](const std::array<double, 3>& point,
                                                                               double secondNodeWeight)
        {
            const vtkIdType pointId = points->InsertNextPoint(point.data());
            ids.push_back(pointId);
            m_springPointReferences.push_back({pointId, firstNodeId, secondNodeId, secondNodeWeight});
        };
        addPoint(first, 0.0);
        for (int index = 0; index <= coilSegments; ++index)
        {
            const double ratio = coilStart + coilLength * static_cast<double>(index) / coilSegments;
            const double angle = twoPi * coilTurns * static_cast<double>(index) / coilSegments;
            const double axialOffset = length * (coincidentNodes ? ratio - 0.5 : ratio);
            const double normalOffset = symbolOffset + coilRadius * std::cos(angle);
            const double binormalOffset = coilRadius * std::sin(angle);
            const std::array<double, 3> point = {
                first[0] + direction[0] * axialOffset + normal[0] * normalOffset + binormal[0] * binormalOffset,
                first[1] + direction[1] * axialOffset + normal[1] * normalOffset + binormal[1] * binormalOffset,
                first[2] + direction[2] * axialOffset + normal[2] * normalOffset + binormal[2] * binormalOffset};
            addPoint(point, ratio);
        }
        addPoint(second, 1.0);
        lines->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());
        m_cellElementIds.push_back(elementId);
        m_elementLabelPositions.push_back({0.5 * (first[0] + second[0]) + normal[0] * symbolOffset,
                                           0.5 * (first[1] + second[1]) + normal[1] * symbolOffset,
                                           0.5 * (first[2] + second[2]) + normal[2] * symbolOffset});
        return true;
    };
    for (const auto& [elementId, element] : structure->Elements())
    {
        if (!element)
            continue;

        if (const auto spring1 = dynamic_cast<ElementSpring1*>(element.get()))
        {
            const auto node = spring1->m_pNode.empty() ? nullptr : spring1->m_pNode.front().lock();
            if (!node || spring1->m_DOF < 0 || spring1->m_DOF > 5)
                continue;
            std::array<double, 3> nodePosition = {node->m_X, node->m_Y, node->m_Z};
            std::array<double, 3> groundPosition = nodePosition;
            groundPosition[static_cast<std::size_t>(spring1->m_DOF % 3)] -= groundSpringLength;
            if (addSpringPolyline(elementId, groundPosition, nodePosition, -1, node->m_Id, spring1->m_DOF % 3))
            {
                const int firstAxis = (spring1->m_DOF + 1) % 3;
                const double width = groundSpringLength * 0.18;
                const std::array<double, 3> beamFirst = {groundPosition[0] - (firstAxis == 0 ? width : 0.0),
                                                         groundPosition[1] - (firstAxis == 1 ? width : 0.0),
                                                         groundPosition[2] - (firstAxis == 2 ? width : 0.0)};
                const std::array<double, 3> beamSecond = {groundPosition[0] + (firstAxis == 0 ? width : 0.0),
                                                          groundPosition[1] + (firstAxis == 1 ? width : 0.0),
                                                          groundPosition[2] + (firstAxis == 2 ? width : 0.0)};
                vtkIdType groundIds[2] = {points->InsertNextPoint(beamFirst.data()),
                                          points->InsertNextPoint(beamSecond.data())};
                lines->InsertNextCell(2, groundIds);
                m_springPointReferences.push_back({groundIds[0], -1, node->m_Id, 0.0});
                m_springPointReferences.push_back({groundIds[1], -1, node->m_Id, 0.0});
                m_cellElementIds.push_back(elementId);
            }
            continue;
        }

        if (dynamic_cast<ElementSpring2*>(element.get()) || dynamic_cast<ElementSpringA*>(element.get()))
        {
            if (element->m_pNode.size() >= 2)
            {
                const auto firstNode = element->m_pNode[0].lock();
                const auto secondNode = element->m_pNode[1].lock();
                if (firstNode && secondNode)
                {
                    const auto* spring2 = dynamic_cast<ElementSpring2*>(element.get());
                    const int springAxis = spring2 ? spring2->m_FirstDOF % 3 : 0;
                    addSpringPolyline(elementId, {firstNode->m_X, firstNode->m_Y, firstNode->m_Z},
                                      {secondNode->m_X, secondNode->m_Y, secondNode->m_Z}, firstNode->m_Id,
                                      secondNode->m_Id, springAxis);
                }
            }
            continue;
        }

        std::vector<vtkIdType> connectivity;
        connectivity.reserve(element->m_pNode.size());
        for (const auto& weakNode : element->m_pNode)
        {
            const auto node = weakNode.lock();
            if (!node)
                continue;
            const auto it = pointIds.find(node.get());
            if (it != pointIds.end())
                connectivity.push_back(it->second);
        }
        if (connectivity.size() < 2)
            continue;

        lines->InsertNextCell(static_cast<vtkIdType>(connectivity.size()), connectivity.data());
        m_cellElementIds.push_back(elementId);
        double center[3] = {0.0, 0.0, 0.0};
        for (const vtkIdType pointId : connectivity)
        {
            double point[3] = {};
            points->GetPoint(pointId, point);
            center[0] += point[0];
            center[1] += point[1];
            center[2] += point[2];
        }
        const double inverseCount = 1.0 / static_cast<double>(connectivity.size());
        center[0] *= inverseCount;
        center[1] *= inverseCount;
        center[2] *= inverseCount;
        m_elementLabelPositions.push_back({center[0], center[1], center[2]});
    }

    vtkNew<vtkCellArray> vertices;
    vertices->AllocateEstimate(nodePoints->GetNumberOfPoints(), 1);
    for (vtkIdType pointId = 0; pointId < nodePoints->GetNumberOfPoints(); ++pointId)
        vertices->InsertNextCell(1, &pointId);

    m_points->ShallowCopy(points);
    m_elementData->SetPoints(m_points);
    m_elementData->SetLines(lines);
    vtkNew<vtkPoints> originalPoints;
    originalPoints->DeepCopy(points);
    m_originalElementData->SetPoints(originalPoints);
    m_originalElementData->SetLines(lines);
    m_originalElementData->Modified();
    m_nodeData->SetPoints(nodePoints);
    m_nodeData->SetVerts(vertices);
    m_elementData->Modified();
    m_nodeData->Modified();
    rebuildSolidGeometry();
    m_selectionIsolated = false;
    applyBaseActorVisibility();
    clearSelectionHighlight();
    updateThemeColors();

    if (resetCamera)
    {
        this->resetCamera();
    }
    else if (!m_rotationCenterValid)
        updateRotationCenterToViewportCenter(false);
    updateAdaptiveLabels();
    m_emptyStateLabel->hide();
    m_emptyStateIconLabel->hide();
    m_renderWindow->Render();
}

void ModelViewport::clearModel()
{
    m_structure.reset();
    m_rotationCenterValid = false;
    m_rotationCenterSnapped = false;
    m_rotationCenterDragActive = false;
    m_selectionPressActive = false;
    m_selectionDragActive = false;
    m_cameraModePressActive = false;
    m_zoomRectPressActive = false;
    m_zoomRectDragActive = false;
    if (m_selectionRubberBand)
        m_selectionRubberBand->hide();
    if (m_rotationCenterIndicator)
        m_rotationCenterIndicator->hide();
    m_pointNodeIds.clear();
    m_cellElementIds.clear();
    m_solidCellElementIds.clear();
    m_nodePointIds.clear();
    m_elementLabelPositions.clear();
    m_originalNodeCoordinates.clear();
    m_points->Reset();
    m_elementData->Initialize();
    m_nodeData->Initialize();
    m_nodeLabelData->Initialize();
    m_elementLabelData->Initialize();
    m_solidData->Initialize();
    m_originalElementData->Initialize();
    m_elementData->SetPoints(m_points);
    m_nodeData->SetPoints(m_points);
    m_elementActor->SetVisibility(false);
    m_nodeActor->SetVisibility(false);
    m_nodeLabelActor->SetVisibility(false);
    m_elementLabelActor->SetVisibility(false);
    m_solidActor->SetVisibility(false);
    m_originalElementActor->SetVisibility(false);
    m_resultScalarBar->SetVisibility(false);
    m_resultDisplayActive = false;
    updateNodeDisplaySize();
    clearSelectionHighlight();
    m_emptyStateLabel->show();
    m_emptyStateIconLabel->show();
    requestRender();
}

void ModelViewport::setThemeIndex(int themeIndex)
{
    m_themeIndex = qBound(0, themeIndex, 3);
    updateThemeColors();
    updateRotationCenterIndicatorAppearance();
    requestRender();
}

void ModelViewport::setNodeSize(int nodeSize)
{
    m_nodeSize = qBound(2, nodeSize, 14);
    if (m_nodeActor)
    {
        updateNodeDisplaySize();
        requestRender();
    }
}

void ModelViewport::setNodeLabelsVisible(bool visible)
{
    visible = visible && !m_idLabelsPlaybackLocked;
    const bool changed = m_nodeLabelsVisible != visible;
    m_nodeLabelsVisible = visible;
    if (visible)
        updateAdaptiveLabels();
    m_nodeLabelActor->SetVisibility(visible && hasModel() && !m_selectionIsolated);
    requestRender();
    if (changed)
        emit nodeLabelsVisibilityChanged(visible);
}

void ModelViewport::setAdaptiveNodeLabels(bool adaptive)
{
    if (m_adaptiveNodeLabels == adaptive)
        return;

    m_adaptiveNodeLabels = adaptive;
    if (m_nodeLabelsVisible)
        updateAdaptiveLabels();
    requestRender();
}

void ModelViewport::setElementLabelsVisible(bool visible)
{
    visible = visible && !m_idLabelsPlaybackLocked;
    const bool changed = m_elementLabelsVisible != visible;
    m_elementLabelsVisible = visible;
    if (visible)
        updateAdaptiveLabels();
    m_elementLabelActor->SetVisibility(visible && hasModel() && !m_selectionIsolated);
    requestRender();
    if (changed)
        emit elementLabelsVisibilityChanged(visible);
}

void ModelViewport::setIdLabelsPlaybackLocked(bool locked)
{
    if (m_idLabelsPlaybackLocked == locked)
        return;

    m_idLabelsPlaybackLocked = locked;
    if (locked)
    {
        setNodeLabelsVisible(false);
        setElementLabelsVisible(false);
    }
}

void ModelViewport::setNodesVisible(bool visible)
{
    const bool changed = m_nodesVisible != visible;
    m_nodesVisible = visible;
    m_nodeActor->SetVisibility(visible && hasModel() && !m_selectionIsolated);
    requestRender();
    if (changed)
        emit nodesVisibilityChanged(visible);
}

void ModelViewport::setElementsVisible(bool visible)
{
    const bool changed = m_elementsVisible != visible;
    m_elementsVisible = visible;
    m_elementActor->SetVisibility(visible && hasModel() && !m_selectionIsolated);
    requestRender();
    if (changed)
        emit elementsVisibilityChanged(visible);
}

void ModelViewport::setSolidVisible(bool visible)
{
    const bool changed = m_solidVisible != visible;
    m_solidVisible = visible;
    m_solidActor->SetVisibility(visible && hasModel() && hasSolidGeometry() && !m_selectionIsolated);
    if (!m_selectedElementCellIds.empty())
        refreshSelectionHighlight();
    else if (m_solidSelectionActor)
        m_solidSelectionActor->SetVisibility(false);
    updateNodeDisplaySize();
    requestRender();
    if (changed)
        emit solidVisibilityChanged(visible);
}

void ModelViewport::applyBaseActorVisibility()
{
    const bool visibleModel = hasModel() && !m_selectionIsolated;
    m_nodeActor->SetVisibility(visibleModel && m_nodesVisible);
    m_elementActor->SetVisibility(visibleModel && m_elementsVisible);
    m_solidActor->SetVisibility(visibleModel && m_solidVisible && hasSolidGeometry());
    m_nodeLabelActor->SetVisibility(visibleModel && m_nodeLabelsVisible);
    m_elementLabelActor->SetVisibility(visibleModel && m_elementLabelsVisible);
    if (m_selectionIsolated)
        m_originalElementActor->SetVisibility(false);
}

void ModelViewport::updateNodeDisplaySize()
{
    if (!m_nodeActor)
        return;

    const int displaySize = m_nodeSize;
    m_nodeActor->GetProperty()->SetPointSize(static_cast<float>(displaySize));
    if (m_selectionActor)
        m_selectionActor->GetProperty()->SetPointSize(static_cast<float>(qMax(12, displaySize + 4)));
}

void ModelViewport::resetCamera()
{
    if (!hasModel() || !m_cameraController || !m_cameraController->resetToVisibleBounds())
        return;

    double bounds[6] = {};
    m_renderer->ComputeVisiblePropBounds(bounds);
    m_rotationCenter = {(bounds[0] + bounds[1]) * 0.5, (bounds[2] + bounds[3]) * 0.5, (bounds[4] + bounds[5]) * 0.5};
    m_rotationCenterValid = true;
    m_rotationCenterSnapped = false;
    updateRotationCenterIndicator();
    updateAdaptiveLabels();
    requestRender();
}

void ModelViewport::setInteractionMode(InteractionMode mode)
{
    if (m_interactionMode == mode)
        return;
    m_rotationCenterDragActive = false;
    m_selectionPressActive = false;
    m_selectionDragActive = false;
    m_cameraModePressActive = false;
    m_zoomRectPressActive = false;
    m_zoomRectDragActive = false;
    if (m_selectionRubberBand)
        m_selectionRubberBand->hide();
    m_interactionMode = mode;
    if (auto* style = ModeInteractorStyle::SafeDownCast(m_interactorStyle))
        style->SetInteractionMode(mode);

    switch (mode)
    {
        case InteractionMode::Select:
            setCursor(Qt::ArrowCursor);
            break;
        case InteractionMode::Rotate:
            setCursor(Qt::SizeAllCursor);
            break;
        case InteractionMode::Pan:
            setCursor(Qt::OpenHandCursor);
            break;
        case InteractionMode::Zoom:
            setCursor(Qt::SizeVerCursor);
            break;
        case InteractionMode::ZoomToRect:
            setCursor(Qt::CrossCursor);
            break;
    }
    double center[3] = {};
    ensureRotationCenter(center);
    updateRotationCenterIndicator();
    emit interactionModeChanged(mode);
}

void ModelViewport::setSelectionTarget(SelectionTarget target)
{
    if (m_selectionTarget == target)
        return;

    m_selectionTarget = target;
    if (!m_selectedNodePointIds.empty() || !m_selectedElementCellIds.empty())
    {
        clearSelectionHighlight();
        emit selectionCleared();
    }
    emit selectionTargetChanged(target);
}

void ModelViewport::setStandardView(StandardView view)
{
    if (!hasModel())
        return;

    double bounds[6] = {};
    m_renderer->ComputeVisiblePropBounds(bounds);
    double center[3] = {(bounds[0] + bounds[1]) * 0.5, (bounds[2] + bounds[3]) * 0.5, (bounds[4] + bounds[5]) * 0.5};
    if (m_rotationCenterValid)
    {
        center[0] = m_rotationCenter[0];
        center[1] = m_rotationCenter[1];
        center[2] = m_rotationCenter[2];
    }
    else
    {
        m_rotationCenter = {center[0], center[1], center[2]};
        m_rotationCenterValid = true;
        m_rotationCenterSnapped = false;
    }
    const double dx = bounds[1] - bounds[0];
    const double dy = bounds[3] - bounds[2];
    const double dz = bounds[5] - bounds[4];
    const double distance = qMax(1.0, std::sqrt(dx * dx + dy * dy + dz * dz) * 2.0);

    double direction[3] = {0.0, 0.0, 1.0};
    double viewUp[3] = {0.0, 1.0, 0.0};
    switch (view)
    {
        case StandardView::Front:
            direction[1] = -1.0;
            direction[2] = 0.0;
            viewUp[1] = 0.0;
            viewUp[2] = 1.0;
            break;
        case StandardView::Back:
            direction[1] = 1.0;
            direction[2] = 0.0;
            viewUp[1] = 0.0;
            viewUp[2] = 1.0;
            break;
        case StandardView::Left:
            direction[0] = -1.0;
            direction[2] = 0.0;
            viewUp[1] = 0.0;
            viewUp[2] = 1.0;
            break;
        case StandardView::Right:
            direction[0] = 1.0;
            direction[2] = 0.0;
            viewUp[1] = 0.0;
            viewUp[2] = 1.0;
            break;
        case StandardView::Top:
            break;
        case StandardView::Bottom:
            direction[2] = -1.0;
            break;
        case StandardView::Isometric:
            direction[0] = 1.0;
            direction[1] = 1.0;
            direction[2] = 1.0;
            {
                const double invLength = 1.0 / std::sqrt(3.0);
                direction[0] *= invLength;
                direction[1] *= invLength;
                direction[2] *= invLength;
            }
            break;
    }

    if (!m_cameraController ||
        !m_cameraController->setViewPose({center[0], center[1], center[2]}, distance,
                                         {direction[0], direction[1], direction[2]},
                                         {viewUp[0], viewUp[1], viewUp[2]}))
    {
        return;
    }
    updateRotationCenterIndicator();
    updateAdaptiveLabels();
    requestRender();
}

bool ModelViewport::updateNodePosition(int nodeId, double x, double y, double z)
{
    const auto it = m_nodePointIds.find(nodeId);
    if (!m_structure || it == m_nodePointIds.end())
        return false;

    m_points->SetPoint(it->second, x, y, z);
    if (vtkPoints* nodePoints = m_nodeData->GetPoints())
        nodePoints->SetPoint(it->second, x, y, z);
    m_points->Modified();
    m_elementData->Modified();
    m_nodeData->Modified();
    updateElementLabelPositions();
    rebuildSolidGeometry();
    m_renderer->ResetCameraClippingRange();
    requestRender();
    return true;
}

bool ModelViewport::displayResultFrame(const Hdf5ResultFrame& frame, ResultField field, double deformationScale,
                                       bool showOriginal, bool refreshSolidGeometry)
{
    if (!hasModel() || frame.nodes.empty())
        return false;

    std::unordered_map<int, const Hdf5NodalResult*> nodalResults;
    nodalResults.reserve(frame.nodes.size());
    double maximumDisplacement = 0.0;
    for (const Hdf5NodalResult& result : frame.nodes)
    {
        nodalResults.emplace(result.id, &result);
        maximumDisplacement = std::max(maximumDisplacement, std::sqrt(result.displacement[0] * result.displacement[0] +
                                                                      result.displacement[1] * result.displacement[1] +
                                                                      result.displacement[2] * result.displacement[2]));
    }

    if (deformationScale <= 0.0)
        deformationScale = automaticDeformationScale(maximumDisplacement);
    m_activeDeformationScale = deformationScale;

    auto pointScalars = vtkSmartPointer<vtkDoubleArray>::New();
    pointScalars->SetName("Result");
    pointScalars->SetNumberOfComponents(1);
    pointScalars->SetNumberOfTuples(m_points->GetNumberOfPoints());
    pointScalars->FillValue(0.0);
    double scalarMinimum = std::numeric_limits<double>::max();
    double scalarMaximum = std::numeric_limits<double>::lowest();
    vtkPoints* nodePoints = m_nodeData->GetPoints();

    for (const auto& [nodeId, pointId] : m_nodePointIds)
    {
        const auto originalIt = m_originalNodeCoordinates.find(nodeId);
        if (originalIt == m_originalNodeCoordinates.end())
            continue;
        const auto resultIt = nodalResults.find(nodeId);
        const Hdf5NodalResult* result = resultIt == nodalResults.end() ? nullptr : resultIt->second;
        const auto& original = originalIt->second;
        double displacement[3] = {};
        if (result)
            std::copy_n(result->displacement, 3, displacement);
        m_points->SetPoint(pointId, original[0] + displacement[0] * deformationScale,
                           original[1] + displacement[1] * deformationScale,
                           original[2] + displacement[2] * deformationScale);
        if (nodePoints)
        {
            nodePoints->SetPoint(pointId, original[0] + displacement[0] * deformationScale,
                                 original[1] + displacement[1] * deformationScale,
                                 original[2] + displacement[2] * deformationScale);
        }

        double scalar = 0.0;
        switch (field)
        {
            case ResultField::DisplacementMagnitude:
                scalar = std::sqrt(displacement[0] * displacement[0] + displacement[1] * displacement[1] +
                                   displacement[2] * displacement[2]);
                break;
            case ResultField::DisplacementX:
                scalar = displacement[0];
                break;
            case ResultField::DisplacementY:
                scalar = displacement[1];
                break;
            case ResultField::DisplacementZ:
                scalar = displacement[2];
                break;
            default:
                break;
        }
        pointScalars->SetValue(pointId, scalar);
        if (field <= ResultField::DisplacementZ && result)
        {
            scalarMinimum = std::min(scalarMinimum, scalar);
            scalarMaximum = std::max(scalarMaximum, scalar);
        }
    }

    // 弹簧螺旋线包含额外的绘制点；按两端节点位移插值更新它们，保证动画中弹簧随节点伸缩。
    for (const SpringPointReference& reference : m_springPointReferences)
    {
        double originalPoint[3] = {};
        m_originalElementData->GetPoint(reference.pointId, originalPoint);
        double displacement[3] = {};
        const auto applyNodeDisplacement = [&nodalResults, &displacement](int nodeId, double weight)
        {
            const auto result = nodalResults.find(nodeId);
            if (nodeId < 0 || result == nodalResults.end())
                return;
            for (int component = 0; component < 3; ++component)
                displacement[component] += result->second->displacement[component] * weight;
        };
        applyNodeDisplacement(reference.firstNodeId, 1.0 - reference.secondNodeWeight);
        applyNodeDisplacement(reference.secondNodeId, reference.secondNodeWeight);
        m_points->SetPoint(reference.pointId, originalPoint[0] + displacement[0] * deformationScale,
                           originalPoint[1] + displacement[1] * deformationScale,
                           originalPoint[2] + displacement[2] * deformationScale);
    }
    if (nodePoints)
        nodePoints->Modified();

    const bool nodalField = field <= ResultField::DisplacementZ;
    if (nodalField)
    {
        m_elementData->GetPointData()->SetScalars(pointScalars);
        m_nodeData->GetPointData()->SetScalars(pointScalars);
        m_elementMapper->SetScalarModeToUsePointData();
        m_nodeMapper->SetScalarModeToUsePointData();
        m_nodeMapper->ScalarVisibilityOn();
    }
    else
    {
        std::unordered_map<int, const Hdf5ElementResult*> elementalResults;
        elementalResults.reserve(frame.elements.size());
        for (const Hdf5ElementResult& result : frame.elements)
            elementalResults.emplace(result.id, &result);
        auto cellScalars = vtkSmartPointer<vtkDoubleArray>::New();
        cellScalars->SetName("Result");
        cellScalars->SetNumberOfComponents(1);
        cellScalars->SetNumberOfTuples(static_cast<vtkIdType>(m_cellElementIds.size()));
        for (std::size_t cell = 0; cell < m_cellElementIds.size(); ++cell)
        {
            double scalar = 0.0;
            const auto it = elementalResults.find(m_cellElementIds[cell]);
            if (it != elementalResults.end())
            {
                switch (field)
                {
                    case ResultField::AxialForce:
                        scalar = it->second->axialForce;
                        break;
                    case ResultField::Stress:
                        scalar = it->second->currentStress;
                        break;
                    case ResultField::Strain:
                        scalar = it->second->strain;
                        break;
                    default:
                        break;
                }
                scalarMinimum = std::min(scalarMinimum, scalar);
                scalarMaximum = std::max(scalarMaximum, scalar);
            }
            cellScalars->SetValue(static_cast<vtkIdType>(cell), scalar);
        }
        m_elementData->GetCellData()->SetScalars(cellScalars);
        m_elementMapper->SetScalarModeToUseCellData();
        m_nodeMapper->ScalarVisibilityOff();
    }

    if (!std::isfinite(scalarMinimum) || !std::isfinite(scalarMaximum))
        scalarMinimum = scalarMaximum = 0.0;
    if (m_resultScalarRangeLocked)
    {
        scalarMinimum = m_resultScalarMinimum;
        scalarMaximum = m_resultScalarMaximum;
    }
    if (std::abs(scalarMaximum - scalarMinimum) < 1.0e-30)
    {
        const double delta = std::max(1.0, std::abs(scalarMaximum)) * 1.0e-6;
        scalarMinimum -= delta;
        scalarMaximum += delta;
    }
    m_resultLookupTable->SetRange(scalarMinimum, scalarMaximum);
    m_resultLookupTable->Build();
    m_elementMapper->SetLookupTable(m_resultLookupTable);
    m_elementMapper->SetScalarRange(scalarMinimum, scalarMaximum);
    m_elementMapper->ScalarVisibilityOn();
    m_nodeMapper->SetLookupTable(m_resultLookupTable);
    m_nodeMapper->SetScalarRange(scalarMinimum, scalarMaximum);
    const char* titles[] = {"|U|", "U-X", "U-Y", "U-Z", "Force", "Stress", "Strain"};
    m_resultScalarBar->SetTitle(titles[static_cast<int>(field)]);
    m_resultScalarBar->SetVisibility(true);
    m_originalElementActor->SetVisibility(showOriginal && !m_selectionIsolated);
    m_resultDisplayActive = true;

    m_points->Modified();
    m_elementData->Modified();
    m_nodeData->Modified();
    // 播放动画时仅更新已存在的线框点和标量。重建实体会为每个单元重新生成截面、
    // 多边形和法线；大模型在 16 ms 定时器内执行这一工作会阻塞 GUI 线程。
    // 暂停或拖动到某帧时再刷新实体，即可保持几何正确且不会拖慢播放。
    if (refreshSolidGeometry)
    {
        updateElementLabelPositions();
        rebuildSolidGeometry();
        m_renderer->ResetCameraClippingRange();
    }
    else if (m_solidActor)
    {
        // 旧实体截面不能与更新后的节点位置混用；播放中暂时隐藏，暂停时再重建恢复。
        m_solidActor->SetVisibility(false);
    }
    requestRender();
    return true;
}

void ModelViewport::setResultScalarRange(double minimum, double maximum)
{
    if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum)
    {
        clearResultScalarRange();
        return;
    }
    m_resultScalarMinimum = minimum;
    m_resultScalarMaximum = maximum;
    m_resultScalarRangeLocked = true;
}

void ModelViewport::clearResultScalarRange()
{
    m_resultScalarMinimum = 0.0;
    m_resultScalarMaximum = 0.0;
    m_resultScalarRangeLocked = false;
}

void ModelViewport::clearResultDisplay()
{
    if (!hasModel())
        return;
    for (const auto& [nodeId, coordinates] : m_originalNodeCoordinates)
    {
        const auto point = m_nodePointIds.find(nodeId);
        if (point != m_nodePointIds.end())
        {
            m_points->SetPoint(point->second, coordinates.data());
            if (vtkPoints* nodePoints = m_nodeData->GetPoints())
                nodePoints->SetPoint(point->second, coordinates.data());
        }
    }
    if (vtkPoints* nodePoints = m_nodeData->GetPoints())
        nodePoints->Modified();
    m_elementData->GetPointData()->SetScalars(nullptr);
    m_elementData->GetCellData()->SetScalars(nullptr);
    m_nodeData->GetPointData()->SetScalars(nullptr);
    m_elementMapper->ScalarVisibilityOff();
    m_nodeMapper->ScalarVisibilityOff();
    m_originalElementActor->SetVisibility(false);
    m_resultScalarBar->SetVisibility(false);
    m_resultDisplayActive = false;
    m_activeDeformationScale = 1.0;
    clearResultScalarRange();
    m_points->Modified();
    m_elementData->Modified();
    m_nodeData->Modified();
    updateElementLabelPositions();
    rebuildSolidGeometry();
    updateThemeColors();
    m_renderer->ResetCameraClippingRange();
    requestRender();
}

double ModelViewport::activeDeformationScale() const
{
    return m_activeDeformationScale;
}

double ModelViewport::automaticDeformationScale(double maximumDisplacement) const
{
    if (!std::isfinite(maximumDisplacement) || maximumDisplacement <= 1.0e-30)
        return 1.0;

    double bounds[6] = {};
    m_originalElementData->GetBounds(bounds);
    const double modelSize = std::max({bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4], 1.0e-12});

    // 将最大位移缩放到模型尺寸的约 5%；动力结果末帧可能接近平衡位置，
    // 因此调用方应传入完整结果历程的最大位移。
    constexpr double targetExtentRatio = 0.05;
    constexpr double minimumAutomaticScale = 0.01;
    constexpr double maximumAutomaticScale = 10.0;
    return std::clamp(modelSize * targetExtentRatio / maximumDisplacement, minimumAutomaticScale,
                      maximumAutomaticScale);
}

void ModelViewport::updateElementLabelPositions()
{
    m_elementLabelPositions.resize(static_cast<std::size_t>(m_elementData->GetNumberOfCells()));

    vtkNew<vtkIdList> pointIds;
    for (vtkIdType cellId = 0; cellId < m_elementData->GetNumberOfCells(); ++cellId)
    {
        m_elementData->GetCellPoints(cellId, pointIds);
        if (pointIds->GetNumberOfIds() == 0)
            continue;

        double center[3] = {0.0, 0.0, 0.0};
        for (vtkIdType index = 0; index < pointIds->GetNumberOfIds(); ++index)
        {
            double point[3] = {};
            m_points->GetPoint(pointIds->GetId(index), point);
            center[0] += point[0];
            center[1] += point[1];
            center[2] += point[2];
        }
        const double inverseCount = 1.0 / static_cast<double>(pointIds->GetNumberOfIds());
        m_elementLabelPositions[static_cast<std::size_t>(cellId)] = {center[0] * inverseCount, center[1] * inverseCount,
                                                                     center[2] * inverseCount};
    }
    updateAdaptiveLabels();
}

void ModelViewport::updateAdaptiveLabels()
{
    constexpr int maximumLabelsPerType = 1600;
    if (!m_renderer || !hasModel())
    {
        m_nodeLabelData->Initialize();
        m_elementLabelData->Initialize();
        return;
    }

    const int* rendererSize = m_renderer->GetSize();
    if (!rendererSize || rendererSize[0] <= 0 || rendererSize[1] <= 0)
        return;
    const double gridSize = std::max(36.0, 52.0 * devicePixelRatioF());
    const int gridColumnCount = std::max(1, static_cast<int>(std::ceil(rendererSize[0] / gridSize)));

    auto projectToGrid = [this, rendererSize, gridSize, gridColumnCount](const double world[3], long long& gridKey)
    {
        m_renderer->SetWorldPoint(world[0], world[1], world[2], 1.0);
        m_renderer->WorldToDisplay();
        const double* display = m_renderer->GetDisplayPoint();
        if (!display || !std::isfinite(display[0]) || !std::isfinite(display[1]) || !std::isfinite(display[2]) ||
            display[2] < 0.0 || display[2] > 1.0 || display[0] < 0.0 || display[0] >= rendererSize[0] ||
            display[1] < 0.0 || display[1] >= rendererSize[1])
        {
            return false;
        }
        const long long column = static_cast<long long>(display[0] / gridSize);
        const long long row = static_cast<long long>(display[1] / gridSize);
        gridKey = row * gridColumnCount + column;
        return true;
    };

    m_nodeLabelData->Initialize();
    m_elementLabelData->Initialize();

    vtkNew<vtkPoints> nodeLabelPoints;
    nodeLabelPoints->SetDataTypeToDouble();
    vtkNew<vtkCellArray> nodeLabelVertices;
    vtkNew<vtkIntArray> nodeLabelIds;
    nodeLabelIds->SetName("NodeIds");
    nodeLabelIds->SetNumberOfComponents(1);

    vtkNew<vtkPoints> elementLabelPoints;
    elementLabelPoints->SetDataTypeToDouble();
    vtkNew<vtkCellArray> elementLabelVertices;
    vtkNew<vtkIntArray> elementLabelIds;
    elementLabelIds->SetName("ElementIds");
    elementLabelIds->SetNumberOfComponents(1);

    auto appendNodeLabel = [&nodeLabelPoints, &nodeLabelVertices, &nodeLabelIds](const double world[3], int id)
    {
        const vtkIdType labelPointId = nodeLabelPoints->InsertNextPoint(world);
        nodeLabelVertices->InsertNextCell(1, &labelPointId);
        nodeLabelIds->InsertNextValue(id);
    };
    auto appendElementLabel = [&elementLabelPoints, &elementLabelVertices, &elementLabelIds](const double world[3], int id)
    {
        const vtkIdType labelPointId = elementLabelPoints->InsertNextPoint(world);
        elementLabelVertices->InsertNextCell(1, &labelPointId);
        elementLabelIds->InsertNextValue(id);
    };

    // 节点和单元标签共用屏幕网格，较小模型 ID 优先占位，避免标签重叠。
    std::unordered_set<long long> occupiedCells;
    occupiedCells.reserve(static_cast<std::size_t>(maximumLabelsPerType) * 2);
    const vtkIdType pointCount =
        std::min<vtkIdType>(m_points->GetNumberOfPoints(), static_cast<vtkIdType>(m_pointNodeIds.size()));

    if (m_nodeLabelsVisible && !m_adaptiveNodeLabels)
    {
        // 全部节点标签遵从用户的显式设置，同时占据可见网格，避免单元标签遮挡。
        for (vtkIdType pointId = 0; pointId < pointCount; ++pointId)
        {
            double world[3] = {};
            m_points->GetPoint(pointId, world);
            appendNodeLabel(world, m_pointNodeIds[static_cast<std::size_t>(pointId)]);
            long long gridKey = 0;
            if (projectToGrid(world, gridKey))
                occupiedCells.insert(gridKey);
        }
    }

    struct LabelCandidate
    {
        bool isNode = false;
        bool isSelected = false;
        int id = 0;
        std::array<double, 3> world{};
    };
    std::vector<LabelCandidate> candidates;
    if (m_nodeLabelsVisible && m_adaptiveNodeLabels)
        candidates.reserve(static_cast<std::size_t>(pointCount) + m_elementLabelPositions.size());
    else if (m_elementLabelsVisible)
        candidates.reserve(m_elementLabelPositions.size());

    if (m_nodeLabelsVisible && m_adaptiveNodeLabels)
    {
        for (vtkIdType pointId = 0; pointId < pointCount; ++pointId)
        {
            double world[3] = {};
            m_points->GetPoint(pointId, world);
            candidates.push_back({true, m_selectedNodePointIds.contains(pointId),
                                  m_pointNodeIds[static_cast<std::size_t>(pointId)], {world[0], world[1], world[2]}});
        }
    }
    if (m_elementLabelsVisible)
    {
        const std::size_t candidateCount = std::min(m_elementLabelPositions.size(), m_cellElementIds.size());
        for (std::size_t index = 0; index < candidateCount; ++index)
            candidates.push_back({false, m_selectedElementCellIds.contains(static_cast<vtkIdType>(index)),
                                  m_cellElementIds[index], m_elementLabelPositions[index]});
    }

    std::sort(candidates.begin(), candidates.end(), [](const LabelCandidate& lhs, const LabelCandidate& rhs)
              {
                  // 自适应显示时保留选中对象标签，其余标签按稳定顺序填充网格。
                  if (lhs.isSelected != rhs.isSelected)
                      return lhs.isSelected;
                  if (lhs.id != rhs.id)
                      return lhs.id < rhs.id;
                  return lhs.isNode && !rhs.isNode;
              });

    int adaptiveNodeCount = 0;
    int elementCount = 0;
    for (const LabelCandidate& candidate : candidates)
    {
        if ((candidate.isNode && adaptiveNodeCount >= maximumLabelsPerType) ||
            (!candidate.isNode && elementCount >= maximumLabelsPerType))
        {
            continue;
        }
        const double world[3] = {candidate.world[0], candidate.world[1], candidate.world[2]};
        long long gridKey = 0;
        if (!projectToGrid(world, gridKey) || !occupiedCells.insert(gridKey).second)
            continue;
        if (candidate.isNode)
        {
            appendNodeLabel(world, candidate.id);
            ++adaptiveNodeCount;
        }
        else
        {
            appendElementLabel(world, candidate.id);
            ++elementCount;
        }
    }

    m_nodeLabelData->SetPoints(nodeLabelPoints);
    m_nodeLabelData->SetVerts(nodeLabelVertices);
    m_nodeLabelData->GetPointData()->SetScalars(nodeLabelIds);
    m_nodeLabelData->Modified();
    m_elementLabelData->SetPoints(elementLabelPoints);
    m_elementLabelData->SetVerts(elementLabelVertices);
    m_elementLabelData->GetPointData()->SetScalars(elementLabelIds);
    m_elementLabelData->Modified();
}

void ModelViewport::rebuildSolidGeometry()
{
    m_solidData->Initialize();
    m_solidCellElementIds.clear();
    if (!m_structure)
    {
        m_solidActor->SetVisibility(false);
        m_solidSelectionActor->SetVisibility(false);
        return;
    }

    vtkNew<vtkPoints> solidPoints;
    solidPoints->SetDataTypeToDouble();
    vtkNew<vtkCellArray> solidPolys;

    const SolidVector3 globalAxes[3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    for (const auto& [elementId, element] : m_structure->Elements())
    {
        if (!element || element->m_pNode.size() < 2)
            continue;
        const auto startNode = element->m_pNode.front().lock();
        const auto endNode = element->m_pNode.back().lock();
        const auto property = element->m_pProperty.lock();
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!startNode || !endNode || !section)
            continue;

        SolidVector3 start = {startNode->m_X, startNode->m_Y, startNode->m_Z};
        SolidVector3 end = {endNode->m_X, endNode->m_Y, endNode->m_Z};
        const auto startPoint = m_nodePointIds.find(startNode->m_Id);
        if (startPoint != m_nodePointIds.end())
            m_points->GetPoint(startPoint->second, start.data());
        const auto endPoint = m_nodePointIds.find(endNode->m_Id);
        if (endPoint != m_nodePointIds.end())
            m_points->GetPoint(endPoint->second, end.data());
        SolidVector3 axis = subtract(end, start);
        if (!normalize(axis))
            continue;

        int referenceIndex = 0;
        double minimumAlignment = std::abs(dot(axis, globalAxes[0]));
        for (int index = 1; index < 3; ++index)
        {
            const double alignment = std::abs(dot(axis, globalAxes[index]));
            if (alignment < minimumAlignment)
            {
                referenceIndex = index;
                minimumAlignment = alignment;
            }
        }
        SolidVector3 localY = cross(axis, globalAxes[referenceIndex]);
        if (!normalize(localY))
            continue;
        SolidVector3 localZ = cross(axis, localY);
        if (!normalize(localZ))
            continue;

        const std::vector<SectionPoint> contour = buildSectionContour(section);
        if (contour.size() < 3)
            continue;

        const vtkIdType startBase = solidPoints->GetNumberOfPoints();
        for (const SectionPoint& point : contour)
        {
            solidPoints->InsertNextPoint(start[0] + point.y * localY[0] + point.z * localZ[0],
                                         start[1] + point.y * localY[1] + point.z * localZ[1],
                                         start[2] + point.y * localY[2] + point.z * localZ[2]);
        }
        const vtkIdType endBase = solidPoints->GetNumberOfPoints();
        for (const SectionPoint& point : contour)
        {
            solidPoints->InsertNextPoint(end[0] + point.y * localY[0] + point.z * localZ[0],
                                         end[1] + point.y * localY[1] + point.z * localZ[1],
                                         end[2] + point.y * localY[2] + point.z * localZ[2]);
        }

        const vtkIdType contourCount = static_cast<vtkIdType>(contour.size());
        for (vtkIdType index = 0; index < contourCount; ++index)
        {
            const vtkIdType next = (index + 1) % contourCount;
            const vtkIdType side[4] = {startBase + index, startBase + next, endBase + next, endBase + index};
            solidPolys->InsertNextCell(4, side);
            m_solidCellElementIds.push_back(elementId);
        }

        std::vector<vtkIdType> startCap(static_cast<std::size_t>(contourCount));
        std::vector<vtkIdType> endCap(static_cast<std::size_t>(contourCount));
        for (vtkIdType index = 0; index < contourCount; ++index)
        {
            startCap[static_cast<std::size_t>(index)] = startBase + contourCount - 1 - index;
            endCap[static_cast<std::size_t>(index)] = endBase + index;
        }
        solidPolys->InsertNextCell(contourCount, startCap.data());
        m_solidCellElementIds.push_back(elementId);
        solidPolys->InsertNextCell(contourCount, endCap.data());
        m_solidCellElementIds.push_back(elementId);
    }

    if (solidPolys->GetNumberOfCells() > 0)
    {
        vtkNew<vtkPolyData> rawSolidData;
        rawSolidData->SetPoints(solidPoints);
        rawSolidData->SetPolys(solidPolys);
        vtkNew<vtkPolyDataNormals> normals;
        normals->SetInputData(rawSolidData);
        normals->ConsistencyOn();
        normals->AutoOrientNormalsOn();
        normals->SplittingOff();
        normals->Update();
        m_solidData->ShallowCopy(normals->GetOutput());
    }
    m_solidData->Modified();
    m_solidActor->SetVisibility(m_solidVisible && hasSolidGeometry());
    if (!m_selectedElementCellIds.empty())
        refreshSelectionHighlight();
    else
        m_solidSelectionActor->SetVisibility(false);
    updateNodeDisplaySize();
}

bool ModelViewport::hasModel() const
{
    return static_cast<bool>(m_structure);
}

bool ModelViewport::hasSolidGeometry() const
{
    return m_solidData && m_solidData->GetNumberOfPolys() > 0;
}

void ModelViewport::resizeEvent(QResizeEvent* event)
{
    QVTKOpenGLNativeWidget::resizeEvent(event);
    updateEmptyStateGeometry();
    QTimer::singleShot(0, this,
                       [this]()
                       {
                           updateAdaptiveLabels();
                           updateRotationCenterIndicator();
                           requestRender();
                       });
}

void ModelViewport::mousePressEvent(QMouseEvent* event)
{
    if (hasModel() && event->button() == Qt::LeftButton && m_interactionMode == InteractionMode::Select &&
        m_rotationCenterIndicator->isVisible() &&
        m_rotationCenterIndicator->geometry().contains(event->position().toPoint()))
    {
        m_rotationCenterDragActive = true;
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    m_cameraModePressActive = hasModel() && event->button() == Qt::LeftButton &&
                              m_interactionMode != InteractionMode::Select;
    if (m_cameraModePressActive)
        m_cameraModePressPosition = event->position();
    m_selectionPressActive = m_structure && event->button() == Qt::LeftButton &&
                             m_interactionMode == InteractionMode::Select;
    if (m_selectionPressActive)
    {
        m_selectionPressPosition = event->position();
        m_selectionPressModifiers = event->modifiers();
        m_selectionDragActive = false;
        if (m_selectionRubberBand)
            m_selectionRubberBand->hide();
    }
    m_zoomRectPressActive = m_structure && event->button() == Qt::LeftButton &&
                            m_interactionMode == InteractionMode::ZoomToRect;
    if (m_zoomRectPressActive)
    {
        m_zoomRectPressPosition = event->position();
        m_zoomRectDragActive = false;
        if (m_selectionRubberBand)
            m_selectionRubberBand->hide();
    }
    QVTKOpenGLNativeWidget::mousePressEvent(event);
}

void ModelViewport::mouseMoveEvent(QMouseEvent* event)
{
    if (m_rotationCenterDragActive)
    {
        const QPoint position(qBound(0, qRound(event->position().x()), width() - 1),
                              qBound(0, qRound(event->position().y()), height() - 1));
        m_rotationCenterIndicator->move(position - QPoint(m_rotationCenterIndicator->width() / 2,
                                                          m_rotationCenterIndicator->height() / 2));
        event->accept();
        return;
    }
    if (m_interactionMode == InteractionMode::Select)
    {
        const bool overCenter = m_rotationCenterIndicator->isVisible() &&
                                m_rotationCenterIndicator->geometry().contains(event->position().toPoint());
        setCursor(overCenter ? Qt::OpenHandCursor : Qt::ArrowCursor);
    }
    if (m_selectionPressActive && (event->buttons() & Qt::LeftButton))
    {
        constexpr qreal selectionClickTolerancePixels = 3.0;
        const QPointF movement = event->position() - m_selectionPressPosition;
        if (!m_selectionDragActive &&
            movement.x() * movement.x() + movement.y() * movement.y() >
                selectionClickTolerancePixels * selectionClickTolerancePixels)
        {
            m_selectionDragActive = true;
        }
        if (m_selectionDragActive && m_selectionRubberBand)
        {
            const QRect selectionRect(m_selectionPressPosition.toPoint(), event->position().toPoint());
            m_selectionRubberBand->setGeometry(selectionRect.normalized().adjusted(0, 0, 1, 1));
            m_selectionRubberBand->show();
            m_selectionRubberBand->raise();
        }
    }
    if (m_zoomRectPressActive && (event->buttons() & Qt::LeftButton))
    {
        constexpr qreal rectangleZoomTolerancePixels = 3.0;
        const QPointF movement = event->position() - m_zoomRectPressPosition;
        if (!m_zoomRectDragActive &&
            movement.x() * movement.x() + movement.y() * movement.y() >
                rectangleZoomTolerancePixels * rectangleZoomTolerancePixels)
        {
            m_zoomRectDragActive = true;
        }
        if (m_zoomRectDragActive && m_selectionRubberBand)
        {
            const QRect zoomRect(m_zoomRectPressPosition.toPoint(), event->position().toPoint());
            m_selectionRubberBand->setGeometry(zoomRect.normalized().adjusted(0, 0, 1, 1));
            m_selectionRubberBand->show();
            m_selectionRubberBand->raise();
        }
    }
    QVTKOpenGLNativeWidget::mouseMoveEvent(event);
}

void ModelViewport::mouseReleaseEvent(QMouseEvent* event)
{
    if (m_rotationCenterDragActive && event->button() == Qt::LeftButton)
    {
        m_rotationCenterDragActive = false;
        setCursor(Qt::ArrowCursor);
        const QPoint position(qBound(0, qRound(event->position().x()), width() - 1),
                              qBound(0, qRound(event->position().y()), height() - 1));
        setRotationCenterAt(position.x(), position.y(), true);
        event->accept();
        return;
    }
    const bool selectClick =
        m_structure && event->button() == Qt::LeftButton && m_interactionMode == InteractionMode::Select;
    const QPointF clickPosition = event->position();
    constexpr qreal selectionClickTolerancePixels = 3.0;
    const QPointF movement = clickPosition - m_selectionPressPosition;
    const QPointF cameraMovement = clickPosition - m_cameraModePressPosition;
    const bool cameraModeRelease = event->button() == Qt::LeftButton && m_cameraModePressActive;
    const bool cameraModeClick =
        cameraModeRelease &&
        cameraMovement.x() * cameraMovement.x() + cameraMovement.y() * cameraMovement.y() <=
            selectionClickTolerancePixels * selectionClickTolerancePixels;
    const bool stationarySelectionClick =
        selectClick && m_selectionPressActive &&
        movement.x() * movement.x() + movement.y() * movement.y() <=
            selectionClickTolerancePixels * selectionClickTolerancePixels;
    const bool rectangleSelection = selectClick && m_selectionPressActive && m_selectionDragActive;
    const bool additive = (cameraModeClick ? event->modifiers() : m_selectionPressModifiers).testFlag(Qt::ControlModifier);
    const QRectF selectionRect(m_selectionPressPosition, clickPosition);
    const bool rectangleZoom = m_structure && event->button() == Qt::LeftButton &&
                               m_interactionMode == InteractionMode::ZoomToRect && m_zoomRectPressActive &&
                               m_zoomRectDragActive;
    const QRectF zoomRect(m_zoomRectPressPosition, clickPosition);
    m_selectionPressActive = false;
    m_selectionDragActive = false;
    m_cameraModePressActive = false;
    m_zoomRectPressActive = false;
    m_zoomRectDragActive = false;
    if (m_selectionRubberBand)
        m_selectionRubberBand->hide();
    QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
    if (stationarySelectionClick || cameraModeClick)
    {
        if (m_skipNextSelectionRelease)
            m_skipNextSelectionRelease = false;
        else
            performSelectionAt(clickPosition.x(), clickPosition.y(), additive);
    }
    else if (rectangleSelection)
    {
        performSelectionInRect(selectionRect.normalized(), additive);
    }
    else if (rectangleZoom)
    {
        zoomToWidgetRect(zoomRect.normalized());
    }
    if (cameraModeRelease)
        setInteractionMode(InteractionMode::Select);
    QTimer::singleShot(0, this,
                       [this]()
                       {
                           updateAdaptiveLabels();
                           requestRender();
                       });
}

void ModelViewport::wheelEvent(QWheelEvent* event)
{
    if (!hasModel() || event->angleDelta().y() == 0)
    {
        QVTKOpenGLNativeWidget::wheelEvent(event);
        return;
    }

    const double steps = static_cast<double>(event->angleDelta().y()) / 120.0;
    const double factor = std::pow(1.18, steps);
    if (!zoomAtRotationCenter(factor))
    {
        QVTKOpenGLNativeWidget::wheelEvent(event);
        return;
    }
    updateAdaptiveLabels();
    updateRotationCenterIndicator();
    requestRender();
    event->accept();
}

void ModelViewport::keyPressEvent(QKeyEvent* event)
{
    if (event->modifiers() == Qt::NoModifier && event->key() == Qt::Key_Escape)
    {
        const bool wasIsolated = m_selectionIsolated;
        const bool hadSelection = !m_selectedNodePointIds.empty() || !m_selectedElementCellIds.empty();
        const bool wasCameraMode = m_interactionMode != InteractionMode::Select;
        if (wasIsolated)
            setSelectionIsolation(false);
        if (hadSelection)
        {
            clearSelectionHighlight();
            emit selectionCleared();
        }
        if (wasCameraMode)
            setInteractionMode(InteractionMode::Select);
        if (wasIsolated || hadSelection || wasCameraMode)
            event->accept();
        else
            QVTKOpenGLNativeWidget::keyPressEvent(event);
        return;
    }
    if (event->modifiers() == Qt::NoModifier && event->key() == Qt::Key_I &&
        setSelectionIsolation(!m_selectionIsolated))
    {
        event->accept();
        return;
    }
    if (event->modifiers() == Qt::NoModifier && event->key() == Qt::Key_Home && hasModel())
    {
        resetCamera();
        event->accept();
        return;
    }
    if (event->modifiers() == Qt::NoModifier)
    {
        const std::array<std::pair<int, InteractionMode>, 5> interactionShortcuts = {
            std::pair{Qt::Key_S, InteractionMode::Select}, std::pair{Qt::Key_R, InteractionMode::Rotate},
            std::pair{Qt::Key_P, InteractionMode::Pan},    std::pair{Qt::Key_Z, InteractionMode::Zoom},
            std::pair{Qt::Key_B, InteractionMode::ZoomToRect}};
        for (const auto& [key, mode] : interactionShortcuts)
        {
            if (event->key() == key)
            {
                setInteractionMode(mode);
                event->accept();
                return;
            }
        }
        const std::array<std::pair<int, SelectionTarget>, 3> selectionTargetShortcuts = {
            std::pair{Qt::Key_A, SelectionTarget::Auto}, std::pair{Qt::Key_N, SelectionTarget::NodesOnly},
            std::pair{Qt::Key_E, SelectionTarget::ElementsOnly}};
        for (const auto& [key, target] : selectionTargetShortcuts)
        {
            if (event->key() == key)
            {
                setSelectionTarget(target);
                setInteractionMode(InteractionMode::Select);
                event->accept();
                return;
            }
        }
        const std::array<std::pair<int, StandardView>, 7> standardViewShortcuts = {
            std::pair{Qt::Key_1, StandardView::Front}, std::pair{Qt::Key_2, StandardView::Back},
            std::pair{Qt::Key_3, StandardView::Left},  std::pair{Qt::Key_4, StandardView::Right},
            std::pair{Qt::Key_5, StandardView::Top},   std::pair{Qt::Key_6, StandardView::Bottom},
            std::pair{Qt::Key_0, StandardView::Isometric}};
        for (const auto& [key, view] : standardViewShortcuts)
        {
            if (event->key() == key && hasModel())
            {
                setStandardView(view);
                event->accept();
                return;
            }
        }
    }
    QVTKOpenGLNativeWidget::keyPressEvent(event);
}

void ModelViewport::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (m_structure && event->button() == Qt::LeftButton && m_interactionMode == InteractionMode::Select)
        m_skipNextSelectionRelease = true;
    QVTKOpenGLNativeWidget::mouseDoubleClickEvent(event);
}

bool ModelViewport::displayToFocalPlaneWorld(double displayX, double displayY, double world[3]) const
{
    return m_cameraController && m_cameraController->displayToFocalPlaneWorld(displayX, displayY, world);
}

bool ModelViewport::panByDisplayDelta(int deltaX, int deltaY)
{
    return hasModel() && m_cameraController && m_cameraController->panByDisplayDelta(deltaX, deltaY);
}

bool ModelViewport::zoomAtRotationCenter(double factor)
{
    if (!hasModel() || !m_cameraController || !std::isfinite(factor) || factor <= 0.0)
        return false;

    double center[3] = {};
    if (!ensureRotationCenter(center))
        return false;
    return m_cameraController->zoomAtWorldPoint(center, factor);
}

bool ModelViewport::zoomToWidgetRect(const QRectF& widgetRect)
{
    if (!hasModel() || !m_cameraController || widgetRect.width() < 5.0 || widgetRect.height() < 5.0)
        return false;

    const double scale = devicePixelRatioF();
    std::array<double, 3> focalPlaneCenter{};
    if (!m_cameraController->zoomToDisplayRect(widgetRect.left() * scale, (height() - widgetRect.bottom()) * scale,
                                               widgetRect.right() * scale, (height() - widgetRect.top()) * scale,
                                               focalPlaneCenter))
    {
        return false;
    }
    m_rotationCenter = focalPlaneCenter;
    m_rotationCenterValid = true;
    m_rotationCenterSnapped = false;
    updateRotationCenterIndicatorAppearance();
    updateRotationCenterIndicator();
    updateAdaptiveLabels();
    requestRender();
    return true;
}

bool ModelViewport::ensureRotationCenter(double center[3])
{
    if (!hasModel() || !center)
        return false;
    if (!m_rotationCenterValid)
        updateRotationCenterToViewportCenter(false);
    if (!m_rotationCenterValid)
        return false;
    center[0] = m_rotationCenter[0];
    center[1] = m_rotationCenter[1];
    center[2] = m_rotationCenter[2];
    return true;
}

void ModelViewport::updateRotationCenterToViewportCenter(bool snapToGeometry)
{
    if (!hasModel())
        return;
    const int* renderSize = m_renderWindow->GetSize();
    if (!renderSize || renderSize[0] <= 0 || renderSize[1] <= 0)
        return;
    const double displayX = renderSize[0] * 0.5;
    const double displayY = renderSize[1] * 0.5;

    double center[3] = {};
    if (!displayToFocalPlaneWorld(displayX, displayY, center))
        return;
    bool snapped = false;

    if (snapToGeometry)
    {
        vtkNew<vtkCellPicker> picker;
        picker->PickFromListOn();
        picker->SetTolerance(0.008);
        const QList<vtkActor*> actors = {m_nodeActor.GetPointer(), m_elementActor.GetPointer(),
                                         m_solidActor.GetPointer()};
        std::array<int, 3> oldPickable = {};
        for (int index = 0; index < actors.size(); ++index)
        {
            vtkActor* actor = actors.at(index);
            oldPickable[static_cast<std::size_t>(index)] = actor->GetPickable();
            if (actor->GetVisibility())
            {
                actor->PickableOn();
                picker->AddPickList(actor);
            }
        }
        snapped = picker->Pick(displayX, displayY, 0.0, m_renderer) != 0;
        if (snapped)
        {
            double* picked = picker->GetPickPosition();
            if (picked && std::isfinite(picked[0]) && std::isfinite(picked[1]) && std::isfinite(picked[2]))
            {
                center[0] = picked[0];
                center[1] = picked[1];
                center[2] = picked[2];
            }
            else
                snapped = false;
        }
        for (int index = 0; index < actors.size(); ++index)
            actors.at(index)->SetPickable(oldPickable[static_cast<std::size_t>(index)]);
    }

    m_rotationCenter = {center[0], center[1], center[2]};
    m_rotationCenterValid = true;
    m_rotationCenterSnapped = snapped;
    updateRotationCenterIndicatorAppearance();
    updateRotationCenterIndicator();
    requestRender();
}

void ModelViewport::updateRotationCenterIndicatorAppearance()
{
    if (!m_rotationCenterIndicator)
        return;
    const RenderColors colors = colorsForTheme(m_themeIndex);
    const QColor accent = QColor::fromRgbF(colors.node[0], colors.node[1], colors.node[2]);
    const QColor outline = m_themeIndex == 3 ? QColor(255, 255, 255, 220) : QColor(5, 12, 20, 220);
    QPixmap pixmap(56, 56);
    pixmap.setDevicePixelRatio(2.0);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.scale(2.0, 2.0);
    painter.setPen(QPen(outline, 4.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawEllipse(QPointF(14, 14), 5.5, 5.5);
    painter.drawLine(QPointF(14, 1.5), QPointF(14, 8));
    painter.drawLine(QPointF(14, 20), QPointF(14, 26.5));
    painter.drawLine(QPointF(1.5, 14), QPointF(8, 14));
    painter.drawLine(QPointF(20, 14), QPointF(26.5, 14));
    painter.setPen(QPen(accent, 2.0, Qt::SolidLine, Qt::RoundCap));
    painter.drawEllipse(QPointF(14, 14), 5.5, 5.5);
    painter.drawLine(QPointF(14, 1.5), QPointF(14, 8));
    painter.drawLine(QPointF(14, 20), QPointF(14, 26.5));
    painter.drawLine(QPointF(1.5, 14), QPointF(8, 14));
    painter.drawLine(QPointF(20, 14), QPointF(26.5, 14));
    if (m_rotationCenterSnapped)
    {
        painter.setPen(Qt::NoPen);
        painter.setBrush(accent);
        painter.drawEllipse(QPointF(14, 14), 2.4, 2.4);
    }
    m_rotationCenterIndicator->setPixmap(pixmap);
}

void ModelViewport::updateRotationCenterIndicator()
{
    if (!m_rotationCenterIndicator || !m_rotationCenterValid || !hasModel())
    {
        if (m_rotationCenterIndicator)
            m_rotationCenterIndicator->hide();
        return;
    }

    m_renderer->SetWorldPoint(m_rotationCenter[0], m_rotationCenter[1], m_rotationCenter[2], 1.0);
    m_renderer->WorldToDisplay();
    const double* display = m_renderer->GetDisplayPoint();
    if (!display || !std::isfinite(display[0]) || !std::isfinite(display[1]))
    {
        m_rotationCenterIndicator->hide();
        return;
    }
    const double scale = devicePixelRatioF();
    const QPoint center(qRound(display[0] / scale), qRound(height() - display[1] / scale));
    if (!rect().adjusted(-14, -14, 14, 14).contains(center))
    {
        m_rotationCenterIndicator->hide();
        return;
    }
    m_rotationCenterIndicator->move(center.x() - m_rotationCenterIndicator->width() / 2,
                                    center.y() - m_rotationCenterIndicator->height() / 2);
    m_rotationCenterIndicator->setToolTip(
        QStringLiteral("旋转/缩放中心（%1）\nX = %2\nY = %3\nZ = %4\n选择模式下可拖动")
            .arg(m_rotationCenterSnapped ? QStringLiteral("已吸附到几何") : QStringLiteral("视图/模型中心"))
            .arg(m_rotationCenter[0], 0, 'g', 8)
            .arg(m_rotationCenter[1], 0, 'g', 8)
            .arg(m_rotationCenter[2], 0, 'g', 8));
    m_rotationCenterIndicator->show();
    m_rotationCenterIndicator->raise();
}

bool ModelViewport::addNodeToSelection(vtkIdType pointId, bool additive)
{
    if (pointId < 0 || pointId >= static_cast<vtkIdType>(m_pointNodeIds.size()))
        return false;
    if (!additive)
    {
        m_selectedNodePointIds.clear();
        m_selectedElementCellIds.clear();
    }
    const auto existing = m_selectedNodePointIds.find(pointId);
    const bool selected = !additive || existing == m_selectedNodePointIds.cend();
    if (selected)
        m_selectedNodePointIds.insert(pointId);
    else
        m_selectedNodePointIds.erase(existing);
    refreshSelectionHighlight();
    return selected;
}

bool ModelViewport::addElementToSelection(vtkIdType cellId, bool additive)
{
    if (cellId < 0 || cellId >= static_cast<vtkIdType>(m_cellElementIds.size()))
        return false;
    if (!additive)
    {
        m_selectedNodePointIds.clear();
        m_selectedElementCellIds.clear();
    }
    const auto existing = m_selectedElementCellIds.find(cellId);
    const bool selected = !additive || existing == m_selectedElementCellIds.cend();
    if (selected)
        m_selectedElementCellIds.insert(cellId);
    else
        m_selectedElementCellIds.erase(existing);
    refreshSelectionHighlight();
    return selected;
}

void ModelViewport::emitSelectionSummary()
{
    if (m_selectedNodePointIds.empty() && m_selectedElementCellIds.empty())
        emit selectionCleared();
    else
        emit selectionChanged(static_cast<int>(m_selectedNodePointIds.size()),
                              static_cast<int>(m_selectedElementCellIds.size()));
}

void ModelViewport::performSelectionAt(double widgetX, double widgetY, bool additive)
{
    const double scale = devicePixelRatioF();
    const double x = widgetX * scale;
    const double y = (height() - widgetY) * scale;

    const auto selectElement = [this, additive](int elementId)
    {
        const auto found = std::find(m_cellElementIds.cbegin(), m_cellElementIds.cend(), elementId);
        if (found == m_cellElementIds.cend())
            return false;
        const vtkIdType cellId = static_cast<vtkIdType>(std::distance(m_cellElementIds.cbegin(), found));
        if (addElementToSelection(cellId, additive))
            emit elementSelected(elementId);
        emitSelectionSummary();
        return true;
    };

    if (m_selectionTarget != SelectionTarget::ElementsOnly)
    {
        m_hardwarePicker->InitializePickList();
        m_hardwarePicker->PickFromListOn();
        m_hardwarePicker->SnapToMeshPointOn();
        m_hardwarePicker->SetPixelTolerance(6);
        if (m_nodeActor->GetVisibility())
            m_hardwarePicker->AddPickList(m_nodeActor);
        if (m_nodeActor->GetVisibility() && m_hardwarePicker->Pick(x, y, 0.0, m_renderer) &&
            m_hardwarePicker->GetActor() == m_nodeActor.GetPointer() && m_hardwarePicker->GetPointId() >= 0)
        {
            const vtkIdType pointId = m_hardwarePicker->GetPointId();
            if (pointId < static_cast<vtkIdType>(m_pointNodeIds.size()))
            {
                m_hardwarePicker->InitializePickList();
                if (addNodeToSelection(pointId, additive))
                    emit nodeSelected(m_pointNodeIds[static_cast<std::size_t>(pointId)]);
                emitSelectionSummary();
                return;
            }
        }
    }

    if (m_selectionTarget == SelectionTarget::NodesOnly)
    {
        m_hardwarePicker->InitializePickList();
        if (!additive)
        {
            clearSelectionHighlight();
            emitSelectionSummary();
        }
        return;
    }

    // 实体表面比中心线宽，优先拾取表面并将多边形映射回所属模型单元，
    // 随后恢复 actor 原有的不可拾取状态。
    if (m_solidActor->GetVisibility() && !m_solidCellElementIds.empty())
    {
        const int originalPickable = m_solidActor->GetPickable();
        m_solidActor->PickableOn();
        m_hardwarePicker->InitializePickList();
        m_hardwarePicker->SnapToMeshPointOff();
        m_hardwarePicker->AddPickList(m_solidActor);
        const bool picked = m_hardwarePicker->Pick(x, y, 0.0, m_renderer) &&
                            m_hardwarePicker->GetActor() == m_solidActor.GetPointer() &&
                            m_hardwarePicker->GetCellId() >= 0;
        const vtkIdType solidCellId = m_hardwarePicker->GetCellId();
        m_solidActor->SetPickable(originalPickable);
        if (picked && solidCellId < static_cast<vtkIdType>(m_solidCellElementIds.size()) &&
            selectElement(m_solidCellElementIds[static_cast<std::size_t>(solidCellId)]))
        {
            m_hardwarePicker->InitializePickList();
            return;
        }
    }

    m_hardwarePicker->InitializePickList();
    m_hardwarePicker->SnapToMeshPointOff();
    if (m_elementActor->GetVisibility())
        m_hardwarePicker->AddPickList(m_elementActor);
    if (m_elementActor->GetVisibility() && m_hardwarePicker->Pick(x, y, 0.0, m_renderer) &&
        m_hardwarePicker->GetActor() == m_elementActor.GetPointer() && m_hardwarePicker->GetCellId() >= 0)
    {
        const vtkIdType cellId = m_hardwarePicker->GetCellId();
        if (cellId < static_cast<vtkIdType>(m_cellElementIds.size()))
        {
            m_hardwarePicker->InitializePickList();
            if (addElementToSelection(cellId, additive))
                emit elementSelected(m_cellElementIds[static_cast<std::size_t>(cellId)]);
            emitSelectionSummary();
            return;
        }
    }

    // 硬件拾取用于精确选中细线；失败时使用带少量屏幕相对容差的单元拾取器兜底，便于点选。
    m_cellPicker->InitializePickList();
    m_cellPicker->PickFromListOn();
    m_cellPicker->SetTolerance(0.006);
    if (m_elementActor->GetVisibility())
        m_cellPicker->AddPickList(m_elementActor);
    if (m_elementActor->GetVisibility() && m_cellPicker->Pick(x, y, 0.0, m_renderer) &&
        m_cellPicker->GetActor() == m_elementActor.GetPointer() && m_cellPicker->GetCellId() >= 0)
    {
        const vtkIdType cellId = m_cellPicker->GetCellId();
        if (cellId < static_cast<vtkIdType>(m_cellElementIds.size()))
        {
            m_cellPicker->InitializePickList();
            if (addElementToSelection(cellId, additive))
                emit elementSelected(m_cellElementIds[static_cast<std::size_t>(cellId)]);
            emitSelectionSummary();
            return;
        }
    }

    m_cellPicker->InitializePickList();
    m_hardwarePicker->InitializePickList();
    if (!additive)
    {
        clearSelectionHighlight();
        emitSelectionSummary();
    }
}

bool ModelViewport::collectVisibleCellsInRect(const QRectF& widgetRect, vtkActor* targetActor,
                                              const std::vector<vtkActor*>& occluderActors,
                                              std::vector<vtkIdType>& cellIds) const
{
    cellIds.clear();
    if (!m_renderer || !m_renderWindow || !targetActor || !targetActor->GetVisibility())
        return false;

    const int* renderSize = m_renderer->GetSize();
    if (!renderSize || renderSize[0] <= 0 || renderSize[1] <= 0)
        return false;

    const QRectF normalizedRect = widgetRect.normalized();
    const double scale = devicePixelRatioF();
    const int minimumX = qBound(0, qFloor(normalizedRect.left() * scale), renderSize[0] - 1);
    const int maximumX = qBound(0, qCeil(normalizedRect.right() * scale), renderSize[0] - 1);
    const int minimumY = qBound(0, qFloor((height() - normalizedRect.bottom()) * scale), renderSize[1] - 1);
    const int maximumY = qBound(0, qCeil((height() - normalizedRect.top()) * scale), renderSize[1] - 1);
    if (minimumX > maximumX || minimumY > maximumY)
        return false;

    struct PickableState
    {
        vtkActor* actor = nullptr;
        int pickable = 0;
    };
    std::vector<PickableState> previousPickableStates;
    const auto makePickable = [&previousPickableStates](vtkActor* actor)
    {
        if (!actor || !actor->GetVisibility() ||
            std::any_of(previousPickableStates.cbegin(), previousPickableStates.cend(),
                        [actor](const PickableState& state) { return state.actor == actor; }))
        {
            return;
        }
        previousPickableStates.push_back({actor, actor->GetPickable()});
        actor->PickableOn();
    };
    makePickable(targetActor);
    for (vtkActor* occluder : occluderActors)
        makePickable(occluder);

    vtkNew<vtkHardwareSelector> selector;
    selector->SetRenderer(m_renderer);
    selector->SetArea(minimumX, minimumY, maximumX, maximumY);
    selector->SetFieldAssociation(vtkDataObject::FIELD_ASSOCIATION_CELLS);
    vtkSmartPointer<vtkSelection> selection = vtkSmartPointer<vtkSelection>::Take(selector->Select());
    selector->SetRenderer(nullptr);

    for (const PickableState& state : previousPickableStates)
        state.actor->SetPickable(state.pickable);
    if (!selection)
        return false;

    for (unsigned int index = 0; index < selection->GetNumberOfNodes(); ++index)
    {
        vtkSelectionNode* selectionNode = selection->GetNode(index);
        if (!selectionNode || !selectionNode->GetProperties())
            continue;
        vtkProp* selectedProp = vtkProp::SafeDownCast(selectionNode->GetProperties()->Get(vtkSelectionNode::PROP()));
        if (selectedProp != targetActor)
            continue;
        vtkIdTypeArray* selectedIds = vtkIdTypeArray::SafeDownCast(selectionNode->GetSelectionList());
        if (!selectedIds)
            continue;
        for (vtkIdType valueIndex = 0; valueIndex < selectedIds->GetNumberOfValues(); ++valueIndex)
        {
            const vtkIdType cellId = selectedIds->GetValue(valueIndex);
            if (cellId >= 0)
                cellIds.push_back(cellId);
        }
    }
    std::sort(cellIds.begin(), cellIds.end());
    cellIds.erase(std::unique(cellIds.begin(), cellIds.end()), cellIds.end());
    return true;
}

void ModelViewport::performSelectionInRect(const QRectF& widgetRect, bool additive)
{
    if (!m_renderer || widgetRect.width() <= 0.0 || widgetRect.height() <= 0.0)
        return;

    if (!additive)
    {
        m_selectedNodePointIds.clear();
        m_selectedElementCellIds.clear();
    }

    bool nodeHardwareSelectionCompleted = false;
    if (m_selectionTarget != SelectionTarget::ElementsOnly && m_nodeActor->GetVisibility())
    {
        std::vector<vtkActor*> occluders;
        if (m_elementActor->GetVisibility())
            occluders.push_back(m_elementActor);
        if (m_solidActor->GetVisibility())
            occluders.push_back(m_solidActor);
        std::vector<vtkIdType> nodeCellIds;
        nodeHardwareSelectionCompleted = collectVisibleCellsInRect(widgetRect, m_nodeActor, occluders, nodeCellIds);
        for (const vtkIdType cellId : nodeCellIds)
        {
            // 每个模型节点对应一个顶点单元，因此硬件拾取的 CellId 就是对应点索引。
            if (cellId >= 0 && cellId < static_cast<vtkIdType>(m_pointNodeIds.size()))
                m_selectedNodePointIds.insert(cellId);
        }
    }

    bool elementHardwareSelectionCompleted = false;
    if (m_selectionTarget != SelectionTarget::NodesOnly &&
        (m_elementActor->GetVisibility() || m_solidActor->GetVisibility()))
    {
        vtkActor* targetActor = m_solidActor->GetVisibility() ? m_solidActor.GetPointer() : m_elementActor.GetPointer();
        std::vector<vtkActor*> occluders;
        if (m_nodeActor->GetVisibility())
            occluders.push_back(m_nodeActor);
        if (targetActor != m_elementActor.GetPointer() && m_elementActor->GetVisibility())
            occluders.push_back(m_elementActor);
        if (targetActor != m_solidActor.GetPointer() && m_solidActor->GetVisibility())
            occluders.push_back(m_solidActor);

        std::vector<vtkIdType> selectedCells;
        elementHardwareSelectionCompleted = collectVisibleCellsInRect(widgetRect, targetActor, occluders, selectedCells);
        for (const vtkIdType selectedCellId : selectedCells)
        {
            const int elementId = targetActor == m_solidActor.GetPointer()
                                      ? (selectedCellId < static_cast<vtkIdType>(m_solidCellElementIds.size())
                                             ? m_solidCellElementIds[static_cast<std::size_t>(selectedCellId)]
                                             : -1)
                                      : (selectedCellId < static_cast<vtkIdType>(m_cellElementIds.size())
                                             ? m_cellElementIds[static_cast<std::size_t>(selectedCellId)]
                                             : -1);
            const auto lineCell = std::find(m_cellElementIds.cbegin(), m_cellElementIds.cend(), elementId);
            if (lineCell != m_cellElementIds.cend())
                m_selectedElementCellIds.insert(static_cast<vtkIdType>(std::distance(m_cellElementIds.cbegin(), lineCell)));
        }
    }

    const double scale = devicePixelRatioF();
    const auto projectToWidget = [this, scale](const double world[3], QPointF& point)
    {
        m_renderer->SetWorldPoint(world[0], world[1], world[2], 1.0);
        m_renderer->WorldToDisplay();
        const double* display = m_renderer->GetDisplayPoint();
        if (!display || !std::isfinite(display[0]) || !std::isfinite(display[1]) || !std::isfinite(display[2]) ||
            display[2] < 0.0 || display[2] > 1.0)
        {
            return false;
        }
        point = QPointF(display[0] / scale, height() - display[1] / scale);
        return true;
    };

    if (!nodeHardwareSelectionCompleted && m_selectionTarget != SelectionTarget::ElementsOnly && m_nodeActor->GetVisibility())
    {
        const vtkIdType nodeCount =
            std::min<vtkIdType>(m_points->GetNumberOfPoints(), static_cast<vtkIdType>(m_pointNodeIds.size()));
        for (vtkIdType pointId = 0; pointId < nodeCount; ++pointId)
        {
            double world[3] = {};
            m_points->GetPoint(pointId, world);
            QPointF displayPoint;
            if (projectToWidget(world, displayPoint) && widgetRect.contains(displayPoint))
                m_selectedNodePointIds.insert(pointId);
        }
    }

    if (!elementHardwareSelectionCompleted && m_selectionTarget != SelectionTarget::NodesOnly &&
        (m_elementActor->GetVisibility() || m_solidActor->GetVisibility()))
    {
        const vtkIdType cellCount =
            std::min<vtkIdType>(m_elementData->GetNumberOfCells(), static_cast<vtkIdType>(m_cellElementIds.size()));
        for (vtkIdType cellId = 0; cellId < cellCount; ++cellId)
        {
            vtkNew<vtkIdList> pointIds;
            m_elementData->GetCellPoints(cellId, pointIds);
            bool intersectsSelection = false;
            QPointF previousPoint;
            bool previousValid = false;
            for (vtkIdType index = 0; index < pointIds->GetNumberOfIds(); ++index)
            {
                double world[3] = {};
                m_points->GetPoint(pointIds->GetId(index), world);
                QPointF displayPoint;
                if (!projectToWidget(world, displayPoint))
                {
                    previousValid = false;
                    continue;
                }
                if (widgetRect.contains(displayPoint) ||
                    (previousValid && QLineF(previousPoint, displayPoint).intersects(QLineF(widgetRect.topLeft(), widgetRect.topRight())) ==
                                          QLineF::BoundedIntersection) ||
                    (previousValid && QLineF(previousPoint, displayPoint).intersects(QLineF(widgetRect.topRight(), widgetRect.bottomRight())) ==
                                          QLineF::BoundedIntersection) ||
                    (previousValid && QLineF(previousPoint, displayPoint).intersects(QLineF(widgetRect.bottomRight(), widgetRect.bottomLeft())) ==
                                          QLineF::BoundedIntersection) ||
                    (previousValid && QLineF(previousPoint, displayPoint).intersects(QLineF(widgetRect.bottomLeft(), widgetRect.topLeft())) ==
                                          QLineF::BoundedIntersection))
                {
                    intersectsSelection = true;
                    break;
                }
                previousPoint = displayPoint;
                previousValid = true;
            }
            if (intersectsSelection)
                m_selectedElementCellIds.insert(cellId);
        }
    }

    refreshSelectionHighlight();
    emitSelectionSummary();
}

bool ModelViewport::setRotationCenterAt(double widgetX, double widgetY, bool allowFocalPlane)
{
    if (!hasModel())
        return false;

    const double scale = devicePixelRatioF();
    const double x = widgetX * scale;
    const double y = (height() - widgetY) * scale;
    double center[3] = {};

    m_hardwarePicker->InitializePickList();
    m_hardwarePicker->PickFromListOn();
    m_hardwarePicker->SnapToMeshPointOn();
    m_hardwarePicker->SetPixelTolerance(6);
    if (m_nodeActor->GetVisibility())
        m_hardwarePicker->AddPickList(m_nodeActor);
    const bool nodePicked = m_nodeActor->GetVisibility() && m_hardwarePicker->Pick(x, y, 0.0, m_renderer) &&
                            m_hardwarePicker->GetActor() == m_nodeActor.GetPointer() &&
                            m_hardwarePicker->GetPointId() >= 0;
    if (nodePicked)
    {
        double* picked = m_hardwarePicker->GetPickPosition();
        if (picked && std::isfinite(picked[0]) && std::isfinite(picked[1]) && std::isfinite(picked[2]))
        {
            center[0] = picked[0];
            center[1] = picked[1];
            center[2] = picked[2];
        }
        else
        {
            m_hardwarePicker->InitializePickList();
            return false;
        }
    }
    m_hardwarePicker->InitializePickList();

    bool geometryPicked = false;
    if (!nodePicked)
    {
        const QList<vtkActor*> actors = {m_nodeActor.GetPointer(), m_elementActor.GetPointer(),
                                         m_solidActor.GetPointer()};
        std::array<int, 3> oldPickable = {};
        m_cellPicker->InitializePickList();
        m_cellPicker->PickFromListOn();
        m_cellPicker->SetTolerance(0.008);
        for (int index = 0; index < actors.size(); ++index)
        {
            vtkActor* actor = actors.at(index);
            oldPickable[static_cast<std::size_t>(index)] = actor->GetPickable();
            if (actor->GetVisibility())
            {
                actor->PickableOn();
                m_cellPicker->AddPickList(actor);
            }
        }
        geometryPicked = m_cellPicker->Pick(x, y, 0.0, m_renderer) != 0;
        if (geometryPicked)
        {
            double* picked = m_cellPicker->GetPickPosition();
            if (picked && std::isfinite(picked[0]) && std::isfinite(picked[1]) && std::isfinite(picked[2]))
            {
                center[0] = picked[0];
                center[1] = picked[1];
                center[2] = picked[2];
            }
            else
                geometryPicked = false;
        }
        for (int index = 0; index < actors.size(); ++index)
            actors.at(index)->SetPickable(oldPickable[static_cast<std::size_t>(index)]);
        m_cellPicker->InitializePickList();
        if (!geometryPicked && (!allowFocalPlane || !displayToFocalPlaneWorld(x, y, center)))
            return false;
    }

    m_rotationCenter = {center[0], center[1], center[2]};
    m_rotationCenterValid = true;
    m_rotationCenterSnapped = nodePicked || geometryPicked;
    updateRotationCenterIndicatorAppearance();
    updateRotationCenterIndicator();
    requestRender();
    return true;
}

void ModelViewport::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);
    QAction* selectAction = menu.addAction(QStringLiteral("选择"));
    QAction* rotateAction = menu.addAction(QStringLiteral("旋转"));
    QAction* panAction = menu.addAction(QStringLiteral("平移"));
    QAction* zoomAction = menu.addAction(QStringLiteral("缩放"));
    QAction* rectangleZoomAction = menu.addAction(QStringLiteral("框选放大"));
    const QList<QAction*> modeActions = {selectAction, rotateAction, panAction, zoomAction, rectangleZoomAction};
    QAction* activeModeAction = modeActions.at(static_cast<int>(m_interactionMode));
    QPixmap activeModePixmap(16, 16);
    activeModePixmap.fill(Qt::transparent);
    {
        QPainter painter(&activeModePixmap);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(QStringLiteral("#32C6D8")));
        painter.drawRoundedRect(QRectF(1.0, 1.0, 14.0, 14.0), 3.0, 3.0);
        painter.setPen(QPen(Qt::white, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(QPointF(4.0, 8.0), QPointF(6.7, 10.5));
        painter.drawLine(QPointF(6.7, 10.5), QPointF(12.0, 4.8));
    }
    activeModeAction->setIcon(QIcon(activeModePixmap));

    QMenu* selectionTargetMenu = menu.addMenu(QStringLiteral("选择过滤"));
    QActionGroup* selectionTargetGroup = new QActionGroup(selectionTargetMenu);
    selectionTargetGroup->setExclusive(true);
    const auto addSelectionTargetAction = [selectionTargetMenu, selectionTargetGroup, this](const QString& text,
                                                                                              SelectionTarget target,
                                                                                              const QString& shortcut)
    {
        QAction* action = selectionTargetMenu->addAction(text + QStringLiteral("\t") + shortcut);
        action->setCheckable(true);
        action->setChecked(m_selectionTarget == target);
        action->setData(static_cast<int>(target));
        selectionTargetGroup->addAction(action);
        return action;
    };
    addSelectionTargetAction(QStringLiteral("自动：节点优先"), SelectionTarget::Auto, QStringLiteral("A"));
    addSelectionTargetAction(QStringLiteral("仅选择节点"), SelectionTarget::NodesOnly, QStringLiteral("N"));
    addSelectionTargetAction(QStringLiteral("仅选择单元"), SelectionTarget::ElementsOnly, QStringLiteral("E"));

    menu.addSeparator();
    QAction* isolateSelectionAction = menu.addAction(m_selectionIsolated ? QStringLiteral("显示全部对象\tI")
                                                                        : QStringLiteral("仅显示选中对象\tI"));
    const bool hasSelection = !m_selectedNodePointIds.empty() || !m_selectedElementCellIds.empty();
    isolateSelectionAction->setEnabled(m_selectionIsolated || hasSelection);
    QMenu* centerMenu = menu.addMenu(QStringLiteral("旋转/缩放中心"));
    QAction* pickCenterAction = centerMenu->addAction(QStringLiteral("拾取当前模型点"));
    QAction* viewportCenterAction = centerMenu->addAction(QStringLiteral("对准视口中心（自动吸附）"));
    QAction* modelCenterAction = centerMenu->addAction(QStringLiteral("恢复到模型中心"));
    pickCenterAction->setEnabled(hasModel());
    viewportCenterAction->setEnabled(hasModel());
    modelCenterAction->setEnabled(hasModel());

    QAction* selected = menu.exec(event->globalPos());
    if (!selected)
        return;
    if (selected == selectAction)
        setInteractionMode(InteractionMode::Select);
    else if (selected == rotateAction)
        setInteractionMode(InteractionMode::Rotate);
    else if (selected == panAction)
        setInteractionMode(InteractionMode::Pan);
    else if (selected == zoomAction)
        setInteractionMode(InteractionMode::Zoom);
    else if (selected == rectangleZoomAction)
        setInteractionMode(InteractionMode::ZoomToRect);
    else if (selectionTargetGroup->actions().contains(selected))
    {
        setSelectionTarget(static_cast<SelectionTarget>(selected->data().toInt()));
        setInteractionMode(InteractionMode::Select);
    }
    else if (selected == isolateSelectionAction)
        setSelectionIsolation(!m_selectionIsolated);
    else if (selected == pickCenterAction)
        setRotationCenterAt(event->pos().x(), event->pos().y());
    else if (selected == viewportCenterAction)
        updateRotationCenterToViewportCenter(true);
    else if (selected == modelCenterAction)
    {
        double bounds[6] = {};
        m_renderer->ComputeVisiblePropBounds(bounds);
        m_rotationCenter = {(bounds[0] + bounds[1]) * 0.5, (bounds[2] + bounds[3]) * 0.5,
                            (bounds[4] + bounds[5]) * 0.5};
        m_rotationCenterValid = true;
        m_rotationCenterSnapped = false;
        updateRotationCenterIndicatorAppearance();
        updateRotationCenterIndicator();
        requestRender();
    }
    event->accept();
}

void ModelViewport::updateThemeColors()
{
    const auto colors = colorsForTheme(m_themeIndex);
    m_renderer->SetBackground(colors.background);
    m_renderer->SetBackground2(colors.backgroundSecond);
    if (colors.gradientBackground)
        m_renderer->GradientBackgroundOn();
    else
        m_renderer->GradientBackgroundOff();
    if (m_elementActor)
        m_elementActor->GetProperty()->SetColor(colors.element[0], colors.element[1], colors.element[2]);
    if (m_nodeActor)
        m_nodeActor->GetProperty()->SetColor(colors.node[0], colors.node[1], colors.node[2]);
    if (m_solidActor)
    {
        m_solidActor->GetProperty()->SetColor(colors.solid[0], colors.solid[1], colors.solid[2]);
        // 环境光保证各主题下背光的实体表面仍清晰可见，避免表面呈现黑色。
        m_solidActor->GetProperty()->SetAmbient(0.48);
        m_solidActor->GetProperty()->SetDiffuse(0.65);
        m_solidActor->GetProperty()->SetSpecular(0.20);
        m_solidActor->GetProperty()->SetSpecularPower(18.0);
        m_solidActor->GetProperty()->EdgeVisibilityOn();
        m_solidActor->GetProperty()->SetEdgeColor(colors.element[0], colors.element[1], colors.element[2]);
    }
    if (m_nodeLabelMapper)
        m_nodeLabelMapper->GetLabelTextProperty()->SetColor(colors.node[0], colors.node[1], colors.node[2]);
    if (m_elementLabelMapper)
        m_elementLabelMapper->GetLabelTextProperty()->SetColor(colors.element[0], colors.element[1], colors.element[2]);
    if (m_resultScalarBar)
    {
        const double scalarText = m_themeIndex == 3 ? 0.12 : 0.94;
        m_resultScalarBar->GetTitleTextProperty()->SetColor(scalarText, scalarText, scalarText);
        m_resultScalarBar->GetLabelTextProperty()->SetColor(scalarText, scalarText, scalarText);
    }
    if (m_emptyStateIconLabel)
    {
        QPixmap icon(96, 96);
        icon.fill(Qt::transparent);
        QPainter painter(&icon);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.scale(3.0, 3.0);
        const QColor line = QColor::fromRgbF(colors.element[0], colors.element[1], colors.element[2]);
        const QColor accent = QColor::fromRgbF(colors.node[0], colors.node[1], colors.node[2]);
        painter.setPen(QPen(line, 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawPolygon(QPolygonF{QPointF(16, 3), QPointF(27, 9), QPointF(16, 15), QPointF(5, 9)});
        painter.drawPolyline(
            QPolygonF{QPointF(5, 9), QPointF(5, 22), QPointF(16, 29), QPointF(27, 22), QPointF(27, 9)});
        painter.setPen(QPen(accent, 2.4, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(QPointF(16, 15), QPointF(16, 29));
        painter.drawLine(QPointF(5, 9), QPointF(16, 15));
        painter.drawLine(QPointF(27, 9), QPointF(16, 15));
        m_emptyStateIconLabel->setPixmap(icon);
    }
    updateRotationCenterIndicatorAppearance();
}

void ModelViewport::updateEmptyStateGeometry()
{
    const QRect content = rect().adjusted(24, 24, -24, -24);
    const int centerY = content.center().y();
    if (m_emptyStateIconLabel)
        m_emptyStateIconLabel->setGeometry(content.center().x() - 28, centerY - 82, 56, 56);
    if (m_emptyStateLabel)
        m_emptyStateLabel->setGeometry(content.left(), centerY - 18, content.width(), 92);
}

void ModelViewport::requestRender()
{
    if (m_renderPending)
        return;
    m_renderPending = true;
    QTimer::singleShot(0, this,
                       [this]()
                       {
                           m_renderPending = false;
                           if (m_renderWindow)
                               m_renderWindow->Render();
                       });
}

void ModelViewport::clearSelectionHighlight()
{
    if (m_selectionIsolated)
    {
        m_selectionIsolated = false;
        applyBaseActorVisibility();
        m_originalElementActor->SetVisibility(m_originalElementVisibleBeforeIsolation && m_resultDisplayActive);
    }
    m_selectedNodePointIds.clear();
    m_selectedElementCellIds.clear();
    m_selectionData->Initialize();
    m_selectionData->SetPoints(m_points);
    m_selectionActor->SetVisibility(false);
    m_solidSelectionData->Initialize();
    m_solidSelectionActor->SetVisibility(false);
    requestRender();
}

bool ModelViewport::setSelectionIsolation(bool enabled)
{
    if (enabled && (!hasModel() || (m_selectedNodePointIds.empty() && m_selectedElementCellIds.empty())))
        return false;
    if (m_selectionIsolated == enabled)
        return false;

    if (enabled)
    {
        m_originalElementVisibleBeforeIsolation = m_originalElementActor->GetVisibility() != 0;
        m_selectionIsolated = true;
        applyBaseActorVisibility();
        refreshSelectionHighlight();
    }
    else
    {
        m_selectionIsolated = false;
        applyBaseActorVisibility();
        m_originalElementActor->SetVisibility(m_originalElementVisibleBeforeIsolation && m_resultDisplayActive);
        refreshSelectionHighlight();
    }
    updateAdaptiveLabels();
    requestRender();
    return true;
}

void ModelViewport::refreshSelectionHighlight()
{
    if (m_selectedNodePointIds.empty() && m_selectedElementCellIds.empty())
    {
        clearSelectionHighlight();
        return;
    }

    vtkNew<vtkCellArray> vertices;
    for (const vtkIdType pointId : m_selectedNodePointIds)
    {
        if (pointId >= 0 && pointId < m_points->GetNumberOfPoints())
            vertices->InsertNextCell(1, &pointId);
    }

    vtkNew<vtkCellArray> lines;
    for (const vtkIdType cellId : m_selectedElementCellIds)
    {
        if (cellId < 0 || cellId >= m_elementData->GetNumberOfCells())
            continue;
        vtkNew<vtkIdList> pointIds;
        m_elementData->GetCellPoints(cellId, pointIds);
        if (pointIds->GetNumberOfIds() >= 2)
            lines->InsertNextCell(pointIds);
    }

    m_selectionData->Initialize();
    m_selectionData->SetPoints(m_points);
    if (vertices->GetNumberOfCells() > 0)
        m_selectionData->SetVerts(vertices);
    if (lines->GetNumberOfCells() > 0)
        m_selectionData->SetLines(lines);
    m_selectionData->Modified();
    m_selectionActor->SetVisibility(vertices->GetNumberOfCells() > 0 || lines->GetNumberOfCells() > 0);

    vtkNew<vtkCellArray> solidPolys;
    if ((m_solidActor->GetVisibility() || m_selectionIsolated) && m_solidData->GetNumberOfCells() > 0)
    {
        for (const vtkIdType lineCellId : m_selectedElementCellIds)
        {
            if (lineCellId < 0 || lineCellId >= static_cast<vtkIdType>(m_cellElementIds.size()))
                continue;
            const int elementId = m_cellElementIds[static_cast<std::size_t>(lineCellId)];
            for (vtkIdType solidCellId = 0;
                 solidCellId < m_solidData->GetNumberOfCells() &&
                 solidCellId < static_cast<vtkIdType>(m_solidCellElementIds.size()); ++solidCellId)
            {
                if (m_solidCellElementIds[static_cast<std::size_t>(solidCellId)] != elementId)
                    continue;
                vtkNew<vtkIdList> solidPointIds;
                m_solidData->GetCellPoints(solidCellId, solidPointIds);
                if (solidPointIds->GetNumberOfIds() >= 3)
                    solidPolys->InsertNextCell(solidPointIds);
            }
        }
    }
    m_solidSelectionData->Initialize();
    if (solidPolys->GetNumberOfCells() > 0)
    {
        m_solidSelectionData->SetPoints(m_solidData->GetPoints());
        m_solidSelectionData->SetPolys(solidPolys);
    }
    m_solidSelectionData->Modified();
    m_solidSelectionActor->SetVisibility(solidPolys->GetNumberOfCells() > 0 &&
                                         (m_solidActor->GetVisibility() || m_selectionIsolated));
    requestRender();
}
