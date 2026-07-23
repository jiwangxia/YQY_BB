#include "Widgets/ModelViewport.h"

#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Section/SectionCircular.h"
#include "DataStructure/Section/SectionRectangle.h"
#include "DataStructure/Section/Section_Ice.h"
#include "Export/Hdf5ModelIO.h"

#include <QLabel>
#include <QContextMenuEvent>
#include <QActionGroup>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QResizeEvent>
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
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkObjectFactory.h>
#include <vtkTextProperty.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>

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
            vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
            break;
        case ModelViewport::InteractionMode::Zoom:
            vtkInteractorStyleTrackballCamera::OnRightButtonDown();
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
            vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
            if (m_panFinished)
                m_panFinished();
            break;
        case ModelViewport::InteractionMode::Zoom:
            vtkInteractorStyleTrackballCamera::OnRightButtonUp();
            break;
        case ModelViewport::InteractionMode::Select:
            break;
        }
    }

    void OnMouseMove() override
    {
        if (m_mode == ModelViewport::InteractionMode::Rotate && m_rotating
            && this->Interactor && this->CurrentRenderer)
        {
            const int* position = this->Interactor->GetEventPosition();
            const int dx = position[0] - m_lastPosition[0];
            const int dy = position[1] - m_lastPosition[1];
            m_lastPosition[0] = position[0];
            m_lastPosition[1] = position[1];
            if (dx == 0 && dy == 0)
                return;

            double center[3] = { 0.0, 0.0, 0.0 };
            vtkCamera* camera = this->CurrentRenderer->GetActiveCamera();
            if (!camera || !m_rotationCenterProvider || !m_rotationCenterProvider(center))
                return;

            double cameraPosition[3] = {};
            double focalPoint[3] = {};
            double viewUp[3] = {};
            camera->GetPosition(cameraPosition);
            camera->GetFocalPoint(focalPoint);
            camera->GetViewUp(viewUp);
            double positionOffset[3] = { cameraPosition[0] - center[0],
                cameraPosition[1] - center[1], cameraPosition[2] - center[2] };
            double focalOffset[3] = { focalPoint[0] - center[0],
                focalPoint[1] - center[1], focalPoint[2] - center[2] };

            const double azimuth = -static_cast<double>(dx) * 0.25;
            rotateVector(positionOffset, viewUp, azimuth, positionOffset);
            rotateVector(focalOffset, viewUp, azimuth, focalOffset);

            double projection[3] = { focalOffset[0] - positionOffset[0],
                focalOffset[1] - positionOffset[1], focalOffset[2] - positionOffset[2] };
            double right[3] = { projection[1] * viewUp[2] - projection[2] * viewUp[1],
                projection[2] * viewUp[0] - projection[0] * viewUp[2],
                projection[0] * viewUp[1] - projection[1] * viewUp[0] };
            if (normalizeVector(right))
            {
                const double elevation = static_cast<double>(dy) * 0.25;
                rotateVector(positionOffset, right, elevation, positionOffset);
                rotateVector(focalOffset, right, elevation, focalOffset);
                rotateVector(viewUp, right, elevation, viewUp);
            }

            camera->SetPosition(center[0] + positionOffset[0], center[1] + positionOffset[1],
                center[2] + positionOffset[2]);
            camera->SetFocalPoint(center[0] + focalOffset[0], center[1] + focalOffset[1],
                center[2] + focalOffset[2]);
            camera->SetViewUp(viewUp);
            camera->OrthogonalizeViewUp();
            this->CurrentRenderer->ResetCameraClippingRange();
            if (m_cameraChanged)
                m_cameraChanged();
            this->Interactor->Render();
            return;
        }

        vtkInteractorStyleTrackballCamera::OnMouseMove();
        if (m_mode != ModelViewport::InteractionMode::Select && m_cameraChanged)
            m_cameraChanged();
    }

    void OnRightButtonDown() override
    {
    }
    void OnRightButtonUp() override
    {
    }

private:
    static bool normalizeVector(double value[3])
    {
        const double length = std::sqrt(value[0] * value[0]
            + value[1] * value[1] + value[2] * value[2]);
        if (!std::isfinite(length) || length <= 1.0e-12)
            return false;
        value[0] /= length; value[1] /= length; value[2] /= length;
        return true;
    }

    static void rotateVector(const double input[3], const double rawAxis[3],
        double degrees, double output[3])
    {
        double axis[3] = { rawAxis[0], rawAxis[1], rawAxis[2] };
        if (!normalizeVector(axis))
            return;
        const double radians = degrees * 3.14159265358979323846 / 180.0;
        const double cosine = std::cos(radians);
        const double sine = std::sin(radians);
        const double axisDot = axis[0] * input[0] + axis[1] * input[1] + axis[2] * input[2];
        const double crossValue[3] = { axis[1] * input[2] - axis[2] * input[1],
            axis[2] * input[0] - axis[0] * input[2],
            axis[0] * input[1] - axis[1] * input[0] };
        const double result[3] = {
            input[0] * cosine + crossValue[0] * sine + axis[0] * axisDot * (1.0 - cosine),
            input[1] * cosine + crossValue[1] * sine + axis[1] * axisDot * (1.0 - cosine),
            input[2] * cosine + crossValue[2] * sine + axis[2] * axisDot * (1.0 - cosine)
        };
        output[0] = result[0]; output[1] = result[1]; output[2] = result[2];
    }

    ModelViewport::InteractionMode m_mode = ModelViewport::InteractionMode::Select;
    bool m_rotating = false;
    int m_lastPosition[2] = { 0, 0 };
    std::function<bool(double*)> m_rotationCenterProvider;
    std::function<void()> m_cameraChanged;
    std::function<void()> m_panFinished;
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
        return { { 0.063, 0.035, 0.106 }, { 0.063, 0.035, 0.106 }, { 0.50, 0.38, 1.00 }, { 0.94, 0.36, 1.00 }, { 0.55, 0.40, 0.95 }, false };
    if (themeIndex == 2)
        return { { 0.051, 0.145, 0.169 }, { 0.051, 0.145, 0.169 }, { 0.27, 0.66, 1.00 }, { 0.33, 0.88, 0.76 }, { 0.20, 0.63, 0.72 }, false };
    if (themeIndex == 3)
    {
        // FEM uses #FFFFFF -> #EEF6FF in its VTK renderer. Dark geometry and
        // blue nodes preserve strong contrast against both ends of the gradient.
        return { { 1.0, 1.0, 1.0 }, { 0.933, 0.965, 1.0 },
            { 0.114, 0.169, 0.227 }, { 0.184, 0.502, 0.929 }, { 0.31, 0.52, 0.72 }, true };
    }
    return { { 0.043, 0.075, 0.106 }, { 0.043, 0.075, 0.106 }, { 0.31, 0.55, 1.00 }, { 0.19, 0.78, 0.85 }, { 0.28, 0.48, 0.68 }, false };
}

using SolidVector3 = std::array<double, 3>;

SolidVector3 subtract(const SolidVector3& first, const SolidVector3& second)
{
    return { first[0] - second[0], first[1] - second[1], first[2] - second[2] };
}

double dot(const SolidVector3& first, const SolidVector3& second)
{
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
}

SolidVector3 cross(const SolidVector3& first, const SolidVector3& second)
{
    return { first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0] };
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
        if (std::isfinite(halfWidth) && std::isfinite(halfHeight)
            && halfWidth > epsilon && halfHeight > epsilon)
        {
            return { { halfWidth, halfHeight }, { -halfWidth, halfHeight },
                { -halfWidth, -halfHeight }, { halfWidth, -halfHeight } };
        }
        return contour;
    }

    const auto ice = std::dynamic_pointer_cast<Section_Ice>(section);
    double radius = section->m_Radius;
    if ((!std::isfinite(radius) || radius <= epsilon)
        && std::isfinite(section->m_Area) && section->m_Area > epsilon)
    {
        radius = std::sqrt(section->m_Area / pi);
    }
    if (!std::isfinite(radius) || radius <= epsilon)
        return contour;

    const int segmentCount = ice ? 32 : 16;
    contour.reserve(segmentCount);
    for (int index = 0; index < segmentCount; ++index)
    {
        const double angle = 2.0 * pi * static_cast<double>(index)
            / static_cast<double>(segmentCount);
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
                outerRadius += ice->m_IceThickness
                    * std::pow(std::max(0.0, std::cos(ratio * pi * 0.5)), ice->m_IceShapeFactor);
            }
        }
        outerRadius *= displayScale;
        contour.push_back({ outerRadius * std::cos(angle), outerRadius * std::sin(angle) });
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
    , m_originalElementData(vtkSmartPointer<vtkPolyData>::New())
    , m_elementMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_nodeMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_selectionMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_solidMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_originalElementMapper(vtkSmartPointer<vtkPolyDataMapper>::New())
    , m_nodeLabelMapper(vtkSmartPointer<vtkLabeledDataMapper>::New())
    , m_elementLabelMapper(vtkSmartPointer<vtkLabeledDataMapper>::New())
    , m_elementActor(vtkSmartPointer<vtkActor>::New())
    , m_nodeActor(vtkSmartPointer<vtkActor>::New())
    , m_selectionActor(vtkSmartPointer<vtkActor>::New())
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
    m_renderWindow->SetMultiSamples(4);
    m_renderWindow->GetInteractor()->SetDesiredUpdateRate(30.0);
    m_renderWindow->GetInteractor()->SetStillUpdateRate(5.0);

    auto interactionStyle = vtkSmartPointer<ModeInteractorStyle>::New();
    interactionStyle->SetDefaultRenderer(m_renderer);
    interactionStyle->SetMotionFactor(8.0);
    interactionStyle->SetRotationCenterProvider([this](double* center) {
        return ensureRotationCenter(center);
    });
    interactionStyle->SetCameraChangedCallback([this]() {
        updateRotationCenterIndicator();
        updateAdaptiveLabels();
    });
    interactionStyle->SetPanFinishedCallback([this]() {
        updateRotationCenterToViewportCenter(true);
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
    m_elementActor->GetProperty()->SetLineWidth(2.0);
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
    m_selectionActor->SetVisibility(false);
    m_selectionActor->PickableOff();

    m_renderer->AddActor(m_elementActor);
    m_renderer->AddActor(m_originalElementActor);
    m_renderer->AddActor(m_solidActor);
    m_renderer->AddActor(m_nodeActor);
    m_renderer->AddActor(m_selectionActor);
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
    m_resultScalarBar->SetTitleRatio(0.16);
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
    m_emptyStateLabel = new QLabel(
        QStringLiteral("尚未加载模型\n\n导入模型后，真实几何与计算结果将在此处显示"), this);
    m_emptyStateLabel->setObjectName(QStringLiteral("viewportEmptyLabel"));
    m_emptyStateLabel->setAlignment(Qt::AlignCenter);
    m_emptyStateLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_emptyStateLabel->setWordWrap(true);
    m_rotationCenterIndicator = new QLabel(this);
    m_rotationCenterIndicator->setObjectName(QStringLiteral("rotationCenterIndicator"));
    m_rotationCenterIndicator->setFixedSize(28, 28);
    m_rotationCenterIndicator->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_rotationCenterIndicator->hide();
    updateThemeColors();
    updateEmptyStateGeometry();
}

void ModelViewport::displayModel(const std::shared_ptr<StructureData>& structure, bool resetCamera)
{
    if (!structure || structure->m_Nodes.empty())
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
    m_resultDisplayActive = false;
    m_originalElementActor->SetVisibility(false);
    m_resultScalarBar->SetVisibility(false);

    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    points->Allocate(static_cast<vtkIdType>(structure->m_Nodes.size()));
    std::unordered_map<const Node*, vtkIdType> pointIds;
    pointIds.reserve(structure->m_Nodes.size());
    m_nodePointIds.clear();
    m_nodePointIds.reserve(structure->m_Nodes.size());
    for (const auto& [nodeId, node] : structure->m_Nodes)
    {
        if (!node)
            continue;
        const vtkIdType pointId = points->InsertNextPoint(node->m_X, node->m_Y, node->m_Z);
        pointIds.emplace(node.get(), pointId);
        m_nodePointIds.emplace(nodeId, pointId);
        m_originalNodeCoordinates.emplace(nodeId,
            std::array<double, 3>{ node->m_X, node->m_Y, node->m_Z });
        m_pointNodeIds.push_back(nodeId);
    }

    vtkNew<vtkCellArray> lines;
    m_elementLabelPositions.reserve(structure->m_Elements.size());
    lines->AllocateEstimate(static_cast<vtkIdType>(structure->m_Elements.size()), 2);
    for (const auto& [elementId, element] : structure->m_Elements)
    {
        if (!element)
            continue;

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
        double center[3] = { 0.0, 0.0, 0.0 };
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
        m_elementLabelPositions.push_back({ center[0], center[1], center[2] });
    }

    vtkNew<vtkCellArray> vertices;
    vertices->AllocateEstimate(points->GetNumberOfPoints(), 1);
    for (vtkIdType pointId = 0; pointId < points->GetNumberOfPoints(); ++pointId)
        vertices->InsertNextCell(1, &pointId);

    m_points->ShallowCopy(points);
    m_elementData->SetPoints(m_points);
    m_elementData->SetLines(lines);
    vtkNew<vtkPoints> originalPoints;
    originalPoints->DeepCopy(points);
    m_originalElementData->SetPoints(originalPoints);
    m_originalElementData->SetLines(lines);
    m_originalElementData->Modified();
    m_nodeData->SetPoints(m_points);
    m_nodeData->SetVerts(vertices);
    m_elementData->Modified();
    m_nodeData->Modified();
    rebuildSolidGeometry();
    m_elementActor->SetVisibility(m_elementsVisible);
    m_nodeActor->SetVisibility(m_nodesVisible);
    m_nodeLabelActor->SetVisibility(m_nodeLabelsVisible);
    m_elementLabelActor->SetVisibility(m_elementLabelsVisible);
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
    const bool changed = m_nodeLabelsVisible != visible;
    m_nodeLabelsVisible = visible;
    if (visible)
        updateAdaptiveLabels();
    m_nodeLabelActor->SetVisibility(visible && hasModel());
    requestRender();
    if (changed)
        emit nodeLabelsVisibilityChanged(visible);
}

void ModelViewport::setElementLabelsVisible(bool visible)
{
    const bool changed = m_elementLabelsVisible != visible;
    m_elementLabelsVisible = visible;
    if (visible)
        updateAdaptiveLabels();
    m_elementLabelActor->SetVisibility(visible && hasModel());
    requestRender();
    if (changed)
        emit elementLabelsVisibilityChanged(visible);
}

void ModelViewport::setNodesVisible(bool visible)
{
    const bool changed = m_nodesVisible != visible;
    m_nodesVisible = visible;
    m_nodeActor->SetVisibility(visible && hasModel());
    requestRender();
    if (changed)
        emit nodesVisibilityChanged(visible);
}

void ModelViewport::setElementsVisible(bool visible)
{
    const bool changed = m_elementsVisible != visible;
    m_elementsVisible = visible;
    m_elementActor->SetVisibility(visible && hasModel());
    requestRender();
    if (changed)
        emit elementsVisibilityChanged(visible);
}

void ModelViewport::setSolidVisible(bool visible)
{
    const bool changed = m_solidVisible != visible;
    m_solidVisible = visible;
    m_solidActor->SetVisibility(visible && hasModel() && hasSolidGeometry());
    updateNodeDisplaySize();
    requestRender();
    if (changed)
        emit solidVisibilityChanged(visible);
}

void ModelViewport::updateNodeDisplaySize()
{
    if (!m_nodeActor)
        return;

    const int displaySize = m_nodeSize;
    m_nodeActor->GetProperty()->SetPointSize(static_cast<float>(displaySize));
    if (m_selectionActor)
        m_selectionActor->GetProperty()->SetPointSize(
            static_cast<float>(qMax(12, displaySize + 4)));
}

void ModelViewport::resetCamera()
{
    if (!hasModel())
        return;
    m_renderer->GetActiveCamera()->ParallelProjectionOn();
    m_renderer->ResetCamera();
    m_renderer->ResetCameraClippingRange();
    double bounds[6] = {};
    m_renderer->ComputeVisiblePropBounds(bounds);
    m_rotationCenter = { (bounds[0] + bounds[1]) * 0.5,
        (bounds[2] + bounds[3]) * 0.5,
        (bounds[4] + bounds[5]) * 0.5 };
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
    m_interactionMode = mode;
    if (auto* style = ModeInteractorStyle::SafeDownCast(m_interactorStyle))
        style->SetInteractionMode(mode);

    switch (mode)
    {
    case InteractionMode::Select: setCursor(Qt::ArrowCursor); break;
    case InteractionMode::Rotate: setCursor(Qt::SizeAllCursor); break;
    case InteractionMode::Pan: setCursor(Qt::OpenHandCursor); break;
    case InteractionMode::Zoom: setCursor(Qt::SizeVerCursor); break;
    }
    if (mode == InteractionMode::Select)
        m_rotationCenterIndicator->hide();
    else
    {
        double center[3] = {};
        ensureRotationCenter(center);
        updateRotationCenterIndicator();
    }
    emit interactionModeChanged(mode);
}

void ModelViewport::setStandardView(StandardView view)
{
    if (!hasModel())
        return;

    double bounds[6] = {};
    m_renderer->ComputeVisiblePropBounds(bounds);
    double center[3] = {
        (bounds[0] + bounds[1]) * 0.5,
        (bounds[2] + bounds[3]) * 0.5,
        (bounds[4] + bounds[5]) * 0.5
    };
    if (m_rotationCenterValid)
    {
        center[0] = m_rotationCenter[0];
        center[1] = m_rotationCenter[1];
        center[2] = m_rotationCenter[2];
    }
    else
    {
        m_rotationCenter = { center[0], center[1], center[2] };
        m_rotationCenterValid = true;
        m_rotationCenterSnapped = false;
    }
    const double dx = bounds[1] - bounds[0];
    const double dy = bounds[3] - bounds[2];
    const double dz = bounds[5] - bounds[4];
    const double distance = qMax(1.0, std::sqrt(dx * dx + dy * dy + dz * dz) * 2.0);

    double direction[3] = { 0.0, 0.0, 1.0 };
    double viewUp[3] = { 0.0, 1.0, 0.0 };
    switch (view)
    {
    case StandardView::Front:
        direction[1] = -1.0; direction[2] = 0.0;
        viewUp[1] = 0.0; viewUp[2] = 1.0;
        break;
    case StandardView::Back:
        direction[1] = 1.0; direction[2] = 0.0;
        viewUp[1] = 0.0; viewUp[2] = 1.0;
        break;
    case StandardView::Left:
        direction[0] = -1.0; direction[2] = 0.0;
        viewUp[1] = 0.0; viewUp[2] = 1.0;
        break;
    case StandardView::Right:
        direction[0] = 1.0; direction[2] = 0.0;
        viewUp[1] = 0.0; viewUp[2] = 1.0;
        break;
    case StandardView::Top:
        break;
    case StandardView::Bottom:
        direction[2] = -1.0;
        break;
    case StandardView::Isometric:
        direction[0] = 1.0; direction[1] = 1.0; direction[2] = 1.0;
        {
            const double invLength = 1.0 / std::sqrt(3.0);
            direction[0] *= invLength; direction[1] *= invLength; direction[2] *= invLength;
        }
        break;
    }

    vtkCamera* camera = m_renderer->GetActiveCamera();
    camera->ParallelProjectionOn();
    camera->SetFocalPoint(center);
    camera->SetPosition(center[0] + direction[0] * distance,
        center[1] + direction[1] * distance,
        center[2] + direction[2] * distance);
    camera->SetViewUp(viewUp);
    camera->OrthogonalizeViewUp();
    m_renderer->ResetCameraClippingRange();
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
    m_points->Modified();
    m_elementData->Modified();
    m_nodeData->Modified();
    updateElementLabelPositions();
    rebuildSolidGeometry();
    m_renderer->ResetCameraClippingRange();
    requestRender();
    return true;
}

bool ModelViewport::displayResultFrame(const Hdf5ResultFrame& frame, ResultField field,
    double deformationScale, bool showOriginal)
{
    if (!hasModel() || frame.nodes.empty())
        return false;

    std::unordered_map<int, const Hdf5NodalResult*> nodalResults;
    nodalResults.reserve(frame.nodes.size());
    double maximumDisplacement = 0.0;
    for (const Hdf5NodalResult& result : frame.nodes)
    {
        nodalResults.emplace(result.id, &result);
        maximumDisplacement = std::max(maximumDisplacement,
            std::sqrt(result.displacement[0] * result.displacement[0]
                + result.displacement[1] * result.displacement[1]
                + result.displacement[2] * result.displacement[2]));
    }

    if (deformationScale <= 0.0)
    {
        double bounds[6] = {};
        m_originalElementData->GetBounds(bounds);
        const double modelSize = std::max({ bounds[1] - bounds[0], bounds[3] - bounds[2],
            bounds[5] - bounds[4], 1.0e-12 });
        deformationScale = maximumDisplacement > 1.0e-30
            ? modelSize * 0.10 / maximumDisplacement : 1.0;
        deformationScale = std::clamp(deformationScale, 1.0e-6, 1.0e6);
    }
    m_activeDeformationScale = deformationScale;

    auto pointScalars = vtkSmartPointer<vtkDoubleArray>::New();
    pointScalars->SetName("Result");
    pointScalars->SetNumberOfComponents(1);
    pointScalars->SetNumberOfTuples(m_points->GetNumberOfPoints());
    pointScalars->FillValue(0.0);
    double scalarMinimum = std::numeric_limits<double>::max();
    double scalarMaximum = std::numeric_limits<double>::lowest();

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
            std::copy(std::begin(result->displacement), std::end(result->displacement), displacement);
        m_points->SetPoint(pointId, original[0] + displacement[0] * deformationScale,
            original[1] + displacement[1] * deformationScale,
            original[2] + displacement[2] * deformationScale);

        double scalar = 0.0;
        switch (field)
        {
        case ResultField::DisplacementMagnitude:
            scalar = std::sqrt(displacement[0] * displacement[0]
                + displacement[1] * displacement[1] + displacement[2] * displacement[2]);
            break;
        case ResultField::DisplacementX: scalar = displacement[0]; break;
        case ResultField::DisplacementY: scalar = displacement[1]; break;
        case ResultField::DisplacementZ: scalar = displacement[2]; break;
        default: break;
        }
        pointScalars->SetValue(pointId, scalar);
        if (field <= ResultField::DisplacementZ && result)
        {
            scalarMinimum = std::min(scalarMinimum, scalar);
            scalarMaximum = std::max(scalarMaximum, scalar);
        }
    }

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
                case ResultField::AxialForce: scalar = it->second->axialForce; break;
                case ResultField::Stress: scalar = it->second->currentStress; break;
                case ResultField::Strain: scalar = it->second->strain; break;
                default: break;
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
    const char* titles[] = { "|U|", "U-X", "U-Y", "U-Z", "Axial force", "Stress", "Strain" };
    m_resultScalarBar->SetTitle(titles[static_cast<int>(field)]);
    m_resultScalarBar->SetVisibility(true);
    m_originalElementActor->SetVisibility(showOriginal);
    m_resultDisplayActive = true;

    m_points->Modified();
    m_elementData->Modified();
    m_nodeData->Modified();
    updateElementLabelPositions();
    rebuildSolidGeometry();
    m_renderer->ResetCameraClippingRange();
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
            m_points->SetPoint(point->second, coordinates.data());
    }
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

void ModelViewport::updateElementLabelPositions()
{
    m_elementLabelPositions.resize(
        static_cast<std::size_t>(m_elementData->GetNumberOfCells()));

    vtkNew<vtkIdList> pointIds;
    for (vtkIdType cellId = 0; cellId < m_elementData->GetNumberOfCells(); ++cellId)
    {
        m_elementData->GetCellPoints(cellId, pointIds);
        if (pointIds->GetNumberOfIds() == 0)
            continue;

        double center[3] = { 0.0, 0.0, 0.0 };
        for (vtkIdType index = 0; index < pointIds->GetNumberOfIds(); ++index)
        {
            double point[3] = {};
            m_points->GetPoint(pointIds->GetId(index), point);
            center[0] += point[0];
            center[1] += point[1];
            center[2] += point[2];
        }
        const double inverseCount = 1.0 / static_cast<double>(pointIds->GetNumberOfIds());
        m_elementLabelPositions[static_cast<std::size_t>(cellId)] = {
            center[0] * inverseCount, center[1] * inverseCount,
            center[2] * inverseCount
        };
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
    const int gridColumnCount = std::max(1,
        static_cast<int>(std::ceil(rendererSize[0] / gridSize)));

    auto projectToGrid = [this, rendererSize, gridSize, gridColumnCount](
        const double world[3], long long& gridKey) {
        m_renderer->SetWorldPoint(world[0], world[1], world[2], 1.0);
        m_renderer->WorldToDisplay();
        const double* display = m_renderer->GetDisplayPoint();
        if (!display || !std::isfinite(display[0]) || !std::isfinite(display[1])
            || !std::isfinite(display[2]) || display[2] < 0.0 || display[2] > 1.0
            || display[0] < 0.0 || display[0] >= rendererSize[0]
            || display[1] < 0.0 || display[1] >= rendererSize[1])
        {
            return false;
        }
        const long long column = static_cast<long long>(display[0] / gridSize);
        const long long row = static_cast<long long>(display[1] / gridSize);
        gridKey = row * gridColumnCount + column;
        return true;
    };

    m_nodeLabelData->Initialize();
    if (m_nodeLabelsVisible)
    {
        vtkNew<vtkPoints> labelPoints;
        labelPoints->SetDataTypeToDouble();
        vtkNew<vtkCellArray> labelVertices;
        vtkNew<vtkIntArray> labelIds;
        labelIds->SetName("NodeIds");
        labelIds->SetNumberOfComponents(1);
        std::unordered_set<long long> occupiedCells;
        occupiedCells.reserve(maximumLabelsPerType);

        const vtkIdType pointCount = std::min<vtkIdType>(m_points->GetNumberOfPoints(),
            static_cast<vtkIdType>(m_pointNodeIds.size()));
        for (vtkIdType pointId = 0; pointId < pointCount
            && static_cast<int>(occupiedCells.size()) < maximumLabelsPerType; ++pointId)
        {
            double world[3] = {};
            m_points->GetPoint(pointId, world);
            long long gridKey = 0;
            if (!projectToGrid(world, gridKey) || !occupiedCells.insert(gridKey).second)
                continue;
            const vtkIdType labelPointId = labelPoints->InsertNextPoint(world);
            labelVertices->InsertNextCell(1, &labelPointId);
            labelIds->InsertNextValue(
                m_pointNodeIds[static_cast<std::size_t>(pointId)]);
        }
        m_nodeLabelData->SetPoints(labelPoints);
        m_nodeLabelData->SetVerts(labelVertices);
        m_nodeLabelData->GetPointData()->SetScalars(labelIds);
        m_nodeLabelData->Modified();
    }

    m_elementLabelData->Initialize();
    if (m_elementLabelsVisible)
    {
        vtkNew<vtkPoints> labelPoints;
        labelPoints->SetDataTypeToDouble();
        vtkNew<vtkCellArray> labelVertices;
        vtkNew<vtkIntArray> labelIds;
        labelIds->SetName("ElementIds");
        labelIds->SetNumberOfComponents(1);
        std::unordered_set<long long> occupiedCells;
        occupiedCells.reserve(maximumLabelsPerType);

        const std::size_t candidateCount = std::min(
            m_elementLabelPositions.size(), m_cellElementIds.size());
        for (std::size_t index = 0; index < candidateCount
            && static_cast<int>(occupiedCells.size()) < maximumLabelsPerType; ++index)
        {
            const auto& position = m_elementLabelPositions[index];
            const double world[3] = { position[0], position[1], position[2] };
            long long gridKey = 0;
            if (!projectToGrid(world, gridKey) || !occupiedCells.insert(gridKey).second)
                continue;
            const vtkIdType labelPointId = labelPoints->InsertNextPoint(world);
            labelVertices->InsertNextCell(1, &labelPointId);
            labelIds->InsertNextValue(m_cellElementIds[index]);
        }
        m_elementLabelData->SetPoints(labelPoints);
        m_elementLabelData->SetVerts(labelVertices);
        m_elementLabelData->GetPointData()->SetScalars(labelIds);
        m_elementLabelData->Modified();
    }
}

void ModelViewport::rebuildSolidGeometry()
{
    m_solidData->Initialize();
    m_solidCellElementIds.clear();
    if (!m_structure)
    {
        m_solidActor->SetVisibility(false);
        return;
    }

    vtkNew<vtkPoints> solidPoints;
    solidPoints->SetDataTypeToDouble();
    vtkNew<vtkCellArray> solidPolys;

    const SolidVector3 globalAxes[3] = {
        { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 }
    };
    for (const auto& [elementId, element] : m_structure->m_Elements)
    {
        if (!element || element->m_pNode.size() < 2)
            continue;
        const auto startNode = element->m_pNode.front().lock();
        const auto endNode = element->m_pNode.back().lock();
        const auto property = element->m_pProperty.lock();
        const auto section = property ? property->m_pSection.lock() : nullptr;
        if (!startNode || !endNode || !section)
            continue;

        SolidVector3 start = { startNode->m_X, startNode->m_Y, startNode->m_Z };
        SolidVector3 end = { endNode->m_X, endNode->m_Y, endNode->m_Z };
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
            solidPoints->InsertNextPoint(
                start[0] + point.y * localY[0] + point.z * localZ[0],
                start[1] + point.y * localY[1] + point.z * localZ[1],
                start[2] + point.y * localY[2] + point.z * localZ[2]);
        }
        const vtkIdType endBase = solidPoints->GetNumberOfPoints();
        for (const SectionPoint& point : contour)
        {
            solidPoints->InsertNextPoint(
                end[0] + point.y * localY[0] + point.z * localZ[0],
                end[1] + point.y * localY[1] + point.z * localZ[1],
                end[2] + point.y * localY[2] + point.z * localZ[2]);
        }

        const vtkIdType contourCount = static_cast<vtkIdType>(contour.size());
        for (vtkIdType index = 0; index < contourCount; ++index)
        {
            const vtkIdType next = (index + 1) % contourCount;
            const vtkIdType side[4] = {
                startBase + index, startBase + next, endBase + next, endBase + index
            };
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
    QTimer::singleShot(0, this, [this]() {
        updateAdaptiveLabels();
        updateRotationCenterIndicator();
        requestRender();
    });
}

void ModelViewport::mouseReleaseEvent(QMouseEvent* event)
{
    const bool selectClick = m_structure
        && event->button() == Qt::LeftButton
        && m_interactionMode == InteractionMode::Select;
    const QPointF clickPosition = event->position();
    QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
    if (selectClick)
    {
        if (m_skipNextSelectionRelease)
            m_skipNextSelectionRelease = false;
        else
            performSelectionAt(clickPosition.x(), clickPosition.y());
    }
    QTimer::singleShot(0, this, [this]() {
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

    vtkCamera* camera = m_renderer->GetActiveCamera();
    camera->ParallelProjectionOn();
    const double scale = devicePixelRatioF();
    const double displayX = event->position().x() * scale;
    const double displayY = (height() - event->position().y()) * scale;
    double worldBefore[3] = {};
    if (!displayToFocalPlaneWorld(displayX, displayY, worldBefore))
    {
        QVTKOpenGLNativeWidget::wheelEvent(event);
        return;
    }

    const double steps = static_cast<double>(event->angleDelta().y()) / 120.0;
    const double factor = std::pow(1.18, steps);
    camera->SetParallelScale(qBound(1.0e-12,
        camera->GetParallelScale() / factor, 1.0e18));

    double worldAfter[3] = {};
    if (displayToFocalPlaneWorld(displayX, displayY, worldAfter))
    {
        double position[3] = {};
        double focal[3] = {};
        camera->GetPosition(position);
        camera->GetFocalPoint(focal);
        const double offset[3] = { worldBefore[0] - worldAfter[0],
            worldBefore[1] - worldAfter[1], worldBefore[2] - worldAfter[2] };
        camera->SetPosition(position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]);
        camera->SetFocalPoint(focal[0] + offset[0], focal[1] + offset[1], focal[2] + offset[2]);
    }

    m_renderer->ResetCameraClippingRange();
    updateAdaptiveLabels();
    updateRotationCenterIndicator();
    requestRender();
    event->accept();
}

void ModelViewport::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (m_structure && event->button() == Qt::LeftButton
        && m_interactionMode == InteractionMode::Select)
        m_skipNextSelectionRelease = true;
    QVTKOpenGLNativeWidget::mouseDoubleClickEvent(event);
}

bool ModelViewport::displayToFocalPlaneWorld(double displayX, double displayY, double world[3]) const
{
    if (!m_renderer || !world)
        return false;
    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (!camera)
        return false;

    double focal[3] = {};
    camera->GetFocalPoint(focal);
    m_renderer->SetWorldPoint(focal[0], focal[1], focal[2], 1.0);
    m_renderer->WorldToDisplay();
    const double* focalDisplay = m_renderer->GetDisplayPoint();
    if (!focalDisplay || !std::isfinite(focalDisplay[2]))
        return false;

    m_renderer->SetDisplayPoint(displayX, displayY, focalDisplay[2]);
    m_renderer->DisplayToWorld();
    const double* homogeneous = m_renderer->GetWorldPoint();
    if (!homogeneous || !std::isfinite(homogeneous[3]) || std::abs(homogeneous[3]) <= 1.0e-12)
        return false;
    world[0] = homogeneous[0] / homogeneous[3];
    world[1] = homogeneous[1] / homogeneous[3];
    world[2] = homogeneous[2] / homogeneous[3];
    return std::isfinite(world[0]) && std::isfinite(world[1]) && std::isfinite(world[2]);
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
        const QList<vtkActor*> actors = { m_nodeActor.GetPointer(), m_elementActor.GetPointer(),
            m_solidActor.GetPointer() };
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
                center[0] = picked[0]; center[1] = picked[1]; center[2] = picked[2];
            }
            else
                snapped = false;
        }
        for (int index = 0; index < actors.size(); ++index)
            actors.at(index)->SetPickable(oldPickable[static_cast<std::size_t>(index)]);
    }

    m_rotationCenter = { center[0], center[1], center[2] };
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
    if (!m_rotationCenterIndicator || !m_rotationCenterValid || !hasModel()
        || m_interactionMode == InteractionMode::Select)
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
    m_rotationCenterIndicator->show();
    m_rotationCenterIndicator->raise();
}

void ModelViewport::performSelectionAt(double widgetX, double widgetY)
{
    const double scale = devicePixelRatioF();
    const double x = widgetX * scale;
    const double y = (height() - widgetY) * scale;

    const auto selectElement = [this](int elementId) {
        const auto found = std::find(m_cellElementIds.cbegin(), m_cellElementIds.cend(), elementId);
        if (found == m_cellElementIds.cend())
            return false;
        const vtkIdType cellId = static_cast<vtkIdType>(
            std::distance(m_cellElementIds.cbegin(), found));
        highlightElement(cellId);
        emit elementSelected(elementId);
        return true;
    };

    m_hardwarePicker->InitializePickList();
    m_hardwarePicker->PickFromListOn();
    m_hardwarePicker->SnapToMeshPointOn();
    m_hardwarePicker->SetPixelTolerance(6);
    if (m_nodeActor->GetVisibility())
        m_hardwarePicker->AddPickList(m_nodeActor);
    if (m_nodeActor->GetVisibility()
        && m_hardwarePicker->Pick(x, y, 0.0, m_renderer)
        && m_hardwarePicker->GetActor() == m_nodeActor.GetPointer()
        && m_hardwarePicker->GetPointId() >= 0)
    {
        const vtkIdType pointId = m_hardwarePicker->GetPointId();
        if (pointId < static_cast<vtkIdType>(m_pointNodeIds.size()))
        {
            m_hardwarePicker->InitializePickList();
            highlightNode(pointId);
            emit nodeSelected(m_pointNodeIds[static_cast<std::size_t>(pointId)]);
            return;
        }
    }

    // In solid mode the visible surface is much wider than the center line.
    // Pick the surface first and translate its polygon back to the owning
    // model element, while preserving the actor's normal non-pickable state.
    if (m_solidActor->GetVisibility() && !m_solidCellElementIds.empty())
    {
        const int originalPickable = m_solidActor->GetPickable();
        m_solidActor->PickableOn();
        m_hardwarePicker->InitializePickList();
        m_hardwarePicker->SnapToMeshPointOff();
        m_hardwarePicker->AddPickList(m_solidActor);
        const bool picked = m_hardwarePicker->Pick(x, y, 0.0, m_renderer)
            && m_hardwarePicker->GetActor() == m_solidActor.GetPointer()
            && m_hardwarePicker->GetCellId() >= 0;
        const vtkIdType solidCellId = m_hardwarePicker->GetCellId();
        m_solidActor->SetPickable(originalPickable);
        if (picked && solidCellId < static_cast<vtkIdType>(m_solidCellElementIds.size())
            && selectElement(m_solidCellElementIds[static_cast<std::size_t>(solidCellId)]))
        {
            m_hardwarePicker->InitializePickList();
            return;
        }
    }

    m_hardwarePicker->InitializePickList();
    m_hardwarePicker->SnapToMeshPointOff();
    if (m_elementActor->GetVisibility())
        m_hardwarePicker->AddPickList(m_elementActor);
    if (m_elementActor->GetVisibility()
        && m_hardwarePicker->Pick(x, y, 0.0, m_renderer)
        && m_hardwarePicker->GetActor() == m_elementActor.GetPointer()
        && m_hardwarePicker->GetCellId() >= 0)
    {
        const vtkIdType cellId = m_hardwarePicker->GetCellId();
        if (cellId < static_cast<vtkIdType>(m_cellElementIds.size()))
        {
            m_hardwarePicker->InitializePickList();
            highlightElement(cellId);
            emit elementSelected(m_cellElementIds[static_cast<std::size_t>(cellId)]);
            return;
        }
    }

    // Hardware picking is exact for thin line primitives. Use a cell picker
    // with a small screen-relative tolerance as a fallback, matching the
    // forgiving point-selection feel of the reference FEM application.
    m_cellPicker->InitializePickList();
    m_cellPicker->PickFromListOn();
    m_cellPicker->SetTolerance(0.006);
    if (m_elementActor->GetVisibility())
        m_cellPicker->AddPickList(m_elementActor);
    if (m_elementActor->GetVisibility()
        && m_cellPicker->Pick(x, y, 0.0, m_renderer)
        && m_cellPicker->GetActor() == m_elementActor.GetPointer()
        && m_cellPicker->GetCellId() >= 0)
    {
        const vtkIdType cellId = m_cellPicker->GetCellId();
        if (cellId < static_cast<vtkIdType>(m_cellElementIds.size()))
        {
            m_cellPicker->InitializePickList();
            highlightElement(cellId);
            emit elementSelected(m_cellElementIds[static_cast<std::size_t>(cellId)]);
            return;
        }
    }

    m_cellPicker->InitializePickList();
    m_hardwarePicker->InitializePickList();
    clearSelectionHighlight();
    emit selectionCleared();
}

void ModelViewport::contextMenuEvent(QContextMenuEvent* event)
{
    QMenu menu(this);
    QMenu* interactionMenu = menu.addMenu(QStringLiteral("交互模式"));
    QActionGroup modeGroup(&menu);
    modeGroup.setExclusive(true);
    QAction* selectAction = interactionMenu->addAction(QStringLiteral("选择"));
    QAction* rotateAction = interactionMenu->addAction(QStringLiteral("旋转"));
    QAction* panAction = interactionMenu->addAction(QStringLiteral("平移"));
    QAction* zoomAction = interactionMenu->addAction(QStringLiteral("缩放"));
    const QList<QAction*> modeActions = { selectAction, rotateAction, panAction, zoomAction };
    for (QAction* action : modeActions)
    {
        action->setCheckable(true);
        modeGroup.addAction(action);
    }
    modeActions.at(static_cast<int>(m_interactionMode))->setChecked(true);

    menu.addSeparator();
    QAction* nodesAction = menu.addAction(QStringLiteral("显示节点"));
    QAction* elementsAction = menu.addAction(QStringLiteral("显示单元"));
    QAction* solidAction = menu.addAction(QStringLiteral("显示实体"));
    QAction* nodeIdsAction = menu.addAction(QStringLiteral("显示节点 ID"));
    QAction* elementIdsAction = menu.addAction(QStringLiteral("显示单元 ID"));
    for (QAction* action : { nodesAction, elementsAction, solidAction, nodeIdsAction, elementIdsAction })
        action->setCheckable(true);
    nodesAction->setChecked(m_nodesVisible);
    elementsAction->setChecked(m_elementsVisible);
    solidAction->setChecked(m_solidVisible);
    solidAction->setEnabled(hasSolidGeometry());
    nodeIdsAction->setChecked(m_nodeLabelsVisible);
    elementIdsAction->setChecked(m_elementLabelsVisible);

    menu.addSeparator();
    QMenu* viewsMenu = menu.addMenu(QStringLiteral("标准视图"));
    QAction* frontAction = viewsMenu->addAction(QStringLiteral("前视图"));
    QAction* backAction = viewsMenu->addAction(QStringLiteral("后视图"));
    QAction* leftAction = viewsMenu->addAction(QStringLiteral("左视图"));
    QAction* rightAction = viewsMenu->addAction(QStringLiteral("右视图"));
    QAction* topAction = viewsMenu->addAction(QStringLiteral("顶视图"));
    QAction* bottomAction = viewsMenu->addAction(QStringLiteral("底视图"));
    QAction* fitAction = menu.addAction(QStringLiteral("适应窗口"));
    QMenu* centerMenu = menu.addMenu(QStringLiteral("旋转中心"));
    QAction* viewportCenterAction = centerMenu->addAction(QStringLiteral("对准视口中心（自动吸附）"));
    QAction* modelCenterAction = centerMenu->addAction(QStringLiteral("恢复到模型中心"));
    viewportCenterAction->setEnabled(hasModel());
    modelCenterAction->setEnabled(hasModel());

    QAction* selected = menu.exec(event->globalPos());
    if (!selected)
        return;
    if (selected == selectAction) setInteractionMode(InteractionMode::Select);
    else if (selected == rotateAction) setInteractionMode(InteractionMode::Rotate);
    else if (selected == panAction) setInteractionMode(InteractionMode::Pan);
    else if (selected == zoomAction) setInteractionMode(InteractionMode::Zoom);
    else if (selected == nodesAction) setNodesVisible(nodesAction->isChecked());
    else if (selected == elementsAction) setElementsVisible(elementsAction->isChecked());
    else if (selected == solidAction) setSolidVisible(solidAction->isChecked());
    else if (selected == nodeIdsAction) setNodeLabelsVisible(nodeIdsAction->isChecked());
    else if (selected == elementIdsAction) setElementLabelsVisible(elementIdsAction->isChecked());
    else if (selected == frontAction) setStandardView(StandardView::Front);
    else if (selected == backAction) setStandardView(StandardView::Back);
    else if (selected == leftAction) setStandardView(StandardView::Left);
    else if (selected == rightAction) setStandardView(StandardView::Right);
    else if (selected == topAction) setStandardView(StandardView::Top);
    else if (selected == bottomAction) setStandardView(StandardView::Bottom);
    else if (selected == fitAction) resetCamera();
    else if (selected == viewportCenterAction) updateRotationCenterToViewportCenter(true);
    else if (selected == modelCenterAction)
    {
        double bounds[6] = {};
        m_renderer->ComputeVisiblePropBounds(bounds);
        m_rotationCenter = { (bounds[0] + bounds[1]) * 0.5,
            (bounds[2] + bounds[3]) * 0.5, (bounds[4] + bounds[5]) * 0.5 };
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
    m_elementActor->GetProperty()->SetColor(
        colors.element[0], colors.element[1], colors.element[2]);
    if (m_nodeActor)
    m_nodeActor->GetProperty()->SetColor(
        colors.node[0], colors.node[1], colors.node[2]);
    if (m_solidActor)
        m_solidActor->GetProperty()->SetColor(
            colors.solid[0], colors.solid[1], colors.solid[2]);
    if (m_nodeLabelMapper)
        m_nodeLabelMapper->GetLabelTextProperty()->SetColor(
            colors.node[0], colors.node[1], colors.node[2]);
    if (m_elementLabelMapper)
        m_elementLabelMapper->GetLabelTextProperty()->SetColor(
            colors.element[0], colors.element[1], colors.element[2]);
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
        painter.drawPolygon(QPolygonF{ QPointF(16, 3), QPointF(27, 9), QPointF(16, 15), QPointF(5, 9) });
        painter.drawPolyline(QPolygonF{ QPointF(5, 9), QPointF(5, 22), QPointF(16, 29), QPointF(27, 22), QPointF(27, 9) });
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
    QTimer::singleShot(0, this, [this]() {
        m_renderPending = false;
        if (m_renderWindow)
            m_renderWindow->Render();
    });
}

void ModelViewport::clearSelectionHighlight()
{
    m_selectionData->Initialize();
    m_selectionData->SetPoints(m_points);
    m_selectionActor->SetVisibility(false);
    requestRender();
}

void ModelViewport::highlightNode(vtkIdType pointId)
{
    vtkNew<vtkCellArray> vertices;
    vertices->InsertNextCell(1, &pointId);
    m_selectionData->Initialize();
    m_selectionData->SetPoints(m_points);
    m_selectionData->SetVerts(vertices);
    m_selectionData->Modified();
    m_selectionActor->SetVisibility(true);
    requestRender();
}

void ModelViewport::highlightElement(vtkIdType cellId)
{
    vtkNew<vtkIdList> pointIds;
    m_elementData->GetCellPoints(cellId, pointIds);
    if (pointIds->GetNumberOfIds() < 2)
    {
        clearSelectionHighlight();
        return;
    }

    vtkNew<vtkCellArray> lines;
    lines->InsertNextCell(pointIds);
    m_selectionData->Initialize();
    m_selectionData->SetPoints(m_points);
    m_selectionData->SetLines(lines);
    m_selectionData->Modified();
    m_selectionActor->SetVisibility(true);
    requestRender();
}
