#pragma once

#include <QVTKOpenGLNativeWidget.h>
#include <vtkSmartPointer.h>
#include <vtkType.h>

#include <memory>
#include <array>
#include <unordered_map>
#include <vector>

class QLabel;
class QMouseEvent;
class QWheelEvent;
class QResizeEvent;
class QContextMenuEvent;
class StructureData;
class vtkActor;
class vtkGenericOpenGLRenderWindow;
class vtkPoints;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkRenderer;
class vtkActor2D;
class vtkLabeledDataMapper;
class vtkOrientationMarkerWidget;
class vtkInteractorStyle;
class vtkHardwarePicker;
class vtkCellPicker;
class vtkLookupTable;
class vtkScalarBarActor;
struct Hdf5ResultFrame;

class ModelViewport final : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    enum class InteractionMode
    {
        Select,
        Rotate,
        Pan,
        Zoom
    };

    enum class StandardView
    {
        Front,
        Back,
        Left,
        Right,
        Top,
        Bottom,
        Isometric
    };

    enum class ResultField
    {
        DisplacementMagnitude,
        DisplacementX,
        DisplacementY,
        DisplacementZ,
        AxialForce,
        Stress,
        Strain
    };

    explicit ModelViewport(QWidget* parent = nullptr);

    void displayModel(const std::shared_ptr<StructureData>& structure, bool resetCamera = true);
    void clearModel();
    void setThemeIndex(int themeIndex);
    void setNodeSize(int nodeSize);
    void setNodeLabelsVisible(bool visible);
    void setAdaptiveNodeLabels(bool adaptive);
    void setElementLabelsVisible(bool visible);
    void setIdLabelsPlaybackLocked(bool locked);
    void setNodesVisible(bool visible);
    void setElementsVisible(bool visible);
    void setSolidVisible(bool visible);
    void resetCamera();
    void setInteractionMode(InteractionMode mode);
    void setStandardView(StandardView view);
    bool updateNodePosition(int nodeId, double x, double y, double z);
    bool hasModel() const;
    bool hasSolidGeometry() const;
    bool displayResultFrame(const Hdf5ResultFrame& frame, ResultField field,
        double deformationScale = 0.0, bool showOriginal = true);
    void setResultScalarRange(double minimum, double maximum);
    void clearResultScalarRange();
    void clearResultDisplay();
    double activeDeformationScale() const;

signals:
    void nodeSelected(int nodeId);
    void elementSelected(int elementId);
    void selectionCleared();
    void interactionModeChanged(ModelViewport::InteractionMode mode);
    void nodesVisibilityChanged(bool visible);
    void elementsVisibilityChanged(bool visible);
    void solidVisibilityChanged(bool visible);
    void nodeLabelsVisibilityChanged(bool visible);
    void elementLabelsVisibilityChanged(bool visible);

protected:
    void resizeEvent(QResizeEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void contextMenuEvent(QContextMenuEvent* event) override;

private:
    void updateThemeColors();
    void updateEmptyStateGeometry();
    void requestRender();
    void clearSelectionHighlight();
    void highlightNode(vtkIdType pointId);
    void highlightElement(vtkIdType cellId);
    void updateElementLabelPositions();
    void updateAdaptiveLabels();
    void rebuildSolidGeometry();
    void updateNodeDisplaySize();
    void performSelectionAt(double widgetX, double widgetY);
    bool ensureRotationCenter(double center[3]);
    void updateRotationCenterToViewportCenter(bool snapToGeometry);
    void updateRotationCenterIndicator();
    void updateRotationCenterIndicatorAppearance();
    bool displayToFocalPlaneWorld(double displayX, double displayY, double world[3]) const;

    int m_themeIndex = 0;
    int m_nodeSize = 5;
    bool m_nodeLabelsVisible = false;
    bool m_adaptiveNodeLabels = false;
    bool m_elementLabelsVisible = false;
    bool m_idLabelsPlaybackLocked = false;
    bool m_nodesVisible = true;
    bool m_elementsVisible = true;
    bool m_solidVisible = false;
    bool m_rotationCenterValid = false;
    bool m_rotationCenterSnapped = false;
    std::array<double, 3> m_rotationCenter{ 0.0, 0.0, 0.0 };
    InteractionMode m_interactionMode = InteractionMode::Select;
    std::shared_ptr<StructureData> m_structure;
    std::vector<int> m_pointNodeIds;
    std::vector<int> m_cellElementIds;
    std::vector<int> m_solidCellElementIds;
    std::unordered_map<int, vtkIdType> m_nodePointIds;
    std::vector<std::array<double, 3>> m_elementLabelPositions;
    std::unordered_map<int, std::array<double, 3>> m_originalNodeCoordinates;
    double m_activeDeformationScale = 1.0;
    double m_resultScalarMinimum = 0.0;
    double m_resultScalarMaximum = 0.0;
    bool m_resultScalarRangeLocked = false;
    bool m_resultDisplayActive = false;
    bool m_renderPending = false;
    bool m_skipNextSelectionRelease = false;
    QLabel* m_emptyStateLabel = nullptr;
    QLabel* m_emptyStateIconLabel = nullptr;
    QLabel* m_rotationCenterIndicator = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkPoints> m_points;
    vtkSmartPointer<vtkPolyData> m_elementData;
    vtkSmartPointer<vtkPolyData> m_nodeData;
    vtkSmartPointer<vtkPolyData> m_nodeLabelData;
    vtkSmartPointer<vtkPolyData> m_elementLabelData;
    vtkSmartPointer<vtkPolyData> m_solidData;
    vtkSmartPointer<vtkPolyData> m_selectionData;
    vtkSmartPointer<vtkPolyData> m_originalElementData;
    vtkSmartPointer<vtkPolyDataMapper> m_elementMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_nodeMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_selectionMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_solidMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_originalElementMapper;
    vtkSmartPointer<vtkLabeledDataMapper> m_nodeLabelMapper;
    vtkSmartPointer<vtkLabeledDataMapper> m_elementLabelMapper;
    vtkSmartPointer<vtkActor> m_elementActor;
    vtkSmartPointer<vtkActor> m_nodeActor;
    vtkSmartPointer<vtkActor> m_selectionActor;
    vtkSmartPointer<vtkActor> m_solidActor;
    vtkSmartPointer<vtkActor> m_originalElementActor;
    vtkSmartPointer<vtkActor2D> m_nodeLabelActor;
    vtkSmartPointer<vtkActor2D> m_elementLabelActor;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_axesWidget;
    vtkSmartPointer<vtkInteractorStyle> m_interactorStyle;
    vtkSmartPointer<vtkHardwarePicker> m_hardwarePicker;
    vtkSmartPointer<vtkCellPicker> m_cellPicker;
    vtkSmartPointer<vtkLookupTable> m_resultLookupTable;
    vtkSmartPointer<vtkScalarBarActor> m_resultScalarBar;
};

Q_DECLARE_METATYPE(ModelViewport::InteractionMode)
