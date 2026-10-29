#pragma once

#include "Widgets/ViewportCameraController.h"

#include <QVTKOpenGLNativeWidget.h>
#include <QPointF>
#include <QRectF>
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
class QKeyEvent;
class QRubberBand;
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
        Zoom,
        ZoomToRect
    };

    enum class SelectionTarget
    {
        Auto,
        NodesOnly,
        ElementsOnly
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

    explicit ModelViewport(QWidget* parent = nullptr);                                           // 创建模型与结果三维视口

    void displayModel(const std::shared_ptr<StructureData>& structure, bool resetCamera = true); // 构建模型显示数据并按需重置相机
    void clearModel();                                                                           // 清除模型、选择和结果显示
    void setThemeIndex(int themeIndex);                                                          // 切换视口背景及模型主题颜色
    void setNodeSize(int nodeSize);                                                              // 设置节点显示尺寸
    void setNodeLabelsVisible(bool visible);                                                     // 切换节点编号的显示状态
    void setAdaptiveNodeLabels(bool adaptive);                                                   // 设置节点编号是否随视图自适应显示
    void setElementLabelsVisible(bool visible);                                                  // 切换单元编号的显示状态
    void setIdLabelsPlaybackLocked(bool locked);                                                 // 设置播放期间编号标注的锁定状态
    void setNodesVisible(bool visible);                                                          // 切换节点几何的显示状态
    void setElementsVisible(bool visible);                                                       // 切换单元线框的显示状态
    void setSolidVisible(bool visible);                                                          // 切换单元实体几何的显示状态
    void resetCamera();                                                                          // 调整相机使可见模型适合当前视口
    void setInteractionMode(InteractionMode mode);                                               // 切换选择、旋转、平移或缩放操作模式
    void setSelectionTarget(SelectionTarget target);                                             // 设置节点、单元或自动识别的拾取目标
    SelectionTarget selectionTarget() const noexcept { return m_selectionTarget; }               // 返回当前拾取目标类型
    void setStandardView(StandardView view);                                                     // 切换到指定标准观察方向
    bool setSelectionIsolation(bool enabled);                                                    // 切换仅显示已选实体的隔离模式
    bool isSelectionIsolated() const noexcept { return m_selectionIsolated; }                    // 查询当前是否处于选择隔离模式
    bool updateNodePosition(int nodeId, double x, double y, double z);                           // 更新指定节点的显示坐标和关联几何
    bool hasModel() const;                                                                       // 查询视口是否已加载模型
    bool hasSolidGeometry() const;                                                               // 查询模型是否包含可显示的实体几何
    bool displayResultFrame(const Hdf5ResultFrame& frame, ResultField field, double deformationScale = 0.0,
                            bool showOriginal = true, bool refreshSolidGeometry = true);         // 显示指定结果帧、结果场和变形比例
    double automaticDeformationScale(double maximumDisplacement) const;                          // 根据模型尺度与最大位移估算变形比例
    void setResultScalarRange(double minimum, double maximum);                                   // 固定结果颜色映射范围
    void clearResultScalarRange();                                                               // 解除结果颜色映射范围锁定
    void clearResultDisplay();                                                                   // 清除结果场并恢复模型显示
    double activeDeformationScale() const;                                                       // 返回当前实际使用的变形比例

signals:
    void nodeSelected(int nodeId);                                                               // 通知选中的节点编号
    void elementSelected(int elementId);                                                         // 通知选中的单元编号
    void selectionCleared();                                                                     // 通知选择已清空
    void selectionChanged(int nodeCount, int elementCount);                                      // 通知当前选中的节点数和单元数
    void interactionModeChanged(ModelViewport::InteractionMode mode);                            // 通知视口操作模式已改变
    void selectionTargetChanged(ModelViewport::SelectionTarget target);                          // 通知拾取目标类型已改变
    void nodesVisibilityChanged(bool visible);                                                   // 通知节点显示状态已改变
    void elementsVisibilityChanged(bool visible);                                                // 通知单元线框显示状态已改变
    void solidVisibilityChanged(bool visible);                                                   // 通知实体几何显示状态已改变
    void nodeLabelsVisibilityChanged(bool visible);                                              // 通知节点编号显示状态已改变
    void elementLabelsVisibilityChanged(bool visible);                                           // 通知单元编号显示状态已改变

protected:
    void resizeEvent(QResizeEvent* event) override;                                              // 在视口尺寸变化时更新提示及显示布局
    void mousePressEvent(QMouseEvent* event) override;                                           // 记录拾取、框选或相机拖动的起点
    void mouseMoveEvent(QMouseEvent* event) override;                                            // 更新框选区域及相机拖动操作
    void mouseDoubleClickEvent(QMouseEvent* event) override;                                     // 抑制双击后松键触发的重复选择
    void mouseReleaseEvent(QMouseEvent* event) override;                                         // 结束拖动并提交选择或框选缩放
    void wheelEvent(QWheelEvent* event) override;                                                // 围绕旋转中心滚轮缩放并刷新标注
    void keyPressEvent(QKeyEvent* event) override;                                               // 处理视口选择与显示快捷键
    void contextMenuEvent(QContextMenuEvent* event) override;                                    // 显示视口操作和显示设置菜单

private:
    void updateThemeColors();                                                                    // 更新模型、标注和背景的主题颜色
    void updateEmptyStateGeometry();                                                             // 调整空模型提示在视口中的位置
    void requestRender();                                                                        // 合并渲染请求并安排视口重绘
    void clearSelectionHighlight();                                                              // 清除已选实体的高亮几何
    void refreshSelectionHighlight();                                                            // 根据当前选择重建高亮几何
    void updateElementLabelPositions();                                                          // 更新单元编号标注位置
    void updateAdaptiveLabels();                                                                 // 按当前视图更新自适应编号标注
    void rebuildSolidGeometry();                                                                 // 根据单元与截面重建实体显示几何
    void updateNodeDisplaySize();                                                                // 更新节点绘制尺寸
    void performSelectionAt(double widgetX, double widgetY, bool additive);                      // 在指定视口位置拾取节点或单元
    void performSelectionInRect(const QRectF& widgetRect, bool additive);                        // 在指定矩形范围内选择可见实体
    bool collectVisibleCellsInRect(const QRectF& widgetRect, vtkActor* targetActor,
                                   const std::vector<vtkActor*>& occluderActors,
                                   std::vector<vtkIdType>& cellIds) const;                       // 收集框选区域内未被遮挡的几何单元
    bool addNodeToSelection(vtkIdType pointId, bool additive);                                   // 将拾取到的节点加入当前选择
    bool addElementToSelection(vtkIdType cellId, bool additive);                                 // 将拾取到的单元加入当前选择
    void emitSelectionSummary();                                                                 // 发送当前选择内容与数量通知
    bool setRotationCenterAt(double widgetX, double widgetY, bool allowFocalPlane = false);      // 通过视口位置拾取并设置旋转中心
    bool ensureRotationCenter(double center[3]);                                                 // 获取或补建有效的旋转中心
    void updateRotationCenterToViewportCenter(bool snapToGeometry);                              // 将旋转中心更新到视口中心附近
    void updateRotationCenterIndicator();                                                        // 更新旋转中心标记的位置和可见性
    void updateRotationCenterIndicatorAppearance();                                              // 更新旋转中心标记的外观
    void applyBaseActorVisibility();                                                             // 统一应用节点、线框和实体显示开关
    bool displayToFocalPlaneWorld(double displayX, double displayY, double world[3]) const;      // 将显示坐标转换到相机焦平面的世界坐标
    bool panByDisplayDelta(int deltaX, int deltaY);                                              // 按显示坐标位移平移相机
    bool zoomAtRotationCenter(double factor);                                                    // 以当前旋转中心为基点缩放视图
    bool zoomToWidgetRect(const QRectF& widgetRect);                                             // 缩放相机以填满指定视口矩形区域

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
    std::array<double, 3> m_rotationCenter{0.0, 0.0, 0.0};
    InteractionMode m_interactionMode = InteractionMode::Select;
    SelectionTarget m_selectionTarget = SelectionTarget::Auto;
    std::shared_ptr<StructureData> m_structure;
    std::vector<int> m_pointNodeIds;
    std::vector<int> m_cellElementIds;
    std::vector<int> m_solidCellElementIds;
    std::unordered_map<int, vtkIdType> m_nodePointIds;
    struct SpringPointReference
    {
        vtkIdType pointId = -1;
        int firstNodeId = -1;
        int secondNodeId = -1;
        double secondNodeWeight = 0.0;
    };
    std::vector<SpringPointReference> m_springPointReferences;
    std::vector<std::array<double, 3>> m_elementLabelPositions;
    std::unordered_map<int, std::array<double, 3>> m_originalNodeCoordinates;
    double m_activeDeformationScale = 1.0;
    double m_resultScalarMinimum = 0.0;
    double m_resultScalarMaximum = 0.0;
    bool m_resultScalarRangeLocked = false;
    bool m_resultDisplayActive = false;
    bool m_selectionIsolated = false;
    bool m_originalElementVisibleBeforeIsolation = false;
    bool m_renderPending = false;
    bool m_skipNextSelectionRelease = false;
    bool m_selectionPressActive = false;
    bool m_selectionDragActive = false;
    bool m_rotationCenterDragActive = false;
    bool m_cameraModePressActive = false;
    QPointF m_selectionPressPosition;
    QPointF m_cameraModePressPosition;
    Qt::KeyboardModifiers m_selectionPressModifiers = Qt::NoModifier;
    bool m_zoomRectPressActive = false;
    bool m_zoomRectDragActive = false;
    QPointF m_zoomRectPressPosition;
    std::unordered_set<vtkIdType> m_selectedNodePointIds;
    std::unordered_set<vtkIdType> m_selectedElementCellIds;
    QLabel* m_emptyStateLabel = nullptr;
    QLabel* m_emptyStateIconLabel = nullptr;
    QLabel* m_rotationCenterIndicator = nullptr;
    QRubberBand* m_selectionRubberBand = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkPoints> m_points;
    vtkSmartPointer<vtkPolyData> m_elementData;
    vtkSmartPointer<vtkPolyData> m_nodeData;
    vtkSmartPointer<vtkPolyData> m_nodeLabelData;
    vtkSmartPointer<vtkPolyData> m_elementLabelData;
    vtkSmartPointer<vtkPolyData> m_solidData;
    vtkSmartPointer<vtkPolyData> m_selectionData;
    vtkSmartPointer<vtkPolyData> m_solidSelectionData;
    vtkSmartPointer<vtkPolyData> m_originalElementData;
    vtkSmartPointer<vtkPolyDataMapper> m_elementMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_nodeMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_selectionMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_solidSelectionMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_solidMapper;
    vtkSmartPointer<vtkPolyDataMapper> m_originalElementMapper;
    vtkSmartPointer<vtkLabeledDataMapper> m_nodeLabelMapper;
    vtkSmartPointer<vtkLabeledDataMapper> m_elementLabelMapper;
    vtkSmartPointer<vtkActor> m_elementActor;
    vtkSmartPointer<vtkActor> m_nodeActor;
    vtkSmartPointer<vtkActor> m_selectionActor;
    vtkSmartPointer<vtkActor> m_solidSelectionActor;
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
    std::unique_ptr<ViewportCameraController> m_cameraController;
};

Q_DECLARE_METATYPE(ModelViewport::InteractionMode)
Q_DECLARE_METATYPE(ModelViewport::SelectionTarget)
