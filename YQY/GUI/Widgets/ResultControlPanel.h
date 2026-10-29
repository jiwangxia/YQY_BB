#pragma once

#include "Export/Hdf5ResultData.h"
#include <functional>
#include <vector>
#include <QWidget>

class QFormLayout;
class QGridLayout;
class QHBoxLayout;
class QVBoxLayout;
class ResultScaleSpinBox;
class ResultSelectionComboBox;

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QEvent;
class QLabel;
class QPushButton;
class QResizeEvent;
class QSlider;
class QTimer;
class ResultControlPanel final : public QWidget
{
public:
    explicit ResultControlPanel(QWidget* parent = nullptr);                 // 创建结果选择、播放与导出面板
    ~ResultControlPanel() override;                                         // 销毁结果选择、播放与导出面板
    QComboBox* fieldCombo() const;                                          // 返回结果场选择框
    QDoubleSpinBox* scaleSpin() const;                                      // 返回变形放大比例输入框
    QCheckBox* originalCheck() const;                                       // 返回叠加显示原始模型的选项
    QPushButton* playButton() const;                                        // 返回结果动画播放按钮
    QSlider* frameSlider() const;                                           // 返回结果帧位置滑块
    QComboBox* speedCombo() const;                                          // 返回动画播放速度选择框
    QLabel* frameLabel() const;                                             // 返回当前帧编号显示标签
    QLabel* timeValueLabel() const;                                         // 返回当前结果时间显示标签
    QLabel* deformationValueLabel() const;                                  // 返回当前变形比例显示标签
    QPushButton* exportButton() const;                                      // 返回节点结果导出按钮
    QPushButton* exportElementButton() const;                               // 返回单元结果导出按钮
    QPushButton* exportIterationButton() const;                             // 返回迭代历史导出按钮
    QPushButton* exportPlotButton() const;                                  // 返回时程曲线查看按钮
    QPushButton* exportEnergyButton() const;                                // 返回能量数据导出按钮
    QComboBox* stepCombo() const;                                           // 返回结果分析步选择框
    std::vector<Hdf5ResultFrameInfo>& frames();                             // 返回已加载的结果帧索引列表
    const std::vector<Hdf5ResultFrameInfo>& frames() const;                 // 返回已加载的结果帧索引列表
    QString& resultFilePath();                                              // 返回当前结果文件路径
    const QString& resultFilePath() const;                                  // 返回当前结果文件路径
    bool& partialResult();                                                  // 返回当前结果是否为未完整输出的状态
    double& automaticScale();                                               // 返回当前自动变形放大比例
    QTimer* playbackTimer() const;                                          // 返回结果动画播放定时器
    void setFrameChangedHandler(std::function<void(double)> handler);       // 设置播放位置改变时的回调
    void setVisualizationChangedHandler(std::function<void()> handler);     // 设置结果显示参数改变时的回调
    void setPlaybackStateChangedHandler(std::function<void(bool)> handler); // 设置播放启停状态改变时的回调
    void setExportHandler(std::function<void()> handler);                   // 设置节点结果导出回调
    void setElementExportHandler(std::function<void()> handler);            // 设置单元结果导出回调
    void setIterationExportHandler(std::function<void()> handler);          // 设置迭代历史导出回调
    void setPlotExportHandler(std::function<void()> handler);               // 设置时程曲线查看回调
    void setEnergyExportHandler(std::function<void()> handler);             // 设置能量数据导出回调
    void stopPlayback();                                                    // 停止结果动画并恢复播放按钮状态

protected:
    bool event(QEvent* event) override;                                      // 显示或样式改变时更新导出按钮分栏
    void resizeEvent(QResizeEvent* event) override;                          // 根据侧栏可用宽度调整导出按钮分栏

private:
    void initializeUi();                                                    // 创建界面控件并组织布局
    void updateExportLayout();                                              // 窄侧栏使用单列，保持操作文字完整

    QVBoxLayout* m_cardLayout = nullptr;
    QFormLayout* m_resultForm = nullptr;
    QLabel* m_fieldLabel = nullptr;
    ResultSelectionComboBox* m_fieldCombo = nullptr;
    QLabel* m_stepLabel = nullptr;
    ResultSelectionComboBox* m_stepCombo = nullptr;
    QLabel* m_scaleLabel = nullptr;
    ResultScaleSpinBox* m_scaleSpin = nullptr;
    QCheckBox* m_originalCheck = nullptr;
    QHBoxLayout* m_frameRow = nullptr;
    QPushButton* m_playButton = nullptr;
    QLabel* m_speedLabel = nullptr;
    ResultSelectionComboBox* m_speedCombo = nullptr;
    QSlider* m_frameSlider = nullptr;
    QGridLayout* m_resultFrameStatusLayout = nullptr;
    QLabel* m_frameCaptionLabel = nullptr;
    QLabel* m_frameLabel = nullptr;
    QLabel* m_timeCaptionLabel = nullptr;
    QLabel* m_timeValueLabel = nullptr;
    QLabel* m_deformationCaptionLabel = nullptr;
    QLabel* m_deformationValueLabel = nullptr;
    QGridLayout* m_resultExportGrid = nullptr;
    int m_exportColumns = 2;
    QPushButton* m_exportButton = nullptr;
    QPushButton* m_exportElementButton = nullptr;
    QPushButton* m_exportIterationButton = nullptr;
    QPushButton* m_exportPlotButton = nullptr;
    QPushButton* m_exportEnergyButton = nullptr;

    std::vector<Hdf5ResultFrameInfo> m_frames;
    QString m_resultFilePath;
    bool m_partialResult = false;
    double m_automaticScale = 1.0;
    double m_playbackPosition = 0.0;
    double m_playbackTime = 0.0;
    QTimer* m_playbackTimer = nullptr;
    std::function<void(double)> m_frameChangedHandler;
    std::function<void()> m_visualizationChangedHandler;
    std::function<void(bool)> m_playbackStateChangedHandler;
    std::function<void()> m_exportHandler;
    std::function<void()> m_elementExportHandler;
    std::function<void()> m_iterationExportHandler;
    std::function<void()> m_plotExportHandler;
    std::function<void()> m_energyExportHandler;
};
