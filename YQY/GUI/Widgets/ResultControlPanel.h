#pragma once

#include "Export/Hdf5ResultData.h"

#include <functional>
#include <vector>
#include <QWidget>
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSlider;
class QTimer;
namespace Ui
{
class ResultControlPanelClass;
}
class ResultControlPanel final : public QWidget
{
public:
    explicit ResultControlPanel(QWidget* parent = nullptr);
    ~ResultControlPanel() override;
    QComboBox* fieldCombo() const;
    QDoubleSpinBox* scaleSpin() const;
    QCheckBox* originalCheck() const;
    QPushButton* playButton() const;
    QSlider* frameSlider() const;
    QComboBox* speedCombo() const;
    QLabel* frameLabel() const;
    QLabel* timeValueLabel() const;
    QLabel* deformationValueLabel() const;
    QPushButton* exportButton() const;
    QPushButton* exportElementButton() const;
    QPushButton* exportIterationButton() const;
    std::vector<Hdf5ResultFrameInfo>& frames();
    const std::vector<Hdf5ResultFrameInfo>& frames() const;
    QString& resultFilePath();
    const QString& resultFilePath() const;
    bool& partialResult();
    double& automaticScale();
    QTimer* playbackTimer() const;
    void setFrameChangedHandler(std::function<void(double)> handler);
    void setVisualizationChangedHandler(std::function<void()> handler);
    void setPlaybackStateChangedHandler(std::function<void(bool)> handler);
    void setExportHandler(std::function<void()> handler);
    void setElementExportHandler(std::function<void()> handler);
    void setIterationExportHandler(std::function<void()> handler);
    void stopPlayback();

private:
    Ui::ResultControlPanelClass* m_ui = nullptr;
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
};
