#pragma once

#include <QWidget>
#include "DataStructure/Load/LoadBase.h"
#include "Base/EmptyOUT.h"
#include <array>
#include <vector>
#include <functional>

class QPaintEvent;
class QWidget;

class LoadTimeFunctionPreview final : public QWidget
{
public:
    explicit LoadTimeFunctionPreview(QWidget* parent);                                                            // 创建荷载时间函数预览控件

    void setDefinition(double baseValue, TimeFunctionType type, double startTime, double endTime, bool continuous,
                       double amplitude, double frequency, double phase, double offset, double rampT0, double rampT1,
                       double decay, double period, double dutyCycle, double suggestedEndTime, double probeTime); // 设置预览所用的荷载幅值和时间函数参数

    double valueAt(double time) const;                                                                            // 计算指定时刻的预览荷载值

protected:
    void paintEvent(QPaintEvent*) override;                                                                       // 绘制荷载时程曲线、坐标轴和标注

private:
    double m_baseValue = 1.0;
    TimeFunctionType m_type = TimeFunctionType::CONSTANT;
    double m_startTime = 0.0;
    double m_endTime = 1.0;
    double m_amplitude = 1.0;
    double m_frequency = 1.0;
    double m_phase = 0.0;
    double m_offset = 0.0;
    double m_rampT0 = 0.0;
    double m_rampT1 = 1.0;
    double m_decay = 1.0;
    double m_period = 1.0;
    double m_dutyCycle = 0.5;
    double m_probeTime = 0.0;
    double m_viewStart = 0.0;
    double m_viewEnd = 1.0;
};
