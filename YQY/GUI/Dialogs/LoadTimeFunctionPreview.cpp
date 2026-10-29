#include "LoadTimeFunctionPreview.h"
#include "Utility/AnalysisDisplay.h"
#include <QColor>
#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QStringLiteral>
#include <QWidget>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <variant>
#include <vector>

LoadTimeFunctionPreview::LoadTimeFunctionPreview(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(122);
}

void LoadTimeFunctionPreview::setDefinition(double baseValue, TimeFunctionType type, double startTime, double endTime,
                                            bool continuous, double amplitude, double frequency, double phase,
                                            double offset, double rampT0, double rampT1, double decay, double period,
                                            double dutyCycle, double suggestedEndTime, double probeTime)
{
    m_baseValue = baseValue;
    m_type = type;
    m_startTime = startTime;
    m_endTime = continuous ? std::max(startTime + 1.0, suggestedEndTime) : endTime;
    m_amplitude = amplitude;
    m_frequency = frequency;
    m_phase = phase;
    m_offset = offset;
    m_rampT0 = rampT0;
    m_rampT1 = rampT1;
    m_decay = decay;
    m_period = period;
    m_dutyCycle = dutyCycle;
    m_probeTime = std::clamp(probeTime, m_startTime, m_endTime);
    const double availableDuration = std::max(1.0e-9, m_endTime - m_startTime);
    double visibleDuration = availableDuration;
    if ((m_type == TimeFunctionType::SIN || m_type == TimeFunctionType::COS) && std::abs(m_frequency) > 1.0e-12)
        visibleDuration = std::min(availableDuration, 3.0 / std::abs(m_frequency));
    else if ((m_type == TimeFunctionType::TRIANGULAR || m_type == TimeFunctionType::SQUARE) && m_period > 1.0e-12)
        visibleDuration = std::min(availableDuration, 3.0 * m_period);
    m_viewStart = std::clamp(m_probeTime - visibleDuration * 0.5, m_startTime,
                             std::max(m_startTime, m_endTime - visibleDuration));
    m_viewEnd = m_viewStart + visibleDuration;
    update();
}

double LoadTimeFunctionPreview::valueAt(double time) const
{
    if (time < m_startTime || time > m_endTime)
        return 0.0;
    return m_baseValue * AnalysisDisplay::evaluateTimeFunction(m_type, time - m_startTime, m_amplitude, m_frequency,
                                                               m_phase, m_offset, m_rampT0, m_rampT1, m_decay, m_period,
                                                               m_dutyCycle);
}

void LoadTimeFunctionPreview::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(QStringLiteral("#101924")));
    const QRectF plot = rect().adjusted(48, 20, -16, -28);
    if (plot.width() <= 4.0 || plot.height() <= 4.0)
        return;

    double minimum = 0.0;
    double maximum = 0.0;
    const double duration = std::max(1.0e-9, m_viewEnd - m_viewStart);
    const int columnCount = std::max(24, static_cast<int>(std::floor(plot.width())));
    std::vector<std::pair<double, double>> envelopes;
    envelopes.reserve(static_cast<std::size_t>(columnCount));
    for (int column = 0; column < columnCount; ++column)
    {
        const double intervalStart = m_viewStart + duration * static_cast<double>(column) / columnCount;
        const double intervalEnd = m_viewStart + duration * static_cast<double>(column + 1) / columnCount;
        const int intervalSamples = std::clamp(
            static_cast<int>(std::ceil(std::max(1.0, std::abs(m_frequency) * (intervalEnd - intervalStart) * 24.0))), 8,
            512);
        double intervalMinimum = valueAt(intervalStart);
        double intervalMaximum = intervalMinimum;
        for (int sample = 1; sample <= intervalSamples; ++sample)
        {
            const double time =
                intervalStart + (intervalEnd - intervalStart) * static_cast<double>(sample) / intervalSamples;
            const double value = valueAt(time);
            intervalMinimum = std::min(intervalMinimum, value);
            intervalMaximum = std::max(intervalMaximum, value);
        }
        envelopes.emplace_back(intervalMinimum, intervalMaximum);
        minimum = std::min(minimum, intervalMinimum);
        maximum = std::max(maximum, intervalMaximum);
    }
    if (std::abs(maximum - minimum) <= 1.0e-12)
    {
        minimum -= 1.0;
        maximum += 1.0;
    }
    const auto mapY = [&plot, minimum, maximum](double value)
    {
        return plot.bottom() - (value - minimum) / (maximum - minimum) * plot.height();
    };
    painter.setPen(QPen(QColor(QStringLiteral("#2A3C52")), 1.0));
    for (int line = 0; line <= 4; ++line)
    {
        const double y = plot.top() + plot.height() * static_cast<double>(line) / 4.0;
        painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    }
    if (minimum <= 0.0 && maximum >= 0.0)
    {
        painter.setPen(QPen(QColor(QStringLiteral("#6A8099")), 1.0, Qt::DashLine));
        const double zeroY = mapY(0.0);
        painter.drawLine(QPointF(plot.left(), zeroY), QPointF(plot.right(), zeroY));
    }
    painter.setPen(QPen(QColor(QStringLiteral("#32C6D8")), 1.0, Qt::SolidLine, Qt::RoundCap));
    for (int column = 0; column < columnCount; ++column)
    {
        const double x = plot.left() + plot.width() * (static_cast<double>(column) + 0.5) / columnCount;
        const auto& envelope = envelopes[static_cast<std::size_t>(column)];
        if (envelope.first == envelope.second)
        {
            // 常量及方波平台不能用零长度竖线绘制，否则曲线会消失。
            const double halfColumn = plot.width() / (2.0 * columnCount);
            painter.drawLine(QPointF(x - halfColumn, mapY(envelope.first)),
                             QPointF(x + halfColumn, mapY(envelope.second)));
        }
        else
        {
            painter.drawLine(QPointF(x, mapY(envelope.first)), QPointF(x, mapY(envelope.second)));
        }
    }
    const double probeX = plot.left() + (m_probeTime - m_viewStart) / duration * plot.width();
    const double probeY = mapY(valueAt(m_probeTime));
    painter.setPen(QPen(QColor(QStringLiteral("#F5C451")), 1.0, Qt::DashLine));
    painter.drawLine(QPointF(probeX, plot.top()), QPointF(probeX, plot.bottom()));
    painter.setBrush(QColor(QStringLiteral("#F5C451")));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPointF(probeX, probeY), 3.5, 3.5);
    painter.setPen(QColor(QStringLiteral("#9CB1C7")));
    painter.drawText(QPointF(plot.left(), height() - 8),
                     QStringLiteral("%1 s").arg(AnalysisDisplay::numberText(m_viewStart)));
    const QString endText = QStringLiteral("%1 s").arg(AnalysisDisplay::numberText(m_viewEnd));
    painter.drawText(QPointF(plot.right() - painter.fontMetrics().horizontalAdvance(endText), height() - 8), endText);
    painter.drawText(QPointF(7, plot.top() + 10), QString::number(maximum, 'g', 5));
    painter.drawText(QPointF(7, plot.bottom()), QString::number(minimum, 'g', 5));
}
