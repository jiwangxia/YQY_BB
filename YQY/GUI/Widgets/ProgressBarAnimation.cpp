#include "ProgressBarAnimation.h"

#include <QProgressBar>
#include <QTimer>

namespace ProgressBarAnimation
{
void animateTo(QProgressBar* progressBar, int targetValue)
{
    if (!progressBar)
        return;

    targetValue = qBound(progressBar->minimum(), targetValue, progressBar->maximum());
    progressBar->setProperty("smoothTargetValue", targetValue);

    // A solve may only produce a handful of visible progress events.  Keep one
    // lightweight 60 FPS timer alive per bar and let the displayed value catch
    // up with the latest value instead of restarting an animation on every event.
    auto* timer = progressBar->findChild<QTimer*>(QStringLiteral("smoothProgressTimer"), Qt::FindDirectChildrenOnly);
    if (!timer)
    {
        timer = new QTimer(progressBar);
        timer->setObjectName(QStringLiteral("smoothProgressTimer"));
        timer->setInterval(16);
        QObject::connect(timer, &QTimer::timeout, progressBar,
                         [progressBar, timer]()
                         {
                             const int target = progressBar->property("smoothTargetValue").toInt();
                             const int current = progressBar->value();
                             const int distance = target - current;
                             if (distance == 0)
                             {
                                 timer->stop();
                                 return;
                             }

                             const int step = qMax(1, qRound(qAbs(distance) * 0.38));
                             const int next =
                                 distance > 0 ? qMin(target, current + step) : qMax(target, current - step);
                             progressBar->setValue(next);
                         });
    }

    // A restarted calculation must return to zero immediately. Forward progress
    // is interpolated so even a very small model remains visually continuous.
    if (targetValue < progressBar->value())
        progressBar->setValue(targetValue);
    else if (targetValue != progressBar->value() && !timer->isActive())
        timer->start();
}
}
