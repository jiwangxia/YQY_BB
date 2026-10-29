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

    // 求解进度事件可能较少；每个进度条维护一个 60 FPS 定时器，
    // 持续追赶最新进度，避免每次事件都重启动画。
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

    // 重新计算时立即归零；后续进度采用插值，使小模型的进度显示也保持连续。
    if (targetValue < progressBar->value())
        progressBar->setValue(targetValue);
    else if (targetValue != progressBar->value() && !timer->isActive())
        timer->start();
}
}
