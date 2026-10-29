#pragma once

class QProgressBar;

namespace ProgressBarAnimation
{
void animateTo(QProgressBar* progressBar, int targetValue); // 将进度条平滑更新到目标值
}
