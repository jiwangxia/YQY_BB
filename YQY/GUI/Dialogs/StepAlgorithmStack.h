#pragma once

#include <QStackedWidget>
#include "Base/EmptyOUT.h"
#include <array>
#include <vector>
#include <functional>

class QWidget;

class StepAlgorithmStack final : public QStackedWidget
{
public:
    explicit StepAlgorithmStack(QWidget* parent); // 创建随当前算法页面调整尺寸的容器

    QSize sizeHint() const override;              // 按当前算法页面计算建议尺寸

    QSize minimumSizeHint() const override;       // 按当前算法页面计算最小建议尺寸

    bool hasHeightForWidth() const override;      // 查询当前算法页面是否按宽度计算高度

    int heightForWidth(int width) const override; // 按指定宽度计算当前算法页面所需高度
};
