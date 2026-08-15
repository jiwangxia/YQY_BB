#pragma once
#include "Base/Base.h"

#include <vector>
#include <QString>

class Node;

/**
 * @brief 位移约束时程中的一个时间点
 */
struct ConstraintTimePoint
{
    double time = 0.0;  ///< 当前分析步伪时间
    double value = 0.0; ///< 基础位移的无量纲缩放系数
};

/**
 * @brief 约束类 - 存储边界条件信息
 */
class Constraint : public Base
{
public:
    Constraint()
    {
    }

    QString m_Name;   ///< 用户可读名称；为空时界面回退为 Constraint-ID
    int m_StepId = 0; ///< 首次生效分析步；0 表示导入的全局/初始约束

    std::weak_ptr<Node> m_pNode;                                          ///< 约束所在节点
    EnumKeyword::Direction m_Direction = EnumKeyword::Direction::UNKNOWN; ///< 约束方向
    double m_Value = 0.0;                                                 ///< 约束位移值

    /**
     * @brief 设置约束自己的分段线性时程
     * @throws std::invalid_argument 数据点不足、非有限或时间不递增
     */
    void SetTimePoints(std::vector<ConstraintTimePoint> points);

    /**
     * @brief 判断该约束是否已经绑定时程
     */
    bool HasTimePoints() const;

    /**
     * @brief 计算当前时间的绝对约束位移
     * @param currentTime 当前分析步伪时间
     * @param factor 旧四字段约束使用的线性增量因子
     */
    double GetValue(double currentTime, double factor) const;

private:
    std::vector<ConstraintTimePoint> m_TimePoints;
};
