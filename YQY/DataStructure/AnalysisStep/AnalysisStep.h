#pragma once
#include "Base/Base.h"
#include "Utility/EnumKeyword.h"

class AnalysisStep : public Base
{
public:
    int m_Id = 0;
    EnumKeyword::StepType m_Type = EnumKeyword::StepType::UNKNOWN;
    double m_Time = 0.0;           // 总时间
    double m_StepSize = 0.0;       // 每步大小
    double m_Tolerance = 1e-5;     // 容差
    int m_MaxIterations = 32;      // 最大迭代次数

    // 获取类型名称
    QString GetTypeName() const
    {
        return EnumKeyword::MapStepType.key(m_Type, "UNKNOWN");
    }
};
