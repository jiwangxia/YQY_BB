#pragma once

#include "Utility/EnumKeyword.h"
#include "DataStructure/Load/LoadBase.h"
#include <QString>
#include <memory>

class AnalysisStep;
class Constraint;
class NonlinearMPCConstraint;

// 分析资源名称、分量和时程预览共用的显示规则。
namespace AnalysisDisplay
{
QString loadTypeName(EnumKeyword::LoadType type);
QString stepDisplayName(int id, const std::shared_ptr<AnalysisStep>& step);
QString directionName(EnumKeyword::Direction direction);
QString loadDisplayName(int id, const std::shared_ptr<LoadBase>& load);
int loadTargetId(const std::shared_ptr<LoadBase>& load);
double loadValue(const std::shared_ptr<LoadBase>& load);
QString numberText(double value);
QString timeFunctionName(TimeFunctionType type);
bool isContinuousLoad(const std::shared_ptr<LoadBase>& load);
QString loadTimeWindowText(const std::shared_ptr<LoadBase>& load);
QString loadVariableName(const std::shared_ptr<LoadBase>& load);
QString loadComponentText(const std::shared_ptr<LoadBase>& load);
QString loadValueText(const std::shared_ptr<LoadBase>& load);
QString loadPhysicalTypeName(const std::shared_ptr<LoadBase>& load);
QString timeFunctionExpression(const std::shared_ptr<LoadBase>& load);
QString loadExpression(const std::shared_ptr<LoadBase>& load);
double evaluateTimeFunction(TimeFunctionType type, double localTime, double amplitude, double frequency, double phase,
                            double offset, double rampT0, double rampT1, double decay, double period, double dutyCycle);
QString constraintDisplayName(int id, const std::shared_ptr<Constraint>& constraint);
QString mpcDisplayName(int id, const std::shared_ptr<NonlinearMPCConstraint>& mpc);
QString mpcDirectionText(const std::shared_ptr<NonlinearMPCConstraint>& mpc);
QString mpcRelationText(const std::shared_ptr<NonlinearMPCConstraint>& mpc);
}
