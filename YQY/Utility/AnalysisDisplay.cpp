#include "AnalysisDisplay.h"
#include "DataStructure/Structure/StructureData.h"
#include "DataStructure/Load/Force_Gravity.h"
#include "DataStructure/Load/Force_Wind.h"
#include <cmath>

namespace AnalysisDisplay
{
QString stepDisplayName(int id, const std::shared_ptr<AnalysisStep>& step)
{
    return step && !step->m_Name.trimmed().isEmpty() ? step->m_Name.trimmed() : QStringLiteral("Step-%1").arg(id);
}

QString directionName(EnumKeyword::Direction direction)
{
    return EnumKeyword::MapDirection.key(direction, QStringLiteral("--"));
}

QString loadTypeName(EnumKeyword::LoadType type)
{
    switch (type)
    {
        case EnumKeyword::LoadType::FORCE_NODE:
            return QStringLiteral("节点集中力");
        case EnumKeyword::LoadType::FORCE_ELEMENT:
            return QStringLiteral("单元荷载");
        case EnumKeyword::LoadType::FORCE_GRAVITY:
            return QStringLiteral("重力");
        case EnumKeyword::LoadType::FORCE_WIND:
            return QStringLiteral("风荷载");
        default:
            return QStringLiteral("未知");
    }
}

QString loadDisplayName(int id, const std::shared_ptr<LoadBase>& load)
{
    return load && !load->m_Name.trimmed().isEmpty() ? load->m_Name.trimmed() : QStringLiteral("Load-%1").arg(id);
}

int loadTargetId(const std::shared_ptr<LoadBase>& load)
{
    if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(load))
    {
        const auto node = nodeLoad->m_pNode.lock();
        return node ? node->m_Id : -1;
    }
    if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(load))
    {
        const auto element = elementLoad->m_pElement.lock();
        return element ? element->m_Id : -1;
    }
    return -1;
}

double loadValue(const std::shared_ptr<LoadBase>& load)
{
    if (const auto nodeLoad = std::dynamic_pointer_cast<Force_Node>(load))
        return nodeLoad->m_Value;
    if (const auto elementLoad = std::dynamic_pointer_cast<Force_Element>(load))
        return elementLoad->m_Value;
    if (const auto gravity = std::dynamic_pointer_cast<Force_Gravity>(load))
        return gravity->m_g;
    if (const auto wind = std::dynamic_pointer_cast<Force_Wind>(load))
        return wind->m_velocity;
    return 0.0;
}

QString numberText(double value)
{
    return QString::number(value, 'g', 10);
}

QString timeFunctionName(TimeFunctionType type)
{
    switch (type)
    {
        case TimeFunctionType::CONSTANT:
            return QStringLiteral("常量 CONSTANT");
        case TimeFunctionType::SIN:
            return QStringLiteral("正弦 SIN");
        case TimeFunctionType::COS:
            return QStringLiteral("余弦 COS");
        case TimeFunctionType::RAMP:
            return QStringLiteral("斜坡 RAMP");
        case TimeFunctionType::EXPONENTIAL:
            return QStringLiteral("指数 EXPONENTIAL");
        case TimeFunctionType::TRIANGULAR:
            return QStringLiteral("三角波 TRIANGULAR");
        case TimeFunctionType::SQUARE:
            return QStringLiteral("方波 SQUARE");
        default:
            return QStringLiteral("未知");
    }
}

bool isContinuousLoad(const std::shared_ptr<LoadBase>& load)
{
    return load && load->m_EndTime >= 1.0e9;
}

QString loadTimeWindowText(const std::shared_ptr<LoadBase>& load)
{
    if (!load)
        return QStringLiteral("--");
    if (isContinuousLoad(load))
        return QStringLiteral("t ≥ %1 s（持续）").arg(numberText(load->m_StartTime));
    return QStringLiteral("%1 ≤ t ≤ %2 s").arg(numberText(load->m_StartTime), numberText(load->m_EndTime));
}

QString loadVariableName(const std::shared_ptr<LoadBase>& load)
{
    if (!load)
        return QStringLiteral("P");
    if (load->m_LoadType == EnumKeyword::LoadType::FORCE_GRAVITY)
        return QStringLiteral("g");
    if (load->m_LoadType == EnumKeyword::LoadType::FORCE_WIND)
        return QStringLiteral("V");
    const int direction = static_cast<int>(load->m_Direction);
    const QString axis = direction == static_cast<int>(EnumKeyword::Direction::X)    ? QStringLiteral("x")
                         : direction == static_cast<int>(EnumKeyword::Direction::Y)  ? QStringLiteral("y")
                         : direction == static_cast<int>(EnumKeyword::Direction::Z)  ? QStringLiteral("z")
                         : direction == static_cast<int>(EnumKeyword::Direction::RX) ? QStringLiteral("x")
                         : direction == static_cast<int>(EnumKeyword::Direction::RY) ? QStringLiteral("y")
                                                                                     : QStringLiteral("z");
    if (load->m_LoadType == EnumKeyword::LoadType::FORCE_NODE &&
        direction >= static_cast<int>(EnumKeyword::Direction::RX))
        return QStringLiteral("M%1").arg(axis);
    return QStringLiteral("F%1").arg(axis);
}

QString loadComponentText(const std::shared_ptr<LoadBase>& load)
{
    if (!load)
        return QStringLiteral("--");
    if (load->m_LoadType == EnumKeyword::LoadType::FORCE_GRAVITY)
        return QStringLiteral("重力加速度");
    if (load->m_LoadType == EnumKeyword::LoadType::FORCE_WIND)
        return QStringLiteral("全局风向");
    const int direction = static_cast<int>(load->m_Direction);
    const QString axis = direction == static_cast<int>(EnumKeyword::Direction::X)    ? QStringLiteral("X")
                         : direction == static_cast<int>(EnumKeyword::Direction::Y)  ? QStringLiteral("Y")
                         : direction == static_cast<int>(EnumKeyword::Direction::Z)  ? QStringLiteral("Z")
                         : direction == static_cast<int>(EnumKeyword::Direction::RX) ? QStringLiteral("X")
                         : direction == static_cast<int>(EnumKeyword::Direction::RY) ? QStringLiteral("Y")
                                                                                     : QStringLiteral("Z");
    return direction >= static_cast<int>(EnumKeyword::Direction::RX) ? QStringLiteral("绕全局 %1 力矩").arg(axis)
                                                                     : QStringLiteral("全局 %1 力").arg(axis);
}

QString loadValueText(const std::shared_ptr<LoadBase>& load)
{
    if (!load)
        return QStringLiteral("--");
    if (load->m_LoadType == EnumKeyword::LoadType::FORCE_GRAVITY)
        return QStringLiteral("%1 m/s²").arg(numberText(loadValue(load)));
    if (load->m_LoadType == EnumKeyword::LoadType::FORCE_WIND)
        return QStringLiteral("%1 m/s").arg(numberText(loadValue(load)));
    const bool moment = static_cast<int>(load->m_Direction) >= static_cast<int>(EnumKeyword::Direction::RX);
    return QStringLiteral("%1 %2").arg(numberText(loadValue(load)),
                                       moment ? QStringLiteral("N·m") : QStringLiteral("N"));
}

QString loadPhysicalTypeName(const std::shared_ptr<LoadBase>& load)
{
    if (!load)
        return QStringLiteral("未知");
    if (load->m_LoadType == EnumKeyword::LoadType::FORCE_NODE &&
        static_cast<int>(load->m_Direction) >= static_cast<int>(EnumKeyword::Direction::RX))
    {
        return QStringLiteral("节点力矩");
    }
    return loadTypeName(load->m_LoadType);
}

QString timeFunctionExpression(const std::shared_ptr<LoadBase>& load)
{
    if (!load)
        return QStringLiteral("0");
    const QString localTime = QStringLiteral("(t − %1)").arg(numberText(load->m_StartTime));
    switch (load->m_FunctionType)
    {
        case TimeFunctionType::CONSTANT:
            return QStringLiteral("1");
        case TimeFunctionType::SIN:
        {
            const QString sine = QStringLiteral("sin(2π × %1 × %2 + %3)")
                                     .arg(numberText(load->m_Frequency), localTime, numberText(load->m_Phase));
            return std::abs(load->m_Amplitude - 1.0) <= 1.0e-12
                       ? sine
                       : QStringLiteral("%1 × %2").arg(numberText(load->m_Amplitude), sine);
        }
        case TimeFunctionType::COS:
        {
            const QString cosine = QStringLiteral("cos(2π × %1 × %2 + %3)")
                                       .arg(numberText(load->m_Frequency), localTime, numberText(load->m_Phase));
            return std::abs(load->m_Amplitude - 1.0) <= 1.0e-12
                       ? cosine
                       : QStringLiteral("%1 × %2").arg(numberText(load->m_Amplitude), cosine);
        }
        case TimeFunctionType::RAMP:
            return QStringLiteral("ramp(%1, %2→%3)")
                .arg(localTime, numberText(load->m_RampT0), numberText(load->m_RampT1));
        case TimeFunctionType::EXPONENTIAL:
            return QStringLiteral("exp(−%1 × %2)").arg(numberText(load->m_Decay), localTime);
        case TimeFunctionType::TRIANGULAR:
            return QStringLiteral("triangular(%1, T=%2)").arg(localTime, numberText(load->m_Period));
        case TimeFunctionType::SQUARE:
            return QStringLiteral("square(%1, T=%2, duty=%3)")
                .arg(localTime, numberText(load->m_Period), numberText(load->m_DutyCycle));
        default:
            return QStringLiteral("0");
    }
}

QString loadExpression(const std::shared_ptr<LoadBase>& load)
{
    if (!load)
        return QStringLiteral("--");
    const QString base = numberText(loadValue(load));
    const QString timeExpression = timeFunctionExpression(load);
    if (load->m_FunctionType == TimeFunctionType::CONSTANT)
        return QStringLiteral("%1(t) = %2").arg(loadVariableName(load), base);
    if ((load->m_FunctionType == TimeFunctionType::SIN || load->m_FunctionType == TimeFunctionType::COS) &&
        std::abs(load->m_Offset) > 1.0e-12)
    {
        return QStringLiteral("%1(t) = %2 × [%3 + %4]")
            .arg(loadVariableName(load), base, numberText(load->m_Offset), timeExpression);
    }
    return QStringLiteral("%1(t) = %2 × %3").arg(loadVariableName(load), base, timeExpression);
}

double evaluateTimeFunction(TimeFunctionType type, double localTime, double amplitude, double frequency, double phase,
                            double offset, double rampT0, double rampT1, double decay, double period, double dutyCycle)
{
    switch (type)
    {
        case TimeFunctionType::CONSTANT:
            return 1.0;
        case TimeFunctionType::SIN:
            return offset + amplitude * std::sin(2.0 * M_PI * frequency * localTime + phase);
        case TimeFunctionType::COS:
            return offset + amplitude * std::cos(2.0 * M_PI * frequency * localTime + phase);
        case TimeFunctionType::RAMP:
            return rampT1 <= rampT0      ? 0.0
                   : localTime <= rampT0 ? 0.0
                   : localTime >= rampT1 ? 1.0
                                         : (localTime - rampT0) / (rampT1 - rampT0);
        case TimeFunctionType::EXPONENTIAL:
            return std::exp(-decay * localTime);
        case TimeFunctionType::TRIANGULAR:
        {
            if (period <= 0.0)
                return 0.0;
            const double normalized = std::fmod(localTime, period) / period;
            return normalized < 0.5 ? 2.0 * normalized : 2.0 * (1.0 - normalized);
        }
        case TimeFunctionType::SQUARE:
        {
            if (period <= 0.0)
                return 0.0;
            return std::fmod(localTime, period) / period < dutyCycle ? 1.0 : 0.0;
        }
        default:
            return 0.0;
    }
}

QString constraintDisplayName(int id, const std::shared_ptr<Constraint>& constraint)
{
    return constraint && !constraint->m_Name.trimmed().isEmpty() ? constraint->m_Name.trimmed()
                                                                 : QStringLiteral("Constraint-%1").arg(id);
}

QString mpcDisplayName(int id, const std::shared_ptr<NonlinearMPCConstraint>& mpc)
{
    return mpc && !mpc->m_Name.trimmed().isEmpty() ? mpc->m_Name.trimmed() : QStringLiteral("MPC-%1").arg(id);
}

QString mpcDirectionText(const std::shared_ptr<NonlinearMPCConstraint>& mpc)
{
    QString result;
    if (mpc)
        for (const int direction : mpc->m_SlaveDirections)
            result.append(QString::number(direction));
    return result;
}

QString mpcRelationText(const std::shared_ptr<NonlinearMPCConstraint>& mpc)
{
    if (std::dynamic_pointer_cast<DistanceMPCConstraint>(mpc))
        return QStringLiteral("定长");
    if (std::dynamic_pointer_cast<RigidOffsetMPCConstraint>(mpc))
        return QStringLiteral("刚性偏置");
    if (std::dynamic_pointer_cast<PlanarShearReleaseMPCConstraint>(mpc))
        return QStringLiteral("非线性剪力释放");
    return QStringLiteral("非线性关系");
}

}
