#pragma once
#include <QDoubleSpinBox>
#include <QLocale>
#include <cmath>
// 使用紧凑小数文本且兼容科学计数法输入的数值框。
class CompactDoubleSpinBox final : public QDoubleSpinBox
{
public:
    explicit CompactDoubleSpinBox(QWidget* parent = nullptr)                 // 初始化高精度数值编辑行为
        : QDoubleSpinBox(parent)
    {
        setDecimals(15);
        setGroupSeparatorShown(false);
        setKeyboardTracking(false);
    }

protected:
    QString textFromValue(double value) const override                       // 去除末尾零，极小或极大值使用科学计数法
    {
        const double magnitude = std::abs(value);
        const bool scientific = magnitude > 0.0 && std::isfinite(value) && (magnitude < 1.0e-4 || magnitude >= 1.0e8);
        QString text = QString::number(value, scientific ? 'e' : 'f', scientific ? 14 : 15);
        QString exponent;
        if (scientific)
        {
            const int exponentStart = text.indexOf(QLatin1Char('e'));
            const int exponentValue = text.mid(exponentStart + 1).toInt();
            exponent = QStringLiteral("e%1%2")
                           .arg(exponentValue >= 0 ? QStringLiteral("+") : QString())
                           .arg(exponentValue);
            text.truncate(exponentStart);
        }
        while (text.contains(QLatin1Char('.')) && text.endsWith(QLatin1Char('0')))
            text.chop(1);
        if (text.endsWith(QLatin1Char('.')))
            text.chop(1);
        return (text == QStringLiteral("-0") ? QStringLiteral("0") : text) + exponent;
    }
    double valueFromText(const QString& text) const override                 // 解析本地或 C 区域数值文本
    {
        const QString numberText = NumberText(text);
        bool ok = false;
        double value = locale().toDouble(numberText, &ok);
        if (!ok)
            value = QLocale::c().toDouble(numberText, &ok);
        return ok ? value : minimum();
    }
    QValidator::State validate(QString& input, int& position) const override // 校验编辑中的数值文本
    {
        Q_UNUSED(position);
        const QString text = NumberText(input);
        if (text.isEmpty() || text == QStringLiteral("+") || text == QStringLiteral("-") ||
            text == QStringLiteral(".") || text == QStringLiteral("+.") || text == QStringLiteral("-.") ||
            text.endsWith(QLatin1Char('e'), Qt::CaseInsensitive) ||
            text.endsWith(QStringLiteral("e+"), Qt::CaseInsensitive) ||
            text.endsWith(QStringLiteral("e-"), Qt::CaseInsensitive))
            return QValidator::Intermediate;
        bool ok = false;
        double value = locale().toDouble(text, &ok);
        if (!ok)
            value = QLocale::c().toDouble(text, &ok);
        if (!ok)
            return QValidator::Invalid;
        return value >= minimum() && value <= maximum() ? QValidator::Acceptable : QValidator::Intermediate;
    }

private:
    QString NumberText(const QString& text) const                            // 去除前缀和后缀后的数值文本
    {
        QString numberText = text.trimmed();
        if (!prefix().isEmpty() && numberText.startsWith(prefix()))
            numberText.remove(0, prefix().size());
        if (!suffix().isEmpty() && numberText.endsWith(suffix()))
            numberText.chop(suffix().size());
        return numberText.trimmed();
    }
};
