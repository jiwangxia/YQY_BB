#pragma once
#include <QDoubleSpinBox>
#include <QLocale>
class CompactDoubleSpinBox final : public QDoubleSpinBox
{
public:
    explicit CompactDoubleSpinBox(QWidget* parent = nullptr) : QDoubleSpinBox(parent)
    {
        setDecimals(15);
        setGroupSeparatorShown(false);
        setKeyboardTracking(false);
    }

protected:
    QString textFromValue(double value) const override
    {
        QString text = QString::number(value, 'f', 15);
        while (text.contains(QLatin1Char('.')) && text.endsWith(QLatin1Char('0')))
            text.chop(1);
        if (text.endsWith(QLatin1Char('.')))
            text.chop(1);
        return text == QStringLiteral("-0") ? QStringLiteral("0") : text;
    }
    double valueFromText(const QString& text) const override
    {
        bool ok = false;
        double value = locale().toDouble(text.trimmed(), &ok);
        if (!ok)
            value = QLocale::c().toDouble(text.trimmed(), &ok);
        return ok ? value : minimum();
    }
    QValidator::State validate(QString& input, int& position) const override
    {
        Q_UNUSED(position);
        const QString text = input.trimmed();
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
};
