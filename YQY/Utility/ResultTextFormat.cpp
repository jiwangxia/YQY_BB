#include "Utility/ResultTextFormat.h"

namespace ResultTextFormat
{
QString FormatPlainResultValue(double value, int width, int decimals)
{
    QString text = QString::number(value, 'f', decimals);
    while (text.contains('.') && text.endsWith('0'))
        text.chop(1);
    if (text.endsWith('.'))
        text.chop(1);
    if (text == QStringLiteral("-0"))
        text = QStringLiteral("0");
    return text.rightJustified(width, ' ');
}
}
