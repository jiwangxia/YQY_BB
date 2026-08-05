#pragma once

#include <QString>

class PaperBeamDynamicsVerification final
{
public:
    static int RunExample1(const QString& outputDirectory);
    static int RunExample1Adaptive(const QString& outputDirectory);
    static int Run(const QString& outputDirectory);
    static int RunAdaptive(const QString& outputDirectory);

private:
    static int RunExample1(
        const QString& outputDirectory, bool useAdaptiveTssbn);
    static int Run(
        const QString& outputDirectory, bool useAdaptiveTssbn);
};
