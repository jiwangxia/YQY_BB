#pragma once

#include <QtTypes>

#include <vector>

struct Hdf5ResultSummary
{
    bool hasModel = false;
    bool hasResult = false;
    qint64 frameCount = 0;
    qint64 displacementRecordCount = 0;
    qint64 stressRecordCount = 0;
    qint64 strainRecordCount = 0;
    bool partialResult = false;
};

struct Hdf5ResultFrameInfo
{
    int domainId = 0;
    int stepId = 0;
    int increment = 0;
    int analysis = 0;
    double time = 0.0;
    double loadFactor = 1.0;
};

struct Hdf5NodalResult
{
    int id = 0;
    double displacement[6] = {};
    double velocity[6] = {};
    double acceleration[6] = {};
    double currentCoordinate[3] = {};
};

struct Hdf5ElementResult
{
    int id = 0;
    double axialForce = 0.0;
    double shearY = 0.0;
    double shearZ = 0.0;
    double torque = 0.0;
    double momentY = 0.0;
    double momentZ = 0.0;
    double initStress = 0.0;
    double currentStress = 0.0;
    double deltaStress = 0.0;
    double strain = 0.0;
};

struct Hdf5ResultFrame
{
    Hdf5ResultFrameInfo info;
    std::vector<Hdf5NodalResult> nodes;
    std::vector<Hdf5ElementResult> elements;
};

struct Hdf5ResultRange
{
    double minimum = 0.0;
    double maximum = 0.0;
    bool valid = false;
};

struct Hdf5ResultRanges
{
    Hdf5ResultRange displacementMagnitude;
    Hdf5ResultRange displacementX;
    Hdf5ResultRange displacementY;
    Hdf5ResultRange displacementZ;
    Hdf5ResultRange axialForce;
    Hdf5ResultRange stress;
    Hdf5ResultRange strain;
};
