#include "CudaBeamBatchEvaluator.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <climits>

namespace SolverNameSpace
{
static __device__ void BeamCross(const double* lhs, const double* rhs, double* result)
{
    result[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1];
    result[1] = lhs[2] * rhs[0] - lhs[0] * rhs[2];
    result[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0];
}

static __device__ double BeamDot(const double* lhs, const double* rhs)
{
    return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
}

static __device__ bool BeamNormalize(double* value)
{
    const double norm = sqrt(BeamDot(value, value));
    if (norm <= 1.0e-14)
        return false;
    for (int component = 0; component < 3; ++component)
        value[component] /= norm;
    return true;
}

static __device__ void BeamSkew(const double* value, double* result)
{
    result[0] = 0.0;
    result[1] = -value[2];
    result[2] = value[1];
    result[3] = value[2];
    result[4] = 0.0;
    result[5] = -value[0];
    result[6] = -value[1];
    result[7] = value[0];
    result[8] = 0.0;
}

static __device__ void BeamMatrix3Transpose(const double* matrix, double* result)
{
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            result[row * 3 + column] = matrix[column * 3 + row];
}

static __device__ void BeamMatrix3Multiply(const double* lhs, const double* rhs, double* result)
{
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            double value = 0.0;
            for (int inner = 0; inner < 3; ++inner)
                value += lhs[row * 3 + inner] * rhs[inner * 3 + column];
            result[row * 3 + column] = value;
        }
    }
}

static __device__ void BeamMatrix3Vector(const double* matrix, const double* vector, double* result)
{
    for (int row = 0; row < 3; ++row)
        result[row] = matrix[row * 3] * vector[0] + matrix[row * 3 + 1] * vector[1] + matrix[row * 3 + 2] * vector[2];
}

static __device__ void BeamRotationMatrix(const double* rotationVector, double* result)
{
    double skew[9]{};
    double skewSquared[9]{};
    BeamSkew(rotationVector, skew);
    BeamMatrix3Multiply(skew, skew, skewSquared);
    const double angleSquared = BeamDot(rotationVector, rotationVector);
    const double angle = sqrt(angleSquared);
    double first = 0.0;
    double second = 0.0;
    if (angle < 1.0e-8)
    {
        const double fourth = angleSquared * angleSquared;
        first = 1.0 - angleSquared / 6.0 + fourth / 120.0;
        second = 0.5 - angleSquared / 24.0 + fourth / 720.0;
    }
    else
    {
        first = sin(angle) / angle;
        second = (1.0 - cos(angle)) / angleSquared;
    }
    for (int index = 0; index < 9; ++index)
        result[index] = (index % 4 == 0 ? 1.0 : 0.0) + first * skew[index] + second * skewSquared[index];
}

static __device__ void BeamSpatialSpin(const double* rotationVector, double* result)
{
    const double angleSquared = BeamDot(rotationVector, rotationVector);
    const double angle = sqrt(angleSquared);
    double first;
    double second;
    if (angle < 1.0e-4)
    {
        const double fourth = angleSquared * angleSquared;
        first = 0.5 - angleSquared / 24.0 + fourth / 720.0;
        second = 1.0 / 6.0 - angleSquared / 120.0 + fourth / 5040.0;
    }
    else
    {
        const double halfSine = sin(0.5 * angle);
        first = 2.0 * halfSine * halfSine / angleSquared;
        second = (angle - sin(angle)) / (angleSquared * angle);
    }
    double skew[9]{};
    double skewSquared[9]{};
    BeamSkew(rotationVector, skew);
    BeamMatrix3Multiply(skew, skew, skewSquared);
    for (int index = 0; index < 9; ++index)
        result[index] = (index % 4 == 0 ? 1.0 : 0.0) + first * skew[index] + second * skewSquared[index];
}

static __device__ void BeamInverseSpatialSpin(const double* rotationVector, double* result)
{
    const double angleSquared = BeamDot(rotationVector, rotationVector);
    const double angle = sqrt(angleSquared);
    double axis[3] = {1.0, 0.0, 0.0};
    double coefficient = 0.0;
    if (angle < 1.0e-8)
    {
        const double fourth = angleSquared * angleSquared;
        coefficient = 1.0 - angleSquared / 12.0 - fourth / 720.0;
    }
    else
    {
        for (int component = 0; component < 3; ++component)
            axis[component] = rotationVector[component] / angle;
        coefficient = 0.5 * angle / tan(0.5 * angle);
    }
    double skew[9]{};
    BeamSkew(rotationVector, skew);
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            const int index = row * 3 + column;
            result[index] = coefficient * (row == column ? 1.0 : 0.0) - 0.5 * skew[index] +
                            (1.0 - coefficient) * axis[row] * axis[column];
        }
    }
}

static __device__ void BeamRotationVector(const double* rotation, double* result)
{
    double quaternion[4]{};
    const double trace = rotation[0] + rotation[4] + rotation[8];
    if (trace > 0.0)
    {
        const double scale = 2.0 * sqrt(fmax(0.0, trace + 1.0));
        quaternion[0] = 0.25 * scale;
        quaternion[1] = (rotation[7] - rotation[5]) / scale;
        quaternion[2] = (rotation[2] - rotation[6]) / scale;
        quaternion[3] = (rotation[3] - rotation[1]) / scale;
    }
    else if (rotation[0] > rotation[4] && rotation[0] > rotation[8])
    {
        const double scale = 2.0 * sqrt(fmax(0.0, 1.0 + rotation[0] - rotation[4] - rotation[8]));
        quaternion[0] = (rotation[7] - rotation[5]) / scale;
        quaternion[1] = 0.25 * scale;
        quaternion[2] = (rotation[1] + rotation[3]) / scale;
        quaternion[3] = (rotation[2] + rotation[6]) / scale;
    }
    else if (rotation[4] > rotation[8])
    {
        const double scale = 2.0 * sqrt(fmax(0.0, 1.0 + rotation[4] - rotation[0] - rotation[8]));
        quaternion[0] = (rotation[2] - rotation[6]) / scale;
        quaternion[1] = (rotation[1] + rotation[3]) / scale;
        quaternion[2] = 0.25 * scale;
        quaternion[3] = (rotation[5] + rotation[7]) / scale;
    }
    else
    {
        const double scale = 2.0 * sqrt(fmax(0.0, 1.0 + rotation[8] - rotation[0] - rotation[4]));
        quaternion[0] = (rotation[3] - rotation[1]) / scale;
        quaternion[1] = (rotation[2] + rotation[6]) / scale;
        quaternion[2] = (rotation[5] + rotation[7]) / scale;
        quaternion[3] = 0.25 * scale;
    }
    double quaternionNorm = 0.0;
    for (double component : quaternion)
        quaternionNorm += component * component;
    quaternionNorm = sqrt(quaternionNorm);
    if (quaternionNorm <= 1.0e-14)
    {
        result[0] = 0.0;
        result[1] = 0.0;
        result[2] = 0.0;
        return;
    }
    for (double& component : quaternion)
        component /= quaternionNorm;
    if (quaternion[0] < 0.0)
        for (double& component : quaternion)
            component = -component;
    const double vectorNorm =
        sqrt(quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    if (vectorNorm < 1.0e-12)
    {
        for (int component = 0; component < 3; ++component)
            result[component] = 2.0 * quaternion[component + 1];
        return;
    }
    const double angle = 2.0 * atan2(vectorNorm, quaternion[0]);
    for (int component = 0; component < 3; ++component)
        result[component] = angle * quaternion[component + 1] / vectorNorm;
}

static __device__ void BeamCalculateKhi(const double* rotation, const double* moment, double* result)
{
    const double angleSquared = BeamDot(rotation, rotation);
    const double angle = sqrt(angleSquared);
    const double fourth = angleSquared * angleSquared;
    double first = 0.0;
    double second = 0.0;
    double third = 0.0;
    if (angle < 1.0e-8)
    {
        first = -1.0 / 6.0 - angleSquared / 180.0 - fourth / 5040.0;
        second = 1.0 / 12.0 + angleSquared / 720.0 + fourth / 30240.0;
        third = 1.0 / 360.0 + angleSquared / 7560.0 + fourth / 201600.0;
    }
    else
    {
        const double sine = sin(angle);
        const double cosine = cos(angle);
        const double halfSine = sin(0.5 * angle);
        const double halfSineSquared = halfSine * halfSine;
        first = (sine - angle) / (4.0 * angle * halfSineSquared);
        second = (2.0 * sine - angle * (1.0 + cosine)) / (2.0 * angleSquared * sine);
        third = (angle * (angle + sine) - 8.0 * halfSineSquared) / (4.0 * fourth * halfSineSquared);
    }
    double skewMoment[9]{};
    BeamSkew(moment, skewMoment);
    const double rotationMoment = BeamDot(rotation, moment);
    double localDerivative[9]{};
    for (int row = 0; row < 3; ++row)
    {
        for (int column = 0; column < 3; ++column)
        {
            const int index = row * 3 + column;
            localDerivative[index] =
                first * moment[row] * rotation[column] + third * rotationMoment * rotation[row] * rotation[column] +
                second * ((row == column ? rotationMoment : 0.0) + rotation[row] * moment[column]) -
                0.5 * skewMoment[index];
        }
    }
    // 局部旋转向量的导数转换为空间左乘转动列，与 CPU Calculate_Khi 一致。
    double spinInverse[9]{};
    BeamInverseSpatialSpin(rotation, spinInverse);
    BeamMatrix3Multiply(localDerivative, spinInverse, result);
}

static __device__ bool BeamBuildFrame(const double* firstPosition, const double* secondPosition, const double* q,
                                      double* frame, double* axis, double& length)
{
    for (int component = 0; component < 3; ++component)
        axis[component] = secondPosition[component] - firstPosition[component];
    length = sqrt(BeamDot(axis, axis));
    if (length <= 1.0e-12)
        return false;
    for (int component = 0; component < 3; ++component)
        axis[component] /= length;
    double third[3]{};
    BeamCross(axis, q, third);
    if (!BeamNormalize(third))
    {
        const double arbitrary[3] = {fabs(axis[0]) < 0.9 ? 1.0 : 0.0, fabs(axis[0]) < 0.9 ? 0.0 : 1.0, 0.0};
        BeamCross(axis, arbitrary, third);
        if (!BeamNormalize(third))
            return false;
    }
    double second[3]{};
    BeamCross(third, axis, second);
    for (int row = 0; row < 3; ++row)
    {
        frame[row * 3] = axis[row];
        frame[row * 3 + 1] = second[row];
        frame[row * 3 + 2] = third[row];
    }
    return true;
}

template <bool ComputeTangent>
static __device__ void BeamAddDynamicContributions(const GpuBeamElementInput& element, double accelerationDerivative,
                                                   double velocityDerivative, double* effectiveTangent,
                                                   double* inertiaForce)
{
    double previousSection[2][9]{};
    BeamMatrix3Multiply(element.previousRotation, element.initialRotation, previousSection[0]);
    BeamMatrix3Multiply(element.previousRotation + 9, element.initialRotation, previousSection[1]);
    double firstTranspose[9]{};
    double relativeMatrix[9]{};
    double previousRelativeRotation[3]{};
    BeamMatrix3Transpose(previousSection[0], firstTranspose);
    BeamMatrix3Multiply(previousSection[1], firstTranspose, relativeMatrix);
    BeamRotationVector(relativeMatrix, previousRelativeRotation);

    double nodalInverseSpin[2][9]{};
    double nodalRotationMapping[2][9]{};
    double nodalOmegaSkew[2][9]{};
    double nodalAlphaSkew[2][9]{};
    if constexpr (ComputeTangent)
        for (int node = 0; node < 2; ++node)
        {
            double previousTranspose[9]{};
            double spatialIncrement[9]{};
            BeamInverseSpatialSpin(element.stepRotation + 3 * node, nodalInverseSpin[node]);
            BeamMatrix3Transpose(element.previousRotation + 9 * node, previousTranspose);
            BeamMatrix3Multiply(element.currentRotation + 9 * node, previousTranspose, spatialIncrement);
            BeamMatrix3Multiply(spatialIncrement, nodalInverseSpin[node], nodalRotationMapping[node]);
            BeamSkew(element.velocity + 6 * node + 3, nodalOmegaSkew[node]);
            BeamSkew(element.acceleration + 6 * node + 3, nodalAlphaSkew[node]);
        }

    const double lineMass = element.density * element.area;
    const double gauss = 1.0 / sqrt(3.0);
    const double coordinates[2] = {-gauss, gauss};
    for (int gaussIndex = 0; gaussIndex < 2; ++gaussIndex)
    {
        const double coordinate = coordinates[gaussIndex];
        const double shape[2] = {0.5 * (1.0 - coordinate), 0.5 * (1.0 + coordinate)};
        double relativeIncrement[3]{};
        double relativeIncrementMatrix[9]{};
        for (int component = 0; component < 3; ++component)
            relativeIncrement[component] = shape[1] * previousRelativeRotation[component];
        BeamRotationMatrix(relativeIncrement, relativeIncrementMatrix);
        double previousGaussRotation[9]{};
        if (element.hasGaussRotationHistory)
            for (int entry = 0; entry < 9; ++entry)
                previousGaussRotation[entry] = element.previousGaussRotation[9 * gaussIndex + entry];
        else
            BeamMatrix3Multiply(relativeIncrementMatrix, previousSection[0], previousGaussRotation);

        double stepRotation[3]{};
        for (int component = 0; component < 3; ++component)
            stepRotation[component] =
                shape[0] * element.stepRotation[component] + shape[1] * element.stepRotation[component + 3];
        double stepRotationMatrix[9]{};
        double currentGaussRotation[9]{};
        BeamRotationMatrix(stepRotation, stepRotationMatrix);
        BeamMatrix3Multiply(stepRotationMatrix, previousGaussRotation, currentGaussRotation);

        double translationalAcceleration[3]{};
        double spatialAngularVelocity[3]{};
        double spatialAngularAcceleration[3]{};
        for (int component = 0; component < 3; ++component)
        {
            translationalAcceleration[component] =
                shape[0] * element.acceleration[component] + shape[1] * element.acceleration[component + 6];
            spatialAngularVelocity[component] =
                shape[0] * element.velocity[component + 3] + shape[1] * element.velocity[component + 9];
            spatialAngularAcceleration[component] =
                shape[0] * element.acceleration[component + 3] + shape[1] * element.acceleration[component + 9];
        }
        double currentTranspose[9]{};
        double materialAngularVelocity[3]{};
        double materialAngularAcceleration[3]{};
        BeamMatrix3Transpose(currentGaussRotation, currentTranspose);
        BeamMatrix3Vector(currentTranspose, spatialAngularVelocity, materialAngularVelocity);
        BeamMatrix3Vector(currentTranspose, spatialAngularAcceleration, materialAngularAcceleration);

        double materialMomentum[3]{};
        double materialDynamicMoment[3]{};
        for (int component = 0; component < 3; ++component)
        {
            materialMomentum[component] = element.rotaryInertia[component] * materialAngularVelocity[component];
            materialDynamicMoment[component] =
                element.rotaryInertia[component] * materialAngularAcceleration[component];
        }
        double gyroscopicMoment[3]{};
        BeamCross(materialAngularVelocity, materialMomentum, gyroscopicMoment);
        for (int component = 0; component < 3; ++component)
            materialDynamicMoment[component] += gyroscopicMoment[component];
        double spatialDynamicMoment[3]{};
        BeamMatrix3Vector(currentGaussRotation, materialDynamicMoment, spatialDynamicMoment);

        double rotationalMass[2][9]{};
        double rotationalGyroscopic[2][9]{};
        double rotationalCentrifugal[2][9]{};
        if constexpr (ComputeTangent)
        {
            // 与 CPU 共用节点空间量插值的完整链式导数。
            double inertiaTensor[9]{};
            for (int component = 0; component < 3; ++component)
                inertiaTensor[component * 3 + component] = element.rotaryInertia[component];
            double inertiaTail[9]{};
            double spatialInertia[9]{};
            BeamMatrix3Multiply(inertiaTensor, currentTranspose, inertiaTail);
            BeamMatrix3Multiply(currentGaussRotation, inertiaTail, spatialInertia);
            double spatialMomentum[3]{};
            double spatialInertiaAlpha[3]{};
            BeamMatrix3Vector(spatialInertia, spatialAngularVelocity, spatialMomentum);
            BeamMatrix3Vector(spatialInertia, spatialAngularAcceleration, spatialInertiaAlpha);
            double omegaSkew[9]{};
            double alphaSkew[9]{};
            double momentumSkew[9]{};
            double inertiaAlphaSkew[9]{};
            BeamSkew(spatialAngularVelocity, omegaSkew);
            BeamSkew(spatialAngularAcceleration, alphaSkew);
            BeamSkew(spatialMomentum, momentumSkew);
            BeamSkew(spatialInertiaAlpha, inertiaAlphaSkew);
            double leftProduct[9]{};
            double rightProduct[9]{};
            double spatialVelocityDerivative[9]{};
            BeamMatrix3Multiply(omegaSkew, spatialInertia, leftProduct);
            for (int index = 0; index < 9; ++index)
                spatialVelocityDerivative[index] = leftProduct[index] - momentumSkew[index];
            BeamMatrix3Multiply(spatialInertia, omegaSkew, rightProduct);
            double configurationTail[9]{};
            for (int index = 0; index < 9; ++index)
                configurationTail[index] = rightProduct[index] - momentumSkew[index];
            BeamMatrix3Multiply(omegaSkew, configurationTail, leftProduct);
            BeamMatrix3Multiply(spatialInertia, alphaSkew, rightProduct);
            double spatialConfigurationDerivative[9]{};
            for (int index = 0; index < 9; ++index)
                spatialConfigurationDerivative[index] =
                    rightProduct[index] - inertiaAlphaSkew[index] + leftProduct[index];
            double gaussSpin[9]{};
            BeamSpatialSpin(stepRotation, gaussSpin);
            for (int node = 0; node < 2; ++node)
            {
                BeamMatrix3Multiply(spatialInertia, nodalRotationMapping[node], rotationalMass[node]);
                BeamMatrix3Multiply(spatialVelocityDerivative, nodalRotationMapping[node], rotationalGyroscopic[node]);
                double rotationMapping[9]{};
                double configurationMapped[9]{};
                double inertiaAlpha[9]{};
                double velocityOmega[9]{};
                BeamMatrix3Multiply(gaussSpin, nodalInverseSpin[node], rotationMapping);
                BeamMatrix3Multiply(spatialConfigurationDerivative, rotationMapping, configurationMapped);
                BeamMatrix3Multiply(spatialInertia, nodalAlphaSkew[node], inertiaAlpha);
                BeamMatrix3Multiply(spatialVelocityDerivative, nodalOmegaSkew[node], velocityOmega);
                for (int index = 0; index < 9; ++index)
                    rotationalCentrifugal[node][index] =
                        configurationMapped[index] - inertiaAlpha[index] - velocityOmega[index];
            }
        }

        const double integrationScale = 0.5 * element.initialLength;
        for (int nodeA = 0; nodeA < 2; ++nodeA)
        {
            for (int component = 0; component < 3; ++component)
            {
                inertiaForce[nodeA * 6 + component] +=
                    integrationScale * shape[nodeA] * lineMass * translationalAcceleration[component];
                inertiaForce[nodeA * 6 + component + 3] +=
                    integrationScale * shape[nodeA] * spatialDynamicMoment[component];
            }
            if constexpr (ComputeTangent)
                for (int nodeB = 0; nodeB < 2; ++nodeB)
                {
                    const double weight = integrationScale * shape[nodeA] * shape[nodeB];
                    for (int row = 0; row < 3; ++row)
                    {
                        const int translationalRow = nodeA * 6 + row;
                        const int rotationalRow = translationalRow + 3;
                        for (int column = 0; column < 3; ++column)
                        {
                            const int translationalColumn = nodeB * 6 + column;
                            const int rotationalColumn = translationalColumn + 3;
                            if (row == column)
                            {
                                effectiveTangent[translationalRow * 12 + translationalColumn] +=
                                    accelerationDerivative * weight * lineMass;
                            }
                            effectiveTangent[rotationalRow * 12 + rotationalColumn] +=
                                weight * (accelerationDerivative * rotationalMass[nodeB][row * 3 + column] +
                                          velocityDerivative * rotationalGyroscopic[nodeB][row * 3 + column] +
                                          rotationalCentrifugal[nodeB][row * 3 + column]);
                        }
                    }
                }
        }
    }
}

template <bool ComputeTangent>
static __device__ bool BeamEvaluateStatic(const GpuBeamElementInput& element, double* effectiveTangent,
                                          double* internalForce, double& stress, double& currentLength)
{
    double deformedPosition[2][3]{};
    double squaredLengthChange = 0.0;
    for (int component = 0; component < 3; ++component)
    {
        const double initialChord = element.initialPosition[component + 3] - element.initialPosition[component];
        const double relativeDisplacement = element.displacement[component + 3] - element.displacement[component];
        deformedPosition[1][component] = initialChord + relativeDisplacement;
        squaredLengthChange =
            fma(relativeDisplacement, 2.0 * initialChord + relativeDisplacement, squaredLengthChange);
    }
    double sectionRotation[2][9]{};
    BeamMatrix3Multiply(element.currentRotation, element.initialRotation, sectionRotation[0]);
    BeamMatrix3Multiply(element.currentRotation + 9, element.initialRotation, sectionRotation[1]);
    double q[3]{};
    for (int component = 0; component < 3; ++component)
        q[component] = 0.5 * (sectionRotation[0][component * 3 + 1] + sectionRotation[1][component * 3 + 1]);

    double frame[9]{};
    double axis[3]{};
    if (!BeamBuildFrame(deformedPosition[0], deformedPosition[1], q, frame, axis, currentLength))
        return false;
    double frameTranspose[9]{};
    BeamMatrix3Transpose(frame, frameTranspose);
    double localRotationMatrix[2][9]{};
    double localRotation[2][3]{};
    BeamMatrix3Multiply(frameTranspose, sectionRotation[0], localRotationMatrix[0]);
    BeamMatrix3Multiply(frameTranspose, sectionRotation[1], localRotationMatrix[1]);
    BeamRotationVector(localRotationMatrix[0], localRotation[0]);
    BeamRotationVector(localRotationMatrix[1], localRotation[1]);

    const double shear = element.young / (2.0 * (1.0 + element.poisson));
    const double axial = element.young * element.area / element.initialLength;
    const double torsion = shear * element.torsionConstant / element.initialLength;
    const double bendingY = element.young * element.inertiaY / element.initialLength;
    const double bendingZ = element.young * element.inertiaZ / element.initialLength;
    double localStiffness[49]{};
    if constexpr (ComputeTangent)
    {
        localStiffness[0] = axial;
        localStiffness[8] = torsion;
        localStiffness[11] = -torsion;
        localStiffness[29] = -torsion;
        localStiffness[32] = torsion;
        localStiffness[16] = 4.0 * bendingY;
        localStiffness[19] = 2.0 * bendingY;
        localStiffness[37] = 2.0 * bendingY;
        localStiffness[40] = 4.0 * bendingY;
        localStiffness[24] = 4.0 * bendingZ;
        localStiffness[27] = 2.0 * bendingZ;
        localStiffness[45] = 2.0 * bendingZ;
        localStiffness[48] = 4.0 * bendingZ;
    }

    double localForce[7]{};
    const double axialExtension = squaredLengthChange / (currentLength + element.initialLength);
    localForce[0] = axial * axialExtension + element.initialStress * element.area;
    localForce[1] = torsion * (localRotation[0][0] - localRotation[1][0]);
    localForce[4] = -localForce[1];
    localForce[2] = 2.0 * bendingY * (2.0 * localRotation[0][1] + localRotation[1][1]);
    localForce[5] = 2.0 * bendingY * (localRotation[0][1] + 2.0 * localRotation[1][1]);
    localForce[3] = 2.0 * bendingZ * (2.0 * localRotation[0][2] + localRotation[1][2]);
    localForce[6] = 2.0 * bendingZ * (localRotation[0][2] + 2.0 * localRotation[1][2]);
    stress = localForce[0] / element.area;

    double localMap[49]{};
    for (int index = 0; index < 7; ++index)
        localMap[index * 7 + index] = 1.0;
    double firstSpin[9]{};
    BeamInverseSpatialSpin(localRotation[0], firstSpin);
    double secondSpin[9]{};
    BeamInverseSpatialSpin(localRotation[1], secondSpin);
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
        {
            localMap[(row + 1) * 7 + column + 1] = firstSpin[row * 3 + column];
            localMap[(row + 4) * 7 + column + 4] = secondSpin[row * 3 + column];
        }

    double projectedForce[7]{};
    for (int row = 0; row < 7; ++row)
        for (int inner = 0; inner < 7; ++inner)
            projectedForce[row] += localMap[inner * 7 + row] * localForce[inner];
    double projectedStiffness[49]{};
    if constexpr (ComputeTangent)
    {
        double localGeometric[49]{};
        double firstKhi[9]{};
        double secondKhi[9]{};
        BeamCalculateKhi(localRotation[0], localForce + 1, firstKhi);
        BeamCalculateKhi(localRotation[1], localForce + 4, secondKhi);
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                localGeometric[(row + 1) * 7 + column + 1] = firstKhi[row * 3 + column];
                localGeometric[(row + 4) * 7 + column + 4] = secondKhi[row * 3 + column];
            }
        }
        double stiffnessTimesMap[49]{};
        for (int row = 0; row < 7; ++row)
        {
            for (int column = 0; column < 7; ++column)
            {
                for (int inner = 0; inner < 7; ++inner)
                    stiffnessTimesMap[row * 7 + column] +=
                        localStiffness[row * 7 + inner] * localMap[inner * 7 + column];
            }
        }
        // 先完成 K·Ba 的所有行，再计算 Baᵀ·K·Ba，避免读取尚未组装的行。
        for (int row = 0; row < 7; ++row)
        {
            for (int column = 0; column < 7; ++column)
            {
                for (int inner = 0; inner < 7; ++inner)
                    projectedStiffness[row * 7 + column] +=
                        localMap[inner * 7 + row] * stiffnessTimesMap[inner * 7 + column];
                projectedStiffness[row * 7 + column] += localGeometric[row * 7 + column];
            }
        }
    }

    double localFirstQ[3]{};
    double localSecondQ[3]{};
    for (int row = 0; row < 3; ++row)
    {
        for (int inner = 0; inner < 3; ++inner)
        {
            localFirstQ[row] += frameTranspose[row * 3 + inner] * sectionRotation[0][inner * 3 + 1];
            localSecondQ[row] += frameTranspose[row * 3 + inner] * sectionRotation[1][inner * 3 + 1];
        }
    }
    double averageQ[3] = {0.5 * (localFirstQ[0] + localSecondQ[0]), 0.5 * (localFirstQ[1] + localSecondQ[1]),
                          0.5 * (localFirstQ[2] + localSecondQ[2])};
    double qy = averageQ[1];
    if (fabs(qy) < 1.0e-12)
        qy = qy < 0.0 ? -1.0e-12 : 1.0e-12;
    const double eta = averageQ[0] / (currentLength * qy);
    const double firstNu[2] = {localFirstQ[1] / (2.0 * qy), -localFirstQ[0] / (2.0 * qy)};
    const double secondNu[2] = {localSecondQ[1] / (2.0 * qy), -localSecondQ[0] / (2.0 * qy)};
    double spinMap[36]{};
    spinMap[2] = eta;
    spinMap[14] = 1.0 / currentLength;
    spinMap[25] = -1.0 / currentLength;
    spinMap[8] = -eta;
    spinMap[20] = -1.0 / currentLength;
    spinMap[31] = 1.0 / currentLength;
    spinMap[3] = firstNu[0];
    spinMap[4] = firstNu[1];
    spinMap[9] = secondNu[0];
    spinMap[10] = secondNu[1];
    double projection[72]{};
    for (int component = 0; component < 3; ++component)
    {
        projection[component * 12 + component + 3] = 1.0;
        projection[(component + 3) * 12 + component + 9] = 1.0;
    }
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 12; ++column)
        {
            projection[row * 12 + column] -= spinMap[row * 12 + column];
            projection[(row + 3) * 12 + column] -= spinMap[row * 12 + column];
        }

    double transformationTranspose[144]{};
    for (int block = 0; block < 4; ++block)
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                transformationTranspose[(block * 3 + row) * 12 + block * 3 + column] = frameTranspose[row * 3 + column];
    double globalMap[84]{};
    for (int component = 0; component < 3; ++component)
    {
        globalMap[component] = -axis[component];
        globalMap[component + 6] = axis[component];
    }
    for (int row = 0; row < 6; ++row)
        for (int column = 0; column < 12; ++column)
            for (int inner = 0; inner < 12; ++inner)
                globalMap[(row + 1) * 12 + column] +=
                    projection[row * 12 + inner] * transformationTranspose[inner * 12 + column];

    for (int row = 0; row < 12; ++row)
    {
        for (int inner = 0; inner < 7; ++inner)
            internalForce[row] += globalMap[inner * 12 + row] * projectedForce[inner];
        if constexpr (ComputeTangent)
            for (int column = 0; column < 12; ++column)
            {
                double value = 0.0;
                for (int left = 0; left < 7; ++left)
                    for (int right = 0; right < 7; ++right)
                        value += globalMap[left * 12 + row] * projectedStiffness[left * 7 + right] *
                                 globalMap[right * 12 + column];
                effectiveTangent[row * 12 + column] = value;
            }
    }

    if constexpr (!ComputeTangent)
        return true;

    const double axialForce = projectedForce[0];
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
        {
            const double value = axialForce / currentLength * ((row == column ? 1.0 : 0.0) - axis[row] * axis[column]);
            effectiveTangent[row * 12 + column] += value;
            effectiveTangent[row * 12 + column + 6] -= value;
            effectiveTangent[(row + 6) * 12 + column] -= value;
            effectiveTangent[(row + 6) * 12 + column + 6] += value;
        }

    double projectedMoment[6] = {projectedForce[1], projectedForce[2], projectedForce[3],
                                 projectedForce[4], projectedForce[5], projectedForce[6]};
    double projectedNodalForce[12]{};
    for (int column = 0; column < 12; ++column)
        for (int inner = 0; inner < 6; ++inner)
            projectedNodalForce[column] += projection[inner * 12 + column] * projectedMoment[inner];
    double localQMatrix[36]{};
    for (int block = 0; block < 4; ++block)
    {
        double skew[9]{};
        BeamSkew(projectedNodalForce + block * 3, skew);
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                localQMatrix[(block * 3 + row) * 3 + column] = skew[row * 3 + column];
    }
    double transformedQ[36]{};
    for (int block = 0; block < 4; ++block)
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                for (int inner = 0; inner < 3; ++inner)
                    transformedQ[(block * 3 + row) * 3 + column] +=
                        frame[row * 3 + inner] * localQMatrix[(block * 3 + inner) * 3 + column];
    double transformedQSpin[144]{};
    for (int row = 0; row < 12; ++row)
        for (int column = 0; column < 12; ++column)
            for (int inner = 0; inner < 3; ++inner)
                transformedQSpin[row * 12 + column] += transformedQ[row * 3 + inner] * spinMap[inner * 12 + column];
    for (int row = 0; row < 12; ++row)
        for (int column = 0; column < 12; ++column)
        {
            double value = 0.0;
            const int columnBlock = column / 3;
            for (int inner = 0; inner < 3; ++inner)
                value += transformedQSpin[row * 12 + columnBlock * 3 + inner] * frameTranspose[inner * 3 + column % 3];
            effectiveTangent[row * 12 + column] -= value;
        }

    const double etaWithoutLength = averageQ[0] / qy;
    double localGa[12]{};
    localGa[1] = -(projectedForce[3] + projectedForce[6]) / (currentLength * currentLength);
    localGa[2] = ((projectedForce[1] + projectedForce[4]) * etaWithoutLength + projectedForce[2] + projectedForce[5]) /
                 (currentLength * currentLength);
    localGa[7] = -localGa[1];
    localGa[8] = -localGa[2];
    double globalGa[12]{};
    for (int block = 0; block < 4; ++block)
        BeamMatrix3Vector(frame, localGa + block * 3, globalGa + block * 3);
    double chordGradient[12] = {-axis[0], -axis[1], -axis[2], 0.0, 0.0, 0.0, axis[0], axis[1], axis[2], 0.0, 0.0, 0.0};
    for (int row = 0; row < 12; ++row)
        for (int column = 0; column < 12; ++column)
            effectiveTangent[row * 12 + column] += globalGa[row] * chordGradient[column];
    // G 首行的截面方向导数；原长度项不足以覆盖非共轴节点转动。
    const double torsionalMomentSum = projectedForce[1] + projectedForce[4];
    if (torsionalMomentSum != 0.0 && fabs(averageQ[1]) >= 1.0e-12)
    {
        const double t = etaWithoutLength;
        const double h = (localFirstQ[0] - localSecondQ[0]) / (2.0 * qy);
        const double c = (localFirstQ[2] - localSecondQ[2]) / 2.0;
        double vt[12]{};
        double vh[12]{};
        double gt[12]{};
        double gh[12]{};
        vt[2] = 1.0 / currentLength;
        vt[8] = -vt[2];
        vt[3] = -h / 2.0;
        vt[4] = -0.5;
        vt[9] = h / 2.0;
        vt[10] = -0.5;
        vh[3] = -t / 2.0;
        vh[4] = -0.5;
        vh[9] = t / 2.0;
        vh[10] = 0.5;
        gt[1] = -(1.0 + t * t) / currentLength;
        gt[7] = -gt[1];
        gt[3] = c * t / (2.0 * qy);
        gt[4] = c / (2.0 * qy);
        gt[5] = -(1.0 + t * t) / 2.0;
        gt[9] = -gt[3];
        gt[10] = -gt[4];
        gt[11] = gt[5];
        gh[2] = -c / (qy * currentLength);
        gh[8] = -gh[2];
        gh[3] = c * h / (2.0 * qy);
        gh[4] = c / (2.0 * qy);
        gh[5] = -(1.0 + h * h) / 2.0;
        gh[9] = -gh[3];
        gh[10] = gh[4];
        gh[11] = -gh[5];
        double vtGlobal[12]{};
        double vhGlobal[12]{};
        double gtGlobal[12]{};
        double ghGlobal[12]{};
        for (int block = 0; block < 4; ++block)
        {
            BeamMatrix3Vector(frame, vt + 3 * block, vtGlobal + 3 * block);
            BeamMatrix3Vector(frame, vh + 3 * block, vhGlobal + 3 * block);
            BeamMatrix3Vector(frame, gt + 3 * block, gtGlobal + 3 * block);
            BeamMatrix3Vector(frame, gh + 3 * block, ghGlobal + 3 * block);
        }
        for (int row = 0; row < 12; ++row)
            for (int column = 0; column < 12; ++column)
                effectiveTangent[row * 12 + column] -=
                    torsionalMomentSum * (vtGlobal[row] * gtGlobal[column] + vhGlobal[row] * ghGlobal[column]);
    }
    return true;
}

template <bool ComputeTangent>
static __global__ void EvaluateBeamKernel(const GpuBeamElementInput* input, double accelerationDerivative,
                                          double velocityDerivative, GpuBeamElementResult* result,
                                          GpuBeamElementState* state, const GpuBeamAssemblyMap* assemblyMap,
                                          double* matrixValues, double* inertiaForce, int elementCount)
{
    const int elementIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (elementIndex >= elementCount)
        return;
    const GpuBeamElementInput element = input[elementIndex];
    GpuBeamElementResult output{};
    if (element.initialLength <= 1.0e-12 || element.young <= 0.0 || element.area <= 0.0 || element.density < 0.0 ||
        1.0 + element.poisson <= 1.0e-12 ||
        !BeamEvaluateStatic<ComputeTangent>(element, output.effectiveTangent, output.internalForce, output.stress,
                                            output.currentLength))
    {
        if (result)
            result[elementIndex] = output;
        if (state)
            state[elementIndex] = {};
        return;
    }
    BeamAddDynamicContributions<ComputeTangent>(element, accelerationDerivative, velocityDerivative,
                                                output.effectiveTangent, output.inertiaForce);
    if (result)
        result[elementIndex] = output;
    if (state)
    {
        GpuBeamElementState elementState{};
        for (int localDof = 0; localDof < 12; ++localDof)
            elementState.internalForce[localDof] = output.internalForce[localDof];
        elementState.stress = output.stress;
        elementState.currentLength = output.currentLength;
        state[elementIndex] = elementState;
    }
    if (matrixValues)
        for (int valueIndex = 0; valueIndex < 144; ++valueIndex)
        {
            const int matrixValueIndex = assemblyMap[elementIndex].matrixValueIndices[valueIndex];
            if (matrixValueIndex >= 0)
                atomicAdd(matrixValues + matrixValueIndex, output.effectiveTangent[valueIndex]);
        }
    if (inertiaForce)
        for (int localDof = 0; localDof < 12; ++localDof)
        {
            const int freeDof = assemblyMap[elementIndex].freeDofs[localDof];
            if (freeDof >= 0)
                atomicAdd(inertiaForce + freeDof, output.inertiaForce[localDof]);
        }
}

static bool CreateCudaBeamTimingEvent(_OUT cudaEvent_t& event)
{
    return event || cudaEventCreate(&event) == cudaSuccess;
}

static bool ReadCudaBeamElapsed(cudaEvent_t start, cudaEvent_t end, _OUT double& milliseconds)
{
    float elapsed = 0.0F;
    if (cudaEventElapsedTime(&elapsed, start, end) != cudaSuccess)
        return false;
    milliseconds = elapsed;
    return true;
}

class CudaBeamBatchEvaluator::Impl final
{
public:
    ~Impl()
    {
        Reset();
    }

    bool Evaluate(const std::vector<GpuBeamElementInput>& input, double accelerationDerivative,
                  double velocityDerivative, _OUT std::vector<GpuBeamElementResult>& result)
    {
        m_lastTiming = {};
        if (input.empty() || input.size() > static_cast<std::size_t>(INT_MAX) || !EnsureStorage(input.size(), true))
            return false;
        std::copy(input.cbegin(), input.cend(), m_hostInput);
        const std::size_t inputBytes = input.size() * sizeof(GpuBeamElementInput);
        if (cudaEventRecord(m_uploadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_deviceInput, m_hostInput, inputBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
            cudaEventRecord(m_uploadEnd, m_stream) != cudaSuccess)
            return false;
        constexpr int threadsPerBlock = 128;
        const int blocks = (static_cast<int>(input.size()) + threadsPerBlock - 1) / threadsPerBlock;
        if (cudaEventRecord(m_kernelStart, m_stream) != cudaSuccess)
            return false;
        EvaluateBeamKernel<true><<<blocks, threadsPerBlock, 0, m_stream>>>(
            m_deviceInput, accelerationDerivative, velocityDerivative, m_deviceResult, nullptr, nullptr, nullptr,
            nullptr, static_cast<int>(input.size()));
        if (cudaGetLastError() != cudaSuccess || cudaEventRecord(m_kernelEnd, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_hostResult, m_deviceResult, input.size() * sizeof(GpuBeamElementResult),
                            cudaMemcpyDeviceToHost, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;
        if (!ReadCudaBeamElapsed(m_uploadStart, m_uploadEnd, m_lastTiming.uploadMilliseconds) ||
            !ReadCudaBeamElapsed(m_kernelStart, m_kernelEnd, m_lastTiming.kernelMilliseconds) ||
            !ReadCudaBeamElapsed(m_downloadStart, m_downloadEnd, m_lastTiming.downloadMilliseconds))
            return false;
        result.assign(m_hostResult, m_hostResult + input.size());
        return true;
    }

    bool EvaluateAndAssemble(const std::vector<GpuBeamElementInput>& input,
                             const std::vector<GpuBeamAssemblyMap>& assemblyMap, double accelerationDerivative,
                             double velocityDerivative, int matrixValueCount, int freeDofCount, bool assembleMatrix)
    {
        m_lastTiming = {};
        m_assemblyResultCount = 0;
        if (input.empty() || input.size() != assemblyMap.size() || input.size() > static_cast<std::size_t>(INT_MAX) ||
            matrixValueCount <= 0 || freeDofCount <= 0 || !EnsureStorage(input.size(), false) ||
            !EnsureAssemblyStorage(input.size(), matrixValueCount, freeDofCount))
            return false;
        std::copy(input.cbegin(), input.cend(), m_hostInput);
        bool mappingChanged = false;
        if (!PrepareAssemblyMap(assemblyMap, mappingChanged))
            return false;
        const std::size_t inputBytes = input.size() * sizeof(GpuBeamElementInput);
        const std::size_t matrixBytes = static_cast<std::size_t>(matrixValueCount) * sizeof(double);
        const std::size_t inertiaBytes = static_cast<std::size_t>(freeDofCount) * sizeof(double);
        if (cudaEventRecord(m_uploadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_deviceInput, m_hostInput, inputBytes, cudaMemcpyHostToDevice, m_stream) != cudaSuccess ||
            (mappingChanged &&
             cudaMemcpyAsync(m_deviceAssemblyMap, m_hostAssemblyMap, input.size() * sizeof(GpuBeamAssemblyMap),
                             cudaMemcpyHostToDevice, m_stream) != cudaSuccess) ||
            (assembleMatrix && cudaMemsetAsync(m_deviceAssemblyValues, 0, matrixBytes, m_stream) != cudaSuccess) ||
            cudaMemsetAsync(m_deviceAssemblyInertia, 0, inertiaBytes, m_stream) != cudaSuccess ||
            cudaEventRecord(m_uploadEnd, m_stream) != cudaSuccess)
            return false;
        constexpr int threadsPerBlock = 128;
        const int blocks = (static_cast<int>(input.size()) + threadsPerBlock - 1) / threadsPerBlock;
        if (cudaEventRecord(m_kernelStart, m_stream) != cudaSuccess)
            return false;
        if (assembleMatrix)
        {
            EvaluateBeamKernel<true><<<blocks, threadsPerBlock, 0, m_stream>>>(
                m_deviceInput, accelerationDerivative, velocityDerivative, nullptr, m_deviceState,
                m_deviceAssemblyMap, m_deviceAssemblyValues, m_deviceAssemblyInertia,
                static_cast<int>(input.size()));
        }
        else
        {
            EvaluateBeamKernel<false><<<blocks, threadsPerBlock, 0, m_stream>>>(
                m_deviceInput, 0.0, 0.0, nullptr, m_deviceState, m_deviceAssemblyMap, nullptr,
                m_deviceAssemblyInertia, static_cast<int>(input.size()));
        }
        if (cudaGetLastError() != cudaSuccess || cudaEventRecord(m_kernelEnd, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(m_hostState, m_deviceState, input.size() * sizeof(GpuBeamElementState),
                            cudaMemcpyDeviceToHost, m_stream) != cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;
        if (!ReadCudaBeamElapsed(m_uploadStart, m_uploadEnd, m_lastTiming.uploadMilliseconds) ||
            !ReadCudaBeamElapsed(m_kernelStart, m_kernelEnd, m_lastTiming.kernelMilliseconds) ||
            !ReadCudaBeamElapsed(m_downloadStart, m_downloadEnd, m_lastTiming.downloadMilliseconds))
            return false;
        m_assemblyResultCount = input.size();
        return true;
    }

    bool CopyAssembledData(_OUT double* matrixValues, int matrixValueCount, _OUT double* inertiaForce, int freeDofCount)
    {
        if (!matrixValues || !inertiaForce || matrixValueCount != m_assemblyMatrixValueCount ||
            freeDofCount != m_assemblyFreeDofCount || !m_deviceAssemblyValues || !m_deviceAssemblyInertia)
            return false;
        const std::size_t matrixBytes = static_cast<std::size_t>(matrixValueCount) * sizeof(double);
        const std::size_t inertiaBytes = static_cast<std::size_t>(freeDofCount) * sizeof(double);
        if (cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(matrixValues, m_deviceAssemblyValues, matrixBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaMemcpyAsync(inertiaForce, m_deviceAssemblyInertia, inertiaBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;
        return ReadCudaBeamElapsed(m_downloadStart, m_downloadEnd, m_lastTiming.assemblyDownloadMilliseconds);
    }

    bool CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount)
    {
        if (!inertiaForce || freeDofCount != m_assemblyFreeDofCount || !m_deviceAssemblyInertia)
            return false;
        const std::size_t inertiaBytes = static_cast<std::size_t>(freeDofCount) * sizeof(double);
        if (cudaEventRecord(m_downloadStart, m_stream) != cudaSuccess ||
            cudaMemcpyAsync(inertiaForce, m_deviceAssemblyInertia, inertiaBytes, cudaMemcpyDeviceToHost, m_stream) !=
                cudaSuccess ||
            cudaEventRecord(m_downloadEnd, m_stream) != cudaSuccess ||
            cudaEventSynchronize(m_downloadEnd) != cudaSuccess)
            return false;
        return ReadCudaBeamElapsed(m_downloadStart, m_downloadEnd, m_lastTiming.assemblyDownloadMilliseconds);
    }

    const GpuBeamElementState* StateData() const
    {
        return m_assemblyResultCount > 0 ? m_hostState : nullptr;
    }

    const double* DeviceMatrixValues() const
    {
        return m_deviceAssemblyValues;
    }

    int DeviceMatrixValueCount() const
    {
        return m_assemblyMatrixValueCount;
    }

    const GpuBeamBatchTiming& LastTiming() const
    {
        return m_lastTiming;
    }

private:
    bool EnsureStorage(std::size_t requestedCapacity, bool requireElementResult)
    {
        if (cudaSetDevice(0) != cudaSuccess)
            return false;
        if (!m_stream && cudaStreamCreate(&m_stream) != cudaSuccess)
            return false;
        if (!CreateCudaBeamTimingEvent(m_uploadStart) || !CreateCudaBeamTimingEvent(m_uploadEnd) ||
            !CreateCudaBeamTimingEvent(m_kernelStart) || !CreateCudaBeamTimingEvent(m_kernelEnd) ||
            !CreateCudaBeamTimingEvent(m_downloadStart) || !CreateCudaBeamTimingEvent(m_downloadEnd))
            return false;
        const bool hasInputStorage = requestedCapacity <= m_capacity && m_deviceInput && m_hostInput;
        if (!hasInputStorage)
        {
            ReleaseStorage();
            if (cudaMalloc(reinterpret_cast<void**>(&m_deviceInput), requestedCapacity * sizeof(GpuBeamElementInput)) !=
                    cudaSuccess ||
                cudaMallocHost(reinterpret_cast<void**>(&m_hostInput),
                               requestedCapacity * sizeof(GpuBeamElementInput)) !=
                    cudaSuccess)
            {
                ReleaseStorage();
                return false;
            }
            m_capacity = requestedCapacity;
        }

        if (!requireElementResult)
            return true;
        const bool hasResultStorage = requestedCapacity <= m_resultCapacity && m_deviceResult && m_hostResult;
        if (hasResultStorage)
            return true;
        ReleaseResultStorage();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceResult), requestedCapacity * sizeof(GpuBeamElementResult)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostResult), requestedCapacity * sizeof(GpuBeamElementResult)) !=
                cudaSuccess)
        {
            ReleaseResultStorage();
            return false;
        }
        m_resultCapacity = requestedCapacity;
        return true;
    }

    bool EnsureAssemblyStorage(std::size_t requestedCapacity, int matrixValueCount, int freeDofCount)
    {
        const bool matchingState = requestedCapacity <= m_assemblyCapacity && m_deviceState && m_hostState &&
                                   m_deviceAssemblyMap && m_hostAssemblyMap;
        const bool matchingMatrix = matrixValueCount == m_assemblyMatrixValueCount &&
                                    freeDofCount == m_assemblyFreeDofCount && m_deviceAssemblyValues &&
                                    m_deviceAssemblyInertia;
        if (matchingState && matchingMatrix)
            return true;
        ReleaseAssemblyStorage();
        if (cudaMalloc(reinterpret_cast<void**>(&m_deviceState), requestedCapacity * sizeof(GpuBeamElementState)) !=
                cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostState), requestedCapacity * sizeof(GpuBeamElementState)) !=
                cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceAssemblyMap),
                       requestedCapacity * sizeof(GpuBeamAssemblyMap)) != cudaSuccess ||
            cudaMallocHost(reinterpret_cast<void**>(&m_hostAssemblyMap),
                           requestedCapacity * sizeof(GpuBeamAssemblyMap)) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceAssemblyValues),
                       static_cast<std::size_t>(matrixValueCount) * sizeof(double)) != cudaSuccess ||
            cudaMalloc(reinterpret_cast<void**>(&m_deviceAssemblyInertia),
                       static_cast<std::size_t>(freeDofCount) * sizeof(double)) != cudaSuccess)
        {
            ReleaseAssemblyStorage();
            return false;
        }
        m_assemblyCapacity = requestedCapacity;
        m_assemblyMatrixValueCount = matrixValueCount;
        m_assemblyFreeDofCount = freeDofCount;
        return true;
    }

    bool PrepareAssemblyMap(const std::vector<GpuBeamAssemblyMap>& assemblyMap, _OUT bool& changed)
    {
        changed = assemblyMap.size() != m_assemblyMapCount;
        if (!changed)
        {
            changed = !std::equal(assemblyMap.cbegin(), assemblyMap.cend(), m_hostAssemblyMap,
                                  [](const GpuBeamAssemblyMap& lhs, const GpuBeamAssemblyMap& rhs)
                                  {
                                      return std::equal(lhs.matrixValueIndices, lhs.matrixValueIndices + 144,
                                                        rhs.matrixValueIndices) &&
                                             std::equal(lhs.freeDofs, lhs.freeDofs + 12, rhs.freeDofs);
                                  });
        }
        if (!changed)
            return true;
        std::copy(assemblyMap.cbegin(), assemblyMap.cend(), m_hostAssemblyMap);
        m_assemblyMapCount = assemblyMap.size();
        return true;
    }

    void ReleaseAssemblyStorage()
    {
        if (m_deviceAssemblyInertia)
            cudaFree(m_deviceAssemblyInertia);
        if (m_deviceAssemblyValues)
            cudaFree(m_deviceAssemblyValues);
        if (m_hostAssemblyMap)
            cudaFreeHost(m_hostAssemblyMap);
        if (m_deviceAssemblyMap)
            cudaFree(m_deviceAssemblyMap);
        if (m_hostState)
            cudaFreeHost(m_hostState);
        if (m_deviceState)
            cudaFree(m_deviceState);
        m_deviceAssemblyInertia = nullptr;
        m_deviceAssemblyValues = nullptr;
        m_hostAssemblyMap = nullptr;
        m_deviceAssemblyMap = nullptr;
        m_hostState = nullptr;
        m_deviceState = nullptr;
        m_assemblyCapacity = 0;
        m_assemblyResultCount = 0;
        m_assemblyMapCount = 0;
        m_assemblyMatrixValueCount = 0;
        m_assemblyFreeDofCount = 0;
    }

    void ReleaseResultStorage()
    {
        if (m_hostResult)
            cudaFreeHost(m_hostResult);
        if (m_deviceResult)
            cudaFree(m_deviceResult);
        m_hostResult = nullptr;
        m_deviceResult = nullptr;
        m_resultCapacity = 0;
    }

    void ReleaseStorage()
    {
        ReleaseAssemblyStorage();
        ReleaseResultStorage();
        if (m_hostInput)
            cudaFreeHost(m_hostInput);
        if (m_deviceInput)
            cudaFree(m_deviceInput);
        m_hostResult = nullptr;
        m_hostInput = nullptr;
        m_deviceResult = nullptr;
        m_deviceInput = nullptr;
        m_capacity = 0;
    }

    void Reset()
    {
        ReleaseStorage();
        if (m_downloadEnd)
            cudaEventDestroy(m_downloadEnd);
        if (m_downloadStart)
            cudaEventDestroy(m_downloadStart);
        if (m_kernelEnd)
            cudaEventDestroy(m_kernelEnd);
        if (m_kernelStart)
            cudaEventDestroy(m_kernelStart);
        if (m_uploadEnd)
            cudaEventDestroy(m_uploadEnd);
        if (m_uploadStart)
            cudaEventDestroy(m_uploadStart);
        if (m_stream)
            cudaStreamDestroy(m_stream);
        m_downloadEnd = nullptr;
        m_downloadStart = nullptr;
        m_kernelEnd = nullptr;
        m_kernelStart = nullptr;
        m_uploadEnd = nullptr;
        m_uploadStart = nullptr;
        m_stream = nullptr;
    }

    cudaStream_t m_stream = nullptr;
    cudaEvent_t m_uploadStart = nullptr;
    cudaEvent_t m_uploadEnd = nullptr;
    cudaEvent_t m_kernelStart = nullptr;
    cudaEvent_t m_kernelEnd = nullptr;
    cudaEvent_t m_downloadStart = nullptr;
    cudaEvent_t m_downloadEnd = nullptr;
    GpuBeamElementInput* m_deviceInput = nullptr;
    GpuBeamElementResult* m_deviceResult = nullptr;
    GpuBeamElementInput* m_hostInput = nullptr;
    GpuBeamElementResult* m_hostResult = nullptr;
    GpuBeamElementState* m_deviceState = nullptr;
    GpuBeamElementState* m_hostState = nullptr;
    GpuBeamAssemblyMap* m_deviceAssemblyMap = nullptr;
    GpuBeamAssemblyMap* m_hostAssemblyMap = nullptr;
    double* m_deviceAssemblyValues = nullptr;
    double* m_deviceAssemblyInertia = nullptr;
    std::size_t m_capacity = 0;
    std::size_t m_resultCapacity = 0;
    std::size_t m_assemblyCapacity = 0;
    std::size_t m_assemblyResultCount = 0;
    std::size_t m_assemblyMapCount = 0;
    int m_assemblyMatrixValueCount = 0;
    int m_assemblyFreeDofCount = 0;
    GpuBeamBatchTiming m_lastTiming;
};

CudaBeamBatchEvaluator::CudaBeamBatchEvaluator()
    : m_impl(std::make_unique<Impl>())
{
}

CudaBeamBatchEvaluator::~CudaBeamBatchEvaluator() = default;

CudaBeamBatchEvaluator::CudaBeamBatchEvaluator(const CudaBeamBatchEvaluator& other)
    : m_impl(std::make_unique<Impl>())
{
    (void)other;
}

CudaBeamBatchEvaluator& CudaBeamBatchEvaluator::operator=(const CudaBeamBatchEvaluator& other)
{
    if (this != &other)
    {
        (void)other;
        m_impl = std::make_unique<Impl>();
    }
    return *this;
}

bool CudaBeamBatchEvaluator::Evaluate(const std::vector<GpuBeamElementInput>& input, double accelerationDerivative,
                                      double velocityDerivative, _OUT std::vector<GpuBeamElementResult>& result)
{
    return m_impl->Evaluate(input, accelerationDerivative, velocityDerivative, result);
}

bool CudaBeamBatchEvaluator::EvaluateAndAssemble(const std::vector<GpuBeamElementInput>& input,
                                                 const std::vector<GpuBeamAssemblyMap>& assemblyMap,
                                                 double accelerationDerivative, double velocityDerivative,
                                                 int matrixValueCount, int freeDofCount)
{
    return m_impl->EvaluateAndAssemble(input, assemblyMap, accelerationDerivative, velocityDerivative,
                                       matrixValueCount, freeDofCount, true);
}

bool CudaBeamBatchEvaluator::EvaluateResidualAndAssemble(
    const std::vector<GpuBeamElementInput>& input, const std::vector<GpuBeamAssemblyMap>& assemblyMap,
    int matrixValueCount, int freeDofCount)
{
    return m_impl->EvaluateAndAssemble(input, assemblyMap, 0.0, 0.0, matrixValueCount, freeDofCount, false);
}

bool CudaBeamBatchEvaluator::CopyAssembledData(_OUT double* matrixValues, int matrixValueCount,
                                               _OUT double* inertiaForce, int freeDofCount)
{
    return m_impl->CopyAssembledData(matrixValues, matrixValueCount, inertiaForce, freeDofCount);
}

bool CudaBeamBatchEvaluator::CopyAssembledInertia(_OUT double* inertiaForce, int freeDofCount)
{
    return m_impl->CopyAssembledInertia(inertiaForce, freeDofCount);
}

const GpuBeamElementState* CudaBeamBatchEvaluator::StateData() const
{
    return m_impl->StateData();
}

const double* CudaBeamBatchEvaluator::DeviceMatrixValues() const
{
    return m_impl->DeviceMatrixValues();
}

int CudaBeamBatchEvaluator::DeviceMatrixValueCount() const
{
    return m_impl->DeviceMatrixValueCount();
}

const GpuBeamBatchTiming& CudaBeamBatchEvaluator::LastTiming() const
{
    return m_impl->LastTiming();
}
}
