#pragma once

#include "arguments.h"

namespace app {

struct RenderSettings {
    unsigned int samplesPerPixel = 1;
    unsigned int maxRayBounces = 0;
    unsigned int maxNewRaysPerBounce = 0;
    unsigned int maxTrianglesPerBVHLeaf = 8;
    float envmapRotation = 0;  // in radians

    RenderSettings() = default;
    explicit RenderSettings(const ConsoleArgs& args);

    RayProgramMode programMode = RayProgramMode::RayCaster;
    AccelerationStructureType accelStructType = AccelerationStructureType::Naive;

    bool operator==(const RenderSettings& other) const = default;
};

}  // namespace app
