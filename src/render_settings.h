#pragma once

#include "arguments.h"

namespace app {

struct RenderSettings {
    unsigned int samplesPerPixel = 1;
    unsigned int maxRayBounces = 0;
    unsigned int maxNewRaysPerBounce = 1;
    unsigned int maxTrianglesPerBVHLeaf = 8;
    float envmapRotation = 0;  // in radians

    float TexturesAOStrength = 1.0f;        // only for raster pipeline
    bool specular_aa_enabled = true;
    float specular_aa_variance = 0.15f;     // only for raster pipeline
    float specular_aa_threshold = 0.2f;     // only for raster pipeline

    RenderSettings() = default;
    explicit RenderSettings(const ConsoleArgs& args);

    RayProgramMode programMode = RayProgramMode::RayCaster;
    AccelerationStructureType accelStructType = AccelerationStructureType::Naive;

    bool operator==(const RenderSettings& other) const = default;
};

}  // namespace app
