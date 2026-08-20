#pragma once

#include "arguments.h"

namespace app {

struct RenderSettings {
    unsigned int samplesPerPixel = 1;
    unsigned int maxRayBounces = 0;
    unsigned int maxNewRaysPerBounce = 1;
    unsigned int maxTrianglesPerBVHLeaf = 8;
    float envmapRotation = 0;  // in radians

    // only for raster pipeline
    float GTAOStrength = 1.0f;
    float AODistance = 1.0f;
    bool AODenoiseEnabled = true;
    float AOReprojectionDepthThreshold = 0.01f;
    float AOSpatialSigma = 10.0f;   // default is nearly uniform weights
    float AODepthSigma = 0.01;
    float AONormalSigma = 0.01;
    float TexturesAOStrength = 1.0f;
    bool SSREnabled = true;
    bool SSRDenoiseEnabled = true;
    bool SSRRayReuseEnabled = true;
    bool SSRZeroAlphaMotionCleanup = true;
    bool DrawSSROnly = false;
    float SSR_GGXClamp = 0.1f;  // 0.0 means no clamp, > 0.0 reduces tail of distribution
    float SSRDepthThreshold = 0.01f;
    float SSRMaxRoughness = 0.8f;
    bool SSRUsePrefiltering = true;
    float SSRPrefilteringDistance = 0.3f;
    bool SSRParallaxReprojection = true;
    bool ReprojectionDebugMode = false;
    bool specular_aa_enabled = true;
    float specular_aa_variance = 0.15f;
    float specular_aa_threshold = 0.2f;

    RenderSettings() = default;
    explicit RenderSettings(const ConsoleArgs& args);

    RayProgramMode programMode = RayProgramMode::RayCaster;
    AccelerationStructureType accelStructType = AccelerationStructureType::Naive;

    bool operator==(const RenderSettings& other) const = default;
};

}  // namespace app
