#pragma once

#include <vector>
#include <glm/fwd.hpp>

#include "GPU_model.h"

namespace app {

struct DrawableSortingInfo {
    const GPU_object_info* object;
    float modelScale;
    float ZDistanceToCamera;
    bool backfaceCulling;
    bool alphaBlending;
    bool transmittance;
};

void sort_objects_for_rendering(
    std::vector<DrawableSortingInfo>& sortedDrawables, 
    fvec3 camera_forward, fvec3 cameraPosition,
    const GPU_model& gpu_model, int& num_opaque_objects, int& num_transmissive_objects, int& num_alpha_blended_objects);

}  // namespace app
