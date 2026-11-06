#include "Drawables.h"

#include <algorithm>

namespace app {

using namespace glm;

void sort_objects_for_rendering(
    std::vector<DrawableSortingInfo>& sortedDrawables, 
    fvec3 camera_forward,
    fvec3 cameraPosition,
    const GPU_model& gpu_model, int& num_opaque_objects, int& num_transmissive_objects, int& num_alpha_blended_objects) 
{
    num_opaque_objects = 0;
    num_transmissive_objects = 0;
    num_alpha_blended_objects = 0;
    sortedDrawables.clear();
    sortedDrawables.reserve(gpu_model.objects.size());
    const auto& materials = gpu_model.get_materials_cpu_array();
    for (const auto& obj : gpu_model.objects) {
        fvec3 obj_pos = fvec3(obj.ModelMatrix[0][3], obj.ModelMatrix[1][3], obj.ModelMatrix[2][3]);
        fvec3 v = obj_pos - cameraPosition;
        const auto& mat = materials[obj.meshIndex];
        float ZDistanceToCamera = dot(camera_forward, v);
        bool backfaceCulling = (mat.doubleSided == 0);
        bool alphaBlending = mat.alphaBlending;
        bool transmittance = mat.hasVolume || (mat.transmisionFactor != 0.0f);

        glm::fvec3 scale;
        scale.x = glm::length(fvec3(obj.ModelMatrix[0][0], obj.ModelMatrix[1][0], obj.ModelMatrix[2][0]));
        scale.y = glm::length(fvec3(obj.ModelMatrix[0][1], obj.ModelMatrix[1][1], obj.ModelMatrix[2][1]));
        scale.z = glm::length(fvec3(obj.ModelMatrix[0][2], obj.ModelMatrix[1][2], obj.ModelMatrix[2][2]));
        float model_scale = max(max(scale.x, scale.y), scale.z);

        if (alphaBlending) {
            num_alpha_blended_objects++;
        } else if (transmittance) {
            num_transmissive_objects++;
        } else {
            num_opaque_objects++;
        }

        sortedDrawables.emplace_back(&obj, model_scale, ZDistanceToCamera, backfaceCulling, alphaBlending, transmittance);
    }
    std::sort(sortedDrawables.begin(), sortedDrawables.end(), [](const DrawableSortingInfo& a, const DrawableSortingInfo& b) {
        if (a.alphaBlending != b.alphaBlending) {
            return !a.alphaBlending;  // opaque first
        }
        if (a.transmittance != b.transmittance) {
            return !a.transmittance;  // without transmittance first
        }
        if (!a.alphaBlending) {
            return a.ZDistanceToCamera < b.ZDistanceToCamera;  // front to back for opaque objects
        } else {
            return a.ZDistanceToCamera > b.ZDistanceToCamera;  // back to front for alpha blended objects
        }
    });
}
}
