#pragma once

#include <cmath>
#include <glm/glm.hpp>

#include "model_loader.h"

namespace app {

// NOTE: underlying values must match `enum LightType` in PunctualLights.hlsl
enum class PackedLightType : int {
    Directional = 0,
    Spot = 1,
    Point = 2,
};

// Packed per-instance light, shared between the GPU light buffer (Common.hlsl struct Light) and the CPU path tracer
struct PackedLight {
    fvec4 color_intensity;             // rgb = color, a = intensity (radiometric units)
    fvec4 position_range;              // xyz = world position, w = range, -1.0f if unused
    fvec4 direction_type;              // xyz = normalized direction, w = PackedLightType
    fvec4 inner_outerConeAngleCos;     // x = cos(inner), y = cos(outer)
};

// lx -> W/m^2
inline float photometric_to_radiometric_irradiance(float lux) { return lux / 683.0f; }

// candela -> W/sr
inline float photometric_to_radiometric_intensity(float candela) { return candela / (683.0f * 0.25f / glm::pi<float>()); }

// converts intensity from Lux and Candela (as in KHR_lights_punctual) to Watts/M^2 and Watts/Sr to use in shader
inline float photometric_to_radiometric_units(const fastgltf::Light& light) {
    if (light.type == fastgltf::LightType::Directional) {
        return photometric_to_radiometric_irradiance(light.intensity);
    } else if (light.type == fastgltf::LightType::Point || light.type == fastgltf::LightType::Spot) {
        return photometric_to_radiometric_intensity(light.intensity);
    }
    return 0.0;
}

// converts KHR_lights_punctual lights into the packed representation used by all renderers.
inline PackedLight make_packed_light(const Light& light, const fmat4x4& transform) {
    PackedLight out_light{};
    float intensity = photometric_to_radiometric_irradiance(light.intensity); // to skip spherical area influence conversion in shader
    out_light.color_intensity = {light.color.x, light.color.y, light.color.z, intensity};
    float range = (light.use_range && light.type != fastgltf::LightType::Directional) ? light.range : -1.0f;
    out_light.position_range = {transform[3][0], transform[3][1], transform[3][2], range};
    auto direction = glm::normalize(transform[2]);
    PackedLightType packed_type = PackedLightType(light.type);
    out_light.direction_type = {direction.x, direction.y, direction.z, static_cast<float>(packed_type)};
    out_light.inner_outerConeAngleCos = {
        std::cos(light.innerConeAngle),
        std::cos(light.outerConeAngle),
        0.0f, 0.0f
    };
    return out_light;
}

}  // namespace app
