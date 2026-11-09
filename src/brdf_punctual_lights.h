#pragma once

#include "brdf.h"
#include "lights.h"

namespace app {
// ---------------------------------------------------------------------------
// Punctual lights (KHR_lights_punctual) next event estimation.
// C++ port of the logic in PunctualLights.hlsl
// ---------------------------------------------------------------------------
inline constexpr float MIN_PERCEPTUAL_ROUGHNESS = 0.045;
inline constexpr float MIN_ROUGHNESS = 0.002025f;
inline constexpr float MIN_POINT_LIGHT_DISTANCE_SQUARED = 0.00001f;

struct NeeMaterialParams {
    fvec3 diffuse_color;             // diffuse color already with transmission factor applied
    float linear_roughness;
    fvec3 f0;
    float transmission_roughness;
    fvec3 transmission_attenuation;  // with transmission factor and albedo applied
};

struct LightRay {
    fvec3 color;
    fvec3 direction;
};

inline float D_GGX(float NoH, float linear_roughness) {
    float a = NoH * linear_roughness;
    float k = linear_roughness / (1.0f - NoH * NoH + a * a);
    return k * k * glm::one_over_pi<float>();
}

inline fvec3 lambertian_BRDF(const fvec3& diffuse_color) { return diffuse_color * glm::one_over_pi<float>(); }

inline float spot_attenuation(const fvec3& light_direction, const fvec3& direction_to_light, const fvec2& cone_cosines) {
    const float b2 = cone_cosines.x;  // inner cone cosine
    const float b1 = cone_cosines.y;  // outer cone cosine
    const float d_cos = dot(light_direction, direction_to_light);
    return glm::smoothstep(b1, b2, d_cos);
}

inline LightRay convert_to_light_ray(const fvec3& pos, const PackedLight& light) {
    LightRay light_ray;
    light_ray.color = xyz(light.color_intensity) * light.color_intensity.a;

    const auto light_type = static_cast<PackedLightType>(static_cast<int>(light.direction_type.w));
    if (light_type == PackedLightType::Directional) {
        light_ray.direction = xyz(light.direction_type);
        return light_ray;
    }

    const fvec3 ray_pl = xyz(light.position_range) - pos;
    const float ray_pl_sq_magnitude = dot(ray_pl, ray_pl);
    light_ray.direction = (ray_pl_sq_magnitude > 0.0f) ? ray_pl * glm::inversesqrt(ray_pl_sq_magnitude) : fvec3(0.0f);

    float falloff = 1.0f / glm::max(ray_pl_sq_magnitude, MIN_POINT_LIGHT_DISTANCE_SQUARED);  // square distance falloff

    if (light.position_range.w > 0.0f) {  // range falloff
        float factor = ray_pl_sq_magnitude / (light.position_range.w * light.position_range.w);
        factor = glm::clamp(1.0f - factor * factor, 0.0f, 1.0f);
        falloff *= factor;  // or factor * factor for smoother falloff?
    }
    if (light_type == PackedLightType::Spot) {  // spotlight falloff
        falloff *= spot_attenuation(xyz(light.direction_type), light_ray.direction, fvec2(light.inner_outerConeAngleCos));
    }

    light_ray.color *= falloff;
    return light_ray;
}

inline fvec3 directional_lighting(const fvec3& n, const fvec3& v, fvec3 refracted_v,
    NeeMaterialParams mat_params, const LightRay& light_ray, const bool has_transmission) {
    const fvec3 f90 = fvec3(1.0f);
    auto h = normalize(v + light_ray.direction);
    float VdH = glm::clamp(dot(v, h), 0.0f, 1.0f);
    float VdN = glm::clamp(dot(v, n), 0.0f, 1.0f);
    float LdN = glm::clamp(dot(n, light_ray.direction), 0.0f, 1.0f);
    float NdH = glm::clamp(dot(n, h), 0.0f, 1.0f);
    auto F = fresnel_schlick(mat_params.f0, f90, VdH);

    mat_params.linear_roughness = glm::max(mat_params.linear_roughness, MIN_ROUGHNESS);  // prevent division by zero

    float G = V_SmithGGXCorrelated(VdN, LdN, mat_params.linear_roughness);
    float D = D_GGX(NdH, mat_params.linear_roughness);

    auto diffuse_brdf = (1.0f - F) * lambertian_BRDF(mat_params.diffuse_color);
    auto specular_brdf = (D * G) * F * LdN;
    specular_brdf = glm::min(specular_brdf, kMaxBRDF);  // clamp to avoid fireflies

    fvec3 R = (diffuse_brdf * LdN + specular_brdf) * light_ray.color;

    if (!has_transmission) {
        return R;
    }
    mat_params.transmission_roughness = glm::max(mat_params.transmission_roughness, MIN_ROUGHNESS);

    refracted_v = -reflect(refracted_v, n);

    h = normalize(refracted_v + light_ray.direction);
    VdH = glm::clamp(dot(refracted_v, h), 0.0f, 1.0f);
    NdH = glm::clamp(dot(-n, h), 0.0f, 1.0f);
    LdN = glm::clamp(dot(-n, light_ray.direction), 0.0f, 1.0f);
    VdN = glm::clamp(dot(-n, refracted_v), 0.0f, 1.0f);

    F = fresnel_schlick(mat_params.f0, f90, VdH);
    G = V_SmithGGXCorrelated(VdN, LdN, mat_params.transmission_roughness);
    D = D_GGX(NdH, mat_params.transmission_roughness);

    auto transmittance_brdf = (D * G) * (1.0f - F) * LdN;
    transmittance_brdf = glm::min(transmittance_brdf, kMaxBRDF);  // clamp to avoid fireflies
    R += transmittance_brdf * light_ray.color * mat_params.transmission_attenuation;

    return R;
}

}
