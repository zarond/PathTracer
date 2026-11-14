#pragma once

#include <cassert>
#include <cmath>
#include <glm/fwd.hpp>
#include <glm/gtc/constants.hpp>
#include <vector>

#include "cpu_texture.h"
#include "model_loader.h"

namespace app {

using glm::fvec2;
using glm::fvec3;
using glm::fvec4;

inline constexpr float kEpsilon = 1e-8f;
inline constexpr float kEpsilon5 = 1e-5f;
inline constexpr float kMaxBRDF = 10.0f;
inline constexpr float GOLDEN_RATIO = 1.618034f;

inline fvec4 sample_srgba(fvec4 colorFactor, const CPUTexture<sdr_pixel>* texture, const fvec2 uv) {
    if (texture) {
        auto sampled_col = texture->sample_bilinear(uv, true);
        colorFactor *= sampled_col;
    }
    return colorFactor;
}
inline fvec4 sample_rgba(fvec4 colorFactor, const CPUTexture<sdr_pixel>* texture, const fvec2 uv) {
    if (texture) {
        auto sampled_col = texture->sample_bilinear(uv);
        colorFactor *= sampled_col;
    }
    return colorFactor;
}
inline float sample_r(float Factor, const CPUTexture<sdr_pixel>* texture, const fvec2 uv) {
    if (texture) {
        auto sampled_col = texture->sample_bilinear(uv);
        Factor *= sampled_col.x;
    }
    return Factor;
}
inline float pow2(float v) { return v * v; }

inline fvec4 sample_albedo(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv) {
    return sample_srgba(material.baseColorFactor,
        material.baseColorTextureIndex >= 0 ? &images[material.baseColorTextureIndex] : nullptr, uv);
}
inline fvec4 sample_roughness_metallic(
    const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv) {
    return sample_rgba(fvec4(1.0f, material.roughnessFactor, material.metallicFactor, 1.0f),
        material.metallicRoughnessTextureIndex >= 0 ? &images[material.metallicRoughnessTextureIndex] : nullptr, uv);
}
inline fvec4 sample_normals(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv) {
    const CPUTexture<sdr_pixel>* texture = material.normalTextureIndex >= 0 ? &images[material.normalTextureIndex] : nullptr;
    if (texture) {
        return texture->sample_bilinear(uv);
    }
    return fvec4(0.5f, 0.5f, 1.0f, 0.0f);  // w = 0.0f means no normal map
}
inline fvec4 sample_emissive(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv) {
    return sample_srgba(xyz1(material.emissiveFactor),
           material.emissiveTextureIndex >= 0 ? &images[material.emissiveTextureIndex] : nullptr, uv) *
           material.emissiveStrength;
}
inline float sample_transmission(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv) {
    return sample_r(material.transmissionFactor,
        material.transmissionTextureIndex >= 0 ? &images[material.transmissionTextureIndex] : nullptr, uv);
}
inline fvec4 sample_environment(const fvec3 dir, const CPUTexture<hdr_pixel>& environment_texture, float y_rotation = 0.0f) {
    // compared to Blender, envmap is rotated 180 degrees around Y (Blender's Z) axis, but same as SP
    assert(glm::abs(length(dir) - 1.0f) < 1e-5f);
    fvec2 uv = fvec2(std::atan2(-dir.z, -dir.x) + y_rotation, -2.0f * std::asin(glm::clamp(dir.y, -1.0f, 1.0f))) * glm::one_over_pi<float>();
    uv = uv * 0.5f + fvec2(0.5f);
    assert(!isnan(uv.x) && !isnan(uv.y));
    return environment_texture.sample_bilinear(uv);
}
inline float f0_dielectric(float transmitted_ior, float incident_ior = 1.0f) {
    return pow2((transmitted_ior - incident_ior) / (transmitted_ior + incident_ior));
}
inline float fibonacci1D(int i) { return std::fmod((static_cast<float>(i) + 1.0f) * GOLDEN_RATIO, 1.0f); }
inline float fibonacci1D(float i) { return std::fmod((i + 1.0f) * GOLDEN_RATIO, 1.0f); }
inline fvec2 fibonacci2D(int i, float inv_nbSamples) {
    float i_f = static_cast<float>(i);
    return fvec2((i_f + 0.5f) * inv_nbSamples, fibonacci1D(i_f));
}

inline fvec3 fresnel_schlick(fvec3 f0, fvec3 f90, float cos_nv) {
    float x = 1.0f - cos_nv;
    float x2 = x * x;
    float x5 = x2 * x2 * x;
    return mix(f0, f90, x5);
}
inline float V_SmithGGXCorrelated(float NoV, float NoL, float a) {  // a is alpha_linear_roughness = perceptual_roughness^2
    // Original formulation of G_SmithGGX Correlated
    // lambda_v = (-1 + sqrt ( alphaG2 * (1 - NdotL2 ) / NdotL2 + 1)) * 0.5 f;
    // lambda_l = (-1 + sqrt ( alphaG2 * (1 - NdotV2 ) / NdotV2 + 1)) * 0.5 f;
    // G_SmithGGXCorrelated = 1 / (1 + lambda_v + lambda_l );
    // V_SmithGGXCorrelated = G_SmithGGXCorrelated / (4.0 f * NdotL * NdotV );
    // This is the optimized version
    float a2 = a * a;
    float GGXV = NoL * sqrt((-NoV * a2 + NoV) * NoV + a2);
    float GGXL = NoV * sqrt((-NoL * a2 + NoL) * NoL + a2);
    return 0.5f / (GGXV + GGXL);
}
inline float G1(float NdW, float k) { return 1.0f / (NdW * (1.0f - k) + k); }  // Schlick's G1 function
// Schlick - Smith visibility term
// [ http://blog.selfshadow.com/publications/s2013-shading-course/karis/s2013_pbs_epic_notes_v2.pdf ]
inline float V_Schlick(float NoL, float NoV, float Roughness)  // Roughness is perceptual roughness
{
    float k = glm::max(Roughness * Roughness * 0.5f, kEpsilon5);
    return G1(NoL, k) * G1(NoV, k) / 4.0f;
}

}  // namespace app
