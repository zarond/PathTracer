#include "brdf.h"

namespace{
    using namespace glm;
    using namespace app;

    constexpr float GOLDEN_RATIO = 1.618034f;

    fvec4 sample_srgba(fvec4 colorFactor, const CPUTexture<sdr_pixel>* texture, const fvec2 uv)
    {
        if (texture) {
            auto sampled_col = texture->sample_bilinear(uv, true);
            colorFactor *= sampled_col;
        }
        return colorFactor;
    }
    fvec4 sample_rgba(fvec4 colorFactor, const CPUTexture<sdr_pixel>* texture, const fvec2 uv)
    {
        if (texture) {
            auto sampled_col = texture->sample_bilinear(uv);
            colorFactor *= sampled_col;
        }
        return colorFactor;
    }
    float sample_r(float Factor, const CPUTexture<sdr_pixel>* texture, const fvec2 uv)
    {
        if (texture) {
            auto sampled_col = texture->sample_bilinear(uv);
            Factor *= sampled_col.x;
        }
        return Factor;
    }
    float pow2(float v) {
        return v * v;
    }
}

namespace app {
    fvec4 sample_albedo(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv)
    {
        return sample_srgba(material.baseColorFactor,
                            material.baseColorTextureIndex >= 0 ? &images[material.baseColorTextureIndex] : nullptr,
                            uv);
    }
    fvec4 sample_roughness_metallic(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv)
    {
        return sample_rgba(fvec4(1.0f, material.roughnessFactor, material.metallicFactor, 1.0f),
            material.metallicRoughnessTextureIndex >= 0 ? &images[material.metallicRoughnessTextureIndex] : nullptr,
            uv);
    }
    fvec4 sample_normals(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv)
    {
        const CPUTexture<sdr_pixel>* texture = material.normalTextureIndex >= 0 ? &images[material.normalTextureIndex] : nullptr;
        if (texture) {
            return texture->sample_bilinear(uv);
        }
        return fvec4(0.5f, 0.5f, 1.0f, 0.0f); // w = 0.0f means no normal map
    }
    fvec4 sample_emissive(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv) {
        return sample_srgba(xyz1(material.emissiveFactor),
            material.emissiveTextureIndex >= 0 ? &images[material.emissiveTextureIndex] : nullptr,
            uv) * material.emissiveStrength;
    }
    float sample_transmission(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv) {
        return sample_r(material.transmisionFactor,
            material.transmissionTextureIndex >= 0 ? &images[material.transmissionTextureIndex] : nullptr,
            uv);
    }
    fvec4 sample_environment(const fvec3 dir, const CPUTexture<hdr_pixel>& environment_texture)
    {
        // compared to blender envmap is rotated 180 degrees around Y (blender's Z) axis, but same as SP
        // Todo: envmap rotation setting?
        assert(abs(length(dir) - 1.0f) < 1e-5);
        fvec2 uv = fvec2( std::atan2(-dir.z, -dir.x), -2.0f * std::asin(dir.y)) * (1.0f / pi<float>());
        uv = uv * 0.5f + fvec2(0.5f);
        return environment_texture.sample_bilinear(uv);
    }
    float f0_dielectric(float transmitted_ior, float incident_ior) {
        return pow2((transmitted_ior - incident_ior) / (transmitted_ior + incident_ior));
    }
    float fibonacci1D(int i) { return std::fmod((static_cast<float>(i) + 1.0f) * GOLDEN_RATIO, 1.0f); }
    float fibonacci1D(float i) { return std::fmod((i + 1.0f) * GOLDEN_RATIO, 1.0f); }
    fvec2 fibonacci2D(int i, float inv_nbSamples)
    {
        float i_f = static_cast<float>(i);
        return fvec2(
            (i_f + 0.5f) * inv_nbSamples,
            fibonacci1D(i_f)
        );
    }
}