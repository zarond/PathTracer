#include "brdf.h"

namespace{
    using namespace fastgltf::math;
    using namespace app;

    float linear_to_srgb(float channel)
    {
        if (channel <= 0.0031308f) {
            return 12.92f * channel;
        } else {
            return 1.055f * std::pow(channel, 1.0f / 2.4f) - 0.055f;
        }
    }
    float srgb_to_linear(float channel)
    {
        if (channel <= 0.04045f) {
            return channel / 12.92f;
        } else {
            return std::pow((channel + 0.055f) / 1.055f, 2.4f);
        }
    }
    fvec4 sample_srgba(fvec4 colorFactor, const CPUTexture<sdr_pixel>* texture, const fvec2 uv)
    {
        if (texture) {
            auto sampled_col = texture->sample_nearest(uv);
            fvec4 tex_col = fvec4(
                srgb_to_linear(static_cast<float>(sampled_col[0]) / 255.0f),
                srgb_to_linear(static_cast<float>(sampled_col[1]) / 255.0f),
                srgb_to_linear(static_cast<float>(sampled_col[2]) / 255.0f),
                static_cast<float>(sampled_col[3]) / 255.0f);
            colorFactor *= tex_col;
        }
        return colorFactor;
    }
    fvec4 sample_rgba(fvec4 colorFactor, const CPUTexture<sdr_pixel>* texture, const fvec2 uv)
    {
        if (texture) {
            auto sampled_col = texture->sample_nearest(uv);
            fvec4 tex_col = fvec4(
                static_cast<float>(sampled_col[0]) / 255.0f,
                static_cast<float>(sampled_col[1]) / 255.0f,
                static_cast<float>(sampled_col[2]) / 255.0f,
                static_cast<float>(sampled_col[3]) / 255.0f);
            colorFactor *= tex_col;
        }
        return colorFactor;
    }

}

namespace app {
    fvec4 sample_albedo(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv)
    {
        return sample_srgba(material.baseColorFactor,
                            material.baseColorTextureIndex >= 0 ? &images[material.baseColorTextureIndex] : nullptr,
                            uv);
    }
}