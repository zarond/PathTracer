#include "brdf.h"

namespace{
    using namespace glm;
    using namespace app;

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

}

namespace app {
    fvec4 sample_albedo(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv)
    {
        return sample_srgba(material.baseColorFactor,
                            material.baseColorTextureIndex >= 0 ? &images[material.baseColorTextureIndex] : nullptr,
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
}