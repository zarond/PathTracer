#include "brdf.h"

namespace{
    using namespace fastgltf::math;
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
}