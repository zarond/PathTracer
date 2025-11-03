#pragma once

#include "cpu_framebuffer.h"
#include "model_loader.h"

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

namespace app {

using namespace glm;

fvec4 sample_albedo(
    const Material& material,
    const std::vector<CPUTexture<sdr_pixel>>& images,
    const fvec2 uv);

fvec4 sample_environment(const fvec3 dir, const CPUTexture<hdr_pixel>& environment_texture);

}
