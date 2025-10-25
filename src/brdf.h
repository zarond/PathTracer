#pragma once

#include "cpu_framebuffer.h"
#include "model_loader.h"

#include <fastgltf/core.hpp>
#include <fastgltf/types.hpp>
#include <fastgltf/tools.hpp>

namespace app {

using namespace fastgltf::math;

fvec4 sample_albedo(
    const Material& material,
    const std::vector<CPUTexture<sdr_pixel>>& images,
    const fvec2 uv);

}