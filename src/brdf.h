#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <vector>

#include "cpu_framebuffer.h"
#include "model_loader.h"

namespace app {

using namespace glm;

fvec4 sample_albedo(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv);

fvec4 sample_environment(const fvec3 dir, const CPUTexture<hdr_pixel>& environment_texture, float y_rotation = 0.0f);

fvec4 sample_roughness_metallic(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv);

fvec4 sample_normals(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv);

fvec4 sample_emissive(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv);

float sample_transmission(const Material& material, const std::vector<CPUTexture<sdr_pixel>>& images, const fvec2 uv);

float f0_dielectric(float transmitted_ior, float incident_ior = 1.0f);

fvec2 fibonacci2D(int i, float inv_nbSamples);

}  // namespace app
