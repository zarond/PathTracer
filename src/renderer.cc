#include "renderer.h"
#include "brdf.h"

#include <ranges>
#include <algorithm>
#include <execution>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

namespace {
using namespace glm;
using namespace app;

fvec4 ndc_from_pixel(float x, float y, int width, int height) {
    float ndc_x = x / width * 2.0f - 1.0f;
    float ndc_y = - y / height * 2.0f + 1.0f; // Flip Y if needed??
    return fvec4(ndc_x, ndc_y, 0.0f, 1.0f);
}

}

namespace app {

void Renderer::update_camera_transform_state(
    fvec3 position,
    fvec3 direction,
    fvec3 up,
    fastgltf::Camera::Perspective perspectiveParams)
{
    origin_ = position;
    viewMatrix_ = glm::lookAt(position, position + direction, up);
    projectionMatrix_ = glm::perspectiveRH(
        perspectiveParams.yfov,
        perspectiveParams.aspectRatio.value_or(1.77777777777777777f), // Todo ????
        perspectiveParams.znear,
        perspectiveParams.zfar.value_or(1000.f));
    auto viewMatrixNoTranslation = viewMatrix_;
    viewMatrixNoTranslation[3] = fvec4(0.0f, 0.0f, 0.0f, 1.0f); // Remove translation from view matrix for direction calculation
    NDC2WorldMatrix_ = glm::inverse(projectionMatrix_ * viewMatrixNoTranslation);
}

void Renderer::load_scene(const Model& model, const CPUTexture<hdr_pixel>& envmap)
{
    accelStruct = std::make_unique<NaiveAS>(model);
    modelRef = &model;
    envmapRef = &envmap;
    rayProgram = std::make_unique<RayCasterProgram>(model, envmap); // Todo
    //rayProgram = std::make_unique<AOProgram>(model, envmap);
}

ray_with_payload Renderer::generate_camera_ray(int x, int y, int width, int height, int sampleIndex) const {
    fvec2 pixel_coords = fvec2{static_cast<float>(x), static_cast<float>(y)} + subsamplesPositions[sampleIndex];
    auto ndc_coords = ndc_from_pixel(pixel_coords.x, pixel_coords.y, width, height);
    auto direction = fvec3(NDC2WorldMatrix_ * ndc_coords);
    return ray_with_payload{
        origin_, 
        normalize(direction), 
        fvec4(1.0f), 
        static_cast<std::uint8_t>(renderSettings_.maxRayBounces),
        false
    };
}

void Renderer::render_frame(CPUFrameBuffer& framebuffer)
{
    assert(modelRef);
    if (modelRef == nullptr || envmapRef == nullptr || accelStruct == nullptr || rayProgram == nullptr) {
        throw 1; // Todo
    }

    generate_subsample_positions();

    framebuffer.clear(hdr_pixel{ 0.0f, 0.0f, 1.0f, 1.0f }); // Clear to color for testing

    int width = framebuffer.width();
    int height = framebuffer.height();
    auto indices = std::views::iota(0, height);

    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
    [this, width, height, &framebuffer](int y) {
        for (int x = 0; x < width; ++x) {
            SamplesAccumulator<fvec3> final_color;

            for (int i = 0; i < renderSettings_.samplesPerPixel; ++i) {
                std::vector<ray_with_payload> rays;
                rays.reserve(1 + renderSettings_.maxRayBounces + renderSettings_.maxNewRaysPerBounce);
                fvec4 sample_col{};

                rays.push_back(generate_camera_ray(x, y, width, height, i));
                while (rays.size() > 0) { // Todo: limit number of iterations
                    auto ray = rays.back();
                    rays.pop_back();
                    auto hit = accelStruct->intersect_ray(ray, ray.any_hit);
                    sample_col += rayProgram->on_hit(ray, hit, rays);
                }

                final_color.add_sample(fvec3(sample_col));
            }
            framebuffer.at(x, y) = hdr_pixel{ final_color.get_mean(), 1.0f };
        }
    });

}

void Renderer::set_render_settings(const RenderSettings& settings) {
    renderSettings_ = settings;
    // Todo: optimize
    int sqrt_of_samples = static_cast<int>(std::round(std::sqrtf(renderSettings_.samplesPerPixel)));
    renderSettings_.samplesPerPixel = sqrt_of_samples * sqrt_of_samples;
}
RenderSettings Renderer::get_render_settings() const { return renderSettings_; }

void Renderer::generate_subsample_positions() {
    if (renderSettings_.samplesPerPixel == subsamplesPositions.size()) {
        return; // already generated
    }
    // Todo: optimize
    subsamplesPositions.resize(renderSettings_.samplesPerPixel); //samplesperpixel is a perfect square
    int sqrt_of_samples = static_cast<int>(std::round(std::sqrtf(renderSettings_.samplesPerPixel)));
    for (int i = 0; i < renderSettings_.samplesPerPixel; ++i) {
        subsamplesPositions[i] = fvec2{
            (static_cast<float>(i / sqrt_of_samples) + 0.5f) / static_cast<float>(sqrt_of_samples),
            (static_cast<float>(i % sqrt_of_samples) + 0.5f) / static_cast<float>(sqrt_of_samples)
        };
    }
}

}