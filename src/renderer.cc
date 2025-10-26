#include "renderer.h"
#include "additional_math.h"
#include "brdf.h"

#include <ranges>
#include <algorithm>
#include <execution>

namespace {
using namespace fastgltf::math;
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
    viewMatrix_ = lookAtRH(position, position + direction, up);
    projectionMatrix_ = perspectiveRH(
        perspectiveParams.yfov,
        perspectiveParams.aspectRatio.value_or(1.77777777777777777f), // Todo ????
        perspectiveParams.znear,
        perspectiveParams.zfar.value_or(1000.f));
    auto viewMatrixNoTranslation = viewMatrix_;
    viewMatrixNoTranslation[3] = fvec4(0.0f, 0.0f, 0.0f, 1.0f); // Remove translation from view matrix for direction calculation
    NDC2WorldMatrix_ = inverse(projectionMatrix_ * viewMatrixNoTranslation);
}

void Renderer::load_scene(const Model& model)
{
    accelStruct = std::make_unique<NaiveAS>(model);
    modelRef = &model;
}

ray Renderer::generate_camera_ray(int x, int y, int width, int height, int sampleIndex) const {
    fvec2 pixel_coords = fvec2{static_cast<float>(x), static_cast<float>(y)} + subsamplesPositions[sampleIndex];
    auto ndc_coords = ndc_from_pixel(pixel_coords.x(), pixel_coords.y(), width, height);
    auto direction = fvec3(NDC2WorldMatrix_ * ndc_coords);
    return ray{ origin_, normalize(direction) };
}

void Renderer::render_frame(CPUFrameBuffer& framebuffer)
{
    assert(modelRef);
    if (modelRef == nullptr) {
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
                auto ray = generate_camera_ray(x, y, width, height, i);    
                auto hit = accelStruct->intersect_ray(ray);
                
                auto mesh_hit_index = hit.meshIndex;
                const auto& mesh_data = modelRef->meshes_[mesh_hit_index];
                auto p1 = mesh_data.indices[hit.triangleIndex];
                auto p2 = mesh_data.indices[hit.triangleIndex + 1];
                auto p3 = mesh_data.indices[hit.triangleIndex + 2];
                auto normal = mesh_data.vertices[p1].normal * hit.b_coords.A +
                            mesh_data.vertices[p2].normal * hit.b_coords.B +
                            mesh_data.vertices[p3].normal * hit.b_coords.C();
                auto uv = mesh_data.vertices[p1].uv * hit.b_coords.A +
                    mesh_data.vertices[p2].uv * hit.b_coords.B +
                    mesh_data.vertices[p3].uv * hit.b_coords.C();

                auto mat_index = mesh_data.materialIndex;
                auto albedo_color = sample_albedo(modelRef->materials_[mat_index], modelRef->images_, uv);
                albedo_color = hit.forward_hit() ? albedo_color : fvec4(0.0f);

                final_color.add_sample(fvec3(albedo_color));
            }
            framebuffer.at(x, y) = fvec4(final_color.get_mean());
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