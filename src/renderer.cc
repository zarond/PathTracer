#include "renderer.h"
#include "additional_math.h"
#include "brdf.h"

#include <ranges>
#include <algorithm>
#include <execution>

namespace {
using namespace fastgltf::math;
using namespace app;

fvec4 ndc_from_pixel(int x, int y, int width, int height) {
    float ndc_x = (static_cast<float>(x) + 0.5f) / static_cast<float>(width) * 2.0f - 1.0f;
    float ndc_y = -(static_cast<float>(y) + 0.5f) / static_cast<float>(height) * 2.0f + 1.0f; // Flip Y if needed??
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
    auto ndc_coords = ndc_from_pixel(x, y, width, height);
    auto direction = fvec3(NDC2WorldMatrix_ * ndc_coords);
    return ray{ origin_, normalize(direction) };
    // need to use samplesPerPixel and distributeSamples as well
}

void Renderer::render_frame(CPUFrameBuffer& framebuffer)
{
    assert(modelRef);
    if (modelRef == nullptr) {
        throw 1; // Todo
    }
    framebuffer.clear(hdr_pixel{ 0.0f, 0.0f, 1.0f, 1.0f }); // Clear to color for testing

    int width = framebuffer.width();
    int height = framebuffer.height();
    auto indices = std::views::iota(0, height);

    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
    [this, width, height, &framebuffer](int y) {
        for (int x = 0; x < width; ++x) {
            auto ray = generate_camera_ray(x, y, width, height);    
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

            auto& pixel = framebuffer.at(x, y);
            //pixel = hit.forward_hit() ? app::pixel{ abs(normal[0]), abs(normal[1]), abs(normal[2]), 1.0f} : pixel;
            //pixel = hit.forward_hit() ? app::pixel{ uv[0], uv[1], 0.0f, 1.0f } : pixel;
            //pixel = hit.forward_hit() ? app::hdr_pixel{ hit.distance, hit.distance, hit.distance, 1.0f} : pixel;
            pixel = hit.forward_hit() ? albedo_color : pixel;
        }
    });

}

}