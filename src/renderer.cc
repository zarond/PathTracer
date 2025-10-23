#include "renderer.h"
#include "additional_math.h"

#include <ranges>
#include <algorithm>
#include <execution>

namespace {
    using namespace fastgltf::math;

    fvec4 ndc_from_pixel(int x, int y, int width, int height) {
        float ndc_x = (static_cast<float>(x) + 0.5f) / static_cast<float>(width) * 2.0f - 1.0f;
        float ndc_y = -(static_cast<float>(y) + 0.5f) / static_cast<float>(height) * 2.0f + 1.0f; // Flip Y if needed??
        return fvec4(ndc_x, ndc_y, 0.0f, 1.0f);
    }
}

namespace app {
    void PerspectiveCameraRenderer::load_scene(const Model& model)
    {
        accelStruct = std::make_unique<NaiveAS>(model); // ???
    }
    void PerspectiveCameraRenderer::render_frame(CPUFrameBuffer& framebuffer)
    {
        framebuffer.clear(pixel{ 0.0f, 0.0f, 1.0f, 1.0f }); // Clear to black for testing

        int width = framebuffer.width();
        int height = framebuffer.height();
        auto indices = std::views::iota(0, height);

        std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
        [this, width, height, &framebuffer](int y) {
            for (int x = 0; x < width; ++x) {
                auto& pixel = framebuffer.at(x, y);
                auto ndc_coords = ndc_from_pixel(x, y, width, height);
                auto direction = fvec3(NDC2WorldMatrix_ * ndc_coords);
                direction = normalize(direction);
                
                size_t object_index = 0;
                auto hit = accelStruct->intersect_ray(ray{ origin_, direction }, object_index);
                
                auto mesh_hit_index = accelStruct->object_data_[object_index].meshIndex;
                const auto& mesh_data = accelStruct->mesh_data_->at(mesh_hit_index);
                auto p1 = mesh_data.indices[hit.triangleIndex];
                auto p2 = mesh_data.indices[hit.triangleIndex + 1];
                auto p3 = mesh_data.indices[hit.triangleIndex + 2];
                auto normal = mesh_data.vertices[p1].normal * hit.b_coords.A +
                           mesh_data.vertices[p2].normal * hit.b_coords.B +
                           mesh_data.vertices[p3].normal * hit.b_coords.C();
                auto uv = mesh_data.vertices[p1].uv * hit.b_coords.A +
                    mesh_data.vertices[p2].uv * hit.b_coords.B +
                    mesh_data.vertices[p3].uv * hit.b_coords.C();
                //pixel = hit.forward_hit() ? app::pixel{ abs(normal[0]), abs(normal[1]), abs(normal[2]), 1.0f} : pixel;
                //pixel = hit.forward_hit() ? app::pixel{ uv[0], uv[1], 0.0f, 1.0f } : pixel;
                pixel = hit.forward_hit() ? app::pixel{ hit.distance, hit.distance, hit.distance, 1.0f} : pixel;
                //pixel = hit.forward_hit() ? app::pixel{ float(object_index), float(object_index), float(object_index), 1.0f} : pixel;
            }
        });

    }
    void PerspectiveCameraRenderer::update_camera_transform_state(
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
        viewMatrix_[3] = fvec4(0.0f, 0.0f, 0.0f, 1.0f); // Remove translation from view matrix for direction calculation
        NDC2WorldMatrix_ = inverse(projectionMatrix_ * viewMatrix_);
    }
}