#include "renderer.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <execution>
#include <glm/ext.hpp>
#include <glm/glm.hpp>
#include <random>
#include <ranges>
#include <stdexcept>
#include <vector>

#include "arguments.h"
#include "brdf.h"

namespace {
using namespace glm;

fvec4 ndc_from_pixel(float x, float y, float inv_width, float inv_height) {
    float ndc_x = x * inv_width * 2.0f - 1.0f;
    float ndc_y = -y * inv_height * 2.0f + 1.0f;  // Flip Y if needed??
    return fvec4(ndc_x, ndc_y, 0.0f, 1.0f);
}

}  // namespace

namespace app {

void Renderer::update_camera_transform_state(
    fvec3 position, fvec3 direction, fvec3 up, fastgltf::Camera::Perspective perspectiveParams) {
    origin_ = position;
    viewMatrix_ = glm::lookAt(position, position + direction, up);
    projectionMatrix_ = glm::perspectiveRH(
        perspectiveParams.yfov, 
        perspectiveParams.aspectRatio.value_or(1.77777777777777777f),
        perspectiveParams.znear, 
        perspectiveParams.zfar.value_or(1000.f));
    auto viewMatrixNoTranslation = viewMatrix_;
    viewMatrixNoTranslation[3] = fvec4(0.0f, 0.0f, 0.0f, 1.0f);  // Remove translation from view matrix for direction calculation
    NDC2WorldMatrix_ = glm::inverse(projectionMatrix_ * viewMatrixNoTranslation);
}

void Renderer::load_scene(
    const Model& model, const CPUTexture<hdr_pixel>& envmap, size_t ModelFenceValue, size_t EnvmapFenceValue) {
    if (ModelFenceValue == last_model_fence_value_ && EnvmapFenceValue == last_envmap_fence_value_) return;
    model_ref_ = &model;
    envmap_ref_ = &envmap;
    reload_acceleration_structure();
    reload_ray_program();
    last_model_fence_value_ = ModelFenceValue;
    last_envmap_fence_value_ = EnvmapFenceValue;
}

void Renderer::load_envmap(const CPUTexture<hdr_pixel>& envmap, size_t EnvmapFenceValue) {
    if (EnvmapFenceValue == last_envmap_fence_value_) return;
    envmap_ref_ = &envmap;
    reload_ray_program();
    last_envmap_fence_value_ = EnvmapFenceValue;
}

void Renderer::load_model(const Model& model, size_t ModelFenceValue) {
    if (ModelFenceValue == last_model_fence_value_) return;
    model_ref_ = &model;
    reload_acceleration_structure();
    reload_ray_program();
    last_model_fence_value_ = ModelFenceValue;
}

void Renderer::reload_ray_program() {
    if (model_ref_ == nullptr || envmap_ref_ == nullptr) return;
    switch (render_settings_.programMode) {
        case RayProgramMode::AmbientOcclusion:
            ray_program_ = std::make_unique<AOProgram>(*model_ref_, render_settings_);
            break;
        case RayProgramMode::PBR:
            ray_program_ = std::make_unique<PBRProgram>(*model_ref_, *envmap_ref_, render_settings_);
            break;
        case RayProgramMode::RayCaster:
        default:
            ray_program_ = std::make_unique<RayCasterProgram>(*model_ref_, *envmap_ref_, render_settings_);
    }
}
void Renderer::reload_acceleration_structure() {
    if (model_ref_ == nullptr) return;
    switch (render_settings_.accelStructType) {
        case AccelerationStructureType::BVH:
            accel_struct_ = std::make_unique<BVH_AS>(*model_ref_, render_settings_.maxTrianglesPerBVHLeaf);
            break;
        case AccelerationStructureType::Naive:
        default:
            accel_struct_ = std::make_unique<NaiveAS>(*model_ref_);
            break;
    }
}

void Renderer::reload_materials() {}

ray_with_payload Renderer::generate_camera_ray(
    int x, int y, float inv_width, float inv_height, int sampleIndex, fvec2 jitter) const noexcept {
    fvec2 pixel_coords = fvec2{static_cast<float>(x), static_cast<float>(y)} + subsamples_positions_[sampleIndex] + jitter;
    auto ndc_coords = ndc_from_pixel(pixel_coords.x, pixel_coords.y, inv_width, inv_height);
    auto world_coords = NDC2WorldMatrix_ * ndc_coords;
    world_coords /= world_coords.w;
    auto direction = normalize(xyz(world_coords));
    return ray_with_payload{
        {origin_, direction}, 
        fvec4(1.0f), 
        static_cast<std::uint8_t>(render_settings_.maxRayBounces), 
        false};
}

void Renderer::render_frame(CPUFrameBuffer& framebuffer, bool continuous, bool iterative, int iteration_count) {
    // atomic<bool>& instead of bool continuous?
    assert(model_ref_);
    if (model_ref_ == nullptr || envmap_ref_ == nullptr || accel_struct_ == nullptr || ray_program_ == nullptr) {
        throw std::runtime_error("One of components is nullptr in Renderer::render_frame()");
    }
    progress_ = 0.0f;
    render_state_ = RenderingState::Rendering;

    generate_subsample_positions();

    glm::fvec2 jitter = glm::fvec2{0.0f};
    float inverse_iteration_count = 1.0f;
    if (iterative) {
        static std::minstd_rand gen = std::minstd_rand(std::random_device{}());
        static std::uniform_real_distribution<float> dist{-0.5f, 0.5f};
        jitter = fvec2{dist(gen), dist(gen)};
        float n_sqrt = sqrtf(render_settings_.samplesPerPixel);
        jitter /= n_sqrt;
        inverse_iteration_count = 1.0f / static_cast<float>(iteration_count);
    }

    int width = framebuffer.width();
    int height = framebuffer.height();
    float inv_width = 1.0f / static_cast<float>(width);
    float inv_height = 1.0f / static_cast<float>(height);
    auto indices = std::views::iota(0, height);

    size_t reserved_size = 1 + render_settings_.maxNewRaysPerBounce * render_settings_.maxRayBounces;
    if (render_settings_.programMode == RayProgramMode::PBR) {
        reserved_size = 1 + (3 - 1) * render_settings_.maxRayBounces;
    }

    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
        [this, width, inv_width, inv_height, &framebuffer, reserved_size, iterative, inverse_iteration_count, jitter](int y) {
            static thread_local std::vector<ray_with_payload> rays;
            rays.clear();
            rays.reserve(reserved_size);

            for (int x = 0; x < width; ++x) {
                if (render_state_.load(std::memory_order_relaxed) == RenderingState::Cancelling) {
                    return;
                }
                SamplesAccumulator<fvec3> final_color;

                for (int i = 0; i < render_settings_.samplesPerPixel; ++i) {
                    fvec3 sample_col{};

                    rays.push_back(generate_camera_ray(x, y, inv_width, inv_height, i, jitter));
                    while (rays.size() > 0) {
                        assert(rays.size() <= reserved_size);
                        auto ray = rays.back();
                        rays.pop_back();
                        auto hit = accel_struct_->intersect_ray(ray, ray.any_hit);
                        sample_col += ray_program_->on_hit(ray, hit, rays);
                    }
                    final_color.add_sample(sample_col);
                }
                if (!iterative) {
                    framebuffer.at(x, y) = hdr_pixel{final_color.get_mean(), 1.0f};
                } else {
                    auto& p = framebuffer.at(x, y);
                    const auto f0 = xyz(p);
                    const auto f1 = final_color.get_mean();
                    p = hdr_pixel{glm::mix(f0, f1, inverse_iteration_count), 1.0f};
                    // todo: make framebuffer into SamplesAccumulator internally?
                }
            }
            progress_ = max(progress_, y * inv_height);
        });

    if (render_state_ == RenderingState::Rendering) {
        progress_ = 1.0f;
    }
    if (continuous && (render_state_ != RenderingState::Cancelling)) {
        render_state_ = RenderingState::ReadyToStart;
    } else {
        render_state_ = RenderingState::Idle;
    }
}

void Renderer::set_render_settings(const RenderSettings& settings) {
    const auto currentSettings = render_settings_;
    render_settings_ = settings;

    if (currentSettings.accelStructType != settings.accelStructType ||
        currentSettings.maxTrianglesPerBVHLeaf != settings.maxTrianglesPerBVHLeaf) {
        reload_acceleration_structure();
    }
    if (currentSettings.programMode != settings.programMode || currentSettings.envmapRotation != settings.envmapRotation ||
        currentSettings.maxNewRaysPerBounce != settings.maxNewRaysPerBounce) {
        reload_ray_program();
    }
}
RenderSettings Renderer::get_render_settings() const { return render_settings_; }

BBox Renderer::get_scene_bound() const {
    if (accel_struct_) {
        return accel_struct_->get_scene_bounds();
    }
    return BBox{};
}

float Renderer::get_progress() const { return progress_; }

void Renderer::cancel_rendering() {
    if (render_state_ == RenderingState::Rendering) {
        render_state_ = RenderingState::Cancelling;
    }
}

RenderingState Renderer::get_rendering_state() const { return render_state_; }

void Renderer::set_render_starting_state() {
    if (render_state_ == RenderingState::Idle) {
        render_state_ = RenderingState::ReadyToStart;
    }
}

void Renderer::generate_subsample_positions() {
    if (render_settings_.samplesPerPixel == subsamples_positions_.size()) {
        return;  // already generated
    }
    subsamples_positions_.resize(render_settings_.samplesPerPixel);
    int sqrt_of_samples = static_cast<int>(sqrtf(render_settings_.samplesPerPixel));
    if (sqrt_of_samples * sqrt_of_samples == render_settings_.samplesPerPixel) {
        // samplesperpixel is a perfect square
        for (unsigned int i = 0; i < render_settings_.samplesPerPixel; ++i) {
            subsamples_positions_[i] = fvec2{
                (static_cast<float>(i / sqrt_of_samples) + 0.5f) / static_cast<float>(sqrt_of_samples),
                (static_cast<float>(i % sqrt_of_samples) + 0.5f) / static_cast<float>(sqrt_of_samples)};
        }
    } else {
        // samplesperpixel is not a perfect square
        float inv_samples = 1.0f / static_cast<float>(render_settings_.samplesPerPixel);
        for (int i = 0; i < render_settings_.samplesPerPixel; ++i) {
            subsamples_positions_[i] = fibonacci2D(i, inv_samples);
        }
    }
}

void Renderer::update_transforms() {
    if (model_ref_ && accel_struct_) {
        accel_struct_->update_transforms(*model_ref_);
    }
}

}  // namespace app