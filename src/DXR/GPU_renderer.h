#pragma once

#include <atomic>
#include <fastgltf/types.hpp>
#include <glm/fwd.hpp>
#include <memory>

#include "../acceleration_structure.h"
#include "../cpu_framebuffer.h"
#include "../model_loader.h"
#include "../render_settings.h"
#include "../renderer.h"
#include "GPU_model.h"
#include "GPU_pipeline.h"

namespace app {

using glm::fvec2;
using glm::fvec3;
using glm::fvec4;

class GPURenderer : public IRenderer {
  public:
    GPURenderer() = default;
    ~GPURenderer() = default;
    GPURenderer(const GPURenderer&) = delete;
    GPURenderer& operator=(const GPURenderer&) = delete;

    void update_camera_transform_state(
        fvec3 position, fvec3 direction, fvec3 up, fastgltf::Camera::Perspective perspectiveParams);

    void load_scene(const Model& model, const CPUTexture<hdr_pixel>& envmap, size_t ModelFenceValue,
        size_t EnvmapFenceValue);  // should be in constructor?
    void load_envmap(const CPUTexture<hdr_pixel>& envmap, size_t EnvmapFenceValue);
    void load_model(const Model& model, size_t ModelFenceValue);
    void reload_ray_program();
    void reload_acceleration_structure();
    void reload_materials();

    void render_frame(CPUFrameBuffer& framebuffer, bool continuous, bool iterative, int iteration_count);
    void set_render_settings(const RenderSettings& settings);
    RenderSettings get_render_settings() const;
    BBox get_scene_bound() const;

    float get_progress() const;
    void cancel_rendering();
    RenderingState get_rendering_state() const;
    void set_render_starting_state();

  protected:
    const Model* modelRef = nullptr;
    const CPUTexture<hdr_pixel>* envmapRef = nullptr;
    std::unique_ptr<GPU_model> gpu_model_;
    std::unique_ptr<GPU_texture> gpu_envmap_;
    RenderSettings renderSettings_;

    fmat4x4 viewMatrix_ = fmat4x4(1.0f);
    fmat4x4 projectionMatrix_ = fmat4x4(1.0f);
    fmat4x4 NDC2WorldMatrix_ = fmat4x4(1.0f);
    fvec3 origin_ = fvec3{0.0f};

    float progress_ = 0.0f;
    std::atomic<RenderingState> render_state_ = RenderingState::Idle;

  private:
    GPU_pipeline pipeline_;

    size_t lastModelFenceValue = 0;
    size_t lastEnvmapFenceValue = 0;

    unsigned int frameID = 0;
};

}  // namespace app